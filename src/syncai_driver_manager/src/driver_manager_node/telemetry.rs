use std::io;
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use rclrs::*;
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::sensor_msgs::msg::BatteryState;
use ros_env::std_msgs::msg::{Header, Int32MultiArray};
use ros_env::syncai_common::msg::{IMUState, MotorState, MotorStates};

use super::protocol::{self, Battery, JOINT_NAMES, NUM_DOF, Sections, Telemetry};
use super::publishers::Publishers;

// 閒置時睡在 kernel 的 recv 裡，但每 100 ms 醒來一次檢查 running
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const LOG_THROTTLE: Duration = Duration::from_secs(1);

pub struct TelemetryWorker {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl TelemetryWorker {
    pub fn spawn(
        socket: Arc<UdpSocket>,
        publishers: Publishers,
        clock: Clock,
        logger: Logger,
    ) -> io::Result<Self> {
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;

        let running = Arc::new(AtomicBool::new(true));
        let handle = thread::Builder::new().name("telemetry".into()).spawn({
            let running = Arc::clone(&running);
            let telemetry = TelemetryLoop {
                publishers,
                clock,
                logger,
            };
            move || telemetry.run(&socket, &running)
        })?;

        Ok(Self {
            running,
            handle: Some(handle),
        })
    }
}

impl Drop for TelemetryWorker {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

struct TelemetryLoop {
    publishers: Publishers,
    clock: Clock,
    logger: Logger,
}

impl TelemetryLoop {
    fn run(&self, socket: &UdpSocket, running: &AtomicBool) {
        // 一個 UDP datagram 最大 65507 bytes
        let mut buf = vec![0u8; 65_507];

        while running.load(Ordering::Relaxed) {
            let (n, from) = match socket.recv_from(&mut buf) {
                Ok(r) => r,
                // 逾時：回去檢查 running
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock
                            | io::ErrorKind::TimedOut
                            | io::ErrorKind::Interrupted
                    ) =>
                {
                    continue;
                }
                Err(e) => {
                    log_warn!(
                        self.logger.throttle(LOG_THROTTLE),
                        "[Telemetry] recv failed: {e}"
                    );
                    continue;
                }
            };

            // 不管有沒有 section 解得出來，這行都是「UDP 連線還活著」最快的確認方式
            log_info!(
                self.logger.throttle(LOG_THROTTLE),
                "[Telemetry] Received {n} bytes from {from}"
            );

            let line = String::from_utf8_lossy(&buf[..n]);
            let (telemetry, warnings) = protocol::parse_telemetry(&line);
            for warning in &warnings {
                log_warn!(
                    self.logger.throttle(LOG_THROTTLE),
                    "[Telemetry] Skipping {warning}"
                );
            }

            match telemetry {
                Some(Telemetry::Battery(battery)) => self.publish_battery(&battery),
                Some(Telemetry::Sections(sections)) => self.publish_sections(&sections),
                None => {}
            }
        }
    }

    fn publish_battery(&self, battery: &Battery) {
        let msg = BatteryState {
            header: Header {
                stamp: self.stamp(),
                ..Default::default()
            },
            voltage: battery.voltage,
            current: battery.current,
            // BMS 回報 0–100，BatteryState.percentage 定義在 0–1
            percentage: battery.soc / 100.0,
            temperature: battery.temperature,
            charge: f32::NAN,
            capacity: f32::NAN,
            design_capacity: f32::NAN,
            power_supply_status: BatteryState::POWER_SUPPLY_STATUS_UNKNOWN,
            power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN,
            power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
            present: true,
            ..Default::default()
        };
        self.publish(&self.publishers.battery_state, msg, "battery_state");

        log_debug!(
            &self.logger,
            "[Telemetry] BMS soc={:.1}% V={:.2} I={:.2} T={:.1}",
            battery.soc,
            battery.voltage,
            battery.current,
            battery.temperature,
        );
    }

    fn publish_sections(&self, s: &Sections) {
        // TODO: 監看 JOINT_TEMP 過熱並觸發 safety shutdown（門檻見 SafetyLock::trigger）。

        if s.has_imu() {
            let msg = IMUState {
                timestamp: self.timestamp_ns(),
                // telemetry 沒有四元數，從 RPY 推；沒有 IMU_RPY section 時用 identity
                quaternion: s
                    .rpy
                    .map_or([1.0, 0.0, 0.0, 0.0], protocol::quaternion_from_rpy),
                gyroscope: s.omega.unwrap_or_default(),
                accelerometer: s.acc.unwrap_or_default(),
                rpy: s.rpy.unwrap_or_default(),
                // IMU 溫度不在 telemetry 裡，維持 0
                temperature: 0,
            };
            self.publish(&self.publishers.imu, msg, "imu");
        }

        if s.has_joints() {
            let pick = |values: Option<[f32; NUM_DOF]>, i: usize| values.map_or(0.0, |v| v[i]);
            let states = JOINT_NAMES
                .iter()
                .enumerate()
                .map(|(i, name)| MotorState {
                    name: (*name).into(),
                    q: pick(s.joint_pos, i),
                    dq: pick(s.joint_vel, i),
                    // telemetry 沒有加速度 section，維持 0
                    ddq: 0.0,
                    tau_est: pick(s.joint_tau, i),
                    temperature: pick(s.joint_temp, i) as i8,
                    error: s.joint_err.map_or(0, |v| v[i] as u16),
                })
                .collect();
            let msg = MotorStates {
                timestamp: self.timestamp_ns(),
                states,
            };
            self.publish(&self.publishers.motor_states, msg, "motor_states");
        }

        if let Some(mode) = s.mode_state {
            let msg = Int32MultiArray {
                data: mode.to_vec(),
                ..Default::default()
            };
            self.publish(&self.publishers.mode, msg, "mode");
        }
    }

    fn publish<T: rosidl_runtime_rs::Message>(
        &self,
        publisher: &Publisher<T>,
        msg: T,
        topic: &str,
    ) {
        if let Err(e) = publisher.publish(msg) {
            log_error!(
                self.logger.throttle(LOG_THROTTLE),
                "[Telemetry] publish {topic} failed: {e}"
            );
        }
    }

    /// IMUState / MotorStates 的 timestamp 是 **nanoseconds**（RobotState 用秒、
    /// ArtifactState 用毫秒，做跨訊息運算前先確認單位）
    fn timestamp_ns(&self) -> u64 {
        u64::try_from(self.clock.now().nsec).unwrap_or(0)
    }

    fn stamp(&self) -> TimeMsg {
        let nsec = self.clock.now().nsec;
        TimeMsg {
            sec: i32::try_from(nsec.div_euclid(1_000_000_000)).unwrap_or(i32::MAX),
            nanosec: nsec.rem_euclid(1_000_000_000) as u32,
        }
    }
}
