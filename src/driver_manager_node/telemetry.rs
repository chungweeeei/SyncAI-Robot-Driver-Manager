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

use super::protocol::{self, Battery, ChargeState, JOINT_NAMES, NUM_DOF, Sections, Telemetry};
use super::publishers::Publishers;
use super::session::UdpSession;

// Sleeps in the kernel's recv while idle, but wakes every 100 ms to check `running`
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const LOG_THROTTLE: Duration = Duration::from_secs(1);

/// Handle to the telemetry receive thread. The thread owns the telemetry socket and the
/// publishers; dropping the handle stops the thread and joins it, which also closes the socket.
pub struct TelemetryWorker {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl TelemetryWorker {
    pub fn spawn(
        session: UdpSession,
        publishers: Publishers,
        clock: Clock,
        logger: Logger,
    ) -> io::Result<Self> {
        session.socket.set_read_timeout(Some(RECV_TIMEOUT))?;

        let running = Arc::new(AtomicBool::new(true));
        let handle = thread::Builder::new().name("telemetry".into()).spawn({
            let running = Arc::clone(&running);
            let telemetry = TelemetryLoop {
                publishers,
                clock,
                logger,
            };
            move || telemetry.run(&session.socket, &running)
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
        // A UDP datagram is at most 65507 bytes
        let mut buf = vec![0u8; 65_507];

        while running.load(Ordering::Relaxed) {
            let (n, from) = match socket.recv_from(&mut buf) {
                Ok(r) => r,
                // Timed out: go back and check `running`
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

            // Whether or not any section parses, this line is the quickest proof that the UDP
            // link is alive
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
            // The BMS reports 0-100; BatteryState.percentage is defined on 0-1
            percentage: battery.soc / 100.0,
            temperature: battery.temperature,
            charge: f32::NAN,
            capacity: f32::NAN,
            design_capacity: f32::NAN,
            power_supply_status: match battery.charge_state {
                ChargeState::Charging => BatteryState::POWER_SUPPLY_STATUS_CHARGING,
                ChargeState::Discharging => BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
                ChargeState::Idle => BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING,
            },
            power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN,
            power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
            present: true,
            cell_voltage: battery.cells.map(Vec::from).unwrap_or_default(),
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
        // TODO: monitor JOINT_TEMP for overheat and trigger a safety shutdown (thresholds in
        //       SafetyLock::trigger).

        if s.has_imu() {
            let msg = IMUState {
                timestamp: self.timestamp_ns(),
                // Telemetry carries no quaternion, so derive it from RPY; identity when there
                // is no IMU_RPY section
                quaternion: s
                    .rpy
                    .map_or([1.0, 0.0, 0.0, 0.0], protocol::quaternion_from_rpy),
                gyroscope: s.omega.unwrap_or_default(),
                accelerometer: s.acc.unwrap_or_default(),
                rpy: s.rpy.unwrap_or_default(),
                // IMU temperature is not in the telemetry; left at 0
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
                    // Telemetry has no acceleration section; left at 0
                    ddq: 0.0,
                    tau_est: pick(s.joint_tau, i),
                    // f32 -> i8 `as` truncates toward zero and saturates at i8::MIN / MAX
                    // (never UB, unlike the C++ static_cast); the message field is i8
                    temperature: pick(s.joint_temp, i) as i8,
                    // Out-of-range codes saturate to u16::MAX instead of wrapping into a
                    // small code that could look valid
                    error: s
                        .joint_err
                        .map_or(0, |v| u16::try_from(v[i]).unwrap_or(u16::MAX)),
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

    /// IMUState / MotorStates timestamps are **nanoseconds** (RobotState uses seconds and
    /// ArtifactState milliseconds; check the unit before doing arithmetic across messages)
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
