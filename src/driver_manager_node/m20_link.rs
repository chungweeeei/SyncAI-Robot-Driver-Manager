use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use rclrs::*;

use super::health::SensorHealth;
use super::m20_protocol as m20;

const LOG_THROTTLE: Duration = Duration::from_secs(1);
// The manual recommends 20 Hz for axis commands (1.2.5)
const SEND_PERIOD: Duration = Duration::from_millis(50);
// The manual asks for >= 1 Hz; 2 Hz survives one lost datagram. The robot reports its status
// to whichever address:port keeps sending these, so the heartbeat is also what makes the
// receive side work.
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(500);
// Wakes the receive thread every 100 ms to check `running`
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
// UDP is lossy and a lost stop leaves the robot walking, so every stop goes out this many times
// (the backend's teleop repeats its stops the same way)
const STOP_REPEATS: u8 = 3;

/// Where the M20 is and how cmd_vel maps onto its axis command (see `M20Parameters`)
#[derive(Debug, Clone, Copy)]
pub struct M20Config {
    pub target: SocketAddr,
    pub max_vx: f64,
    pub max_vy: f64,
    pub max_wz: f64,
    pub cmd_timeout: Duration,
}

/// The link to a DEEP Robotics Lynx M20: one UDP socket, connected to the robot, used both to
/// send and to receive (the robot reports to the heartbeat's source port).
///
/// Two threads:
/// * `m20_send` (20 Hz): the heartbeat every 500 ms, and the axis command. Unlike the G23 path,
///   which forwards each cmd_vel once and leaves stopping to the controller, this re-sends the
///   newest cmd_vel at the 20 Hz the manual asks for while it is younger than `cmd_timeout`, then
///   sends zero `STOP_REPEATS` times and goes quiet. That is a cmd_vel watchdog the G23 path does
///   not have: the manual does not say what the M20 does when axis commands stop, so a publisher
///   that dies mid-motion must not leave it walking.
/// * `m20_recv`: counts every datagram for the monitor's telemetry row and logs `MotionState`
///   changes from the basic status report, so a stand or lie-down can be checked in the pane.
///   Nothing is published.
///
/// Motion-state commands (stand / lie down) are sent from the caller's thread, once: like the
/// G23's motion keys, there is no acknowledgement and no retry.
pub struct M20Link {
    shared: Arc<Shared>,
    threads: Vec<JoinHandle<()>>,
}

struct Shared {
    socket: UdpSocket,
    config: M20Config,
    next_id: AtomicU16,
    axis: Mutex<AxisState>,
    running: AtomicBool,
    health: Arc<SensorHealth>,
    logger: Logger,
}

/// What the send thread owes the axis command, under one lock so that a stand / lie-down can
/// cancel both the command and its trailing stops at once
#[derive(Debug, Default)]
struct AxisState {
    /// The newest cmd_vel as ratios, and when it arrived
    velocity: Option<Velocity>,
    /// Zero commands still to send after the last cmd_vel went stale
    stops_left: u8,
}

#[derive(Debug, Clone, Copy)]
struct Velocity {
    x: f64,
    y: f64,
    yaw: f64,
    received: Instant,
}

impl M20Link {
    pub fn open(config: M20Config, health: Arc<SensorHealth>, logger: Logger) -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(config.target)?;
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;
        let recv_socket = socket.try_clone()?;

        let shared = Arc::new(Shared {
            socket,
            config,
            next_id: AtomicU16::new(0),
            axis: Mutex::new(AxisState::default()),
            running: AtomicBool::new(true),
            health,
            logger,
        });

        let mut link = Self {
            shared: Arc::clone(&shared),
            threads: Vec::with_capacity(2),
        };
        // Pushed one at a time so a failed second spawn still stops and joins the first (Drop)
        link.threads.push(thread::Builder::new().name("m20_send".into()).spawn({
            let shared = Arc::clone(&shared);
            move || shared.send_loop()
        })?);
        link.threads.push(thread::Builder::new().name("m20_recv".into()).spawn({
            let shared = Arc::clone(&shared);
            move || shared.recv_loop(&recv_socket)
        })?);
        Ok(link)
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.shared.socket.local_addr()
    }

    /// Takes a cmd_vel [m/s, m/s, rad/s]; the send thread forwards it at 20 Hz until it is
    /// `cmd_timeout` old
    pub fn set_velocity(&self, vx: f64, vy: f64, wz: f64) {
        let c = &self.shared.config;
        let velocity = Velocity {
            x: m20::velocity_ratio(vx, c.max_vx),
            y: m20::velocity_ratio(vy, c.max_vy),
            yaw: m20::velocity_ratio(wz, c.max_wz),
            received: Instant::now(),
        };
        self.shared.axis().velocity = Some(velocity);
    }

    /// Sends one motion-state switch (`MotionParam`). The current axis command and its trailing
    /// stops are dropped and one zero sent first, so the robot is not still being driven while it
    /// stands up or lies down.
    /// Standing also re-sends Regular mode first: axis commands are honoured only in it, and the
    /// robot's remote or app may have switched it since this node started.
    pub fn set_motion_state(&self, param: i32) {
        *self.shared.axis() = AxisState::default();
        self.shared.send(m20::AXIS, &m20::axis_items(0.0, 0.0, 0.0));
        if param == m20::MOTION_PARAM_STAND {
            self.shared.send(
                m20::USAGE_MODE,
                &m20::usage_mode_items(m20::USAGE_MODE_REGULAR),
            );
        }
        self.shared.send(m20::MOTION_STATE, &m20::motion_state_items(param));
    }
}

impl Drop for M20Link {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Relaxed);
        for handle in self.threads.drain(..) {
            let _ = handle.join();
        }
    }
}

impl Shared {
    fn axis(&self) -> MutexGuard<'_, AxisState> {
        self.axis.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn send(&self, kind: (u32, u32), items: &str) {
        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let asdu = m20::asdu(kind, &m20::timestamp(secs), items);
        // Wraps at 65 535 back to 0, as the manual specifies
        let id = self.next_id.fetch_add(1, Ordering::Relaxed);
        let Some(packet) = m20::packet(id, &asdu) else {
            log_error!(&self.logger, "[M20] ASDU too long to send: {asdu}");
            return;
        };
        let result = self.socket.send(&packet);
        self.health.record_command(result.is_ok());
        if let Err(e) = result {
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[M20] send {kind:?} to {} failed: {e}",
                self.config.target,
            );
        }
    }

    fn send_loop(&self) {
        let mut next_heartbeat = Instant::now();
        let mut regular_mode_sent = false;

        while self.running.load(Ordering::Relaxed) {
            let now = Instant::now();

            if now >= next_heartbeat {
                self.send(m20::HEARTBEAT, &m20::heartbeat_items());
                next_heartbeat = now + HEARTBEAT_PERIOD;
                // Once, right after the first heartbeat (the vendor's own teleop does the same):
                // without Regular mode the robot ignores axis commands
                if !regular_mode_sent {
                    self.send(
                        m20::USAGE_MODE,
                        &m20::usage_mode_items(m20::USAGE_MODE_REGULAR),
                    );
                    regular_mode_sent = true;
                }
            }

            // Decided under the lock, sent outside it, so cmd_vel never waits on a send
            let items = {
                let mut axis = self.axis();
                let velocity = axis.velocity;
                match velocity {
                    Some(v) if now.duration_since(v.received) <= self.config.cmd_timeout => {
                        axis.stops_left = STOP_REPEATS;
                        Some(m20::axis_items(v.x, v.y, v.yaw))
                    }
                    _ if axis.stops_left > 0 => {
                        axis.stops_left -= 1;
                        Some(m20::axis_items(0.0, 0.0, 0.0))
                    }
                    _ => None,
                }
            };
            if let Some(items) = items {
                self.send(m20::AXIS, &items);
            }

            thread::sleep(SEND_PERIOD);
        }
    }

    fn recv_loop(&self, socket: &UdpSocket) {
        let mut buf = [0u8; 65_535];
        let mut last_state: Option<i64> = None;

        while self.running.load(Ordering::Relaxed) {
            let n = match socket.recv(&mut buf) {
                Ok(n) => n,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue;
                }
                Err(e) => {
                    // ECONNREFUSED on a connected UDP socket: nothing listens at the target yet
                    log_warn!(
                        self.logger.throttle(LOG_THROTTLE),
                        "[M20] receive from {} failed: {e}",
                        self.config.target,
                    );
                    thread::sleep(RECV_TIMEOUT);
                    continue;
                }
            };

            let asdu = m20::parse_packet(&buf[..n]);
            self.health.record_telemetry(self.config.target, asdu.is_none());
            let Some(state) = asdu.and_then(m20::reported_motion_state) else {
                continue;
            };
            if last_state != Some(state) {
                log_info!(
                    &self.logger,
                    "[M20] MotionState {state} ({})",
                    m20::motion_state_name(state),
                );
                last_state = Some(state);
            }
        }
    }
}

#[cfg(test)]
#[path = "tests/m20_link.rs"]
mod tests;
