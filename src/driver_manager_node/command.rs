use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};
use std::time::Duration;

use rclrs::*;
use ros_env::std_msgs::msg::Bool;

use super::protocol;
use super::session::UdpSession;

const LOG_THROTTLE: Duration = Duration::from_secs(1);

/// Sends ASCII commands to the gait controller.
///
/// UDP has no acknowledgement: a successful `send` only means the datagram reached the kernel,
/// not that the controller received it. A dropped command is simply lost: nothing retries it
/// and nothing notices.
pub struct CommandLink {
    session: UdpSession,
    logger: Logger,
}

impl CommandLink {
    pub fn new(session: UdpSession, logger: Logger) -> Self {
        Self { session, logger }
    }

    pub fn send(&self, command: &str) {
        if let Err(e) = self.session.socket.send(command.as_bytes()) {
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[Command] send {:?} to {:?} failed: {e}",
                command.trim_end(),
                self.session.peer_addr,
            );
        }
    }
}

/// While engaged, `set_motion_key` only lets ESTOP through; only the `reset_safety` service
/// releases it.
///
/// Nothing triggers it yet (low battery and JOINT_TEMP overheat are both still TODO), so in
/// practice it is never engaged.
/// Also, unlike the reference implementation, cmd_vel and set_policy_mode are **not** gated
/// by it.
///
/// Every state change is published on the latched `safety_locked` topic (see
/// [`SafetyLock::new`]); `Default` builds one without a publisher, for tests.
#[derive(Default)]
pub struct SafetyLock {
    engaged: AtomicBool,
    state: Option<StatePublisher>,
}

struct StatePublisher {
    publisher: Publisher<Bool>,
    // Serializes publishes; see `SafetyLock::publish_state`
    order: Mutex<()>,
    logger: Logger,
}

impl SafetyLock {
    /// Publishes the initial (released) state right away, so a transient-local subscriber
    /// always has a value even if the lock never changes.
    pub fn new(publisher: Publisher<Bool>, logger: Logger) -> Self {
        let lock = Self {
            engaged: AtomicBool::new(false),
            state: Some(StatePublisher {
                publisher,
                order: Mutex::new(()),
                logger,
            }),
        };
        lock.publish_state();
        lock
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged.load(Ordering::Acquire)
    }

    /// Releases the lock; returns whether it was engaged
    pub fn release(&self) -> bool {
        let was_engaged = self.engaged.swap(false, Ordering::AcqRel);
        if was_engaged {
            self.publish_state();
        }
        was_engaged
    }

    /// Engages the lock and lies the robot down (MODE X). The swap is an atomic check-and-set,
    /// so concurrent triggers act exactly once.
    /// Safe to call from the telemetry thread.
    // TODO: wire up the triggers. JOINT_TEMP thresholds from the reference implementation:
    //       >= 75 °C warn, >= 95 °C lie down, >= 115 °C ESTOP.
    #[cfg_attr(
        not(test),
        expect(dead_code, reason = "no safety trigger is wired yet")
    )]
    pub fn trigger(&self, reason: &str, command: &CommandLink, logger: &Logger) {
        if !self.engaged.swap(true, Ordering::AcqRel) {
            log_error!(logger, "[Safety] !!! SAFETY TRIGGERED: {reason} !!!");
            log_error!(
                logger,
                "[Safety] Executing LieDown [MODE X] and blocking control"
            );
            command.send(protocol::LIE_DOWN_COMMAND);
            self.publish_state();
        }
    }

    /// Publishes the state as it is *now*, not the value the caller swapped in. The read
    /// happens under the mutex, so the last publish always follows the last swap: when a
    /// trigger and a release race, the latched message still ends up matching `engaged`.
    fn publish_state(&self) {
        let Some(state) = &self.state else { return };
        let _order = state.order.lock().unwrap_or_else(PoisonError::into_inner);
        let msg = Bool {
            data: self.is_engaged(),
        };
        if let Err(e) = state.publisher.publish(msg) {
            log_error!(&state.logger, "[Safety] publish safety_locked failed: {e}");
        }
    }
}
