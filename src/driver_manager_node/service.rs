use rclrs::*;
use ros_env::std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response};
use ros_env::syncai_common::srv::{
    SetMotionKey, SetMotionKey_Request, SetMotionKey_Response, SetPolicyMode,
    SetPolicyMode_Request, SetPolicyMode_Response, SetSpeedScale, SetSpeedScale_Request,
    SetSpeedScale_Response,
};
use std::sync::Arc;

use super::command::{CommandLink, SafetyLock};
use super::parameters::VelocityScale;
use super::protocol;

/// The four services share one worker (the equivalent of an rclcpp MutuallyExclusive callback
/// group): they run one at a time, but on a different thread from the cmd_vel worker, so the
/// two can run concurrently.
pub struct Services {
    _worker: Worker<ServiceContext>,
    _set_policy: WorkerService<SetPolicyMode, ServiceContext>,
    _set_motion_key: WorkerService<SetMotionKey, ServiceContext>,
    _set_speed_scale: WorkerService<SetSpeedScale, ServiceContext>,
    _set_safety_lock: WorkerService<SetBool, ServiceContext>,
}

/// Payload of the services worker; every callback receives `&mut ServiceContext`.
///
/// Everything in it is an Arc or otherwise thread-safe, because it is also used outside the
/// worker: cmd_vel reads the velocity scales, and the telemetry thread will need to trigger the
/// safety lock.
pub struct ServiceContext {
    pub velocity_scale: Arc<VelocityScale>,
    pub command: Arc<CommandLink>,
    pub safety: Arc<SafetyLock>,
    pub logger: Logger,
}

impl Services {
    pub fn create(node: &Node, context: ServiceContext) -> Result<Self, RclrsError> {
        let worker = node.create_worker(context);

        Ok(Self {
            _set_policy: worker.create_service::<SetPolicyMode, _>(
                "set_policy_mode",
                |ctx: &mut ServiceContext, req: SetPolicyMode_Request| {
                    ctx.command.send(&protocol::policy_mode_command(req.mode));
                    SetPolicyMode_Response {
                        success: true,
                        message: "Policy updated".into(),
                    }
                },
            )?,
            _set_motion_key: worker.create_service::<SetMotionKey, _>(
                "set_motion_key",
                |ctx: &mut ServiceContext, req: SetMotionKey_Request| {
                    set_motion_key(&ctx.command, &ctx.safety, &req.key)
                },
            )?,
            _set_speed_scale: worker.create_service::<SetSpeedScale, _>(
                "set_speed_scale",
                |ctx: &mut ServiceContext, req: SetSpeedScale_Request| {
                    set_speed_scale(&ctx.velocity_scale, &ctx.logger, req)
                },
            )?,
            _set_safety_lock: worker.create_service::<SetBool, _>(
                "set_safety_lock",
                |ctx: &mut ServiceContext, req: SetBool_Request| {
                    set_safety_lock(&ctx.safety, &ctx.logger, req.data)
                },
            )?,
            _worker: worker,
        })
    }
}

fn set_motion_key(command: &CommandLink, safety: &SafetyLock, key: &str) -> SetMotionKey_Response {
    let response = |success: bool, message: String| SetMotionKey_Response { success, message };

    // While locked only the emergency stop passes; unlock with set_safety_lock, not a motion key
    if safety.is_engaged() && key != protocol::ESTOP_KEY {
        return response(false, "LOCKED".into());
    }

    match protocol::motion_key_command(key) {
        Some(wire) => {
            command.send(wire);
            if key == protocol::ESTOP_KEY {
                response(true, "Emergency stop sent".into())
            } else {
                response(true, "Motion key sent".into())
            }
        }
        None => response(false, format!("Unknown motion key '{key}'")),
    }
}

/// `true` engages the lock, `false` releases it. Engaging only blocks control; unlike
/// `SafetyLock::trigger` it sends no lie-down command. Setting the state it is already in is not
/// an error, so `success` is always true and `message` says whether anything changed.
fn set_safety_lock(safety: &SafetyLock, logger: &Logger, lock: bool) -> SetBool_Response {
    let message = if lock {
        if safety.engage() {
            log_warn!(logger, "[Safety] Safety lock engaged; control blocked");
            "Safety lock engaged. Remote control blocked."
        } else {
            "System was already locked."
        }
    } else if safety.release() {
        log_info!(logger, "[Safety] Safety lock released; control restored");
        "Safety lock released. Remote control restored."
    } else {
        "System was not locked."
    };

    SetBool_Response {
        success: true,
        message: message.into(),
    }
}

fn set_speed_scale(
    scale: &VelocityScale,
    logger: &Logger,
    req: SetSpeedScale_Request,
) -> SetSpeedScale_Response {
    let success = [
        scale.forward.set(req.fwd_scale),
        scale.backward.set(req.back_scale),
        scale.left.set(req.left_scale),
        scale.right.set(req.right_scale),
        scale.angular_left.set(req.turn_l_scale),
        scale.angular_right.set(req.turn_r_scale),
    ]
    .iter()
    .all(Result::is_ok);

    log_info!(
        logger,
        "[SetSpeedScale] Speed scales {}: F: {:.2}, B: {:.2}, L: {:.2}, R: {:.2}, TL: {:.2}, TR: {:.2}",
        if success {
            "updated"
        } else {
            "partially rejected (must be >= 0.0)"
        },
        scale.forward.get(),
        scale.backward.get(),
        scale.left.get(),
        scale.right.get(),
        scale.angular_left.get(),
        scale.angular_right.get(),
    );

    SetSpeedScale_Response { success }
}

#[cfg(test)]
mod tests {
    use std::net::UdpSocket;
    use std::time::Duration;

    use super::super::session::UdpSession;
    use super::*;

    /// A `CommandLink` connected to a loopback listener, plus the listener to read it from
    fn loopback_link() -> (CommandLink, UdpSocket) {
        let listener = UdpSocket::bind("127.0.0.1:0").unwrap();
        listener
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let session = UdpSession::connect(listener.local_addr().unwrap()).unwrap();
        (CommandLink::new(session, Logger::default()), listener)
    }

    fn recv(listener: &UdpSocket) -> String {
        let mut buf = [0u8; 64];
        let n = listener.recv(&mut buf).unwrap();
        String::from_utf8_lossy(&buf[..n]).into_owned()
    }

    fn assert_nothing_received(listener: &UdpSocket) {
        let mut buf = [0u8; 64];
        assert!(listener.recv(&mut buf).is_err(), "unexpected datagram");
    }

    #[test]
    fn motion_key_is_sent_when_unlocked() {
        let (command, listener) = loopback_link();
        let safety = SafetyLock::default();

        let resp = set_motion_key(&command, &safety, "0");
        assert!(resp.success);
        assert_eq!(resp.message, "Motion key sent");
        assert_eq!(recv(&listener), "MODE Z\n");

        let resp = set_motion_key(&command, &safety, protocol::ESTOP_KEY);
        assert!(resp.success);
        assert_eq!(resp.message, "Emergency stop sent");
        assert_eq!(recv(&listener), "ESTOP\n");
    }

    #[test]
    fn unknown_motion_key_sends_nothing() {
        let (command, listener) = loopback_link();
        let safety = SafetyLock::default();

        let resp = set_motion_key(&command, &safety, "9");
        assert!(!resp.success);
        assert_eq!(resp.message, "Unknown motion key '9'");
        assert_nothing_received(&listener);
    }

    #[test]
    fn safety_lock_lies_down_once_and_only_lets_estop_through() {
        let (command, listener) = loopback_link();
        let safety = SafetyLock::default();
        let logger = Logger::default();

        safety.trigger("test", &command, &logger);
        safety.trigger("test again", &command, &logger);
        assert!(safety.is_engaged());
        assert_eq!(recv(&listener), protocol::LIE_DOWN_COMMAND);
        assert_nothing_received(&listener);

        let resp = set_motion_key(&command, &safety, "1");
        assert!(!resp.success);
        assert_eq!(resp.message, "LOCKED");
        assert_nothing_received(&listener);

        let resp = set_motion_key(&command, &safety, protocol::ESTOP_KEY);
        assert!(resp.success);
        assert_eq!(recv(&listener), "ESTOP\n");

        assert!(safety.release());
        assert!(!safety.release());
        assert!(set_motion_key(&command, &safety, "1").success);
        assert_eq!(recv(&listener), "MODE C\n");
    }

    #[test]
    fn set_safety_lock_toggles_without_sending_commands() {
        let (command, listener) = loopback_link();
        let safety = SafetyLock::default();
        let logger = Logger::default();

        let resp = set_safety_lock(&safety, &logger, false);
        assert!(resp.success);
        assert_eq!(resp.message, "System was not locked.");
        assert!(!safety.is_engaged());

        let resp = set_safety_lock(&safety, &logger, true);
        assert!(resp.success);
        assert_eq!(resp.message, "Safety lock engaged. Remote control blocked.");
        assert!(safety.is_engaged());

        let resp = set_safety_lock(&safety, &logger, true);
        assert!(resp.success);
        assert_eq!(resp.message, "System was already locked.");
        assert!(safety.is_engaged());

        // Engaging from outside sends no lie-down, but still gates motion keys
        assert!(!set_motion_key(&command, &safety, "1").success);
        assert_nothing_received(&listener);

        let resp = set_safety_lock(&safety, &logger, false);
        assert!(resp.success);
        assert_eq!(
            resp.message,
            "Safety lock released. Remote control restored."
        );
        assert!(!safety.is_engaged());
        assert_nothing_received(&listener);
    }
}
