use rclrs::*;
use ros_env::std_srvs::srv::{SetBool, SetBool_Request, SetBool_Response};
use ros_env::syncai_common::srv::{
    SetMotionKey, SetMotionKey_Request, SetMotionKey_Response, SetPolicyMode,
    SetPolicyMode_Request, SetPolicyMode_Response, SetSpeedScale, SetSpeedScale_Request,
    SetSpeedScale_Response,
};
use std::sync::Arc;

use super::command::{CommandLink, SafetyLock};
use super::drive::{Drive, M20_UNSUPPORTED};
use super::m20_link::M20Link;
use super::m20_protocol;
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
/// worker: cmd_vel reads the drive (and the G23's velocity scales in it), and the telemetry
/// thread will need to trigger the safety lock.
pub struct ServiceContext {
    pub drive: Arc<Drive>,
    pub safety: Arc<SafetyLock>,
    pub logger: Logger,
}

impl Services {
    pub fn create(node: &Node, context: ServiceContext) -> Result<Self, RclrsError> {
        let worker = node.create_worker(context);

        Ok(Self {
            _set_policy: worker.create_service::<SetPolicyMode, _>(
                "set_policy_mode",
                |ctx: &mut ServiceContext, req: SetPolicyMode_Request| match &*ctx.drive {
                    Drive::G23 { command, .. } => {
                        command.send(&protocol::policy_mode_command(req.mode));
                        SetPolicyMode_Response {
                            success: true,
                            message: "Policy updated".into(),
                        }
                    }
                    Drive::M20(_) => SetPolicyMode_Response {
                        success: false,
                        message: M20_UNSUPPORTED.into(),
                    },
                },
            )?,
            _set_motion_key: worker.create_service::<SetMotionKey, _>(
                "set_motion_key",
                |ctx: &mut ServiceContext, req: SetMotionKey_Request| match &*ctx.drive {
                    Drive::G23 { command, .. } => set_motion_key(command, &ctx.safety, &req.key),
                    Drive::M20(m20) => set_motion_key_m20(m20, &ctx.safety, &req.key),
                },
            )?,
            _set_speed_scale: worker.create_service::<SetSpeedScale, _>(
                "set_speed_scale",
                |ctx: &mut ServiceContext, req: SetSpeedScale_Request| match &*ctx.drive {
                    Drive::G23 { scale, .. } => set_speed_scale(scale, &ctx.logger, req),
                    // The gains are the G23 gait's and not declared on an M20, which maps
                    // cmd_vel through its own max_* (read only).
                    // The response has no message field, so the refusal is only a false.
                    Drive::M20(_) => SetSpeedScale_Response { success: false },
                },
            )?,
            _set_safety_lock: worker.create_service::<SetBool, _>(
                "set_safety_lock",
                |ctx: &mut ServiceContext, req: SetBool_Request| match &*ctx.drive {
                    Drive::G23 { command, .. } => {
                        set_safety_lock(command, &ctx.safety, &ctx.logger, req.data)
                    }
                    // RESET_SAFETY is the G23 controller's; the M20 path has no lock of its own
                    // to engage or release yet
                    Drive::M20(_) => SetBool_Response {
                        success: false,
                        message: M20_UNSUPPORTED.into(),
                    },
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

/// The M20's `set_motion_key`: the G23's keys "0" (stand) and "2" (lie down) only, so the same
/// calls stand and lie either robot down. Every other key is refused with nothing sent: the
/// G23's locomotion / damping / MPC have no M20 counterpart, and ESTOP is not mapped yet.
fn set_motion_key_m20(m20: &M20Link, safety: &SafetyLock, key: &str) -> SetMotionKey_Response {
    let response = |success: bool, message: String| SetMotionKey_Response { success, message };

    // Nothing engages the lock on the M20 today (set_safety_lock is refused); kept so a future
    // trigger gates it the same way as on the G23
    if safety.is_engaged() {
        return response(false, "LOCKED".into());
    }

    match m20_protocol::motion_param_for_key(key) {
        Some(param) => {
            m20.set_motion_state(param);
            response(true, "Motion key sent".into())
        }
        None => response(
            false,
            format!("Motion key '{key}' is not supported on the M20 (0 stand, 2 lie down)"),
        ),
    }
}

/// `true` engages the lock, `false` releases it. Engaging only blocks control; unlike
/// `SafetyLock::trigger` it sends no lie-down command. Setting the state it is already in is not
/// an error, so `success` is always true and `message` says whether anything changed.
///
/// Releasing also sends `RESET_SAFETY`, **every time**, whether or not this node's lock was
/// engaged: the controller locks itself (ESTOP, damping, a lost-control judgement) without this
/// node knowing, and before this nothing on the ROS side could release that lock, so the robot sat
/// in damping until the controller was restarted. On a controller that is not locked the command
/// is benign: RL / MPC / lying down ignore the target it sets, and it only zeroes the velocity the
/// controller holds, which the next cmd_vel restores.
fn set_safety_lock(
    command: &CommandLink,
    safety: &SafetyLock,
    logger: &Logger,
    lock: bool,
) -> SetBool_Response {
    let message = if lock {
        if safety.engage() {
            log_warn!(logger, "[Safety] Safety lock engaged; control blocked");
            "Safety lock engaged. Remote control blocked."
        } else {
            "System was already locked."
        }
    } else {
        // Reset the controller first, so no cmd_vel let through by the release below reaches a
        // controller that is still forcing damping
        command.send(protocol::RESET_SAFETY_COMMAND);
        log_info!(
            logger,
            "[Safety] RESET_SAFETY sent; the controller waits for a stand command (motion key 0)"
        );
        if safety.release() {
            log_info!(logger, "[Safety] Safety lock released; control restored");
            "Safety lock released and controller reset. Send motion key 0 to stand up."
        } else {
            "Controller reset. Send motion key 0 to stand up."
        }
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
#[path = "tests/service.rs"]
mod tests;
