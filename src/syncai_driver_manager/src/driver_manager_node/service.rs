use rclrs::*;
use ros_env::std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};
use ros_env::syncai_common::srv::{
    SetMotionKey, SetMotionKey_Request, SetMotionKey_Response, SetPolicyMode,
    SetPolicyMode_Request, SetPolicyMode_Response, SetSpeedScale, SetSpeedScale_Request,
    SetSpeedScale_Response,
};
use std::sync::Arc;

use super::command::{CommandLink, SafetyLock};
use super::parameters::VelocityScale;
use super::protocol;

/// 四個 service 共用一個 worker（相當於 rclcpp 的一個 MutuallyExclusive callback group）：
/// 彼此依序執行，但跟 cmd_vel 的 worker 在不同 thread，可以同時跑。
pub struct Services {
    _worker: Worker<ServiceContext>,
    _set_policy: WorkerService<SetPolicyMode, ServiceContext>,
    _set_motion_key: WorkerService<SetMotionKey, ServiceContext>,
    _set_speed_scale: WorkerService<SetSpeedScale, ServiceContext>,
    _reset_safety: WorkerService<Trigger, ServiceContext>,
}

/// services worker 的 payload，每個 callback 都會拿到 `&mut ServiceContext`。
///
/// 裡面全是 Arc / thread-safe 的東西，因為它們也會在 worker 之外被用到：
/// 速度增益 cmd_vel 也要讀，safety lock 之後 telemetry thread 也要能觸發。
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
            _reset_safety: worker.create_service::<Trigger, _>(
                "reset_safety",
                |ctx: &mut ServiceContext, _req: Trigger_Request| {
                    if ctx.safety.release() {
                        log_info!(
                            &ctx.logger,
                            "[Safety] Safety lock released; control restored"
                        );
                        Trigger_Response {
                            success: true,
                            message: "Safety lock released. Remote control restored.".into(),
                        }
                    } else {
                        Trigger_Response {
                            success: true,
                            message: "System was not locked.".into(),
                        }
                    }
                },
            )?,
            _worker: worker,
        })
    }
}

fn set_motion_key(command: &CommandLink, safety: &SafetyLock, key: &str) -> SetMotionKey_Response {
    let response = |success: bool, message: String| SetMotionKey_Response { success, message };

    // 鎖住時只放行急停；解鎖要用 reset_safety，不是 motion key
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
