use rclrs::*;
use ros_env::std_srvs::srv::{Trigger, Trigger_Request, Trigger_Response};
use ros_env::syncai_common::srv::{
    SetMotionKey, SetMotionKey_Request, SetMotionKey_Response, SetPolicyMode,
    SetPolicyMode_Request, SetPolicyMode_Response, SetSpeedScale, SetSpeedScale_Request,
    SetSpeedScale_Response,
};
use std::sync::Arc;

use super::parameters::VelocityScale;

pub struct Services {
    pub set_policy: Service<SetPolicyMode>,
    pub set_motion_key: Service<SetMotionKey>,
    pub set_speed_scale: Service<SetSpeedScale>,
    pub restart_safety: Service<Trigger>,
}

impl Services {
    pub fn create(node: &Node, velocity_scale: Arc<VelocityScale>) -> Result<Self, RclrsError> {
        Ok(Self {
            set_policy: node.create_service::<SetPolicyMode, _>(
                "set_policy_mode",
                |req: SetPolicyMode_Request| SetPolicyMode_Response {
                    success: true,
                    message: format!("policy mode = {}", req.mode),
                },
            )?,
            set_motion_key: node.create_service::<SetMotionKey, _>(
                "set_motion_key",
                |req: SetMotionKey_Request| SetMotionKey_Response {
                    success: true,
                    message: format!("motion key = {}", req.key),
                },
            )?,
            set_speed_scale: node.create_service::<SetSpeedScale, _>(
                "set_speed_scale",
                move |req: SetSpeedScale_Request| set_speed_scale(&velocity_scale, req),
            )?,
            restart_safety: node.create_service::<Trigger, _>(
                "restart_safety",
                |_req: Trigger_Request| Trigger_Response {
                    success: true,
                    message: String::new(),
                },
            )?,
        })
    }
}

fn set_speed_scale(scale: &VelocityScale, req: SetSpeedScale_Request) -> SetSpeedScale_Response {
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

    SetSpeedScale_Response { success }
}
