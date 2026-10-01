use rclrs::*;
use ros_env::geometry_msgs::msg::Twist;
use std::sync::Arc;

use super::parameters::VelocityScale;

pub struct Subscribers {
    pub cmd_vel: Subscription<Twist>,
}

impl Subscribers {
    pub fn create(node: &Node, velocity_scale: Arc<VelocityScale>) -> Result<Self, RclrsError> {
        let logger = node.logger().clone();

        Ok(Self {
            cmd_vel: node.create_subscription("cmd_vel".keep_last(10), move |msg: Twist| {
                let vx = if msg.linear.x >= 0.0 {
                    msg.linear.x * velocity_scale.forward.get()
                } else {
                    msg.linear.x * velocity_scale.backward.get()
                };

                let vy = if msg.linear.y >= 0.0 {
                    msg.linear.y * velocity_scale.forward.get()
                } else {
                    msg.linear.y * velocity_scale.backward.get()
                };

                let wz = if msg.angular.z >= 0.0 {
                    msg.angular.z * velocity_scale.angular_left.get()
                } else {
                    msg.angular.z * velocity_scale.angular_right.get()
                };

                log_info!(&logger, "[cmd_vel] vx = {vx}, vy = {vy}, wz = {wz}");
            })?,
        })
    }
}
