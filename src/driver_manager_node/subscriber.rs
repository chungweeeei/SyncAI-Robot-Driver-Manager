use rclrs::*;
use ros_env::geometry_msgs::msg::Twist;
use std::sync::Arc;

use super::command::{CommandLink, SafetyLock};
use super::parameters::VelocityScale;
use super::protocol;

/// cmd_vel gets its own worker (the equivalent of an rclcpp MutuallyExclusive callback group):
/// the high-rate command stream runs on its own thread and is never blocked by a service.
pub struct Subscribers {
    _worker: Worker<CmdVelContext>,
    _cmd_vel: WorkerSubscription<Twist, CmdVelContext>,
}

/// Payload of the cmd_vel worker
pub struct CmdVelContext {
    pub velocity_scale: Arc<VelocityScale>,
    pub command: Arc<CommandLink>,
    pub safety: Arc<SafetyLock>,
}

impl Subscribers {
    pub fn create(node: &Node, context: CmdVelContext) -> Result<Self, RclrsError> {
        let worker = node.create_worker(context);

        Ok(Self {
            // No watchdog: when upstream stops publishing cmd_vel this simply stops sending AXES
            // and never sends a stop command. Whether the robot halts is up to the controller's
            // own timeout.
            _cmd_vel: worker.create_subscription(
                "cmd_vel".keep_last(10),
                |ctx: &mut CmdVelContext, msg: Twist| {
                    // While the safety lock is engaged cmd_vel is dropped: nothing goes out on
                    // the command socket until set_safety_lock releases it
                    if ctx.safety.is_engaged() {
                        return;
                    }
                    // Quadruped planar command: forward velocity, lateral velocity, yaw rate
                    let (vx, vy, wz) = protocol::scale_velocity(
                        [msg.linear.x, msg.linear.y, msg.angular.z],
                        &ctx.velocity_scale.gains(),
                    );
                    ctx.command.send(&protocol::axes_command(vx, vy, wz));
                },
            )?,
            _worker: worker,
        })
    }
}
