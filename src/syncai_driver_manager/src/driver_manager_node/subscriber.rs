use rclrs::*;
use ros_env::geometry_msgs::msg::Twist;
use std::sync::Arc;

use super::command::CommandLink;
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
                    let (vx, vy, wz) = scaled(&ctx.velocity_scale, &msg);
                    ctx.command.send(&protocol::axes_command(vx, vy, wz));
                },
            )?,
            _worker: worker,
        })
    }
}

/// Quadruped planar command: forward velocity, lateral velocity, yaw rate, each scaled by the
/// gain for its direction, picked by sign
fn scaled(scale: &VelocityScale, msg: &Twist) -> (f64, f64, f64) {
    let pick = |v: f64, pos: &MandatoryParameter<f64>, neg: &MandatoryParameter<f64>| {
        if v >= 0.0 {
            v * pos.get()
        } else {
            v * neg.get()
        }
    };

    // The reference implementation's comment says the controller's turn sign is opposite to
    // REP 103 and must be negated, but its code does not negate.
    // This follows the code (no negation); trust neither until verified on hardware.
    (
        pick(msg.linear.x, &scale.forward, &scale.backward),
        pick(msg.linear.y, &scale.left, &scale.right),
        pick(msg.angular.z, &scale.angular_left, &scale.angular_right),
    )
}
