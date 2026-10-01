use rclrs::*;
use ros_env::geometry_msgs::msg::Twist;
use std::sync::Arc;

use super::command::CommandLink;
use super::parameters::VelocityScale;
use super::protocol;

/// cmd_vel 自己一個 worker（相當於 rclcpp 的一個 MutuallyExclusive callback group）：
/// 高頻的指令流在自己的 thread 上跑，不會被 service 卡住。
pub struct Subscribers {
    _worker: Worker<CmdVelContext>,
    _cmd_vel: WorkerSubscription<Twist, CmdVelContext>,
}

/// cmd_vel worker 的 payload
pub struct CmdVelContext {
    pub velocity_scale: Arc<VelocityScale>,
    pub command: Arc<CommandLink>,
}

impl Subscribers {
    pub fn create(node: &Node, context: CmdVelContext) -> Result<Self, RclrsError> {
        let worker = node.create_worker(context);

        Ok(Self {
            // 沒有 watchdog：上游停止發 cmd_vel 時這裡也只是停止送 AXES，不會補送停止指令；
            // 機器人會不會停下來，取決於下位機自己的 timeout。
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

/// 四足的平面指令：前進速度、側向速度、yaw rate，依正負號挑該方向的增益
fn scaled(scale: &VelocityScale, msg: &Twist) -> (f64, f64, f64) {
    let pick = |v: f64, pos: &MandatoryParameter<f64>, neg: &MandatoryParameter<f64>| {
        if v >= 0.0 {
            v * pos.get()
        } else {
            v * neg.get()
        }
    };

    // 參考實作的註解說控制器的轉向正負號跟 REP 103 相反、要先取負號，但程式碼並沒有取。
    // 這裡照程式碼的行為（不取負號）；上實機確認之前兩種說法都不要相信。
    (
        pick(msg.linear.x, &scale.forward, &scale.backward),
        pick(msg.linear.y, &scale.left, &scale.right),
        pick(msg.angular.z, &scale.angular_left, &scale.angular_right),
    )
}
