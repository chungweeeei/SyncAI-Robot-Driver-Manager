use super::command::CommandLink;
use super::m20_link::M20Link;
use super::parameters::VelocityScale;

/// The robot this node drives, picked once at startup by the `robot_protocol` parameter.
///
/// The ROS side (cmd_vel, the four services, their names and types) is the same for both, so a
/// caller written for the G23 drives an M20 without knowing it; what each robot accepts behind
/// it differs, and the M20 refuses whatever it has no meaning for yet.
///
/// Each variant owns the parameters only its robot has, so they are declared only on that
/// robot (a parameter is undeclared when its handle drops, and the Drive lives as long as the
/// cmd_vel and service workers that hold it).
pub enum Drive {
    /// The G23 gait controller's ASCII protocol (`protocol.rs`), with the per-direction
    /// velocity scales cmd_vel goes through (`scale_*`, settable by `set_speed_scale`)
    G23 {
        command: CommandLink,
        scale: VelocityScale,
    },
    /// DEEP Robotics' Lynx M20 (`m20_protocol.rs`): stand, lie down, cmd_vel and the gait switch
    /// (`set_policy_mode` 0 flat / 1 stairs) only
    M20(M20Link),
}

/// The message every call the M20 path does not implement answers with
pub const M20_UNSUPPORTED: &str = "Not supported on the M20 (robot_protocol m20)";
