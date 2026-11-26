//! The DEEP Robotics Lynx M20's control protocol, as pure functions.
//!
//! Like `protocol.rs` (the G23 gait controller's ASCII), nothing here touches a socket, a Node
//! or a ROS message type, so all of it runs under `cargo test` without a robot.
//!
//! Source: "Lynx M20 Series Software Development Manual (Beta)" V0.0.9-0 (2025-08-21), section 1
//! ("Inspection Communication Protocol"), cross-checked against DEEP Robotics' own keyboard
//! teleop script. The robot is the UDP server (10.21.31.103:30000 on its internal LAN); every
//! datagram is a 16-byte header followed by one JSON ASDU:
//!
//! ```text
//! EB 91 EB 90 | ASDU length u16 LE | message id u16 LE | 0x01 (JSON) | 7 x 0x00 | ASDU
//! {"PatrolDevice":{"Type":<u32>,"Command":<u32>,"Time":"YYYY-MM-DD HH:MM:SS","Items":{...}}}
//! ```
//!
//! Only what stand / lie down / move / switch gait needs is here: heartbeat (100/100), usage mode
//! (1101/5), motion state (2/22), the axis command (2/21) and the gait switch (2/23), plus reading
//! `MotionState` and `Gait` back out of the robot's basic status report (1002/6). The JSON is written by hand: every body is a fixed
//! shape of numbers, which keeps the package free of a serde dependency.

/// The four synchronisation bytes every datagram starts with, in both directions
pub const SYNC: [u8; 4] = [0xEB, 0x91, 0xEB, 0x90];
pub const HEADER_LEN: usize = 16;
/// Header byte 8: the ASDU is JSON (0x00 would be XML)
const ASDU_JSON: u8 = 0x01;

/// (Type, Command) of each message this node sends
pub const HEARTBEAT: (u32, u32) = (100, 100);
pub const USAGE_MODE: (u32, u32) = (1101, 5);
pub const MOTION_STATE: (u32, u32) = (2, 22);
pub const AXIS: (u32, u32) = (2, 21);
/// Gait switch (manual 1.2.4). Not in DEEP Robotics' keyboard script, which only reads `Gait`
/// back; the (Type, Command) and `GaitParam` values are the manual's, as an earlier in-house M20
/// bridge (`robot_bridge`'s `send_gait_switch`) sent them.
pub const GAIT: (u32, u32) = (2, 23);

/// `Mode` of the usage-mode switch: axis commands are honoured only in Regular mode (0);
/// Navigation mode (1) hands the robot to the vendor's own navigation, Assist (2) to its
/// obstacle avoidance.
pub const USAGE_MODE_REGULAR: i32 = 0;

/// `MotionParam` values of the motion-state switch this node sends (the manual's table also has
/// Idle 0, Power-on Damping 3 and Standard Motion State 6)
pub const MOTION_PARAM_STAND: i32 = 1;
/// Soft Emergency Stop: the M20's counterpart of the G23's `ESTOP` (motion key 4). Like the
/// G23's, it is one command on the link and nothing more: the socket, the heartbeat and the
/// axis path stay up, so a stand (key 0) afterwards reaches the robot without a restart.
pub const MOTION_PARAM_SOFT_ESTOP: i32 = 2;
pub const MOTION_PARAM_SIT: i32 = 4;

/// The `set_motion_key` keys the M20 answers, mapped to its `MotionParam`.
///
/// The keys are the G23's (`syncai_common/SetMotionKey`, see `protocol::motion_key_command`),
/// so a caller written for the G23 stands, lies down and emergency-stops the M20 unchanged.
/// Only those three have an M20 meaning so far; every other key (locomotion, damping, MPC) is
/// refused rather than guessed at.
pub fn motion_param_for_key(key: &str) -> Option<i32> {
    match key {
        "0" => Some(MOTION_PARAM_STAND),
        "2" => Some(MOTION_PARAM_SIT),
        "4" => Some(MOTION_PARAM_SOFT_ESTOP),
        _ => None,
    }
}

/// `GaitParam` of the gait switch: the standard (non-agile) Basic and Stair gaits, the two the
/// axis command drives (manual 1.2.5). The manual's table also has the agile gaits, 0x3002 flat
/// and 0x3003 stairs, which nothing here sends yet.
pub const GAIT_PARAM_BASIC: i32 = 0x1001;
pub const GAIT_PARAM_STAIRS: i32 = 0x1003;

/// The `set_policy_mode` modes the M20 answers, mapped to its `GaitParam`: 0 the flat (Basic)
/// gait, 1 the stair gait. 1 is what the controllers' stair mode sends (`stair_policy_mode: 1`,
/// HIMLOCO on the G23), so the same call climbs on either robot. Every other mode is refused
/// rather than guessed at.
pub fn gait_param_for_policy(mode: u8) -> Option<i32> {
    match mode {
        0 => Some(GAIT_PARAM_BASIC),
        1 => Some(GAIT_PARAM_STAIRS),
        _ => None,
    }
}

/// A human name for a reported `Gait`, for the log. The basic status report numbers its gaits
/// differently from the `GaitParam` sent: DEEP Robotics' keyboard script reads 1 as Basic and 14
/// as Stair, the only two it names.
pub fn gait_name(gait: i64) -> &'static str {
    match gait {
        1 => "basic",
        14 => "stair",
        _ => "unknown",
    }
}

/// A human name for a reported `MotionState`, for the log. The manual lists 0-8; 17 is what
/// DEEP Robotics' teleop script calls "RL control / standing" and is not in the manual.
pub fn motion_state_name(state: i64) -> &'static str {
    match state {
        0 => "idle",
        1 => "stand",
        2 => "soft emergency stop",
        3 => "power-on damping",
        4 => "sitting",
        6 => "standard motion",
        8 => "agile motion",
        17 => "RL control (standing)",
        _ => "unknown",
    }
}

/// cmd_vel [m/s or rad/s] -> the axis command's ratio of the robot's maximum speed.
///
/// The axis command takes no unit: each value in [-1, 1] is a fraction of the gait's maximum
/// speed on that axis, so `max` is that maximum in cmd_vel's unit. A non-finite command or a
/// non-positive `max` gives 0, never a full-speed command.
pub fn velocity_ratio(value: f64, max: f64) -> f64 {
    if !value.is_finite() || max.is_nan() || max <= 0.0 {
        return 0.0;
    }
    (value / max).clamp(-1.0, 1.0)
}

pub fn heartbeat_items() -> String {
    "{}".into()
}

pub fn usage_mode_items(mode: i32) -> String {
    format!("{{\"Mode\":{mode}}}")
}

pub fn motion_state_items(param: i32) -> String {
    format!("{{\"MotionParam\":{param}}}")
}

/// The gait switch's Items, `GaitParam` as a decimal number (JSON has no hex literal)
pub fn gait_items(param: i32) -> String {
    format!("{{\"GaitParam\":{param}}}")
}

/// The axis command's Items. Z / Roll / Pitch are always 0: only X, Y and Yaw act in the Basic
/// and Stair gaits (manual 1.2.5). Each value is clamped to [-1, 1] again here, so nothing out of
/// range reaches the wire whatever the caller did.
pub fn axis_items(x: f64, y: f64, yaw: f64) -> String {
    let c = |v: f64| if v.is_finite() { v.clamp(-1.0, 1.0) } else { 0.0 };
    format!(
        "{{\"X\":{:.4},\"Y\":{:.4},\"Z\":0.0,\"Roll\":0.0,\"Pitch\":0.0,\"Yaw\":{:.4}}}",
        c(x),
        c(y),
        c(yaw)
    )
}

/// The JSON ASDU. `items` must already be a JSON object (one of the `*_items` functions).
pub fn asdu(kind: (u32, u32), time: &str, items: &str) -> String {
    format!(
        "{{\"PatrolDevice\":{{\"Type\":{},\"Command\":{},\"Time\":\"{time}\",\"Items\":{items}}}}}",
        kind.0, kind.1
    )
}

/// Header + ASDU, ready for `send`. The ASDU length field is a u16, so a longer ASDU is refused
/// (nothing this module builds comes close).
pub fn packet(message_id: u16, asdu: &str) -> Option<Vec<u8>> {
    let len = u16::try_from(asdu.len()).ok()?;
    let mut out = Vec::with_capacity(HEADER_LEN + asdu.len());
    out.extend_from_slice(&SYNC);
    out.extend_from_slice(&len.to_le_bytes());
    out.extend_from_slice(&message_id.to_le_bytes());
    out.push(ASDU_JSON);
    out.extend_from_slice(&[0u8; 7]);
    out.extend_from_slice(asdu.as_bytes());
    Some(out)
}

/// The JSON ASDU of a received datagram, or None when it is not one of ours (wrong sync bytes,
/// XML, truncated, not UTF-8)
pub fn parse_packet(datagram: &[u8]) -> Option<&str> {
    if datagram.len() < HEADER_LEN || datagram[..4] != SYNC || datagram[8] != ASDU_JSON {
        return None;
    }
    let len = usize::from(u16::from_le_bytes([datagram[4], datagram[5]]));
    let body = datagram.get(HEADER_LEN..HEADER_LEN + len)?;
    std::str::from_utf8(body).ok()
}

/// `BasicStatus.MotionState` out of a basic status report (1002/6), the only report that
/// carries the key. A plain key search rather than a JSON parser: the value is a bare integer,
/// and the manual's own sample of this report is not valid JSON (full-width colons), so a strict
/// parser would buy nothing but a dependency.
pub fn reported_motion_state(asdu: &str) -> Option<i64> {
    reported_integer(asdu, "MotionState")
}

/// `BasicStatus.Gait` out of a basic status report (1002/6), numbered as `gait_name` reads it
pub fn reported_gait(asdu: &str) -> Option<i64> {
    reported_integer(asdu, "Gait")
}

/// The bare integer after the first `"<key>":` in `asdu`
fn reported_integer(asdu: &str, key: &str) -> Option<i64> {
    let quoted = format!("\"{key}\"");
    let rest = &asdu[asdu.find(&quoted)? + quoted.len()..];
    let rest = rest.trim_start().strip_prefix(':')?.trim_start();
    let end = rest
        .char_indices()
        .find(|&(i, c)| !(c.is_ascii_digit() || (i == 0 && c == '-')))
        .map_or(rest.len(), |(i, _)| i);
    rest[..end].parse().ok()
}

/// "YYYY-MM-DD HH:MM:SS" for the ASDU's `Time` field, from seconds since the Unix epoch, in UTC.
///
/// The manual asks for local time, but std has no time zone database and the field is not known
/// to be read by the robot (its samples all say 2023-01-01). UTC keeps the package free of a
/// chrono dependency; switch if the robot turns out to care.
pub fn timestamp(unix_secs: u64) -> String {
    // Howard Hinnant's civil_from_days
    let days = i64::try_from(unix_secs / 86_400).unwrap_or(0);
    let rem = unix_secs % 86_400;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02}",
        rem / 3_600,
        rem % 3_600 / 60,
        rem % 60
    )
}

#[cfg(test)]
#[path = "tests/m20_protocol.rs"]
mod tests;
