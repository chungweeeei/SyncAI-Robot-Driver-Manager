//! Conversion between low-level UDP packets and data structures.
//!
//! Pure functions only: no sockets, no Node and no ROS message types, so everything here can
//! be tested with `cargo test` without a ROS environment or the real robot.
//!
//! The gait controller speaks ASCII in both directions:
//!
//! * Commands (out): `AXES <vx> <vy> <wz>\n`, `MODE <char>\n`, `MODE <uint>\n`, `ESTOP\n`
//! * Telemetry (in): one line per datagram, made of whitespace-separated sections. Each section
//!   is a keyword followed by its values, up to the next keyword; a datagram may carry any
//!   subset of sections:
//!
//!   ```text
//!   IMU_RPY r p y  ACC ax ay az  OMEGA wx wy wz  JOINT_POS q0 ... q11  MODE_STATE pol mot
//!   ```
//!
//!   `BMS_V2` is the exception: it always starts the line and is alone in its datagram.

use std::fmt;

/// Number of values in each JOINT_* section (quadruped: 4 legs x 3 DOF)
pub const NUM_DOF: usize = 12;

/// Value order within each JOINT_* section, using the G23 URDF's actuated joint names (the
/// Ankle joints are fixed and not reported).
// TODO: confirm this matches the controller's joint order; motor_states names are assigned
//       positionally.
pub const JOINT_NAMES: [&str; NUM_DOF] = [
    "FL_HipX_joint",
    "FL_HipY_joint",
    "FL_Knee_joint",
    "FR_HipX_joint",
    "FR_HipY_joint",
    "FR_Knee_joint",
    "HL_HipX_joint",
    "HL_HipY_joint",
    "HL_Knee_joint",
    "HR_HipX_joint",
    "HR_HipY_joint",
    "HR_Knee_joint",
];

/// All section keywords; used to find where a section's values end
const KEYWORDS: [&str; 10] = [
    "BMS_V2",
    "IMU_RPY",
    "ACC",
    "OMEGA",
    "JOINT_POS",
    "JOINT_VEL",
    "JOINT_TAU",
    "JOINT_TEMP",
    "JOINT_ERR",
    "MODE_STATE",
];

/// Minimum number of BMS_V2 values (voltage current soc ... temp1 temp2)
const BMS_MIN_VALUES: usize = 8;

/// Lies the robot down; sent when the safety lock is triggered
pub const LIE_DOWN_COMMAND: &str = "MODE X\n";

/// The emergency-stop motion key. It is not a MODE character, and it is the only key let
/// through while the safety lock is engaged.
pub const ESTOP_KEY: &str = "4";

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

pub fn axes_command(vx: f64, vy: f64, wz: f64) -> String {
    format!("AXES {vx:.6} {vy:.6} {wz:.6}\n")
}

/// Per-direction cmd_vel -> AXES correction gains, a plain snapshot of the `VelocityScale`
/// parameters
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityGains {
    pub forward: f64,
    pub backward: f64,
    pub left: f64,
    pub right: f64,
    pub angular_left: f64,
    pub angular_right: f64,
}

/// Scales a planar command `[vx, vy, wz]` by the gain for each component's direction, picked by
/// sign (zero counts as positive). Returns `(vx, vy, wz)` ready for `axes_command`.
// The reference implementation's comment says the controller's turn sign is opposite to
// REP 103 and must be negated, but its code does not negate.
// This follows the code (no negation); trust neither until verified on hardware.
pub fn scale_velocity([vx, vy, wz]: [f64; 3], gains: &VelocityGains) -> (f64, f64, f64) {
    let pick = |v: f64, pos: f64, neg: f64| if v >= 0.0 { v * pos } else { v * neg };
    (
        pick(vx, gains.forward, gains.backward),
        pick(vy, gains.left, gains.right),
        pick(wz, gains.angular_left, gains.angular_right),
    )
}

/// `set_policy_mode`: MODE followed by a **number** (the RL policy index)
pub fn policy_mode_command(mode: u8) -> String {
    format!("MODE {mode}\n")
}

/// `set_motion_key`: the service contract is a numeric string "0"-"5"; MODE is followed by a
/// **character**.
/// This and `policy_mode_command` are two commands on the same keyword; the controller tells
/// them apart by the argument.
pub fn motion_key_command(key: &str) -> Option<&'static str> {
    match key {
        "0" => Some("MODE Z\n"), // Stand
        "1" => Some("MODE C\n"), // Locomotion (RL)
        "2" => Some("MODE X\n"), // Lie down
        "3" => Some("MODE R\n"), // Damping
        ESTOP_KEY => Some("ESTOP\n"),
        "5" => Some("MODE M\n"), // MPC
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Battery {
    pub voltage: f32,
    pub current: f32,
    /// State of charge as reported by the BMS, 0-100 (not 0-1)
    pub soc: f32,
    /// Mean of the two temperature sensors
    pub temperature: f32,
}

/// A non-BMS datagram; sections that are absent or fail to parse are None
#[derive(Debug, Default, PartialEq)]
pub struct Sections {
    pub rpy: Option<[f32; 3]>,
    pub acc: Option<[f32; 3]>,
    pub omega: Option<[f32; 3]>,
    pub joint_pos: Option<[f32; NUM_DOF]>,
    pub joint_vel: Option<[f32; NUM_DOF]>,
    pub joint_tau: Option<[f32; NUM_DOF]>,
    pub joint_temp: Option<[f32; NUM_DOF]>,
    pub joint_err: Option<[i32; NUM_DOF]>,
    /// [policy state, motion state]
    pub mode_state: Option<[i32; 2]>,
}

impl Sections {
    /// Publish IMUState if any IMU section is present; missing fields are zeroed
    // TODO: confirm the firmware always packs IMU_RPY / ACC / OMEGA into one datagram;
    //       if it splits them, this publishes IMUState with zeroed fields instead of
    //       dropping it.
    pub fn has_imu(&self) -> bool {
        self.rpy.is_some() || self.acc.is_some() || self.omega.is_some()
    }

    /// Publish MotorStates if any JOINT_* section is present; missing fields are zeroed
    pub fn has_joints(&self) -> bool {
        self.joint_pos.is_some()
            || self.joint_vel.is_some()
            || self.joint_tau.is_some()
            || self.joint_temp.is_some()
            || self.joint_err.is_some()
    }
}

// Each datagram is consumed right after parsing and never stored, so the size difference does
// not matter and is not worth a heap allocation per packet
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Telemetry {
    Battery(Battery),
    Sections(Sections),
}

/// Why a section was skipped; other sections in the same datagram are unaffected
#[derive(Debug, PartialEq)]
pub enum ParseWarning {
    TooFewValues {
        section: &'static str,
        expected: usize,
        got: usize,
    },
    InvalidToken {
        section: &'static str,
        token: String,
    },
}

impl fmt::Display for ParseWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewValues {
                section,
                expected,
                got,
            } => write!(f, "{section}: expected {expected} values, got {got}"),
            Self::InvalidToken { section, token } => {
                write!(f, "{section}: invalid token '{token}'")
            }
        }
    }
}

/// Parses one telemetry datagram.
///
/// Parsing is defensive: a section with too few values, or a token that is not a finite number,
/// is skipped and reported in the warnings instead of producing garbage; other sections in the
/// same datagram are unaffected.
/// An empty datagram returns `(None, [])`.
pub fn parse_telemetry(line: &str) -> (Option<Telemetry>, Vec<ParseWarning>) {
    let mut parser = SectionParser {
        tokens: line.split_whitespace().collect(),
        warnings: Vec::new(),
    };

    let telemetry = match parser.tokens.first() {
        None => None,
        Some(&"BMS_V2") => parser.battery().map(Telemetry::Battery),
        Some(_) => Some(Telemetry::Sections(Sections {
            rpy: parser.values("IMU_RPY", parse_f32),
            acc: parser.values("ACC", parse_f32),
            omega: parser.values("OMEGA", parse_f32),
            joint_pos: parser.values("JOINT_POS", parse_f32),
            joint_vel: parser.values("JOINT_VEL", parse_f32),
            joint_tau: parser.values("JOINT_TAU", parse_f32),
            joint_temp: parser.values("JOINT_TEMP", parse_f32),
            joint_err: parser.values("JOINT_ERR", parse_i32),
            mode_state: parser.values("MODE_STATE", parse_i32),
        })),
    };

    (telemetry, parser.warnings)
}

/// Derives a quaternion from RPY (ZYX, radians assumed), ordered **[w, x, y, z]**.
// TODO: confirm IMU_RPY is in radians, not degrees; if degrees, orientation is nonsense.
pub fn quaternion_from_rpy([roll, pitch, yaw]: [f32; 3]) -> [f32; 4] {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

fn parse_f32(token: &str) -> Option<f32> {
    token.parse::<f32>().ok().filter(|v| v.is_finite())
}

fn parse_i32(token: &str) -> Option<i32> {
    // Same as the reference implementation (strtol, then cast to int): only out of i64 range is
    // an error; out of i32 range is truncated
    token.parse::<i64>().ok().map(|v| v as i32)
}

struct SectionParser<'a> {
    tokens: Vec<&'a str>,
    warnings: Vec<ParseWarning>,
}

impl SectionParser<'_> {
    /// Finds the (first) `section` keyword and takes N values from what follows it, up to the
    /// next keyword.
    /// More than N values is fine; fewer than N, or any value failing to parse, skips the whole
    /// section.
    fn values<T: Copy + Default, const N: usize>(
        &mut self,
        section: &'static str,
        parse: fn(&str) -> Option<T>,
    ) -> Option<[T; N]> {
        let start = self.tokens.iter().position(|&t| t == section)? + 1;
        let raw: Vec<&str> = self.tokens[start..]
            .iter()
            .copied()
            .take_while(|t| !KEYWORDS.contains(t))
            .collect();

        if raw.len() < N {
            self.warnings.push(ParseWarning::TooFewValues {
                section,
                expected: N,
                got: raw.len(),
            });
            return None;
        }

        let mut out = [T::default(); N];
        for (slot, token) in out.iter_mut().zip(raw) {
            let Some(value) = parse(token) else {
                self.warnings.push(ParseWarning::InvalidToken {
                    section,
                    token: token.to_owned(),
                });
                return None;
            };
            *slot = value;
        }
        Some(out)
    }

    // TODO: trigger a safety shutdown when soc < 20%. The judgement has moved to
    //       syncai_robot_state (RobotStatus::WARNING below 20%, cleared above 25%); what is
    //       missing is the actuation: that node only reports, and this one exposes no service
    //       for it to call, so nothing lies the robot down yet.
    fn battery(&mut self) -> Option<Battery> {
        let got = self.tokens.len() - 1;
        if got < BMS_MIN_VALUES {
            self.warnings.push(ParseWarning::TooFewValues {
                section: "BMS_V2",
                expected: BMS_MIN_VALUES,
                got,
            });
            return None;
        }

        // As lenient as the reference implementation (strtod): an unparsable field becomes 0
        // rather than dropping the whole packet
        let value = |i: usize| self.tokens[i].parse::<f32>().unwrap_or(0.0);
        Some(Battery {
            voltage: value(1),
            current: value(2),
            soc: value(3),
            temperature: (value(7) + value(8)) / 2.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sections(line: &str) -> (Sections, Vec<ParseWarning>) {
        match parse_telemetry(line) {
            (Some(Telemetry::Sections(s)), w) => (s, w),
            other => panic!("expected sections, got {other:?}"),
        }
    }

    fn joints(start: f32) -> String {
        (0..NUM_DOF)
            .map(|i| (start + i as f32).to_string())
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn empty_datagram() {
        assert_eq!(parse_telemetry(""), (None, vec![]));
        assert_eq!(parse_telemetry(" \n\t "), (None, vec![]));
    }

    #[test]
    fn battery() {
        let (t, w) = parse_telemetry("BMS_V2 48.5 -2.25 87 x x x 30 34 3.7 3.7\n");
        assert!(w.is_empty());
        assert_eq!(
            t,
            Some(Telemetry::Battery(Battery {
                voltage: 48.5,
                current: -2.25,
                soc: 87.0,
                temperature: 32.0,
            }))
        );
    }

    #[test]
    fn battery_too_short() {
        let (t, w) = parse_telemetry("BMS_V2 48.5 1 87");
        assert_eq!(t, None);
        assert_eq!(
            w,
            vec![ParseWarning::TooFewValues {
                section: "BMS_V2",
                expected: 8,
                got: 3
            }]
        );
    }

    #[test]
    fn full_datagram() {
        let line = format!(
            "IMU_RPY 0.1 0.2 0.3 ACC 0 0 9.8 OMEGA 1 2 3 JOINT_POS {} JOINT_ERR {} MODE_STATE 1 8",
            joints(0.0),
            joints(100.0),
        );
        let (s, w) = sections(&line);
        assert!(w.is_empty(), "{w:?}");
        assert_eq!(s.rpy, Some([0.1, 0.2, 0.3]));
        assert_eq!(s.acc, Some([0.0, 0.0, 9.8]));
        assert_eq!(s.omega, Some([1.0, 2.0, 3.0]));
        assert_eq!(s.joint_pos.unwrap()[11], 11.0);
        assert_eq!(s.joint_err.unwrap()[0], 100);
        assert_eq!(s.joint_vel, None);
        assert_eq!(s.mode_state, Some([1, 8]));
        assert!(s.has_imu() && s.has_joints());
    }

    #[test]
    fn sections_in_any_order_and_subset() {
        let (s, w) = sections("MODE_STATE 0 1 OMEGA 1 2 3");
        assert!(w.is_empty());
        assert_eq!(s.mode_state, Some([0, 1]));
        assert_eq!(s.omega, Some([1.0, 2.0, 3.0]));
        assert!(s.has_imu());
        assert!(!s.has_joints());
    }

    #[test]
    fn short_section_is_skipped_others_kept() {
        let (s, w) = sections("IMU_RPY 0.1 0.2 ACC 1 2 3");
        assert_eq!(s.rpy, None);
        assert_eq!(s.acc, Some([1.0, 2.0, 3.0]));
        assert_eq!(
            w,
            vec![ParseWarning::TooFewValues {
                section: "IMU_RPY",
                expected: 3,
                got: 2
            }]
        );
    }

    #[test]
    fn invalid_or_non_finite_token_skips_section() {
        let (s, w) = sections("ACC 1 abc 3 OMEGA 1 inf 3 MODE_STATE 1.5 2");
        assert_eq!((s.acc, s.omega, s.mode_state), (None, None, None));
        assert_eq!(w.len(), 3);
        assert_eq!(w[0].to_string(), "ACC: invalid token 'abc'");
    }

    #[test]
    fn extra_values_are_ignored() {
        let (s, w) = sections("MODE_STATE 3 4 5");
        assert!(w.is_empty());
        assert_eq!(s.mode_state, Some([3, 4]));
    }

    #[test]
    fn unknown_tokens_only() {
        let (s, w) = sections("HELLO 1 2 3");
        assert_eq!(s, Sections::default());
        assert!(w.is_empty());
    }

    #[test]
    fn identity_and_yaw_quaternion() {
        assert_eq!(quaternion_from_rpy([0.0; 3]), [1.0, 0.0, 0.0, 0.0]);

        let q = quaternion_from_rpy([0.0, 0.0, std::f32::consts::FRAC_PI_2]);
        let h = std::f32::consts::FRAC_1_SQRT_2;
        for (a, b) in q.iter().zip([h, 0.0, 0.0, h]) {
            assert!((a - b).abs() < 1e-6, "{q:?}");
        }
    }

    #[test]
    fn scale_velocity_picks_gain_by_sign() {
        let gains = VelocityGains {
            forward: 1.0,
            backward: 2.0,
            left: 3.0,
            right: 4.0,
            angular_left: 5.0,
            angular_right: 6.0,
        };
        assert_eq!(scale_velocity([1.0, 1.0, 1.0], &gains), (1.0, 3.0, 5.0));
        assert_eq!(
            scale_velocity([-1.0, -1.0, -1.0], &gains),
            (-2.0, -4.0, -6.0)
        );
        // Zero is "positive": no sign flip and no NaN
        assert_eq!(scale_velocity([0.0; 3], &gains), (0.0, 0.0, 0.0));
        // Each axis picks independently
        assert_eq!(scale_velocity([0.5, -0.5, 2.0], &gains), (0.5, -2.0, 10.0));
    }

    #[test]
    fn commands() {
        assert_eq!(
            axes_command(0.5, -0.25, 1.0),
            "AXES 0.500000 -0.250000 1.000000\n"
        );
        assert_eq!(policy_mode_command(2), "MODE 2\n");
        assert_eq!(motion_key_command("0"), Some("MODE Z\n"));
        assert_eq!(motion_key_command(ESTOP_KEY), Some("ESTOP\n"));
        assert_eq!(motion_key_command("5"), Some("MODE M\n"));
        assert_eq!(motion_key_command("6"), None);
        assert_eq!(motion_key_command("mpc"), None);
    }
}
