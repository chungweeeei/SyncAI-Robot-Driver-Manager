//! 底層 UDP 封包 <-> 資料結構 的轉換。
//!
//! 這裡只放純函式：不碰 socket、不碰 Node、也不碰 ROS 訊息型別，所以可以直接
//! `cargo test`，不需要 ROS 環境或實機。
//!
//! 下位機（gait controller）兩個方向都是 ASCII：
//!
//! * 指令（送出）：`AXES <vx> <vy> <wz>\n`、`MODE <char>\n`、`MODE <uint>\n`、`ESTOP\n`
//! * telemetry（接收）：一個 datagram 一行，由空白分隔的 section 組成，每個 section 是
//!   關鍵字加上數值，一直到下一個關鍵字為止；一個 datagram 可以只帶任意幾個 section：
//!
//!   ```text
//!   IMU_RPY r p y  ACC ax ay az  OMEGA wx wy wz  JOINT_POS q0 … q11  MODE_STATE pol mot
//!   ```
//!
//!   `BMS_V2` 是特例：它一定在行首，而且整個 datagram 只有它。

use std::fmt;

/// 每個 JOINT_* section 的數值個數（四足：4 腿 × 3 DOF）
pub const NUM_DOF: usize = 12;

/// JOINT_* section 內的數值順序，用 G23 URDF 的主動關節名稱（Ankle 是固定關節，不會回報）。
// TODO: 確認跟控制器的關節順序一致；motor_states 的名稱是照位置硬配的。
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

/// 所有 section 關鍵字；用來判斷一個 section 的數值到哪裡結束
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

/// BMS_V2 至少要有的數值個數（voltage current soc … temp1 temp2）
const BMS_MIN_VALUES: usize = 8;

/// 讓控制器趴下；safety lock 觸發時送的指令
pub const LIE_DOWN_COMMAND: &str = "MODE X\n";

/// 急停的 motion key。它不是 MODE 字元，而且是 safety lock 鎖住時唯一放行的 key。
pub const ESTOP_KEY: &str = "4";

// ---------------------------------------------------------------------------
// 指令
// ---------------------------------------------------------------------------

pub fn axes_command(vx: f64, vy: f64, wz: f64) -> String {
    format!("AXES {vx:.6} {vy:.6} {wz:.6}\n")
}

/// `set_policy_mode`：MODE 後面接的是**數字**（RL policy index）
pub fn policy_mode_command(mode: u8) -> String {
    format!("MODE {mode}\n")
}

/// `set_motion_key`：service 合約是數字字串 "0"–"5"，MODE 後面接的是**字元**。
/// 跟 `policy_mode_command` 是同一個關鍵字的兩種指令，控制器靠參數分辨。
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
    /// BMS 回報的 state-of-charge，0–100（不是 0–1）
    pub soc: f32,
    /// 兩顆溫度感測器的平均
    pub temperature: f32,
}

/// 一個非 BMS 的 datagram；沒出現或解不出來的 section 是 None
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
    /// 任一個 IMU section 在就發 IMUState，缺的欄位補 0
    // TODO: 確認韌體是否一定把 IMU_RPY / ACC / OMEGA 放在同一個 datagram；
    //       若會拆開，這裡會發出欄位歸零的 IMUState 而不是丟掉。
    pub fn has_imu(&self) -> bool {
        self.rpy.is_some() || self.acc.is_some() || self.omega.is_some()
    }

    /// 任一個 JOINT_* section 在就發 MotorStates，缺的欄位補 0
    pub fn has_joints(&self) -> bool {
        self.joint_pos.is_some()
            || self.joint_vel.is_some()
            || self.joint_tau.is_some()
            || self.joint_temp.is_some()
            || self.joint_err.is_some()
    }
}

// 每個 datagram 解完馬上就用掉、不會存起來，大小差距無所謂，不值得每包多一次 heap 配置
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Telemetry {
    Battery(Battery),
    Sections(Sections),
}

/// 某個 section 被略過的原因；不影響同一個 datagram 裡的其他 section
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

/// 解一個 telemetry datagram。
///
/// 防禦式解析：數值不夠、或 token 不是有限數字的 section 會被略過並回報在 warnings，
/// 而不是發出垃圾值；同一個 datagram 裡的其他 section 不受影響。
/// 空的 datagram 回傳 `(None, [])`。
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

/// 由 RPY 推出四元數（ZYX，假設單位是 radian），順序為 **[w, x, y, z]**。
// TODO: 確認 IMU_RPY 的單位是 radian 不是 degree；若是 degree，姿態會完全錯。
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
    // 跟參考實作（strtol 後轉 int）一樣：超出 i64 才算錯，超出 i32 直接截斷
    token.parse::<i64>().ok().map(|v| v as i32)
}

struct SectionParser<'a> {
    tokens: Vec<&'a str>,
    warnings: Vec<ParseWarning>,
}

impl SectionParser<'_> {
    /// 找到 `section` 關鍵字（第一次出現），取它後面直到下一個關鍵字的 N 個數值。
    /// 數值比 N 多沒關係，少於 N 或任一個解不出來就整個 section 略過。
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

    // TODO: soc < 20% 時觸發 safety shutdown。判斷已經移到 syncai_robot_state（低於 20%
    //       回報 RobotStatus::WARNING，25% 解除），缺的是「動作」：那邊只回報，這個節點
    //       也沒有讓它呼叫的 service，所以目前沒有任何東西會讓機器人趴下。
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

        // 跟參考實作（strtod）一樣寬鬆：解不出來的欄位當 0，不整包丟掉
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
