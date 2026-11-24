//! Unit tests of `protocol.rs`, included from there with `#[path]`: still a child module of
//! `protocol`, so `use super::*` reaches its private items.

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

fn battery(line: &str) -> Battery {
    match parse_telemetry(line) {
        (Some(Telemetry::Battery(b)), w) if w.is_empty() => b,
        other => panic!("expected battery, got {other:?}"),
    }
}

#[test]
fn battery_minimum_packet() {
    let b = battery("BMS_V2 48.5 -2.25 87 x x x 30 34 3.7 3.7\n");
    assert_eq!(
        b,
        Battery {
            voltage: 48.5,
            current: -2.25,
            soc: 87.0,
            temperature: 32.0,
            charge_state: ChargeState::Discharging,
            cells: None,
        }
    );
}

#[test]
fn battery_full_packet_from_bms_rs() {
    // The exact format bms.rs emits; cell 6 has not been read yet
    let b = battery(
        "BMS_V2 30.12 3.45 85.8 1000 1 0 25.0 26.0 27.0 30.0 \
         3.765 3.766 3.767 3.768 3.769 3.770 0.000 3.772\n",
    );
    assert_eq!(b.temperature, 25.5);
    assert_eq!(b.charge_state, ChargeState::Charging);
    let cells = b.cells.unwrap();
    assert_eq!(cells[0], 3.765);
    assert!(cells[6].is_nan());
    assert_eq!(cells[7], 3.772);
}

#[test]
fn battery_missing_thermistors() {
    // One missing: the other is used as-is, not halved
    assert_eq!(battery("BMS_V2 30 0 50 0 0 0 0.0 26.0").temperature, 26.0);
    // Both missing: NaN, BatteryState's "unmeasured"
    assert!(battery("BMS_V2 30 0 50 0 0 0 0.0 0.0").temperature.is_nan());
}

#[test]
fn battery_charge_state_deadband() {
    let state = |i: &str| battery(&format!("BMS_V2 30 {i} 50 0 0 0 25 25")).charge_state;
    assert_eq!(state("0.05"), ChargeState::Idle);
    assert_eq!(state("-0.1"), ChargeState::Idle);
    assert_eq!(state("0.2"), ChargeState::Charging);
    assert_eq!(state("-0.2"), ChargeState::Discharging);
}

#[test]
fn battery_too_short() {
    let (t, w) = parse_telemetry("BMS_V2 48.5 1 87");
    assert_eq!(t, None);
    assert_eq!(w, vec![ParseWarning::BmsTooShort { got: 3 }]);
    assert_eq!(
        w[0].to_string(),
        "BMS_V2: expected 18 values (minimum 8), got 3"
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
