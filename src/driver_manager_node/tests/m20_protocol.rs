//! Unit tests of `m20_protocol.rs`, included from there with `#[path]`: still a child module of
//! `m20_protocol`, so `use super::*` reaches its private items.

use super::*;

#[test]
fn header_layout_matches_the_manual() {
    let body = asdu(HEARTBEAT, "2023-01-01 00:00:00", &heartbeat_items());
    let pkt = packet(0x0102, &body).unwrap();
    assert_eq!(&pkt[..4], &[0xEB, 0x91, 0xEB, 0x90]);
    assert_eq!(u16::from_le_bytes([pkt[4], pkt[5]]) as usize, body.len());
    assert_eq!(&pkt[6..8], &[0x02, 0x01]); // little endian
    assert_eq!(pkt[8], 0x01);
    assert_eq!(&pkt[9..16], &[0u8; 7]);
    assert_eq!(&pkt[16..], body.as_bytes());
    assert_eq!(
        body,
        r#"{"PatrolDevice":{"Type":100,"Command":100,"Time":"2023-01-01 00:00:00","Items":{}}}"#
    );
}

#[test]
fn packet_round_trips() {
    let body = asdu(MOTION_STATE, "t", &motion_state_items(MOTION_PARAM_STAND));
    let pkt = packet(7, &body).unwrap();
    assert_eq!(parse_packet(&pkt), Some(body.as_str()));
    assert_eq!(
        body,
        r#"{"PatrolDevice":{"Type":2,"Command":22,"Time":"t","Items":{"MotionParam":1}}}"#
    );
}

#[test]
fn foreign_or_truncated_datagrams_are_rejected() {
    let pkt = packet(0, "{}").unwrap();
    assert_eq!(parse_packet(&pkt[..pkt.len() - 1]), None);
    assert_eq!(parse_packet(b"AXES 0 0 0\n"), None);
    let mut xml = pkt.clone();
    xml[8] = 0x00;
    assert_eq!(parse_packet(&xml), None);
}

#[test]
fn only_stand_and_lie_down_keys_map() {
    assert_eq!(motion_param_for_key("0"), Some(1));
    assert_eq!(motion_param_for_key("2"), Some(4));
    for key in ["1", "3", "4", "5", "", "x"] {
        assert_eq!(motion_param_for_key(key), None, "key {key:?}");
    }
}

#[test]
fn velocity_ratio_clamps_and_refuses_nonsense() {
    assert_eq!(velocity_ratio(0.5, 2.0), 0.25);
    assert_eq!(velocity_ratio(-5.0, 2.0), -1.0);
    assert_eq!(velocity_ratio(f64::NAN, 2.0), 0.0);
    assert_eq!(velocity_ratio(1.0, 0.0), 0.0);
    assert_eq!(velocity_ratio(1.0, -1.0), 0.0);
}

#[test]
fn axis_items_zero_the_unused_axes_and_clamp() {
    assert_eq!(
        axis_items(0.3, -2.0, f64::INFINITY),
        r#"{"X":0.3000,"Y":-1.0000,"Z":0.0,"Roll":0.0,"Pitch":0.0,"Yaw":0.0000}"#
    );
}

#[test]
fn motion_state_is_read_from_a_basic_status_report() {
    let report = r#"{"PatrolDevice":{"Type":1002,"Command":6,"Time":"t","Items":{"BasicStatus":{"MotionState": 17,"Gait":1}}}}"#;
    assert_eq!(reported_motion_state(report), Some(17));
    assert_eq!(reported_motion_state(r#"{"MotionState":-1}"#), Some(-1));
    assert_eq!(reported_motion_state(r#"{"Gait":1}"#), None);
    assert_eq!(reported_motion_state(r#"{"MotionState":"x"}"#), None);
}

#[test]
fn only_flat_and_stair_policy_modes_map() {
    assert_eq!(gait_param_for_policy(0), Some(0x1001));
    assert_eq!(gait_param_for_policy(1), Some(0x1003));
    assert_eq!(gait_param_for_policy(2), None);
    assert_eq!(gait_param_for_policy(255), None);
}

#[test]
fn gait_switch_is_type_2_command_23_with_a_decimal_gait_param() {
    assert_eq!(gait_items(GAIT_PARAM_STAIRS), r#"{"GaitParam":4099}"#);
    assert_eq!(
        asdu(GAIT, "t", &gait_items(GAIT_PARAM_BASIC)),
        r#"{"PatrolDevice":{"Type":2,"Command":23,"Time":"t","Items":{"GaitParam":4097}}}"#
    );
}

#[test]
fn gait_is_read_from_a_basic_status_report() {
    let report = r#"{"PatrolDevice":{"Type":1002,"Command":6,"Time":"t","Items":{"BasicStatus":{"MotionState":17,"Gait": 14}}}}"#;
    assert_eq!(reported_gait(report), Some(14));
    assert_eq!(gait_name(14), "stair");
    assert_eq!(gait_name(1), "basic");
    assert_eq!(reported_gait(r#"{"MotionState":1}"#), None);
}

#[test]
fn timestamp_is_utc_civil_time() {
    assert_eq!(timestamp(0), "1970-01-01 00:00:00");
    assert_eq!(timestamp(1_700_000_000), "2023-11-14 22:13:20");
    assert_eq!(timestamp(951_782_400), "2000-02-29 00:00:00");
}
