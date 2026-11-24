//! Unit tests of `m20_battery.rs`, included from there with `#[path]`: still a child module of
//! `m20_battery`, so `use super::*` reaches its private items.

use super::*;

const TEMPS_A: [f32; 3] = [28.7, 31.7, 29.2];
const TEMPS_B: [f32; 3] = [28.2, 31.3, 27.7];

/// robot01's two packs, lying down, 2026-11-24 (`ros2 topic echo --once /BATTERY_DATA`)
fn robot01() -> [PackReading<'static>; 2] {
    [
        PackReading {
            voltage: 7346,
            current: -104,
            remaining_capacity: 262,
            nominal_capacity: 450,
            battery_level: 52,
            temperatures: &TEMPS_A,
        },
        PackReading {
            voltage: 7351,
            current: -107,
            remaining_capacity: 207,
            nominal_capacity: 450,
            battery_level: 45,
            temperatures: &TEMPS_B,
        },
    ]
}

fn close(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-3
}

#[test]
fn robot01_reading() {
    let b = combine(&robot01()).unwrap();
    assert!(close(b.voltage, 73.485), "voltage {}", b.voltage);
    assert!(close(b.current, -2.11), "current {}", b.current);
    assert!(close(b.soc, 48.5), "soc {}", b.soc);
    assert!(close(b.charge.unwrap(), 4.69));
    assert!(close(b.design_capacity.unwrap(), 9.0));
    assert!(close(b.temperature, 176.8 / 6.0), "temperature {}", b.temperature);
    assert_eq!(b.charge_state, ChargeState::Discharging);
    assert_eq!(b.cells, None);
}

#[test]
fn charging_follows_the_summed_current() {
    let mut packs = robot01();
    packs[0].current = 300;
    packs[1].current = 290;
    assert_eq!(combine(&packs).unwrap().charge_state, ChargeState::Charging);
    // 0.06 A each, 0.12 A together: past the 0.1 A deadband only as a sum
    packs[0].current = 6;
    packs[1].current = 6;
    assert_eq!(combine(&packs).unwrap().charge_state, ChargeState::Charging);
    packs[1].current = -6;
    assert_eq!(combine(&packs).unwrap().charge_state, ChargeState::Idle);
}

#[test]
fn an_empty_slot_is_left_out() {
    let [a, _] = robot01();
    let empty = PackReading {
        voltage: 0,
        current: 0,
        remaining_capacity: 0,
        nominal_capacity: 0,
        battery_level: 0,
        temperatures: &[0.0, 0.0, 0.0],
    };
    let b = combine(&[a, empty]).unwrap();
    // One pack, not the mean with a zero
    assert!(close(b.voltage, 73.46));
    assert!(close(b.soc, 52.0));
    assert!(close(b.temperature, (28.7 + 31.7 + 29.2) / 3.0));
    assert!(close(b.design_capacity.unwrap(), 4.5));
}

#[test]
fn no_pack_is_no_reading() {
    assert_eq!(combine(&[]), None);
    let empty = PackReading {
        voltage: 0,
        current: 0,
        remaining_capacity: 0,
        nominal_capacity: 0,
        battery_level: 0,
        temperatures: &[],
    };
    assert_eq!(combine(&[empty, empty]), None);
}

#[test]
fn soc_is_weighted_by_capacity() {
    let mut packs = robot01();
    packs[1].nominal_capacity = 150;
    // (52 x 4.5 + 45 x 1.5) / 6.0
    assert!(close(combine(&packs).unwrap().soc, 50.25));
    // No nominal capacity at all: the plain mean, and no design capacity
    packs[0].nominal_capacity = 0;
    packs[1].nominal_capacity = 0;
    let b = combine(&packs).unwrap();
    assert!(close(b.soc, 48.5));
    assert_eq!(b.design_capacity, None);
}

#[test]
fn missing_thermistors() {
    let mut packs = robot01();
    packs[0].temperatures = &[0.0, f32::NAN];
    packs[1].temperatures = &[];
    assert!(combine(&packs).unwrap().temperature.is_nan());
}
