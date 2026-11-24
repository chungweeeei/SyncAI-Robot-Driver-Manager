//! **Pure** Lynx M20 battery functions: the vendor's `/BATTERY_DATA` packs -> one `Battery`.
//! No ROS message types (the subscription in `m20_battery_forwarder.rs` copies each
//! `drdds/msg/BatteryDataValue` into a `PackReading`); unit tested (`tests/m20_battery.rs`).
//!
//! `/BATTERY_DATA` (`drdds/msg/BatteryData`, 1 Hz, reliable, published by the M20's own stack on
//! domain 0) carries one `BatteryDataValue` per pack. The M20 has two packs in parallel: on
//! robot01 (2026-11-24) both read 73.5 V, -1.04 A, 4.50 Ah nominal while lying down, at 52 % and
//! 45 %. `battery_state` is one `sensor_msgs/BatteryState`, so the packs are combined as one
//! battery: what `syncai_robot_state` and charge.xml's `WaitForCharging` want is "the robot's
//! battery", not a pack.

use super::protocol::{Battery, ChargeState};

/// One pack of `/BATTERY_DATA`, in the BMS's raw units (`BatteryDataValue.msg`)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PackReading<'a> {
    /// Total voltage [10 mV]
    pub voltage: u16,
    /// Total current [10 mA]; negative while discharging (robot01 lying: -104)
    pub current: i16,
    /// Remaining capacity [10 mAh]
    pub remaining_capacity: u16,
    /// Nominal capacity [10 mAh]
    pub nominal_capacity: u16,
    /// State of charge [%], the BMS's own figure
    pub battery_level: u8,
    /// Thermistor readings [°C]; robot01's packs send three although `battery_ntc` says five
    pub temperatures: &'a [f32],
}

impl PackReading<'_> {
    /// A pack slot with no pack in it reads all zeros; 0 V is never a real pack's voltage
    fn is_present(&self) -> bool {
        self.voltage > 0
    }

    fn voltage_v(&self) -> f32 {
        f32::from(self.voltage) * 0.01
    }

    fn current_a(&self) -> f32 {
        f32::from(self.current) * 0.01
    }

    fn remaining_ah(&self) -> f32 {
        f32::from(self.remaining_capacity) * 0.01
    }

    fn nominal_ah(&self) -> f32 {
        f32::from(self.nominal_capacity) * 0.01
    }

    fn level(&self) -> f32 {
        f32::from(self.battery_level)
    }
}

/// The packs as one battery, or None when no pack is present (nothing to report, rather than a
/// 0 V, 0 % battery that `syncai_robot_state` would read as empty).
///
/// - voltage: the mean of the packs' (they are in parallel, so they agree to ~10 mV)
/// - current: the sum (parallel packs share the load); its sign gives `charge_state`, with the
///   G23's 0.1 A deadband, for the reason `ChargeState` gives: a current, not a BMS mode bit
/// - soc: the packs' `battery_level` weighted by nominal capacity (equal packs: the plain mean),
///   and not remaining / nominal: the BMS's level already accounts for the pack's aged capacity,
///   which the nominal figure does not (robot01: 2.62 / 4.50 Ah = 58 % against a level of 52 %)
/// - charge / design_capacity: the sums, in Ah
/// - temperature: the mean of every thermistor that reported a value (0 or non-finite is
///   "missing", as on the G23), NaN when none did
pub fn combine(packs: &[PackReading]) -> Option<Battery> {
    let present: Vec<&PackReading> = packs.iter().filter(|p| p.is_present()).collect();
    if present.is_empty() {
        return None;
    }
    let count = present.len() as f32;

    let voltage = present.iter().map(|p| p.voltage_v()).sum::<f32>() / count;
    let current = present.iter().map(|p| p.current_a()).sum::<f32>();
    let charge = present.iter().map(|p| p.remaining_ah()).sum::<f32>();
    let design_capacity = present.iter().map(|p| p.nominal_ah()).sum::<f32>();

    let soc = if design_capacity > 0.0 {
        let weighted: f32 = present.iter().map(|p| p.level() * p.nominal_ah()).sum();
        weighted / design_capacity
    } else {
        present.iter().map(|p| p.level()).sum::<f32>() / count
    };

    let temps: Vec<f32> = present
        .iter()
        .flat_map(|p| p.temperatures.iter().copied())
        .filter(|t| t.is_finite() && *t != 0.0)
        .collect();
    let temperature = if temps.is_empty() {
        f32::NAN
    } else {
        temps.iter().sum::<f32>() / temps.len() as f32
    };

    Some(Battery {
        voltage,
        current,
        soc,
        temperature,
        charge_state: ChargeState::from_current(current),
        // The packs report no cell voltages
        cells: None,
        charge: Some(charge),
        design_capacity: (design_capacity > 0.0).then_some(design_capacity),
    })
}

#[cfg(test)]
#[path = "tests/m20_battery.rs"]
mod tests;
