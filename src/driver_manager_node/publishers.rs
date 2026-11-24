use rclrs::*;
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::sensor_msgs::msg::BatteryState;
use ros_env::std_msgs::msg::{Bool, Header, Int32MultiArray};
use ros_env::syncai_common::msg::{IMUState, MotorStates};

use super::protocol::{Battery, ChargeState};

pub struct Publishers {
    pub imu: Publisher<IMUState>,
    pub motor_states: Publisher<MotorStates>,
    // `motor_states` thinned to MOTOR_STATES_LOW_RATE_PERIOD (telemetry.rs) for the consumers
    // that only show it: syncai_robot_state (1 Hz RobotState) and the backend's 3D model
    pub motor_states_low_rate: Publisher<MotorStates>,
    pub battery_state: Publisher<BatteryState>,
    // data[0] = policy state, data[1] = motion state (see RobotLowLevelMode.msg in syncai_common)
    pub mode: Publisher<Int32MultiArray>,
}

impl Publishers {
    pub fn create(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            imu: node.create_publisher("imu".sensor_data_qos())?,
            motor_states: node.create_publisher("motor_states".sensor_data_qos())?,
            // Same QoS as motor_states, so a subscriber moves over by changing only the name
            motor_states_low_rate: node
                .create_publisher("motor_states_low_rate".sensor_data_qos())?,
            // Reliable depth 10, same as the C++ version: syncai_robot_state subscribes to
            // `mode` as reliable, which a best-effort publisher would not match
            battery_state: create_battery_state(node)?,
            mode: node.create_publisher("mode".keep_last(10))?,
        })
    }
}

/// `battery_state`, reliable depth 10 (the C++ version's QoS). Created by `Publishers` on a G23
/// and by `M20BatteryForwarder` on an M20, so both robots publish the same topic with the same
/// QoS: `syncai_robot_state` and charge.xml's `WaitForCharging` read it.
pub fn create_battery_state(node: &Node) -> Result<Publisher<BatteryState>, RclrsError> {
    node.create_publisher("battery_state".keep_last(10))
}

/// One `BatteryState` from either robot's reading
pub fn battery_state_msg(battery: &Battery, stamp: TimeMsg) -> BatteryState {
    BatteryState {
        header: Header {
            stamp,
            ..Default::default()
        },
        voltage: battery.voltage,
        current: battery.current,
        // Both BMSs report 0-100; BatteryState.percentage is defined on 0-1
        percentage: battery.soc / 100.0,
        temperature: battery.temperature,
        // NaN is BatteryState's "unmeasured"
        charge: battery.charge.unwrap_or(f32::NAN),
        capacity: f32::NAN,
        design_capacity: battery.design_capacity.unwrap_or(f32::NAN),
        power_supply_status: match battery.charge_state {
            ChargeState::Charging => BatteryState::POWER_SUPPLY_STATUS_CHARGING,
            ChargeState::Discharging => BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
            ChargeState::Idle => BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING,
        },
        power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN,
        power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
        present: true,
        cell_voltage: battery.cells.map(Vec::from).unwrap_or_default(),
        ..Default::default()
    }
}

/// A ROS stamp of `clock`'s now. Built by hand from `Time::nsec`: rclrs's own `to_ros_msg`
/// returns its ros-env 0.3 type, which is not this crate's 0.2 one.
pub fn stamp_now(clock: &Clock) -> TimeMsg {
    let nsec = clock.now().nsec;
    TimeMsg {
        sec: i32::try_from(nsec.div_euclid(1_000_000_000)).unwrap_or(i32::MAX),
        nanosec: nsec.rem_euclid(1_000_000_000) as u32,
    }
}

/// `safety_locked`: whether the safety lock is engaged. Published only when it changes (plus
/// once at startup), so it is reliable + transient local depth 1: a late subscriber still
/// gets the current state. Owned by `SafetyLock`, not `Publishers`, because the lock changes
/// on the services worker as well as the telemetry thread.
pub fn create_safety_locked(node: &Node) -> Result<Publisher<Bool>, RclrsError> {
    node.create_publisher("safety_locked".keep_last(1).reliable().transient_local())
}
