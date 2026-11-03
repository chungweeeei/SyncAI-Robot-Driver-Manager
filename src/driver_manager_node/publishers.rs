use rclrs::*;
use ros_env::sensor_msgs::msg::BatteryState;
use ros_env::std_msgs::msg::{Bool, Int32MultiArray};
use ros_env::syncai_common::msg::{IMUState, MotorStates};

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
            battery_state: node.create_publisher("battery_state".keep_last(10))?,
            mode: node.create_publisher("mode".keep_last(10))?,
        })
    }
}

/// `safety_locked`: whether the safety lock is engaged. Published only when it changes (plus
/// once at startup), so it is reliable + transient local depth 1: a late subscriber still
/// gets the current state. Owned by `SafetyLock`, not `Publishers`, because the lock changes
/// on the services worker as well as the telemetry thread.
pub fn create_safety_locked(node: &Node) -> Result<Publisher<Bool>, RclrsError> {
    node.create_publisher("safety_locked".keep_last(1).reliable().transient_local())
}
