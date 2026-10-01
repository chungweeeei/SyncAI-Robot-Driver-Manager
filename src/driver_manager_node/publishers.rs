use rclrs::*;
use ros_env::sensor_msgs::msg::BatteryState;
use ros_env::std_msgs::msg::Int32MultiArray;
use ros_env::syncai_common::msg::{IMUState, MotorStates};

pub struct Publishers {
    pub imu: Publisher<IMUState>,
    pub motor_states: Publisher<MotorStates>,
    pub battery_state: Publisher<BatteryState>,
    // data[0] = policy state, data[1] = motion state (see RobotLowLevelMode.msg in syncai_common)
    pub mode: Publisher<Int32MultiArray>,
}

impl Publishers {
    pub fn create(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            imu: node.create_publisher("imu".sensor_data_qos())?,
            motor_states: node.create_publisher("motor_states".sensor_data_qos())?,
            // Reliable depth 10, same as the C++ version: syncai_robot_state subscribes to
            // `mode` as reliable, which a best-effort publisher would not match
            battery_state: node.create_publisher("battery_state".keep_last(10))?,
            mode: node.create_publisher("mode".keep_last(10))?,
        })
    }
}
