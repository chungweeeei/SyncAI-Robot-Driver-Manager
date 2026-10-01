use rclrs::*;
use ros_env::std_msgs::msg::Int32MultiArray;
use ros_env::syncai_common::msg::{IMUState, MotorStates, RobotBatteryStatus};

pub struct Publishers {
    pub imu: Publisher<IMUState>,
    pub motor_states: Publisher<MotorStates>,
    pub battery_state: Publisher<RobotBatteryStatus>,
    // data[0] = policy state, data[1] = motion state（見 syncai_common 的 RobotLowLevelMode.msg）
    pub mode: Publisher<Int32MultiArray>,
}

impl Publishers {
    pub fn create(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            imu: node.create_publisher("imu".sensor_data_qos())?,
            motor_states: node.create_publisher("motor_states".sensor_data_qos())?,
            battery_state: node.create_publisher("battery_state".sensor_data_qos())?,
            mode: node.create_publisher("mode".sensor_data_qos())?,
        })
    }
}
