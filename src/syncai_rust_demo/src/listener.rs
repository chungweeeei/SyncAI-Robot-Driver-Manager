//! 最小的 ROS 2 subscriber：訂閱 `/syncai/chatter` 並印出收到的內容。
//!
//! 執行方式：
//!   ros2 run syncai_rust_demo listener

use rclrs::*;
use ros_env::std_msgs::msg::String as StringMsg;

/// 話題名稱，需與 talker 一致
const TOPIC: &str = "/syncai/chatter";

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_listener")?;

    // Worker 讓 callback 能安全地持有可變狀態，不需要自己包 Arc<Mutex<..>>。
    let worker = node.create_worker::<u64>(0);

    let logger = node.logger().clone();
    let _subscription =
        worker.create_subscription::<StringMsg, _>(TOPIC, move |received: &mut u64, msg: StringMsg| {
            *received += 1;
            log_info!(&logger, "[#{received}] I heard: '{}'", msg.data);
        })?;

    log_info!(node.logger(), "Listener 啟動，等待 '{TOPIC}' 的訊息…");

    executor.spin(SpinOptions::default()).first_error()
}
