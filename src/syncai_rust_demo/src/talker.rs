//! 最小的 ROS 2 publisher：每秒往 `/syncai/chatter` 發一則 std_msgs/msg/String。
//!
//! 執行方式：
//!   ros2 run syncai_rust_demo talker

use rclrs::*;
use ros_env::std_msgs::msg::String as StringMsg;
use std::time::Duration;

/// 發布週期
const PUBLISH_PERIOD: Duration = Duration::from_secs(1);
/// 話題名稱
const TOPIC: &str = "/syncai/chatter";

fn main() -> Result<(), RclrsError> {
    // Context 會讀取命令列參數（例如 --ros-args -r __node:=xxx）與 ROS 環境變數
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_talker")?;

    let publisher = node.create_publisher::<StringMsg>(TOPIC)?;

    // Worker 持有 callback 之間共用的狀態；這裡就是已送出的訊息數。
    let worker = node.create_worker::<u64>(0);

    let logger = node.logger().clone();
    let _timer = worker.create_timer_repeating(PUBLISH_PERIOD, move |count: &mut u64| {
        *count += 1;

        let message = StringMsg {
            data: format!("Hello from rclrs! #{count}"),
        };

        match publisher.publish(&message) {
            Ok(()) => log_info!(&logger, "Publishing: '{}'", message.data),
            Err(err) => log_error!(&logger, "Failed to publish: {err}"),
        }
    })?;

    log_info!(node.logger(), "Talker 啟動，發布到 '{TOPIC}'");

    // spin 會阻塞直到收到 Ctrl-C（或 context 被關閉）
    executor.spin(SpinOptions::default()).first_error()
}
