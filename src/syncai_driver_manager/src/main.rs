use rclrs::*;
use ros_env::std_msgs::msg::String as StringMsg;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_driver_manager")?;

    let publisher = node.create_publisher::<StringMsg>("/chatter")?;
    let worker = node.create_worker::<u64>(0);

    let _timer = worker.create_timer_repeating(Duration::from_secs(1), move |n: &mut u64| {
        *n += 1;
        let _ = publisher.publish(&StringMsg { data: format!("hi #{n}") });
    })?;

    executor.spin(SpinOptions::default()).first_error()
}
