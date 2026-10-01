use rclrs::*;

mod driver_manager_node;
use driver_manager_node::DriverManagerNode;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("syncai_driver_manager")?;
    let _driver_manager = DriverManagerNode::new(node)?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
