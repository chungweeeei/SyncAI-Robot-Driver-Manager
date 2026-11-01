mod command;
mod health;
mod monitor;
mod parameters;
mod protocol;
mod publishers;
mod service;
mod session;
mod subscriber;
mod telemetry;

use std::error::Error;
use std::sync::Arc;

use rclrs::*;

use command::{CommandLink, SafetyLock};
use health::SensorHealth;
use monitor::{MonitorWorker, SensorSubscriptions};
use parameters::{MonitorConfig, UdpConfig, VelocityScale};
use publishers::Publishers;
use service::{ServiceContext, Services};
use session::UdpSession;
use subscriber::{CmdVelContext, Subscribers};
use telemetry::TelemetryWorker;

/// The boundary between ROS 2 and the gait controller: nothing below this node is ROS.
///
/// Both UDP sockets are opened during construction; if either fails an error is returned, so
/// the node never comes up half-connected.
///
/// Threading (mirrors the callback groups of the C++ version):
///
/// | Work              | Runs on                                                       |
/// |-------------------|---------------------------------------------------------------|
/// | cmd_vel           | Its own rclrs Worker (its own thread)                         |
/// | The four services | One shared Worker: serialized, but concurrent with cmd_vel    |
/// | Telemetry receive | Its own std::thread, entirely outside the executor            |
/// | Lidar / camera    | One Worker, only counting messages for the monitor            |
/// | Status table      | Its own std::thread (`monitor`), reading `SensorHealth`       |
///
/// `SensorHealth` is the one piece of state all of them share: the telemetry thread, the
/// command link (from cmd_vel and the services) and the sensor worker write it, and the monitor
/// thread reads it to print the stdout status table.
pub struct DriverManagerNode {
    // Fields drop in declaration order. The monitor and telemetry threads go first so they are
    // joined (and the telemetry socket and publishers closed) before the workers and node
    // handle are torn down.
    _monitor: MonitorWorker,
    _telemetry: TelemetryWorker,
    _sensor_subscriptions: SensorSubscriptions,
    _subscribers: Subscribers,
    _services: Services,
    // Parameters are undeclared when their handles drop, so they are kept for the node's life
    _velocity_scale: Arc<VelocityScale>,
    _udp_config: UdpConfig,
    _monitor_config: MonitorConfig,
    _node: Node,
}

impl DriverManagerNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let velocity_scale = Arc::new(VelocityScale::declare(&node)?);
        let udp_config = UdpConfig::declare(&node)?;
        let monitor_config = MonitorConfig::declare(&node)?;
        let health = Arc::new(SensorHealth::default());

        let telemetry_session = UdpSession::listen(udp_config.telemetry_addr()?)
            .map_err(|e| format!("failed to open UDP telemetry socket: {e}"))?;
        let command_addr = udp_config.command_addr()?;
        let command_session = UdpSession::connect(command_addr)
            .map_err(|e| format!("failed to open UDP command socket: {e}"))?;

        log_info!(
            node.logger(),
            "[DriverManagerNode] Telemetry on {}, command to {}",
            telemetry_session.local_addr,
            command_addr,
        );
        log_info!(
            node.logger(),
            "[DriverManagerNode] Speed scales F: {:.2}, B: {:.2}, L: {:.2}, R: {:.2}, TL: {:.2}, TR: {:.2}",
            velocity_scale.forward.get(),
            velocity_scale.backward.get(),
            velocity_scale.left.get(),
            velocity_scale.right.get(),
            velocity_scale.angular_left.get(),
            velocity_scale.angular_right.get(),
        );

        let command = Arc::new(CommandLink::new(
            command_session,
            Arc::clone(&health),
            node.logger().clone(),
        ));
        let safety = Arc::new(SafetyLock::new(
            publishers::create_safety_locked(&node)?,
            node.logger().clone(),
        ));

        let subscribers = Subscribers::create(
            &node,
            CmdVelContext {
                velocity_scale: Arc::clone(&velocity_scale),
                command: Arc::clone(&command),
                safety: Arc::clone(&safety),
            },
        )?;
        let services = Services::create(
            &node,
            ServiceContext {
                velocity_scale: Arc::clone(&velocity_scale),
                command: Arc::clone(&command),
                safety: Arc::clone(&safety),
                logger: node.logger().clone(),
            },
        )?;

        let telemetry = TelemetryWorker::spawn(
            telemetry_session,
            Publishers::create(&node)?,
            Arc::clone(&health),
            node.get_clock(),
            node.logger().clone(),
        )?;

        let sensor_subscriptions = SensorSubscriptions::create(&node, Arc::clone(&health))?;
        let monitor = MonitorWorker::spawn(
            health,
            safety,
            monitor_config.thresholds(),
            monitor_config.report_period(),
            node.logger().clone(),
        )?;

        log_info!(
            node.logger(),
            "[DriverManagerNode] Driver Manager initialized successfully"
        );

        Ok(Self {
            _monitor: monitor,
            _telemetry: telemetry,
            _sensor_subscriptions: sensor_subscriptions,
            _subscribers: subscribers,
            _services: services,
            _velocity_scale: velocity_scale,
            _udp_config: udp_config,
            _monitor_config: monitor_config,
            _node: node,
        })
    }
}
