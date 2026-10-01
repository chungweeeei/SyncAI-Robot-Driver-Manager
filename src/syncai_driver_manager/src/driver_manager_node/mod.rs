use rclrs::*;
use std::error::Error;
use std::sync::Arc;

mod command;
use command::{CommandLink, SafetyLock};

mod parameters;
use parameters::{UdpConfig, VelocityScale};

mod protocol;

mod publishers;
use publishers::Publishers;

mod subscriber;
use subscriber::{CmdVelContext, Subscribers};

mod service;
use service::{ServiceContext, Services};

mod session;
use session::UdpSession;

mod telemetry;
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
pub struct DriverManagerNode {
    _node: Node,
    _udp_config: UdpConfig,
    _velocity_scale: Arc<VelocityScale>,
    // Dropped first: stops and joins the telemetry thread before the sockets are closed
    _telemetry: TelemetryWorker,
    _subscribers: Subscribers,
    _services: Services,
}

impl DriverManagerNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let velocity_scale = Arc::new(VelocityScale::declare(&node)?);
        let udp_config = UdpConfig::declare(&node)?;

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

        let command = Arc::new(CommandLink::new(command_session, node.logger().clone()));
        let safety = Arc::new(SafetyLock::default());

        let subscribers = Subscribers::create(
            &node,
            CmdVelContext {
                velocity_scale: Arc::clone(&velocity_scale),
                command: Arc::clone(&command),
            },
        )?;
        let services = Services::create(
            &node,
            ServiceContext {
                velocity_scale: Arc::clone(&velocity_scale),
                command: Arc::clone(&command),
                safety,
                logger: node.logger().clone(),
            },
        )?;

        let telemetry = TelemetryWorker::spawn(
            Arc::clone(&telemetry_session.socket),
            Publishers::create(&node)?,
            node.get_clock(),
            node.logger().clone(),
        )?;

        log_info!(
            node.logger(),
            "[DriverManagerNode] Driver Manager initialized successfully"
        );

        Ok(Self {
            _node: node,
            _udp_config: udp_config,
            _velocity_scale: velocity_scale,
            _telemetry: telemetry,
            _subscribers: subscribers,
            _services: services,
        })
    }
}
