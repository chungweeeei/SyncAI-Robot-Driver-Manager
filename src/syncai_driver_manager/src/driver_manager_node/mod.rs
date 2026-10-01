use rclrs::*;
use std::error::Error;
use std::sync::Arc;

mod parameters;
use parameters::{UdpConfig, VelocityScale};

mod protocol;

mod publishers;
use publishers::Publishers;

mod subscriber;
use subscriber::Subscribers;

mod service;
use service::Services;

mod session;
use session::UdpSession;

mod telemetry;
use telemetry::TelemetryWorker;

pub struct DriverManagerNode {
    _node: Node,
    _udp_config: UdpConfig,
    velocity_scale: Arc<VelocityScale>,
    _telemetry: TelemetryWorker,
    command: UdpSession,
    _publishers: Publishers,
    _subscribers: Subscribers,
    _services: Services,
}

impl DriverManagerNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let velocity_scale = Arc::new(VelocityScale::declare(&node)?);
        let udp_config = UdpConfig::declare(&node)?;
        let publishers = Publishers::create(&node)?;
        let subscribers = Subscribers::create(&node, Arc::clone(&velocity_scale))?;
        let services = Services::create(&node, Arc::clone(&velocity_scale))?;

        let telemetry_session = UdpSession::listen(udp_config.telemetry_bind.get().parse()?)?;
        let command = UdpSession::connect(udp_config.command_peer.get().parse()?)?;

        let telemetry = TelemetryWorker::spawn(
            Arc::clone(&telemetry_session.socket),
            publishers.imu.clone(),
            node.logger().clone(),
        )?;

        log_info!(
            node.logger(),
            "[DriverManagerNode] Telemetry on {}, command to {}",
            telemetry_session.local_addr,
            udp_config.command_peer.get(),
        );
        log_info!(
            node.logger(),
            "[DriverManagerNode] Speed scales F: {:?}, B: {:?}, L: {:?}, R: {:?}, TL: {:?}, TR: {:?}",
            velocity_scale.forward.get(),
            velocity_scale.backward.get(),
            velocity_scale.left.get(),
            velocity_scale.right.get(),
            velocity_scale.angular_left.get(),
            velocity_scale.angular_right.get(),
        );

        Ok(Self {
            _node: node,
            _udp_config: udp_config,
            velocity_scale,
            _telemetry: telemetry,
            command,
            _publishers: publishers,
            _subscribers: subscribers,
            _services: services,
        })
    }
}
