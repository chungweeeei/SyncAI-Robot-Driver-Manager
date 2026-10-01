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

/// ROS 2 與下位機（gait controller）之間的邊界：這個節點以下都不是 ROS。
///
/// 兩個 UDP socket 都在建構時開好，任何一個失敗就直接回傳錯誤，節點不會半連線地起來。
///
/// 執行緒（對應 C++ 版的 callback group 設計）：
///
/// | 工作            | 執行在                                                  |
/// |-----------------|---------------------------------------------------------|
/// | cmd_vel         | 自己的 rclrs Worker（自己一條 thread）                  |
/// | 四個 service    | 共用一個 Worker：彼此依序執行，但跟 cmd_vel 可以同時跑  |
/// | telemetry 接收  | 自己的 std::thread，完全不經過 executor                 |
pub struct DriverManagerNode {
    _node: Node,
    _udp_config: UdpConfig,
    _velocity_scale: Arc<VelocityScale>,
    // 先 drop：停下並 join telemetry thread，之後 socket 才會跟著關掉
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
