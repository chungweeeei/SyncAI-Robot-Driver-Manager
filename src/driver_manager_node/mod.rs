mod command;
mod drive;
mod health;
mod m20_battery;
mod m20_battery_forwarder;
mod m20_link;
mod m20_protocol;
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
use drive::Drive;
use health::SensorHealth;
use m20_battery_forwarder::M20BatteryForwarder;
use m20_link::M20Link;
use monitor::{LidarTopics, MonitorWorker, SensorSubscriptions};
use parameters::{
    G23Parameters, M20Parameters, MonitorConfig, Protocol, RobotProtocol, VelocityScale,
};
use publishers::Publishers;
use service::{ServiceContext, Services};
use session::UdpSession;
use subscriber::{CmdVelContext, Subscribers};
use telemetry::TelemetryWorker;

/// The boundary between ROS 2 and the gait controller: nothing below this node is ROS.
///
/// The `robot_protocol` parameter picks the robot (`Drive`): the G23 gait controller (two UDP
/// sockets: telemetry in, ASCII commands out) or a DEEP Robotics Lynx M20 (one `M20Link`). Every
/// socket is opened during construction; if any fails an error is returned, so the node never
/// comes up half-connected.
///
/// Threading (mirrors the callback groups of the C++ version):
///
/// | Work              | Runs on                                                       |
/// |-------------------|---------------------------------------------------------------|
/// | cmd_vel           | Its own rclrs Worker (its own thread)                         |
/// | The four services | One shared Worker: serialized, but concurrent with cmd_vel    |
/// | Telemetry receive | Its own std::thread, entirely outside the executor (G23 only) |
/// | M20 send / recv   | Two std::threads in `M20Link` (M20 only)                      |
/// | M20 battery       | Its own Worker: /BATTERY_DATA -> battery_state (M20 only)     |
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
    /// G23 only: the M20 link receives on its own socket and publishes nothing
    _telemetry: Option<TelemetryWorker>,
    /// M20 only: its battery_state, which the G23's comes with `_telemetry`
    _m20_battery: Option<M20BatteryForwarder>,
    _sensor_subscriptions: SensorSubscriptions,
    _subscribers: Subscribers,
    _services: Services,
    // Parameters are undeclared when their handles drop, so they are kept for the node's life.
    // The G23's scale_* live in Drive::G23 (held by the cmd_vel and service workers)
    _endpoint: Endpoint,
    _robot_protocol: RobotProtocol,
    _monitor_config: MonitorConfig,
    _node: Node,
}

/// The robot's connection parameters, read once to open its sockets and kept only so they stay
/// declared: only the robot's own set exists, so `ros2 param list` on an M20 shows no G23
/// telemetry_* / command_* / scale_* and a G23 no motion_host_* / max_* / cmd_timeout
enum Endpoint {
    G23(G23Parameters),
    M20(M20Parameters),
}

impl DriverManagerNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        // The protocol first: it decides which robot's parameters are declared at all
        let robot_protocol = RobotProtocol::declare(&node)?;
        let monitor_config = MonitorConfig::declare(&node)?;
        let health = Arc::new(SensorHealth::default());

        // The G23 path opens its two sockets and the telemetry thread exactly as before; the M20
        // path opens neither (nothing listens on the G23 ports) and one link of its own instead
        let (drive, telemetry_session, endpoint) = match robot_protocol.protocol()? {
            Protocol::G23 => {
                let velocity_scale = VelocityScale::declare(&node)?;
                let g23_parameters = G23Parameters::declare(&node)?;
                let telemetry_session = UdpSession::listen(g23_parameters.telemetry_addr()?)
                    .map_err(|e| format!("failed to open UDP telemetry socket: {e}"))?;
                let command_addr = g23_parameters.command_addr()?;
                let command_session = UdpSession::connect(command_addr)
                    .map_err(|e| format!("failed to open UDP command socket: {e}"))?;
                log_info!(
                    node.logger(),
                    "[DriverManagerNode] G23: telemetry on {}, command to {}",
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
                let command = CommandLink::new(
                    command_session,
                    Arc::clone(&health),
                    node.logger().clone(),
                );
                (
                    Drive::G23 {
                        command,
                        scale: velocity_scale,
                    },
                    Some(telemetry_session),
                    Endpoint::G23(g23_parameters),
                )
            }
            Protocol::M20 => {
                let m20_parameters = M20Parameters::declare(&node)?;
                let config = m20_parameters.config()?;
                let link = M20Link::open(config, Arc::clone(&health), node.logger().clone())
                    .map_err(|e| format!("failed to open the M20 link: {e}"))?;
                log_info!(
                    node.logger(),
                    "[DriverManagerNode] M20: protocol to {} from {}; full scale vx {:.2} m/s, \
                     vy {:.2} m/s, wz {:.2} rad/s; cmd_vel timeout {:.2} s",
                    config.target,
                    link.local_addr().map_or_else(|e| e.to_string(), |a| a.to_string()),
                    config.max_vx,
                    config.max_vy,
                    config.max_wz,
                    config.cmd_timeout.as_secs_f64(),
                );
                (Drive::M20(link), None, Endpoint::M20(m20_parameters))
            }
        };
        let drive = Arc::new(drive);

        let safety = Arc::new(SafetyLock::new(
            publishers::create_safety_locked(&node)?,
            node.logger().clone(),
        ));

        let subscribers = Subscribers::create(
            &node,
            CmdVelContext {
                drive: Arc::clone(&drive),
                safety: Arc::clone(&safety),
            },
        )?;
        let services = Services::create(
            &node,
            ServiceContext {
                drive: Arc::clone(&drive),
                safety: Arc::clone(&safety),
                logger: node.logger().clone(),
            },
        )?;

        let telemetry = match telemetry_session {
            Some(session) => Some(TelemetryWorker::spawn(
                session,
                Publishers::create(&node)?,
                Arc::clone(&health),
                node.get_clock(),
                node.logger().clone(),
            )?),
            None => None,
        };

        // battery_state on both robots, from whichever source the robot has: on a G23 the
        // telemetry thread's BMS packet (Publishers above), on an M20 the vendor's topic
        let m20_battery = match &endpoint {
            Endpoint::G23(_) => None,
            Endpoint::M20(m20_parameters) => {
                let topic = m20_parameters.battery_topic.get();
                log_info!(node.logger(), "[DriverManagerNode] M20: battery_state from {topic}");
                Some(M20BatteryForwarder::create(&node, &topic)?)
            }
        };

        let lidar_cloud_topic = monitor_config.lidar_topic.get();
        let lidar_imu_topic = monitor_config.imu_topic.get();
        let lidar_topics = LidarTopics {
            cloud: &lidar_cloud_topic,
            cloud_type: monitor_config.lidar_type()?,
            imu: &lidar_imu_topic,
        };
        log_info!(
            node.logger(),
            "[DriverManagerNode] Monitor: lidar {} ({:?}), imu {}",
            lidar_topics.cloud,
            lidar_topics.cloud_type,
            lidar_topics.imu,
        );
        let sensor_subscriptions =
            SensorSubscriptions::create(&node, Arc::clone(&health), &lidar_topics)?;
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
            _m20_battery: m20_battery,
            _sensor_subscriptions: sensor_subscriptions,
            _subscribers: subscribers,
            _services: services,
            _endpoint: endpoint,
            _robot_protocol: robot_protocol,
            _monitor_config: monitor_config,
            _node: node,
        })
    }
}
