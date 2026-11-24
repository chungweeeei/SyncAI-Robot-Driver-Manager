use rclrs::*;
use std::error::Error;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

use super::health::Thresholds;
use super::m20_link::M20Config;
use super::protocol::VelocityGains;

/// Per-direction cmd_vel -> actual velocity correction gains.
///
/// The gait controller tracks each direction differently (forward vs backward, ...), so each
/// direction has its own empirical gain, picked by the sign of the command. Defaults to 1.0;
/// the G23's calibrated values live in params/models/g23.yaml.
///
/// `set_speed_scale` overwrites these at runtime but does not write back to the YAML, so a
/// restart returns to the YAML values.
pub struct VelocityScale {
    pub forward: MandatoryParameter<f64>,
    pub backward: MandatoryParameter<f64>,
    pub left: MandatoryParameter<f64>,
    pub right: MandatoryParameter<f64>,
    pub angular_left: MandatoryParameter<f64>,
    pub angular_right: MandatoryParameter<f64>,
}

impl VelocityScale {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        let scale = |name: &str| {
            node.declare_parameter(name)
                .default(1.0)
                // No upper bound: calibrated values are > 1 (1.40 as shipped). Negative values
                // would reverse the direction, so they are rejected.
                .range(ParameterRange {
                    lower: Some(0.0),
                    upper: None,
                    step: None,
                })
                .description("cmd_vel -> AXES velocity correction gain, >= 0.0")
                .mandatory()
        };

        Ok(Self {
            forward: scale("scale_fwd")?,
            backward: scale("scale_back")?,
            left: scale("scale_left")?,
            right: scale("scale_right")?,
            angular_left: scale("scale_turn_l")?,
            angular_right: scale("scale_turn_r")?,
        })
    }

    /// Reads all six gains at once. Each read is atomic on its own, but the six together are
    /// not: a concurrent `set_speed_scale` may land between two reads.
    pub fn gains(&self) -> VelocityGains {
        VelocityGains {
            forward: self.forward.get(),
            backward: self.backward.get(),
            left: self.left.get(),
            right: self.right.get(),
            angular_left: self.angular_left.get(),
            angular_right: self.angular_right.get(),
        }
    }
}

/// The G23 gait controller's UDP endpoints, declared only when `robot_protocol` is `g23` (the
/// M20's counterpart is `M20Parameters`). They cannot change once the sockets are open, so they
/// are read-only.
pub struct G23Parameters {
    pub telemetry_recv_ip: ReadOnlyParameter<Arc<str>>,
    pub telemetry_recv_port: ReadOnlyParameter<i64>,
    pub command_target_ip: ReadOnlyParameter<Arc<str>>,
    pub command_target_port: ReadOnlyParameter<i64>,
}

impl G23Parameters {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        let ip = |name: &str, default: &str, description: &str| {
            node.declare_parameter(name)
                .default(Arc::from(default))
                .description(description)
                .read_only()
        };
        let port = |name: &str, default: i64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .range(ParameterRange {
                    lower: Some(0),
                    upper: Some(i64::from(u16::MAX)),
                    step: None,
                })
                .description(description)
                .read_only()
        };

        Ok(Self {
            telemetry_recv_ip: ip(
                "telemetry_recv_ip",
                "0.0.0.0",
                "local interface address to receive telemetry on",
            )?,
            telemetry_recv_port: port(
                "telemetry_recv_port",
                50012,
                "local port to receive telemetry on",
            )?,
            command_target_ip: ip(
                "command_target_ip",
                "192.168.1.120",
                "controller address to send command to",
            )?,
            command_target_port: port(
                "command_target_port",
                50051,
                "controller port to send command to",
            )?,
        })
    }

    pub fn telemetry_addr(&self) -> Result<SocketAddr, Box<dyn Error>> {
        socket_addr(
            "telemetry_recv",
            &self.telemetry_recv_ip.get(),
            self.telemetry_recv_port.get(),
        )
    }

    pub fn command_addr(&self) -> Result<SocketAddr, Box<dyn Error>> {
        socket_addr(
            "command_target",
            &self.command_target_ip.get(),
            self.command_target_port.get(),
        )
    }
}

/// The message type on the monitor's lidar topic: what decides which subscription is made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarMsgType {
    /// `livox_ros_driver2/CustomMsg`: the Livox driver in the nav and mapping sessions (G23)
    Livox,
    /// `sensor_msgs/PointCloud2`: the Lynx M20's vendor stack (`/LIDAR/POINTS_UDP`)
    PointCloud2,
}

impl LidarMsgType {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "livox" => Ok(Self::Livox),
            "pointcloud2" => Ok(Self::PointCloud2),
            other => Err(format!(
                "invalid monitor_lidar_type '{other}' (livox or pointcloud2)"
            )),
        }
    }
}

/// The sensor monitor's report period, per-stream timeouts (seconds) and the lidar topics it
/// counts. Read once when the monitor starts, so they are read-only.
///
/// The topics are parameters because the robots publish them differently: the G23's Livox
/// driver publishes `livox/lidar` (CustomMsg) and `livox/imu` under `/<robot_id>`, while the
/// M20's vendor stack publishes `/LIDAR/POINTS_UDP` (PointCloud2) and `/IMU` with no namespace
/// at all, so its params file gives absolute names.
pub struct MonitorConfig {
    pub report_period: ReadOnlyParameter<f64>,
    pub telemetry_timeout: ReadOnlyParameter<f64>,
    pub lidar_timeout: ReadOnlyParameter<f64>,
    pub imu_timeout: ReadOnlyParameter<f64>,
    pub camera_timeout: ReadOnlyParameter<f64>,
    pub command_idle: ReadOnlyParameter<f64>,
    pub lidar_topic: ReadOnlyParameter<Arc<str>>,
    pub lidar_type: ReadOnlyParameter<Arc<str>>,
    pub imu_topic: ReadOnlyParameter<Arc<str>>,
}

impl MonitorConfig {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        let seconds = |name: &str, default: f64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                // A zero period would spin the monitor thread; a zero timeout would read every
                // stream as STALE between two messages
                .range(ParameterRange {
                    lower: Some(0.1),
                    upper: None,
                    step: None,
                })
                .description(description)
                .read_only()
        };
        let text = |name: &str, default: &str, description: &str| {
            node.declare_parameter(name)
                .default(Arc::from(default))
                .description(description)
                .read_only()
        };

        Ok(Self {
            report_period: seconds(
                "monitor_report_period",
                5.0,
                "seconds between two sensor status tables on stdout",
            )?,
            telemetry_timeout: seconds(
                "monitor_telemetry_timeout",
                1.0,
                "seconds without a telemetry datagram before telemetry reads STALE",
            )?,
            lidar_timeout: seconds(
                "monitor_lidar_timeout",
                0.5,
                "seconds without a lidar cloud before the lidar reads STALE",
            )?,
            imu_timeout: seconds(
                "monitor_imu_timeout",
                0.2,
                "seconds without an IMU message before the imu reads STALE",
            )?,
            camera_timeout: seconds(
                "monitor_camera_timeout",
                1.0,
                "seconds without image_raw/compressed before the camera reads STALE",
            )?,
            command_idle: seconds(
                "monitor_command_idle",
                1.0,
                "seconds without a sent command before the command link reads IDLE",
            )?,
            lidar_topic: text(
                "monitor_lidar_topic",
                "livox/lidar",
                "lidar cloud topic the monitor counts (relative = under /<robot_id>)",
            )?,
            lidar_type: text(
                "monitor_lidar_type",
                "livox",
                "message type on monitor_lidar_topic: livox (CustomMsg) or pointcloud2",
            )?,
            imu_topic: text(
                "monitor_imu_topic",
                "livox/imu",
                "lidar IMU topic (sensor_msgs/Imu) the monitor counts",
            )?,
        })
    }

    /// Fails on an unknown type, so a typo stops the node at startup rather than leaving the
    /// lidar row at NO DATA for a reason nobody can see
    pub fn lidar_type(&self) -> Result<LidarMsgType, String> {
        LidarMsgType::parse(&self.lidar_type.get())
    }

    pub fn report_period(&self) -> Duration {
        Duration::from_secs_f64(self.report_period.get())
    }

    pub fn thresholds(&self) -> Thresholds {
        let seconds = |p: &ReadOnlyParameter<f64>| Duration::from_secs_f64(p.get());
        Thresholds {
            telemetry_timeout: seconds(&self.telemetry_timeout),
            lidar_timeout: seconds(&self.lidar_timeout),
            imu_timeout: seconds(&self.imu_timeout),
            camera_timeout: seconds(&self.camera_timeout),
            command_idle: seconds(&self.command_idle),
        }
    }
}

/// Which robot this node drives (`drive::Drive`): `"g23"` (default) or `"m20"`. Read once at
/// startup, since it decides which sockets and threads exist.
pub struct RobotProtocol {
    pub value: ReadOnlyParameter<Arc<str>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Protocol {
    G23,
    M20,
}

impl RobotProtocol {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        Ok(Self {
            value: node
                .declare_parameter("robot_protocol")
                .default(Arc::from("g23"))
                .description("robot to drive: g23 or m20")
                .read_only()?,
        })
    }

    pub fn protocol(&self) -> Result<Protocol, Box<dyn Error>> {
        match &*self.value.get() {
            "g23" => Ok(Protocol::G23),
            "m20" => Ok(Protocol::M20),
            other => Err(format!("invalid robot_protocol '{other}' (g23 or m20)").into()),
        }
    }
}

/// The Lynx M20's endpoint and how cmd_vel maps onto its axis command. Declared only when
/// `robot_protocol` is `m20`, as `G23Parameters` / `VelocityScale` only when it is `g23`.
///
/// The axis command is a fraction of the gait's maximum speed per axis, not a velocity, so
/// `max_*` are those maxima in cmd_vel's units. None of them is measured: the defaults sit
/// above any speed the M20 is likely to reach in its basic gait, so a command maps to a smaller
/// fraction than it should, never a larger one. Calibrate them the way the G23's `scale_*` were.
pub struct M20Parameters {
    pub motion_host_ip: ReadOnlyParameter<Arc<str>>,
    pub motion_host_port: ReadOnlyParameter<i64>,
    pub max_vx: ReadOnlyParameter<f64>,
    pub max_vy: ReadOnlyParameter<f64>,
    pub max_wz: ReadOnlyParameter<f64>,
    pub cmd_timeout: ReadOnlyParameter<f64>,
}

impl M20Parameters {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        let positive = |name: &str, default: f64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .range(ParameterRange {
                    lower: Some(0.05),
                    upper: None,
                    step: None,
                })
                .description(description)
                .read_only()
        };
        Ok(Self {
            motion_host_ip: node
                .declare_parameter("motion_host_ip")
                .default(Arc::from("10.21.31.103"))
                .description("Lynx M20 motion host address (its UDP protocol server)")
                .read_only()?,
            motion_host_port: node
                .declare_parameter("motion_host_port")
                .default(30000)
                .range(ParameterRange {
                    lower: Some(0),
                    upper: Some(i64::from(u16::MAX)),
                    step: None,
                })
                .description("Lynx M20 UDP protocol port")
                .read_only()?,
            max_vx: positive(
                "max_vx",
                2.0,
                "Lynx M20: m/s that maps to a full-scale forward axis command (unmeasured)",
            )?,
            max_vy: positive(
                "max_vy",
                1.0,
                "Lynx M20: m/s that maps to a full-scale lateral axis command (unmeasured)",
            )?,
            max_wz: positive(
                "max_wz",
                2.0,
                "Lynx M20: rad/s that maps to a full-scale yaw axis command (unmeasured)",
            )?,
            cmd_timeout: positive(
                "cmd_timeout",
                0.3,
                "seconds a cmd_vel is re-sent at 20 Hz before the M20 is sent a stop",
            )?,
        })
    }

    pub fn config(&self) -> Result<M20Config, Box<dyn Error>> {
        Ok(M20Config {
            target: socket_addr(
                "motion_host",
                &self.motion_host_ip.get(),
                self.motion_host_port.get(),
            )?,
            max_vx: self.max_vx.get(),
            max_vy: self.max_vy.get(),
            max_wz: self.max_wz.get(),
            cmd_timeout: Duration::from_secs_f64(self.cmd_timeout.get()),
        })
    }
}

fn socket_addr(name: &str, ip: &str, port: i64) -> Result<SocketAddr, Box<dyn Error>> {
    let ip: IpAddr = ip
        .parse()
        .map_err(|e| format!("invalid {name}_ip '{ip}': {e}"))?;
    let port = u16::try_from(port).map_err(|e| format!("invalid {name}_port {port}: {e}"))?;
    Ok(SocketAddr::new(ip, port))
}

#[cfg(test)]
#[path = "tests/parameters.rs"]
mod tests;
