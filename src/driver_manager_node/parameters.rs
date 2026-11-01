use rclrs::*;
use std::error::Error;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

use super::health::Thresholds;
use super::protocol::VelocityGains;

/// Per-direction cmd_vel -> actual velocity correction gains.
///
/// The gait controller tracks each direction differently (forward vs backward, ...), so each
/// direction has its own empirical gain, picked by the sign of the command. Defaults to 1.0;
/// the calibrated values live in params/driver_manager_params.yaml.
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

/// The UDP endpoints cannot change once the sockets are open, so they are read-only
pub struct UdpConfig {
    pub telemetry_recv_ip: ReadOnlyParameter<Arc<str>>,
    pub telemetry_recv_port: ReadOnlyParameter<i64>,
    pub command_target_ip: ReadOnlyParameter<Arc<str>>,
    pub command_target_port: ReadOnlyParameter<i64>,
}

impl UdpConfig {
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

/// The sensor monitor's report period and per-stream timeouts, in seconds. Read once when the
/// monitor thread starts, so they are read-only.
pub struct MonitorConfig {
    pub report_period: ReadOnlyParameter<f64>,
    pub telemetry_timeout: ReadOnlyParameter<f64>,
    pub lidar_timeout: ReadOnlyParameter<f64>,
    pub camera_timeout: ReadOnlyParameter<f64>,
    pub command_idle: ReadOnlyParameter<f64>,
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
                "seconds without livox/lidar or livox/imu before the lidar reads STALE",
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
        })
    }

    pub fn report_period(&self) -> Duration {
        Duration::from_secs_f64(self.report_period.get())
    }

    pub fn thresholds(&self) -> Thresholds {
        let seconds = |p: &ReadOnlyParameter<f64>| Duration::from_secs_f64(p.get());
        Thresholds {
            telemetry_timeout: seconds(&self.telemetry_timeout),
            lidar_timeout: seconds(&self.lidar_timeout),
            camera_timeout: seconds(&self.camera_timeout),
            command_idle: seconds(&self.command_idle),
        }
    }
}

fn socket_addr(name: &str, ip: &str, port: i64) -> Result<SocketAddr, Box<dyn Error>> {
    let ip: IpAddr = ip
        .parse()
        .map_err(|e| format!("invalid {name}_ip '{ip}': {e}"))?;
    let port = u16::try_from(port).map_err(|e| format!("invalid {name}_port {port}: {e}"))?;
    Ok(SocketAddr::new(ip, port))
}
