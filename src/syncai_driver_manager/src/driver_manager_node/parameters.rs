use rclrs::*;
use std::error::Error;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;

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
                "local interface address to receive telemetry datagrams on",
            )?,
            telemetry_recv_port: port(
                "telemetry_recv_port",
                50012,
                "local port to receive telemetry datagrams on",
            )?,
            command_target_ip: ip(
                "command_target_ip",
                "192.168.1.120",
                "controller address to send command datagrams to",
            )?,
            command_target_port: port(
                "command_target_port",
                50051,
                "controller port to send command datagrams to",
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

fn socket_addr(name: &str, ip: &str, port: i64) -> Result<SocketAddr, Box<dyn Error>> {
    let ip: IpAddr = ip
        .parse()
        .map_err(|e| format!("invalid {name}_ip '{ip}': {e}"))?;
    let port = u16::try_from(port).map_err(|e| format!("invalid {name}_port {port}: {e}"))?;
    Ok(SocketAddr::new(ip, port))
}
