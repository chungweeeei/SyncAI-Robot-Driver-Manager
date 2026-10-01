use rclrs::*;
use std::sync::Arc;

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
            node.declare_parameter(format!("velocity_scale.{name}"))
                .default(1.0)
                .range(ParameterRange {
                    lower: Some(0.0),
                    upper: Some(1.0),
                    step: None,
                })
                .description("velocity scale ratio, 0.0 ~ 1.0")
                .mandatory()
        };

        Ok(Self {
            forward: scale("forward")?,
            backward: scale("backward")?,
            left: scale("left")?,
            right: scale("right")?,
            angular_left: scale("angular_left")?,
            angular_right: scale("angular_right")?,
        })
    }
}

/// UDP 位址在 socket 建立後就不能改，所以宣告成 read-only
pub struct UdpConfig {
    pub telemetry_bind: ReadOnlyParameter<Arc<str>>,
    pub command_peer: ReadOnlyParameter<Arc<str>>,
}

impl UdpConfig {
    pub fn declare(node: &Node) -> Result<Self, DeclarationError> {
        let addr = |name: &str, default: &str, description: &str| {
            node.declare_parameter(format!("udp.{name}"))
                .default(Arc::from(default))
                .description(description)
                .read_only()
        };

        Ok(Self {
            telemetry_bind: addr(
                "telemetry_bind",
                "0.0.0.0:50012",
                "local address to receive telemetry packets on",
            )?,
            command_peer: addr(
                "command_peer",
                "192.168.1.120:50051",
                "remote address to send command packets to",
            )?,
        })
    }
}
