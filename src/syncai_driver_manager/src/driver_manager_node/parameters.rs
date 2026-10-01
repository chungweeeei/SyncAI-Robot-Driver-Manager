use rclrs::*;
use std::error::Error;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;

/// 各方向的 cmd_vel -> 實際速度修正增益。
///
/// 下位機對不同方向的追蹤程度不一樣（前進 vs 後退……），所以每個方向各自一個經驗值，
/// 依指令的正負號挑選。預設 1.0，校正值放在 params/driver_manager_params.yaml。
///
/// `set_speed_scale` 會在執行時改寫這些參數，但不會寫回 YAML，重啟後回到 YAML 的值。
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
                // 沒有上限：校正值本來就會 > 1（目前出貨值是 1.40）；負值會讓方向反轉，不接受
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

/// UDP 位址在 socket 建立後就不能改，所以宣告成 read-only
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
