use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;

pub struct UdpSession {
    pub local_addr: SocketAddr,
    /// `connect` 時的對方位址；`listen` 時為 None
    pub peer_addr: Option<SocketAddr>,
    pub socket: Arc<UdpSocket>,
}

impl UdpSession {
    pub fn listen(addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        Ok(Self {
            local_addr: socket.local_addr()?,
            peer_addr: None,
            socket: Arc::new(socket),
        })
    }

    pub fn connect(addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(addr)?;
        Ok(Self {
            local_addr: socket.local_addr()?,
            peer_addr: Some(addr),
            socket: Arc::new(socket),
        })
    }
}
