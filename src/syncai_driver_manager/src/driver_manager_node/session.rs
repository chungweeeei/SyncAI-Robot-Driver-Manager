use std::io;
use std::net::{SocketAddr, UdpSocket};

/// One UDP socket and the addresses it was opened with. Owned by exactly one user (the
/// telemetry thread or the `CommandLink`), so no sharing is needed.
pub struct UdpSession {
    pub local_addr: SocketAddr,
    /// The peer address for `connect`; None for `listen`
    pub peer_addr: Option<SocketAddr>,
    pub socket: UdpSocket,
}

impl UdpSession {
    pub fn listen(addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        Ok(Self {
            local_addr: socket.local_addr()?,
            peer_addr: None,
            socket,
        })
    }

    pub fn connect(addr: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(addr)?;
        Ok(Self {
            local_addr: socket.local_addr()?,
            peer_addr: Some(addr),
            socket,
        })
    }
}
