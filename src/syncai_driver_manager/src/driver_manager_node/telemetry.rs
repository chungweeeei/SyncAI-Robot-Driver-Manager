use std::io;
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use rclrs::*;
use ros_env::syncai_common::msg::IMUState;

use super::protocol;

const RECV_TIMEOUT: Duration = Duration::from_millis(200);
const LOG_THROTTLE: Duration = Duration::from_secs(5);

pub struct TelemetryWorker {
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl TelemetryWorker {
    pub fn spawn(
        socket: Arc<UdpSocket>,
        imu_pub: Publisher<IMUState>,
        logger: Logger,
    ) -> io::Result<Self> {
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;

        let running = Arc::new(AtomicBool::new(true));
        let handle = thread::Builder::new().name("telemetry".into()).spawn({
            let running = Arc::clone(&running);
            move || run(&socket, &imu_pub, &logger, &running)
        })?;

        Ok(Self {
            running,
            handle: Some(handle),
        })
    }
}

impl Drop for TelemetryWorker {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn run(socket: &UdpSocket, imu_pub: &Publisher<IMUState>, logger: &Logger, running: &AtomicBool) {
    // 一個 UDP datagram 最大 65507 bytes
    let mut buf = vec![0u8; 65_507];

    while running.load(Ordering::Relaxed) {
        let n = match socket.recv(&mut buf) {
            Ok(n) => n,
            // 逾時：回去檢查 running
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(e) => {
                log_error!(
                    logger.throttle(LOG_THROTTLE),
                    "[Telemetry] recv failed: {e}"
                );
                continue;
            }
        };

        match protocol::parse_imu(&buf[..n]) {
            Ok(msg) => {
                if let Err(e) = imu_pub.publish(msg) {
                    log_error!(
                        logger.throttle(LOG_THROTTLE),
                        "[Telemetry] publish failed: {e}"
                    );
                }
            }
            Err(e) => {
                log_warn!(
                    logger.throttle(LOG_THROTTLE),
                    "[Telemetry] drop {n}-byte packet: {e}"
                );
            }
        }
    }
}
