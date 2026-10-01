use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use rclrs::*;

use super::protocol;
use super::session::UdpSession;

const LOG_THROTTLE: Duration = Duration::from_secs(1);

/// 往下位機送 ASCII 指令。
///
/// UDP 沒有 ack：`send` 成功只代表封包進了 kernel，不代表控制器收到；
/// 掉了就是掉了，沒有重送，也沒有人會知道。
pub struct CommandLink {
    session: UdpSession,
    logger: Logger,
}

impl CommandLink {
    pub fn new(session: UdpSession, logger: Logger) -> Self {
        Self { session, logger }
    }

    pub fn send(&self, command: &str) {
        if let Err(e) = self.session.socket.send(command.as_bytes()) {
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[Command] send {:?} to {:?} failed: {e}",
                command.trim_end(),
                self.session.peer_addr,
            );
        }
    }
}

/// 鎖住時 `set_motion_key` 只放行 ESTOP，只有 `reset_safety` service 能解鎖。
///
/// 目前沒有任何東西會觸發它（電量 / JOINT_TEMP 過熱都還是 TODO），所以實際上永遠不會鎖。
/// 另外，cmd_vel 與 set_policy_mode 都**不受**這把鎖限制，跟參考實作不同。
#[derive(Default)]
pub struct SafetyLock {
    engaged: AtomicBool,
}

impl SafetyLock {
    pub fn is_engaged(&self) -> bool {
        self.engaged.load(Ordering::Acquire)
    }

    /// 解鎖；回傳解鎖前是否鎖著
    pub fn release(&self) -> bool {
        self.engaged.swap(false, Ordering::AcqRel)
    }

    /// 上鎖並讓機器人趴下（MODE X）。用 swap 做 check-and-set，同時多個觸發也只會動作一次。
    /// 可以從 telemetry thread 呼叫。
    // TODO: 接上觸發條件。JOINT_TEMP 的門檻沿用參考實作：>= 75 °C 警告、>= 95 °C 趴下、
    //       >= 115 °C ESTOP。
    #[expect(dead_code, reason = "no safety trigger is wired yet")]
    pub fn trigger(&self, reason: &str, command: &CommandLink, logger: &Logger) {
        if !self.engaged.swap(true, Ordering::AcqRel) {
            log_error!(logger, "[Safety] !!! SAFETY TRIGGERED: {reason} !!!");
            log_error!(
                logger,
                "[Safety] Executing LieDown [MODE X] and blocking control"
            );
            command.send(protocol::LIE_DOWN_COMMAND);
        }
    }
}
