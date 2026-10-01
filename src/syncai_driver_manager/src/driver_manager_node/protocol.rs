//! 底層 UDP 封包 <-> 資料結構 的轉換。
//!
//! 這裡只放純函式：不碰 socket、不碰 Node，所以可以直接 `cargo test`，
//! 不需要 ROS 環境或實機。

use std::fmt;

use ros_env::syncai_common::msg::IMUState;

#[derive(Debug, PartialEq, Eq)]
pub enum ProtocolError {
    Unimplemented,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unimplemented => write!(f, "telemetry packet decoding is not implemented yet"),
        }
    }
}

impl std::error::Error for ProtocolError {}

/// 把一個 telemetry 封包解成 IMUState。
// TODO: 依照下位機的封包格式（header / 欄位順序 / endianness / checksum）實作，
//       並補上用真實封包 bytes 寫的 #[cfg(test)] 測試。
pub fn parse_imu(_packet: &[u8]) -> Result<IMUState, ProtocolError> {
    Err(ProtocolError::Unimplemented)
}
