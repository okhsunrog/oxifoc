//! Transport abstraction for oxifoc host communication.
//!
//! Provides different transport backends:
//! - Serial (UART over USB VCP) — COBS stream
//! - TCP (for oxifoc-virtual or remote devices) — COBS stream
//! - UDP — framed (no COBS)
//! - USB (via nusb bulk endpoints) — framed (no COBS)

pub mod ble;
#[cfg(feature = "desktop")]
pub mod serial;
pub mod tcp;
pub mod udp;
pub mod usb;

use serde::Deserialize;
use tokio::io::{AsyncRead, AsyncWrite};

/// Transport type selection.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TransportType {
    /// Serial port (UART over USB VCP) - default on desktop
    #[cfg_attr(feature = "desktop", default)]
    #[cfg(feature = "desktop")]
    Serial,
    /// Espressif built-in USB Serial/JTAG (COBS stream, USB speed independent of baud).
    #[cfg(feature = "desktop")]
    #[serde(rename = "usb-serial-jtag")]
    UsbSerialJtag,
    /// TCP connection (for oxifoc-virtual or remote devices)
    Tcp,
    /// UDP connection
    Udp,
    /// USB bulk (via nusb, ergot device class)
    Usb,
    /// BLE (via blec) - default on Android
    #[cfg_attr(not(feature = "desktop"), default)]
    Ble,
}

impl TransportType {
    /// Largest raw telemetry block one frame of this link carries, the
    /// `max_block_bytes` of every session Arm. The device lowers it further
    /// to its own link limit. BLE carries one 505-byte frame per GATT
    /// notification and nothing fragments a frame on the way, so a session
    /// through the bridge's radio gets 15-record blocks; every other host
    /// transport takes full blocks.
    pub fn raw_block_limit(&self) -> u16 {
        match self {
            Self::Ble => oxifoc_core::icd::raw_block_limit(ble::BLE_MTU),
            _ => oxifoc_core::raw_telemetry::RAW_MAX_BLOCK_BYTES as u16,
        }
    }
}

/// Configuration for transport selection.
#[derive(Debug, Clone)]
pub enum TransportConfig {
    #[cfg(feature = "desktop")]
    Serial {
        path: String,
        baud: u32,
    },
    Tcp {
        host: String,
        port: u16,
    },
    Udp {
        host: String,
        port: u16,
    },
    Usb,
    Ble {
        device: String,
    },
}

/// A COBS-stream transport (TCP, serial).
///
/// Returns AsyncRead/AsyncWrite pairs for use with ergot's `tokio_stream` toolkit.
/// UDP and USB use different ergot toolkits and don't go through this struct.
pub struct CobsStreamTransport {
    /// Reader for ergot data (device -> host)
    pub reader: Box<dyn AsyncRead + Send + Unpin>,
    /// Writer for ergot data (host -> device)
    pub writer: Box<dyn AsyncWrite + Send + Unpin>,
}

#[cfg(test)]
mod tests {
    use super::TransportType;
    use oxifoc_core::raw_telemetry::RAW_MAX_BLOCK_BYTES;

    #[test]
    fn ble_sessions_use_blocks_that_fit_one_notification() {
        // 505-byte frame: 24-byte Ergot header, 3-byte length prefix, then a
        // 32-byte block header and 15 Packed28 records (452 bytes).
        assert_eq!(TransportType::Ble.raw_block_limit(), 478);
        assert_eq!(
            usize::from(TransportType::Usb.raw_block_limit()),
            RAW_MAX_BLOCK_BYTES
        );
        assert_eq!(
            usize::from(TransportType::Tcp.raw_block_limit()),
            RAW_MAX_BLOCK_BYTES
        );
    }
}
