//! Transport abstraction for oxifoc host communication.
//!
//! Provides different transport backends:
//! - Serial (UART over USB VCP) — COBS stream
//! - RTT (via debug probe using probe-rs) — COBS stream
//! - TCP (for oxifoc-virtual or remote devices) — COBS stream
//! - UDP — framed (no COBS)
//! - USB (via nusb bulk endpoints) — framed (no COBS)

pub mod ble;
#[cfg(feature = "desktop")]
pub mod rtt;
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
    /// RTT via debug probe (probe-rs)
    #[cfg(feature = "desktop")]
    Rtt,
    /// TCP connection (for oxifoc-virtual or remote devices)
    Tcp,
    /// UDP connection
    Udp,
    /// USB bulk (via nusb, ergot device class)
    Usb,
    /// BLE (via bluest) - default on Android
    #[cfg_attr(not(feature = "desktop"), default)]
    Ble,
}

/// Configuration for transport selection.
#[derive(Debug, Clone)]
pub enum TransportConfig {
    #[cfg(feature = "desktop")]
    Serial {
        path: String,
        baud: u32,
    },
    #[cfg(feature = "desktop")]
    Rtt {
        probe: Option<String>,
        chip: String,
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
        device: bluest::Device,
    },
}

/// A COBS-stream transport (TCP, serial, RTT).
///
/// Returns AsyncRead/AsyncWrite pairs for use with ergot's `tokio_stream` toolkit.
/// UDP and USB use different ergot toolkits and don't go through this struct.
pub struct CobsStreamTransport {
    /// Reader for ergot data (device -> host)
    pub reader: Box<dyn AsyncRead + Send + Unpin>,
    /// Writer for ergot data (host -> device)
    pub writer: Box<dyn AsyncWrite + Send + Unpin>,
    /// Optional reader for defmt data (RTT mode only, channel 0).
    /// In serial/TCP mode, defmt is forwarded over ergot network instead.
    pub defmt_reader: Option<Box<dyn AsyncRead + Send + Unpin>>,
}
