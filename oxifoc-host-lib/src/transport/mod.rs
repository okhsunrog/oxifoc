//! Transport abstraction for oxifoc host communication.
//!
//! Provides a unified interface for different transport backends:
//! - Serial (UART over USB VCP)
//! - RTT (via debug probe using probe-rs)
//! - TCP (for oxifoc-virtual or remote devices)

pub mod rtt;
pub mod serial;
pub mod tcp;

use anyhow::Result;
use serde::Deserialize;
use tokio::io::{AsyncRead, AsyncWrite};

/// Transport type selection.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TransportType {
    /// Serial port (UART over USB VCP) - default
    #[default]
    Serial,
    /// RTT via debug probe (probe-rs)
    Rtt,
    /// TCP connection (for oxifoc-virtual or remote devices)
    Tcp,
}

/// Configuration for transport selection.
#[derive(Debug, Clone)]
pub enum TransportConfig {
    Serial { path: String, baud: u32 },
    Rtt { probe: Option<String>, chip: String },
    Tcp { host: String, port: u16 },
}

/// A boxed transport that can be used for async I/O.
pub struct Transport {
    /// Reader for ergot data (device -> host)
    pub reader: Box<dyn AsyncRead + Send + Unpin>,
    /// Writer for ergot data (host -> device)
    pub writer: Box<dyn AsyncWrite + Send + Unpin>,
    /// Optional reader for defmt data (RTT mode only, channel 0)
    /// In serial mode, defmt is forwarded over ergot network instead.
    pub defmt_reader: Option<Box<dyn AsyncRead + Send + Unpin>>,
}

impl Transport {
    /// Create a new transport from the given configuration.
    pub async fn new(config: TransportConfig) -> Result<Self> {
        match config {
            TransportConfig::Serial { path, baud } => serial::connect(&path, baud).await,
            TransportConfig::Rtt { probe, chip } => rtt::connect(probe.as_deref(), &chip).await,
            TransportConfig::Tcp { host, port } => tcp::connect(&host, port).await,
        }
    }
}
