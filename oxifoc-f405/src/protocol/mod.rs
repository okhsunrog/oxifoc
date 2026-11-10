//! Protocol layer for ergot communication over USB and optional RTT

pub mod servers;
#[cfg(feature = "bench-usb-metrics")]
pub mod usb_metrics;

use static_cell::StaticCell;

use crate::config::MAX_PACKET_SIZE;

/// Buffers for RX workers
pub static USB_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
#[cfg(feature = "transport-usb")]
pub static USB_IDENT: embassy_sync::once_lock::OnceLock<u8> =
    embassy_sync::once_lock::OnceLock::new();
#[cfg(feature = "transport-rtt")]
pub static RTT_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
pub static RTT_SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();
