//! Protocol layer for ergot communication over USB + UART

pub mod servers;

use static_cell::StaticCell;

use crate::config::MAX_PACKET_SIZE;

/// Buffers for RX workers
pub static USB_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
#[cfg(feature = "transport-usb")]
pub static USB_IDENT: embassy_sync::once_lock::OnceLock<u8> =
    embassy_sync::once_lock::OnceLock::new();
pub static UART_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
pub static UART_SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
pub static RTT_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
pub static RTT_SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();
