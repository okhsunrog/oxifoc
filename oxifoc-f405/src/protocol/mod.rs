//! Protocol services and receive storage for enabled Ergot transports.

pub mod servers;

#[cfg(feature = "transport-usb")]
use static_cell::StaticCell;

#[cfg(feature = "transport-usb")]
use crate::config::MAX_PACKET_SIZE;

/// Buffers for RX workers
#[cfg(feature = "transport-usb")]
pub static USB_RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
#[cfg(feature = "transport-usb")]
pub static USB_IDENT: embassy_sync::once_lock::OnceLock<u8> =
    embassy_sync::once_lock::OnceLock::new();
