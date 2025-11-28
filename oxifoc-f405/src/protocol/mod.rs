//! Protocol layer for ergot communication over USB

pub mod servers;

use crate::config::MAX_PACKET_SIZE;
use crate::transport::{Queue, Stack};
use static_cell::StaticCell;

// ========== Ergot Stack ==========

/// Statically store our outgoing packet buffer
pub static OUTQ: Queue = ergot::toolkits::embassy_usb_v0_5::Queue::new();

/// Statically store our netstack
pub static STACK: Stack = ergot::toolkits::embassy_usb_v0_5::new_target_stack(
    OUTQ.framed_producer(),
    MAX_PACKET_SIZE as u16,
);

/// Buffer for RX worker
pub static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
