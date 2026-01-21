//! Protocol layer for ergot communication and device management

pub mod servers;

use core::sync::atomic::{AtomicU8, Ordering};
use static_cell::StaticCell;

use crate::config::MAX_PACKET_SIZE;
use crate::transport::{Queue, Stack};

// ========== Ergot Stack ==========

/// Statically store our outgoing packet buffer
pub static OUTQ: Queue = Queue::new();

/// Statically store our netstack
pub static STACK: Stack = ergot::toolkits::embedded_io_async::new_target_stack(
    OUTQ.stream_producer(),
    MAX_PACKET_SIZE as u16,
);

/// Buffers for RX worker
pub static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
pub static SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// ========== Device State Management ==========

/// Device operational state
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DeviceState {
    Boot = 0,
    WaitingLink = 1,
    Linked = 2,
    Error = 3,
}

static DEVICE_STATE: AtomicU8 = AtomicU8::new(DeviceState::Boot as u8);

pub fn set_device_state(s: DeviceState) {
    DEVICE_STATE.store(s as u8, Ordering::Relaxed);
}

pub fn get_device_state() -> DeviceState {
    match DEVICE_STATE.load(Ordering::Relaxed) {
        0 => DeviceState::Boot,
        1 => DeviceState::WaitingLink,
        2 => DeviceState::Linked,
        _ => DeviceState::Error,
    }
}
