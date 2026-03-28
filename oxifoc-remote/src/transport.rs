//! Transport setup for the remote controller.
//!
//! The remote is a BLE central (edge device) that connects to
//! the oxifoc-bridge's NUS GATT service.

use bbqueue::BBQueue;
use bbqueue::traits::coordination::cas::AtomicCoord;
use bbqueue::traits::notifier::maitake::MaiNotSpsc;
use bbqueue::traits::storage::Inline;
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
use ergot::NetStack;
use mutex::raw_impls::cs::CriticalSectionRawMutex;

// ========== Constants ==========

pub const BLE_QUEUE_SIZE: usize = 1024;

/// BLE NUS MTU: PacketPool MTU (512) - L2CAP (4) - ATT (3) = 505
pub const BLE_MTU: u16 = 505;

// ========== Queue types ==========

pub type BleQueue = BBQueue<Inline<BLE_QUEUE_SIZE>, AtomicCoord, MaiNotSpsc>;
type BleQueueRef = &'static BleQueue;

// ========== BLE NUS Interface ==========

pub struct BleNusInterface;

impl ergot::interface_manager::Interface for BleNusInterface {
    type Sink = ergot::interface_manager::utils::framed_stream::Sink<BleQueueRef>;
}

// ========== Stack ==========

pub type Stack = NetStack<CriticalSectionRawMutex, DirectEdge<BleNusInterface>>;

// ========== Static resources ==========

pub static BLE_OUTQ: BleQueue = BBQueue::new();
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();
