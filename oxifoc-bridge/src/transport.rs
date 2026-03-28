//! Transport setup for the bridge device.
//!
//! The bridge runs a Router in bridge mode:
//! - Upstream: UART1 on GPIO19 (TX) / GPIO20 (RX), COBS stream
//! - Downstream: BLE NUS (GATT) → devices connect via BLE

use bbqueue::BBQueue;
use bbqueue::traits::coordination::cas::AtomicCoord;
use bbqueue::traits::notifier::maitake::MaiNotSpsc;
use bbqueue::traits::storage::Inline;
use ergot::NetStack;
use ergot::interface_manager::interface_impls::embedded_io::IoInterface;
use ergot::interface_manager::profiles::router::{Router, RouterFrameProcessor};
use ergot::interface_manager::transports::eio::RxWorker as EioRxWorker;
use esp_hal::Async;
use esp_hal::uart::UartRx;
use mutex::raw_impls::cs::CriticalSectionRawMutex;

// ========== Constants ==========

pub const UART_QUEUE_SIZE: usize = 2048;
pub const BLE_QUEUE_SIZE: usize = 1024;
pub const UART_MTU: u16 = 512;
/// BLE NUS MTU: PacketPool MTU (512) - L2CAP header (4) - ATT header (3) = 505
pub const BLE_MTU: u16 = 505;
pub const UART_BAUD: u32 = 921_600;
pub const LIVENESS_TIMEOUT_MS: u64 = 5000;

// ========== Queue types ==========

pub type UartQueue = BBQueue<Inline<UART_QUEUE_SIZE>, AtomicCoord, MaiNotSpsc>;
pub type BleQueue = BBQueue<Inline<BLE_QUEUE_SIZE>, AtomicCoord, MaiNotSpsc>;
type UartQueueRef = &'static UartQueue;
type BleQueueRef = &'static BleQueue;

// ========== BLE NUS Interface ==========

pub struct BleNusInterface;

impl ergot::interface_manager::Interface for BleNusInterface {
    type Sink = ergot::interface_manager::utils::framed_stream::Sink<BleQueueRef>;
}

// ========== Multi-interface ==========

ergot::multi_interface! {
    pub enum BridgeSink for BridgeInterface {
        Uart(IoInterface<UartQueueRef>),
        Ble(BleNusInterface),
    }
}

// ========== Router & Stack ==========

pub type Rng = esp_hal::rng::Rng;
type BridgeRouter = Router<BridgeInterface, Rng, 2, 0>;
pub type Stack = NetStack<CriticalSectionRawMutex, BridgeRouter>;

/// UART RxWorker: eio RxWorker with esp-hal async UartRx
pub type UartRxWorker = EioRxWorker<&'static Stack, UartRx<'static, Async>, RouterFrameProcessor>;

// ========== Static resources ==========

pub static UART_OUTQ: UartQueue = BBQueue::new();
pub static BLE_OUTQ: BleQueue = BBQueue::new();
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();
