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
use ergot::interface_manager::profiles::direct_edge::EdgeFrameProcessor;
use ergot::interface_manager::profiles::router::Router;
use ergot::interface_manager::transports::eio::RxWorker as EioRxWorker;
use esp_hal::Async;
use esp_hal::uart::UartRx;
use mutex::raw_impls::cs::CriticalSectionRawMutex;

// ========== Constants ==========

/// Sized to hold one max-size UART frame plus headroom for small frames, so a
/// telemetry burst can't starve protocol responses.
pub const UART_QUEUE_SIZE: usize = 4096;
pub const BLE_QUEUE_SIZE: usize = 1024;
/// Matches the host stack's ERGOT_MTU (2048) so the bridge is transparent for
/// anything the rest of the system can emit. The STM32 firmwares cap their own
/// frames at MAX_PACKET_SIZE (400-512), so device->bridge traffic is far below
/// this; the limit matters for host->device frames routed through the bridge.
pub const UART_MTU: u16 = 2048;
/// BLE NUS MTU: PacketPool MTU (512) - L2CAP header (4) - ATT header (3) = 505.
///
/// This is a physical ATT cap, not a tunable: GATT notifications carry at most
/// ATT_MTU-3 bytes and OS stacks (Android/macOS via bluest) top out near 512.
/// Frames larger than this cannot traverse the BLE leg — when streaming
/// telemetry over BLE, the batch size must be configured to fit.
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
/// 2 direct interfaces (BLE slot + room to grow), 8 seed routes: the BLE
/// segment's net is leased from the root, and nested bridges would add more.
type BridgeRouter = Router<BridgeInterface, Rng, 2, 8>;
pub type Stack = NetStack<CriticalSectionRawMutex, BridgeRouter>;

/// UART RxWorker: eio RxWorker with esp-hal async UartRx. The upstream is an
/// edge of the root's segment, so it uses [`EdgeFrameProcessor`] (net_id
/// discovered from inbound frames), not the router-side processor.
pub type UartRxWorker = EioRxWorker<&'static Stack, UartRx<'static, Async>, EdgeFrameProcessor>;

// ========== Static resources ==========

pub static UART_OUTQ: UartQueue = BBQueue::new();
pub static BLE_OUTQ: BleQueue = BBQueue::new();
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();
