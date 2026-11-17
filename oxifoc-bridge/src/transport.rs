//! C5 bridge: root router with SPI, USB Serial/JTAG and optional BLE segments.
use ergot::{
    NetStack,
    exports::bbqueue::traits::coordination::cas::AtomicCoord,
    interface_manager::{Interface, profiles::router::Router, utils::framed_stream},
    toolkits::embedded_io_async_v0_7 as kit,
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;

pub mod spi;
pub mod usb;
pub const MTU: u16 = 2304;
pub const BLE_MTU: u16 = 505;
pub type SpiQueue = kit::Queue<16384, AtomicCoord>;
// About 200 ms of 20-kHz Packed28 traffic, including framing and control.
// Host scheduling jitter is buffered on C5, without consuming STM32 SRAM.
pub type UsbQueue = kit::Queue<131072, AtomicCoord>;
pub type BleQueue = kit::Queue<1024, AtomicCoord>;
/// Simultaneous BLE centrals, e.g. the remote and a phone or laptop. Each link
/// is its own ergot interface with its own outbound queue.
pub const BLE_LINKS: usize = 2;
pub struct SpiInterface;
impl Interface for SpiInterface {
    type Sink = framed_stream::Sink<&'static SpiQueue>;
}
pub struct BleNusInterface;
pub struct UsbInterface;
impl Interface for UsbInterface {
    type Sink = usb::UsbSink;
}
impl Interface for BleNusInterface {
    type Sink = framed_stream::Sink<&'static BleQueue>;
}
ergot::multi_interface! {
    pub enum BridgeSink for BridgeInterface {
        Spi(SpiInterface),
        Usb(UsbInterface),
        Ble(BleNusInterface),
    }
}
pub type Stack =
    NetStack<CriticalSectionRawMutex, Router<BridgeInterface, esp_hal::rng::Rng, 4, 8>>;
pub static SPI_OUTQ: SpiQueue = kit::Queue::new();
pub static USB_OUTQ: UsbQueue = kit::Queue::new();
pub static BLE_OUTQ: [BleQueue; BLE_LINKS] = [const { kit::Queue::new() }; BLE_LINKS];
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();

/// C5 owns all directly attached segments. No upstream seed leases are needed.
#[embassy_executor::task]
pub async fn root_services(stack: &'static Stack) {
    embassy_futures::join::join3(
        stack.services().seed_router_request_handler::<2>(),
        stack.services().ping_handler::<2>(),
        stack.services().socket_query_handler::<2>(),
    )
    .await;
}
