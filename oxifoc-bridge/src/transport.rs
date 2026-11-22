//! C5 bridge: root router with SPI, USB Serial/JTAG and optional BLE segments.
use core::sync::atomic::{AtomicU32, Ordering};
use ergot::{
    NetStack,
    exports::bbqueue::traits::coordination::cas::AtomicCoord,
    interface_manager::{Interface, profiles::router::Router, utils::framed_stream},
    toolkits::embedded_io_async_v0_7 as kit,
};

use mutex::raw_impls::cs::CriticalSectionRawMutex;

pub mod spi;
#[cfg(feature = "wifi")]
pub mod udp;
pub mod usb;
pub const MTU: u16 = 2304;
pub const BLE_MTU: u16 = 505;
pub type SpiQueue = kit::Queue<16384, AtomicCoord>;
// Wi-Fi builds with a configured Ergot peer reserve RAM for the network
// queue and datagram history. The USB-only budget covers about 200 ms at 20 kHz.
const USB_QUEUE_BYTES: usize =
    if cfg!(feature = "wifi") && option_env!("OXIFOC_BRIDGE_ERGOT_PEER").is_some() {
        44 * 1024
    } else {
        124 * 1024
    };
pub type UsbQueue = kit::Queue<USB_QUEUE_BYTES, AtomicCoord>;
pub type WifiQueue = kit::Queue<{ 24 * 1024 }, AtomicCoord>;
pub type BleQueue = kit::Queue<1024, AtomicCoord>;
/// Simultaneous BLE centrals, e.g. the remote and a phone or laptop. Each link
/// is its own ergot interface with its own outbound queue.
pub const BLE_LINKS: usize = 2;
pub struct SpiInterface;
impl Interface for SpiInterface {
    type Sink = framed_stream::Sink<&'static SpiQueue>;
}

/// A sink that counts the frames its queue refuses (`req/bridge_diag`).
pub struct Counted<S> {
    inner: S,
    dropped: &'static AtomicU32,
}

impl<S> Counted<S> {
    pub const fn new(inner: S, dropped: &'static AtomicU32) -> Self {
        Self { inner, dropped }
    }

    fn count(&self, result: Result<(), ()>) -> Result<(), ()> {
        if result.is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        result
    }
}

impl<S: ergot::interface_manager::InterfaceSink> ergot::interface_manager::InterfaceSink
    for Counted<S>
{
    fn mtu(&self) -> u16 {
        self.inner.mtu()
    }
    fn send_ty<T: serde::Serialize>(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        body: &T,
    ) -> Result<(), ()> {
        let result = self.inner.send_ty(link, hdr, body);
        self.count(result)
    }
    fn send_raw(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        body: &[u8],
    ) -> Result<(), ()> {
        let result = self.inner.send_raw(link, hdr, body);
        self.count(result)
    }
    fn send_err(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        error: ergot::ProtocolError,
    ) -> Result<(), ()> {
        let result = self.inner.send_err(link, hdr, error);
        self.count(result)
    }
}
pub struct WifiInterface;
impl Interface for WifiInterface {
    type Sink = framed_stream::Sink<&'static WifiQueue>;
}
pub struct BleNusInterface;
pub struct UsbInterface;
impl Interface for UsbInterface {
    type Sink = Counted<ergot::interface_manager::utils::cobs_stream::Sink<&'static UsbQueue>>;
}
impl Interface for BleNusInterface {
    type Sink = Counted<framed_stream::Sink<&'static BleQueue>>;
}
ergot::multi_interface! {
    pub enum BridgeSink for BridgeInterface {
        Spi(SpiInterface),
        Usb(UsbInterface),
        Ble(BleNusInterface),
        Wifi(WifiInterface),
    }
}
pub type Stack = NetStack<
    CriticalSectionRawMutex,
    Router<BridgeInterface, esp_hal::rng::Rng, { 4 + cfg!(feature = "wifi") as usize }, 8>,
>;
pub static SPI_OUTQ: SpiQueue = kit::Queue::new();
pub static USB_OUTQ: UsbQueue = kit::Queue::new();
pub static BLE_OUTQ: [BleQueue; BLE_LINKS] = [const { kit::Queue::new() }; BLE_LINKS];
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();

/// C5 owns all directly attached segments. No upstream seed leases are needed.
#[embassy_executor::task]
pub async fn root_services(stack: &'static Stack) {
    embassy_futures::join::join4(
        stack.services().seed_router_request_handler::<2>(),
        stack.services().ping_handler::<2>(),
        stack.services().socket_query_handler::<2>(),
        crate::diag::server(stack),
    )
    .await;
}
