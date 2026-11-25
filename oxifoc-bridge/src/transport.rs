//! C5 bridge: a router with SPI, USB Serial/JTAG, CAN FD and optional BLE
//! and Wi-Fi segments. It is the root, or has the CAN bus as its upstream.
use core::sync::atomic::{AtomicU32, Ordering};
use ergot::{
    NetStack,
    exports::bbqueue::traits::coordination::cas::AtomicCoord,
    interface_manager::{Interface, profiles::router::Router, utils::framed_stream},
    toolkits::embedded_io_async_v0_7 as kit,
};

use mutex::raw_impls::cs::CriticalSectionRawMutex;

pub mod can;
pub mod spi;
#[cfg(feature = "wifi")]
pub mod tcp;
pub mod usb;
/// Task priority of the forwarding path, ahead of DAP and housekeeping.
pub const FORWARDING: u8 = 1;
/// Task priority of the path a fast telemetry stream takes, ahead of the
/// Wi-Fi stack: routing a received slot, the relay, and emptying the host
/// link's queue into USB or the TCP socket. The BLE host shares it, so the
/// stream does not delay a remote's commands. The Wi-Fi stack sends several
/// segments in one poll; at this priority it kept the SPI link below a
/// 20 kHz stream.
pub const LINK: u8 = 2;
pub const MTU: u16 = 2304;
pub const BLE_MTU: u16 = 505;
pub type SpiQueue = kit::Queue<16384, AtomicCoord>;
/// Wi-Fi station credentials were set when the image was built.
pub const WIFI_CONFIGURED: bool = cfg!(feature = "wifi")
    && matches!(option_env!("OXIFOC_PROBE_WIFI_SSID"), Some(ssid) if !ssid.is_empty());
// An image with Wi-Fi reserves RAM for the TCP transmit buffer, its queue
// and the radio heap, and leaves USB the smaller queue.
const USB_QUEUE_BYTES: usize = if WIFI_CONFIGURED {
    28 * 1024
} else {
    108 * 1024
};
pub type UsbQueue = kit::Queue<USB_QUEUE_BYTES, AtomicCoord>;
/// The Wi-Fi backlog: frames wait here, not in the TCP window.
pub type WifiQueue = kit::Queue<{ 36 * 1024 }, AtomicCoord>;
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
    type Sink = Counted<ergot::interface_manager::utils::cobs_stream::Sink<&'static WifiQueue>>;
}
pub struct BleNusInterface;
pub struct UsbInterface;
impl Interface for UsbInterface {
    type Sink = Counted<ergot::interface_manager::utils::cobs_stream::Sink<&'static UsbQueue>>;
}
impl Interface for BleNusInterface {
    type Sink = Counted<framed_stream::Sink<&'static BleQueue>>;
}
pub struct CanLink;
impl Interface for CanLink {
    type Sink = ergot::interface_manager::utils::can::Sink<&'static can::Queue, { can::MTU }>;
}
ergot::multi_interface! {
    pub enum BridgeSink for BridgeInterface {
        Spi(SpiInterface),
        Usb(UsbInterface),
        Ble(BleNusInterface),
        Wifi(WifiInterface),
        Can(CanLink),
    }
}
/// Segments a bridge can route: SPI, USB, two BLE links, the CAN bus when it
/// is not the upstream, and the Wi-Fi port.
pub const SEGMENTS: usize = 5 + cfg!(feature = "wifi") as usize;
/// Nets the root leases to the bridges behind the bus, one for each of
/// their segments. A bridge that restarts asks for new ones while its old
/// ones run out.
pub const LEASED_NETS: usize = 16;
pub type Stack = NetStack<
    CriticalSectionRawMutex,
    Router<BridgeInterface, esp_hal::rng::Rng, SEGMENTS, LEASED_NETS, { can::CLAIMS }>,
>;
pub static SPI_OUTQ: SpiQueue = kit::Queue::new();
pub static USB_OUTQ: UsbQueue = kit::Queue::new();
pub static BLE_OUTQ: [BleQueue; BLE_LINKS] = [const { kit::Queue::new() }; BLE_LINKS];
pub static STATE_NOTIFY: ergot::exports::maitake_sync::WaitQueue =
    ergot::exports::maitake_sync::WaitQueue::new();

/// Wait until the outbound queue of the link to network `net` has contiguous
/// room for `bytes`. Returns at once for a link without a host, and for BLE,
/// whose queue is smaller than one SPI slot.
pub async fn room(net: u16, bytes: usize) {
    if net == usb::NET.load(Ordering::Relaxed) {
        // The probe reserves nothing: an uncommitted grant is released.
        drop(USB_OUTQ.stream_producer().wait_grant_exact(bytes).await);
    }
    #[cfg(feature = "wifi")]
    if net == tcp::NET.load(Ordering::Relaxed) {
        tcp::room(bytes).await;
    }
}

/// Registers a segment of this bridge. The root numbers its segments
/// itself; a bridge with an upstream leaves each without a net until the
/// root has leased it one ([`can::uplink`]).
pub fn register(
    stack: &'static Stack,
    sink: BridgeSink,
) -> Result<u8, ergot::interface_manager::profiles::router::RegisterError> {
    stack.manage_profile(|p| {
        if can::UPSTREAM {
            p.register_interface_pending(sink)
        } else {
            p.register_interface(sink)
        }
    })
}

/// What every bridge answers, and what the root does besides: it owns the
/// nets, and grants the node addresses of the CAN bus. A bridge with an
/// upstream passes requests for nets on to the root.
#[embassy_executor::task]
pub async fn root_services(stack: &'static Stack) {
    embassy_futures::join::join5(
        stack.services().seed_router_request_handler::<2>(),
        stack.services().ping_handler::<2>(),
        stack.services().socket_query_handler::<2>(),
        crate::diag::server(stack),
        async {
            if !can::UPSTREAM {
                stack.services().address_claim_handler::<2>().await;
            }
        },
    )
    .await;
}
