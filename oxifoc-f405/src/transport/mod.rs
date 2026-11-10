//! MK5 SPI DirectEdge and standalone diagnostic USB/RTT router profiles.
//! defmt diagnostics retain a separate RTT channel (read via probe-rs).

#[cfg(feature = "transport-usb")]
use embassy_stm32::Peri;
use embassy_stm32::rng;
#[cfg(feature = "transport-usb")]
use embassy_stm32::usb;
use embassy_stm32::{bind_interrupts, peripherals};
use ergot::NetStack;
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::exports::maitake_sync::WaitQueue;
#[cfg(any(feature = "transport-usb", feature = "transport-rtt"))]
use ergot::interface_manager::LivenessConfig;
#[cfg(feature = "transport-usb")]
use ergot::interface_manager::interface_impls::embassy_usb::EmbassyInterface;
#[cfg(any(feature = "transport-rtt", feature = "transport-usb"))]
use ergot::interface_manager::interface_impls::embedded_io::IoInterface;
#[cfg(feature = "transport-spi")]
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
#[cfg(not(feature = "transport-spi"))]
use ergot::interface_manager::profiles::router::Router;
#[cfg(any(feature = "transport-usb", feature = "transport-rtt"))]
use ergot::interface_manager::profiles::router::RouterFrameProcessor;
#[cfg(feature = "transport-rtt")]
use ergot::interface_manager::utils::cobs_stream;
use ergot::interface_manager::utils::framed_stream;
#[cfg(feature = "transport-usb")]
use ergot::toolkits::embassy_usb_v0_6 as usb_kit;
use ergot::toolkits::embedded_io_async_v0_7 as io_kit;
#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::{RttReader, RttWriter};
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

#[cfg(any(feature = "transport-usb", feature = "transport-rtt"))]
use crate::config::MAX_PACKET_SIZE;
#[cfg(any(feature = "transport-rtt", feature = "transport-usb"))]
use crate::config::RTT_OUT_QUEUE_SIZE;
#[cfg(feature = "transport-usb")]
use crate::config::USB_OUT_QUEUE_SIZE;
#[cfg(feature = "transport-rtt")]
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;

mod stack_mutex;

#[cfg(any(
    all(feature = "bench-raw-usb", feature = "bench-raw-rtt"),
    all(feature = "bench-raw-usb", feature = "fast-telemetry"),
    all(feature = "bench-raw-rtt", feature = "fast-telemetry")
))]
compile_error!("select only one raw telemetry sink");
#[cfg(all(
    any(
        feature = "bench-raw-usb",
        feature = "bench-raw-rtt",
        feature = "fast-telemetry"
    ),
    feature = "transport-rtt"
))]
compile_error!("raw telemetry benchmarks use Ergot USB control, not transport-rtt");

#[cfg(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "fast-telemetry"
))]
pub mod raw_fast;

// A leaf has exactly one upstream. USB/RTT remain standalone diagnostic
// builds, rather than accidentally introducing a second root on the SPI link.
#[cfg(all(
    feature = "transport-spi",
    any(feature = "transport-usb", feature = "transport-rtt")
))]
compile_error!("SPI DirectEdge cannot be combined with diagnostic USB/RTT routing");

// ========== Multi-Interface Definition ==========
#[cfg(feature = "transport-usb")]
type UsbQ = &'static UsbQueue;
#[cfg(any(feature = "transport-rtt", feature = "transport-usb"))]
type RttQ = &'static RttQueue;

#[cfg(feature = "transport-spi")]
pub mod spi;
pub type SpiQueue = io_kit::Queue<16384, AtomicCoord>;
pub struct SpiInterface;
impl ergot::interface_manager::Interface for SpiInterface {
    type Sink = framed_stream::Sink<&'static SpiQueue>;
}
#[cfg(feature = "transport-spi")]
pub static SPI_OUTQ: SpiQueue = io_kit::Queue::new();

#[cfg(feature = "transport-usb")]
ergot::multi_interface! {
    pub enum McSink for McInterface {
        Usb(EmbassyInterface<UsbQ>),
        Spi(SpiInterface),
        Rtt(IoInterface<RttQ>),
    }
}
#[cfg(all(not(feature = "transport-usb"), feature = "transport-rtt"))]
ergot::multi_interface! {
    pub enum McSink for McInterface {
        Spi(SpiInterface),
        Rtt(IoInterface<RttQ>),
    }
}
// No tagged multi-interface dispatch or dormant RTT interface in SPI-only
// firmware. The leaf's sink is the SPI framed queue itself.
#[cfg(not(any(feature = "transport-usb", feature = "transport-rtt")))]
pub type McInterface = SpiInterface;

// ========== Type Aliases ==========

pub type Rng = rng::Rng<'static, peripherals::RNG>;
/// Standalone diagnostic router capacity. Production SPI is a single leaf.
#[cfg(not(feature = "transport-spi"))]
const INTERFACES: usize =
    1 + cfg!(feature = "transport-usb") as usize + cfg!(feature = "transport-rtt") as usize;
#[cfg(not(feature = "transport-spi"))]
pub type McProfile = Router<McInterface, Rng, INTERFACES, 8>;
#[cfg(feature = "transport-spi")]
pub type McProfile = DirectEdge<McInterface>;
pub type InterfaceIdent = <McProfile as ergot::interface_manager::Profile>::InterfaceIdent;
pub type Stack = NetStack<stack_mutex::StackRawMutex, McProfile>;

#[cfg(feature = "transport-usb")]
pub type UsbQueue = usb_kit::Queue<USB_OUT_QUEUE_SIZE, AtomicCoord>;
#[cfg(any(feature = "transport-rtt", feature = "transport-usb"))]
pub type RttQueue = io_kit::Queue<RTT_OUT_QUEUE_SIZE, AtomicCoord>;

#[cfg(feature = "transport-usb")]
pub type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;

// RxWorker types — generic over FrameProcessor
#[cfg(feature = "transport-usb")]
pub type UsbRxWorker = ergot::interface_manager::transports::eusb_0_6::RxWorker<
    &'static Stack,
    AppDriver,
    RouterFrameProcessor,
>;
#[cfg(feature = "transport-rtt")]
pub type RttRxWorker = ergot::interface_manager::transports::eio::RxWorker<
    &'static Stack,
    RttReader,
    RouterFrameProcessor,
>;

// ========== Static Storage ==========

/// Static ownership for the selected network profile.
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

/// Output queues (one per interface)
#[cfg(feature = "transport-usb")]
pub static USB_OUTQ: UsbQueue = usb_kit::Queue::new();
#[cfg(feature = "transport-rtt")]
pub static RTT_OUTQ: RttQueue = io_kit::Queue::new();

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// RTT defmt channel storage
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_DOWN: StaticCell<rtt_target::DownChannel> = StaticCell::new();

// USB wire storage and EP buffer
#[cfg(feature = "transport-usb")]
static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();
#[cfg(feature = "transport-usb")]
static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();

// ========== Interrupt Bindings ==========

#[cfg(feature = "transport-usb")]
bind_interrupts!(struct UsbIrqs {
    OTG_FS =>
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::UsbIrqBegin,
        usb::InterruptHandler<peripherals::USB_OTG_FS>,
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::UsbIrqEnd;
});

bind_interrupts!(pub struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// ========== Initialization ==========

/// Initialize RTT + network for defmt logging (no ergot-over-RTT interface).
/// Returns a DefmtConsumer for forwarding frames over the ergot network.
/// Used when `transport-rtt` is disabled.
#[cfg(not(feature = "transport-rtt"))]
pub fn init_defmt() -> ergot::logging::defmt_sink::DefmtConsumer {
    use ergot::logging::defmt_sink;
    #[cfg(not(feature = "bench-raw-rtt"))]
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
        }
    };
    #[cfg(feature = "bench-raw-rtt")]
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            // Preserve the channel numbering used by the normal RTT transport.
            1: { }
            2: { size: 16384, mode: NoBlockSkip, name: "raw-fast" }
        }
    };
    #[cfg(feature = "bench-raw-rtt")]
    raw_fast::init_rtt(channels.up.2);
    let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
    defmt_sink::init_network_and_rtt(defmt_up)
}

/// RTT transport handles (ergot over RTT channels; defmt stays on channel 0).
#[cfg(feature = "transport-rtt")]
pub struct RttTransport {
    pub rx_worker: RttRxWorker,
    pub tx: RttWriter,
}

/// Initialize RTT: defmt sink (RTT ch0 + network forwarding) **and** an ergot
/// interface on the ergot up/down channels, registered on the Router. Replaces
/// `init_defmt` when `transport-rtt` is enabled (one `rtt_init!` owns the
/// control block, so defmt and ergot channels are declared together). Needs the
/// stack, so call right after `init_stack`.
#[cfg(feature = "transport-rtt")]
pub fn init_rtt(
    stack: &'static Stack,
) -> (ergot::logging::defmt_sink::DefmtConsumer, RttTransport, u8) {
    use ergot::logging::defmt_sink;

    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            // NoBlockTrim, NOT NoBlockSkip (back-ported from the g431 fix,
            // e1f65b5): the ergot TX path hands multi-KB stream grants to
            // this channel; Skip refuses partial writes, returns 0 and
            // re-polls — a hot loop that monopolizes the cooperative
            // executor and starves the other tasks. Trim always makes
            // forward progress into whatever space the host has freed.
            1: { size: 8192, mode: NoBlockTrim, name: "ergot" }
        }
        down: {
            0: { size: 1024, name: "ergot-down" }
        }
    };
    let defmt_consumer = defmt_sink::init_network_and_rtt(RTT_DEFMT_UP.init(channels.up.0));

    let rtt_rx = RttReader::new(RTT_ERGOT_DOWN.init(channels.down.0));
    let rtt_tx = RttWriter::new(RTT_ERGOT_UP.init(channels.up.1));

    let sink = McSink::Rtt(cobs_stream::Sink::new(
        RTT_OUTQ.stream_producer(),
        MAX_PACKET_SIZE as u16,
    ));
    let ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(sink)),
        "RTT interface registration failed"
    );
    let net_id = defmt::unwrap!(stack.manage_profile(|router| router.net_id_of(ident)));

    let rx_worker = RttRxWorker::new(stack, rtt_rx, RouterFrameProcessor::new(net_id), ident)
        .with_liveness(LivenessConfig {
            timeout_ms: LIVENESS_TIMEOUT_MS,
        })
        .with_state_notify(&STATE_NOTIFY);

    (
        defmt_consumer,
        RttTransport {
            rx_worker,
            tx: rtt_tx,
        },
        ident,
    )
}

/// Initialize a SPI leaf, or a standalone diagnostic router.
/// Returns a static reference for use by all tasks.
pub fn init_stack(hw_rng: Rng) -> &'static Stack {
    #[cfg(feature = "bench-netstack-trace")]
    stack_mutex::self_test();
    #[cfg(not(feature = "transport-spi"))]
    let router = McProfile::new(hw_rng);
    #[cfg(feature = "transport-spi")]
    let router = {
        // RNG is only needed to choose this boot's SPI epoch, before init_stack.
        drop(hw_rng);
        McProfile::new_target(framed_stream::Sink::new(
            SPI_OUTQ.framed_producer(),
            crate::config::MAX_PACKET_SIZE as u16,
        ))
    };
    let stack = NetStack::new_with_profile(router);
    STACK_CELL.init(stack)
}

/// USB transport handles
#[cfg(feature = "transport-usb")]
pub struct UsbTransport {
    pub usb_dev: embassy_usb::UsbDevice<'static, AppDriver>,
    pub ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    pub rx_worker: UsbRxWorker,
    #[cfg(feature = "bench-raw-usb")]
    pub raw_ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
}

/// Initialize USB transport and register it on the Router.
/// Returns the USB transport handles and the assigned interface ident.
#[cfg(feature = "transport-usb")]
pub fn init_usb(
    stack: &'static Stack,
    usb_otg_fs: Peri<'static, peripherals::USB_OTG_FS>,
    pa12: Peri<'static, peripherals::PA12>,
    pa11: Peri<'static, peripherals::PA11>,
) -> (UsbTransport, u8) {
    let ep_out_buf = EP_OUT_BUF.init([0u8; 256]);

    let mut usb_cfg = usb::Config::default();
    usb_cfg.vbus_detection = false;

    let driver = usb::Driver::new_fs(usb_otg_fs, UsbIrqs, pa12, pa11, ep_out_buf, usb_cfg);

    let mut usb_dev_cfg = embassy_usb::Config::new(0x16c0, 0x27DD);
    usb_dev_cfg.manufacturer = Some("oxifoc");
    usb_dev_cfg.product = Some("oxifoc-f405");
    #[cfg(feature = "board-cf2")]
    {
        usb_dev_cfg.serial_number = Some("simple-focer2");
    }
    #[cfg(feature = "board-vesc6-mk5")]
    {
        usb_dev_cfg.serial_number = Some("vesc6-mk5");
    }

    #[cfg(not(feature = "bench-raw-usb"))]
    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);
    #[cfg(feature = "bench-raw-usb")]
    let (usb_dev, ep_in, ep_out, raw_ep_in) = {
        // Keep Ergot's interface, identity string and Microsoft descriptors intact.
        let (mut builder, ep_in, ep_out) = USB_STORAGE.init_without_build(driver, usb_dev_cfg);
        let raw_ep_in = raw_fast::add_usb_interface(&mut builder);
        (builder.build(), ep_in, ep_out, raw_ep_in)
    };

    // Register USB interface on Router
    let usb_sink = McSink::Usb(framed_stream::Sink::new(
        USB_OUTQ.framed_producer(),
        MAX_PACKET_SIZE as u16,
    ));
    let usb_ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(usb_sink)),
        "USB interface registration failed"
    );

    let rx_worker = UsbRxWorker::new(
        stack,
        ep_out,
        RouterFrameProcessor::new(defmt::unwrap!(
            stack.manage_profile(|router| router.net_id_of(usb_ident))
        )),
        usb_ident,
    )
    .with_liveness(LivenessConfig { timeout_ms: 3000 })
    .with_state_notify(&STATE_NOTIFY);

    (
        UsbTransport {
            usb_dev,
            ep_in,
            rx_worker,
            #[cfg(feature = "bench-raw-usb")]
            raw_ep_in,
        },
        usb_ident,
    )
}
