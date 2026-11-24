//! MK5 SPI DirectEdge and standalone diagnostic USB router profiles.
//! defmt diagnostics retain a separate RTT channel (read via probe-rs).

#[cfg(feature = "transport-usb")]
use embassy_stm32::{Peri, usb};
use embassy_stm32::{bind_interrupts, peripherals, rng};
use ergot::NetStack;
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::exports::maitake_sync::WaitQueue;
#[cfg(feature = "transport-usb")]
use ergot::interface_manager::LivenessConfig;
#[cfg(feature = "transport-usb")]
use ergot::interface_manager::interface_impls::embassy_usb::EmbassyInterface;
#[cfg(feature = "transport-spi")]
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
#[cfg(not(feature = "transport-spi"))]
use ergot::interface_manager::profiles::router::Router;
#[cfg(feature = "transport-usb")]
use ergot::interface_manager::profiles::router::RouterFrameProcessor;
use ergot::interface_manager::utils::framed_stream;
#[cfg(feature = "transport-usb")]
use ergot::toolkits::embassy_usb_v0_7_preview as usb_kit;
use ergot::toolkits::embedded_io_async_v0_7 as io_kit;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

#[cfg(feature = "transport-usb")]
use crate::config::MAX_PACKET_SIZE;
#[cfg(feature = "transport-usb")]
use crate::config::USB_OUT_QUEUE_SIZE;
use oxifoc_core::stm32_rng::ErgotRng;

mod stack_mutex;

pub mod raw_fast;

// A leaf has exactly one upstream. USB remains a standalone diagnostic
// builds, rather than accidentally introducing a second root on the SPI link.
#[cfg(all(feature = "transport-spi", feature = "transport-usb"))]
compile_error!("SPI DirectEdge cannot be combined with diagnostic USB routing");

// ========== Multi-Interface Definition ==========
#[cfg(feature = "transport-usb")]
type UsbQ = &'static UsbQueue;

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
    }
}
// No tagged multi-interface dispatch in SPI-only firmware. The leaf's sink
// is the SPI framed queue itself.
#[cfg(not(feature = "transport-usb"))]
pub type McInterface = SpiInterface;

// ========== Type Aliases ==========

pub type Rng = ErgotRng;
/// Standalone diagnostic router capacity. Production SPI is a single leaf.
#[cfg(not(feature = "transport-spi"))]
const INTERFACES: usize = 1 + cfg!(feature = "transport-usb") as usize;
#[cfg(not(feature = "transport-spi"))]
pub type McProfile = Router<McInterface, Rng, INTERFACES, 8>;
#[cfg(feature = "transport-spi")]
pub type McProfile = DirectEdge<McInterface>;
pub type InterfaceIdent = <McProfile as ergot::interface_manager::Profile>::InterfaceIdent;
pub type Stack = NetStack<stack_mutex::StackRawMutex, McProfile>;

#[cfg(feature = "transport-usb")]
pub type UsbQueue = usb_kit::Queue<USB_OUT_QUEUE_SIZE, AtomicCoord>;

#[cfg(feature = "transport-usb")]
pub type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;

// RxWorker types — generic over FrameProcessor
#[cfg(feature = "transport-usb")]
pub type UsbRxWorker = ergot::interface_manager::transports::eusb_0_6::RxWorker<
    &'static Stack,
    AppDriver,
    RouterFrameProcessor,
>;

// ========== Static Storage ==========

/// Static ownership for the selected network profile.
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

/// Output queues (one per interface)
#[cfg(feature = "transport-usb")]
pub static USB_OUTQ: UsbQueue = usb_kit::Queue::new();

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

// USB wire storage and EP buffer
#[cfg(feature = "transport-usb")]
static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();
#[cfg(feature = "transport-usb")]
static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();

// ========== Interrupt Bindings ==========

#[cfg(feature = "transport-usb")]
bind_interrupts!(struct UsbIrqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

bind_interrupts!(pub struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// ========== Initialization ==========

/// Initialize the RTT-only defmt logger.
pub fn init_defmt() {
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
        }
    };
    crate::logging::init(channels.up.0);
}

/// Initialize a SPI leaf, or a standalone diagnostic router.
/// Returns a static reference for use by all tasks.
pub fn init_stack(hw_rng: Rng) -> &'static Stack {
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

    let driver = usb::Driver::new_fs(usb_otg_fs, pa12, pa11, UsbIrqs, ep_out_buf, usb_cfg);

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

    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);

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
        },
        usb_ident,
    )
}
