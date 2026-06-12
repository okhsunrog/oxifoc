//! Multi-interface transport layer for ergot communication
//!
//! Supports USB bulk endpoints and UART (USART3) simultaneously.
//! The device runs as a Router with both interfaces registered.
//! defmt logs go over a separate RTT channel (read via probe-rs).

pub use oxifoc_core::runtime::io::*;

use embassy_stm32::rng;
use embassy_stm32::usb;
use embassy_stm32::{Peri, bind_interrupts, peripherals};
use ergot::NetStack;
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::exports::maitake_sync::WaitQueue;
use ergot::interface_manager::LivenessConfig;
use ergot::interface_manager::interface_impls::embassy_usb::EmbassyInterface;
use ergot::interface_manager::interface_impls::embedded_io::IoInterface;
use ergot::interface_manager::profiles::router::{Router, RouterFrameProcessor};
use ergot::interface_manager::utils::{cobs_stream, framed_stream};
use ergot::toolkits::embassy_usb_v0_6 as usb_kit;
use ergot::toolkits::embedded_io_async_v0_7 as io_kit;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::{MAX_PACKET_SIZE, UART_BAUD, UART_RX_BUF_LEN, UART_TX_BUF_LEN};
use crate::config::{UART_OUT_QUEUE_SIZE, USB_OUT_QUEUE_SIZE};
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;

// ========== Multi-Interface Definition ==========

type UsbQ = &'static UsbQueue;
type UartQ = &'static UartQueue;

ergot::multi_interface! {
    pub enum McSink for McInterface {
        Usb(EmbassyInterface<UsbQ>),
        Uart(IoInterface<UartQ>),
    }
}

// ========== Type Aliases ==========

pub type Rng = rng::Rng<'static, peripherals::RNG>;
pub type McRouter = Router<McInterface, Rng, 2, 0>;
pub type Stack = NetStack<CriticalSectionRawMutex, McRouter>;

pub type UsbQueue = usb_kit::Queue<USB_OUT_QUEUE_SIZE, AtomicCoord>;
pub type UartQueue = io_kit::Queue<UART_OUT_QUEUE_SIZE, AtomicCoord>;

pub type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;

// RxWorker types — generic over FrameProcessor
pub type UsbRxWorker = ergot::interface_manager::transports::eusb_0_6::RxWorker<
    &'static Stack,
    AppDriver,
    RouterFrameProcessor,
>;
pub type UartRxWorker = ergot::interface_manager::transports::eio::RxWorker<
    &'static Stack,
    UartReader,
    RouterFrameProcessor,
>;

// ========== Static Storage ==========

/// Stack cell — initialized at runtime because Router needs RNG
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

/// Output queues (one per interface)
pub static USB_OUTQ: UsbQueue = usb_kit::Queue::new();
pub static UART_OUTQ: UartQueue = io_kit::Queue::new();

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// RTT defmt channel storage
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

// USB wire storage and EP buffer
static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();
static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();

// UART buffers
static UART_TX_BUF: StaticCell<[u8; UART_TX_BUF_LEN]> = StaticCell::new();
static UART_RX_BUF: StaticCell<[u8; UART_RX_BUF_LEN]> = StaticCell::new();

// ========== Interrupt Bindings ==========

bind_interrupts!(struct UsbIrqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

bind_interrupts!(struct UsartIrqs {
    USART3 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART3>;
});

bind_interrupts!(pub struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// ========== Initialization ==========

/// Initialize RTT + network for defmt logging. Must be called before any defmt macros.
/// Returns a DefmtConsumer for forwarding frames over the ergot network.
pub fn init_defmt() -> ergot::logging::defmt_sink::DefmtConsumer {
    use ergot::logging::defmt_sink;
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
        }
    };
    let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
    defmt_sink::init_network_and_rtt(defmt_up)
}

/// Initialize the ergot Router stack with hardware RNG.
/// Returns a static reference for use by all tasks.
pub fn init_stack(hw_rng: Rng) -> &'static Stack {
    let router = McRouter::new(hw_rng);
    let stack = NetStack::new_with_profile(router);
    STACK_CELL.init(stack)
}

/// USB transport handles
pub struct UsbTransport {
    pub usb_dev: embassy_usb::UsbDevice<'static, AppDriver>,
    pub ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    pub rx_worker: UsbRxWorker,
}

/// Initialize USB transport and register it on the Router.
/// Returns the USB transport handles and the assigned interface ident.
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
    usb_dev_cfg.serial_number = Some("simple-focer2");

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

/// UART transport handles
pub struct UartTransport {
    pub rx_worker: UartRxWorker,
    pub tx: UartWriter,
}

/// Initialize UART transport on USART3 and register it on the Router.
/// Returns the UART transport handles and the assigned interface ident.
pub fn init_uart(
    stack: &'static Stack,
    usart3: Peri<'static, peripherals::USART3>,
    pb10: Peri<'static, peripherals::PB10>,
    pb11: Peri<'static, peripherals::PB11>,
) -> (UartTransport, u8) {
    use embassy_stm32::usart::{Config as UartConfig, Parity, StopBits};

    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = UART_BAUD;
    uart_cfg.parity = Parity::ParityNone;
    uart_cfg.stop_bits = StopBits::STOP1;

    let tx_buf = UART_TX_BUF.init([0u8; UART_TX_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; UART_RX_BUF_LEN]);

    // BufferedUart::new(usart, rx_pin, tx_pin, ...) — PB11 is RX, PB10 is TX
    let uart = defmt::unwrap!(
        embassy_stm32::usart::BufferedUart::new(
            usart3, pb11, pb10, tx_buf, rx_buf, UsartIrqs, uart_cfg,
        ),
        "USART3 init failed"
    );

    let (uart_tx, uart_rx) = uart.split();

    // Register UART interface on Router
    let uart_sink = McSink::Uart(cobs_stream::Sink::new(
        UART_OUTQ.stream_producer(),
        MAX_PACKET_SIZE as u16,
    ));
    let uart_ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(uart_sink)),
        "UART interface registration failed"
    );

    let rx_worker = UartRxWorker::new(
        stack,
        UartReader::new(uart_rx),
        RouterFrameProcessor::new(defmt::unwrap!(
            stack.manage_profile(|router| router.net_id_of(uart_ident))
        )),
        uart_ident,
    )
    .with_liveness(LivenessConfig {
        timeout_ms: LIVENESS_TIMEOUT_MS,
    })
    .with_state_notify(&STATE_NOTIFY);

    (
        UartTransport {
            rx_worker,
            tx: UartWriter::new(uart_tx),
        },
        uart_ident,
    )
}
