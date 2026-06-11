//! Multi-interface transport layer for ergot communication
//!
//! Both transports are always active (no feature flags):
//! - LPUART1: ergot over ST-LINK VCP (PA2 TX, PA3 RX)
//! - USB FS:  ergot over USB bulk endpoints (PA12 D+, PA11 D-)
//!
//! defmt goes over a separate RTT channel (read via probe-rs).
//!
//! NUCLEO-G474RE connections:
//! - LPUART1 VCP: PA2 (TX), PA3 (RX)
//! - USB FS:      PA12 (D+), PA11 (D-)

pub mod io;
pub use io::*;

use embassy_stm32::{Peri, bind_interrupts, peripherals, rng, usb};
use ergot::NetStack;
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::exports::maitake_sync::WaitQueue;
use ergot::interface_manager::LivenessConfig;
use ergot::interface_manager::interface_impls::embassy_usb::EmbassyInterface;
use ergot::interface_manager::interface_impls::embedded_io::IoInterface;
use ergot::interface_manager::profiles::router::{Router, RouterFrameProcessor};
use ergot::interface_manager::transports::eio::RxWorker as EioRxWorker;
use ergot::interface_manager::transports::eusb_0_6::RxWorker as UsbRxWorker;
use ergot::interface_manager::utils::{cobs_stream, framed_stream};
use ergot::toolkits::embassy_usb_v0_6 as usb_kit;
use ergot::toolkits::embedded_io_async_v0_7 as io_kit;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::{UART_OUT_QUEUE_SIZE, USB_OUT_QUEUE_SIZE};

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

pub type Rng = embassy_stm32::rng::Rng<'static, peripherals::RNG>;
pub type McRouter = Router<McInterface, Rng, 2, 0>;
pub type Stack = NetStack<CriticalSectionRawMutex, McRouter>;

pub type UsbQueue = usb_kit::Queue<USB_OUT_QUEUE_SIZE, AtomicCoord>;
pub type UartQueue = io_kit::Queue<UART_OUT_QUEUE_SIZE, AtomicCoord>;

pub type AppDriver = usb::Driver<'static, peripherals::USB>;

pub type UartRxWorkerType = EioRxWorker<&'static Stack, UartReader, RouterFrameProcessor>;
pub type UsbRxWorkerType = UsbRxWorker<&'static Stack, AppDriver, RouterFrameProcessor>;

// ========== Static Storage ==========

/// Stack cell — initialized at runtime (Router needs RNG)
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

/// Output queues (one per interface)
pub static USB_OUTQ: UsbQueue = usb_kit::Queue::new();
pub static UART_OUTQ: UartQueue = io_kit::Queue::new();

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// RTT defmt channel storage
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

/// UART buffers
static UART_TX_BUF: StaticCell<[u8; crate::config::UART_BUF_LEN]> = StaticCell::new();
static UART_RX_BUF: StaticCell<[u8; crate::config::UART_BUF_LEN]> = StaticCell::new();

/// USB descriptor buffers
static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();

// ========== Interrupt Bindings ==========

bind_interrupts!(struct LpuartIrqs {
    LPUART1 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::LPUART1>;
});

bind_interrupts!(struct UsbIrqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

bind_interrupts!(pub struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// ========== Initialization ==========

/// Initialize RTT for defmt logging. Must be called before any defmt macros.
pub fn init_defmt_rtt() {
    use ergot::logging::defmt_sink;
    let channels = rtt_init! {
        up: {
            0: { size: 2048, mode: NoBlockSkip, name: "defmt" }
        }
    };
    let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);
}

/// Initialize the ergot Router stack with hardware RNG.
/// Returns a static reference for use by all tasks.
pub fn init_stack(hw_rng: Rng) -> &'static Stack {
    let router = McRouter::new(hw_rng);
    STACK_CELL.init(NetStack::new_with_profile(router))
}

// ========== UART Transport ==========

/// UART transport handles
pub struct UartTransport {
    pub rx_worker: UartRxWorkerType,
    pub tx: UartWriter,
}

/// Initialize LPUART1 transport and register it on the Router.
/// Returns the UART transport handles and the assigned interface ident.
pub fn init_uart(
    stack: &'static Stack,
    lpuart1: Peri<'static, peripherals::LPUART1>,
    pa2: Peri<'static, peripherals::PA2>,
    pa3: Peri<'static, peripherals::PA3>,
) -> (UartTransport, u8) {
    use embassy_stm32::usart::{BufferedUart, Config as UartConfig};

    defmt::info!("Oxifoc G474 - initializing ergot over LPUART1 VCP");

    // Configure LPUART1 on ST-LINK VCP (PA2 TX, PA3 RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = crate::config::UART_BAUD;
    let tx_buf = UART_TX_BUF.init([0u8; crate::config::UART_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; crate::config::UART_BUF_LEN]);
    let uart = defmt::unwrap!(
        BufferedUart::new(lpuart1, pa3, pa2, tx_buf, rx_buf, LpuartIrqs, uart_cfg),
        "LPUART1 init failed"
    );
    let (uart_tx, uart_rx) = uart.split();

    // Register UART interface on Router
    let uart_sink = McSink::Uart(cobs_stream::Sink::new(
        UART_OUTQ.stream_producer(),
        crate::config::MAX_PACKET_SIZE as u16,
    ));
    let ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(uart_sink)),
        "UART interface registration failed"
    );
    let net_id = defmt::unwrap!(stack.manage_profile(|router| router.net_id_of(ident)));

    let rx_worker = UartRxWorkerType::new(
        stack,
        UartReader::new(uart_rx),
        RouterFrameProcessor::new(net_id),
        ident,
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
        ident,
    )
}

// ========== USB Transport ==========

/// USB transport handles
pub struct UsbTransport {
    pub usb_dev: embassy_usb::UsbDevice<'static, AppDriver>,
    pub ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    pub rx_worker: UsbRxWorkerType,
}

/// Initialize USB FS transport and register it on the Router.
///
/// G474 has a non-OTG USB peripheral (not USB_OTG_FS). Uses HSI48 (48MHz) as
/// USB clock source — must be enabled in the clock config before calling this.
///
/// Returns the USB transport handles and the assigned interface ident.
pub fn init_usb(
    stack: &'static Stack,
    usb_periph: Peri<'static, peripherals::USB>,
    pa12: Peri<'static, peripherals::PA12>,
    pa11: Peri<'static, peripherals::PA11>,
) -> (UsbTransport, u8) {
    defmt::info!("Oxifoc G474 - initializing ergot over USB FS");

    // Non-OTG USB driver: no ep_out_buf, no Config needed
    let driver = usb::Driver::new(usb_periph, UsbIrqs, pa12, pa11);

    let mut usb_dev_cfg = embassy_usb::Config::new(0x16c0, 0x27DD);
    usb_dev_cfg.manufacturer = Some("oxifoc");
    usb_dev_cfg.product = Some("oxifoc-g474");
    usb_dev_cfg.serial_number = Some("nucleo-g474re");

    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);

    // Register USB interface on Router
    let usb_sink = McSink::Usb(framed_stream::Sink::new(
        USB_OUTQ.framed_producer(),
        crate::config::MAX_PACKET_SIZE as u16,
    ));
    let ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(usb_sink)),
        "USB interface registration failed"
    );
    let net_id = defmt::unwrap!(stack.manage_profile(|router| router.net_id_of(ident)));

    let rx_worker = UsbRxWorkerType::new(stack, ep_out, RouterFrameProcessor::new(net_id), ident)
        .with_liveness(LivenessConfig {
            timeout_ms: crate::config::USB_LIVENESS_TIMEOUT_MS,
        })
        .with_state_notify(&STATE_NOTIFY);

    (
        UsbTransport {
            usb_dev,
            ep_in,
            rx_worker,
        },
        ident,
    )
}
