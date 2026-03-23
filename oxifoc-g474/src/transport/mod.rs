//! Transport layer abstraction for ergot communication
//!
//! Supports three transport modes selected at compile time via feature flags:
//! - `transport-uart`: ergot over LPUART1 VCP + defmt over RTT with network forwarding
//! - `transport-rtt`:  ergot over RTT channels + defmt over separate RTT channel
//! - `transport-usb`:  ergot over USB FS bulk endpoints + defmt over RTT
//!
//! NUCLEO-G474RE connections:
//! - LPUART1 VCP: PA2 (TX), PA3 (RX)
//! - USB FS:      PA12 (D+), PA11 (D-)

pub mod io;

use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use mutex::raw_impls::cs::CriticalSectionRawMutex;

use crate::config::OUT_QUEUE_SIZE;

// ========== Transport-specific imports ==========

#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
use ergot::toolkits::embedded_io_async_v0_7 as kit;

#[cfg(feature = "transport-uart")]
use embassy_stm32::{
    Peri, bind_interrupts, peripherals,
    usart::{BufferedUart, Config as UartConfig},
};
#[cfg(feature = "transport-uart")]
use ergot::logging::defmt_sink;
#[cfg(feature = "transport-uart")]
pub use io::{UartReader, UartWriter};

#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::{RttReader, RttWriter};

#[cfg(feature = "transport-usb")]
use embassy_stm32::{Peri, bind_interrupts, peripherals, usb};
#[cfg(feature = "transport-usb")]
use ergot::toolkits::embassy_usb_v0_6 as usb_kit;

// ========== Queue / Stack / RxWorker type aliases ==========
// Exactly one branch is active at a time (compile_error in main.rs enforces this).

#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
pub type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;

#[cfg(feature = "transport-uart")]
pub type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, UartReader>;
#[cfg(feature = "transport-rtt")]
pub type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, RttReader>;

#[cfg(feature = "transport-usb")]
pub type Queue = usb_kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
#[cfg(feature = "transport-usb")]
pub type Stack = usb_kit::Stack<&'static Queue, CriticalSectionRawMutex>;
#[cfg(feature = "transport-usb")]
pub type AppDriver = usb::Driver<'static, peripherals::USB>;
#[cfg(feature = "transport-usb")]
pub type RxWorker = usb_kit::RxWorker<&'static Queue, CriticalSectionRawMutex, AppDriver>;

// ========== Static Storage ==========

/// UART buffers (only for UART transport)
#[cfg(feature = "transport-uart")]
static UART_TX_BUF: StaticCell<[u8; crate::config::UART_BUF_LEN]> = StaticCell::new();
#[cfg(feature = "transport-uart")]
static UART_RX_BUF: StaticCell<[u8; crate::config::UART_BUF_LEN]> = StaticCell::new();

/// RTT defmt channel storage (for UART mode - hybrid defmt sink)
#[cfg(feature = "transport-uart")]
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

/// RTT channel storage (for RTT transport mode)
#[cfg(feature = "transport-rtt")]
static RTT_DEFMT_CHANNEL: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_DOWN: StaticCell<rtt_target::DownChannel> = StaticCell::new();

/// USB descriptor buffers (only for USB transport)
#[cfg(feature = "transport-usb")]
static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();
/// RTT defmt channel storage for USB mode
#[cfg(feature = "transport-usb")]
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

// ========== UART Transport (feature = "transport-uart") ==========

#[cfg(feature = "transport-uart")]
bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::LPUART1>;
});

#[cfg(feature = "transport-uart")]
pub struct UartTransport {
    pub rx_worker: RxWorker,
    pub tx: UartWriter,
}

#[cfg(feature = "transport-uart")]
pub fn init_uart(
    stack: &'static Stack,
    lpuart1: Peri<'static, peripherals::LPUART1>,
    pa2: Peri<'static, peripherals::PA2>,
    pa3: Peri<'static, peripherals::PA3>,
) -> UartTransport {
    // Initialize RTT for defmt (hybrid: RTT + network forwarding)
    let _defmt_consumer = {
        let channels = rtt_init! {
            up: {
                0: { size: 2048, mode: NoBlockSkip, name: "defmt" }
            }
        };
        let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
        defmt_sink::init_network_and_rtt(defmt_up)
    };

    defmt::info!("Oxifoc G474 starting - ergot over LPUART1 VCP + defmt sink");

    // Configure LPUART1 on ST-LINK VCP (PA2 TX, PA3 RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = crate::config::UART_BAUD;
    let tx_buf = UART_TX_BUF.init([0u8; crate::config::UART_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; crate::config::UART_BUF_LEN]);
    let uart = BufferedUart::new(lpuart1, pa3, pa2, tx_buf, rx_buf, Irqs, uart_cfg)
        .expect("LPUART1 init failed");
    let (uart_tx, uart_rx) = uart.split();
    let rx_worker = RxWorker::new_target(stack, UartReader::new(uart_rx), ());

    UartTransport {
        rx_worker,
        tx: UartWriter::new(uart_tx),
    }
}

// ========== RTT Transport (feature = "transport-rtt") ==========

#[cfg(feature = "transport-rtt")]
pub struct RttTransport {
    pub rx_worker: RxWorker,
    pub tx: RttWriter,
}

#[cfg(feature = "transport-rtt")]
pub fn init_rtt(stack: &'static Stack) -> RttTransport {
    use ergot::logging::defmt_sink;

    // Initialize RTT channels
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            1: { size: 2048, mode: NoBlockSkip, name: "ergot" }
        }
        down: {
            0: { size: 1024, name: "ergot-down" }
        }
    };

    // Initialize defmt sink (RTT only, no network forwarding)
    let defmt_up = RTT_DEFMT_CHANNEL.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);

    defmt::info!("Oxifoc G474 starting - ergot over RTT");

    // Store ergot RTT channels
    let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
    let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
    let rtt_rx = RttReader::new(ergot_down);
    let rtt_tx = RttWriter::new(ergot_up);

    let rx_worker = RxWorker::new_target(stack, rtt_rx, ());

    RttTransport {
        rx_worker,
        tx: rtt_tx,
    }
}

// ========== USB Transport (feature = "transport-usb") ==========

#[cfg(feature = "transport-usb")]
bind_interrupts!(struct UsbIrqs {
    USB_LP => usb::InterruptHandler<peripherals::USB>;
});

#[cfg(feature = "transport-usb")]
pub struct UsbTransport {
    pub usb_dev: embassy_usb::UsbDevice<'static, AppDriver>,
    pub ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    pub rx_worker: RxWorker,
}

/// Initialize USB FS transport for ergot communication.
///
/// G474 has a non-OTG USB peripheral (not USB_OTG_FS). Uses HSI48 (48MHz) as
/// USB clock source — must be enabled in the clock config before calling this.
#[cfg(feature = "transport-usb")]
pub fn init_usb(
    stack: &'static Stack,
    usb_periph: Peri<'static, peripherals::USB>,
    pa12: Peri<'static, peripherals::PA12>,
    pa11: Peri<'static, peripherals::PA11>,
) -> UsbTransport {
    use ergot::logging::defmt_sink;

    // Initialize RTT for defmt (USB mode: RTT only, no network forwarding needed
    // since defmt-sink-rtt is always active in ergot deps)
    let channels = rtt_init! {
        up: {
            0: { size: 2048, mode: NoBlockSkip, name: "defmt" }
        }
    };
    let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);

    defmt::info!("Oxifoc G474 starting - ergot over USB FS + defmt over RTT");

    // Non-OTG USB driver: no ep_out_buf, no Config needed
    let driver = usb::Driver::new(usb_periph, UsbIrqs, pa12, pa11);

    let mut usb_dev_cfg = embassy_usb::Config::new(0x16c0, 0x27DD);
    usb_dev_cfg.manufacturer = Some("oxifoc");
    usb_dev_cfg.product = Some("oxifoc-g474");
    usb_dev_cfg.serial_number = Some("nucleo-g474re");

    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);
    let rx_worker = usb_kit::RxWorker::new(stack, ep_out);

    UsbTransport {
        usb_dev,
        ep_in,
        rx_worker,
    }
}
