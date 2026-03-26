//! Transport layer abstraction for ergot communication
//!
//! Supports two transport modes:
//! - UART: ergot over USART2 VCP + defmt over RTT with network forwarding
//! - RTT: ergot over RTT channels + defmt over separate RTT channel

// I/O wrappers — re-exported from oxifoc-core
pub use oxifoc_core::runtime::io::*;

use ergot::exports::maitake_sync::WaitQueue;
use ergot::interface_manager::LivenessConfig;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

#[cfg(feature = "transport-uart")]
use embassy_stm32::{
    Peri, bind_interrupts, peripherals,
    usart::{BufferedUart, Config as UartConfig, Parity, StopBits},
};
#[cfg(feature = "transport-uart")]
use ergot::logging::defmt_sink;
// UartReader and UartWriter are already available via `pub use oxifoc_core::runtime::io::*` above

#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::{RttReader, RttWriter};

use crate::config::OUT_QUEUE_SIZE;
#[cfg(feature = "transport-uart")]
use crate::config::{UART_BAUD, UART_RX_BUF_LEN, UART_TX_BUF_LEN};
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::toolkits::embedded_io_async_v0_7 as kit;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

// Type aliases for our application
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
pub type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;

#[cfg(feature = "transport-uart")]
pub type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, UartReader>;
#[cfg(feature = "transport-rtt")]
pub type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, RttReader>;

// ========== Static Storage ==========

/// UART buffers (only for UART transport)
#[cfg(feature = "transport-uart")]
static UART_TX_BUF: StaticCell<[u8; UART_TX_BUF_LEN]> = StaticCell::new();
#[cfg(feature = "transport-uart")]
static UART_RX_BUF: StaticCell<[u8; UART_RX_BUF_LEN]> = StaticCell::new();

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

// ========== UART Transport (feature = "transport-uart") ==========

#[cfg(feature = "transport-uart")]
bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[cfg(feature = "transport-uart")]
pub struct UartTransport {
    pub rx_worker: RxWorker,
    pub tx: UartWriter,
}

#[cfg(feature = "transport-uart")]
pub fn init_uart(
    stack: &'static Stack,
    usart2: Peri<'static, peripherals::USART2>,
    pb4: Peri<'static, peripherals::PB4>,
    pb3: Peri<'static, peripherals::PB3>,
) -> UartTransport {
    // Initialize RTT for defmt (RTT only — no network forwarding)
    {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            }
        };
        let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
        defmt_sink::init_rtt(defmt_up);
    };

    defmt::info!("Oxifoc starting - ergot over USART2 VCP + defmt sink");

    // Configure USART2 on ST-LINK VCP (PB3 TX, PB4 RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = UART_BAUD;
    uart_cfg.parity = Parity::ParityNone;
    uart_cfg.stop_bits = StopBits::STOP1;
    let tx_buf = UART_TX_BUF.init([0u8; UART_TX_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; UART_RX_BUF_LEN]);
    let uart = BufferedUart::new(usart2, pb4, pb3, tx_buf, rx_buf, Irqs, uart_cfg)
        .expect("USART2 init failed");
    let (uart_tx, uart_rx) = uart.split();
    let rx_worker = RxWorker::new_target(stack, UartReader::new(uart_rx), ())
        .with_liveness(LivenessConfig {
            timeout_ms: LIVENESS_TIMEOUT_MS,
        })
        .with_state_notify(&STATE_NOTIFY);

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

    defmt::info!("Oxifoc starting - ergot over RTT");

    // Store ergot RTT channels
    let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
    let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
    let rtt_rx = RttReader::new(ergot_down);
    let rtt_tx = RttWriter::new(ergot_up);

    let rx_worker = RxWorker::new_target(stack, rtt_rx, ())
        .with_liveness(LivenessConfig {
            timeout_ms: LIVENESS_TIMEOUT_MS,
        })
        .with_state_notify(&STATE_NOTIFY);

    RttTransport {
        rx_worker,
        tx: rtt_tx,
    }
}
