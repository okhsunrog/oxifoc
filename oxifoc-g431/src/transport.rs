//! Transport layer abstraction for ergot communication
//!
//! Supports two transport modes:
//! - UART: ergot over USART2 VCP + defmt over RTT with network forwarding
//! - RTT: ergot over RTT channels + defmt over separate RTT channel
//!
//! Device runs as a Router (single interface) so addressing is consistent
//! with multi-interface devices.

// I/O wrappers — re-exported from oxifoc-core
pub use oxifoc_core::runtime::io::*;

use embassy_stm32::{bind_interrupts, peripherals, rng};
use ergot::NetStack;
use ergot::exports::bbqueue::traits::coordination::cas::AtomicCoord;
use ergot::exports::maitake_sync::WaitQueue;
use ergot::interface_manager::LivenessConfig;
use ergot::interface_manager::interface_impls::embedded_io::IoInterface;
use ergot::interface_manager::profiles::router::{Router, RouterFrameProcessor};
use ergot::interface_manager::transports::eio::RxWorker as EioRxWorker;
use ergot::interface_manager::utils::cobs_stream;
use ergot::toolkits::embedded_io_async_v0_7 as kit;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::OUT_QUEUE_SIZE;
#[cfg(feature = "transport-uart")]
use crate::config::{UART_BAUD, UART_RX_BUF_LEN, UART_TX_BUF_LEN};

// ========== Type Aliases ==========

pub type Rng = embassy_stm32::rng::Rng<'static, peripherals::RNG>;
type McRouter = Router<IoInterface<QueueRef>, Rng, 1, 0>;
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type QueueRef = &'static Queue;
pub type Stack = NetStack<CriticalSectionRawMutex, McRouter>;

#[cfg(feature = "transport-uart")]
pub type RxWorker = EioRxWorker<&'static Stack, UartReader, RouterFrameProcessor>;
#[cfg(feature = "transport-rtt")]
pub type RxWorker = EioRxWorker<&'static Stack, RttReader, RouterFrameProcessor>;

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// Output queue
pub static OUTQ: Queue = kit::Queue::new();

/// Stack cell — initialized at runtime (Router needs RNG)
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

// ========== Static Storage ==========

#[cfg(feature = "transport-uart")]
static UART_TX_BUF: StaticCell<[u8; UART_TX_BUF_LEN]> = StaticCell::new();
#[cfg(feature = "transport-uart")]
static UART_RX_BUF: StaticCell<[u8; UART_RX_BUF_LEN]> = StaticCell::new();

#[cfg(feature = "transport-uart")]
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

#[cfg(feature = "transport-rtt")]
static RTT_DEFMT_CHANNEL: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_DOWN: StaticCell<rtt_target::DownChannel> = StaticCell::new();

// ========== Interrupt Bindings ==========

bind_interrupts!(pub struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// ========== Initialization ==========

/// Initialize the ergot Router stack with hardware RNG.
pub fn init_stack(hw_rng: Rng) -> &'static Stack {
    let router = McRouter::new(hw_rng);
    STACK_CELL.init(NetStack::new_with_profile(router))
}

// ========== UART Transport (feature = "transport-uart") ==========

#[cfg(feature = "transport-uart")]
bind_interrupts!(struct UsartIrqs {
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
    usart2: embassy_stm32::Peri<'static, peripherals::USART2>,
    pb4: embassy_stm32::Peri<'static, peripherals::PB4>,
    pb3: embassy_stm32::Peri<'static, peripherals::PB3>,
) -> (UartTransport, u8) {
    use embassy_stm32::usart::{BufferedUart, Config as UartConfig, Parity, StopBits};
    use ergot::logging::defmt_sink;

    // Initialize RTT for defmt
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

    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = UART_BAUD;
    uart_cfg.parity = Parity::ParityNone;
    uart_cfg.stop_bits = StopBits::STOP1;
    let tx_buf = UART_TX_BUF.init([0u8; UART_TX_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; UART_RX_BUF_LEN]);
    let uart = defmt::unwrap!(
        BufferedUart::new(usart2, pb4, pb3, tx_buf, rx_buf, UsartIrqs, uart_cfg),
        "USART2 init failed"
    );
    let (uart_tx, uart_rx) = uart.split();

    // Register UART interface on Router
    let sink = cobs_stream::Sink::new(
        OUTQ.stream_producer(),
        crate::config::MAX_PACKET_SIZE as u16,
    );
    let ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(sink)),
        "UART interface registration failed"
    );
    let net_id = defmt::unwrap!(stack.manage_profile(|router| router.net_id_of(ident)));

    let rx_worker = RxWorker::new(
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

// ========== RTT Transport (feature = "transport-rtt") ==========

#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::{RttReader, RttWriter};

#[cfg(feature = "transport-rtt")]
pub struct RttTransport {
    pub rx_worker: RxWorker,
    pub tx: RttWriter,
}

#[cfg(feature = "transport-rtt")]
pub fn init_rtt(stack: &'static Stack) -> (RttTransport, u8) {
    use ergot::logging::defmt_sink;

    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            1: { size: 2048, mode: NoBlockSkip, name: "ergot" }
        }
        down: {
            0: { size: 1024, name: "ergot-down" }
        }
    };

    let defmt_up = RTT_DEFMT_CHANNEL.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);

    defmt::info!("Oxifoc starting - ergot over RTT");

    let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
    let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
    let rtt_rx = RttReader::new(ergot_down);
    let rtt_tx = RttWriter::new(ergot_up);

    // Register RTT interface on Router
    let sink = cobs_stream::Sink::new(
        OUTQ.stream_producer(),
        crate::config::MAX_PACKET_SIZE as u16,
    );
    let ident = defmt::unwrap!(
        stack.manage_profile(|router| router.register_interface(sink)),
        "RTT interface registration failed"
    );
    let net_id = defmt::unwrap!(stack.manage_profile(|router| router.net_id_of(ident)));

    let rx_worker = RxWorker::new(stack, rtt_rx, RouterFrameProcessor::new(net_id), ident)
        .with_liveness(LivenessConfig {
            timeout_ms: LIVENESS_TIMEOUT_MS,
        })
        .with_state_notify(&STATE_NOTIFY);

    (
        RttTransport {
            rx_worker,
            tx: rtt_tx,
        },
        ident,
    )
}
