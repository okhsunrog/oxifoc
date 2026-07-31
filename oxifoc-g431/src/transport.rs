//! Transport layer abstraction for ergot communication
//!
//! Supports two transport modes:
//! - UART: ergot over USART2 VCP + defmt over RTT with network forwarding
//! - RTT: ergot over RTT channels + defmt over separate RTT channel
//!
//! Device runs as a Router (single interface) so addressing is consistent
//! with multi-interface devices.

// I/O wrappers — re-exported from oxifoc-core
#[cfg(feature = "transport-uart")]
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
use mutex::raw_impls::single_core_thread_mode::ThreadModeRawMutex;
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::OUT_QUEUE_SIZE;
#[cfg(feature = "transport-uart")]
use crate::config::{UART_BAUD, UART_RX_BUF_LEN, UART_TX_BUF_LEN};

// ========== Type Aliases ==========

pub type Rng = rng::Rng<'static, peripherals::RNG>;
type McRouter = Router<IoInterface<QueueRef>, Rng, 1, 0>;
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type QueueRef = &'static Queue;
// ThreadModeRawMutex, NOT CriticalSectionRawMutex: every NetStack access on
// this device is from thread-mode embassy tasks (the ADC1_2/TIM4 ISRs publish
// via FAST_TELEM_Q / atomics, never the stack). A critical-section mutex here
// masked ALL interrupts for the whole postcard+COBS serialization of every
// outgoing packet (~100+ µs per telemetry batch), starving the 20 kHz FOC ISR
// of ~20% of its ADC triggers under load (measured 15.97 kHz effective).
// Thread-mode locking never masks IRQs; a (buggy) ISR-side stack call fails
// as WouldDeadlock instead of corrupting state.
pub type Stack = NetStack<ThreadModeRawMutex, McRouter>;

#[cfg(feature = "transport-uart")]
pub type RxWorker = EioRxWorker<&'static Stack, UartReader, RouterFrameProcessor>;
#[cfg(feature = "transport-rtt")]
pub type RxWorker = EioRxWorker<&'static Stack, MeteredRttReader, RouterFrameProcessor>;

/// Down-pump scheduling stats (RTT transport), 1 Hz-reported by the stats
/// task. `READ_GAP_MAX_US` is the max time between successive data deliveries
/// out of the RTT down channel: under host affirms (50 ms cadence) plus slow
/// telemetry (100 ms) a healthy pump stays under ~110 000; a spike ≥150 000 in a
/// deadman-trip second proves the frames sat in the down buffer while the
/// pump (RxWorker on the thread executor) wasn't being scheduled — the
/// 2026-07-06 drive-engage trips.
#[cfg(feature = "transport-rtt")]
pub mod pump_stats {
    use core::sync::atomic::AtomicU32;
    /// Reads that returned data (per window).
    pub static READS: AtomicU32 = AtomicU32::new(0);
    /// Max µs between successive data deliveries (per window).
    pub static READ_GAP_MAX_US: AtomicU32 = AtomicU32::new(0);
    /// Timestamp (µs, wrapping) of the previous delivery — internal.
    pub static LAST_READ_US: AtomicU32 = AtomicU32::new(0);
}

/// [`RttReader`] wrapped with the [`pump_stats`] meter: stamps every data
/// delivery so the stats task can expose the pump's scheduling latency.
#[cfg(feature = "transport-rtt")]
pub struct MeteredRttReader {
    inner: RttReader,
}

#[cfg(feature = "transport-rtt")]
impl embedded_io_async::ErrorType for MeteredRttReader {
    type Error = <RttReader as embedded_io_async::ErrorType>::Error;
}

#[cfg(feature = "transport-rtt")]
impl embedded_io_async::Read for MeteredRttReader {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use core::sync::atomic::Ordering;
        let n = self.inner.read(buf).await?;
        let now = embassy_time::Instant::now().as_micros() as u32;
        let last = pump_stats::LAST_READ_US.swap(now, Ordering::Relaxed);
        if last != 0 {
            pump_stats::READ_GAP_MAX_US.fetch_max(now.wrapping_sub(last), Ordering::Relaxed);
        }
        pump_stats::READS.fetch_add(1, Ordering::Relaxed);
        Ok(n)
    }
}

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// Output queue
pub static OUTQ: Queue = kit::Queue::new();

/// Stack cell — initialized at runtime (Router needs RNG).
/// In `.ccmdata` (CPU-only CCM, zeroed by main's first block): thread-mode
/// tasks are the only NetStack users, and every byte counts in the 22K
/// SRAM region since the stack→CCM migration.
#[unsafe(link_section = ".ccmdata")]
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
            // 504 (was 1024, stack→CCM migration; 512 → 504 2026-07-07:
            // 8 B ceded to the observer's cached readiness error on the
            // obs-debug-telem build): defmt is low-rate diagnostics, the
            // host polls every few ms; bursts larger than the ring drop
            // (NoBlockSkip) — acceptable.
            0: { size: 504, mode: NoBlockSkip, name: "defmt" }
            // NoBlockTrim, NOT NoBlockSkip: ergot's tx_worker hands multi-KB
            // stream grants to this channel; Skip refuses partial writes,
            // returns 0 and re-polls — a hot loop that monopolizes the
            // cooperative executor and starves the telemetry stream task
            // (FAST_TELEM_Q is only ~5 ms deep at 20 kHz). Trim always makes
            // forward progress into whatever space the host has freed.
            // (8192 would be the SWD-read knee per docs/notes/rtt-telemetry-
            // throughput.md §4.3, but +4 KB of static RAM is exactly what
            // the 32 KB part doesn't have. 3072 was tried in the stack→CCM
            // migration and REGRESSED the 20 kHz Stopped capture to 2.7%
            // loss — 4096 is the floor; re-verify the capture after any
            // change here.)
            1: { size: 4096, mode: NoBlockTrim, name: "ergot" }
        }
        down: {
            // 496 (was 1024, stack→CCM migration; 512 → 496 2026-07-07:
            // 16 B ceded to the observer's readiness lag-compensation
            // state — OUT_QUEUE and the ergot up channel are both at
            // documented floors): down traffic is ~30 small command
            // frames/s (~1 KB/s) — still half a second of buffer.
            0: { size: 496, name: "ergot-down" }
        }
    };

    let defmt_up = RTT_DEFMT_CHANNEL.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);

    defmt::info!("Oxifoc starting - ergot over RTT");

    let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
    let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
    let rtt_rx = MeteredRttReader {
        inner: RttReader::new(ergot_down),
    };
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
