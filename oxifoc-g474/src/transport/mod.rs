//! Transport layer for ergot communication — RTT-only (throughput bench).
//!
//! The NUCLEO-G474RE normally runs ergot over USB + LPUART VCP. For RTT
//! throughput characterization (measuring what the onboard STLINK-V3E gives
//! over SWD/RTT) this board is built as a single ergot-over-RTT interface,
//! mirroring the g431 `transport-rtt` path:
//! - RTT channel 0 (up):   defmt log sink
//! - RTT channel 1 (up):   ergot frames device → host
//! - RTT channel 0 (down): ergot frames host → device
//!
//! Device runs as a Router (single interface) so addressing matches the
//! multi-interface boards.

pub mod io;
pub use io::*;

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
use ergot::transport::rtt::{RttReader, RttWriter};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::{MAX_PACKET_SIZE, OUT_QUEUE_SIZE};

// ========== Type Aliases ==========

pub type Rng = rng::Rng<'static, peripherals::RNG>;
type McRouter = Router<IoInterface<QueueRef>, Rng, 1, 0>;
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type QueueRef = &'static Queue;
pub type Stack = NetStack<CriticalSectionRawMutex, McRouter>;

pub type RxWorker = EioRxWorker<&'static Stack, RttReader, RouterFrameProcessor>;

/// State notification queue — woken on interface state transitions
pub static STATE_NOTIFY: WaitQueue = WaitQueue::new();

/// Output queue (ergot frames awaiting RTT transmit)
pub static OUTQ: Queue = kit::Queue::new();

/// Stack cell — initialized at runtime (Router needs RNG)
static STACK_CELL: StaticCell<Stack> = StaticCell::new();

// ========== Static Storage ==========

static RTT_DEFMT_CHANNEL: StaticCell<rtt_target::UpChannel> = StaticCell::new();
static RTT_ERGOT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();
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

/// RTT transport handles
pub struct RttTransport {
    pub rx_worker: RxWorker,
    pub tx: RttWriter,
}

/// Initialize ergot-over-RTT (and the defmt RTT sink) and register the
/// interface on the Router. Must be called before any defmt macros.
pub fn init_rtt(stack: &'static Stack) -> (RttTransport, u8) {
    use ergot::logging::defmt_sink;

    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
            1: { size: 16384, mode: NoBlockSkip, name: "ergot" }
        }
        down: {
            0: { size: 1024, name: "ergot-down" }
        }
    };

    let defmt_up = RTT_DEFMT_CHANNEL.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);

    defmt::info!("Oxifoc G474 starting - ergot over RTT (throughput bench)");

    let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
    let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
    let rtt_rx = RttReader::new(ergot_down);
    let rtt_tx = RttWriter::new(ergot_up);

    // Register RTT interface on Router
    let sink = cobs_stream::Sink::new(OUTQ.stream_producer(), MAX_PACKET_SIZE as u16);
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
