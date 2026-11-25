//! The CAN FD segment between bridges: ergot's CAN transport on the C5's
//! controller.
//!
//! One bridge is the segment's router. It has no upstream, owns the net
//! numbers and grants the node addresses of the bus. Any other bridge has
//! the bus as its upstream ([`UPSTREAM`]): it claims a node address there
//! and leases from the router the nets of its own segments.
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::{Duration, Instant, Timer, with_timeout};
use ergot::{
    exports::bbqueue::traits::coordination::cas::AtomicCoord,
    interface_manager::{
        FrameProcessor, InterfaceState, Profile, SeedLease,
        profiles::{
            direct_edge::EdgeFrameProcessor,
            router::{RouterFrameProcessor, UPSTREAM_IDENT},
        },
        transports::can::{CanConfig, CanError, CanErrorKind, CanRx, CanRxTxWorker, CanTx},
        utils::can::{CanFrame, CanId, Sink},
    },
    net_stack::services::{
        NodeClaimLease, bridge_seed_assign, bridge_seed_refresh, bus_claim_refresh,
        bus_claim_with_retry,
    },
    toolkits::embedded_io_async_v0_7 as kit,
};
use esp_hal::{
    Async,
    canfd::{
        CanFd, CanFdRx, CanFdTx, Config, Error, ErrorState, ExtendedId, Frame, Id, Timing,
        TxBufferState,
    },
    gpio::AnyPin,
    peripherals::TWAI0,
};
use static_cell::StaticCell;

use super::{STATE_NOTIFY, Stack};

/// The bus is this bridge's upstream, not a segment it routes. Set when the
/// image is built, with `OXIFOC_BRIDGE_UPSTREAM=can`.
pub const UPSTREAM: bool =
    matches!(option_env!("OXIFOC_BRIDGE_UPSTREAM"), Some(upstream) if is_can(upstream));

const fn is_can(upstream: &str) -> bool {
    matches!(upstream.as_bytes(), b"can")
}

/// The largest ergot frame the bus carries, and the size of one reassembly
/// buffer: a chunk of a firmware update fits, a block of fast telemetry
/// does not. The root has little RAM to spare beside its radio.
pub const MTU: usize = 512;
/// A CAN FD frame's payload.
pub const MAX_PAYLOAD: u8 = 64;
/// Messages assembled at once.
const SLOTS: usize = 2;
/// Node addresses the router of the bus grants.
pub const CLAIMS: usize = 8;

/// One of the two queues of outgoing frames. A frame needs `MTU + 5`
/// contiguous bytes, which a queue can only promise at twice that.
pub type Queue = kit::Queue<1536, AtomicCoord>;
// The queues and the worker live in RTC RAM, to leave stack room for BLE
// pairing, as the TCP port's receive buffers do.
#[esp_hal::ram(unstable(rtc_fast))]
pub static CONTROL: Queue = kit::Queue::new();
#[esp_hal::ram(unstable(rtc_fast))]
pub static BULK: Queue = kit::Queue::new();

/// The sink of the bus. A stack has one.
pub fn sink() -> Sink<&'static Queue, MTU> {
    Sink::new(&CONTROL, &BULK, MAX_PAYLOAD)
}

#[unsafe(no_mangle)]
pub static CAN_RX_FRAMES: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static CAN_RX_OVERRUNS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static CAN_TX_FRAMES: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static CAN_TX_FAILED: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static CAN_BUS_OFFS: AtomicU32 = AtomicU32::new(0);

fn count(counter: &AtomicU32) {
    counter.fetch_add(1, Ordering::Relaxed);
}

/// The controller and its two pins at the transceiver.
pub struct Resources {
    pub controller: TWAI0<'static>,
    pub txd: AnyPin<'static>,
    pub rxd: AnyPin<'static>,
}

/// 1 Mbit/s and 5 Mbit/s from the controller's 80 MHz, as measured between
/// two boards (docs/hw/xiao-esp32c5.md): 16 clock periods a data bit,
/// sampled at 75 %, and the secondary sample point 12 periods after the
/// transceiver's delay.
fn config() -> Config {
    Config::default()
        .with_nominal_timing(Timing {
            baud_rate_prescaler: 5,
            propagation_segment: 7,
            phase_segment_1: 5,
            phase_segment_2: 3,
            sync_jump_width: 3,
        })
        .with_fd_timing(Timing {
            baud_rate_prescaler: 1,
            propagation_segment: 6,
            phase_segment_1: 5,
            phase_segment_2: 4,
            sync_jump_width: 4,
        })
        .with_secondary_sample_point_offset(12)
}

/// Why a frame was not sent.
#[derive(Debug)]
pub enum Fault {
    Controller(Error),
    /// The controller was off the bus and has been asked to rejoin.
    BusOff,
    /// Not a frame the controller takes.
    Frame,
}

impl CanError for Fault {
    fn kind(&self) -> CanErrorKind {
        match self {
            Self::Controller(Error::ControllerStopped) => CanErrorKind::Stopped,
            Self::Controller(_) | Self::BusOff | Self::Frame => CanErrorKind::TxFailed,
        }
    }
}

struct Tx {
    half: CanFdTx<'static, Async>,
}

/// Takes back a frame the controller still holds when its send is given up.
struct Armed<'a> {
    half: &'a mut CanFdTx<'static, Async>,
    done: bool,
}

impl Drop for Armed<'_> {
    fn drop(&mut self) {
        if self.done {
            return;
        }
        for buffer in 0..self.half.tx_buffer_count() {
            if matches!(
                self.half.tx_buffer_state(buffer),
                TxBufferState::Ready | TxBufferState::InProgress
            ) {
                self.half.abort_transmit(buffer);
            }
        }
    }
}

impl CanTx for Tx {
    type Error = Fault;

    fn max_payload(&self) -> u8 {
        MAX_PAYLOAD
    }

    /// One frame is with the controller at a time, so frames leave in the
    /// order they are handed over: the controller orders its buffers by
    /// index, not by age.
    async fn send(&mut self, frame: &CanFrame) -> Result<(), Fault> {
        if self.half.error_state() == ErrorState::BusOff {
            // The controller rejoins only when asked.
            self.half.request_bus_off_recovery();
            count(&CAN_BUS_OFFS);
            return Err(Fault::BusOff);
        }
        let id = ExtendedId::new(frame.id.raw()).ok_or(Fault::Frame)?;
        let out = Frame::new_fd(id, frame.payload())
            .map_err(|_| Fault::Frame)?
            .with_bit_rate_switch(true);
        let mut armed = Armed {
            half: &mut self.half,
            done: false,
        };
        let sent = armed.half.transmit_async(&out).await;
        armed.done = true;
        match sent {
            Ok(()) => {
                count(&CAN_TX_FRAMES);
                Ok(())
            }
            Err(error) => {
                count(&CAN_TX_FAILED);
                Err(Fault::Controller(error))
            }
        }
    }
}

struct Rx {
    half: CanFdRx<'static, Async>,
}

/// The controller's RX buffer was full and frames were lost.
#[derive(Debug)]
pub struct Overrun;

impl CanError for Overrun {
    fn kind(&self) -> CanErrorKind {
        CanErrorKind::Overrun
    }
}

impl CanRx for Rx {
    type Error = Overrun;

    /// The worker filters by node itself: this half cannot change the
    /// controller's acceptance filters (esp-hal #6437).
    async fn recv(&mut self) -> Result<CanFrame, Overrun> {
        loop {
            // What the buffer still holds came before the frames that were
            // lost. It goes too, so the worker starts clean behind the gap.
            if self.half.rx_overrun() {
                self.half.flush_rx();
                self.half.clear_rx_overrun();
                count(&CAN_RX_OVERRUNS);
                return Err(Overrun);
            }
            let frame = self.half.receive_async().await;
            // Anything else on the bus is not ergot's.
            let Id::Extended(id) = frame.id() else {
                continue;
            };
            let Some(id) = CanId::from_raw(id.as_raw()) else {
                continue;
            };
            if let Some(frame) = CanFrame::new(id, frame.payload()) {
                count(&CAN_RX_FRAMES);
                return Ok(frame);
            }
        }
    }
}

/// The frames of the bus go to the router's processor on the segment it
/// routes, and to an edge's where the bus is the upstream.
enum Processor {
    Segment(RouterFrameProcessor),
    Upstream(EdgeFrameProcessor),
}

impl FrameProcessor<&'static Stack> for Processor {
    fn process_frame(&mut self, data: &[u8], nsh: &&'static Stack, ident: u8) -> bool {
        match self {
            Self::Segment(processor) => processor.process_frame(data, nsh, ident),
            Self::Upstream(processor) => processor.process_frame(data, nsh, ident),
        }
    }

    fn reset(&mut self) {
        match self {
            Self::Segment(processor) => {
                <RouterFrameProcessor as FrameProcessor<&'static Stack>>::reset(processor);
            }
            Self::Upstream(processor) => {
                <EdgeFrameProcessor as FrameProcessor<&'static Stack>>::reset(processor);
            }
        }
    }
}

type Worker = CanRxTxWorker<&'static Stack, Rx, Tx, &'static Queue, Processor, SLOTS, MTU>;

/// Runs the bus: for the router, the segment `segment` it registered; for a
/// bridge with the bus as upstream, from the candidate address `candidate`.
#[embassy_executor::task]
pub async fn link(
    stack: &'static Stack,
    resources: Resources,
    segment: Option<u8>,
    candidate: u8,
    tid_seed: u8,
) {
    let can = match CanFd::new(resources.controller, config()) {
        Ok(can) => can
            .with_rx(resources.rxd)
            .with_tx(resources.txd)
            .into_async(),
        Err(error) => {
            log::error!("CAN: configuration refused: {error:?}");
            return;
        }
    };
    static CAN: StaticCell<CanFd<'static, Async>> = StaticCell::new();
    let can = CAN.init(can);
    if let Err(error) = can.start() {
        log::error!("CAN: the controller did not start: {error:?}");
        return;
    }
    let (rx, tx) = can.split();
    let (processor, ident, state) = match segment {
        Some(ident) => {
            let net = stack.manage_profile(|p| p.net_id_of(ident)).unwrap_or(0);
            let state = stack
                .manage_profile(|p| p.interface_state(ident))
                .unwrap_or(InterfaceState::Down);
            (
                Processor::Segment(RouterFrameProcessor::new(net)),
                ident,
                state,
            )
        }
        None => (
            Processor::Upstream(EdgeFrameProcessor::new()),
            UPSTREAM_IDENT,
            InterfaceState::link_local(candidate),
        ),
    };
    // The worker holds the buffers messages are assembled in.
    #[esp_hal::ram(unstable(rtc_fast))]
    static WORKER: StaticCell<Worker> = StaticCell::new();
    let worker = WORKER.init_with(|| {
        CanRxTxWorker::new(
            stack,
            Rx { half: rx },
            Tx { half: tx },
            processor,
            ident,
            CONTROL.framed_consumer(),
            BULK.framed_consumer(),
            CanConfig::new(tid_seed),
        )
        .with_state_notify(&STATE_NOTIFY)
    });
    let stopped = worker.run(state).await;
    log::error!("CAN: the link stopped: {stopped:?}");
}

/// How long a claim or a lease request may take.
const ANSWER: Duration = Duration::from_secs(2);
/// How long to wait before the router is asked again after it refused or
/// did not answer.
const RETRY: Duration = Duration::from_secs(2);
/// A lease is renewed this long after the router first takes a renewal, and
/// this long before it runs out at the latest.
const MARGIN: u16 = 5;

/// When to renew a lease just granted or renewed. The router takes a renewal
/// once less than `min_refresh_seconds` are left of it.
fn renewal(expires_seconds: u16, min_refresh_seconds: u16) -> Instant {
    let wait = expires_seconds
        .saturating_sub(min_refresh_seconds)
        .saturating_add(MARGIN)
        .min(expires_seconds.saturating_sub(MARGIN))
        .max(1);
    Instant::now() + Duration::from_secs(u64::from(wait))
}

/// The net leased for one segment of this bridge.
struct Leased {
    segment: u8,
    lease: SeedLease,
    renew: Instant,
}

/// Keeps a bridge on the bus as its upstream: a node address claimed from
/// the bus's router, and from the same router a net for every segment of
/// this bridge. Starts over when the router stops answering or refuses a
/// renewal, as after it restarted.
#[embassy_executor::task]
pub async fn uplink(stack: &'static Stack, candidate: u8, nonce: u64) {
    loop {
        // Addresses 3..=254 are the ones a node may claim.
        let first = u16::from(candidate.clamp(3, 254)) - 3;
        let candidates = (0..8u16).map(move |step| 3 + ((first + step * 37) % 252) as u8);
        let claim = with_timeout(
            ANSWER,
            bus_claim_with_retry(&stack, UPSTREAM_IDENT, candidates, nonce),
        )
        .await;
        let Ok(Ok(claim)) = claim else {
            Timer::after(RETRY).await;
            continue;
        };
        log::info!(
            "CAN: node {} of net {} on the bus",
            claim.node_id,
            claim.net_id
        );
        held(stack, claim).await;
        log::warn!("CAN: the bus's router refused a net or a renewal, asking again");
        Timer::after(RETRY).await;
    }
}

/// Holds the claim and the nets of the segments for as long as the router
/// renews them.
async fn held(stack: &'static Stack, mut claim: NodeClaimLease) {
    let mut claim_renew = renewal(claim.expires_seconds, claim.min_refresh_seconds);
    let mut leased: heapless::Vec<Leased, { super::SEGMENTS }> = heapless::Vec::new();
    loop {
        // A segment registered since the last round, such as the Wi-Fi port,
        // has no net yet.
        for segment in 0..super::SEGMENTS as u8 {
            let registered = stack
                .manage_profile(|p| p.interface_state(segment))
                .is_some();
            if !registered || leased.iter().any(|held| held.segment == segment) {
                continue;
            }
            let Ok(Ok(lease)) =
                with_timeout(ANSWER, bridge_seed_assign(&stack, UPSTREAM_IDENT, segment)).await
            else {
                return;
            };
            log::info!("CAN: net {} for segment {}", lease.net_id, segment);
            STATE_NOTIFY.wake_all();
            let renew = renewal(lease.expires_seconds, lease.min_refresh_seconds);
            if leased
                .push(Leased {
                    segment,
                    lease,
                    renew,
                })
                .is_err()
            {
                return;
            }
        }
        let next = leased
            .iter()
            .map(|held| held.renew)
            .chain([claim_renew])
            .min()
            .unwrap_or(claim_renew);
        // No later than a second from now, so that a new segment is seen.
        Timer::at(next.min(Instant::now() + Duration::from_secs(1))).await;
        let now = Instant::now();
        if claim_renew <= now {
            let Ok(Ok(fresh)) = with_timeout(ANSWER, bus_claim_refresh(&stack, &claim)).await
            else {
                return;
            };
            claim_renew = renewal(fresh.expires_seconds, fresh.min_refresh_seconds);
            claim = fresh;
        }
        for held in leased.iter_mut().filter(|held| held.renew <= now) {
            let Ok(Ok(fresh)) =
                with_timeout(ANSWER, bridge_seed_refresh(&stack, &held.lease)).await
            else {
                return;
            };
            held.renew = renewal(fresh.expires_seconds, fresh.min_refresh_seconds);
            held.lease = fresh;
        }
    }
}
