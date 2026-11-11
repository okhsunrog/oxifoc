//! Exclusive session-bound consumer of the canonical fixed-record ISR queue.
//! A stop ACK disables the producer; only the session-bound END proves drain.

use super::streaming::{
    FAST_DECIM_CTR, FAST_TELEM_GATE, FAST_TELEM_PERIOD, FAST_TELEM_Q, fast_telem_stats,
};
use super::velocity_debug;
use crate::{
    icd::{RawTelemetryEndpoint, VelocityDebugArmEndpoint},
    raw_telemetry::{
        RAW_HEADER_BYTES, RawBlockEncoder, RawBlockHeader, RawEncoding, RawTelemetryAck,
        RawTelemetryRequest, RawTelemetryStatus, VelocityDebugArm,
    },
    timer::Timer,
    types::{ControlTiming, TelemetryConfigAck},
    velocity_telemetry::VELOCITY_RECORD_BYTES,
};
use core::{
    pin::pin,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

pub(super) static PRODUCER_DROPS: AtomicU32 = AtomicU32::new(0);
static READY: AtomicBool = AtomicBool::new(false);
static CONTROL: Mutex<CriticalSectionRawMutex, RawControl> = Mutex::new(RawControl::new());

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawSinkError {
    Unavailable,
}

/// A successful call transferred the entire block. Implementations bound their
/// own transient retries; timeout/disconnect must return an error, never success
/// after truncation. No queue or control mutex is held while this future runs.
#[allow(async_fn_in_trait)]
pub trait RawTelemetrySink {
    async fn write_block(
        &mut self,
        destination: ergot::Address,
        bytes: &[u8],
    ) -> Result<(), RawSinkError>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Open,
    Closing,
    Complete,
    Failed,
}
#[derive(Clone, Copy)]
struct RawControl {
    session: u64,
    encoding: RawEncoding,
    fast_hz: u16,
    period: u32,
    velocity_debug: bool,
    phase: Phase,
    owner: Option<ergot::Address>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Action {
    None,
    Arm(u32),
    Stop,
}
impl RawControl {
    const fn new() -> Self {
        Self {
            session: 0,
            encoding: RawEncoding::Packed28,
            fast_hz: 0,
            period: 0,
            velocity_debug: false,
            phase: Phase::Idle,
            owner: None,
        }
    }
    fn active(self) -> bool {
        matches!(self.phase, Phase::Open | Phase::Closing)
    }
    fn status(self) -> RawTelemetryStatus {
        match self.phase {
            Phase::Open => RawTelemetryStatus::Armed,
            Phase::Closing => RawTelemetryStatus::Stopping,
            Phase::Idle | Phase::Complete => RawTelemetryStatus::Stopped,
            Phase::Failed => RawTelemetryStatus::Unavailable,
        }
    }
    #[cfg(test)]
    fn request(
        &mut self,
        request: RawTelemetryRequest,
        foc_hz: u32,
        ready: bool,
        flash: bool,
    ) -> (RawTelemetryStatus, Action) {
        self.request_with_options(request, foc_hz, ready, flash, false)
    }
    fn request_with_options(
        &mut self,
        request: RawTelemetryRequest,
        foc_hz: u32,
        ready: bool,
        flash: bool,
        debug: bool,
    ) -> (RawTelemetryStatus, Action) {
        match request {
            RawTelemetryRequest::Status => (self.status(), Action::None),
            RawTelemetryRequest::Arm {
                session,
                previous_session,
                encoding,
                fast_hz,
            } => {
                if debug && encoding != RawEncoding::Packed28 {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                if debug && !velocity_debug::available() {
                    return (RawTelemetryStatus::Unavailable, Action::None);
                }
                if !cfg!(any(test, feature = "diagnostic-telemetry-codecs"))
                    && encoding != RawEncoding::Packed28
                {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                if session == 0 || fast_hz == 0 || foc_hz == 0 {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                if session == self.session {
                    return if encoding == self.encoding
                        && fast_hz == self.fast_hz
                        && debug == self.velocity_debug
                    {
                        (self.status(), Action::None)
                    } else {
                        (RawTelemetryStatus::Busy, Action::None)
                    };
                }
                if previous_session != self.session || self.active() || flash {
                    return (RawTelemetryStatus::Busy, Action::None);
                }
                if !ready {
                    return (RawTelemetryStatus::Unavailable, Action::None);
                }
                let period = (foc_hz / u32::from(fast_hz)).max(1);
                // The companion records ride in each native Packed28 envelope.
                // A decimated carrier cannot guarantee sufficient bandwidth.
                if debug && period != 1 {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                *self = Self {
                    session,
                    encoding,
                    fast_hz,
                    period,
                    velocity_debug: debug,
                    phase: Phase::Open,
                    owner: None,
                };
                (RawTelemetryStatus::Armed, Action::Arm(period))
            }
            RawTelemetryRequest::Stop { session } => {
                if session == 0 {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                if session != self.session {
                    return (
                        if self.active() {
                            RawTelemetryStatus::Busy
                        } else {
                            RawTelemetryStatus::Invalid
                        },
                        Action::None,
                    );
                }
                if self.phase == Phase::Open {
                    self.phase = Phase::Closing;
                    (RawTelemetryStatus::Stopping, Action::Stop)
                } else {
                    (self.status(), Action::None)
                }
            }
        }
    }
}

#[cfg(test)]
async fn control_request(
    request: RawTelemetryRequest,
    foc_hz: u32,
    timing: ControlTiming,
) -> RawTelemetryAck {
    control_request_from_peer(
        request,
        foc_hz,
        timing,
        ergot::Address {
            network_id: 1,
            node_id: 2,
            port_id: 0,
        },
    )
    .await
}

async fn control_request_from_peer(
    request: RawTelemetryRequest,
    foc_hz: u32,
    timing: ControlTiming,
    peer: ergot::Address,
) -> RawTelemetryAck {
    control_request_with_options(request, foc_hz, timing, peer, false).await
}

async fn control_request_with_options(
    request: RawTelemetryRequest,
    foc_hz: u32,
    timing: ControlTiming,
    mut peer: ergot::Address,
    debug: bool,
) -> RawTelemetryAck {
    let _gate = FAST_TELEM_GATE.lock().await;
    let mut control = CONTROL.lock().await;
    // Endpoints allocate a fresh source port per call. Ownership belongs to
    // the requesting node, not that transient reply port.
    peer.port_id = 0;
    let foreign = control.owner.is_some_and(|owner| owner != peer)
        && !matches!(request, RawTelemetryRequest::Status)
        && (control.active()
            || match request {
                RawTelemetryRequest::Arm { session, .. }
                | RawTelemetryRequest::Stop { session } => session == control.session,
                RawTelemetryRequest::Status => false,
            });
    let (mut status, action) = critical_section::with(|_| {
        if foreign {
            return (RawTelemetryStatus::Busy, Action::None);
        }
        let outcome = control.request_with_options(
            request,
            foc_hz,
            READY.load(Ordering::Acquire),
            crate::state::FLASH_OP_PENDING.load(Ordering::SeqCst),
            debug,
        );
        if !matches!(outcome.1, Action::None) {
            FAST_TELEM_PERIOD.store(0, Ordering::Release);
            velocity_debug::disable();
        }
        outcome
    });
    if let Action::Arm(period) = action {
        control.owner = Some(peer);
        // Disabled producer + task gate give exclusive queue ownership. Do not
        // mask the FOC interrupt while discarding an old queue's whole prefix.
        FAST_TELEM_Q.discard();
        velocity_debug::discard();
        PRODUCER_DROPS.store(0, Ordering::Relaxed);
        FAST_DECIM_CTR.store(0, Ordering::Relaxed);
        critical_section::with(|_| {
            if crate::state::FLASH_OP_PENDING.load(Ordering::SeqCst) {
                control.phase = Phase::Failed;
                status = RawTelemetryStatus::Busy;
            } else {
                if control.velocity_debug {
                    velocity_debug::enable();
                }
                FAST_TELEM_PERIOD.store(period, Ordering::Release);
            }
        });
    }
    let requested_session = match request {
        RawTelemetryRequest::Arm { session, .. } | RawTelemetryRequest::Stop { session } => session,
        RawTelemetryRequest::Status => control.session,
    };
    let encoding = match request {
        RawTelemetryRequest::Arm { encoding, .. } => encoding,
        RawTelemetryRequest::Stop { .. } | RawTelemetryRequest::Status => control.encoding,
    };
    let period = if requested_session == control.session && control.phase == Phase::Open {
        control.period
    } else {
        0
    };
    RawTelemetryAck {
        session: requested_session,
        encoding,
        status,
        timing: TelemetryConfigAck {
            actual_fast_hz: timing.stream_hz(period),
            decimation_m: period,
            control_timing: timing,
        },
    }
}

pub async fn raw_telemetry_server<NS: NetStackHandle, const N: usize>(
    endpoints: Endpoints<NS>,
    foc_hz: u32,
    timing: ControlTiming,
) {
    let server = endpoints.bounded_server::<RawTelemetryEndpoint, N>(Some("raw_telemetry"));
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve_full(
                |message: &ergot::socket::HeaderMessage<RawTelemetryRequest>| {
                    control_request_from_peer(message.t, foc_hz, timing, message.hdr.src)
                },
            )
            .await;
    }
}

pub async fn velocity_debug_arm_server<NS: NetStackHandle, const N: usize>(
    endpoints: Endpoints<NS>,
    foc_hz: u32,
    timing: ControlTiming,
) {
    let server = endpoints.bounded_server::<VelocityDebugArmEndpoint, N>(Some("velocity_debug"));
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve_full(|message: &ergot::socket::HeaderMessage<VelocityDebugArm>| {
                let arm = message.t;
                control_request_with_options(
                    RawTelemetryRequest::Arm {
                        session: arm.session,
                        previous_session: arm.previous_session,
                        encoding: RawEncoding::Packed28,
                        fast_hz: arm.fast_hz,
                    },
                    foc_hz,
                    timing,
                    message.hdr.src,
                    true,
                )
            })
            .await;
    }
}

async fn fail_session(session: u64) {
    let _gate = FAST_TELEM_GATE.lock().await;
    let mut control = CONTROL.lock().await;
    if control.session == session {
        critical_section::with(|_| {
            FAST_TELEM_PERIOD.store(0, Ordering::Release);
            velocity_debug::disable();
        });
        control.phase = Phase::Failed;
        FAST_TELEM_Q.discard();
        velocity_debug::discard();
    }
}

fn append_velocity_block(
    buffer: &mut [u8],
    session: u64,
    index: u32,
    final_total: Option<u64>,
) -> (usize, usize) {
    let space = buffer.len().saturating_sub(RAW_HEADER_BYTES);
    let capacity = (space / VELOCITY_RECORD_BYTES).min(crate::raw_telemetry::RAW_MAX_RECORDS);
    if capacity == 0 {
        return (0, 0);
    }
    let payload = velocity_debug::drain(
        &mut buffer[RAW_HEADER_BYTES..RAW_HEADER_BYTES + capacity * VELOCITY_RECORD_BYTES],
    );
    let count = payload / VELOCITY_RECORD_BYTES;
    let end = if count == 0 { final_total } else { None };
    if count == 0 && end.is_none() {
        return (0, 0);
    }
    RawBlockHeader::velocity(
        session,
        index,
        count as u8,
        velocity_debug::DROPS.load(Ordering::Acquire),
        end,
    )
    .write(buffer);
    (RAW_HEADER_BYTES + payload, count)
}

pub async fn raw_telemetry_stream<S: RawTelemetrySink, T: Timer>(sink: &mut S) {
    let mut encoder = RawBlockEncoder::new();
    let mut debug_total = 0_u64;
    let mut debug_ended = false;
    let mut session = 0;
    let mut block_index = 0;
    let mut total_records = 0_u64;
    READY.store(true, Ordering::Release);
    loop {
        let gate = FAST_TELEM_GATE.lock().await;
        let control = *CONTROL.lock().await;
        if !control.active() {
            drop(gate);
            T::after_millis(1).await;
            continue;
        }
        if session != control.session {
            session = control.session;
            block_index = 0;
            total_records = 0;
            debug_total = 0;
            debug_ended = false;
            encoder.begin(
                session,
                block_index,
                control.encoding,
                control.period as u16,
            );
        }
        let closing = control.phase == Phase::Closing;
        let mut refused = false;
        let read = FAST_TELEM_Q.drain_raw(|frame| {
            let accepted = encoder.push(frame);
            refused |= !accepted;
            accepted
        });
        if read != 0 {
            fast_telem_stats::READ_OK.fetch_add(read, Ordering::Relaxed);
        }
        if !encoder.is_full() && !refused && !closing {
            drop(gate);
            T::after_micros(500).await;
            continue;
        }
        let count = encoder.len();
        if refused && count == 0 {
            // An unrepresentable first sample cannot be drained by retrying
            // empty blocks. Fail explicitly; keep the control loop running.
            drop(gate);
            fail_session(session).await;
            continue;
        }
        let closing_velocity = count == 0 && closing && control.velocity_debug && !debug_ended;
        let end = count == 0 && closing && !closing_velocity;
        let drops = PRODUCER_DROPS.load(Ordering::Acquire);
        let mut velocity_records = 0;
        let bytes = if closing_velocity {
            let buffer = encoder.empty_scratch();
            let (len, records) =
                append_velocity_block(buffer, session, block_index, Some(debug_total));
            velocity_records = records;
            &buffer[..len]
        } else if end {
            encoder.end(session, block_index, control.encoding, total_records, drops)
        } else if control.velocity_debug {
            // Native samples always get first claim on the envelope. Append a
            // companion block to the existing buffer, with no extra send/await.
            encoder.finish_with_suffix(drops, |suffix| {
                let (len, records) =
                    append_velocity_block(suffix, session, block_index.wrapping_add(1), None);
                velocity_records = records;
                len
            })
        } else {
            encoder.finish(drops)
        };
        drop(gate);
        // A stop arriving during this await disables only the producer. It
        // cannot invalidate already accepted records or manufacture an END.
        if let Some(destination) = control.owner {
            if sink.write_block(destination, bytes).await.is_err() {
                fail_session(session).await;
                continue;
            }
        } else {
            fail_session(session).await;
            continue;
        }
        if end {
            let _gate = FAST_TELEM_GATE.lock().await;
            let mut current = CONTROL.lock().await;
            if current.session == session && current.phase == Phase::Closing {
                current.phase = Phase::Complete;
            }
        } else {
            total_records += count as u64;
            debug_total += velocity_records as u64;
            if closing_velocity && velocity_records == 0 {
                debug_ended = true;
            }
            block_index =
                block_index.wrapping_add(1 + u32::from(!closing_velocity && velocity_records != 0));
            encoder.begin(
                session,
                block_index,
                control.encoding,
                control.period as u16,
            );
        }
        embassy_futures::yield_now().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{raw_telemetry::decode_raw_block, types::FastTelemetry};
    use core::{
        future::Future,
        task::{Context, Poll, Waker},
    };
    use std::sync::{Arc, Mutex as StdMutex};
    #[test]
    fn session_retries_conflicts_stop_and_stale_tokens_do_not_rearm() {
        let mut state = RawControl::new();
        let arm = RawTelemetryRequest::Arm {
            session: 7,
            previous_session: 0,
            encoding: RawEncoding::Packed26,
            fast_hz: 20_000,
        };
        assert_eq!(
            state.request(arm, 20_000, false, false),
            (RawTelemetryStatus::Unavailable, Action::None)
        );
        assert_eq!(
            state.request(arm, 20_000, true, false),
            (RawTelemetryStatus::Armed, Action::Arm(1))
        );
        assert_eq!(
            state.request(arm, 20_000, true, false),
            (RawTelemetryStatus::Armed, Action::None)
        );
        assert_eq!(
            state
                .request(
                    RawTelemetryRequest::Arm {
                        session: 8,
                        previous_session: 7,
                        encoding: RawEncoding::Raw34,
                        fast_hz: 5000
                    },
                    20_000,
                    true,
                    false
                )
                .0,
            RawTelemetryStatus::Busy
        );
        assert_eq!(
            state
                .request(
                    RawTelemetryRequest::Stop { session: 8 },
                    20_000,
                    true,
                    false
                )
                .0,
            RawTelemetryStatus::Busy
        );
        assert_eq!(
            state.request(
                RawTelemetryRequest::Stop { session: 7 },
                20_000,
                true,
                false
            ),
            (RawTelemetryStatus::Stopping, Action::Stop)
        );
        assert_eq!(
            state.request(arm, 20_000, true, false),
            (RawTelemetryStatus::Stopping, Action::None)
        );
        state.phase = Phase::Complete;
        assert_eq!(
            state.request(arm, 20_000, true, false),
            (RawTelemetryStatus::Stopped, Action::None)
        );
        assert_eq!(
            state
                .request(
                    RawTelemetryRequest::Arm {
                        session: 8,
                        previous_session: 7,
                        encoding: RawEncoding::Raw34,
                        fast_hz: 5000
                    },
                    20_000,
                    true,
                    true
                )
                .0,
            RawTelemetryStatus::Busy
        );
        let next = RawTelemetryRequest::Arm {
            session: 8,
            previous_session: 7,
            encoding: RawEncoding::Raw34,
            fast_hz: 5000,
        };
        assert_eq!(
            state.request(next, 20_000, true, false),
            (RawTelemetryStatus::Armed, Action::Arm(4))
        );
        state.phase = Phase::Complete;
        assert_eq!(
            state.request(arm, 20_000, true, false),
            (RawTelemetryStatus::Busy, Action::None),
            "old Arm payload cannot replay after newer completion"
        );
        assert_eq!(
            state.request(next, 20_000, true, false),
            (RawTelemetryStatus::Stopped, Action::None)
        );
        assert_eq!(
            state
                .request(RawTelemetryRequest::Status, 20_000, true, false)
                .0,
            RawTelemetryStatus::Stopped
        );
        assert_eq!(state.session, 8);
    }

    struct YieldTimer;
    impl Timer for YieldTimer {
        async fn after_millis(_: u64) {
            embassy_futures::yield_now().await;
        }
        async fn after_micros(_: u64) {
            embassy_futures::yield_now().await;
        }
    }
    struct HeldSink {
        held: Arc<AtomicBool>,
        fail: Arc<AtomicBool>,
        blocks: Arc<StdMutex<Vec<Vec<u8>>>>,
    }
    impl RawTelemetrySink for HeldSink {
        async fn write_block(
            &mut self,
            _destination: ergot::Address,
            bytes: &[u8],
        ) -> Result<(), RawSinkError> {
            core::future::poll_fn(|_| {
                if self.held.load(Ordering::Relaxed) {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
            if self.fail.load(Ordering::Relaxed) {
                return Err(RawSinkError::Unavailable);
            }
            self.blocks.lock().unwrap().push(bytes.to_vec());
            Ok(())
        }
    }
    fn ready<F: Future>(future: F) -> F::Output {
        let mut future = pin!(future);
        match future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
        {
            Poll::Ready(value) => value,
            Poll::Pending => panic!("control path blocked by raw I/O"),
        }
    }
    #[test]
    fn stop_ack_does_not_wait_for_io_and_end_proves_drain_before_new_session() {
        let _guard = super::super::streaming::FAST_TELEM_TEST_LOCK
            .lock()
            .unwrap();
        *CONTROL.try_lock().unwrap() = RawControl::new();
        FAST_TELEM_Q.discard();
        FAST_TELEM_PERIOD.store(0, Ordering::Relaxed);
        let held = Arc::new(AtomicBool::new(true));
        let fail = Arc::new(AtomicBool::new(false));
        let blocks = Arc::new(StdMutex::new(Vec::new()));
        let mut sink = HeldSink {
            held: held.clone(),
            fail: fail.clone(),
            blocks: blocks.clone(),
        };
        let mut stream = pin!(raw_telemetry_stream::<_, YieldTimer>(&mut sink));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(stream.as_mut().poll(&mut cx).is_pending());
        let timing = ControlTiming {
            clock_hz: 168_000_000,
            period_ticks: 8400,
        };
        let arm = |session, previous_session| RawTelemetryRequest::Arm {
            session,
            previous_session,
            encoding: RawEncoding::Packed26,
            fast_hz: 20_000,
        };
        assert_eq!(
            ready(control_request(arm(7, 0), 20_000, timing)).status,
            RawTelemetryStatus::Armed
        );
        for seq in [65535, 0] {
            FAST_TELEM_Q
                .push(&FastTelemetry {
                    seq,
                    ..FastTelemetry::default()
                })
                .unwrap();
        }
        assert!(stream.as_mut().poll(&mut cx).is_pending());
        assert_eq!(
            ready(control_request(
                RawTelemetryRequest::Stop { session: 7 },
                20_000,
                timing
            ))
            .status,
            RawTelemetryStatus::Stopping
        );
        assert_eq!(FAST_TELEM_PERIOD.load(Ordering::Relaxed), 0);
        assert!(stream.as_mut().poll(&mut cx).is_pending(), "sink is held");
        assert_eq!(
            ready(control_request(arm(8, 7), 20_000, timing)).status,
            RawTelemetryStatus::Busy
        );
        assert_eq!(
            ready(control_request(
                RawTelemetryRequest::Stop { session: 7 },
                20_000,
                timing
            ))
            .status,
            RawTelemetryStatus::Stopping
        );
        assert_eq!(
            ready(super::super::streaming::set_fast_telemetry_period(1)),
            0
        );
        assert!(blocks.lock().unwrap().is_empty());
        held.store(false, Ordering::Relaxed);
        for _ in 0..3 {
            assert!(stream.as_mut().poll(&mut cx).is_pending());
        }
        let written = blocks.lock().unwrap();
        assert_eq!(written.len(), 2);
        let data = decode_raw_block(&written[0]).unwrap();
        assert_eq!(
            data.samples.iter().map(|f| f.seq).collect::<Vec<_>>(),
            [65535, 0]
        );
        let end = decode_raw_block(&written[1]).unwrap();
        assert!(end.header.is_end());
        assert_eq!(end.header.total_records, 2);
        assert_eq!(end.header.block_index, 1);
        drop(written);
        assert_eq!(
            ready(control_request(RawTelemetryRequest::Status, 20_000, timing)).status,
            RawTelemetryStatus::Stopped
        );
        assert_eq!(
            ready(control_request(arm(7, 0), 20_000, timing)).status,
            RawTelemetryStatus::Stopped,
            "duplicate Arm cannot restart completed cookie"
        );
        assert_eq!(
            ready(control_request(arm(8, 7), 20_000, timing)).status,
            RawTelemetryStatus::Armed
        );
        FAST_TELEM_Q
            .push(&FastTelemetry {
                seq: 53,
                ..FastTelemetry::default()
            })
            .unwrap();
        ready(control_request(
            RawTelemetryRequest::Stop { session: 8 },
            20_000,
            timing,
        ));
        for _ in 0..3 {
            assert!(stream.as_mut().poll(&mut cx).is_pending());
        }
        let written = blocks.lock().unwrap();
        assert_eq!(written.len(), 4);
        assert_eq!(decode_raw_block(&written[2]).unwrap().header.session, 8);
        assert_eq!(decode_raw_block(&written[2]).unwrap().samples[0].seq, 53);
        assert_eq!(
            decode_raw_block(&written[3]).unwrap().header.total_records,
            1
        );
        drop(written);
        assert_eq!(
            ready(control_request(arm(9, 8), 20_000, timing)).status,
            RawTelemetryStatus::Armed
        );
        FAST_TELEM_Q
            .push(&FastTelemetry {
                seq: 61,
                ..FastTelemetry::default()
            })
            .unwrap();
        ready(control_request(
            RawTelemetryRequest::Stop { session: 9 },
            20_000,
            timing,
        ));
        fail.store(true, Ordering::Relaxed);
        for _ in 0..3 {
            assert!(stream.as_mut().poll(&mut cx).is_pending());
        }
        assert_eq!(
            blocks.lock().unwrap().len(),
            4,
            "failed data cannot fabricate END"
        );
        assert_eq!(
            ready(control_request(RawTelemetryRequest::Status, 20_000, timing)).status,
            RawTelemetryStatus::Unavailable
        );
        assert_eq!(FAST_TELEM_PERIOD.load(Ordering::Relaxed), 0);
        assert_eq!(
            ready(control_request(arm(9, 8), 20_000, timing)).status,
            RawTelemetryStatus::Unavailable
        );
        fail.store(false, Ordering::Relaxed);
        assert_eq!(
            ready(control_request(arm(10, 9), 20_000, timing)).status,
            RawTelemetryStatus::Armed
        );
        ready(control_request(
            RawTelemetryRequest::Stop { session: 10 },
            20_000,
            timing,
        ));
        for _ in 0..3 {
            assert!(stream.as_mut().poll(&mut cx).is_pending());
        }
        let written = blocks.lock().unwrap();
        assert_eq!(written.len(), 5);
        let end = decode_raw_block(&written[4]).unwrap();
        assert!(end.header.is_end());
        assert_eq!(end.header.session, 10);
        assert_eq!(end.header.total_records, 0);
        drop(written);
        *CONTROL.try_lock().unwrap() = RawControl::new();
        FAST_TELEM_PERIOD.store(0, Ordering::Relaxed);
        READY.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod routed_owner_tests {
    use super::*;
    #[test]
    fn a_raw_session_follows_its_owner_node_across_endpoint_ports() {
        let _guard = super::super::streaming::FAST_TELEM_TEST_LOCK
            .lock()
            .unwrap();
        *CONTROL.try_lock().unwrap() = RawControl::new();
        READY.store(true, Ordering::Release);
        let timing = ControlTiming::from_hz(20_000);
        let peer = ergot::Address {
            network_id: 7,
            node_id: 2,
            port_id: 31,
        };
        let arm = RawTelemetryRequest::Arm {
            session: 17,
            previous_session: 0,
            encoding: RawEncoding::Packed26,
            fast_hz: 20_000,
        };
        let run = |request, peer| {
            embassy_futures::block_on(control_request_from_peer(request, 20_000, timing, peer))
        };
        assert_eq!(run(arm, peer).status, RawTelemetryStatus::Armed);
        assert_eq!(
            CONTROL.try_lock().unwrap().owner,
            Some(ergot::Address { port_id: 0, ..peer })
        );
        assert_eq!(
            run(
                arm,
                ergot::Address {
                    port_id: 32,
                    ..peer
                }
            )
            .status,
            RawTelemetryStatus::Armed
        );
        let other = ergot::Address {
            network_id: 8,
            ..peer
        };
        assert_eq!(run(arm, other).status, RawTelemetryStatus::Busy);
        assert_eq!(
            run(RawTelemetryRequest::Stop { session: 17 }, other).status,
            RawTelemetryStatus::Busy
        );
        assert_eq!(CONTROL.try_lock().unwrap().phase, Phase::Open);
        assert_eq!(
            run(RawTelemetryRequest::Stop { session: 17 }, peer).status,
            RawTelemetryStatus::Stopping
        );
        *CONTROL.try_lock().unwrap() = RawControl::new();
        FAST_TELEM_PERIOD.store(0, Ordering::Release);
        FAST_TELEM_Q.discard();
        READY.store(false, Ordering::Release);
    }
}
