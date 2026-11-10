//! Exclusive bench consumer. The ISR and canonical 34-byte queue stay unchanged.
//! A stop ACK disables the producer; only the session-bound END proves drain.

use super::streaming::{
    FAST_DECIM_CTR, FAST_TELEM_GATE, FAST_TELEM_PERIOD, FAST_TELEM_Q, fast_telem_stats,
};
use crate::{
    icd::RawTelemetryEndpoint,
    raw_telemetry::{
        RawBlockEncoder, RawEncoding, RawTelemetryAck, RawTelemetryRequest, RawTelemetryStatus,
    },
    timer::Timer,
    types::{ControlTiming, TelemetryConfigAck},
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
    async fn write_block(&mut self, bytes: &[u8]) -> Result<(), RawSinkError>;
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
    phase: Phase,
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
            encoding: RawEncoding::Raw34,
            fast_hz: 0,
            period: 0,
            phase: Phase::Idle,
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
    fn request(
        &mut self,
        request: RawTelemetryRequest,
        foc_hz: u32,
        ready: bool,
        flash: bool,
    ) -> (RawTelemetryStatus, Action) {
        match request {
            RawTelemetryRequest::Status => (self.status(), Action::None),
            RawTelemetryRequest::Arm {
                session,
                previous_session,
                encoding,
                fast_hz,
            } => {
                if session == 0 || fast_hz == 0 || foc_hz == 0 {
                    return (RawTelemetryStatus::Invalid, Action::None);
                }
                if session == self.session {
                    return if encoding == self.encoding && fast_hz == self.fast_hz {
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
                *self = Self {
                    session,
                    encoding,
                    fast_hz,
                    period,
                    phase: Phase::Open,
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

async fn control_request(
    request: RawTelemetryRequest,
    foc_hz: u32,
    timing: ControlTiming,
) -> RawTelemetryAck {
    let _gate = FAST_TELEM_GATE.lock().await;
    let mut control = CONTROL.lock().await;
    let (mut status, action) = critical_section::with(|_| {
        let outcome = control.request(
            request,
            foc_hz,
            READY.load(Ordering::Acquire),
            crate::state::FLASH_OP_PENDING.load(Ordering::SeqCst),
        );
        if !matches!(outcome.1, Action::None) {
            FAST_TELEM_PERIOD.store(0, Ordering::Release);
        }
        outcome
    });
    if let Action::Arm(period) = action {
        // Disabled producer + task gate give exclusive queue ownership. Do not
        // mask the FOC interrupt while discarding an old queue's whole prefix.
        FAST_TELEM_Q.discard();
        PRODUCER_DROPS.store(0, Ordering::Relaxed);
        FAST_DECIM_CTR.store(0, Ordering::Relaxed);
        critical_section::with(|_| {
            if crate::state::FLASH_OP_PENDING.load(Ordering::SeqCst) {
                control.phase = Phase::Failed;
                status = RawTelemetryStatus::Busy;
            } else {
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
            .serve(|request: &RawTelemetryRequest| control_request(*request, foc_hz, timing))
            .await;
    }
}

async fn fail_session(session: u64) {
    let _gate = FAST_TELEM_GATE.lock().await;
    let mut control = CONTROL.lock().await;
    if control.session == session {
        FAST_TELEM_PERIOD.store(0, Ordering::Release);
        control.phase = Phase::Failed;
        FAST_TELEM_Q.discard();
    }
}

pub async fn raw_telemetry_stream<S: RawTelemetrySink, T: Timer>(sink: &mut S) {
    let mut encoder = RawBlockEncoder::new();
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
            encoder.begin(
                session,
                block_index,
                control.encoding,
                control.period as u16,
            );
        }
        let mut refused = false;
        let read = FAST_TELEM_Q.drain_raw(|frame| {
            let accepted = encoder.push(frame);
            refused |= !accepted;
            accepted
        });
        if read != 0 {
            fast_telem_stats::READ_OK.fetch_add(read, Ordering::Relaxed);
        }
        let closing = control.phase == Phase::Closing;
        if !encoder.is_full() && !refused && !closing {
            drop(gate);
            T::after_micros(500).await;
            continue;
        }
        let count = encoder.len();
        let end = count == 0 && closing;
        let drops = PRODUCER_DROPS.load(Ordering::Acquire);
        let bytes = if end {
            encoder.end(session, block_index, control.encoding, total_records, drops)
        } else {
            encoder.finish(drops)
        };
        drop(gate);
        // A stop arriving during this await disables only the producer. It
        // cannot invalidate already accepted records or manufacture an END.
        if sink.write_block(bytes).await.is_err() {
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
            block_index = block_index.wrapping_add(1);
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
        async fn write_block(&mut self, bytes: &[u8]) -> Result<(), RawSinkError> {
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
