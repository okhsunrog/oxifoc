//! Fast telemetry streaming via lock-free bbqueue.
//!
//! ISR/sim writes `FastTelemetry` structs into a bbqueue at the decimated rate.
//! The async streaming task wakes on a timer, drains the queue, and broadcasts
//! batches of up to FAST_BATCH_SAMPLES samples as a single ergot topic message.
//!
//! Streaming is disabled by default — the host must send `TelemetryConfig`
//! with a nonzero `fast_hz` to start.

use core::mem::size_of;
use core::sync::atomic::{AtomicU32, Ordering};

use bbqueue::{nicknames::Churrasco, prod_cons::framed::FramedConsumer, traits::bbqhdl::BbqHandle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use ergot::net_stack::NetStackHandle;

use crate::foc::controller::FocOutput;
use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::icd::FastTelemetryTopic;
use crate::icd::FaultTopic;
use crate::state::{MotorControlState, update_telemetry};
use crate::timer::Timer;
use crate::types::FastTelemetry;

/// Lock-free queue for ISR → async telemetry transfer.
///
/// ~22 frames of 46 bytes (44 data + 2 header) fit in 1024 bytes.
/// Churrasco = Inline + Atomic + Polling + Static reference.
///
/// Firmware uses 4 KB (about 110 frames, 5.5 ms at 20 kHz). This absorbs
/// short scheduling/transport pauses, not sustained overload. Host
/// builds (virtual device, sims) get a deep queue: tokio sleep jitter
/// reaches tens of milliseconds under load, which at 10–20 kHz overruns a
/// 42-frame buffer and fakes telemetry loss the real device doesn't have.
#[cfg(feature = "std")]
pub static FAST_TELEM_Q: Churrasco<65536> = Churrasco::new();
#[cfg(not(feature = "std"))]
#[cfg_attr(
    all(feature = "fast-telem-ccm", target_arch = "arm", target_os = "none"),
    unsafe(link_section = ".ccm.data")
)]
pub static FAST_TELEM_Q: Churrasco<4096> = Churrasco::new();

/// Decimation period: ISR writes to bbqueue every `period` FOC cycles.
/// 0 = streaming disabled. Set by `telemetry_config_server` when host
/// sends `TelemetryConfig { fast_hz }`.
pub static FAST_TELEM_PERIOD: AtomicU32 = AtomicU32::new(0);

// Serializes configuration with task-side broadcasts, without blocking the
// FOC interrupt. A stop ACK must follow every previously accepted batch.
static FAST_TELEM_GATE: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());
// Invalidates task-held samples as well as the ISR queue on stop/restart.
// The task may miss an intermediate period=0 if a new session starts promptly.
static FAST_TELEM_SESSION: AtomicU32 = AtomicU32::new(0);

/// Set the FOC decimation period and return the applied period.
///
/// Zero disables streaming, waits for any task-side broadcasts and discards
/// pending samples before returning. Nonzero changes are refused during flash
/// maintenance. The FOC producer never waits on the task-side gate.
pub async fn set_fast_telemetry_period(period: u32) -> u32 {
    let _gate = FAST_TELEM_GATE.lock().await;
    let actual = critical_section::with(|_| {
        if crate::state::FLASH_OP_PENDING.load(Ordering::SeqCst) && period != 0 {
            FAST_TELEM_PERIOD.load(Ordering::Acquire)
        } else {
            FAST_TELEM_PERIOD.store(period, Ordering::Release);
            FAST_TELEM_SESSION.fetch_add(1, Ordering::Release);
            period
        }
    });
    if actual == 0 {
        // The stream task holds no read grant outside the gate. Discard the
        // disabled session's pending samples so a later start cannot send them.
        let cons = FAST_TELEM_Q.framed_consumer();
        while let Ok(grant) = cons.read() {
            grant.release();
        }
    }
    actual
}

/// Decimation counter for the raw diagnostic stream: emit one sample every
/// `period`-th FOC cycle. The raw-ADC frame is unfiltered (the host does any
/// anti-alias/decimation), so a plain modulo counter replaces the old CIC.
static FAST_DECIM_CTR: AtomicU32 = AtomicU32::new(0);

/// Diagnostic counters for the fast-telemetry pipeline. Each loss point
/// increments its counter; a board task can report + reset them (e.g. a 1 Hz
/// defmt line) to attribute stream loss to a stage without a debugger.
pub mod fast_telem_stats {
    use core::sync::atomic::AtomicU32;
    /// ISR-side `push_fast_telemetry` successful queue commits.
    pub static PUSH_OK: AtomicU32 = AtomicU32::new(0);
    /// ISR-side `push_fast_telemetry` grant failures (FAST_TELEM_Q full).
    pub static PUSH_DROPS: AtomicU32 = AtomicU32::new(0);
    /// Stream-task frames read from the queue with the expected length.
    pub static READ_OK: AtomicU32 = AtomicU32::new(0);
    /// Stream-task frames discarded for a wrong length.
    pub static READ_BADLEN: AtomicU32 = AtomicU32::new(0);
    /// Stream-task broadcasts that returned an error (e.g. interface queue full).
    pub static BCAST_FAILS: AtomicU32 = AtomicU32::new(0);
    pub static BCAST_FULL: AtomicU32 = AtomicU32::new(0);
    pub static BCAST_BUSY: AtomicU32 = AtomicU32::new(0);
    /// Broadcast reported NoRoute. This also covers remote backpressure:
    /// ergot folds a failed broadcast with no local recipient into NoRoute.
    pub static BCAST_UNROUTED: AtomicU32 = AtomicU32::new(0);
    pub static BCAST_OTHER: AtomicU32 = AtomicU32::new(0);
    /// Wall-clock microseconds, including preemption; zero without Embassy profiling.
    pub static MAX_DRAIN_GAP_US: AtomicU32 = AtomicU32::new(0);
    /// Maximum wall time of one drain slice, through its optional broadcast.
    /// Includes empty/partial slices, but excludes waits between coalesced polls.
    pub static MAX_BATCH_US: AtomicU32 = AtomicU32::new(0);
    /// Wall time spent draining/encoding across all slices, including partial
    /// and empty polls. Retained batches count only work done in the new slice.
    pub static ENCODE_US: AtomicU32 = AtomicU32::new(0);
    /// Wall time spent in synchronous broadcast attempts, including failures.
    pub static BROADCAST_US: AtomicU32 = AtomicU32::new(0);
    /// Stream-task broadcasts accepted by the stack.
    pub static BCAST_OK: AtomicU32 = AtomicU32::new(0);
    /// Successfully broadcast encoded payload bytes (excluding wire headers).
    pub static BCAST_BYTES: AtomicU32 = AtomicU32::new(0);
}

/// Command-path counters (host→device direction), 1 Hz-reported by the
/// device stats task. Bracket the RX pipeline: requests that reached the
/// MotorEndpoint server vs SetModes actually drained by the ISR — a healthy
/// link under host affirms shows ~20/s on both; zeros while the host is
/// affirming localize a silent drop to the transport/routing in between.
pub mod cmd_stats {
    use core::sync::atomic::AtomicU32;
    /// MotorEndpoint requests received by the command server.
    pub static MOTOR_REQS: AtomicU32 = AtomicU32::new(0);
    /// `DriverCommand::SetMode` drained by the ISR (deadman stamp events).
    pub static SETMODE_DRAINED: AtomicU32 = AtomicU32::new(0);
}

/// Two-stage decimating anti-alias filter (CIC order 2 equivalent).
///
/// Plain decimation-by-dropping folds everything above the new Nyquist
/// straight into the diagnostic band — wideband sensor noise raises the
/// floor by √M and real high harmonics (5th/7th at high eRPM, the HFI
/// carrier) alias into **false spectral lines**. This filter convolves a
/// triangular window of length `2M−1` (= two cascaded length-M boxcars,
/// sinc² response) before each decimation: transfer-function **nulls land
/// exactly on the bands that fold to DC** (multiples of f_out), sidelobes
/// −26 dB.
///
/// Implementation avoids classic CIC integrators (unbounded f32 state
/// drifts): per window it keeps the plain sum `A = Σx` and the
/// ramp-weighted sum `U = Σ(k+1)·x`, both reset every dump. The
/// triangular output over windows (m−1, m) is then
/// `y = [(U₋₁ − A₋₁) + ((M+1)·A − U)] / M²` — ascending weights 0..M−1
/// from the previous window, descending M..1 from the current one
/// (weight total M²). Cost: 2 multiply-accumulates per channel per cycle.
///
/// Properties: DC passes exactly; `M = 1` degenerates to the identity;
/// group delay is `M−1` input samples (recorded in the parquet metadata
/// by the host tooling — phase-sensitive analysis must account for it).
pub struct CicDecimator2<const N: usize> {
    m: u32,
    count: u32,
    a: [f32; N],
    u: [f32; N],
    a_prev: [f32; N],
    u_prev: [f32; N],
    primed: bool,
}

impl<const N: usize> CicDecimator2<N> {
    /// Unconfigured decimator (`m = 0`): `push` returns `None` until
    /// [`configure`](Self::configure) is called.
    pub const fn new() -> Self {
        Self {
            m: 0,
            count: 0,
            a: [0.0; N],
            u: [0.0; N],
            a_prev: [0.0; N],
            u_prev: [0.0; N],
            primed: false,
        }
    }

    /// Current decimation factor (0 = unconfigured).
    pub fn m(&self) -> u32 {
        self.m
    }

    /// Set the decimation factor and reset all accumulator state.
    pub fn configure(&mut self, m: u32) {
        *self = Self::new();
        self.m = m;
    }

    /// Push one input frame; returns the filtered output frame on every
    /// `m`-th call (the first window is swallowed for priming when m > 1 —
    /// its triangle would be missing the ascending half).
    pub fn push(&mut self, x: &[f32; N]) -> Option<[f32; N]> {
        if self.m == 0 {
            return None;
        }
        let w = (self.count + 1) as f32;
        for (i, &xi) in x.iter().enumerate() {
            self.a[i] += xi;
            self.u[i] += w * xi;
        }
        self.count += 1;
        if self.count < self.m {
            return None;
        }

        let mf = self.m as f32;
        let inv = 1.0 / (mf * mf);
        let mut y = [0.0f32; N];
        for (i, yi) in y.iter_mut().enumerate() {
            let desc_cur = (mf + 1.0) * self.a[i] - self.u[i];
            let asc_prev = self.u_prev[i] - self.a_prev[i];
            *yi = (asc_prev + desc_cur) * inv;
        }

        self.a_prev = self.a;
        self.u_prev = self.u;
        self.a = [0.0; N];
        self.u = [0.0; N];
        self.count = 0;

        if !self.primed && self.m > 1 {
            self.primed = true;
            return None;
        }
        self.primed = true;
        Some(y)
    }
}

impl<const N: usize> Default for CicDecimator2<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Publish one cycle's telemetry, shared by every platform ISR: update the
/// global state (waking calibration/detection listeners) and, when fast
/// streaming is enabled, emit one raw diagnostic frame every `period`-th
/// FOC cycle into the bbqueue.
///
/// Decimation is plain sample-dropping — NO anti-alias filter (the raw
/// frame ships unprocessed ADC counts; the host applies calibration and
/// any filtering downstream). [`CicDecimator2`] below is currently unused
/// by this path — kept only for a future filtered channel.
pub fn publish_cycle_telemetry(
    state_mutex: &critical_section::Mutex<core::cell::RefCell<MotorControlState>>,
    adc: AdcSnapshot,
    hall: Option<HallSnapshot>,
    foc: FocOutput,
    seq: u32,
) {
    let pole_pairs = update_telemetry(state_mutex, adc, hall, foc);

    let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);
    if period == 0 {
        return;
    }
    // Raw diagnostic frame: emit one sample every `period`-th FOC cycle. No CIC
    // anti-alias — the currents ship as raw ADC counts and the host applies
    // calibration/decimation downstream, so there is nothing to filter here.
    if FAST_DECIM_CTR
        .fetch_add(1, Ordering::Relaxed)
        .is_multiple_of(period)
    {
        let telem = build_fast_telemetry(&adc, &foc, pole_pairs, seq);
        push_fast_telemetry(&telem);
    }
}

pub use crate::types::FAST_BATCH_SAMPLES;

/// Build the compact raw diagnostic [`FastTelemetry`] from the cycle's snapshots.
///
/// Currents/vbus ship as raw ADC counts / fixed-point so the host reconstructs
/// engineering units (and `id/iq/iα/iβ`, duty) with the same core math. Only the
/// non-reconstructable quantities are encoded directly: `angle` (estimator
/// output), `vd/vq` (PI outputs), `rpm` (the ACTIVE angle source's velocity).
/// Callable from ISR (no allocation, pure computation).
pub fn build_fast_telemetry(
    adc: &AdcSnapshot,
    foc: &FocOutput,
    pole_pairs: u8,
    seq: u32,
) -> FastTelemetry {
    use core::f32::consts::TAU;
    // Scalars go through the shared fixed-point codec (`FastTelemetry::pack_*`)
    // so this encode stays the exact inverse of the host `enrich` decode — one
    // LSB constant per field, round-trip tested in `foc::telemetry`.
    //
    // `FocOutput::velocity_rad_s` is the ACTIVE angle source's electrical
    // velocity (hall / observer / HFI / startup ramp), stamped by
    // `FocDriver::step` — previously this read the hall estimator
    // unconditionally and showed 0 on sensorless boards while spinning.
    // The fast frame stores mechanical RPM; host enrichment multiplies it
    // back by pole pairs for eRPM.
    let mech_rpm = if pole_pairs > 0 {
        foc.velocity_rad_s / f32::from(pole_pairs) * (60.0 / TAU)
    } else {
        0.0
    };
    FastTelemetry {
        ia: adc.ia,
        ib: adc.ib,
        ic: adc.ic,
        vbus: FastTelemetry::pack_vbus(adc.vbus_mv as f32 / 1000.0),
        angle: FastTelemetry::pack_angle(foc.angle_rad),
        vd: FastTelemetry::pack_volt(foc.vd),
        vq: FastTelemetry::pack_volt(foc.vq),
        rpm: FastTelemetry::pack_rpm(mech_rpm),
        seq: seq as u16,
        vphase_adc: adc.vphase.unwrap_or([u16::MAX; 3]),
        pwm_ccr: adc.pwm.map_or([0; 3], |p| p.ccr),
        pwm_arr: adc.pwm.map_or(0, |p| p.arr),
        pwm_enables: adc.pwm.map_or(0, |p| p.enables),
    }
}

/// Push a `FastTelemetry` sample into the bbqueue.
///
/// Called from ISR or sim loop after decimation check.
/// If the queue is full, the sample is silently dropped (correct for real-time).
pub fn push_fast_telemetry(telem: &FastTelemetry) {
    let prod = FAST_TELEM_Q.framed_producer();
    if let Ok(mut grant) = prod.grant(size_of::<FastTelemetry>() as u16) {
        grant.copy_from_slice(bytemuck::bytes_of(telem));
        grant.commit(size_of::<FastTelemetry>() as u16);
        fast_telem_stats::PUSH_OK.fetch_add(1, Ordering::Relaxed);
    } else {
        fast_telem_stats::PUSH_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Run the fault topic publisher.
///
/// Broadcasts the FULL fault snapshot (`FaultSnapshot`) on
/// [`crate::icd::FaultTopic`]: once at start (a reconnecting consumer gets
/// the current state without waiting for a change) and then on every
/// registry change — fault raised, payload refined (the registry signals
/// on value changes, e.g. a sticky HallError upgrading `InvalidState` →
/// `WireDead`), or cleared, so the consumer can drop its indication too.
///
/// Snapshot-not-delta: ergot topics are fire-and-forget, a lost packet must
/// cost staleness rather than a wrong state. A failed enqueue remains dirty
/// and retries the latest coalesced snapshot. The consumer's regular
/// SlowTelemetry poll compares `fault_generation`, so refinement and
/// clear+add losses are detected even when `fault_count` stays unchanged.
pub async fn fault_topic_stream<NS, F, T>(stack: NS, fault_registry: &'static FaultRegistry<F>)
where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
    T: Timer,
{
    let mut _retrying = false;
    loop {
        let snapshot = fault_registry.snapshot_response();
        let result = stack
            .stack()
            .topics()
            .broadcast::<FaultTopic>(&snapshot, None);
        #[cfg(feature = "log")]
        if result.is_err() && !_retrying {
            log::warn!("fault topic broadcast failed: {result:?}; retrying latest snapshot");
        }
        if result.is_ok() {
            _retrying = false;
            fault_registry.wait_for_change().await;
        } else {
            _retrying = true;
            // Keep the state dirty. A bounded retry publishes the latest
            // coalesced snapshot even if no later registry mutation occurs.
            T::after_millis(100).await;
        }
    }
}

/// Run the fast telemetry streaming task.
///
/// Drains `FAST_TELEM_Q` on a timer and broadcasts [`crate::types::FastTelemetryBatch`]
/// via the ergot topic system. When streaming is disabled
/// (`FAST_TELEM_PERIOD == 0`), polls periodically waiting for the host to
/// enable it. Lossless byte deltas are encoded here, outside the ISR. Both
/// byte capacity and sample count are bounded independently of compression.
pub async fn fast_telemetry_stream<NS, T: Timer>(stack: NS, foc_freq_hz: u32)
where
    NS: NetStackHandle + Clone,
{
    let cons = FAST_TELEM_Q.framed_consumer();
    let mut pending = PendingFastBatch::new();
    #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
    let mut last_drain = None;

    #[cfg(feature = "log")]
    log::info!("fast_telemetry_stream: waiting for host to enable streaming");

    loop {
        let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);

        if period == 0 {
            pending.batch.clear();
            #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
            {
                last_drain = None;
            }
            // Enable may precede this wake by more than the embedded queue's
            // few milliseconds of buffering. Capture::start discards that
            // startup transient before collecting the experiment baseline.
            T::after_millis(10).await;
            continue;
        }

        // Sleep for HALF a batch at (foc_freq_hz / period) Hz: draining at
        // exactly one batch per buffer-full leaves zero slack for timer
        // jitter. Half-batch cadence gives margin before the bounded ISR
        // queue fills; incompressible batches can flush before the sample cap.
        let sample_hz = foc_freq_hz / period;
        let interval_us = if sample_hz > 0 {
            ((FAST_BATCH_SAMPLES as u64 / 2).max(1) * 1_000_000) / u64::from(sample_hz)
        } else {
            100_000 // fallback 100ms
        }
        .min(20_000);

        T::after_micros(interval_us).await;

        let mut gate = FAST_TELEM_GATE.lock().await;
        pending.enter_session(FAST_TELEM_SESSION.load(Ordering::Acquire));
        // The host can stop us while the timer is pending. Do not broadcast
        // residual samples after that stop has already been acknowledged.
        if FAST_TELEM_PERIOD.load(Ordering::Acquire) == 0 {
            pending.batch.clear();
            continue;
        }

        // Encode raw ISR frames into independently decodable batches. The
        // encoder reserves worst-case full-frame space before reading a
        // grant, so incompressible input cannot discard a consumed sample.
        loop {
            // The wall clock survives debugger detach, which may stop DWT.
            #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
            let batch_start = embassy_time::Instant::now().as_micros() as u32;
            #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
            if let Some(previous) = last_drain.replace(batch_start) {
                fast_telem_stats::MAX_DRAIN_GAP_US
                    .fetch_max(batch_start.wrapping_sub(previous), Ordering::Relaxed);
            }
            let batch = &mut pending.batch;

            drain_fast_batch(&cons, batch);

            #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
            let encoded_at = embassy_time::Instant::now().as_micros() as u32;

            if batch.is_empty() {
                #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
                record_fast_slice_timing(batch_start, encoded_at, None);
                break; // nothing left to send
            }

            // At high rates retain a partial batch until the next poll rather
            // than flooding USB with half-sized and one-sample messages. At
            // low rates flush partial batches; full-batch latency must stay
            // within a nominal 20 ms. Session invalidation also covers these
            // coalesced samples when a stop/restart occurs between polls.
            if !batch.is_full()
                && u64::from(sample_hz) * 20_000 >= FAST_BATCH_SAMPLES as u64 * 1_000_000
            {
                #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
                record_fast_slice_timing(batch_start, encoded_at, None);
                break;
            }

            let batch_full = batch.is_full();
            let _result = stack
                .stack()
                .topics()
                .broadcast::<FastTelemetryTopic>(batch.batch(), None);
            #[cfg(all(feature = "isr-profiling", feature = "embassy"))]
            {
                let end = embassy_time::Instant::now().as_micros() as u32;
                record_fast_slice_timing(batch_start, encoded_at, Some(end));
            }

            if _result.is_ok() {
                fast_telem_stats::BCAST_OK.fetch_add(1, Ordering::Relaxed);
                fast_telem_stats::BCAST_BYTES
                    .fetch_add(batch.batch().data.len() as u32, Ordering::Relaxed);
                batch.clear();
            } else {
                fast_telem_stats::BCAST_FAILS.fetch_add(1, Ordering::Relaxed);
                use ergot::{interface_manager::InterfaceSendError, net_stack::NetStackSendError};
                let counter = match &_result {
                    Err(NetStackSendError::InterfaceSend(InterfaceSendError::InterfaceFull)) => {
                        &fast_telem_stats::BCAST_FULL
                    }
                    Err(NetStackSendError::WouldDeadlock) => &fast_telem_stats::BCAST_BUSY,
                    Err(NetStackSendError::NoRoute) => &fast_telem_stats::BCAST_UNROUTED,
                    _ => &fast_telem_stats::BCAST_OTHER,
                };
                counter.fetch_add(1, Ordering::Relaxed);
            }

            #[cfg(feature = "log")]
            if _result.is_err() {
                log::warn!("fast_telemetry broadcast failed: {_result:?}");
            }

            if _result.is_err() {
                // Retain every sample and release the task-side gate before
                // waiting. USB TX gets executor time to drain its queue, and
                // a telemetry stop can still acquire the gate immediately.
                // enter_session discards these samples if stop/restart occurs.
                break;
            }

            // If we got fewer than BATCH, the queue is drained
            if !batch_full {
                break;
            }

            // A continuously replenished ISR queue need never become empty.
            // Bound each poll to one batch even when USB runs preemptively:
            // otherwise this task can starve protocol/detection for tens of
            // milliseconds while its USB IRQ drains every successful send.
            // Release the gate BEFORE yielding so stream stop can complete.
            drop(gate);
            embassy_futures::yield_now().await;
            gate = FAST_TELEM_GATE.lock().await;
            pending.enter_session(FAST_TELEM_SESSION.load(Ordering::Acquire));
            if FAST_TELEM_PERIOD.load(Ordering::Acquire) == 0 {
                pending.batch.clear();
                break;
            }
        }
    }
}

fn drain_fast_batch<Q: BbqHandle>(
    cons: &FramedConsumer<Q>,
    batch: &mut crate::types::FastTelemetryBatchEncoder,
) {
    let mut read_ok = 0;
    let mut read_badlen = 0;
    while !batch.is_full()
        && let Ok(grant) = cons.read()
    {
        if batch.push_bytes(&grant) {
            read_ok += 1;
        } else {
            read_badlen += 1;
        }
        grant.release();
    }

    // Publish before returning to any break, yield or await in the task.
    // The F405 reporter shares its executor, so it cannot observe a partial
    // slice. ISR-side push/drop counters remain atomic for every sample.
    if read_ok != 0 {
        fast_telem_stats::READ_OK.fetch_add(read_ok, Ordering::Relaxed);
    }
    if read_badlen != 0 {
        fast_telem_stats::READ_BADLEN.fetch_add(read_badlen, Ordering::Relaxed);
    }
}

// Exactly one call per slice: either a partial/empty exit or a completed
// broadcast attempt. Timestamps are wall time, so ISR preemption is included.
#[cfg(any(test, all(feature = "isr-profiling", feature = "embassy")))]
fn record_fast_slice_timing(start: u32, encoded_at: u32, broadcast_end: Option<u32>) {
    fast_telem_stats::ENCODE_US.fetch_add(encoded_at.wrapping_sub(start), Ordering::Relaxed);
    if let Some(end) = broadcast_end {
        fast_telem_stats::BROADCAST_US.fetch_add(end.wrapping_sub(encoded_at), Ordering::Relaxed);
    }
    fast_telem_stats::MAX_BATCH_US.fetch_max(
        broadcast_end.unwrap_or(encoded_at).wrapping_sub(start),
        Ordering::Relaxed,
    );
}

struct PendingFastBatch {
    session: u32,
    batch: crate::types::FastTelemetryBatchEncoder,
}

impl PendingFastBatch {
    fn new() -> Self {
        Self {
            session: 0,
            batch: crate::types::FastTelemetryBatchEncoder::new(),
        }
    }

    fn enter_session(&mut self, session: u32) {
        if self.session != session {
            self.batch.clear();
            self.session = session;
        }
    }
}

#[cfg(test)]
mod pending_tests {
    use super::*;

    #[test]
    fn slice_timing_counts_partial_full_empty_and_retry_work_once() {
        use fast_telem_stats::{BROADCAST_US, ENCODE_US, MAX_BATCH_US};
        for counter in [&ENCODE_US, &BROADCAST_US, &MAX_BATCH_US] {
            counter.store(0, Ordering::Relaxed);
        }
        // Retain a partial batch, then finish it in a later poll. The idle
        // interval is neither encoding work nor part of a coalesced maximum.
        record_fast_slice_timing(100, 130, None);
        record_fast_slice_timing(1_000, 1_020, Some(1_027));
        assert_eq!(ENCODE_US.load(Ordering::Relaxed), 50);
        assert_eq!(BROADCAST_US.load(Ordering::Relaxed), 7);
        assert_eq!(MAX_BATCH_US.load(Ordering::Relaxed), 30);

        // An empty poll still costs time. A failed send and a later retry
        // each count just their own drain and broadcast work, even though
        // the encoded samples are retained between those attempts.
        record_fast_slice_timing(2_000, 2_003, None);
        record_fast_slice_timing(3_000, 3_008, Some(3_012));
        record_fast_slice_timing(4_000, 4_002, Some(4_007));
        assert_eq!(ENCODE_US.load(Ordering::Relaxed), 63);
        assert_eq!(BROADCAST_US.load(Ordering::Relaxed), 16);
        assert_eq!(MAX_BATCH_US.load(Ordering::Relaxed), 30);

        // The microsecond clock is truncated to u32 on the device.
        record_fast_slice_timing(u32::MAX - 4, 2, Some(6));
        assert_eq!(ENCODE_US.load(Ordering::Relaxed), 70);
        assert_eq!(BROADCAST_US.load(Ordering::Relaxed), 20);
        assert_eq!(MAX_BATCH_US.load(Ordering::Relaxed), 30);
    }

    #[test]
    fn pending_samples_survive_same_session_and_cannot_leak_after_restart() {
        let mut pending = PendingFastBatch::new();
        pending.enter_session(11);
        let first = FastTelemetry {
            seq: 100,
            ..Default::default()
        };
        assert!(pending.batch.push(&first));
        // A rejected broadcast leaves this pending batch intact. The next
        // poll must retain it and can coalesce the following sample into it.
        pending.enter_session(11);
        assert!(pending.batch.push(&FastTelemetry {
            seq: 101,
            ..Default::default()
        }));
        let mut samples = pending.batch.batch().samples().unwrap().into_iter();
        assert_eq!(samples.next().unwrap().seq, 100);
        assert_eq!(samples.next().unwrap().seq, 101);
        assert!(samples.next().is_none());
        drop(samples);
        // Stop and restart can happen while the stream task is asleep, so it
        // might never observe period=0. A session change must still discard
        // every previously accepted pending sample before the next broadcast.
        pending.enter_session(13);
        assert!(pending.batch.is_empty());
        assert!(pending.batch.push(&FastTelemetry {
            seq: 900,
            ..Default::default()
        }));
        assert_eq!(pending.batch.batch().samples().unwrap()[0].seq, 900);
    }
}

#[cfg(test)]
mod drain_tests {
    use super::*;
    use ergot::{
        exports::mutex::raw_impls::cs::CriticalSectionRawMutex as StackMutex,
        interface_manager::{
            Interface, InterfaceState, profiles::direct_edge::DirectEdge,
            utils::framed_stream::Sink,
        },
    };
    use fast_telem_stats::{READ_BADLEN, READ_OK};

    struct QueuedInterface<'a>(core::marker::PhantomData<&'a ()>);

    impl<'a> Interface for QueuedInterface<'a> {
        type Sink = Sink<&'a Churrasco<1024>>;
    }

    #[test]
    fn drain_flushes_partial_full_malformed_empty_and_retry_counts() {
        let queue: Churrasco<4096> = Churrasco::new();
        let prod = queue.framed_producer();
        let cons = queue.framed_consumer();
        let mut batch = crate::types::FastTelemetryBatchEncoder::new();
        READ_OK.store(0, Ordering::Relaxed);
        READ_BADLEN.store(0, Ordering::Relaxed);

        let enqueue = |bytes: &[u8]| {
            let mut grant = prod.grant(bytes.len() as u16).unwrap();
            grant.copy_from_slice(bytes);
            grant.commit(bytes.len() as u16);
        };
        let sample = |seq| FastTelemetry {
            seq,
            ..Default::default()
        };

        // Wrong-length grants must be released and reported alongside valid
        // samples, including when the poll retains a partial batch.
        enqueue(&[1, 2, 3]);
        enqueue(bytemuck::bytes_of(&sample(100)));
        enqueue(&[0; size_of::<FastTelemetry>() + 1]);
        enqueue(bytemuck::bytes_of(&sample(101)));
        drain_fast_batch(&cons, &mut batch);
        assert_eq!(batch.len(), 2);
        assert!(!batch.is_full());
        // Simulate the same-executor reporter's window boundary. All reads
        // are visible before this slice can sleep, with none deferred.
        assert_eq!(READ_OK.swap(0, Ordering::Relaxed), 2);
        assert_eq!(READ_BADLEN.swap(0, Ordering::Relaxed), 2);

        let partial_bytes = batch.batch().data.clone();
        drain_fast_batch(&cons, &mut batch);
        assert_eq!(batch.batch().data, partial_bytes);
        assert_eq!(READ_OK.load(Ordering::Relaxed), 0);
        assert_eq!(READ_BADLEN.load(Ordering::Relaxed), 0);

        // Complete the retained partial batch. The following sample stays
        // queued while a full batch is pending, even across rejected sends.
        for seq in 102..=100 + FAST_BATCH_SAMPLES as u16 {
            enqueue(bytemuck::bytes_of(&sample(seq)));
        }
        drain_fast_batch(&cons, &mut batch);
        assert!(batch.is_full());
        assert_eq!(batch.len(), FAST_BATCH_SAMPLES);
        assert_eq!(READ_OK.swap(0, Ordering::Relaxed), 22);
        assert_eq!(READ_BADLEN.swap(0, Ordering::Relaxed), 0);
        let encoded_bytes = batch.batch().data.clone();

        // Fill the transport queue to reject the send, then free it for
        // the retry. Re-entering the drain with retained bytes must neither
        // consume the next grant nor count already encoded samples again.
        let wire: Churrasco<1024> = Churrasco::new();
        wire.framed_producer().grant(1022).unwrap().commit(1022);
        let stack: ergot::NetStack<StackMutex, DirectEdge<QueuedInterface<'_>>> =
            ergot::NetStack::new_with_profile(DirectEdge::new_controller(
                Sink::new(wire.framed_producer(), 512),
                InterfaceState::Active {
                    net_id: 1,
                    node_id: 1,
                },
            ));
        assert!(
            stack
                .topics()
                .broadcast::<FastTelemetryTopic>(batch.batch(), None)
                .is_err()
        );
        drain_fast_batch(&cons, &mut batch);
        assert_eq!(batch.batch().data, encoded_bytes);
        assert_eq!(READ_OK.load(Ordering::Relaxed), 0);
        assert_eq!(READ_BADLEN.load(Ordering::Relaxed), 0);
        wire.framed_consumer().read().unwrap().release();
        let receiver = stack
            .topics()
            .bounded_receiver::<FastTelemetryTopic, 1>(None);
        let mut receiver = core::pin::pin!(receiver);
        let mut receiver = receiver.as_mut().subscribe();
        assert!(
            stack
                .topics()
                .broadcast::<FastTelemetryTopic>(batch.batch(), None)
                .is_ok()
        );
        let received = receiver.try_recv().unwrap().t;
        wire.framed_consumer().read().unwrap().release();
        assert_eq!(received.data, encoded_bytes);
        let samples = received.samples().unwrap();
        for (index, sample) in samples.iter().enumerate() {
            assert_eq!(sample.seq, 100 + index as u16);
        }
        batch.clear();

        drain_fast_batch(&cons, &mut batch);
        assert_eq!(batch.batch().samples().unwrap()[0].seq, 124);
        assert_eq!(READ_OK.swap(0, Ordering::Relaxed), 1);
        assert_eq!(READ_BADLEN.swap(0, Ordering::Relaxed), 0);
        batch.clear();
        // An empty batch and a malformed-only slice both stop without a
        // broadcast; their counters must still be flushed immediately.
        drain_fast_batch(&cons, &mut batch);
        assert!(batch.is_empty());
        assert_eq!(READ_OK.load(Ordering::Relaxed), 0);
        assert_eq!(READ_BADLEN.load(Ordering::Relaxed), 0);
        enqueue(&[0]);
        drain_fast_batch(&cons, &mut batch);
        assert!(batch.is_empty());
        assert_eq!(READ_OK.swap(0, Ordering::Relaxed), 0);
        assert_eq!(READ_BADLEN.swap(0, Ordering::Relaxed), 1);
        assert!(cons.read().is_err());
    }
}

#[cfg(test)]
mod stream_control_tests {
    use super::*;
    use core::{
        future::Future,
        task::{Context, Poll, Waker},
    };

    #[test]
    fn stop_waits_for_broadcaster_and_discards_pending_samples() {
        let mut cx = Context::from_waker(Waker::noop());
        let gate = FAST_TELEM_GATE.try_lock().unwrap();
        FAST_TELEM_PERIOD.store(1, Ordering::Release);
        let prod = FAST_TELEM_Q.framed_producer();
        let mut grant = prod.grant(4).unwrap();
        grant.copy_from_slice(&[1, 2, 3, 4]);
        grant.commit(4);

        let mut stop = core::pin::pin!(set_fast_telemetry_period(0));
        assert!(stop.as_mut().poll(&mut cx).is_pending());
        assert_eq!(FAST_TELEM_PERIOD.load(Ordering::Acquire), 1);
        drop(gate);
        assert_eq!(stop.as_mut().poll(&mut cx), Poll::Ready(0));
        assert_eq!(FAST_TELEM_PERIOD.load(Ordering::Acquire), 0);
        assert!(FAST_TELEM_Q.framed_consumer().read().is_err());
    }
}

#[cfg(test)]
mod cic_tests {
    use super::CicDecimator2;

    /// Collect `n_out` outputs of a 1-channel decimator fed by `f(t)`.
    fn run(m: u32, n_out: usize, f: impl Fn(usize) -> f32) -> Vec<f32> {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(m);
        let mut out = Vec::new();
        let mut n = 0usize;
        while out.len() < n_out {
            if let Some(y) = cic.push(&[f(n)]) {
                out.push(y[0]);
            }
            n += 1;
        }
        out
    }

    #[test]
    fn dc_passes_exactly() {
        for m in [1u32, 2, 4, 8, 200] {
            let out = run(m, 5, |_| 3.25);
            for y in out {
                assert!((y - 3.25).abs() < 1e-5, "m={m}: DC distorted: {y}");
            }
        }
    }

    #[test]
    fn m1_is_identity() {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(1);
        for n in 0..10 {
            let x = (n as f32).sin();
            assert_eq!(cic.push(&[x]), Some([x]), "m=1 must be transparent");
        }
    }

    /// A sine exactly at the output rate (the first band that folds to DC
    /// under decimation) must be strongly attenuated — that is the whole
    /// point of the sinc² nulls. Naive sample-dropping passes it at full
    /// amplitude as a fake DC offset.
    #[test]
    fn alias_band_is_nulled() {
        use core::f32::consts::TAU;
        let m = 4u32;
        // f = f_in/M with an arbitrary phase, amplitude 1
        let out = run(m, 16, |n| (TAU * n as f32 / m as f32 + 0.7).sin());
        // skip the settle, measure the tail
        let peak = out[4..].iter().fold(0.0f32, |p, y| p.max(y.abs()));
        assert!(
            peak < 0.02,
            "alias at f_out must be ≥34 dB down, got peak {peak}"
        );
    }

    /// Reconfiguration must fully reset state (no bleed from the old M).
    #[test]
    fn reconfigure_resets() {
        let mut cic = CicDecimator2::<1>::new();
        cic.configure(2);
        cic.push(&[100.0]);
        cic.configure(2);
        // a fresh m=2 run over a constant must converge to the constant
        let mut last = None;
        for _ in 0..6 {
            if let Some(y) = cic.push(&[1.0]) {
                last = Some(y[0]);
            }
        }
        assert!(
            (last.unwrap() - 1.0).abs() < 1e-6,
            "stale state bled through"
        );
    }
}
