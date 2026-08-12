//! Fast telemetry streaming via lock-free bbqueue.
//!
//! ISR/sim writes `FastTelemetry` structs into a bbqueue at the decimated rate.
//! The async streaming task wakes on a timer, drains the queue, and broadcasts
//! batches of up to 8 samples as a single ergot topic message.
//!
//! Streaming is disabled by default — the host must send `TelemetryConfig`
//! with a nonzero `fast_hz` to start.

use core::mem::size_of;
use core::sync::atomic::{AtomicU32, Ordering};

use bbqueue::nicknames::Churrasco;
use ergot::net_stack::NetStackHandle;

use crate::foc::controller::FocOutput;
use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::icd::FastTelemetryTopic;
use crate::icd::FaultTopic;
use crate::state::{MotorControlState, update_telemetry};
use crate::timer::Timer;
use crate::types::FastTelemetry;
use crate::types::FastTelemetryBatch;

/// Lock-free queue for ISR → async telemetry transfer.
///
/// ~22 frames of 46 bytes (44 data + 2 header) fit in 1024 bytes.
/// Churrasco = Inline + Atomic + Polling + Static reference.
///
/// Firmware keeps the queue small (2 KB ≈ 42 frames — embassy timer
/// wakeups are microsecond-accurate, so the drain cadence holds). Host
/// builds (virtual device, sims) get a deep queue: tokio sleep jitter
/// reaches tens of milliseconds under load, which at 10–20 kHz overruns a
/// 42-frame buffer and fakes telemetry loss the real device doesn't have.
#[cfg(feature = "std")]
pub static FAST_TELEM_Q: Churrasco<65536> = Churrasco::new();
#[cfg(not(feature = "std"))]
pub static FAST_TELEM_Q: Churrasco<2048> = Churrasco::new();

/// Decimation period: ISR writes to bbqueue every `period` FOC cycles.
/// 0 = streaming disabled. Set by `telemetry_config_server` when host
/// sends `TelemetryConfig { fast_hz }`.
pub static FAST_TELEM_PERIOD: AtomicU32 = AtomicU32::new(0);

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
    /// Stream-task broadcasts accepted by the stack.
    pub static BCAST_OK: AtomicU32 = AtomicU32::new(0);
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
    /// Max command staleness (µs since the last drained `SetMode`) observed
    /// while a deadman-covered mode was active, per stats window. The
    /// deadman's own margin meter: healthy 50 ms affirms keep this ≲70 000;
    /// a spike toward 150 000 in the trip second says the silence built up
    /// device-visible (frames not arriving/being drained) as opposed to the
    /// host merely missing responses.
    pub static STALENESS_MAX_US: AtomicU32 = AtomicU32::new(0);
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
    pole_pairs: u8,
    seq: u32,
) {
    update_telemetry(state_mutex, adc, hall, foc);

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
/// Drains `FAST_TELEM_Q` on a timer and broadcasts [`FastTelemetryBatch`]
/// via the ergot topic system. When streaming is disabled
/// (`FAST_TELEM_PERIOD == 0`), polls periodically waiting for the host to
/// enable it. Batch capacity is fixed at [`FAST_BATCH_SAMPLES`] — raw-Pod
/// batches have a compile-time wire size, sized once against the smallest
/// device MTU instead of per-board const generics.
pub async fn fast_telemetry_stream<NS, T: Timer>(stack: NS, foc_freq_hz: u32)
where
    NS: NetStackHandle + Clone,
{
    let cons = FAST_TELEM_Q.framed_consumer();

    #[cfg(feature = "log")]
    log::info!("fast_telemetry_stream: waiting for host to enable streaming");

    loop {
        let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);

        if period == 0 {
            // Streaming disabled — check again in 10ms. The bbqueue holds
            // ~40 frames, so the enable→first-drain latency must stay well
            // under the time the queue takes to fill at the highest rate
            // or the capture starts with a guaranteed gap.
            T::after_millis(10).await;
            continue;
        }

        // Sleep for HALF a batch at (foc_freq_hz / period) Hz: draining at
        // exactly one batch per buffer-full leaves zero slack for timer
        // jitter (at 5 kHz the ~40-frame queue is only 8 ms deep — one
        // late wakeup drops frames). Half-batch cadence doubles the margin
        // for a few hundred extra wakeups per second at worst.
        let sample_hz = foc_freq_hz / period;
        let interval_us = if sample_hz > 0 {
            ((FAST_BATCH_SAMPLES as u64 / 2).max(1) * 1_000_000) / u64::from(sample_hz)
        } else {
            100_000 // fallback 100ms
        };

        T::after_micros(interval_us).await;

        // Drain bbqueue into batches and broadcast. The queue already holds
        // raw Pod frame bytes and the batch ships raw Pod bytes, so the copy
        // below is the whole per-sample "serialization".
        loop {
            let mut batch = FastTelemetryBatch::new();

            while !batch.is_full() {
                match cons.read() {
                    Ok(grant) => {
                        if batch.push_bytes(&grant) {
                            fast_telem_stats::READ_OK.fetch_add(1, Ordering::Relaxed);
                        } else {
                            fast_telem_stats::READ_BADLEN.fetch_add(1, Ordering::Relaxed);
                        }
                        grant.release();
                    }
                    Err(_) => break, // queue empty
                }
            }

            if batch.is_empty() {
                break; // nothing left to send
            }

            let batch_full = batch.is_full();
            let _result = stack
                .stack()
                .topics()
                .broadcast::<FastTelemetryTopic>(&batch, None);

            if _result.is_ok() {
                fast_telem_stats::BCAST_OK.fetch_add(1, Ordering::Relaxed);
            } else {
                fast_telem_stats::BCAST_FAILS.fetch_add(1, Ordering::Relaxed);
            }

            #[cfg(feature = "log")]
            if _result.is_err() {
                log::warn!("fast_telemetry broadcast failed: {_result:?}");
            }

            // If we got fewer than BATCH, the queue is drained
            if !batch_full {
                break;
            }
        }
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
