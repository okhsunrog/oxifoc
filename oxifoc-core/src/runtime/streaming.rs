//! Fast telemetry producer side and the fault topic.
//!
//! ISR/sim writes `FastTelemetry` structs into a lock-free fixed-record queue
//! at the decimated rate. The only consumer is the Packed28 session stream
//! ([`super::raw_stream`]); streaming is disabled until a host arms a session.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::types::HallAngle;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use ergot::net_stack::NetStackHandle;

mod fast_queue;
use fast_queue::FastTelemetryQueue;

use crate::foc::controller::FocOutput;
use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::icd::FaultTopic;
use crate::state::{MotorControlState, update_telemetry};
use crate::timer::Timer;
use crate::types::FastTelemetry;
use crate::units::AngleRepr;

/// Typed, lock-free fixed-record queue for ISR → async telemetry transfer.
///
/// Queue capacity absorbs scheduling and transport pauses. Host builds have
/// a deeper queue to tolerate the host scheduler's jitter.
#[cfg(feature = "std")]
pub(super) static FAST_TELEM_Q: FastTelemetryQueue<65536> = FastTelemetryQueue::new();
#[cfg(not(any(feature = "std", feature = "ccm-telemetry")))]
pub(super) static FAST_TELEM_Q: FastTelemetryQueue<4096> = FastTelemetryQueue::new();

// Churrasco starts with a zeroed backing buffer, zero indices and no active
// grants. F405 startup clears this section before either queue owner runs.
#[cfg(all(not(feature = "std"), feature = "ccm-telemetry"))]
#[unsafe(link_section = ".ccm.bss.fast_telemetry")]
pub(super) static FAST_TELEM_Q: FastTelemetryQueue<{ 48 * 1024 }> = FastTelemetryQueue::new();

/// Telemetry producer state shared by the control ISR and the session
/// tasks. One static, so the ISR reaches every field from one base address
/// (see [`crate::state::IsrFlags`]).
pub struct FastProducer {
    /// Decimation period: the ISR queues a sample every `period` FOC cycles.
    /// 0 = streaming disabled. Set by the raw session server on Arm and
    /// cleared on Stop or failure.
    pub period: AtomicU32,
    /// Modulo counter selecting every `period`-th cycle. The frame is
    /// unfiltered raw ADC data; the host does any anti-alias decimation.
    pub(super) decimation: AtomicU32,
    /// Samples the ISR could not queue in the current session (queue full).
    pub(super) producer_drops: AtomicU32,
    /// Loss attribution, reported and reset by a board's 1-Hz stats task:
    /// successful ISR queue commits,
    pub push_ok: AtomicU32,
    /// ISR grant failures (queue full),
    pub push_drops: AtomicU32,
    /// and frames the session stream read from the queue.
    pub read_ok: AtomicU32,
}

pub static FAST_PRODUCER: FastProducer = FastProducer {
    period: AtomicU32::new(0),
    decimation: AtomicU32::new(0),
    producer_drops: AtomicU32::new(0),
    push_ok: AtomicU32::new(0),
    push_drops: AtomicU32::new(0),
    read_ok: AtomicU32::new(0),
};

// Serializes session control with the task-side block stream, without
// blocking the FOC interrupt.
pub(super) static FAST_TELEM_GATE: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());
#[cfg(test)]
pub(super) static FAST_TELEM_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Command-path counters (host→device direction), 1 Hz-reported by the
/// device stats task. Bracket the RX pipeline: requests that reached the
/// MotorEndpoint server vs SetModes actually drained by the ISR — a healthy
/// link under host affirms shows ~20/s on both; zeros while the host is
/// affirming localize a silent drop to the transport/routing in between.
pub mod cmd_stats {
    use core::sync::atomic::AtomicU32;
    /// MotorEndpoint requests received by the command server.
    pub static MOTOR_REQS: AtomicU32 = AtomicU32::new(0);
    /// Requests processed by the server whose reply the local route refused.
    pub static MOTOR_ACK_SEND_ERRORS: AtomicU32 = AtomicU32::new(0);
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

    let period = FAST_PRODUCER.period.load(Ordering::Relaxed);
    if period == 0 {
        return;
    }
    // Raw diagnostic frame: emit one sample every `period`-th FOC cycle. No CIC
    // anti-alias — the currents ship as raw ADC counts and the host applies
    // calibration/decimation downstream, so there is nothing to filter here.
    if FAST_PRODUCER
        .decimation
        .fetch_add(1, Ordering::Relaxed)
        .is_multiple_of(period)
    {
        let telem = build_fast_telemetry(&adc, &foc, hall, pole_pairs, seq);
        push_fast_telemetry(&telem);
    }
}

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
    hall: Option<HallSnapshot>,
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
        foc.velocity.value / f32::from(pole_pairs) * (60.0 / TAU)
    } else {
        0.0
    };
    FastTelemetry {
        ia: adc.ia,
        ib: adc.ib,
        ic: adc.ic,
        vbus: FastTelemetry::pack_vbus(adc.vbus_mv as f32 * 0.001),
        angle: FastTelemetry::pack_angle(foc.angle.value),
        hall_angle: HallAngle::from_radians(hall.map(|h| h.angle.radians())),
        vd: FastTelemetry::pack_volt(foc.vd.value),
        vq: FastTelemetry::pack_volt(foc.vq.value),
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
    if FAST_TELEM_Q.push(telem).is_ok() {
        FAST_PRODUCER.push_ok.fetch_add(1, Ordering::Relaxed);
    } else {
        FAST_PRODUCER.push_drops.fetch_add(1, Ordering::Relaxed);
        FAST_PRODUCER.producer_drops.fetch_add(1, Ordering::Relaxed);
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
