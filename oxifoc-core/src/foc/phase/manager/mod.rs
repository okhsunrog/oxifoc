//! Phase manager for FOC control
//!
//! Manages multiple angle sources (Hall, Encoder, Observer) and provides
//! a unified interface to FocDriver via the PhaseProvider trait.

mod accessors;
#[cfg(feature = "storage")]
mod config;
mod constructors;
mod hall;
mod open_loop;
mod select;
mod tracker;
mod update;

#[cfg(test)]
mod tests;

#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::phase::observer::Observer;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::phase::source::PhaseSource;
use crate::foc::phase::startup::SensorlessStartup;
use crate::foc::sensors::AngleSensor;
use crate::foc::sensors::NoSensor;
use crate::foc::trig::LibmSinCos;
use crate::foc::trig::SinCos;
use heapless::Vec as HeaplessVec;

/// Hysteresis band for the sharp HFI↔sensor crossovers (fraction of the
/// switch velocity). Switch up at `switch_vel`, back down only below
/// `switch_vel × (1 − this)` — otherwise velocity noise around the single
/// threshold chatters between two discontinuous angle sources.
pub const CROSSOVER_HYSTERESIS: f32 = 0.2;

/// Hysteresis band (V) of the voltage-based HfiToObserverVolts crossover —
/// MESC hardcodes the same 1 V between "carrier off" and "carrier on".
pub const HFI_VOLTS_HYSTERESIS: f32 = 1.0;

// ============================================================================
// Hall Health Tracking (VESC-style)
// ============================================================================

/// Hall sensor health status
///
/// Used to detect Hall sensor failures and trigger fallback to observer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HallHealth {
    /// Hall working normally
    #[default]
    Ok,
    /// Hall data is stale (no edges for timeout period)
    Stale,
    /// Hall returning invalid states (0 or 7)
    Invalid,
    /// Hall not configured/present
    NotPresent,
}

// ============================================================================
// Open-Loop Override (VESC-style startup/recovery)
// ============================================================================

/// Damping-path low-pass (s) of the commutation phase tracker: turns the
/// PI-tracker's zero into a pole–zero pair so the 35–100 Hz mid-band
/// estimate wobble keeps second-order rolloff (~×0.2 at 60 Hz with the
/// defaults) while the 8–20 Hz rotor-hunting band keeps its damping
/// (~×0.98 follow at 14 Hz).
const TRACKER_KD_TAU_S: f32 = 0.005;

/// Hard bound (rad) on how far the commutation frame may trail the raw
/// estimate — the geometric guarantee against the flat-top/pull-out ride
/// (see the clamp block in [`PhaseManager::tracker_output`]). 0.6 rad =
/// 34°: torque per amp ≥ 0.83 of aligned, stiffness slope still steep.
const TRACKER_MAX_LAG_RAD: f32 = 0.6;

/// Frequency catch-up time constant (s) while the load-angle clamp
/// binds: fast enough to release the clamp within tens of ms after the
/// transient, slow enough not to import the wobble velocity.
///
/// The catch-up TARGET carries an `a_est·τ` lead (see the clamp block):
/// a first-order pull toward a RAMPING velocity has a structural lag of
/// ω̇·τ — at 20 ms and the measured 9 300 el rad/s² free spin-up that
/// was ~186 rad/s of standing frequency deficit, θ re-hit the clamp
/// within milliseconds and the tracker RODE the clamp for the entire
/// acceleration (bench 2026-07-07: drive-vs-estimate gap pinned at
/// 0.62–0.67 rad the whole climb = commutating ~35° behind). With the
/// lead, the pull's fixed point is ramp-consistent once the a_est
/// feedforward converges (~3× TRACKER_FF_TAU_S ≈ 90 ms) and the clamp
/// releases into soft tracking — an entry transient instead of a
/// standing regime. (A shorter τ instead — 2 ms — released the ride
/// too, but every wobble-peak clamp GRAZE then slammed the frequency
/// to the instantaneous wobble velocity: the 60 Hz attenuation test
/// read ratio 1.14, worse than no tracker.)
const TRACKER_CATCHUP_TAU_S: f32 = 0.02;

/// Acceleration-feedforward filter time constant (s): slow enough to
/// reject the mid-band wobble (33 rad/s corner), fast enough to pick a
/// punch's acceleration trend up within ~2τ (the load-angle clamp covers
/// the pickup window). Without this feedforward the tracker is pure
/// type-2 and its lag ω̇/ωn² under the bench accelerations
/// (0.5–5.6 k rad/s²) lands on the torque-curve flat top for any ωn soft
/// enough to filter the wobble — the freq-led dossier's tension, now
/// resolved by feeding the trend instead of stiffening the loop.
const TRACKER_FF_TAU_S: f32 = 0.03;

/// Rotor-hunting damper: fraction of the low-passed RELATIVE velocity
/// (estimate vs frame) fed into the angle advance, and its filter (s).
/// Above ωn the frame is quasi-synchronous and the rotor's hunt mode
/// (8–20 Hz) is undamped by the kp/kd paths — those act on POSITION
/// (stiffness); damping needs the velocity variable. Bench 2026-07-08
/// (captures/trk-dbg-2k-1): with the damper omitted the estimate-vs-
/// frame gap swung ±45–73° sustained and the ride limit-cycled with
/// trust-loss restarts. Same structure as the freq-led hunting damper
/// that measurably slowed the swing there; the τ keeps the 35–100 Hz
/// wobble out of the damper (×0.7 at 14 Hz, ×0.2 at 60 Hz).
const TRACKER_HUNT_DAMP: f32 = 0.5;
const TRACKER_HUNT_TAU_S: f32 = 0.012;

/// Second-order phase tracker over the observer estimate — the freq-led
/// REDESIGN (2026-07-08).
///
/// The frequency-led filter it replaces was structurally an I/f drive:
/// its slew-limited frame imposed the rotation, the rotor flux locked
/// onto the commanded current vector, and the estimate therefore sat
/// ~90° off the frame BY CONSTRUCTION (bench dossier in docs/TODO.md —
/// no frame-side pull can close a gap the plant re-establishes; torque
/// ceiling ~28 k erpm el vs the ~50 k vbus ceiling of observer-frame
/// commutation).
///
/// This tracker keeps the TORQUE AXIS with the observer: a classic
/// critically-damped PLL on the estimate angle —
///   ω̇ = ωn²·Δ,   θ̇ = ω + 2ζωn·LPF(Δ)
/// with Δ = the estimate-vs-frame gap. Δ → 0 structurally (type-2: zero
/// steady-state error at constant speed, lag ω̇/ωn² under acceleration —
/// self-limiting through the torque curve, ~8° at the bench cruise
/// accel). No slew limiter, no pull clamp, no speed bands: wobble is
/// rejected quadratically above ωn, hunting is followed (= damped)
/// below it.
#[derive(Clone, Copy, Default)]
struct PhaseTracker {
    /// ωn² (1/s²); 0 = tracker disabled (raw observer commutation).
    kp: f32,
    /// 2ζωn (1/s) — damping-path gain (applied to the low-passed Δ).
    kd: f32,
    /// Engaged — cleared whenever the startup sequencer owns commutation
    /// (re-seeds from the estimate at the next closed-loop cycle).
    active: bool,
    theta: f32,
    omega: f32,
    /// Low-passed Δ for the damping path (see TRACKER_KD_TAU_S).
    d_filt: f32,
    /// Slow-filtered estimate velocity (τ = TRACKER_FF_TAU_S) — the
    /// acceleration-feedforward tap.
    v_slow: f32,
    /// Low-passed relative velocity for the hunting damper (rad/s).
    hunt_filt: f32,
    /// Filtered estimate acceleration (rad/s², same τ): fed forward into
    /// the frequency integrator, it removes the type-2 lag for slow
    /// acceleration TRENDS while the τ keeps the 35–100 Hz wobble out.
    a_est: f32,
}

/// Open-loop override state for startup or Hall failure recovery
///
/// When Hall fails and observer isn't ready, the motor can be driven
/// in open-loop mode at minimum speed until the observer syncs.
#[derive(Clone, Copy, Debug, Default)]
pub struct OpenLoopOverride {
    /// Whether open-loop override is active
    pub active: bool,
    /// Current open-loop angle (radians)
    pub angle: f32,
    /// Target velocity for open-loop ramp (rad/s)
    pub velocity: f32,
    /// Time remaining in override mode (seconds)
    pub timer: f32,
}

/// Default open-loop override duration (seconds)
pub const DEFAULT_OPENLOOP_TIME: f32 = 0.5;

/// Default minimum open-loop velocity (electrical rad/s, ~500 eRPM)
pub const DEFAULT_OPENLOOP_MIN_VEL: f32 = 52.0;

// ============================================================================
// Phase Faults
// ============================================================================

/// Phase estimation fault
///
/// Faults are set when the phase manager detects problems and cleared
/// when the issue is resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseFault {
    /// Hall sensor timeout (no edges for configured timeout period)
    HallTimeout,
    /// Hall sensor returning invalid state (0 or 7)
    HallInvalidState,
    /// Observer not converged when needed for fallback
    ObserverNotReady,
}

/// Phase manager for FOC control
///
/// Manages multiple angle sources and provides a unified interface.
/// Implements `PhaseProvider` for use with `FocDriver`.
///
/// # Type Parameters
/// * `H` - Hall sensor type (implements `AngleSensor`, optionally `HallSensorTrait`)
/// * `E` - Encoder type (implements `AngleSensor`)
///
/// # Example
/// ```rust,ignore
/// // Hall only
/// let phase = PhaseManager::with_hall(hall_sensor);
///
/// // Hall + sensorless hybrid
/// let mut phase = PhaseManager::with_hall(hall_sensor);
/// phase.set_observer(Observer::BackEmf(BackEmfObserver::new(r, l, lambda)));
/// phase.set_source(PhaseSource::HallToObserver {
///     blend_low: 300.0,
///     blend_high: 600.0,
/// })?;
/// ```
pub struct PhaseManager<H = NoSensor, E = NoSensor, S = LibmSinCos>
where
    H: AngleSensor,
    E: AngleSensor,
    S: SinCos,
{
    // Hardware sensors
    hall: H,
    encoder: E,

    // Software estimators. Two slots run concurrently so the HfiToX
    // crossovers can hand over between them: `hfi` covers zero/low speed
    // (needs carrier injection), `observer` covers medium/high speed.
    // `S` is the HFI sin/cos backend (CORDIC on G4, FastSinCos on F405);
    // constructors pin the `LibmSinCos` default — firmware rebinds via
    // `with_sincos`.
    observer: Observer,
    #[cfg(feature = "hfi")]
    hfi: Option<HfiObserver<S>>,
    // `S` (the HFI sin/cos backend) is only consumed by the `hfi` field; keep
    // the type parameter live when HFI is compiled out.
    #[cfg(not(feature = "hfi"))]
    _sincos: core::marker::PhantomData<S>,

    // Configuration
    source: PhaseSource,

    // State
    output: PhaseOutput,
    manual_angle: f32,
    open_loop_angle: f32,
    open_loop_velocity: f32,

    // Timebase
    ticks_per_sec: u64,

    // Hall health tracking
    hall_health: HallHealth,

    // Open-loop override state (for Hall failure recovery)
    open_loop_override: OpenLoopOverride,

    // Sensorless cold-start sequencer (align → ramp → handoff). Distinct from
    // `open_loop_override` above: that is the reactive hall-dropout nudge from
    // a known angle; this is the proactive from-standstill start that a pure
    // back-EMF observer needs (it can't commutate below ~READY_MIN_VELOCITY).
    startup: SensorlessStartup,

    // Commutation phase tracker for the pure-Observer source (see
    // [`Self::set_phase_tracker`]). Bundled so the many literal
    // constructors stay one line each.
    tracker: PhaseTracker,

    // Hysteresis memory for the HfiToX crossovers: true = running on the
    // high-speed source (observer/hall/encoder), false = on HFI.
    crossover_latched: bool,

    // Whether HFI (carrier + demod update) ran last cycle — a rising edge
    // restarts the demod filters so stale pre-pause state can't masquerade
    // as confidence (see HfiObserver::restart_demod).
    #[cfg(feature = "hfi")]
    hfi_was_active: bool,

    // |vq − R·iq| of the last update — back-EMF share of the drive voltage,
    // the regime signal for the voltage-based HFI crossover.
    bemf_proxy_v: f32,

    // Decimation counters for the ~2.4 Hz ISR traces (observer internals /
    // startup-hold convergence). Plain fields — ISR-only state; statics
    // were per-cycle LDREX/STREX atomics (tier-3 PC sampling).
    obs_trace_ticks: u32,
    hold_trace_ticks: u32,

    // Fault tracking
    faults: HeaplessVec<PhaseFault, 4>,
}
