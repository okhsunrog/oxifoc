//! Phase manager for FOC control
//!
//! Manages multiple angle sources (Hall, Encoder, Observer) and provides
//! a unified interface to FocDriver via the PhaseProvider trait.

use core::f32::consts::TAU;

use heapless::Vec as HeaplessVec;

#[cfg(feature = "hfi")]
use super::observer::HfiObserver;
use super::observer::{Observer, ObserverInput};
use super::provider::{PhaseInput, PhaseOutput, PhaseProvider};
use super::source::{PhaseSource, PhaseSourceError};
use super::startup::{ConfirmResult, DeadshortResult, SensorlessStartup, StartupPhase};
use crate::foc::fast_math::sqrtf;
use crate::foc::hall_calibration::HallCalibrationResult;
use crate::foc::hall_sensor::HallFaultKind;
use crate::foc::sensors::{AngleSample, AngleSensor, HallSensorTrait, NoSensor};
use crate::foc::transforms::park;
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::foc::{angle_difference, wrap_angle};
#[cfg(feature = "storage")]
use crate::storage::RuntimeConfig;

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
    /// Ticks when Hall failure was first detected
    hall_failure_ticks: Option<u64>,

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

// ============================================================================
// Constructors
// ============================================================================

impl PhaseManager<NoSensor, NoSensor> {
    /// Create a sensorless phase manager (observer only)
    pub fn sensorless() -> Self {
        Self {
            hall: NoSensor,
            encoder: NoSensor,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Manual,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }
}

impl<H: AngleSensor> PhaseManager<H, NoSensor> {
    /// Create a phase manager with Hall sensor
    pub fn with_hall(hall: H) -> Self {
        Self {
            hall,
            encoder: NoSensor,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Hall,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::Ok,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }

    /// Add an encoder to the phase manager
    pub fn with_encoder<E2: AngleSensor>(self, encoder: E2) -> PhaseManager<H, E2> {
        PhaseManager {
            hall: self.hall,
            encoder,
            observer: self.observer,
            #[cfg(feature = "hfi")]
            hfi: self.hfi,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            hall_failure_ticks: self.hall_failure_ticks,
            open_loop_override: self.open_loop_override,
            startup: self.startup,
            tracker: self.tracker,
            crossover_latched: self.crossover_latched,
            #[cfg(feature = "hfi")]
            hfi_was_active: self.hfi_was_active,
            bemf_proxy_v: self.bemf_proxy_v,
            obs_trace_ticks: self.obs_trace_ticks,
            hold_trace_ticks: self.hold_trace_ticks,
            faults: self.faults,
        }
    }
}

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Rebind the HFI sin/cos backend (state-preserving). Firmware calls
    /// this right after construction: CORDIC on G4, FastSinCos on F405.
    pub fn with_sincos<S2: SinCos>(self) -> PhaseManager<H, E, S2> {
        PhaseManager {
            hall: self.hall,
            encoder: self.encoder,
            observer: self.observer,
            #[cfg(feature = "hfi")]
            hfi: self.hfi.map(HfiObserver::with_sincos),
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            hall_failure_ticks: self.hall_failure_ticks,
            open_loop_override: self.open_loop_override,
            startup: self.startup,
            tracker: self.tracker,
            crossover_latched: self.crossover_latched,
            #[cfg(feature = "hfi")]
            hfi_was_active: self.hfi_was_active,
            bemf_proxy_v: self.bemf_proxy_v,
            obs_trace_ticks: self.obs_trace_ticks,
            hold_trace_ticks: self.hold_trace_ticks,
            faults: self.faults,
        }
    }
}

impl<E: AngleSensor> PhaseManager<NoSensor, E> {
    /// Create a phase manager with encoder only
    pub fn with_encoder_only(encoder: E) -> Self {
        Self {
            hall: NoSensor,
            encoder,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Encoder,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }
}

// ============================================================================
// Common implementation for all PhaseManager variants
// ============================================================================

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Set the timebase for tick conversions
    pub fn set_ticks_per_sec(&mut self, ticks_per_sec: u64) {
        self.ticks_per_sec = ticks_per_sec.max(1);
    }

    /// Get current phase source
    pub fn source(&self) -> PhaseSource {
        self.source
    }

    /// Set phase source
    ///
    /// Returns error if the requested source is not available.
    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError> {
        // Validate source availability
        if source.requires_hall() && !self.has_hall() {
            return Err(PhaseSourceError::HallNotAvailable);
        }
        if source.requires_encoder() && !self.has_encoder() {
            return Err(PhaseSourceError::EncoderNotAvailable);
        }
        if source.requires_observer() && !self.observer.is_configured() {
            return Err(PhaseSourceError::ObserverNotConfigured);
        }
        // HFI sources need the estimator that actually generates a carrier:
        // without one in the dedicated slot, injection() stays zero and
        // the source would silently never produce an estimate. With HFI
        // compiled out (`hfi` feature off) the variant stays in the wire enum
        // but is always rejected here.
        if source.requires_hfi() {
            #[cfg(feature = "hfi")]
            if self.hfi.is_none() {
                return Err(PhaseSourceError::HfiNotConfigured);
            }
            #[cfg(not(feature = "hfi"))]
            return Err(PhaseSourceError::HfiNotConfigured);
        }

        self.source = source;
        // Crossover memory belongs to the previous source's thresholds.
        self.crossover_latched = false;
        // The open-loop override likewise belongs to the source that armed
        // it (only Hall arms it). Without this, switching to a non-hall
        // source while the override is live strands it: the deactivation
        // paths are all `requires_hall()`-gated, so `angle_trustworthy()`
        // for Encoder/EncoderToObserver — which reads `!override.active` —
        // would stay false forever on a healthy encoder. If the new source
        // still can't produce an angle, `compute_phase_with_fallback`
        // re-arms the override next cycle.
        self.deactivate_open_loop_override();
        // A live cold-start sequence belongs to the source that began it.
        self.startup.deactivate();
        Ok(())
    }

    /// Set observer
    /// Configure the back-EMF observer's eddy L(f) ladder (no-op without
    /// a configured observer) — see `BackEmfObserver::with_eddy_ladder`.
    pub fn set_observer_eddy_ladder(&mut self, delta_l: f32, tau_s: f32) {
        if let Observer::BackEmf(o) = &mut self.observer {
            o.set_eddy_ladder(delta_l, tau_s);
        }
    }

    /// Enable the commutation phase tracker for the pure-Observer source
    /// (see [`PhaseTracker`]): `omega_n` = tracker natural frequency
    /// (el rad/s — wobble above it is rejected quadratically, hunting
    /// below it is followed/damped), `zeta` = damping ratio (1.0 =
    /// critical). `omega_n <= 0` disables (raw observer output).
    pub fn set_phase_tracker(&mut self, omega_n: f32, zeta: f32) {
        if omega_n.is_finite() && zeta.is_finite() && zeta >= 0.0 {
            let wn = omega_n.max(0.0);
            self.tracker.kp = wn * wn;
            self.tracker.kd = 2.0 * zeta * wn;
            self.tracker.active = false;
        }
    }

    /// Configure the back-EMF observer's physics acceleration prior
    /// (no-op without a configured observer) — see
    /// `BackEmfObserver::set_accel_prior`.
    pub fn set_observer_accel_prior(&mut self, floor_el: f32, per_amp_el: f32) {
        self.observer.set_accel_prior(floor_el, per_amp_el);
    }

    /// Override the back-EMF observer's PLL gains (no-op without a
    /// configured observer) — bench tuning of the slip-kick loop gain.
    pub fn set_observer_pll_gains(&mut self, kp: f32, ki: f32) {
        if let Observer::BackEmf(o) = &mut self.observer {
            o.set_pll_gains(kp, ki);
        }
    }

    /// Seed the back-EMF observer from an external (angle, velocity) —
    /// the same trusted-handoff semantics the deadshort catch uses
    /// internally. Sim/bench forensics: injecting a phase kick mid-drive
    /// (seed at `phase + Δ`) emulates one accumulated slip-kick.
    pub fn seed_observer(&mut self, angle: f32, velocity: f32) {
        self.observer.seed(angle, velocity);
    }

    pub fn set_observer(&mut self, observer: Observer) {
        self.observer = observer;
    }

    /// Get observer reference
    pub fn observer(&self) -> &Observer {
        &self.observer
    }

    /// Get mutable observer reference
    pub fn observer_mut(&mut self) -> &mut Observer {
        &mut self.observer
    }

    /// Set the HFI estimator (dedicated low-speed slot)
    #[cfg(feature = "hfi")]
    pub fn set_hfi_observer(&mut self, hfi: HfiObserver<S>) {
        self.hfi = Some(hfi);
    }

    /// Get HFI estimator reference
    #[cfg(feature = "hfi")]
    pub fn hfi_observer(&self) -> Option<&HfiObserver<S>> {
        self.hfi.as_ref()
    }

    /// Get mutable HFI estimator reference
    #[cfg(feature = "hfi")]
    pub fn hfi_observer_mut(&mut self) -> Option<&mut HfiObserver<S>> {
        self.hfi.as_mut()
    }

    /// Current HFI estimate as a phase output (None if no HFI configured).
    /// Always `None` when HFI is compiled out — the Hfi* `PhaseSource` arms
    /// that call this are then unreachable (rejected by `set_source`).
    #[cfg(feature = "hfi")]
    fn hfi_output(&self) -> Option<PhaseOutput> {
        self.hfi.as_ref().map(|h| PhaseOutput {
            angle: h.phase(),
            velocity: h.velocity(),
        })
    }

    #[cfg(not(feature = "hfi"))]
    fn hfi_output(&self) -> Option<PhaseOutput> {
        None
    }

    /// Check if Hall sensor is available.
    ///
    /// Presence is structural ([`AngleSensor::is_present`]), not "has data":
    /// a healthy hall on a rotor that has not moved yet has no sample and no
    /// errors, and gating on data made `set_source(Hall…)` from stored config
    /// fail spuriously on cold start.
    pub fn has_hall(&self) -> bool {
        self.hall.is_present()
    }

    /// Check if encoder is available (structural presence, see [`Self::has_hall`])
    pub fn has_encoder(&self) -> bool {
        self.encoder.is_present()
    }

    /// Set manual angle (for Manual source)
    pub fn set_manual_angle(&mut self, angle: f32) {
        self.manual_angle = wrap_angle(angle);
    }

    /// Get manual angle
    pub fn manual_angle(&self) -> f32 {
        self.manual_angle
    }

    /// Set open-loop velocity (for OpenLoop source)
    pub fn set_open_loop_velocity(&mut self, velocity: f32) {
        self.open_loop_velocity = velocity;
    }

    /// Get open-loop velocity
    pub fn open_loop_velocity(&self) -> f32 {
        self.open_loop_velocity
    }

    /// Set open-loop angle directly
    pub fn set_open_loop_angle(&mut self, angle: f32) {
        self.open_loop_angle = wrap_angle(angle);
    }

    /// Get open-loop angle
    pub fn open_loop_angle(&self) -> f32 {
        self.open_loop_angle
    }

    /// Get mutable reference to Hall sensor
    pub fn hall_mut(&mut self) -> &mut H {
        &mut self.hall
    }

    /// Get reference to Hall sensor
    pub fn hall(&self) -> &H {
        &self.hall
    }

    /// Get mutable reference to encoder
    pub fn encoder_mut(&mut self) -> &mut E {
        &mut self.encoder
    }

    /// Get reference to encoder
    pub fn encoder(&self) -> &E {
        &self.encoder
    }

    // ========================================================================
    // Hall Health and Fault Tracking
    // ========================================================================

    /// Get Hall sensor health status
    pub fn hall_health(&self) -> HallHealth {
        self.hall_health
    }

    /// Get active faults
    pub fn faults(&self) -> &[PhaseFault] {
        &self.faults
    }

    /// Check if a specific fault is active
    pub fn has_fault(&self, fault: PhaseFault) -> bool {
        self.faults.contains(&fault)
    }

    /// Clear all faults
    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    /// Set a fault (adds if not already present)
    fn set_fault(&mut self, fault: PhaseFault) {
        if !self.faults.contains(&fault) {
            let _ = self.faults.push(fault);
        }
    }

    /// Clear a specific fault
    fn clear_fault(&mut self, fault: PhaseFault) {
        self.faults.retain(|f| *f != fault);
    }

    /// Get open-loop override state
    pub fn open_loop_override(&self) -> &OpenLoopOverride {
        &self.open_loop_override
    }

    /// True while the cold-start sequencer owns commutation.
    pub fn is_starting(&self) -> bool {
        self.startup.is_active()
    }

    /// The cold-start sequencer (diagnostics).
    pub fn startup(&self) -> &SensorlessStartup {
        &self.startup
    }

    /// Check if open-loop override is active
    pub fn is_open_loop_override_active(&self) -> bool {
        self.open_loop_override.active
    }

    /// Activate open-loop override with default parameters
    ///
    /// This is called automatically when Hall fails and observer isn't ready.
    /// Can also be called manually for startup.
    pub fn activate_open_loop_override(&mut self, initial_angle: f32, velocity: f32) {
        self.open_loop_override.active = true;
        self.open_loop_override.angle = wrap_angle(initial_angle);
        self.open_loop_override.velocity = velocity;
        self.open_loop_override.timer = DEFAULT_OPENLOOP_TIME;
    }

    /// Deactivate open-loop override
    pub fn deactivate_open_loop_override(&mut self) {
        self.open_loop_override.active = false;
        self.open_loop_override.timer = 0.0;
    }

    /// Update Hall health status from sample availability and staleness.
    fn update_hall_health(&mut self, sample_valid: bool, stale: bool, now_ticks: u64) {
        // Skip if Hall is not configured
        if matches!(self.hall_health, HallHealth::NotPresent) {
            return;
        }

        if sample_valid && !stale {
            // Hall is working - clear failures
            if self.hall_health != HallHealth::Ok {
                self.hall_health = HallHealth::Ok;
                self.hall_failure_ticks = None;
                self.clear_fault(PhaseFault::HallTimeout);
                self.clear_fault(PhaseFault::HallInvalidState);
            }
        } else {
            // Hall failed - track when failure started
            if self.hall_failure_ticks.is_none() {
                self.hall_failure_ticks = Some(now_ticks);
            }

            if stale {
                // Edges stopped while the rotor was spinning (velocity-
                // adaptive check in the sensor) — dead cable / dead sensor.
                self.hall_health = HallHealth::Stale;
                self.set_fault(PhaseFault::HallTimeout);
            } else {
                // No valid sample at all (invalid 0/7 states, never seen an
                // edge, ...).
                self.hall_health = HallHealth::Invalid;
                self.set_fault(PhaseFault::HallInvalidState);
            }
        }
    }

    /// Try to get observer output for fallback, with open-loop override as last resort
    fn try_observer_fallback(&mut self) -> Option<PhaseOutput> {
        // Gate on is_ready(), not phase().is_some(): every *configured*
        // observer returns a phase, including one frozen at 0 with zero
        // confidence. Handing it commutation on a sensor dropout would swap
        // a good angle for garbage.
        let ready_output = if self.observer.is_ready() {
            match (self.observer.phase(), self.observer.velocity()) {
                (Some(angle), Some(vel)) => Some(PhaseOutput {
                    angle,
                    velocity: vel,
                }),
                _ => None,
            }
        } else {
            None
        };

        if let Some(out) = ready_output {
            self.clear_fault(PhaseFault::ObserverNotReady);
            if self.open_loop_override.active {
                self.deactivate_open_loop_override();
            }
            Some(out)
        } else {
            // Observer not ready - use open-loop override if available
            self.set_fault(PhaseFault::ObserverNotReady);

            if !self.open_loop_override.active {
                // Activate open-loop override for recovery, continuing from
                // the last known output angle at minimum velocity. Signed by
                // the last known velocity (VESC-style): a board rolling
                // backward must not be spun forward by the recovery override.
                let dir = if self.output.velocity < 0.0 {
                    -1.0
                } else {
                    1.0
                };
                self.activate_open_loop_override(self.output.angle, dir * DEFAULT_OPENLOOP_MIN_VEL);
            }
            Some(PhaseOutput {
                angle: self.open_loop_override.angle,
                velocity: self.open_loop_override.velocity,
            })
        }
    }

    /// Update open-loop override state (advance angle, decrement timer).
    ///
    /// The override stays active until a real source (hall or a ready
    /// observer) takes over — `try_observer_fallback` deactivates it then.
    /// The timer is purely informational dwell time; it does NOT terminate
    /// the override, because dropping to a dead sensor at its expiry would
    /// be strictly worse than continuing open loop (VESC keeps its
    /// observer-override running for the same reason).
    fn update_open_loop_override(&mut self, dt: f32) {
        if self.open_loop_override.active {
            // Advance angle based on velocity
            self.open_loop_override.angle += self.open_loop_override.velocity * dt;
            self.open_loop_override.angle = wrap_angle(self.open_loop_override.angle);

            self.open_loop_override.timer = (self.open_loop_override.timer - dt).max(0.0);
        }
    }

    // ========================================================================
    // Internal phase computation
    // ========================================================================

    /// One step of the commutation phase tracker (see [`PhaseTracker`]).
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn tracker_output(&mut self, raw: PhaseOutput, dt: f32) -> PhaseOutput {
        let tr = &mut self.tracker;
        if !tr.active {
            // ANGLE from the estimate (zero initial gap — the same angle
            // step a plain confirmed handoff performs), FREQUENCY from the
            // last output (the instantaneous raw.velocity carries the full
            // estimate wobble; the outgoing ramp's frequency is already
            // ≈ the rotor's). The freq-led predecessor's `self.output`
            // angle seed froze the startup ramp's ~90° load angle into
            // cruise — see the PhaseTracker doc and docs/TODO.md dossier.
            tr.theta = raw.angle;
            tr.omega = self.output.velocity;
            tr.d_filt = 0.0;
            tr.v_slow = self.output.velocity;
            tr.a_est = 0.0;
            tr.hunt_filt = 0.0;
            tr.active = true;
        }
        let d = angle_difference(raw.angle, tr.theta);
        // Acceleration feedforward (see TRACKER_FF_TAU_S): the filtered
        // trend of the estimate's own velocity.
        let a_ff = (dt / TRACKER_FF_TAU_S).min(1.0);
        let v_slow_prev = tr.v_slow;
        tr.v_slow += a_ff * (raw.velocity - tr.v_slow);
        tr.a_est += a_ff * ((tr.v_slow - v_slow_prev) / dt - tr.a_est);
        // Frequency integrator: type-2 + acceleration feedforward — Δ → 0
        // at constant speed AND under slow acceleration trends; only the
        // trend ERROR (unmodeled load steps, the first ~2τ of a punch)
        // produces lag, and the hard clamp below bounds that.
        tr.omega += (tr.kp * d + tr.a_est) * dt;
        // Damping path on the low-passed Δ (see TRACKER_KD_TAU_S): the
        // LPF removes the PI zero's high-frequency leakage, NOT the
        // restoring force — the lagged-pull negative-damping trap of the
        // freq-led era applied lag to the whole stiffness; here the
        // un-lagged kp path carries it.
        let a_d = (dt / TRACKER_KD_TAU_S).min(1.0);
        tr.d_filt += a_d * (d - tr.d_filt);
        // Hunting damper (see TRACKER_HUNT_DAMP): yield with the rotor's
        // swing velocity so the quasi-synchronous frame extracts energy
        // from the hunt mode instead of storing it.
        let a_h = (dt / TRACKER_HUNT_TAU_S).min(1.0);
        tr.hunt_filt += a_h * (TRACKER_HUNT_DAMP * (raw.velocity - tr.omega) - tr.hunt_filt);
        tr.theta = wrap_angle(tr.theta + (tr.omega + tr.kd * tr.d_filt + tr.hunt_filt) * dt);
        // Hard load-angle bound. A soft tracker that filters the 35–100 Hz
        // mid-band wobble cannot also follow an unloaded max-torque punch
        // (ω̇ up to ~5.6 k rad/s² at 1.5 A): the type-2 lag ω̇/ωn²
        // self-consistently settles near the torque-curve flat top —
        // bench 2026-07-08: ωn=30 climbed at 1.36 k rad/s² with an ~77°
        // standing lag and died in the same dq OC as the freq-led
        // geometry. The clamp makes the failure mode impossible instead
        // of tuning it away: the frame never trails the estimate by more
        // than TRACKER_MAX_LAG_RAD (cos 34° = 0.83 of full torque, far
        // from the flat top). While it binds the output slides WITH the
        // raw estimate (wobble passes — acceptable: hard transients are
        // loud, the mid-band cycle is a light-load phenomenon) and the
        // frequency catches up on a fast τ so the clamp releases into
        // soft tracking.
        let d2 = angle_difference(raw.angle, tr.theta);
        if d2.abs() > TRACKER_MAX_LAG_RAD {
            let sign = if d2 > 0.0 { 1.0 } else { -1.0 };
            tr.theta = wrap_angle(raw.angle - sign * TRACKER_MAX_LAG_RAD);
            let a_v = (dt / TRACKER_CATCHUP_TAU_S).min(1.0);
            tr.omega += a_v * (raw.velocity - tr.omega);
        }
        PhaseOutput {
            angle: tr.theta,
            velocity: tr.omega,
        }
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn compute_phase_with_fallback(
        &mut self,
        hall_sample: Option<AngleSample>,
        encoder_sample: Option<AngleSample>,
        dt: f32,
    ) -> PhaseOutput {
        // A live hall sample retakes commutation from the recovery
        // override. Without this, one glitch-triggered override outlived
        // the glitch: nothing deactivated it when the SENSOR (rather than
        // the observer) came back, and `angle_trustworthy()` stayed false
        // forever on a healthy hall.
        if hall_sample.is_some() && self.open_loop_override.active && self.source.requires_hall() {
            self.deactivate_open_loop_override();
        }

        match self.source {
            PhaseSource::Hall => {
                // VESC-style: try Hall first, fall back to observer if Hall failed
                if let Some(sample) = hall_sample {
                    PhaseOutput {
                        angle: sample.angle,
                        velocity: sample.omega,
                    }
                } else {
                    // Hall failed - try observer fallback
                    self.try_observer_fallback().unwrap_or(self.output)
                }
            }

            PhaseSource::Encoder => sample_to_output(encoder_sample, &self.output),

            PhaseSource::Observer => {
                // Pure sensorless: commutate from a converged observer; else
                // run the cold-start sequencer open-loop (align→ramp→handoff)
                // if it is active; else hold the last output (and raise the
                // fault). The sequencer is advanced in `update`, which hands
                // off (deactivates) the moment the observer converges.
                if self.observer.is_ready() {
                    self.clear_fault(PhaseFault::ObserverNotReady);
                    match (self.observer.phase(), self.observer.velocity()) {
                        (Some(angle), Some(vel)) => {
                            let raw = PhaseOutput {
                                angle,
                                velocity: vel,
                            };
                            // Filter only POST-handoff: while the startup
                            // sequencer is still active a ready observer
                            // takes commutation raw (pre-existing behavior
                            // the hold/confirm dynamics are tuned around);
                            // the filter seeds continuity at the handoff.
                            if self.tracker.kp > 0.0 && !self.startup.is_active() {
                                self.tracker_output(raw, dt)
                            } else {
                                self.tracker.active = false;
                                raw
                            }
                        }
                        _ => self.output,
                    }
                } else if self.startup.is_active() {
                    // The sequencer owns commutation: the freq-led filter
                    // re-seeds continuity at the next handoff.
                    self.tracker.active = false;
                    PhaseOutput {
                        angle: self.startup.angle(),
                        velocity: self.startup.velocity(),
                    }
                } else {
                    self.set_fault(PhaseFault::ObserverNotReady);
                    self.tracker.active = false;
                    self.output
                }
            }

            PhaseSource::Hfi => {
                // HFI estimate straight from the dedicated slot. No readiness
                // gate: HFI is valid from standstill by design. The carrier
                // reaches the motor via PhaseProvider::injection() →
                // FocController::step_with_injection in FocDriver.
                self.hfi_output().unwrap_or(self.output)
            }

            PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            } => {
                // VESC-style full Hall mode:
                // 1. Hall failed (invalid state / stale at speed) → observer
                //    if ready, else the open-loop recovery override.
                // 2. Hall healthy → blend Hall→observer by velocity.
                if hall_sample.is_none() {
                    return self.try_observer_fallback().unwrap_or(self.output);
                }
                let sensor = sample_to_output(hall_sample, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
            }

            PhaseSource::EncoderToObserver {
                blend_low,
                blend_high,
            } => {
                let sensor = sample_to_output(encoder_sample, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
            }

            // Both estimator slots run concurrently; the output blends from
            // HFI to the observer across the velocity band
            // [min_vel·(1−CROSSOVER_HYSTERESIS), min_vel]. A sharp switch is
            // not enough here: the HFI demod/PLL lag grows with speed, so at
            // the crossover the two estimates legitimately disagree by tenths
            // of a radian — blending absorbs that, a latch jump would not.
            // `crossover_latched` marks the fully-blended regime: only there
            // is the carrier injection switched off, and re-entering the band
            // from above reseeds the HFI estimator from the last output
            // (its own estimate drifted while the carrier was off).
            PhaseSource::HfiToObserver {
                min_vel,
                min_confidence,
            } => {
                let mut hfi_out = self.hfi_output().unwrap_or(self.output);
                let obs_ready =
                    self.observer.is_ready() && self.observer.confidence() >= min_confidence;
                if !obs_ready {
                    // Observer can't be trusted at any blend weight yet.
                    if self.crossover_latched {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                        hfi_out = self.output;
                    }
                    return hfi_out;
                }

                // Speed reference: the faster of the two estimates, for the
                // same reason as the hall blend — near the handoff the
                // low-speed source's velocity may already be degrading.
                let obs_vel = self.observer.velocity().unwrap_or(0.0).abs();
                let speed = hfi_out.velocity.abs().max(obs_vel);
                let blend_low = min_vel * (1.0 - CROSSOVER_HYSTERESIS);
                let blend = compute_blend(speed, blend_low, min_vel);

                let was_latched = self.crossover_latched;
                self.crossover_latched = blend >= 1.0;
                if was_latched && !self.crossover_latched {
                    self.seed_hfi_from_output();
                    hfi_out = self.output;
                }

                match (self.observer.phase(), self.observer.velocity()) {
                    (Some(obs_angle), Some(obs_velocity)) => PhaseOutput {
                        angle: blend_angles(hfi_out.angle, obs_angle, blend),
                        velocity: hfi_out.velocity * (1.0 - blend) + obs_velocity * blend,
                    },
                    _ => hfi_out,
                }
            }

            // Voltage-criterion variant of HfiToObserver (MESC): the blend
            // weight comes from the back-EMF share of the drive voltage
            // instead of a velocity threshold. Same latch/reseed mechanics.
            PhaseSource::HfiToObserverVolts {
                toggle_v,
                min_confidence,
            } => {
                let mut hfi_out = self.hfi_output().unwrap_or(self.output);
                let obs_ready =
                    self.observer.is_ready() && self.observer.confidence() >= min_confidence;
                if !obs_ready {
                    if self.crossover_latched {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                        hfi_out = self.output;
                    }
                    return hfi_out;
                }

                let blend_low = (toggle_v - HFI_VOLTS_HYSTERESIS).max(0.0);
                let blend = compute_blend(self.bemf_proxy_v, blend_low, toggle_v);

                let was_latched = self.crossover_latched;
                self.crossover_latched = blend >= 1.0;
                if was_latched && !self.crossover_latched {
                    self.seed_hfi_from_output();
                    hfi_out = self.output;
                }

                match (self.observer.phase(), self.observer.velocity()) {
                    (Some(obs_angle), Some(obs_velocity)) => PhaseOutput {
                        angle: blend_angles(hfi_out.angle, obs_angle, blend),
                        velocity: hfi_out.velocity * (1.0 - blend) + obs_velocity * blend,
                    },
                    _ => hfi_out,
                }
            }

            PhaseSource::HfiToHall { switch_vel } => {
                let hall = sample_to_output(hall_sample, &self.output);
                if self.crossover_latched {
                    // Drop back to HFI only below the hysteresis band or on
                    // hall failure.
                    if hall_sample.is_none()
                        || hall.velocity.abs() < switch_vel * (1.0 - CROSSOVER_HYSTERESIS)
                    {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                    }
                } else if hall_sample.is_some() && hall.velocity.abs() >= switch_vel {
                    self.crossover_latched = true;
                }

                if self.crossover_latched {
                    hall
                } else {
                    self.hfi_output().unwrap_or(self.output)
                }
            }

            PhaseSource::HfiToEncoder { switch_vel } => {
                let enc = sample_to_output(encoder_sample, &self.output);
                if self.crossover_latched {
                    if encoder_sample.is_none()
                        || enc.velocity.abs() < switch_vel * (1.0 - CROSSOVER_HYSTERESIS)
                    {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                    }
                } else if encoder_sample.is_some() && enc.velocity.abs() >= switch_vel {
                    self.crossover_latched = true;
                }

                if self.crossover_latched {
                    enc
                } else {
                    self.hfi_output().unwrap_or(self.output)
                }
            }

            PhaseSource::Manual => PhaseOutput {
                angle: self.manual_angle,
                velocity: 0.0,
            },

            PhaseSource::OpenLoop => PhaseOutput {
                angle: self.open_loop_angle,
                velocity: self.open_loop_velocity,
            },
        }
    }

    /// Whether HFI runs this cycle: carrier injection AND the demod update
    /// — always as a pair (a demod without carrier measures silence, a
    /// carrier without demod is pure loss).
    ///
    /// Above the crossover latch the fast source commutates and HFI is off
    /// entirely — keeping the carrier on at speed only costs losses and
    /// acoustic noise while the saliency response degrades anyway, and
    /// running the demod update without carrier is wasted ISR time (in
    /// non-Hfi sources it never runs at all for the same reason).
    ///
    /// PRE-HEAT: in the latched regime HFI resumes one hysteresis band
    /// EARLY — while the speed (or back-EMF proxy) is still a margin above
    /// the latch-release threshold. The demod filters need several carrier
    /// periods of real current to lock after a carrier-off gap
    /// ([`HfiObserver::restart_demod`]); the margin buys that time for any
    /// controller-bounded deceleration, so by the time the latch releases
    /// and blend weight starts flowing to HFI it is already locked. The
    /// margin deliberately does NOT try to cover mechanically-unbounded
    /// deceleration (a wheel jam crosses any finite band instantly) — that
    /// tail is covered by the trust gate instead: a cold demod reports
    /// zero confidence, `angle_trustworthy()` stays false and the driver
    /// keeps iq at zero for the few ms the lock takes.
    #[cfg(feature = "hfi")]
    fn hfi_active(&self) -> bool {
        match self.source {
            PhaseSource::Hfi => true,
            PhaseSource::HfiToObserver { min_vel, .. } => {
                !self.crossover_latched
                    || self.output.velocity.abs() < min_vel * (1.0 + CROSSOVER_HYSTERESIS)
            }
            PhaseSource::HfiToObserverVolts { toggle_v, .. } => {
                !self.crossover_latched || self.bemf_proxy_v < toggle_v + HFI_VOLTS_HYSTERESIS
            }
            PhaseSource::HfiToHall { switch_vel } | PhaseSource::HfiToEncoder { switch_vel } => {
                !self.crossover_latched
                    || self.output.velocity.abs() < switch_vel * (1.0 + CROSSOVER_HYSTERESIS)
            }
            _ => false,
        }
    }

    /// Reseed the HFI estimator from the last managed output (downward
    /// crossover handoff). The angle comes from a source that was trusted
    /// for commutation, so this also resolves the HFI π ambiguity. No-op when
    /// HFI is compiled out (the callers — Hfi* arms — are then unreachable).
    #[cfg(feature = "hfi")]
    fn seed_hfi_from_output(&mut self) {
        let out = self.output;
        if let Some(hfi) = &mut self.hfi {
            hfi.set_phase(out.angle);
            hfi.set_velocity(out.velocity);
        }
    }

    #[cfg(not(feature = "hfi"))]
    fn seed_hfi_from_output(&mut self) {}

    /// Blend sensor output with observer based on velocity
    fn blend_with_observer(
        &mut self,
        sensor: PhaseOutput,
        blend_low: f32,
        blend_high: f32,
    ) -> PhaseOutput {
        // An unconverged observer must not pull the output anywhere — the
        // sensor stays authoritative until the observer is actually locked.
        if !self.observer.is_ready() {
            return sensor;
        }

        // Speed reference for regime selection: the faster of the two
        // estimates. Using the sensor velocity alone is perverse when the
        // sensor dies at speed — its decaying estimate drags the blend back
        // toward trusting the dying sensor MORE, while the (ready) observer
        // still reports the true speed.
        let obs_speed = self.observer.velocity().unwrap_or(0.0).abs();
        let blend = compute_blend(sensor.velocity.abs().max(obs_speed), blend_low, blend_high);

        // π-flip guard: back-EMF observers carry a half-turn ambiguity from
        // standstill. While the sensor still has any weight it is the truth
        // reference — a >90° disagreement means the observer locked onto the
        // inverted flux vector, and blending toward it collapses torque
        // mid-crossover. Reseed instead of blending.
        if blend < 1.0
            && let Some(obs_angle) = self.observer.phase()
            && angle_difference(obs_angle, sensor.angle).abs() > core::f32::consts::FRAC_PI_2
        {
            self.observer.seed(sensor.angle, sensor.velocity);
        }

        if let (Some(obs_angle), Some(obs_vel)) = (self.observer.phase(), self.observer.velocity())
        {
            PhaseOutput {
                angle: blend_angles(sensor.angle, obs_angle, blend),
                velocity: sensor.velocity * (1.0 - blend) + obs_vel * blend,
            }
        } else {
            sensor
        }
    }
}

// ============================================================================
// PhaseProvider implementation
// ============================================================================

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseProvider for PhaseManager<H, E, S> {
    fn get(&self) -> PhaseOutput {
        self.output
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn update(&mut self, input: &PhaseInput, now_ticks: u64) {
        // Sample hardware sensors — only for sources that consume them. The
        // stateful path matters for hall: it carries the rate limiter that
        // smooths sector-edge discontinuities. A stale hall (edges stopped
        // while spinning) is treated as having no sample at all, so every
        // consumer below falls back uniformly. On a sensorless (Observer)
        // source both samples are dead weight at 20 kHz — the interpolation
        // math ran every cycle for data nothing read (2026-07-06 ISR
        // profiling); everything downstream is already `requires_*`-gated
        // and treats `None` as "no sensor".
        let (hall_stale, hall_sample) = if self.source.requires_hall() {
            let stale = self.hall.is_stale(now_ticks);
            (
                stale,
                if stale {
                    None
                } else {
                    self.hall.sample_mut(now_ticks)
                },
            )
        } else {
            (false, None)
        };
        let encoder_sample = if self.source.requires_encoder() {
            self.encoder.sample_mut(now_ticks)
        } else {
            None
        };

        // Hall health is only meaningful for sources that consume hall data:
        // an idle hall during Manual-angle calibration or pure-observer
        // operation is not a failure.
        if self.source.requires_hall() {
            self.update_hall_health(hall_sample.is_some(), hall_stale, now_ticks);
        }

        // Back-EMF proxy for the voltage-based crossover: park the drive
        // voltage and current onto the current output frame and remove the
        // resistive drop. Only meaningful (and only computed) when the
        // active source uses it.
        if matches!(self.source, PhaseSource::HfiToObserverVolts { .. }) {
            use crate::foc::trig::FastSinCos;
            let r = self.observer.resistance().unwrap_or(0.0);
            let (sin_t, cos_t) = FastSinCos::sin_cos(self.output.angle);
            let (_, vq) = park(input.v_alpha, input.v_beta, sin_t, cos_t);
            let (_, iq) = park(input.i_alpha, input.i_beta, sin_t, cos_t);
            let bemf = vq - r * iq;
            self.bemf_proxy_v = if bemf < 0.0 { -bemf } else { bemf };
        }

        // Update the estimators. The back-EMF observer always runs — it is
        // the fallback/crossover target for every source and has its own
        // signal (back-EMF) regardless of mode. HFI runs only while its
        // carrier is injected (see hfi_active()): without carrier the demod
        // measures silence — pure wasted ISR time, ~10% of the cycle budget
        // in the default hall ride configuration. A rising edge restarts
        // the demod filters so the stale pre-pause state can't masquerade
        // as confidence.
        let obs_input = ObserverInput {
            v_alpha: input.v_alpha,
            v_beta: input.v_beta,
            i_alpha: input.i_alpha,
            i_beta: input.i_beta,
            dt: input.dt,
        };
        let prof_t0 = crate::isr_prof::now();
        self.observer.update(&obs_input);
        crate::isr_prof::add(&crate::isr_prof::EST_OBS, prof_t0, crate::isr_prof::now());

        // ~2.4 Hz observer-internals trace while the estimate is in motion
        // (decimated; one atomic RMW per cycle). The estimator sessions keep
        // needing exactly this: the fast frame only carries the ACTIVE
        // source's angle/velocity, so a diverging observer riding behind an
        // open-loop source (startup hold, OpenLoop bench drives) was
        // invisible — the 2026-07-06 hold-ratchet (observer 219→756 rad/s
        // at a constant 180 rad/s hold) could only be inferred from confirm
        // probe logs.
        {
            // Plain field, not an atomic: this runs only in the ISR, and a
            // static AtomicU32 here was a per-cycle LDREX/STREX pair (PC
            // sampling, tier-3 shave).
            self.obs_trace_ticks = self.obs_trace_ticks.wrapping_add(1);
            if self.obs_trace_ticks.is_multiple_of(8192)
                && let Some(vel) = self.observer.velocity()
                && vel.abs() > 20.0
            {
                let (bemf_q, travel) = self.observer.validity().unwrap_or((0.0, 0.0));
                info!(
                    "obs: vel={} conf={} e_q={} travel={} lambda={}",
                    vel,
                    self.observer.confidence(),
                    bemf_q,
                    travel,
                    self.observer.lambda().unwrap_or(0.0)
                );
            }
        }
        #[cfg(feature = "hfi")]
        {
            let hfi_active = self.hfi_active();
            if hfi_active && let Some(hfi) = &mut self.hfi {
                if !self.hfi_was_active {
                    hfi.restart_demod();
                }
                hfi.update(&obs_input);
            }
            self.hfi_was_active = hfi_active;
        }

        // Advance open-loop angle if in OpenLoop mode
        if matches!(self.source, PhaseSource::OpenLoop) {
            self.open_loop_angle += self.open_loop_velocity * input.dt;
            self.open_loop_angle = wrap_angle(self.open_loop_angle);
        }

        // Advance the sensorless cold-start sequencer (if running): it drives
        // the open-loop angle the Observer arm commutates from, and hands off
        // — deactivates — the moment the observer has actually converged at
        // handoff speed (it keeps running on commanded-v + measured-i the
        // whole time, so by handoff its angle is the true rotor angle).
        let prof_t0 = crate::isr_prof::now();
        // Startup-log rate limit (see startup.rs LOG_TOKENS_PER_WINDOW):
        // tick the window every cycle; when frames were dropped, one
        // summary frame per window keeps the churn visible.
        let log_suppressed = self.startup.log_tick(input.dt);
        if log_suppressed > 0 {
            warn!(
                "startup: {} log frames suppressed (restart churn)",
                log_suppressed
            );
        }
        if self.startup.is_active() {
            let phase_before = self.startup.phase();
            if phase_before == StartupPhase::Deadshort {
                // Flying-restart probe: the driver holds the bridge at zero
                // voltage; feed the back-EMF-driven current to the probe. A
                // spinning rotor → seed the observer and go straight to closed
                // loop; standstill → the probe falls through to the align ramp.
                let r = self.observer.resistance().unwrap_or(0.0);
                let l = self.observer.inductance().unwrap_or(0.0);
                let lambda = self.observer.lambda().unwrap_or(0.0);
                if let DeadshortResult::Caught { angle, velocity } =
                    self.startup
                        .feed_deadshort(input.i_alpha, input.i_beta, input.dt, r, l, lambda)
                {
                    if self.startup.log_allow() {
                        info!(
                            "startup: deadshort caught spinning rotor (angle={} vel={}), seeding observer",
                            angle, velocity
                        );
                    }
                    self.observer.seed(angle, velocity);
                } else if self.startup.phase() == StartupPhase::Ramp && self.startup.log_allow() {
                    info!("startup: deadshort saw standstill, ramp cold start");
                }
            } else if phase_before == StartupPhase::Confirm {
                // Handoff-confirm probe: bridge shorted, the measured
                // back-EMF must corroborate the observer before closed loop
                // engages (a phantom-locked observer passes every internal
                // gate — see BackEmfObserver::is_ready).
                let r = self.observer.resistance().unwrap_or(0.0);
                let l = self.observer.inductance().unwrap_or(0.0);
                let lambda = self.observer.lambda().unwrap_or(0.0);
                let claim = PhaseOutput {
                    angle: self.observer.phase().unwrap_or(0.0),
                    velocity: self.observer.velocity().unwrap_or(0.0),
                };
                let obs_vel = claim.velocity;
                match self.startup.feed_confirm(
                    input.i_alpha,
                    input.i_beta,
                    input.dt,
                    r,
                    l,
                    lambda,
                    claim,
                ) {
                    ConfirmResult::Confirmed { velocity } => {
                        if self.startup.log_allow() {
                            info!(
                                "startup: handoff confirmed by probe (probe_vel={} observer_vel={})",
                                velocity, obs_vel
                            );
                        }
                    }
                    ConfirmResult::Unconfirmed { velocity } => {
                        if self.startup.log_allow() {
                            info!(
                                "startup: handoff unconfirmed (probe_vel={} observer_vel={}), \
                                 holding for retry",
                                velocity, obs_vel
                            );
                        }
                    }
                    ConfirmResult::SeedAndHandoff { angle, velocity } => {
                        // The probe measured a real spinning rotor on
                        // CONFIRM_SEED_PROBES consecutive tries while the
                        // observer's claim kept diverging (hold-ratchet):
                        // trust the measurement, reseed the observer from
                        // it — same as the deadshort catch.
                        if self.startup.log_allow() {
                            warn!(
                                "startup: observer diverged from probed rotor \
                                 (probe_vel={} observer_vel={}), seeding observer from probe",
                                velocity, obs_vel
                            );
                        }
                        self.observer.seed(angle, velocity);
                    }
                    ConfirmResult::Probing => {}
                }
            } else {
                let i_mag = sqrtf(input.i_alpha * input.i_alpha + input.i_beta * input.i_beta);
                let out = self.startup.tick(
                    input.dt,
                    i_mag,
                    self.observer.is_ready(),
                    self.observer.velocity().unwrap_or(0.0),
                );
                // Transitions are one-shot per start — but starts themselves
                // repeat several times a second during restart churn, hence
                // the token bucket on every frame here.
                let phase_now = self.startup.phase();
                if phase_now != phase_before && self.startup.log_allow() {
                    info!(
                        "startup: {} -> {} (vel={} |i|={})",
                        phase_before.name(),
                        phase_now.name(),
                        out.velocity,
                        i_mag
                    );
                }
                // Hold give-up recycle: the observer failed to confirm for
                // the whole hold window — presume a phantom lock and
                // restart it from scratch while the deadshort→ramp start
                // re-acquires the rotor.
                if self.startup.take_recycled() {
                    if self.startup.log_allow() {
                        warn!(
                            "startup: hold gave up (no confirmed handoff), observer reset + recycle"
                        );
                    }
                    self.observer.reset();
                }
                if out.handoff {
                    if self.startup.log_allow() {
                        info!(
                            "startup: handoff gates passed (openloop_vel={} observer_vel={}), \
                             running confirm probe",
                            out.velocity,
                            self.observer.velocity().unwrap_or(0.0)
                        );
                    }
                } else if phase_now == StartupPhase::Hold {
                    // Waiting on the observer: ~2 Hz convergence trace so a
                    // hold that never hands off (observer incoherent, see
                    // HANDOFF_COHERENCE_FRAC) is diagnosable from the log.
                    self.hold_trace_ticks = self.hold_trace_ticks.wrapping_add(1);
                    if self.hold_trace_ticks.is_multiple_of(8192) && self.startup.log_allow() {
                        info!(
                            "startup: holding (openloop_vel={} observer_vel={} ready={} conf={})",
                            out.velocity,
                            self.observer.velocity().unwrap_or(0.0),
                            self.observer.is_ready(),
                            self.observer.confidence()
                        );
                    }
                }
            }
        }

        let prof_t1 = crate::isr_prof::now();
        crate::isr_prof::add(&crate::isr_prof::EST_STARTUP, prof_t0, prof_t1);

        // Update open-loop override state (for Hall failure recovery)
        self.update_open_loop_override(input.dt);

        // Compute output based on source (with potential fallback)
        self.output = self.compute_phase_with_fallback(hall_sample, encoder_sample, input.dt);
        crate::isr_prof::add(&crate::isr_prof::EST_OUT, prof_t1, crate::isr_prof::now());
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        self.set_source(source).is_ok()
    }

    fn begin_cold_start(&mut self, dir: f32) {
        // Only a pure back-EMF source needs the open-loop bootstrap: a sensored
        // source (Hall/Encoder, or a *ToObserver blend) already has an angle at
        // standstill. No-op once the observer is tracking. Starts from the last
        // output angle so the field doesn't jump on engage.
        if !matches!(self.source, PhaseSource::Observer) {
            return;
        }
        if self.observer.is_ready() {
            if self.startup.log_allow() {
                info!(
                    "startup: cold start skipped, observer already tracking (vel={})",
                    self.observer.velocity().unwrap_or(0.0)
                );
            }
            return;
        }
        if self.startup.log_allow() {
            info!(
                "startup: cold start engaged (angle0={} dir={})",
                self.output.angle, dir
            );
        }
        self.startup.begin_cold_start(self.output.angle, dir);
    }

    fn wants_short(&self) -> bool {
        self.startup.wants_short()
    }

    fn startup_current_scale(&self) -> f32 {
        self.startup.current_scale()
    }

    fn is_starting(&self) -> bool {
        self.startup.is_active()
    }

    /// Trustworthy down to standstill when a hardware sensor backs the active
    /// source (Hall/Encoder track to a stop), or when HFI is locked (valid at
    /// zero speed by design). A pure back-EMF observer is only trusted while
    /// `is_ready()` — it drops below its speed floor, so the failsafe brake
    /// coasts the last bit instead of commutating blind.
    ///
    /// While the open-loop recovery override carries commutation (sensor
    /// dead AND observer not ready), the angle is FABRICATED — a rotating
    /// frame with no relation to the rotor. Reporting it trustworthy put
    /// the full commanded iq into that frame (random-direction torque
    /// jolts under a rider at low speed) and let the failsafe commutate
    /// the brake blind. Untrusted, the existing iq gate coasts instead;
    /// recovery comes from physical motion (kick-push → back-EMF →
    /// observer locks → override deactivates → trust returns). The
    /// deliberate cost: no self-start from standstill with a dead hall
    /// until the sensorless promotion lands (fault-overhaul.md phase 6,
    /// variant A decision 2026-06-13).
    fn angle_trustworthy(&self) -> bool {
        match self.source {
            PhaseSource::Hall
            | PhaseSource::Encoder
            | PhaseSource::HallToObserver { .. }
            | PhaseSource::EncoderToObserver { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => !self.open_loop_override.active,
            // Calibration/detection sources: the commanded angle IS the
            // reference frame, trusted by construction.
            PhaseSource::Manual | PhaseSource::OpenLoop => true,
            // Pure sensorless: trusted once the observer locks, OR while the
            // cold-start sequencer drives — there the open-loop angle IS the
            // intended I/f reference and torque MUST flow into it to spin the
            // rotor up (unlike the hall-dropout recovery override above, which
            // is a fabricated frame under a moving rider and stays untrusted).
            // Only `begin_cold_start` (a deliberate sensorless start) arms it.
            PhaseSource::Observer => self.observer.is_ready() || self.startup.is_active(),
            #[cfg(feature = "hfi")]
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. } => {
                self.hfi.as_ref().is_some_and(HfiObserver::is_ready) || self.observer.is_ready()
            }
            // HFI compiled out: these sources are unreachable (set_source
            // rejects them), but the match must stay exhaustive.
            #[cfg(not(feature = "hfi"))]
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. } => self.observer.is_ready(),
        }
    }

    #[cfg(feature = "hfi")]
    fn injection(&self) -> (f32, f32) {
        if self.hfi_active()
            && let Some(hfi) = &self.hfi
        {
            hfi.get_injection()
        } else {
            (0.0, 0.0)
        }
    }

    #[cfg(not(feature = "hfi"))]
    fn injection(&self) -> (f32, f32) {
        (0.0, 0.0)
    }

    /// Most specific active hall degradation: the sensor's wire verdicts
    /// (named dead wires / error rate) win over the coarse health kinds —
    /// "wire H2 dead" tells the bench which pin to buzz out, "invalid
    /// state" only says something is wrong. Only sources that consume hall
    /// data report; an idle hall during Manual calibration or pure
    /// sensorless is not a failure.
    fn set_slip_gate(&mut self, gated: bool) {
        self.observer.set_slip_gate(gated);
    }

    fn note_torque_current(&mut self, iq_abs: f32, dt: f32) {
        self.observer.note_torque_current(iq_abs, dt);
    }

    fn debug_observer(&self) -> Option<(f32, f32, f32, f32)> {
        match (
            self.observer.phase(),
            self.observer.phase_raw(),
            self.observer.velocity(),
            self.observer.readiness_phase_err(),
        ) {
            (Some(pll), Some(raw), Some(vel), Some(err)) => Some((pll, raw, vel, err)),
            _ => None,
        }
    }

    fn hall_fault(&self) -> Option<HallFaultKind> {
        if !self.source.requires_hall() {
            return None;
        }
        if let Some(kind) = self.hall.fault_kind() {
            return Some(kind);
        }
        match self.hall_health {
            HallHealth::Stale => Some(HallFaultKind::StaleAtSpeed),
            HallHealth::Invalid => Some(HallFaultKind::InvalidState),
            HallHealth::Ok | HallHealth::NotPresent => None,
        }
    }
}

// ============================================================================
// Configuration from stored runtime config
// ============================================================================

#[cfg(feature = "storage")]
impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Configure the software estimators from stored motor parameters.
    ///
    /// With valid detected params both slots are armed: a back-EMF observer
    /// (R, L_avg, λ) and an HFI estimator with default carrier settings.
    /// The sources stay untouched — the estimators only run; the host
    /// selects a sensorless source explicitly when it wants one.
    #[cfg_attr(not(feature = "hfi"), allow(unused_variables))]
    pub fn configure_observers_from_config(&mut self, config: &RuntimeConfig, vbus: f32) {
        use super::observer::{BackEmfObserver, Observer};
        #[cfg(feature = "hfi")]
        use super::observer::{HFI_DEFAULT_AMPLITUDE_RATIO, HFI_DEFAULT_FREQ_HZ};

        if let Some(ref mp) = config.motor_params
            && mp.is_valid()
            && mp.flux_linkage_wb > 0.0
        {
            let l_avg = (mp.inductance_d_h + mp.inductance_q_h) / 2.0;
            // Detected params arm the full model: salient Lα/Lβ when the
            // motor measurably is (scalar L on an IPM gives load-dependent
            // angle error), and online λ tracking (λ drifts with saturation
            // and magnet temperature; the stored value is one bench point).
            let mut obs = BackEmfObserver::new(mp.resistance_ohm, l_avg, mp.flux_linkage_wb)
                .with_lambda_tracking(super::observer::DEFAULT_LAMBDA_GAIN);
            let saliency = (mp.inductance_q_h - mp.inductance_d_h).abs();
            if saliency > 0.05 * l_avg {
                obs = obs.with_saliency(mp.inductance_d_h, mp.inductance_q_h);
            }
            self.set_observer(Observer::BackEmf(obs));
            // Carrier amplitude solved from the measured inductance for a
            // target ripple current, ceilinged by the legacy vbus ratio:
            // on a low-L outrunner (25 µH eskate motor) the raw ratio
            // would drive tens of amps of carrier ripple; on high-L motors
            // the solve exceeds the ceiling and the ratio default applies
            // unchanged. The ripple target scales with the motor's RATING
            // when detection stored one (see HFI_RIPPLE_RATING_FRACTION —
            // not the session current limit), absolute default otherwise.
            #[cfg(feature = "hfi")]
            {
                let i_target = match mp.rating_current_a() {
                    Some(rating) => (super::observer::HFI_RIPPLE_RATING_FRACTION * rating)
                        .clamp(0.05, super::observer::HFI_CARRIER_RIPPLE_TARGET_A),
                    None => super::observer::HFI_CARRIER_RIPPLE_TARGET_A,
                };
                let omega_c = HFI_DEFAULT_FREQ_HZ * TAU;
                let amplitude = (i_target * omega_c * l_avg)
                    .min(vbus * HFI_DEFAULT_AMPLITUDE_RATIO)
                    .max(0.05);
                self.set_hfi_observer(
                    HfiObserver::new(HFI_DEFAULT_FREQ_HZ, amplitude).with_sincos(),
                );
            }
        }
    }
}

// ============================================================================
// Conditional methods for Hall sensor
// ============================================================================

impl<H: HallSensorTrait, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Set Hall calibration table
    pub fn set_hall_calibration(&mut self, table: [f32; 6]) {
        self.hall.set_calibration(table);
    }

    /// Apply Hall calibration result
    pub fn apply_hall_calibration(&mut self, result: &HallCalibrationResult) -> bool {
        self.hall.apply_calibration(result)
    }

    /// Get raw Hall state (0-7)
    pub fn hall_raw_state(&self) -> u8 {
        self.hall.raw_state()
    }

    /// Get logical Hall state (0-5)
    pub fn hall_logical_state(&self) -> u8 {
        self.hall.logical_state()
    }

    /// Set Hall timing advance
    pub fn set_hall_advance(&mut self, advance_rad: f32) {
        self.hall.set_advance(advance_rad);
    }

    /// Get Hall timing advance
    pub fn hall_advance(&self) -> f32 {
        self.hall.advance()
    }

    /// Get Hall electrical velocity
    pub fn hall_velocity(&self) -> f32 {
        self.hall.electrical_velocity()
    }
}

// ============================================================================
// Utility functions
// ============================================================================

/// Convert angle sample to phase output
fn sample_to_output(sample: Option<AngleSample>, fallback: &PhaseOutput) -> PhaseOutput {
    match sample {
        Some(s) => PhaseOutput {
            angle: s.angle,
            velocity: s.omega,
        },
        None => *fallback,
    }
}

/// Compute blend factor (0.0 = sensor, 1.0 = observer)
fn compute_blend(velocity: f32, blend_low: f32, blend_high: f32) -> f32 {
    if velocity <= blend_low {
        0.0
    } else if velocity >= blend_high {
        1.0
    } else {
        (velocity - blend_low) / (blend_high - blend_low)
    }
}

/// Blend two angles with given factor (0.0 = a, 1.0 = b)
fn blend_angles(a: f32, b: f32, blend: f32) -> f32 {
    // Handle angle wraparound properly
    let diff = wrap_angle(b - a);
    let diff_signed = if diff > core::f32::consts::PI {
        diff - TAU
    } else {
        diff
    };

    wrap_angle(a + diff_signed * blend)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::hall_sensor::Direction;
    #[cfg(feature = "virtual-motor")]
    use crate::foc::hall_sensor::HallSensor;
    #[cfg(feature = "virtual-motor")]
    use crate::virtual_motor::VirtualMotorOutput;

    /// Phase-tracker shape (see PhaseTracker, TRACKER_KD_TAU_S), measured
    /// as amplitude ratios of an injected estimate wobble: the 35-100 Hz
    /// mid-band wobble is attenuated, the unavoidable low-frequency
    /// resonant bump stays mild (a strong peak in the 5-25 Hz band would
    /// PUMP rotor hunting), and the type-2 acceleration lag stays bounded
    /// at ~omega_dot/wn^2 — never the ~90° standing angle of the freq-led
    /// I/f geometry (dossier in docs/TODO.md).
    #[test]
    fn phase_tracker_attenuates_wobble_without_resonance_and_bounds_lag() {
        let dt = 1.0 / 20_000.0;
        // Amplitude ratio of the output wobble vs an injected angle wobble
        // riding a constant-speed ramp (measured over the last quarter).
        let ratio = |f_hz: f32| -> f32 {
            let amp = 0.5f32;
            let omega0 = 1000.0f32;
            let mut m = PhaseManager::sensorless();
            m.set_phase_tracker(60.0, 1.2);
            m.output = PhaseOutput {
                angle: 0.0,
                velocity: omega0,
            };
            let mut theta_clean = 0.0f32;
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for k in 0..16_000 {
                theta_clean = wrap_angle(theta_clean + omega0 * dt);
                #[allow(clippy::cast_precision_loss)]
                let ph = TAU * f_hz * (k as f32) * dt;
                let raw = PhaseOutput {
                    angle: wrap_angle(theta_clean + amp * libm::sinf(ph)),
                    velocity: omega0 + amp * TAU * f_hz * libm::cosf(ph),
                };
                let out = m.tracker_output(raw, dt);
                if k >= 12_000 {
                    let d = angle_difference(out.angle, theta_clean);
                    lo = lo.min(d);
                    hi = hi.max(d);
                }
            }
            (hi - lo) / 2.0 / amp
        };
        let r60 = ratio(60.0);
        assert!(r60 < 0.4, "60 Hz wobble must be attenuated, ratio {r60}");
        let r90 = ratio(90.0);
        assert!(r90 < 0.2, "90 Hz wobble must be attenuated, ratio {r90}");
        let r14 = ratio(14.0);
        assert!(
            r14 < 1.5,
            "hunt-band resonant bump must stay mild, ratio {r14}"
        );

        // Constant-acceleration lag (bench 0.3 A unloaded ~1500 rad/s² el).
        let mut m = PhaseManager::sensorless();
        m.set_phase_tracker(60.0, 1.2);
        m.output = PhaseOutput {
            angle: 0.0,
            velocity: 300.0,
        };
        let mut theta_clean = 0.0f32;
        let mut vel = 300.0f32;
        let mut lag = 0.0f32;
        for k in 0..12_000 {
            vel += 1500.0 * dt;
            theta_clean = wrap_angle(theta_clean + vel * dt);
            let raw = PhaseOutput {
                angle: theta_clean,
                velocity: vel,
            };
            let out = m.tracker_output(raw, dt);
            if k >= 11_000 {
                lag = lag.max(angle_difference(raw.angle, out.angle).abs());
            }
        }
        assert!(lag < 0.6, "acceleration lag must stay bounded, got {lag}");
    }

    #[test]
    fn test_sensorless_manager() {
        let phase = PhaseManager::sensorless();
        assert!(!phase.has_hall());
        assert!(!phase.has_encoder());
        assert_eq!(phase.source(), PhaseSource::Manual);
    }

    #[test]
    fn test_manual_angle() {
        let mut phase = PhaseManager::sensorless();
        phase.set_manual_angle(1.0);
        assert!((phase.manual_angle() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_open_loop() {
        let mut phase = PhaseManager::sensorless();
        phase.set_source(PhaseSource::OpenLoop).unwrap();
        phase.set_open_loop_velocity(100.0);

        let dt = 0.001;
        phase.update(
            &PhaseInput {
                dt,
                ..Default::default()
            },
            0,
        );

        // Angle should advance
        assert!(phase.open_loop_angle() > 0.0);
    }

    #[test]
    fn test_blend_angles() {
        // Same angle
        assert!((blend_angles(0.0, 0.0, 0.5) - 0.0).abs() < 1e-6);

        // 50% blend
        let result = blend_angles(0.0, 1.0, 0.5);
        assert!((result - 0.5).abs() < 1e-6);

        // Wraparound case: blend from 6.0 to 0.5 (crossing 2π)
        let result = blend_angles(6.0, 0.5, 0.5);
        // Should interpolate through 0, not through 3
        assert!(!(1.0..=5.0).contains(&result));
    }

    #[test]
    fn test_compute_blend() {
        assert_eq!(compute_blend(100.0, 200.0, 400.0), 0.0);
        assert_eq!(compute_blend(300.0, 200.0, 400.0), 0.5);
        assert_eq!(compute_blend(500.0, 200.0, 400.0), 1.0);
    }

    #[test]
    #[cfg(feature = "hfi")]
    fn injection_forwarded_only_in_hfi_regimes() {
        use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};

        let mut phase = PhaseManager::sensorless();
        phase.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        // HfiToObserver below also needs the fast slot configured.
        phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));

        // Manual source: HFI estimator configured but not commutating —
        // injecting would just heat the motor during calibration.
        assert_eq!(phase.injection(), (0.0, 0.0));

        // Pure HFI: carrier must flow (fresh carrier phase 0 → vd = A).
        phase.set_source(PhaseSource::Hfi).unwrap();
        let (vd, vq) = phase.injection();
        assert!(vd.abs() > 1.0, "expected carrier voltage, got vd = {vd}");
        assert_eq!(vq, 0.0, "pulsating injection is d-axis only");

        // Crossover source: inject below the latch; above the latch the
        // carrier stays on only inside the pre-heat margin
        // (speed < min_vel·(1+CROSSOVER_HYSTERESIS)) and stops above it.
        phase
            .set_source(PhaseSource::HfiToObserver {
                min_vel: 100.0,
                min_confidence: 0.5,
            })
            .unwrap();
        assert!(phase.injection().0.abs() > 1.0);
        phase.crossover_latched = true;
        // Latched but near the release threshold: pre-heat — carrier back
        // on so the demod locks before any blend weight arrives.
        phase.output.velocity = 110.0;
        assert!(
            phase.injection().0.abs() > 1.0,
            "carrier must pre-heat inside the margin above the latch"
        );
        // Latched and clear of the band: carrier off.
        phase.output.velocity = 100.0 * (1.0 + CROSSOVER_HYSTERESIS) + 10.0;
        assert_eq!(phase.injection(), (0.0, 0.0));
    }

    #[test]
    fn hfi_source_rejected_without_hfi_observer() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        // The HFI sources need the dedicated HFI slot: a back-EMF observer
        // can't generate a carrier, so they would silently never estimate.
        let mut phase = PhaseManager::sensorless();
        assert_eq!(
            phase.set_source(PhaseSource::Hfi),
            Err(PhaseSourceError::HfiNotConfigured)
        );
        phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));
        assert_eq!(
            phase.set_source(PhaseSource::Hfi),
            Err(PhaseSourceError::HfiNotConfigured)
        );
    }

    #[test]
    fn test_hall_health_default() {
        // Sensorless manager should have HallHealth::NotPresent
        let phase = PhaseManager::sensorless();
        assert_eq!(phase.hall_health(), HallHealth::NotPresent);
    }

    #[test]
    fn test_fault_tracking() {
        let mut phase = PhaseManager::sensorless();

        // No faults initially
        assert!(phase.faults().is_empty());
        assert!(!phase.has_fault(PhaseFault::HallTimeout));

        // Set a fault
        phase.set_fault(PhaseFault::HallTimeout);
        assert!(phase.has_fault(PhaseFault::HallTimeout));
        assert_eq!(phase.faults().len(), 1);

        // Setting same fault twice should not add duplicate
        phase.set_fault(PhaseFault::HallTimeout);
        assert_eq!(phase.faults().len(), 1);

        // Add another fault
        phase.set_fault(PhaseFault::ObserverNotReady);
        assert_eq!(phase.faults().len(), 2);
        assert!(phase.has_fault(PhaseFault::ObserverNotReady));

        // Clear specific fault
        phase.clear_fault(PhaseFault::HallTimeout);
        assert!(!phase.has_fault(PhaseFault::HallTimeout));
        assert!(phase.has_fault(PhaseFault::ObserverNotReady));
        assert_eq!(phase.faults().len(), 1);

        // Clear all faults
        phase.set_fault(PhaseFault::HallInvalidState);
        assert_eq!(phase.faults().len(), 2);
        phase.clear_faults();
        assert!(phase.faults().is_empty());
    }

    // Mock Hall sensor that can fail on demand
    struct MockHallSensor {
        valid: bool,
        angle: f32,
        omega: f32,
    }

    impl MockHallSensor {
        fn new() -> Self {
            Self {
                valid: true,
                angle: 0.5,
                omega: 100.0,
            }
        }

        fn set_valid(&mut self, valid: bool) {
            self.valid = valid;
        }

        fn set_reading(&mut self, angle: f32, omega: f32) {
            self.angle = angle;
            self.omega = omega;
        }
    }

    impl AngleSensor for MockHallSensor {
        fn sample(&self, _now_ticks: u64) -> Option<AngleSample> {
            if self.valid {
                Some(AngleSample {
                    angle: self.angle,
                    omega: self.omega,
                    direction: Direction::Clockwise,
                })
            } else {
                None
            }
        }

        fn read_angle(&self) -> f32 {
            self.angle
        }

        fn read_direction(&self) -> Direction {
            Direction::Clockwise
        }

        fn error_count(&self) -> u32 {
            if self.valid { 0 } else { 1 }
        }

        fn reset_errors(&mut self) {}
    }

    #[test]
    fn test_hall_failure_triggers_fallback() {
        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        // Initially Hall is healthy
        assert_eq!(phase.hall_health(), HallHealth::Ok);

        // Update with valid Hall
        phase.update(&PhaseInput::default(), 0);
        assert_eq!(phase.hall_health(), HallHealth::Ok);
        assert!(!phase.has_fault(PhaseFault::HallInvalidState));

        // Make Hall fail
        phase.hall_mut().set_valid(false);
        phase.update(&PhaseInput::default(), 1000);

        // Hall should now be invalid
        assert_eq!(phase.hall_health(), HallHealth::Invalid);
        assert!(phase.has_fault(PhaseFault::HallInvalidState));

        // No observer configured, so also should have ObserverNotReady fault
        assert!(phase.has_fault(PhaseFault::ObserverNotReady));

        // Recover Hall
        phase.hall_mut().set_valid(true);
        phase.update(&PhaseInput::default(), 2000);

        // Hall should be healthy again and faults cleared
        assert_eq!(phase.hall_health(), HallHealth::Ok);
        assert!(!phase.has_fault(PhaseFault::HallInvalidState));
    }

    #[test]
    fn test_hall_failure_uses_observer_when_available() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        // Set up observer with known angle
        let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
        observer.force_phase(1.5); // Force observer to have this phase
        observer.set_velocity(200.0);
        phase.set_observer(Observer::BackEmf(observer));

        // Make Hall fail
        phase.hall_mut().set_valid(false);
        phase.update(&PhaseInput::default(), 1000);

        // Should have fallen back to observer
        let output = phase.get();
        assert!((output.angle - 1.5).abs() < 0.01);
        assert!((output.velocity - 200.0).abs() < 0.01);

        // Should not have ObserverNotReady since observer was available
        assert!(!phase.has_fault(PhaseFault::ObserverNotReady));
    }

    #[test]
    fn test_open_loop_override_activates_on_failure() {
        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        // Make Hall fail (no observer configured)
        phase.hall_mut().set_valid(false);

        // Get a first update to establish baseline
        phase.update(&PhaseInput::default(), 0);
        let _initial_output = phase.get();

        // Continue with Hall failed
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            1000,
        );

        // Open-loop override should be active
        assert!(phase.is_open_loop_override_active());
        assert!(phase.open_loop_override().timer > 0.0);

        // Output should still be valid (from open-loop override)
        let output = phase.get();
        assert!(output.velocity > 0.0); // Should have minimum velocity

        // After more updates, angle should advance
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            2000,
        );
        let output2 = phase.get();
        // Angle should have advanced
        assert!(output2.angle != output.angle || output.velocity > 0.0);
    }

    #[test]
    fn unconverged_observer_does_not_take_over_on_hall_failure() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        // A freshly constructed observer is configured but NOT converged:
        // its phase is frozen at 0 with zero confidence. Handing it the
        // commutation on a Hall dropout swaps a good angle for garbage.
        // The manager must treat it as not-ready: raise ObserverNotReady
        // and use the open-loop override instead.
        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);
        phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));

        phase.hall_mut().set_valid(false);
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            0,
        );

        assert!(
            phase.has_fault(PhaseFault::ObserverNotReady),
            "unconverged observer must raise ObserverNotReady"
        );
        assert!(
            phase.is_open_loop_override_active(),
            "must fall back to open-loop override, not the frozen observer"
        );
    }

    /// Variant A of the dead-hall decision (2026-06-13, fault-overhaul.md):
    /// while the open-loop recovery override fabricates the angle, the
    /// manager must NOT report it trustworthy — the driver's iq gate then
    /// coasts instead of pushing random-direction torque. Trust returns
    /// with the sensor (and the override must actually deactivate when the
    /// hall recovers — it used to outlive the glitch forever).
    #[test]
    fn override_makes_angle_untrusted_until_hall_recovers() {
        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        phase.update(&PhaseInput::default(), 0);
        assert!(phase.angle_trustworthy(), "healthy hall is trusted");

        // Hall dies, no observer: override fabricates the angle.
        phase.hall_mut().set_valid(false);
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            1_000,
        );
        assert!(phase.is_open_loop_override_active());
        assert!(
            !phase.angle_trustworthy(),
            "a fabricated angle must not be trusted"
        );

        // Hall recovers: the live sample retakes commutation, the override
        // deactivates, trust returns.
        phase.hall_mut().set_valid(true);
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            2_000,
        );
        assert!(
            !phase.is_open_loop_override_active(),
            "live hall must deactivate the recovery override"
        );
        assert!(phase.angle_trustworthy());
    }

    /// With a ready observer the fallback never reaches the override, so
    /// the angle stays trusted through a hall dropout at speed.
    #[test]
    fn observer_fallback_keeps_angle_trusted() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);
        let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
        observer.force_phase(1.5);
        observer.set_velocity(200.0);
        phase.set_observer(Observer::BackEmf(observer));

        phase.hall_mut().set_valid(false);
        phase.update(&PhaseInput::default(), 1_000);

        assert!(!phase.is_open_loop_override_active());
        assert!(
            phase.angle_trustworthy(),
            "observer-carried fallback is trusted"
        );
    }

    /// `hall_fault()` reports the most specific degradation and only for
    /// sources that consume hall data.
    #[test]
    fn hall_fault_reports_health_kinds() {
        use crate::foc::hall_sensor::HallFaultKind;

        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);
        assert_eq!(phase.hall_fault(), None, "healthy hall reports nothing");

        phase.hall_mut().set_valid(false);
        phase.update(&PhaseInput::default(), 1_000);
        assert_eq!(
            phase.hall_fault(),
            Some(HallFaultKind::InvalidState),
            "invalid health maps to the InvalidState kind"
        );

        // A non-hall source must not report an idle hall as a failure.
        phase.set_manual_angle(0.0);
        phase.set_source(PhaseSource::Manual).unwrap();
        phase.update(&PhaseInput::default(), 2_000);
        assert_eq!(phase.hall_fault(), None);
    }

    #[test]
    fn blend_reseeds_pi_flipped_observer_from_sensor() {
        use crate::foc::phase::{BackEmfObserver, Observer};
        use core::f32::consts::PI;

        // Back-EMF observers started from standstill have a π ambiguity.
        // When the sensor is still trustworthy, a half-turn disagreement
        // means the observer is flipped — it must be reseeded from the
        // sensor instead of letting the blend pull the output toward the
        // inverted angle (torque collapse mid-crossover).
        let mut mock_hall = MockHallSensor::new();
        let hall_angle = 1.0f32;
        mock_hall.set_reading(hall_angle, 400.0); // mid blend band
        let mut phase = PhaseManager::with_hall(mock_hall);

        let mut observer = BackEmfObserver::new(0.1, 1e-4, 0.01);
        observer.force_phase(hall_angle + PI); // flipped
        observer.set_velocity(400.0);
        phase.set_observer(Observer::BackEmf(observer));
        phase
            .set_source(PhaseSource::HallToObserver {
                blend_low: 300.0,
                blend_high: 600.0,
            })
            .unwrap();

        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            0,
        );

        let out = phase.get();
        let err = angle_difference(out.angle, hall_angle).abs();
        assert!(
            err < 0.3,
            "output {} drifted {} rad toward the flipped observer (hall at {})",
            out.angle,
            err,
            hall_angle
        );
    }

    #[test]
    fn blend_ignores_unready_observer() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        // In the blend band with an unconverged observer the sensor must
        // stay authoritative — blending with a frozen phase-0 estimate
        // drags the output toward garbage.
        let mut mock_hall = MockHallSensor::new();
        let hall_angle = 2.0f32;
        mock_hall.set_reading(hall_angle, 450.0); // mid blend band
        let mut phase = PhaseManager::with_hall(mock_hall);
        phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));
        phase
            .set_source(PhaseSource::HallToObserver {
                blend_low: 300.0,
                blend_high: 600.0,
            })
            .unwrap();

        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            0,
        );

        let out = phase.get();
        let err = angle_difference(out.angle, hall_angle).abs();
        assert!(
            err < 0.05,
            "output {} must equal the hall angle {} while observer is not ready (err {})",
            out.angle,
            hall_angle,
            err
        );
    }

    /// Closed-loop sensorless harness: VirtualMotor + FocController +
    /// PhaseManager(HallSensor + BackEmfObserver), HallToObserver source.
    ///
    /// Spins from standstill on hall commutation; `hall_feed` maps the
    /// plant's true hall state to what the firmware observes per step
    /// (`None` = no signal at all — cable cut; a masked bit = a dead
    /// wire). Deduplicates equal consecutive OBSERVED states: capture
    /// hardware only fires on a pin change. Returns the manager, final
    /// motor output and the largest one-cycle angle jump seen above the
    /// hall interpolation regime.
    #[cfg(feature = "virtual-motor")]
    fn run_sensorless_sim(
        total_steps: u64,
        hall_feed: impl Fn(u64, u8) -> Option<u8>,
    ) -> (PhaseManager<HallSensor>, VirtualMotorOutput, f32) {
        use crate::foc::controller::FocController;
        use crate::foc::hall_sensor::HallSensor;
        use crate::foc::phase::{BackEmfObserver, Observer};
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const VBUS: f32 = 24.0;
        // High friction so the motor settles around ωe ≈ 700 rad/s — well
        // above the blend band but below voltage saturation.
        let params = MotorParams {
            friction_b: 2e-3,
            ..MotorParams::default()
        };

        let mut motor = VirtualMotor::new(params);
        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r, params.ld, VBUS,
        );

        // Prime the hall estimator with the initial rotor state so the
        // manager sees a working sensor from the start.
        let mut out = motor.step(0.0, 0.0, 0.0, DT);
        let mut hall = HallSensor::new(1_000_000); // µs timebase
        hall.update(out.hall_state, 0);
        let mut last_fed = out.hall_state;

        let mut mgr = PhaseManager::with_hall(hall);
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            (params.ld + params.lq) / 2.0,
            params.lambda,
        )));
        mgr.set_source(PhaseSource::HallToObserver {
            blend_low: 150.0,
            blend_high: 300.0,
        })
        .unwrap();

        let iq_target = 2.0;
        let mut prev_angle: Option<f32> = None;
        let mut max_step_at_speed = 0.0f32;

        for step in 1..total_steps {
            let t_us = step * 50;

            if let Some(observed) = hall_feed(step, out.hall_state)
                && observed != last_fed
            {
                mgr.hall_mut().update(observed, t_us);
                last_fed = observed;
            }

            let angle = mgr.get().angle;
            let telem = foc.step((out.ia, out.ib, out.ic), angle, 0.0, iq_target, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            mgr.update(
                &PhaseInput {
                    v_alpha: telem.v_alpha,
                    v_beta: telem.v_beta,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                },
                t_us,
            );

            // Continuity: above the hall interpolation regime the managed
            // angle must never jump (π flips, blend discontinuities). Below
            // it, 60° sector snaps are expected hall behavior.
            let new_angle = mgr.get().angle;
            if out.omega_e > 100.0 {
                if let Some(prev) = prev_angle {
                    let jump = angle_difference(new_angle, prev).abs();
                    max_step_at_speed = max_step_at_speed.max(jump);
                }
                prev_angle = Some(new_angle);
            } else {
                prev_angle = None;
            }
        }

        (mgr, out, max_step_at_speed)
    }

    /// Spin-up through the blend band to pure observer, halls healthy.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn closed_loop_hall_to_observer_crossover() {
        const DT: f32 = 1.0 / 20_000.0;
        let (mgr, out, max_step_at_speed) = run_sensorless_sim(20_000, |_, s| Some(s));

        // Motor must have spun up past the blend band → pure observer.
        assert!(
            out.omega_e > 400.0,
            "motor did not spin up: ωe = {}",
            out.omega_e
        );
        assert!(
            mgr.observer().is_ready(),
            "observer must be converged at speed"
        );

        // Managed angle tracks the true rotor angle.
        let true_angle = wrap_angle(out.angle_rad);
        let err = angle_difference(mgr.get().angle, true_angle).abs();
        assert!(
            err < 0.25,
            "managed angle {} vs true {} (err {} rad)",
            mgr.get().angle,
            true_angle,
            err
        );

        // Velocity estimate agrees with the true electrical speed.
        let vel_err = (mgr.get().velocity - out.omega_e).abs() / out.omega_e;
        assert!(
            vel_err < 0.10,
            "managed velocity {} vs true {} ({}%)",
            mgr.get().velocity,
            out.omega_e,
            vel_err * 100.0
        );

        // No angle discontinuities across the whole crossover.
        let nominal_step = out.omega_e * DT;
        assert!(
            max_step_at_speed < nominal_step + 0.15,
            "angle jumped {max_step_at_speed} rad in one cycle (nominal step {nominal_step})"
        );
    }

    /// Hall cable cut at full speed: the manager must detect the stale
    /// sensor (edges stopped arriving although the rotor demonstrably
    /// spins), raise HallTimeout, and keep tracking on the observer alone.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn closed_loop_hall_dropout_at_speed() {
        // 1 s with halls, then 0.3 s with the cable cut.
        let (mgr, out, max_step_at_speed) =
            run_sensorless_sim(26_000, |step, s| (step < 20_000).then_some(s));

        assert!(
            out.omega_e > 400.0,
            "motor must still be spinning: ωe = {}",
            out.omega_e
        );
        assert!(
            mgr.has_fault(PhaseFault::HallTimeout),
            "stale hall at speed must raise HallTimeout (health: {:?})",
            mgr.hall_health()
        );

        // Observer must be carrying the commutation accurately.
        let true_angle = wrap_angle(out.angle_rad);
        let err = angle_difference(mgr.get().angle, true_angle).abs();
        assert!(
            err < 0.25,
            "angle lost after hall dropout: managed {} vs true {} (err {})",
            mgr.get().angle,
            true_angle,
            err
        );

        // And the handoff itself must have been continuous.
        let nominal_step = out.omega_e * (1.0 / 20_000.0);
        assert!(
            max_step_at_speed < nominal_step + 0.15,
            "angle jumped {max_step_at_speed} rad during hall dropout handoff"
        );
    }

    /// Partial hall failure at speed: H1 goes stuck-low mid-ride. Above the
    /// blend band the observer carries commutation, so the ride continues
    /// smoothly — and the per-bit wire detector must name the dead wire
    /// from the corrupted edge stream (one invalid state per electrical
    /// revolution gates the verdict, see HallSensor::note_wire_activity).
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn closed_loop_partial_hall_failure_rides_through_and_names_wire() {
        use crate::foc::hall_sensor::HallFaultKind;

        const FAIL_AT: u64 = 20_000; // 1 s spin-up, then the wire breaks
        let (mgr, out, max_step_at_speed) = run_sensorless_sim(40_000, |step, s| {
            Some(if step < FAIL_AT { s } else { s & 0b110 })
        });

        assert!(
            out.omega_e > 400.0,
            "motor must ride through a partial hall failure: ωe = {}",
            out.omega_e
        );
        assert_eq!(
            mgr.hall().fault_kind(),
            Some(HallFaultKind::WireDead { mask: 0b001 }),
            "the dead wire must be named (health: {:?})",
            mgr.hall_health()
        );

        // Commutation stayed accurate and continuous on the observer.
        let true_angle = wrap_angle(out.angle_rad);
        let err = angle_difference(mgr.get().angle, true_angle).abs();
        assert!(err < 0.25, "angle err {err} rad with a dead hall wire");
        let nominal_step = out.omega_e * (1.0 / 20_000.0);
        assert!(
            max_step_at_speed < nominal_step + 0.15,
            "angle jumped {max_step_at_speed} rad during partial-failure ride-through"
        );
    }

    /// Full sensorless lifecycle on the dual estimator slots: HFI finds a
    /// π-flipped rotor at standstill (saturation probe corrects it), the
    /// motor accelerates on the HFI angle, and the manager crosses over to
    /// the back-EMF observer at speed — continuously, with the carrier
    /// injection shut off once latched.
    #[test]
    #[cfg(feature = "virtual-motor")]
    #[cfg(feature = "hfi")]
    fn closed_loop_hfi_to_observer_crossover() {
        use crate::foc::controller::FocController;
        use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const MIN_VEL: f32 = 150.0;
        // Salient IPM with d-axis saturation (polarity probe needs it).
        // Light enough rotor to accelerate within the sim, enough friction
        // to settle around ωe ≈ 500 rad/s — well above the crossover.
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-3,
            friction_b: 2e-3,
            hall_offset: 0.0,
            sat_k: 0.05,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);
        // Far side: the HFI PLL's nearest saliency lock is the flipped one.
        motor.set_angle(2.5);

        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );

        let mut mgr = PhaseManager::sensorless();
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            (params.ld + params.lq) / 2.0,
            params.lambda,
        )));
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::HfiToObserver {
            min_vel: MIN_VEL,
            min_confidence: 0.5,
        })
        .unwrap();

        let mut out = VirtualMotorOutput {
            angle_rad: 2.5,
            ..Default::default()
        };
        let mut prev_angle: Option<f32> = None;
        let mut max_step_at_speed = 0.0f32;

        // Phase 1: standstill, zero torque — HFI must lock and resolve
        // polarity. Phase 2: torque on — spin up through the crossover.
        // Phase 3: torque off — coast back down through the band (reverse
        // handoff: observer → reseeded HFI).
        const STANDSTILL_STEPS: u64 = 10_000;
        const SPIN_STEPS: u64 = 40_000;
        const TOTAL_STEPS: u64 = 80_000;
        let mut spun_up = false;
        let mut was_latched_at_speed = false;
        // Downward-handoff instrumentation: the carrier must resume in the
        // pre-heat margin while still latched, and the demod must be locked
        // by the moment the latch releases (blend weight returns to HFI).
        let mut preheat_seen = false;
        let mut released_with_ready: Option<bool> = None;
        let mut was_latched = false;
        for step in 1..TOTAL_STEPS {
            let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
                2.0
            } else {
                0.0
            };

            let angle = mgr.get().angle;
            let (vd_inj, vq_inj) = mgr.injection();
            let telem = foc.step_with_injection(
                (out.ia, out.ib, out.ic),
                angle,
                0.0, // velocity (no decoupling configured here)
                0.0, // id_target
                iq_target,
                vd_inj,
                vq_inj,
                1000,
                DT,
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            mgr.update(
                &PhaseInput {
                    v_alpha: telem.v_alpha,
                    v_beta: telem.v_beta,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                },
                step * 50,
            );

            if step == SPIN_STEPS - 1 {
                spun_up = out.omega_e > MIN_VEL * 1.5;
                // Fully blended at speed → the carrier must be off.
                was_latched_at_speed = mgr.injection() == (0.0, 0.0);
            }

            // Downward handoff (coast phase): pre-heat + lock-before-weight.
            if step > SPIN_STEPS {
                let latched_now = mgr.crossover_latched;
                if latched_now && mgr.injection() != (0.0, 0.0) {
                    preheat_seen = true;
                }
                if was_latched && !latched_now && released_with_ready.is_none() {
                    released_with_ready =
                        Some(mgr.hfi_observer().is_some_and(HfiObserver::is_ready));
                }
                was_latched = latched_now;
            } else {
                was_latched = mgr.crossover_latched;
            }

            if step == STANDSTILL_STEPS - 1 {
                // HFI must have found the flipped rotor at standstill.
                let err = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
                assert!(
                    err < 0.2,
                    "HFI standstill lock failed: est {} vs rotor {} (err {} full-circle)",
                    mgr.get().angle,
                    wrap_angle(out.angle_rad),
                    err
                );
            }

            // Continuity above the crossover band (same check as the hall
            // crossover sims).
            let new_angle = mgr.get().angle;
            if out.omega_e > 100.0 {
                if let Some(prev) = prev_angle {
                    let jump = angle_difference(new_angle, prev).abs();
                    max_step_at_speed = max_step_at_speed.max(jump);
                }
                prev_angle = Some(new_angle);
            } else {
                prev_angle = None;
            }
        }

        assert!(spun_up, "motor did not spin up during the torque phase");
        assert!(
            was_latched_at_speed,
            "carrier injection must stop once fully blended onto the observer"
        );

        // Phase 3 outcome: coasted back down — HFI carries commutation
        // again (carrier on) and still tracks the slowing rotor.
        assert!(
            out.omega_e.abs() < MIN_VEL,
            "motor should have coasted below the band: ωe = {}",
            out.omega_e
        );
        assert_ne!(
            mgr.injection(),
            (0.0, 0.0),
            "carrier must be back on below the crossover band"
        );
        let true_angle = wrap_angle(out.angle_rad);
        let err = angle_difference(mgr.get().angle, true_angle).abs();
        assert!(
            err < 0.25,
            "managed angle {} vs true {} (err {} rad) after the down handoff",
            mgr.get().angle,
            true_angle,
            err
        );
        // Continuity through BOTH crossover directions.
        let nominal_step = 700.0 * DT; // generous bound: peak ωe during the run
        assert!(
            max_step_at_speed < nominal_step + 0.15,
            "angle jumped {max_step_at_speed} rad in one cycle through a crossover"
        );

        // Pre-heat: the carrier resumed in the margin while still latched —
        // i.e. before any blend weight could flow back to HFI…
        assert!(
            preheat_seen,
            "carrier must pre-heat inside the margin above the latch release"
        );
        // …and by the moment the latch released, the demod had re-locked
        // (confidence-before-weight). Without the pre-heat margin the demod
        // enters the band cold and this fails.
        assert_eq!(
            released_with_ready,
            Some(true),
            "HFI demod must be locked before blend weight returns to it"
        );
    }

    /// Mechanically-unbounded deceleration (wheel jam) crosses any finite
    /// pre-heat margin instantly — that tail is covered by the TRUST GATE,
    /// not the margin: the restarted demod reports zero confidence, so
    /// `angle_trustworthy()` must stay false (driver keeps iq at zero)
    /// until the lock is re-earned from real carrier current, and recover
    /// at standstill afterwards. This is the explicit scope split: the
    /// margin handles controller-bounded decel (test above), the gate
    /// handles the unbounded case (this test).
    #[test]
    #[cfg(feature = "virtual-motor")]
    #[cfg(feature = "hfi")]
    fn closed_loop_hfi_jam_gates_torque_until_relock() {
        use crate::foc::controller::FocController;
        use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const MIN_VEL: f32 = 150.0;
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-3,
            friction_b: 2e-3,
            hall_offset: 0.0,
            sat_k: 0.05,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);

        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );

        let mut mgr = PhaseManager::sensorless();
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            (params.ld + params.lq) / 2.0,
            params.lambda,
        )));
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::HfiToObserver {
            min_vel: MIN_VEL,
            min_confidence: 0.5,
        })
        .unwrap();

        let mut out = VirtualMotorOutput::default();

        // Phase 1: standstill HFI lock + polarity. Phase 2: spin up into
        // the latched regime. Phase 3: JAM — a brake torque far beyond the
        // motor's own, stopping the rotor within a few ms.
        const STANDSTILL_STEPS: u64 = 10_000;
        const SPIN_STEPS: u64 = 40_000;
        const TOTAL_STEPS: u64 = 60_000;

        let mut latched_at_speed = false;
        let mut cold_gate_seen = false;
        let mut trustworthy_during_cold = true;
        for step in 1..TOTAL_STEPS {
            let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
                2.0
            } else {
                0.0
            };
            // The jam: 50 N·m of external braking while the rotor still
            // turns forward (pp/J scaled, that is ~200 krad/s² — any
            // pre-heat margin is crossed in well under a millisecond).
            let load = if step >= SPIN_STEPS && out.omega_e > 2.0 {
                50.0
            } else {
                0.0
            };

            let angle = mgr.get().angle;
            let (vd_inj, vq_inj) = mgr.injection();
            let telem = foc.step_with_injection(
                (out.ia, out.ib, out.ic),
                angle,
                0.0,
                0.0,
                iq_target,
                vd_inj,
                vq_inj,
                1000,
                DT,
            );
            out = motor.step(telem.v_alpha, telem.v_beta, load, DT);

            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            mgr.update(
                &PhaseInput {
                    v_alpha: telem.v_alpha,
                    v_beta: telem.v_beta,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                },
                step * 50,
            );

            if step == SPIN_STEPS - 1 {
                latched_at_speed = mgr.crossover_latched && out.omega_e > MIN_VEL * 1.5;
            }

            // After the jam: the cold window is "latch released, demod not
            // yet ready". The trust gate must hold iq at zero throughout it.
            if step > SPIN_STEPS && !mgr.crossover_latched {
                let hfi_ready = mgr.hfi_observer().is_some_and(HfiObserver::is_ready);
                if !hfi_ready {
                    cold_gate_seen = true;
                    if mgr.angle_trustworthy() && !mgr.observer().is_ready() {
                        trustworthy_during_cold = false;
                    }
                }
            }
        }

        assert!(
            latched_at_speed,
            "motor must reach the latched regime before the jam"
        );
        assert!(
            out.omega_e.abs() < 10.0,
            "jam must have stopped the rotor: ωe = {}",
            out.omega_e
        );
        assert!(
            cold_gate_seen,
            "the jam must produce a cold-demod window after latch release"
        );
        assert!(
            trustworthy_during_cold,
            "angle_trustworthy() must be false while the demod is cold \
             (the driver's iq gate is the only cover for unbounded decel)"
        );
        // Recovery: at standstill the carrier is on, the demod re-locks and
        // commutation trust returns.
        assert!(
            mgr.hfi_observer().is_some_and(HfiObserver::is_ready),
            "HFI must re-lock at standstill after the jam"
        );
        assert!(
            mgr.angle_trustworthy(),
            "trust must recover once the demod is locked"
        );
    }

    /// Saliency collapse under load — the classic HFI failure mode the
    /// plant can now produce (`lq_sat_k`): torque current saturates the
    /// q-axis iron until `Lq_eff ≈ Ld`, the demod error signal loses its
    /// gradient and the angle estimate stops being corrected while the
    /// rotor keeps moving.
    ///
    /// The failure is INSIDIOUS: the carrier amplitude — and with it the
    /// demod confidence — stays healthy (eps ≈ 0 reads as "no error"), so
    /// no existing gate fires. This test pins both halves: the linear
    /// control run keeps tracking under the identical load profile, the
    /// saturating run silently loses the rotor at high confidence. The
    /// detection gap (saliency monitor / dual-axis HFI45) is a TODO.
    #[test]
    #[cfg(feature = "virtual-motor")]
    #[cfg(feature = "hfi")]
    fn closed_loop_hfi_saliency_collapse_loses_tracking_silently() {
        use crate::foc::controller::FocController;
        use crate::foc::phase::HfiObserver;
        use crate::foc::phase::observer::HFI_READY_CONFIDENCE;
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const LOCK_STEPS: u64 = 10_000;
        const TOTAL_STEPS: u64 = 16_000;
        const IQ_LOAD: f32 = 10.0;

        // (max tracking error during the loaded window, min HFI confidence
        // during that window)
        let run = |lq_sat_k: f32| -> (f32, f32) {
            // 3:1 IPM; at iq = 10 A with lq_sat_k = 0.3 the q-axis
            // saturation factor clamps at 4× → Lq_eff = 75 µH < Ld: the
            // saliency does not just vanish, it INVERTS, flipping the
            // PLL equilibrium to unstable (the catastrophic variant of
            // the failure).
            let params = MotorParams {
                r: 0.1,
                ld: 100e-6,
                lq: 300e-6,
                lambda: 0.02,
                pole_pairs: 4,
                j: 2e-3,
                friction_b: 1e-3,
                sat_k: 0.05,
                lq_sat_k,
                ..MotorParams::default()
            };
            let mut motor = VirtualMotor::new(params);
            let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
                params.r,
                (params.ld + params.lq) / 2.0,
                24.0,
            );

            let mut mgr = PhaseManager::sensorless();
            mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
            mgr.set_source(PhaseSource::Hfi).unwrap();

            let mut out = VirtualMotorOutput::default();
            let mut max_err = 0.0f32;
            let mut min_conf = 1.0f32;
            for step in 1..TOTAL_STEPS {
                // Phase 1: standstill lock + polarity. Phase 2: heavy
                // torque current against a load that nearly balances it —
                // the rotor creeps slowly (HFI regime) while iq saturates
                // the q axis.
                let (iq_target, load) = if step < LOCK_STEPS {
                    (0.0, 0.0)
                } else {
                    (IQ_LOAD, 1.05)
                };

                let angle = mgr.get().angle;
                let (vd_inj, vq_inj) = mgr.injection();
                let telem = foc.step_with_injection(
                    (out.ia, out.ib, out.ic),
                    angle,
                    0.0,
                    0.0,
                    iq_target,
                    vd_inj,
                    vq_inj,
                    1000,
                    DT,
                );
                out = motor.step(telem.v_alpha, telem.v_beta, load, DT);

                let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
                mgr.update(
                    &PhaseInput {
                        v_alpha: telem.v_alpha,
                        v_beta: telem.v_beta,
                        i_alpha: i_a,
                        i_beta: i_b,
                        dt: DT,
                    },
                    step * 50,
                );

                if step >= LOCK_STEPS + 1_000 {
                    let err = angle_difference(mgr.get().angle, out.angle_rad).abs();
                    max_err = max_err.max(err);
                    if let Some(h) = mgr.hfi_observer() {
                        min_conf = min_conf.min(h.confidence());
                    }
                }
            }
            (max_err, min_conf)
        };

        let (err_linear, _) = run(0.0);
        let (err_collapsed, conf_collapsed) = run(0.3);

        assert!(
            err_linear < 0.35,
            "linear plant: HFI must track the creeping rotor under load (max err {err_linear})"
        );
        assert!(
            err_collapsed > 0.7,
            "saturating plant: saliency collapse must lose the rotor (max err {err_collapsed})"
        );
        // The documented blind spot: confidence does NOT see the collapse
        // (eps ≈ 0 looks like a perfect lock). If this ever starts failing
        // because confidence drops, a saliency monitor exists — update the
        // test and close the TODO.
        assert!(
            conf_collapsed >= HFI_READY_CONFIDENCE,
            "expected the collapse to stay invisible to confidence, got {conf_collapsed}"
        );
    }

    /// The voltage-criterion crossover (HfiToObserverVolts) must behave
    /// like the velocity one: carrier on at standstill, handoff to the
    /// observer as the drive voltage (back-EMF proxy |vq − R·iq|) rises,
    /// carrier off once latched, reseeded return to HFI on coast-down —
    /// with the threshold in volts instead of per-motor eRPM tuning.
    #[test]
    #[cfg(feature = "virtual-motor")]
    #[cfg(feature = "hfi")]
    fn closed_loop_hfi_to_observer_volts_crossover() {
        use crate::foc::controller::FocController;
        use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        // ωe settles ≈ 500 rad/s → bemf ≈ ω·λ = 10 V ≫ toggle; at
        // standstill the proxy is ≈ 0 V. Crossover happens around
        // ω ≈ toggle_v/λ = 150 rad/s, same regime as the velocity test.
        const TOGGLE_V: f32 = 3.0;
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-3,
            friction_b: 2e-3,
            hall_offset: 0.0,
            sat_k: 0.05,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(0.5);

        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );

        let mut mgr = PhaseManager::sensorless();
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            (params.ld + params.lq) / 2.0,
            params.lambda,
        )));
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::HfiToObserverVolts {
            toggle_v: TOGGLE_V,
            min_confidence: 0.5,
        })
        .unwrap();

        let mut out = VirtualMotorOutput {
            angle_rad: 0.5,
            ..Default::default()
        };

        const STANDSTILL_STEPS: u64 = 10_000;
        const SPIN_STEPS: u64 = 40_000;
        const TOTAL_STEPS: u64 = 70_000;
        let mut latched_at_speed = false;
        let mut err_at_speed = 0.0f32;
        for step in 1..TOTAL_STEPS {
            let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
                2.0
            } else {
                0.0
            };

            let angle = mgr.get().angle;
            let (vd_inj, vq_inj) = mgr.injection();
            let telem = foc.step_with_injection(
                (out.ia, out.ib, out.ic),
                angle,
                0.0,
                0.0,
                iq_target,
                vd_inj,
                vq_inj,
                1000,
                DT,
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            mgr.update(
                &PhaseInput {
                    v_alpha: telem.v_alpha,
                    v_beta: telem.v_beta,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                },
                step * 50,
            );

            if step == SPIN_STEPS - 1 {
                assert!(
                    out.omega_e > 300.0,
                    "motor did not spin up: ωe = {}",
                    out.omega_e
                );
                latched_at_speed = mgr.injection() == (0.0, 0.0);
                err_at_speed = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
            }
        }

        assert!(
            latched_at_speed,
            "carrier must be off once the drive voltage exceeds toggle_v"
        );
        assert!(
            err_at_speed < 0.25,
            "angle error {err_at_speed} rad at speed on the voltage crossover"
        );

        // Coasted down: bemf proxy below the band → back on HFI, carrier on.
        assert!(
            out.omega_e.abs() * params.lambda < TOGGLE_V - HFI_VOLTS_HYSTERESIS,
            "motor should have coasted below the voltage band: ωe = {}",
            out.omega_e
        );
        assert_ne!(
            mgr.injection(),
            (0.0, 0.0),
            "carrier must be back on below the voltage band"
        );
        let err = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
        assert!(
            err < 0.3,
            "angle error {err} rad after the downward voltage handoff"
        );
    }

    /// Sources that don't use the hall sensor must not raise hall faults:
    /// a manager doing Manual-angle calibration with an idle (or absent)
    /// hall signal is not a failure condition.
    #[test]
    fn no_hall_faults_in_sources_that_ignore_hall() {
        let mut mock_hall = MockHallSensor::new();
        mock_hall.set_valid(false); // nothing on the hall lines
        let mut phase = PhaseManager::with_hall(mock_hall);
        phase.set_source(PhaseSource::Manual).unwrap();

        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            1000,
        );

        assert!(
            phase.faults().is_empty(),
            "Manual source must not raise hall faults, got {:?}",
            phase.faults()
        );
        assert!(!phase.is_open_loop_override_active());
    }

    #[test]
    fn test_open_loop_override_deactivates_when_observer_ready() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        // Make Hall fail first (will activate open-loop override)
        phase.hall_mut().set_valid(false);
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            0,
        );

        assert!(phase.is_open_loop_override_active());

        // Now configure an observer with valid phase
        let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
        observer.force_phase(2.0);
        observer.set_velocity(300.0);
        phase.set_observer(Observer::BackEmf(observer));

        // Update - observer should take over and deactivate open-loop override
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            1000,
        );

        // Open-loop override should be deactivated
        assert!(!phase.is_open_loop_override_active());

        // Output should come from observer (velocity matches)
        // Note: angle may have drifted slightly due to update() but velocity should match
        let output = phase.get();
        assert!((output.velocity - 300.0).abs() < 10.0); // Velocity should be from observer
    }

    /// Switching to a non-hall source must clear a live open-loop override:
    /// the override only belongs to the hall source that armed it, and its
    /// deactivation paths are all `requires_hall()`-gated. Left stranded, it
    /// pins `angle_trustworthy()` false forever on a healthy encoder (whose
    /// trust reads `!override.active`), and the driver coasts indefinitely.
    #[test]
    fn set_source_clears_stranded_open_loop_override() {
        let mut phase =
            PhaseManager::with_hall(MockHallSensor::new()).with_encoder(MockHallSensor::new());

        // Arm the override: hall dies with no observer ready.
        phase.hall_mut().set_valid(false);
        phase.update(
            &PhaseInput {
                dt: 0.001,
                ..Default::default()
            },
            0,
        );
        assert!(
            phase.is_open_loop_override_active(),
            "hall failure with no observer must arm the open-loop override"
        );

        // Host switches to the encoder — a non-hall source.
        phase
            .set_source(PhaseSource::Encoder)
            .expect("encoder is present");

        assert!(
            !phase.is_open_loop_override_active(),
            "switching to a non-hall source must clear the stranded override"
        );
        assert!(
            phase.angle_trustworthy(),
            "a healthy encoder must be trustworthy once the override is cleared"
        );
    }
}
