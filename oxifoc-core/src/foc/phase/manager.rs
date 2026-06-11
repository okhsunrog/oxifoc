//! Phase manager for FOC control
//!
//! Manages multiple angle sources (Hall, Encoder, Observer) and provides
//! a unified interface to FocDriver via the PhaseProvider trait.

use core::f32::consts::TAU;

use heapless::Vec as HeaplessVec;

use super::observer::{HfiObserver, Observer, ObserverInput};
use super::provider::{PhaseInput, PhaseOutput, PhaseProvider};
use super::source::{PhaseSource, PhaseSourceError};
use crate::foc::hall_calibration::HallCalibrationResult;
use crate::foc::sensors::{AngleSample, AngleSensor, HallSensorTrait, NoSensor};
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::foc::{angle_difference, wrap_angle};

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
    hfi: Option<HfiObserver<S>>,

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

    // Hysteresis memory for the HfiToX crossovers: true = running on the
    // high-speed source (observer/hall/encoder), false = on HFI.
    crossover_latched: bool,

    // |vq − R·iq| of the last update — back-EMF share of the drive voltage,
    // the regime signal for the voltage-based HFI crossover.
    bemf_proxy_v: f32,

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
            hfi: None,
            source: PhaseSource::Manual,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            crossover_latched: false,
            bemf_proxy_v: 0.0,
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
            hfi: None,
            source: PhaseSource::Hall,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::Ok,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            crossover_latched: false,
            bemf_proxy_v: 0.0,
            faults: HeaplessVec::new(),
        }
    }

    /// Add an encoder to the phase manager
    pub fn with_encoder<E2: AngleSensor>(self, encoder: E2) -> PhaseManager<H, E2> {
        PhaseManager {
            hall: self.hall,
            encoder,
            observer: self.observer,
            hfi: self.hfi,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            hall_failure_ticks: self.hall_failure_ticks,
            open_loop_override: self.open_loop_override,
            crossover_latched: self.crossover_latched,
            bemf_proxy_v: self.bemf_proxy_v,
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
            hfi: self.hfi.map(HfiObserver::with_sincos),
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            hall_failure_ticks: self.hall_failure_ticks,
            open_loop_override: self.open_loop_override,
            crossover_latched: self.crossover_latched,
            bemf_proxy_v: self.bemf_proxy_v,
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
            hfi: None,
            source: PhaseSource::Encoder,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            crossover_latched: false,
            bemf_proxy_v: 0.0,
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
        // the source would silently never produce an estimate.
        if source.requires_hfi() && self.hfi.is_none() {
            return Err(PhaseSourceError::HfiNotConfigured);
        }

        self.source = source;
        // Crossover memory belongs to the previous source's thresholds.
        self.crossover_latched = false;
        Ok(())
    }

    /// Set observer
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
    pub fn set_hfi_observer(&mut self, hfi: HfiObserver<S>) {
        self.hfi = Some(hfi);
    }

    /// Get HFI estimator reference
    pub fn hfi_observer(&self) -> Option<&HfiObserver<S>> {
        self.hfi.as_ref()
    }

    /// Get mutable HFI estimator reference
    pub fn hfi_observer_mut(&mut self) -> Option<&mut HfiObserver<S>> {
        self.hfi.as_mut()
    }

    /// Current HFI estimate as a phase output (None if no HFI configured)
    fn hfi_output(&self) -> Option<PhaseOutput> {
        self.hfi.as_ref().map(|h| PhaseOutput {
            angle: h.phase(),
            velocity: h.velocity(),
        })
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

    /// Compute phase output with automatic fallback on Hall failure
    fn compute_phase_with_fallback(
        &mut self,
        hall_sample: Option<AngleSample>,
        encoder_sample: Option<AngleSample>,
    ) -> PhaseOutput {
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
                // Pure sensorless: only commutate from a converged observer;
                // hold the last output (and raise the fault) otherwise.
                if self.observer.is_ready() {
                    self.clear_fault(PhaseFault::ObserverNotReady);
                    match (self.observer.phase(), self.observer.velocity()) {
                        (Some(angle), Some(vel)) => PhaseOutput {
                            angle,
                            velocity: vel,
                        },
                        _ => self.output,
                    }
                } else {
                    self.set_fault(PhaseFault::ObserverNotReady);
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
                // If Hall failed, go full observer
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

            PhaseSource::HallWithFallback {
                blend_low,
                blend_high,
                timeout_us: _, // TODO: Use for timeout detection
            } => {
                // VESC-style full Hall mode:
                // 1. If Hall failed, use observer
                // 2. Otherwise blend Hall→Observer based on velocity
                if hall_sample.is_none() {
                    // Hall failed - try observer fallback
                    // TODO: If observer also not ready, use open-loop override
                    return self.try_observer_fallback().unwrap_or(self.output);
                }

                // Hall is working - blend with observer based on velocity
                let sensor = sample_to_output(hall_sample, &self.output);
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

    /// Whether the HFI estimate is currently carrying (or may need to
    /// carry) commutation, so its carrier must be injected. Above the
    /// crossover latch the fast source commutates and injection stops —
    /// keeping the carrier on at speed only costs losses and acoustic
    /// noise while the saliency response degrades anyway.
    fn hfi_injection_active(&self) -> bool {
        match self.source {
            PhaseSource::Hfi => true,
            PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => !self.crossover_latched,
            _ => false,
        }
    }

    /// Reseed the HFI estimator from the last managed output (downward
    /// crossover handoff). The angle comes from a source that was trusted
    /// for commutation, so this also resolves the HFI π ambiguity.
    fn seed_hfi_from_output(&mut self) {
        let out = self.output;
        if let Some(hfi) = &mut self.hfi {
            hfi.set_phase(out.angle);
            hfi.set_velocity(out.velocity);
        }
    }

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

    fn update(&mut self, input: &PhaseInput, now_ticks: u64) {
        // Sample hardware sensors. The stateful path matters for hall: it
        // carries the rate limiter that smooths sector-edge discontinuities.
        // A stale hall (edges stopped while spinning) is treated as having
        // no sample at all, so every consumer below falls back uniformly.
        let hall_stale = self.hall.is_stale(now_ticks);
        let hall_sample = if hall_stale {
            None
        } else {
            self.hall.sample_mut(now_ticks)
        };
        let encoder_sample = self.encoder.sample_mut(now_ticks);

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
            let (_, vq) = crate::foc::transforms::park(input.v_alpha, input.v_beta, sin_t, cos_t);
            let (_, iq) = crate::foc::transforms::park(input.i_alpha, input.i_beta, sin_t, cos_t);
            let bemf = vq - r * iq;
            self.bemf_proxy_v = if bemf < 0.0 { -bemf } else { bemf };
        }

        // Update both estimators (always run, for fallback/crossover)
        let obs_input = ObserverInput {
            v_alpha: input.v_alpha,
            v_beta: input.v_beta,
            i_alpha: input.i_alpha,
            i_beta: input.i_beta,
            dt: input.dt,
        };
        self.observer.update(&obs_input);
        if let Some(hfi) = &mut self.hfi {
            hfi.update(&obs_input);
        }

        // Advance open-loop angle if in OpenLoop mode
        if matches!(self.source, PhaseSource::OpenLoop) {
            self.open_loop_angle += self.open_loop_velocity * input.dt;
            self.open_loop_angle = wrap_angle(self.open_loop_angle);
        }

        // Update open-loop override state (for Hall failure recovery)
        self.update_open_loop_override(input.dt);

        // Compute output based on source (with potential fallback)
        self.output = self.compute_phase_with_fallback(hall_sample, encoder_sample);
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        self.set_source(source).is_ok()
    }

    /// Trustworthy down to standstill when a hardware sensor backs the active
    /// source (Hall/Encoder track to a stop), or when HFI is locked (valid at
    /// zero speed by design). A pure back-EMF observer is only trusted while
    /// `is_ready()` — it drops below its speed floor, so the failsafe brake
    /// coasts the last bit instead of commutating blind.
    fn angle_trustworthy(&self) -> bool {
        match self.source {
            PhaseSource::Hall
            | PhaseSource::Encoder
            | PhaseSource::Manual
            | PhaseSource::OpenLoop
            | PhaseSource::HallToObserver { .. }
            | PhaseSource::HallWithFallback { .. }
            | PhaseSource::EncoderToObserver { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => true,
            PhaseSource::Observer => self.observer.is_ready(),
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. } => {
                self.hfi.as_ref().is_some_and(|h| h.is_ready()) || self.observer.is_ready()
            }
        }
    }

    fn injection(&self) -> (f32, f32) {
        if self.hfi_injection_active()
            && let Some(hfi) = &self.hfi
        {
            hfi.get_injection()
        } else {
            (0.0, 0.0)
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
    pub fn configure_observers_from_config(
        &mut self,
        config: &crate::storage::RuntimeConfig,
        vbus: f32,
    ) {
        use super::observer::{
            BackEmfObserver, HFI_DEFAULT_AMPLITUDE_RATIO, HFI_DEFAULT_FREQ_HZ, Observer,
        };

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
            self.set_hfi_observer(
                HfiObserver::new(HFI_DEFAULT_FREQ_HZ, vbus * HFI_DEFAULT_AMPLITUDE_RATIO)
                    .with_sincos(),
            );
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
        assert!(vd.abs() > 1.0, "expected carrier voltage, got vd = {}", vd);
        assert_eq!(vq, 0.0, "pulsating injection is d-axis only");

        // Crossover source: inject below the latch, stop above it.
        phase
            .set_source(PhaseSource::HfiToObserver {
                min_vel: 100.0,
                min_confidence: 0.5,
            })
            .unwrap();
        assert!(phase.injection().0.abs() > 1.0);
        phase.crossover_latched = true;
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
                    direction: crate::foc::hall_sensor::Direction::Clockwise,
                })
            } else {
                None
            }
        }

        fn read_angle(&self) -> f32 {
            self.angle
        }

        fn read_direction(&self) -> crate::foc::hall_sensor::Direction {
            crate::foc::hall_sensor::Direction::Clockwise
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
        let err = crate::foc::angle_difference(out.angle, hall_angle).abs();
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
        let err = crate::foc::angle_difference(out.angle, hall_angle).abs();
        assert!(
            err < 0.05,
            "output {} must equal the hall angle {} while observer is not ready (err {})",
            out.angle,
            hall_angle,
            err
        );
    }

    /// Closed-loop sensorless harness: VirtualMotor + FocController +
    /// PhaseManager(HallSensor + BackEmfObserver), HallWithFallback source.
    ///
    /// Spins from standstill on hall commutation; hall edges stop being fed
    /// after `hall_until_step` (cable-cut simulation). Returns the manager,
    /// final motor output and the largest one-cycle angle jump seen above
    /// the hall interpolation regime.
    #[cfg(feature = "virtual-motor")]
    fn run_sensorless_sim(
        total_steps: u64,
        hall_until_step: u64,
    ) -> (
        PhaseManager<crate::foc::hall_sensor::HallSensor>,
        crate::virtual_motor::VirtualMotorOutput,
        f32,
    ) {
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
        let mut last_hall_state = out.hall_state;

        let mut mgr = PhaseManager::with_hall(hall);
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            (params.ld + params.lq) / 2.0,
            params.lambda,
        )));
        mgr.set_source(PhaseSource::HallWithFallback {
            blend_low: 150.0,
            blend_high: 300.0,
            timeout_us: 100_000,
        })
        .unwrap();

        let iq_target = 2.0;
        let mut prev_angle: Option<f32> = None;
        let mut max_step_at_speed = 0.0f32;

        for step in 1..total_steps {
            let t_us = step * 50;

            if step < hall_until_step && out.hall_state != last_hall_state {
                mgr.hall_mut().update(out.hall_state, t_us);
                last_hall_state = out.hall_state;
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
                    let jump = crate::foc::angle_difference(new_angle, prev).abs();
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
        let (mgr, out, max_step_at_speed) = run_sensorless_sim(20_000, u64::MAX);

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
        let err = crate::foc::angle_difference(mgr.get().angle, true_angle).abs();
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
            "angle jumped {} rad in one cycle (nominal step {})",
            max_step_at_speed,
            nominal_step
        );
    }

    /// Hall cable cut at full speed: the manager must detect the stale
    /// sensor (edges stopped arriving although the rotor demonstrably
    /// spins), raise HallTimeout, and keep tracking on the observer alone.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn closed_loop_hall_dropout_at_speed() {
        // 1 s with halls, then 0.3 s with the cable cut.
        let (mgr, out, max_step_at_speed) = run_sensorless_sim(26_000, 20_000);

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
        let err = crate::foc::angle_difference(mgr.get().angle, true_angle).abs();
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
            "angle jumped {} rad during hall dropout handoff",
            max_step_at_speed
        );
    }

    /// Full sensorless lifecycle on the dual estimator slots: HFI finds a
    /// π-flipped rotor at standstill (saturation probe corrects it), the
    /// motor accelerates on the HFI angle, and the manager crosses over to
    /// the back-EMF observer at speed — continuously, with the carrier
    /// injection shut off once latched.
    #[test]
    #[cfg(feature = "virtual-motor")]
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

        let mut out = crate::virtual_motor::VirtualMotorOutput {
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

            if step == STANDSTILL_STEPS - 1 {
                // HFI must have found the flipped rotor at standstill.
                let err =
                    crate::foc::angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
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
                    let jump = crate::foc::angle_difference(new_angle, prev).abs();
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
        let err = crate::foc::angle_difference(mgr.get().angle, true_angle).abs();
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
            "angle jumped {} rad in one cycle through a crossover",
            max_step_at_speed
        );
    }

    /// The voltage-criterion crossover (HfiToObserverVolts) must behave
    /// like the velocity one: carrier on at standstill, handoff to the
    /// observer as the drive voltage (back-EMF proxy |vq − R·iq|) rises,
    /// carrier off once latched, reseeded return to HFI on coast-down —
    /// with the threshold in volts instead of per-motor eRPM tuning.
    #[test]
    #[cfg(feature = "virtual-motor")]
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

        let mut out = crate::virtual_motor::VirtualMotorOutput {
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
                err_at_speed =
                    crate::foc::angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
            }
        }

        assert!(
            latched_at_speed,
            "carrier must be off once the drive voltage exceeds toggle_v"
        );
        assert!(
            err_at_speed < 0.25,
            "angle error {} rad at speed on the voltage crossover",
            err_at_speed
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
        let err = crate::foc::angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
        assert!(
            err < 0.3,
            "angle error {} rad after the downward voltage handoff",
            err
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
}
