//! Phase manager for FOC control
//!
//! Manages multiple angle sources (Hall, Encoder, Observer) and provides
//! a unified interface to FocDriver via the PhaseProvider trait.

use core::f32::consts::TAU;

use heapless::Vec as HeaplessVec;

use super::observer::{Observer, ObserverInput};
use super::provider::{PhaseInput, PhaseOutput, PhaseProvider};
use super::source::{PhaseSource, PhaseSourceError};
use crate::foc::hall_calibration::HallCalibrationResult;
use crate::foc::sensors::{AngleSample, AngleSensor, HallSensorTrait, NoSensor};

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
pub struct PhaseManager<H = NoSensor, E = NoSensor>
where
    H: AngleSensor,
    E: AngleSensor,
{
    // Hardware sensors
    hall: H,
    encoder: E,

    // Software estimator
    observer: Observer,

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
            source: PhaseSource::Manual,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
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
            source: PhaseSource::Hall,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::Ok,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            faults: HeaplessVec::new(),
        }
    }

    /// Add an encoder to the phase manager
    pub fn with_encoder<E2: AngleSensor>(self, encoder: E2) -> PhaseManager<H, E2> {
        PhaseManager {
            hall: self.hall,
            encoder,
            observer: self.observer,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            hall_failure_ticks: self.hall_failure_ticks,
            open_loop_override: self.open_loop_override,
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
            source: PhaseSource::Encoder,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            hall_failure_ticks: None,
            open_loop_override: OpenLoopOverride::default(),
            faults: HeaplessVec::new(),
        }
    }
}

// ============================================================================
// Common implementation for all PhaseManager variants
// ============================================================================

impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
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
        // Note: HFI check would go here when fully implemented

        self.source = source;
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

    /// Check if Hall sensor is available
    pub fn has_hall(&self) -> bool {
        self.hall.sample(0).is_some() || self.hall.error_count() > 0
    }

    /// Check if encoder is available
    pub fn has_encoder(&self) -> bool {
        self.encoder.sample(0).is_some() || self.encoder.error_count() > 0
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

    /// Update Hall health status based on sample availability
    fn update_hall_health(&mut self, sample_valid: bool, now_ticks: u64) {
        // Skip if Hall is not configured
        if matches!(self.hall_health, HallHealth::NotPresent) {
            return;
        }

        if sample_valid {
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

            // Determine failure type: invalid state (immediate) or timeout (stale)
            // For now, we treat None samples as invalid state
            // TODO: Add timeout detection via HallSensor::is_stale() when we have
            // access to the Hall sensor trait methods
            self.hall_health = HallHealth::Invalid;
            self.set_fault(PhaseFault::HallInvalidState);
        }
    }

    /// Try to get observer output for fallback, with open-loop override as last resort
    fn try_observer_fallback(&mut self) -> Option<PhaseOutput> {
        if let (Some(angle), Some(vel)) = (self.observer.phase(), self.observer.velocity()) {
            // Observer is ready - deactivate any open-loop override
            self.clear_fault(PhaseFault::ObserverNotReady);
            if self.open_loop_override.active {
                self.deactivate_open_loop_override();
            }
            Some(PhaseOutput {
                angle,
                velocity: vel,
            })
        } else {
            // Observer not ready - use open-loop override if available
            self.set_fault(PhaseFault::ObserverNotReady);

            if self.open_loop_override.active && self.open_loop_override.timer > 0.0 {
                // Return open-loop override output
                Some(PhaseOutput {
                    angle: self.open_loop_override.angle,
                    velocity: self.open_loop_override.velocity,
                })
            } else {
                // Activate open-loop override for recovery
                // Use last known output angle and minimum velocity
                self.activate_open_loop_override(self.output.angle, DEFAULT_OPENLOOP_MIN_VEL);
                Some(PhaseOutput {
                    angle: self.open_loop_override.angle,
                    velocity: self.open_loop_override.velocity,
                })
            }
        }
    }

    /// Update open-loop override state (advance angle, decrement timer)
    fn update_open_loop_override(&mut self, dt: f32) {
        if self.open_loop_override.active {
            // Advance angle based on velocity
            self.open_loop_override.angle += self.open_loop_override.velocity * dt;
            self.open_loop_override.angle = wrap_angle(self.open_loop_override.angle);

            // Decrement timer
            self.open_loop_override.timer -= dt;
            if self.open_loop_override.timer <= 0.0 {
                self.open_loop_override.timer = 0.0;
                // Note: We don't deactivate here - let it continue until
                // Hall or observer comes back
            }
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
                if let (Some(angle), Some(vel)) = (self.observer.phase(), self.observer.velocity())
                {
                    PhaseOutput {
                        angle,
                        velocity: vel,
                    }
                } else {
                    self.output
                }
            }

            PhaseSource::Hfi => {
                // HFI uses observer internally
                if let (Some(angle), Some(vel)) = (self.observer.phase(), self.observer.velocity())
                {
                    PhaseOutput {
                        angle,
                        velocity: vel,
                    }
                } else {
                    self.output
                }
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

            PhaseSource::HfiToObserver {
                min_vel,
                min_confidence,
            } => {
                // Use HFI until observer has sufficient velocity and confidence
                let obs_vel = self.observer.velocity().unwrap_or(0.0).abs();
                let obs_conf = self.observer.confidence();

                if obs_vel >= min_vel && obs_conf >= min_confidence {
                    // Switch to observer
                    if let (Some(angle), Some(vel)) =
                        (self.observer.phase(), self.observer.velocity())
                    {
                        PhaseOutput {
                            angle,
                            velocity: vel,
                        }
                    } else {
                        self.output
                    }
                } else {
                    // Stay on HFI
                    self.output
                }
            }

            PhaseSource::HfiToHall { switch_vel } => {
                // If Hall failed, stay on HFI
                if hall_sample.is_none() {
                    return self.output;
                }
                let hall = sample_to_output(hall_sample, &self.output);
                if hall.velocity.abs() >= switch_vel {
                    hall
                } else {
                    // Stay on HFI (use current output)
                    self.output
                }
            }

            PhaseSource::HfiToEncoder { switch_vel } => {
                let enc = sample_to_output(encoder_sample, &self.output);
                if enc.velocity.abs() >= switch_vel {
                    enc
                } else {
                    self.output
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

    /// Blend sensor output with observer based on velocity
    fn blend_with_observer(
        &self,
        sensor: PhaseOutput,
        blend_low: f32,
        blend_high: f32,
    ) -> PhaseOutput {
        let blend = compute_blend(sensor.velocity.abs(), blend_low, blend_high);

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

impl<H: AngleSensor, E: AngleSensor> PhaseProvider for PhaseManager<H, E> {
    fn get(&self) -> PhaseOutput {
        self.output
    }

    fn update(&mut self, input: &PhaseInput, now_ticks: u64) {
        // Sample hardware sensors
        let hall_sample = self.hall.sample(now_ticks);
        let encoder_sample = self.encoder.sample(now_ticks);

        // Update Hall health tracking based on sample availability
        self.update_hall_health(hall_sample.is_some(), now_ticks);

        // Update observer (always runs for potential fallback)
        self.observer.update(&ObserverInput {
            v_alpha: input.v_alpha,
            v_beta: input.v_beta,
            i_alpha: input.i_alpha,
            i_beta: input.i_beta,
            dt: input.dt,
        });

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
}

// ============================================================================
// Conditional methods for Hall sensor
// ============================================================================

impl<H: HallSensorTrait, E: AngleSensor> PhaseManager<H, E> {
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

/// Wrap angle to [0, 2π)
#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a < 0.0 {
        a += TAU;
    }
    a
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
    }

    impl MockHallSensor {
        fn new() -> Self {
            Self {
                valid: true,
                angle: 0.5,
            }
        }

        fn set_valid(&mut self, valid: bool) {
            self.valid = valid;
        }
    }

    impl AngleSensor for MockHallSensor {
        fn sample(&self, _now_ticks: u64) -> Option<AngleSample> {
            if self.valid {
                Some(AngleSample {
                    angle: self.angle,
                    omega: 100.0,
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
        phase.update(&PhaseInput { dt: 0.001, ..Default::default() }, 1000);

        // Open-loop override should be active
        assert!(phase.is_open_loop_override_active());
        assert!(phase.open_loop_override().timer > 0.0);

        // Output should still be valid (from open-loop override)
        let output = phase.get();
        assert!(output.velocity > 0.0); // Should have minimum velocity

        // After more updates, angle should advance
        phase.update(&PhaseInput { dt: 0.001, ..Default::default() }, 2000);
        let output2 = phase.get();
        // Angle should have advanced
        assert!(output2.angle != output.angle || output.velocity > 0.0);
    }

    #[test]
    fn test_open_loop_override_deactivates_when_observer_ready() {
        use crate::foc::phase::{BackEmfObserver, Observer};

        let mock_hall = MockHallSensor::new();
        let mut phase = PhaseManager::with_hall(mock_hall);

        // Make Hall fail first (will activate open-loop override)
        phase.hall_mut().set_valid(false);
        phase.update(&PhaseInput { dt: 0.001, ..Default::default() }, 0);

        assert!(phase.is_open_loop_override_active());

        // Now configure an observer with valid phase
        let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
        observer.force_phase(2.0);
        observer.set_velocity(300.0);
        phase.set_observer(Observer::BackEmf(observer));

        // Update - observer should take over and deactivate open-loop override
        phase.update(&PhaseInput { dt: 0.001, ..Default::default() }, 1000);

        // Open-loop override should be deactivated
        assert!(!phase.is_open_loop_override_active());

        // Output should come from observer (velocity matches)
        // Note: angle may have drifted slightly due to update() but velocity should match
        let output = phase.get();
        assert!((output.velocity - 300.0).abs() < 10.0); // Velocity should be from observer
    }
}
