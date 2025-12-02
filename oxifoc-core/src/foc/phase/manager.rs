//! Phase manager for FOC control
//!
//! Manages multiple angle sources (Hall, Encoder, Observer) and provides
//! a unified interface to FocDriver via the PhaseProvider trait.

use core::f32::consts::TAU;

use super::observer::{Observer, ObserverInput};
use super::provider::{PhaseInput, PhaseOutput, PhaseProvider};
use super::source::{PhaseSource, PhaseSourceError};
use crate::foc::hall_calibration::HallCalibrationResult;
use crate::foc::sensors::{AngleSample, AngleSensor, HallSensorTrait, NoSensor};

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
    // Internal phase computation
    // ========================================================================

    /// Compute phase output based on current source
    fn compute_phase(
        &self,
        hall_sample: Option<AngleSample>,
        encoder_sample: Option<AngleSample>,
    ) -> PhaseOutput {
        match self.source {
            PhaseSource::Hall => sample_to_output(hall_sample, &self.output),

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

        // Update observer
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

        // Compute output based on source
        self.output = self.compute_phase(hall_sample, encoder_sample);
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
}
