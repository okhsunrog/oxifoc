//! Motor phase resistance measurement algorithm.
//!
//! Measures motor resistance by applying a DC current and measuring
//! the resulting voltage drop. The rotor is locked at electrical angle 0
//! using d-axis alignment.
//!
//! # Algorithm
//!
//! 1. Apply d-axis current at electrical angle 0 (locks rotor in place)
//! 2. Ramp current slowly to avoid transients
//! 3. Wait for thermal and electrical settling
//! 4. Sample steady-state voltage (Vd) and current (Id)
//! 5. Calculate R = Vd / Id
//!
//! # Current Selection (VESC-style)
//!
//! The test current is determined by motor size to prevent overheating:
//! 1. Start with current_max / 50
//! 2. Quick R measurement, increase by 1.5× each iteration
//! 3. Stop when I²R × 1.5 >= max_power_loss / 5
//! 4. Final accurate measurement at that safe current

use super::types::{DetectionError, MotorSize};
use crate::motor::dissipation::heat_current_limit;
use crate::units::{
    ConstZero, ElectricCurrent, ElectricPotential, ElectricalResistance, amps, ohms,
};

/// Minimum valid resistance (below this suggests short circuit)
pub(crate) const MIN_VALID_RESISTANCE: ElectricalResistance = ohms(0.001);

/// Maximum valid resistance (above this suggests open circuit)
pub(crate) const MAX_VALID_RESISTANCE: ElectricalResistance = ohms(100.0);

/// Minimum current for valid measurement
const MIN_VALID_CURRENT: ElectricCurrent = amps(0.1);

/// Accumulator for resistance measurement samples.
///
/// Collects voltage and current samples during steady-state operation
/// and computes the average resistance.
#[derive(Clone, Debug)]
pub struct ResistanceMeasurement {
    /// Sum of voltage samples
    voltage_sum: ElectricPotential,
    /// Sum of current samples
    current_sum: ElectricCurrent,
    /// Number of samples collected
    sample_count: u32,
    /// Minimum samples required for valid measurement
    min_samples: u32,
}

impl ResistanceMeasurement {
    /// Create a new resistance measurement accumulator.
    ///
    /// # Arguments
    /// * `min_samples` - Minimum number of samples required
    #[inline]
    pub fn new(min_samples: u32) -> Self {
        Self {
            voltage_sum: ElectricPotential::ZERO,
            current_sum: ElectricCurrent::ZERO,
            sample_count: 0,
            min_samples,
        }
    }

    /// Record a voltage/current sample.
    ///
    /// # Arguments
    /// * `vd` - d-axis voltage
    /// * `id` - d-axis current
    ///
    /// Note: During resistance measurement with phase override at angle 0,
    /// the d-axis voltage and current are the relevant values.
    #[inline]
    pub fn record(&mut self, vd: ElectricPotential, id: ElectricCurrent) {
        self.voltage_sum += vd;
        self.current_sum += id;
        self.sample_count += 1;
    }

    /// Reset the accumulator for a new measurement.
    #[inline]
    pub fn reset(&mut self) {
        self.voltage_sum = ElectricPotential::ZERO;
        self.current_sum = ElectricCurrent::ZERO;
        self.sample_count = 0;
    }

    /// Get the current sample count.
    #[inline]
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Check if enough samples have been collected.
    #[inline]
    pub fn has_enough_samples(&self) -> bool {
        self.sample_count >= self.min_samples
    }

    /// Compute the resistance from accumulated samples.
    ///
    /// # Returns
    /// * `Ok(resistance)` - Measured resistance
    /// * `Err(DetectionError)` - If measurement failed
    pub fn finish(self) -> Result<ElectricalResistance, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let avg_current = self.current_sum / self.sample_count as f32;
        let avg_voltage = self.voltage_sum / self.sample_count as f32;

        // Check for valid current
        if avg_current.abs() < MIN_VALID_CURRENT {
            return Err(DetectionError::MotorNotResponding);
        }

        // Use absolute values: sign depends on Clarke/Park convention
        // and current sensing polarity, but resistance is always positive.
        let resistance = avg_voltage.abs() / avg_current.abs();

        // Validate result
        if resistance < MIN_VALID_RESISTANCE {
            return Err(DetectionError::OutOfRange);
        }
        if resistance > MAX_VALID_RESISTANCE {
            return Err(DetectionError::MotorNotResponding);
        }

        Ok(resistance)
    }

    /// Get intermediate result without consuming the accumulator.
    ///
    /// Useful for monitoring during iterative current finding.
    pub fn current_estimate(&self) -> Option<ElectricalResistance> {
        if self.sample_count < 5 {
            return None;
        }

        let avg_current = self.current_sum / self.sample_count as f32;
        if avg_current.abs() < MIN_VALID_CURRENT {
            return None;
        }

        let avg_voltage = self.voltage_sum / self.sample_count as f32;
        Some(avg_voltage.abs() / avg_current.abs())
    }
}

/// Calculate maximum safe continuous current from measured resistance.
///
/// Formula: I_max = sqrt(max_power_loss / 1.5·R), the same copper-loss
/// bound as [`heat_current_limit`].
///
/// # Arguments
/// * `resistance` - Measured resistance
/// * `motor_size` - Motor size for power limit
///
/// # Returns
/// Maximum safe continuous current
#[inline]
pub fn calculate_max_current(
    resistance: ElectricalResistance,
    motor_size: MotorSize,
) -> ElectricCurrent {
    heat_current_limit(motor_size.max_power_loss(), resistance)
}

/// Validate measured resistance is physically reasonable.
///
/// # Arguments
/// * `resistance` - Measured resistance in Ohms
/// * `motor_size` - Expected motor size for sanity checking
///
/// # Returns
/// * `Ok(())` - Resistance is valid
/// * `Err(DetectionError)` - Resistance is out of expected range
pub fn validate_resistance(resistance: f32, motor_size: MotorSize) -> Result<(), DetectionError> {
    if ohms(resistance) < MIN_VALID_RESISTANCE {
        return Err(DetectionError::OutOfRange);
    }

    if ohms(resistance) > MAX_VALID_RESISTANCE {
        return Err(DetectionError::MotorNotResponding);
    }

    // Sanity check against the expected range for the declared motor size.
    // The value may still be physically real (mislabeled size), so this is
    // LowConfidence — distinguishable from the hard open/short gates above.
    let expected_range = match motor_size {
        MotorSize::Mini => ohms(0.01)..=ohms(10.0),
        MotorSize::Small => ohms(0.01)..=ohms(5.0),
        MotorSize::Medium => ohms(0.005)..=ohms(2.0),
        MotorSize::Large => ohms(0.001)..=ohms(0.5),
        MotorSize::Custom(_) => MIN_VALID_RESISTANCE..=MAX_VALID_RESISTANCE,
    };

    if !expected_range.contains(&ohms(resistance)) {
        return Err(DetectionError::LowConfidence);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::volts;

    // Plain-SI-float view for the numeric checks below.
    struct ResistanceMeasurement(super::ResistanceMeasurement);
    impl ResistanceMeasurement {
        fn new(min_samples: u32) -> Self {
            Self(super::ResistanceMeasurement::new(min_samples))
        }
        fn record(&mut self, vd: f32, id: f32) {
            self.0.record(volts(vd), amps(id));
        }
        fn finish(self) -> Result<f32, DetectionError> {
            self.0.finish().map(|r| r.value)
        }
        fn current_estimate(&self) -> Option<f32> {
            self.0.current_estimate().map(|r| r.value)
        }
    }

    #[test]
    fn test_resistance_measurement_basic() {
        let mut measurement = ResistanceMeasurement::new(10);

        // Simulate steady-state samples: V=1V, I=10A -> R=0.1Ω
        for _ in 0..10 {
            measurement.record(1.0, 10.0);
        }

        let r = measurement.finish().unwrap();
        assert!((r - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_resistance_measurement_averaging() {
        let mut measurement = ResistanceMeasurement::new(4);

        // Varying samples that should average out
        measurement.record(0.9, 10.0);
        measurement.record(1.1, 10.0);
        measurement.record(1.0, 9.5);
        measurement.record(1.0, 10.5);

        let r = measurement.finish().unwrap();
        // Average V = 1.0, Average I = 10.0, R = 0.1
        assert!((r - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_resistance_measurement_insufficient_samples() {
        let mut measurement = ResistanceMeasurement::new(10);

        for _ in 0..5 {
            measurement.record(1.0, 10.0);
        }

        assert_eq!(
            measurement.finish(),
            Err(DetectionError::InsufficientSamples)
        );
    }

    #[test]
    fn test_resistance_measurement_low_current() {
        let mut measurement = ResistanceMeasurement::new(10);

        // Very low current - motor not responding
        for _ in 0..10 {
            measurement.record(0.001, 0.01);
        }

        assert_eq!(
            measurement.finish(),
            Err(DetectionError::MotorNotResponding)
        );
    }

    #[test]
    fn test_calculate_max_current() {
        // R = 0.1Ω, Medium motor (120W)
        // I_max = sqrt(120 / 0.1 / 1.5) = sqrt(800) = 28.3A
        let i_max = calculate_max_current(ohms(0.1), MotorSize::Medium);
        assert!((i_max.value - 28.3).abs() < 0.5);
    }

    #[test]
    fn test_validate_resistance() {
        // Valid resistances
        assert!(validate_resistance(0.1, MotorSize::Medium).is_ok());
        assert!(validate_resistance(0.05, MotorSize::Large).is_ok());

        // Invalid: too low (short circuit)
        assert!(validate_resistance(0.0001, MotorSize::Medium).is_err());

        // Invalid: too high (open circuit)
        assert!(validate_resistance(200.0, MotorSize::Medium).is_err());

        // Physically real but implausible for the declared size:
        // soft LowConfidence, not the hard open/short errors.
        assert_eq!(
            validate_resistance(8.0, MotorSize::Medium),
            Err(DetectionError::LowConfidence)
        );
        // The same 8 Ω is normal for a Mini gimbal motor.
        assert!(validate_resistance(8.0, MotorSize::Mini).is_ok());
    }

    #[test]
    fn test_current_estimate() {
        let mut measurement = ResistanceMeasurement::new(100);

        // Not enough samples yet
        measurement.record(1.0, 10.0);
        assert!(measurement.current_estimate().is_none());

        // After 5 samples, should have estimate
        for _ in 0..5 {
            measurement.record(1.0, 10.0);
        }
        let estimate = measurement.current_estimate().unwrap();
        assert!((estimate - 0.1).abs() < 0.01);
    }
}
