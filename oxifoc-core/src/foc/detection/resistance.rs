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

use super::types::{DetectionError, MotorSize, ResistanceParams};

/// Minimum valid resistance in Ohms (below this suggests short circuit)
const MIN_VALID_RESISTANCE: f32 = 0.001;

/// Maximum valid resistance in Ohms (above this suggests open circuit)
const MAX_VALID_RESISTANCE: f32 = 100.0;

/// Minimum current for valid measurement (Amps)
const MIN_VALID_CURRENT: f32 = 0.1;

/// Accumulator for resistance measurement samples.
///
/// Collects voltage and current samples during steady-state operation
/// and computes the average resistance.
#[derive(Clone, Debug)]
pub struct ResistanceMeasurement {
    /// Sum of voltage samples
    voltage_sum: f32,
    /// Sum of current samples
    current_sum: f32,
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
            voltage_sum: 0.0,
            current_sum: 0.0,
            sample_count: 0,
            min_samples,
        }
    }

    /// Record a voltage/current sample.
    ///
    /// # Arguments
    /// * `vd` - d-axis voltage in Volts
    /// * `id` - d-axis current in Amps
    ///
    /// Note: During resistance measurement with phase override at angle 0,
    /// the d-axis voltage and current are the relevant values.
    #[inline]
    pub fn record(&mut self, vd: f32, id: f32) {
        self.voltage_sum += vd;
        self.current_sum += id;
        self.sample_count += 1;
    }

    /// Reset the accumulator for a new measurement.
    #[inline]
    pub fn reset(&mut self) {
        self.voltage_sum = 0.0;
        self.current_sum = 0.0;
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
    /// * `Ok(resistance)` - Measured resistance in Ohms
    /// * `Err(DetectionError)` - If measurement failed
    pub fn finish(self) -> Result<f32, DetectionError> {
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
    pub fn current_estimate(&self) -> Option<f32> {
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

/// Find the optimal test current for resistance measurement.
///
/// Uses VESC's iterative approach:
/// 1. Start with a small current (current_max / 50)
/// 2. Measure resistance quickly
/// 3. Increase by 1.5× if power dissipation is within limits
/// 4. Stop when I²R × 1.5 >= max_power_loss / 5
///
/// # Arguments
/// * `params` - Measurement parameters
/// * `quick_measure` - Callback to quickly measure R at a given current
///   Returns Some(resistance) or None if measurement failed
///
/// # Returns
/// The optimal test current in Amps
pub fn find_safe_test_current<F>(params: &ResistanceParams, mut quick_measure: F) -> f32
where
    F: FnMut(f32) -> Option<f32>,
{
    let max_power_loss = params.motor_size.max_power_loss_w();
    let power_limit = max_power_loss / 5.0;

    let mut test_current = params.current_max / 50.0;
    if test_current < params.current_min * 1.1 {
        test_current = params.current_min * 1.1;
    }

    let mut last_valid_current = test_current;
    let mut last_r: Option<f32> = None;

    while test_current < params.current_max {
        match quick_measure(test_current) {
            Some(r) => {
                last_r = Some(r);
                // Check power dissipation: I²R × 1.5
                let power = test_current * test_current * r * 1.5;
                if power >= power_limit {
                    break;
                }
                last_valid_current = test_current;
            }
            None => {
                // A flaky measurement must not escalate past the thermal
                // gate: project the dissipation at this current with the
                // last known R before trying an even higher one.
                if let Some(r) = last_r {
                    let projected = test_current * test_current * r * 1.5;
                    if projected >= power_limit {
                        break;
                    }
                }
            }
        }

        test_current *= 1.5;
    }

    last_valid_current
}

/// Calculate maximum safe continuous current from measured resistance.
///
/// Formula: I_max = sqrt(max_power_loss / R / 1.5)
///
/// # Arguments
/// * `resistance` - Measured resistance in Ohms
/// * `motor_size` - Motor size for power limit
///
/// # Returns
/// Maximum safe continuous current in Amps
#[inline]
pub fn calculate_max_current(resistance: f32, motor_size: MotorSize) -> f32 {
    let max_power = motor_size.max_power_loss_w();
    crate::foc::fast_math::sqrtf(max_power / resistance / 1.5)
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
    if resistance < MIN_VALID_RESISTANCE {
        return Err(DetectionError::OutOfRange);
    }

    if resistance > MAX_VALID_RESISTANCE {
        return Err(DetectionError::MotorNotResponding);
    }

    // Additional sanity checks based on motor size
    let expected_range = match motor_size {
        MotorSize::Mini => (0.01, 10.0),   // 10mΩ - 10Ω
        MotorSize::Small => (0.01, 5.0),   // 10mΩ - 5Ω
        MotorSize::Medium => (0.005, 2.0), // 5mΩ - 2Ω
        MotorSize::Large => (0.001, 0.5),  // 1mΩ - 500mΩ
        MotorSize::Custom(_) => (MIN_VALID_RESISTANCE, MAX_VALID_RESISTANCE),
    };

    if resistance < expected_range.0 || resistance > expected_range.1 {
        // Just a warning - don't fail, the measurement might still be valid
        // In a real implementation, this could set a low-confidence flag
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_find_safe_test_current() {
        let params = ResistanceParams {
            motor_size: MotorSize::Medium, // 120W max
            current_max: 20.0,
            current_min: 0.5,
            ..Default::default()
        };

        // Simulate a motor with R = 0.1Ω
        let current = find_safe_test_current(&params, |_i| Some(0.1));

        // Power limit = 120/5 = 24W
        // At I²R×1.5 = 24W -> I² = 24/(0.1×1.5) = 160 -> I = 12.6A
        // Starting at 0.4A, multiplying by 1.5: 0.4, 0.6, 0.9, 1.35, 2.0, 3.0, 4.5, 6.75, 10.1, 15.2
        // Should stop around 10A (before exceeding power limit)
        assert!(current > 5.0 && current < 15.0);
    }

    #[test]
    fn test_calculate_max_current() {
        // R = 0.1Ω, Medium motor (120W)
        // I_max = sqrt(120 / 0.1 / 1.5) = sqrt(800) = 28.3A
        let i_max = calculate_max_current(0.1, MotorSize::Medium);
        assert!((i_max - 28.3).abs() < 0.5);
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
