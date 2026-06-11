//! Hall sensor calibration algorithm
//!
//! Provides platform-agnostic Hall sensor calibration following the VESC algorithm:
//! 1. Sweep motor through electrical angles in open-loop mode
//! 2. Record which Hall state is active at each angle
//! 3. Use sin/cos averaging to find center angle for each Hall state
//! 4. Validate that exactly 6 states are observed (2 invalid: 0 and 7)
//!
//! # Usage
//!
//! Platform code implements the async calibration sweep, this module provides
//! the accumulation and computation logic.
//!
//! ```rust,ignore
//! use oxifoc_core::foc::hall_calibration::{HallCalibrator, HallCalibrationParams};
//!
//! let mut calibrator = HallCalibrator::new();
//!
//! // Platform sweeps electrical angle and records samples
//! for angle in (0..360).map(|d| d as f32 * TAU / 360.0) {
//!     let hall_state = read_hall_sensors(); // 0-7
//!     calibrator.record(angle, hall_state);
//! }
//!
//! // Compute result
//! match calibrator.finish() {
//!     Ok(result) => {
//!         hall_sensor.apply_calibration(&result);
//!     }
//!     Err(e) => {
//!         // Handle calibration failure
//!     }
//! }
//! ```

use core::f32::consts::TAU;

use super::hall_sensor::HALL_STATE_TABLE;

/// Minimum samples per Hall state (like VESC's threshold of 30)
pub const DEFAULT_MIN_SAMPLES: u32 = 30;

/// Default calibration parameters
impl Default for HallCalibrationParams {
    fn default() -> Self {
        Self {
            current_amps: 2.0,
            ramp_time_ms: 1000,
            sweep_count: 6,
            step_delay_us: 5000,
        }
    }
}

/// Trait for reading raw Hall sensor state (platform-specific)
///
/// Platforms implement this to provide Hall state reading during calibration.
pub trait HallReader {
    /// Read current Hall state (0-7, 3-bit value: H3<<2 | H2<<1 | H1)
    fn read_hall_state(&self) -> u8;
}

/// Parameters for Hall sensor calibration
#[derive(Clone, Copy, Debug)]
pub struct HallCalibrationParams {
    /// Open-loop current magnitude (Amps)
    pub current_amps: f32,
    /// Time to ramp up current (milliseconds)
    pub ramp_time_ms: u32,
    /// Number of full 360° sweeps (VESC uses 6: 3 forward + 3 reverse)
    pub sweep_count: u8,
    /// Delay between angle steps (microseconds)
    pub step_delay_us: u32,
}

/// Accumulator for Hall sensor calibration samples
///
/// Uses sin/cos averaging to find the center electrical angle for each Hall state.
/// This is more robust than simple averaging because angles wrap around at 2π.
pub struct HallCalibrator {
    /// Sin component accumulator for each raw Hall state (0-7)
    sin_acc: [f32; 8],
    /// Cos component accumulator for each raw Hall state (0-7)
    cos_acc: [f32; 8],
    /// Sample count per raw Hall state
    counts: [u32; 8],
    /// Minimum samples required per state for validity
    min_samples: u32,
}

impl Default for HallCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

impl HallCalibrator {
    /// Create a new Hall calibrator with default minimum sample threshold
    pub fn new() -> Self {
        Self::with_min_samples(DEFAULT_MIN_SAMPLES)
    }

    /// Create a Hall calibrator with custom minimum sample threshold
    pub fn with_min_samples(min_samples: u32) -> Self {
        Self {
            sin_acc: [0.0; 8],
            cos_acc: [0.0; 8],
            counts: [0; 8],
            min_samples,
        }
    }

    /// Record a sample: electrical angle and raw Hall state
    ///
    /// # Arguments
    /// * `angle_rad` - Current electrical angle (radians, 0 to 2π)
    /// * `raw_hall_state` - Raw 3-bit Hall reading (0-7)
    pub fn record(&mut self, angle_rad: f32, raw_hall_state: u8) {
        let idx = (raw_hall_state & 0x07) as usize;
        let (s, c) = {
            use crate::foc::trig::SinCos;
            crate::foc::trig::FastSinCos::sin_cos(angle_rad)
        };
        self.sin_acc[idx] += s;
        self.cos_acc[idx] += c;
        self.counts[idx] += 1;
    }

    /// Reset the calibrator for a new calibration run
    pub fn reset(&mut self) {
        self.sin_acc = [0.0; 8];
        self.cos_acc = [0.0; 8];
        self.counts = [0; 8];
    }

    /// Get sample count for a specific Hall state
    pub fn sample_count(&self, raw_hall_state: u8) -> u32 {
        self.counts[(raw_hall_state & 0x07) as usize]
    }

    /// Compute calibration result from accumulated samples
    ///
    /// # Returns
    /// * `Ok(HallCalibrationResult)` - Valid calibration with angles for each state
    /// * `Err(CalibrationError)` - Calibration failed (insufficient samples or wrong state count)
    pub fn finish(self) -> Result<HallCalibrationResult, CalibrationError> {
        let mut angles = [0.0_f32; 8];
        let mut valid = [false; 8];
        let mut valid_count = 0u8;

        for i in 0..8 {
            let count = self.counts[i];

            if count >= self.min_samples {
                // Compute average angle using atan2 of accumulated sin/cos
                // This correctly handles wraparound at 2π
                let avg_sin = self.sin_acc[i] / count as f32;
                let avg_cos = self.cos_acc[i] / count as f32;
                let mut angle = libm::atan2f(avg_sin, avg_cos);

                // Normalize to [0, 2π)
                if angle < 0.0 {
                    angle += TAU;
                }

                angles[i] = angle;
                valid[i] = true;
                valid_count += 1;
            } else if count > 0 {
                // Some samples but not enough - this is an error for states 1-6
                // States 0 and 7 are expected to have 0 samples
                if i != 0 && i != 7 {
                    return Err(CalibrationError::InsufficientSamples {
                        state: i as u8,
                        count,
                        required: self.min_samples,
                    });
                }
            }
        }

        // Validate: expect exactly 6 valid states (1-6), states 0 and 7 are invalid
        if valid_count != 6 {
            return Err(CalibrationError::InvalidStateCount { found: valid_count });
        }

        // Additional check: states 0 and 7 should NOT be valid
        if valid[0] || valid[7] {
            return Err(CalibrationError::InvalidStateCount { found: valid_count });
        }

        Ok(HallCalibrationResult {
            angles,
            valid,
            valid_count,
        })
    }
}

/// Result of Hall sensor calibration
#[derive(Clone, Copy, Debug)]
pub struct HallCalibrationResult {
    /// Electrical angle (radians, 0 to 2π) for each raw Hall state (0-7)
    ///
    /// Invalid states (0, 7) will have angle 0.0.
    pub angles: [f32; 8],
    /// Validity flags for each raw state
    pub valid: [bool; 8],
    /// Number of valid states detected (should be 6)
    pub valid_count: u8,
}

impl HallCalibrationResult {
    /// Check if calibration is valid (exactly 6 valid states)
    pub fn is_valid(&self) -> bool {
        self.valid_count == 6 && !self.valid[0] && !self.valid[7]
    }

    /// Convert to 6-element table for logical Hall states (0-5)
    ///
    /// Maps raw Hall states to logical states via `HALL_STATE_TABLE`,
    /// returning angles in the order expected by `HallSensor::set_calibration()`.
    pub fn to_logical_table(&self) -> [f32; 6] {
        let mut table = [0.0_f32; 6];

        // For each valid raw state, place its angle at the corresponding logical index
        for raw in 1..=6u8 {
            if self.valid[raw as usize] {
                let logical = HALL_STATE_TABLE[raw as usize];
                table[logical as usize] = self.angles[raw as usize];
            }
        }

        table
    }

    /// Get angle for a specific raw Hall state
    pub fn angle_for_raw_state(&self, raw_state: u8) -> Option<f32> {
        let idx = (raw_state & 0x07) as usize;
        if self.valid[idx] {
            Some(self.angles[idx])
        } else {
            None
        }
    }
}

/// Error during Hall calibration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalibrationError {
    /// Not enough samples collected for a Hall state
    InsufficientSamples {
        state: u8,
        count: u32,
        required: u32,
    },
    /// Wrong number of valid Hall states detected (expected 6)
    InvalidStateCount { found: u8 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calibrator_basic() {
        let mut cal = HallCalibrator::with_min_samples(2);

        // Simulate ideal Hall sensor: each state spans 60° (TAU/6)
        // Raw state 1 -> logical 0 -> centered at 30° = TAU/12
        // Raw state 3 -> logical 1 -> centered at 90° = TAU/4
        // Raw state 2 -> logical 2 -> centered at 150° = 5*TAU/12
        // Raw state 6 -> logical 3 -> centered at 210° = 7*TAU/12
        // Raw state 4 -> logical 4 -> centered at 270° = 3*TAU/4
        // Raw state 5 -> logical 5 -> centered at 330° = 11*TAU/12

        let state_centers = [
            (1, TAU / 12.0),        // State 1 at 30°
            (3, TAU / 4.0),         // State 3 at 90°
            (2, 5.0 * TAU / 12.0),  // State 2 at 150°
            (6, 7.0 * TAU / 12.0),  // State 6 at 210°
            (4, 3.0 * TAU / 4.0),   // State 4 at 270°
            (5, 11.0 * TAU / 12.0), // State 5 at 330°
        ];

        // Record samples around each center
        for (state, center) in state_centers {
            for offset in [-0.05, 0.0, 0.05] {
                cal.record(center + offset, state);
            }
        }

        let result = cal.finish().expect("Calibration should succeed");
        assert!(result.is_valid());
        assert_eq!(result.valid_count, 6);

        // Check angles are close to expected centers
        for (state, center) in state_centers {
            let angle = result.angle_for_raw_state(state).unwrap();
            let diff = (angle - center).abs();
            assert!(
                diff < 0.1,
                "State {} angle {} not close to {}",
                state,
                angle,
                center
            );
        }
    }

    #[test]
    fn test_calibrator_insufficient_samples() {
        let mut cal = HallCalibrator::with_min_samples(10);

        // Only record a few samples for state 1
        for _ in 0..5 {
            cal.record(0.5, 1);
        }

        // Record enough for other states
        for state in [2, 3, 4, 5, 6] {
            for i in 0..15 {
                cal.record(i as f32 * 0.1, state);
            }
        }

        let result = cal.finish();
        assert!(matches!(
            result,
            Err(CalibrationError::InsufficientSamples { state: 1, .. })
        ));
    }

    #[test]
    fn test_calibrator_invalid_state_count() {
        let mut cal = HallCalibrator::with_min_samples(2);

        // Only record for 4 states
        for state in [1, 2, 3, 4] {
            for _ in 0..5 {
                cal.record(state as f32 * 0.5, state);
            }
        }

        let result = cal.finish();
        assert!(matches!(
            result,
            Err(CalibrationError::InvalidStateCount { found: 4 })
        ));
    }

    #[test]
    fn test_to_logical_table() {
        let mut cal = HallCalibrator::with_min_samples(1);

        // Record specific angles for each raw state
        let test_angles = [
            (1, 0.0), // raw 1 -> logical 0
            (3, 1.0), // raw 3 -> logical 1
            (2, 2.0), // raw 2 -> logical 2
            (6, 3.0), // raw 6 -> logical 3
            (4, 4.0), // raw 4 -> logical 4
            (5, 5.0), // raw 5 -> logical 5
        ];

        for (state, angle) in test_angles {
            cal.record(angle, state);
        }

        let result = cal.finish().unwrap();
        let table = result.to_logical_table();

        // Verify logical table order
        // HALL_STATE_TABLE[1]=0, [3]=1, [2]=2, [6]=3, [4]=4, [5]=5
        assert!((table[0] - 0.0).abs() < 0.01); // logical 0 from raw 1
        assert!((table[1] - 1.0).abs() < 0.01); // logical 1 from raw 3
        assert!((table[2] - 2.0).abs() < 0.01); // logical 2 from raw 2
        assert!((table[3] - 3.0).abs() < 0.01); // logical 3 from raw 6
        assert!((table[4] - 4.0).abs() < 0.01); // logical 4 from raw 4
        assert!((table[5] - 5.0).abs() < 0.01); // logical 5 from raw 5
    }

    #[test]
    fn test_angle_wraparound() {
        let mut cal = HallCalibrator::with_min_samples(2);

        // Record angles near 0/2π boundary for state 1
        cal.record(TAU - 0.1, 1);
        cal.record(0.0, 1);
        cal.record(0.1, 1);

        // Record other states
        for state in [2, 3, 4, 5, 6] {
            for _ in 0..3 {
                cal.record(state as f32 * 0.5, state);
            }
        }

        let result = cal.finish().unwrap();
        let angle = result.angle_for_raw_state(1).unwrap();

        // Should be close to 0 or TAU (they're equivalent)
        let near_zero = !(0.2..=TAU - 0.2).contains(&angle);
        assert!(near_zero, "Angle {} should be near 0 or 2π", angle);
    }
}
