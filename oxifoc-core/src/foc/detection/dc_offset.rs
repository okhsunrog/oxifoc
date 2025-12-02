//! Enhanced DC offset calibration for current sensors.
//!
//! Provides VESC-style multi-state calibration that measures offsets
//! at different PWM states for more accurate results.
//!
//! # Simple vs Enhanced Calibration
//!
//! Simple (existing in current_sense.rs):
//! - All PWM off, measure zero-current offsets
//! - Quick and simple, sufficient for most cases
//!
//! Enhanced (this module):
//! - Measures at multiple PWM states (V0, V7, per-phase)
//! - Accounts for noise coupling from switching
//! - More accurate for high-current applications
//!
//! # VESC Approach
//!
//! VESC's `mcpwm_foc_dc_cal` measures current sensor offsets at:
//! 1. All switches off (undriven state)
//! 2. Zero vector (all high or all low)
//! 3. Each phase individually at 50% duty
//!
//! This captures any offset variations due to PWM noise coupling.

use super::types::{DcOffsets, DetectionError};

/// Number of PWM states for enhanced calibration.
const NUM_CAL_STATES: usize = 4;

/// Minimum valid offset (should be near mid-scale ADC).
const MIN_VALID_OFFSET: f32 = 100.0;

/// Maximum valid offset (should be near mid-scale ADC for bipolar sensing).
const MAX_VALID_OFFSET: f32 = 4000.0;

/// Result of enhanced DC offset calibration.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EnhancedDcOffsets {
    /// Phase A offset in ADC counts
    pub phase_a: f32,

    /// Phase B offset in ADC counts
    pub phase_b: f32,

    /// Phase C offset in ADC counts (if measured)
    pub phase_c: f32,

    /// Offset measured during undriven state
    pub undriven_offset: f32,

    /// Offset measured during zero vector (V0)
    pub v0_offset: f32,

    /// Maximum offset variation between states (for diagnostics)
    pub max_variation: f32,
}

/// PWM state for offset calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationState {
    /// All switches off (motor floating)
    Undriven,
    /// Zero vector V0 (all low-side on)
    ZeroVectorLow,
    /// Zero vector V7 (all high-side on)
    ZeroVectorHigh,
    /// Phase A at 50% duty, others at 50%
    MidDuty,
}

impl CalibrationState {
    /// Get duty cycles for each phase during this calibration state.
    ///
    /// Returns (duty_a, duty_b, duty_c) as fractions (0.0 to 1.0).
    pub fn duty_cycles(&self) -> (f32, f32, f32) {
        match self {
            CalibrationState::Undriven => (0.0, 0.0, 0.0),
            CalibrationState::ZeroVectorLow => (0.0, 0.0, 0.0),
            CalibrationState::ZeroVectorHigh => (1.0, 1.0, 1.0),
            CalibrationState::MidDuty => (0.5, 0.5, 0.5),
        }
    }

    /// Check if PWM should be active during this state.
    pub fn pwm_enabled(&self) -> bool {
        !matches!(self, CalibrationState::Undriven)
    }
}

/// Accumulator for DC offset samples.
#[derive(Clone, Debug, Default)]
pub struct DcOffsetAccumulator {
    /// Sum of phase A samples
    sum_a: f32,
    /// Sum of phase B samples
    sum_b: f32,
    /// Sum of phase C samples
    sum_c: f32,
    /// Sample count
    count: u32,
}

impl DcOffsetAccumulator {
    /// Create a new accumulator.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a sample.
    #[inline]
    pub fn record(&mut self, adc_a: u16, adc_b: u16, adc_c: u16) {
        self.sum_a += adc_a as f32;
        self.sum_b += adc_b as f32;
        self.sum_c += adc_c as f32;
        self.count += 1;
    }

    /// Record a sample (f32 version for already-converted values).
    #[inline]
    pub fn record_f32(&mut self, a: f32, b: f32, c: f32) {
        self.sum_a += a;
        self.sum_b += b;
        self.sum_c += c;
        self.count += 1;
    }

    /// Get the sample count.
    #[inline]
    pub fn count(&self) -> u32 {
        self.count
    }

    /// Reset the accumulator.
    #[inline]
    pub fn reset(&mut self) {
        self.sum_a = 0.0;
        self.sum_b = 0.0;
        self.sum_c = 0.0;
        self.count = 0;
    }

    /// Compute the average offsets.
    pub fn finish(self) -> Result<DcOffsets, DetectionError> {
        if self.count == 0 {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.count as f32;
        let offsets = DcOffsets {
            phase_a: self.sum_a / n,
            phase_b: self.sum_b / n,
            phase_c: self.sum_c / n,
        };

        // Validate offsets are in reasonable range
        validate_offsets(&offsets)?;

        Ok(offsets)
    }
}

/// Enhanced calibration state machine.
///
/// Manages multi-state calibration for accurate offset measurement.
#[derive(Clone, Debug)]
pub struct EnhancedCalibration {
    /// Accumulator for each calibration state
    accumulators: [DcOffsetAccumulator; NUM_CAL_STATES],
    /// Current calibration state index
    current_state: usize,
    /// Samples collected for current state
    samples_per_state: u32,
    /// Target samples per state
    target_samples: u32,
}

impl EnhancedCalibration {
    /// Create a new enhanced calibration.
    pub fn new(samples_per_state: u32) -> Self {
        Self {
            accumulators: [
                DcOffsetAccumulator::new(),
                DcOffsetAccumulator::new(),
                DcOffsetAccumulator::new(),
                DcOffsetAccumulator::new(),
            ],
            current_state: 0,
            samples_per_state: 0,
            target_samples: samples_per_state,
        }
    }

    /// Get the current calibration state.
    pub fn current_state(&self) -> CalibrationState {
        match self.current_state {
            0 => CalibrationState::Undriven,
            1 => CalibrationState::ZeroVectorLow,
            2 => CalibrationState::ZeroVectorHigh,
            3 => CalibrationState::MidDuty,
            _ => CalibrationState::Undriven,
        }
    }

    /// Record a sample for the current state.
    ///
    /// Returns true if we should move to the next state.
    pub fn record(&mut self, adc_a: u16, adc_b: u16, adc_c: u16) -> bool {
        if self.current_state < NUM_CAL_STATES {
            self.accumulators[self.current_state].record(adc_a, adc_b, adc_c);
            self.samples_per_state += 1;

            if self.samples_per_state >= self.target_samples {
                self.current_state += 1;
                self.samples_per_state = 0;
                return true; // State change
            }
        }
        false
    }

    /// Check if calibration is complete.
    pub fn is_complete(&self) -> bool {
        self.current_state >= NUM_CAL_STATES
    }

    /// Get overall progress as percentage.
    pub fn progress_percent(&self) -> u8 {
        let total_samples = NUM_CAL_STATES as u32 * self.target_samples;
        let completed = self.current_state as u32 * self.target_samples + self.samples_per_state;
        ((completed * 100) / total_samples) as u8
    }

    /// Finish calibration and compute final offsets.
    ///
    /// Uses weighted average of different states.
    pub fn finish(self) -> Result<EnhancedDcOffsets, DetectionError> {
        if !self.is_complete() {
            return Err(DetectionError::InsufficientSamples);
        }

        // Get offsets from each state
        let undriven = self.accumulators[0].clone().finish()?;
        let v0 = self.accumulators[1].clone().finish()?;
        let v7 = self.accumulators[2].clone().finish()?;
        let mid = self.accumulators[3].clone().finish()?;

        // Calculate variation between states
        let offsets = [&undriven, &v0, &v7, &mid];
        let mut max_var = 0.0f32;

        for i in 0..offsets.len() {
            for j in (i + 1)..offsets.len() {
                let var_a = (offsets[i].phase_a - offsets[j].phase_a).abs();
                let var_b = (offsets[i].phase_b - offsets[j].phase_b).abs();
                let var_c = (offsets[i].phase_c - offsets[j].phase_c).abs();
                max_var = max_var.max(var_a).max(var_b).max(var_c);
            }
        }

        // Use mid-duty offsets as primary (most representative of normal operation)
        // but average with other states for robustness
        let phase_a = (undriven.phase_a + v0.phase_a + v7.phase_a + mid.phase_a * 2.0) / 5.0;
        let phase_b = (undriven.phase_b + v0.phase_b + v7.phase_b + mid.phase_b * 2.0) / 5.0;
        let phase_c = (undriven.phase_c + v0.phase_c + v7.phase_c + mid.phase_c * 2.0) / 5.0;

        Ok(EnhancedDcOffsets {
            phase_a,
            phase_b,
            phase_c,
            undriven_offset: (undriven.phase_a + undriven.phase_b + undriven.phase_c) / 3.0,
            v0_offset: (v0.phase_a + v0.phase_b + v0.phase_c) / 3.0,
            max_variation: max_var,
        })
    }
}

/// Validate that offsets are in a reasonable range.
fn validate_offsets(offsets: &DcOffsets) -> Result<(), DetectionError> {
    let valid_range = MIN_VALID_OFFSET..=MAX_VALID_OFFSET;
    for &offset in [offsets.phase_a, offsets.phase_b, offsets.phase_c].iter() {
        if !valid_range.contains(&offset) {
            return Err(DetectionError::OutOfRange);
        }
    }
    Ok(())
}

/// Simple offset calibration (single-state, for compatibility).
///
/// Measures offsets with motor undriven. Simpler than enhanced
/// calibration but sufficient for most applications.
pub fn calibrate_simple(samples: &[(u16, u16, u16)]) -> Result<DcOffsets, DetectionError> {
    let mut acc = DcOffsetAccumulator::new();

    for &(a, b, c) in samples {
        acc.record(a, b, c);
    }

    acc.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accumulator_basic() {
        let mut acc = DcOffsetAccumulator::new();

        // Mid-scale ADC values (12-bit = 2048)
        for _ in 0..100 {
            acc.record(2048, 2040, 2056);
        }

        let offsets = acc.finish().unwrap();

        assert!((offsets.phase_a - 2048.0).abs() < 1.0);
        assert!((offsets.phase_b - 2040.0).abs() < 1.0);
        assert!((offsets.phase_c - 2056.0).abs() < 1.0);
    }

    #[test]
    fn test_accumulator_empty() {
        let acc = DcOffsetAccumulator::new();
        assert_eq!(acc.finish(), Err(DetectionError::InsufficientSamples));
    }

    #[test]
    fn test_calibration_state_duty() {
        assert_eq!(CalibrationState::Undriven.duty_cycles(), (0.0, 0.0, 0.0));
        assert_eq!(
            CalibrationState::ZeroVectorHigh.duty_cycles(),
            (1.0, 1.0, 1.0)
        );
        assert_eq!(CalibrationState::MidDuty.duty_cycles(), (0.5, 0.5, 0.5));
    }

    #[test]
    fn test_enhanced_calibration_flow() {
        let mut cal = EnhancedCalibration::new(10);

        assert!(!cal.is_complete());
        assert_eq!(cal.current_state(), CalibrationState::Undriven);

        // Fill first state
        for i in 0..10 {
            let state_changed = cal.record(2048, 2048, 2048);
            if i == 9 {
                assert!(state_changed);
            }
        }

        assert_eq!(cal.current_state(), CalibrationState::ZeroVectorLow);

        // Fill remaining states
        for _ in 0..30 {
            cal.record(2048, 2048, 2048);
        }

        assert!(cal.is_complete());
    }

    #[test]
    fn test_enhanced_calibration_result() {
        let mut cal = EnhancedCalibration::new(10);

        // Fill all states with consistent values
        for _ in 0..(4 * 10) {
            cal.record(2048, 2040, 2056);
        }

        let result = cal.finish().unwrap();

        // Should be close to input values
        assert!((result.phase_a - 2048.0).abs() < 1.0);
        assert!((result.phase_b - 2040.0).abs() < 1.0);
        assert!((result.phase_c - 2056.0).abs() < 1.0);

        // Low variation since all states have same values
        assert!(result.max_variation < 1.0);
    }

    #[test]
    fn test_simple_calibration() {
        let samples: Vec<(u16, u16, u16)> = (0..100).map(|_| (2048, 2040, 2056)).collect();

        let offsets = calibrate_simple(&samples).unwrap();

        assert!((offsets.phase_a - 2048.0).abs() < 1.0);
        assert!((offsets.phase_b - 2040.0).abs() < 1.0);
        assert!((offsets.phase_c - 2056.0).abs() < 1.0);
    }

    #[test]
    fn test_validate_offsets_valid() {
        let offsets = DcOffsets {
            phase_a: 2048.0,
            phase_b: 2040.0,
            phase_c: 2056.0,
        };
        assert!(validate_offsets(&offsets).is_ok());
    }

    #[test]
    fn test_validate_offsets_invalid() {
        let offsets = DcOffsets {
            phase_a: 50.0, // Too low
            phase_b: 2040.0,
            phase_c: 2056.0,
        };
        assert!(validate_offsets(&offsets).is_err());
    }
}
