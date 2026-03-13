//! Shunt resistor current sensing for FOC
//!
//! Converts raw ADC readings from shunt resistor + OPAMP amplifiers
//! into calibrated current measurements in Amperes.
//!
//! # Architecture
//!
//! - `ShuntCurrentSense`: Core converter (ADC counts → Amps) with calibration
//! - Platform code: Provides raw ADC values via atomics, implements `CurrentSensor` trait
//! - Calibration: Async function in platform code, uses `calibrate_offsets()` for math
//!
//! # Usage Pattern (for platform crates)
//!
//! ```ignore
//! // In platform crate (e.g., oxifoc-f405/src/sensors/current.rs):
//! use oxifoc_core::foc::current_sense::ShuntCurrentSense;
//! use oxifoc_core::foc::sensors::CurrentSensor;
//!
//! static RAW_A: AtomicU16 = AtomicU16::new(2048);
//! static RAW_B: AtomicU16 = AtomicU16::new(2048);
//! static RAW_C: AtomicU16 = AtomicU16::new(2048);
//! static CONVERTER: Mutex<RefCell<Option<ShuntCurrentSense>>> = ...;
//!
//! pub struct PlatformCurrentSensor { /* config */ }
//!
//! impl CurrentSensor for PlatformCurrentSensor {
//!     fn read_currents(&self) -> (f32, f32, f32) { ... }
//!     fn read_raw(&self) -> (u16, u16, u16) { ... }
//!     fn is_calibrated(&self) -> bool { ... }
//!     fn get_offsets(&self) -> (f32, f32, f32) { ... }
//! }
//!
//! pub async fn calibrate_offsets(num_samples: usize, delay_us: u64) {
//!     // Collect samples with Timer::after() delays
//!     // Call CONVERTER.calibrate_offsets(&samples)
//! }
//! ```

/// Shunt-based current sense converter
///
/// Converts raw ADC counts to calibrated current measurements.
/// Platform-agnostic - works with any shunt + OPAMP + ADC configuration.
pub struct ShuntCurrentSense {
    /// Shunt resistance in Ohms (e.g., 0.003 for 3mΩ)
    shunt_ohms: f32,
    /// OPAMP gain (e.g., 16.0 for 16x)
    opamp_gain: f32,
    /// ADC reference voltage in millivolts (e.g., 3300 for 3.3V)
    adc_vref_mv: u32,
    /// Maximum ADC count (e.g., 4095 for 12-bit)
    adc_max_counts: u16,
    /// Zero-current ADC offset for phase A (in ADC counts)
    offset_a: f32,
    /// Zero-current ADC offset for phase B (in ADC counts)
    offset_b: f32,
    /// Zero-current ADC offset for phase C (in ADC counts)
    offset_c: f32,
    /// Per-phase calibration bitmask (bit 0=A, 1=B, 2=C)
    calibrated_phases: u8,
}

impl ShuntCurrentSense {
    /// Create a new shunt current sense converter
    pub fn new(shunt_ohms: f32, opamp_gain: f32, adc_vref_mv: u32, adc_max_counts: u16) -> Self {
        Self {
            shunt_ohms,
            opamp_gain,
            adc_vref_mv,
            adc_max_counts,
            offset_a: adc_max_counts as f32 / 2.0, // Default to mid-scale
            offset_b: adc_max_counts as f32 / 2.0,
            offset_c: adc_max_counts as f32 / 2.0,
            calibrated_phases: 0,
        }
    }

    /// Convert raw ADC counts to current in Amperes
    pub fn convert_raw(&self, adc_a: u16, adc_b: u16, adc_c: u16) -> (f32, f32, f32) {
        let ia = self.adc_to_current(adc_a, self.offset_a);
        let ib = self.adc_to_current(adc_b, self.offset_b);
        let ic = self.adc_to_current(adc_c, self.offset_c);
        (ia, ib, ic)
    }

    /// Calibrate zero-current offsets from sample data (all phases simultaneously)
    ///
    /// This is the simple calibration method - samples all phases at once.
    /// For VESC-style per-phase calibration, use `calibrate_phase_offset`.
    pub fn calibrate_offsets(&mut self, samples: &[(u16, u16, u16)]) {
        if samples.is_empty() {
            return;
        }

        let mut sum_a = 0u32;
        let mut sum_b = 0u32;
        let mut sum_c = 0u32;

        for &(a, b, c) in samples {
            sum_a += a as u32;
            sum_b += b as u32;
            sum_c += c as u32;
        }

        let count = samples.len() as f32;
        self.offset_a = sum_a as f32 / count;
        self.offset_b = sum_b as f32 / count;
        self.offset_c = sum_c as f32 / count;
        self.calibrated_phases = 0b111; // All three phases calibrated
    }

    /// Calibrate a single phase offset from samples (VESC-style per-phase calibration)
    ///
    /// This allows calibrating each phase individually while PWM is active on that phase.
    /// VESC uses this approach: enable PWM on one phase at a time, sample that phase's ADC,
    /// then move to the next phase. This provides more accurate offsets under load conditions.
    ///
    /// # Arguments
    /// * `phase` - Phase index (0=A, 1=B, 2=C)
    /// * `samples` - Raw ADC samples for this phase
    ///
    /// # Note
    /// `is_calibrated()` returns true only after all three phases have been calibrated.
    pub fn calibrate_phase_offset(&mut self, phase: usize, samples: &[u16]) {
        if samples.is_empty() {
            return;
        }

        let sum: u32 = samples.iter().map(|&s| s as u32).sum();
        let avg = sum as f32 / samples.len() as f32;

        match phase {
            0 => self.offset_a = avg,
            1 => self.offset_b = avg,
            2 => self.offset_c = avg,
            _ => return, // Invalid phase
        }

        // Mark this phase as calibrated; is_calibrated() requires all three
        self.calibrated_phases |= 1 << phase;
    }

    /// Reset calibration state (useful before starting a new per-phase calibration)
    pub fn reset_calibration(&mut self) {
        self.offset_a = self.adc_max_counts as f32 / 2.0;
        self.offset_b = self.adc_max_counts as f32 / 2.0;
        self.offset_c = self.adc_max_counts as f32 / 2.0;
        self.calibrated_phases = 0;
    }

    /// Check if all three phase offsets have been calibrated
    pub fn is_calibrated(&self) -> bool {
        self.calibrated_phases == 0b111
    }

    /// Get current calibration offsets (in ADC counts)
    pub fn get_offsets(&self) -> (f32, f32, f32) {
        (self.offset_a, self.offset_b, self.offset_c)
    }

    /// Manually set calibration offsets
    pub fn set_offsets(&mut self, offset_a: f32, offset_b: f32, offset_c: f32) {
        self.offset_a = offset_a;
        self.offset_b = offset_b;
        self.offset_c = offset_c;
        self.calibrated_phases = 0b111;
    }

    /// Internal helper: convert a single ADC reading to current (A)
    fn adc_to_current(&self, adc_counts: u16, offset: f32) -> f32 {
        // ADC counts relative to zero-current offset
        let delta_counts = adc_counts as f32 - offset;

        // Convert ADC counts to voltage (in millivolts)
        let v_mv = delta_counts * (self.adc_vref_mv as f32) / (self.adc_max_counts as f32);

        // Convert voltage to current (V = I × R × G)
        // I = V / (R_shunt × OPAMP_gain)
        // Note: v_mv is in millivolts, so divide by 1000 to get volts
        (v_mv / 1000.0) / (self.shunt_ohms * self.opamp_gain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // B-G431B-ESC1 hardware specs
    const SHUNT_OHMS: f32 = 0.003;
    const OPAMP_GAIN: f32 = 16.0;
    const ADC_VREF_MV: u32 = 3300;
    const ADC_MAX: u16 = 4095;

    #[test]
    fn test_converter_creation() {
        let converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        assert!(!converter.is_calibrated());

        // Default offsets should be mid-scale
        let (oa, ob, oc) = converter.get_offsets();
        assert!((oa - 2047.5).abs() < 0.1);
        assert!((ob - 2047.5).abs() < 0.1);
        assert!((oc - 2047.5).abs() < 0.1);
    }

    #[test]
    fn test_zero_current() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.set_offsets(2048.0, 2048.0, 2048.0);

        // ADC at offset should read zero current
        let (ia, ib, ic) = converter.convert_raw(2048, 2048, 2048);
        assert!(ia.abs() < 0.001);
        assert!(ib.abs() < 0.001);
        assert!(ic.abs() < 0.001);
    }

    #[test]
    fn test_positive_current() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.set_offsets(2000.0, 2000.0, 2000.0);

        let (ia, ib, ic) = converter.convert_raw(2100, 2050, 2000);
        assert!(ia > 0.0);
        assert!(ib > 0.0);
        assert!(ic.abs() < 1e-6);
    }

    #[test]
    fn test_negative_current() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.set_offsets(2100.0, 2100.0, 2100.0);

        let (ia, ib, ic) = converter.convert_raw(2000, 2050, 2100);
        assert!(ia < 0.0);
        assert!(ib < 0.0);
        assert!(ic.abs() < 1e-6);
    }

    #[test]
    fn test_calibration() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);

        let samples = [
            (2040, 2050, 2060),
            (2042, 2052, 2062),
            (2044, 2054, 2064),
            (2046, 2056, 2066),
        ];

        converter.calibrate_offsets(&samples);
        let (oa, ob, oc) = converter.get_offsets();
        assert!((oa - 2043.0).abs() < 0.1);
        assert!((ob - 2053.0).abs() < 0.1);
        assert!((oc - 2063.0).abs() < 0.1);
        assert!(converter.is_calibrated());
    }

    #[test]
    fn test_realistic_motor_current() {
        // Using B-G431B-ESC1 parameters: 3mΩ, 16x, 3.3V, 12-bit
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.set_offsets(2048.0, 2048.0, 2048.0);

        // Simulate +10A on phase A, -5A on phase B, 0A on phase C
        // ADC delta = I * R * G * ADC_MAX / Vref
        let counts_per_volt = ADC_MAX as f32 / ADC_VREF_MV as f32; // counts per mV
        let delta_a = (10.0 * SHUNT_OHMS * OPAMP_GAIN * 1000.0 * counts_per_volt) as i32;
        let delta_b = (-5.0 * SHUNT_OHMS * OPAMP_GAIN * 1000.0 * counts_per_volt) as i32;

        let adc_a = (2048 + delta_a) as u16;
        let adc_b = (2048 + delta_b) as u16;
        let adc_c = 2048;

        let (ia, ib, ic) = converter.convert_raw(adc_a, adc_b, adc_c);
        assert!((ia - 10.0).abs() < 0.5);
        assert!((ib + 5.0).abs() < 0.3);
        assert!(ic.abs() < 0.1);
    }

    #[test]
    fn test_empty_calibration() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.calibrate_offsets(&[]);
        assert!(!converter.is_calibrated());
    }

    #[test]
    fn test_per_phase_calibration() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);

        // Calibrate phase A
        let samples_a: Vec<u16> = (0..100).map(|_| 2040).collect();
        converter.calibrate_phase_offset(0, &samples_a);

        // Calibrate phase B
        let samples_b: Vec<u16> = (0..100).map(|_| 2050).collect();
        converter.calibrate_phase_offset(1, &samples_b);

        // Calibrate phase C
        let samples_c: Vec<u16> = (0..100).map(|_| 2060).collect();
        converter.calibrate_phase_offset(2, &samples_c);

        let (oa, ob, oc) = converter.get_offsets();
        assert!((oa - 2040.0).abs() < 0.1);
        assert!((ob - 2050.0).abs() < 0.1);
        assert!((oc - 2060.0).abs() < 0.1);
        assert!(converter.is_calibrated());
    }

    #[test]
    fn test_reset_calibration() {
        let mut converter = ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX);
        converter.set_offsets(2000.0, 2100.0, 2200.0);
        assert!(converter.is_calibrated());

        converter.reset_calibration();
        assert!(!converter.is_calibrated());

        let (oa, ob, oc) = converter.get_offsets();
        assert!((oa - 2047.5).abs() < 0.1);
        assert!((ob - 2047.5).abs() < 0.1);
        assert!((oc - 2047.5).abs() < 0.1);
    }
}
