//! Motor inductance measurement using High-Frequency Injection (HFI).
//!
//! Measures d-axis and q-axis inductance (Ld, Lq) by injecting a rotating
//! high-frequency voltage vector and analyzing the current response using FFT.
//!
//! # Algorithm (VESC-equivalent)
//!
//! 1. Lock rotor at a fixed electrical position with holding current
//! 2. Inject rotating HFI voltage vector in alpha-beta (stator) frame
//! 3. Measure differential current response (di = i_now - i_prev)
//! 4. Store inverse inductance: 1/L = (ω × di) / V_inject
//! 5. Apply FFT to extract frequency components:
//!    - Bin 0 (DC): Average inverse inductance = (1/Ld + 1/Lq) / 2
//!    - Bin 2 (2nd harmonic): Saliency = (1/Ld - 1/Lq) / 2
//! 6. Calculate: Ld = 1/(offset + amplitude), Lq = 1/(offset - amplitude)
//!
//! # Theory
//!
//! For a rotating injection vector V = V_amp × e^(j×θ_inj) where θ_inj advances
//! through 360° over FFT_SIZE samples, the current response depends on the
//! inductance at each angle.
//!
//! For an IPM motor with saliency:
//! - L(θ) = L_avg + L_diff × cos(2×θ)  (inductance varies with angle)
//! - 1/L(θ) = 1/L_avg × (1 - (L_diff/L_avg) × cos(2×θ))  (approximately)
//!
//! The FFT of 1/L samples gives:
//! - Bin 0: DC component = average of 1/L
//! - Bin 2: 2nd harmonic = saliency information
//!
//! This allows measuring both Ld and Lq in a single sweep, and is robust
//! to small rotor position errors.

use core::marker::PhantomData;

use super::types::{DetectionError, InductanceParams};
use crate::foc::trig::SinCos;

#[cfg(feature = "microfft")]
use microfft::real::rfft_32;

/// Number of samples for FFT (must be power of 2)
pub const FFT_SIZE: usize = 32;

/// Number of HFI injection cycles per FFT window.
/// With FFT_SIZE=32 and 1 cycle, the injection angle θ sweeps 0→2π.
/// The inductance saliency L(θ) = L_avg + L_diff×cos(2θ) creates a 2nd harmonic,
/// which appears at FFT bin 2.
const HFI_CYCLES_PER_FFT: usize = 1;

/// Scale factor applied to measured inductance.
///
/// Set to 1.0 (no scaling) — the measurement should report the true value.
/// Downstream code (e.g. the flux observer) can apply its own stability
/// margin if needed.  A previous value of 0.9 (from VESC) was compensating
/// for systematic overestimation that has since been fixed by proper HFI
/// carrier demodulation and resistance compensation.
const INDUCTANCE_SCALE_FACTOR: f32 = 1.0;

/// Minimum valid inductance in Henries
const MIN_VALID_INDUCTANCE: f32 = 1e-7; // 0.1 µH

/// Maximum valid inductance in Henries
const MAX_VALID_INDUCTANCE: f32 = 0.1; // 100 mH

/// Result of inductance measurement.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InductanceResult {
    /// d-axis inductance in Henries
    pub ld: f32,

    /// q-axis inductance in Henries
    pub lq: f32,

    /// Average inductance (Ld + Lq) / 2 in Henries
    pub l_avg: f32,

    /// Inductance difference (Lq - Ld) in Henries (saliency)
    pub l_diff: f32,

    /// Average current during measurement (for validation)
    pub avg_current: f32,
}

/// Rotating HFI voltage injection generator (VESC-style).
///
/// Generates a rotating injection vector in the alpha-beta (stator) frame.
/// The vector completes `HFI_CYCLES_PER_FFT` rotations over `FFT_SIZE` samples.
#[derive(Clone, Debug)]
pub struct HfiInjector<S: SinCos = crate::foc::trig::LibmSinCos> {
    /// Base HFI frequency in rad/s (the carrier frequency)
    omega_hfi: f32,
    /// Voltage amplitude in Volts
    voltage_amplitude: f32,
    /// HFI phase accumulator (carrier phase)
    hfi_phase: f32,
    /// Injection angle (rotates through 360° over FFT_SIZE samples)
    injection_angle: f32,
    /// Angle increment per sample (for rotating injection)
    angle_increment: f32,
    /// Current sample index within FFT window
    sample_index: usize,
    _sincos: PhantomData<S>,
}

impl<S: SinCos> HfiInjector<S> {
    /// Create a new rotating HFI injector.
    ///
    /// # Arguments
    /// * `hfi_frequency_hz` - HFI carrier frequency in Hz (typically 500-2000 Hz)
    /// * `voltage_amplitude` - Peak voltage amplitude in Volts
    /// * `pwm_frequency_hz` - PWM/sampling frequency in Hz
    pub fn new(hfi_frequency_hz: f32, voltage_amplitude: f32, _pwm_frequency_hz: f32) -> Self {
        // The injection angle advances such that we complete HFI_CYCLES_PER_FFT
        // full rotations over FFT_SIZE samples
        let angle_increment =
            (HFI_CYCLES_PER_FFT as f32 * core::f32::consts::TAU) / FFT_SIZE as f32;

        Self {
            omega_hfi: hfi_frequency_hz * core::f32::consts::TAU,
            voltage_amplitude,
            hfi_phase: 0.0,
            injection_angle: 0.0,
            angle_increment,
            sample_index: 0,
            _sincos: PhantomData,
        }
    }

    /// Get the injection voltage for the current sample.
    ///
    /// # Arguments
    /// * `dt` - Time step in seconds (1 / PWM_frequency)
    ///
    /// # Returns
    /// (v_alpha, v_beta) - Voltage to apply in stator frame
    pub fn step(&mut self, dt: f32) -> (f32, f32) {
        // HFI carrier: sin(ω_hfi × t)
        let (hfi_sin, _) = S::sin_cos(self.hfi_phase);
        let hfi_signal = self.voltage_amplitude * hfi_sin;

        // Rotate the injection vector in alpha-beta frame
        // This sweeps through all electrical angles over FFT_SIZE samples
        let (inj_sin, inj_cos) = S::sin_cos(self.injection_angle);
        let v_alpha = hfi_signal * inj_cos;
        let v_beta = hfi_signal * inj_sin;

        // Advance HFI carrier phase
        self.hfi_phase += self.omega_hfi * dt;
        if self.hfi_phase > core::f32::consts::TAU {
            self.hfi_phase -= core::f32::consts::TAU;
        }

        // Advance injection angle (for next sample)
        self.injection_angle += self.angle_increment;
        if self.injection_angle > core::f32::consts::TAU {
            self.injection_angle -= core::f32::consts::TAU;
        }

        self.sample_index += 1;

        (v_alpha, v_beta)
    }

    /// Reset for a new FFT window.
    pub fn reset(&mut self) {
        self.hfi_phase = 0.0;
        self.injection_angle = 0.0;
        self.sample_index = 0;
    }

    /// Get current sample index within FFT window.
    #[inline]
    pub fn sample_index(&self) -> usize {
        self.sample_index
    }

    /// Get current injection angle.
    #[inline]
    pub fn injection_angle(&self) -> f32 {
        self.injection_angle
    }

    /// Get voltage amplitude.
    #[inline]
    pub fn voltage_amplitude(&self) -> f32 {
        self.voltage_amplitude
    }

    /// Get HFI frequency in rad/s.
    #[inline]
    pub fn omega_hfi(&self) -> f32 {
        self.omega_hfi
    }
}

/// VESC-style inductance measurement using rotating HFI and FFT.
///
/// Collects inverse inductance samples during rotating injection,
/// then uses FFT to extract Ld and Lq from the frequency components.
///
/// The `record()` method accepts the injection voltage that caused the
/// measured current change.  Dividing `di` by the actual instantaneous
/// voltage (instead of the peak amplitude) cancels the HFI carrier
/// modulation, producing clean `1/L(θ)` samples for the FFT.  When
/// the phase resistance is known, it is subtracted from the voltage
/// to remove the resistive contamination that would otherwise create
/// false saliency in SPM motors.
#[derive(Clone)]
pub struct InductanceMeasurement<S: SinCos = crate::foc::trig::LibmSinCos> {
    /// FFT input buffer for inverse inductance samples (1/L)
    samples: [f32; FFT_SIZE],
    /// Current sample index within FFT window
    sample_idx: usize,
    /// Previous current sample for differential calculation
    prev_i_alpha: f32,
    prev_i_beta: f32,
    /// Previous injection angle (for aligning di with injection direction)
    prev_injection_angle: f32,
    /// Number of complete FFT cycles collected
    cycles_completed: u32,
    /// Target number of FFT cycles to average
    target_cycles: u32,
    /// Accumulated Ld sum (for averaging across cycles)
    ld_sum: f32,
    /// Accumulated Lq sum
    lq_sum: f32,
    /// Accumulated current sum (for validation)
    current_sum: f32,
    /// Total sample count
    total_samples: u32,
    /// PWM/sampling frequency in Hz (used for 1/L calculation)
    pwm_freq_hz: f32,
    /// Injection voltage amplitude (used only as fallback/threshold)
    voltage_amplitude: f32,
    /// Previously measured phase resistance (Ω) for compensation.
    /// Set to 0 when unknown.
    resistance_ohm: f32,
    /// DC holding current (A) for separating AC from DC in R compensation.
    hold_current: f32,
    /// Is this the first sample? (skip differential on first)
    first_sample: bool,
    _sincos: PhantomData<S>,
}

impl<S: SinCos> InductanceMeasurement<S> {
    /// Create a new inductance measurement.
    ///
    /// # Arguments
    /// * `params` - Measurement parameters
    /// * `pwm_freq_hz` - PWM frequency in Hz
    pub fn new(params: &InductanceParams, pwm_freq_hz: f32) -> Self {
        // Each FFT cycle is FFT_SIZE samples
        // num_cycles from params is how many FFT windows to average
        let target_cycles = params.num_cycles;

        Self {
            samples: [0.0; FFT_SIZE],
            sample_idx: 0,
            prev_i_alpha: 0.0,
            prev_i_beta: 0.0,
            prev_injection_angle: 0.0,
            cycles_completed: 0,
            target_cycles,
            ld_sum: 0.0,
            lq_sum: 0.0,
            current_sum: 0.0,
            total_samples: 0,
            pwm_freq_hz,
            voltage_amplitude: params.hfi_voltage_v,
            resistance_ohm: params.resistance_ohm,
            hold_current: params.hold_current_a,
            first_sample: true,
            _sincos: PhantomData,
        }
    }

    /// Record a current sample together with the injection voltage that
    /// caused it.
    ///
    /// The injection voltage is used to cancel the HFI carrier modulation
    /// from the `1/L` samples, and — when phase resistance is known — to
    /// subtract the resistive voltage drop.
    ///
    /// # Arguments
    /// * `i_alpha`, `i_beta` - Measured α/β current (A)
    /// * `injection_angle` - Injection direction angle from [`HfiInjector`]
    /// * `v_inj_alpha`, `v_inj_beta` - Injection voltage (V) applied at the
    ///   **previous** time-step (the one that produced the current being
    ///   measured now).
    ///
    /// # Returns
    /// `true` when a complete FFT window has been processed.
    #[cfg(feature = "microfft")]
    pub fn record(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        injection_angle: f32,
        v_inj_alpha: f32,
        v_inj_beta: f32,
    ) -> bool {
        let i_magnitude = libm::sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        self.current_sum += i_magnitude;
        self.total_samples += 1;

        if self.first_sample {
            self.prev_i_alpha = i_alpha;
            self.prev_i_beta = i_beta;
            self.prev_injection_angle = injection_angle;
            self.first_sample = false;
            return false;
        }

        // Differential current
        let di_alpha = i_alpha - self.prev_i_alpha;
        let di_beta = i_beta - self.prev_i_beta;

        // Project di and the previous voltage onto the injection direction
        let (sin_angle, cos_angle) = S::sin_cos(self.prev_injection_angle);
        let di_projected = di_alpha * cos_angle + di_beta * sin_angle;
        let v_projected = v_inj_alpha * cos_angle + v_inj_beta * sin_angle;

        // Resistance compensation: subtract R × i_AC from the voltage.
        // The DC holding current does NOT contribute to di so it must be
        // excluded.  At injection angle θ the DC hold projects as
        // i_hold·cos(θ); the remainder is the AC ripple caused by HFI.
        let i_projected = self.prev_i_alpha * cos_angle + self.prev_i_beta * sin_angle;
        let i_hold_proj = self.hold_current * cos_angle;
        let i_ac_proj = i_projected - i_hold_proj;
        let v_inductive = v_projected - self.resistance_ohm * i_ac_proj;

        // Clamp to avoid division by near-zero at carrier zero-crossings.
        let min_v = self.voltage_amplitude * 0.1;
        let inverse_l = if v_inductive.abs() > min_v {
            (self.pwm_freq_hz * di_projected) / v_inductive
        } else {
            // Carrier near zero crossing — carry forward the last valid sample
            if self.sample_idx > 0 {
                self.samples[self.sample_idx - 1]
            } else {
                0.0
            }
        };

        self.samples[self.sample_idx] = inverse_l;
        self.sample_idx += 1;

        self.prev_i_alpha = i_alpha;
        self.prev_i_beta = i_beta;
        self.prev_injection_angle = injection_angle;

        if self.sample_idx >= FFT_SIZE {
            self.process_fft_cycle();
            self.sample_idx = 0;
            true
        } else {
            false
        }
    }

    /// Fallback record method when microfft is not available.
    #[cfg(not(feature = "microfft"))]
    pub fn record(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        injection_angle: f32,
        v_inj_alpha: f32,
        v_inj_beta: f32,
    ) -> bool {
        let i_magnitude = libm::sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        self.current_sum += i_magnitude;
        self.total_samples += 1;

        if self.first_sample {
            self.prev_i_alpha = i_alpha;
            self.prev_i_beta = i_beta;
            self.prev_injection_angle = injection_angle;
            self.first_sample = false;
            return false;
        }

        let di_alpha = i_alpha - self.prev_i_alpha;
        let di_beta = i_beta - self.prev_i_beta;

        let (sin_angle, cos_angle) = S::sin_cos(self.prev_injection_angle);
        let v_projected = v_inj_alpha * cos_angle + v_inj_beta * sin_angle;
        let i_projected = self.prev_i_alpha * cos_angle + self.prev_i_beta * sin_angle;
        let i_hold_proj = self.hold_current * cos_angle;
        let i_ac_proj = i_projected - i_hold_proj;
        let v_inductive = v_projected - self.resistance_ohm * i_ac_proj;

        let di_magnitude = libm::sqrtf(di_alpha * di_alpha + di_beta * di_beta);
        let min_v = self.voltage_amplitude * 0.1;
        let sample = if v_inductive.abs() > min_v {
            di_magnitude * v_inductive.signum() // preserve sign
        } else if self.sample_idx > 0 {
            self.samples[self.sample_idx - 1]
        } else {
            0.0
        };

        self.samples[self.sample_idx] = sample;
        self.sample_idx += 1;

        self.prev_i_alpha = i_alpha;
        self.prev_i_beta = i_beta;
        self.prev_injection_angle = injection_angle;

        if self.sample_idx >= FFT_SIZE {
            self.process_simple_cycle();
            self.sample_idx = 0;
            true
        } else {
            false
        }
    }

    /// Process a complete FFT cycle and extract inductance.
    #[cfg(feature = "microfft")]
    fn process_fft_cycle(&mut self) {
        // Copy samples for FFT (microfft works in-place)
        let mut fft_input = self.samples;

        // Perform real FFT
        let spectrum = rfft_32(&mut fft_input);

        // Extract frequency bins
        // Bin 0: DC component = average of 1/L samples
        let bin0_real = spectrum[0].re;

        // Bin 2: 2nd harmonic = saliency information
        // (2nd harmonic because inductance varies at 2× electrical frequency)
        let bin2_real = spectrum[2].re;
        let bin2_imag = spectrum[2].im;
        let bin2_magnitude = libm::sqrtf(bin2_real * bin2_real + bin2_imag * bin2_imag);

        // Normalize by FFT size
        // offset = average inverse inductance = (1/Ld + 1/Lq) / 2
        let offset = bin0_real / FFT_SIZE as f32;

        // amplitude = saliency in inverse inductance = (1/Ld - 1/Lq) / 2
        // Factor of 2 for single-sided spectrum
        let amplitude = bin2_magnitude * 2.0 / FFT_SIZE as f32;

        // Prevent division by zero
        let offset = if offset.abs() < 1e-10 {
            1e-10
        } else {
            offset.abs() // 1/L should always be positive
        };

        // Calculate Ld and Lq from offset and amplitude
        // 1/Ld = offset + amplitude
        // 1/Lq = offset - amplitude
        let inv_ld = offset + amplitude;
        let inv_lq = (offset - amplitude).abs().max(1e-10); // Ensure positive

        let ld_est = 1.0 / inv_ld;
        let lq_est = 1.0 / inv_lq;

        // Accumulate for averaging
        if ld_est.is_finite() && lq_est.is_finite() {
            self.ld_sum += ld_est;
            self.lq_sum += lq_est;
            self.cycles_completed += 1;
        }
    }

    /// Simple cycle processing without FFT (fallback).
    #[cfg(not(feature = "microfft"))]
    fn process_simple_cycle(&mut self) {
        // Find average di amplitude
        let mut sum = 0.0;
        for &sample in &self.samples {
            sum += sample;
        }
        let avg_di = sum / FFT_SIZE as f32;

        // Estimate inductance from average: L ≈ V / (f_sample × di)
        // From V = L × di/dt, and di/dt ≈ di × f_sample
        if avg_di > 1e-6 {
            let l_est = self.voltage_amplitude / (self.pwm_freq_hz * avg_di);
            self.ld_sum += l_est;
            self.lq_sum += l_est; // Can't distinguish without FFT
            self.cycles_completed += 1;
        }
    }

    /// Check if measurement is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.cycles_completed >= self.target_cycles
    }

    /// Get progress as percentage.
    #[inline]
    pub fn progress_percent(&self) -> u8 {
        if self.target_cycles == 0 {
            return 100;
        }
        ((self.cycles_completed * 100) / self.target_cycles).min(100) as u8
    }

    /// Get number of completed FFT cycles.
    #[inline]
    pub fn cycles_completed(&self) -> u32 {
        self.cycles_completed
    }

    /// Finish measurement and return results.
    pub fn finish(self) -> Result<InductanceResult, DetectionError> {
        if self.cycles_completed == 0 {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.cycles_completed as f32;

        // Average the accumulated values
        let ld = (self.ld_sum / n) * INDUCTANCE_SCALE_FACTOR;
        let lq = (self.lq_sum / n) * INDUCTANCE_SCALE_FACTOR;
        let l_avg = (ld + lq) / 2.0;
        let l_diff = lq - ld;

        // Validate results
        if !(MIN_VALID_INDUCTANCE..=MAX_VALID_INDUCTANCE).contains(&ld) {
            return Err(DetectionError::OutOfRange);
        }
        if !(MIN_VALID_INDUCTANCE..=MAX_VALID_INDUCTANCE).contains(&lq) {
            return Err(DetectionError::OutOfRange);
        }

        let avg_current = if self.total_samples > 0 {
            self.current_sum / self.total_samples as f32
        } else {
            0.0
        };

        Ok(InductanceResult {
            ld,
            lq,
            l_avg,
            l_diff,
            avg_current,
        })
    }
}

/// Validate measured inductance values.
pub fn validate_inductance(ld: f32, lq: f32) -> Result<(), DetectionError> {
    let valid_range = MIN_VALID_INDUCTANCE..=MAX_VALID_INDUCTANCE;
    if !valid_range.contains(&ld) || !valid_range.contains(&lq) {
        return Err(DetectionError::OutOfRange);
    }

    // Check Ld/Lq ratio is reasonable (0.3 to 3.5).
    // SPM: Ld ≈ Lq (ratio ≈ 1.0)
    // IPM: Ld < Lq (ratio 0.3–0.8 typical)
    // Anything outside this range likely indicates measurement error.
    let ratio = ld / lq;
    if !(0.3..=3.5).contains(&ratio) {
        return Err(DetectionError::LowConfidence);
    }

    Ok(())
}

// ============================================================================
// Axis enum for compatibility with other code
// ============================================================================

/// Axis identifier (for compatibility, not used in VESC-style measurement).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Axis {
    /// d-axis (direct axis, aligned with rotor flux)
    D,
    /// q-axis (quadrature axis, 90° from rotor flux)
    Q,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hfi_injector_rotation() {
        let mut injector = HfiInjector::new(1000.0, 3.0, 20000.0);

        // Collect injection angles over one FFT window
        let dt = 1.0 / 20000.0;
        let mut angles = Vec::new();

        for _ in 0..FFT_SIZE {
            let angle = injector.injection_angle();
            angles.push(angle);
            injector.step(dt);
        }

        // First angle should be ~0
        assert!(angles[0].abs() < 0.1);

        // The angle increment per sample
        let increment = (HFI_CYCLES_PER_FFT as f32 * core::f32::consts::TAU) / FFT_SIZE as f32;

        // After 31 steps, total angle = 31 × increment
        // With HFI_CYCLES_PER_FFT=1, this is 31 × (2π/32) = 31π/16 ≈ 6.09 rad
        let total_angle = 31.0 * increment;
        let expected_final = total_angle % core::f32::consts::TAU;
        let final_angle = angles[FFT_SIZE - 1];

        assert!(
            (final_angle - expected_final).abs() < 0.2,
            "Final angle {} vs expected {} (total unwrapped: {})",
            final_angle,
            expected_final,
            total_angle
        );

        // Verify we complete approximately HFI_CYCLES_PER_FFT rotations
        // by checking the total angular travel
        let total_travel = FFT_SIZE as f32 * increment;
        let expected_rotations = HFI_CYCLES_PER_FFT as f32;
        let actual_rotations = total_travel / core::f32::consts::TAU;
        assert!(
            (actual_rotations - expected_rotations).abs() < 0.1,
            "Expected {} rotations, got {}",
            expected_rotations,
            actual_rotations
        );
    }

    #[test]
    fn test_hfi_injector_output() {
        let mut injector = HfiInjector::new(1000.0, 3.0, 20000.0);
        let dt = 1.0 / 20000.0;

        // At angle = 0, v_alpha should be non-zero, v_beta should be ~0
        let (v_alpha, v_beta) = injector.step(dt);
        // First sample: sin(0) = 0 for HFI, so both should be small
        assert!(v_alpha.abs() < 0.5);
        assert!(v_beta.abs() < 0.5);

        // After a few samples, we should see non-zero values
        for _ in 0..4 {
            injector.step(dt);
        }
        let (v_alpha, v_beta) = injector.step(dt);
        let v_mag = libm::sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        assert!(v_mag > 0.5, "Expected non-zero voltage, got {}", v_mag);
    }

    #[cfg(feature = "microfft")]
    #[test]
    fn test_inductance_measurement_spm() {
        // Test with simulated SPM motor (Ld ≈ Lq)
        let pwm_freq_hz = 20000.0;
        let params = InductanceParams {
            hfi_frequency_hz: 1000.0,
            hfi_voltage_v: 3.0,
            num_cycles: 5,
            ..Default::default()
        };

        let mut measurement = InductanceMeasurement::new(&params, pwm_freq_hz);
        let mut injector = HfiInjector::new(1000.0, 3.0, pwm_freq_hz);
        let dt = 1.0 / pwm_freq_hz;

        // Simulate SPM motor: L = 100µH (same for all angles)
        let l_actual = 0.0001; // 100µH

        // Track current for simulation (integrating di over time)
        let mut i_alpha = 0.0f32;
        let mut i_beta = 0.0f32;
        let mut prev_v_alpha = 0.0f32;
        let mut prev_v_beta = 0.0f32;

        while !measurement.is_complete() {
            let injection_angle = injector.injection_angle();
            let (v_alpha, v_beta) = injector.step(dt);

            // Physics: V = L × di/dt, so di = (V/L) × dt
            let di_alpha = v_alpha * dt / l_actual;
            let di_beta = v_beta * dt / l_actual;

            i_alpha += di_alpha;
            i_beta += di_beta;

            measurement.record(i_alpha, i_beta, injection_angle, prev_v_alpha, prev_v_beta);
            prev_v_alpha = v_alpha;
            prev_v_beta = v_beta;
        }

        let result = measurement.finish().unwrap();

        // For SPM, Ld ≈ Lq ≈ L_actual
        // Allow larger tolerance due to simulation simplifications
        let ld_error = (result.ld - l_actual).abs() / l_actual;
        let lq_error = (result.lq - l_actual).abs() / l_actual;

        assert!(
            ld_error < 0.5,
            "Ld error too large: {:.1}% (got {:.2}µH, expected {:.2}µH)",
            ld_error * 100.0,
            result.ld * 1e6,
            l_actual * 1e6
        );
        assert!(
            lq_error < 0.5,
            "Lq error too large: {:.1}% (got {:.2}µH, expected {:.2}µH)",
            lq_error * 100.0,
            result.lq * 1e6,
            l_actual * 1e6
        );

        // Ld and Lq should be similar for SPM
        let ratio = result.ld / result.lq;
        assert!(
            (0.5..=2.0).contains(&ratio),
            "Ld/Lq ratio {} outside expected range for SPM",
            ratio
        );
    }

    #[test]
    fn test_validate_inductance() {
        // Valid values (SPM motor, Ld ≈ Lq)
        assert!(validate_inductance(0.0001, 0.00012).is_ok());

        // Valid values (IPM motor, Ld < Lq)
        assert!(validate_inductance(0.00008, 0.00015).is_ok());

        // Invalid: too low
        assert!(validate_inductance(1e-9, 0.0001).is_err());

        // Invalid: too high
        assert!(validate_inductance(1.0, 0.0001).is_err());

        // Invalid: extreme ratio
        assert!(validate_inductance(0.0001, 0.001).is_err()); // 10:1 ratio
    }
}
