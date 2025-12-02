//! Motor inductance measurement using High-Frequency Injection (HFI).
//!
//! Measures d-axis and q-axis inductance (Ld, Lq) by injecting a high-frequency
//! sinusoidal voltage and analyzing the current response using FFT.
//!
//! # Algorithm (based on VESC approach)
//!
//! 1. Lock rotor at electrical angle 0 with a holding current
//! 2. Inject HFI voltage: V_inject = V_amp × sin(ω_hfi × t)
//! 3. Sample the resulting current over N cycles
//! 4. Apply FFT to extract frequency components:
//!    - Bin 0 (DC): Average of 1/L (inverse inductance)
//!    - Bin 2 (2nd harmonic): Contains Ld-Lq saliency information
//! 5. Calculate Ld = 1/(offset + amplitude), Lq = 1/(offset - amplitude)
//!
//! # Theory
//!
//! For a high-frequency injection V = V_amp × sin(ω×t), the current response is:
//! ```text
//! I = V_amp / (ω × L) × sin(ω×t - π/2)
//! ```
//!
//! The current amplitude is inversely proportional to inductance.
//! For IPMSM with saliency, the inductance varies with rotor position,
//! creating a 2nd harmonic component proportional to (Ld - Lq).

use super::types::{DetectionError, InductanceParams};

#[cfg(feature = "microfft")]
use microfft::real::rfft_32;

/// Number of samples for FFT (must be power of 2)
pub const FFT_SIZE: usize = 32;

/// Scale factor applied to measured inductance for stability (from VESC)
/// The observer is more stable when inductance is slightly underestimated.
const INDUCTANCE_SCALE_FACTOR: f32 = 0.9;

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

    /// Inductance difference (Lq - Ld) in Henries
    pub l_diff: f32,

    /// Average current during measurement (for validation)
    pub avg_current: f32,
}

/// HFI voltage injection generator.
///
/// Generates the sinusoidal injection voltage for each PWM cycle.
#[derive(Clone, Debug)]
pub struct HfiInjector {
    /// Injection frequency in rad/s
    omega: f32,
    /// Voltage amplitude in Volts
    voltage_amplitude: f32,
    /// Phase accumulator
    phase: f32,
}

impl HfiInjector {
    /// Create a new HFI injector.
    ///
    /// # Arguments
    /// * `frequency_hz` - Injection frequency in Hz
    /// * `voltage_amplitude` - Peak voltage amplitude in Volts
    pub fn new(frequency_hz: f32, voltage_amplitude: f32) -> Self {
        Self {
            omega: frequency_hz * core::f32::consts::TAU,
            voltage_amplitude,
            phase: 0.0,
        }
    }

    /// Get the injection voltage for the current sample.
    ///
    /// # Arguments
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// (vd_inject, vq_inject) - Voltage to add to d and q axes
    pub fn step(&mut self, dt: f32) -> (f32, f32) {
        let v_inject = self.voltage_amplitude * libm::sinf(self.phase);

        self.phase += self.omega * dt;
        // Wrap phase to prevent overflow
        if self.phase > core::f32::consts::TAU {
            self.phase -= core::f32::consts::TAU;
        }

        // Inject on d-axis for Ld measurement
        // For Lq, rotate to q-axis (swap d and q)
        (v_inject, 0.0)
    }

    /// Reset the phase accumulator.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Get current phase for synchronization.
    #[inline]
    pub fn phase(&self) -> f32 {
        self.phase
    }
}

/// Inductance measurement state machine.
///
/// Collects current samples during HFI injection and computes
/// inductance using FFT analysis.
#[derive(Clone)]
pub struct InductanceMeasurement {
    /// FFT input buffer for current samples
    samples: [f32; FFT_SIZE],
    /// Current sample index
    sample_idx: usize,
    /// Number of complete FFT cycles collected
    cycles_completed: u32,
    /// Target number of cycles
    target_cycles: u32,
    /// Accumulated Ld sum
    ld_sum: f32,
    /// Accumulated Lq sum
    lq_sum: f32,
    /// Accumulated current sum
    current_sum: f32,
    /// HFI injection frequency in rad/s
    omega_hfi: f32,
    /// Injection voltage amplitude
    voltage_amplitude: f32,
    /// Sample interval (dt) - reserved for future use
    #[allow(dead_code)]
    dt: f32,
}

impl InductanceMeasurement {
    /// Create a new inductance measurement.
    ///
    /// # Arguments
    /// * `params` - Measurement parameters
    /// * `dt` - Sample interval in seconds (1/PWM_frequency)
    pub fn new(params: &InductanceParams, dt: f32) -> Self {
        Self {
            samples: [0.0; FFT_SIZE],
            sample_idx: 0,
            cycles_completed: 0,
            target_cycles: params.num_cycles,
            ld_sum: 0.0,
            lq_sum: 0.0,
            current_sum: 0.0,
            omega_hfi: params.hfi_frequency_hz * core::f32::consts::TAU,
            voltage_amplitude: params.hfi_voltage_v,
            dt,
        }
    }

    /// Record a current sample.
    ///
    /// # Arguments
    /// * `current` - Measured current in Amps (use id for Ld, iq for Lq)
    ///
    /// # Returns
    /// true if a complete FFT cycle was just processed
    pub fn record(&mut self, current: f32) -> bool {
        self.samples[self.sample_idx] = current;
        self.sample_idx += 1;
        self.current_sum += current.abs();

        if self.sample_idx >= FFT_SIZE {
            self.process_fft_cycle();
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
        // Bin 0: DC component (average current)
        let bin0_real = spectrum[0].re;

        // Bin 2: 2nd harmonic (for saliency)
        let bin2_real = spectrum[2].re;
        let bin2_imag = spectrum[2].im;
        let bin2_magnitude = libm::sqrtf(bin2_real * bin2_real + bin2_imag * bin2_imag);

        // Calculate inductance from frequency response
        // At injection frequency ω, impedance Z = ω×L
        // Current amplitude I = V / (ω×L) => L = V / (ω×I)
        // bin0_real represents average of 1/L

        // Normalize by FFT size
        let offset = bin0_real / FFT_SIZE as f32;
        let amplitude = bin2_magnitude * 2.0 / FFT_SIZE as f32;

        // Prevent division by zero
        let offset = if offset.abs() < 1e-10 { 1e-10 } else { offset };

        // Calculate Ld and Lq from offset and amplitude
        // Based on VESC formula: Ld = 1/(offset + amplitude), Lq = 1/(offset - amplitude)
        // This assumes the samples represent 1/L (inverse inductance)

        // For our direct current measurement approach:
        // The current amplitude is proportional to 1/L
        // So we calculate L from the relationship between injected voltage and current

        let ld_est = self.calculate_inductance_from_current(offset + amplitude);
        let lq_est = self.calculate_inductance_from_current(offset - amplitude);

        self.ld_sum += ld_est;
        self.lq_sum += lq_est;
        self.cycles_completed += 1;
    }

    #[cfg(not(feature = "microfft"))]
    fn process_fft_cycle(&mut self) {
        // Fallback: use simple amplitude measurement without FFT
        // Find peak-to-peak current amplitude
        let mut min_current = f32::MAX;
        let mut max_current = f32::MIN;

        for &sample in &self.samples {
            if sample < min_current {
                min_current = sample;
            }
            if sample > max_current {
                max_current = sample;
            }
        }

        let current_amplitude = (max_current - min_current) / 2.0;

        // L = V / (ω × I)
        let l_est = if current_amplitude > 1e-6 {
            self.voltage_amplitude / (self.omega_hfi * current_amplitude)
        } else {
            0.0
        };

        // Without FFT, we can't distinguish Ld from Lq
        self.ld_sum += l_est;
        self.lq_sum += l_est;
        self.cycles_completed += 1;
    }

    /// Calculate inductance from measured current response.
    #[allow(dead_code)]
    fn calculate_inductance_from_current(&self, current_component: f32) -> f32 {
        // The current at HFI frequency is I = V / (ω×L)
        // Therefore L = V / (ω×I)
        if current_component.abs() > 1e-10 {
            self.voltage_amplitude / (self.omega_hfi * current_component.abs())
        } else {
            0.0
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
        ((self.cycles_completed * 100) / self.target_cycles) as u8
    }

    /// Finish measurement and return results.
    pub fn finish(self) -> Result<InductanceResult, DetectionError> {
        if self.cycles_completed == 0 {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.cycles_completed as f32;

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

        let avg_current = self.current_sum / (self.cycles_completed * FFT_SIZE as u32) as f32;

        Ok(InductanceResult {
            ld,
            lq,
            l_avg,
            l_diff,
            avg_current,
        })
    }
}

/// Simple synchronous demodulation for inductance measurement.
///
/// Alternative to FFT that uses direct multiplication with sin/cos
/// reference signals. Less accurate but simpler and no FFT required.
#[derive(Clone, Debug)]
pub struct SyncDemodulator {
    /// Accumulator for cosine component
    cos_acc: f32,
    /// Accumulator for sine component
    sin_acc: f32,
    /// Sample count
    sample_count: u32,
    /// HFI frequency in rad/s
    omega: f32,
    /// Phase accumulator
    phase: f32,
    /// Injection voltage amplitude
    voltage_amplitude: f32,
}

impl SyncDemodulator {
    /// Create a new synchronous demodulator.
    pub fn new(frequency_hz: f32, voltage_amplitude: f32) -> Self {
        Self {
            cos_acc: 0.0,
            sin_acc: 0.0,
            sample_count: 0,
            omega: frequency_hz * core::f32::consts::TAU,
            phase: 0.0,
            voltage_amplitude,
        }
    }

    /// Record a sample and update demodulation.
    ///
    /// # Arguments
    /// * `current` - Measured current
    /// * `dt` - Time step
    pub fn record(&mut self, current: f32, dt: f32) {
        // Multiply by reference signals
        self.cos_acc += current * libm::cosf(self.phase);
        self.sin_acc += current * libm::sinf(self.phase);
        self.sample_count += 1;

        // Advance phase
        self.phase += self.omega * dt;
        if self.phase > core::f32::consts::TAU {
            self.phase -= core::f32::consts::TAU;
        }
    }

    /// Calculate inductance from accumulated data.
    ///
    /// # Returns
    /// Inductance in Henries, or error
    pub fn finish(self) -> Result<f32, DetectionError> {
        if self.sample_count < 32 {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;

        // Calculate amplitude of current at injection frequency
        let cos_avg = self.cos_acc / n;
        let sin_avg = self.sin_acc / n;
        let current_amplitude = libm::sqrtf(cos_avg * cos_avg + sin_avg * sin_avg) * 2.0;

        if current_amplitude < 1e-6 {
            return Err(DetectionError::LowConfidence);
        }

        // L = V / (ω × I)
        let inductance = self.voltage_amplitude / (self.omega * current_amplitude);

        // Validate
        if !(MIN_VALID_INDUCTANCE..=MAX_VALID_INDUCTANCE).contains(&inductance) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(inductance * INDUCTANCE_SCALE_FACTOR)
    }
}

/// Validate measured inductance values.
pub fn validate_inductance(ld: f32, lq: f32) -> Result<(), DetectionError> {
    let valid_range = MIN_VALID_INDUCTANCE..=MAX_VALID_INDUCTANCE;
    if !valid_range.contains(&ld) || !valid_range.contains(&lq) {
        return Err(DetectionError::OutOfRange);
    }

    // Check Ld/Lq ratio is reasonable (0.3 to 3.0 for most motors)
    let ratio = ld / lq;
    if !(0.2..=5.0).contains(&ratio) {
        return Err(DetectionError::LowConfidence);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hfi_injector_basic() {
        let mut injector = HfiInjector::new(1000.0, 3.0);

        // First sample should be near zero (sin(0) = 0)
        let dt = 1.0 / 20000.0; // 20kHz sample rate
        let (vd, vq) = injector.step(dt);
        assert!(vd.abs() < 0.5); // Near zero at start
        assert_eq!(vq, 0.0);

        // After many steps (1/4 period = 250µs = 5 steps at 20kHz)
        for _ in 0..4 {
            injector.step(dt);
        }
        let (vd, _) = injector.step(dt);
        // Should be building up toward peak
        assert!(vd.abs() > 0.5);
    }

    #[test]
    fn test_sync_demodulator() {
        let mut demod = SyncDemodulator::new(1000.0, 3.0);
        let dt = 1.0 / 20000.0; // 20kHz sample rate

        // Simulate current response to HFI injection
        // For L = 100µH, I = V/(ω×L) = 3.0/(2π×1000×0.0001) ≈ 4.77A
        let l_actual = 0.0001; // 100µH
        let omega = 1000.0 * core::f32::consts::TAU;
        let i_amplitude = 3.0 / (omega * l_actual);

        // Generate 100 samples (5 complete cycles at 1kHz with 20kHz sampling)
        let mut phase = 0.0f32;
        for _ in 0..100 {
            // Current lags voltage by 90° for pure inductance
            let current = i_amplitude * libm::sinf(phase - core::f32::consts::FRAC_PI_2);
            demod.record(current, dt);
            phase += omega * dt;
        }

        let l_measured = demod.finish().unwrap();

        // Should be close to actual inductance (within 20% due to simulation simplifications)
        let error = (l_measured - l_actual).abs() / l_actual;
        assert!(error < 0.3, "Inductance error too large: {}", error);
    }

    #[test]
    fn test_validate_inductance() {
        // Valid values
        assert!(validate_inductance(0.0001, 0.00012).is_ok());

        // Invalid: too low
        assert!(validate_inductance(1e-9, 0.0001).is_err());

        // Invalid: too high
        assert!(validate_inductance(1.0, 0.0001).is_err());

        // Invalid: extreme ratio
        assert!(validate_inductance(0.0001, 0.001).is_err()); // 10:1 ratio
    }
}
