//! Stationary motor inductance measurement using rotating HFI.
//!
//! A 32-sample window identifies the full two-axis discrete RL response from
//! paired voltage increments and current changes. Constant command-voltage
//! bias is fitted as an intercept. The zero-order-hold response is converted
//! back to inductance, including finite R*dt/L rather than an Euler scale.
//!
//! No sample is divided by a small carrier voltage or replaced by its neighbor.
//! Principal inductances retain signed d/q labeling relative to the assumed
//! rotor lock; off-axis response remains an alignment diagnostic. Absolute
//! accuracy still depends on the applied-voltage model and current acquisition.

use core::marker::PhantomData;

use super::types::{DetectionError, InductanceParams};
use crate::foc::trig::SinCos;

use crate::foc::fast_math::sqrtf;
use crate::foc::trig::LibmSinCos;
#[path = "inductance_fit.rs"]
mod fit;
use fit::RlWindow;

/// Identification-window length; historical FFT_SIZE name retained for callers.
pub const FFT_SIZE: usize = 32;

/// Number of HFI injection cycles per identification window.
/// With FFT_SIZE=32 and 1 cycle, the injection angle θ sweeps 0→2π.
/// The inductance saliency L(θ) = L_avg + L_diff×cos(2θ) creates a 2nd harmonic,
/// which appears at FFT bin 2.
const HFI_CYCLES_PER_FFT: usize = 1;

/// No empirical inductance scaling is applied to the identified response.
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
pub struct HfiInjector<S: SinCos = LibmSinCos> {
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
    /// Current sample index within identification window
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

    /// Reset for a new identification window.
    pub fn reset(&mut self) {
        self.hfi_phase = 0.0;
        self.injection_angle = 0.0;
        self.sample_index = 0;
    }

    /// Change the injection amplitude (e.g. mid-measurement adaptation).
    #[inline]
    pub fn set_amplitude(&mut self, voltage_amplitude: f32) {
        self.voltage_amplitude = voltage_amplitude;
    }

    /// Get current sample index within identification window.
    #[inline]
    pub fn sample_index(&self) -> usize {
        self.sample_index
    }

    /// Get current injection angle.
    #[inline]
    pub fn injection_angle(&self) -> f32 {
        self.injection_angle
    }

    /// Get the current carrier phase (rad) — the φ in `A·sin(φ)·dir(θ)`.
    /// Needed by the pipeline-lag probe to build correlation references.
    #[inline]
    pub fn carrier_phase(&self) -> f32 {
        self.hfi_phase
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

/// Streaming two-axis inductance identification under rotating HFI.
///
/// `record` receives the command paired with the measured current interval.
/// Window moments fit `di = B*(u - R*i_previous) + constant`; the symmetric
/// response's eigenvalues recover the principal inductances. `FFT_SIZE` is
/// retained as the historical public name for the 32-sample window length.
#[derive(Clone)]
pub struct InductanceMeasurement<S: SinCos = LibmSinCos> {
    /// Streaming two-axis response fit; no carrier division or filled samples.
    window: RlWindow,
    fit_error: Option<DetectionError>,
    /// Current sample index within identification window
    sample_idx: usize,
    /// Previous current sample for differential calculation
    prev_i_alpha: f32,
    prev_i_beta: f32,
    /// Number of complete identification windows collected
    cycles_completed: u32,
    /// Target number of identification windows to average
    target_cycles: u32,
    /// Accumulated Ld sum (for averaging across cycles)
    ld_sum: f32,
    /// Accumulated signed saliency (1/Ld − 1/Lq)/2 per identification window.
    saliency_sum: f32,
    /// Accumulated quadrature (45°-off) saliency — alignment diagnostic.
    quadrature_sum: f32,
    /// Accumulated Lq sum
    lq_sum: f32,
    /// Accumulated current sum (for validation)
    current_sum: f32,
    /// Total sample count
    total_samples: u32,
    /// PWM/sampling frequency for the zero-order-hold response conversion.
    pwm_freq_hz: f32,
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
        // Each identification window is FFT_SIZE samples
        // num_cycles from params is how many identification windows to average
        let target_cycles = params.num_cycles;

        Self {
            window: RlWindow::default(),
            fit_error: None,
            sample_idx: 0,
            prev_i_alpha: 0.0,
            prev_i_beta: 0.0,
            cycles_completed: 0,
            target_cycles,
            ld_sum: 0.0,
            saliency_sum: 0.0,
            quadrature_sum: 0.0,
            lq_sum: 0.0,
            current_sum: 0.0,
            total_samples: 0,
            pwm_freq_hz,
            resistance_ohm: params.resistance_ohm,
            hold_current: params.hold_current_a,
            first_sample: true,
            _sincos: PhantomData,
        }
    }

    /// Record a current sample together with the injection command that
    /// caused it.
    ///
    /// The supplied voltage vector and previous current enter the discrete
    /// RL response fit. No instantaneous carrier normalization is performed.
    ///
    /// PAIRING CONTRACT: the `di` evaluated by this call spans the interval
    /// between the previous `record` call and this one; the caller must
    /// pass the injection command (angle + voltage) that was ACTUALLY
    /// APPLIED during that interval. With a command→apply pipeline of `D`
    /// cycles that is the command issued `D` iterations before this read —
    /// the caller keeps the history and resolves `D` (see
    /// `probe_hfi_pipeline_lag` in sweep.rs). A one-cycle mispairing
    /// rotates the carrier reference by `ω_c·T_pwm` (90° at 5 kHz/20 kHz)
    /// and corrupts the per-sample `di/v` division catastrophically.
    ///
    /// # Arguments
    /// * `i_alpha`, `i_beta` - Measured α/β current (A)
    /// * `injection_angle` - direction angle of the injection command that
    ///   drove this `di`
    /// * `v_inj_alpha`, `v_inj_beta` - that command's injection voltage (V)
    ///
    /// # Returns
    /// `true` when a complete identification window has been processed.
    pub fn record(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        _injection_angle: f32,
        v_inj_alpha: f32,
        v_inj_beta: f32,
    ) -> bool {
        let i_magnitude = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        self.current_sum += i_magnitude;
        self.total_samples += 1;

        if self.first_sample {
            self.prev_i_alpha = i_alpha;
            self.prev_i_beta = i_beta;
            self.first_sample = false;
            return false;
        }

        let di = [i_alpha - self.prev_i_alpha, i_beta - self.prev_i_beta];
        let x = [
            v_inj_alpha - self.resistance_ohm * (self.prev_i_alpha - self.hold_current),
            v_inj_beta - self.resistance_ohm * self.prev_i_beta,
        ];
        self.window.record(x, di);
        self.sample_idx += 1;
        self.prev_i_alpha = i_alpha;
        self.prev_i_beta = i_beta;
        if self.sample_idx >= FFT_SIZE {
            match self
                .window
                .finish(self.resistance_ohm, 1.0 / self.pwm_freq_hz)
            {
                Ok(result) => {
                    self.ld_sum += result.ld;
                    self.lq_sum += result.lq;
                    self.saliency_sum += result.saliency;
                    self.quadrature_sum += result.quadrature;
                    self.cycles_completed += 1;
                }
                Err(error) => self.fit_error = Some(error),
            }
            self.window = RlWindow::default();
            self.sample_idx = 0;
            true
        } else {
            false
        }
    }

    /// Check if measurement is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.fit_error.is_some() || self.cycles_completed >= self.target_cycles
    }

    /// Get progress as percentage.
    #[inline]
    pub fn progress_percent(&self) -> u8 {
        if self.target_cycles == 0 {
            return 100;
        }
        ((self.cycles_completed * 100) / self.target_cycles).min(100) as u8
    }

    /// Get number of completed identification windows.
    #[inline]
    pub fn cycles_completed(&self) -> u32 {
        self.cycles_completed
    }

    pub fn remaining_cycles(&self) -> u32 {
        self.target_cycles.saturating_sub(self.cycles_completed)
    }

    /// Start another contiguous packet without mixing its DC hold interval
    /// into a differential current sample or discarding completed identification windows.
    pub fn prime_currents(&mut self, alpha: f32, beta: f32) {
        self.prev_i_alpha = alpha;
        self.prev_i_beta = beta;
        self.sample_idx = 0;
        self.window = RlWindow::default();
        self.first_sample = false;
    }

    /// Interim average inductance ((Ld+Lq)/2) over the windows so far.
    ///
    /// Diagnostic only: near the ADC noise floor this estimate can be badly
    /// biased, so it must not size the next excitation's voltage.
    pub fn interim_l_avg(&self) -> Option<f32> {
        if self.cycles_completed == 0 {
            return None;
        }
        let n = self.cycles_completed as f32;
        Some((self.ld_sum / n + self.lq_sum / n) / 2.0)
    }

    /// Restart accumulation with a new injection amplitude, keeping the
    /// configuration (target cycles, R compensation, hold current).
    ///
    /// The amplitude argument remains for compatibility; the fit uses the
    /// supplied voltage vector directly, without an amplitude threshold.
    pub fn restart(&mut self, _voltage_amplitude: f32) {
        self.sample_idx = 0;
        self.window = RlWindow::default();
        self.fit_error = None;
        self.cycles_completed = 0;
        self.ld_sum = 0.0;
        self.lq_sum = 0.0;
        self.saliency_sum = 0.0;
        self.quadrature_sum = 0.0;
        self.current_sum = 0.0;
        self.total_samples = 0;
        self.first_sample = true;
    }

    /// Whether the saliency axes look aligned with the rotor lock.
    ///
    /// The fitted off-diagonal response is the quadrature saliency — near
    /// zero for a well-locked rotor. When it dominates the in-phase part,
    /// the lock angle is far off d (≈45°+) and the Ld/Lq split is not
    /// trustworthy. Only meaningful for motors with measurable saliency:
    /// for an SPM both components are noise around zero, so a tiny signed
    /// sum is treated as aligned.
    pub fn axes_aligned(&self) -> bool {
        let offset_scale = if self.cycles_completed > 0 {
            // Typical 1/L magnitude for "is the saliency significant at all"
            (self.ld_sum / self.cycles_completed as f32).max(1e-9)
        } else {
            return true;
        };
        let signal = self.saliency_sum.abs();
        let quad = self.quadrature_sum.abs();
        // Insignificant saliency (≪ 1% of 1/L_avg): nothing to align.
        if (signal + quad) * offset_scale < 0.01 * self.cycles_completed as f32 {
            return true;
        }
        quad <= signal
    }

    /// Finish measurement and return results.
    pub fn finish(self) -> Result<InductanceResult, DetectionError> {
        if let Some(error) = self.fit_error {
            return Err(error);
        }
        if self.cycles_completed == 0 {
            return Err(DetectionError::InsufficientSamples);
        }

        #[cfg(feature = "defmt")]
        if !self.axes_aligned() {
            defmt::warn!(
                "Inductance: quadrature saliency dominates (lock angle far off d) — Ld/Lq split untrustworthy"
            );
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

        // The holding current must actually be flowing: a disconnected
        // phase still yields plausible-looking 1/L samples from noise, and
        // the range gates above can't tell. |i| averages ≥ the DC hold for
        // a connected motor, so well under half of it means open circuit.
        if self.hold_current > 0.05 && avg_current < 0.3 * self.hold_current {
            return Err(DetectionError::MotorNotResponding);
        }

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
    fn exact_packet_plant_recovers_saliency_at_quarter_rate_and_other_carriers() {
        let fs = 20_004.272;
        let dt = 1.0 / fs;
        let r = 0.030444;
        for carrier in [1000.0, 2000.0, 3000.0, 5000.0] {
            for (ld, lq) in [(18e-6, 28e-6), (28e-6, 18e-6), (25e-6, 25e-6)] {
                for angle in [0.0, 0.2617994] {
                    let params = InductanceParams {
                        resistance_ohm: r,
                        hold_current_a: 3.0,
                        num_cycles: 8,
                        hfi_frequency_hz: carrier,
                        ..Default::default()
                    };
                    let mut measurement = InductanceMeasurement::<LibmSinCos>::new(&params, fs);
                    let (sin, cos) = LibmSinCos::sin_cos(angle);
                    let decay = [ld, lq].map(|l| libm::expf(-r * dt / l));
                    let response = decay.map(|a| (1.0 - a) / r);
                    // Two finite packets, with the commanded voltage held for
                    // each complete interval. The state is not the estimator's
                    // Euler update. Constant bias/current mismatch is allowed.
                    for _ in 0..2 {
                        let baseline = [3.07, -0.04];
                        let mut current = baseline;
                        measurement.prime_currents(current[0], current[1]);
                        for k in 0..128 {
                            let theta = k as f32 * core::f32::consts::TAU / 32.0;
                            let phase = k as f32 * core::f32::consts::TAU * carrier / fs;
                            let (s, c) = LibmSinCos::sin_cos(theta);
                            let amplitude = 0.4 * libm::sinf(phase);
                            let u = [amplitude * c, amplitude * s];
                            let previous = [current[0] - baseline[0], current[1] - baseline[1]];
                            let previous_dq = [
                                cos * previous[0] + sin * previous[1],
                                -sin * previous[0] + cos * previous[1],
                            ];
                            let voltage_dq = [cos * u[0] + sin * u[1], -sin * u[0] + cos * u[1]];
                            let next_dq = [
                                decay[0] * previous_dq[0] + response[0] * voltage_dq[0],
                                decay[1] * previous_dq[1] + response[1] * voltage_dq[1],
                            ];
                            current = [
                                baseline[0] + cos * next_dq[0] - sin * next_dq[1],
                                baseline[1] + sin * next_dq[0] + cos * next_dq[1],
                            ];
                            measurement.record(current[0], current[1], theta, u[0], u[1]);
                        }
                    }
                    let result = measurement.finish().unwrap();
                    assert!(
                        (result.ld / ld - 1.0).abs() < 0.001,
                        "carrier={carrier}, ld={}",
                        result.ld
                    );
                    assert!(
                        (result.lq / lq - 1.0).abs() < 0.001,
                        "carrier={carrier}, lq={}",
                        result.lq
                    );
                }
            }
        }
    }

    #[test]
    fn test_hfi_injector_rotation() {
        let mut injector = HfiInjector::<LibmSinCos>::new(1000.0, 3.0, 20000.0);

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
            "Final angle {final_angle} vs expected {expected_final} (total unwrapped: {total_angle})"
        );

        // Verify we complete approximately HFI_CYCLES_PER_FFT rotations
        // by checking the total angular travel
        let total_travel = FFT_SIZE as f32 * increment;
        let expected_rotations = HFI_CYCLES_PER_FFT as f32;
        let actual_rotations = total_travel / core::f32::consts::TAU;
        assert!(
            (actual_rotations - expected_rotations).abs() < 0.1,
            "Expected {expected_rotations} rotations, got {actual_rotations}"
        );
    }

    #[test]
    fn test_hfi_injector_output() {
        let mut injector = HfiInjector::<LibmSinCos>::new(1000.0, 3.0, 20000.0);
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
        let v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        assert!(v_mag > 0.5, "Expected non-zero voltage, got {v_mag}");
    }

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

        let mut measurement = InductanceMeasurement::<LibmSinCos>::new(&params, pwm_freq_hz);
        let mut injector = HfiInjector::<LibmSinCos>::new(1000.0, 3.0, pwm_freq_hz);
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
            "Ld/Lq ratio {ratio} outside expected range for SPM"
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
