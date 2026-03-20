//! Motor flux linkage (λ) measurement algorithm.
//!
//! Measures the permanent magnet flux linkage by spinning the motor
//! in open-loop mode and analyzing the voltage/current relationship.
//!
//! # Algorithm
//!
//! 1. Apply open-loop current at a controlled angular velocity
//! 2. Ramp up to target eRPM (electrical RPM)
//! 3. Wait for steady-state operation
//! 4. Sample Vq, Iq, and angular velocity
//! 5. Calculate flux linkage: λ = (Vq - R×Iq) / ωe
//!
//! # Theory
//!
//! In a PMSM at steady state (dI/dt = 0), the q-axis voltage equation is:
//! ```text
//! Vq = R × Iq + ωe × λ
//! ```
//!
//! Solving for flux linkage:
//! ```text
//! λ = (Vq - R × Iq) / ωe
//! ```
//!
//! Where:
//! - Vq = q-axis voltage (V)
//! - Iq = q-axis current (A)
//! - R = phase resistance (Ω)
//! - ωe = electrical angular velocity (rad/s)
//! - λ = flux linkage (Wb)

use super::types::{DetectionError, FluxLinkageParams};

/// Minimum valid flux linkage in Weber
const MIN_VALID_FLUX: f32 = 0.0001; // 0.1 mWb

/// Maximum valid flux linkage in Weber
const MAX_VALID_FLUX: f32 = 1.0; // 1 Wb

/// Minimum angular velocity for valid measurement (rad/s)
const MIN_VALID_OMEGA: f32 = 10.0; // ~95 eRPM

/// Accumulator for flux linkage measurement samples.
///
/// Collects steady-state voltage, current, and velocity samples
/// and computes the average flux linkage.
#[derive(Clone, Debug)]
pub struct FluxLinkageMeasurement {
    /// Known phase resistance (from previous measurement)
    resistance_ohm: f32,
    /// Sum of Vq samples
    vq_sum: f32,
    /// Sum of Iq samples
    iq_sum: f32,
    /// Sum of angular velocity samples (rad/s)
    omega_sum: f32,
    /// Number of samples collected
    sample_count: u32,
    /// Minimum samples required
    min_samples: u32,
}

impl FluxLinkageMeasurement {
    /// Create a new flux linkage measurement accumulator.
    ///
    /// # Arguments
    /// * `resistance_ohm` - Previously measured motor resistance
    /// * `min_samples` - Minimum samples required for valid result
    pub fn new(resistance_ohm: f32, min_samples: u32) -> Self {
        Self {
            resistance_ohm,
            vq_sum: 0.0,
            iq_sum: 0.0,
            omega_sum: 0.0,
            sample_count: 0,
            min_samples,
        }
    }

    /// Create from flux linkage parameters.
    pub fn from_params(params: &FluxLinkageParams) -> Result<Self, DetectionError> {
        if params.resistance_ohm <= 0.0 {
            return Err(DetectionError::MissingPrerequisite);
        }

        Ok(Self::new(params.resistance_ohm, params.num_samples))
    }

    /// Record a sample during steady-state spinning.
    ///
    /// # Arguments
    /// * `vq` - q-axis voltage in Volts
    /// * `iq` - q-axis current in Amps
    /// * `omega_e` - electrical angular velocity in rad/s
    ///
    /// Note: Only call this during steady-state operation at target speed.
    #[inline]
    pub fn record(&mut self, vq: f32, iq: f32, omega_e: f32) {
        self.vq_sum += vq;
        self.iq_sum += iq;
        self.omega_sum += omega_e;
        self.sample_count += 1;
    }

    /// Record a sample with mechanical RPM and pole pairs.
    ///
    /// Convenience method that converts mechanical RPM to electrical rad/s.
    ///
    /// # Arguments
    /// * `vq` - q-axis voltage in Volts
    /// * `iq` - q-axis current in Amps
    /// * `rpm` - mechanical RPM
    /// * `pole_pairs` - number of motor pole pairs
    #[inline]
    pub fn record_with_rpm(&mut self, vq: f32, iq: f32, rpm: f32, pole_pairs: u8) {
        let omega_e = rpm_to_omega_e(rpm, pole_pairs);
        self.record(vq, iq, omega_e);
    }

    /// Reset the accumulator for a new measurement.
    #[inline]
    pub fn reset(&mut self) {
        self.vq_sum = 0.0;
        self.iq_sum = 0.0;
        self.omega_sum = 0.0;
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

    /// Get progress as percentage.
    #[inline]
    pub fn progress_percent(&self) -> u8 {
        ((self.sample_count * 100) / self.min_samples) as u8
    }

    /// Compute the flux linkage from accumulated samples.
    ///
    /// # Returns
    /// * `Ok(flux_linkage)` - Flux linkage in Weber
    /// * `Err(DetectionError)` - If measurement failed
    pub fn finish(self) -> Result<f32, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;
        let avg_vq = self.vq_sum / n;
        let avg_iq = self.iq_sum / n;
        let avg_omega = self.omega_sum / n;

        // Check for valid angular velocity
        if avg_omega.abs() < MIN_VALID_OMEGA {
            return Err(DetectionError::LowConfidence);
        }

        // Calculate flux linkage: λ = (Vq - R×Iq) / ωe
        let flux_linkage = (avg_vq - self.resistance_ohm * avg_iq) / avg_omega;

        // Validate result
        if flux_linkage < MIN_VALID_FLUX {
            return Err(DetectionError::OutOfRange);
        }
        if flux_linkage > MAX_VALID_FLUX {
            return Err(DetectionError::OutOfRange);
        }

        Ok(flux_linkage)
    }

    /// Get intermediate flux estimate without consuming the accumulator.
    ///
    /// Useful for monitoring convergence during measurement.
    pub fn current_estimate(&self) -> Option<f32> {
        if self.sample_count < 10 {
            return None;
        }

        let n = self.sample_count as f32;
        let avg_vq = self.vq_sum / n;
        let avg_iq = self.iq_sum / n;
        let avg_omega = self.omega_sum / n;

        if avg_omega.abs() < MIN_VALID_OMEGA {
            return None;
        }

        Some((avg_vq - self.resistance_ohm * avg_iq) / avg_omega)
    }
}

// ============================================================================
// Magnitude-based driven flux linkage measurement (VESC-style)
// ============================================================================

/// Accumulator for magnitude-based driven flux linkage measurement.
///
/// Uses the VESC approach: voltage and current magnitudes instead of
/// q-axis components.  This is **rotation-invariant** — angle tracking
/// errors do not affect the result.
///
///   `λ = (|V| − R·|I|) / ωe − |I|·L`
///
/// The `|I|·L` term compensates for the cross-coupling reactance
/// `ωe·Ld·id` that appears in the q-axis voltage equation of the dq
/// model.  When using vector magnitudes instead of axis-decomposed
/// values, this term would otherwise inflate the apparent back-EMF.
/// It is **not** a transient di/dt term — it arises from the steady-
/// state rotating-frame voltage equations.
///
/// Requires both R and L from previous detection steps.
#[derive(Clone, Debug)]
pub struct MagnitudeFluxMeasurement {
    resistance_ohm: f32,
    /// Average inductance (Henries) for cross-coupling compensation.
    inductance_h: f32,
    v_mag_sum: f32,
    i_mag_sum: f32,
    omega_sum: f32,
    sample_count: u32,
    min_samples: u32,
}

impl MagnitudeFluxMeasurement {
    /// Create a new magnitude-based flux measurement.
    ///
    /// # Arguments
    /// * `resistance_ohm` — previously measured R
    /// * `inductance_h` — previously measured L_avg (Henries)
    /// * `min_samples` — minimum samples for valid result
    pub fn new(resistance_ohm: f32, inductance_h: f32, min_samples: u32) -> Self {
        Self {
            resistance_ohm,
            inductance_h,
            v_mag_sum: 0.0,
            i_mag_sum: 0.0,
            omega_sum: 0.0,
            sample_count: 0,
            min_samples,
        }
    }

    /// Record a sample using all four dq components.
    ///
    /// Computes magnitudes internally: `|V| = √(vd²+vq²)`, `|I| = √(id²+iq²)`.
    #[inline]
    pub fn record(&mut self, vd: f32, vq: f32, id: f32, iq: f32, omega_e: f32) {
        self.v_mag_sum += libm::sqrtf(vd * vd + vq * vq);
        self.i_mag_sum += libm::sqrtf(id * id + iq * iq);
        self.omega_sum += omega_e;
        self.sample_count += 1;
    }

    /// Compute the flux linkage.
    pub fn finish(self) -> Result<f32, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;
        let avg_v = self.v_mag_sum / n;
        let avg_i = self.i_mag_sum / n;
        let avg_omega = self.omega_sum / n;

        if avg_omega.abs() < MIN_VALID_OMEGA {
            return Err(DetectionError::LowConfidence);
        }

        // VESC formula: λ = (|V| - R·|I|) / ω - |I|·L
        let flux = (avg_v - self.resistance_ohm * avg_i) / avg_omega - avg_i * self.inductance_h;

        if !(MIN_VALID_FLUX..=MAX_VALID_FLUX).contains(&flux) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(flux)
    }
}

// ============================================================================
// Spin-down (undriven) flux linkage measurement
// ============================================================================

/// Accumulator for spin-down (undriven) flux linkage measurement.
///
/// Collects back-EMF voltage magnitude and angular velocity samples
/// during motor coast-down (all FETs off), then computes:
///
///   `λ = avg(|V_bemf|) / avg(|ωe|)`
///
/// This method does **not** depend on resistance or inductance — only
/// on voltage and speed measurement.
#[derive(Clone, Debug)]
pub struct SpinDownFluxMeasurement {
    v_bemf_sum: f32,
    omega_sum: f32,
    sample_count: u32,
    min_samples: u32,
    min_omega: f32,
}

impl SpinDownFluxMeasurement {
    /// Create from flux linkage parameters.
    ///
    /// Uses `params.num_samples` and `params.min_coast_omega_e`.
    pub fn from_params(params: &super::types::FluxLinkageParams) -> Self {
        Self::new(params.num_samples, params.min_coast_omega_e)
    }

    /// Create a new spin-down flux measurement.
    ///
    /// Prefer [`from_params`](Self::from_params) to avoid divergence
    /// between the threshold here and `FluxLinkageParams::min_coast_omega_e`.
    ///
    /// # Arguments
    /// * `min_samples` - Minimum samples required for a valid result
    /// * `min_omega` - Minimum |ωe| (rad/s) to accept a sample
    pub fn new(min_samples: u32, min_omega: f32) -> Self {
        Self {
            v_bemf_sum: 0.0,
            omega_sum: 0.0,
            sample_count: 0,
            min_samples,
            min_omega,
        }
    }

    /// Record a coast-down sample.
    ///
    /// Samples with |ωe| below `min_omega` are rejected (back-EMF
    /// too small for accurate measurement).
    ///
    /// # Arguments
    /// * `v_bemf` - Back-EMF voltage magnitude `√(vα² + vβ²)` in Volts
    /// * `omega_e` - Electrical angular velocity in rad/s
    ///
    /// # Returns
    /// `true` if the sample was accepted, `false` if rejected.
    pub fn record(&mut self, v_bemf: f32, omega_e: f32) -> bool {
        if omega_e.abs() < self.min_omega {
            return false;
        }
        self.v_bemf_sum += v_bemf.abs();
        self.omega_sum += omega_e.abs();
        self.sample_count += 1;
        true
    }

    /// Check whether enough samples have been collected.
    pub fn has_enough_samples(&self) -> bool {
        self.sample_count >= self.min_samples
    }

    /// Compute the flux linkage from accumulated samples.
    pub fn finish(self) -> Result<f32, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;
        let avg_v = self.v_bemf_sum / n;
        let avg_omega = self.omega_sum / n;

        if avg_omega < self.min_omega {
            return Err(DetectionError::LowConfidence);
        }

        let flux = avg_v / avg_omega;

        if !(MIN_VALID_FLUX..=MAX_VALID_FLUX).contains(&flux) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(flux)
    }

    /// Get an intermediate estimate without consuming the accumulator.
    pub fn current_estimate(&self) -> Option<f32> {
        if self.sample_count < 5 {
            return None;
        }
        let n = self.sample_count as f32;
        let avg_omega = self.omega_sum / n;
        if avg_omega < self.min_omega {
            return None;
        }
        Some(self.v_bemf_sum / n / avg_omega)
    }
}

/// Convert mechanical RPM to electrical angular velocity.
///
/// # Arguments
/// * `rpm` - Mechanical RPM
/// * `pole_pairs` - Number of motor pole pairs
///
/// # Returns
/// Electrical angular velocity in rad/s
#[inline]
pub fn rpm_to_omega_e(rpm: f32, pole_pairs: u8) -> f32 {
    // ωe = (rpm / 60) × 2π × pole_pairs
    rpm * core::f32::consts::TAU * pole_pairs as f32 / 60.0
}

/// Convert electrical angular velocity to mechanical RPM.
///
/// # Arguments
/// * `omega_e` - Electrical angular velocity in rad/s
/// * `pole_pairs` - Number of motor pole pairs
///
/// # Returns
/// Mechanical RPM
#[inline]
pub fn omega_e_to_rpm(omega_e: f32, pole_pairs: u8) -> f32 {
    omega_e * 60.0 / (core::f32::consts::TAU * pole_pairs as f32)
}

/// Convert eRPM (electrical RPM) to mechanical RPM.
///
/// # Arguments
/// * `erpm` - Electrical RPM
/// * `pole_pairs` - Number of motor pole pairs
///
/// # Returns
/// Mechanical RPM
#[inline]
pub fn erpm_to_rpm(erpm: f32, pole_pairs: u8) -> f32 {
    erpm / pole_pairs as f32
}

/// Convert mechanical RPM to eRPM (electrical RPM).
///
/// # Arguments
/// * `rpm` - Mechanical RPM
/// * `pole_pairs` - Number of motor pole pairs
///
/// # Returns
/// Electrical RPM
#[inline]
pub fn rpm_to_erpm(rpm: f32, pole_pairs: u8) -> f32 {
    rpm * pole_pairs as f32
}

/// Calculate motor Kv from flux linkage.
///
/// Kv (velocity constant) is the motor's no-load speed per volt.
///
/// # Arguments
/// * `flux_linkage_wb` - Flux linkage in Weber
/// * `pole_pairs` - Number of pole pairs
///
/// # Returns
/// Kv in RPM per Volt
///
/// # Example
/// ```
/// use oxifoc_core::foc::detection::flux_linkage::calculate_kv;
///
/// // 10 mWb flux, 7 pole pairs
/// let kv = calculate_kv(0.01, 7);
/// // Kv ≈ 136.5 RPM/V
/// assert!((kv - 136.5).abs() < 1.0);
/// ```
#[inline]
pub fn calculate_kv(flux_linkage_wb: f32, pole_pairs: u8) -> f32 {
    // Kv = 60 / (2π × λ × pole_pairs)
    60.0 / (core::f32::consts::TAU * flux_linkage_wb * pole_pairs as f32)
}

/// Calculate flux linkage from Kv.
///
/// # Arguments
/// * `kv_rpm_per_v` - Kv rating in RPM per Volt
/// * `pole_pairs` - Number of pole pairs
///
/// # Returns
/// Flux linkage in Weber
#[inline]
pub fn calculate_flux_from_kv(kv_rpm_per_v: f32, pole_pairs: u8) -> f32 {
    // λ = 60 / (2π × Kv × pole_pairs)
    60.0 / (core::f32::consts::TAU * kv_rpm_per_v * pole_pairs as f32)
}

/// Calculate observer gain from flux linkage (VESC formula).
///
/// The observer gain is used for sensorless position estimation.
///
/// # Arguments
/// * `flux_linkage_wb` - Flux linkage in Weber
///
/// # Returns
/// Observer gain
#[inline]
pub fn calculate_observer_gain(flux_linkage_wb: f32) -> f32 {
    // VESC uses: gain = 0.5e3 / λ²
    0.5e3 / (flux_linkage_wb * flux_linkage_wb)
}

/// Validate measured flux linkage.
///
/// # Arguments
/// * `flux_linkage` - Measured flux linkage in Weber
///
/// # Returns
/// * `Ok(())` - Flux linkage is valid
/// * `Err(DetectionError)` - Flux linkage is out of expected range
pub fn validate_flux_linkage(flux_linkage: f32) -> Result<(), DetectionError> {
    if flux_linkage < MIN_VALID_FLUX {
        return Err(DetectionError::OutOfRange);
    }
    if flux_linkage > MAX_VALID_FLUX {
        return Err(DetectionError::OutOfRange);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flux_measurement_basic() {
        let mut measurement = FluxLinkageMeasurement::new(0.1, 10);

        // Simulate steady-state: 500 RPM, 7 pole pairs, 10A current
        // Expected λ ≈ 0.01 Wb
        let pole_pairs = 7u8;
        let rpm = 500.0;
        let omega_e = rpm_to_omega_e(rpm, pole_pairs);
        let iq = 10.0;
        let flux = 0.01; // 10 mWb

        // Vq = R×Iq + ωe×λ = 0.1×10 + 366.5×0.01 = 1.0 + 3.665 = 4.665V
        let vq = 0.1 * iq + omega_e * flux;

        for _ in 0..10 {
            measurement.record(vq, iq, omega_e);
        }

        let measured_flux = measurement.finish().unwrap();

        // Should be close to expected flux (within 5%)
        let error = (measured_flux - flux).abs() / flux;
        assert!(error < 0.05, "Flux error too large: {}", error);
    }

    #[test]
    fn test_flux_measurement_with_rpm() {
        let mut measurement = FluxLinkageMeasurement::new(0.1, 10);

        let pole_pairs = 7u8;
        let rpm = 500.0;
        let iq = 10.0;
        let flux = 0.01;
        let omega_e = rpm_to_omega_e(rpm, pole_pairs);
        let vq = 0.1 * iq + omega_e * flux;

        for _ in 0..10 {
            measurement.record_with_rpm(vq, iq, rpm, pole_pairs);
        }

        let measured_flux = measurement.finish().unwrap();
        let error = (measured_flux - flux).abs() / flux;
        assert!(error < 0.05);
    }

    #[test]
    fn test_flux_measurement_insufficient_samples() {
        let mut measurement = FluxLinkageMeasurement::new(0.1, 100);

        // Only add 10 samples when 100 are required
        for _ in 0..10 {
            measurement.record(5.0, 10.0, 100.0);
        }

        assert_eq!(
            measurement.finish(),
            Err(DetectionError::InsufficientSamples)
        );
    }

    #[test]
    fn test_flux_measurement_low_speed() {
        let mut measurement = FluxLinkageMeasurement::new(0.1, 10);

        // Very low angular velocity
        for _ in 0..10 {
            measurement.record(1.0, 10.0, 1.0); // Only 1 rad/s
        }

        assert_eq!(measurement.finish(), Err(DetectionError::LowConfidence));
    }

    #[test]
    fn test_rpm_conversions() {
        let rpm = 1000.0;
        let pole_pairs = 7u8;

        let omega_e = rpm_to_omega_e(rpm, pole_pairs);
        let rpm_back = omega_e_to_rpm(omega_e, pole_pairs);

        assert!((rpm - rpm_back).abs() < 0.01);
    }

    #[test]
    fn test_erpm_conversions() {
        let erpm = 7000.0;
        let pole_pairs = 7u8;

        let rpm = erpm_to_rpm(erpm, pole_pairs);
        assert!((rpm - 1000.0).abs() < 0.01);

        let erpm_back = rpm_to_erpm(rpm, pole_pairs);
        assert!((erpm - erpm_back).abs() < 0.01);
    }

    #[test]
    fn test_kv_calculation() {
        // λ = 10mWb, 7 pole pairs
        let flux = 0.01;
        let pole_pairs = 7u8;

        let kv = calculate_kv(flux, pole_pairs);
        // Kv = 60 / (2π × 0.01 × 7) ≈ 136.5 RPM/V
        assert!((kv - 136.5).abs() < 1.0);

        // Reverse calculation
        let flux_back = calculate_flux_from_kv(kv, pole_pairs);
        assert!((flux - flux_back).abs() < 0.0001);
    }

    #[test]
    fn test_observer_gain() {
        let flux = 0.01; // 10mWb
        let gain = calculate_observer_gain(flux);

        // gain = 0.5e3 / 0.01² = 0.5e3 / 0.0001 = 5e6
        assert!((gain - 5e6).abs() < 1e5);
    }

    #[test]
    fn test_current_estimate() {
        let mut measurement = FluxLinkageMeasurement::new(0.1, 100);

        // Not enough samples yet
        for _ in 0..5 {
            measurement.record(5.0, 10.0, 100.0);
        }
        assert!(measurement.current_estimate().is_none());

        // After 10 samples, should have estimate
        for _ in 0..10 {
            measurement.record(5.0, 10.0, 100.0);
        }
        assert!(measurement.current_estimate().is_some());
    }
}
