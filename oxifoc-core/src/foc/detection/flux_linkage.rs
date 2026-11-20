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
use crate::foc::constants;
use crate::foc::fast_math::sqrtf;
use crate::units::{
    ConstZero, ElectricCurrent, ElectricPotential, ElectricalAngularVelocity, ElectricalResistance,
    Inductance, MagneticFlux, erad_per_s, ohms, seconds, volts, webers,
};

/// Plausible permanent-magnet flux linkage.
const VALID_FLUX: core::ops::RangeInclusive<MagneticFlux> = webers(0.0001)..=webers(1.0);

/// Minimum electrical angular velocity for a valid measurement (~95 eRPM).
const MIN_VALID_OMEGA: ElectricalAngularVelocity = erad_per_s(10.0);

/// One coherent undriven voltage sample. Hall sectors are optional for model
/// backends; physical Hall-equipped boards also supply the captured edge age.
#[derive(Clone, Copy, Debug)]
pub struct CoastSample {
    pub sequence: u32,
    pub timestamp_us: u64,
    pub v_alpha: f32,
    pub v_beta: f32,
    pub omega_e: f32,
    pub hall_sector: Option<u8>,
    pub hall_edge_age_s: f32,
}

impl CoastSample {
    pub fn magnitude(&self) -> f32 {
        sqrtf(self.v_alpha * self.v_alpha + self.v_beta * self.v_beta)
    }
}

/// Integrate BEMF magnitude over complete Hall revolutions:
/// `lambda = integral(|V_bemf| dt) / (2*pi*turns)`.
/// Unlike pointwise V/omega this does not use a lagging sector-speed estimate
/// during deceleration. Edge timestamps interpolate the integration boundaries.
pub struct HallCoastMeasurement {
    previous: Option<(CoastSample, ElectricPotential)>,
    /// `∫|V_bemf| dt`: one electrical turn integrates to `2π·λ`.
    integral: MagneticFlux,
    anchor: Option<(u8, MagneticFlux)>,
    steps: u8,
    direction: i8,
    samples: u32,
    turns: u32,
    lambda_sum: MagneticFlux,
    lambda_min: MagneticFlux,
    lambda_max: MagneticFlux,
}

impl Default for HallCoastMeasurement {
    fn default() -> Self {
        Self {
            previous: None,
            integral: MagneticFlux::ZERO,
            anchor: None,
            steps: 0,
            direction: 0,
            samples: 0,
            turns: 0,
            lambda_sum: MagneticFlux::ZERO,
            lambda_min: webers(f32::INFINITY),
            lambda_max: MagneticFlux::ZERO,
        }
    }
}

impl HallCoastMeasurement {
    /// Three complete turns and at least 32 samples qualify the window.
    pub fn has_enough_samples(&self) -> bool {
        self.turns >= 3 && self.samples >= 32
    }

    pub fn record(&mut self, sample: CoastSample) -> Result<Option<MagneticFlux>, DetectionError> {
        let sector = sample
            .hall_sector
            .filter(|s| *s < 6)
            .ok_or(DetectionError::LowConfidence)?;
        let voltage = volts(sample.magnitude());
        if !voltage.is_finite()
            || !sample.omega_e.is_finite()
            || !sample.hall_edge_age_s.is_finite()
            || sample.hall_edge_age_s < 0.0
        {
            return Err(DetectionError::LowConfidence);
        }
        self.samples += 1;
        let Some((previous, previous_voltage)) = self.previous.replace((sample, voltage)) else {
            return Ok(None);
        };
        let dt = seconds(
            sample
                .timestamp_us
                .checked_sub(previous.timestamp_us)
                .filter(|dt| *dt > 0)
                .ok_or(DetectionError::LowConfidence)? as f32
                * 1e-6,
        );
        self.integral += 0.5 * (previous_voltage + voltage) * dt;
        let previous_sector = previous.hall_sector.ok_or(DetectionError::LowConfidence)?;
        if sector == previous_sector {
            return Ok(None);
        }
        let step = (sector + 6 - previous_sector) % 6;
        let direction = match step {
            1 => 1,
            5 => -1,
            _ => return Err(DetectionError::LowConfidence),
        };
        if self.direction != 0 && direction != self.direction {
            return Err(DetectionError::LowConfidence);
        }
        self.direction = direction;
        let tail = seconds(sample.hall_edge_age_s).min(dt);
        let edge_voltage = voltage + (previous_voltage - voltage) * tail / dt;
        let at_edge = self.integral - 0.5 * (edge_voltage + voltage) * tail;
        let Some((anchor_sector, anchor_integral)) = self.anchor else {
            self.anchor = Some((sector, at_edge));
            return Ok(None);
        };
        self.steps += 1;
        if sector != anchor_sector {
            return Ok(None);
        }
        if self.steps != 6 {
            return Err(DetectionError::LowConfidence);
        }
        let lambda = (at_edge - anchor_integral) / core::f32::consts::TAU;
        if !VALID_FLUX.contains(&lambda) {
            return Err(DetectionError::OutOfRange);
        }
        self.turns += 1;
        self.lambda_sum += lambda;
        self.lambda_min = self.lambda_min.min(lambda);
        self.lambda_max = self.lambda_max.max(lambda);
        self.anchor = Some((sector, at_edge));
        self.steps = 0;
        Ok(Some(lambda))
    }

    pub fn finish(&self) -> Result<MagneticFlux, DetectionError> {
        if !self.has_enough_samples() {
            return Err(DetectionError::InsufficientSamples);
        }
        let lambda = self.lambda_sum / self.turns as f32;
        if (self.lambda_max - self.lambda_min) > 0.1 * lambda {
            return Err(DetectionError::LowConfidence);
        }
        Ok(lambda)
    }
}

/// Accumulator for flux linkage measurement samples.
///
/// Collects steady-state voltage, current, and velocity samples
/// and computes the average flux linkage.
#[derive(Clone, Debug)]
pub struct FluxLinkageMeasurement {
    /// Known phase resistance (from previous measurement)
    resistance: ElectricalResistance,
    /// Sum of Vq samples
    vq_sum: ElectricPotential,
    /// Sum of Iq samples
    iq_sum: ElectricCurrent,
    /// Sum of electrical angular velocity samples
    omega_sum: ElectricalAngularVelocity,
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
    pub fn new(resistance: ElectricalResistance, min_samples: u32) -> Self {
        Self {
            resistance,
            vq_sum: ElectricPotential::ZERO,
            iq_sum: ElectricCurrent::ZERO,
            omega_sum: ElectricalAngularVelocity::ZERO,
            sample_count: 0,
            min_samples,
        }
    }

    /// Create from flux linkage parameters.
    pub fn from_params(params: &FluxLinkageParams) -> Result<Self, DetectionError> {
        if params.resistance_ohm <= 0.0 {
            return Err(DetectionError::MissingPrerequisite);
        }

        Ok(Self::new(ohms(params.resistance_ohm), params.num_samples))
    }

    /// Record a sample during steady-state spinning.
    ///
    /// # Arguments
    /// * `vq` - q-axis voltage
    /// * `iq` - q-axis current
    /// * `omega_e` - electrical angular velocity
    ///
    /// Note: Only call this during steady-state operation at target speed.
    #[inline]
    pub fn record(
        &mut self,
        vq: ElectricPotential,
        iq: ElectricCurrent,
        omega_e: ElectricalAngularVelocity,
    ) {
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
    /// * `vq` - q-axis voltage
    /// * `iq` - q-axis current
    /// * `rpm` - mechanical RPM
    /// * `pole_pairs` - number of motor pole pairs
    #[inline]
    pub fn record_with_rpm(
        &mut self,
        vq: ElectricPotential,
        iq: ElectricCurrent,
        rpm: f32,
        pole_pairs: u8,
    ) {
        self.record(vq, iq, erad_per_s(rpm_to_omega_e(rpm, pole_pairs)));
    }

    /// Reset the accumulator for a new measurement.
    #[inline]
    pub fn reset(&mut self) {
        self.vq_sum = ElectricPotential::ZERO;
        self.iq_sum = ElectricCurrent::ZERO;
        self.omega_sum = ElectricalAngularVelocity::ZERO;
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
    /// * `Ok(flux_linkage)` - Flux linkage
    /// * `Err(DetectionError)` - If measurement failed
    pub fn finish(self) -> Result<MagneticFlux, DetectionError> {
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
        let flux_linkage: MagneticFlux = (avg_vq - self.resistance * avg_iq) / avg_omega;

        // Validate result
        if !VALID_FLUX.contains(&flux_linkage) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(flux_linkage)
    }

    /// Get intermediate flux estimate without consuming the accumulator.
    ///
    /// Useful for monitoring convergence during measurement.
    pub fn current_estimate(&self) -> Option<MagneticFlux> {
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

        Some((avg_vq - self.resistance * avg_iq) / avg_omega)
    }
}

// ============================================================================
// Back-EMF-vector driven flux linkage measurement
// ============================================================================

/// Accumulator for driven flux linkage measurement via the back-EMF vector.
///
/// Solves the steady-state rotating-frame voltage equations for the
/// back-EMF vector and takes its magnitude:
///
///   `e_d = vd − R·id + ωe·L·iq`
///   `e_q = vq − R·iq − ωe·L·id`
///   `λ = |e⃗| / ωe`
///
/// This is exact at steady state for **any** load angle. That matters in
/// open-loop drive: the rotor pulls its d axis onto the current vector, so
/// it leads the command frame by up to 90° and the back-EMF ends up nearly
/// perpendicular to the current. VESC's scalar approximation
/// `(|V| − R·|I|)/ω − |I|·L` subtracts the resistive drop as if it were
/// collinear with V and picks up a ≈ −R·|I|/ω bias there (tens of percent
/// for small high-R motors); the vector form has no such geometry
/// assumption. e⃗ is DC in the command frame at steady state, so the
/// components are averaged separately before taking the magnitude.
///
/// Requires both R and L from previous detection steps (SPM assumption:
/// L = (Ld+Lq)/2 is used for both axes).
#[derive(Clone, Debug)]
pub struct MagnitudeFluxMeasurement {
    resistance: ElectricalResistance,
    /// Average inductance for the reactance term.
    inductance: Inductance,
    e_d_sum: ElectricPotential,
    e_q_sum: ElectricPotential,
    omega_sum: ElectricalAngularVelocity,
    sample_count: u32,
    min_samples: u32,
}

impl MagnitudeFluxMeasurement {
    /// Create a new magnitude-based flux measurement.
    ///
    /// # Arguments
    /// * `resistance` — previously measured R
    /// * `inductance` — previously measured L_avg
    /// * `min_samples` — minimum samples for valid result
    pub fn new(resistance: ElectricalResistance, inductance: Inductance, min_samples: u32) -> Self {
        Self {
            resistance,
            inductance,
            e_d_sum: ElectricPotential::ZERO,
            e_q_sum: ElectricPotential::ZERO,
            omega_sum: ElectricalAngularVelocity::ZERO,
            sample_count: 0,
            min_samples,
        }
    }

    /// Record a sample using all four dq components.
    #[inline]
    pub fn record(
        &mut self,
        vd: ElectricPotential,
        vq: ElectricPotential,
        id: ElectricCurrent,
        iq: ElectricCurrent,
        omega_e: ElectricalAngularVelocity,
    ) {
        // Back out the back-EMF vector from the steady-state dq equations.
        let reactance = omega_e * self.inductance;
        self.e_d_sum += vd - self.resistance * id + reactance * iq;
        self.e_q_sum += vq - self.resistance * iq - reactance * id;
        self.omega_sum += omega_e;
        self.sample_count += 1;
    }

    /// Compute the flux linkage.
    pub fn finish(self) -> Result<MagneticFlux, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;
        let avg_e_d = self.e_d_sum / n;
        let avg_e_q = self.e_q_sum / n;
        let avg_omega = self.omega_sum / n;

        if avg_omega.abs() < MIN_VALID_OMEGA {
            return Err(DetectionError::LowConfidence);
        }

        // λ = |e⃗| / ω
        let emf = (avg_e_d * avg_e_d + avg_e_q * avg_e_q).sqrt_with(sqrtf);
        let flux: MagneticFlux = emf / avg_omega.abs();

        if !VALID_FLUX.contains(&flux) {
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
    v_bemf_sum: ElectricPotential,
    omega_sum: ElectricalAngularVelocity,
    sample_count: u32,
    min_samples: u32,
    min_omega: ElectricalAngularVelocity,
}

impl SpinDownFluxMeasurement {
    /// Create from flux linkage parameters.
    ///
    /// Uses `params.num_samples` (capped at 500) and
    /// `params.min_coast_omega_e`. The cap decouples this method from the
    /// driven-method window size: `num_samples` defaults to a full second
    /// of sampling to average out open-loop rotor hunting, but a coasting
    /// motor only spends so long above the ω floor — demanding the driven
    /// window here would force the fallback on motors whose coast is
    /// perfectly measurable. 500 samples (~0.25 s) is statistically ample
    /// for the V/ω ratio.
    pub fn from_params(params: &FluxLinkageParams) -> Self {
        Self::new(
            params.num_samples.min(500),
            erad_per_s(params.min_coast_omega_e),
        )
    }

    /// Create a new spin-down flux measurement.
    ///
    /// Prefer [`from_params`](Self::from_params) to avoid divergence
    /// between the threshold here and `FluxLinkageParams::min_coast_omega_e`.
    ///
    /// # Arguments
    /// * `min_samples` - Minimum samples required for a valid result
    /// * `min_omega` - Minimum |ωe| to accept a sample
    pub fn new(min_samples: u32, min_omega: ElectricalAngularVelocity) -> Self {
        Self {
            v_bemf_sum: ElectricPotential::ZERO,
            omega_sum: ElectricalAngularVelocity::ZERO,
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
    /// * `v_bemf` - Back-EMF voltage magnitude `√(vα² + vβ²)`
    /// * `omega_e` - Electrical angular velocity
    ///
    /// # Returns
    /// `true` if the sample was accepted, `false` if rejected.
    pub fn record(
        &mut self,
        v_bemf: ElectricPotential,
        omega_e: ElectricalAngularVelocity,
    ) -> bool {
        if !v_bemf.is_finite() || !omega_e.is_finite() || omega_e.abs() < self.min_omega {
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
    pub fn finish(self) -> Result<MagneticFlux, DetectionError> {
        if self.sample_count < self.min_samples {
            return Err(DetectionError::InsufficientSamples);
        }

        let n = self.sample_count as f32;
        let avg_v = self.v_bemf_sum / n;
        let avg_omega = self.omega_sum / n;

        if avg_omega < self.min_omega {
            return Err(DetectionError::LowConfidence);
        }

        let flux: MagneticFlux = avg_v / avg_omega;

        if !VALID_FLUX.contains(&flux) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(flux)
    }

    /// Get an intermediate estimate without consuming the accumulator.
    pub fn current_estimate(&self) -> Option<MagneticFlux> {
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
    rpm * core::f32::consts::TAU * f32::from(pole_pairs) / 60.0
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
    omega_e * 60.0 / (core::f32::consts::TAU * f32::from(pole_pairs))
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
    erpm / f32::from(pole_pairs)
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
    rpm * f32::from(pole_pairs)
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
/// # Convention
/// Kv is reported against the **line-to-line** voltage (nameplate / VESC /
/// ODrive convention), so the per-phase λ from the amplitude-invariant Park
/// frame (where `Vq` is the peak *phase* voltage) carries a √3 phase→line
/// factor: `Kv = 60 / (√3 · 2π · λ · Pp)`. Omitting it returns the per-phase Kv,
/// which is √3 (≈1.73×) too high vs the nameplate.
///
/// # Example
/// ```
/// use oxifoc_core::foc::detection::flux_linkage::calculate_kv;
///
/// // 10 mWb flux, 7 pole pairs
/// let kv = calculate_kv(0.01, 7);
/// // Kv ≈ 78.8 RPM/V (line-to-line)
/// assert!((kv - 78.8).abs() < 1.0);
/// ```
#[inline]
pub fn calculate_kv(flux_linkage_wb: f32, pole_pairs: u8) -> f32 {
    // Kv = 60 / (√3 × 2π × λ × pole_pairs)  — √3 = peak phase → peak line-to-line
    60.0 / (constants::SQRT_3 * core::f32::consts::TAU * flux_linkage_wb * f32::from(pole_pairs))
}

/// Calculate flux linkage from Kv.
///
/// # Arguments
/// * `kv_rpm_per_v` - Kv rating in RPM per Volt
/// * `pole_pairs` - Number of pole pairs
///
/// # Returns
/// Flux linkage in Weber
///
/// Inverse of [`calculate_kv`] — `kv_rpm_per_v` is the line-to-line Kv, so the
/// same √3 factor applies: `λ = 60 / (√3 · 2π · Kv · Pp)` (matches VESC).
#[inline]
pub fn calculate_flux_from_kv(kv_rpm_per_v: f32, pole_pairs: u8) -> f32 {
    // λ = 60 / (√3 × 2π × Kv × pole_pairs)
    60.0 / (constants::SQRT_3 * core::f32::consts::TAU * kv_rpm_per_v * f32::from(pole_pairs))
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
    if VALID_FLUX.contains(&webers(flux_linkage)) {
        Ok(())
    } else {
        Err(DetectionError::OutOfRange)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{amps, henries};

    // The numeric checks below are in plain SI floats; these views shadow
    // the typed accumulators at the test boundary.
    struct FluxLinkageMeasurement(super::FluxLinkageMeasurement);
    impl FluxLinkageMeasurement {
        fn new(resistance_ohm: f32, min_samples: u32) -> Self {
            Self(super::FluxLinkageMeasurement::new(
                ohms(resistance_ohm),
                min_samples,
            ))
        }
        fn record(&mut self, vq: f32, iq: f32, omega_e: f32) {
            self.0.record(volts(vq), amps(iq), erad_per_s(omega_e));
        }
        fn record_with_rpm(&mut self, vq: f32, iq: f32, rpm: f32, pole_pairs: u8) {
            self.0.record_with_rpm(volts(vq), amps(iq), rpm, pole_pairs);
        }
        fn finish(self) -> Result<f32, DetectionError> {
            self.0.finish().map(|l| l.value)
        }
        fn current_estimate(&self) -> Option<f32> {
            self.0.current_estimate().map(|l| l.value)
        }
    }

    struct MagnitudeFluxMeasurement(super::MagnitudeFluxMeasurement);
    impl MagnitudeFluxMeasurement {
        fn new(resistance_ohm: f32, inductance_h: f32, min_samples: u32) -> Self {
            Self(super::MagnitudeFluxMeasurement::new(
                ohms(resistance_ohm),
                henries(inductance_h),
                min_samples,
            ))
        }
        fn record(&mut self, vd: f32, vq: f32, id: f32, iq: f32, omega_e: f32) {
            self.0.record(
                volts(vd),
                volts(vq),
                amps(id),
                amps(iq),
                erad_per_s(omega_e),
            );
        }
        fn finish(self) -> Result<f32, DetectionError> {
            self.0.finish().map(|l| l.value)
        }
    }

    struct SpinDownFluxMeasurement(super::SpinDownFluxMeasurement);
    impl SpinDownFluxMeasurement {
        fn new(min_samples: u32, min_omega: f32) -> Self {
            Self(super::SpinDownFluxMeasurement::new(
                min_samples,
                erad_per_s(min_omega),
            ))
        }
        fn record(&mut self, v_bemf: f32, omega_e: f32) -> bool {
            self.0.record(volts(v_bemf), erad_per_s(omega_e))
        }
        fn has_enough_samples(&self) -> bool {
            self.0.has_enough_samples()
        }
        fn finish(self) -> Result<f32, DetectionError> {
            self.0.finish().map(|l| l.value)
        }
    }

    #[derive(Default)]
    struct HallCoastMeasurement(super::HallCoastMeasurement);
    impl HallCoastMeasurement {
        fn record(&mut self, sample: CoastSample) -> Result<Option<f32>, DetectionError> {
            self.0.record(sample).map(|l| l.map(|l| l.value))
        }
        fn has_enough_samples(&self) -> bool {
            self.0.has_enough_samples()
        }
        fn finish(&self) -> Result<f32, DetectionError> {
            self.0.finish().map(|l| l.value)
        }
    }

    /// Synthesize exact steady-state command-frame quantities for an SPM
    /// motor spinning synchronously at `omega` with the stator current on
    /// the command q axis and the rotor d axis leading the command frame by
    /// `gamma` radians:
    ///   e⃗ = jωλ∠γ  →  e_d = −ωλ·sin γ,  e_q = ωλ·cos γ
    ///   v⃗ = R·i⃗ + jωL·i⃗ + e⃗,  i⃗ = (0, I)  →  jωL·i⃗ = (−ωLI, 0)
    fn steady_state_dq(r: f32, l: f32, lambda: f32, i: f32, omega: f32, gamma: f32) -> [f32; 4] {
        let (s, c) = (libm::sinf(gamma), libm::cosf(gamma));
        let vd = -omega * l * i - omega * lambda * s;
        let vq = r * i + omega * lambda * c;
        [vd, vq, 0.0, i]
    }

    #[test]
    fn driven_flux_is_load_angle_invariant() {
        // In open-loop drive the rotor pulls its d axis onto the current
        // vector, so at light load it LEADS the command frame by ~90°
        // (γ ≈ π/2) and the back-EMF is nearly perpendicular to the
        // current. The scalar VESC formula (|V| − R|I|)/ω − |I|L subtracts
        // R|I| as if it were collinear with V and is biased by ≈ −RI/ω
        // there. The exact relation e⃗ = V⃗ − R·i⃗ − jωL·i⃗, λ = |e⃗|/ω holds
        // for ANY load angle.
        let (r, l, lambda, i, omega) = (0.5, 0.5e-3, 0.01, 2.0, 300.0);

        for gamma_deg in [0.0f32, 30.0, 60.0, 90.0] {
            let gamma = gamma_deg.to_radians();
            let [vd, vq, id, iq] = steady_state_dq(r, l, lambda, i, omega, gamma);

            let mut m = MagnitudeFluxMeasurement::new(r, l, 10);
            for _ in 0..10 {
                m.record(vd, vq, id, iq, omega);
            }
            let est = m.finish().unwrap();
            let err = (est - lambda).abs() / lambda;
            assert!(
                err < 0.02,
                "λ error {:.1}% at load angle {}° (est {:.4} vs true {:.4})",
                err * 100.0,
                gamma_deg,
                est,
                lambda
            );
        }
    }

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
        assert!(error < 0.05, "Flux error too large: {error}");
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

    /// Spin-down accumulator: a decelerating coast (|e| = λ·ω throughout)
    /// gives λ, samples below the ω floor are refused, and a coast that
    /// ends before the window fills reports it.
    #[test]
    fn spin_down_measurement_reads_lambda_off_a_coast() {
        let lambda = 0.0085;
        let mut m = SpinDownFluxMeasurement::new(100, 50.0);
        let mut omega = 1_000.0f32;
        while !m.has_enough_samples() {
            assert!(m.record(lambda * omega, -omega), "ω {omega} refused");
            omega *= 0.99;
        }
        assert!(!m.record(lambda * 40.0, 40.0), "below the ω floor");
        let measured = m.finish().unwrap();
        assert!((measured - lambda).abs() < 1e-3 * lambda, "{measured}");

        let mut short = SpinDownFluxMeasurement::new(100, 50.0);
        for k in 0..20 {
            short.record(lambda * 100.0, 100.0 - k as f32);
        }
        assert_eq!(short.finish(), Err(DetectionError::InsufficientSamples));
    }

    #[test]
    fn coast_nonfinite_samples_cannot_satisfy_the_collection_window() {
        let mut measurement = SpinDownFluxMeasurement::new(2, 50.0);
        for (voltage, omega) in [
            (f32::NAN, 100.0),
            (1.0, f32::NAN),
            (f32::INFINITY, 100.0),
            (1.0, f32::INFINITY),
        ] {
            assert!(!measurement.record(voltage, omega));
        }
        assert!(measurement.record(0.3, 100.0));
        assert!(!measurement.has_enough_samples());
        assert!(measurement.record(0.6, 200.0));
        assert!((measurement.finish().unwrap() - 0.003).abs() < 1e-7);
    }

    fn coast_plant_sample(n: u32, time: f64, direction: f64, decel: f64) -> CoastSample {
        let omega = 600.0 - decel * time;
        let theta = 600.0 * time - 0.5 * decel * time * time;
        let edge = (theta / (core::f64::consts::TAU / 6.0)).floor();
        let edge_theta = edge * core::f64::consts::TAU / 6.0;
        let edge_time = if decel == 0.0 {
            edge_theta / 600.0
        } else {
            (600.0 - (360000.0 - 2.0 * decel * edge_theta).sqrt()) / decel
        };
        let sector = if direction > 0.0 {
            edge as u8 % 6
        } else {
            (6 - edge as u8 % 6) % 6
        };
        CoastSample {
            sequence: n,
            timestamp_us: (time * 1e6).round() as u64,
            v_alpha: (0.0032 * omega * (direction * theta).cos()) as f32,
            v_beta: (0.0032 * omega * (direction * theta).sin()) as f32,
            omega_e: (direction * omega) as f32,
            hall_sector: Some(sector),
            hall_edge_age_s: (time - edge_time) as f32,
        }
    }

    #[test]
    fn complete_coast_turns_measure_flux_during_fast_deceleration_both_directions() {
        for direction in [-1.0, 1.0] {
            for decel in [0.0, 3000.0] {
                let mut measurement = HallCoastMeasurement::default();
                let mut time = 0.0;
                for n in 0..200 {
                    let mut sample = coast_plant_sample(n, time, direction, decel);
                    // Instantaneous Hall speed may lag badly during deceleration;
                    // only captured sector boundaries establish total rotation.
                    sample.omega_e = direction as f32 * 700.0;
                    sample.v_alpha += 0.02;
                    sample.v_beta -= 0.015;
                    measurement.record(sample).unwrap();
                    if measurement.has_enough_samples() {
                        break;
                    }
                    time += if n % 3 == 0 { 0.00091 } else { 0.00053 };
                }
                let flux = measurement.finish().unwrap();
                assert!(
                    (flux - 0.0032).abs() / 0.0032 < 0.002,
                    "{direction} {decel}: {flux}"
                );
            }
        }
    }

    #[test]
    fn coast_turn_window_rejects_skipped_edges_nonfinite_data_and_nonmonotonic_time() {
        for kind in 0..3 {
            let mut measurement = HallCoastMeasurement::default();
            let first = coast_plant_sample(0, 0.0, 1.0, 0.0);
            measurement.record(first).unwrap();
            let mut sample = coast_plant_sample(1, 0.0005, 1.0, 0.0);
            if kind == 0 {
                sample.hall_sector = Some(2);
            }
            if kind == 1 {
                sample.timestamp_us = 0;
            }
            if kind == 2 {
                sample.v_alpha = f32::NAN;
            }
            assert_eq!(
                measurement.record(sample),
                Err(DetectionError::LowConfidence)
            );
            assert_eq!(
                measurement.finish(),
                Err(DetectionError::InsufficientSamples)
            );
        }
    }

    #[test]
    fn coast_turn_window_rejects_a_direction_reversal() {
        let mut measurement = HallCoastMeasurement::default();
        measurement
            .record(coast_plant_sample(0, 0.0, 1.0, 0.0))
            .unwrap();
        measurement
            .record(coast_plant_sample(1, 0.002, 1.0, 0.0))
            .unwrap();
        let mut reversed = coast_plant_sample(2, 0.004, 1.0, 0.0);
        reversed.hall_sector = Some(0);
        assert_eq!(
            measurement.record(reversed),
            Err(DetectionError::LowConfidence)
        );
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
        // Kv = 60 / (√3 × 2π × 0.01 × 7) ≈ 78.8 RPM/V (line-to-line)
        assert!((kv - 78.8).abs() < 1.0);

        // Reverse calculation (inverse carries the same √3, so it round-trips)
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
