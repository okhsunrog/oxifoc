//! Automatic PI controller tuning from measured motor parameters.
//!
//! Calculates optimal PI controller gains (Kp, Ki) for current control loops
//! based on measured motor resistance and inductance.
//!
//! # Theory
//!
//! For a current control loop in FOC, the plant model is:
//! ```text
//! V = R·I + L·dI/dt
//! ```
//!
//! A PI controller with gains Kp and Ki, tuned for a desired bandwidth ω_bw:
//! ```text
//! Kp = L × ω_bw
//! Ki = R × ω_bw
//! ```
//!
//! This places the closed-loop pole at -ω_bw, giving first-order response
//! with time constant τ = 1/ω_bw.

use core::ops::Div;

use super::types::MotorParams;
#[cfg(feature = "detection")]
use crate::foc::clamp_f32;
#[cfg(feature = "detection")]
use crate::units::MagneticFlux;
use crate::units::{
    AngularVelocity, ElectricCurrent, ElectricPotential, ElectricalResistance, Frequency,
    Inductance, Scalar, Sign, Time, henries, rad_per_s,
};

/// Integral gain of a current PI: volts per ampere-second (`R·ω_bw`).
pub type CurrentIntegralGain = <ElectricalResistance as Div<Time>>::Output;

/// Default current loop bandwidth.
///
/// 1000 rad/s corresponds to ~160 Hz, suitable for most applications.
pub const DEFAULT_BANDWIDTH: AngularVelocity = rad_per_s(1000.0);

/// Maximum recommended bandwidth.
///
/// Higher bandwidths may cause instability due to PWM delays and sampling.
pub const MAX_BANDWIDTH: AngularVelocity = rad_per_s(5000.0);

/// Minimum recommended bandwidth in rad/s.
pub const MIN_BANDWIDTH: AngularVelocity = rad_per_s(100.0);

/// Result of PI tuning calculation.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PiGains {
    /// Proportional gain for d-axis current controller
    pub kp_d: ElectricalResistance,

    /// Integral gain for d-axis current controller
    pub ki_d: CurrentIntegralGain,

    /// Proportional gain for q-axis current controller
    pub kp_q: ElectricalResistance,

    /// Integral gain for q-axis current controller
    pub ki_q: CurrentIntegralGain,

    /// Actual bandwidth achieved
    pub bandwidth: AngularVelocity,
}

/// Calculate PI gains for current control from motor parameters.
///
/// # Arguments
/// * `resistance` - Motor phase resistance
/// * `inductance` - Motor inductance (use Ld for d-axis, Lq for q-axis)
/// * `bandwidth` - Desired control bandwidth
///
/// # Returns
/// `(Kp, Ki)`: Kp in V/A (= Ω), Ki in V/(A·s). Swapping R and L does not
/// compile.
///
/// # Example
/// ```
/// use oxifoc_core::foc::detection::pi_tuning::calculate_current_gains;
/// use oxifoc_core::units::{henries, ohms, rad_per_s};
///
/// // Motor with R=0.1Ω, L=100µH, 1000 rad/s bandwidth
/// let (kp, ki) = calculate_current_gains(ohms(0.1), henries(100e-6), rad_per_s(1000.0));
/// assert!((kp.value - 0.1).abs() < 0.001);  // Kp = L × ω = 0.0001 × 1000
/// assert!((ki.value - 100.0).abs() < 1.0);   // Ki = R × ω = 0.1 × 1000
/// ```
///
/// ```compile_fail
/// use oxifoc_core::foc::detection::pi_tuning::calculate_current_gains;
/// use oxifoc_core::units::{henries, ohms, rad_per_s};
///
/// // R and L in the wrong order: a type error, not a 1000× wrong gain.
/// let _ = calculate_current_gains(henries(100e-6), ohms(0.1), rad_per_s(1000.0));
/// ```
#[inline]
pub fn calculate_current_gains(
    resistance: ElectricalResistance,
    inductance: Inductance,
    bandwidth: AngularVelocity,
) -> (ElectricalResistance, CurrentIntegralGain) {
    let kp: ElectricalResistance = inductance * bandwidth;
    let ki: CurrentIntegralGain = resistance * bandwidth;
    (kp, ki)
}

/// Calculate PI gains with voltage limiting consideration.
///
/// If the calculated gains would produce excessive control voltages,
/// the bandwidth is reduced to stay within limits.
///
/// # Arguments
/// * `resistance` - Motor phase resistance
/// * `inductance` - Motor inductance
/// * `target_bandwidth` - Desired bandwidth
/// * `max_voltage` - Maximum available control voltage
/// * `max_current` - Maximum expected current step
///
/// # Returns
/// Tuple of (Kp, Ki, actual_bandwidth)
pub fn calculate_current_gains_limited(
    resistance: ElectricalResistance,
    inductance: Inductance,
    target_bandwidth: AngularVelocity,
    max_voltage: ElectricPotential,
    max_current: ElectricCurrent,
) -> (ElectricalResistance, CurrentIntegralGain, AngularVelocity) {
    // Maximum voltage needed for step response: V = Kp × I_step
    // Limit Kp such that Kp × I_max <= V_max
    let max_kp: ElectricalResistance = max_voltage / max_current;
    let max_bandwidth_from_kp: AngularVelocity = (max_kp / inductance).into();

    // Use the more limiting bandwidth
    let actual_bandwidth = target_bandwidth
        .min(max_bandwidth_from_kp)
        .clamp_to(MIN_BANDWIDTH, MAX_BANDWIDTH);

    let (kp, ki) = calculate_current_gains(resistance, inductance, actual_bandwidth);
    (kp, ki, actual_bandwidth)
}

/// Calculate complete PI gains for both d and q axes.
///
/// For surface-mount PMSMs (SPMSM), Ld ≈ Lq, so both axes get the same gains.
/// For interior PMSMs (IPMSM), Ld < Lq, so q-axis gets higher Kp.
///
/// # Arguments
/// * `params` - Measured motor parameters
/// * `bandwidth` - Desired control bandwidth
///
/// # Returns
/// Complete PI gains for both axes, or None if parameters are invalid
pub fn calculate_foc_gains(params: &MotorParams, bandwidth: AngularVelocity) -> Option<PiGains> {
    if !params.resistance.is_positive() {
        return None;
    }
    // A measured axis wins; a missing one falls back to the average.
    let axis = |measured: Inductance| {
        if measured.is_positive() {
            Some(measured)
        } else if params.inductance_avg.is_positive() {
            Some(params.inductance_avg)
        } else {
            None
        }
    };
    let ld = axis(params.inductance_d)?;
    let lq = axis(params.inductance_q)?;

    let bandwidth = bandwidth.clamp_to(MIN_BANDWIDTH, MAX_BANDWIDTH);
    let (kp_d, ki_d) = calculate_current_gains(params.resistance, ld, bandwidth);
    let (kp_q, ki_q) = calculate_current_gains(params.resistance, lq, bandwidth);

    Some(PiGains {
        kp_d,
        ki_d,
        kp_q,
        ki_q,
        bandwidth,
    })
}

/// Estimate suitable bandwidth from motor parameters.
///
/// Uses heuristics based on typical motor characteristics:
/// - Smaller inductance → LOWER bandwidth: a low-L motor has large PWM
///   ripple and amplifies measurement noise into voltage (Kp = L·ω is small
///   but the plant responds fast), so it gets the conservative end
/// - Capped at 1/10 of the PWM frequency (sampling/delay limit)
///
/// # Returns
/// Recommended bandwidth
pub fn estimate_bandwidth(inductance: Inductance, pwm_freq: Frequency) -> AngularVelocity {
    use crate::units::unit::hertz;
    // Rule of thumb: bandwidth should be at most 1/10 of PWM frequency
    // to ensure adequate sampling and avoid aliasing (2π rad per cycle).
    let max_from_pwm = pwm_freq.get::<hertz>() * core::f32::consts::TAU / 10.0;

    // Also limit based on inductance - very low inductance motors
    // may have too fast dynamics for stable control
    let suggested: f32 = if inductance < henries(10e-6) {
        // Very low inductance (< 10µH): be conservative
        500.0
    } else if inductance < henries(100e-6) {
        // Typical small motor (10-100µH)
        1000.0
    } else if inductance < henries(1e-3) {
        // Medium motor (100µH - 1mH)
        2000.0
    } else {
        // Large motor (> 1mH)
        3000.0
    };

    rad_per_s(suggested.min(max_from_pwm)).clamp_to(MIN_BANDWIDTH, MAX_BANDWIDTH)
}

/// Calculate observer gain from motor parameters.
///
/// The observer gain is used for sensorless FOC to estimate rotor position.
/// Delegates to the single VESC formula (`gain = 0.5e3 / λ²`, the value the
/// VESC detection wizard stores: conf_general.c:1181) so there is exactly one
/// source of truth, and clamps the result to a sane range.
///
/// # Arguments
/// * `flux_linkage` - Motor flux linkage
///
/// # Returns
/// Observer gain, or None if flux linkage is invalid
#[cfg(feature = "detection")]
pub fn calculate_observer_gain(flux_linkage: MagneticFlux) -> Option<f32> {
    if !flux_linkage.is_positive() {
        return None;
    }

    let gain = super::flux_linkage::calculate_observer_gain(flux_linkage);
    Some(clamp_f32(gain, 1e3, 1e9))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "detection")]
    use crate::units::webers;
    use crate::units::{amps, hertz, ohms, volts};

    #[test]
    fn test_calculate_current_gains() {
        // R = 0.1Ω, L = 100µH, ω = 1000 rad/s
        let (kp, ki) = calculate_current_gains(ohms(0.1), henries(0.0001), rad_per_s(1000.0));

        // Kp = L × ω = 0.0001 × 1000 = 0.1
        assert!((kp - ohms(0.1)).abs() < ohms(1e-6));

        // Ki = R × ω = 0.1 × 1000 = 100
        assert!((ki.value - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_current_gains_limited() {
        // R = 0.1Ω, L = 100µH, target ω = 5000 rad/s
        // But with V_max = 24V and I_max = 50A
        let (kp, ki, actual_bw) = calculate_current_gains_limited(
            ohms(0.1),
            henries(0.0001),
            rad_per_s(5000.0),
            volts(24.0),
            amps(50.0),
        );

        // Max Kp = 24/50 = 0.48
        // Max ω from Kp = 0.48/0.0001 = 4800 rad/s
        // Should use limited bandwidth
        assert!(actual_bw <= rad_per_s(5000.0));
        assert!(kp <= ohms(0.5));
        assert!(ki.is_positive());
    }

    #[test]
    fn test_calculate_foc_gains() {
        let params = MotorParams {
            resistance: ohms(0.1),
            inductance_d: henries(0.0001),
            inductance_q: henries(0.00012),
            inductance_avg: henries(0.00011),
            ..Default::default()
        };

        let gains = calculate_foc_gains(&params, rad_per_s(1000.0)).unwrap();

        // d-axis: Kp = 0.0001 × 1000 = 0.1
        assert!((gains.kp_d - ohms(0.1)).abs() < ohms(1e-6));

        // q-axis: Kp = 0.00012 × 1000 = 0.12
        assert!((gains.kp_q - ohms(0.12)).abs() < ohms(1e-6));

        // Both axes same Ki = 0.1 × 1000 = 100
        assert!((gains.ki_d.value - 100.0).abs() < 1e-6);
        assert!((gains.ki_q.value - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_foc_gains_invalid() {
        let params = MotorParams::default();

        // Should return None for invalid parameters
        assert!(calculate_foc_gains(&params, rad_per_s(1000.0)).is_none());
    }

    #[test]
    fn test_estimate_bandwidth() {
        // Low inductance motor
        let bw = estimate_bandwidth(henries(50e-6), hertz(20000.0));
        assert!((MIN_BANDWIDTH..=MAX_BANDWIDTH).contains(&bw));

        // Should not exceed PWM limit
        let bw_limited = estimate_bandwidth(henries(10e-6), hertz(5000.0));
        let pwm_limit = rad_per_s(5000.0 * core::f32::consts::TAU / 10.0);
        assert!(bw_limited <= pwm_limit);
    }

    #[test]
    #[cfg(feature = "detection")]
    fn test_calculate_observer_gain() {
        // Must match the VESC detection-wizard formula gain = 0.5e3/λ²
        // (conf_general.c:1181) — the same one flux_linkage.rs implements.
        // λ = 10mWb => 0.5e3 / (0.01)² = 5e6
        let gain = calculate_observer_gain(webers(0.01)).unwrap();
        assert!((gain - 5e6).abs() < 1e3);

        // λ = 5mWb (typical hobby motor) => 0.5e3 / (0.005)² = 2e7
        let gain2 = calculate_observer_gain(webers(0.005)).unwrap();
        assert!((gain2 - 2e7).abs() < 1e4);

        // Both call sites must agree (one source of truth)
        let from_flux = super::super::flux_linkage::calculate_observer_gain(webers(0.005));
        assert!((gain2 - from_flux).abs() < 1.0);

        // Huge flux linkage clamps to the minimum
        let gain3 = calculate_observer_gain(webers(1.0)).unwrap();
        assert!((gain3 - 1e3).abs() < 1.0);

        // Invalid flux linkage
        assert!(calculate_observer_gain(webers(0.0)).is_none());
        assert!(calculate_observer_gain(webers(-1.0)).is_none());
    }
}
