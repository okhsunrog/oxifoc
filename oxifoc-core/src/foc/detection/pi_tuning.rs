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

use super::types::MotorParams;

/// Default current loop bandwidth in rad/s.
///
/// 1000 rad/s corresponds to ~160 Hz, suitable for most applications.
pub const DEFAULT_BANDWIDTH_RAD_S: f32 = 1000.0;

/// Maximum recommended bandwidth in rad/s.
///
/// Higher bandwidths may cause instability due to PWM delays and sampling.
pub const MAX_BANDWIDTH_RAD_S: f32 = 5000.0;

/// Minimum recommended bandwidth in rad/s.
pub const MIN_BANDWIDTH_RAD_S: f32 = 100.0;

/// Result of PI tuning calculation.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PiGains {
    /// Proportional gain for d-axis current controller
    pub kp_d: f32,

    /// Integral gain for d-axis current controller
    pub ki_d: f32,

    /// Proportional gain for q-axis current controller
    pub kp_q: f32,

    /// Integral gain for q-axis current controller
    pub ki_q: f32,

    /// Actual bandwidth achieved (rad/s)
    pub bandwidth_rad_s: f32,
}

/// Calculate PI gains for current control from motor parameters.
///
/// # Arguments
/// * `resistance` - Motor phase resistance in Ohms
/// * `inductance` - Motor inductance in Henries (use Ld for d-axis, Lq for q-axis)
/// * `bandwidth_rad_s` - Desired control bandwidth in rad/s
///
/// # Returns
/// Tuple of (Kp, Ki) gains
///
/// # Example
/// ```
/// use oxifoc_core::foc::detection::pi_tuning::calculate_current_gains;
///
/// // Motor with R=0.1Ω, L=100µH, 1000 rad/s bandwidth
/// let (kp, ki) = calculate_current_gains(0.1, 0.0001, 1000.0);
/// assert!((kp - 0.1).abs() < 0.001);  // Kp = L × ω = 0.0001 × 1000
/// assert!((ki - 100.0).abs() < 1.0);   // Ki = R × ω = 0.1 × 1000
/// ```
#[inline]
pub fn calculate_current_gains(
    resistance: f32,
    inductance: f32,
    bandwidth_rad_s: f32,
) -> (f32, f32) {
    let kp = inductance * bandwidth_rad_s;
    let ki = resistance * bandwidth_rad_s;
    (kp, ki)
}

/// Calculate PI gains with voltage limiting consideration.
///
/// If the calculated gains would produce excessive control voltages,
/// the bandwidth is reduced to stay within limits.
///
/// # Arguments
/// * `resistance` - Motor phase resistance in Ohms
/// * `inductance` - Motor inductance in Henries
/// * `target_bandwidth` - Desired bandwidth in rad/s
/// * `max_voltage` - Maximum available control voltage
/// * `max_current` - Maximum expected current step
///
/// # Returns
/// Tuple of (Kp, Ki, actual_bandwidth)
pub fn calculate_current_gains_limited(
    resistance: f32,
    inductance: f32,
    target_bandwidth: f32,
    max_voltage: f32,
    max_current: f32,
) -> (f32, f32, f32) {
    // Maximum voltage needed for step response: V = Kp × I_step
    // Limit Kp such that Kp × I_max <= V_max
    let max_kp = max_voltage / max_current;
    let max_bandwidth_from_kp = max_kp / inductance;

    // Use the more limiting bandwidth
    let actual_bandwidth = target_bandwidth.min(max_bandwidth_from_kp);
    let actual_bandwidth =
        crate::foc::clamp_f32(actual_bandwidth, MIN_BANDWIDTH_RAD_S, MAX_BANDWIDTH_RAD_S);

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
/// * `bandwidth_rad_s` - Desired control bandwidth in rad/s
///
/// # Returns
/// Complete PI gains for both axes, or None if parameters are invalid
pub fn calculate_foc_gains(params: &MotorParams, bandwidth_rad_s: f32) -> Option<PiGains> {
    if params.resistance_ohm <= 0.0 {
        return None;
    }

    let ld = if params.inductance_d_h > 0.0 {
        params.inductance_d_h
    } else if params.inductance_avg_h > 0.0 {
        params.inductance_avg_h
    } else {
        return None;
    };

    let lq = if params.inductance_q_h > 0.0 {
        params.inductance_q_h
    } else if params.inductance_avg_h > 0.0 {
        params.inductance_avg_h
    } else {
        return None;
    };

    let bandwidth =
        crate::foc::clamp_f32(bandwidth_rad_s, MIN_BANDWIDTH_RAD_S, MAX_BANDWIDTH_RAD_S);

    let (kp_d, ki_d) = calculate_current_gains(params.resistance_ohm, ld, bandwidth);
    let (kp_q, ki_q) = calculate_current_gains(params.resistance_ohm, lq, bandwidth);

    Some(PiGains {
        kp_d,
        ki_d,
        kp_q,
        ki_q,
        bandwidth_rad_s: bandwidth,
    })
}

/// Estimate suitable bandwidth from motor parameters.
///
/// Uses heuristics based on typical motor characteristics:
/// - Smaller inductance → can use higher bandwidth
/// - Accounts for PWM switching frequency limits
///
/// # Arguments
/// * `inductance_h` - Motor inductance in Henries
/// * `pwm_freq_hz` - PWM switching frequency in Hz
///
/// # Returns
/// Recommended bandwidth in rad/s
pub fn estimate_bandwidth(inductance_h: f32, pwm_freq_hz: f32) -> f32 {
    // Rule of thumb: bandwidth should be at most 1/10 of PWM frequency
    // to ensure adequate sampling and avoid aliasing
    let max_from_pwm = pwm_freq_hz * core::f32::consts::TAU / 10.0;

    // Also limit based on inductance - very low inductance motors
    // may have too fast dynamics for stable control
    let suggested: f32 = if inductance_h < 10e-6 {
        // Very low inductance (< 10µH): be conservative
        500.0
    } else if inductance_h < 100e-6 {
        // Typical small motor (10-100µH)
        1000.0
    } else if inductance_h < 1e-3 {
        // Medium motor (100µH - 1mH)
        2000.0
    } else {
        // Large motor (> 1mH)
        3000.0
    };

    crate::foc::clamp_f32(
        suggested.min(max_from_pwm),
        MIN_BANDWIDTH_RAD_S,
        MAX_BANDWIDTH_RAD_S,
    )
}

/// Calculate observer gain from motor parameters.
///
/// The observer gain is used for sensorless FOC to estimate rotor position.
/// Delegates to the single VESC formula (`gain = 0.5e3 / λ²`, the value the
/// VESC detection wizard stores: conf_general.c:1181) so there is exactly one
/// source of truth, and clamps the result to a sane range.
///
/// # Arguments
/// * `flux_linkage_wb` - Motor flux linkage in Weber
///
/// # Returns
/// Observer gain, or None if flux linkage is invalid
#[cfg(feature = "detection")]
pub fn calculate_observer_gain(flux_linkage_wb: f32) -> Option<f32> {
    if flux_linkage_wb <= 0.0 {
        return None;
    }

    let gain = super::flux_linkage::calculate_observer_gain(flux_linkage_wb);
    Some(crate::foc::clamp_f32(gain, 1e3, 1e9))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_current_gains() {
        // R = 0.1Ω, L = 100µH, ω = 1000 rad/s
        let (kp, ki) = calculate_current_gains(0.1, 0.0001, 1000.0);

        // Kp = L × ω = 0.0001 × 1000 = 0.1
        assert!((kp - 0.1).abs() < 1e-6);

        // Ki = R × ω = 0.1 × 1000 = 100
        assert!((ki - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_current_gains_limited() {
        // R = 0.1Ω, L = 100µH, target ω = 5000 rad/s
        // But with V_max = 24V and I_max = 50A
        let (kp, ki, actual_bw) = calculate_current_gains_limited(0.1, 0.0001, 5000.0, 24.0, 50.0);

        // Max Kp = 24/50 = 0.48
        // Max ω from Kp = 0.48/0.0001 = 4800 rad/s
        // Should use limited bandwidth
        assert!(actual_bw <= 5000.0);
        assert!(kp <= 0.5);
        assert!(ki > 0.0);
    }

    #[test]
    fn test_calculate_foc_gains() {
        let params = MotorParams {
            resistance_ohm: 0.1,
            inductance_d_h: 0.0001,
            inductance_q_h: 0.00012,
            inductance_avg_h: 0.00011,
            ..Default::default()
        };

        let gains = calculate_foc_gains(&params, 1000.0).unwrap();

        // d-axis: Kp = 0.0001 × 1000 = 0.1
        assert!((gains.kp_d - 0.1).abs() < 1e-6);

        // q-axis: Kp = 0.00012 × 1000 = 0.12
        assert!((gains.kp_q - 0.12).abs() < 1e-6);

        // Both axes same Ki = 0.1 × 1000 = 100
        assert!((gains.ki_d - 100.0).abs() < 1e-6);
        assert!((gains.ki_q - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_calculate_foc_gains_invalid() {
        let params = MotorParams::default();

        // Should return None for invalid parameters
        assert!(calculate_foc_gains(&params, 1000.0).is_none());
    }

    #[test]
    fn test_estimate_bandwidth() {
        // Low inductance motor
        let bw = estimate_bandwidth(50e-6, 20000.0);
        assert!((MIN_BANDWIDTH_RAD_S..=MAX_BANDWIDTH_RAD_S).contains(&bw));

        // Should not exceed PWM limit
        let bw_limited = estimate_bandwidth(10e-6, 5000.0);
        let pwm_limit = 5000.0 * core::f32::consts::TAU / 10.0;
        assert!(bw_limited <= pwm_limit);
    }

    #[test]
    #[cfg(feature = "detection")]
    fn test_calculate_observer_gain() {
        // Must match the VESC detection-wizard formula gain = 0.5e3/λ²
        // (conf_general.c:1181) — the same one flux_linkage.rs implements.
        // λ = 10mWb => 0.5e3 / (0.01)² = 5e6
        let gain = calculate_observer_gain(0.01).unwrap();
        assert!((gain - 5e6).abs() < 1e3);

        // λ = 5mWb (typical hobby motor) => 0.5e3 / (0.005)² = 2e7
        let gain2 = calculate_observer_gain(0.005).unwrap();
        assert!((gain2 - 2e7).abs() < 1e4);

        // Both call sites must agree (one source of truth)
        let from_flux = crate::foc::detection::flux_linkage::calculate_observer_gain(0.005);
        assert!((gain2 - from_flux).abs() < 1.0);

        // Huge flux linkage clamps to the minimum
        let gain3 = calculate_observer_gain(1.0).unwrap();
        assert!((gain3 - 1e3).abs() < 1.0);

        // Invalid flux linkage
        assert!(calculate_observer_gain(0.0).is_none());
        assert!(calculate_observer_gain(-1.0).is_none());
    }
}
