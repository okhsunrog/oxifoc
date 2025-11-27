//! Park and Clarke transformations for Field-Oriented Control
//!
//! These transforms convert between different reference frames:
//! - Clarke: ABC (3-phase) → αβ (2-phase stationary)
//! - Park: αβ (stationary) → dq (rotating with rotor)
//! - Inverse Park: dq → αβ
//! - Inverse Clarke: αβ → ABC
//!
//! Based on Microsemi implementation guide and foc-calebfletcher reference.

use super::constants::{FRAC_1_SQRT_3, SQRT_3};

/// Clarke transform: Convert 3-phase currents (ABC) to 2-phase stationary frame (αβ)
///
/// Input: Two phase currents (third is calculated from ia + ib + ic = 0)
/// Output: (alpha, beta) in stationary reference frame
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::transforms::clarke;
///
/// let (alpha, beta) = clarke(1.0, 0.5);
/// assert!(alpha > 0.0 && beta > 0.0);
/// ```
#[inline]
pub fn clarke(ia: f32, ib: f32) -> (f32, f32) {
    // Alpha axis aligned with phase A
    let alpha = ia;

    // Beta axis 90° ahead of alpha
    // β = (ia + 2*ib) / √3
    let beta = FRAC_1_SQRT_3 * (ia + 2.0 * ib);

    (alpha, beta)
}

/// Inverse Clarke transform: Convert 2-phase (αβ) back to 3-phase (ABC)
///
/// Input: (alpha, beta) stationary frame voltages/currents
/// Output: (a, b, c) three-phase values
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::transforms::inverse_clarke;
///
/// let (va, vb, vc) = inverse_clarke(0.8, 0.2);
/// assert!((va + vb + vc).abs() < 1e-6);
/// ```
#[inline]
pub fn inverse_clarke(alpha: f32, beta: f32) -> (f32, f32, f32) {
    // Phase A aligned with alpha
    let a = alpha;

    // Phase B at 120° (-0.5α + √3/2 β)
    let b = (-alpha + SQRT_3 * beta) / 2.0;

    // Phase C at 240° (-0.5α - √3/2 β)
    let c = (-alpha - SQRT_3 * beta) / 2.0;

    (a, b, c)
}

/// Park transform: Convert stationary frame (αβ) to rotating frame (dq)
///
/// The dq frame rotates with the rotor's electrical angle.
/// - d-axis: aligned with rotor flux (field)
/// - q-axis: 90° ahead of d-axis (torque)
///
/// Input: (alpha, beta, sin_theta, cos_theta)
/// Output: (id, iq) in rotating reference frame
///
/// # Arguments
/// * `alpha` - Alpha axis current/voltage
/// * `beta` - Beta axis current/voltage
/// * `sin_theta` - sin(electrical angle)
/// * `cos_theta` - cos(electrical angle)
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::transforms::park;
///
/// let i_alpha = 1.0;
/// let i_beta = 0.0;
/// let sin_theta = 0.0;
/// let cos_theta = 1.0;
///
/// let (id, iq) = park(i_alpha, i_beta, sin_theta, cos_theta);
/// assert_eq!(id, i_alpha);
/// assert_eq!(iq, i_beta);
/// ```
#[inline]
pub fn park(alpha: f32, beta: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    // d-axis: component aligned with rotor
    // d = α*cos(θ) + β*sin(θ)
    let d = cos_theta * alpha + sin_theta * beta;

    // q-axis: component 90° ahead (torque-producing)
    // q = β*cos(θ) - α*sin(θ)
    let q = cos_theta * beta - sin_theta * alpha;

    (d, q)
}

/// Inverse Park transform: Convert rotating frame (dq) back to stationary frame (αβ)
///
/// Input: (d, q, sin_theta, cos_theta)
/// Output: (alpha, beta) in stationary frame
///
/// # Arguments
/// * `d` - d-axis voltage (field component)
/// * `q` - q-axis voltage (torque component)
/// * `sin_theta` - sin(electrical angle)
/// * `cos_theta` - cos(electrical angle)
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::transforms::inverse_park;
///
/// let vd = 1.2;
/// let vq = -0.3;
/// let sin_theta = 0.0;
/// let cos_theta = 1.0;
///
/// let (v_alpha, v_beta) = inverse_park(vd, vq, sin_theta, cos_theta);
/// assert_eq!(v_alpha, vd);
/// assert_eq!(v_beta, vq);
/// ```
#[inline]
pub fn inverse_park(d: f32, q: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    // α = d*cos(θ) - q*sin(θ)
    let alpha = cos_theta * d - sin_theta * q;

    // β = d*sin(θ) + q*cos(θ)
    let beta = sin_theta * d + cos_theta * q;

    (alpha, beta)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.001;

    #[test]
    fn test_clarke_zero() {
        let (alpha, beta) = clarke(0.0, 0.0);
        assert!((alpha - 0.0).abs() < EPSILON);
        assert!((beta - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_clarke_roundtrip() {
        let test_cases = [
            (1.0, 0.0),
            (0.0, 1.0),
            (1.0, 0.5),
            (-0.5, -0.5),
            (2.5, -1.3),
        ];

        for (ia, ib) in test_cases {
            let (alpha, beta) = clarke(ia, ib);
            let (a, b, _c) = inverse_clarke(alpha, beta);

            assert!(
                (a - ia).abs() < EPSILON,
                "Clarke roundtrip failed for ia={}: got a={}",
                ia,
                a
            );
            assert!(
                (b - ib).abs() < EPSILON,
                "Clarke roundtrip failed for ib={}: got b={}",
                ib,
                b
            );
        }
    }

    #[test]
    fn test_clarke_inverse_sum_zero() {
        // For a balanced 3-phase system: a + b + c = 0
        let (alpha, beta) = clarke(1.0, -0.5);
        let (a, b, c) = inverse_clarke(alpha, beta);
        let sum = a + b + c;

        assert!(
            sum.abs() < EPSILON,
            "3-phase sum should be zero, got {}",
            sum
        );
    }

    #[test]
    fn test_park_zero_angle() {
        // At θ=0, dq should equal αβ
        let alpha = 1.5;
        let beta = 0.8;
        let (d, q) = park(alpha, beta, 0.0, 1.0); // sin(0)=0, cos(0)=1

        assert!((d - alpha).abs() < EPSILON);
        assert!((q - beta).abs() < EPSILON);
    }

    #[test]
    fn test_park_roundtrip() {
        let test_angles = [
            0.0,
            0.5,
            1.0,
            core::f32::consts::PI / 4.0,
            core::f32::consts::PI,
        ];
        let test_values = [(1.0, 0.0), (0.0, 1.0), (1.5, -0.8), (2.3, 1.7)];

        for theta in test_angles {
            let sin_theta = libm::sinf(theta);
            let cos_theta = libm::cosf(theta);

            for (alpha, beta) in test_values {
                let (d, q) = park(alpha, beta, sin_theta, cos_theta);
                let (alpha_result, beta_result) = inverse_park(d, q, sin_theta, cos_theta);

                assert!(
                    (alpha_result - alpha).abs() < EPSILON,
                    "Park roundtrip failed for alpha={} at θ={}: got {}",
                    alpha,
                    theta,
                    alpha_result
                );
                assert!(
                    (beta_result - beta).abs() < EPSILON,
                    "Park roundtrip failed for beta={} at θ={}: got {}",
                    beta,
                    theta,
                    beta_result
                );
            }
        }
    }

    #[test]
    fn test_full_transform_chain() {
        // Test complete chain: ABC → αβ → dq → αβ → ABC
        let ia = 1.5;
        let ib = -0.8;
        let theta = 0.7;

        // Forward transforms
        let (alpha, beta) = clarke(ia, ib);
        let sin_theta = libm::sinf(theta);
        let cos_theta = libm::cosf(theta);
        let (d, q) = park(alpha, beta, sin_theta, cos_theta);

        // Inverse transforms
        let (alpha2, beta2) = inverse_park(d, q, sin_theta, cos_theta);
        let (a, b, _c) = inverse_clarke(alpha2, beta2);

        assert!(
            (a - ia).abs() < EPSILON,
            "Full chain roundtrip failed for ia"
        );
        assert!(
            (b - ib).abs() < EPSILON,
            "Full chain roundtrip failed for ib"
        );
    }

    #[test]
    fn test_park_magnitude_preservation() {
        // Park transform should preserve magnitude (√(α²+β²) = √(d²+q²))
        let alpha = 1.5;
        let beta = 2.3;
        let theta = 1.2;

        let sin_theta = libm::sinf(theta);
        let cos_theta = libm::cosf(theta);
        let (d, q) = park(alpha, beta, sin_theta, cos_theta);

        let mag_ab = libm::sqrtf(alpha * alpha + beta * beta);
        let mag_dq = libm::sqrtf(d * d + q * q);

        assert!(
            (mag_ab - mag_dq).abs() < EPSILON,
            "Park transform should preserve magnitude"
        );
    }
}
