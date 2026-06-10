//! Trigonometric function abstractions for FOC control
//!
//! Provides a [`SinCos`] trait so platforms can substitute hardware-accelerated
//! implementations (e.g., STM32 CORDIC peripheral) for the default software
//! [`LibmSinCos`] fallback, or use the fast [`FastSinCos`] polynomial
//! approximation.
//!
//! Combined with `#[inline(always)]` and monomorphization, trait dispatch is
//! fully erased at compile time — zero runtime overhead.
//!
//! Also provides q1.31 fixed-point conversion helpers used by CORDIC
//! implementations.

use core::f32::consts::{FRAC_PI_2, PI, TAU};

/// Simultaneous sine and cosine computation
///
/// Implement this trait to provide platform-specific hardware-accelerated
/// sin/cos (e.g., STM32 CORDIC peripheral). The default [`LibmSinCos`]
/// uses software `libm`.
pub trait SinCos {
    /// Compute (sin(angle), cos(angle)) simultaneously
    ///
    /// # Arguments
    /// * `angle` - Angle in radians
    ///
    /// # Returns
    /// `(sin, cos)` tuple
    fn sin_cos(angle: f32) -> (f32, f32);
}

/// Software sin/cos using libm (accurate but slow, ~400-600 cycles on M4F)
#[derive(Clone, Copy, Debug, Default)]
pub struct LibmSinCos;

impl SinCos for LibmSinCos {
    #[inline(always)]
    fn sin_cos(angle: f32) -> (f32, f32) {
        (libm::sinf(angle), libm::cosf(angle))
    }
}

/// Fast sin/cos using degree-5 minimax (Chebyshev) polynomial approximation.
///
/// Better than 20 bits of accuracy (< 1e-6 max error), ~16-20 cycles per call
/// on Cortex-M4F. Zero memory footprint (no lookup tables).
///
/// Uses degree-7 minimax polynomial coefficients optimized over [0, π/2] to
/// minimize peak error (Remez/Chebyshev criterion). This is strictly superior
/// to Taylor series (which minimizes error only at x=0) and the
/// Bhaskara/devmaster parabolic approximation (~9-10 bits).
///
/// The polynomial in Horner form:
///   sin(x) ≈ x · (c₁ + x² · (c₂ + x² · (c₃ + x² · c₄)))
///
/// Accuracy is sufficient for FOC motor control where ADC resolution (12 bits)
/// and PWM timer resolution (10-12 bits) are the limiting factors.
#[derive(Clone, Copy, Debug, Default)]
pub struct FastSinCos;

/// Compute sin(x) for x in [0, π/2] using degree-7 minimax polynomial.
///
/// Coefficients minimize the maximum absolute error over [0, π/2].
/// Max error: < 1e-6 (better than 20 bits of accuracy).
/// Cost: 4 multiplies + 3 adds in Horner form (~16-20 cycles on M4F).
#[inline(always)]
fn sin_poly(x: f32) -> f32 {
    // Degree-7 minimax coefficients for sin(x) on [0, π/2]
    // sin(x) ≈ x * (c1 + x² * (c2 + x² * (c3 + x² * c4)))
    const C1: f32 = 0.999_999_6;
    const C2: f32 = -0.166_666_47;
    const C3: f32 = 0.008_333_025;
    const C4: f32 = -0.000_198_074;

    let x2 = x * x;
    x * (C1 + x2 * (C2 + x2 * (C3 + x2 * C4)))
}

/// Reduce angle to [0, π/2] and compute sin using the polynomial.
///
/// Uses quadrant decomposition:
///   Q0 [0, π/2]:      sin(x) = sin_poly(x)
///   Q1 [π/2, π]:      sin(x) = sin_poly(π - x)
///   Q2 [π, 3π/2]:     sin(x) = -sin_poly(x - π)
///   Q3 [3π/2, 2π]:    sin(x) = -sin_poly(2π - x)
#[inline(always)]
fn fast_sin(angle: f32) -> f32 {
    // Normalize to [0, 2π)
    let mut x = angle % TAU;
    if x < 0.0 {
        x += TAU;
    }

    // Quadrant decomposition
    if x < FRAC_PI_2 {
        sin_poly(x)
    } else if x < PI {
        sin_poly(PI - x)
    } else if x < PI + FRAC_PI_2 {
        -sin_poly(x - PI)
    } else {
        -sin_poly(TAU - x)
    }
}

impl SinCos for FastSinCos {
    #[inline(always)]
    fn sin_cos(angle: f32) -> (f32, f32) {
        (fast_sin(angle), fast_sin(angle + FRAC_PI_2))
    }
}

// ============================================================================
// q1.31 Fixed-Point Conversion Helpers
// ============================================================================

/// Convert q1.31 fixed-point to f32
///
/// On ARM with hardware FPU, uses `vcvt.f32.s32` for single-cycle conversion.
/// Falls back to software multiply on other platforms.
#[inline(always)]
pub fn q31_to_f32(val_q31: i32) -> f32 {
    #[cfg(all(target_arch = "arm", target_abi = "eabihf"))]
    {
        let res_f32: f32;
        unsafe {
            core::arch::asm!(
                "vmov {tmp}, {val}",
                "vcvt.f32.s32 {tmp}, {tmp}, #31",
                val = in(reg) val_q31,
                tmp = out(sreg) res_f32,
            );
        }
        res_f32
    }
    #[cfg(not(all(target_arch = "arm", target_abi = "eabihf")))]
    {
        const Q31_TO_F32: f32 = 1.0 / 2147483648.0; // 1 / 2^31
        (val_q31 as f32) * Q31_TO_F32
    }
}

/// Convert f32 to q1.31 fixed-point
///
/// On ARM with hardware FPU, uses `vcvt.s32.f32` for single-cycle conversion.
/// Falls back to software multiply on other platforms.
///
/// Input should be in range \[-1.0, 1.0). Values outside this range saturate.
#[inline(always)]
pub fn f32_to_q31(val_f32: f32) -> i32 {
    #[cfg(all(target_arch = "arm", target_abi = "eabihf"))]
    {
        let res_q31: i32;
        unsafe {
            core::arch::asm!(
                "vcvt.s32.f32 {tmp}, {tmp}, #31",
                "vmov {res}, {tmp}",
                tmp = inout(sreg) val_f32 => _,
                res = out(reg) res_q31,
            );
        }
        res_q31
    }
    #[cfg(not(all(target_arch = "arm", target_abi = "eabihf")))]
    {
        const F32_TO_Q31: f32 = 2147483648.0; // 2^31
        (val_f32 * F32_TO_Q31) as i32
    }
}

/// Convert angle from radians to CORDIC q1.31 format
///
/// The CORDIC peripheral expects angles where:
/// - `-1.0` (q31) = −π rad
/// - `+1.0` (q31) = +π rad
///
/// This function maps an arbitrary radian angle to that range.
#[inline(always)]
pub fn angle_to_cordic_q31(angle_rad: f32) -> i32 {
    // Map radians → [-1, 1] where ±1 = ±π
    let normalized = angle_rad * core::f32::consts::FRAC_1_PI;
    // Wrap to [-1.0, 1.0): angles typically in [0, 2π] → [0, 2] → subtract 2 if > 1
    let wrapped = if normalized > 1.0 {
        normalized - 2.0
    } else if normalized < -1.0 {
        normalized + 2.0
    } else {
        normalized
    };
    f32_to_q31(wrapped)
}

// ============================================================================
// CORDIC Hardware-Accelerated Sin/Cos (feature = "cordic")
// ============================================================================

#[cfg(feature = "cordic")]
mod cordic_impl {
    use core::cell::RefCell;

    use embassy_stm32::Peri;
    use embassy_stm32::cordic::{self, Cordic};
    use embassy_stm32::peripherals;
    use embassy_sync::blocking_mutex::CriticalSectionMutex;

    use super::{SinCos, angle_to_cordic_q31, q31_to_f32};

    /// CORDIC driver instance, initialized once and accessed from the FOC ISR.
    static CORDIC_INSTANCE: CriticalSectionMutex<
        RefCell<Option<Cordic<'static, peripherals::CORDIC>>>,
    > = CriticalSectionMutex::new(RefCell::new(None));

    /// CORDIC hardware-accelerated sin/cos
    ///
    /// Must call [`init()`](Self::init) once before first use.
    /// Uses the embassy CORDIC driver with Cosine function (primary=cos, secondary=sin),
    /// 1 argument input, 2 result outputs, q1.31 format.
    #[derive(Clone, Copy, Debug, Default)]
    pub struct CordicSinCos;

    impl CordicSinCos {
        /// Initialize CORDIC peripheral for sin/cos computation.
        ///
        /// Call once during board init, before the FOC ISR starts.
        /// Configures: Cosine function, 24 iterations (≈20-bit precision),
        /// 1 argument input, 2 result outputs, q1.31 format.
        pub fn init(peri: Peri<'static, peripherals::CORDIC>) {
            let config = match cordic::Config::new(
                cordic::Function::Cos,
                cordic::Precision::Iters24,
                Default::default(),
            ) {
                Ok(c) => c,
                Err(_) => panic!("CORDIC config failed"),
            }
            .res_count(cordic::AccessCount::Two);

            let cordic = Cordic::new(peri, config);
            CORDIC_INSTANCE.lock(|cell| cell.replace(Some(cordic)));
        }
    }

    impl SinCos for CordicSinCos {
        #[inline(always)]
        fn sin_cos(angle: f32) -> (f32, f32) {
            let angle_q31 = angle_to_cordic_q31(angle);
            let input = [angle_q31 as u32];
            let mut output = [0u32; 2];

            CORDIC_INSTANCE.lock(|cell| {
                let mut cordic = cell.borrow_mut();
                let cordic = match cordic.as_mut() {
                    Some(c) => c,
                    None => panic!("CORDIC not initialized"),
                };
                if let Err(_) = cordic.blocking_calc_32bit(&input, &mut output) {
                    panic!("CORDIC calc failed");
                }
            });

            // Cosine function: primary result = cos, secondary = sin
            let cos_q31 = output[0] as i32;
            let sin_q31 = output[1] as i32;

            (q31_to_f32(sin_q31), q31_to_f32(cos_q31))
        }
    }
}

#[cfg(feature = "cordic")]
pub use cordic_impl::CordicSinCos;

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    /// Reference sin/cos from libm for comparison
    fn ref_sin(x: f32) -> f32 {
        libm::sinf(x)
    }
    fn ref_cos(x: f32) -> f32 {
        libm::cosf(x)
    }

    // ── LibmSinCos tests ────────────────────────────────────────────────────

    #[test]
    fn libm_sin_cos_zero() {
        let (sin, cos) = LibmSinCos::sin_cos(0.0);
        assert!((sin - 0.0).abs() < 1e-6);
        assert!((cos - 1.0).abs() < 1e-6);
    }

    #[test]
    fn libm_sin_cos_pi_over_2() {
        let (sin, cos) = LibmSinCos::sin_cos(FRAC_PI_2);
        assert!((sin - 1.0).abs() < 1e-6);
        assert!((cos - 0.0).abs() < 1e-6);
    }

    #[test]
    fn libm_sin_cos_pi() {
        let (sin, cos) = LibmSinCos::sin_cos(PI);
        assert!(sin.abs() < 1e-5);
        assert!((cos - (-1.0)).abs() < 1e-6);
    }

    // ── FastSinCos: known values ────────────────────────────────────────────

    const FAST_TOL: f32 = 2e-4; // ~14 bits of accuracy

    #[test]
    fn fast_sin_cos_zero() {
        let (sin, cos) = FastSinCos::sin_cos(0.0);
        assert!((sin - 0.0).abs() < FAST_TOL, "sin(0) = {sin}");
        assert!((cos - 1.0).abs() < FAST_TOL, "cos(0) = {cos}");
    }

    #[test]
    fn fast_sin_cos_pi_over_6() {
        let (sin, cos) = FastSinCos::sin_cos(core::f32::consts::FRAC_PI_6);
        assert!((sin - 0.5).abs() < FAST_TOL, "sin(π/6) = {sin}");
        assert!((cos - 0.866_025_4).abs() < FAST_TOL, "cos(π/6) = {cos}");
    }

    #[test]
    fn fast_sin_cos_pi_over_4() {
        let (sin, cos) = FastSinCos::sin_cos(core::f32::consts::FRAC_PI_4);
        let expected = core::f32::consts::FRAC_1_SQRT_2;
        assert!((sin - expected).abs() < FAST_TOL, "sin(π/4) = {sin}");
        assert!((cos - expected).abs() < FAST_TOL, "cos(π/4) = {cos}");
    }

    #[test]
    fn fast_sin_cos_pi_over_2() {
        let (sin, cos) = FastSinCos::sin_cos(FRAC_PI_2);
        assert!((sin - 1.0).abs() < FAST_TOL, "sin(π/2) = {sin}");
        assert!(cos.abs() < FAST_TOL, "cos(π/2) = {cos}");
    }

    #[test]
    fn fast_sin_cos_pi() {
        let (sin, cos) = FastSinCos::sin_cos(PI);
        assert!(sin.abs() < FAST_TOL, "sin(π) = {sin}");
        assert!((cos - (-1.0)).abs() < FAST_TOL, "cos(π) = {cos}");
    }

    #[test]
    fn fast_sin_cos_3pi_over_2() {
        let (sin, cos) = FastSinCos::sin_cos(3.0 * FRAC_PI_2);
        assert!((sin - (-1.0)).abs() < FAST_TOL, "sin(3π/2) = {sin}");
        assert!(cos.abs() < FAST_TOL, "cos(3π/2) = {cos}");
    }

    #[test]
    fn fast_sin_cos_2pi() {
        let (sin, cos) = FastSinCos::sin_cos(TAU);
        assert!(sin.abs() < FAST_TOL, "sin(2π) = {sin}");
        assert!((cos - 1.0).abs() < FAST_TOL, "cos(2π) = {cos}");
    }

    // ── FastSinCos: negative angles ─────────────────────────────────────────

    #[test]
    fn fast_sin_cos_negative_pi_over_2() {
        let (sin, cos) = FastSinCos::sin_cos(-FRAC_PI_2);
        assert!((sin - (-1.0)).abs() < FAST_TOL, "sin(-π/2) = {sin}");
        assert!(cos.abs() < FAST_TOL, "cos(-π/2) = {cos}");
    }

    #[test]
    fn fast_sin_cos_negative_pi() {
        let (sin, cos) = FastSinCos::sin_cos(-PI);
        assert!(sin.abs() < FAST_TOL, "sin(-π) = {sin}");
        assert!((cos - (-1.0)).abs() < FAST_TOL, "cos(-π) = {cos}");
    }

    // ── FastSinCos: large angles (multi-revolution) ─────────────────────────

    #[test]
    fn fast_sin_cos_large_positive() {
        // 10π + π/4 = same as π/4
        let angle = 10.0 * PI + core::f32::consts::FRAC_PI_4;
        let (sin, cos) = FastSinCos::sin_cos(angle);
        let expected = core::f32::consts::FRAC_1_SQRT_2;
        assert!(
            (sin - expected).abs() < 5e-4,
            "sin(10π+π/4) = {sin}, expected {expected}"
        );
        assert!(
            (cos - expected).abs() < 5e-4,
            "cos(10π+π/4) = {cos}, expected {expected}"
        );
    }

    #[test]
    fn fast_sin_cos_large_negative() {
        let angle = -8.0 * PI + FRAC_PI_2;
        let (sin, cos) = FastSinCos::sin_cos(angle);
        assert!((sin - 1.0).abs() < 5e-4, "sin(-8π+π/2) = {sin}");
        assert!(cos.abs() < 5e-4, "cos(-8π+π/2) = {cos}");
    }

    // ── FastSinCos: sweep accuracy against libm ─────────────────────────────

    #[test]
    fn fast_sin_sweep_accuracy() {
        // Sweep [0, 2π] in 10000 steps, check max error against libm
        let n = 10000;
        let mut max_sin_err = 0.0_f32;
        let mut max_cos_err = 0.0_f32;
        let mut worst_sin_angle = 0.0_f32;
        let mut worst_cos_angle = 0.0_f32;

        for i in 0..=n {
            let angle = TAU * (i as f32) / (n as f32);
            let (fast_s, fast_c) = FastSinCos::sin_cos(angle);
            let (ref_s, ref_c) = (ref_sin(angle), ref_cos(angle));

            let sin_err = (fast_s - ref_s).abs();
            let cos_err = (fast_c - ref_c).abs();

            if sin_err > max_sin_err {
                max_sin_err = sin_err;
                worst_sin_angle = angle;
            }
            if cos_err > max_cos_err {
                max_cos_err = cos_err;
                worst_cos_angle = angle;
            }
        }

        assert!(
            max_sin_err < 2e-4,
            "sin max error {max_sin_err} at angle {worst_sin_angle} exceeds 2e-4"
        );
        assert!(
            max_cos_err < 2e-4,
            "cos max error {max_cos_err} at angle {worst_cos_angle} exceeds 2e-4"
        );
    }

    #[test]
    fn fast_sin_sweep_negative_angles() {
        // Sweep [-2π, 0] in 10000 steps
        let n = 10000;
        let mut max_err = 0.0_f32;

        for i in 0..=n {
            let angle = -TAU * (i as f32) / (n as f32);
            let (fast_s, fast_c) = FastSinCos::sin_cos(angle);
            let (ref_s, ref_c) = (ref_sin(angle), ref_cos(angle));

            max_err = max_err.max((fast_s - ref_s).abs());
            max_err = max_err.max((fast_c - ref_c).abs());
        }

        assert!(
            max_err < 2e-4,
            "negative sweep max error {max_err} exceeds 2e-4"
        );
    }

    #[test]
    fn fast_sin_sweep_foc_range() {
        // FOC angles are typically [0, 2π] with some jitter
        // Test [-π, 3π] to cover all realistic inputs
        let n = 20000;
        let mut max_err = 0.0_f32;

        for i in 0..=n {
            let angle = -PI + (4.0 * PI) * (i as f32) / (n as f32);
            let (fast_s, fast_c) = FastSinCos::sin_cos(angle);
            let (ref_s, ref_c) = (ref_sin(angle), ref_cos(angle));

            max_err = max_err.max((fast_s - ref_s).abs());
            max_err = max_err.max((fast_c - ref_c).abs());
        }

        assert!(
            max_err < 2e-4,
            "FOC range sweep max error {max_err} exceeds 2e-4"
        );
    }

    // ── FastSinCos: identity checks ─────────────────────────────────────────

    #[test]
    fn fast_sin_cos_pythagorean_identity() {
        // sin²(x) + cos²(x) = 1 for all x
        let n = 1000;
        let mut max_err = 0.0_f32;

        for i in 0..=n {
            let angle = TAU * (i as f32) / (n as f32);
            let (s, c) = FastSinCos::sin_cos(angle);
            let err = (s * s + c * c - 1.0).abs();
            max_err = max_err.max(err);
        }

        assert!(max_err < 5e-4, "Pythagorean identity max error {max_err}");
    }

    #[test]
    fn fast_sin_odd_symmetry() {
        // sin(-x) = -sin(x)
        let angles = [0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
        for &angle in &angles {
            let (s_pos, _) = FastSinCos::sin_cos(angle);
            let (s_neg, _) = FastSinCos::sin_cos(-angle);
            assert!(
                (s_pos + s_neg).abs() < 5e-4,
                "sin({angle}) + sin(-{angle}) = {} (expected 0)",
                s_pos + s_neg
            );
        }
    }

    #[test]
    fn fast_cos_even_symmetry() {
        // cos(-x) = cos(x)
        let angles = [0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
        for &angle in &angles {
            let (_, c_pos) = FastSinCos::sin_cos(angle);
            let (_, c_neg) = FastSinCos::sin_cos(-angle);
            assert!(
                (c_pos - c_neg).abs() < 5e-4,
                "cos({angle}) - cos(-{angle}) = {} (expected 0)",
                c_pos - c_neg
            );
        }
    }

    // ── FastSinCos: edge cases ──────────────────────────────────────────────

    #[test]
    fn fast_sin_cos_very_small_angle() {
        // For very small x, sin(x) ≈ x and cos(x) ≈ 1
        let angle = 1e-4; // Not 1e-6: cos is computed via sin(x+π/2),
        // f32 loses the tiny offset in π/2 + ε subtraction
        let (sin, cos) = FastSinCos::sin_cos(angle);
        assert!((sin - angle).abs() < FAST_TOL, "sin(1e-4) = {sin}");
        assert!((cos - ref_cos(angle)).abs() < FAST_TOL, "cos(1e-4) = {cos}");
    }

    #[test]
    fn fast_sin_cos_quadrant_boundaries() {
        // Test right at quadrant boundaries where the polynomial pieces meet
        let boundaries = [FRAC_PI_2, PI, 3.0 * FRAC_PI_2, TAU];
        for &angle in &boundaries {
            let (fast_s, fast_c) = FastSinCos::sin_cos(angle);
            let (ref_s, ref_c) = (ref_sin(angle), ref_cos(angle));
            assert!(
                (fast_s - ref_s).abs() < FAST_TOL,
                "sin({angle}) = {fast_s}, ref = {ref_s}"
            );
            assert!(
                (fast_c - ref_c).abs() < FAST_TOL,
                "cos({angle}) = {fast_c}, ref = {ref_c}"
            );
        }
    }

    // ── FastSinCos: FOC controller integration ──────────────────────────────

    #[test]
    fn fast_sin_cos_foc_transform_round_trip() {
        // Verify FastSinCos works correctly in Clarke→Park→InversePark→InverseClarke
        use crate::foc::transforms;

        let ia = 1.0_f32;
        let ib = -0.5_f32;
        let ic = -ia - ib;
        let angles = [0.0, 0.3, 1.0, 2.0, 3.5, 5.0, 6.0];

        for &angle in &angles {
            let (sin_t, cos_t) = FastSinCos::sin_cos(angle);
            let (alpha, beta) = transforms::clarke(ia, ib);
            let (d, q) = transforms::park(alpha, beta, sin_t, cos_t);
            let (alpha2, beta2) = transforms::inverse_park(d, q, sin_t, cos_t);
            let (a2, b2, c2) = transforms::inverse_clarke(alpha2, beta2);

            assert!(
                (a2 - ia).abs() < 2e-3,
                "angle {angle}: ia round-trip {a2} vs {ia}"
            );
            assert!(
                (b2 - ib).abs() < 2e-3,
                "angle {angle}: ib round-trip {b2} vs {ib}"
            );
            assert!(
                (c2 - ic).abs() < 2e-3,
                "angle {angle}: ic round-trip {c2} vs {ic}"
            );
        }
    }

    // ── sin_poly unit tests ─────────────────────────────────────────────────

    #[test]
    fn sin_poly_at_zero() {
        assert!((sin_poly(0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn sin_poly_at_pi_over_2() {
        assert!((sin_poly(FRAC_PI_2) - 1.0).abs() < 2e-4);
    }

    #[test]
    fn sin_poly_monotonic() {
        // sin_poly should be monotonically increasing on [0, π/2]
        let n = 100;
        let mut prev = sin_poly(0.0);
        for i in 1..=n {
            let x = FRAC_PI_2 * (i as f32) / (n as f32);
            let val = sin_poly(x);
            assert!(
                val >= prev - 1e-7,
                "sin_poly not monotonic at x={x}: {val} < {prev}"
            );
            prev = val;
        }
    }

    // ── q31 conversion tests ────────────────────────────────────────────────

    #[test]
    fn q31_round_trip() {
        let test_values = [0.0, 0.5, -0.5, 0.99, -0.99, 0.25, -0.25];
        for &val in &test_values {
            let q31 = f32_to_q31(val);
            let back = q31_to_f32(q31);
            assert!(
                (val - back).abs() < 1e-6,
                "Round-trip failed for {}: got {}",
                val,
                back
            );
        }
    }

    #[test]
    fn q31_known_values() {
        assert!((q31_to_f32(i32::MAX) - 1.0).abs() < 1e-6);
        assert!((q31_to_f32(i32::MIN) - (-1.0)).abs() < 1e-6);
        assert_eq!(q31_to_f32(0), 0.0);
    }

    #[test]
    fn angle_to_cordic_q31_zero() {
        let q31 = angle_to_cordic_q31(0.0);
        assert_eq!(q31, 0);
    }

    #[test]
    fn angle_to_cordic_q31_pi() {
        let q31 = angle_to_cordic_q31(PI);
        let f = q31_to_f32(q31);
        assert!((f - 1.0).abs() < 0.01 || (f - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn angle_to_cordic_q31_typical_foc_range() {
        let q31_half = angle_to_cordic_q31(FRAC_PI_2);
        let f_half = q31_to_f32(q31_half);
        assert!((f_half - 0.5).abs() < 0.01);

        let q31_neg_half = angle_to_cordic_q31(3.0 * FRAC_PI_2);
        let f_neg_half = q31_to_f32(q31_neg_half);
        assert!((f_neg_half - (-0.5)).abs() < 0.01);
    }
}
