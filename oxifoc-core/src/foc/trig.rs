//! Trigonometric function abstractions for FOC control
//!
//! Provides a [`SinCos`] trait so platforms can substitute hardware-accelerated
//! implementations (e.g., STM32 CORDIC peripheral) for the default software
//! [`LibmSinCos`] fallback.
//!
//! Combined with `#[inline(always)]` and monomorphization, trait dispatch is
//! fully erased at compile time — zero runtime overhead.
//!
//! Also provides q1.31 fixed-point conversion helpers used by CORDIC
//! implementations.

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

/// Software sin/cos using libm (default, works on all platforms)
pub struct LibmSinCos;

impl SinCos for LibmSinCos {
    #[inline(always)]
    fn sin_cos(angle: f32) -> (f32, f32) {
        (libm::sinf(angle), libm::cosf(angle))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_libm_sin_cos_zero() {
        let (sin, cos) = LibmSinCos::sin_cos(0.0);
        assert!((sin - 0.0).abs() < 1e-6);
        assert!((cos - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_libm_sin_cos_pi_over_2() {
        let (sin, cos) = LibmSinCos::sin_cos(core::f32::consts::FRAC_PI_2);
        assert!((sin - 1.0).abs() < 1e-6);
        assert!((cos - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_libm_sin_cos_pi() {
        let (sin, cos) = LibmSinCos::sin_cos(core::f32::consts::PI);
        assert!(sin.abs() < 1e-5);
        assert!((cos - (-1.0)).abs() < 1e-6);
    }

    #[test]
    fn test_q31_round_trip() {
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
    fn test_q31_known_values() {
        // q31 max positive ≈ 1.0 (f32 precision rounds 2147483647/2^31 to 1.0)
        assert!((q31_to_f32(i32::MAX) - 1.0).abs() < 1e-6);
        // q31 min is -1.0
        assert!((q31_to_f32(i32::MIN) - (-1.0)).abs() < 1e-6);
        // Zero
        assert_eq!(q31_to_f32(0), 0.0);
    }

    #[test]
    fn test_angle_to_cordic_q31_zero() {
        let q31 = angle_to_cordic_q31(0.0);
        assert_eq!(q31, 0); // 0 radians → 0 in q31
    }

    #[test]
    fn test_angle_to_cordic_q31_pi() {
        let q31 = angle_to_cordic_q31(core::f32::consts::PI);
        // π → 1.0 in normalized → wrapping edge, but should be close to max positive
        let f = q31_to_f32(q31);
        assert!((f - 1.0).abs() < 0.01 || (f - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_angle_to_cordic_q31_typical_foc_range() {
        // FOC angles are typically [0, 2π]
        // 0 → 0, π/2 → 0.5, π → 1.0 (wrap), 3π/2 → -0.5, 2π → 0
        let q31_half = angle_to_cordic_q31(core::f32::consts::FRAC_PI_2);
        let f_half = q31_to_f32(q31_half);
        assert!((f_half - 0.5).abs() < 0.01);

        let q31_neg_half = angle_to_cordic_q31(3.0 * core::f32::consts::FRAC_PI_2);
        let f_neg_half = q31_to_f32(q31_neg_half);
        assert!((f_neg_half - (-0.5)).abs() < 0.01);
    }
}
