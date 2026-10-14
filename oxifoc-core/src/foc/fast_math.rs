//! Fast scalar math for the FOC hot path.
//!
//! `libm` is written for accuracy, not for a 8500-cycle ISR budget: on our
//! Cortex-M4F targets `atan2f` costs ~170 cycles (measured
//! on the G431 @ 170 MHz, see docs/perf-bench-2026-06-11.md). The functions
//! here are the hot-path replacements:
//!
//! - [`atan2f`]: a polynomial approximation (|err| ≤ ~0.011 rad), the same
//!   pure-Rust code on every target — the simulation exercises exactly what
//!   the firmware runs. 46 vs 169 cycles.

/// Fast atan2 — VESC-style polynomial (`utils_fast_atan2`).
///
/// Max error vs `libm::atan2f`: **≤ ~0.011 rad (0.6°)** over the unit
/// circle (measured on target). Intended for PLL inputs (the back-EMF
/// observer's flux-vector angle): the PLL low-passes the estimate, so a
/// sub-degree static error is far below what dead time and inverter
/// nonlinearity already contribute.
///
/// Same pure-Rust code on every target — no sim/hardware divergence.
/// Returns **π/2 for (0, 0)** (the 1e-20 bias on |y| makes x = 0 resolve
/// as if y were an infinitesimal positive), unlike `libm::atan2f`'s 0.0.
/// Callers gate the degenerate zero-flux case on confidence, not angle.
#[inline]
pub fn atan2f(y: f32, x: f32) -> f32 {
    use core::f32::consts::FRAC_PI_4;

    let abs_y = if y < 0.0 { -y } else { y } + 1e-20;
    let angle = if x >= 0.0 {
        let r = (x - abs_y) / (x + abs_y);
        let rsq = r * r;
        (0.1963 * rsq - 0.9817) * r + FRAC_PI_4
    } else {
        let r = (x + abs_y) / (abs_y - x);
        let rsq = r * r;
        (0.1963 * rsq - 0.9817) * r + 3.0 * FRAC_PI_4
    };
    if y < 0.0 { -angle } else { angle }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;
    use core::f32::math::sqrt as sqrtf;

    #[test]
    fn sqrt_matches_libm() {
        let mut v = 1e-9f32;
        while v < 1e9 {
            assert_eq!(sqrtf(v), libm::sqrtf(v), "sqrt({v})");
            v *= 3.7;
        }
        assert_eq!(sqrtf(0.0), 0.0);
        assert!(sqrtf(-1.0).is_nan());
    }

    #[test]
    fn sqrt_matches_libm_across_bit_patterns() {
        // Include signed zero, subnormals, infinities and NaNs, then sample
        // the full f32 bit space deterministically. NaN payloads may differ.
        for bits in [
            0,
            0x8000_0000,
            1,
            0x007f_ffff,
            0x0080_0000,
            0x7f7f_ffff,
            0x7f80_0000,
            0xff80_0000,
            0x7fc0_0000,
        ]
        .into_iter()
        .chain((0..1_000_000u32).map(|i| i.wrapping_mul(2_654_435_761)))
        {
            let x = f32::from_bits(bits);
            let actual = sqrtf(x);
            let expected = libm::sqrtf(x);
            assert!(
                actual.to_bits() == expected.to_bits() || actual.is_nan() && expected.is_nan(),
                "sqrt({bits:08x}): {actual:?} != {expected:?}"
            );
        }
    }

    #[test]
    fn atan2_error_bounded() {
        // Sweep the unit circle at the flux-linkage magnitude scale the
        // observer actually feeds in.
        let mut max_err = 0.0f32;
        for i in 0..4096 {
            let a = (i as f32) / 4096.0 * 2.0 * PI - PI;
            let (y, x) = (0.02 * libm::sinf(a), 0.02 * libm::cosf(a));
            let err = libm::fabsf(atan2f(y, x) - libm::atan2f(y, x));
            max_err = max_err.max(err);
        }
        assert!(max_err < 0.011, "atan2 max error {max_err} rad");
    }

    #[test]
    fn atan2_quadrants_and_axes() {
        for (y, x, expect) in [
            (0.0f32, 1.0f32, 0.0f32),
            (1.0, 0.0, PI / 2.0),
            (-1.0, 0.0, -PI / 2.0),
            (1.0, 1.0, PI / 4.0),
            (-1.0, -1.0, -3.0 * PI / 4.0),
        ] {
            let err = libm::fabsf(atan2f(y, x) - expect);
            assert!(err < 0.011, "atan2({y},{x}) = {} != {expect}", atan2f(y, x));
        }
    }
}
