//! Fast scalar math for the FOC hot path.
//!
//! `libm` is written for accuracy, not for a 8500-cycle ISR budget: on our
//! Cortex-M4F targets `sqrtf` costs ~110 cycles and `atan2f` ~170 (measured
//! on the G431 @ 170 MHz, see docs/perf-bench-2026-06-11.md). The functions
//! here are the hot-path replacements:
//!
//! - [`sqrtf`]: the M4F `vsqrt.f32` instruction (IEEE correctly rounded —
//!   bit-identical to libm, measured 25 vs 110 cycles) behind a target cfg,
//!   libm elsewhere. No behavioral divergence between sim and hardware.
//! - [`atan2f`]: a polynomial approximation (|err| ≤ ~0.011 rad), the same
//!   pure-Rust code on every target — the simulation exercises exactly what
//!   the firmware runs. 46 vs 169 cycles.

/// Square root.
///
/// On ARM with VFP (all our Cortex-M4F boards set `+vfp2`) this is the
/// hardware `vsqrt.f32` instruction — IEEE correctly rounded, so results
/// are bit-identical to `libm::sqrtf` (verified on target). Negative
/// inputs return NaN, same as libm.
#[cfg(all(target_arch = "arm", target_feature = "vfp2"))]
#[inline(always)]
pub fn sqrtf(x: f32) -> f32 {
    let r: f32;
    unsafe {
        core::arch::asm!(
            "vsqrt.f32 {o}, {i}",
            o = out(sreg) r,
            i = in(sreg) x,
            options(pure, nomem, nostack),
        )
    };
    r
}

/// Square root (portable fallback — bit-identical to the hardware path).
#[cfg(not(all(target_arch = "arm", target_feature = "vfp2")))]
#[inline(always)]
pub fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

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
