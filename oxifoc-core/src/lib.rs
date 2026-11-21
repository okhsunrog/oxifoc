//! # oxifoc-core
//!
//! Platform-agnostic Field-Oriented Control (FOC) algorithms and motor control logic.
//!
//! This crate provides the mathematical foundation for FOC motor control:
//! - Coordinate transformations (Clarke, Park)
//! - Space Vector PWM (SVPWM) modulation
//! - PI controllers with anti-windup
//! - FOC control loops
//! - Motor parameter detection (R, L, λ)
//!
//! ## Feature Flags
//!
//! - **`algorithms`** (default): FOC math algorithms
//! - **`icd`**: Interface Control Document with ergot endpoints
//! - **`runtime`**: Async runtime with servers
//! - **`virtual-motor`**: Motor simulation for testing
//! - **`defmt`**: defmt logging support for embedded
//! - **`log`**: log crate support for std
//! - **`std`**: Standard library support
//!
//! ## Usage Examples
//!
//! ### Host application with ICD
//! ```toml
//! oxifoc-core = { version = "0.1", default-features = false, features = ["icd"] }
//! ```
//!
//! ### Embedded firmware
//! ```toml
//! oxifoc-core = { version = "0.1", features = ["runtime", "defmt"] }
//! ```
//!
//! ### Testing with virtual motor
//! ```toml
//! oxifoc-core = { version = "0.1", features = ["virtual-motor"] }
//! ```
//!
//! ## FOC Algorithm Example
//!
//! ```rust
//! use oxifoc_core::foc::{transforms, svpwm, pi_controller::PIController};
//!
//! // Example: Current control loop
//! let mut id_controller = PIController::new(0.5, 10.0);
//! let mut iq_controller = PIController::new(0.5, 10.0);
//!
//! // Sample feedback and setpoints
//! let ia = 1.2;
//! let ib = -0.6;
//! let theta = 0.5_f32;
//! let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
//! let id_target = 0.0;
//! let iq_target = 5.0;
//! let dt = 0.0001;
//! let max_duty = 1000;
//!
//! // Measure currents and transform to dq frame
//! let (i_alpha, i_beta) = transforms::clarke(ia, ib);
//! let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);
//!
//! // Run PI controllers
//! let vd = id_controller.update(id_target, id, dt);
//! let vq = iq_controller.update(iq_target, iq, dt);
//!
//! // Transform back and generate PWM
//! let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
//! let duties = svpwm::space_vector_pwm(v_alpha, v_beta, max_duty);
//! assert_eq!(duties.len(), 3);
//! ```

#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(
    all(feature = "atan2-fma", not(any(test, feature = "std"))),
    feature(core_float_math)
)]
// The 20 kHz ISR hot path is GENERIC (FocDriver/PhaseManager/FocController
// over platform types), so it monomorphizes into the DEVICE crate and
// inherits its size-optimized profile — the per-package opt-level override
// for oxifoc-core cannot reach it. At opt-level "z" the machine outliner
// chops the ISR into cross-calls and struct returns become memcpys (SWD PC
// sampling, tier-2 2026-07-06 and tier-3 2026-07-07). `#[optimize(speed)]`
// on the hot entry points strips the minsize hint per-function wherever
// they are instantiated. The STM32 firmware enables this feature on its
// pinned nightly toolchain.
// Firmware panic policy: this crate runs inside the FOC ISR of a vehicle.
// Composes ON TOP of the shared [workspace.lints] table; the deliberate
// fail-fast sites carry #[expect(..., reason = "...")]. Test code is
// exempted via clippy.toml (allow-unwrap-in-tests & co).
#![warn(
    clippy::unwrap_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented,
    clippy::get_unwrap,
    clippy::unwrap_in_result,
    clippy::panic_in_result_fn
)]

/// Logging macros abstraction (defmt/log/none)
#[macro_use]
mod fmt;

/// `assert!((actual - expected).abs() < tolerance)` for plain `f32`s and
/// quantities alike, reporting all three values when it fails. Defined
/// before the modules so every test module sees it without an import.
#[cfg(test)]
macro_rules! assert_near {
    ($actual:expr, $expected:expr, $tolerance:expr $(,)?) => {{
        let (actual, expected, tolerance) = ($actual, $expected, $tolerance);
        assert!(
            (actual - expected).abs() < tolerance,
            "{} = {:?}, expected {:?} within {:?}",
            stringify!($actual),
            actual,
            expected,
            tolerance,
        );
    }};
}

/// BLE GATT UUIDs of the ergot transport service
pub mod ble;

/// Timer abstraction for async delays
pub mod timer;

/// Per-section ISR cycle profiling (feature `isr-profiling`, device-only)
pub mod isr_prof;

/// High-level motor driver combining FOC with sensors and PWM
pub mod motor;

/// Typed SI quantities (uom) for motor-model arithmetic
pub mod units;

/// Race-free clear of selected **rc_w0** status flags (STM32 `TIMx_SR` and
/// friends: "write 0 to clear, writing 1 has no effect").
///
/// Starts from an all-ones template and lets the closure body zero exactly
/// the flags to clear, then performs a single volatile write — the written
/// value is a compile-time constant (`mvn`+`str` on Cortex-M), so unlike
/// `reg.modify(...)` there is no read→write window in which a flag set by
/// hardware gets written back as 0 and silently erased. This is the same
/// pattern as ST HAL's `SR = ~FLAG` and embassy's time driver ("RMWing
/// won't work, they can miss interrupts", `time_driver/gp16.rs`).
///
/// A macro rather than a function so the fieldset type (`SrAdv`/`SrGp16`/
/// `SrGp32`, no common raw-access trait) is inferred at the call site —
/// and so the safe pattern has a name: the near-identical
/// `reg.write(|w| w.set_x(false))` starts from *zeros* and clears every
/// flag in the register.
///
/// **Only valid for rc_w0 registers.** On rc_w1 registers (e.g. G4 ADC ISR,
/// "write 1 to clear") an all-ones write would clear everything — do not
/// use this there.
///
/// ```ignore
/// oxifoc_core::clear_rc_w0!(pac::TIM1.sr(), |w| w.set_bif(0, false));
/// oxifoc_core::clear_rc_w0!(pac::TIM4.sr(), |w| {
///     w.set_uif(false);
///     w.set_ccof(0, false);
/// });
/// ```
#[macro_export]
macro_rules! clear_rc_w0 {
    ($reg:expr, |$w:ident| $body:expr) => {{
        $reg.write(|$w| {
            $w.0 = !0;
            $body;
        });
    }};
}

/// Shared types for protocol communication
///
/// Contains serializable types shared between firmware and host applications:
/// - Motor state and control types (MotorState, ControlMode)
/// - Telemetry types (HallSensorData, AdcSample, MotorStatus)
/// - Device info and events
pub mod types;

/// Independent raw diagnostic blocks, shared by the optional bench sinks.
pub mod raw_telemetry;
#[cfg(feature = "spi-link")]
pub mod spi_link;
pub mod velocity_telemetry;

/// Interface Control Document with ergot endpoints (requires `icd` feature)
///
/// Defines the communication protocol between host and device:
/// - Endpoint definitions for ergot framework
/// - Re-exports all types from the `types` module
#[cfg(feature = "icd")]
pub mod icd;

/// Delivery semantics (requires `delivery` feature)
///
/// A typed ladder of delivery guarantees over ergot's at-most-once transport:
/// - `Command` + delivery classes (Idempotent / Deduplicated / AtMostOnce)
/// - a pure, testable retry policy
/// - `Keyed`/`ReqId` and server-side dedup for effectively-once
#[cfg(feature = "delivery")]
pub mod delivery;

/// Motor state management (requires `runtime` feature)
///
/// Centralized state management for motor control:
/// - Global STATE with motor state, telemetry
/// - CMD_CHANNEL for protocol commands
/// - TELEMETRY watch for streaming
/// - Helper functions for ISR use
#[cfg(feature = "runtime")]
pub mod state;

/// Dynamic PMSM motor simulation (requires `virtual-motor` feature)
#[cfg(feature = "virtual-motor")]
pub mod virtual_motor;

/// Persistent configuration storage types (requires `storage` feature)
#[cfg(feature = "storage")]
pub mod storage;

#[cfg(feature = "storage")]
pub mod drive_setup;
#[cfg(feature = "embassy-hal")]
pub mod stm32_rng;

/// Async runtime with servers (requires `runtime` feature)
///
/// Provides async protocol servers that access state directly:
/// - Servers for Hall, ADC, motor commands, device info
/// - No MotorRuntime trait needed - servers use state module
#[cfg(feature = "runtime")]
pub mod runtime;

/// Field-Oriented Control algorithms
pub mod foc {
    use crate::units::AngleRepr;
    use core::f32::consts::TAU;

    /// Panic-free f32 clamp. Equivalent to `f32::clamp()` but without the
    /// `debug_assert!(min <= max)` that pulls in `core::fmt::float` (~4KB)
    /// when `opt-level = "z"` changes inlining decisions.
    #[inline(always)]
    pub fn clamp_f32(val: f32, min: f32, max: f32) -> f32 {
        if val < min {
            min
        } else if val > max {
            max
        } else {
            val
        }
    }

    /// Wrap angle to [0, 2π)
    ///
    /// Hot path is branch+subtract: every per-cycle caller (observer, PLL,
    /// startup ramp, hall interpolation) feeds an already-wrapped angle plus
    /// a small increment, so the input is within (-2π, 4π) essentially
    /// always. The `%` fallback (f32 `%` = `fmodf` → libm's remquo path,
    /// ~100+ cycles — 2026-07-06 ISR PC-profiling caught it at ~1% of the
    /// whole CPU) only runs for arbitrary out-of-range inputs.
    ///
    /// The common case — already inside [0, 2π) — is decided by one integer
    /// compare of the bit pattern: non-negative floats order like their
    /// bits, while negative values, -0.0, infinities and NaN all compare
    /// above 2π and take the general path. On the F405 each float constant
    /// is a flash literal load that usually misses the 8-line ART data
    /// cache (PC profile 2026-11-14: 30–40 cycles per call before this).
    #[inline(always)]
    pub fn wrap_angle<A: AngleRepr>(angle: A) -> A {
        A::from_radians(wrap_radians(angle.radians()))
    }

    #[inline]
    fn wrap_radians(angle: f32) -> f32 {
        if angle.to_bits() < TAU.to_bits() {
            return angle;
        }
        wrap_angle_general(angle)
    }

    /// One-turn steps decided on the bit pattern, without Flash literals or
    /// FP compares. Every other input keeps the complete fallback below.
    #[cold]
    #[inline(never)]
    fn wrap_angle_general(angle: f32) -> f32 {
        let bits = angle.to_bits();
        let magnitude = bits & 0x7fff_ffff;
        // [TAU, 2*TAU) loses one turn; [-TAU, -0) gains one. -0.0, exact
        // 2*TAU, NaN and everything larger stay with the complete fallback.
        let one_turn = if bits == magnitude {
            bits >= TAU.to_bits() && bits < (2.0 * TAU).to_bits()
        } else {
            magnitude != 0 && magnitude <= TAU.to_bits()
        };
        if one_turn {
            return angle - f32::from_bits(TAU.to_bits() | (bits & 0x8000_0000));
        }
        wrap_angle_complete(angle)
    }

    #[cold]
    #[inline(never)]
    fn wrap_angle_complete(angle: f32) -> f32 {
        let mut a = angle;
        // Keep exact 2*TAU cold: one subtraction would still leave TAU.
        if !(-TAU..2.0 * TAU).contains(&a) {
            a = wrap_remainder(a);
        }
        if a < 0.0 {
            a += TAU;
        } else if a >= TAU {
            a -= TAU;
        }
        a
    }

    // Keep this out of line: LLVM otherwise speculates fmodf before the
    // range check, charging every incremental update for the cold fallback.
    #[cold]
    #[inline(never)]
    fn wrap_remainder(angle: f32) -> f32 {
        angle % TAU
    }

    /// Compute signed angle difference (a - b), handling wraparound.
    /// Result is in range (-π, π].
    ///
    /// Same hot/cold split as [`wrap_angle`]: a signed atan2 angle minus a
    /// [0, 2π) PLL angle gives a difference in (-3π, 3π). One ±2π step
    /// lands in range; both common angle conventions take this fast path —
    /// `libm::remainderf` (which cost ~1% of the ISR CPU) stays as the cold
    /// fallback for arbitrary inputs.
    ///
    /// |diff| < π needs no correction and is decided by one integer compare
    /// of the magnitude bits (NaN and infinities fall through), for the
    /// same flash-literal reason as [`wrap_angle`] (40–70 cycles per call
    /// before this).
    #[inline(always)]
    pub fn angle_difference<A: AngleRepr>(a: A, b: A) -> A {
        A::from_radians(radians_difference(a.radians(), b.radians()))
    }

    #[inline]
    fn radians_difference(a: f32, b: f32) -> f32 {
        let diff = a - b;
        if diff.to_bits() & 0x7fff_ffff < core::f32::consts::PI.to_bits() {
            return diff;
        }
        angle_difference_general(diff)
    }

    /// Signed atan2 angle minus a [0, 2*pi) angle: the difference lies in
    /// (-3*pi, pi], so only the negative side ever needs a turn. That side
    /// covers half of every revolution, hence the inline step. Results match
    /// [`angle_difference`] bit for bit; other inputs keep the fallback.
    #[inline(always)]
    pub fn angle_difference_signed_minus_wrapped<A: AngleRepr>(a: A, b: A) -> A {
        A::from_radians(signed_minus_wrapped(a.radians(), b.radians()))
    }

    #[inline(always)]
    fn signed_minus_wrapped(a: f32, b: f32) -> f32 {
        let diff = a - b;
        let bits = diff.to_bits();
        let magnitude = bits & 0x7fff_ffff;
        if magnitude < core::f32::consts::PI.to_bits() {
            return diff;
        }
        if bits != magnitude && magnitude < (TAU + core::f32::consts::PI).to_bits() {
            return diff + TAU;
        }
        angle_difference_general(diff)
    }

    /// One-turn correction on the bit pattern. The complete fallback keeps
    /// exact +/-3*pi, NaN and unwrapped inputs.
    #[cold]
    #[inline(never)]
    fn angle_difference_general(diff: f32) -> f32 {
        let bits = diff.to_bits();
        let magnitude = bits & 0x7fff_ffff;
        if magnitude < (TAU + core::f32::consts::PI).to_bits() {
            // (-pi, pi] is already the result; -pi itself moves to +pi.
            if magnitude < core::f32::consts::PI.to_bits()
                || bits == core::f32::consts::PI.to_bits()
            {
                return diff;
            }
            return diff - f32::from_bits(TAU.to_bits() | (bits & 0x8000_0000));
        }
        angle_difference_complete(diff)
    }

    #[cold]
    #[inline(never)]
    fn angle_difference_complete(mut diff: f32) -> f32 {
        let three_pi = TAU + core::f32::consts::PI;
        if diff <= -three_pi || diff > three_pi || diff.is_nan() {
            // Cold: inputs weren't wrapped angles.
            diff = libm::remainderf(diff, TAU);
        }
        if diff <= -core::f32::consts::PI {
            diff += TAU;
        } else if diff > core::f32::consts::PI {
            diff -= TAU;
        }
        diff
    }

    #[cfg(test)]
    mod angle_tests {
        use super::*;

        // Verbatim pre-fast-path implementations (8fa1e5b).
        fn frozen_wrap(angle: f32) -> f32 {
            let mut a = angle;
            if !(-TAU..2.0 * TAU).contains(&a) {
                a %= TAU;
            }
            if a < 0.0 {
                a += TAU;
            } else if a >= TAU {
                a -= TAU;
            }
            a
        }

        fn frozen_difference(a: f32, b: f32) -> f32 {
            let mut diff = a - b;
            let three_pi = TAU + core::f32::consts::PI;
            if diff <= -three_pi || diff > three_pi || diff.is_nan() {
                diff = libm::remainderf(diff, TAU);
            }
            if diff <= -core::f32::consts::PI {
                diff += TAU;
            } else if diff > core::f32::consts::PI {
                diff -= TAU;
            }
            diff
        }

        fn same(actual: f32, expected: f32) -> bool {
            (actual.is_nan() && expected.is_nan()) || actual.to_bits() == expected.to_bits()
        }

        // Special values plus every multiple of π/2 up to ±9π/2 … ±9·2π with
        // three neighbours on each side; no allocation (the crate is no_std).
        fn edge_values(mut visit: impl FnMut(f32)) {
            use core::f32::consts::PI;
            for value in [
                f32::NAN,
                f32::INFINITY,
                f32::NEG_INFINITY,
                f32::MAX,
                f32::MIN,
                f32::MIN_POSITIVE,
                f32::from_bits(1),
                -f32::from_bits(1),
                0.0,
                -0.0,
                1e-30,
                -1e-30,
                1e30,
                -1e30,
            ] {
                visit(value);
            }
            for k in -9_i32..=9 {
                for edge in [k as f32 * PI, k as f32 * TAU, k as f32 * 0.5 * PI] {
                    let (mut below, mut above) = (edge, edge);
                    visit(edge);
                    for _ in 0..3 {
                        below = below.next_down();
                        above = above.next_up();
                        visit(below);
                        visit(above);
                    }
                }
            }
        }

        #[test]
        fn integer_fast_paths_are_bit_identical_to_the_frozen_implementations() {
            edge_values(|a| {
                assert!(same(wrap_angle(a), frozen_wrap(a)), "wrap({a:?})");
                edge_values(|b| {
                    assert!(
                        same(angle_difference(a, b), frozen_difference(a, b)),
                        "difference({a:?}, {b:?})"
                    );
                });
            });
            // Every exponent and sign, plus dense coverage of the angle range.
            let mut bits = 0x9e37_79b9_u32;
            let mut next = || {
                bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                bits
            };
            for _ in 0..2_000_000 {
                let raw = f32::from_bits(next());
                let angle = (next() as f32 / u32::MAX as f32 - 0.5) * 8.0 * TAU;
                let other = (next() as f32 / u32::MAX as f32 - 0.5) * 8.0 * TAU;
                for a in [raw, angle] {
                    assert!(same(wrap_angle(a), frozen_wrap(a)), "wrap({a:?})");
                    assert!(
                        same(angle_difference(a, other), frozen_difference(a, other)),
                        "difference({a:?}, {other:?})"
                    );
                }
            }
        }

        #[test]
        fn observer_turn_matches_frozen_boundaries_and_random_values() {
            edge_values(|a| {
                edge_values(|b| {
                    assert!(same(
                        angle_difference_signed_minus_wrapped(a, b),
                        frozen_difference(a, b)
                    ));
                });
            });
            let mut bits = 0x2545_f491_u32;
            for _ in 0..2_000_000 {
                bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let a = f32::from_bits(bits);
                bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let b = f32::from_bits(bits);
                assert!(same(
                    angle_difference_signed_minus_wrapped(a, b),
                    frozen_difference(a, b)
                ));
                // The observer's own domain: atan2 output against a wrapped PLL angle.
                let a = (bits >> 8) as f32 / (1 << 24) as f32 * TAU - core::f32::consts::PI;
                bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let b = (bits >> 8) as f32 / (1 << 24) as f32 * TAU;
                assert!(same(
                    angle_difference_signed_minus_wrapped(a, b),
                    frozen_difference(a, b)
                ));
            }
        }

        fn previous_wrap(angle: f32) -> f32 {
            let mut a = angle;
            if !(-TAU..=TAU).contains(&a) {
                a %= TAU;
            }
            if a < 0.0 {
                a += TAU;
            } else if a >= TAU {
                a -= TAU;
            }
            a
        }

        fn assert_wrap_bits(angle: f32) {
            assert_eq!(
                wrap_angle(angle).to_bits(),
                previous_wrap(angle).to_bits(),
                "input bits: {:#010x}",
                angle.to_bits()
            );
        }

        #[test]
        fn wrap_boundaries_and_cold_inputs_preserve_bits() {
            for boundary in [-2.0 * TAU, -TAU, TAU, 2.0 * TAU] {
                for bits in boundary.to_bits() - 8..=boundary.to_bits() + 8 {
                    assert_wrap_bits(f32::from_bits(bits));
                }
            }
            for angle in [
                -f32::MAX,
                -100.0 * TAU,
                -f32::MIN_POSITIVE,
                -f32::from_bits(1),
                -0.0,
                0.0,
                f32::from_bits(1),
                f32::MIN_POSITIVE,
                100.0 * TAU,
                f32::MAX,
                f32::NEG_INFINITY,
                f32::INFINITY,
                f32::NAN,
                f32::from_bits(0xffc0_1234),
                f32::from_bits(0x7f80_0001),
            ] {
                assert_wrap_bits(angle);
            }

            assert_eq!(wrap_angle(-0.0).to_bits(), (-0.0_f32).to_bits());
            assert_eq!(wrap_angle(0.0).to_bits(), 0.0_f32.to_bits());
            assert_eq!(wrap_angle(2.0 * TAU).to_bits(), 0.0_f32.to_bits());
            // Preserve the existing rounding to TAU for tiny negative inputs.
            assert_eq!(wrap_angle(-f32::from_bits(1)).to_bits(), TAU.to_bits());
        }

        #[test]
        fn wrap_positive_crossing_exhaustive_preserves_bits() {
            let start = TAU.to_bits();
            let end = (2.0 * TAU).to_bits();
            assert_eq!(end - start, 8_388_608);
            for bits in start..end {
                assert_wrap_bits(f32::from_bits(bits));
            }
        }

        fn previous_difference(a: f32, b: f32) -> f32 {
            let mut diff = a - b;
            if !(-TAU..=TAU).contains(&diff) {
                diff = libm::remainderf(diff, TAU);
            }
            if diff <= -core::f32::consts::PI {
                diff += TAU;
            } else if diff > core::f32::consts::PI {
                diff -= TAU;
            }
            diff
        }

        #[test]
        fn signed_atan2_and_unsigned_pll_difference_preserve_bits() {
            use core::f32::consts::PI;
            for i in 0..=512 {
                let signed = -PI + TAU * (i as f32 / 512.0);
                for j in 0..=512 {
                    let unsigned = TAU * (j as f32 / 512.0);
                    assert_eq!(
                        angle_difference(signed, unsigned).to_bits(),
                        previous_difference(signed, unsigned).to_bits()
                    );
                }
            }
            for a in [
                -f32::MAX,
                -10.0 * TAU,
                -3.0 * PI,
                -PI,
                0.0,
                PI,
                3.0 * PI,
                10.0 * TAU,
                f32::MAX,
            ] {
                for b in [-TAU, -PI, -0.0, 0.0, PI, TAU] {
                    assert_eq!(
                        angle_difference(a, b).to_bits(),
                        previous_difference(a, b).to_bits()
                    );
                }
            }
            for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert!(angle_difference(value, 0.0).is_nan());
            }
        }
    }

    /// Board configuration and ADC utilities
    pub mod config;

    /// Mathematical constants (√3, 1/√3, etc.)
    pub mod constants;

    /// High-level FOC control loop
    pub mod controller;

    /// Fault registry shared across targets
    pub mod fault;

    /// Shunt resistor current sensing
    pub mod current_sense;

    /// ISR-owned current-sensor offset calibration
    pub mod current_offset;

    /// Phase-terminal voltage sensing (back-EMF / undriven rotation detection)
    pub mod phase_voltage;

    /// Hall sensor calibration algorithm
    pub mod hall_calibration;

    /// Hall sensor angle estimation
    pub mod hall_sensor;

    /// PI controller with anti-windup
    pub mod pi_controller;

    /// Position control: unwrapped position + the P law feeding the
    /// velocity loop
    pub mod position;

    /// Phase PWM trait for platform drivers
    pub mod pwm;

    /// Sensor trait definitions (CurrentSensor, AngleSensor)
    pub mod sensors;

    /// Space Vector PWM modulation
    pub mod svpwm;

    /// Coordinate transformations (Clarke, Park, and their inverses)
    pub mod transforms;

    /// Fast-telemetry fixed-point codec + shared raw→engineering enrichment
    pub mod telemetry;

    /// Fast hot-path scalar math (hardware sqrt, polynomial atan2)
    pub mod fast_math;

    /// Trigonometric abstractions (SinCos trait, q1.31 helpers for CORDIC)
    pub mod trig;

    /// Velocity control loop building block (slew-limited reference + PI)
    pub mod velocity;

    /// Motor parameter detection (R, L, λ)
    pub mod detection;

    /// Phase management (PhaseProvider, PhaseManager, Observer)
    pub mod phase;

    /// Shared Hall sensor state management for embassy-based platforms
    #[cfg(feature = "embassy")]
    pub mod hall_embassy;

    /// 16-bit timer capture → 64-bit timestamp extension (hall edge timebase)
    pub mod capture_timebase;
}
