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
// The 20 kHz ISR hot path is GENERIC (FocDriver/PhaseManager/FocController
// over platform types), so it monomorphizes into the DEVICE crate and
// inherits its size-optimized profile — the per-package opt-level override
// for oxifoc-core cannot reach it. At opt-level "z" the machine outliner
// chops the ISR into cross-calls and struct returns become memcpys (SWD PC
// sampling, tier-2 2026-07-06 and tier-3 2026-07-07). `#[optimize(speed)]`
// on the hot entry points strips the minsize hint per-function wherever
// they are instantiated. Nightly feature; the repo is nightly-pinned
// (build-std in the device crates).
#![cfg_attr(feature = "isr-speed", feature(optimize_attribute))]
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

/// Timer abstraction for async delays
pub mod timer;

/// Per-section ISR cycle profiling (feature `isr-profiling`, device-only)
pub mod isr_prof;

/// High-level motor driver combining FOC with sensors and PWM
pub mod motor;

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

/// Async runtime with servers (requires `runtime` feature)
///
/// Provides async protocol servers that access state directly:
/// - Servers for Hall, ADC, motor commands, device info
/// - No MotorRuntime trait needed - servers use state module
#[cfg(feature = "runtime")]
pub mod runtime;

/// Field-Oriented Control algorithms
pub mod foc {
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
    #[inline]
    pub fn wrap_angle(angle: f32) -> f32 {
        let mut a = angle;
        if !(-TAU..=TAU).contains(&a) {
            // Cold: arbitrary input far outside the incremental domain.
            a %= TAU;
        }
        if a < 0.0 {
            a += TAU;
        } else if a >= TAU {
            a -= TAU;
        }
        a
    }

    /// Compute signed angle difference (a - b), handling wraparound.
    /// Result is in range (-π, π].
    ///
    /// Same hot/cold split as [`wrap_angle`]: wrapped inputs give a raw
    /// difference in (-2π, 2π), where one conditional ±2π lands in range —
    /// `libm::remainderf` (which cost ~1% of the ISR CPU) stays as the cold
    /// fallback for arbitrary inputs.
    #[inline]
    pub fn angle_difference(a: f32, b: f32) -> f32 {
        let mut diff = a - b;
        if !(-TAU..=TAU).contains(&diff) {
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

    /// Sector-based phase current reconstruction for unipolar shunt sensing
    pub mod current_reconstruction;

    /// Hall sensor calibration algorithm
    pub mod hall_calibration;

    /// Hall sensor angle estimation
    pub mod hall_sensor;

    /// PI controller with anti-windup
    pub mod pi_controller;

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
