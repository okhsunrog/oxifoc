//! Compiled-in configuration for the baked profile (`storage` feature off).
//!
//! Workflow (see docs/flash-size.md):
//! 1. Flash this firmware (detection is on by default), run detection and
//!    tune live from the host — the RAM-backed config server accepts writes
//!    and live-applies them, nothing touches flash.
//! 2. Extract the result: `oxifoc-host-cli config dump --rust` prints this
//!    file's `baked()` body with every group the device currently holds.
//! 3. Paste it here, rebuild, reflash — the configuration is now permanent
//!    (and the 4 KB flash storage region is reclaimed for code).
//!
//! `None` fields fall back to the board defaults in `config.rs`, exactly
//! like an empty flash store would.

use oxifoc_core::storage::{CurrentLimitsConfig, FailsafeConfigStored, RuntimeConfig};

/// The baked configuration. Replace with `config dump --rust` output.
///
/// CURRENT PROFILE: **bench / lab-PSU safe** (2026-06-12, see
/// docs/TODO.md bench section and decisions.md bus-limits entry). A lab PSU
/// cannot absorb reverse current, so nothing may regen into the bus:
/// - failsafe policy `RampToZero` (1): link loss just unloads the motor —
///   no regen braking. Return to `ControlledStop` (2) + `ParkBrake` for
///   battery riding.
/// - `bus_regen_max_a = 0.0`: hard ban on charge current into the supply
///   (ControlledStop would self-degrade to coast via the no-progress
///   watchdog, windings-short Brake never touches the bus).
/// - `bus_in_max_a = 10.0`: conservative draw cap below a typical bench
///   PSU rating; raise to taste at the bench.
pub fn baked() -> RuntimeConfig {
    RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: Some(CurrentLimitsConfig {
            max_iq_a: 10.0,
            max_phase_current_a: 40.0,
            bus_in_max_a: 10.0,
            bus_regen_max_a: 0.0,
        }),
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
        failsafe: Some(FailsafeConfigStored {
            policy: 1, // RampToZero — PSU-safe: unload, never regen
            ..FailsafeConfigStored::default()
        }),
        velocity: None,
    }
}
