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

use oxifoc_core::storage::RuntimeConfig;

/// The baked configuration. Replace with `config dump --rust` output.
pub fn baked() -> RuntimeConfig {
    RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: None,
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
        failsafe: None,
        velocity: None,
    }
}
