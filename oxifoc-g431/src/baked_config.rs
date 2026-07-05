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

use oxifoc_core::storage::{
    CurrentLimitsConfig, FailsafeConfigStored, MotorParamsConfig, RuntimeConfig,
};

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
        // ZD2808 700 KV (wye, 12N14P), bench-measured 2026-07-05 (see
        // docs/TODO.md bench section). With SENSORLESS=true a present+valid
        // motor_params makes the board boot on the back-EMF observer.
        // - R: 2-point DC detection (includes residual dead-time; that is
        //   the effective R the drive sees).
        // - L: the AC value (~24 µH/ph, LCR 1–10 kHz plateau), NOT the
        //   voltage-pulse DC 86–129 µH — PI gains scale kp = L·bw and the
        //   observer's L·di/dt term lives at control bandwidth; the pulse
        //   value runs ~4× hot (2026-06-13 lesson). Non-salient: Ld = Lq.
        // - flux: 1/ω-extrapolated true value (single-speed measurements
        //   read high by V_err/ω — 1.28 mWb at the default 700 eRPM).
        // - rating: √(10 W / R / 1.5) — the 10 W detection class.
        motor_params: Some(MotorParamsConfig {
            resistance_ohm: 0.127,
            inductance_d_h: 24.0e-6,
            inductance_q_h: 24.0e-6,
            flux_linkage_wb: 1.145e-3,
            pole_pairs: 7,
            max_current_a: 7.2,
            max_power_loss_w: 10.0,
        }),
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
        derating: None,
    }
}
