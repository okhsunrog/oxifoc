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
    CurrentLimitsConfig, FailsafeConfigStored, MotorParamsConfig, PiGainsConfig, RuntimeConfig,
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
/// - `bus_in_max_a = 3.5`: just under the bench PSU's 4 A CC limit, so the
///   firmware derates iq before the supply folds. Load-bearing since the
///   readiness lag compensation (2026-07-07): the chattering trust gate
///   had been bang-bang-capping speed (and therefore bus power) by
///   accident; with sustained torque the unloaded motor runs toward its
///   real drag equilibrium and bus draw climbs well past the old regime.
pub fn baked() -> RuntimeConfig {
    RuntimeConfig {
        // ZD2808 700 KV (wye, 12N14P), bench-measured 2026-07-05 (see
        // docs/TODO.md bench section). With SENSORLESS=true a present+valid
        // motor_params makes the board boot on the back-EMF observer.
        // - R: 2-point DC detection (includes residual dead-time; that is
        //   the effective R the drive sees).
        // - L: the AC (HF-plateau) value, ~24 µH. This is the ESTIMATION-
        //   CHAIN inductance: the observer's L·i subtraction and the
        //   deadshort probe's e = −L·dI/dt both read it from here, and both
        //   are hardware-validated at 24 µH — baking the fundamental
        //   85.7/129.4 µH instead made the observer couple the current
        //   oscillation band into its flux vector (false readiness off the
        //   align swing) and scaled the deadshort ω estimate 4.5× (false
        //   catches returned). The fundamental Ld/Lq live in the explicit
        //   decoupling override in foc.rs until MotorParamsConfig grows a
        //   second inductance field (see TODO "two-inductance model").
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
            // Fundamental (voltage-pulse) Ld/Lq — the decoupling values
            // (previously the hardcoded set_decoupling override in foc.rs).
            ld_fundamental_h: 85.7e-6,
            lq_fundamental_h: 129.4e-6,
            // Eddy L(f) ladder: OFF. The 2026-07-08 rotating-carrier sweep
            // "measured" ΔL 146 µH / τ 1.39 ms — and the same-day pulsating
            // (torque-free) carrier proved that curve was the ROTOR
            // swinging on the hold-current spring, not winding eddy
            // currents: the true d-axis L is FLAT ~17 µH from 100 Hz to
            // 1.68 kHz (and per the bench LCR, to 100 kHz). This winding
            // has no meaningful L(f) transition to compensate.
            eddy_delta_l_h: 0.0,
            eddy_tau_s: 0.0,
        }),
        hall_calibration: None,
        dc_offsets: None,
        current_limits: Some(CurrentLimitsConfig {
            max_iq_a: 10.0,
            max_phase_current_a: 40.0,
            bus_in_max_a: 3.5,
            bus_regen_max_a: 0.0,
        }),
        voltage_limits: None,
        pwm_config: None,
        // Explicit current-loop gains — kp from the HF (AC) inductance
        // (24 µH × 1000 rad/s), ki = R × 1000. Overrides the l_avg-derived
        // tuning (l_avg is the fundamental L now, 4.5× the HF value).
        // 2026-07-06 gain experiment: kp = 0.1075 (bw 1000 against the
        // fundamental L) did NOT damp the 1.5 A mid-speed limit cycle and
        // made startup worse (align-swing excitation → false instant
        // handoffs) — the cycle is an estimation-chain problem, not loop
        // bandwidth; see TODO "current loop at speed". This kp is the
        // best-behaved bench configuration.
        pi_gains: Some(PiGainsConfig {
            kp: 0.024,
            ki: 127.0,
            bandwidth_rad_s: 1000.0,
        }),
        hall_tuning: None,
        failsafe: Some(FailsafeConfigStored {
            policy: 1, // RampToZero — PSU-safe: unload, never regen
            // 400 ms (default 150): bench tuning for THIS board's ISR
            // budget. Sustained 1.5 A drive under a 1 kHz capture runs the
            // 20 kHz ISR at ~87-90% — the thread-mode affirm pump falls
            // behind ~20 ms/s and crosses 150 ms of staleness within a few
            // seconds (2026-07-06 endurance runs; staleness ratchet
            // 100→160 ms). 400 ms still stops a dead-link bench spin fast;
            // the 150 ms vehicle bound returns with the tier-2 ISR shave
            // or on G474-class hardware.
            // 2026-07-07: 400 → 800. The host's single ordered
            // command/affirm task goes silent for a DETERMINISTIC ~410 ms
            // whenever an acked command's response round-trip stalls (three
            // consecutive bench runs measured stale_max 410.15-410.19 ms —
            // a host-side retry quantum, not link loss), and every such
            // hole crossed the 400 ms bound with the pump otherwise
            // perfectly alive at 20 affirms/s. 800 ms still stops a
            // dead-link bench spin inside a second; the riding default
            // stays 150 ms.
            staleness_timeout_ms: 800,
            ..FailsafeConfigStored::default()
        }),
        velocity: None,
        // Speed soft ceiling: the ONLY thing bounding speed under a torque
        // command on an unloaded rotor. 2026-07-07 (gate-fix-3/5): with the
        // readiness lag compensation the trust gate no longer chatters, and
        // an un-speed-limited 1.5 A free spin runs the rotor past ~19 k
        // erpm within 150 ms of handoff — the current loop (ωc = 1000
        // rad/s) loses regulation around ~2 000 el rad/s and trips OC. The
        // chattering gate had been an accidental bang-bang speed governor
        // at 4–5 k erpm; this ceiling replaces it with the intended
        // mechanism. Sized for THIS board's ISR budget, not the current
        // loop: a 1 200 el rad/s ceiling (~11.5 k erpm) survived the OC but
        // starved thread mode (794 ms exec stall → deadman failsafe,
        // gate-fix-6) — sustained 12 k+ erpm under a 2 kHz debug capture
        // pins the ISR. 600 el rad/s (~5.7 k erpm) sits in the bench-proven
        // band; free-rotor momentum overshoots a 9 300 el rad/s² spin-up
        // ~30% past the ceiling before drag settles it. Raise on G474-class
        // hardware or after the next ISR shave.
        //
        // Ceiling 400 / rolloff from 0.5× = 200: sized so the free-rotor
        // momentum OVERSHOOT (~1.3-1.6× past the ceiling at 9 300 el
        // rad/s², measured on the 600- and 500-ceiling probes: peaks 973
        // and 808 el rad/s) stays under the ISR wall — the per-cycle FOC
        // cost grows with speed (isre `out` 176 cy @420 el rad/s →
        // 1 090 cy @~700, cause unknown, see docs/TODO.md) and past
        // ~650 el rad/s the core saturates: overrun cascade, thread mode
        // starved for ~1 s, deadman failsafe (gate-fix-6/7/8). 400 keeps
        // the overshoot peak ≤ ~550 and the sustained cruise ~3.8 k erpm —
        // the bench-proven regime, now STEADY instead of gate-chattered.
        derating: Some(oxifoc_core::storage::DeratingConfigStored {
            max_speed_erad_s: 400.0,
            speed_start_frac: 0.5,
            ..oxifoc_core::storage::DeratingConfigStored::default()
        }),
    }
}
