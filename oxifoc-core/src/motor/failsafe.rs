//! Self-contained failsafe controller for the command-staleness deadman.
//!
//! When fresh setpoints stop reaching the FOC ISR (host gone, async executor
//! hung, or link dropped — see [`crate::state::run_foc_cycle`]), the driver
//! arms this controller and runs [`FailsafeController::step`] every cycle. It
//! takes **no async/channel input** — `FocDriver` pushes in the quantities it
//! already has (electrical velocity, `dt`, vbus), so it keeps working even
//! when the executor that would normally feed commands is wedged.
//!
//! Three policies (configurable, see [`FailsafePolicy`]); the important one is
//! [`ControlledStop`](FailsafePolicy::ControlledStop): ramp the q-current to
//! zero, then **regen-brake** the rotor to a standstill (or a time / OV
//! limit), so the vehicle stops on link loss instead of free-wheeling away.
//!
//! Safety: the brake target is *intent* — it is routed back through the
//! normal current-control path in `FocDriver`, so the current-limit clamp and
//! the measured-overcurrent trip remain the last line of defense. A
//! regen-induced over-voltage latches the OverVoltage fault, which the
//! `run_foc_cycle` fault gate turns into high-Z (and resets this controller).

use crate::foc::clamp_f32;

/// Standstill must persist this long (s) before [`ControlledStop`] declares
/// the rotor stopped — rides out velocity noise / a momentary zero crossing.
///
/// [`ControlledStop`]: FailsafePolicy::ControlledStop
const STANDSTILL_DEBOUNCE_S: f32 = 0.05;

/// Top fraction of the OV window over which the regen brake current is
/// derated to zero — a soft landing instead of bang-bang into the OV fault.
const OV_DERATE_BAND: f32 = 0.1;

/// What the FOC driver should do this cycle while the failsafe is active.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FailsafeAction {
    /// Drive the current loop to these dq targets. The driver routes this
    /// through `step_current_control`, so `clamp_targets` and the
    /// measured-overcurrent check still apply — the failsafe never bypasses
    /// protection.
    Drive { id_target: f32, iq_target: f32 },
    /// Terminal: cut PWM (high-Z / free-wheel) and leave the failsafe.
    Stop,
}

/// Configurable reaction to a stale command link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FailsafePolicy {
    /// Free-wheel: cut PWM (high-Z). Matches the legacy link-loss behavior.
    Coast = 0,
    /// Ramp the q-current to zero, then free-wheel.
    RampToZero = 1,
    /// Ramp to zero, then regen-brake to a standstill (or a time / OV limit),
    /// then coast.
    ///
    /// Brakes to a full stop only while the angle source tracks to standstill
    /// (Hall, or HFI). A pure back-EMF observer loses lock below its speed
    /// floor, so sensorless-only this brakes to that floor and then coasts.
    ControlledStop = 2,
}

impl FailsafePolicy {
    /// Decode a stored `u8`; an unknown value falls back to the safest policy.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => FailsafePolicy::RampToZero,
            2 => FailsafePolicy::ControlledStop,
            _ => FailsafePolicy::Coast,
        }
    }
}

/// Runtime failsafe tuning (SI units; the deadman timeout is µs to match the
/// 1 MHz `now_ticks` domain). Cached in `FocDriver`; host-tunable via the
/// stored `FailsafeConfigStored` (see `crate::storage`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FailsafeConfig {
    /// Reaction policy.
    pub policy: FailsafePolicy,
    /// Command-staleness threshold (µs). Past this with no fresh setpoint and
    /// the motor running, the deadman arms the failsafe.
    pub staleness_timeout_us: u64,
    /// Regen-brake current *intent* (A); clamped to the current limit at use.
    pub brake_current_a: f32,
    /// Time (s) to slew the q-current by `brake_current_a` (sets the ramp rate
    /// for both ramp-down and brake-up).
    pub ramp_s: f32,
    /// Maximum brake duration (s) before giving up and coasting.
    pub brake_time_s: f32,
    /// |ω_e| (electrical rad/s) below which the rotor counts as stopped.
    pub standstill_rad_s: f32,
}

impl Default for FailsafeConfig {
    /// Longboard default: brake to a controlled stop on link loss.
    fn default() -> Self {
        Self {
            policy: FailsafePolicy::ControlledStop,
            staleness_timeout_us: 150_000, // ≈3× the 50 ms host affirmation
            brake_current_a: 15.0,
            ramp_s: 0.1,
            brake_time_s: 3.0,
            standstill_rad_s: 20.0,
        }
    }
}

impl FailsafeConfig {
    /// Build from the stored (host-writable) form: ms → µs/s, `policy: u8` →
    /// enum (unknown → Coast). A missing or non-sane stored value falls back
    /// to [`Default`] — a corrupt config can never disable the deadman.
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&crate::storage::FailsafeConfigStored>) -> Self {
        match cfg {
            Some(c) => {
                let candidate = Self {
                    policy: FailsafePolicy::from_u8(c.policy),
                    staleness_timeout_us: (c.staleness_timeout_ms as u64) * 1_000,
                    brake_current_a: c.brake_current_a,
                    ramp_s: c.ramp_ms * 1e-3,
                    brake_time_s: c.brake_time_ms * 1e-3,
                    standstill_rad_s: c.standstill_rad_s,
                };
                if candidate.is_sane() {
                    candidate
                } else {
                    Self::default()
                }
            }
            None => Self::default(),
        }
    }

    /// All numeric fields finite and in a usable range.
    pub fn is_sane(&self) -> bool {
        self.staleness_timeout_us > 0
            && self.brake_current_a.is_finite()
            && self.brake_current_a >= 0.0
            && self.ramp_s.is_finite()
            && self.ramp_s > 0.0
            && self.brake_time_s.is_finite()
            && self.brake_time_s >= 0.0
            && self.standstill_rad_s.is_finite()
            && self.standstill_rad_s > 0.0
    }
}

/// Internal phase of the failsafe sequence.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Not engaged.
    Inactive,
    /// Slewing the q-current toward zero (bumpless from the last command).
    RampDown { iq: f32 },
    /// Regen-braking: capped q-current opposing the *original* rotation
    /// direction (`brake_sign` = −sign(ω) captured at entry). Unidirectional
    /// by design — it never flips to drive the rotor the other way, so a
    /// lagging velocity estimate can't pump a limit cycle.
    Brake {
        iq: f32,
        elapsed_s: f32,
        standstill_s: f32,
        brake_sign: f32,
    },
    /// Sequence finished — the driver cuts PWM and clears the controller.
    Done,
}

/// Per-cycle failsafe state machine. Owned by `FocDriver`; armed by the
/// deadman / link-loss path, stepped every ISR cycle until it reaches `Done`.
#[derive(Clone, Copy, Debug)]
pub struct FailsafeController {
    phase: Phase,
}

impl Default for FailsafeController {
    fn default() -> Self {
        Self::new()
    }
}

impl FailsafeController {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Inactive,
        }
    }

    /// Whether the failsafe is currently carrying the motor.
    pub fn is_active(&self) -> bool {
        !matches!(self.phase, Phase::Inactive)
    }

    /// Clear back to inactive (fresh command re-armed normal control, or a
    /// fault took over).
    pub fn reset(&mut self) {
        self.phase = Phase::Inactive;
    }

    /// Arm from the q-current currently being commanded (bumpless) and the
    /// policy. Idempotent: a second arm while already active is ignored, so
    /// the deadman and link-loss paths can't restart a brake in progress.
    pub fn arm(&mut self, current_iq: f32, policy: FailsafePolicy) {
        if self.is_active() {
            return;
        }
        self.phase = match policy {
            // Coast: nothing to drive — the driver maps Done -> Stopped ->
            // pwm.disable() = high-Z, which is the free-wheel we want.
            FailsafePolicy::Coast => Phase::Done,
            FailsafePolicy::RampToZero | FailsafePolicy::ControlledStop => {
                Phase::RampDown { iq: current_iq }
            }
        };
    }

    /// One ISR cycle.
    ///
    /// * `omega_e` — electrical velocity (rad/s) from the phase provider.
    /// * `max_current_a` — hard current-limit magnitude (0 = unset). Caps the
    ///   brake intent; the driver's `clamp_targets` is still the real ceiling.
    /// * `vbus`, `ov_threshold_v` — for the proactive regen derate near OV
    ///   (`ov_threshold_v == 0` disables it; the OV fault is the backstop).
    /// * `angle_trustworthy` — false when the (sensorless) angle source has
    ///   lost lock; braking blind would mis-commutate, so coast instead.
    #[allow(clippy::too_many_arguments)] // inputs the driver already has on hand
    pub fn step(
        &mut self,
        omega_e: f32,
        dt: f32,
        cfg: &FailsafeConfig,
        max_current_a: f32,
        vbus: f32,
        ov_threshold_v: f32,
        angle_trustworthy: bool,
    ) -> FailsafeAction {
        let cap = if max_current_a > 0.0 {
            cfg.brake_current_a.min(max_current_a)
        } else {
            cfg.brake_current_a
        };
        // Slew the q-current by brake_current_a over ramp_s (A per cycle).
        let slew = (cfg.brake_current_a / cfg.ramp_s) * dt;

        match self.phase {
            Phase::Inactive | Phase::Done => FailsafeAction::Stop,

            Phase::RampDown { iq } => {
                let iq = ramp_toward(iq, 0.0, slew);
                if iq.abs() <= slew.max(1e-3) {
                    // Reached zero current.
                    if matches!(cfg.policy, FailsafePolicy::ControlledStop) && angle_trustworthy {
                        // Lock the brake to oppose the direction the rotor is
                        // spinning *now* — it won't change for the rest of the
                        // sequence.
                        self.phase = Phase::Brake {
                            iq: 0.0,
                            elapsed_s: 0.0,
                            standstill_s: 0.0,
                            brake_sign: -sign(omega_e),
                        };
                        FailsafeAction::Drive {
                            id_target: 0.0,
                            iq_target: 0.0,
                        }
                    } else {
                        // RampToZero, Coast-after-ramp, or no trustworthy angle
                        // to brake against: free-wheel from here.
                        self.phase = Phase::Done;
                        FailsafeAction::Stop
                    }
                } else {
                    self.phase = Phase::RampDown { iq };
                    FailsafeAction::Drive {
                        id_target: 0.0,
                        iq_target: iq,
                    }
                }
            }

            Phase::Brake {
                iq,
                elapsed_s,
                standstill_s,
                brake_sign,
            } => {
                // Sensorless angle dropped below its floor mid-brake -> coast
                // the rest rather than commutate on a stale angle.
                if !angle_trustworthy {
                    self.phase = Phase::Done;
                    return FailsafeAction::Stop;
                }

                let elapsed_s = elapsed_s + dt;
                let stopped = omega_e.abs() < cfg.standstill_rad_s;
                let standstill_s = if stopped { standstill_s + dt } else { 0.0 };
                // The estimate has crossed zero (now spinning the way the brake
                // torque points) — we've stopped/overshot; never drive past it.
                let reversed = omega_e * brake_sign > 0.0;

                // Terminate: reversed past zero, held standstill, or out of time.
                if reversed
                    || standstill_s >= STANDSTILL_DEBOUNCE_S
                    || elapsed_s >= cfg.brake_time_s
                {
                    self.phase = Phase::Done;
                    return FailsafeAction::Stop;
                }

                // Coast below the standstill threshold (the velocity estimate
                // is unreliable there); above it apply the full capped brake in
                // the fixed original-opposing direction. Constant + unidirectional
                // means no sign flip to pump a limit cycle across the noisy
                // low-speed estimate.
                let target = if stopped {
                    0.0
                } else {
                    derate_for_ov(brake_sign * cap, vbus, ov_threshold_v)
                };
                let iq = ramp_toward(iq, target, slew);

                self.phase = Phase::Brake {
                    iq,
                    elapsed_s,
                    standstill_s,
                    brake_sign,
                };
                FailsafeAction::Drive {
                    id_target: 0.0,
                    iq_target: iq,
                }
            }
        }
    }
}

/// Move `value` toward `target` by at most `step` (≥ 0).
#[inline]
fn ramp_toward(value: f32, target: f32, step: f32) -> f32 {
    let d = target - value;
    if d.abs() <= step {
        target
    } else {
        value + sign(d) * step
    }
}

/// Sign with a zero that returns 0 (so a stationary rotor commands no torque).
#[inline]
fn sign(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Linearly derate the brake target to zero across the top [`OV_DERATE_BAND`]
/// of the over-voltage window — regen pushes energy into the bus, so back off
/// as it approaches the OV trip instead of slamming into it.
#[inline]
fn derate_for_ov(target: f32, vbus: f32, ov_threshold_v: f32) -> f32 {
    if ov_threshold_v <= 0.0 {
        return target;
    }
    let start = (1.0 - OV_DERATE_BAND) * ov_threshold_v;
    if vbus <= start {
        return target;
    }
    let frac = clamp_f32((ov_threshold_v - vbus) / (ov_threshold_v - start), 0.0, 1.0);
    target * frac
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 20_000.0;

    fn cfg(policy: FailsafePolicy) -> FailsafeConfig {
        FailsafeConfig {
            policy,
            ..FailsafeConfig::default()
        }
    }

    /// Run the controller to completion against a constant ω_e, returning the
    /// last non-Stop iq target seen and how many cycles it took.
    fn run_to_done(
        ctrl: &mut FailsafeController,
        omega_e: f32,
        cfg: &FailsafeConfig,
        max_a: f32,
        trustworthy: bool,
        max_cycles: u32,
    ) -> (f32, u32) {
        let mut last_iq = 0.0;
        for n in 0..max_cycles {
            match ctrl.step(omega_e, DT, cfg, max_a, 0.0, 0.0, trustworthy) {
                FailsafeAction::Drive { iq_target, .. } => last_iq = iq_target,
                FailsafeAction::Stop => return (last_iq, n),
            }
        }
        (last_iq, max_cycles)
    }

    #[test]
    fn coast_stops_immediately() {
        let mut c = FailsafeController::new();
        c.arm(5.0, FailsafePolicy::Coast);
        assert!(c.is_active());
        assert_eq!(
            c.step(300.0, DT, &cfg(FailsafePolicy::Coast), 40.0, 0.0, 0.0, true),
            FailsafeAction::Stop
        );
    }

    #[test]
    fn ramp_to_zero_brings_iq_down_then_stops() {
        let mut c = FailsafeController::new();
        c.arm(8.0, FailsafePolicy::RampToZero);
        // First cycle still drives a (reduced) current, not Stop.
        match c.step(
            300.0,
            DT,
            &cfg(FailsafePolicy::RampToZero),
            40.0,
            0.0,
            0.0,
            true,
        ) {
            FailsafeAction::Drive { iq_target, .. } => assert!(iq_target < 8.0 && iq_target > 0.0),
            FailsafeAction::Stop => panic!("should ramp, not stop immediately"),
        }
        let (_iq, cycles) = run_to_done(
            &mut c,
            300.0,
            &cfg(FailsafePolicy::RampToZero),
            40.0,
            true,
            100_000,
        );
        // ~8 A at 150 A/s ≈ 53 ms ≈ 1067 cycles; never enters Brake.
        assert!(cycles > 100 && cycles < 5_000, "cycles {cycles}");
    }

    #[test]
    fn controlled_stop_brakes_negative_for_forward_rotation() {
        // ω_e > 0 → braking torque must be negative iq.
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop);
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut min_iq = 0.0f32;
        for _ in 0..2_000 {
            if let FailsafeAction::Drive { iq_target, .. } =
                c.step(300.0, DT, &cfg, 40.0, 0.0, 0.0, true)
            {
                min_iq = min_iq.min(iq_target);
            }
        }
        assert!(min_iq < -1.0, "expected negative brake iq, got {min_iq}");
        // Capped at min(brake_current, limit) = min(15, 40) = 15 A.
        assert!(min_iq >= -15.5, "brake exceeded cap: {min_iq}");
    }

    #[test]
    fn controlled_stop_brakes_positive_for_reverse_rotation() {
        // ω_e < 0 → braking torque must be positive iq (sign test).
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop);
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut max_iq = 0.0f32;
        for _ in 0..2_000 {
            if let FailsafeAction::Drive { iq_target, .. } =
                c.step(-300.0, DT, &cfg, 40.0, 0.0, 0.0, true)
            {
                max_iq = max_iq.max(iq_target);
            }
        }
        assert!(max_iq > 1.0, "expected positive brake iq, got {max_iq}");
    }

    #[test]
    fn controlled_stop_terminates_on_standstill() {
        // ω_e below the standstill threshold from the start → no real braking,
        // terminates after the debounce window.
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop);
        let (_iq, cycles) = run_to_done(
            &mut c,
            5.0, // < standstill_rad_s (20)
            &cfg(FailsafePolicy::ControlledStop),
            40.0,
            true,
            100_000,
        );
        // RampDown(~instant) + STANDSTILL_DEBOUNCE_S (50 ms = 1000 cycles).
        assert!(cycles > 500 && cycles < 2_000, "cycles {cycles}");
        // Terminal is stable: the controller stays in Done (returning Stop)
        // until the *driver* resets it on the Stop action.
        assert_eq!(
            c.step(
                5.0,
                DT,
                &cfg(FailsafePolicy::ControlledStop),
                40.0,
                0.0,
                0.0,
                true
            ),
            FailsafeAction::Stop
        );
    }

    #[test]
    fn controlled_stop_terminates_on_brake_timeout() {
        // Rotor never slows (infinite inertia in the harness) → give up after
        // brake_time_s.
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop);
        let mut cfg = cfg(FailsafePolicy::ControlledStop);
        cfg.brake_time_s = 0.5;
        let (_iq, cycles) = run_to_done(&mut c, 300.0, &cfg, 40.0, true, 100_000);
        // ~0.5 s = 10_000 cycles (+ ramp); generous bounds.
        assert!(cycles > 9_000 && cycles < 12_000, "cycles {cycles}");
    }

    #[test]
    fn controlled_stop_coasts_without_trustworthy_angle() {
        // Sensorless angle not trustworthy → must not brake blind; coast.
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop);
        let (_iq, cycles) = run_to_done(
            &mut c,
            300.0,
            &cfg(FailsafePolicy::ControlledStop),
            40.0,
            false, // not trustworthy
            100_000,
        );
        // RampDown reaches 0 then -> Done (no Brake phase); short.
        assert!(cycles < 200, "should coast quickly, took {cycles}");
    }

    #[test]
    fn ov_derate_reduces_brake_near_threshold() {
        // At vbus = ov_threshold, the derate must zero the brake target.
        assert_eq!(derate_for_ov(-15.0, 45.0, 45.0), 0.0);
        // Below the band, untouched.
        assert_eq!(derate_for_ov(-15.0, 30.0, 45.0), -15.0);
        // Mid-band: partial.
        let mid = derate_for_ov(-10.0, 0.5 * (45.0 + 0.9 * 45.0), 45.0);
        assert!(mid > -10.0 && mid < 0.0, "mid-band derate {mid}");
    }

    #[test]
    fn arm_is_idempotent_while_active() {
        let mut c = FailsafeController::new();
        c.arm(5.0, FailsafePolicy::ControlledStop);
        let p1 = c.phase;
        c.arm(0.0, FailsafePolicy::Coast); // ignored
        assert_eq!(c.phase, p1);
    }

    #[test]
    fn config_default_is_sane() {
        assert!(FailsafeConfig::default().is_sane());
        let bad = FailsafeConfig {
            ramp_s: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
    }

    #[test]
    #[cfg(feature = "storage")]
    fn from_stored_converts_units_and_falls_back() {
        use crate::storage::FailsafeConfigStored;

        // Default stored → the runtime default, with ms→µs/s conversion.
        let d = FailsafeConfig::from_stored(Some(&FailsafeConfigStored::default()));
        assert_eq!(d.policy, FailsafePolicy::ControlledStop);
        assert_eq!(d.staleness_timeout_us, 150_000);
        assert!((d.ramp_s - 0.1).abs() < 1e-6);
        assert!((d.brake_time_s - 3.0).abs() < 1e-6);

        // Missing → default.
        assert_eq!(
            FailsafeConfig::from_stored(None).policy,
            FailsafePolicy::ControlledStop
        );

        // Unknown policy byte → safest (Coast).
        let unknown = FailsafeConfigStored {
            policy: 99,
            ..FailsafeConfigStored::default()
        };
        assert_eq!(
            FailsafeConfig::from_stored(Some(&unknown)).policy,
            FailsafePolicy::Coast
        );

        // Non-sane stored (zero ramp) → full default fallback, never disabled.
        let bad = FailsafeConfigStored {
            ramp_ms: 0.0,
            ..FailsafeConfigStored::default()
        };
        assert_eq!(
            FailsafeConfig::from_stored(Some(&bad)),
            FailsafeConfig::default()
        );
    }
}
