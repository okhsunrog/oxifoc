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
//! zero, then **regen-brake to a standstill at a bounded deceleration** —
//! the velocity reference ramps to zero at [`FailsafeConfig::decel_rad_s2`]
//! through a dedicated [`VelocityLoop`] instance with fixed conservative
//! gains, so the stop feels the same on a slope as on the flat (a constant
//! *current* brake would not — gravity adds/subtracts). The same machinery
//! also serves the user-commanded ramp-into-parking-brake
//! (`FocDriver::enter_brake_ramp`).
//!
//! The loop instance is the failsafe's own — the host-tunable cruise loop in
//! `FocDriver` is never used here, so a mis-tuned cruise config cannot become
//! the link-loss safety net.
//!
//! Safety: the brake target is *intent* — it is routed back through the
//! normal current-control path in `FocDriver`, so the current-limit clamp and
//! the measured-overcurrent trip remain the last line of defense. A
//! regen-induced over-voltage latches the OverVoltage fault, which the
//! `run_foc_cycle` fault gate turns into high-Z (and resets this controller).

use crate::foc::clamp_f32;
use crate::foc::velocity::{VelocityLoop, VelocityLoopConfig};
#[cfg(feature = "storage")]
use crate::storage::FailsafeConfigStored;

/// Standstill must persist this long (s) before [`ControlledStop`] declares
/// the rotor stopped — rides out velocity noise / a momentary zero crossing.
///
/// [`ControlledStop`]: FailsafePolicy::ControlledStop
const STANDSTILL_DEBOUNCE_S: f32 = 0.05;

/// Top fraction of the OV window over which the regen brake current is
/// derated to zero — a soft landing instead of bang-bang into the OV fault.
const OV_DERATE_BAND: f32 = 0.1;

/// Give up (coast) if |ω| hasn't decreased by [`NO_PROGRESS_MARGIN_RAD_S`]
/// for this long — the velocity estimate is broken, or the brake physically
/// can't slow the vehicle (steep descent at the current cap). Deliberately
/// generous: a brake that *holds* speed on a hill is still better than a
/// coast, so only a sustained total lack of progress gives up. The hard cap
/// is [`FailsafeConfig::brake_time_s`].
const NO_PROGRESS_WINDOW_S: f32 = 2.0;

/// Minimum |ω| improvement (electrical rad/s) that counts as progress.
const NO_PROGRESS_MARGIN_RAD_S: f32 = 10.0;

/// Fixed gains for the failsafe's own velocity loop. Soft on purpose: the
/// hall velocity estimate only updates at edges, so the loop must not change
/// the speed much within one edge interval (see `foc::velocity`), and these
/// must be safe on *any* motor — the decel ramp does the shaping, the gains
/// only have to track it.
const BRAKE_VEL_KP: f32 = 0.01;
const BRAKE_VEL_KI: f32 = 0.2;

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
    /// Terminal: engage the parking brake (windings short) and leave the
    /// failsafe. Emitted **only** after a clean stop (standstill debounce or
    /// zero crossing) — the give-up paths (timeout, no progress, lost angle
    /// trust) always coast, never short the windings at speed.
    EngageBrake,
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
            1 => Self::RampToZero,
            2 => Self::ControlledStop,
            _ => Self::Coast,
        }
    }
}

/// What a clean [`ControlledStop`](FailsafePolicy::ControlledStop) leaves
/// behind. The give-up exits (timeout, no progress, lost angle trust) always
/// end high-Z regardless — the parking brake is never engaged at speed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FailsafeTerminal {
    /// Cut PWM (high-Z): the legacy behavior; the board can roll on a slope.
    HighZ = 0,
    /// Engage `ControlMode::Brake` (windings short): the stopped board
    /// resists rolling away, draws nothing at standstill.
    ParkBrake = 1,
}

impl FailsafeTerminal {
    /// Decode a stored `u8`; unknown falls back to the conservative high-Z.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::ParkBrake,
            _ => Self::HighZ,
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
    /// Regen-brake current cap (A); also clamped to the current limit at use.
    /// With the decel-ramped brake this is the *ceiling*, not the operating
    /// point — the loop only saturates here when the ramp can't be met
    /// (heavy load, downhill).
    pub brake_current_a: f32,
    /// Time (s) to slew the q-current by `brake_current_a` (the bumpless
    /// ramp-down from the last drive command).
    pub ramp_s: f32,
    /// Hard cap on the brake duration (s); the earlier give-up is the
    /// no-progress detector ([`NO_PROGRESS_WINDOW_S`]).
    pub brake_time_s: f32,
    /// |ω_e| (electrical rad/s) below which the rotor counts as stopped.
    pub standstill_rad_s: f32,
    /// Brake deceleration (electrical rad/s²): the velocity reference ramps
    /// to zero at this rate, so the felt deceleration is the same on a slope
    /// as on the flat — up to the current cap.
    pub decel_rad_s2: f32,
    /// What a clean stop leaves behind (high-Z vs parking brake).
    pub terminal: FailsafeTerminal,
}

impl Default for FailsafeConfig {
    /// Longboard default: brake to a controlled stop on link loss, then hold
    /// the parking brake so the stopped board doesn't roll away on a slope.
    fn default() -> Self {
        Self {
            policy: FailsafePolicy::ControlledStop,
            staleness_timeout_us: 150_000, // ≈3× the 50 ms host affirmation
            brake_current_a: 15.0,
            ramp_s: 0.1,
            brake_time_s: 10.0,
            standstill_rad_s: 20.0,
            decel_rad_s2: 1_000.0,
            terminal: FailsafeTerminal::ParkBrake,
        }
    }
}

impl FailsafeConfig {
    /// Build from the stored (host-writable) form: ms → µs/s, `policy: u8` →
    /// enum (unknown → Coast). A missing or non-sane stored value falls back
    /// to [`Default`] — a corrupt config can never disable the deadman.
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&FailsafeConfigStored>) -> Self {
        match cfg {
            Some(c) => {
                let candidate = Self {
                    policy: FailsafePolicy::from_u8(c.policy),
                    staleness_timeout_us: u64::from(c.staleness_timeout_ms) * 1_000,
                    brake_current_a: c.brake_current_a,
                    ramp_s: c.ramp_ms * 1e-3,
                    brake_time_s: c.brake_time_ms * 1e-3,
                    standstill_rad_s: c.standstill_rad_s,
                    decel_rad_s2: c.decel_rad_s2,
                    terminal: FailsafeTerminal::from_u8(c.terminal),
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

    /// Upper bound on the deadman staleness timeout (µs). The host affirms
    /// every 50 ms; anything beyond a few seconds is indistinguishable from
    /// "deadman off", which a config write must never be able to express.
    pub const MAX_STALENESS_TIMEOUT_US: u64 = 5_000_000;

    /// All numeric fields finite and in a usable range.
    ///
    /// `brake_current_a` must be strictly positive: it sets the RampDown
    /// slew rate, and a zero would freeze the ramp at the seeded current —
    /// a failsafe that drives the last commanded torque forever.
    pub fn is_sane(&self) -> bool {
        self.staleness_timeout_us > 0
            && self.staleness_timeout_us <= Self::MAX_STALENESS_TIMEOUT_US
            && self.brake_current_a.is_finite()
            && self.brake_current_a > 0.0
            && self.ramp_s.is_finite()
            && self.ramp_s > 0.0
            && self.brake_time_s.is_finite()
            && self.brake_time_s >= 0.0
            && self.standstill_rad_s.is_finite()
            && self.standstill_rad_s > 0.0
            && self.decel_rad_s2.is_finite()
            && self.decel_rad_s2 > 0.0
    }
}

/// Internal phase of the failsafe sequence.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Not engaged.
    Inactive,
    /// Slewing the q-current toward zero (bumpless from the last command).
    RampDown { iq: f32 },
    /// Regen-braking: the velocity reference ramps to zero at the configured
    /// deceleration through the failsafe's own velocity loop. The output is
    /// clamped to oppose the *original* rotation direction (`brake_sign` =
    /// −sign(ω) captured at entry) — unidirectional by design, so a lagging
    /// velocity estimate can't pump a limit cycle by driving past zero.
    Brake {
        elapsed_s: f32,
        standstill_s: f32,
        brake_sign: f32,
        /// Lowest |ω| seen so far (progress tracking).
        best_speed: f32,
        /// Time (s) since |ω| last improved by [`NO_PROGRESS_MARGIN_RAD_S`].
        no_progress_s: f32,
    },
    /// Clean stop reached — the driver applies the configured terminal.
    Stopped,
    /// Gave up (timeout / no progress / lost angle trust) — always high-Z.
    GaveUp,
}

/// Per-cycle failsafe state machine. Owned by `FocDriver`; armed by the
/// deadman / link-loss path (or a user ramp-into-brake), stepped every ISR
/// cycle until it reaches a terminal phase.
#[derive(Debug)]
pub struct FailsafeController {
    phase: Phase,
    /// What a clean stop leaves behind — from the config at arm time, or
    /// forced to ParkBrake for a user-commanded ramp-into-brake.
    terminal: FailsafeTerminal,
    /// The failsafe's own velocity loop (fixed conservative gains; the
    /// decel limit is set from the config at arm time).
    vel_loop: VelocityLoop,
}

impl Default for FailsafeController {
    fn default() -> Self {
        Self::new()
    }
}

impl FailsafeController {
    pub fn new() -> Self {
        Self {
            phase: Phase::Inactive,
            terminal: FailsafeTerminal::HighZ,
            vel_loop: VelocityLoop::new(VelocityLoopConfig {
                kp: BRAKE_VEL_KP,
                ki: BRAKE_VEL_KI,
                accel_limit: 0.0, // set from the config at arm time
                // No feedforward in the safety instance: fixed conservative
                // gains, no dependence on measured motor constants.
                accel_ff: 0.0,
            }),
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

    /// Change what a clean stop leaves behind, mid-sequence. Used when a
    /// user Brake command arrives while a stop is already in progress —
    /// `arm` is idempotent then, but the user's parking-brake intent must
    /// not be silently dropped. The give-up paths still always end high-Z.
    pub fn set_terminal(&mut self, terminal: FailsafeTerminal) {
        self.terminal = terminal;
    }

    /// Arm from the q-current currently being commanded (bumpless), the
    /// policy, and the terminal for a clean stop. Idempotent: a second arm
    /// while already active is ignored, so the deadman and link-loss paths
    /// can't restart a brake in progress.
    pub fn arm(&mut self, current_iq: f32, policy: FailsafePolicy, terminal: FailsafeTerminal) {
        if self.is_active() {
            return;
        }
        self.terminal = terminal;
        self.phase = match policy {
            // Coast: nothing to drive; always high-Z (a coast never ends at
            // standstill deliberately, so the parking-brake terminal does
            // not apply).
            FailsafePolicy::Coast => Phase::GaveUp,
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
            Phase::Inactive | Phase::GaveUp => FailsafeAction::Stop,
            Phase::Stopped => match self.terminal {
                FailsafeTerminal::HighZ => FailsafeAction::Stop,
                FailsafeTerminal::ParkBrake => FailsafeAction::EngageBrake,
            },

            Phase::RampDown { iq } => {
                let iq = ramp_toward(iq, 0.0, slew);
                if iq.abs() <= slew.max(1e-3) {
                    // Reached zero current.
                    if matches!(cfg.policy, FailsafePolicy::ControlledStop) && angle_trustworthy {
                        // Lock the brake to oppose the direction the rotor is
                        // spinning *now* — it won't change for the rest of the
                        // sequence — and seed the decel ramp at that speed.
                        self.vel_loop.set_config(VelocityLoopConfig {
                            kp: BRAKE_VEL_KP,
                            ki: BRAKE_VEL_KI,
                            accel_limit: cfg.decel_rad_s2,
                            accel_ff: 0.0,
                        });
                        self.vel_loop.reset(omega_e);
                        self.phase = Phase::Brake {
                            elapsed_s: 0.0,
                            standstill_s: 0.0,
                            brake_sign: -sign(omega_e),
                            best_speed: omega_e.abs(),
                            no_progress_s: 0.0,
                        };
                        FailsafeAction::Drive {
                            id_target: 0.0,
                            iq_target: 0.0,
                        }
                    } else {
                        // RampToZero, Coast-after-ramp, or no trustworthy angle
                        // to brake against: free-wheel from here.
                        self.phase = Phase::GaveUp;
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
                elapsed_s,
                standstill_s,
                brake_sign,
                best_speed,
                no_progress_s,
            } => {
                // Sensorless angle dropped below its floor mid-brake -> coast
                // the rest rather than commutate on a stale angle.
                if !angle_trustworthy {
                    self.phase = Phase::GaveUp;
                    return FailsafeAction::Stop;
                }

                let elapsed_s = elapsed_s + dt;
                let speed = omega_e.abs();
                let stopped = speed < cfg.standstill_rad_s;
                let standstill_s = if stopped { standstill_s + dt } else { 0.0 };
                // The estimate has crossed zero (now spinning the way the brake
                // torque points) — we've stopped/overshot; never drive past it.
                let reversed = omega_e * brake_sign > 0.0;

                // Clean stop: held standstill, or crossed zero.
                if reversed || standstill_s >= STANDSTILL_DEBOUNCE_S {
                    self.phase = Phase::Stopped;
                    return match self.terminal {
                        FailsafeTerminal::HighZ => FailsafeAction::Stop,
                        FailsafeTerminal::ParkBrake => FailsafeAction::EngageBrake,
                    };
                }

                // Progress watchdog: |ω| must keep coming down. A frozen
                // estimate or a descent the cap can't beat eventually gives
                // up (coast); a brake merely *holding* speed on a hill gets
                // the full window before that — better than coasting early.
                let (best_speed, no_progress_s) = if speed < best_speed - NO_PROGRESS_MARGIN_RAD_S {
                    (speed, 0.0)
                } else {
                    (best_speed, no_progress_s + dt)
                };

                // Give up: out of time or no progress — high-Z regardless of
                // the configured terminal (never short windings at speed).
                if elapsed_s >= cfg.brake_time_s || no_progress_s >= NO_PROGRESS_WINDOW_S {
                    self.phase = Phase::GaveUp;
                    return FailsafeAction::Stop;
                }

                // Decel-limited stop: the loop's internal reference ramps to
                // zero at cfg.decel_rad_s2; clamp the output unidirectional
                // (only oppose the original rotation) and inside the
                // OV-derated cap — regen pushes energy into the bus, so back
                // off approaching the trip instead of slamming into it.
                let eff_cap = derate_for_ov(cap, vbus, ov_threshold_v);
                let (iq_min, iq_max) = if brake_sign < 0.0 {
                    (-eff_cap, 0.0)
                } else {
                    (0.0, eff_cap)
                };
                let iq = self.vel_loop.step_clamped(0.0, omega_e, iq_min, iq_max, dt);

                self.phase = Phase::Brake {
                    elapsed_s,
                    standstill_s,
                    brake_sign,
                    best_speed,
                    no_progress_s,
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
            terminal: FailsafeTerminal::HighZ,
            ..FailsafeConfig::default()
        }
    }

    /// Run the controller to completion against a constant ω_e, returning the
    /// last Drive iq seen, how many cycles it took, and the terminal action.
    fn run_to_done(
        ctrl: &mut FailsafeController,
        omega_e: f32,
        cfg: &FailsafeConfig,
        max_a: f32,
        trustworthy: bool,
        max_cycles: u32,
    ) -> (f32, u32, FailsafeAction) {
        let mut last_iq = 0.0;
        for n in 0..max_cycles {
            match ctrl.step(omega_e, DT, cfg, max_a, 0.0, 0.0, trustworthy) {
                FailsafeAction::Drive { iq_target, .. } => last_iq = iq_target,
                terminal => return (last_iq, n, terminal),
            }
        }
        (last_iq, max_cycles, FailsafeAction::Stop)
    }

    #[test]
    fn coast_stops_immediately() {
        let mut c = FailsafeController::new();
        c.arm(5.0, FailsafePolicy::Coast, FailsafeTerminal::HighZ);
        assert!(c.is_active());
        assert_eq!(
            c.step(300.0, DT, &cfg(FailsafePolicy::Coast), 40.0, 0.0, 0.0, true),
            FailsafeAction::Stop
        );
    }

    #[test]
    fn coast_never_engages_park_brake() {
        // Even with the ParkBrake terminal configured, Coast gives up to
        // high-Z — it never deliberately reaches standstill.
        let mut c = FailsafeController::new();
        c.arm(5.0, FailsafePolicy::Coast, FailsafeTerminal::ParkBrake);
        assert_eq!(
            c.step(300.0, DT, &cfg(FailsafePolicy::Coast), 40.0, 0.0, 0.0, true),
            FailsafeAction::Stop
        );
    }

    #[test]
    fn ramp_to_zero_brings_iq_down_then_stops() {
        let mut c = FailsafeController::new();
        c.arm(8.0, FailsafePolicy::RampToZero, FailsafeTerminal::HighZ);
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
            other => panic!("should ramp, not terminate immediately: {other:?}"),
        }
        let (_iq, cycles, terminal) = run_to_done(
            &mut c,
            300.0,
            &cfg(FailsafePolicy::RampToZero),
            40.0,
            true,
            100_000,
        );
        // ~8 A at 150 A/s ≈ 53 ms ≈ 1067 cycles; never enters Brake.
        assert!(cycles > 100 && cycles < 5_000, "cycles {cycles}");
        assert_eq!(terminal, FailsafeAction::Stop);
    }

    #[test]
    fn controlled_stop_brakes_negative_for_forward_rotation() {
        // ω_e > 0 → braking torque must be negative iq, and never positive
        // (unidirectional clamp).
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut min_iq = 0.0f32;
        let mut max_iq = 0.0f32;
        for _ in 0..4_000 {
            if let FailsafeAction::Drive { iq_target, .. } =
                c.step(300.0, DT, &cfg, 40.0, 0.0, 0.0, true)
            {
                min_iq = min_iq.min(iq_target);
                max_iq = max_iq.max(iq_target);
            }
        }
        assert!(min_iq < -1.0, "expected negative brake iq, got {min_iq}");
        // Capped at min(brake_current, limit) = min(15, 40) = 15 A.
        assert!(min_iq >= -15.5, "brake exceeded cap: {min_iq}");
        assert!(max_iq <= 1e-3, "must never drive forward: {max_iq}");
    }

    #[test]
    fn controlled_stop_brakes_positive_for_reverse_rotation() {
        // ω_e < 0 → braking torque must be positive iq (sign test).
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut max_iq = 0.0f32;
        for _ in 0..4_000 {
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
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let (_iq, cycles, terminal) = run_to_done(
            &mut c,
            5.0, // < standstill_rad_s (20)
            &cfg(FailsafePolicy::ControlledStop),
            40.0,
            true,
            100_000,
        );
        // RampDown(~instant) + STANDSTILL_DEBOUNCE_S (50 ms = 1000 cycles).
        assert!(cycles > 500 && cycles < 2_000, "cycles {cycles}");
        assert_eq!(terminal, FailsafeAction::Stop);
        // Terminal is stable: the controller stays terminal (returning the
        // same action) until the *driver* resets it.
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
    fn park_brake_terminal_engages_on_clean_stop_only() {
        // Clean stop with the ParkBrake terminal → EngageBrake, stable.
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let (_iq, _cycles, terminal) = run_to_done(&mut c, 5.0, &cfg, 40.0, true, 100_000);
        assert_eq!(terminal, FailsafeAction::EngageBrake);
        assert_eq!(
            c.step(5.0, DT, &cfg, 40.0, 0.0, 0.0, true),
            FailsafeAction::EngageBrake
        );

        // Give-up path (constant speed, hard time cap) → Stop (high-Z), the
        // windings are never shorted at speed regardless of the terminal.
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let mut short_cap = cfg;
        short_cap.brake_time_s = 0.5;
        let (_iq, _cycles, terminal) = run_to_done(&mut c, 300.0, &short_cap, 40.0, true, 100_000);
        assert_eq!(terminal, FailsafeAction::Stop);
    }

    #[test]
    fn controlled_stop_terminates_on_brake_timeout() {
        // Rotor never slows (infinite inertia in the harness) → the hard time
        // cap fires (set below the no-progress window here).
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let mut cfg = cfg(FailsafePolicy::ControlledStop);
        cfg.brake_time_s = 0.5;
        let (_iq, cycles, terminal) = run_to_done(&mut c, 300.0, &cfg, 40.0, true, 100_000);
        // ~0.5 s = 10_000 cycles (+ ramp); generous bounds.
        assert!(cycles > 9_000 && cycles < 12_000, "cycles {cycles}");
        assert_eq!(terminal, FailsafeAction::Stop);
    }

    #[test]
    fn controlled_stop_gives_up_when_no_progress() {
        // |ω| never improves (broken estimate / un-brakeable descent) → the
        // no-progress watchdog coasts after NO_PROGRESS_WINDOW_S, well before
        // the 10 s hard cap.
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let cfg = cfg(FailsafePolicy::ControlledStop); // brake_time_s = 10
        let (_iq, cycles, terminal) = run_to_done(&mut c, 300.0, &cfg, 40.0, true, 300_000);
        let expected = (NO_PROGRESS_WINDOW_S / DT) as u32;
        assert!(
            cycles > expected - 2_000 && cycles < expected + 4_000,
            "cycles {cycles}, expected ≈{expected}"
        );
        // Gave up at speed → high-Z, not the parking brake.
        assert_eq!(terminal, FailsafeAction::Stop);
    }

    #[test]
    fn controlled_stop_coasts_without_trustworthy_angle() {
        // Sensorless angle not trustworthy → must not brake blind; coast.
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let (_iq, cycles, terminal) = run_to_done(
            &mut c,
            300.0,
            &cfg(FailsafePolicy::ControlledStop),
            40.0,
            false, // not trustworthy
            100_000,
        );
        // RampDown reaches 0 then gives up (no Brake phase); short.
        assert!(cycles < 200, "should coast quickly, took {cycles}");
        assert_eq!(terminal, FailsafeAction::Stop);
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
        c.arm(5.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let p1 = c.phase;
        c.arm(0.0, FailsafePolicy::Coast, FailsafeTerminal::ParkBrake); // ignored
        assert_eq!(c.phase, p1);
        assert_eq!(c.terminal, FailsafeTerminal::HighZ);
    }

    #[test]
    fn config_default_is_sane() {
        assert!(FailsafeConfig::default().is_sane());
        let bad = FailsafeConfig {
            ramp_s: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
        let bad = FailsafeConfig {
            decel_rad_s2: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
        // Zero brake current would freeze RampDown at the seeded iq (slew
        // = brake/ramp_s = 0) — a failsafe driving the last commanded
        // torque forever. Must be rejected outright.
        let bad = FailsafeConfig {
            brake_current_a: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
        // A staleness timeout beyond the cap is "deadman off" in disguise.
        let bad = FailsafeConfig {
            staleness_timeout_us: FailsafeConfig::MAX_STALENESS_TIMEOUT_US + 1,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
    }

    /// A user Brake command while a stop is already running must not be
    /// silently dropped: the terminal switches to ParkBrake mid-sequence.
    #[test]
    fn set_terminal_updates_running_sequence() {
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        c.set_terminal(FailsafeTerminal::ParkBrake);
        // Run to a clean stop from standstill speed: must end in the brake.
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let (_iq, _cycles, terminal) = run_to_done(&mut c, 5.0, &cfg, 40.0, true, 100_000);
        assert_eq!(terminal, FailsafeAction::EngageBrake);
    }

    #[test]
    #[cfg(feature = "storage")]
    fn from_stored_converts_units_and_falls_back() {
        use crate::storage::FailsafeConfigStored;

        // Default stored → the runtime default, with ms→µs/s conversion.
        let d = FailsafeConfig::from_stored(Some(&FailsafeConfigStored::default()));
        assert_eq!(d, FailsafeConfig::default());
        assert_eq!(d.staleness_timeout_us, 150_000);
        assert!((d.ramp_s - 0.1).abs() < 1e-6);
        assert!((d.brake_time_s - 10.0).abs() < 1e-6);
        assert_eq!(d.terminal, FailsafeTerminal::ParkBrake);

        // Missing → default.
        assert_eq!(
            FailsafeConfig::from_stored(None).policy,
            FailsafePolicy::ControlledStop
        );

        // Unknown policy/terminal bytes → safest (Coast / HighZ).
        let unknown = FailsafeConfigStored {
            policy: 99,
            terminal: 99,
            ..FailsafeConfigStored::default()
        };
        let decoded = FailsafeConfig::from_stored(Some(&unknown));
        assert_eq!(decoded.policy, FailsafePolicy::Coast);
        assert_eq!(decoded.terminal, FailsafeTerminal::HighZ);

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
