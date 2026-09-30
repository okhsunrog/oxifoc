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
//! [`ControlledStop`](FailsafePolicy::ControlledStop): **regen-brake to a
//! standstill**. The actuator is the brake CURRENT — proportional to the
//! force on the wheel whatever the rider weighs or the slope does — so the
//! stop is shaped in amps:
//!
//! - the q current moves at [`FailsafeConfig::slew_a_per_s`] from whatever
//!   was flowing (a drive command, the rider's own brake) toward the brake,
//!   in both directions — no step anywhere, including the hand-over;
//! - the brake settles at [`FailsafeConfig::brake_current_a`] (within the
//!   general `max_brake_a`), or at the rider's harder brake if one was in
//!   progress — the stop never weakens a brake it took over;
//! - [`FailsafeConfig::decel_rad_s2`] only bounds the stop from above (a
//!   light rotor under a strong brake);
//! - it brakes the way the rotor moves when the stop begins (a board
//!   already rolling back is braked back);
//! - near standstill the active brake stands down and the parking brake —
//!   the passive windings short, which opposes motion either way without
//!   an angle or a speed estimate — takes over and holds a roll-back
//!   (turning current-controlled if pushed past its limit, see
//!   `FocDriver`'s parking brake);
//! - losing the angle mid-stop (sensorless below the observer's floor)
//!   hands over to the parking brake too, which needs no angle.
//!
//! While the rotor moves and the angle is trustworthy, the stop never
//! releases the brake on its own: a brake that cannot slow a descent still
//! beats a coast. The same machinery also serves the user-commanded
//! ramp-into-parking-brake (`FocDriver::enter_brake_ramp`).
//!
//! Safety: the brake target is *intent* — it is routed back through the
//! normal current-control path in `FocDriver`, so the current-limit clamp and
//! the measured-overcurrent trip remain the last line of defense. A
//! regen-induced over-voltage latches the OverVoltage fault, which the
//! `run_foc_cycle` fault gate turns into high-Z (and resets this controller).

use crate::foc::clamp_f32;
#[cfg(feature = "storage")]
use crate::storage::FailsafeConfigStored;

/// Standstill must persist this long (s) before [`ControlledStop`] declares
/// the rotor stopped — rides out velocity noise / a momentary zero crossing.
///
/// [`ControlledStop`]: FailsafePolicy::ControlledStop
const STANDSTILL_DEBOUNCE_S: f32 = 0.05;

/// A reversal read above the parking-brake gate (an estimate jump) must
/// persist this long (s) before the brake turns around; a one-sample glitch
/// never gets there.
const REVERSAL_DEBOUNCE_S: f32 = 0.05;

/// Low-pass time constant (s) of the deceleration estimate (the derivative
/// of the velocity estimate — hall velocity steps at every edge).
const DECEL_FILTER_TAU_S: f32 = 0.05;

/// The deceleration bound acts once the measured deceleration passes this
/// fraction of it; below, the current cap alone shapes the stop.
const DECEL_BOUND_ENGAGE: f32 = 0.25;

/// For this long (s) after arming, the measured deceleration raises the
/// deceleration bound instead of being held to it: the rider's own brake
/// in progress is taken over, never softened by the bound.
const TAKEOVER_WINDOW_S: f32 = 0.1;

/// Top fraction of the OV window over which the regen brake current is
/// derated to zero — a soft landing instead of bang-bang into the OV fault.
const OV_DERATE_BAND: f32 = 0.1;

/// Maximum |ω_e| (electrical rad/s) at which the windings may be shorted
/// (`ControlMode::Brake`, the parking brake) — above this the short-circuit
/// current is governed by back-EMF against the motor impedance, outside any
/// control loop. Gates both the user `Brake` command and the failsafe's
/// parking-brake terminal. Kept a little above the failsafe standstill
/// threshold (20 rad/s default) so a rider standing next to the board can
/// always engage it. Bench-tune.
pub const BRAKE_ENTRY_MAX_E_RAD_S: f32 = 50.0;

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
    /// failsafe. Emitted **only** after a clean stop at a speed the short
    /// can take (see [`BRAKE_ENTRY_MAX_E_RAD_S`]).
    EngageBrake,
}

/// Configurable reaction to a stale command link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FailsafePolicy {
    /// Free-wheel: cut PWM (high-Z). Matches the legacy link-loss behavior.
    Coast = 0,
    /// Slew the q-current to zero, then free-wheel.
    RampToZero = 1,
    /// Regen-brake to a standstill, then apply the configured terminal.
    ///
    /// The current brake runs while the angle is trustworthy (Hall, HFI,
    /// the back-EMF observer above its speed floor). When the angle drops
    /// out — a pure observer below its floor — the terminal takes over
    /// early: the parking brake's short needs no angle (`ParkBrake`),
    /// high-Z coasts the rest.
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

/// What a [`ControlledStop`](FailsafePolicy::ControlledStop) leaves behind:
/// at standstill, or when the angle is lost (the parking brake's short needs
/// none, and guards itself by its current — see `FocDriver`'s parking
/// brake).
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
    /// Brake current of the stop (A); the general `max_brake_a` still
    /// bounds it. `0` = the general brake limit alone. A harder brake the
    /// rider had in progress is kept.
    pub brake_current_a: f32,
    /// Rate (A/s) at which the stop changes the q current: building the
    /// brake, handing over from the last command, turning it around,
    /// fading it out. The force on the wheel moves at this rate — what the
    /// rider feels as the brake coming on.
    pub slew_a_per_s: f32,
    /// |ω_e| (electrical rad/s) below which the rotor counts as stopped.
    pub standstill_rad_s: f32,
    /// Upper bound on the stop's deceleration (electrical rad/s²): the brake
    /// current backs off when the rotor slows faster (a light rotor under a
    /// strong brake). Not a target — with a rider the current usually binds
    /// first.
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
            slew_a_per_s: 150.0,
            standstill_rad_s: 20.0,
            decel_rad_s2: 1_000.0,
            terminal: FailsafeTerminal::ParkBrake,
        }
    }
}

impl FailsafeConfig {
    /// Build from the stored (host-writable) form: ms → µs, `policy: u8` →
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
                    slew_a_per_s: c.slew_a_per_s,
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
    /// `slew_a_per_s` must be strictly positive: a zero would freeze the
    /// q current where the failsafe found it — a stop that drives the last
    /// commanded torque forever.
    pub fn is_sane(&self) -> bool {
        self.staleness_timeout_us > 0
            && self.staleness_timeout_us <= Self::MAX_STALENESS_TIMEOUT_US
            && self.brake_current_a.is_finite()
            && self.brake_current_a >= 0.0
            && self.slew_a_per_s.is_finite()
            && self.slew_a_per_s > 0.0
            && self.standstill_rad_s.is_finite()
            && self.standstill_rad_s > 0.0
            && self.decel_rad_s2.is_finite()
            && self.decel_rad_s2 > 0.0
    }
}

/// Where the failsafe sequence is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Not engaged.
    Inactive,
    /// RampToZero: slewing the q-current to zero, then high-Z.
    RampToZero,
    /// ControlledStop: braking whichever way the rotor moves.
    Brake,
    /// Clean stop reached — the driver applies the configured terminal.
    Stopped,
    /// Gave up (lost angle trust with the high-Z terminal, or a coast
    /// policy) — high-Z.
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
    /// The q current being commanded (A), slewed.
    iq: f32,
    /// Direction of motion being braked (±1; 0 until the first brake step
    /// has seen the rotor).
    brake_dir: f32,
    /// Brake magnitude taken over from the rider at arm time (A).
    takeover_a: f32,
    /// Time since arming (s).
    since_arm_s: f32,
    /// Time the rotor has spent below the standstill threshold (s).
    standstill_s: f32,
    /// Time the rotor has moved against `brake_dir` (s).
    reverse_s: f32,
    /// Previous velocity sample, for the deceleration estimate.
    omega_prev: Option<f32>,
    /// Low-passed dω/dt (electrical rad/s²).
    accel_filt: f32,
    /// Low-passed brake current along the braked direction (A), filtered
    /// like `accel_filt`.
    brake_filt: f32,
    /// Measured deceleration per amp of brake (el rad/s² / A; 0 = not
    /// measured yet).
    decel_per_a: f32,
    /// Keep applying the deceleration bound after a turn-around until it
    /// is measured afresh in the new direction.
    hold_bound: bool,
    /// Deceleration bound in force (el rad/s²): the configured one, raised
    /// by the rider's harder brake during the takeover window.
    decel_limit: f32,
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
            iq: 0.0,
            brake_dir: 0.0,
            takeover_a: 0.0,
            since_arm_s: 0.0,
            standstill_s: 0.0,
            reverse_s: 0.0,
            omega_prev: None,
            accel_filt: 0.0,
            brake_filt: 0.0,
            decel_per_a: 0.0,
            hold_bound: false,
            decel_limit: 0.0,
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
    /// not be silently dropped.
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
        *self = Self {
            phase: match policy {
                // Coast: nothing to drive; always high-Z (a coast never ends
                // at standstill deliberately, so the parking-brake terminal
                // does not apply).
                FailsafePolicy::Coast => Phase::GaveUp,
                FailsafePolicy::RampToZero => Phase::RampToZero,
                FailsafePolicy::ControlledStop => Phase::Brake,
            },
            terminal,
            iq: current_iq,
            ..Self::new()
        };
    }

    /// One ISR cycle.
    ///
    /// * `omega_e` — electrical velocity (rad/s) from the phase provider.
    /// * `max_brake_a` — the general braking-current limit (0 = unset).
    ///   Bounds the stop; the driver's clamps are still the real ceiling.
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
        max_brake_a: f32,
        vbus: f32,
        ov_threshold_v: f32,
        angle_trustworthy: bool,
    ) -> FailsafeAction {
        let slew = cfg.slew_a_per_s * dt;
        match self.phase {
            Phase::Inactive | Phase::GaveUp => FailsafeAction::Stop,
            Phase::Stopped => self.terminal_action(omega_e),
            Phase::RampToZero => {
                self.iq = ramp_toward(self.iq, 0.0, slew);
                if self.iq == 0.0 {
                    self.phase = Phase::GaveUp;
                    return FailsafeAction::Stop;
                }
                drive(self.iq)
            }
            Phase::Brake => {
                // Without a trustworthy angle a q current is torque in an
                // unknown direction. The parking brake's short needs no
                // angle — typically the sensorless observer just dropped
                // below its speed floor, where the short carries ~15 A on a
                // 5065 — and releases to high-Z by itself if its current
                // says the rotor is faster than it may take. Without that
                // terminal: coast.
                if !angle_trustworthy {
                    return if self.terminal == FailsafeTerminal::ParkBrake {
                        self.phase = Phase::Stopped;
                        FailsafeAction::EngageBrake
                    } else {
                        self.phase = Phase::GaveUp;
                        FailsafeAction::Stop
                    };
                }
                self.brake_step(omega_e, dt, cfg, max_brake_a, vbus, ov_threshold_v)
            }
        }
    }

    fn brake_step(
        &mut self,
        omega_e: f32,
        dt: f32,
        cfg: &FailsafeConfig,
        max_brake_a: f32,
        vbus: f32,
        ov_threshold_v: f32,
    ) -> FailsafeAction {
        let speed = omega_e.abs();
        let motion = sign(omega_e);
        if self.omega_prev.is_none() {
            // First look at the rotor: brake the way it moves, and keep a
            // brake the rider already had on (iq opposing the motion).
            self.brake_dir = motion;
            self.takeover_a = if self.iq * omega_e < 0.0 {
                self.iq.abs()
            } else {
                0.0
            };
            self.decel_limit = cfg.decel_rad_s2;
        }
        self.since_arm_s += dt;

        // Deceleration of the braked motion and the brake current that
        // acted over the same interval, low-passed alike (so their ratio
        // carries no filter lag).
        if let Some(prev) = self.omega_prev
            && dt > 0.0
        {
            let a = (dt / DECEL_FILTER_TAU_S).min(1.0);
            self.accel_filt += a * ((omega_e - prev) / dt - self.accel_filt);
            self.brake_filt += a * (-self.iq * self.brake_dir - self.brake_filt);
        }
        self.omega_prev = Some(omega_e);
        let decel = -self.accel_filt * self.brake_dir;
        // Taking over the rider's brake: hold it for the window (below) and
        // let the deceleration it produces raise the bound, so the bound
        // never softens it afterwards. Our own brake must not raise it.
        let taking_over = self.takeover_a > 0.0 && self.since_arm_s < TAKEOVER_WINDOW_S;
        if taking_over {
            self.decel_limit = self.decel_limit.max(decel);
        }

        // Near standstill the active brake stands down and the parking
        // brake takes over. The velocity estimate lags there (hall: tens of
        // el rad/s behind at a reversal), and a q current commanded on it
        // pushed the rotor through zero and pumped a light one into a
        // ±700 el rad/s limit cycle. The windings short opposes any motion
        // without an angle or a speed estimate — VESC shorts the phases
        // around zero for the same reason — and holds a roll-back the same
        // way.
        let slow = speed < cfg.standstill_rad_s;
        self.standstill_s = if slow { self.standstill_s + dt } else { 0.0 };
        let crossed = motion != 0.0 && motion != self.brake_dir;
        // Clean stop: held standstill with the brake released, or a zero
        // crossing read at a speed the short can take.
        if (self.standstill_s >= STANDSTILL_DEBOUNCE_S && self.iq == 0.0)
            || (crossed && speed <= BRAKE_ENTRY_MAX_E_RAD_S)
        {
            self.phase = Phase::Stopped;
            return self.terminal_action(omega_e);
        }

        // A reversal read above that is an estimate jump (a sampled rotor
        // crosses zero slowly). While it lasts, the old brake would drive
        // the rotor: release. If it persists, brake the new direction, held
        // to the deceleration bound measured so far — the rotor's mass
        // does not change with direction.
        if crossed {
            self.reverse_s += dt;
            if self.reverse_s >= REVERSAL_DEBOUNCE_S || self.brake_dir == 0.0 {
                self.brake_dir = motion;
                self.reverse_s = 0.0;
                // Both filters straddle the jump; start them afresh.
                self.brake_filt = 0.0;
                self.accel_filt = 0.0;
                self.hold_bound = self.decel_per_a > 0.0;
            }
        } else {
            self.reverse_s = 0.0;
        }
        let against = motion != 0.0 && motion != self.brake_dir;

        // Brake magnitude: the configured stop current (or the general
        // limit), never below a brake taken over from the rider, never
        // above the general limit; derated approaching OV.
        let general = if max_brake_a > 0.0 {
            max_brake_a
        } else {
            f32::INFINITY
        };
        let configured = if cfg.brake_current_a > 0.0 {
            cfg.brake_current_a
        } else {
            general
        };
        let cap = derate_for_ov(
            configured.max(self.takeover_a).min(general),
            vbus,
            ov_threshold_v,
        );
        let mut magnitude = if cap.is_finite() && !against && !slow {
            cap
        } else {
            0.0
        };
        // Deceleration bound, once the rotor decelerates noticeably: brake
        // current = bound / (deceleration per amp). The per-amp figure is
        // measured with brake and deceleration filtered alike (no lag in
        // the ratio) and it includes the slope, so the fixed point is
        // exactly the bound. Far below the bound (a loaded board) the
        // current cap governs.
        let bound_engaged = !taking_over && decel > DECEL_BOUND_ENGAGE * self.decel_limit;
        // (Not across a reversal read: the estimate's jump is no
        // deceleration the brake produced.)
        if bound_engaged && !crossed && self.brake_filt > 0.0 {
            self.decel_per_a = decel / self.brake_filt;
            self.hold_bound = false;
        }
        if taking_over {
            magnitude = magnitude.min(self.takeover_a);
        } else if (bound_engaged || self.hold_bound) && self.decel_per_a > 0.0 {
            magnitude = magnitude.min(self.decel_limit / self.decel_per_a);
        }

        self.iq = ramp_toward(self.iq, -self.brake_dir * magnitude, cfg.slew_a_per_s * dt);
        drive(self.iq)
    }

    /// What a clean stop hands over to. The parking brake shorts the
    /// windings, so it is only engaged below [`BRAKE_ENTRY_MAX_E_RAD_S`]
    /// even when a host-configured standstill threshold sits above it.
    fn terminal_action(&self, omega_e: f32) -> FailsafeAction {
        match self.terminal {
            FailsafeTerminal::ParkBrake if omega_e.abs() <= BRAKE_ENTRY_MAX_E_RAD_S => {
                FailsafeAction::EngageBrake
            }
            _ => FailsafeAction::Stop,
        }
    }
}

/// A q-current-only command.
#[inline]
fn drive(iq: f32) -> FailsafeAction {
    FailsafeAction::Drive {
        id_target: 0.0,
        iq_target: iq,
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

    /// A rotor the failsafe current acts on: dω/dt = k·iq + load, with k in
    /// el rad/s² per A (≈ 8.7 for a loaded one-motor 5065 board, thousands
    /// for a bare bench rotor) and `load` the slope's pull (el rad/s², along
    /// +ω). Runs `secs` or until a terminal action; `probe` sees every
    /// cycle. Returns the final ω and the last action.
    #[allow(clippy::too_many_arguments)]
    fn ride(
        c: &mut FailsafeController,
        cfg: &FailsafeConfig,
        max_brake_a: f32,
        omega0: f32,
        k: f32,
        load: f32,
        secs: f32,
        mut probe: impl FnMut(f32, FailsafeAction),
    ) -> (f32, FailsafeAction) {
        let mut omega = omega0;
        let mut last = FailsafeAction::Stop;
        for _ in 0..(secs / DT) as u32 {
            last = c.step(omega, DT, cfg, max_brake_a, 0.0, 0.0, true);
            probe(omega, last);
            match last {
                FailsafeAction::Drive { iq_target, .. } => omega += (k * iq_target + load) * DT,
                _ => break,
            }
        }
        (omega, last)
    }

    fn iq_of(a: FailsafeAction) -> f32 {
        match a {
            FailsafeAction::Drive { iq_target, .. } => iq_target,
            other => panic!("expected Drive, got {other:?}"),
        }
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
    fn ramp_to_zero_slews_at_the_configured_rate() {
        let cfg = cfg(FailsafePolicy::RampToZero);
        let mut c = FailsafeController::new();
        c.arm(8.0, FailsafePolicy::RampToZero, FailsafeTerminal::HighZ);
        let mut cycles = 0u32;
        let (_, last) = ride(&mut c, &cfg, 40.0, 300.0, 0.0, 0.0, 1.0, |_, _| cycles += 1);
        assert_eq!(last, FailsafeAction::Stop);
        // 8 A at 150 A/s = 53 ms = 1067 cycles.
        let expected = (8.0 / cfg.slew_a_per_s / DT) as u32;
        assert!(cycles.abs_diff(expected) <= 2, "{cycles} vs {expected}");
    }

    /// From a drive command the stop crosses zero into the brake at the
    /// configured rate — no step anywhere — and settles at the stop current.
    #[test]
    fn stop_builds_the_brake_at_the_slew_rate() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            10.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::HighZ,
        );
        let (mut prev, mut max_step, mut last_iq) = (10.0f32, 0.0f32, 0.0);
        ride(&mut c, &cfg, 40.0, 500.0, 0.0, 0.0, 0.5, |_, a| {
            last_iq = iq_of(a);
            max_step = max_step.max((last_iq - prev).abs());
            prev = last_iq;
        });
        assert!(
            max_step <= cfg.slew_a_per_s * DT * 1.001,
            "step {max_step} A"
        );
        assert!(
            (last_iq + cfg.brake_current_a).abs() < 1e-3,
            "settled at {last_iq}"
        );
    }

    /// A rotor spinning backward is braked with positive q current.
    #[test]
    fn stop_brakes_reverse_rotation_with_positive_current() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let mut last_iq = 0.0;
        ride(&mut c, &cfg, 40.0, -500.0, 0.0, 0.0, 0.3, |_, a| {
            last_iq = iq_of(a);
        });
        assert!((last_iq - cfg.brake_current_a).abs() < 1e-3, "{last_iq}");
    }

    /// The rider was braking harder than the stop current when the link
    /// died: the stop keeps that brake instead of easing off to its own.
    #[test]
    fn stop_takes_over_a_harder_brake_without_weakening() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            -25.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::HighZ,
        );
        let mut weakest = f32::INFINITY;
        // A loaded board: the rider's 25 A decelerates ~220 el rad/s².
        ride(&mut c, &cfg, 40.0, 1_500.0, 8.7, 0.0, 1.0, |omega, a| {
            if omega > cfg.standstill_rad_s {
                weakest = weakest.min(-iq_of(a));
            }
        });
        assert!(weakest >= 24.999, "brake eased to {weakest} A");
    }

    /// ...but never beyond the general brake limit: a brake above it is
    /// brought down to it at the configured rate, not in one step.
    #[test]
    fn stop_brings_a_brake_above_the_limit_down_smoothly() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            -25.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::HighZ,
        );
        let (mut prev, mut max_step, mut last_iq) = (-25.0f32, 0.0f32, 0.0);
        ride(&mut c, &cfg, 20.0, 1_500.0, 0.0, 0.0, 0.3, |_, a| {
            last_iq = iq_of(a);
            max_step = max_step.max((last_iq - prev).abs());
            prev = last_iq;
        });
        assert!(
            max_step <= cfg.slew_a_per_s * DT * 1.001,
            "step {max_step} A"
        );
        assert!((last_iq + 20.0).abs() < 1e-3, "settled at {last_iq}");
    }

    /// A descent the brake cannot beat: the board keeps speeding up, and the
    /// stop keeps braking — it used to give up to a coast after 2 s.
    #[test]
    fn stop_keeps_braking_a_descent_it_cannot_beat() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        // 15 A × 8.7 = 130 el rad/s² of brake against 200 of slope.
        let (omega, last) = ride(&mut c, &cfg, 40.0, 1_000.0, 8.7, 200.0, 20.0, |_, _| {});
        assert!(omega > 1_000.0, "the slope wins: {omega}");
        assert!((iq_of(last) + cfg.brake_current_a).abs() < 1e-3, "{last:?}");
    }

    /// `brake_current_a = 0`: the stop uses the general brake limit.
    #[test]
    fn zero_stop_current_means_the_general_limit() {
        let cfg = FailsafeConfig {
            brake_current_a: 0.0,
            ..cfg(FailsafePolicy::ControlledStop)
        };
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        let mut last_iq = 0.0;
        ride(&mut c, &cfg, 32.0, 1_000.0, 0.0, 0.0, 0.5, |_, a| {
            last_iq = iq_of(a);
        });
        assert!((last_iq + 32.0).abs() < 1e-3, "{last_iq}");
    }

    /// A light rotor under the stop current would decelerate far past the
    /// bound: the current backs off to hold it.
    #[test]
    fn deceleration_bound_softens_a_violent_stop() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        // 15 A × 500 = 7500 el rad/s² unbounded.
        let (mut prev_omega, mut worst) = (3_000.0f32, 0.0f32);
        let mut t = 0.0f32;
        ride(&mut c, &cfg, 40.0, 3_000.0, 500.0, 0.0, 3.0, |omega, _| {
            t += DT;
            if t > 0.3 && omega > 200.0 {
                worst = worst.max((prev_omega - omega) / DT);
            }
            prev_omega = omega;
        });
        assert!(
            worst < 1.3 * cfg.decel_rad_s2,
            "decelerated at {worst} el rad/s² (bound {})",
            cfg.decel_rad_s2
        );
    }

    /// Stopped by a slope that then pulls it back: the zero crossing hands
    /// over to the parking brake, which holds the roll-back passively.
    #[test]
    fn roll_back_hands_over_to_the_parking_brake() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        // Slope pulls backward harder than the brake: 15 A × 50 = 750
        // against 2000.
        let (omega, last) = ride(&mut c, &cfg, 40.0, 300.0, 50.0, -2_000.0, 1.0, |_, _| {});
        assert_eq!(last, FailsafeAction::EngageBrake);
        assert!(omega.abs() <= BRAKE_ENTRY_MAX_E_RAD_S, "{omega}");
    }

    /// A reversal that persists above the parking-brake gate (the estimate
    /// jumped across zero): released at once, then braked the other way,
    /// held to the deceleration bound measured before the turn.
    #[test]
    fn persistent_reversal_turns_the_brake_around() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        for _ in 0..4_000 {
            c.step(500.0, DT, &cfg, 40.0, 0.0, 0.0, true);
        }
        let first = iq_of(c.step(-500.0, DT, &cfg, 40.0, 0.0, 0.0, true));
        assert!(first > -cfg.brake_current_a, "released: {first}");
        let mut last = 0.0;
        for _ in 0..4_000 {
            last = iq_of(c.step(-500.0, DT, &cfg, 40.0, 0.0, 0.0, true));
        }
        assert!((last - cfg.brake_current_a).abs() < 1e-3, "{last}");
    }

    /// One fast "reverse" reading mid-brake (an estimate glitch at speed)
    /// neither ends the stop nor turns the brake around.
    #[test]
    fn reverse_glitch_at_speed_changes_nothing() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        for _ in 0..2_000 {
            c.step(500.0, DT, &cfg, 40.0, 0.0, 0.0, true);
        }
        let before = iq_of(c.step(500.0, DT, &cfg, 40.0, 0.0, 0.0, true));
        let glitch = iq_of(c.step(-400.0, DT, &cfg, 40.0, 0.0, 0.0, true));
        let after = iq_of(c.step(500.0, DT, &cfg, 40.0, 0.0, 0.0, true));
        assert!(
            before < 0.0 && glitch < 0.0 && after < 0.0,
            "{before} {glitch} {after}"
        );
    }

    /// Approaching standstill the active brake stands down and the parking
    /// brake takes over; no current is ever pushed through zero.
    #[test]
    fn stop_hands_over_to_the_parking_brake() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let mut drove_backward = false;
        let (omega, last) = ride(&mut c, &cfg, 40.0, 600.0, 20.0, 0.0, 10.0, |omega, a| {
            if let FailsafeAction::Drive { iq_target, .. } = a {
                // A negative brake on a rotor already moving backward.
                drove_backward |= omega < 0.0 && iq_target < 0.0;
            }
        });
        assert_eq!(last, FailsafeAction::EngageBrake);
        assert!(omega.abs() <= BRAKE_ENTRY_MAX_E_RAD_S, "{omega}");
        assert!(!drove_backward);
    }

    #[test]
    fn stop_without_trustworthy_angle_hands_over_to_the_short() {
        // Sensorless angle not trustworthy → no current brake (torque in an
        // unknown direction). The parking brake's short needs no angle (and
        // guards itself by its current); without it, coast.
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            5.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        assert_eq!(
            c.step(300.0, DT, &cfg, 40.0, 0.0, 0.0, false),
            FailsafeAction::EngageBrake
        );
        let mut c = FailsafeController::new();
        c.arm(5.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        assert_eq!(
            c.step(300.0, DT, &cfg, 40.0, 0.0, 0.0, false),
            FailsafeAction::Stop
        );
    }

    #[test]
    fn park_brake_terminal_engages_on_clean_stop() {
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let (_, last) = ride(&mut c, &cfg, 40.0, 5.0, 0.0, 0.0, 1.0, |_, _| {});
        assert_eq!(last, FailsafeAction::EngageBrake);
        // Terminal is stable until the driver resets it.
        assert_eq!(
            c.step(5.0, DT, &cfg, 40.0, 0.0, 0.0, true),
            FailsafeAction::EngageBrake
        );
    }

    /// A host-configured standstill threshold above the parking-brake gate
    /// ends the stop high-Z rather than shorting the windings there.
    #[test]
    fn park_brake_terminal_respects_the_entry_gate() {
        let mut cfg = cfg(FailsafePolicy::ControlledStop);
        cfg.standstill_rad_s = 2.0 * BRAKE_ENTRY_MAX_E_RAD_S;
        let mut c = FailsafeController::new();
        c.arm(
            0.0,
            FailsafePolicy::ControlledStop,
            FailsafeTerminal::ParkBrake,
        );
        let omega = 1.5 * BRAKE_ENTRY_MAX_E_RAD_S;
        let (_, last) = ride(&mut c, &cfg, 40.0, omega, 0.0, 0.0, 1.0, |_, _| {});
        assert_eq!(last, FailsafeAction::Stop);
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
        assert_eq!(c.iq, 5.0);
    }

    #[test]
    fn config_default_is_sane() {
        assert!(FailsafeConfig::default().is_sane());
        let bad = FailsafeConfig {
            decel_rad_s2: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
        // A zero slew would freeze the q current where the failsafe found
        // it — a stop driving the last commanded torque forever.
        let bad = FailsafeConfig {
            slew_a_per_s: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(!bad.is_sane());
        // Zero stop current is meaningful (the general limit alone).
        let zero = FailsafeConfig {
            brake_current_a: 0.0,
            ..FailsafeConfig::default()
        };
        assert!(zero.is_sane());
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
        let cfg = cfg(FailsafePolicy::ControlledStop);
        let mut c = FailsafeController::new();
        c.arm(0.0, FailsafePolicy::ControlledStop, FailsafeTerminal::HighZ);
        c.set_terminal(FailsafeTerminal::ParkBrake);
        let (_, last) = ride(&mut c, &cfg, 40.0, 5.0, 0.0, 0.0, 1.0, |_, _| {});
        assert_eq!(last, FailsafeAction::EngageBrake);
    }

    #[test]
    #[cfg(feature = "storage")]
    fn from_stored_converts_units_and_falls_back() {
        use crate::storage::FailsafeConfigStored;

        // Default stored → the runtime default, with ms→µs conversion.
        let d = FailsafeConfig::from_stored(Some(&FailsafeConfigStored::default()));
        assert_eq!(d, FailsafeConfig::default());
        assert_eq!(d.staleness_timeout_us, 150_000);
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

        // Non-sane stored (zero slew) → full default fallback, never disabled.
        let bad = FailsafeConfigStored {
            slew_a_per_s: 0.0,
            ..FailsafeConfigStored::default()
        };
        assert_eq!(
            FailsafeConfig::from_stored(Some(&bad)),
            FailsafeConfig::default()
        );
    }
}
