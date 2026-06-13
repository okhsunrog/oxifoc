//! Sensorless startup state machine: **align → ramp → handoff** for a cold
//! start, plus the hall-dropout **recovery** nudge.
//!
//! This is the in-firmware sequencer the pure back-EMF [`Observer`] needs to
//! get a motor moving from standstill: the observer cannot commutate below
//! ~[`READY_MIN_VELOCITY`] (no back-EMF to observe), so something must drive
//! the rotor open-loop until there *is* back-EMF, then hand over. Modeled on
//! VESC's I/f start (`mcpwm_foc.c` `t_lock`/`t_ramp`/`t_const` ramp with a
//! current-scheduled ceiling) and MESC's open-loop ramp.
//!
//! Phase B (deadshort flying restart for an already-spinning rotor) prepends a
//! [`StartupPhase::Deadshort`] probe to this machine — see `foc_driver`.
//!
//! Pure logic: the manager [`tick`](SensorlessStartup::tick)s it each control
//! cycle with the measured current magnitude and the observer's readiness, and
//! applies the returned open-loop `(angle, velocity)` exactly as it applied the
//! old fixed override. Handoff is gated on the observer actually converging.
//!
//! [`Observer`]: super::observer::Observer
//! [`READY_MIN_VELOCITY`]: super::observer::READY_MIN_VELOCITY

use crate::foc::clamp_f32;
use crate::foc::wrap_angle;

/// Align dwell (s): hold the field at a fixed angle so the rotor latches to a
/// known position before the ramp. VESC defaults this to 0 (skips align); on a
/// cold bench start a brief latch is safer than ramping from an unknown angle.
pub const DEFAULT_ALIGN_TIME_S: f32 = 0.3;

/// Ramp duration (s): linear `0 → ceiling` open-loop velocity.
pub const DEFAULT_RAMP_TIME_S: f32 = 0.4;

/// Target electrical velocity (rad/s) to hand off to the observer at. Margin
/// above [`super::observer::READY_MIN_VELOCITY`] (30) so the back-EMF is
/// comfortably observable; ~570 eRPM.
pub const DEFAULT_HANDOFF_VEL: f32 = 60.0;

/// Hall-dropout recovery velocity (rad/s, ~500 eRPM). A *fast* nudge from the
/// last known angle — not a ramp — because there a real angle history exists
/// and the rotor is already moving; we only bridge to observer re-lock.
pub const DEFAULT_RECOVERY_VEL: f32 = 52.0;

/// Informational dwell (s) reported by [`SensorlessStartup::timer`]. Does NOT
/// terminate the sequence (dropping to a dead estimator at its expiry is worse
/// than continuing open loop — VESC keeps its override running for the same
/// reason); handoff is gated solely on the observer converging.
pub const DEFAULT_OPENLOOP_TIME_S: f32 = 0.5;

/// Current (A, |i_αβ|) at which the ramp ceiling reaches its max. Below it the
/// ceiling scales down toward [`DEFAULT_HANDOFF_VEL`] so a gentle throttle
/// ramps gently (VESC `openloop_rpm_max = map(iq, 0..I_max, …)`).
const CURRENT_REF_A: f32 = 5.0;

/// Max ramp ceiling as a multiple of the handoff velocity at full current.
const CEILING_MAX_FACTOR: f32 = 2.0;

/// Where the open-loop sequencer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartupPhase {
    /// Not sequencing — the real source (sensor / ready observer) drives.
    Inactive,
    /// Holding the field at a fixed angle (velocity 0) so the rotor latches.
    Align,
    /// Linearly ramping the open-loop velocity 0 → ceiling.
    Ramp,
    /// Holding at the ceiling, waiting for the observer to converge.
    Hold,
    /// Hall-dropout recovery: fixed-velocity nudge from the last known angle.
    Recover,
}

/// What the manager should drive this cycle.
#[derive(Clone, Copy, Debug)]
pub struct StartupOutput {
    /// Open-loop commutation angle (rad).
    pub angle: f32,
    /// Open-loop commutation velocity (rad/s, signed by direction).
    pub velocity: f32,
    /// The observer has converged at handoff speed — seed it from
    /// `(angle, velocity)` and [`deactivate`](SensorlessStartup::deactivate).
    pub handoff: bool,
}

/// Cold-start / recovery open-loop sequencer. See the module docs.
#[derive(Clone, Copy, Debug)]
pub struct SensorlessStartup {
    phase: StartupPhase,
    angle: f32,
    velocity: f32,
    /// Commanded direction (+1 / −1); the ramp and recovery are signed by it.
    dir: f32,
    /// Target handoff speed magnitude (rad/s).
    handoff_vel: f32,
    timer: f32,
}

impl Default for SensorlessStartup {
    fn default() -> Self {
        Self {
            phase: StartupPhase::Inactive,
            angle: 0.0,
            velocity: 0.0,
            dir: 1.0,
            handoff_vel: DEFAULT_HANDOFF_VEL,
            timer: 0.0,
        }
    }
}

impl SensorlessStartup {
    /// Begin a cold start from `angle0` in `direction` (sign of the commanded
    /// torque/velocity): align → ramp → hold → handoff.
    pub fn begin_cold_start(&mut self, angle0: f32, direction: f32) {
        self.phase = StartupPhase::Align;
        self.angle = wrap_angle(angle0);
        self.velocity = 0.0;
        self.dir = if direction < 0.0 { -1.0 } else { 1.0 };
        self.handoff_vel = DEFAULT_HANDOFF_VEL;
        self.timer = DEFAULT_OPENLOOP_TIME_S + DEFAULT_ALIGN_TIME_S + DEFAULT_RAMP_TIME_S;
    }

    /// Begin a hall-dropout recovery: a fast fixed-velocity nudge from the last
    /// known `angle0`, signed by `direction` (the last known velocity sign — a
    /// board rolling backward must not be spun forward).
    pub fn begin_recovery(&mut self, angle0: f32, direction: f32) {
        self.phase = StartupPhase::Recover;
        self.angle = wrap_angle(angle0);
        self.dir = if direction < 0.0 { -1.0 } else { 1.0 };
        self.velocity = self.dir * DEFAULT_RECOVERY_VEL;
        self.handoff_vel = DEFAULT_RECOVERY_VEL;
        self.timer = DEFAULT_OPENLOOP_TIME_S;
    }

    /// True while the sequencer owns commutation.
    pub fn is_active(&self) -> bool {
        self.phase != StartupPhase::Inactive
    }

    /// Current phase (for diagnostics / tests).
    pub fn phase(&self) -> StartupPhase {
        self.phase
    }

    /// Stop sequencing (handed off to the observer, or source changed).
    pub fn deactivate(&mut self) {
        self.phase = StartupPhase::Inactive;
        self.timer = 0.0;
        self.velocity = 0.0;
    }

    /// Current open-loop angle (rad).
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Current open-loop velocity (rad/s, signed).
    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    /// Informational dwell remaining (s). Does not gate handoff.
    pub fn timer(&self) -> f32 {
        self.timer
    }

    /// Ramp ceiling for the measured current magnitude — at least the handoff
    /// speed, scaled up toward `CEILING_MAX_FACTOR×` at `CURRENT_REF_A`.
    fn ramp_ceiling(&self, current_mag: f32) -> f32 {
        let frac = clamp_f32(current_mag / CURRENT_REF_A, 0.0, 1.0);
        self.handoff_vel * (1.0 + frac * (CEILING_MAX_FACTOR - 1.0))
    }

    /// Advance one control cycle.
    ///
    /// * `current_mag` — measured |i_αβ| (A), for the current-scheduled ceiling.
    /// * `observer_ready` — `Observer::is_ready()`.
    /// * `observer_vel` — observer's velocity estimate (rad/s), for the handoff
    ///   speed gate (the open-loop velocity alone can outrun a not-yet-locked
    ///   observer).
    ///
    /// Returns the `(angle, velocity)` to commutate at and whether to hand off.
    pub fn tick(
        &mut self,
        dt: f32,
        current_mag: f32,
        observer_ready: bool,
        observer_vel: f32,
    ) -> StartupOutput {
        self.timer = (self.timer - dt).max(0.0);

        match self.phase {
            StartupPhase::Inactive => {
                return StartupOutput {
                    angle: self.angle,
                    velocity: 0.0,
                    handoff: false,
                };
            }
            StartupPhase::Align => {
                // Hold the field still so the rotor latches; the align dwell is
                // tracked against the informational timer's head room.
                self.velocity = 0.0;
                let align_elapsed =
                    (DEFAULT_OPENLOOP_TIME_S + DEFAULT_ALIGN_TIME_S + DEFAULT_RAMP_TIME_S)
                        - self.timer;
                if align_elapsed >= DEFAULT_ALIGN_TIME_S {
                    self.phase = StartupPhase::Ramp;
                }
            }
            StartupPhase::Ramp => {
                let ceiling = self.ramp_ceiling(current_mag);
                let rate = ceiling / DEFAULT_RAMP_TIME_S;
                let target = self.dir * ceiling;
                // Step the magnitude toward the ceiling; sign stays `dir`.
                let next = self.velocity + self.dir * rate * dt;
                self.velocity = if self.dir >= 0.0 {
                    next.min(target)
                } else {
                    next.max(target)
                };
                if self.velocity.abs() >= ceiling {
                    self.phase = StartupPhase::Hold;
                }
            }
            StartupPhase::Hold | StartupPhase::Recover => {
                // Velocity already at the ceiling / recovery speed; hold it.
            }
        }

        self.angle = wrap_angle(self.angle + self.velocity * dt);

        // Hand off only when the observer has actually converged AND there is
        // enough speed for it to track — both the open-loop command and the
        // observer's own estimate must clear the handoff floor.
        let fast_enough =
            self.velocity.abs() >= self.handoff_vel && observer_vel.abs() >= self.handoff_vel * 0.5;
        let handoff = observer_ready && fast_enough && self.phase != StartupPhase::Align;

        StartupOutput {
            angle: self.angle,
            velocity: self.velocity,
            handoff,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 20_000.0;

    /// Drive the SM forward, never reporting the observer ready, for a number
    /// of seconds; return the phase history boundaries.
    fn run(sm: &mut SensorlessStartup, secs: f32, current: f32) -> Vec<(StartupPhase, f32)> {
        let mut out = Vec::new();
        let mut last = sm.phase();
        out.push((last, sm.velocity()));
        let steps = (secs / DT) as usize;
        for _ in 0..steps {
            sm.tick(DT, current, false, 0.0);
            if sm.phase() != last {
                last = sm.phase();
                out.push((last, sm.velocity()));
            }
        }
        out
    }

    #[test]
    fn cold_start_sequences_align_ramp_hold() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(1.0, 1.0);
        assert_eq!(sm.phase(), StartupPhase::Align);
        assert!(sm.is_active());

        let hist = run(&mut sm, 1.0, 3.0);
        let phases: Vec<_> = hist.iter().map(|(p, _)| *p).collect();
        assert_eq!(
            phases,
            vec![StartupPhase::Align, StartupPhase::Ramp, StartupPhase::Hold]
        );
        // Settles at or above the handoff velocity.
        assert!(sm.velocity() >= DEFAULT_HANDOFF_VEL);
    }

    #[test]
    fn align_holds_field_still() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.5, 1.0);
        // Through the whole align window, velocity stays 0 and angle is fixed.
        let steps = (DEFAULT_ALIGN_TIME_S / DT) as usize / 2;
        for _ in 0..steps {
            let o = sm.tick(DT, 2.0, false, 0.0);
            assert_eq!(o.velocity, 0.0);
            assert!((o.angle - 0.5).abs() < 1e-4);
        }
        assert_eq!(sm.phase(), StartupPhase::Align);
    }

    #[test]
    fn ramp_is_monotonic_and_signed_by_direction() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, -1.0); // reverse
        // skip align
        run(&mut sm, DEFAULT_ALIGN_TIME_S + 0.001, 5.0);
        assert_eq!(sm.phase(), StartupPhase::Ramp);
        let mut prev = sm.velocity();
        for _ in 0..100 {
            sm.tick(DT, 5.0, false, 0.0);
            assert!(
                sm.velocity() <= prev + 1e-3,
                "velocity must not reverse-jump"
            );
            assert!(sm.velocity() <= 0.0, "reverse start stays negative");
            prev = sm.velocity();
        }
    }

    #[test]
    fn higher_current_raises_the_ceiling() {
        let mut lo = SensorlessStartup::default();
        let mut hi = SensorlessStartup::default();
        lo.begin_cold_start(0.0, 1.0);
        hi.begin_cold_start(0.0, 1.0);
        run(&mut lo, 1.0, 0.0); // no current → floor ceiling
        run(&mut hi, 1.0, CURRENT_REF_A); // full current → max ceiling
        assert!(hi.velocity() > lo.velocity() + 1.0);
        assert!((lo.velocity() - DEFAULT_HANDOFF_VEL).abs() < 1.0);
    }

    #[test]
    fn no_handoff_until_observer_ready_and_fast() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, 1.0);
        // Run past align+ramp (0.8 s) with the observer NOT ready — never hands
        // off, and ends solidly in Hold above the handoff speed.
        for _ in 0..(20_000 * 8 / 10) {
            let o = sm.tick(DT, 5.0, false, 0.0);
            assert!(!o.handoff);
        }
        assert_eq!(sm.phase(), StartupPhase::Hold);
        assert!(sm.velocity() >= DEFAULT_HANDOFF_VEL);
        // Observer ready but slow estimate → still no handoff.
        let o = sm.tick(DT, 5.0, true, 5.0);
        assert!(!o.handoff);
        // Observer ready AND tracking at speed → handoff.
        let o = sm.tick(DT, 5.0, true, DEFAULT_HANDOFF_VEL);
        assert!(o.handoff);
    }

    #[test]
    fn recovery_is_a_fixed_nudge_from_known_angle() {
        let mut sm = SensorlessStartup::default();
        sm.begin_recovery(2.0, 1.0);
        assert_eq!(sm.phase(), StartupPhase::Recover);
        assert_eq!(sm.velocity(), DEFAULT_RECOVERY_VEL);
        assert!(sm.timer() > 0.0);
        // No align/ramp — straight to the nudge speed, angle advances.
        let o = sm.tick(DT, 0.0, false, 0.0);
        assert_eq!(o.velocity, DEFAULT_RECOVERY_VEL);
        assert!(o.angle > 2.0);
    }

    #[test]
    fn recovery_reverse_does_not_spin_forward() {
        let mut sm = SensorlessStartup::default();
        sm.begin_recovery(0.0, -1.0);
        assert_eq!(sm.velocity(), -DEFAULT_RECOVERY_VEL);
        let o = sm.tick(DT, 0.0, false, 0.0);
        assert!(o.velocity < 0.0);
    }

    #[test]
    fn deactivate_goes_inactive() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, 1.0);
        sm.deactivate();
        assert!(!sm.is_active());
        assert_eq!(sm.phase(), StartupPhase::Inactive);
    }
}
