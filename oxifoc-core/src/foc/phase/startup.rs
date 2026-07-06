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
use crate::foc::fast_math::{atan2f, sqrtf};
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

/// Deadshort flying-restart probe length (PWM periods). MESC shorts ~9; long
/// enough for a measurable dI/dt, short enough that the current stays bounded.
pub const DEADSHORT_CYCLES: u16 = 8;

/// Shorted-bridge settle time (PWM periods) before the probe captures its
/// baseline. The first cycles after the bridge (re)enables into the short
/// carry a genuine decaying current transient — measured ~0.4 A pk over
/// ~200 µs on B-G431B-ESC1 + ZD2808 (2026-07-06) — which the probe read as
/// back-EMF and falsely "caught" a spinning rotor at standstill (ω≈46 with
/// the old 45 threshold). 8 periods = 400 µs ≈ 2× the measured decay; a
/// rotor with real back-EMF keeps driving current after the settle window,
/// so a true catch survives the wait.
pub const DEADSHORT_SETTLE_CYCLES: u16 = 8;

/// Minimum |ω| (rad/s elec) the deadshort must resolve to declare the rotor
/// "spinning" and seed the observer for a flying restart. Below it (standstill
/// or barely turning), fall through to the align→ramp cold start. Set at the
/// handoff velocity: a seed below handoff speed lands the observer in a band
/// the handoff gates would not trust anyway, and the bench false-catch
/// (enable transient, see [`DEADSHORT_SETTLE_CYCLES`]) resolved to ω≈46 —
/// comfortably below this bar as a second line of defense.
pub const DEADSHORT_MIN_CATCH_VEL: f32 = 60.0;

/// Abort the probe early if |i_αβ| exceeds this (A): the back-EMF drives the
/// shorted winding toward `e/R`, which on a low-R motor is large — stop and
/// estimate from the dI/dt accumulated so far rather than build current
/// without bound. Mirrors MESC's `DEADSHORT_CURRENT`.
const DEADSHORT_MAX_CURRENT_A: f32 = 15.0;

/// Where the open-loop sequencer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartupPhase {
    /// Not sequencing — the real source (sensor / ready observer) drives.
    Inactive,
    /// Flying-restart probe (Phase B): the bridge is shorted (zero voltage)
    /// and the back-EMF-driven current slope is measured to catch an
    /// already-spinning rotor before the cold-start ramp.
    Deadshort,
    /// Holding the field at a fixed angle (velocity 0) so the rotor latches.
    Align,
    /// Linearly ramping the open-loop velocity 0 → ceiling.
    Ramp,
    /// Holding at the ceiling, waiting for the observer to converge.
    Hold,
    /// Hall-dropout recovery: fixed-velocity nudge from the last known angle.
    Recover,
}

impl StartupPhase {
    /// Short lowercase name for transition logs.
    pub fn name(self) -> &'static str {
        match self {
            Self::Inactive => "inactive",
            Self::Deadshort => "deadshort",
            Self::Align => "align",
            Self::Ramp => "ramp",
            Self::Hold => "hold",
            Self::Recover => "recover",
        }
    }
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

/// Outcome of feeding one cycle of shorted-winding current to the deadshort
/// probe (Phase B).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DeadshortResult {
    /// Still probing — keep the bridge shorted.
    Probing,
    /// Caught a spinning rotor: seed the observer from `(angle, velocity)` and
    /// go straight to closed loop (the sequencer has deactivated itself).
    Caught { angle: f32, velocity: f32 },
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
    // ── Deadshort probe state (Phase B) ──
    ds_settle: u16,
    ds_cycles: u16,
    ds_i0_alpha: f32,
    ds_i0_beta: f32,
    ds_elapsed: f32,
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
            ds_settle: 0,
            ds_cycles: 0,
            ds_i0_alpha: 0.0,
            ds_i0_beta: 0.0,
            ds_elapsed: 0.0,
        }
    }
}

impl SensorlessStartup {
    /// Begin a sensorless start from `angle0` in `direction` (sign of the
    /// commanded torque/velocity). Opens with the deadshort flying-restart
    /// probe (Phase B): if the rotor is already spinning it is caught and
    /// handed straight to the observer; otherwise the machine falls through to
    /// the align→ramp→handoff cold start (Phase A).
    pub fn begin_cold_start(&mut self, angle0: f32, direction: f32) {
        self.phase = StartupPhase::Deadshort;
        self.angle = wrap_angle(angle0);
        self.velocity = 0.0;
        self.dir = if direction < 0.0 { -1.0 } else { 1.0 };
        self.handoff_vel = DEFAULT_HANDOFF_VEL;
        self.timer = DEFAULT_OPENLOOP_TIME_S + DEFAULT_ALIGN_TIME_S + DEFAULT_RAMP_TIME_S;
        self.ds_settle = DEADSHORT_SETTLE_CYCLES;
        self.ds_cycles = 0;
        self.ds_elapsed = 0.0;
    }

    /// True while the deadshort probe needs the bridge held shorted (zero
    /// voltage). The driver honors this instead of normal commutation.
    pub fn wants_short(&self) -> bool {
        self.phase == StartupPhase::Deadshort
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
            // Deadshort is driven by `feed_deadshort`, not `tick`; if ticked,
            // hold the angle (the driver applies zero voltage anyway).
            StartupPhase::Inactive | StartupPhase::Deadshort => {
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

        // Hand off when the observer has converged AND is tracking at a
        // usable speed. Two ways to be "fast enough": the open-loop drag
        // reached the handoff velocity (nominal synchronous ramp), or the
        // observer itself reads at/above it — an unloaded rotor slips ahead
        // of the I/f ramp and runs away (bench + sim 2026-07-06: rotor at
        // 380–800 rad/s while the ramp was still at 50–65; the phase-current
        // frequency in the capture confirmed the observer was RIGHT). Waiting
        // for the ramp in that case only lets the runaway grow — a ready
        // observer at handoff speed takes over immediately.
        //
        // The runaway path additionally requires the ramp to have actually
        // dragged the rotor a while (velocity ≥ 20% of handoff): the align
        // phase leaves the rotor swinging on its undamped magnetic spring
        // (~8 Hz mech on the bench motor), the observer can lock onto that
        // swing and report "ready" at hundreds of rad/s right at ramp entry
        // (bench: observer_vel 585–786 at openloop_vel 0–1.2). Every real
        // runaway observed fired at openloop_vel ≥ 23.
        let ramp_moving = self.velocity.abs() >= self.handoff_vel * 0.2;
        let fast_enough = (self.velocity.abs() >= self.handoff_vel
            && observer_vel.abs() >= self.handoff_vel * 0.5)
            || (ramp_moving && observer_vel.abs() >= self.handoff_vel);
        let handoff = observer_ready && fast_enough && self.phase != StartupPhase::Align;

        StartupOutput {
            angle: self.angle,
            velocity: self.velocity,
            handoff,
        }
    }

    /// Feed one shorted-winding current sample to the deadshort probe (Phase B).
    ///
    /// `l`/`lambda` are the observer's motor model. The first call captures the
    /// baseline current (the back-EMF response is the *change* from there); over
    /// the next [`DEADSHORT_CYCLES`] (or until |i| hits the abort cap) it
    /// accumulates dI/dt and estimates the back-EMF `e = −L·dI/dt`, hence the
    /// rotor angle and speed (see [`deadshort_estimate`]). A spinning rotor →
    /// `Caught` and the machine deactivates (the manager seeds the observer);
    /// standstill / too slow → it falls through to the align ramp, returning
    /// `Probing`.
    pub fn feed_deadshort(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        dt: f32,
        l: f32,
        lambda: f32,
    ) -> DeadshortResult {
        if self.phase != StartupPhase::Deadshort {
            return DeadshortResult::Probing;
        }
        // Let the bridge-enable current transient decay into the short before
        // measuring (see DEADSHORT_SETTLE_CYCLES). A current already at the
        // abort cap means a genuinely energetic rotor — skip straight to the
        // probe rather than sit shorted on a large current.
        if self.ds_settle > 0 {
            let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
            if i_mag < DEADSHORT_MAX_CURRENT_A {
                self.ds_settle -= 1;
                return DeadshortResult::Probing;
            }
            self.ds_settle = 0;
        }
        if self.ds_cycles == 0 {
            self.ds_i0_alpha = i_alpha;
            self.ds_i0_beta = i_beta;
            self.ds_elapsed = 0.0;
            self.ds_cycles = 1;
            return DeadshortResult::Probing;
        }

        self.ds_cycles += 1;
        self.ds_elapsed += dt;
        let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        if self.ds_cycles < DEADSHORT_CYCLES && i_mag < DEADSHORT_MAX_CURRENT_A {
            return DeadshortResult::Probing;
        }

        // Probe window complete — estimate the back-EMF from the net dI/dt.
        let di_alpha = i_alpha - self.ds_i0_alpha;
        let di_beta = i_beta - self.ds_i0_beta;
        match deadshort_estimate(di_alpha, di_beta, self.ds_elapsed, l, lambda, self.dir) {
            Some((angle, velocity)) => {
                self.deactivate();
                DeadshortResult::Caught { angle, velocity }
            }
            None => {
                // Standstill / too slow → align→ramp cold start from here.
                self.phase = StartupPhase::Align;
                self.velocity = 0.0;
                DeadshortResult::Probing
            }
        }
    }
}

/// Back-EMF / rotor estimate from the shorted-winding dI/dt over `window_dt`.
///
/// While the current is small the shorted (zero-voltage) winding obeys
/// `0 = R·i + L·di/dt + e`, so `e ≈ −L·dI/dt`. The back-EMF leads the rotor
/// flux by 90° (the sign of that lead is the rotation direction), giving
/// `θ = atan2(e_β, e_α) − dir·π/2` and `|ω| = |e|/λ`. `dir` is the commanded
/// direction, taken as the rotation sign (true for the kick-push restart;
/// a rotor freewheeling *against* the command is the known v1 limitation —
/// the PLL would have to pull a ±180° seed, which it can't). Returns `None`
/// when |ω| is below the catch threshold (standstill / barely turning → use
/// the cold-start ramp instead).
fn deadshort_estimate(
    di_alpha: f32,
    di_beta: f32,
    window_dt: f32,
    l: f32,
    lambda: f32,
    dir: f32,
) -> Option<(f32, f32)> {
    if window_dt <= 0.0 || lambda <= 1e-9 || l <= 0.0 {
        return None;
    }
    let e_alpha = -l * di_alpha / window_dt;
    let e_beta = -l * di_beta / window_dt;
    let e_mag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
    let omega = e_mag / lambda;
    if omega < DEADSHORT_MIN_CATCH_VEL {
        return None;
    }
    let angle = wrap_angle(atan2f(e_beta, e_alpha) - dir * core::f32::consts::FRAC_PI_2);
    Some((angle, dir * omega))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 20_000.0;

    /// Feed the deadshort probe a standstill (zero-current) stream until it
    /// gives up and falls through to the cold-start ramp (Align). Phase A tests
    /// use this to get past the Phase B probe that now opens every cold start.
    fn skip_deadshort(sm: &mut SensorlessStartup) {
        for _ in 0..=(DEADSHORT_SETTLE_CYCLES + DEADSHORT_CYCLES) {
            sm.feed_deadshort(0.0, 0.0, DT, 200e-6, 0.01);
            if sm.phase() != StartupPhase::Deadshort {
                break;
            }
        }
        assert_eq!(sm.phase(), StartupPhase::Align);
    }

    /// Feed zero current through the settle window so the next
    /// `feed_deadshort` call captures the probe baseline.
    fn skip_settle(sm: &mut SensorlessStartup) {
        for _ in 0..DEADSHORT_SETTLE_CYCLES {
            assert_eq!(
                sm.feed_deadshort(0.0, 0.0, DT, 200e-6, 0.01),
                DeadshortResult::Probing
            );
            assert_eq!(sm.phase(), StartupPhase::Deadshort);
        }
    }

    /// Begin a cold start and skip straight to the Align phase.
    fn cold_start_to_align(angle: f32, dir: f32) -> SensorlessStartup {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(angle, dir);
        skip_deadshort(&mut sm);
        sm
    }

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
        let mut sm = cold_start_to_align(1.0, 1.0);
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
        let mut sm = cold_start_to_align(0.5, 1.0);
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
        let mut sm = cold_start_to_align(0.0, -1.0); // reverse
        run(&mut sm, DEFAULT_ALIGN_TIME_S + 0.001, 5.0); // skip align
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
        let mut lo = cold_start_to_align(0.0, 1.0);
        let mut hi = cold_start_to_align(0.0, 1.0);
        run(&mut lo, 1.0, 0.0); // no current → floor ceiling
        run(&mut hi, 1.0, CURRENT_REF_A); // full current → max ceiling
        assert!(hi.velocity() > lo.velocity() + 1.0);
        assert!((lo.velocity() - DEFAULT_HANDOFF_VEL).abs() < 1.0);
    }

    #[test]
    fn no_handoff_until_observer_ready_and_fast() {
        let mut sm = cold_start_to_align(0.0, 1.0);
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
    fn runaway_rotor_hands_off_early_but_not_at_ramp_entry() {
        // I/f runaway: an unloaded rotor slips ahead of the ramp and
        // accelerates freely. A READY observer tracking it at handoff speed
        // must take over — but only once the ramp has actually dragged the
        // rotor (≥20% of handoff): at ramp entry a "ready" observer at
        // speed is the align-swing artifact (bench 2026-07-06: observer
        // 585–786 rad/s at openloop 0–1.2) and must be ignored.
        let mut sm = cold_start_to_align(0.0, 1.0);
        run(&mut sm, DEFAULT_ALIGN_TIME_S + 0.005, 5.0); // ramp just started
        assert_eq!(sm.phase(), StartupPhase::Ramp);
        assert!(sm.velocity() < DEFAULT_HANDOFF_VEL * 0.2);
        let o = sm.tick(DT, 5.0, true, 400.0);
        assert!(
            !o.handoff,
            "align-swing false-ready must not hand off at ramp entry"
        );
        // Ramp genuinely dragging → the runaway path fires below the
        // nominal handoff velocity.
        run(&mut sm, 0.08, 5.0);
        assert!(sm.velocity() >= DEFAULT_HANDOFF_VEL * 0.2);
        assert!(sm.velocity() < DEFAULT_HANDOFF_VEL);
        let o = sm.tick(DT, 5.0, true, 400.0);
        assert!(
            o.handoff,
            "ready+fast observer must take over a runaway rotor"
        );
    }

    // ── Phase B: deadshort flying restart ──

    /// Synthesize the shorted-winding dI over the probe window for a rotor
    /// spinning at `omega` (elec rad/s) parked at `theta`: `e = ωλ[−sinθ, cosθ]`
    /// drives `dI ≈ −e·window/L` while the current is small.
    fn synth_deadshort_di(omega: f32, theta: f32, l: f32, lambda: f32) -> (f32, f32) {
        let e_a = omega * lambda * -theta.sin();
        let e_b = omega * lambda * theta.cos();
        let window = f32::from(DEADSHORT_CYCLES - 1) * DT;
        (-e_a * window / l, -e_b * window / l)
    }

    #[test]
    fn deadshort_estimate_recovers_rotor() {
        use crate::foc::angle_difference;
        let (l, lambda, omega, theta) = (150e-6, 0.008, 300.0, -0.8);
        let window = f32::from(DEADSHORT_CYCLES - 1) * DT;
        let (di_a, di_b) = synth_deadshort_di(omega, theta, l, lambda);
        let (angle, vel) = deadshort_estimate(di_a, di_b, window, l, lambda, 1.0).unwrap();
        assert!(angle_difference(angle, theta).abs() < 0.05, "angle {angle}");
        assert!((vel - omega).abs() < 5.0, "vel {vel}");
        // A barely-moving rotor (1% of the dI) is below the catch floor → None.
        assert!(deadshort_estimate(di_a * 0.01, di_b * 0.01, window, l, lambda, 1.0).is_none());
    }

    #[test]
    fn deadshort_catches_spinning_rotor() {
        use crate::foc::angle_difference;
        let (l, lambda, omega, theta) = (200e-6, 0.01, 250.0, 1.2);
        let (di_a, di_b) = synth_deadshort_di(omega, theta, l, lambda);

        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, 1.0); // dir matches the (forward) rotation
        assert_eq!(sm.phase(), StartupPhase::Deadshort);
        assert!(sm.wants_short());
        skip_settle(&mut sm);

        // Cycle 1 captures the baseline; middle cycles only count; the last
        // carries the accumulated dI.
        assert_eq!(
            sm.feed_deadshort(0.0, 0.0, DT, l, lambda),
            DeadshortResult::Probing
        );
        for _ in 0..(DEADSHORT_CYCLES - 2) {
            assert_eq!(
                sm.feed_deadshort(0.0, 0.0, DT, l, lambda),
                DeadshortResult::Probing
            );
        }
        match sm.feed_deadshort(di_a, di_b, DT, l, lambda) {
            DeadshortResult::Caught { angle, velocity } => {
                assert!(angle_difference(angle, theta).abs() < 0.1, "angle {angle}");
                assert!((velocity - omega).abs() < 25.0, "vel {velocity}");
            }
            other => panic!("expected Caught, got {other:?}"),
        }
        // Handed straight to the observer — sequencer is done, bridge released.
        assert!(!sm.is_active());
        assert!(!sm.wants_short());
    }

    #[test]
    fn deadshort_standstill_falls_through_to_align() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.7, 1.0);
        // No back-EMF (zero dI) over the whole settle+probe → no catch →
        // cold start.
        for _ in 0..=(DEADSHORT_SETTLE_CYCLES + DEADSHORT_CYCLES) {
            sm.feed_deadshort(0.0, 0.0, DT, 200e-6, 0.01);
        }
        assert_eq!(sm.phase(), StartupPhase::Align);
        assert!(sm.is_active());
        assert!(!sm.wants_short());
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
