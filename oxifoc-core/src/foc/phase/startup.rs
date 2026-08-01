//! Sensorless startup state machine: **ramp → handoff** for a cold
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

/// Ramp current soft-start (s): the torque command scales 0 → 1 over this
/// window at ramp entry (see [`SensorlessStartup::current_scale`]).
///
/// There is deliberately NO align phase (VESC's default `align time = 0`):
/// holding the field at a fixed angle parks the rotor on an undamped
/// magnetic spring — bench 2026-07-06, ZD2808: the align swing tripped the
/// dq overcurrent on 1/3–4/5 of cold starts depending on where the rotor
/// last parked, current soft-start alone could not fix a resonance with no
/// damping, and the observer kept locking onto the swing (false-ready).
/// A field that ROTATES from the first cycle never gives the spring a
/// fixed anchor to oscillate around — the rotor slips into sync during the
/// slow early ramp instead. The soft-start keeps the initial pull gentle
/// for unlucky initial rotor angles.
pub const RAMP_CURRENT_SOFT_START_S: f32 = 0.15;

/// Ramp duration (s): linear `0 → ceiling` open-loop velocity.
///
/// Scaled with [`DEFAULT_HANDOFF_VEL`] to keep the ramp acceleration at the
/// hardware-validated ~150 rad/s² el (60 rad/s over 0.4 s): the open-loop
/// capture is an undamped synchronous machine, and its hunt amplitude
/// scales with the slip acceleration — at 3× the acceleration a light
/// 0.3 A ramp fails to capture the rotor at all (sim: cold start never
/// hands off). The longer open-loop dwell is affordable post-shave.
pub const DEFAULT_RAMP_TIME_S: f32 = 1.2;

/// Target electrical velocity (rad/s) to hand off to the observer at.
///
/// The binding constraint is the inverter distortion floor, not the
/// observer's nominal speed floor ([`super::observer::READY_MIN_VELOCITY`],
/// 30): at the original 60 rad/s the ZD2808's back-EMF is λω ≈ 69 mV —
/// at/below the post-compensation dead-time residual, and the bench
/// (2026-07-06, staircase + debug-start) showed the observer reading
/// 32–62 rad/s with confidence *decaying* and the e_q external-validity
/// check never corroborating: the signal genuinely wasn't above the noise.
/// 180 rad/s el puts λω ≈ 0.21 V, an order of magnitude over the residual
/// (~1720 eRPM ≈ 250 mech RPM on the 7-pp bench motor). The longer dwell
/// in the open-loop startup path is affordable since the tier-2 ISR shave
/// (startup path ~87% load, command pump alive throughout).
pub const DEFAULT_HANDOFF_VEL: f32 = 180.0;

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

/// Shorted-bridge settle time (PWM periods) before the probe starts
/// averaging the short-circuit current. Two transients must decay first:
/// the bridge-enable glitch (~0.4 A pk over ~200 µs measured on
/// B-G431B-ESC1 + ZD2808, 2026-07-06 — the old ΔI probe read it as
/// back-EMF and falsely "caught" a spinning rotor at standstill), and the
/// exponential approach of the short current to its steady state
/// (τ = L/R ≈ 0.19 ms ≈ 4 PWM periods on the ZD2808 — the settled-current
/// estimator *requires* steady state, see [`short_current_estimate`]).
/// 16 periods = 800 µs ≈ 4τ (98% settled).
pub const DEADSHORT_SETTLE_CYCLES: u16 = 16;

/// Minimum |ω| (rad/s elec) the deadshort must resolve to declare the rotor
/// "spinning" and seed the observer for a flying restart. Below it (standstill
/// or too slow to RIDE), fall through to the ramp cold start.
///
/// The bound is not what the probe can MEASURE (the shorted bridge has no
/// PWM switching, so the probe resolves well below this) — it is what the
/// closed loop can subsequently ride. Bench 2026-07-06/07: catches at
/// 70–90 rad/s seeded a closed loop below the inverter-distortion floor,
/// which promptly lost trust and recycled — a restart churn loop, each
/// iteration re-catching the still-coasting rotor a little slower. Sending
/// those through the ramp instead re-accelerates to the
/// [`DEFAULT_HANDOFF_VEL`] (180) band where the observer is validated.
/// Kept below the handoff velocity: a rotor already coasting at ≥140 pulls
/// into closed loop reliably (the distortion floor is ~120 on the ZD2808).
/// The bench false-catch (enable transient, see
/// [`DEADSHORT_SETTLE_CYCLES`]) resolved to ω≈46 — far below this bar.
pub const DEADSHORT_MIN_CATCH_VEL: f32 = 140.0;

/// Minimum probe-measured |ω| (rad/s elec) for a confirm probe to count as
/// "strong" toward the [`CONFIRM_SEED_PROBES`] hold-ratchet escape.
/// Deliberately SEPARATE from (and lower than) [`DEADSHORT_MIN_CATCH_VEL`]:
/// during the escape the drive is still holding the rotor at the handoff
/// velocity — the question is "is a real rotor turning at all while the
/// observer's claim diverges", not "can a coasting rotor be ridden from
/// here". The 2026-07-06 hold-ratchet escape measured 32–108 rad/s on a
/// genuinely captured rotor (probes sample random phases of the capture
/// hunt); a floor at the catch bound would have starved the escape.
const CONFIRM_STRONG_PROBE_VEL: f32 = 60.0;

/// Abort the probe early if |i_αβ| exceeds this (A): the back-EMF drives the
/// shorted winding toward `e/R`, which on a low-R motor is large — stop and
/// estimate from the dI/dt accumulated so far rather than build current
/// without bound. Mirrors MESC's `DEADSHORT_CURRENT`.
///
/// MUST stay below the driver's software OC trip
/// (`CurrentLimits::overcurrent_threshold_a` — 10.8 A on the bench ZD2808
/// config, 1.5× the 7.2 A rating): the shorted-bridge step checks the trip
/// too, so a cap above it can never engage — the probe FAULTS instead of
/// capping (bench 2026-07-07 confirm2-2: a confirm retry against a rotor
/// that had ratcheted to ~750 el rad/s built λω/|Z| ≈ 7 A of settled short
/// current plus the entry transient and tripped the 10.8 A dq OC; the old
/// 15 A cap sat unreachable above it). 8 A keeps a margin for the entry
/// transient; a cap-ended window still yields a valid estimate from the
/// accumulated current (and the cold-start catch treats "capped" as
/// proven-fast). TODO: derive from the live `CurrentLimits` instead of a
/// const once the probe plumbing carries them.
const DEADSHORT_MAX_CURRENT_A: f32 = 8.0;

/// Handoff-confirm probe: settle time (PWM periods) for the DRIVE current to
/// decay into the short before the baseline is captured. Longer than the
/// cold-start probe's settle: here amps of commanded current are flowing at
/// entry (τ = L/R ≈ 0.8 ms on the ZD2808), and residual decay would read as
/// back-EMF. 32 periods = 1.6 ms ≈ 2τ; the rotor coasts through it with
/// negligible speed loss (J ≥ 5e-5, friction ~µN·m).
pub const CONFIRM_SETTLE_CYCLES: u16 = 32;

/// Handoff-confirm probe: the measured |ω| must reach this fraction of the
/// observer's claim to confirm. Generous — the probe fights residual current
/// decay and sensor noise — but a phantom measures ≈ 0 (no rotor, no
/// back-EMF), so the gap is wide.
const CONFIRM_MIN_VEL_FRACTION: f32 = 0.5;

/// Handoff-confirm probe: the measured |ω| must not EXCEED this multiple of
/// the observer's claim either. The check was originally one-sided (probe ≥
/// 0.5×claim — built against phantoms, which read ≈ 0) and a probe measuring
/// a rotor FASTER than the claim confirmed trivially: bench 2026-07-07
/// (gate-fix-5 and every spin-punch run of that day) — the unloaded rotor
/// slip-ratchets ahead of the 90 rad/s I/f ramp, the probe honestly reads
/// 520–580 el rad/s, the observer claims ~195 (still catching up), and the
/// "confirmed" handoff engages the tracker 2.8× below the real frequency —
/// the estimator then chases the rotor for ~100 ms with the current loop
/// slamming across the frame error (the reproducible ~6 A handoff spike).
/// A high-side mismatch now falls through to the strong-probe streak, whose
/// [`CONFIRM_SEED_PROBES`] escape seeds the observer FROM the measurement —
/// the right owner of "the rotor is real but the claim is wrong".
const CONFIRM_MAX_VEL_MULTIPLE: f32 = 2.0;

/// A single probe reading at/above this |ω| (el rad/s) that also fails the
/// velocity check HIGH (probe > [`CONFIRM_MAX_VEL_MULTIPLE`]×claim) seeds
/// the observer immediately — no [`CONFIRM_SEED_PROBES`] streak. Two
/// reasons. (1) The streak guards against LOW misreads (a probe sampling
/// the slow phase of a capture hunt honestly reads ~0); a fast read has no
/// such failure mode — it needs amps of settled short current that only
/// real back-EMF can drive (the worst bench artifact, the enable
/// transient, resolves to ω ≈ 46). (2) Retrying against a runaway rotor is
/// actively DANGEROUS: the rotor keeps accelerating through every Hold
/// cooldown and the short-circuit current of the next probe grows with ω
/// toward the OC trip — bench 2026-07-07 (confirm2-2): first probe 568
/// vs claim 196 correctly unconfirmed, rotor reached ~750 by the retry,
/// and the second probe's short current tripped the 10.8 A dq OC.
/// 300 ≈ 1.7× the handoff velocity: unreachable by any known artifact,
/// modest enough to catch the ratchet before the probe-current danger
/// zone (i_short ≈ λω/|Z| ≈ 6.7 A at 750).
const CONFIRM_FAST_SEED_VEL: f32 = 300.0;

/// Handoff-confirm probe: max |angle| disagreement (rad) between the probe's
/// rotor estimate and the observer's. A converged observer is within ~0.3
/// rad of truth and the probe within ~0.1; a rotor freewheeling AGAINST the
/// commanded direction (which the probe's ±90° convention cannot sign) shows
/// up here as a ~π disagreement and is rejected.
const CONFIRM_MAX_ANGLE_ERR_RAD: f32 = 1.0;

/// Cooldown (s) between confirm probes. A rotor being captured by the ramp
/// HUNTS around the synchronous speed (undamped ±40 rad/s swings in the
/// sims); a single 0.35 ms probe samples an instant of that hunt and can
/// honestly read near-zero on a genuinely captured rotor. An unconfirmed
/// probe therefore returns to Hold and retries — the hunt period is
/// ~200–300 ms, so successive probes land on different hunt phases and a
/// real capture confirms within a few tries. A phantom never does.
const CONFIRM_RETRY_S: f32 = 0.1;

/// Consecutive confirm probes that must each measure a genuinely spinning
/// rotor (probe |ω| ≥ [`CONFIRM_STRONG_PROBE_VEL`]) before the sequencer
/// stops retrying against the observer's claim and instead SEEDS the
/// observer from the probe measurement (same mechanics as the deadshort
/// catch). This is the hold-ratchet escape (bench 2026-07-06 late,
/// prof-hold-t3): during the 180 rad/s hold the observer ran away
/// 219→756 rad/s el (~+54 per retry, internal gates all green) while the
/// probe consistently measured a real captured rotor at 32–108 —
/// confirmation against the runaway claim is structurally unreachable, and
/// waiting for the [`HOLD_GIVEUP_S`] recycle throws away a good capture
/// the probe has already measured. Three consecutive strong reads make a
/// standing-rotor false positive implausible: each probe follows a
/// [`CONFIRM_SETTLE_CYCLES`] decay window, and the bench enable-transient
/// artifact resolves to ω≈46, below the catch floor.
const CONFIRM_SEED_PROBES: u8 = 3;

/// Hold-phase give-up (s): if the observer cannot pass a confirm probe for
/// this long at the ceiling, recycle the whole start (deadshort → ramp) and
/// have the caller reset the observer. This is what finally breaks a
/// phantom lock (the observer restarts from scratch while the ramp
/// re-captures the rotor) and re-catches a genuinely spinning rotor via
/// the deadshort's own measurement — the sooner the better when the rotor
/// has run away from the ramp and only a measured seed can close the loop
/// on it (sim: a runaway under a never-ready observer builds toward the
/// dq overcurrent within ~1.5 s). Still several confirm retries above the
/// ~0.3 s a healthy capture needs to damp its hunt and confirm.
const HOLD_GIVEUP_S: f32 = 1.0;

/// Where the open-loop sequencer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartupPhase {
    /// Not sequencing — the real source (sensor / ready observer) drives.
    Inactive,
    /// Flying-restart probe (Phase B): the bridge is shorted (zero voltage)
    /// and the back-EMF-driven current slope is measured to catch an
    /// already-spinning rotor before the cold-start ramp.
    Deadshort,
    /// Linearly ramping the open-loop velocity 0 → ceiling (from a cold
    /// start this begins at velocity 0 from the last output angle — there
    /// is no align phase, see [`RAMP_CURRENT_SOFT_START_S`]).
    Ramp,
    /// Holding at the ceiling, waiting for the observer to converge.
    Hold,
    /// Handoff-confirm probe: the observer passed its readiness gates, but
    /// before closed loop engages the bridge is shorted for ~2 ms and the
    /// back-EMF-driven dI/dt must corroborate the claimed rotation. A
    /// converged observer whose flux is actually the machine's own residual
    /// inverter distortion (phantom lock — internally indistinguishable
    /// from a real rotation, see `BackEmfObserver::is_ready`) measures ≈ 0
    /// here and is sent back to the ramp instead of deadlocking closed
    /// loop on a standing rotor.
    Confirm,
    /// Hall-dropout recovery: fixed-velocity nudge from the last known angle.
    Recover,
}

impl StartupPhase {
    /// Short lowercase name for transition logs.
    pub fn name(self) -> &'static str {
        match self {
            Self::Inactive => "inactive",
            Self::Deadshort => "deadshort",
            Self::Ramp => "ramp",
            Self::Hold => "hold",
            Self::Confirm => "confirm",
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
    /// The observer passed its readiness gates this cycle — the machine has
    /// entered the [`StartupPhase::Confirm`] probe (informational; the
    /// actual handoff happens when the probe confirms).
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

/// Outcome of feeding one cycle of shorted-winding current to the
/// handoff-confirm probe ([`StartupPhase::Confirm`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConfirmResult {
    /// Still probing — keep the bridge shorted.
    Probing,
    /// The measured back-EMF corroborates the observer: hand off (the
    /// sequencer has deactivated itself). `velocity` is the probe's own
    /// |ω| estimate, for logging.
    Confirmed { velocity: f32 },
    /// The probe could not corroborate the claimed rotation THIS time —
    /// either a phantom lock, or a genuinely captured rotor sampled at the
    /// slow phase of its capture hunt. The sequencer returned to Hold and
    /// will re-probe after [`CONFIRM_RETRY_S`]; a phantom that never
    /// confirms is broken up by the [`HOLD_GIVEUP_S`] recycle.
    Unconfirmed { velocity: f32 },
    /// The probe disagrees with the observer's claim, but it has measured a
    /// genuinely spinning rotor ([`CONFIRM_SEED_PROBES`] consecutive reads
    /// ≥ the deadshort catch floor): the rotor is real and the OBSERVER is
    /// the diverged party (the bench hold-ratchet, see
    /// [`CONFIRM_SEED_PROBES`]). Seed the observer from `(angle, velocity)`
    /// and go straight to closed loop — the sequencer has deactivated
    /// itself; same contract as [`DeadshortResult::Caught`].
    SeedAndHandoff { angle: f32, velocity: f32 },
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
    /// Time spent in Hold without a confirmed handoff (s) — drives the
    /// [`HOLD_GIVEUP_S`] recycle.
    hold_time: f32,
    /// Remaining cooldown (s) before the next confirm probe may start.
    confirm_cooldown: f32,
    /// Consecutive confirm probes that measured a genuinely spinning rotor
    /// while still failing the observer-claim comparison — the hold-ratchet
    /// escape counter (see [`CONFIRM_SEED_PROBES`]). Reset by any weak
    /// probe, a fresh start, and the give-up recycle.
    strong_probes: u8,
    /// The Hold → Deadshort give-up recycle fired this tick: the caller
    /// must reset the observer (a phantom lock is the usual reason the
    /// hold never confirmed). Cleared on read via [`take_recycled`].
    recycled: bool,
    // ── Deadshort probe state (Phase B) ──
    ds_settle: u16,
    /// The probe was entered with drive current flowing (Confirm, or the
    /// Hold give-up recycle) rather than from an unenergised bridge (cold
    /// start). Drive residual is indistinguishable from back-EMF until it
    /// has decayed, so the settle countdown must run to completion — the
    /// over-cap early skip is only valid when the bridge was off and any
    /// large current can ONLY be back-EMF. Without this, a sensorless start
    /// at iq ≥ [`DEADSHORT_MAX_CURRENT_A`] skips the settle, one sample of
    /// drive current "completes" the window, and `short_current_estimate`
    /// fabricates a fast rotor ~π off the real one (phantom handoff).
    ds_settle_must_complete: bool,
    /// Samples accumulated into the probe-window current average.
    ds_cycles: u16,
    /// Running sums of the α/β current over the probe window — the
    /// settled-current estimator works on the window AVERAGE (see
    /// [`short_current_estimate`]).
    ds_sum_alpha: f32,
    ds_sum_beta: f32,
    ds_elapsed: f32,
    // ── ISR log rate limit (token bucket) ──
    /// Log frames still allowed in the current window.
    log_tokens: u8,
    /// Elapsed time in the current window (s).
    log_window_t: f32,
    /// Frames dropped in the current window.
    log_suppressed: u16,
}

/// ISR log rate limit: window length (s) and frames allowed per window.
///
/// Every startup transition logs multi-arg defmt frames straight from the
/// current-loop ISR. One clean start emits a handful over seconds — free
/// (the reason logging from the ISR was acceptable at all). Restart CHURN
/// (trust loss → recycle → deadshort → ramp → hold → probe, several times
/// a second) is a different regime: bench 2026-07-07 (freq-led first
/// trial) measured single ISR runs of 26k cycles and 126–138% sustained
/// ISR load, starving the host command pump into deadman territory. The
/// bucket bounds the sustained frame rate; [`SensorlessStartup::log_tick`]
/// reports what was dropped once per window so the churn itself stays
/// visible in the log.
const LOG_WINDOW_S: f32 = 1.0;
const LOG_TOKENS_PER_WINDOW: u8 = 10;

impl Default for SensorlessStartup {
    fn default() -> Self {
        Self {
            phase: StartupPhase::Inactive,
            angle: 0.0,
            velocity: 0.0,
            dir: 1.0,
            handoff_vel: DEFAULT_HANDOFF_VEL,
            timer: 0.0,
            hold_time: 0.0,
            confirm_cooldown: 0.0,
            strong_probes: 0,
            recycled: false,
            ds_settle: 0,
            ds_settle_must_complete: false,
            ds_cycles: 0,
            ds_sum_alpha: 0.0,
            ds_sum_beta: 0.0,
            ds_elapsed: 0.0,
            log_tokens: LOG_TOKENS_PER_WINDOW,
            log_window_t: 0.0,
            log_suppressed: 0,
        }
    }
}

impl SensorlessStartup {
    /// Begin a sensorless start from `angle0` in `direction` (sign of the
    /// commanded torque/velocity). Opens with the deadshort flying-restart
    /// probe (Phase B): if the rotor is already spinning it is caught and
    /// handed straight to the observer; otherwise the machine falls through to
    /// the ramp→handoff cold start (Phase A; no align — see
    /// [`RAMP_CURRENT_SOFT_START_S`]).
    pub fn begin_cold_start(&mut self, angle0: f32, direction: f32) {
        self.phase = StartupPhase::Deadshort;
        self.angle = wrap_angle(angle0);
        self.velocity = 0.0;
        self.dir = if direction < 0.0 { -1.0 } else { 1.0 };
        self.handoff_vel = DEFAULT_HANDOFF_VEL;
        self.timer = DEFAULT_OPENLOOP_TIME_S + DEFAULT_RAMP_TIME_S;
        self.hold_time = 0.0;
        self.confirm_cooldown = 0.0;
        self.strong_probes = 0;
        self.ds_settle = DEADSHORT_SETTLE_CYCLES;
        // Cold start: the bridge was off, so a large settled current can
        // only be back-EMF — the over-cap settle skip is valid here.
        self.ds_settle_must_complete = false;
        self.ds_cycles = 0;
        self.ds_elapsed = 0.0;
    }

    /// True while a probe needs the bridge held shorted (zero voltage): the
    /// cold-start deadshort or the handoff-confirm probe. The driver honors
    /// this instead of normal commutation.
    pub fn wants_short(&self) -> bool {
        matches!(self.phase, StartupPhase::Deadshort | StartupPhase::Confirm)
    }

    /// Advance the ISR log rate-limit window (call once per control cycle,
    /// active or not — see [`LOG_WINDOW_S`]). Returns the number of frames
    /// dropped in the window that just closed (0 while a window is still
    /// open) so the caller can log a one-frame suppression summary.
    pub fn log_tick(&mut self, dt: f32) -> u16 {
        self.log_window_t += dt;
        if self.log_window_t < LOG_WINDOW_S {
            return 0;
        }
        self.log_window_t = 0.0;
        self.log_tokens = LOG_TOKENS_PER_WINDOW;
        core::mem::take(&mut self.log_suppressed)
    }

    /// Consume one log-frame token; `false` means the frame must be dropped
    /// (it is counted for the next [`log_tick`](Self::log_tick) summary).
    pub fn log_allow(&mut self) -> bool {
        if self.log_tokens > 0 {
            self.log_tokens -= 1;
            true
        } else {
            self.log_suppressed = self.log_suppressed.saturating_add(1);
            false
        }
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
        self.hold_time = 0.0;
        self.confirm_cooldown = 0.0;
        self.strong_probes = 0;
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
        self.hold_time = 0.0;
        self.confirm_cooldown = 0.0;
    }

    /// Whether the give-up recycle fired since the last call (one-shot).
    /// The caller must reset the observer when it did: the recycle exists
    /// because the observer spent [`HOLD_GIVEUP_S`] failing to confirm —
    /// its state is presumed to be a phantom lock.
    pub fn take_recycled(&mut self) -> bool {
        core::mem::take(&mut self.recycled)
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

    /// Torque-command scale (0..=1) for the current cycle: ramps in over
    /// [`RAMP_CURRENT_SOFT_START_S`] at ramp entry, full command everywhere
    /// else (Deadshort holds the bridge shorted, so the value is moot there).
    pub fn current_scale(&self) -> f32 {
        match self.phase {
            StartupPhase::Ramp => {
                let ramp_elapsed = (DEFAULT_OPENLOOP_TIME_S + DEFAULT_RAMP_TIME_S) - self.timer;
                clamp_f32(ramp_elapsed / RAMP_CURRENT_SOFT_START_S, 0.0, 1.0)
            }
            _ => 1.0,
        }
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
        self.confirm_cooldown = (self.confirm_cooldown - dt).max(0.0);

        match self.phase {
            // The probes are driven by `feed_deadshort`/`feed_confirm`, not
            // `tick`; if ticked, hold the angle (the driver applies zero
            // voltage anyway).
            StartupPhase::Inactive | StartupPhase::Deadshort | StartupPhase::Confirm => {
                return StartupOutput {
                    angle: self.angle,
                    velocity: 0.0,
                    handoff: false,
                };
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
                    self.hold_time = 0.0;
                }
            }
            StartupPhase::Hold => {
                // Velocity at the ceiling; hold it. A hold that cannot get
                // a confirmed handoff within HOLD_GIVEUP_S is recycled
                // through a fresh deadshort→ramp start: a phantom-locked
                // observer never confirms (the caller resets it, see
                // take_recycled), and a genuinely spinning rotor is
                // re-caught by the deadshort.
                self.hold_time += dt;
                if self.hold_time >= HOLD_GIVEUP_S {
                    self.phase = StartupPhase::Deadshort;
                    self.velocity = 0.0;
                    self.timer = DEFAULT_OPENLOOP_TIME_S + DEFAULT_RAMP_TIME_S;
                    self.hold_time = 0.0;
                    self.confirm_cooldown = 0.0;
                    self.strong_probes = 0;
                    self.recycled = true;
                    self.ds_settle = DEADSHORT_SETTLE_CYCLES;
                    // The ramp/hold drive current is still decaying into
                    // this short — never mistake it for back-EMF.
                    self.ds_settle_must_complete = true;
                    self.ds_cycles = 0;
                    self.ds_elapsed = 0.0;
                    return StartupOutput {
                        angle: self.angle,
                        velocity: 0.0,
                        handoff: false,
                    };
                }
            }
            StartupPhase::Recover => {
                // Velocity already at the recovery speed; hold it.
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
        // dragged the rotor a while (velocity ≥ 35% of handoff): the catch
        // transient at ramp entry (the rotor being yanked into sync from an
        // unknown angle) can swing the rotor on its magnetic spring, and the
        // observer can lock onto that swing and read "ready" at speed — on
        // the align-era bench a 20% gate let that artifact hand off the
        // instant the gate opened (openloop_vel 12.0, observer 755). Every
        // real runaway observed fired at openloop_vel ≥ 23; the artifact
        // cases sat at 0–12.
        let ramp_moving = self.velocity.abs() >= self.handoff_vel * 0.35;
        let fast_enough = (self.velocity.abs() >= self.handoff_vel
            && observer_vel.abs() >= self.handoff_vel * 0.5)
            || (ramp_moving && observer_vel.abs() >= self.handoff_vel);
        let handoff = observer_ready && fast_enough && self.confirm_cooldown <= 0.0;
        let velocity = self.velocity;
        if handoff {
            // Gates passed → confirm before engaging closed loop: short the
            // bridge and demand the back-EMF actually be there
            // (`feed_confirm`). A phantom-locked observer passes every
            // internal gate; this probe is the external check it cannot.
            self.phase = StartupPhase::Confirm;
            self.ds_settle = CONFIRM_SETTLE_CYCLES;
            // Amps of commanded drive current are flowing at entry; the
            // settle window exists to decay them and must run in full.
            self.ds_settle_must_complete = true;
            self.ds_cycles = 0;
            self.ds_elapsed = 0.0;
        }

        StartupOutput {
            angle: self.angle,
            velocity,
            handoff,
        }
    }

    /// Feed one shorted-winding current sample to the deadshort probe (Phase B).
    ///
    /// `l`/`lambda` are the observer's motor model. The first call captures the
    /// baseline current (the back-EMF response is the *change* from there); over
    /// the next [`DEADSHORT_CYCLES`] (or until |i| hits the abort cap) it
    /// accumulates dI/dt and estimates the back-EMF `e = −L·dI/dt`, hence the
    /// rotor angle and speed (see [`short_current_estimate`]). A spinning rotor →
    /// `Caught` and the machine deactivates (the manager seeds the observer);
    /// standstill / too slow → it falls through to the cold-start ramp,
    /// returning `Probing`.
    pub fn feed_deadshort(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        dt: f32,
        r: f32,
        l: f32,
        lambda: f32,
    ) -> DeadshortResult {
        if self.phase != StartupPhase::Deadshort {
            return DeadshortResult::Probing;
        }
        // Let the bridge-enable transient AND the short current's own
        // exponential settle decay before averaging (see
        // DEADSHORT_SETTLE_CYCLES). On a COLD start a current already at
        // the abort cap can only be back-EMF (the bridge was off) — skip
        // straight to the probe rather than sit shorted on a large
        // current. On the Hold give-up recycle the cap is met by the
        // decaying DRIVE current, which proves nothing about the rotor:
        // the settle must run in full (`ds_settle_must_complete`).
        if self.ds_settle > 0 {
            let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
            if i_mag < DEADSHORT_MAX_CURRENT_A || self.ds_settle_must_complete {
                self.ds_settle -= 1;
                return DeadshortResult::Probing;
            }
            self.ds_settle = 0;
        }
        // Accumulate the settled short-circuit current over the window.
        if self.ds_cycles == 0 {
            self.ds_sum_alpha = 0.0;
            self.ds_sum_beta = 0.0;
            self.ds_elapsed = 0.0;
        } else {
            self.ds_elapsed += dt;
        }
        self.ds_cycles += 1;
        self.ds_sum_alpha += i_alpha;
        self.ds_sum_beta += i_beta;
        let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        if self.ds_cycles < DEADSHORT_CYCLES && i_mag < DEADSHORT_MAX_CURRENT_A {
            return DeadshortResult::Probing;
        }

        // Probe window complete — rotor estimate from the settled current.
        // When the CURRENT CAP ended the window early, the rotor is
        // unambiguously fast (|i_short| ≥ cap ⇒ |e| ≥ cap·R ⇒ |ω| ≥
        // cap·R/λ) but the averaged phasor is still RISING toward its
        // steady state (τ = L/R) and the |i|-based ω is only a lower
        // bound — on a high-τ motor it can sit under the catch floor at
        // twice the floor's true speed. Skip the floor for a capped
        // probe: the floor exists to send SLOW coasting rotors to the
        // ramp, and ramping from near-zero frequency INTO a proven-fast
        // rotor is the worse failure (full-speed slip at engage).
        let capped = i_mag >= DEADSHORT_MAX_CURRENT_A;
        let n = f32::from(self.ds_cycles);
        match short_current_estimate(
            self.ds_sum_alpha / n,
            self.ds_sum_beta / n,
            self.ds_elapsed,
            r,
            l,
            lambda,
            self.dir,
            if capped { 0.0 } else { DEADSHORT_MIN_CATCH_VEL },
        ) {
            Some((angle, velocity)) => {
                self.deactivate();
                DeadshortResult::Caught { angle, velocity }
            }
            None => {
                // Standstill / too slow → ramp cold start from here (no
                // align — the soft-started rotating field catches the rotor).
                self.phase = StartupPhase::Ramp;
                self.velocity = 0.0;
                DeadshortResult::Probing
            }
        }
    }

    /// Feed one shorted-winding current sample to the handoff-confirm probe
    /// ([`StartupPhase::Confirm`]; same measurement as [`feed_deadshort`],
    /// different question and different judge).
    ///
    /// `claim` is the observer's angle/velocity under test. On `Confirmed`
    /// the sequencer deactivates (handoff complete — the observer keeps its
    /// own converged state, no reseed). On `Unconfirmed` it returns to Hold
    /// for a cooldown-paced retry (see [`ConfirmResult`]).
    #[allow(clippy::too_many_arguments)] // motor params travel as plain scalars, same as the estimator
    pub fn feed_confirm(
        &mut self,
        i_alpha: f32,
        i_beta: f32,
        dt: f32,
        r: f32,
        l: f32,
        lambda: f32,
        claim: super::provider::PhaseOutput,
    ) -> ConfirmResult {
        if self.phase != StartupPhase::Confirm {
            return ConfirmResult::Probing;
        }
        // Drive-current decay into the short (τ = L/R); see
        // CONFIRM_SETTLE_CYCLES. NO over-cap skip here, ever: Confirm is
        // always entered with amps of commanded drive current flowing, and
        // treating that residual as rotor evidence is exactly the phantom
        // handoff this probe exists to prevent (a start at iq ≥ the cap
        // would otherwise "complete" the window on one drive-current
        // sample and seed the observer ~π off the rotor).
        if self.ds_settle > 0 {
            self.ds_settle -= 1;
            return ConfirmResult::Probing;
        }
        // Accumulate the settled short-circuit current over the window.
        if self.ds_cycles == 0 {
            self.ds_sum_alpha = 0.0;
            self.ds_sum_beta = 0.0;
            self.ds_elapsed = 0.0;
        } else {
            self.ds_elapsed += dt;
        }
        self.ds_cycles += 1;
        self.ds_sum_alpha += i_alpha;
        self.ds_sum_beta += i_beta;
        let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        if self.ds_cycles < DEADSHORT_CYCLES && i_mag < DEADSHORT_MAX_CURRENT_A {
            return ConfirmResult::Probing;
        }

        // Window complete — rotor estimate from the settled current (no
        // catch floor here: the question is agreement with the observer,
        // not absolute speed; a standing rotor honestly estimates ~0).
        let n = f32::from(self.ds_cycles);
        let (angle, omega) = match short_current_estimate(
            self.ds_sum_alpha / n,
            self.ds_sum_beta / n,
            self.ds_elapsed,
            r,
            l,
            lambda,
            self.dir,
            0.0,
        ) {
            Some((angle, velocity)) => (angle, velocity.abs()),
            // Degenerate parameters (no motor params baked) — cannot
            // corroborate anything; behave like a failed probe.
            None => (claim.angle, 0.0),
        };
        if self.log_allow() {
            info!(
                "probe: i_avg=({},{}) -> omega={} angle={} (window_us={})",
                self.ds_sum_alpha / n,
                self.ds_sum_beta / n,
                omega,
                angle,
                self.ds_elapsed * 1e6
            );
        }

        // Two-sided: a probe far BELOW the claim is the phantom signature,
        // far ABOVE it means the claim lags a real runaway rotor (see
        // CONFIRM_MAX_VEL_MULTIPLE) — both are "do not hand off to this
        // claim"; the strong-probe streak below decides whether the
        // measurement itself is trustworthy enough to seed from instead.
        let claim_vel = claim.velocity.abs();
        let vel_ok = omega >= CONFIRM_MIN_VEL_FRACTION * claim_vel
            && omega <= CONFIRM_MAX_VEL_MULTIPLE * claim_vel;
        let angle_ok =
            crate::foc::angle_difference(angle, claim.angle).abs() <= CONFIRM_MAX_ANGLE_ERR_RAD;
        if vel_ok && angle_ok {
            self.deactivate();
            return ConfirmResult::Confirmed { velocity: omega };
        }

        // Unambiguous fast rotation the claim does not track: seed from
        // this single measurement NOW — every retry probes a faster rotor
        // and walks the short current toward the OC trip (see
        // CONFIRM_FAST_SEED_VEL).
        if omega >= CONFIRM_FAST_SEED_VEL && omega > CONFIRM_MAX_VEL_MULTIPLE * claim_vel {
            self.deactivate();
            return ConfirmResult::SeedAndHandoff {
                angle,
                velocity: self.dir * omega,
            };
        }

        // Streak of probes that measured a genuinely spinning rotor while
        // still failing the claim comparison; a weak read restarts it —
        // seeding demands CONSECUTIVE physical evidence.
        if omega >= CONFIRM_STRONG_PROBE_VEL {
            self.strong_probes += 1;
        } else {
            self.strong_probes = 0;
        }
        if self.strong_probes >= CONFIRM_SEED_PROBES {
            // Hold-ratchet escape: the probe keeps measuring a genuinely
            // spinning rotor while failing the comparison against the
            // observer's claim — the observer is the diverged party. Seed
            // it from the probe (the physical measurement) and hand off,
            // exactly like the deadshort catch. Retrying against a runaway
            // claim can never succeed, and the give-up recycle would throw
            // away a capture the probe has already measured three times.
            self.deactivate();
            return ConfirmResult::SeedAndHandoff {
                angle,
                velocity: self.dir * omega,
            };
        }

        // Unconfirmed: back to Hold, keep dragging, re-probe after the
        // cooldown — a genuinely captured rotor sampled at the slow
        // phase of its hunt confirms on a later try; a phantom never
        // does and the HOLD_GIVEUP_S recycle breaks it up.
        self.phase = StartupPhase::Hold;
        self.velocity = self.dir * self.handoff_vel.max(self.velocity.abs());
        self.confirm_cooldown = CONFIRM_RETRY_S;
        ConfirmResult::Unconfirmed { velocity: omega }
    }
}

/// Rotor estimate from the SETTLED short-circuit current phasor.
///
/// A shorted stator at speed obeys `e = (R + jωL)·i` in steady state, and
/// on a low-τ motor (τ = L/R ≈ 0.19 ms ≈ 4 PWM periods on the ZD2808) the
/// current IS in steady state by the time the probe window opens — the
/// settle window exists precisely to let the entry transient decay. The
/// back-EMF information is therefore in the current's MAGNITUDE and PHASE,
/// not in its slope: the original `e = −L·ΔI/Δt` estimate measured only
/// the residual rotation of the settled phasor and under-read |ω| by
/// roughly a factor ω·τ (bench 2026-07-06 late, spin-gentle-180-2
/// probe-raw windows: probes read 56–102 rad/s while the settled
/// 5.5–7.6 A short current pinned the rotor at 530–800 — a genuine I/f
/// runaway that the old probe kept vetoing as "unconfirmed" against a
/// CORRECT observer).
///
/// `i_alpha`/`i_beta` is the current phasor averaged over the probe
/// window (noise suppression); the average lags "now" by half the window,
/// compensated in the returned angle. |ω| from `|e| = |Z(ω)|·|i|` with one
/// fixed-point refinement of the ωL term; the angle chain: the current
/// lags the back-EMF by the impedance angle `φ_z = atan(ωL/R)` in the
/// rotation direction, and the back-EMF leads the rotor flux by
/// 90°·`dir`. `dir` is the commanded direction, taken as the rotation
/// sign (true for the kick-push restart; a rotor freewheeling *against*
/// the command is the known v1 limitation — the PLL would have to pull a
/// ±180° seed, which it can't). Returns `None` below `min_vel`
/// (standstill / too slow) or with degenerate parameters.
#[allow(clippy::too_many_arguments)]
fn short_current_estimate(
    i_alpha: f32,
    i_beta: f32,
    window_s: f32,
    r: f32,
    l: f32,
    lambda: f32,
    dir: f32,
    min_vel: f32,
) -> Option<(f32, f32)> {
    if lambda <= 1e-9 || r <= 0.0 || l < 0.0 {
        return None;
    }
    let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
    // |e| = |i|·√(R² + (ωL)²); ωL ≪ R below ~2 krad/s el on the bench
    // motor, one refinement pass is plenty.
    let omega0 = i_mag * r / lambda;
    let omega = i_mag * sqrtf(r * r + (omega0 * l) * (omega0 * l)) / lambda;
    if omega < min_vel {
        return None;
    }
    let phi_z = atan2f(omega * l, r);
    // Sign chain: `0 = R·i + L·di/dt + e` ⇒ the settled short current
    // OPPOSES the back-EMF, `i = −e/(R + jωL)` — hence the π. On top of
    // that: half-window advance (the average lags "now"), the impedance
    // lag (i lags −e by φ_z in the rotation direction), and e leading the
    // rotor flux by 90°·dir. Caught by the independent-plant sim test
    // (deadshort_catches_a_spinning_rotor_on_start) — the local unit
    // tests share the synth convention and cannot see a global flip.
    let angle_e =
        atan2f(i_beta, i_alpha) + core::f32::consts::PI + dir * (phi_z + omega * window_s * 0.5);
    let angle = wrap_angle(angle_e - dir * core::f32::consts::FRAC_PI_2);
    Some((angle, dir * omega))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 20_000.0;

    /// Feed the deadshort probe a standstill (zero-current) stream until it
    /// gives up and falls through to the cold-start ramp. Phase A tests use
    /// this to get past the Phase B probe that now opens every cold start.
    fn skip_deadshort(sm: &mut SensorlessStartup) {
        for _ in 0..=(DEADSHORT_SETTLE_CYCLES + DEADSHORT_CYCLES) {
            sm.feed_deadshort(0.0, 0.0, DT, 0.1, 200e-6, 0.01);
            if sm.phase() != StartupPhase::Deadshort {
                break;
            }
        }
        assert_eq!(sm.phase(), StartupPhase::Ramp);
    }

    /// Feed zero current through the settle window so the next
    /// `feed_deadshort` call captures the probe baseline.
    fn skip_settle(sm: &mut SensorlessStartup) {
        for _ in 0..DEADSHORT_SETTLE_CYCLES {
            assert_eq!(
                sm.feed_deadshort(0.0, 0.0, DT, 0.1, 200e-6, 0.01),
                DeadshortResult::Probing
            );
            assert_eq!(sm.phase(), StartupPhase::Deadshort);
        }
    }

    /// Begin a cold start and skip straight to the Ramp phase.
    fn cold_start_to_ramp(angle: f32, dir: f32) -> SensorlessStartup {
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
    fn cold_start_sequences_ramp_hold() {
        let mut sm = cold_start_to_ramp(1.0, 1.0);
        assert!(sm.is_active());

        let hist = run(&mut sm, DEFAULT_RAMP_TIME_S + 0.3, 3.0);
        let phases: Vec<_> = hist.iter().map(|(p, _)| *p).collect();
        assert_eq!(phases, vec![StartupPhase::Ramp, StartupPhase::Hold]);
        // Settles at or above the handoff velocity.
        assert!(sm.velocity() >= DEFAULT_HANDOFF_VEL);
    }

    #[test]
    fn ramp_current_soft_starts() {
        let mut sm = cold_start_to_ramp(0.0, 1.0);
        // Ramp entry: command fully suppressed, ramps in linearly.
        assert!(sm.current_scale() < 0.1, "got {}", sm.current_scale());
        let half = (RAMP_CURRENT_SOFT_START_S / 2.0 / DT) as usize;
        for _ in 0..half {
            sm.tick(DT, 1.0, false, 0.0);
        }
        let s = sm.current_scale();
        assert!((0.3..0.7).contains(&s), "mid-soft-start scale {s}");
        // Past the soft-start window (still ramping): full command.
        for _ in 0..half + 400 {
            sm.tick(DT, 1.0, false, 0.0);
        }
        assert_eq!(sm.phase(), StartupPhase::Ramp);
        assert_eq!(sm.current_scale(), 1.0);
        // Hold: never scaled.
        run(&mut sm, 1.0, 3.0);
        assert_eq!(sm.current_scale(), 1.0);
    }

    #[test]
    fn ramp_rotates_from_the_first_cycle() {
        // No align: the field must start moving immediately — a fixed-angle
        // dwell is exactly the undamped-spring resonance this design removes.
        let mut sm = cold_start_to_ramp(0.5, 1.0);
        let mut o = sm.tick(DT, 2.0, false, 0.0);
        for _ in 0..200 {
            o = sm.tick(DT, 2.0, false, 0.0);
        }
        assert!(o.velocity > 0.0, "field velocity must grow from tick one");
        assert!(o.angle != 0.5, "angle must advance");
    }

    #[test]
    fn ramp_is_monotonic_and_signed_by_direction() {
        let mut sm = cold_start_to_ramp(0.0, -1.0); // reverse
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
        let mut lo = cold_start_to_ramp(0.0, 1.0);
        let mut hi = cold_start_to_ramp(0.0, 1.0);
        run(&mut lo, DEFAULT_RAMP_TIME_S + 0.2, 0.0); // no current → floor ceiling
        run(&mut hi, DEFAULT_RAMP_TIME_S + 0.2, CURRENT_REF_A); // full current → max ceiling
        assert!(hi.velocity() > lo.velocity() + 1.0);
        assert!((lo.velocity() - DEFAULT_HANDOFF_VEL).abs() < 1.0);
    }

    #[test]
    fn no_handoff_until_observer_ready_and_fast() {
        let mut sm = cold_start_to_ramp(0.0, 1.0);
        // Run past the ramp with the observer NOT ready — never hands
        // off, and ends solidly in Hold above the handoff speed.
        for _ in 0..((DEFAULT_RAMP_TIME_S / DT) as usize + 4_000) {
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
        // rotor (≥35% of handoff): at ramp entry a "ready" observer at
        // speed is the catch-swing artifact (bench 2026-07-06: observer
        // 585–786 rad/s at openloop 0–1.2) and must be ignored.
        let mut sm = cold_start_to_ramp(0.0, 1.0);
        run(&mut sm, 0.005, 5.0); // ramp just started
        assert_eq!(sm.phase(), StartupPhase::Ramp);
        assert!(sm.velocity() < DEFAULT_HANDOFF_VEL * 0.2);
        let o = sm.tick(DT, 5.0, true, 400.0);
        assert!(
            !o.handoff,
            "align-swing false-ready must not hand off at ramp entry"
        );
        // Ramp genuinely dragging → the runaway path fires below the
        // nominal handoff velocity. Duration scaled to the ramp: past the
        // 35% ramp_moving gate, still below the handoff velocity.
        run(&mut sm, DEFAULT_RAMP_TIME_S * 0.25, 5.0);
        assert!(sm.velocity() >= DEFAULT_HANDOFF_VEL * 0.2);
        assert!(sm.velocity() < DEFAULT_HANDOFF_VEL);
        let o = sm.tick(DT, 5.0, true, 400.0);
        assert!(
            o.handoff,
            "ready+fast observer must take over a runaway rotor"
        );
    }

    // ── Phase B: deadshort flying restart ──

    /// Synthesize the SETTLED short-circuit current phasor for a rotor
    /// spinning at `omega` (elec rad/s) whose flux sits at `theta` at the
    /// END of the probe window: `i = e/(R + jωL)` with `e` leading the flux
    /// by 90°·dir and the current lagging `e` by the impedance angle. Fed
    /// CONSTANT through the window, so the estimator's half-window advance
    /// is pre-compensated here.
    fn synth_short_current(
        omega: f32,
        theta: f32,
        r: f32,
        l: f32,
        lambda: f32,
        dir: f32,
    ) -> (f32, f32) {
        let window = f32::from(DEADSHORT_CYCLES - 1) * DT;
        let z = (r * r + (omega * l) * (omega * l)).sqrt();
        let i_mag = lambda * omega / z;
        let phi_z = (omega * l).atan2(r);
        // i = −e/(R+jωL): π offset from the back-EMF direction (see
        // short_current_estimate's sign chain).
        let ang = theta
            + core::f32::consts::PI
            + dir * (core::f32::consts::FRAC_PI_2 - phi_z - omega * window * 0.5);
        (i_mag * ang.cos(), i_mag * ang.sin())
    }

    #[test]
    fn short_current_estimate_recovers_rotor() {
        use crate::foc::angle_difference;
        let (r, l, lambda, omega, theta) = (0.1, 150e-6, 0.008, 300.0, -0.8);
        let window = f32::from(DEADSHORT_CYCLES - 1) * DT;
        let (i_a, i_b) = synth_short_current(omega, theta, r, l, lambda, 1.0);
        let (angle, vel) =
            short_current_estimate(i_a, i_b, window, r, l, lambda, 1.0, DEADSHORT_MIN_CATCH_VEL)
                .unwrap();
        assert!(angle_difference(angle, theta).abs() < 0.05, "angle {angle}");
        assert!((vel - omega).abs() < 0.05 * omega, "vel {vel}");
        // A barely-moving rotor (1% of the current) is below the catch
        // floor → None.
        assert!(
            short_current_estimate(
                i_a * 0.01,
                i_b * 0.01,
                window,
                r,
                l,
                lambda,
                1.0,
                DEADSHORT_MIN_CATCH_VEL
            )
            .is_none()
        );
    }

    #[test]
    fn deadshort_catches_spinning_rotor() {
        use crate::foc::angle_difference;
        let (r, l, lambda, omega, theta) = (0.15, 200e-6, 0.005, 250.0, 1.2);
        let (i_a, i_b) = synth_short_current(omega, theta, r, l, lambda, 1.0);

        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, 1.0); // dir matches the (forward) rotation
        assert_eq!(sm.phase(), StartupPhase::Deadshort);
        assert!(sm.wants_short());
        skip_settle(&mut sm);

        // The settled current phasor is fed constant through the window;
        // the last sample completes it.
        let mut res = DeadshortResult::Probing;
        for _ in 0..DEADSHORT_CYCLES {
            assert_eq!(res, DeadshortResult::Probing);
            res = sm.feed_deadshort(i_a, i_b, DT, r, l, lambda);
        }
        match res {
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
    fn deadshort_standstill_falls_through_to_ramp() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.7, 1.0);
        // No back-EMF (zero dI) over the whole settle+probe → no catch →
        // cold start.
        for _ in 0..=(DEADSHORT_SETTLE_CYCLES + DEADSHORT_CYCLES) {
            sm.feed_deadshort(0.0, 0.0, DT, 0.1, 200e-6, 0.01);
        }
        assert_eq!(sm.phase(), StartupPhase::Ramp);
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

    // ── Handoff-confirm probe ──

    use crate::foc::phase::PhaseOutput;

    /// Cold start driven into Hold (observer never ready), ready to fire
    /// the handoff gate.
    fn ramp_to_hold() -> SensorlessStartup {
        let mut sm = cold_start_to_ramp(0.0, 1.0);
        run(&mut sm, DEFAULT_RAMP_TIME_S + 0.2, 3.0);
        assert_eq!(sm.phase(), StartupPhase::Hold);
        sm
    }

    /// Feed the confirm probe a constant settled current phasor through its
    /// settle + probe window; returns the final result.
    fn run_confirm(
        sm: &mut SensorlessStartup,
        i: (f32, f32),
        r: f32,
        l: f32,
        lambda: f32,
        claim: PhaseOutput,
    ) -> ConfirmResult {
        for _ in 0..CONFIRM_SETTLE_CYCLES {
            assert_eq!(
                sm.feed_confirm(i.0, i.1, DT, r, l, lambda, claim),
                ConfirmResult::Probing
            );
        }
        let mut res = ConfirmResult::Probing;
        for _ in 0..DEADSHORT_CYCLES {
            assert_eq!(res, ConfirmResult::Probing);
            res = sm.feed_confirm(i.0, i.1, DT, r, l, lambda, claim);
        }
        res
    }

    #[test]
    fn handoff_gate_runs_confirm_probe_not_instant_handoff() {
        let mut sm = ramp_to_hold();
        let o = sm.tick(DT, 3.0, true, DEFAULT_HANDOFF_VEL);
        assert!(o.handoff, "gates passed must be reported");
        assert_eq!(sm.phase(), StartupPhase::Confirm);
        assert!(sm.wants_short(), "confirm probe needs the bridge shorted");
        assert!(sm.is_active(), "handoff is NOT complete until confirmed");
    }

    #[test]
    fn confirm_rejects_still_rotor_and_retries_after_cooldown() {
        let (r, l, lambda) = (0.1, 150e-6, 1.145e-3);
        let claim = PhaseOutput {
            angle: 1.0,
            velocity: DEFAULT_HANDOFF_VEL,
        };
        let mut sm = ramp_to_hold();
        sm.tick(DT, 3.0, true, claim.velocity);
        // Zero current over the whole window: no back-EMF where the
        // observer claims rotation → unconfirmed, back to Hold.
        let res = run_confirm(&mut sm, (0.0, 0.0), r, l, lambda, claim);
        assert!(matches!(res, ConfirmResult::Unconfirmed { .. }));
        assert_eq!(sm.phase(), StartupPhase::Hold);
        assert!(sm.is_active());
        // Cooldown: the gate must not refire immediately...
        let o = sm.tick(DT, 3.0, true, claim.velocity);
        assert!(!o.handoff);
        assert_eq!(sm.phase(), StartupPhase::Hold);
        // ...but does after it expires.
        run(&mut sm, CONFIRM_RETRY_S + 0.01, 3.0);
        let o = sm.tick(DT, 3.0, true, claim.velocity);
        assert!(o.handoff, "cooldown expiry must allow a retry");
        assert_eq!(sm.phase(), StartupPhase::Confirm);
    }

    /// Regression for the drive-current phantom: Confirm is entered with
    /// commanded drive current still flowing, and the old over-cap settle
    /// skip let ONE sample of that residual "complete" the probe window --
    /// `short_current_estimate` then fabricated a fast rotor ~pi off the
    /// real one and seeded the observer from it. The settle must run in
    /// full no matter how large the entry current is.
    #[test]
    fn confirm_settle_survives_overcap_drive_current() {
        let (r, l, lambda) = (0.1, 150e-6, 1.145e-3);
        let claim = PhaseOutput {
            angle: 1.0,
            velocity: DEFAULT_HANDOFF_VEL,
        };
        let mut sm = ramp_to_hold();
        sm.tick(DT, 3.0, true, claim.velocity);
        assert_eq!(sm.phase(), StartupPhase::Confirm);
        // Residual drive current far above DEADSHORT_MAX_CURRENT_A for the
        // whole settle window: every cycle must still be Probing.
        for _ in 0..CONFIRM_SETTLE_CYCLES {
            assert_eq!(
                sm.feed_confirm(10.0, 3.0, DT, r, l, lambda, claim),
                ConfirmResult::Probing,
                "over-cap drive current must not skip the confirm settle"
            );
        }
        // By the time the window samples, the residual has decayed; a
        // standing rotor honestly reads ~0 -> the claim is NOT confirmed.
        let mut res = ConfirmResult::Probing;
        for _ in 0..DEADSHORT_CYCLES {
            assert_eq!(res, ConfirmResult::Probing);
            res = sm.feed_confirm(0.0, 0.0, DT, r, l, lambda, claim);
        }
        assert!(matches!(res, ConfirmResult::Unconfirmed { .. }));
        assert_eq!(sm.phase(), StartupPhase::Hold);
    }

    /// Same class on the Hold give-up recycle: the ramp/hold drive current
    /// decays into the recycled deadshort, and the old skip turned it into
    /// a `capped` probe with the catch floor waived -- a false Caught on a
    /// slow rotor. The recycled probe must run its full settle too.
    #[test]
    fn recycled_deadshort_settle_survives_drive_current() {
        let mut sm = ramp_to_hold();
        run(&mut sm, HOLD_GIVEUP_S + 0.01, 3.0);
        assert_eq!(sm.phase(), StartupPhase::Deadshort);
        assert!(sm.take_recycled());
        for _ in 0..DEADSHORT_SETTLE_CYCLES {
            assert_eq!(
                sm.feed_deadshort(10.0, 3.0, DT, 0.1, 150e-6, 1.145e-3),
                DeadshortResult::Probing,
                "recycle entry current must not skip the deadshort settle"
            );
            assert_eq!(sm.phase(), StartupPhase::Deadshort);
        }
        // Decayed residual + standing rotor: falls through to the ramp.
        for _ in 0..DEADSHORT_CYCLES {
            sm.feed_deadshort(0.0, 0.0, DT, 0.1, 150e-6, 1.145e-3);
        }
        assert_eq!(sm.phase(), StartupPhase::Ramp);
    }

    /// The COLD-start skip stays: with the bridge previously off, an
    /// over-cap current can only be back-EMF, and sitting shorted on a
    /// large current through a pointless settle would be the worse failure.
    #[test]
    fn cold_start_deadshort_keeps_the_overcap_settle_skip() {
        let mut sm = SensorlessStartup::default();
        sm.begin_cold_start(0.0, 1.0);
        let mut calls: u32 = 0;
        loop {
            let res = sm.feed_deadshort(20.0, 0.0, DT, 0.1, 150e-6, 1.145e-3);
            calls += 1;
            if res != DeadshortResult::Probing || sm.phase() != StartupPhase::Deadshort {
                break;
            }
            assert!(
                calls <= u32::from(DEADSHORT_CYCLES) + 1,
                "cold-start settle skip regressed"
            );
        }
    }

    #[test]
    fn confirm_passes_real_rotor_and_hands_off() {
        use crate::foc::angle_difference;
        // Rotor speed relative to the handoff gate (observer_vel must be
        // ≥ 0.5 × handoff_vel for the hold to fire a probe at all).
        let (r, l, lambda, omega, theta) = (0.1, 150e-6, 1.145e-3, DEFAULT_HANDOFF_VEL * 1.2, 0.9);
        let claim = PhaseOutput {
            angle: theta,
            velocity: omega,
        };
        let mut sm = ramp_to_hold();
        sm.tick(DT, 3.0, true, omega);
        let i = synth_short_current(omega, theta, r, l, lambda, 1.0);
        match run_confirm(&mut sm, i, r, l, lambda, claim) {
            ConfirmResult::Confirmed { velocity } => {
                assert!(
                    (velocity - omega).abs() < 0.3 * omega,
                    "probe velocity {velocity} vs rotor {omega}"
                );
            }
            other => panic!("expected Confirmed, got {other:?}"),
        }
        assert!(!sm.is_active(), "confirmed probe completes the handoff");
        assert!(!sm.wants_short());
        // Sanity on the synth: the probe's angle estimate matches the claim.
        let _ = angle_difference;
    }

    #[test]
    fn confirm_rejects_angle_disagreement() {
        // Back-EMF present at the claimed SPEED but ~π away in angle (e.g.
        // a rotor freewheeling against the commanded direction).
        let (r, l, lambda, omega) = (0.1, 150e-6, 1.145e-3, DEFAULT_HANDOFF_VEL * 1.2);
        let claim = PhaseOutput {
            angle: 0.9 + core::f32::consts::PI,
            velocity: omega,
        };
        let mut sm = ramp_to_hold();
        sm.tick(DT, 3.0, true, omega);
        let i = synth_short_current(omega, 0.9, r, l, lambda, 1.0);
        let res = run_confirm(&mut sm, i, r, l, lambda, claim);
        assert!(
            matches!(res, ConfirmResult::Unconfirmed { .. }),
            "π-off angle must not confirm, got {res:?}"
        );
        assert_eq!(sm.phase(), StartupPhase::Hold);
    }

    #[test]
    fn confirm_seeds_observer_from_probe_after_ratchet() {
        // Bench hold-ratchet (2026-07-06, prof-hold-t3): the observer runs
        // away (+54 rad/s per retry) while the probe keeps measuring the
        // genuinely captured rotor — confirmation against the runaway claim
        // is structurally unreachable. The CONFIRM_SEED_PROBES-th
        // consecutive strong probe must seed the observer from the
        // measurement and hand off instead of waiting out the give-up.
        use crate::foc::angle_difference;
        let (r, l, lambda) = (0.1, 150e-6, 1.145e-3);
        let (omega_r, theta) = (DEFAULT_HANDOFF_VEL, 0.9); // the real rotor
        // Runaway observer: far off in speed AND angle.
        let claim = PhaseOutput {
            angle: theta + 2.0,
            velocity: omega_r * 2.5,
        };
        let mut sm = ramp_to_hold();
        let i = synth_short_current(omega_r, theta, r, l, lambda, 1.0);
        for attempt in 0..CONFIRM_SEED_PROBES - 1 {
            sm.tick(DT, 3.0, true, claim.velocity);
            assert_eq!(sm.phase(), StartupPhase::Confirm, "attempt {attempt}");
            let res = run_confirm(&mut sm, i, r, l, lambda, claim);
            assert!(
                matches!(res, ConfirmResult::Unconfirmed { .. }),
                "attempt {attempt}: {res:?}"
            );
            assert_eq!(sm.phase(), StartupPhase::Hold);
            run(&mut sm, CONFIRM_RETRY_S + 0.01, 3.0);
        }
        sm.tick(DT, 3.0, true, claim.velocity);
        assert_eq!(sm.phase(), StartupPhase::Confirm);
        match run_confirm(&mut sm, i, r, l, lambda, claim) {
            ConfirmResult::SeedAndHandoff { angle, velocity } => {
                assert!(
                    (velocity - omega_r).abs() < 0.3 * omega_r,
                    "seed velocity {velocity} vs rotor {omega_r}"
                );
                assert!(
                    angle_difference(angle, theta).abs() < 0.2,
                    "seed angle {angle} vs rotor {theta}"
                );
            }
            other => panic!("expected SeedAndHandoff on the 3rd strong probe, got {other:?}"),
        }
        assert!(!sm.is_active(), "the seed completes the handoff");
        assert!(!sm.wants_short());
    }

    #[test]
    fn confirm_rejects_claim_lagging_a_runaway_rotor_and_seeds_immediately() {
        // Bench 2026-07-07 (gate-fix-5 and every spin-punch of the day):
        // the unloaded rotor slip-ratchets ahead of the I/f ramp; at the
        // gates the probe honestly measures ~550 el rad/s while the
        // observer still claims ~195 — the same angle neighborhood, just
        // 2.8× slow. The original ONE-SIDED velocity check confirmed this
        // (probe ≥ 0.5×claim holds trivially) and handed the tracker a
        // 2.8×-wrong frequency. Two-sided + fast-seed: the FIRST such
        // probe must seed the observer from the measurement — retrying
        // probes an ever-faster rotor whose short current walks into the
        // OC trip (bench confirm2-2; see CONFIRM_FAST_SEED_VEL).
        use crate::foc::angle_difference;
        let (r, l, lambda) = (0.1, 150e-6, 1.145e-3);
        let (omega_r, theta) = (DEFAULT_HANDOFF_VEL * 2.8, 0.9); // real rotor, fast
        let claim = PhaseOutput {
            angle: theta, // angle agrees — velocity alone must reject
            velocity: DEFAULT_HANDOFF_VEL,
        };
        let mut sm = ramp_to_hold();
        let i = synth_short_current(omega_r, theta, r, l, lambda, 1.0);
        sm.tick(DT, 3.0, true, claim.velocity);
        assert_eq!(sm.phase(), StartupPhase::Confirm);
        match run_confirm(&mut sm, i, r, l, lambda, claim) {
            ConfirmResult::SeedAndHandoff { angle, velocity } => {
                assert!(
                    (velocity - omega_r).abs() < 0.3 * omega_r,
                    "seed velocity {velocity} vs rotor {omega_r}"
                );
                assert!(
                    angle_difference(angle, theta).abs() < 0.2,
                    "seed angle {angle} vs rotor {theta}"
                );
            }
            other => panic!("expected immediate SeedAndHandoff from the fast rotor, got {other:?}"),
        }
        assert!(!sm.is_active());
    }

    #[test]
    fn phantom_probes_never_seed() {
        // A standing rotor measures ~0 on every probe — the strong-probe
        // streak must never accumulate, whatever the (phantom) observer
        // claims; the hold breaks up via the give-up recycle instead.
        let (r, l, lambda) = (0.1, 150e-6, 1.145e-3);
        let claim = PhaseOutput {
            angle: 1.0,
            velocity: DEFAULT_HANDOFF_VEL,
        };
        let mut sm = ramp_to_hold();
        for attempt in 0..=CONFIRM_SEED_PROBES {
            sm.tick(DT, 3.0, true, claim.velocity);
            assert_eq!(sm.phase(), StartupPhase::Confirm, "attempt {attempt}");
            let res = run_confirm(&mut sm, (0.0, 0.0), r, l, lambda, claim);
            assert!(
                matches!(res, ConfirmResult::Unconfirmed { .. }),
                "attempt {attempt} must stay unconfirmed, got {res:?}"
            );
            run(&mut sm, CONFIRM_RETRY_S + 0.01, 3.0);
        }
        assert!(sm.is_active(), "a standing rotor must never seed a handoff");
    }

    #[test]
    fn hold_gives_up_and_recycles_through_deadshort() {
        let mut sm = ramp_to_hold();
        assert!(!sm.take_recycled());
        // Observer never ready: the hold cannot confirm and must recycle.
        run(&mut sm, HOLD_GIVEUP_S + 0.05, 3.0);
        assert_eq!(sm.phase(), StartupPhase::Deadshort);
        assert!(sm.wants_short());
        assert!(sm.take_recycled(), "recycle must be reported (once)");
        assert!(!sm.take_recycled(), "one-shot");
        // And the recycled start runs the normal deadshort → ramp path.
        skip_deadshort(&mut sm);
        assert_eq!(sm.phase(), StartupPhase::Ramp);
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
