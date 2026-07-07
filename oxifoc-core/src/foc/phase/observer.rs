//! Sensorless observers for FOC control
//!
//! Two estimators with complementary speed ranges, designed to run
//! **concurrently** in `PhaseManager`'s two slots (see `manager.rs` for
//! the crossover/blend policies that pick between them):
//!
//! - [`BackEmfObserver`] — flux integrator + PLL, valid once back-EMF is
//!   measurable (≳ [`READY_MIN_VELOCITY`]). Lineage: MXLEMMING algorithm
//!   (David Molony, MESC), with MESC/VESC extensions (λ tracking,
//!   one-sided nonlinear centering) and Boldea's "active flux" form for
//!   salient motors.
//! - [`HfiObserver`] — pulsating d-axis injection + synchronous
//!   demodulation, valid from standstill on salient motors (Ld ≠ Lq).
//!   Carries a π ambiguity until the saturation probe (or a sensor seed)
//!   resolves it.
//!
//! Conventions: angles are ELECTRICAL radians, inputs are stationary-frame
//! (αβ) volts/amps, `dt` per call (no fixed-rate assumption). Every design
//! decision here is pinned by the closed-loop sims in this file and
//! `manager.rs` (VirtualMotor plant), plus on-target parity tests in
//! `tests/stm32g431`; numbers quoted in comments come from
//! docs/perf-bench-2026-06-11.md.

// TAU (carrier advance), PhantomData (`HfiObserver<S>` marker) and the SinCos
// backend are HFI-only; BackEmf's `force_phase` imports SinCos locally.
#[cfg(feature = "hfi")]
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::foc::wrap_angle;
#[cfg(feature = "hfi")]
use core::f32::consts::TAU;
#[cfg(feature = "hfi")]
use core::marker::PhantomData;

/// Input for observer update
#[derive(Clone, Copy, Debug, Default)]
pub struct ObserverInput {
    /// α-axis voltage (V)
    pub v_alpha: f32,
    /// β-axis voltage (V)
    pub v_beta: f32,
    /// α-axis current (A)
    pub i_alpha: f32,
    /// β-axis current (A)
    pub i_beta: f32,
    /// Time step (seconds)
    pub dt: f32,
}

/// Runtime-switchable observer implementations
///
/// This is the back-EMF/"fast" estimator slot of `PhaseManager`. The HFI
/// estimator lives in its own dedicated slot ([`HfiObserver`] directly):
/// it needs carrier injection plumbed through the control loop, and the
/// HfiToObserver crossover requires both estimators to run concurrently.
#[derive(Clone, Debug)]
pub enum Observer {
    /// No observer configured
    None,
    /// Back-EMF flux observer (VESC-style)
    BackEmf(BackEmfObserver),
}

#[allow(clippy::derivable_impls)] // Other variants have data, can't derive
impl Default for Observer {
    fn default() -> Self {
        Self::None
    }
}

impl Observer {
    /// Update observer with new measurements
    pub fn update(&mut self, input: &ObserverInput) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => o.update(input),
        }
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.phase()),
        }
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.velocity()),
        }
    }

    /// Raw flux-vector angle from the last update (pre-PLL) — see
    /// [`BackEmfObserver::phase_raw`].
    pub fn phase_raw(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.phase_raw()),
        }
    }

    /// Assert/clear the slip gate — see [`BackEmfObserver::set_slip_gate`].
    pub fn set_slip_gate(&mut self, gated: bool) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => o.set_slip_gate(gated),
        }
    }

    /// Configure the physics acceleration prior — see
    /// [`BackEmfObserver::set_accel_prior`].
    pub fn set_accel_prior(&mut self, floor_el: f32, per_amp_el: f32) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => o.set_accel_prior(floor_el, per_amp_el),
        }
    }

    /// Feed the measured |iq| for the acceleration prior — see
    /// [`BackEmfObserver::note_torque_current`].
    pub fn note_torque_current(&mut self, iq_abs: f32, dt: f32) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => o.note_torque_current(iq_abs, dt),
        }
    }

    /// Whether the observer's estimate can be trusted for commutation.
    ///
    /// Unlike [`phase`](Self::phase), which returns a value for any
    /// *configured* observer (including one frozen at 0 with zero
    /// confidence), this checks actual convergence. All fallback and
    /// crossover decisions must gate on this.
    pub fn is_ready(&self) -> bool {
        match self {
            Self::None => false,
            Self::BackEmf(o) => o.is_ready(),
        }
    }

    /// Seed the estimate from a trusted external source (sensor handoff).
    pub fn seed(&mut self, angle: f32, velocity: f32) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => {
                o.force_phase(angle);
                o.set_velocity(velocity);
            }
        }
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        match self {
            Self::None => 0.0,
            Self::BackEmf(o) => o.confidence(),
        }
    }

    /// External-validity diagnostics of the underlying estimator, if any —
    /// see [`BackEmfObserver::validity`].
    pub fn validity(&self) -> Option<(f32, f32)> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.validity()),
        }
    }

    /// Check if observer is configured
    pub fn is_configured(&self) -> bool {
        !matches!(self, Self::None)
    }

    /// Phase resistance of the underlying motor model, if any — used by
    /// voltage-based crossover criteria (|vq − R·iq| back-EMF proxy).
    pub fn resistance(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.resistance()),
        }
    }

    /// d/q-average inductance of the model, if any (deadshort flying restart:
    /// `e = −L·dI/dt`).
    pub fn inductance(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.inductance()),
        }
    }

    /// Flux linkage λ of the model, if any (deadshort speed estimate `|ω| =
    /// |e|/λ`).
    pub fn lambda(&self) -> Option<f32> {
        match self {
            Self::None => None,
            Self::BackEmf(o) => Some(o.lambda()),
        }
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        match self {
            Self::None => {}
            Self::BackEmf(o) => o.reset(),
        }
    }
}

// ============================================================================
// Back-EMF Observer (VESC-style flux observer)
// ============================================================================

/// Back-EMF flux observer for sensorless FOC
///
/// MXLEMMING-style flux observer (original algorithm by David Molony, MESC
/// project; also available in VESC as `FOC_OBSERVER_MXLEMMING`,
/// foc_math.c). Integrates `(v − R·i)·dt − L·Δi` to track the rotor flux
/// vector directly, drains integrator drift back onto the λ circle
/// (one-sided radial centering + component clamp backstop), then uses a
/// PLL to extract phase and velocity. Optional extensions: online λ
/// tracking ([`with_lambda_tracking`](Self::with_lambda_tracking)) and the
/// salient "active flux" form for IPM motors
/// ([`with_saliency`](Self::with_saliency)).
///
/// Works well at medium to high speeds where back-EMF is measurable.
/// At low speeds, HFI should be used instead.
#[derive(Clone, Debug)]
pub struct BackEmfObserver {
    // Flux integrator state
    x1: f32, // α-axis rotor flux estimate (Wb)
    x2: f32, // β-axis rotor flux estimate (Wb)

    // Previous currents for the incremental −L·Δi stator-flux removal
    i_alpha_last: f32,
    i_beta_last: f32,

    // PLL state
    phase_pll: f32,    // PLL-filtered phase
    velocity_pll: f32, // PLL-filtered velocity

    // Motor parameters
    r: f32, // Phase resistance (Ω)
    /// Inductance subtracted from the flux integral (H): Lq in the salient
    /// "active flux" configuration (`with_saliency`), the plain phase
    /// inductance otherwise.
    l: f32,
    /// Lq − Ld (H), informational (active-flux magnitude shift under
    /// d-current). 0 = round-rotor.
    l_delta: f32,
    /// Eddy-ladder ΔL (H): the L(f) drop between the HF plateau (`l`) and
    /// the low-frequency inductance. 0 = single-L model (default).
    eddy_delta_l: f32,
    /// Eddy-ladder time constant (s); `L(jω) = l + ΔL/(1 + jωτ)`.
    eddy_tau_s: f32,
    /// Eddy-branch filtered currents (αβ).
    i_f_alpha: f32,
    i_f_beta: f32,
    /// Slip gate (see [`Self::set_slip_gate`]): while set, the PLL holds —
    /// no velocity integration, no phase correction (dead reckoning), no
    /// error/validity/λ filter updates. The flux integrator keeps running.
    slip_gate: bool,
    /// Continuous gated time (s) — the duty limiter against a latched
    /// gate (e.g. permanently unreachable iq at voltage saturation).
    slip_gate_time: f32,
    /// Physics acceleration prior (el rad/s² per A of |iq|) and floor —
    /// see [`Self::set_accel_prior`]. 0 = clamp off.
    accel_per_amp: f32,
    accel_floor: f32,
    /// Low-passed |iq| feeding the prior (τ ≈ 10 ms).
    iq_abs_filt: f32,
    /// Velocity-magnitude envelope for the prior: slews at the allowed
    /// rate in both directions; |ω̂| is clamped to it. Ringing below the
    /// envelope is untouched (a per-step Δv clamp asymmetrically clips
    /// PLL ringing and biases tracking down).
    vel_cap: f32,
    /// Low-passed |ω̂| (τ = 50 ms) the envelope acts on — see the
    /// filtered-trend note in the envelope block of `update`.
    vel_mag_filt: f32,
    lambda: f32, // Flux linkage (Wb); adapted online when lambda_gain > 0

    // Online λ adaptation (MESC/VESC lambda-comp): first-order tracker of
    // the raw flux magnitude. 0 = off. Bounds keep a transient from
    // dragging λ to nonsense.
    lambda_gain: f32, // 1/s
    lambda_min: f32,
    lambda_max: f32,

    /// Nonlinear centering gain (1/s, normalized): radial pull of the flux
    /// vector toward the λ circle. The component-wise hard clamp alone
    /// distorts the trajectory near the ±λ square's corners (angle bias);
    /// the radial pull bleeds integrator drift without bending the angle.
    /// The clamp stays as a backstop. 0 = clamp only.
    centering_gain: f32,

    // Observer tuning
    pll_kp: f32, // PLL proportional gain
    pll_ki: f32, // PLL integral gain

    // State
    confidence: f32,     // Confidence estimate (0-1)
    phase_err_filt: f32, // Low-passed |PLL phase error| (rad), for readiness
    /// Last raw flux-vector angle (atan2 of the integrator, pre-PLL) —
    /// debug telemetry (`obs-debug-telem`) and divergence forensics.
    phase_raw_last: f32,

    // External-validity state (see `is_ready`): low-passed back-EMF proxy
    // along the estimated q axis (V, signed by the rotation direction), the
    // phase travel (rad) accumulated while that proxy corroborated the
    // claimed velocity, and how long (s) the corroboration has been
    // continuously violated while validity was granted.
    bemf_q_filt: f32,
    valid_travel: f32,
    invalid_time: f32,
    /// Phase travel (rad) of EARNED corroboration since the last
    /// reset/seed — unlike `valid_travel` this is never credited by
    /// `force_phase`. λ adaptation gates on it: a seed grants validity so
    /// the torque path engages immediately, but the flux integrator was
    /// just fabricated and during restart churn (seed → trust loss → seed)
    /// the raw magnitude is inverter distortion — the tracker legally
    /// walked λ to its λ₀/2 clamp on it (bench 2026-07-06 late). Requiring
    /// the corroboration to be re-earned after every seed starves the
    /// tracker in churn while costing a real cruise ~2 revolutions.
    lambda_learn_travel: f32,
}

/// Minimum confidence (flux magnitude / λ) for [`BackEmfObserver::is_ready`].
pub const READY_MIN_CONFIDENCE: f32 = 0.5;

/// Maximum filtered PLL phase error (rad) for "locked" in
/// [`BackEmfObserver::is_ready`]. ~11°: a converged PLL tracks well under
/// this; a diverging one sits near π.
pub const READY_MAX_PHASE_ERR_RAD: f32 = 0.2;

/// Minimum |electrical velocity| (rad/s) for [`BackEmfObserver::is_ready`].
///
/// Below this the back-EMF is too small to observe — flux magnitude and a
/// locked PLL can both look fine at standstill on pure integrator memory.
/// ~286 eRPM; well under typical sensor→observer crossover bands.
pub const READY_MIN_VELOCITY: f32 = 30.0;

/// Time constant (s) of the PLL phase-error low-pass used for readiness.
/// Slow enough to ride out per-revolution ripple at crossover speeds.
const PHASE_ERR_FILTER_TAU_S: f32 = 0.01;

/// External validity: how many electrical revolutions the back-EMF proxy
/// must corroborate the claimed velocity before [`BackEmfObserver::is_ready`]
/// reports true (see `is_ready` for the physics). 2 revolutions ≈ 210 ms at
/// the 60 rad/s handoff speed, 31 ms for a 400 rad/s runaway catch.
pub const READY_MIN_VALID_REVS: f32 = 2.0;

/// External validity: acceptance window for `e_q / (λ·ω̂)`. Real rotation
/// sits near 1 (the L·di/dt rotation term pushes it above; PLL lag and
/// flux-angle error pull it below); a phantom lock driven by the machine's
/// own stator flux sits at ≤ L·i/λ (≈ 0.03–0.14 on the ZD2808 at bench
/// currents) and a direction mismatch goes negative. The floor is the
/// discriminating edge; the ceiling only rejects gross nonsense.
/// 0.4 → 0.25 (2026-07-08): the 2 kHz spectra of the mid-band ride
/// (captures/trk-damp-2k-1) exposed the "limit cycle" as a PROTECTION
/// relaxation oscillator — the 30–90 Hz estimate wobble transiently dips
/// the ratio below the floor, validity revokes after VALID_REVOKE_S, the
/// iq gate chops torque, the rotor coasts, corroboration returns,
/// re-grant, torque transient re-excites the wobble: a 2–6 Hz envelope
/// (±90 rad/s ω̂ swings, the visible jerk) with iq beats at 5–25 Hz.
/// The phantom it discriminates against measures ≤ 0.14 — the floor has
/// margin to sit below the wobble dips and above the phantom.
const VALID_BEMF_RATIO_MIN: f32 = 0.25;
const VALID_BEMF_RATIO_MAX: f32 = 2.5;

/// Time constant (s) of the back-EMF proxy low-pass. A few electrical
/// ripple periods at handoff speeds: long enough to average the 6th-harmonic
/// dead-time residue out of the rotating-frame projection, short enough to
/// track a spin-up.
/// 10 → 25 ms (2026-07-08): average the mid-band wobble (30–90 Hz) out
/// of the corroboration ratio itself — see VALID_BEMF_RATIO_MIN.
const BEMF_PROXY_TAU_S: f32 = 0.025;

/// External validity: once granted, how long (s) the corroboration must be
/// violated CONTINUOUSLY before validity is revoked. Grant and revoke are
/// deliberately asymmetric: granting demands [`READY_MIN_VALID_REVS`]
/// coherent revolutions, but revoking on the first bad sample turned the
/// readiness gate into a torque chopper — every revocation zeroes iq, the
/// step transient perturbs the estimate further, and the closed-loop sim
/// went from a clean spin-up into a gate-flicker limit cycle. A real trust
/// loss (the bench deadlock: proxy ≈ 0.003·λω̂ for 15 s) violates without
/// interruption, so a couple hundred ms of debounce distinguishes it from
/// an acceleration transient at no cost to the failure case.
/// 0.2 → 0.4 s (2026-07-08): the revoke debounce is half of the
/// protection-oscillator period (see VALID_BEMF_RATIO_MIN) — the real
/// trust-loss cases it exists for (deadlocked phantom, proxy ≈ 0.003·λω
/// for 15 s) violate for seconds, so doubling the debounce costs the
/// failure case nothing and pushes the relaxation cycle below the
/// mid-band ride's violation windows.
const VALID_REVOKE_S: f32 = 0.4;

/// Default nonlinear centering gain (1/s, normalized error): drains a
/// radius error with τ ≈ 2 ms — orders of magnitude faster than
/// measurement-offset drift accumulates, while staying gentle on spin-up
/// transients (an aggressive pull there measurably perturbs the
/// hall→observer handoff timing in the closed-loop sims; 5000 broke
/// their continuity assertions, 500 does not).
pub const DEFAULT_CENTERING_GAIN: f32 = 500.0;

/// Acceleration-envelope governor gain (1/s): how hard the PLL velocity
/// is pulled back per rad/s of filtered-trend excess over the envelope.
/// Sized so a phantom's maximum rebuild rate (ki·err ≲ 60 k rad/s² at
/// err ≈ π) balances at a few hundred rad/s of excess, while normal
/// tracking (excess ≈ 0) never feels it.
const ACCEL_GOVERNOR_GAIN: f32 = 300.0;

/// Default λ-tracking gain (1/s): τ = 0.2 s. λ moves with saturation and
/// magnet temperature on the timescale of seconds; tracking must stay far
/// slower than the flux/PLL dynamics or it eats the error signal itself.
pub const DEFAULT_LAMBDA_GAIN: f32 = 5.0;

impl BackEmfObserver {
    /// Create a new back-EMF observer with motor parameters.
    ///
    /// Round-rotor model with nonlinear centering on; add
    /// [`with_saliency`](Self::with_saliency) /
    /// [`with_lambda_tracking`](Self::with_lambda_tracking) for IPM motors
    /// and online λ adaptation.
    ///
    /// # Arguments
    /// * `r` - Phase resistance (Ω)
    /// * `l` - Phase inductance (H)
    /// * `lambda` - Flux linkage (Wb)
    pub fn new(r: f32, l: f32, lambda: f32) -> Self {
        let lambda = lambda.max(1e-6);
        Self {
            x1: 0.0,
            x2: 0.0,
            i_alpha_last: 0.0,
            i_beta_last: 0.0,
            phase_pll: 0.0,
            velocity_pll: 0.0,
            r,
            l,
            l_delta: 0.0,
            eddy_delta_l: 0.0,
            eddy_tau_s: 0.0,
            i_f_alpha: 0.0,
            i_f_beta: 0.0,
            slip_gate: false,
            slip_gate_time: 0.0,
            accel_per_amp: 0.0,
            accel_floor: 0.0,
            iq_abs_filt: 0.0,
            vel_cap: 0.0,
            vel_mag_filt: 0.0,
            lambda,
            lambda_gain: 0.0,
            // Bounds are inert until with_lambda_tracking() rebinds them.
            lambda_min: lambda,
            lambda_max: lambda,
            centering_gain: DEFAULT_CENTERING_GAIN,
            // PLL: ωn = √ki ≈ 140 rad/s, ζ = kp/(2·ωn) ≈ 3.5 — heavily
            // overdamped tracker, sim-validated to lock through spin-up
            // without overshooting into the π-flip guard's >90° zone.
            pll_kp: 1000.0,
            pll_ki: 20000.0,
            confidence: 0.0,
            // Start "unlocked": a fresh observer must not look ready until
            // the PLL has actually tracked something.
            phase_err_filt: core::f32::consts::PI,
            bemf_q_filt: 0.0,
            phase_raw_last: 0.0,
            valid_travel: 0.0,
            invalid_time: 0.0,
            lambda_learn_travel: 0.0,
        }
    }

    /// Use the salient (IPM) "active flux" model (Boldea): the integrator
    /// subtracts Lq·i (not L_avg·i), which leaves
    /// (λ + (Ld−Lq)·id)·e^{jθ} — a vector exactly aligned with the d axis
    /// for any load. A scalar L_avg on an IPM motor gives a load-dependent
    /// angle bias instead (0.56 rad at Lq·I ≈ λ in the sims).
    ///
    /// Two rejected alternatives, for the record (both sim-tested worse
    /// than even the scalar model under pure-q load): MESC's diagonal
    /// Lα(θ)/Lβ(θ) approximation drops the off-diagonal coupling term,
    /// and an "exact" invPark(Ld·id, Lq·iq) subtraction at the estimated
    /// angle feeds the angle error back into itself (loop gain ≈ ΔL·I/λ).
    /// The active-flux form has no angle-dependent terms, so no feedback
    /// path exists. Sim result: < 0.05 rad under the same load.
    pub fn with_saliency(mut self, ld: f32, lq: f32) -> Self {
        if ld > 0.0 && lq > 0.0 {
            self.l = lq;
            self.l_delta = lq - ld;
        }
        self
    }

    /// Configure the eddy-current L(f) ladder for the stator-flux
    /// subtraction: `ψ_stator = l·i + ΔL·i_f` with `τ·di_f/dt = i − i_f`,
    /// i.e. `L(jω) = l + ΔL/(1 + jωτ)` — `l` stays the HF (AC) value the
    /// estimation chain is validated on, `ΔL` adds the low-frequency rise
    /// (ZD2808: l = 24 µH AC, DC Lq 129 µH ⇒ ΔL ≈ 105 µH, τ ≈ 0.3 ms).
    ///
    /// Why (bench 2026-07-06 night, captures/sawtooth-obsdbg-1): a
    /// pole-slip transient is a 100–300 Hz event where the true stator
    /// flux follows L(f) ≈ 40–80 µH; subtracting only the 24 µH plateau
    /// under-removes stator flux and every slip KICKS the flux vector
    /// forward ~0.3 rad — the PLL integrates the kicks into a runaway
    /// (slip-kick ratchet, ~2 Hz estimate sawtooth, dq OC). Steady
    /// tracking has Δi ≈ 0, which is why the single-L subtraction looked
    /// fine in every constant-speed test.
    pub fn with_eddy_ladder(mut self, delta_l: f32, tau_s: f32) -> Self {
        self.set_eddy_ladder(delta_l, tau_s);
        self
    }

    /// Runtime setter for [`with_eddy_ladder`](Self::with_eddy_ladder).
    pub fn set_eddy_ladder(&mut self, delta_l: f32, tau_s: f32) {
        if delta_l >= 0.0 && tau_s >= 0.0 && delta_l.is_finite() && tau_s.is_finite() {
            self.eddy_delta_l = delta_l;
            self.eddy_tau_s = tau_s;
        }
    }

    /// Enable online λ adaptation: λ tracks the raw flux magnitude with a
    /// first-order filter (`gain` in 1/s), clamped to [λ₀/2, 2λ₀]. Makes
    /// the configured flux linkage non-critical (it drifts with saturation
    /// and magnet temperature) and un-breaks `confidence = |flux|/λ` when
    /// the stored value is off.
    ///
    /// The factor-2 bounds are deliberately loose: physical λ drift is
    /// tens of percent at most, so hitting a bound means the stored value
    /// is wrong outright (re-run detection) — and the bound is what keeps
    /// a fault transient from dragging λ to nonsense meanwhile.
    pub fn with_lambda_tracking(mut self, gain: f32) -> Self {
        if gain > 0.0 {
            self.lambda_gain = gain;
            self.lambda_min = self.lambda * 0.5;
            self.lambda_max = self.lambda * 2.0;
        }
        self
    }

    /// Override the nonlinear centering gain (1/s; 0 = hard clamp only).
    pub fn with_centering_gain(mut self, gain: f32) -> Self {
        self.centering_gain = gain.max(0.0);
        self
    }

    /// Phase resistance (Ω) — used by voltage-based crossover criteria.
    pub fn resistance(&self) -> f32 {
        self.r
    }

    /// d/q-average inductance (H) of the model — deadshort flying restart.
    pub fn inductance(&self) -> f32 {
        self.l
    }

    /// Current (possibly adapted) flux-linkage estimate (Wb).
    pub fn lambda(&self) -> f32 {
        self.lambda
    }

    /// Update observer with new measurements
    pub fn update(&mut self, input: &ObserverInput) {
        let dt = input.dt;
        if dt <= 0.0 {
            return;
        }

        // MXLEMMING flux integrator: x += (v − R·i)·dt − L·Δi
        //
        // ∫(v − R·i) is the *total* stator flux; the rotor flux is that minus
        // L·i. Removing L·i incrementally keeps x tracking the rotor flux
        // directly, so the estimated angle stays unbiased under load (without
        // this term a q-axis current of L·I = λ skews the angle by 45°).
        // For salient (IPM) motors `l` is Lq — the "active flux" form
        // (Boldea): subtracting Lq·i in the stationary frame leaves
        // (λ + (Ld−Lq)·id)·e^{jθ}, a vector aligned with the d axis for ANY
        // load split, with no angle-dependent terms (a scalar subtraction
        // can't feed the estimate back into itself). At id = 0 its
        // magnitude is exactly λ; under field weakening (id < 0, Ld < Lq)
        // it grows, which the λ tracker absorbs.
        // Eddy-ladder share of the stator-flux increment (see
        // `with_eddy_ladder`): dψ_eddy = ΔL·Δi_f, i_f = LPF(i, τ). Zero
        // when the ladder is off — the classic single-L subtraction.
        let (dpsi_e_alpha, dpsi_e_beta) = if self.eddy_delta_l > 0.0 && self.eddy_tau_s > 0.0 {
            let k = (dt / self.eddy_tau_s).min(1.0);
            let df_alpha = k * (input.i_alpha - self.i_f_alpha);
            let df_beta = k * (input.i_beta - self.i_f_beta);
            self.i_f_alpha += df_alpha;
            self.i_f_beta += df_beta;
            (self.eddy_delta_l * df_alpha, self.eddy_delta_l * df_beta)
        } else {
            (0.0, 0.0)
        };
        self.x1 += (input.v_alpha - self.r * input.i_alpha) * dt
            - self.l * (input.i_alpha - self.i_alpha_last)
            - dpsi_e_alpha;
        self.x2 += (input.v_beta - self.r * input.i_beta) * dt
            - self.l * (input.i_beta - self.i_beta_last)
            - dpsi_e_beta;
        self.i_alpha_last = input.i_alpha;
        self.i_beta_last = input.i_beta;

        // Raw (pre-correction) flux magnitude: the honest amplitude signal
        // for λ tracking and confidence, taken before centering/clamping
        // pull it onto the configured circle.
        let flux_mag = crate::foc::fast_math::sqrtf(self.x1 * self.x1 + self.x2 * self.x2);

        // Online λ adaptation (MESC MXLEMMING_LAMBDA / VESC lambda-comp):
        // slow first-order tracker, bounded. Adapts the clamp/centering
        // circle and the confidence normalization with it.
        //
        // Gated on GRANTED external validity (one cycle stale — the accrual
        // happens below): the tracker follows the raw flux magnitude, and
        // during a failed catch / phantom churn that magnitude is inverter
        // distortion, not rotor flux. Bench 2026-07-06 late: startup
        // transients dragged λ to its λ₀/2 clamp, which then INFLATED
        // confidence (flux/λ with λ halved), widened the e_q corroboration
        // onto a runaway observer (the hold-ratchet's enabler), and left
        // the ±λ component clamp clipping the REAL flux vector after the
        // probe seed. Physical λ drift (saturation, magnet temperature) is
        // slow — only learn it while the rotation is externally
        // corroborated; the [0.4, 2.5] corroboration window tolerates a
        // stored λ far more wrong than physical drift can make it.
        // Slip gate bookkeeping first (see set_slip_gate): during a flagged
        // slip transient nothing LEARNS — λ, the PLL and the quality/
        // validity filters all hold; only the flux integrator (real
        // physics) and the dead-reckoned angle advance. Duty-limited so a
        // latched gate cannot freeze the estimator forever.
        if self.slip_gate {
            self.slip_gate_time += dt;
        }
        let gated = self.slip_gate && self.slip_gate_time <= Self::SLIP_GATE_MAX_S;

        let validity_granted =
            self.valid_travel >= READY_MIN_VALID_REVS * core::f32::consts::TAU - 1e-3;
        // λ learning additionally requires EARNED corroboration travel (see
        // the field doc): seeded validity engages torque, not the tracker.
        let lambda_learn_ok =
            self.lambda_learn_travel >= READY_MIN_VALID_REVS * core::f32::consts::TAU - 1e-3;
        if self.lambda_gain > 0.0 && validity_granted && lambda_learn_ok && !gated {
            self.lambda += self.lambda_gain * (flux_mag - self.lambda) * dt;
            self.lambda = crate::foc::clamp_f32(self.lambda, self.lambda_min, self.lambda_max);
        }

        // Nonlinear centering (MESC-inspired, one-sided): radial pull back
        // onto the λ circle when the integral drifts OUTSIDE it. Normalized
        // form, λ-independent gain: x += (1 − |x|²/λ²)·x·k·dt, applied only
        // for |x| > λ. Unlike the component clamp this never bends the flux
        // angle; unlike a two-sided pull it never inflates a small flux to
        // the configured circle — that would fabricate confidence and feed
        // the λ tracker its own output.
        // One reciprocal for the three λ normalizations below (centering,
        // e_q projection, confidence) — λ ≥ 1e-6 by construction.
        let inv_lambda = 1.0 / self.lambda;

        if self.centering_gain > 0.0 && flux_mag > self.lambda {
            let mag_norm = flux_mag * inv_lambda;
            let err_norm = 1.0 - mag_norm * mag_norm;
            // Clamp: never drain more than half the radius in one cycle,
            // whatever gain·dt·overshoot multiplies out to.
            let pull = crate::foc::clamp_f32(err_norm * self.centering_gain * dt, -0.5, 0.0);
            self.x1 += self.x1 * pull;
            self.x2 += self.x2 * pull;
        }

        // Component-wise truncation to ±λ stays as the hard backstop
        // (original MXLEMMING mechanism): the true rotor flux components
        // never exceed λ, whatever the centering gain is doing.
        self.x1 = crate::foc::clamp_f32(self.x1, -self.lambda, self.lambda);
        self.x2 = crate::foc::clamp_f32(self.x2, -self.lambda, self.lambda);

        // Extract phase from the rotor flux vector. The polynomial atan2's
        // ≤0.011 rad error feeds a PLL that low-passes it — negligible next
        // to dead-time distortion, and 3.7× cheaper than libm in the ISR.
        let phase_raw = crate::foc::fast_math::atan2f(self.x2, self.x1);
        self.phase_raw_last = phase_raw;

        if gated {
            self.phase_pll = wrap_angle(self.phase_pll + self.velocity_pll * dt);
            // Confidence stays an honest instantaneous ratio of the (still
            // integrated) flux magnitude.
            self.confidence = crate::foc::clamp_f32(flux_mag / self.lambda, 0.0, 1.0);
            return;
        }

        // PLL tracking. The error must be the SIGNED shortest angular distance
        // (like VESC's foc_pll_run): wrapping to [0, 2π) would make the error
        // always non-negative and the velocity integrator could only grow.
        let phase_error = crate::foc::angle_difference(phase_raw, self.phase_pll);
        self.velocity_pll += self.pll_ki * phase_error * dt;
        // Physics acceleration prior (see set_accel_prior): |ω̂| is clamped
        // to an envelope that slews at the physically-allowed rate. The
        // envelope follows |ω̂| down at the same rate (an upper cap only —
        // load-driven deceleration is never fought), and PLL ringing below
        // it stays untouched (a per-step Δv clamp asymmetrically clips the
        // ringing and biases legitimate tracking down).
        if self.accel_per_amp > 0.0 || self.accel_floor > 0.0 {
            // Envelope clamp with a RECTIFIER FIX (2026-07-08,
            // captures/trk-damp-2k-1 vs noprior-2k-1): the up-branch stays
            // hard on the instantaneous |ω̂| (the slow-phantom wind-up must
            // meet an unfiltered wall), but the down-branch decays the cap
            // toward the FILTERED magnitude (τ = 50 ms), not the
            // instantaneous one. The original `.max(mag)` let every
            // mid-band wobble trough drag the cap down at full slew and
            // the next recovery was clipped — a rectifier that biased ω̂
            // low at 2–6 Hz, lagged the drive behind the rotor and pumped
            // the torque-beat/wobble loop (THE mid-band envelope
            // oscillation: Δσ ±60° with the old prior, ±15–20° and a
            // clean climb with the prior off). Zero-mean wobble barely
            // moves the filtered magnitude, so the cap now rides the
            // trend and recoveries pass; a real deceleration moves the
            // filtered magnitude within ~2τ and the cap follows as
            // before.
            let a_m = (dt / 0.05).min(1.0);
            self.vel_mag_filt += a_m * (self.velocity_pll.abs() - self.vel_mag_filt);
            let step = (self.accel_floor + self.accel_per_amp * self.iq_abs_filt) * dt;
            if self.vel_mag_filt > self.vel_cap {
                self.vel_cap += step;
                if self.vel_mag_filt > self.vel_cap {
                    // Proportional governor on the TREND excess: a strong,
                    // continuous pull of ω̂ toward the envelope. Symmetric
                    // over a wobble period (the excess is a filtered,
                    // slow quantity — no half-wave rectification, which
                    // is what both a hard clip of the instantaneous
                    // magnitude and a trough-chasing decay did, pumping
                    // the mid-band envelope oscillation), and strong
                    // enough that a sustained phantom's PLL rebuild
                    // (ki·err) balances at a bounded excess instead of
                    // running between clamp events (a per-event rescale
                    // leaked ~4× the envelope rate).
                    let excess = self.vel_mag_filt - self.vel_cap;
                    let pull = ACCEL_GOVERNOR_GAIN * excess * dt;
                    if self.velocity_pll > 0.0 {
                        self.velocity_pll -= pull;
                    } else {
                        self.velocity_pll += pull;
                    }
                }
            } else {
                // Decay floor = max(instantaneous, filtered), but the
                // floor may never RAISE the cap (only the up-branch's
                // rate-limited growth may): an unclamped floor followed a
                // fast rise for free and the envelope never engaged. The
                // filtered term keeps wobble TROUGHS from dragging the
                // cap down; the instantaneous term keeps the cap from
                // sagging below a rising magnitude inside the filter lag.
                let floor = self
                    .velocity_pll
                    .abs()
                    .max(self.vel_mag_filt)
                    .min(self.vel_cap);
                self.vel_cap = (self.vel_cap - step).max(floor);
            }
        }
        self.phase_pll =
            wrap_angle(self.phase_pll + (self.velocity_pll + self.pll_kp * phase_error) * dt);

        // Track lock quality for is_ready(): low-passed |phase error|.
        let alpha = (dt / PHASE_ERR_FILTER_TAU_S).min(1.0);
        self.phase_err_filt += alpha * (phase_error.abs() - self.phase_err_filt);

        // External validity: does the terminal voltage actually contain the
        // back-EMF the claimed rotation implies? Project the instantaneous
        // back-EMF estimate e = v − R·i onto the estimated q axis using the
        // flux vector itself (e leads the rotor flux by 90°·sign(ω), so for
        // a REAL rotation e_q ≈ λ·ω, signed). A phantom lock — the observer
        // tracking the machine's own rotating stator flux with the rotor
        // standing — has nothing there but the rotation term ω·L·i
        // (bench + sim: ≤ 0.14·λω at drive currents), and the deadlocked
        // gate case measures vq ≈ 0.01 V outright. The low-pass averages the
        // 6th-harmonic dead-time residue out; the DC part of that residue is
        // already inside the detected R (2-point DC detection).
        let e_alpha = input.v_alpha - self.r * input.i_alpha;
        let e_beta = input.v_beta - self.r * input.i_beta;
        // (x1,x2)/|x| is the unit flux direction; cross product = q-axis
        // projection. Normalized by the MEASURED flux magnitude, not λ:
        // with a mis-stored λ the integrator tracks the true flux while the
        // clamp circle sits elsewhere, and a λ-normalized projection would
        // carry the λ error TWICE (once here, once in bemf_expected) —
        // λ_cfg 1.8× true put the ratio at 0.31, below the 0.4 window, and
        // validity could never be granted (which the λ-adaptation gate
        // below would deadlock on). The 0.1·λ floor only bounds the
        // amplification while the integrator is still building up from a
        // reset — there the direction is noise and the 2-revolution
        // consecutive-corroboration requirement is what protects the grant.
        let e_q = (e_beta * self.x1 - e_alpha * self.x2) / flux_mag.max(0.1 * self.lambda);
        let a_e = (dt / BEMF_PROXY_TAU_S).min(1.0);
        self.bemf_q_filt += a_e * (e_q - self.bemf_q_filt);
        // Corroborated while the signed ratio e_q/(λ·ω̂) sits in the real-
        // rotation window at an observable speed. Grant/revoke asymmetry
        // below: earning validity demands N consecutive coherent
        // revolutions, losing it demands a sustained violation.
        let speed_ok = self.velocity_pll.abs() >= READY_MIN_VELOCITY;
        let bemf_expected = self.lambda * self.velocity_pll;
        let corroborated = speed_ok
            && self.bemf_q_filt * bemf_expected > 0.0
            && (VALID_BEMF_RATIO_MIN * bemf_expected.abs()
                ..=VALID_BEMF_RATIO_MAX * bemf_expected.abs())
                .contains(&self.bemf_q_filt.abs());
        let granted = validity_granted;
        if corroborated {
            self.invalid_time = 0.0;
            // Saturate at the threshold: no unbounded growth.
            self.valid_travel = (self.valid_travel + self.velocity_pll.abs() * dt)
                .min(READY_MIN_VALID_REVS * core::f32::consts::TAU);
            self.lambda_learn_travel = (self.lambda_learn_travel + self.velocity_pll.abs() * dt)
                .min(READY_MIN_VALID_REVS * core::f32::consts::TAU);
        } else if granted {
            // Sticky once granted: revoke only on a SUSTAINED violation
            // (see VALID_REVOKE_S) so accel/decel transients don't chop
            // the torque path.
            self.invalid_time += dt;
            if self.invalid_time >= VALID_REVOKE_S {
                self.valid_travel = 0.0;
                self.lambda_learn_travel = 0.0;
                self.invalid_time = 0.0;
            }
        } else {
            // Still earning: any violation restarts the accrual, so grant
            // means N *consecutive* coherent revolutions.
            self.valid_travel = 0.0;
            self.lambda_learn_travel = 0.0;
            self.invalid_time = 0.0;
        }

        // Confidence: how close the (raw, pre-correction) flux magnitude is
        // to λ. A weak heuristic — measurement offsets can also saturate the
        // integrator — but cheap and monotonic during real spin-up. With λ
        // tracking enabled the normalization adapts along with the clamp.
        self.confidence = crate::foc::clamp_f32(flux_mag * inv_lambda, 0.0, 1.0);
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> f32 {
        self.phase_pll
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> f32 {
        self.velocity_pll
    }

    /// Raw flux-vector angle from the last update (pre-PLL) — the signal
    /// the PLL tracks; `angle_difference(phase_raw, phase_pll)` is the
    /// instantaneous PLL error (debug telemetry / divergence forensics).
    pub fn phase_raw(&self) -> f32 {
        self.phase_raw_last
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    /// External-validity diagnostics: `(bemf_q_filt, valid_travel)` — the
    /// low-passed q-axis back-EMF proxy (V, signed) and the coherent phase
    /// travel accumulated toward [`READY_MIN_VALID_REVS`] (rad). For bench
    /// telemetry and sims; `is_ready` is the consumer that matters.
    pub fn validity(&self) -> (f32, f32) {
        (self.bemf_q_filt, self.valid_travel)
    }

    /// Whether the estimate is trustworthy for commutation.
    ///
    /// Four independent criteria, all required:
    /// - flux magnitude near λ (the integrator has built up a real flux
    ///   vector, not just noise),
    /// - PLL locked (filtered |phase error| small — a diverging PLL sits
    ///   near π),
    /// - enough speed for back-EMF to be observable at all (at standstill
    ///   the first two can hold on pure integrator memory),
    /// - EXTERNAL validity: the terminal voltage carried the back-EMF the
    ///   claimed rotation implies (e_q ≈ λ·ω̂) for
    ///   [`READY_MIN_VALID_REVS`] consecutive electrical revolutions.
    ///
    /// The first three are *internal* convergence — and all of them hold on
    /// a phantom lock, where residual inverter distortion feeds the flux
    /// integrator a rotating vector while the rotor stands still (bench
    /// 2026-07-06 staircase: handoff at observer 231 rad/s over a ramp at
    /// 23 → trust-loss deadlock; deterministic in the dead-time sim). The
    /// fourth is the physics check that a phantom cannot fake: pushing a
    /// current vector around a standing rotor costs ω·L·i volts on the q
    /// axis, a real rotation costs λ·ω — an order of magnitude apart at
    /// drive currents.
    pub fn is_ready(&self) -> bool {
        self.confidence >= READY_MIN_CONFIDENCE
            && self.phase_err_filt < READY_MAX_PHASE_ERR_RAD
            && self.velocity_pll.abs() >= READY_MIN_VELOCITY
            && self.valid_travel >= READY_MIN_VALID_REVS * core::f32::consts::TAU - 1e-3
    }

    /// Reset observer state. The adapted λ is kept — it is a physical
    /// parameter estimate, more current than the stored configuration.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.i_alpha_last = 0.0;
        self.i_beta_last = 0.0;
        self.phase_pll = 0.0;
        self.velocity_pll = 0.0;
        self.confidence = 0.0;
        self.phase_err_filt = core::f32::consts::PI;
        self.phase_raw_last = 0.0;
        self.i_f_alpha = 0.0;
        self.i_f_beta = 0.0;
        self.vel_cap = 0.0;
        self.vel_mag_filt = 0.0;
        self.bemf_q_filt = 0.0;
        self.valid_travel = 0.0;
        self.invalid_time = 0.0;
        self.lambda_learn_travel = 0.0;
    }

    /// Set motor parameters (re-anchors the λ-tracking bounds; resets the
    /// round-rotor model — re-apply saliency via the builder if needed)
    pub fn set_motor_params(&mut self, r: f32, l: f32, lambda: f32) {
        self.r = r;
        self.l = l;
        self.l_delta = 0.0;
        self.lambda = lambda.max(1e-6);
        if self.lambda_gain > 0.0 {
            self.lambda_min = self.lambda * 0.5;
            self.lambda_max = self.lambda * 2.0;
        }
    }

    /// Set PLL gains
    pub fn set_pll_gains(&mut self, kp: f32, ki: f32) {
        self.pll_kp = kp;
        self.pll_ki = ki;
    }

    /// Maximum continuous slip-gated time (s): past it the gate force-opens
    /// even if the driver keeps asserting it — a latched gate (iq
    /// permanently unreachable, e.g. at voltage saturation) would otherwise
    /// freeze the PLL forever while the rotor walks away. 30 ms covers the
    /// bench slip transients (5–15 ms) with margin.
    pub const SLIP_GATE_MAX_S: f32 = 0.03;

    /// Assert/clear the slip gate for the NEXT update (the driver detects a
    /// slip transient as a large |iq_ref − iq_meas| and calls this every
    /// cycle).
    ///
    /// The slip-kick ratchet (bench 2026-07-06/07, captures/sawtooth-*):
    /// every pole slip kicks the flux integrator forward ~0.3 rad, and the
    /// PLL integrates the kicks into a velocity runaway — slip → kick → ω̂
    /// up → drive faster → next slip sooner (the estimate sawtooth, dq OC
    /// when a beat pulse crosses the trip). The kicks live exactly in the
    /// windows where the current is far from its reference, so the PLL
    /// simply refuses to LEARN during them: the flux integrator stays
    /// honest (real physics), the angle dead-reckons at the held ω̂, and
    /// tracking resumes the moment the transient ends. Full gains
    /// everywhere else — a global gain reduction measurably degrades the
    /// healthy loop (sim) and only declaws the ratchet (bench).
    pub fn set_slip_gate(&mut self, gated: bool) {
        if !gated {
            self.slip_gate_time = 0.0;
        }
        self.slip_gate = gated;
    }

    /// Physics acceleration prior: cap the GROWTH RATE of |ω̂| at
    /// `floor + per_amp·|iq|` (el rad/s²; |iq| low-passed at τ ≈ 10 ms).
    ///
    /// The slip gate stops the kick-driven ratchet, but the bench
    /// (2026-07-07, captures/slipgate-1,2) showed a second escape mode
    /// with NOTHING for it to catch: a slow coherent phantom — currents
    /// perfectly regulated on the estimated frame, PLL error small but
    /// persistently positive (~+0.1 rad ⇒ ki·err ≈ +2000 el/s²), the PI
    /// winding vq up as the "back-EMF" of its own acceleration. The one
    /// physical fact it cannot fake: torque. |ω̂| growing 3× faster than
    /// `kt·|iq|/J` allows is not a rotor. Only |ω̂| GROWTH is capped —
    /// deceleration and any magnitude-shrinking correction stay free, so
    /// load-driven braking (a vehicle hitting a hill) is never fought;
    /// the floor keeps modest load-driven acceleration (downhill) inside
    /// the cap. `per_amp = margin·1.5·pp²·λ/J` — needs the rotor inertia,
    /// which detection does not measure yet, hence configured per bench.
    pub fn set_accel_prior(&mut self, floor_el: f32, per_amp_el: f32) {
        if floor_el >= 0.0 && per_amp_el >= 0.0 && floor_el.is_finite() && per_amp_el.is_finite() {
            self.accel_floor = floor_el;
            self.accel_per_amp = per_amp_el;
        }
    }

    /// Feed the measured |iq| for the acceleration prior (driver, per
    /// cycle; low-passed internally).
    pub fn note_torque_current(&mut self, iq_abs: f32, dt: f32) {
        let a = (dt / 0.01).min(1.0);
        self.iq_abs_filt += a * (iq_abs - self.iq_abs_filt);
    }

    /// Force phase to specific value (for testing or handoff from other source)
    pub fn force_phase(&mut self, phase: f32) {
        use crate::foc::trig::{FastSinCos, SinCos};
        self.phase_pll = wrap_angle(phase);
        // Also set flux state to match. FastSinCos (not libm): this runs in
        // the ISR on crossover reseed, and libm's f64-softfloat sinf would
        // blow the cycle budget on its own (-fp64 targets).
        let (s, c) = FastSinCos::sin_cos(phase);
        self.x1 = self.lambda * c;
        self.x2 = self.lambda * s;
        // Seeded from a trusted source: flux magnitude is exactly λ and the
        // PLL is on target by construction. The seed also carries the
        // external validity — the sources that seed (hall/encoder handoff,
        // the deadshort probe's measured back-EMF) are themselves physical
        // evidence, and a crossover reseed must not stall for two
        // revolutions re-proving it.
        self.confidence = 1.0;
        self.phase_err_filt = 0.0;
        self.valid_travel = READY_MIN_VALID_REVS * core::f32::consts::TAU;
        self.bemf_q_filt = self.lambda * self.velocity_pll;
        self.invalid_time = 0.0;
        // Seeded validity is NOT learning credit: λ adaptation stays frozen
        // until the corroboration is re-earned on the live flux integrator
        // (see `lambda_learn_travel`).
        self.lambda_learn_travel = 0.0;
    }

    /// Set velocity estimate (for testing or handoff from other source)
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity_pll = velocity;
        // A trusted seed carries its own envelope: the accel prior must
        // not clamp the estimate back toward the pre-seed speed.
        self.vel_cap = velocity.abs();
        self.vel_mag_filt = velocity.abs();
        // Handoff state, like force_phase: keep the external-validity proxy
        // consistent with the seeded velocity — seeds arrive as
        // force_phase + set_velocity in either order, and a stale proxy
        // would revoke the seed's validity credit on the first update.
        self.bemf_q_filt = self.lambda * velocity;
    }
}

// ============================================================================
// HFI Observer (High-Frequency Injection)
// ============================================================================

/// High-frequency injection observer for low/zero speed sensorless
///
/// Injects a high-frequency voltage signal and measures the current response
/// to estimate rotor position based on magnetic saliency (Ld ≠ Lq).
///
/// Works at zero and low speeds where back-EMF is too small to measure.
///
/// Generic over the sin/cos backend `S` — three trig calls run every ISR
/// cycle, and `libm`'s f64-based sinf/cosf cost ~6200 cycles/pair on the
/// `-fp64` Cortex-M4F targets (150% of the ISR budget on their own; see
/// docs/perf-bench-2026-06-11.md). Firmware picks `CordicSinCos` (G4) or
/// `FastSinCos` (F405); the `LibmSinCos` default keeps host sims maximally
/// accurate.
#[derive(Clone, Debug)]
#[cfg(feature = "hfi")]
pub struct HfiObserver<S: SinCos = LibmSinCos> {
    // Injection parameters
    frequency: f32,     // Injection frequency (Hz)
    amplitude: f32,     // Injection amplitude (V)
    carrier_phase: f32, // Carrier phase of the sample get_injection() returns

    // Demodulation state (see update() for the math)
    id_lp: f32,       // slow (fundamental) d-current tracker
    iq_lp: f32,       // slow (fundamental) q-current tracker
    eps_filt: f32,    // demodulated q-channel error (A)
    amp_d_filt: f32,  // demodulated d-channel carrier amplitude (A)
    err_quality: f32, // LPF of |normalized error|, for confidence

    phase_est: f32,    // Estimated rotor phase
    velocity_est: f32, // Estimated velocity

    // Tuning
    pll_kp: f32,
    pll_ki: f32,
    min_hf_current: f32, // d-channel carrier amplitude floor (A)

    // State
    confidence: f32,

    // Polarity probe state (π-ambiguity resolution, see update())
    polarity: HfiPolarity,
    probe_step: u32, // cycle index into the probe schedule
    probe_acc: f32,  // Σ sign·|id| over the probe pulses
    probe_ref: f32,  // Σ |id| (significance reference)

    // sin/cos backend marker (ZST)
    _sincos: PhantomData<S>,
}

/// Polarity resolution state. The saliency signal is 2θ-periodic, so the
/// PLL lock carries a π ambiguity that only magnetic saturation (or a
/// trusted sensor seed) can resolve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg(feature = "hfi")]
enum HfiPolarity {
    /// PLL not locked yet; the probe starts when confidence crosses ready.
    Pending,
    /// Saturation probe running: carrier suspended, ±d pulses injected.
    Probing,
    /// Resolved: probed, seeded from a sensor, or ambiguous-kept.
    Done,
}

/// Fundamental-tracker low-pass time constant (s). Its cutoff must sit well
/// below the carrier so the high-pass residual keeps the carrier content.
#[cfg(feature = "hfi")]
const HFI_FUND_TAU_S: f32 = 0.005;

/// Demodulation low-pass time constant (s) — a few carrier periods.
#[cfg(feature = "hfi")]
const HFI_DEMOD_TAU_S: f32 = 0.002;

/// Error-quality low-pass time constant (s), for confidence/readiness.
#[cfg(feature = "hfi")]
const HFI_QUALITY_TAU_S: f32 = 0.01;

/// Default d-channel carrier-amplitude floor (A): below this no injection
/// response is measurably flowing and the estimate is meaningless.
#[cfg(feature = "hfi")]
pub const HFI_MIN_HF_CURRENT_A: f32 = 0.05;

/// Confidence threshold for [`HfiObserver::is_ready`] and for starting
/// the polarity probe.
#[cfg(feature = "hfi")]
pub const HFI_READY_CONFIDENCE: f32 = 0.5;

/// Default HFI carrier frequency (Hz) when configuring from stored params.
#[cfg(feature = "hfi")]
pub const HFI_DEFAULT_FREQ_HZ: f32 = 1000.0;

/// Default HFI carrier amplitude as a fraction of vbus (3 V at 24 V — the
/// operating point validated in the closed-loop sims; bench-tune per motor).
/// Acts as the CEILING when the amplitude is solved from measured motor
/// inductance (see [`HFI_CARRIER_RIPPLE_TARGET_A`]) — on low-L motors the
/// raw ratio is dangerously large (3 V across an eskate outrunner's ~25 µH
/// at 1 kHz would drive tens of amps of carrier ripple).
#[cfg(feature = "hfi")]
pub const HFI_DEFAULT_AMPLITUDE_RATIO: f32 = 0.125;

/// Target peak carrier ripple current (A) when the HFI amplitude is solved
/// from measured motor inductance: `V = I_target · ω_c · L`. Large enough
/// to clear a real ADC noise floor, small enough to stay a perturbation —
/// the same reasoning as the detection-side adaptive amplitude
/// (`HFI_RIPPLE_FRACTION` in detection/sweep.rs), but with a fixed target
/// because there is no holding current to scale from at runtime.
// Sole consumer (configure_observers_from_config) is storage-gated.
#[cfg(feature = "storage")]
#[cfg(feature = "hfi")]
pub const HFI_CARRIER_RIPPLE_TARGET_A: f32 = 2.0;

/// Fraction of the motor's continuous-current RATING used as the carrier
/// ripple target when the rating is known (detection's thermal solve,
/// stored in the MotorParams group). Scales the perturbation to the
/// motor: ~2 A on a 15 A eskate outrunner, ~0.2 A on a 1.3 A gimbal —
/// capped by [`HFI_CARRIER_RIPPLE_TARGET_A`]. Deliberately scaled from
/// the RATING, not the session current limit: a bench config capping iq
/// at 2 A must not shrink the carrier SNR on a 30 A motor (the carrier
/// is a perturbation the MOTOR has to tolerate, not the session).
#[cfg(feature = "storage")]
#[cfg(feature = "hfi")]
pub const HFI_RIPPLE_RATING_FRACTION: f32 = 0.15;

/// Polarity probe: drive cycles per pulse. At 20 kHz this is 0.4 ms — with
/// the carrier amplitude on Ld in the 100 µH range the current reaches
/// ~V·t/Ld ≈ 10 A, enough to move the iron along its saturation curve.
#[cfg(feature = "hfi")]
const HFI_POLARITY_PULSE_CYCLES: u32 = 8;

/// Polarity probe: zero-voltage cycles after each pulse so the current
/// (τ = L/R, typically ~1 ms) decays before the opposite-sign pulse.
#[cfg(feature = "hfi")]
const HFI_POLARITY_GAP_CYCLES: u32 = 24;

/// Polarity probe pulse signs. The palindromic (+,−,−,+) order cancels the
/// first-order bias from residual current decaying across the schedule.
#[cfg(feature = "hfi")]
const HFI_POLARITY_PATTERN: [f32; 4] = [1.0, -1.0, -1.0, 1.0];

/// Significance floor for the flip decision: |Σ sign·|id|| must exceed this
/// fraction of Σ|id|. Below it there is no measurable saturation asymmetry
/// (SPM motor, probe too weak) and the current lock is kept as-is.
#[cfg(feature = "hfi")]
const HFI_POLARITY_MIN_RATIO: f32 = 0.05;

#[cfg(feature = "hfi")]
impl HfiObserver {
    /// Create a new HFI observer (with the default `LibmSinCos` backend —
    /// rebind via [`with_sincos`](Self::with_sincos) for firmware).
    ///
    /// # Arguments
    /// * `frequency` - Injection frequency (Hz), typically 500-2000 Hz
    /// * `amplitude` - Injection voltage amplitude (V)
    pub fn new(frequency: f32, amplitude: f32) -> Self {
        Self {
            frequency,
            amplitude,
            carrier_phase: 0.0,
            id_lp: 0.0,
            iq_lp: 0.0,
            eps_filt: 0.0,
            amp_d_filt: 0.0,
            err_quality: 1.0, // start "unlocked"
            phase_est: 0.0,
            velocity_est: 0.0,
            // PLL: ωn = √ki ≈ 45 rad/s, ζ ≈ 1.1 — an order slower than the
            // back-EMF PLL, because the input (demodulated saliency error)
            // is already low-passed by HFI_DEMOD_TAU_S and only needs to
            // track low-speed motion by design.
            pll_kp: 100.0,
            pll_ki: 2000.0,
            min_hf_current: HFI_MIN_HF_CURRENT_A,
            confidence: 0.0,
            polarity: HfiPolarity::Pending,
            probe_step: 0,
            probe_acc: 0.0,
            probe_ref: 0.0,
            _sincos: PhantomData,
        }
    }
}

#[cfg(feature = "hfi")]
impl<S: SinCos> HfiObserver<S> {
    /// Rebind the sin/cos backend (state-preserving, fields move as-is).
    pub fn with_sincos<S2: SinCos>(self) -> HfiObserver<S2> {
        HfiObserver {
            frequency: self.frequency,
            amplitude: self.amplitude,
            carrier_phase: self.carrier_phase,
            id_lp: self.id_lp,
            iq_lp: self.iq_lp,
            eps_filt: self.eps_filt,
            amp_d_filt: self.amp_d_filt,
            err_quality: self.err_quality,
            phase_est: self.phase_est,
            velocity_est: self.velocity_est,
            pll_kp: self.pll_kp,
            pll_ki: self.pll_ki,
            min_hf_current: self.min_hf_current,
            confidence: self.confidence,
            polarity: self.polarity,
            probe_step: self.probe_step,
            probe_acc: self.probe_acc,
            probe_ref: self.probe_ref,
            _sincos: PhantomData,
        }
    }

    /// Update observer with new measurements.
    ///
    /// # Contract with the control loop
    ///
    /// Each FOC cycle must call [`get_injection`](Self::get_injection)
    /// first, apply the returned dq voltage at [`phase`](Self::phase), and
    /// then call `update` with the resulting currents — the demodulator
    /// correlates them with the same carrier sample that produced them,
    /// and `update` advances the carrier afterwards.
    ///
    /// # Math (pulsating d-axis injection)
    ///
    /// With `v_d̂ = A·cos(θc)` injected on the *estimated* d axis and an
    /// estimation error `e = θ̂ − θ`, the carrier current measured back in
    /// the estimated frame is (R and rotation neglected at the carrier
    /// frequency):
    ///
    /// ```text
    /// i_d̂ = (A·sin θc / ωc) · (cos²e/Ld + sin²e/Lq)
    /// i_q̂ = (A·sin θc / 2ωc) · sin 2e · (1/Lq − 1/Ld)
    /// ```
    ///
    /// Synchronous demodulation by `sin θc` and low-passing gives a DC
    /// error channel `ε ∝ sin 2e · (1/Lq − 1/Ld)` and an always-positive
    /// d channel used for normalization (gains become independent of
    /// A, ωc and the absolute inductance) and for the "is any carrier
    /// current flowing at all" confidence floor. The saliency is
    /// 2θ-periodic, so the lock point carries a π ambiguity — resolved
    /// after the first PLL lock by the saturation probe (palindromic ±d
    /// pulses, `update_polarity_probe`), or by a trusted sensor seed via
    /// [`set_phase`](Self::set_phase).
    pub fn update(&mut self, input: &ObserverInput) {
        let dt = input.dt;
        if dt <= 0.0 {
            return;
        }

        // Measured currents into the estimated rotor frame.
        let (s_est, c_est) = S::sin_cos(self.phase_est);
        let (id, iq) = crate::foc::transforms::park(input.i_alpha, input.i_beta, s_est, c_est);

        // Split carrier content from the fundamental.
        let a_f = (dt / HFI_FUND_TAU_S).min(1.0);
        self.id_lp += a_f * (id - self.id_lp);
        self.iq_lp += a_f * (iq - self.iq_lp);
        let hf_id = id - self.id_lp;
        let hf_iq = iq - self.iq_lp;

        // While the polarity probe runs, the carrier is suspended and the
        // currents are pulse responses — feeding them to the demodulator or
        // PLL would corrupt the lock. Only the fundamental trackers above
        // keep running (so post-probe re-entry starts from the residual
        // current level); everything else is frozen until the probe ends.
        if self.polarity == HfiPolarity::Probing {
            self.update_polarity_probe(id);
            return;
        }

        // Synchronous demodulation with the carrier sample that generated
        // this response (see the call contract above).
        let dem = S::sin_cos(self.carrier_phase).0;
        let a_d = (dt / HFI_DEMOD_TAU_S).min(1.0);
        self.eps_filt += a_d * (hf_iq * dem - self.eps_filt);
        self.amp_d_filt += a_d * (hf_id * dem - self.amp_d_filt);

        // Normalized error: ∝ sin(2e) scaled by the saliency. The sign
        // below makes e = 0 the stable PLL equilibrium for normal saliency
        // (Ld < Lq) with this codebase's Park convention — validated
        // closed-loop against the salient VirtualMotor (a flipped sign
        // locks exactly 90° off). Inverse-saliency machines (Ld > Lq)
        // would need it inverted — not supported yet.
        let norm = self.amp_d_filt.abs().max(self.min_hf_current * 0.5);
        let err = self.eps_filt / norm;

        // PLL tracking.
        self.velocity_est += self.pll_ki * err * dt;
        self.phase_est = wrap_angle(self.phase_est + (self.velocity_est + self.pll_kp * err) * dt);

        // Confidence: a real carrier response is flowing AND the error
        // channel has settled near zero (locked).
        let a_q = (dt / HFI_QUALITY_TAU_S).min(1.0);
        self.err_quality += a_q * (err.abs() - self.err_quality);
        self.confidence = if self.amp_d_filt.abs() > self.min_hf_current * 0.5 {
            crate::foc::clamp_f32(1.0 - 2.0 * self.err_quality, 0.0, 1.0)
        } else {
            0.0
        };

        // Advance the carrier for the next get_injection().
        self.carrier_phase = wrap_angle(self.carrier_phase + TAU * self.frequency * dt);

        // First PLL lock → resolve the π ambiguity before reporting ready.
        if self.polarity == HfiPolarity::Pending && self.confidence >= HFI_READY_CONFIDENCE {
            self.polarity = HfiPolarity::Probing;
            self.probe_step = 0;
            self.probe_acc = 0.0;
            self.probe_ref = 0.0;
        }
    }

    /// One cycle of the saturation probe (see [`get_injection`](Self::get_injection)
    /// for the matching voltage schedule).
    ///
    /// A d-axis pulse aligned with the magnet flux saturates the iron →
    /// lower incremental Ld → larger current for the same volt-seconds.
    /// If the pulses along −d̂ consistently draw more current than +d̂,
    /// the estimated d axis points at the magnet's south pole: flip π.
    fn update_polarity_probe(&mut self, id: f32) {
        const SLOT: u32 = HFI_POLARITY_PULSE_CYCLES + HFI_POLARITY_GAP_CYCLES;
        let slot = (self.probe_step / SLOT) as usize;
        let pos = self.probe_step % SLOT;

        // Sample at the last drive cycle of each pulse — peak response.
        if slot < HFI_POLARITY_PATTERN.len() && pos == HFI_POLARITY_PULSE_CYCLES - 1 {
            self.probe_acc += HFI_POLARITY_PATTERN[slot] * id.abs();
            self.probe_ref += id.abs();
        }

        self.probe_step += 1;
        if self.probe_step >= HFI_POLARITY_PATTERN.len() as u32 * SLOT {
            if self.probe_acc < -HFI_POLARITY_MIN_RATIO * self.probe_ref {
                self.phase_est = wrap_angle(self.phase_est + core::f32::consts::PI);
            }
            // Ambiguous result (|acc| under the floor) keeps the current
            // lock: with no measurable saturation we cannot do better, and
            // retrying would just stall readiness forever.
            self.polarity = HfiPolarity::Done;
        }
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> f32 {
        self.phase_est
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> f32 {
        self.velocity_est
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    /// Whether the estimate is trustworthy: carrier current measurably
    /// flowing, the demodulated error settled near zero, AND the π
    /// ambiguity resolved — commutating on a possibly-flipped angle would
    /// produce torque in the wrong direction.
    pub fn is_ready(&self) -> bool {
        self.confidence >= HFI_READY_CONFIDENCE && self.polarity == HfiPolarity::Done
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        self.carrier_phase = 0.0;
        self.id_lp = 0.0;
        self.iq_lp = 0.0;
        self.eps_filt = 0.0;
        self.amp_d_filt = 0.0;
        self.err_quality = 1.0;
        self.phase_est = 0.0;
        self.velocity_est = 0.0;
        self.confidence = 0.0;
        self.polarity = HfiPolarity::Pending;
        self.probe_step = 0;
        self.probe_acc = 0.0;
        self.probe_ref = 0.0;
    }

    /// Get the injection voltage for the current FOC cycle.
    ///
    /// Returns `(vd_inject, vq_inject)` to apply in the estimated rotor
    /// frame (at [`phase`](Self::phase)) — e.g. via
    /// `FocController::step_with_injection` or `apply_dq`. Must be called
    /// before [`update`](Self::update) each cycle; see the contract there.
    ///
    /// During the polarity probe this returns the ±d saturation pulses
    /// instead of the carrier.
    pub fn get_injection(&self) -> (f32, f32) {
        if self.polarity == HfiPolarity::Probing {
            const SLOT: u32 = HFI_POLARITY_PULSE_CYCLES + HFI_POLARITY_GAP_CYCLES;
            let slot = (self.probe_step / SLOT) as usize;
            let pos = self.probe_step % SLOT;
            if slot < HFI_POLARITY_PATTERN.len() && pos < HFI_POLARITY_PULSE_CYCLES {
                return (HFI_POLARITY_PATTERN[slot] * self.amplitude, 0.0);
            }
            return (0.0, 0.0);
        }
        let vd = self.amplitude * S::sin_cos(self.carrier_phase).1;
        (vd, 0.0)
    }

    /// Set injection parameters
    pub fn set_injection(&mut self, frequency: f32, amplitude: f32) {
        self.frequency = frequency;
        self.amplitude = amplitude;
    }

    /// Set PLL gains
    pub fn set_pll_gains(&mut self, kp: f32, ki: f32) {
        self.pll_kp = kp;
        self.pll_ki = ki;
    }

    /// Whether the π ambiguity has been resolved (saturation probe done
    /// or estimate seeded from a trusted sensor).
    pub fn polarity_resolved(&self) -> bool {
        self.polarity == HfiPolarity::Done
    }

    /// Set initial phase estimate (for handoff from other source)
    ///
    /// A trusted external angle carries no π ambiguity, so this also marks
    /// polarity as resolved — no saturation probe needed.
    pub fn set_phase(&mut self, phase: f32) {
        self.phase_est = wrap_angle(phase);
        self.polarity = HfiPolarity::Done;
    }

    /// Set initial velocity estimate (for handoff from other source)
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity_est = velocity;
    }

    /// Restart the demodulator state after a carrier-off period.
    ///
    /// While the carrier is not injected the demod filters hold whatever
    /// they last saw — if `update()` was paused (the manager skips it when
    /// the carrier is off), that is the STALE pre-pause state, and resuming
    /// with it would report a spuriously high confidence for the first
    /// cycles even though no carrier response has been measured yet. Zero
    /// the carrier-content filters and mark the lock cold (`err_quality =
    /// 1`, confidence collapses to 0 on the next update) so readiness has
    /// to be re-earned from real carrier current. Phase, velocity and the
    /// resolved polarity are kept — they are handoff state
    /// ([`set_phase`](Self::set_phase)/[`set_velocity`](Self::set_velocity)),
    /// not demod state.
    pub fn restart_demod(&mut self) {
        self.eps_filt = 0.0;
        self.amp_d_filt = 0.0;
        self.err_quality = 1.0;
        self.confidence = 0.0;
    }
}

// ============================================================================
// Utility functions
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::angle_difference;

    /// Drive the observer with ideal PMSM terminal quantities for `steps`
    /// cycles and return the final true flux angle.
    ///
    /// The stator current is placed on the q axis (90° ahead of the flux),
    /// like a loaded motor under FOC. Voltages follow the PMSM model
    /// v = R·i + L·di/dt + e with e = ωλ·(−sinθ, cosθ).
    #[allow(clippy::too_many_arguments)] // test helper, positional args read fine
    fn run_observer(
        obs: &mut BackEmfObserver,
        r: f32,
        l: f32,
        lambda: f32,
        omega: f32,
        i_q: f32,
        steps: usize,
        dt: f32,
    ) -> f32 {
        let mut theta: f32 = 1.0;
        for _ in 0..steps {
            let theta_next = wrap_angle(theta + omega * dt);
            let (i_a, i_b) = (-i_q * libm::sinf(theta), i_q * libm::cosf(theta));
            let (i_a_next, i_b_next) =
                (-i_q * libm::sinf(theta_next), i_q * libm::cosf(theta_next));
            let v_a = r * i_a + l * (i_a_next - i_a) / dt - omega * lambda * libm::sinf(theta);
            let v_b = r * i_b + l * (i_b_next - i_b) / dt + omega * lambda * libm::cosf(theta);
            obs.update(&ObserverInput {
                v_alpha: v_a,
                v_beta: v_b,
                i_alpha: i_a,
                i_beta: i_b,
                dt,
            });
            theta = theta_next;
        }
        theta
    }

    #[test]
    fn back_emf_observer_converges_at_no_load() {
        // The PLL must lock onto a rotating flux vector. A PLL whose phase
        // error is wrapped to [0, 2π) (always non-negative) can only spin its
        // velocity estimate up and never converges — the error must be the
        // signed angle difference, like VESC's foc_pll_run (foc_math.c:227).
        let (r, l, lambda) = (0.1, 50e-6, 0.005);
        let omega = 300.0; // rad/s electrical
        let dt = 5e-5; // 20 kHz
        let mut obs = BackEmfObserver::new(r, l, lambda);
        let theta = run_observer(&mut obs, r, l, lambda, omega, 0.0, 40_000, dt);

        assert!(
            (obs.velocity() - omega).abs() < 0.05 * omega,
            "velocity {} did not lock to {}",
            obs.velocity(),
            omega
        );
        let err = angle_difference(obs.phase(), theta);
        assert!(err.abs() < 0.1, "phase error {err} rad too large");
        assert!(obs.confidence() > 0.5, "confidence {}", obs.confidence());
    }

    /// Drive the observer with EXACT salient-PMSM terminal quantities
    /// (id = 0, iq = I): ψ_αβ = (−Lq·I·sinθ + λ·cosθ, Lq·I·cosθ + λ·sinθ),
    /// v = R·i + Δψ/dt. Returns the final true flux angle.
    #[allow(clippy::too_many_arguments)] // test helper, positional args read fine
    fn run_observer_salient(
        obs: &mut BackEmfObserver,
        r: f32,
        lq: f32,
        lambda: f32,
        omega: f32,
        i_q: f32,
        steps: usize,
        dt: f32,
    ) -> f32 {
        let psi = |theta: f32| {
            (
                -lq * i_q * libm::sinf(theta) + lambda * libm::cosf(theta),
                lq * i_q * libm::cosf(theta) + lambda * libm::sinf(theta),
            )
        };
        let mut theta: f32 = 1.0;
        for _ in 0..steps {
            let theta_next = wrap_angle(theta + omega * dt);
            let (i_a, i_b) = (-i_q * libm::sinf(theta), i_q * libm::cosf(theta));
            let (pa, pb) = psi(theta);
            let (pa_n, pb_n) = psi(theta_next);
            obs.update(&ObserverInput {
                v_alpha: r * i_a + (pa_n - pa) / dt,
                v_beta: r * i_b + (pb_n - pb) / dt,
                i_alpha: i_a,
                i_beta: i_b,
                dt,
            });
            theta = theta_next;
        }
        theta
    }

    #[test]
    fn slip_gate_holds_pll_and_dead_reckons() {
        // Lock the observer on a clean rotation, then flag a slip window
        // and feed it a garbage current transient: the PLL velocity must
        // HOLD (no kick integration), the angle must dead-reckon at the
        // held velocity, and tracking must resume cleanly after the gate.
        let (r, l, lambda_true) = (0.1, 50e-6, 0.005);
        let omega = 300.0;
        let dt = 5e-5;
        let mut obs = BackEmfObserver::new(r, l, lambda_true);
        run_observer(&mut obs, r, l, lambda_true, omega, 0.0, 20_000, dt);
        let v_locked = obs.velocity();
        assert!((v_locked - omega).abs() < 5.0, "not locked: {v_locked}");

        // Gated garbage: zero voltage + a big DC current step for 4 ms —
        // exactly the kind of transient that kicks the flux integrator.
        obs.set_slip_gate(true);
        let phase_before = obs.phase();
        let mut expected_phase = phase_before;
        for _ in 0..80 {
            obs.update(&ObserverInput {
                v_alpha: 0.0,
                v_beta: 0.0,
                i_alpha: 5.0,
                i_beta: -3.0,
                dt,
            });
            expected_phase = wrap_angle(expected_phase + v_locked * dt);
        }
        assert!(
            (obs.velocity() - v_locked).abs() < 1e-3,
            "gated velocity must hold: {} vs {}",
            obs.velocity(),
            v_locked
        );
        assert!(
            angle_difference(obs.phase(), expected_phase).abs() < 1e-3,
            "gated angle must dead-reckon: {} vs {}",
            obs.phase(),
            expected_phase
        );
        obs.set_slip_gate(false);

        // Resume clean rotation from wherever the plant is: re-locks.
        run_observer(&mut obs, r, l, lambda_true, omega, 0.0, 20_000, dt);
        assert!(
            (obs.velocity() - omega).abs() < 5.0,
            "must re-lock after the gate: {}",
            obs.velocity()
        );
    }

    #[test]
    fn slip_gate_duty_limit_force_opens() {
        // A latched gate (driver asserting forever, e.g. unreachable iq at
        // voltage saturation) must force-open after SLIP_GATE_MAX_S so the
        // PLL cannot stay frozen while the rotor walks away.
        let (r, l, lambda_true) = (0.1, 50e-6, 0.005);
        let omega = 300.0;
        let dt = 5e-5;
        let mut obs = BackEmfObserver::new(r, l, lambda_true);
        run_observer(&mut obs, r, l, lambda_true, omega, 0.0, 20_000, dt);
        obs.set_slip_gate(true);
        // Rotor genuinely accelerates to a new speed while the gate is
        // held far past the duty limit: the estimate must follow anyway.
        let omega2 = 500.0;
        let steps = (10.0 * BackEmfObserver::SLIP_GATE_MAX_S / dt) as usize;
        run_observer(&mut obs, r, l, lambda_true, omega2, 0.0, steps, dt);
        assert!(
            (obs.velocity() - omega2).abs() < 25.0,
            "duty-limited gate must not freeze tracking: {}",
            obs.velocity()
        );
    }

    #[test]
    fn accel_prior_caps_phantom_growth_but_tracks_real_torque() {
        // A rotation accelerating far past what the measured torque
        // current allows is a phantom: the prior must cap the estimate's
        // growth. The same profile WITH the current to justify it must
        // track.
        let (r, l, lambda_true) = (0.1, 50e-6, 0.005);
        let dt = 5e-5;
        let run_accel = |obs: &mut BackEmfObserver, w0: f32, alpha: f32, secs: f32| {
            let mut theta: f32 = 0.0;
            let steps = (secs / dt) as usize;
            for k in 0..steps {
                let w = w0 + alpha * k as f32 * dt;
                let e = w * lambda_true;
                obs.update(&ObserverInput {
                    v_alpha: -e * libm::sinf(theta),
                    v_beta: e * libm::cosf(theta),
                    i_alpha: 0.0,
                    i_beta: 0.0,
                    dt,
                });
                theta = wrap_angle(theta + w * dt);
            }
        };
        // Lock at 300, then the source accelerates at 5000 el/s² with ZERO
        // torque current: growth must be capped at the 500 floor.
        let mut obs = BackEmfObserver::new(r, l, lambda_true);
        obs.set_accel_prior(500.0, 3400.0);
        // Seed at speed (the cap would otherwise pace the from-zero lock-in
        // itself — with zero measured torque that is exactly the intended
        // behavior, but here we test the locked regime).
        obs.force_phase(0.0);
        obs.set_velocity(300.0);
        run_accel(&mut obs, 300.0, 0.0, 0.5);
        assert!(
            (obs.velocity() - 300.0).abs() < 5.0,
            "lock: {}",
            obs.velocity()
        );
        run_accel(&mut obs, 300.0, 5000.0, 0.1);
        let capped1 = obs.velocity();
        // The 50 ms trend filter grants a one-time ≈ α·τ transient before
        // the envelope engages (the price of not rectifying the mid-band
        // wobble — see the envelope block); the protection property is
        // the SUSTAINED rate below.
        assert!(
            capped1 < 300.0 + 500.0 * 0.1 + 5000.0 * 0.05 + 20.0,
            "phantom transient must stay bounded: {capped1}"
        );
        run_accel(&mut obs, 300.0 + 500.0, 5000.0, 0.2);
        let capped2 = obs.velocity();
        // Governor property: sustained growth ≤ ~2× the envelope rate
        // (the excess equilibrium where the PLL rebuild balances the
        // governor pull) — vs the unbounded 2–3× physics of the bench
        // slow-phantom this exists to stop.
        assert!(
            capped2 - capped1 < 500.0 * 0.2 * 2.0,
            "sustained phantom growth must be rate-capped: {capped1} -> {capped2}"
        );
        // Same acceleration with 2 A of measured iq (cap 500+6800): tracks.
        let mut obs2 = BackEmfObserver::new(r, l, lambda_true);
        obs2.set_accel_prior(500.0, 3400.0);
        obs2.force_phase(0.0);
        obs2.set_velocity(300.0);
        run_accel(&mut obs2, 300.0, 0.0, 0.5);
        for _ in 0..400 {
            obs2.note_torque_current(2.0, dt);
        }
        let mut theta: f32 = 0.0;
        let steps = (0.1 / dt) as usize;
        for k in 0..steps {
            let w = 300.0 + 5000.0 * k as f32 * dt;
            let e = w * lambda_true;
            obs2.note_torque_current(2.0, dt);
            obs2.update(&ObserverInput {
                v_alpha: -e * libm::sinf(theta),
                v_beta: e * libm::cosf(theta),
                i_alpha: 0.0,
                i_beta: 0.0,
                dt,
            });
            theta = wrap_angle(theta + w * dt);
        }
        // Control: same profile with the prior OFF — isolates whether a
        // shortfall is the clamp or plain PLL dynamics.
        let mut obs3 = BackEmfObserver::new(r, l, lambda_true);
        obs3.force_phase(0.0);
        obs3.set_velocity(300.0);
        run_accel(&mut obs3, 300.0, 0.0, 0.5);
        let mut theta: f32 = 0.0;
        for k in 0..steps {
            let w = 300.0 + 5000.0 * k as f32 * dt;
            let e = w * lambda_true;
            obs3.update(&ObserverInput {
                v_alpha: -e * libm::sinf(theta),
                v_beta: e * libm::cosf(theta),
                i_alpha: 0.0,
                i_beta: 0.0,
                dt,
            });
            theta = wrap_angle(theta + w * dt);
        }
        // The prior must not impede legitimate (torque-justified) tracking:
        // with 2 A measured the cap (500 + 6800 el/s²) sits above both the
        // source ramp AND the PLL's own tracking dynamics, so the estimate
        // must match the no-prior control (which itself lags the ramp by
        // the PLL's velocity pole ki/kp ≈ 20 rad/s — that lag is PLL
        // dynamics, not the prior).
        assert!(
            (obs2.velocity() - obs3.velocity()).abs() < 5.0,
            "prior must not impede justified tracking: {} vs control {}",
            obs2.velocity(),
            obs3.velocity()
        );
    }

    #[test]
    fn lambda_tracking_follows_true_flux() {
        // Stored λ is one bench point; the real value drifts with magnet
        // temperature and saturation. With tracking, the observer's λ must
        // converge to the true flux magnitude and confidence must recover
        // (without tracking it would sit at λ_true/λ_cfg ≈ 0.55).
        let (r, l, lambda_true) = (0.1, 50e-6, 0.005);
        let lambda_cfg = lambda_true * 1.8;
        let omega = 300.0;
        let dt = 5e-5;
        let mut obs =
            BackEmfObserver::new(r, l, lambda_cfg).with_lambda_tracking(DEFAULT_LAMBDA_GAIN);
        // 2 s of sim: λ tracker has τ = 0.2 s.
        run_observer(&mut obs, r, l, lambda_true, omega, 0.0, 40_000, dt);

        assert!(
            (obs.lambda() - lambda_true).abs() < 0.1 * lambda_true,
            "λ estimate {} did not converge to true {}",
            obs.lambda(),
            lambda_true
        );
        assert!(
            obs.confidence() > 0.9,
            "confidence {} should recover once λ adapts",
            obs.confidence()
        );
        assert!(obs.is_ready());
    }

    #[test]
    fn salient_observer_beats_scalar_on_ipm() {
        // IPM under load: Lq·I is comparable to λ, so the scalar-L observer
        // (l_avg) misattributes part of the stator flux to the rotor and
        // locks with a load-dependent angle bias. The salient model must cut
        // that error substantially (the diagonal Lα/Lβ approximation is not
        // exact, so "substantially", not "to zero").
        let (r, ld, lq, lambda) = (0.1, 100e-6, 300e-6, 0.005);
        let (omega, i_q, dt) = (300.0, 15.0, 5e-5); // Lq·I = 4.5 mWb ≈ λ
        let l_avg = (ld + lq) / 2.0;

        let mut scalar = BackEmfObserver::new(r, l_avg, lambda);
        let theta_s = run_observer_salient(&mut scalar, r, lq, lambda, omega, i_q, 40_000, dt);
        let err_scalar = angle_difference(scalar.phase(), theta_s).abs();

        let mut salient = BackEmfObserver::new(r, l_avg, lambda).with_saliency(ld, lq);
        let theta_x = run_observer_salient(&mut salient, r, lq, lambda, omega, i_q, 40_000, dt);
        let err_salient = angle_difference(salient.phase(), theta_x).abs();

        // Active flux is exact for this plant (id = 0): only numerical and
        // PLL-lag residue remains.
        assert!(
            err_salient < 0.05,
            "active-flux observer error {err_salient} rad (scalar comparison: {err_scalar})"
        );
        assert!(
            err_salient < err_scalar * 0.25,
            "active flux should slash the load bias: scalar {err_scalar} rad, salient {err_salient} rad"
        );
    }

    #[test]
    fn centering_keeps_lock_under_voltage_offset() {
        // A DC offset on the measured/commanded voltage (ADC offset, dead
        // time residue) makes the flux integral drift; the centering (and
        // clamp backstop) must bleed it off without losing the lock. The
        // radial pull corrects without the angle distortion the bare
        // component clamp causes at the ±λ square's corners.
        let (r, l, lambda) = (0.1, 50e-6, 0.005);
        let (omega, dt) = (300.0, 5e-5);
        let mut obs = BackEmfObserver::new(r, l, lambda);

        let mut theta: f32 = 1.0;
        let mut max_err = 0.0f32;
        for step in 0..40_000 {
            let theta_next = wrap_angle(theta + omega * dt);
            let v_a = -omega * lambda * libm::sinf(theta) + 0.05; // ADC-scale DC offset
            let v_b = omega * lambda * libm::cosf(theta);
            obs.update(&ObserverInput {
                v_alpha: v_a,
                v_beta: v_b,
                i_alpha: 0.0,
                i_beta: 0.0,
                dt,
            });
            theta = theta_next;
            // Measure over the last electrical revolution.
            if step > 40_000 - 420 {
                max_err = max_err.max(angle_difference(obs.phase(), theta).abs());
            }
        }
        assert!(
            max_err < 0.15,
            "lock lost under DC voltage offset: max err {max_err} rad"
        );
        assert!(obs.is_ready());
    }

    #[test]
    fn back_emf_observer_unbiased_under_load() {
        // The observer integrates total stator flux ∫(v − R·i); the rotor flux
        // is that minus L·i. Without the −L·Δi correction (VESC MXLEMMING,
        // foc_math.c:118-137) a q-axis load current of L·I = λ skews the
        // estimated angle by atan(L·I/λ) = 45°.
        let (r, l, lambda) = (0.1, 500e-6, 0.005);
        let omega = 300.0;
        let i_q = 10.0; // L·I = 0.005 = λ
        let dt = 5e-5;
        let mut obs = BackEmfObserver::new(r, l, lambda);
        let theta = run_observer(&mut obs, r, l, lambda, omega, i_q, 40_000, dt);

        let err = angle_difference(obs.phase(), theta);
        assert!(
            err.abs() < 0.15,
            "phase error {err} rad under load (L·I = λ) — missing −L·Δi term?"
        );
    }

    /// |angle error| folded to the saliency period: HFI is 2θ-periodic, so
    /// without polarity detection θ and θ+π are equally valid lock points.
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn angle_err_mod_pi(a: f32, b: f32) -> f32 {
        let err = angle_difference(a, b).abs();
        err.min(core::f32::consts::PI - err)
    }

    /// Closed-loop HFI harness: pulsating injection on the estimated d axis
    /// through the real FocController voltage path into a salient
    /// VirtualMotor. `sat_k` enables d-axis saturation (needed for the
    /// polarity tests). Returns (observer, final motor output).
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn run_hfi_sim(
        rotor_angle: f32,
        load_torque: f32,
        sat_k: f32,
        steps: usize,
    ) -> (HfiObserver, crate::virtual_motor::VirtualMotorOutput) {
        use crate::virtual_motor::MotorParams;
        // IPM with 3:1 saliency; heavy rotor + friction so the injection
        // itself doesn't move it.
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-2,
            friction_b: 5e-2,
            hall_offset: 0.0,
            sat_k,
            ..MotorParams::default()
        };
        run_hfi_sim_params(params, rotor_angle, load_torque, steps)
    }

    /// Closed-loop HFI harness over an explicit plant parameterization
    /// (lets tests opt into sensor noise / quantization / saturation).
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn run_hfi_sim_params(
        params: crate::virtual_motor::MotorParams,
        rotor_angle: f32,
        load_torque: f32,
        steps: usize,
    ) -> (HfiObserver, crate::virtual_motor::VirtualMotorOutput) {
        use crate::foc::controller::FocController;
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::virtual_motor::VirtualMotor;

        const DT: f32 = 1.0 / 20_000.0;
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(rotor_angle);

        let foc = FocController::<SvpwmModulator>::new(24.0);
        let mut obs = HfiObserver::new(1000.0, 3.0);

        let mut out = crate::virtual_motor::VirtualMotorOutput::default();
        for _ in 0..steps {
            let (vd_inj, vq_inj) = obs.get_injection();
            let (i_a_m, i_b_m) = transforms::clarke(out.ia, out.ib);
            let telem = foc.apply_dq(vd_inj, vq_inj, obs.phase(), i_a_m, i_b_m, 1000);
            out = motor.step(telem.v_alpha, telem.v_beta, load_torque, DT);
            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            obs.update(&ObserverInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha: i_a,
                i_beta: i_b,
                dt: DT,
            });
        }
        (obs, out)
    }

    #[test]
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn hfi_finds_rotor_angle_at_standstill() {
        // Rotor parked 1.2 rad away from the initial estimate; the observer
        // must find it from saliency alone, without moving the rotor.
        let (obs, out) = run_hfi_sim(1.2, 0.0, 0.0, 20_000);

        let err = angle_err_mod_pi(obs.phase(), wrap_angle(out.angle_rad));
        assert!(
            err < 0.1,
            "HFI did not converge: est {} vs rotor {} (err {} rad mod π)",
            obs.phase(),
            wrap_angle(out.angle_rad),
            err
        );
        assert!(
            angle_difference(out.angle_rad, 1.2).abs() < 0.15,
            "injection moved the rotor: {} rad",
            out.angle_rad
        );
        assert!(
            obs.confidence() > 0.5,
            "converged HFI must report confidence, got {}",
            obs.confidence()
        );
    }

    #[test]
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn hfi_tracks_slow_rotation() {
        // External load torque turns the rotor slowly (well below any
        // back-EMF-observable speed); HFI must keep tracking.
        let (obs, out) = run_hfi_sim(0.3, 0.5, 0.0, 30_000);

        assert!(
            out.omega_e.abs() > 5.0,
            "rotor should be turning under load: ωe = {}",
            out.omega_e
        );
        let err = angle_err_mod_pi(obs.phase(), wrap_angle(out.angle_rad));
        assert!(
            err < 0.15,
            "HFI lost a slowly turning rotor: est {} vs {} (err {})",
            obs.phase(),
            wrap_angle(out.angle_rad),
            err
        );
    }

    #[test]
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn hfi_polarity_probe_corrects_pi_flipped_lock() {
        // Rotor at 2.5 rad, estimate starting at 0: the PLL's nearest
        // saliency equilibrium is the flipped one (e = −π), so it locks
        // π off. With saturation modeled (sat_k > 0) the polarity probe
        // must detect the inverted d axis and flip the estimate — the
        // final angle must match FULL-circle, not just mod π.
        let (obs, out) = run_hfi_sim(2.5, 0.0, 0.05, 20_000);

        assert!(
            obs.polarity_resolved(),
            "probe must have run and resolved polarity"
        );
        let err = angle_difference(obs.phase(), wrap_angle(out.angle_rad)).abs();
        assert!(
            err < 0.15,
            "polarity not corrected: est {} vs rotor {} (err {} rad full-circle)",
            obs.phase(),
            wrap_angle(out.angle_rad),
            err
        );
    }

    #[test]
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn hfi_polarity_probe_keeps_correct_lock() {
        // Rotor at 1.2 rad: the PLL locks on the true d axis. The probe
        // must confirm (not flip) it.
        let (obs, out) = run_hfi_sim(1.2, 0.0, 0.05, 20_000);

        assert!(obs.polarity_resolved());
        let err = angle_difference(obs.phase(), wrap_angle(out.angle_rad)).abs();
        assert!(
            err < 0.15,
            "correct lock was flipped: est {} vs rotor {} (err {} rad full-circle)",
            obs.phase(),
            wrap_angle(out.angle_rad),
            err
        );
    }

    /// The demod must lock through a realistic current sensor: 12-bit ADC
    /// quantization (±31 A FS → ~15 mA LSB), one LSB of uniform noise and
    /// sub-stepped plant integration. The confidence/readiness thresholds
    /// were originally tuned on noiseless currents — this pins that they
    /// hold under honest sensing (carrier ripple ≈ 4.8 A ≫ noise floor,
    /// but the demod filters see every sample).
    #[test]
    #[cfg(all(feature = "virtual-motor", feature = "hfi"))]
    fn hfi_locks_through_quantized_noisy_sensor() {
        use crate::virtual_motor::MotorParams;
        const ADC_LSB_A: f32 = 62.0 / 4096.0;
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-2,
            friction_b: 5e-2,
            sat_k: 0.05,
            substeps: 10,
            adc_lsb_a: ADC_LSB_A,
            adc_noise_a: ADC_LSB_A,
            ..MotorParams::default()
        };
        let (obs, out) = run_hfi_sim_params(params, 1.2, 0.0, 20_000);

        assert!(
            obs.polarity_resolved(),
            "polarity probe must resolve through sensor noise"
        );
        assert!(
            obs.is_ready(),
            "HFI must reach readiness through sensor noise: confidence {}",
            obs.confidence()
        );
        let err = angle_difference(obs.phase(), wrap_angle(out.angle_rad)).abs();
        assert!(
            err < 0.15,
            "HFI did not converge through sensor noise: est {} vs rotor {} (err {})",
            obs.phase(),
            wrap_angle(out.angle_rad),
            err
        );
    }

    /// The firmware feeds estimators the PREVIOUS cycle's voltage
    /// (`update_phase_with_prev_voltage`): with a one-period actuation
    /// pipeline, the currents measured now responded to the voltage
    /// commanded one cycle ago. This pins that convention against a plant
    /// that actually has the delay — previous-cycle pairing must match the
    /// no-delay baseline, same-cycle pairing degrades roughly 2×.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn observer_prev_voltage_pairing_matches_actuation_delay() {
        use crate::foc::controller::FocController;
        use crate::foc::pi_controller::PIController;
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

        const DT: f32 = 1.0 / 20_000.0;
        const VBUS: f32 = 24.0;

        // mean |angle error| of a passive observer after settling
        let run = |delay: u8, prev_pairing: bool| -> f32 {
            let params = MotorParams {
                actuation_delay_steps: delay,
                substeps: 10,
                friction_b: 1.2e-3, // settle ≈ 1200 rad/s el
                ..MotorParams::default()
            };
            let kp = params.ld * 1000.0;
            let ki = params.r * 1000.0;
            let mut foc = FocController::<SvpwmModulator>::new(VBUS);
            foc.id_pi = PIController::new(kp, ki);
            foc.iq_pi = PIController::new(kp, ki);
            let mut motor = VirtualMotor::new(params);
            let mut out = VirtualMotorOutput::default();
            let mut obs =
                BackEmfObserver::new(params.r, (params.ld + params.lq) / 2.0, params.lambda);
            let (mut pva, mut pvb) = (0.0f32, 0.0f32);
            let mut err_sum = 0.0f32;
            let mut n = 0u32;
            for step in 0..30_000 {
                // commutation from the true rotor state (perfect estimator);
                // the observer under test runs passively
                foc.set_actuation_advance(out.omega_e * DT);
                let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 4250, DT);
                out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
                let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
                let (va, vb) = if prev_pairing {
                    (pva, pvb)
                } else {
                    (telem.v_alpha, telem.v_beta)
                };
                obs.update(&ObserverInput {
                    v_alpha: va,
                    v_beta: vb,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                });
                pva = telem.v_alpha;
                pvb = telem.v_beta;
                if step >= 20_000 {
                    err_sum += angle_difference(obs.phase(), wrap_angle(out.angle_rad)).abs();
                    n += 1;
                }
            }
            err_sum / n as f32
        };

        let baseline = run(0, false);
        let faithful = run(1, true);
        let naive = run(1, false);
        assert!(
            faithful < baseline * 1.2 + 0.01,
            "prev-voltage pairing must match the no-delay baseline: {faithful} vs {baseline}"
        );
        assert!(
            naive > faithful * 1.7,
            "same-cycle pairing must visibly degrade on a delayed plant: {naive} vs {faithful}"
        );
    }

    #[test]
    fn test_observer_none() {
        let obs = Observer::None;
        assert!(!obs.is_configured());
        assert!(obs.phase().is_none());
        assert!(obs.velocity().is_none());
        assert_eq!(obs.confidence(), 0.0);
    }

    #[test]
    fn test_back_emf_observer_creation() {
        let obs = BackEmfObserver::new(0.1, 0.0001, 0.01);
        assert_eq!(obs.phase(), 0.0);
        assert_eq!(obs.velocity(), 0.0);
        assert_eq!(obs.confidence(), 0.0);
    }

    #[test]
    #[cfg(feature = "hfi")]
    fn test_hfi_observer_creation() {
        let obs = HfiObserver::new(1000.0, 3.0);
        assert_eq!(obs.phase(), 0.0);
        assert_eq!(obs.velocity(), 0.0);
    }

    #[test]
    fn test_observer_enum() {
        let mut obs = Observer::BackEmf(BackEmfObserver::new(0.1, 0.0001, 0.01));
        assert!(obs.is_configured());
        assert!(obs.phase().is_some());

        obs.reset();
        assert_eq!(obs.phase(), Some(0.0));
    }
}
