//! Sensorless observers for FOC control
//!
//! Provides software-based angle estimation for sensorless motor control.
//! Includes back-EMF observer for medium/high speed and HFI for low/zero speed.

use core::f32::consts::TAU;
use core::marker::PhantomData;

use crate::foc::trig::{LibmSinCos, SinCos};
use crate::foc::wrap_angle;

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
        Observer::None
    }
}

impl Observer {
    /// Update observer with new measurements
    pub fn update(&mut self, input: &ObserverInput) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => o.update(input),
        }
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.phase()),
        }
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.velocity()),
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
            Observer::None => false,
            Observer::BackEmf(o) => o.is_ready(),
        }
    }

    /// Seed the estimate from a trusted external source (sensor handoff).
    pub fn seed(&mut self, angle: f32, velocity: f32) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => {
                o.force_phase(angle);
                o.set_velocity(velocity);
            }
        }
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        match self {
            Observer::None => 0.0,
            Observer::BackEmf(o) => o.confidence(),
        }
    }

    /// Check if observer is configured
    pub fn is_configured(&self) -> bool {
        !matches!(self, Observer::None)
    }

    /// Phase resistance of the underlying motor model, if any — used by
    /// voltage-based crossover criteria (|vq − R·iq| back-EMF proxy).
    pub fn resistance(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.resistance()),
        }
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => o.reset(),
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
    l: f32, // Inductance subtracted from the flux integral (H); Lq in the
    // salient "active flux" configuration, the plain phase L otherwise
    /// Lq − Ld (H), informational (active-flux magnitude shift under
    /// d-current). 0 = round-rotor.
    l_delta: f32,
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

/// Default nonlinear centering gain (1/s, normalized error): drains a
/// radius error with τ ≈ 2 ms — orders of magnitude faster than
/// measurement-offset drift accumulates, while staying gentle on spin-up
/// transients (an aggressive pull there measurably perturbs the
/// hall→observer handoff timing in the closed-loop sims; 5000 broke
/// their continuity assertions, 500 does not).
pub const DEFAULT_CENTERING_GAIN: f32 = 500.0;

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
            lambda,
            lambda_gain: 0.0,
            lambda_min: lambda,
            lambda_max: lambda,
            centering_gain: DEFAULT_CENTERING_GAIN,
            pll_kp: 1000.0,
            pll_ki: 20000.0,
            confidence: 0.0,
            // Start "unlocked": a fresh observer must not look ready until
            // the PLL has actually tracked something.
            phase_err_filt: core::f32::consts::PI,
        }
    }

    /// Use the salient (IPM) "active flux" model: the integrator subtracts
    /// Lq·i (not L_avg·i), which leaves a vector exactly aligned with the
    /// d axis for any load — a scalar L_avg on an IPM motor gives a
    /// load-dependent angle bias instead.
    pub fn with_saliency(mut self, ld: f32, lq: f32) -> Self {
        if ld > 0.0 && lq > 0.0 {
            self.l = lq;
            self.l_delta = lq - ld;
        }
        self
    }

    /// Enable online λ adaptation: λ tracks the raw flux magnitude with a
    /// first-order filter (`gain` in 1/s), clamped to [λ₀/2, 2λ₀]. Makes
    /// the configured flux linkage non-critical (it drifts with saturation
    /// and magnet temperature) and un-breaks `confidence = |flux|/λ` when
    /// the stored value is off.
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
        self.x1 += (input.v_alpha - self.r * input.i_alpha) * dt
            - self.l * (input.i_alpha - self.i_alpha_last);
        self.x2 += (input.v_beta - self.r * input.i_beta) * dt
            - self.l * (input.i_beta - self.i_beta_last);
        self.i_alpha_last = input.i_alpha;
        self.i_beta_last = input.i_beta;

        // Raw (pre-correction) flux magnitude: the honest amplitude signal
        // for λ tracking and confidence, taken before centering/clamping
        // pull it onto the configured circle.
        let flux_mag = crate::foc::fast_math::sqrtf(self.x1 * self.x1 + self.x2 * self.x2);

        // Online λ adaptation (MESC MXLEMMING_LAMBDA / VESC lambda-comp):
        // slow first-order tracker, bounded. Adapts the clamp/centering
        // circle and the confidence normalization with it.
        if self.lambda_gain > 0.0 {
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
        if self.centering_gain > 0.0 && flux_mag > self.lambda {
            let err_norm = 1.0 - (flux_mag * flux_mag) / (self.lambda * self.lambda);
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

        // PLL tracking. The error must be the SIGNED shortest angular distance
        // (like VESC's foc_pll_run): wrapping to [0, 2π) would make the error
        // always non-negative and the velocity integrator could only grow.
        let phase_error = crate::foc::angle_difference(phase_raw, self.phase_pll);
        self.velocity_pll += self.pll_ki * phase_error * dt;
        self.phase_pll =
            wrap_angle(self.phase_pll + (self.velocity_pll + self.pll_kp * phase_error) * dt);

        // Track lock quality for is_ready(): low-passed |phase error|.
        let alpha = (dt / PHASE_ERR_FILTER_TAU_S).min(1.0);
        self.phase_err_filt += alpha * (phase_error.abs() - self.phase_err_filt);

        // Confidence: how close the (raw, pre-correction) flux magnitude is
        // to λ. A weak heuristic — measurement offsets can also saturate the
        // integrator — but cheap and monotonic during real spin-up. With λ
        // tracking enabled the normalization adapts along with the clamp.
        self.confidence = crate::foc::clamp_f32(flux_mag / self.lambda, 0.0, 1.0);
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> f32 {
        self.phase_pll
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> f32 {
        self.velocity_pll
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    /// Whether the estimate is trustworthy for commutation.
    ///
    /// Three independent criteria, all required:
    /// - flux magnitude near λ (the integrator has built up a real flux
    ///   vector, not just noise),
    /// - PLL locked (filtered |phase error| small — a diverging PLL sits
    ///   near π),
    /// - enough speed for back-EMF to be observable at all (at standstill
    ///   the first two can hold on pure integrator memory).
    pub fn is_ready(&self) -> bool {
        self.confidence >= READY_MIN_CONFIDENCE
            && self.phase_err_filt < READY_MAX_PHASE_ERR_RAD
            && self.velocity_pll.abs() >= READY_MIN_VELOCITY
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
        // PLL is on target by construction.
        self.confidence = 1.0;
        self.phase_err_filt = 0.0;
    }

    /// Set velocity estimate (for testing or handoff from other source)
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity_pll = velocity;
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
const HFI_FUND_TAU_S: f32 = 0.005;

/// Demodulation low-pass time constant (s) — a few carrier periods.
const HFI_DEMOD_TAU_S: f32 = 0.002;

/// Error-quality low-pass time constant (s), for confidence/readiness.
const HFI_QUALITY_TAU_S: f32 = 0.01;

/// Default d-channel carrier-amplitude floor (A): below this no injection
/// response is measurably flowing and the estimate is meaningless.
pub const HFI_MIN_HF_CURRENT_A: f32 = 0.05;

/// Confidence threshold for [`HfiObserver::is_ready`] and for starting
/// the polarity probe.
pub const HFI_READY_CONFIDENCE: f32 = 0.5;

/// Default HFI carrier frequency (Hz) when configuring from stored params.
pub const HFI_DEFAULT_FREQ_HZ: f32 = 1000.0;

/// Default HFI carrier amplitude as a fraction of vbus (3 V at 24 V — the
/// operating point validated in the closed-loop sims; bench-tune per motor).
pub const HFI_DEFAULT_AMPLITUDE_RATIO: f32 = 0.125;

/// Polarity probe: drive cycles per pulse. At 20 kHz this is 0.4 ms — with
/// the carrier amplitude on Ld in the 100 µH range the current reaches
/// ~V·t/Ld ≈ 10 A, enough to move the iron along its saturation curve.
const HFI_POLARITY_PULSE_CYCLES: u32 = 8;

/// Polarity probe: zero-voltage cycles after each pulse so the current
/// (τ = L/R, typically ~1 ms) decays before the opposite-sign pulse.
const HFI_POLARITY_GAP_CYCLES: u32 = 24;

/// Polarity probe pulse signs. The palindromic (+,−,−,+) order cancels the
/// first-order bias from residual current decaying across the schedule.
const HFI_POLARITY_PATTERN: [f32; 4] = [1.0, -1.0, -1.0, 1.0];

/// Significance floor for the flip decision: |Σ sign·|id|| must exceed this
/// fraction of Σ|id|. Below it there is no measurable saturation asymmetry
/// (SPM motor, probe too weak) and the current lock is kept as-is.
const HFI_POLARITY_MIN_RATIO: f32 = 0.05;

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
        assert!(err.abs() < 0.1, "phase error {} rad too large", err);
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
            "active-flux observer error {} rad (scalar comparison: {})",
            err_salient,
            err_scalar
        );
        assert!(
            err_salient < err_scalar * 0.25,
            "active flux should slash the load bias: scalar {} rad, salient {} rad",
            err_scalar,
            err_salient
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
            "lock lost under DC voltage offset: max err {} rad",
            max_err
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
            "phase error {} rad under load (L·I = λ) — missing −L·Δi term?",
            err
        );
    }

    /// |angle error| folded to the saliency period: HFI is 2θ-periodic, so
    /// without polarity detection θ and θ+π are equally valid lock points.
    #[cfg(feature = "virtual-motor")]
    fn angle_err_mod_pi(a: f32, b: f32) -> f32 {
        let err = angle_difference(a, b).abs();
        err.min(core::f32::consts::PI - err)
    }

    /// Closed-loop HFI harness: pulsating injection on the estimated d axis
    /// through the real FocController voltage path into a salient
    /// VirtualMotor. `sat_k` enables d-axis saturation (needed for the
    /// polarity tests). Returns (observer, final motor output).
    #[cfg(feature = "virtual-motor")]
    fn run_hfi_sim(
        rotor_angle: f32,
        load_torque: f32,
        sat_k: f32,
        steps: usize,
    ) -> (HfiObserver, crate::virtual_motor::VirtualMotorOutput) {
        use crate::foc::controller::FocController;
        use crate::foc::pwm::SvpwmModulator;
        use crate::foc::transforms;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
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
        };
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(rotor_angle);

        let foc = FocController::<SvpwmModulator>::new(24.0);
        let mut obs = HfiObserver::new(1000.0, 3.0);

        let mut out = crate::virtual_motor::VirtualMotorOutput::default();
        for _ in 0..steps {
            let (vd_inj, vq_inj) = obs.get_injection();
            let telem = foc.apply_dq(vd_inj, vq_inj, obs.phase(), 1000);
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
    #[cfg(feature = "virtual-motor")]
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
    #[cfg(feature = "virtual-motor")]
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
    #[cfg(feature = "virtual-motor")]
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
    #[cfg(feature = "virtual-motor")]
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
