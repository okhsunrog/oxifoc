//! Sensorless observers for FOC control
//!
//! Provides software-based angle estimation for sensorless motor control.
//! Includes back-EMF observer for medium/high speed and HFI for low/zero speed.

use core::f32::consts::TAU;

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
#[derive(Clone, Debug)]
pub enum Observer {
    /// No observer configured
    None,
    /// Back-EMF flux observer (VESC-style)
    BackEmf(BackEmfObserver),
    /// High-frequency injection observer
    Hfi(HfiObserver),
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
            Observer::Hfi(o) => o.update(input),
        }
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.phase()),
            Observer::Hfi(o) => Some(o.phase()),
        }
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.velocity()),
            Observer::Hfi(o) => Some(o.velocity()),
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
            Observer::Hfi(o) => o.is_ready(),
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
            Observer::Hfi(o) => {
                o.set_phase(angle);
                o.set_velocity(velocity);
            }
        }
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        match self {
            Observer::None => 0.0,
            Observer::BackEmf(o) => o.confidence(),
            Observer::Hfi(o) => o.confidence(),
        }
    }

    /// Carrier voltage to inject this cycle (dq frame, at the estimated
    /// angle). Zero for observers that don't inject — only HFI needs the
    /// control loop's cooperation. See [`HfiObserver::get_injection`].
    pub fn injection(&self) -> (f32, f32) {
        match self {
            Observer::Hfi(o) => o.get_injection(),
            Observer::None | Observer::BackEmf(_) => (0.0, 0.0),
        }
    }

    /// Whether this observer is an HFI estimator (needs carrier injection
    /// from the control loop to function at all).
    pub fn is_hfi(&self) -> bool {
        matches!(self, Observer::Hfi(_))
    }

    /// Check if observer is configured
    pub fn is_configured(&self) -> bool {
        !matches!(self, Observer::None)
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => o.reset(),
            Observer::Hfi(o) => o.reset(),
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
/// vector directly, truncates each component to ±λ to bleed off integrator
/// drift, then uses a PLL to extract phase and velocity.
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
    r: f32,      // Phase resistance (Ω)
    l: f32,      // Phase inductance (H)
    lambda: f32, // Flux linkage (Wb)

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

impl BackEmfObserver {
    /// Create a new back-EMF observer with motor parameters
    ///
    /// # Arguments
    /// * `r` - Phase resistance (Ω)
    /// * `l` - Phase inductance (H)
    /// * `lambda` - Flux linkage (Wb)
    pub fn new(r: f32, l: f32, lambda: f32) -> Self {
        Self {
            x1: 0.0,
            x2: 0.0,
            i_alpha_last: 0.0,
            i_beta_last: 0.0,
            phase_pll: 0.0,
            velocity_pll: 0.0,
            r,
            l,
            lambda: lambda.max(1e-6),
            pll_kp: 1000.0,
            pll_ki: 20000.0,
            confidence: 0.0,
            // Start "unlocked": a fresh observer must not look ready until
            // the PLL has actually tracked something.
            phase_err_filt: core::f32::consts::PI,
        }
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
        self.x1 += (input.v_alpha - self.r * input.i_alpha) * dt
            - self.l * (input.i_alpha - self.i_alpha_last);
        self.x2 += (input.v_beta - self.r * input.i_beta) * dt
            - self.l * (input.i_beta - self.i_beta_last);
        self.i_alpha_last = input.i_alpha;
        self.i_beta_last = input.i_beta;

        // Component-wise truncation to ±λ is the MXLEMMING correction
        // mechanism: instead of an explicit gain·error feedback term it bleeds
        // off integrator drift (DC offsets in v/i measurements) every cycle,
        // because the true rotor flux components never exceed λ.
        self.x1 = crate::foc::clamp_f32(self.x1, -self.lambda, self.lambda);
        self.x2 = crate::foc::clamp_f32(self.x2, -self.lambda, self.lambda);

        // Extract phase from the rotor flux vector
        let phase_raw = libm::atan2f(self.x2, self.x1);

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

        // Confidence: how close the estimated flux magnitude is to λ.
        // A weak heuristic — measurement offsets can also saturate the
        // integrator — but cheap and monotonic during real spin-up.
        let flux_mag = libm::sqrtf(self.x1 * self.x1 + self.x2 * self.x2);
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

    /// Reset observer state
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

    /// Set motor parameters
    pub fn set_motor_params(&mut self, r: f32, l: f32, lambda: f32) {
        self.r = r;
        self.l = l;
        self.lambda = lambda.max(1e-6);
    }

    /// Set PLL gains
    pub fn set_pll_gains(&mut self, kp: f32, ki: f32) {
        self.pll_kp = kp;
        self.pll_ki = ki;
    }

    /// Force phase to specific value (for testing or handoff from other source)
    pub fn force_phase(&mut self, phase: f32) {
        self.phase_pll = wrap_angle(phase);
        // Also set flux state to match
        self.x1 = self.lambda * libm::cosf(phase);
        self.x2 = self.lambda * libm::sinf(phase);
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
#[derive(Clone, Debug)]
pub struct HfiObserver {
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

impl HfiObserver {
    /// Create a new HFI observer
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
    /// 2θ-periodic, so the lock point carries a π ambiguity — polarity
    /// detection (saturation pulse) is not implemented yet.
    pub fn update(&mut self, input: &ObserverInput) {
        let dt = input.dt;
        if dt <= 0.0 {
            return;
        }

        // Measured currents into the estimated rotor frame.
        let (s_est, c_est) = (libm::sinf(self.phase_est), libm::cosf(self.phase_est));
        let (id, iq) = crate::foc::transforms::park(input.i_alpha, input.i_beta, s_est, c_est);

        // Split carrier content from the fundamental.
        let a_f = (dt / HFI_FUND_TAU_S).min(1.0);
        self.id_lp += a_f * (id - self.id_lp);
        self.iq_lp += a_f * (iq - self.iq_lp);
        let hf_id = id - self.id_lp;
        let hf_iq = iq - self.iq_lp;

        // Synchronous demodulation with the carrier sample that generated
        // this response (see the call contract above).
        let dem = libm::sinf(self.carrier_phase);
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
    /// flowing and the demodulated error settled near zero.
    pub fn is_ready(&self) -> bool {
        self.confidence >= 0.5
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
    }

    /// Get the injection voltage for the current FOC cycle.
    ///
    /// Returns `(vd_inject, vq_inject)` to apply in the estimated rotor
    /// frame (at [`phase`](Self::phase)) — e.g. via
    /// `FocController::step_with_injection` or `apply_dq`. Must be called
    /// before [`update`](Self::update) each cycle; see the contract there.
    pub fn get_injection(&self) -> (f32, f32) {
        let vd = self.amplitude * libm::cosf(self.carrier_phase);
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

    /// Set initial phase estimate (for handoff from other source)
    pub fn set_phase(&mut self, phase: f32) {
        self.phase_est = wrap_angle(phase);
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
    /// VirtualMotor. Returns (observer, final motor output).
    #[cfg(feature = "virtual-motor")]
    fn run_hfi_sim(
        rotor_angle: f32,
        load_torque: f32,
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
        let (obs, out) = run_hfi_sim(1.2, 0.0, 20_000);

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
        let (obs, out) = run_hfi_sim(0.3, 0.5, 30_000);

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
