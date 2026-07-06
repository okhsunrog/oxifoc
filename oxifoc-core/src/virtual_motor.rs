//! Dynamic PMSM motor simulation.
//!
//! Port of VESC's `virtual_motor.c` (Maximiliano Cordoba) rewritten in safe
//! Rust with corrected multi-pole-pair dq-frame equations.
//!
//! The model integrates the standard surface/interior PMSM equations in the
//! rotor-fixed dq frame using forward Euler at whatever `dt` you pass to
//! [`VirtualMotor::step`].  One call per FOC period is the typical usage:
//!
//! ```rust,ignore
//! use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor};
//! use oxifoc_core::foc::controller::FocController;
//!
//! let params = MotorParams::default();
//! let mut motor = VirtualMotor::new(params);
//! let mut foc   = FocController::new(24.0);
//! let mut out   = Default::default();
//!
//! loop {
//!     let dt = 1.0 / 20_000.0;
//!     let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad,
//!                          0.0, 5.0, 1000, dt);
//!     out = motor.step(telem.v_alpha, telem.v_beta, 0.0, dt);
//! }
//! ```

use crate::foc::clamp_f32;
use crate::foc::constants::FRAC_1_SQRT_3;
use crate::foc::transforms::inverse_clarke;

/// CW Hall raw-state sequence: sector index 0-5 → 3-bit raw Hall reading.
///
/// As the rotor advances clockwise through one full electrical revolution the
/// Hall comparators produce the sequence 1 → 3 → 2 → 6 → 4 → 5, which is
/// the standard 3-bit Gray code wiring used by most BLDC motors.
const HALL_CW_RAW: [u8; 6] = [1, 3, 2, 6, 4, 5];

/// PMSM motor parameters.
#[derive(Clone, Copy, Debug)]
pub struct MotorParams {
    /// Phase resistance (Ω)
    pub r: f32,
    /// D-axis inductance (H)
    pub ld: f32,
    /// Q-axis inductance (H)
    pub lq: f32,
    /// Permanent-magnet flux linkage (Wb)
    pub lambda: f32,
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Rotor + load inertia (N·m·s²)
    pub j: f32,
    /// Viscous friction coefficient (N·m·s/rad, mechanical).
    /// Creates a speed-proportional drag: T_friction = friction_b × ωm.
    /// A value of ~1e-4 is realistic for a small BLDC with bearing losses.
    pub friction_b: f32,
    /// Hall sensor mounting offset (electrical radians).
    ///
    /// Shifts all Hall transition edges by this amount relative to the
    /// back-EMF zero crossing.  Zero means ideal alignment; real motors
    /// typically have ±5–15° of mechanical tolerance (×pole_pairs electrical).
    pub hall_offset: f32,
    /// D-axis magnetic saturation coefficient (1/A). 0 = linear (default).
    ///
    /// Models the incremental-inductance asymmetry that HFI polarity
    /// detection relies on: positive id adds to the PM flux and saturates
    /// the iron (`Ld_eff` drops), negative id demagnetizes it (`Ld_eff`
    /// rises): `Ld_eff = Ld / (1 + sat_k·id)`, clamped to 0.25–4× Ld.
    /// Only the d-axis current dynamics use `Ld_eff`; torque and the
    /// q-axis equation keep the nominal Ld (the model stays minimal).
    pub sat_k: f32,
    /// Q-axis saturation coefficient (1/A). 0 = linear (default).
    ///
    /// Models saliency collapse under load — the classic HFI failure mode.
    /// Torque current saturates the q-axis iron regardless of sign:
    /// `Lq_eff = Lq / (1 + lq_sat_k·|iq|)`, clamped to 0.25–4× Lq. As
    /// `Lq_eff` approaches `Ld` the HFI error signal vanishes and the
    /// estimate stops being corrected (while the carrier amplitude — and
    /// therefore the demod confidence — stays healthy). Only the q-axis
    /// current dynamics use `Lq_eff`; torque and the d-axis cross-coupling
    /// keep the nominal Lq, mirroring the `sat_k` modeling choice.
    pub lq_sat_k: f32,
    /// Internal forward-Euler sub-steps per `step()` call (0 acts as 1).
    ///
    /// The default of 1 integrates the plant at exactly the caller's `dt`,
    /// which puts sim and estimators on a shared discretization grid — a
    /// detection error of 0.0% against that plant is partly
    /// self-confirmation. With `substeps = n` the plant integrates `n`
    /// Euler steps of `dt/n` under a zero-order-hold αβ voltage (the
    /// rotor moves *within* the FOC period, like real hardware), breaking
    /// the lockstep.
    pub substeps: u32,
    /// Dead-time voltage distortion magnitude per phase (V). 0 = ideal
    /// bridge (default). Physically `t_dt × f_pwm × vbus`.
    ///
    /// During dead time the body diode of the leg carrying positive
    /// current conducts to the negative rail, so that phase *loses* this
    /// much average terminal voltage (and gains it for negative current).
    /// Applied per sub-step from the instantaneous phase-current signs, so
    /// zero-crossing clamping and the 6th-harmonic dq ripple emerge
    /// naturally. `step_shorted` never applies it (no switching while all
    /// low-side FETs are held on).
    pub dead_time_v: f32,
    /// Current-sensor quantization step (A). 0 = continuous (default).
    ///
    /// Reported phase currents are rounded to this grid, e.g. a 12-bit ADC
    /// spanning ±31 A gives `62.0 / 4096.0 ≈ 15 mA`. Only the *measured*
    /// outputs are quantized; the internal plant state stays exact.
    pub adc_lsb_a: f32,
    /// Current-sensor noise amplitude (A). 0 = noiseless (default).
    ///
    /// Uniform noise in `±adc_noise_a` from a deterministic xorshift32
    /// stream is added to each reported phase current (before
    /// quantization). Deterministic seed → reproducible test runs.
    pub adc_noise_a: f32,
    /// Eddy-current inductance drop on the d axis (H). 0 = ideal iron
    /// (default).
    ///
    /// Real laminated/solid-magnet rotors lose apparent inductance with
    /// frequency: eddy currents in the iron/magnets act as a shorted
    /// secondary winding. Modeled as the classic first-order ladder —
    /// per-axis flux `ψ = L_hf·i + ΔL·i_f` with `τ_e·di_f/dt = i − i_f`,
    /// giving `L(jω) = L_hf + ΔL/(1 + jω·τ_e)`: the DC inductance is the
    /// configured `ld`/`lq`, the HF plateau is `ld − eddy_delta_l_d`, and
    /// the imaginary part contributes the frequency-dependent eddy LOSS
    /// (R(f) rise) for free. ZD2808 bench values: DC Ld/Lq 86/129 µH,
    /// AC plateau ~24 µH from ~1 kHz ⇒ ΔL_d ≈ 62 µH, ΔL_q ≈ 105 µH
    /// (docs/notes/inductance-freq-detection.md). This is the plant
    /// feature the 2026-07-06 sawtooth investigation identified as the
    /// sim-vs-bench fidelity gap: the single-L plant tracks cleanly in
    /// exactly the closed-loop conditions where the bench estimate runs
    /// away.
    pub eddy_delta_l_d: f32,
    /// Eddy-current inductance drop on the q axis (H). 0 = ideal (default).
    /// See [`eddy_delta_l_d`](Self::eddy_delta_l_d).
    pub eddy_delta_l_q: f32,
    /// Eddy branch time constant (s); the `L(f)` crossover sits at
    /// `1/(2π·τ_e)`. Only used when a ΔL is nonzero. ZD2808: plateau
    /// reached by ~1 kHz ⇒ τ_e ≈ 0.2–0.5 ms.
    pub eddy_tau_s: f32,
    /// Actuation pipeline delay in whole `step()` calls (0 = none,
    /// default; clamped to 3).
    ///
    /// Real hardware applies the voltage computed in ISR cycle N during
    /// PWM period N+1 (timer registers latch on the update event), so the
    /// rotor has moved `ωe·T_pwm` by the time the command acts — ~17°
    /// electrical for a Flipsky-class motor at full speed. The firmware
    /// compensates with `phase_advance_cycles` (foc_driver) and feeds
    /// estimators the previous cycle's voltage
    /// (`update_phase_with_prev_voltage`) — with `actuation_delay_steps =
    /// 1` both of those conventions become exactly right in sim instead
    /// of injecting the error they exist to cancel. `step_coast` /
    /// `step_shorted` push zeros through the pipeline (gates off / low).
    pub actuation_delay_steps: u8,
}

impl MotorParams {
    /// Incremental d-axis inductance at the given d current (see
    /// [`sat_k`](Self::sat_k)).
    fn ld_eff(&self, id: f32) -> f32 {
        let denom = clamp_f32(1.0 + self.sat_k * id, 0.25, 4.0);
        self.ld / denom
    }

    /// Incremental q-axis inductance at the given q current (see
    /// [`lq_sat_k`](Self::lq_sat_k)).
    fn lq_eff(&self, iq: f32) -> f32 {
        let denom = clamp_f32(1.0 + self.lq_sat_k * iq.abs(), 0.25, 4.0);
        self.lq / denom
    }
}

impl Default for MotorParams {
    /// Sensible defaults for a small hobby BLDC (e.g. 24 V, ~100 W).
    ///
    /// R = 0.5 Ω, L = 0.5 mH (surface PM: Ld = Lq), λ = 10 mWb,
    /// 7 pole pairs, J = 0.1 g·m², friction_b = 1e-4 N·m·s/rad,
    /// hall_offset = 0 rad (ideal sensor alignment).
    fn default() -> Self {
        Self {
            r: 0.5,
            ld: 5e-4,
            lq: 5e-4,
            lambda: 0.01,
            pole_pairs: 7,
            j: 1e-4,
            friction_b: 1e-4,
            hall_offset: 0.0,
            sat_k: 0.0,
            lq_sat_k: 0.0,
            substeps: 1,
            dead_time_v: 0.0,
            adc_lsb_a: 0.0,
            adc_noise_a: 0.0,
            eddy_delta_l_d: 0.0,
            eddy_delta_l_q: 0.0,
            eddy_tau_s: 0.0,
            actuation_delay_steps: 0,
        }
    }
}

/// Output produced by one call to [`VirtualMotor::step`].
#[derive(Clone, Copy, Debug, Default)]
pub struct VirtualMotorOutput {
    /// Phase currents (A)
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    /// Electrical rotor angle (rad, wrapped to −π … π)
    pub angle_rad: f32,
    /// Electrical angular velocity (rad/s)
    pub omega_e: f32,
    /// Electromagnetic torque (N·m)
    pub torque: f32,
    /// Simulated Hall sensor raw state (3-bit value, 1–6).
    ///
    /// Encoded as H3<<2 | H2<<1 | H1, matching the convention used by
    /// [`crate::foc::hall_sensor::HallSensor`].  The state advances through
    /// the CW sequence 1 → 3 → 2 → 6 → 4 → 5 as the rotor spins forward.
    pub hall_state: u8,
    /// Open-circuit back-EMF in α-β stator frame (V).
    ///
    /// `e_α = −ωe × λ × sin(φ)`, `e_β = ωe × λ × cos(φ)`.
    /// Only physically meaningful when no current flows (coast mode).
    pub bemf_alpha: f32,
    pub bemf_beta: f32,
}

/// Dynamic PMSM model using forward-Euler integration in the dq frame.
///
/// All quantities are in SI units.  The ideal-default model is minimal
/// (no iron losses, no temperature effects); the non-idealities that our
/// own firmware compensates for are available behind opt-in parameters
/// with ideal defaults — d/q saturation (`sat_k`, `lq_sat_k`), dead-time
/// distortion (`dead_time_v`), sensor quantization/noise (`adc_lsb_a`,
/// `adc_noise_a`) and sub-stepped integration (`substeps`).
pub struct VirtualMotor {
    params: MotorParams,
    // Integrator state
    id: f32,
    iq: f32,
    /// Eddy-branch filtered currents (see `MotorParams::eddy_delta_l_d`).
    i_fd: f32,
    i_fq: f32,
    omega_e: f32, // electrical angular velocity (rad/s)
    phi: f32,     // electrical rotor angle (rad)
    sin_phi: f32,
    cos_phi: f32,
    // xorshift32 state for current-sensor noise (deterministic seed so
    // every test run sees the identical noise stream)
    rng: u32,
    // Actuation pipeline ring buffer (αβ commands awaiting application);
    // sized for the max supported delay of 3 steps.
    v_ring: [(f32, f32); 4],
    v_ring_idx: usize,
}

impl VirtualMotor {
    /// Create a new virtual motor with the given parameters.
    pub fn new(params: MotorParams) -> Self {
        Self {
            params,
            id: 0.0,
            iq: 0.0,
            i_fd: 0.0,
            i_fq: 0.0,
            omega_e: 0.0,
            phi: 0.0,
            sin_phi: 0.0,
            cos_phi: 1.0,
            rng: 0x6F78_6601, // "oxf\x01"
            v_ring: [(0.0, 0.0); 4],
            v_ring_idx: 0,
        }
    }

    /// Pass an αβ command through the actuation pipeline: returns the
    /// command issued `actuation_delay_steps` calls ago (zeros until the
    /// pipeline fills). Delay 0 is a transparent pass-through.
    fn delayed_voltage(&mut self, v_alpha: f32, v_beta: f32) -> (f32, f32) {
        let delay = (self.params.actuation_delay_steps as usize).min(3);
        if delay == 0 {
            return (v_alpha, v_beta);
        }
        let out = self.v_ring[(self.v_ring_idx + 4 - delay) % 4];
        self.v_ring[self.v_ring_idx] = (v_alpha, v_beta);
        self.v_ring_idx = (self.v_ring_idx + 1) % 4;
        out
    }

    /// Next noise sample, uniform in [−1, 1) (xorshift32).
    fn next_noise(&mut self) -> f32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x as f32 / u32::MAX as f32) * 2.0 - 1.0
    }

    /// Pass a true phase current through the simulated sensor chain:
    /// uniform noise (`adc_noise_a`), then quantization (`adc_lsb_a`).
    fn measure(&mut self, i: f32) -> f32 {
        let p = self.params;
        let mut v = i;
        if p.adc_noise_a > 0.0 {
            v += p.adc_noise_a * self.next_noise();
        }
        if p.adc_lsb_a > 0.0 {
            v = libm::roundf(v / p.adc_lsb_a) * p.adc_lsb_a;
        }
        v
    }

    /// Override the initial rotor angle (electrical radians).
    pub fn set_angle(&mut self, phi_rad: f32) {
        self.phi = phi_rad;
        self.sin_phi = libm::sinf(phi_rad);
        self.cos_phi = libm::cosf(phi_rad);
    }

    /// Override the rotor electrical velocity (rad/s) — sim helper to set up a
    /// freewheeling rotor (e.g. flying-restart tests).
    pub fn set_velocity(&mut self, omega_e: f32) {
        self.omega_e = omega_e;
    }

    /// Compute the Hall sensor raw state from the current rotor angle.
    ///
    /// Sector `k` is CENTERED on `k·60° + hall_offset` (spans ±30° around
    /// it) — the same convention as `HallSensor`'s calibration table, whose
    /// entries are sector CENTROIDS (that is what `HallCalibrator`'s
    /// sin/cos averaging measures). With `hall_offset = 0` the default
    /// estimator table therefore matches this simulated motor exactly, and
    /// the hall edges fire on the true sector boundaries (`k·60° − 30°`).
    /// The previous convention (sector spanning `[k·60°, (k+1)·60°)`) put
    /// the simulated centroids 30° off the default table — invisible while
    /// the estimator anchored interpolation at the centroid (two errors
    /// canceled), exposed when it switched to boundary anchoring.
    fn hall_state(&self) -> u8 {
        use core::f32::consts::TAU;
        let phi_pos = if self.phi < 0.0 {
            self.phi + TAU
        } else {
            self.phi
        };
        let phi_hall = libm::fmodf(phi_pos - self.params.hall_offset + TAU + TAU / 12.0, TAU);
        let sector = ((phi_hall * 6.0 / TAU) as usize).min(5);
        HALL_CW_RAW[sector]
    }

    /// Run one simulation step with all FETs off (high-impedance).
    ///
    /// No current flows.  The only torque is the external load plus
    /// viscous friction.  The motor decelerates purely mechanically.
    pub fn step_coast(&mut self, load_torque: f32, dt: f32) -> VirtualMotorOutput {
        // Gates off: nothing is commanded this cycle, the pipeline moves on.
        let _ = self.delayed_voltage(0.0, 0.0);
        let p = self.params;
        let pp = f32::from(p.pole_pairs);
        let n = p.substeps.max(1);
        let dt_sub = dt / n as f32;

        // Zero current — no electromagnetic torque; the eddy-branch
        // currents decay with their own time constant.
        self.id = 0.0;
        self.iq = 0.0;
        if self.params.eddy_tau_s > 0.0 {
            let k = (dt / self.params.eddy_tau_s).min(1.0);
            self.i_fd -= k * self.i_fd;
            self.i_fq -= k * self.i_fq;
        } else {
            self.i_fd = 0.0;
            self.i_fq = 0.0;
        }

        for _ in 0..n {
            // Mechanical dynamics: only friction + external load
            let friction_torque = (p.friction_b / pp) * self.omega_e;
            self.omega_e -= dt_sub * pp / p.j * (load_torque + friction_torque);

            // Angle integration
            self.phi += self.omega_e * dt_sub;
            self.phi = libm::remainderf(self.phi, 2.0 * core::f32::consts::PI);
        }
        self.sin_phi = libm::sinf(self.phi);
        self.cos_phi = libm::cosf(self.phi);

        let bemf_alpha = -self.omega_e * p.lambda * self.sin_phi;
        let bemf_beta = self.omega_e * p.lambda * self.cos_phi;

        VirtualMotorOutput {
            ia: self.measure(0.0),
            ib: self.measure(0.0),
            ic: self.measure(0.0),
            angle_rad: self.phi,
            omega_e: self.omega_e,
            torque: 0.0,
            hall_state: self.hall_state(),
            bemf_alpha,
            bemf_beta,
        }
    }

    /// Run one simulation step with shorted terminals (V = 0).
    ///
    /// All low-side FETs on: the motor windings are short-circuited.
    /// Currents circulate creating strong braking torque.
    pub fn step_shorted(&mut self, load_torque: f32, dt: f32) -> VirtualMotorOutput {
        // All low-side FETs held on: no switching, hence no dead-time
        // distortion regardless of `dead_time_v`.
        self.step_inner(0.0, 0.0, load_torque, dt, false)
    }

    /// Run one simulation step with a voltage source connected.
    ///
    /// # Arguments
    /// * `v_alpha`      – α-axis voltage applied to the motor (V)
    /// * `v_beta`       – β-axis voltage applied to the motor (V)
    /// * `load_torque`  – externally applied load torque (N·m, positive = braking)
    /// * `dt`           – integration time step (s); use the FOC loop period
    pub fn step(
        &mut self,
        v_alpha: f32,
        v_beta: f32,
        load_torque: f32,
        dt: f32,
    ) -> VirtualMotorOutput {
        self.step_inner(v_alpha, v_beta, load_torque, dt, true)
    }

    fn step_inner(
        &mut self,
        v_alpha: f32,
        v_beta: f32,
        load_torque: f32,
        dt: f32,
        switching: bool,
    ) -> VirtualMotorOutput {
        let (v_alpha, v_beta) = self.delayed_voltage(v_alpha, v_beta);
        let p = self.params;
        let pp = f32::from(p.pole_pairs);
        let n = p.substeps.max(1);
        let dt_sub = dt / n as f32;
        let mut torque = 0.0;

        // The αβ voltage is a zero-order hold over the whole FOC period
        // (that is what the inverter applies); the rotor and currents keep
        // evolving inside it, so everything below is re-derived per sub-step.
        for _ in 0..n {
            // ── Dead-time distortion (instantaneous phase-current signs) ──
            let (va, vb) = if switching && p.dead_time_v != 0.0 {
                let i_alpha = self.cos_phi * self.id - self.sin_phi * self.iq;
                let i_beta = self.cos_phi * self.iq + self.sin_phi * self.id;
                let (ia, ib, ic) = inverse_clarke(i_alpha, i_beta);
                let sa = if ia >= 0.0 { 1.0f32 } else { -1.0 };
                let sb = if ib >= 0.0 { 1.0f32 } else { -1.0 };
                let sc = if ic >= 0.0 { 1.0f32 } else { -1.0 };
                // Per-phase −sign(i)·V_dt mapped through the (amplitude-
                // invariant) Clarke transform; the common mode drops out.
                let e_alpha = (2.0 * sa - sb - sc) * (p.dead_time_v / 3.0);
                let e_beta = (sb - sc) * FRAC_1_SQRT_3 * p.dead_time_v;
                (v_alpha - e_alpha, v_beta - e_beta)
            } else {
                (v_alpha, v_beta)
            };

            // ── Park transform: αβ → dq (using current rotor angle) ──────
            let vd = self.cos_phi * va + self.sin_phi * vb;
            let vq = self.cos_phi * vb - self.sin_phi * va;

            // ── Flux linkages (eddy ladder; ΔL = 0 reduces to ψ = L·i) ────
            // ψd = (Ld−ΔLd)·id + ΔLd·i_fd + λ, ψq analog. The nominal L
            // builds the fluxes (cross-coupling + torque, as before); the
            // saturation-effective L stays confined to the di/dt slope.
            let dl_d = p.eddy_delta_l_d;
            let dl_q = p.eddy_delta_l_q;
            let psi_d = (p.ld - dl_d) * self.id + dl_d * self.i_fd + p.lambda;
            let psi_q = (p.lq - dl_q) * self.iq + dl_q * self.i_fq;
            // Eddy branch EMF ΔL·d(i_f)/dt = ΔL·(i − i_f)/τ_e — the lossy
            // part of L(jω) (frequency-dependent eddy resistance).
            let (e_fd, e_fq) = if p.eddy_tau_s > 0.0 {
                (
                    dl_d * (self.id - self.i_fd) / p.eddy_tau_s,
                    dl_q * (self.iq - self.i_fq) / p.eddy_tau_s,
                )
            } else {
                (0.0, 0.0)
            };

            // ── D-axis current ────────────────────────────────────────────
            // (Ld_eff−ΔLd)·did/dt = Vd − R·id + ωe·ψq − ΔLd·(id−i_fd)/τe
            // Ld_eff(id) models d-axis saturation when sat_k ≠ 0 (HFI polarity).
            self.id += (vd + self.omega_e * psi_q - p.r * self.id - e_fd) * dt_sub
                / (p.ld_eff(self.id) - dl_d).max(1e-9);

            // ── Q-axis current ────────────────────────────────────────────
            // (Lq_eff−ΔLq)·diq/dt = Vq − R·iq − ωe·ψd − ΔLq·(iq−i_fq)/τe
            // Lq_eff(iq) models saliency collapse when lq_sat_k ≠ 0.
            self.iq += (vq - self.omega_e * psi_d - p.r * self.iq - e_fq) * dt_sub
                / (p.lq_eff(self.iq) - dl_q).max(1e-9);

            // ── Eddy branch state ─────────────────────────────────────────
            if p.eddy_tau_s > 0.0 {
                let k = (dt_sub / p.eddy_tau_s).min(1.0);
                self.i_fd += k * (self.id - self.i_fd);
                self.i_fq += k * (self.iq - self.i_fq);
            }

            // ── Electromagnetic torque (flux cross product) ───────────────
            // Te = (3/2)·p·(ψd·iq − ψq·id) — reduces to the familiar
            // (3/2)·p·[λPM + (Ld − Lq)·id]·iq at ΔL = 0.
            torque = 1.5 * pp * (psi_d * self.iq - psi_q * self.id);

            // ── Mechanical dynamics ───────────────────────────────────────
            // J·dωm/dt = Te − TL − B·ωm  →  dωe/dt = p·(Te − TL − B·ωm)/J
            // ωm = ωe/pp, so B·ωm = (friction_b/pp)·ωe
            let friction_torque = (p.friction_b / pp) * self.omega_e;
            self.omega_e += dt_sub * pp / p.j * (torque - load_torque - friction_torque);

            // ── Electrical angle integration ──────────────────────────────
            self.phi += self.omega_e * dt_sub;
            // Wrap to (−π, π] — use remainder for robustness at high speeds
            self.phi = libm::remainderf(self.phi, 2.0 * core::f32::consts::PI);

            // Update cached sin/cos
            self.sin_phi = libm::sinf(self.phi);
            self.cos_phi = libm::cosf(self.phi);
        }

        // ── Inverse Park + inverse Clarke → phase currents ────────────────────
        let i_alpha = self.cos_phi * self.id - self.sin_phi * self.iq;
        let i_beta = self.cos_phi * self.iq + self.sin_phi * self.id;
        let (ia, ib, ic) = inverse_clarke(i_alpha, i_beta);

        let hall_state = self.hall_state();

        // Open-circuit back-EMF: e = ωe × λ × [−sin(φ), cos(φ)]
        let bemf_alpha = -self.omega_e * p.lambda * self.sin_phi;
        let bemf_beta = self.omega_e * p.lambda * self.cos_phi;

        VirtualMotorOutput {
            ia: self.measure(ia),
            ib: self.measure(ib),
            ic: self.measure(ic),
            angle_rad: self.phi,
            omega_e: self.omega_e,
            torque,
            hall_state,
            bemf_alpha,
            bemf_beta,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::controller::FocController;
    use crate::foc::hall_calibration::HallCalibrator;
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::pi_controller::PIController;
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::wrap_angle;

    #[test]
    fn closed_loop_accelerates() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let kp = params.ld * 1000.0;
        let ki = params.r * 1000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Run 500 steps = 25 ms.  Back-EMF is still small, so the FOC can
        // inject close to the target 2 A and the motor accelerates.
        for _ in 0..500 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        // Motor should be spinning and producing torque.
        assert!(
            out.omega_e > 10.0,
            "motor should spin up: ωe={}",
            out.omega_e
        );
        assert!(out.torque > 0.0, "torque should be positive");
        let i_mag = libm::sqrtf(out.ia * out.ia + out.ib * out.ib + out.ic * out.ic);
        assert!(
            i_mag > 0.3,
            "motor should carry current at low speed: |i|={i_mag}"
        );

        // Continue for a full second.  Viscous friction limits the terminal
        // speed, but ωe should still be well above zero.
        for _ in 0..20_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }
        assert!(
            out.omega_e > 100.0,
            "motor should reach steady-state speed: ωe={}",
            out.omega_e
        );
    }

    /// Negative iq_target must produce reverse rotation.
    ///
    /// Verifies sign conventions are consistent through the entire chain:
    /// FOC controller → inverse Park/Clarke → virtual motor → negative ωe.
    /// Direction bugs are notoriously subtle in FOC code.
    #[test]
    fn closed_loop_reverse_direction() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let kp = params.ld * 1000.0;
        let ki = params.r * 1000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Run 500 steps with negative iq_target
        for _ in 0..500 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, -2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        // Motor should spin in reverse
        assert!(
            out.omega_e < -10.0,
            "motor should spin in reverse: ωe={}",
            out.omega_e
        );
        assert!(out.torque < 0.0, "torque should be negative");

        // Continue for a full second to reach steady state
        for _ in 0..20_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, -2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        assert!(
            out.omega_e < -100.0,
            "motor should reach steady-state reverse speed: ωe={}",
            out.omega_e
        );
    }

    /// Apply a load torque step mid-run and verify the controller doesn't
    /// diverge. Speed should drop under load but remain positive, and iq
    /// should increase to compensate.
    #[test]
    fn load_torque_step_rejection() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let kp = params.ld * 1000.0;
        let ki = params.r * 1000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Phase 1: spin up to steady state with no load (1 second)
        for _ in 0..20_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }
        let speed_no_load = out.omega_e;
        assert!(
            speed_no_load > 100.0,
            "should reach steady state before load: ωe={speed_no_load}"
        );

        // Phase 2: apply load torque step, settle for 1 second
        let load = 0.005; // N·m — significant but within controller capability
        for _ in 0..20_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, load, DT);
        }
        let speed_loaded = out.omega_e;

        // Speed should drop but motor must keep spinning forward
        assert!(
            speed_loaded > 0.0,
            "motor should not stall under load: ωe={speed_loaded}"
        );
        assert!(
            speed_loaded < speed_no_load,
            "speed should decrease under load: {speed_loaded} vs {speed_no_load}"
        );

        // Phase 3: remove load, verify recovery (1 second)
        for _ in 0..20_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }
        let speed_recovered = out.omega_e;

        // Should recover close to original no-load speed
        assert!(
            (speed_recovered - speed_no_load).abs() / speed_no_load < 0.05,
            "speed should recover after load removal: {speed_recovered} vs {speed_no_load}"
        );
    }

    /// Realistic Hall sensor closed-loop test: calibrate first, then drive.
    ///
    /// Uses a 30° electrical mounting offset (`hall_offset = π/6`) to mimic
    /// real-world sensor misalignment, then:
    ///   1. Runs an open-loop calibration sweep to find the true sector angles.
    ///   2. Applies the calibration to `HallSensor`.
    ///   3. Runs closed-loop FOC using only the Hall-estimated angle.
    ///
    /// Without step 1 the angle estimate would be systematically wrong by the
    /// offset, reducing effective torque and degrading steady-state speed.
    #[test]
    fn closed_loop_hall_sensor_with_calibration() {
        use core::f32::consts::{PI, TAU};
        const DT: f32 = 1.0 / 20_000.0;
        const TICKS_PER_SEC: u64 = 20_000;

        // 30° electrical offset — plausible real-world sensor misalignment.
        // For a 7-pole-pair motor this equals ~4.3° mechanical.
        let params = MotorParams {
            hall_offset: PI / 6.0,
            ..MotorParams::default()
        };

        // ── Step 1: calibration sweep ─────────────────────────────────────────
        // Simulate the open-loop rotor lock-and-sweep that real firmware does
        // with `calibrate_hall()`.  We force the motor to a sequence of known
        // electrical angles and record which Hall state is active at each one.
        let mut cal_motor = VirtualMotor::new(params);
        let mut calibrator = HallCalibrator::with_min_samples(20);

        // Two full electrical revolutions, 720 evenly-spaced angles.
        // Each of the 6 Hall sectors will see ~240 samples — well above the
        // min_samples threshold.
        let n = 720usize;
        for i in 0..n {
            let phi_e = (i as f32 / n as f32) * TAU - PI; // (−π, π]
            cal_motor.set_angle(phi_e);
            // Zero-voltage, tiny dt: we only need the Hall state output.
            let out = cal_motor.step(0.0, 0.0, 0.0, 1e-9);
            // `HallCalibrator::record` expects angle in [0, 2π).
            let phi_cal = if phi_e < 0.0 { phi_e + TAU } else { phi_e };
            calibrator.record(phi_cal, out.hall_state);
        }

        let cal_result = calibrator.finish().expect("calibration must succeed");
        assert!(
            cal_result.is_valid(),
            "calibration result must cover all 6 states"
        );

        // Sanity-check: sector centroids sit at k·60° + offset, so the
        // calibrated angle for raw state 1 (sector 0) should be ≈ π/6.
        let angle_raw1 = cal_result.angle_for_raw_state(1).unwrap();
        assert!(
            (angle_raw1 - PI / 6.0).abs() < 0.1,
            "calibrated angle for state 1 should be ≈π/6 with offset=π/6, got {angle_raw1:.4}"
        );

        // ── Step 2: closed-loop FOC with calibrated Hall sensor ───────────────
        let kp = params.ld * 1_000.0;
        let ki = params.r * 1_000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);

        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        let mut hall = HallSensor::new(TICKS_PER_SEC);
        hall.apply_calibration(&cal_result); // <-- uses measured sector angles

        let mut prev_hall_state = 0u8;
        let mut hall_angle = 0.0_f32;

        for tick in 0u64..20_500 {
            let hs = out.hall_state;
            if hs != prev_hall_state && hs != 0 {
                if let Some(angle) = hall.update(hs, tick) {
                    hall_angle = angle;
                }
                prev_hall_state = hs;
            }
            if let Some(sample) = hall.sample_at_mut(tick) {
                hall_angle = sample.angle;
            }

            let telem = foc.step((out.ia, out.ib, out.ic), hall_angle, 0.0, 2.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        assert!(
            out.omega_e > 100.0,
            "motor should reach speed with calibrated Hall sensor (offset=π/6): ωe={}",
            out.omega_e
        );
        assert!(
            out.torque > 0.0,
            "torque should be positive: T={}",
            out.torque
        );
    }

    /// Dead-time compensation must cancel the plant's dead-time distortion.
    ///
    /// The compensation lives in the duty domain, so the plant is driven
    /// from the *reconstructed duty voltages* (what the inverter applies),
    /// not the pre-modulation command — the same path the detection harness
    /// uses. Without the plant distortion this test could not exist: the
    /// compensation used to run against a plant that never produced the
    /// disturbance it cancels (docs/notes/virtual-motor-fidelity.md).
    #[test]
    fn dead_time_compensation_cancels_plant_distortion() {
        const DT: f32 = 1.0 / 20_000.0;
        const VBUS: f32 = 24.0;
        const MAX_DUTY: u16 = 4250;
        const DEAD_TIME_NS: u32 = 800; // g431 PWM config

        let run = |comp: bool| -> f32 {
            let params = MotorParams {
                dead_time_v: DEAD_TIME_NS as f32 * 1e-9 * 20_000.0 * VBUS, // 0.384 V
                substeps: 10,
                // Settle near ωe ≈ 735 rad/s — fast enough that the
                // 6th-harmonic distortion (~700 Hz) sits above the current
                // loop bandwidth, far below bus-voltage saturation.
                friction_b: 2e-3,
                ..MotorParams::default()
            };
            let kp = params.ld * 1000.0;
            let ki = params.r * 1000.0;
            let mut foc = FocController::<SvpwmModulator>::new(VBUS);
            foc.id_pi = PIController::new(kp, ki);
            foc.iq_pi = PIController::new(kp, ki);
            if comp {
                foc.set_dead_time_comp(DEAD_TIME_NS, 20_000);
            }
            let mut motor = VirtualMotor::new(params);
            let mut out = VirtualMotorOutput::default();

            // Spin up 0.5 s, then accumulate the dq current tracking error
            // over 1 s: the uncompensated distortion appears as a
            // 6th-harmonic dq ripple the PI can only partially reject.
            let mut err_sq = 0.0f32;
            let mut n = 0u32;
            for step in 0..30_000 {
                let telem = foc.step(
                    (out.ia, out.ib, out.ic),
                    out.angle_rad,
                    0.0,
                    2.0,
                    MAX_DUTY,
                    DT,
                );
                let scale = VBUS / f32::from(MAX_DUTY);
                let va = f32::from(telem.duties[0]) * scale;
                let vb = f32::from(telem.duties[1]) * scale;
                let vc = f32::from(telem.duties[2]) * scale;
                let v_alpha = (2.0 * va - vb - vc) / 3.0;
                let v_beta = (vb - vc) * FRAC_1_SQRT_3;
                out = motor.step(v_alpha, v_beta, 0.0, DT);
                if step >= 10_000 {
                    let ed = telem.id;
                    let eq = telem.iq - 2.0;
                    err_sq += ed * ed + eq * eq;
                    n += 1;
                }
            }
            libm::sqrtf(err_sq / n as f32)
        };

        let uncompensated = run(false);
        let compensated = run(true);
        assert!(
            uncompensated > 0.05,
            "plant must produce real dead-time current distortion: RMS = {uncompensated}"
        );
        assert!(
            compensated < uncompensated * 0.5,
            "compensation must cancel most of the distortion: {compensated} vs {uncompensated}"
        );
    }

    /// Pipeline-delay compensation must rotate the ACTUATION frame only.
    ///
    /// With `actuation_delay_steps = 1` the plant applies each command one
    /// FOC period late. The firmware compensates via
    /// `set_actuation_advance` (output-vector rotation); the original code
    /// instead advanced the single commutation angle, which also advanced
    /// the measurement Park — the PI then regulated the current vector
    /// `δ = ωe·dt` off the true q axis (`id_true = −iq·sin δ`, ~29% of iq
    /// for a Flipsky-class motor at full speed). The steady-state benefit
    /// of the actuation-side advance itself is absorbed by the PI and is
    /// not directly observable here; what this test pins is the frame
    /// split — compensation must not displace the regulated current.
    #[test]
    fn actuation_advance_must_not_displace_current_vector() {
        use crate::foc::transforms;
        const DT: f32 = 1.0 / 20_000.0;
        const VBUS: f32 = 24.0;

        // true-frame mean d-current after settling
        let run = |advance_measurement_frame: bool| -> f32 {
            let params = MotorParams {
                actuation_delay_steps: 1,
                substeps: 10,
                friction_b: 1.2e-3, // settle ≈ 1200 rad/s el (δ ≈ 0.06 rad)
                ..MotorParams::default()
            };
            let kp = params.ld * 1000.0;
            let ki = params.r * 1000.0;
            let mut foc = FocController::<SvpwmModulator>::new(VBUS);
            foc.id_pi = PIController::new(kp, ki);
            foc.iq_pi = PIController::new(kp, ki);
            let mut motor = VirtualMotor::new(params);
            let mut out = VirtualMotorOutput::default();
            let mut id_sum = 0.0f32;
            let mut n = 0u32;
            for step in 0..30_000 {
                // Perfect estimator: true rotor state stands in for the
                // phase provider.
                let delta = out.omega_e * DT;
                let (angle_cmd, advance) = if advance_measurement_frame {
                    // the old firmware behavior: one advanced angle for both
                    (wrap_angle(out.angle_rad + delta), 0.0)
                } else {
                    // the fixed path: raw Park + actuation-side rotation
                    (out.angle_rad, delta)
                };
                foc.set_actuation_advance(advance);
                let telem = foc.step((out.ia, out.ib, out.ic), angle_cmd, 0.0, 2.0, 4250, DT);
                out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
                if step >= 20_000 {
                    let (i_alpha, i_beta) = transforms::clarke(out.ia, out.ib);
                    let (s, c) = (libm::sinf(out.angle_rad), libm::cosf(out.angle_rad));
                    let (id, _) = transforms::park(i_alpha, i_beta, s, c);
                    id_sum += id;
                    n += 1;
                }
            }
            id_sum / n as f32
        };

        let id_split = run(false).abs();
        let id_both_frames = run(true).abs();
        assert!(
            id_both_frames > 0.08,
            "advancing the measurement frame must displace the current vector: |id| = {id_both_frames}"
        );
        assert!(
            id_split < 0.03,
            "actuation-only advance must keep the true d-current clean: |id| = {id_split}"
        );
    }

    /// Check that the Hall state cycles through the correct CW Gray-code
    /// sequence as the rotor advances one full electrical revolution.
    #[test]
    fn hall_state_follows_cw_sequence() {
        use core::f32::consts::TAU;
        let params = MotorParams::default();
        let mut motor = VirtualMotor::new(params);

        // Advance the rotor by tiny steps and collect Hall states.
        let n = 600usize;
        let mut states = Vec::new();
        let mut prev = 0u8;
        for i in 0..=n {
            let phi = TAU * i as f32 / n as f32 - core::f32::consts::PI;
            motor.set_angle(phi);
            // One zero-voltage step to populate the output.
            let out = motor.step(0.0, 0.0, 0.0, 1e-6);
            if out.hall_state != prev {
                states.push(out.hall_state);
                prev = out.hall_state;
            }
        }

        // Drop the last entry if it duplicates the first (wrap-around artifact).
        if states.len() > 1 && states.last() == states.first() {
            states.pop();
        }

        // Exactly 6 distinct transitions per electrical revolution.
        assert_eq!(
            states.len(),
            6,
            "expected 6 Hall transitions, got {states:?}"
        );

        // The sequence is the CW Gray-code [1,3,2,6,4,5] (or any cyclic
        // rotation, depending on where in the revolution the sweep starts).
        const CW: [u8; 6] = [1, 3, 2, 6, 4, 5];
        let is_rotation = (0..6).any(|offset| (0..6).all(|i| states[i] == CW[(i + offset) % 6]));
        assert!(
            is_rotation,
            "sequence {states:?} is not a CW rotation of {CW:?}"
        );
    }

    /// The eddy ladder must produce a frequency-dependent inductance:
    /// at low frequency the plant presents the DC `ld`, at high frequency
    /// the `ld − ΔL` plateau (larger current for the same voltage). This
    /// test FAILS on the ΔL = 0 plant (the sim-vs-bench fidelity gap of
    /// the 2026-07-06 sawtooth investigation): without the eddy branch the
    /// HF amplitude matches the DC-L prediction instead of exceeding it.
    #[test]
    fn eddy_branch_drops_inductance_with_frequency() {
        use core::f32::consts::TAU;
        let params = MotorParams {
            r: 0.127,
            ld: 86e-6,
            lq: 129e-6,
            lambda: 1.145e-3,
            pole_pairs: 7,
            j: 5e-5,
            friction_b: 4e-6,
            substeps: 10,
            eddy_delta_l_d: 62e-6,
            eddy_delta_l_q: 105e-6,
            eddy_tau_s: 0.2e-3,
            ..MotorParams::default()
        };
        let dt = 1.0 / 20_000.0;
        let amp_at = |p: MotorParams, f_hz: f32| -> f32 {
            // Rotor parked at φ = 0: v_alpha maps to vd, ia = id, iq stays
            // zero → zero torque, the rotor never moves.
            let mut m = VirtualMotor::new(p);
            let steps = (0.3 / dt) as usize;
            let mut amp = 0.0f32;
            for k in 0..steps {
                let v = 0.5 * libm::sinf(TAU * f_hz * k as f32 * dt);
                let out = m.step(v, 0.0, 0.0, dt);
                if k > steps * 8 / 10 {
                    amp = amp.max(out.ia.abs());
                }
            }
            amp
        };
        // DC-L predictions |i| = V/|R + jωL_dc|.
        let pred_dc = |f_hz: f32| {
            let w = TAU * f_hz;
            0.5 / libm::sqrtf(0.127f32 * 0.127 + (w * 86e-6) * (w * 86e-6))
        };
        // Low frequency: the eddy branch is invisible, DC L holds.
        let lo = amp_at(params, 50.0);
        assert!(
            (lo - pred_dc(50.0)).abs() < 0.15 * pred_dc(50.0),
            "50 Hz amplitude {lo} vs DC-L prediction {}",
            pred_dc(50.0)
        );
        // High frequency: the apparent L collapses toward the plateau —
        // materially MORE current than the DC L would pass.
        let hi = amp_at(params, 2_000.0);
        assert!(
            hi > 1.5 * pred_dc(2_000.0),
            "2 kHz amplitude {hi} must exceed the DC-L prediction {} — \
             eddy branch inactive?",
            pred_dc(2_000.0)
        );
        // And the ideal (ΔL = 0) plant matches the DC prediction at HF —
        // the exact behavior that hid the bench sawtooth from the sim.
        let ideal = MotorParams {
            eddy_delta_l_d: 0.0,
            eddy_delta_l_q: 0.0,
            eddy_tau_s: 0.0,
            ..params
        };
        let hi_ideal = amp_at(ideal, 2_000.0);
        assert!(
            (hi_ideal - pred_dc(2_000.0)).abs() < 0.2 * pred_dc(2_000.0),
            "ideal plant at 2 kHz {hi_ideal} vs {}",
            pred_dc(2_000.0)
        );
    }
}
