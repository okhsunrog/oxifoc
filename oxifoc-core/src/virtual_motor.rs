//! Dynamic PMSM motor simulation.
//!
//! Written from the standard PMSM equations in the rotor-fixed dq frame, with
//! multi-pole-pair handling. VESC's `virtual_motor.c` served as a reference
//! for comparison; see `docs/notes/virtual-motor-fidelity.md`.
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
//! let mut foc   = FocController::new(volts(24.0));
//! let mut out   = Default::default();
//!
//! loop {
//!     let dt = seconds(1.0 / 20_000.0);
//!     let telem = foc.step((out.ia, out.ib, out.ic), out.angle,
//!                          amps(0.0), amps(5.0), 1000, dt);
//!     out = motor.step(telem.v_alpha, telem.v_beta, 0.0, dt);
//! }
//! ```
//!
//! The electrical side (currents, voltages, fluxes, angle and speed) is
//! typed. The mechanical side — load torque, inertia and viscous friction —
//! stays in plain SI `f32` (N·m, kg·m², N·m·s/rad): torque is an angle-kind
//! quantity in uom, and nothing outside the plant consumes it.

pub mod io;

use core::ops::Div;

use crate::foc::clamp_f32;
use crate::foc::constants::FRAC_1_SQRT_3;
use crate::foc::transforms::inverse_clarke;
use crate::units::{
    AngleRepr, ConstZero, ElectricCurrent, ElectricPotential, ElectricalAngle,
    ElectricalAngularVelocity, ElectricalResistance, Fraction, Inductance, Linear, MagneticFlux,
    Ratio, Sign, Time, erad, erad_per_s, henries, ohms, webers,
};

/// Saturation coefficient: the relative inductance change per ampere.
pub type PerAmp = <Ratio as Div<ElectricCurrent>>::Output;

/// CW Hall raw-state sequence: sector index 0-5 → 3-bit raw Hall reading.
///
/// As the rotor advances clockwise through one full electrical revolution the
/// Hall comparators produce the sequence 1 → 3 → 2 → 6 → 4 → 5, which is
/// the standard 3-bit Gray code wiring used by most BLDC motors.
const HALL_CW_RAW: [u8; 6] = [1, 3, 2, 6, 4, 5];

/// PMSM motor parameters.
#[derive(Clone, Copy, Debug)]
pub struct MotorParams {
    /// Phase resistance
    pub r: ElectricalResistance,
    /// D-axis inductance
    pub ld: Inductance,
    /// Q-axis inductance
    pub lq: Inductance,
    /// Permanent-magnet flux linkage
    pub lambda: MagneticFlux,
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Rotor + load inertia (N·m·s²)
    pub j: f32,
    /// Viscous friction coefficient (N·m·s/rad, mechanical).
    /// Creates a speed-proportional drag: T_friction = friction_b × ωm.
    /// A value of ~1e-4 is realistic for a small BLDC with bearing losses.
    pub friction_b: f32,
    /// Hall sensor mounting offset.
    ///
    /// Shifts all Hall transition edges by this amount relative to the
    /// back-EMF zero crossing.  Zero means ideal alignment; real motors
    /// typically have ±5–15° of mechanical tolerance (×pole_pairs electrical).
    pub hall_offset: ElectricalAngle,
    /// D-axis magnetic saturation coefficient. 0 = linear (default).
    ///
    /// Models the incremental-inductance asymmetry that HFI polarity
    /// detection relies on: positive id adds to the PM flux and saturates
    /// the iron (`Ld_eff` drops), negative id demagnetizes it (`Ld_eff`
    /// rises): `Ld_eff = Ld / (1 + sat_k·id)`, clamped to 0.25–4× Ld.
    /// Only the d-axis current dynamics use `Ld_eff`; torque and the
    /// q-axis equation keep the nominal Ld (the model stays minimal).
    pub sat_k: PerAmp,
    /// Q-axis saturation coefficient. 0 = linear (default).
    ///
    /// Models saliency collapse under load — the classic HFI failure mode.
    /// Torque current saturates the q-axis iron regardless of sign:
    /// `Lq_eff = Lq / (1 + lq_sat_k·|iq|)`, clamped to 0.25–4× Lq. As
    /// `Lq_eff` approaches `Ld` the HFI error signal vanishes and the
    /// estimate stops being corrected (while the carrier amplitude — and
    /// therefore the demod confidence — stays healthy). Only the q-axis
    /// current dynamics use `Lq_eff`; torque and the d-axis cross-coupling
    /// keep the nominal Lq, mirroring the `sat_k` modeling choice.
    pub lq_sat_k: PerAmp,
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
    /// Dead-time voltage distortion magnitude per phase. 0 = ideal
    /// bridge (default). Physically `t_dt × f_pwm × vbus`.
    ///
    /// During dead time the body diode of the leg carrying positive
    /// current conducts to the negative rail, so that phase *loses* this
    /// much average terminal voltage (and gains it for negative current).
    /// Applied per sub-step from the instantaneous phase-current signs, so
    /// zero-crossing clamping and the 6th-harmonic dq ripple emerge
    /// naturally. `step_shorted` never applies it (no switching while all
    /// low-side FETs are held on).
    pub dead_time_voltage: ElectricPotential,
    /// Current-sensor quantization step. 0 = continuous (default).
    ///
    /// Reported phase currents are rounded to this grid, e.g. a 12-bit ADC
    /// spanning ±31 A gives `62.0 / 4096.0 ≈ 15 mA`. Only the *measured*
    /// outputs are quantized; the internal plant state stays exact.
    pub adc_lsb: ElectricCurrent,
    /// Current-sensor noise amplitude. 0 = noiseless (default).
    ///
    /// Uniform noise in `±adc_noise` from a deterministic xorshift32
    /// stream is added to each reported phase current (before
    /// quantization). Deterministic seed → reproducible test runs.
    pub adc_noise: ElectricCurrent,
    /// Eddy-current inductance drop on the d axis. 0 = ideal iron
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
    pub eddy_delta_l_d: Inductance,
    /// Eddy-current inductance drop on the q axis. 0 = ideal (default).
    /// See [`eddy_delta_l_d`](Self::eddy_delta_l_d).
    pub eddy_delta_l_q: Inductance,
    /// Eddy branch time constant; the `L(f)` crossover sits at
    /// `1/(2π·τ_e)`. Only used when a ΔL is nonzero. ZD2808: plateau
    /// reached by ~1 kHz ⇒ τ_e ≈ 0.2–0.5 ms.
    pub eddy_tau: Time,
    /// Actuation pipeline delay in `step()` periods (0 = none, default;
    /// clamped to 0..=3, fractions allowed).
    ///
    /// The command passed to call N starts acting `actuation_delay` periods
    /// after that call's start and holds until the next command takes
    /// over. Real hardware cannot act on a command before it is computed:
    /// the ISR samples the currents, computes the duties, and the timer
    /// latches them at its next update event — half a period later on a
    /// center-aligned timer that updates at both ends (the ISR finished
    /// before the valley), a full period later when it did not. The
    /// firmware models the same quantity as
    /// `FocDriver::set_actuation_delay`; set both equal to test a
    /// compensation against the delay it exists for. A fractional delay
    /// splits each period's volt-seconds exactly between the previous and
    /// the new command, whatever `substeps` is. `step_coast` /
    /// `step_shorted` push zeros through the pipeline (gates off / low).
    pub actuation_delay: f32,
}

impl MotorParams {
    /// Incremental d-axis inductance at the given d current (see
    /// [`sat_k`](Self::sat_k)).
    fn ld_eff(&self, id: ElectricCurrent) -> Inductance {
        let denom = clamp_f32(1.0 + (self.sat_k * id).fraction(), 0.25, 4.0);
        self.ld / denom
    }

    /// Incremental q-axis inductance at the given q current (see
    /// [`lq_sat_k`](Self::lq_sat_k)).
    fn lq_eff(&self, iq: ElectricCurrent) -> Inductance {
        let denom = clamp_f32(1.0 + (self.lq_sat_k * iq.abs()).fraction(), 0.25, 4.0);
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
            r: ohms(0.5),
            ld: henries(5e-4),
            lq: henries(5e-4),
            lambda: webers(0.01),
            pole_pairs: 7,
            j: 1e-4,
            friction_b: 1e-4,
            hall_offset: ElectricalAngle::ZERO,
            sat_k: PerAmp::ZERO,
            lq_sat_k: PerAmp::ZERO,
            substeps: 1,
            dead_time_voltage: ElectricPotential::ZERO,
            adc_lsb: ElectricCurrent::ZERO,
            adc_noise: ElectricCurrent::ZERO,
            eddy_delta_l_d: Inductance::ZERO,
            eddy_delta_l_q: Inductance::ZERO,
            eddy_tau: Time::ZERO,
            actuation_delay: 0.0,
        }
    }
}

/// Output produced by one call to [`VirtualMotor::step`].
#[derive(Clone, Copy, Debug, Default)]
pub struct VirtualMotorOutput {
    /// Phase currents
    pub ia: ElectricCurrent,
    pub ib: ElectricCurrent,
    pub ic: ElectricCurrent,
    /// Electrical rotor angle (wrapped to −π … π)
    pub angle: ElectricalAngle,
    /// Electrical angular velocity
    pub omega_e: ElectricalAngularVelocity,
    /// Electromagnetic torque (N·m)
    pub torque: f32,
    /// Simulated Hall sensor raw state (3-bit value, 1–6).
    ///
    /// Encoded as H3<<2 | H2<<1 | H1, matching the convention used by
    /// [`crate::foc::hall_sensor::HallSensor`].  The state advances through
    /// the CW sequence 1 → 3 → 2 → 6 → 4 → 5 as the rotor spins forward.
    pub hall_state: u8,
    /// Open-circuit back-EMF in α-β stator frame.
    ///
    /// `e_α = −ωe × λ × sin(φ)`, `e_β = ωe × λ × cos(φ)`.
    /// Only physically meaningful when no current flows (coast mode).
    pub bemf_alpha: ElectricPotential,
    pub bemf_beta: ElectricPotential,
}

/// Dynamic PMSM model using forward-Euler integration in the dq frame.
///
/// All quantities are in SI units.  The ideal-default model is minimal
/// (no iron losses, no temperature effects); the non-idealities that our
/// own firmware compensates for are available behind opt-in parameters
/// with ideal defaults — d/q saturation (`sat_k`, `lq_sat_k`), dead-time
/// distortion (`dead_time_voltage`), sensor quantization/noise (`adc_lsb`,
/// `adc_noise`) and sub-stepped integration (`substeps`).
pub struct VirtualMotor {
    params: MotorParams,
    // Integrator state
    id: ElectricCurrent,
    iq: ElectricCurrent,
    /// Eddy-branch filtered currents (see `MotorParams::eddy_delta_l_d`).
    i_fd: ElectricCurrent,
    i_fq: ElectricCurrent,
    omega_e: ElectricalAngularVelocity,
    phi: ElectricalAngle,
    sin_phi: f32,
    cos_phi: f32,
    // xorshift32 state for current-sensor noise (deterministic seed so
    // every test run sees the identical noise stream)
    rng: u32,
    // Actuation pipeline: the last `V_RING` αβ commands, newest at
    // `v_ring_idx − 1`. A delay of up to 3 periods needs the command from
    // 4 calls ago for the fractional share.
    v_ring: [(ElectricPotential, ElectricPotential); V_RING],
    v_ring_idx: usize,
}

/// Command history depth of the actuation pipeline (max delay 3 + 1).
const V_RING: usize = 5;

/// The commands acting during one `step()` period: `older` for the first
/// `older_share` of it, `newer` for the rest.
#[derive(Clone, Copy)]
struct Actuation {
    older: (ElectricPotential, ElectricPotential),
    newer: (ElectricPotential, ElectricPotential),
    older_share: f32,
}

impl Actuation {
    /// Volt-second-weighted command over the sub-interval `[t0, t1)` of the
    /// period (fractions of it).
    fn over(&self, t0: f32, t1: f32) -> (ElectricPotential, ElectricPotential) {
        let w = clamp_f32((self.older_share - t0) / (t1 - t0), 0.0, 1.0);
        (
            w * self.older.0 + (1.0 - w) * self.newer.0,
            w * self.older.1 + (1.0 - w) * self.newer.1,
        )
    }
}

impl VirtualMotor {
    /// Create a new virtual motor with the given parameters.
    pub fn new(params: MotorParams) -> Self {
        Self {
            params,
            id: ElectricCurrent::ZERO,
            iq: ElectricCurrent::ZERO,
            i_fd: ElectricCurrent::ZERO,
            i_fq: ElectricCurrent::ZERO,
            omega_e: ElectricalAngularVelocity::ZERO,
            phi: ElectricalAngle::ZERO,
            sin_phi: 0.0,
            cos_phi: 1.0,
            rng: 0x6F78_6601, // "oxf\x01"
            v_ring: [(ElectricPotential::ZERO, ElectricPotential::ZERO); V_RING],
            v_ring_idx: 0,
        }
    }

    /// Push an αβ command into the actuation pipeline and return what acts
    /// during this period (zeros until the pipeline fills). A delay
    /// `d = m + f` puts the command from `m + 1` calls ago on the first `f`
    /// of the period and the one from `m` calls ago on the rest; delay 0 is
    /// a transparent pass-through.
    fn actuation(&mut self, v_alpha: ElectricPotential, v_beta: ElectricPotential) -> Actuation {
        self.v_ring[self.v_ring_idx] = (v_alpha, v_beta);
        let newest = self.v_ring_idx;
        self.v_ring_idx = (self.v_ring_idx + 1) % V_RING;
        let delay = clamp_f32(self.params.actuation_delay, 0.0, 3.0);
        let m = delay as usize; // floor for delay ≥ 0
        let ago = |k: usize| self.v_ring[(newest + V_RING - k) % V_RING];
        Actuation {
            older: ago(m + 1),
            newer: ago(m),
            older_share: delay - m as f32,
        }
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
    /// uniform noise (`adc_noise`), then quantization (`adc_lsb`).
    fn measure(&mut self, i: ElectricCurrent) -> ElectricCurrent {
        let p = self.params;
        let mut v = i;
        if p.adc_noise.is_positive() {
            v += p.adc_noise * self.next_noise();
        }
        if p.adc_lsb.is_positive() {
            v = p.adc_lsb * libm::roundf(v.ratio_to(p.adc_lsb));
        }
        v
    }

    /// Override the initial rotor angle.
    pub fn set_angle(&mut self, phi: ElectricalAngle) {
        self.phi = phi;
        self.sin_phi = libm::sinf(phi.radians());
        self.cos_phi = libm::cosf(phi.radians());
    }

    /// Override the rotor electrical velocity — sim helper to set up a
    /// freewheeling rotor (e.g. flying-restart tests).
    pub fn set_velocity(&mut self, omega_e: ElectricalAngularVelocity) {
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
        let phi = self.phi.radians();
        let phi_pos = if phi < 0.0 { phi + TAU } else { phi };
        let phi_hall = libm::fmodf(
            phi_pos - self.params.hall_offset.radians() + TAU + TAU / 12.0,
            TAU,
        );
        let sector = ((phi_hall * 6.0 / TAU) as usize).min(5);
        HALL_CW_RAW[sector]
    }

    /// Advance the electrical angle and refresh its cached sin/cos.
    fn advance_angle(&mut self, dt: Time) {
        self.phi += self.omega_e * dt;
        // Wrap to (−π, π] — use remainder for robustness at high speeds
        self.phi = erad(libm::remainderf(
            self.phi.radians(),
            2.0 * core::f32::consts::PI,
        ));
        self.sin_phi = libm::sinf(self.phi.radians());
        self.cos_phi = libm::cosf(self.phi.radians());
    }

    /// Mechanical dynamics over `dt`, in plain SI:
    /// `J·dωm/dt = Te − TL − B·ωm` →  `dωe/dt = p·(Te − TL − B·ωm)/J`, with
    /// `ωm = ωe/pp` so `B·ωm = (friction_b/pp)·ωe`.
    fn accelerate(&mut self, torque: f32, load_torque: f32, dt: Time) {
        let p = self.params;
        let pp = f32::from(p.pole_pairs);
        let friction_torque = (p.friction_b / pp) * self.omega_e.value;
        self.omega_e += erad_per_s(dt.value * pp / p.j * (torque - load_torque - friction_torque));
    }

    /// Open-circuit back-EMF: e = ωe × λ × [−sin(φ), cos(φ)]
    fn back_emf(&self) -> (ElectricPotential, ElectricPotential) {
        let e = self.omega_e * self.params.lambda;
        (-e * self.sin_phi, e * self.cos_phi)
    }

    /// Run one simulation step with all FETs off (high-impedance).
    ///
    /// No current flows.  The only torque is the external load plus
    /// viscous friction.  The motor decelerates purely mechanically.
    pub fn step_coast(&mut self, load_torque: f32, dt: Time) -> VirtualMotorOutput {
        // Gates off: nothing is commanded this cycle, the pipeline moves on.
        let _ = self.actuation(ElectricPotential::ZERO, ElectricPotential::ZERO);
        let p = self.params;
        let n = p.substeps.max(1);
        let dt_sub = dt / n as f32;

        // Zero current — no electromagnetic torque; the eddy-branch
        // currents decay with their own time constant.
        self.id = ElectricCurrent::ZERO;
        self.iq = ElectricCurrent::ZERO;
        if p.eddy_tau.is_positive() {
            let k = dt.ratio_to(p.eddy_tau).min(1.0);
            self.i_fd -= k * self.i_fd;
            self.i_fq -= k * self.i_fq;
        } else {
            self.i_fd = ElectricCurrent::ZERO;
            self.i_fq = ElectricCurrent::ZERO;
        }

        for _ in 0..n {
            // Mechanical dynamics: only friction + external load
            self.accelerate(0.0, load_torque, dt_sub);
            self.advance_angle(dt_sub);
        }

        let (bemf_alpha, bemf_beta) = self.back_emf();
        VirtualMotorOutput {
            ia: self.measure(ElectricCurrent::ZERO),
            ib: self.measure(ElectricCurrent::ZERO),
            ic: self.measure(ElectricCurrent::ZERO),
            angle: self.phi,
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
    pub fn step_shorted(&mut self, load_torque: f32, dt: Time) -> VirtualMotorOutput {
        // All low-side FETs held on: no switching, hence no dead-time
        // distortion regardless of `dead_time_voltage`.
        self.step_inner(
            ElectricPotential::ZERO,
            ElectricPotential::ZERO,
            load_torque,
            dt,
            false,
        )
    }

    /// Run one simulation step with a voltage source connected.
    ///
    /// # Arguments
    /// * `v_alpha`      – α-axis voltage applied to the motor
    /// * `v_beta`       – β-axis voltage applied to the motor
    /// * `load_torque`  – externally applied load torque (N·m, positive = braking)
    /// * `dt`           – integration time step; use the FOC loop period
    pub fn step(
        &mut self,
        v_alpha: ElectricPotential,
        v_beta: ElectricPotential,
        load_torque: f32,
        dt: Time,
    ) -> VirtualMotorOutput {
        self.step_inner(v_alpha, v_beta, load_torque, dt, true)
    }

    fn step_inner(
        &mut self,
        v_alpha: ElectricPotential,
        v_beta: ElectricPotential,
        load_torque: f32,
        dt: Time,
        switching: bool,
    ) -> VirtualMotorOutput {
        let act = self.actuation(v_alpha, v_beta);
        let p = self.params;
        let pp = f32::from(p.pole_pairs);
        let n = p.substeps.max(1);
        let dt_sub = dt / n as f32;
        let mut torque = 0.0;
        // Keeps the effective winding inductance of the di/dt slope positive.
        let min_inductance = henries(1e-9);

        // The αβ voltage is a zero-order hold per command (that is what the
        // inverter applies) — one command per period, or two split at the
        // fractional actuation delay; the rotor and currents keep evolving
        // inside it, so everything below is re-derived per sub-step.
        for j in 0..n {
            let (v_alpha, v_beta) = act.over(j as f32 / n as f32, (j + 1) as f32 / n as f32);
            // ── Dead-time distortion (instantaneous phase-current signs) ──
            let (va, vb) = if switching && p.dead_time_voltage != ElectricPotential::ZERO {
                let i_alpha = self.cos_phi * self.id - self.sin_phi * self.iq;
                let i_beta = self.cos_phi * self.iq + self.sin_phi * self.id;
                let (ia, ib, ic) = inverse_clarke(i_alpha, i_beta);
                let sign = |i: ElectricCurrent| if i.is_non_negative() { 1.0f32 } else { -1.0 };
                let (sa, sb, sc) = (sign(ia), sign(ib), sign(ic));
                // Per-phase −sign(i)·V_dt mapped through the (amplitude-
                // invariant) Clarke transform; the common mode drops out.
                let e_alpha = (2.0 * sa - sb - sc) * (p.dead_time_voltage / 3.0);
                let e_beta = (sb - sc) * FRAC_1_SQRT_3 * p.dead_time_voltage;
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
            let psi_q: MagneticFlux = (p.lq - dl_q) * self.iq + dl_q * self.i_fq;
            // Eddy branch EMF ΔL·d(i_f)/dt = ΔL·(i − i_f)/τ_e — the lossy
            // part of L(jω) (frequency-dependent eddy resistance).
            let (e_fd, e_fq): (ElectricPotential, ElectricPotential) = if p.eddy_tau.is_positive() {
                (
                    dl_d * (self.id - self.i_fd) / p.eddy_tau,
                    dl_q * (self.iq - self.i_fq) / p.eddy_tau,
                )
            } else {
                (ElectricPotential::ZERO, ElectricPotential::ZERO)
            };

            // ── D-axis current ────────────────────────────────────────────
            // (Ld_eff−ΔLd)·did/dt = Vd − R·id + ωe·ψq − ΔLd·(id−i_fd)/τe
            // Ld_eff(id) models d-axis saturation when sat_k ≠ 0 (HFI polarity).
            self.id += (vd + self.omega_e * psi_q - p.r * self.id - e_fd) * dt_sub
                / (p.ld_eff(self.id) - dl_d).max(min_inductance);

            // ── Q-axis current ────────────────────────────────────────────
            // (Lq_eff−ΔLq)·diq/dt = Vq − R·iq − ωe·ψd − ΔLq·(iq−i_fq)/τe
            // Lq_eff(iq) models saliency collapse when lq_sat_k ≠ 0.
            // ψd from the id just updated (semi-implicit Euler): the ω
            // cross-coupling is a rotation of the current vector, which
            // explicit Euler stretches by √(1 + (ω·dt)²) per step — unstable
            // past ω·dt ≈ √(2·R·dt/L), 8000 el rad/s on a 5065 at one
            // substep, well inside a 12S board's speed range. This order
            // keeps the rotation norm-preserving.
            let psi_d: MagneticFlux = (p.ld - dl_d) * self.id + dl_d * self.i_fd + p.lambda;
            self.iq += (vq - self.omega_e * psi_d - p.r * self.iq - e_fq) * dt_sub
                / (p.lq_eff(self.iq) - dl_q).max(min_inductance);
            let psi_q: MagneticFlux = (p.lq - dl_q) * self.iq + dl_q * self.i_fq;

            // ── Eddy branch state ─────────────────────────────────────────
            if p.eddy_tau.is_positive() {
                let k = dt_sub.ratio_to(p.eddy_tau).min(1.0);
                self.i_fd += k * (self.id - self.i_fd);
                self.i_fq += k * (self.iq - self.i_fq);
            }

            // ── Electromagnetic torque (flux cross product), N·m ──────────
            // Te = (3/2)·p·(ψd·iq − ψq·id) — reduces to the familiar
            // (3/2)·p·[λPM + (Ld − Lq)·id]·iq at ΔL = 0.
            torque = 1.5 * pp * (psi_d * self.iq - psi_q * self.id).value;

            self.accelerate(torque, load_torque, dt_sub);
            self.advance_angle(dt_sub);
        }

        // ── Inverse Park + inverse Clarke → phase currents ────────────────────
        let i_alpha = self.cos_phi * self.id - self.sin_phi * self.iq;
        let i_beta = self.cos_phi * self.iq + self.sin_phi * self.id;
        let (ia, ib, ic) = inverse_clarke(i_alpha, i_beta);

        let hall_state = self.hall_state();
        let (bemf_alpha, bemf_beta) = self.back_emf();

        VirtualMotorOutput {
            ia: self.measure(ia),
            ib: self.measure(ib),
            ic: self.measure(ic),
            angle: self.phi,
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
    use crate::foc::hall_calibration::{HallCalibrationResult, HallCalibrator};
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::pi_controller::PIController;
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::{angle_difference, wrap_angle};
    use crate::units::{
        amps, erad, erad_per_s, henries, ohms, per_second, quantity, seconds, si, sqrt_current,
        volts, webers,
    };

    #[test]
    fn closed_loop_accelerates() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let kp = params.ld * per_second(1000.0);
        let ki = params.r * per_second(1000.0);

        let mut foc = FocController::<SvpwmModulator>::new(volts(24.0));
        foc.id_pi = PIController::with_gains(kp, ki);
        foc.iq_pi = PIController::with_gains(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Run 500 steps = 25 ms.  Back-EMF is still small, so the FOC can
        // inject close to the target 2 A and the motor accelerates.
        for _ in 0..500 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }

        // Motor should be spinning and producing torque.
        assert!(
            out.omega_e > erad_per_s(10.0),
            "motor should spin up: ωe={:?}",
            out.omega_e
        );
        assert!(out.torque > 0.0, "torque should be positive");
        let i_mag = libm::sqrtf((out.ia * out.ia + out.ib * out.ib + out.ic * out.ic).value);
        assert!(
            i_mag > 0.3,
            "motor should carry current at low speed: |i|={i_mag}"
        );

        // Continue for a full second.  Viscous friction limits the terminal
        // speed, but ωe should still be well above zero.
        for _ in 0..20_000 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }
        assert!(
            out.omega_e > erad_per_s(100.0),
            "motor should reach steady-state speed: ωe={:?}",
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
        let kp = params.ld * per_second(1000.0);
        let ki = params.r * per_second(1000.0);

        let mut foc = FocController::<SvpwmModulator>::new(volts(24.0));
        foc.id_pi = PIController::with_gains(kp, ki);
        foc.iq_pi = PIController::with_gains(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Run 500 steps with negative iq_target
        for _ in 0..500 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(-2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }

        // Motor should spin in reverse
        assert!(
            out.omega_e < erad_per_s(-10.0),
            "motor should spin in reverse: ωe={:?}",
            out.omega_e
        );
        assert!(out.torque < 0.0, "torque should be negative");

        // Continue for a full second to reach steady state
        for _ in 0..20_000 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(-2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }

        assert!(
            out.omega_e < erad_per_s(-100.0),
            "motor should reach steady-state reverse speed: ωe={:?}",
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
        let kp = params.ld * per_second(1000.0);
        let ki = params.r * per_second(1000.0);

        let mut foc = FocController::<SvpwmModulator>::new(volts(24.0));
        foc.id_pi = PIController::with_gains(kp, ki);
        foc.iq_pi = PIController::with_gains(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Phase 1: spin up to steady state with no load (1 second)
        for _ in 0..20_000 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }
        let speed_no_load = out.omega_e;
        assert!(
            speed_no_load > erad_per_s(100.0),
            "should reach steady state before load: ωe={speed_no_load:?}"
        );

        // Phase 2: apply load torque step, settle for 1 second
        let load = 0.005; // N·m — significant but within controller capability
        for _ in 0..20_000 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, load, seconds(DT));
        }
        let speed_loaded = out.omega_e;

        // Speed should drop but motor must keep spinning forward
        assert!(
            speed_loaded > erad_per_s(0.0),
            "motor should not stall under load: ωe={speed_loaded:?}"
        );
        assert!(
            speed_loaded < speed_no_load,
            "speed should decrease under load: {speed_loaded:?} vs {speed_no_load:?}"
        );

        // Phase 3: remove load, verify recovery (1 second)
        for _ in 0..20_000 {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle,
                amps(0.0),
                amps(2.0),
                1000,
                seconds(DT),
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
        }
        let speed_recovered = out.omega_e;

        // Should recover close to original no-load speed
        assert!(
            (speed_recovered - speed_no_load).abs() / speed_no_load < quantity(0.05),
            "speed should recover after load removal: {speed_recovered:?} vs {speed_no_load:?}"
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
            hall_offset: erad(PI / 6.0),
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
            cal_motor.set_angle(erad(phi_e));
            // Zero-voltage, tiny dt: we only need the Hall state output.
            let out = cal_motor.step(volts(0.0), volts(0.0), 0.0, seconds(1e-9));
            // `HallCalibrator::record` expects angle in [0, 2π).
            let phi_cal = if phi_e < 0.0 { phi_e + TAU } else { phi_e };
            calibrator.record(erad(phi_cal), out.hall_state);
        }

        let cal_result = calibrator.finish().expect("calibration must succeed");
        assert!(
            cal_result.is_valid(),
            "calibration result must cover all 6 states"
        );

        // Every sector centroid sits at k·60° + offset.
        for (k, raw) in HALL_CW_RAW.iter().enumerate() {
            let angle = cal_result.angle_for_raw_state(*raw).unwrap();
            let expected = k as f32 * PI / 3.0 + PI / 6.0;
            assert!(
                angle_difference(angle, erad(expected)).abs() < erad(0.02),
                "raw state {raw:?}: calibrated {angle:.4?}, expected {expected:.4?}"
            );
        }

        // ── Step 2: closed-loop FOC on the Hall angle ─────────────────────────
        // Mean |frame error| (Hall angle vs rotor) over the last quarter
        // second of a 1 s spin-up, with or without the calibration.
        let run = |calibration: Option<&HallCalibrationResult>| {
            let kp = params.ld * per_second(1_000.0);
            let ki = params.r * per_second(1_000.0);
            let mut foc = FocController::<SvpwmModulator>::new(volts(24.0));
            foc.id_pi = PIController::with_gains(kp, ki);
            foc.iq_pi = PIController::with_gains(kp, ki);

            let mut motor = VirtualMotor::new(params);
            let mut out = VirtualMotorOutput::default();
            let mut hall = HallSensor::new(TICKS_PER_SEC);
            if let Some(c) = calibration {
                hall.apply_calibration(c);
            }
            let mut prev_hall_state = 0u8;
            let mut hall_angle = 0.0_f32;
            let mut err_sum = 0.0f32;
            let mut err_n = 0u32;
            for tick in 0u64..20_000 {
                let hs = out.hall_state;
                if hs != prev_hall_state && hs != 0 {
                    if let Some(angle) = hall.update(hs, tick) {
                        hall_angle = angle.value;
                    }
                    prev_hall_state = hs;
                }
                if let Some(sample) = hall.sample_at_mut(tick) {
                    hall_angle = sample.angle.value;
                }
                if tick >= 15_000 {
                    err_sum += angle_difference(hall_angle, out.angle.value).abs();
                    err_n += 1;
                }
                let telem = foc.step(
                    (out.ia, out.ib, out.ic),
                    erad(hall_angle),
                    amps(0.0),
                    amps(2.0),
                    1000,
                    seconds(DT),
                );
                out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
            }
            assert!(
                out.omega_e > erad_per_s(100.0),
                "must spin up: ωe={:?}",
                out.omega_e
            );
            err_sum / err_n as f32
        };
        let calibrated = run(Some(&cal_result));
        let uncalibrated = run(None);
        assert!(calibrated < 0.1, "calibrated frame error {calibrated} rad");
        // The test must be able to tell: the default table is off by the
        // full 30° offset.
        assert!(
            uncalibrated > 0.4,
            "uncalibrated frame error {uncalibrated} rad"
        );
    }

    /// A shorted 5065 at 12 000 el rad/s (12S top speed) on ONE substep:
    /// ω·dt = 0.6. The short current must settle at λω/|Z|; the explicit
    /// Euler cross-coupling grew it ×1.1 per step instead.
    #[test]
    fn single_substep_stays_stable_at_high_speed() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams {
            r: ohms(0.036),
            ld: henries(22e-6),
            lq: henries(22e-6),
            lambda: webers(2.92e-3),
            pole_pairs: 7,
            j: 1e3, // speed held constant
            friction_b: 0.0,
            ..MotorParams::default()
        };
        let omega = erad_per_s(12_000.0);
        let mut motor = VirtualMotor::new(params);
        motor.set_velocity(omega);
        let reactance: ElectricalResistance = omega * params.ld;
        let impedance = (params.r * params.r + reactance * reactance).sqrt_with(libm::sqrtf);
        let settled: ElectricCurrent = omega * params.lambda / impedance;
        let magnitude = |m: &VirtualMotor| sqrt_current(m.id * m.id + m.iq * m.iq);
        let mut peak = ElectricCurrent::ZERO;
        for _ in 0..2_000 {
            motor.step_shorted(0.0, seconds(DT));
            peak = peak.max(magnitude(&motor));
        }
        let last = magnitude(&motor);
        assert!(peak < 2.5 * settled, "peak {peak:?} vs settled {settled:?}");
        assert!(
            (last.ratio_to(settled) - 1.0).abs() < 0.1,
            "{last:?} vs {settled:?}"
        );
    }

    /// Regression for the first 0.06 A reference step on the MK5 bench.
    /// The ideal inverter is a deliberate over-compensation bound: an unknown
    /// low-current distortion must not make a tiny reference inject the full
    /// 0.23 V correction. This is not a calibrated model of discontinuous
    /// conduction on the real power stage.
    #[test]
    fn dead_time_linear_region_bounds_small_reference_startup_impulse() {
        const VBUS: f32 = 24.0;
        const ARR: u16 = 4199;
        const DT: f32 = 1.0 / 20_000.0;
        let run = |linear: bool| {
            let mut motor = VirtualMotor::new(MotorParams {
                r: ohms(0.031),
                ld: henries(25e-6),
                lq: henries(35e-6),
                substeps: 10,
                ..MotorParams::default()
            });
            let mut foc = FocController::<SvpwmModulator>::new(volts(VBUS));
            foc.id_pi = PIController::with_gains(ohms(0.01), si(10.0));
            foc.iq_pi = PIController::with_gains(ohms(0.01), si(10.0));
            foc.set_dead_time_comp(360, 20_000);
            if linear {
                foc.set_dead_time_comp_linear_region(amps(1.0)).unwrap();
            }
            let mut out = VirtualMotorOutput::default();
            let mut peak = 0.0f32;
            for step in 0..4000 {
                let target = if step < 400 { 0.0 } else { 0.06 };
                let telem = foc.step(
                    (out.ia, out.ib, out.ic),
                    erad(0.0),
                    amps(target),
                    amps(0.0),
                    ARR,
                    seconds(DT),
                );
                let [a, b, c] = telem.duties.map(|d| f32::from(d) * VBUS / f32::from(ARR));
                out = motor.step(
                    volts((2.0 * a - b - c) / 3.0),
                    volts((b - c) * FRAC_1_SQRT_3),
                    0.0,
                    seconds(DT),
                );
                peak = peak.max(motor.id.abs().value);
            }
            (peak, motor.id)
        };
        let (hard_peak, _) = run(false);
        let (smooth_peak, settled) = run(true);
        assert!(hard_peak > 3.0, "missing regression stimulus: {hard_peak}");
        assert!(
            smooth_peak < 0.3 && smooth_peak < hard_peak / 10.0,
            "startup impulse {smooth_peak} A vs legacy {hard_peak} A"
        );
        assert!(
            (settled - amps(0.06)).abs() < amps(0.01),
            "tracking did not settle: {settled:?}"
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

        // RMS dq tracking error at `iq_target` with `noise_a` of uniform
        // current-sensor noise.
        let run = |comp: bool, iq_target: f32, noise_a: f32| -> f32 {
            let params = MotorParams {
                dead_time_voltage: volts(DEAD_TIME_NS as f32 * 1e-9 * 20_000.0 * VBUS), // 0.384 V
                adc_noise: amps(noise_a),
                substeps: 10,
                // Settle near ωe ≈ 735 rad/s — fast enough that the
                // 6th-harmonic distortion (~700 Hz) sits above the current
                // loop bandwidth, far below bus-voltage saturation.
                friction_b: 2e-3,
                ..MotorParams::default()
            };
            let kp = params.ld * per_second(1000.0);
            let ki = params.r * per_second(1000.0);
            let mut foc = FocController::<SvpwmModulator>::new(volts(VBUS));
            foc.id_pi = PIController::with_gains(kp, ki);
            foc.iq_pi = PIController::with_gains(kp, ki);
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
                    out.angle,
                    amps(0.0),
                    amps(iq_target),
                    MAX_DUTY,
                    seconds(DT),
                );
                let scale = VBUS / f32::from(MAX_DUTY);
                let va = f32::from(telem.duties[0]) * scale;
                let vb = f32::from(telem.duties[1]) * scale;
                let vc = f32::from(telem.duties[2]) * scale;
                let v_alpha = (2.0 * va - vb - vc) / 3.0;
                let v_beta = (vb - vc) * FRAC_1_SQRT_3;
                out = motor.step(volts(v_alpha), volts(v_beta), 0.0, seconds(DT));
                if step >= 10_000 {
                    // TRUE dq current (noise-free plant state) — the
                    // measured one would score the sensor noise itself.
                    let ed = motor.id;
                    let eq = motor.iq - amps(iq_target);
                    err_sq += (ed * ed + eq * eq).value;
                    n += 1;
                }
            }
            libm::sqrtf(err_sq / n as f32)
        };

        let uncompensated = run(false, 2.0, 0.0);
        let compensated = run(true, 2.0, 0.0);
        let noisy_off = run(false, 0.3, 0.15);
        let noisy_on = run(true, 0.3, 0.15);
        assert!(
            uncompensated > 0.05,
            "plant must produce real dead-time current distortion: RMS = {uncompensated}"
        );
        // ~0.19× now; the old volt-space coefficients compensated only 2/3
        // of the dead time and landed at 0.35× — under the 0.5 this bound
        // used to be, so it never noticed.
        assert!(
            compensated < uncompensated * 0.25,
            "compensation must cancel most of the distortion: {compensated} vs {uncompensated}"
        );
        // Low current under sensor noise: signs taken from the MEASURED
        // current flipped on noise and made the error worse than no
        // compensation (0.078 vs 0.063 A); reference signs reach ~0.35×.
        assert!(
            noisy_on < noisy_off * 0.5,
            "compensation must still help at low current under noise: {noisy_on} vs {noisy_off}"
        );
    }

    /// Pipeline-delay compensation must rotate the ACTUATION frame only.
    ///
    /// With `actuation_delay = 1` the plant applies each command one
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
                actuation_delay: 1.0,
                substeps: 10,
                friction_b: 1.2e-3, // settle ≈ 1200 rad/s el (δ ≈ 0.06 rad)
                ..MotorParams::default()
            };
            let kp = params.ld * per_second(1000.0);
            let ki = params.r * per_second(1000.0);
            let mut foc = FocController::<SvpwmModulator>::new(volts(VBUS));
            foc.id_pi = PIController::with_gains(kp, ki);
            foc.iq_pi = PIController::with_gains(kp, ki);
            let mut motor = VirtualMotor::new(params);
            let mut out = VirtualMotorOutput::default();
            let mut id_sum = 0.0f32;
            let mut n = 0u32;
            for step in 0..30_000 {
                // Perfect estimator: true rotor state stands in for the
                // phase provider.
                let delta = out.omega_e * seconds(DT);
                let (angle_cmd, advance) = if advance_measurement_frame {
                    // the old firmware behavior: one advanced angle for both
                    (wrap_angle(out.angle + delta), ElectricalAngle::ZERO)
                } else {
                    // the fixed path: raw Park + actuation-side rotation
                    (out.angle, delta)
                };
                foc.set_actuation_advance(advance);
                let telem = foc.step(
                    (out.ia, out.ib, out.ic),
                    angle_cmd,
                    amps(0.0),
                    amps(2.0),
                    4250,
                    seconds(DT),
                );
                out = motor.step(telem.v_alpha, telem.v_beta, 0.0, seconds(DT));
                if step >= 20_000 {
                    let (i_alpha, i_beta) = transforms::clarke(out.ia, out.ib);
                    let (s, c) = (libm::sinf(out.angle.value), libm::cosf(out.angle.value));
                    let (id, _) = transforms::park(i_alpha, i_beta, s, c);
                    id_sum += id.value;
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
            motor.set_angle(erad(phi));
            // One zero-voltage step to populate the output.
            let out = motor.step(volts(0.0), volts(0.0), 0.0, seconds(1e-6));
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
            r: ohms(0.127),
            ld: henries(86e-6),
            lq: henries(129e-6),
            lambda: webers(1.145e-3),
            pole_pairs: 7,
            j: 5e-5,
            friction_b: 4e-6,
            substeps: 10,
            eddy_delta_l_d: henries(62e-6),
            eddy_delta_l_q: henries(105e-6),
            eddy_tau: seconds(0.2e-3),
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
                let out = m.step(volts(v), volts(0.0), 0.0, seconds(dt));
                if k > steps * 8 / 10 {
                    amp = amp.max(out.ia.abs().value);
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
            eddy_delta_l_d: henries(0.0),
            eddy_delta_l_q: henries(0.0),
            eddy_tau: seconds(0.0),
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
