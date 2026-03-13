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
}

/// Dynamic PMSM model using forward-Euler integration in the dq frame.
///
/// All quantities are in SI units.  The model is intentionally minimal
/// (no saturation, no iron losses, no temperature effects) to keep the
/// simulation simple and fast.
pub struct VirtualMotor {
    params: MotorParams,
    // Integrator state
    id: f32,
    id_int: f32, // integral state: id_int = id + lambda/ld
    iq: f32,
    omega_e: f32, // electrical angular velocity (rad/s)
    phi: f32,     // electrical rotor angle (rad)
    sin_phi: f32,
    cos_phi: f32,
}

impl VirtualMotor {
    /// Create a new virtual motor with the given parameters.
    ///
    /// The integrators are initialised so that all *currents* start at zero
    /// (the PM flux is accounted for in `id_int`).
    pub fn new(params: MotorParams) -> Self {
        // Pre-load id_int so that id = id_int - lambda/ld = 0 at t=0
        let id_int = params.lambda / params.ld;
        Self {
            params,
            id: 0.0,
            id_int,
            iq: 0.0,
            omega_e: 0.0,
            phi: 0.0,
            sin_phi: 0.0,
            cos_phi: 1.0,
        }
    }

    /// Override the initial rotor angle (electrical radians).
    pub fn set_angle(&mut self, phi_rad: f32) {
        self.phi = phi_rad;
        self.sin_phi = libm::sinf(phi_rad);
        self.cos_phi = libm::cosf(phi_rad);
    }

    /// Run one simulation step.
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
        let p = &self.params;
        let pp = p.pole_pairs as f32;

        // ── Park transform: αβ → dq (using current rotor angle) ──────────────
        let vd = self.cos_phi * v_alpha + self.sin_phi * v_beta;
        let vq = self.cos_phi * v_beta - self.sin_phi * v_alpha;

        // ── D-axis current (flux model with PM offset) ────────────────────────
        // Ld·did/dt = Vd − R·id + ωe·Lq·iq
        self.id_int += (vd + self.omega_e * p.lq * self.iq - p.r * self.id) * dt / p.ld;
        self.id = self.id_int - p.lambda / p.ld;

        // ── Q-axis current ────────────────────────────────────────────────────
        // Lq·diq/dt = Vq − R·iq − ωe·(Ld·id + λPM)
        self.iq += (vq - self.omega_e * (p.ld * self.id + p.lambda) - p.r * self.iq) * dt / p.lq;

        // ── Electromagnetic torque (with reluctance term) ─────────────────────
        // Te = (3/2)·p·[λPM + (Ld − Lq)·id]·iq
        let torque = 1.5 * pp * (p.lambda + (p.ld - p.lq) * self.id) * self.iq;

        // ── Mechanical dynamics ───────────────────────────────────────────────
        // J·dωm/dt = Te − TL − B·ωm  →  dωe/dt = p·(Te − TL − B·ωm)/J
        // ωm = ωe/pp, so B·ωm = (friction_b/pp)·ωe
        let friction_torque = (p.friction_b / pp) * self.omega_e;
        self.omega_e += dt * pp / p.j * (torque - load_torque - friction_torque);

        // ── Electrical angle integration ──────────────────────────────────────
        self.phi += self.omega_e * dt;
        // Wrap to (−π, π]
        use core::f32::consts::PI;
        if self.phi > PI {
            self.phi -= 2.0 * PI;
        } else if self.phi < -PI {
            self.phi += 2.0 * PI;
        }

        // Update cached sin/cos
        self.sin_phi = libm::sinf(self.phi);
        self.cos_phi = libm::cosf(self.phi);

        // ── Inverse Park + inverse Clarke → phase currents ────────────────────
        let i_alpha = self.cos_phi * self.id - self.sin_phi * self.iq;
        let i_beta = self.cos_phi * self.iq + self.sin_phi * self.id;
        let (ia, ib, ic) = inverse_clarke(i_alpha, i_beta);

        // ── Hall sensor simulation ────────────────────────────────────────────
        // phi is in (−π, π].  Convert to [0, 2π) then apply the mounting
        // offset, and map the resulting electrical angle to one of the 6 CW
        // sectors.  Each sector spans TAU/6 electrical radians.
        use core::f32::consts::TAU;
        let phi_pos = if self.phi < 0.0 {
            self.phi + TAU
        } else {
            self.phi
        };
        // fmodf keeps the result in [0, TAU) even when hall_offset > phi_pos
        let phi_hall = libm::fmodf(phi_pos - p.hall_offset + TAU, TAU);
        let sector = ((phi_hall * 6.0 / TAU) as usize).min(5);
        let hall_state = HALL_CW_RAW[sector];

        VirtualMotorOutput {
            ia,
            ib,
            ic,
            angle_rad: self.phi,
            omega_e: self.omega_e,
            torque,
            hall_state,
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

    #[test]
    fn closed_loop_accelerates() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let kp = params.ld * 1000.0;
        let ki = params.r * 1000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki).with_limits(-24.0, 24.0);
        foc.iq_pi = PIController::new(kp, ki).with_limits(-24.0, 24.0);
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
            "motor should carry current at low speed: |i|={}",
            i_mag
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

        // Sanity-check: with offset=π/6 the centre of sector 0 is at π/3.
        // The calibrated angle for raw state 1 (sector 0) should be close to π/3.
        let angle_raw1 = cal_result.angle_for_raw_state(1).unwrap();
        assert!(
            (angle_raw1 - PI / 3.0).abs() < 0.1,
            "calibrated angle for state 1 should be ≈π/3 with offset=π/6, got {angle_raw1:.4}"
        );

        // ── Step 2: closed-loop FOC with calibrated Hall sensor ───────────────
        let kp = params.ld * 1_000.0;
        let ki = params.r * 1_000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki).with_limits(-24.0, 24.0);
        foc.iq_pi = PIController::new(kp, ki).with_limits(-24.0, 24.0);

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
            "expected 6 Hall transitions, got {:?}",
            states
        );

        // The sequence is the CW Gray-code [1,3,2,6,4,5] (or any cyclic
        // rotation, depending on where in the revolution the sweep starts).
        const CW: [u8; 6] = [1, 3, 2, 6, 4, 5];
        let is_rotation = (0..6).any(|offset| (0..6).all(|i| states[i] == CW[(i + offset) % 6]));
        assert!(
            is_rotation,
            "sequence {:?} is not a CW rotation of {:?}",
            states, CW
        );
    }
}
