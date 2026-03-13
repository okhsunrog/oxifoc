//! High-level Field-Oriented Control loop
//!
//! This module keeps the full FOC math inside `oxifoc-core` so platform
//! crates only need to provide sensors and PWM drivers implementing the
//! shared traits. The controller is intentionally minimal: it runs one
//! current loop step and returns the computed PWM duties plus telemetry.

use super::{
    detection::pi_tuning::{self, DEFAULT_BANDWIDTH_RAD_S},
    pi_controller::PIController,
    pwm::{Modulator, SvpwmModulator},
    transforms,
    trig::{LibmSinCos, SinCos},
};

/// Result of a single FOC update
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct FocTelemetry {
    /// Raw phase currents (A)
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    /// Electrical angle used for this step (radians)
    pub angle_rad: f32,
    /// Clarke-transformed currents (α, β)
    pub i_alpha: f32,
    pub i_beta: f32,
    /// Park-transformed currents (d, q)
    pub id: f32,
    pub iq: f32,
    /// PI controller outputs in dq frame
    pub vd: f32,
    pub vq: f32,
    /// Commanded stationary-frame voltages
    pub v_alpha: f32,
    pub v_beta: f32,
    /// Duty cycles to apply (already clamped)
    pub duties: [u16; 3],
}

impl FocTelemetry {
    /// Create an empty telemetry struct (const for static initialization)
    pub const fn empty() -> Self {
        Self {
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            angle_rad: 0.0,
            i_alpha: 0.0,
            i_beta: 0.0,
            id: 0.0,
            iq: 0.0,
            vd: 0.0,
            vq: 0.0,
            v_alpha: 0.0,
            v_beta: 0.0,
            duties: [0; 3],
        }
    }
}

/// Field-Oriented Controller (current loop)
///
/// The controller is hardware-agnostic: it consumes measured currents and
/// electrical angle, then returns duty cycles that can be written through a
/// platform `PhasePwm` implementation.
pub struct FocController<M: Modulator = SvpwmModulator, S: SinCos = LibmSinCos> {
    /// d-axis PI controller (flux)
    pub id_pi: PIController,
    /// q-axis PI controller (torque)
    pub iq_pi: PIController,
    /// DC bus voltage (Volts)
    vbus: f32,
    /// Modulation limit as a fraction of `vbus` (0.0–1.0)
    modulation_limit: f32,
    /// Modulator + SinCos phantom (both are ZSTs)
    _phantom: core::marker::PhantomData<(M, S)>,
}

impl<M: Modulator, S: SinCos> FocController<M, S> {
    /// Conservative default modulation limit (keeps us inside linear SVPWM)
    pub const DEFAULT_MODULATION_LIMIT: f32 = 0.577; // ≈ 1/√3

    /// Minimum bus voltage to avoid divide-by-zero (used in both `new` and `set_vbus`).
    const MIN_VBUS: f32 = 0.5;

    /// Create a new controller with reasonable default gains and limits.
    ///
    /// The default gains (kp=0.4, ki=40) are a conservative starting point
    /// but won't be optimal for most motors. Prefer [`from_motor_params`](Self::from_motor_params)
    /// when resistance and inductance are known.
    pub fn new(vbus: f32) -> Self {
        Self {
            id_pi: PIController::new(0.4, 40.0).with_limits(-12.0, 12.0),
            iq_pi: PIController::new(0.4, 40.0).with_limits(-12.0, 12.0),
            vbus: vbus.max(Self::MIN_VBUS),
            modulation_limit: Self::DEFAULT_MODULATION_LIMIT,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Create a controller with PI gains computed from motor parameters.
    ///
    /// Uses the standard pole-placement formula:
    /// - `Kp = L × ω_bw`
    /// - `Ki = R × ω_bw`
    ///
    /// PI output limits are set to `±vbus × modulation_limit` so the controller
    /// cannot request more voltage than the bus can deliver.
    ///
    /// # Arguments
    /// * `resistance` - Phase resistance in Ohms
    /// * `inductance` - Phase inductance in Henries (use Ld≈Lq for SPMSM)
    /// * `vbus` - DC bus voltage in Volts
    ///
    /// Uses the default bandwidth of 1000 rad/s (~160 Hz). For custom
    /// bandwidth, use [`from_motor_params_with_bw`](Self::from_motor_params_with_bw).
    pub fn from_motor_params(resistance: f32, inductance: f32, vbus: f32) -> Self {
        Self::from_motor_params_with_bw(resistance, inductance, vbus, DEFAULT_BANDWIDTH_RAD_S)
    }

    /// Create a controller with PI gains computed from motor parameters and custom bandwidth.
    ///
    /// # Arguments
    /// * `resistance` - Phase resistance in Ohms
    /// * `inductance` - Phase inductance in Henries
    /// * `vbus` - DC bus voltage in Volts
    /// * `bandwidth_rad_s` - Desired current loop bandwidth in rad/s
    pub fn from_motor_params_with_bw(
        resistance: f32,
        inductance: f32,
        vbus: f32,
        bandwidth_rad_s: f32,
    ) -> Self {
        let (kp, ki) = pi_tuning::calculate_current_gains(resistance, inductance, bandwidth_rad_s);
        let vbus = vbus.max(Self::MIN_VBUS);
        let v_limit = vbus * Self::DEFAULT_MODULATION_LIMIT;

        Self {
            id_pi: PIController::new(kp, ki).with_limits(-v_limit, v_limit),
            iq_pi: PIController::new(kp, ki).with_limits(-v_limit, v_limit),
            vbus,
            modulation_limit: Self::DEFAULT_MODULATION_LIMIT,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Override the modulation limit (fraction of `vbus`).
    ///
    /// Values above 1.0 request over-modulation; keep within 0.0–1.0 for
    /// predictable behavior.
    pub fn with_modulation_limit(mut self, limit: f32) -> Self {
        self.modulation_limit = limit.clamp(0.0, 1.2);
        self
    }

    /// Update the cached DC bus voltage.
    pub fn set_vbus(&mut self, vbus: f32) {
        // Avoid divide-by-zero while tolerating brief brownouts.
        self.vbus = vbus.max(Self::MIN_VBUS);
    }

    /// Current DC bus voltage cached in the controller.
    pub fn vbus(&self) -> f32 {
        self.vbus
    }

    /// Fractional modulation limit (0.0–1.2).
    pub fn modulation_limit(&self) -> f32 {
        self.modulation_limit
    }

    /// Reset both PI controller integrators.
    pub fn reset(&mut self) {
        self.id_pi.reset();
        self.iq_pi.reset();
    }

    /// Run one FOC current loop step.
    ///
    /// # Arguments
    /// * `currents`   - (ia, ib, ic) phase currents in Amps
    /// * `angle_rad`  - electrical angle in radians
    /// * `id_target`  - d-axis current target (flux), typically 0
    /// * `iq_target`  - q-axis current target (torque)
    /// * `max_duty`   - timer ARR value used for PWM normalization
    /// * `dt`         - loop period in seconds
    ///
    /// # Returns
    /// Telemetry containing intermediate values and final duty cycles.
    pub fn step(
        &mut self,
        currents: (f32, f32, f32),
        angle_rad: f32,
        id_target: f32,
        iq_target: f32,
        max_duty: u16,
        dt: f32,
    ) -> FocTelemetry {
        self.step_with_injection(currents, angle_rad, id_target, iq_target, 0.0, 0.0, max_duty, dt)
    }

    /// Run one FOC current loop step with voltage injection.
    ///
    /// Used for HFI-based inductance measurement. The injection voltages
    /// are added to the PI controller outputs before inverse Park transform.
    ///
    /// # Arguments
    /// * `currents`   - (ia, ib, ic) phase currents in Amps
    /// * `angle_rad`  - electrical angle in radians
    /// * `id_target`  - d-axis current target (flux)
    /// * `iq_target`  - q-axis current target (torque)
    /// * `vd_inject`  - d-axis voltage to inject (V)
    /// * `vq_inject`  - q-axis voltage to inject (V)
    /// * `max_duty`   - timer ARR value used for PWM normalization
    /// * `dt`         - loop period in seconds
    ///
    /// # Returns
    /// Telemetry containing intermediate values and final duty cycles.
    /// The `vd` and `vq` fields include the injected voltages.
    #[allow(clippy::too_many_arguments)]
    pub fn step_with_injection(
        &mut self,
        currents: (f32, f32, f32),
        angle_rad: f32,
        id_target: f32,
        iq_target: f32,
        vd_inject: f32,
        vq_inject: f32,
        max_duty: u16,
        dt: f32,
    ) -> FocTelemetry {
        let (ia, ib, ic) = currents;
        let (sin_theta, cos_theta) = S::sin_cos(angle_rad);

        // Phase currents -> stationary frame
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);
        // Stationary -> rotating frame
        let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);

        // Current controllers (dq frame) + optional HFI injection
        let mut vd = self.id_pi.update(id_target, id, dt) + vd_inject;
        let mut vq = self.iq_pi.update(iq_target, iq, dt) + vq_inject;

        // Clamp to allowed modulation to avoid distorting SVPWM
        let v_limit = self.vbus * self.modulation_limit;
        vd = vd.clamp(-v_limit, v_limit);
        vq = vq.clamp(-v_limit, v_limit);

        // dq -> stationary frame
        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
        let inv_vbus = 1.0 / self.vbus;
        let duties = M::to_duties(v_alpha * inv_vbus, v_beta * inv_vbus, max_duty);

        FocTelemetry {
            ia,
            ib,
            ic,
            angle_rad,
            i_alpha,
            i_beta,
            id,
            iq,
            vd,
            vq,
            v_alpha,
            v_beta,
            duties,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.0001;

    #[test]
    fn zero_currents_zero_setpoint_centered_pwm() {
        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        let telem = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1000, DT);

        // Duties should be near mid-scale for zero voltage command
        for duty in telem.duties {
            assert!((490..=510).contains(&duty));
        }
        assert!((telem.id).abs() < 1e-6);
        assert!((telem.iq).abs() < 1e-6);
    }

    #[test]
    fn positive_q_axis_command_generates_voltage() {
        let mut foc = FocController::<SvpwmModulator>::new(48.0);
        let telem = foc.step((0.0, 0.0, 0.0), 1.0, 0.0, 5.0, 1200, DT);

        assert!(telem.vq > 0.0);
        assert!(telem.duties.iter().all(|d| *d <= 1200));
    }

    #[test]
    fn modulation_limit_is_respected() {
        let mut foc = FocController::<SvpwmModulator>::new(30.0).with_modulation_limit(0.25);
        let telem = foc.step((2.0, -1.0, -1.0), 0.7, 0.0, 20.0, 800, DT);

        let v_limit = foc.vbus() * foc.modulation_limit() + 1e-6;
        assert!(telem.vd.abs() <= v_limit);
        assert!(telem.vq.abs() <= v_limit);
    }

    #[test]
    fn from_motor_params_computes_correct_gains() {
        // R = 0.5 Ω, L = 0.5 mH, vbus = 24 V, default bandwidth = 1000 rad/s
        let foc = FocController::<SvpwmModulator>::from_motor_params(0.5, 5e-4, 24.0);

        // Kp = L × ω = 5e-4 × 1000 = 0.5
        // Ki = R × ω = 0.5 × 1000 = 500
        // We can't read gains directly, but we can verify behavior:
        // A 1A error at dt=0.001 should produce output ≈ kp*1 + ki*1*0.001 = 0.5 + 0.5 = 1.0
        let mut foc = foc;
        let telem = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, 0.001);
        // vq should be positive and around 1.0 (kp*error + ki*error*dt)
        assert!(telem.vq > 0.4, "vq should reflect computed gains, got {}", telem.vq);
        assert!(telem.vq < 1.5, "vq should be reasonable, got {}", telem.vq);

        // PI limits should be ±vbus*modulation_limit = ±24*0.577 ≈ ±13.85
        let v_limit = 24.0 * FocController::<SvpwmModulator>::DEFAULT_MODULATION_LIMIT;
        let telem_saturated = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 100.0, 1000, 0.001);
        assert!(
            telem_saturated.vq <= v_limit + 1e-3,
            "PI output should be limited to {}, got {}",
            v_limit,
            telem_saturated.vq
        );
    }

    #[test]
    fn from_motor_params_with_custom_bandwidth() {
        // Higher bandwidth should give proportionally higher gains
        let foc_default = FocController::<SvpwmModulator>::from_motor_params(0.5, 5e-4, 24.0);
        let foc_fast = FocController::<SvpwmModulator>::from_motor_params_with_bw(0.5, 5e-4, 24.0, 2000.0);

        let mut foc_d = foc_default;
        let mut foc_f = foc_fast;

        let telem_d = foc_d.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, DT);
        let telem_f = foc_f.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, DT);

        // 2× bandwidth → 2× Kp → ~2× initial voltage response
        assert!(
            telem_f.vq > telem_d.vq * 1.5,
            "Higher bandwidth should give stronger response: {} vs {}",
            telem_f.vq,
            telem_d.vq
        );
    }
}
