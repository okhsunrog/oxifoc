//! High-level Field-Oriented Control loop
//!
//! This module keeps the full FOC math inside `oxifoc-core` so platform
//! crates only need to provide sensors and PWM drivers implementing the
//! shared traits. The controller is intentionally minimal: it runs one
//! current loop step and returns the computed PWM duties plus telemetry.

use super::{pi_controller::PIController, svpwm, transforms};

/// Result of a single FOC update
#[derive(Clone, Copy, Debug, PartialEq)]
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

/// Field-Oriented Controller (current loop)
///
/// The controller is hardware-agnostic: it consumes measured currents and
/// electrical angle, then returns duty cycles that can be written through a
/// platform `PhasePwm` implementation.
pub struct FocController {
    /// d-axis PI controller (flux)
    pub id_pi: PIController,
    /// q-axis PI controller (torque)
    pub iq_pi: PIController,
    /// DC bus voltage (Volts)
    vbus: f32,
    /// Modulation limit as a fraction of `vbus` (0.0–1.0)
    modulation_limit: f32,
}

impl FocController {
    /// Conservative default modulation limit (keeps us inside linear SVPWM)
    pub const DEFAULT_MODULATION_LIMIT: f32 = 0.577; // ≈ 1/√3

    /// Create a new controller with reasonable default gains and limits.
    pub fn new(vbus: f32) -> Self {
        Self {
            id_pi: PIController::new(0.4, 40.0).with_limits(-12.0, 12.0),
            iq_pi: PIController::new(0.4, 40.0).with_limits(-12.0, 12.0),
            vbus: vbus.max(1.0),
            modulation_limit: Self::DEFAULT_MODULATION_LIMIT,
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
        self.vbus = vbus.max(0.5);
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
        let (ia, ib, ic) = currents;
        let sin_theta = libm::sinf(angle_rad);
        let cos_theta = libm::cosf(angle_rad);

        // Phase currents -> stationary frame
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);
        // Stationary -> rotating frame
        let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);

        // Current controllers (dq frame)
        let mut vd = self.id_pi.update(id_target, id, dt);
        let mut vq = self.iq_pi.update(iq_target, iq, dt);

        // Clamp to allowed modulation to avoid distorting SVPWM
        let v_limit = self.vbus * self.modulation_limit;
        vd = vd.clamp(-v_limit, v_limit);
        vq = vq.clamp(-v_limit, v_limit);

        // dq -> stationary frame
        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
        let inv_vbus = 1.0 / self.vbus;
        let duties = svpwm::space_vector_pwm(v_alpha * inv_vbus, v_beta * inv_vbus, max_duty);

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
        let mut foc = FocController::new(24.0);
        let telem = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1000, DT);

        // Duties should be near mid-scale for zero voltage command
        for duty in telem.duties {
            assert!(duty >= 490 && duty <= 510);
        }
        assert!((telem.id).abs() < 1e-6);
        assert!((telem.iq).abs() < 1e-6);
    }

    #[test]
    fn positive_q_axis_command_generates_voltage() {
        let mut foc = FocController::new(48.0);
        let telem = foc.step((0.0, 0.0, 0.0), 1.0, 0.0, 5.0, 1200, DT);

        assert!(telem.vq > 0.0);
        assert!(telem.duties.iter().all(|d| *d <= 1200));
    }

    #[test]
    fn modulation_limit_is_respected() {
        let mut foc = FocController::new(30.0).with_modulation_limit(0.25);
        let telem = foc.step((2.0, -1.0, -1.0), 0.7, 0.0, 20.0, 800, DT);

        let v_limit = foc.vbus() * foc.modulation_limit() + 1e-6;
        assert!(telem.vd.abs() <= v_limit);
        assert!(telem.vq.abs() <= v_limit);
    }
}
