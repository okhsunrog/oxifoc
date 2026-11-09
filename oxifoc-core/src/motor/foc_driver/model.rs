//! One prepared model for boot and live configuration.

use super::{CurrentLimits, FocDriver};
use crate::foc::controller::{Decoupling, FocController};
use crate::foc::phase::{PhaseProvider, PhaseSource, PreparedPhaseModel};
use crate::foc::pwm::{PhasePwm, SvpwmModulator};
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::storage::RuntimeConfig;
use crate::types::VoltageFeedforwardMode;

/// Model construction can evaluate filter coefficients; installing this value
/// only replaces prepared state and scalar settings. Keep it out of ISR queues.
pub struct PreparedMotorModel {
    phase: Option<PreparedPhaseModel>,
    gains: (f32, f32),
    decoupling: Option<Decoupling>,
    voltage_feedforward_mode: VoltageFeedforwardMode,
    limits: CurrentLimits,
    pub pole_pairs: u8,
}

impl PreparedMotorModel {
    pub fn from_config(config: &RuntimeConfig, vbus: f32, board_max_current_a: f32) -> Self {
        let controller =
            FocController::<SvpwmModulator, LibmSinCos>::from_runtime_config(config, vbus);
        let motor = config.motor_params.as_ref().filter(|mp| mp.is_valid());
        Self {
            phase: Some(PreparedPhaseModel::from_config(config, vbus)),
            gains: controller.id_pi.gains(),
            decoupling: controller.decoupling(),
            voltage_feedforward_mode: controller.voltage_feedforward_mode(),
            limits: CurrentLimits::from_stored(
                config.current_limits.as_ref(),
                board_max_current_a,
                motor
                    .and_then(crate::storage::MotorParamsConfig::rating_current_a)
                    .unwrap_or(0.0),
            ),
            pole_pairs: motor.map_or(0, |mp| mp.pole_pairs),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorModelError {
    Busy,
    Unsupported,
    Consumed,
}

impl<P: PhasePwm, C: CurrentSensor, Ph: PhaseProvider, S: SinCos> FocDriver<P, C, Ph, S> {
    /// Install the complete model at a stopped control-cycle boundary. Hardware
    /// sensor state, voltage correction, PWM topology and source survive.
    pub fn apply_motor_model(
        &mut self,
        model: &mut PreparedMotorModel,
    ) -> Result<PhaseSource, MotorModelError> {
        if self.mode() != crate::motor::ControlMode::Stopped || self.current_offset_active() {
            return Err(MotorModelError::Busy);
        }
        let phase = model.phase.as_mut().ok_or(MotorModelError::Consumed)?;
        let source = self
            .phase
            .apply_motor_model(phase)
            .ok_or(MotorModelError::Unsupported)?;
        model.phase = None;
        self.set_current_pi_gains(model.gains.0, model.gains.1);
        self.controller.set_decoupling(model.decoupling);
        self.controller
            .set_voltage_feedforward_mode(model.voltage_feedforward_mode);
        self.controller.reset();
        self.set_current_limits(model.limits);
        Ok(source)
    }
}
