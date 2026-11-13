//! Reviewed motor setup, independent of board calibration and protection.
//! Wire/storage values must be validated against the actual inverter before use.
pub mod record;

use crate::storage::{CurrentLimitsConfig, HallCalibrationConfig, MotorParamsConfig};
#[cfg(any(feature = "runtime", test))]
use crate::storage::{MotorFeedbackConfig, MotorFeedbackMode};
use crate::types::commissioning::SourceLimits;

#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveCurrent {
    Auto,
    Override(f32),
}

#[derive(
    Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Feedback {
    Sensorless,
    Hall(HallCalibrationConfig),
}

#[derive(
    Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DriveSetup {
    pub motor: MotorParamsConfig,
    pub feedback: Feedback,
    pub source: SourceLimits,
    pub current: DriveCurrent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetupError {
    Motor,
    Hall,
    Source,
    Protection,
    Current,
}

/// Validated installation input. Cannot be constructed by deserialization.
/// Protection is supplied by the controller, never by the candidate.
pub struct ValidatedDriveSetup {
    setup: DriveSetup,
    limits: CurrentLimitsConfig,
    thermal_current_a: f32,
}
impl DriveSetup {
    pub fn validate(
        self,
        board_trip_a: f32,
        configured_trip_a: f32,
    ) -> Result<ValidatedDriveSetup, SetupError> {
        use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
        if !self.motor.is_valid()
            || self.motor.flux_linkage_wb <= 0.0
            || !self.motor.max_power_loss_w.is_finite()
            || self.motor.max_power_loss_w <= 0.0
            || [self.motor.ld_fundamental_h, self.motor.lq_fundamental_h]
                .iter()
                .any(|v| !v.is_finite() || *v < 0.0)
            || ((self.motor.ld_fundamental_h == 0.0) != (self.motor.lq_fundamental_h == 0.0))
        {
            return Err(SetupError::Motor);
        }
        if let Feedback::Hall(hall) = &self.feedback
            && !hall.is_calibrated()
        {
            return Err(SetupError::Hall);
        }
        if !self.source.is_valid() {
            return Err(SetupError::Source);
        }
        if !board_trip_a.is_finite()
            || board_trip_a <= 0.0
            || !configured_trip_a.is_finite()
            || configured_trip_a < 0.0
        {
            return Err(SetupError::Protection);
        }
        let trip = if configured_trip_a == 0.0 {
            board_trip_a
        } else {
            board_trip_a.min(configured_trip_a)
        };
        let thermal = self.motor.thermal_current_a().ok_or(SetupError::Motor)?;
        let ceiling = thermal.min(trip / OVERCURRENT_HEADROOM);
        let current = match self.current {
            DriveCurrent::Auto => ceiling,
            DriveCurrent::Override(a) if a.is_finite() && a > 0.0 && a <= ceiling => a,
            DriveCurrent::Override(_) => return Err(SetupError::Current),
        };
        let limits = CurrentLimitsConfig {
            max_iq_a: current,
            max_brake_a: current,
            max_phase_current_a: configured_trip_a,
            bus_in_max_a: self.source.input_a,
            bus_regen_max_a: self.source.regen_a,
            // A new setup must not inherit dissipative braking from another motor.
            max_dissipation_w: 0.0,
        };
        let mut setup = self;
        // The thermal estimate is derived, not an independent user assertion.
        setup.motor.max_current_a = thermal;
        Ok(ValidatedDriveSetup {
            setup,
            limits,
            thermal_current_a: thermal,
        })
    }
}
impl ValidatedDriveSetup {
    pub fn setup(&self) -> &DriveSetup {
        &self.setup
    }
    pub fn limits(&self) -> &CurrentLimitsConfig {
        &self.limits
    }

    #[cfg(any(feature = "runtime", test))]
    pub(crate) fn current_policy(&self) -> CurrentLimitsConfig {
        match self.setup.current {
            DriveCurrent::Auto => CurrentLimitsConfig {
                max_iq_a: 0.0,
                max_brake_a: 0.0,
                ..self.limits.clone()
            },
            DriveCurrent::Override(_) => self.limits.clone(),
        }
    }

    #[cfg(any(feature = "runtime", test))]
    pub(crate) fn motor_policy(&self) -> MotorParamsConfig {
        MotorParamsConfig {
            max_current_a: 0.0,
            ..self.setup.motor.clone()
        }
    }
    pub fn hall(&self) -> Option<HallCalibrationConfig> {
        match self.setup.feedback {
            Feedback::Hall(hall) => Some(hall),
            Feedback::Sensorless => None,
        }
    }

    #[cfg(any(feature = "runtime", test))]
    pub(crate) fn motor_feedback(&self) -> MotorFeedbackConfig {
        MotorFeedbackConfig {
            mode: match self.setup.feedback {
                Feedback::Hall(_) => MotorFeedbackMode::Hall,
                Feedback::Sensorless => MotorFeedbackMode::Observer,
            },
            ..Default::default()
        }
    }

    #[cfg(all(feature = "runtime", feature = "detection", feature = "delivery"))]
    pub(crate) fn project(&self, config: &mut crate::storage::RuntimeConfig) {
        config.motor_params = Some(self.motor_policy());
        config.hall_calibration = self.hall();
        config.hall_calibration_invalid = false;
        config.current_limits = Some(self.current_policy());
        config.motor_feedback = Some(self.motor_feedback());
        config.motor_feedback_invalid = false;
    }

    pub fn thermal_current_a(&self) -> f32 {
        self.thermal_current_a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn candidate() -> DriveSetup {
        DriveSetup {
            motor: MotorParamsConfig {
                resistance_ohm: 0.063,
                inductance_d_h: 31e-6,
                inductance_q_h: 51e-6,
                flux_linkage_wb: 0.004,
                pole_pairs: 7,
                max_power_loss_w: 120.0,
                max_current_a: 999.0,
                ..Default::default()
            },
            feedback: Feedback::Sensorless,
            source: SourceLimits {
                input_a: 5.0,
                regen_a: 0.0,
            },
            current: DriveCurrent::Auto,
        }
    }
    #[test]
    fn auto_separates_motor_estimate_from_inverter_and_source() {
        let prepared = candidate().validate(70.0, 30.0).unwrap();
        assert!(prepared.thermal_current_a() > 35.0);
        assert_eq!(
            prepared.limits().max_iq_a,
            30.0 / crate::motor::foc_driver::OVERCURRENT_HEADROOM
        );
        assert_eq!(prepared.limits().bus_in_max_a, 5.0);
        assert_eq!(prepared.limits().bus_regen_max_a, 0.0);
        assert_eq!(prepared.limits().max_phase_current_a, 30.0);
        assert_eq!(
            prepared.setup().motor.max_current_a,
            prepared.thermal_current_a()
        );
    }
    #[test]
    fn override_cannot_raise_protection_or_motor_rating() {
        for a in [0.0, -1.0, f32::NAN, f32::INFINITY, 24.0] {
            let mut c = candidate();
            c.current = DriveCurrent::Override(a);
            assert!(matches!(c.validate(70.0, 30.0), Err(SetupError::Current)));
        }
        let mut c = candidate();
        c.current = DriveCurrent::Override(20.0);
        assert_eq!(c.validate(70.0, 30.0).unwrap().limits().max_iq_a, 20.0);
    }
    #[test]
    fn bad_source_model_and_feedback_are_rejected() {
        let mut c = candidate();
        c.source.regen_a = -1.0;
        assert!(matches!(c.validate(70.0, 30.0), Err(SetupError::Source)));
        let mut c = candidate();
        c.motor.flux_linkage_wb = 0.0;
        assert!(matches!(c.validate(70.0, 30.0), Err(SetupError::Motor)));
        let mut c = candidate();
        let mut hall = HallCalibrationConfig::default();
        hall.valid[1] = false;
        c.feedback = Feedback::Hall(hall);
        assert!(matches!(c.validate(70.0, 30.0), Err(SetupError::Hall)));
        assert!(matches!(
            candidate().validate(f32::NAN, 30.0),
            Err(SetupError::Protection)
        ));
    }
}
