//! Reviewed motor setup, independent of board calibration and protection.
//! Wire/storage values must be validated against the actual inverter before use.
pub mod record;

#[cfg(all(feature = "delivery", feature = "detection", feature = "runtime"))]
use crate::storage::RuntimeConfig;
use crate::storage::{CurrentLimitsConfig, HallCalibrationConfig, MotorParamsConfig};
#[cfg(any(feature = "runtime", test))]
use crate::storage::{MotorFeedbackConfig, MotorFeedbackMode};
use crate::types::commissioning::SourceLimits;
use crate::units::{ElectricCurrent, Sign, watts};

#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveCurrent {
    Auto,
    Override(ElectricCurrent),
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
    thermal_current: ElectricCurrent,
}
impl DriveSetup {
    pub fn validate(
        self,
        board_trip: ElectricCurrent,
        configured_trip: Option<ElectricCurrent>,
    ) -> Result<ValidatedDriveSetup, SetupError> {
        use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
        if !self.motor.is_valid()
            || self.motor.flux_linkage.is_non_positive()
            || !self
                .motor
                .max_power_loss
                .is_some_and(|w| w.is_finite() && w.is_positive())
            || [self.motor.ld_fundamental, self.motor.lq_fundamental]
                .iter()
                .flatten()
                .any(|l| !l.is_finite() || l.is_non_positive())
            || self.motor.ld_fundamental.is_some() != self.motor.lq_fundamental.is_some()
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
        if !board_trip.is_finite()
            || board_trip.is_non_positive()
            || configured_trip.is_some_and(|trip| !trip.is_finite() || trip.is_non_positive())
        {
            return Err(SetupError::Protection);
        }
        let trip = configured_trip.map_or(board_trip, |configured| board_trip.min(configured));
        let thermal = self.motor.thermal_current().ok_or(SetupError::Motor)?;
        let ceiling = thermal.min(trip / OVERCURRENT_HEADROOM);
        let current = match self.current {
            DriveCurrent::Auto => ceiling,
            DriveCurrent::Override(a) if a.is_finite() && a.is_positive() && a <= ceiling => a,
            DriveCurrent::Override(_) => return Err(SetupError::Current),
        };
        let limits = CurrentLimitsConfig {
            max_iq: Some(current),
            max_brake: Some(current),
            max_phase_current: configured_trip,
            bus_in_max: Some(self.source.input),
            bus_regen_max: Some(self.source.regen),
            // A new setup must not inherit dissipative braking from another motor.
            max_dissipation: watts(0.0),
        };
        let mut setup = self;
        // The thermal estimate is derived, not an independent user assertion.
        setup.motor.max_current = Some(thermal);
        Ok(ValidatedDriveSetup {
            setup,
            limits,
            thermal_current: thermal,
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
                max_iq: None,
                max_brake: None,
                ..self.limits.clone()
            },
            DriveCurrent::Override(_) => self.limits.clone(),
        }
    }

    #[cfg(any(feature = "runtime", test))]
    pub(crate) fn motor_policy(&self) -> MotorParamsConfig {
        MotorParamsConfig {
            max_current: None,
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
    pub(crate) fn project(&self, config: &mut RuntimeConfig) {
        config.motor_params = Some(self.motor_policy());
        config.hall_calibration = self.hall();
        config.hall_calibration_invalid = false;
        config.current_limits = Some(self.current_policy());
        config.motor_feedback = Some(self.motor_feedback());
        config.motor_feedback_invalid = false;
    }

    pub fn thermal_current(&self) -> ElectricCurrent {
        self.thermal_current
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
    use crate::units::{amps, henries, ohms, quantity, watts, webers};
    fn candidate() -> DriveSetup {
        DriveSetup {
            motor: MotorParamsConfig {
                resistance: ohms(0.063),
                inductance_d: henries(31e-6),
                inductance_q: henries(51e-6),
                flux_linkage: webers(0.004),
                pole_pairs: 7,
                max_power_loss: Some(watts(120.0)),
                max_current: Some(amps(999.0)),
                ..Default::default()
            },
            feedback: Feedback::Sensorless,
            source: SourceLimits {
                input: amps(5.0),
                regen: amps(0.0),
            },
            current: DriveCurrent::Auto,
        }
    }
    #[test]
    fn auto_separates_motor_estimate_from_inverter_and_source() {
        let prepared = candidate().validate(amps(70.0), Some(amps(30.0))).unwrap();
        assert!(prepared.thermal_current() > quantity(35.0));
        assert_eq!(
            prepared.limits().max_iq,
            Some(amps(30.0 / OVERCURRENT_HEADROOM))
        );
        assert_eq!(prepared.limits().bus_in_max, Some(amps(5.0)));
        assert_eq!(prepared.limits().bus_regen_max, Some(amps(0.0)));
        assert_eq!(prepared.limits().max_phase_current, Some(amps(30.0)));
        assert_eq!(
            prepared.setup().motor.max_current,
            Some(prepared.thermal_current())
        );
    }
    #[test]
    fn override_cannot_raise_protection_or_motor_rating() {
        for a in [0.0, -1.0, f32::NAN, f32::INFINITY, 24.0] {
            let mut c = candidate();
            c.current = DriveCurrent::Override(amps(a));
            assert!(matches!(
                c.validate(amps(70.0), Some(amps(30.0))),
                Err(SetupError::Current)
            ));
        }
        let mut c = candidate();
        c.current = DriveCurrent::Override(amps(20.0));
        assert_eq!(
            c.validate(amps(70.0), Some(amps(30.0)))
                .unwrap()
                .limits()
                .max_iq,
            Some(amps(20.0))
        );
    }
    #[test]
    fn bad_source_model_and_feedback_are_rejected() {
        let mut c = candidate();
        c.source.regen.value = -1.0;
        assert!(matches!(
            c.validate(amps(70.0), Some(amps(30.0))),
            Err(SetupError::Source)
        ));
        let mut c = candidate();
        c.motor.flux_linkage.value = 0.0;
        assert!(matches!(
            c.validate(amps(70.0), Some(amps(30.0))),
            Err(SetupError::Motor)
        ));
        let mut c = candidate();
        let mut hall = HallCalibrationConfig::default();
        hall.valid[1] = false;
        c.feedback = Feedback::Hall(hall);
        assert!(matches!(
            c.validate(amps(70.0), Some(amps(30.0))),
            Err(SetupError::Hall)
        ));
        assert!(matches!(
            candidate().validate(amps(f32::NAN), Some(amps(30.0))),
            Err(SetupError::Protection)
        ));
    }
}
