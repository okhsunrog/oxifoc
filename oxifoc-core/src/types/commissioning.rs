//! Motor classes and immutable commissioning conditions shared by every client.
use super::DetectRequest;
use crate::units::{ElectricCurrent, ElectricalAngularVelocity, Power, Sign, erpm, watts};

#[derive(
    Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
pub enum MotorClass {
    MiniOutrunner,
    SmallOutrunner,
    MediumOutrunner,
    LargeOutrunner,
    SmallInrunner,
    MediumInrunner,
    LargeInrunner,
    Hub,
    SmallEdf,
}
impl MotorClass {
    pub fn preset(self) -> &'static MotorPreset {
        &PRESETS[self as usize]
    }
}

/// Wire input. Validate after decoding; zero regeneration is an explicit value.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SourceLimits {
    pub input: ElectricCurrent,
    pub regen: ElectricCurrent,
}
impl SourceLimits {
    pub fn is_valid(self) -> bool {
        self.input.is_finite()
            && self.input.is_positive()
            && self.regen.is_finite()
            && self.regen.is_non_negative()
    }
}

#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
pub struct CommissioningPlan {
    pub source: SourceLimits,
    pub motor_class: MotorClass,
    pub pole_pairs: u8,
    pub flux_target_speed: ElectricalAngularVelocity,
    pub current_limit: Option<ElectricCurrent>,
    pub calibrate_hall: bool,
}
impl CommissioningPlan {
    pub fn is_valid(self) -> bool {
        self.source.is_valid()
            && self.pole_pairs > 0
            && self.flux_target_speed.is_finite()
            && self.flux_target_speed.is_positive()
            && self
                .current_limit
                .is_none_or(|v| v.is_finite() && v.value >= 0.5)
    }
    /// Stage requests may supply freshly measured prerequisites, but cannot
    /// silently change the conditions confirmed when the session was opened.
    pub fn allows(self, request: DetectRequest) -> bool {
        let loss = self.motor_class.preset().max_power_loss;
        match request {
            DetectRequest::MeasureResistance {
                max_power_loss,
                current_limit,
            }
            | DetectRequest::MeasureInductance {
                max_power_loss,
                current_limit,
                ..
            } => max_power_loss == loss && current_limit == self.current_limit,
            DetectRequest::MeasureFlux {
                max_power_loss,
                current_limit,
                pole_pairs,
                openloop_speed,
                ..
            } => {
                max_power_loss == loss
                    && current_limit == self.current_limit
                    && pole_pairs == self.pole_pairs
                    && openloop_speed == self.flux_target_speed
            }
            DetectRequest::CalibrateHall {
                max_power_loss,
                current_limit,
                ..
            } => {
                self.calibrate_hall && max_power_loss == loss && current_limit == self.current_limit
            }
            DetectRequest::MeasureCurrentOffsets { .. } => false,
        }
    }
}

/// Motor preset for pre-filling detection parameters.
///
/// Values match VESC Tool's FOC detection dialog presets.
#[derive(Clone, Copy, Debug)]
pub struct MotorPreset {
    pub class: MotorClass,
    pub id: &'static str,
    pub name: &'static str,
    /// Number of magnetic pole pairs (poles / 2)
    pub pole_pairs: u8,
    /// Maximum power dissipation during detection
    pub max_power_loss: Power,
    /// Open-loop speed for sensorless startup
    pub openloop_speed: ElectricalAngularVelocity,
    /// Speed threshold for sensorless transition
    pub sensorless_speed: ElectricalAngularVelocity,
}

pub static PRESETS: &[MotorPreset] = &[
    MotorPreset {
        class: MotorClass::MiniOutrunner,
        id: "mini-outrunner",
        name: "Mini Outrunner (~75g)",
        pole_pairs: 7,
        max_power_loss: watts(20.0),
        openloop_speed: erpm(1400.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::SmallOutrunner,
        id: "small-outrunner",
        name: "Small Outrunner (~200g)",
        pole_pairs: 7,
        max_power_loss: watts(50.0),
        openloop_speed: erpm(1400.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::MediumOutrunner,
        id: "medium-outrunner",
        name: "Medium Outrunner (~750g)",
        pole_pairs: 7,
        max_power_loss: watts(120.0),
        openloop_speed: erpm(700.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::LargeOutrunner,
        id: "large-outrunner",
        name: "Large Outrunner (~2kg)",
        pole_pairs: 7,
        max_power_loss: watts(400.0),
        openloop_speed: erpm(700.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::SmallInrunner,
        id: "small-inrunner",
        name: "Small Inrunner (~200g)",
        pole_pairs: 1,
        max_power_loss: watts(50.0),
        openloop_speed: erpm(1400.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::MediumInrunner,
        id: "medium-inrunner",
        name: "Medium Inrunner (~750g)",
        pole_pairs: 2,
        max_power_loss: watts(140.0),
        openloop_speed: erpm(1400.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::LargeInrunner,
        id: "large-inrunner",
        name: "Large Inrunner (~2kg)",
        pole_pairs: 2,
        max_power_loss: watts(400.0),
        openloop_speed: erpm(1000.0),
        sensorless_speed: erpm(4000.0),
    },
    MotorPreset {
        class: MotorClass::Hub,
        id: "hub",
        name: "E-Bike DD Hub Motor",
        pole_pairs: 23,
        max_power_loss: watts(150.0),
        openloop_speed: erpm(300.0),
        sensorless_speed: erpm(2000.0),
    },
    MotorPreset {
        class: MotorClass::SmallEdf,
        id: "small-edf",
        name: "EDF Inrunner Small (~200g)",
        pole_pairs: 3,
        max_power_loss: watts(110.0),
        openloop_speed: erpm(1400.0),
        sensorless_speed: erpm(4000.0),
    },
];
