#[cfg(feature = "runtime")]
use crate::types::ControlMode;

/// Drive behavior in the rider's coordinate system.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct DriveConfig {
    /// Reverse the mapping from rider commands to motor coordinates.
    pub invert_direction: bool,
}

impl DriveConfig {
    pub const DEFAULT: Self = Self {
        invert_direction: false,
    };

    pub(super) fn rider_sign(self, value: f32) -> f32 {
        if self.invert_direction { -value } else { value }
    }

    /// Convert a rider-facing command once, at the driver boundary.
    #[cfg(feature = "runtime")]
    pub(crate) fn orient(self, mode: ControlMode) -> ControlMode {
        if !self.invert_direction {
            return mode;
        }
        match mode {
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => ControlMode::CurrentControl {
                iq_target: -iq_target,
                id_target,
            },
            ControlMode::Throttle { fraction } => ControlMode::Throttle {
                fraction: -fraction,
            },
            ControlMode::VelocityControl { target_vel } => ControlMode::VelocityControl {
                target_vel: -target_vel,
            },
            ControlMode::PositionControl { target_pos } => ControlMode::PositionControl {
                target_pos: -target_pos,
            },
            diagnostic_or_neutral => diagnostic_or_neutral,
        }
    }
}

impl Default for DriveConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}
