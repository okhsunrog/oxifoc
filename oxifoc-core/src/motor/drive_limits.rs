#[cfg(feature = "runtime")]
use crate::types::ControlMode;

/// Direction and limits in the rider's coordinate system.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DriveLimitsConfig {
    pub invert_direction: bool,
    /// Electrical rad/s. None inherits the legacy ceiling, Some(0) disables it.
    pub max_forward_erad_s: Option<f32>,
    pub max_reverse_erad_s: Option<f32>,
    /// Fraction of available modulation where torque starts to taper.
    /// One disables the taper.
    pub duty_start_frac: f32,
}

impl DriveLimitsConfig {
    pub const DEFAULT: Self = Self {
        invert_direction: false,
        max_forward_erad_s: None,
        max_reverse_erad_s: None,
        duty_start_frac: 1.0,
    };

    pub const E_SKATE_DUTY_START: f32 = 0.85;

    pub fn is_sane(self) -> bool {
        let valid_limit = |v: f32| v.is_finite() && v >= 0.0;
        self.max_forward_erad_s.is_none_or(valid_limit)
            && self.max_reverse_erad_s.is_none_or(valid_limit)
            && (0.0..=1.0).contains(&self.duty_start_frac)
    }

    pub(super) fn ceiling(self, physical_direction: f32, legacy: f32) -> f32 {
        let reverse = (physical_direction < 0.0) ^ self.invert_direction;
        if reverse {
            self.max_reverse_erad_s
        } else {
            self.max_forward_erad_s
        }
        .unwrap_or(legacy)
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

impl Default for DriveLimitsConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}
