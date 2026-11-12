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
    /// Forward speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_forward_erad_s: f32,
    /// Reverse speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_reverse_erad_s: f32,
    /// Fraction of each enabled speed ceiling at which motoring torque tapers.
    pub speed_start_frac: f32,
    /// Fraction of available modulation where torque starts to taper.
    /// One disables the taper.
    pub duty_start_frac: f32,
}

impl DriveConfig {
    pub const DEFAULT: Self = Self {
        invert_direction: false,
        max_forward_erad_s: 0.0,
        max_reverse_erad_s: 0.0,
        speed_start_frac: 0.8,
        duty_start_frac: 1.0,
    };

    pub const E_SKATE_DUTY_START: f32 = 0.85;

    pub fn is_sane(self) -> bool {
        let valid_limit = |v: f32| v.is_finite() && v >= 0.0;
        valid_limit(self.max_forward_erad_s)
            && valid_limit(self.max_reverse_erad_s)
            && self.speed_start_frac > 0.0
            && self.speed_start_frac < 1.0
            && (0.0..=1.0).contains(&self.duty_start_frac)
    }

    pub(super) fn speed_ceiling(self, physical_direction: f32) -> f32 {
        let reverse = (physical_direction < 0.0) ^ self.invert_direction;
        if reverse {
            self.max_reverse_erad_s
        } else {
            self.max_forward_erad_s
        }
    }

    /// Evaluate on every FOC cycle. A free rotor can cross the taper
    /// between updates of the slower temperature and bus voltage scales.
    pub(super) fn speed_cut(self, omega: f32) -> f32 {
        let ceiling = self.speed_ceiling(omega);
        let speed = omega.abs();
        let start = ceiling * self.speed_start_frac;
        if ceiling == 0.0 {
            1.0
        } else if speed >= ceiling {
            0.0
        } else if speed <= start {
            1.0
        } else {
            (ceiling - speed) / (ceiling - start)
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_taper_can_be_enabled_in_only_one_direction() {
        let drive = DriveConfig {
            max_forward_erad_s: 100.0,
            ..DriveConfig::DEFAULT
        };
        assert!(drive.is_sane());
        assert_eq!(drive.speed_cut(80.0), 1.0);
        assert_eq!(drive.speed_cut(90.0), 0.5);
        assert_eq!(drive.speed_cut(100.0), 0.0);
        assert_eq!(drive.speed_cut(-200.0), 1.0);
    }

    #[test]
    fn invalid_speed_taper_or_ceiling_is_rejected() {
        let mut drive = DriveConfig::DEFAULT;
        for start in [0.0, 1.0, f32::NAN] {
            drive.speed_start_frac = start;
            assert!(!drive.is_sane());
        }
        drive.speed_start_frac = 0.8;
        for ceiling in [-1.0, f32::INFINITY, f32::NAN] {
            drive.max_reverse_erad_s = ceiling;
            assert!(!drive.is_sane());
        }
    }
}
