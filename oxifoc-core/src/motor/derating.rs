//! Graduated derating: continuous power rolloff BEFORE any fault.
//!
//! VESC's `update_override_limits` lesson (docs/notes/fault-overhaul.md):
//! temperature and bus voltage ramps reduce the available torque before a
//! fault. This module computes three live scale factors:
//!
//! - `drive` scales torque in the direction of motion (acceleration),
//! - `brake` scales torque opposing it,
//! - `regen` scales the share of the brake's regen the supply takes,
//!
//! each the `min`-composition of independent linear ramps:
//!
//! | Ramp | Side | Shape |
//! |---|---|---|
//! | FET temperature | drive + brake, accel earlier | 1 below `start`, 0 at `end`; the accel copy has start/end lerped toward 25 °C by `accel_dec` (VESC `l_temp_accel_dec`) — a hot board loses acceleration first and keeps braking |
//! | Motor temperature | drive + brake, accel earlier | same shape, separate thresholds (off when no NTC) |
//! | Battery cutoff | drive | 1 above `vbus_cut_start`, 0 at `vbus_cut_end` (start > end) — a sagging pack sheds drive current before the UV fault can trip |
//! | Regen overvoltage | regen | 1 below `vbus_regen_start`, 0 at `vbus_regen_end` — braking into a full pack sheds regen before the OV fault |
//!
//! `regen` is not a torque scale by itself: the driver either burns the
//! refused share in the windings (dissipative braking,
//! `motor::dissipation`) or, with that off, scales the brake torque by it.
//!
//! The scales are computed ISR-side (decimated — temperatures and bus
//! voltage are slow), so the protection survives an async-executor hang,
//! same philosophy as the Layer-2 deadman. The faults of the severity
//! ladder remain the backstops at the ramp ends.

use crate::foc::clamp_f32;

/// Live derating factors (1.0 = no derate), applied by the driver's
/// torque grant (`FocDriver::grant_torque`), selected by direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeratingScales {
    /// Torque in the direction of motion (acceleration), 0..=1
    pub drive: f32,
    /// Torque opposing motion (braking), 0..=1
    pub brake: f32,
    /// Share of the brake's regen the supply takes, 0..=1
    pub regen: f32,
}

impl Default for DeratingScales {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl DeratingScales {
    /// No derate (also usable in const initializers, unlike `default()`).
    pub const IDENTITY: Self = Self {
        drive: 1.0,
        brake: 1.0,
        regen: 1.0,
    };

    /// The strongest derate (for the Warning threshold) — `regen`
    /// included: burning refused regen heats the motor, the rider should
    /// know the pack is full.
    pub fn worst(&self) -> f32 {
        self.drive.min(self.brake).min(self.regen)
    }
}

/// Forward and reverse speed ceilings in rider coordinates.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct SpeedDerating {
    /// Forward speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_forward_erad_s: f32,
    /// Reverse speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_reverse_erad_s: f32,
    /// Fraction of each enabled speed ceiling at which motoring torque tapers.
    pub speed_start_frac: f32,
}

impl Default for SpeedDerating {
    fn default() -> Self {
        Self {
            max_forward_erad_s: 0.0,
            max_reverse_erad_s: 0.0,
            speed_start_frac: 0.8,
        }
    }
}

impl SpeedDerating {
    fn is_sane(self) -> bool {
        let valid_limit = |v: f32| v.is_finite() && v >= 0.0;
        valid_limit(self.max_forward_erad_s)
            && valid_limit(self.max_reverse_erad_s)
            && self.speed_start_frac > 0.0
            && self.speed_start_frac < 1.0
    }

    pub(super) fn speed_ceiling(self, rider_direction: f32) -> f32 {
        if rider_direction < 0.0 {
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
}

/// Motoring taper as the voltage vector approaches the modulation limit.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct ModulationDerating {
    /// Fraction of available modulation where torque starts to taper.
    /// One disables the taper.
    pub duty_start_frac: f32,
}

impl ModulationDerating {
    pub const E_SKATE_DUTY_START: f32 = 0.85;
}

impl Default for ModulationDerating {
    fn default() -> Self {
        Self {
            duty_start_frac: 1.0,
        }
    }
}

/// Temperature-dependent drive and braking ramps.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct TemperatureDerating {
    /// FET temperature ramp start (°C); `0` disables the FET ramp
    pub temp_fet_start_c: f32,
    /// FET temperature ramp end (°C, scale = 0 here). Default equals the
    /// board OverTemp fault threshold: the fault means the ramp failed.
    pub temp_fet_end_c: f32,
    /// Motor temperature ramp start (°C); `0` disables (no NTC wired)
    pub temp_motor_start_c: f32,
    /// Motor temperature ramp end (°C)
    pub temp_motor_end_c: f32,
    /// Acceleration-derate factor 0..=1 (VESC `l_temp_accel_dec`): lerps
    /// the ACCEL copy of each temperature ramp toward 25 °C, so drive
    /// torque rolls off earlier than brake torque. 0 = symmetric.
    pub accel_dec: f32,
}

/// Voltage-dependent motoring and regeneration ramps.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct BusVoltageDerating {
    /// Battery cutoff ramp start (V, full drive above); `0` disables
    pub vbus_cut_start_v: f32,
    /// Battery cutoff ramp end (V, zero drive at/below; must be < start)
    pub vbus_cut_end_v: f32,
    /// Regen overvoltage ramp start (V, full brake below); `0` disables
    pub vbus_regen_start_v: f32,
    /// Regen overvoltage ramp end (V, zero regen at/above; must be > start)
    pub vbus_regen_end_v: f32,
}

/// Speed, modulation, temperature and bus voltage derating.
#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    PartialEq,
    serde::Serialize,
    serde::Deserialize,
    postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct DeratingConfig {
    pub speed: SpeedDerating,
    pub modulation: ModulationDerating,
    pub temperature: TemperatureDerating,
    pub bus_voltage: BusVoltageDerating,
}

impl Default for TemperatureDerating {
    fn default() -> Self {
        Self {
            temp_fet_start_c: 85.0,
            temp_fet_end_c: 100.0,
            temp_motor_start_c: 0.0,
            temp_motor_end_c: 0.0,
            accel_dec: 0.15,
        }
    }
}

impl Default for BusVoltageDerating {
    fn default() -> Self {
        Self {
            vbus_cut_start_v: 0.0,
            vbus_cut_end_v: 0.0,
            vbus_regen_start_v: 0.0,
            vbus_regen_end_v: 0.0,
        }
    }
}

impl DeratingConfig {
    /// All fields finite and every ENABLED ramp well-formed (the config
    /// server also rejects insane writes loudly via this check).
    pub fn is_sane(&self) -> bool {
        let f = |v: f32| v.is_finite();
        let all_finite = f(self.temperature.temp_fet_start_c)
            && f(self.temperature.temp_fet_end_c)
            && f(self.temperature.temp_motor_start_c)
            && f(self.temperature.temp_motor_end_c)
            && f(self.temperature.accel_dec)
            && f(self.bus_voltage.vbus_cut_start_v)
            && f(self.bus_voltage.vbus_cut_end_v)
            && f(self.bus_voltage.vbus_regen_start_v)
            && f(self.bus_voltage.vbus_regen_end_v);
        if !all_finite {
            return false;
        }
        let fet_ok = self.temperature.temp_fet_start_c <= 0.0
            || self.temperature.temp_fet_end_c > self.temperature.temp_fet_start_c;
        let motor_ok = self.temperature.temp_motor_start_c <= 0.0
            || self.temperature.temp_motor_end_c > self.temperature.temp_motor_start_c;
        // Battery cutoff ramps DOWNWARD in voltage: start (full power)
        // above end (zero).
        let cut_ok = self.bus_voltage.vbus_cut_start_v <= 0.0
            || (self.bus_voltage.vbus_cut_end_v > 0.0
                && self.bus_voltage.vbus_cut_start_v > self.bus_voltage.vbus_cut_end_v);
        let regen_ok = self.bus_voltage.vbus_regen_start_v <= 0.0
            || self.bus_voltage.vbus_regen_end_v > self.bus_voltage.vbus_regen_start_v;
        self.speed.is_sane()
            && (0.0..=1.0).contains(&self.modulation.duty_start_frac)
            && (0.0..=1.0).contains(&self.temperature.accel_dec)
            && fet_ok
            && motor_ok
            && cut_ok
            && regen_ok
    }

    /// Compute the live scales. `None` measurements skip their ramps
    /// (a sensor the board does not carry must not derate anything).
    pub fn compute(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
        vbus_v: f32,
    ) -> DeratingScales {
        let thermal = self.thermal_scales(fet_temp_c, motor_temp_c);
        let bus = self.bus_voltage_scales(vbus_v);
        DeratingScales {
            drive: thermal.drive.min(bus.drive),
            brake: thermal.brake,
            regen: bus.regen,
        }
    }

    pub(super) fn thermal_scales(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
    ) -> DeratingScales {
        let mut drive = 1.0f32;
        let mut brake = 1.0f32;

        let mut thermal = |temp: Option<f32>, start: f32, end: f32| {
            let Some(t) = temp else { return };
            if start <= 0.0 || end <= start {
                return;
            }
            // brake follows the configured ramp...
            let b = ramp_down(t, start, end);
            // ...the accel copy is lerped toward 25 °C by accel_dec, so it
            // always sits at or below the brake ramp (VESC formula).
            let a_start = lerp(start, 25.0, self.temperature.accel_dec);
            let a_end = lerp(end, 25.0, self.temperature.accel_dec);
            let d = ramp_down(t, a_start, a_end);
            brake = brake.min(b);
            drive = drive.min(d.min(b));
        };
        thermal(
            fet_temp_c,
            self.temperature.temp_fet_start_c,
            self.temperature.temp_fet_end_c,
        );
        thermal(
            motor_temp_c,
            self.temperature.temp_motor_start_c,
            self.temperature.temp_motor_end_c,
        );

        DeratingScales {
            drive,
            brake,
            regen: 1.0,
        }
    }

    pub(super) fn bus_voltage_scales(&self, vbus_v: f32) -> DeratingScales {
        let mut drive = 1.0f32;
        let mut regen = 1.0f32;
        if self.bus_voltage.vbus_cut_start_v > 0.0
            && self.bus_voltage.vbus_cut_start_v > self.bus_voltage.vbus_cut_end_v
        {
            // ramp_down mirrored in voltage: full above start, zero at end.
            drive = drive.min(
                1.0 - ramp_down(
                    vbus_v,
                    self.bus_voltage.vbus_cut_end_v,
                    self.bus_voltage.vbus_cut_start_v,
                ),
            );
        }
        if self.bus_voltage.vbus_regen_start_v > 0.0
            && self.bus_voltage.vbus_regen_end_v > self.bus_voltage.vbus_regen_start_v
        {
            regen = ramp_down(
                vbus_v,
                self.bus_voltage.vbus_regen_start_v,
                self.bus_voltage.vbus_regen_end_v,
            );
        }

        DeratingScales {
            drive,
            brake: 1.0,
            regen,
        }
    }
}

/// 1.0 at/below `start`, 0.0 at/above `end`, linear between.
fn ramp_down(x: f32, start: f32, end: f32) -> f32 {
    if end <= start {
        return if x >= end { 0.0 } else { 1.0 };
    }
    clamp_f32((end - x) / (end - start), 0.0, 1.0)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * clamp_f32(t, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> DeratingConfig {
        DeratingConfig::default()
    }

    #[test]
    fn cold_board_no_derate() {
        let s = cfg().compute(Some(40.0), None, 24.0);
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn fet_ramp_derates_accel_before_brake() {
        // Mid-ramp on the brake curve (85→100): the accel copy (lerped
        // toward 25 °C) must already be lower.
        let s = cfg().compute(Some(92.0), None, 24.0);
        assert!(s.brake > 0.4 && s.brake < 0.7, "brake mid-ramp: {s:?}");
        assert!(s.drive < s.brake, "accel must derate earlier: {s:?}");

        // At the ramp end: both at zero — the OverTemp fault right past
        // this point means the ramp already failed.
        let s = cfg().compute(Some(100.0), None, 24.0);
        assert_eq!(s.brake, 0.0);
        assert_eq!(s.drive, 0.0);
    }

    #[test]
    fn missing_sensor_skips_ramp() {
        let s = cfg().compute(None, None, 24.0);
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn battery_cutoff_sheds_drive_keeps_brake() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                vbus_cut_start_v: 40.0,
                vbus_cut_end_v: 36.0,
                ..Default::default()
            },
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, 38.0);
        assert!((s.drive - 0.5).abs() < 1e-3, "mid-cut: {s:?}");
        assert_eq!(s.brake, 1.0, "sag must never cost braking");
        let s = c.compute(Some(40.0), None, 35.0);
        assert_eq!(s.drive, 0.0, "below cut end: no drive");
    }

    #[test]
    fn regen_ov_sheds_regen_only() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                vbus_regen_start_v: 56.0,
                vbus_regen_end_v: 58.0,
                ..Default::default()
            },
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, 57.0);
        assert!((s.regen - 0.5).abs() < 1e-3, "mid-regen-cut: {s:?}");
        assert_eq!((s.drive, s.brake), (1.0, 1.0));
        let s = c.compute(Some(40.0), None, 58.5);
        assert_eq!(s.regen, 0.0, "full pack: no regen");
        assert_eq!(s.worst(), 0.0, "and the rider is warned");
    }

    #[test]
    fn ramps_compose_via_min() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                vbus_cut_start_v: 40.0,
                vbus_cut_end_v: 36.0,
                ..Default::default()
            },
            ..cfg()
        };
        // Hot + sagging: drive takes the tighter of the two ramps.
        let s = c.compute(Some(92.0), None, 38.0);
        let thermal_only = c.compute(Some(92.0), None, 50.0);
        assert!(s.drive <= thermal_only.drive.min(0.5));
    }

    #[test]
    fn sanity_checks_reject_malformed_ramps() {
        assert!(cfg().is_sane());
        assert!(
            !DeratingConfig {
                temperature: TemperatureDerating {
                    temp_fet_end_c: 80.0, // end below start
                    ..Default::default()
                },
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                bus_voltage: BusVoltageDerating {
                    vbus_cut_start_v: 36.0,
                    vbus_cut_end_v: 40.0, // cutoff must ramp downward
                    ..Default::default()
                },
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                temperature: TemperatureDerating {
                    accel_dec: 1.5,
                    ..Default::default()
                },
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                temperature: TemperatureDerating {
                    temp_fet_start_c: f32::NAN,
                    ..Default::default()
                },
                ..cfg()
            }
            .is_sane()
        );
        // Disabled ramps don't need well-formed pairs.
        assert!(
            DeratingConfig {
                temperature: TemperatureDerating {
                    temp_fet_start_c: 0.0,
                    temp_fet_end_c: 0.0,
                    ..Default::default()
                },
                ..cfg()
            }
            .is_sane()
        );
    }
}

#[cfg(test)]
mod speed_tests {
    use super::*;

    #[test]
    fn speed_taper_can_be_enabled_in_only_one_direction() {
        let speed = SpeedDerating {
            max_forward_erad_s: 100.0,
            ..SpeedDerating::default()
        };
        assert!(speed.is_sane());
        assert_eq!(speed.speed_cut(80.0), 1.0);
        assert_eq!(speed.speed_cut(90.0), 0.5);
        assert_eq!(speed.speed_cut(100.0), 0.0);
        assert_eq!(speed.speed_cut(-200.0), 1.0);
    }

    #[test]
    fn invalid_speed_taper_or_ceiling_is_rejected() {
        let mut speed = SpeedDerating::default();
        for start in [0.0, 1.0, f32::NAN] {
            speed.speed_start_frac = start;
            assert!(!speed.is_sane());
        }
        speed.speed_start_frac = 0.8;
        for ceiling in [-1.0, f32::INFINITY, f32::NAN] {
            speed.max_reverse_erad_s = ceiling;
            assert!(!speed.is_sane());
        }
    }
}
