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
use crate::units::{ElectricPotential, ElectricalAngularVelocity, Linear, Sign};
use uom::ConstZero;

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

/// A linear ramp between two thresholds. Which side is "full" depends on
/// the ramp; see the fields that use it.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct Ramp<T> {
    pub start: T,
    pub end: T,
}

/// Forward and reverse speed ceilings in rider coordinates.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct SpeedDerating {
    /// Forward speed ceiling (electrical); `None` = no ceiling.
    pub max_forward: Option<ElectricalAngularVelocity>,
    /// Reverse speed ceiling (electrical, a magnitude); `None` = no ceiling.
    pub max_reverse: Option<ElectricalAngularVelocity>,
    /// Fraction of each set speed ceiling at which motoring torque tapers.
    pub speed_start_frac: f32,
}

impl Default for SpeedDerating {
    fn default() -> Self {
        Self {
            max_forward: None,
            max_reverse: None,
            speed_start_frac: 0.8,
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
    /// FET temperature ramp (°C): full torque below `start`, none at `end`.
    /// The default end equals the board OverTemp fault threshold: the fault
    /// means the ramp failed. `None` = no FET ramp.
    pub fet_c: Option<Ramp<f32>>,
    /// Motor temperature ramp (°C), same shape; `None` = off (no NTC wired).
    pub motor_c: Option<Ramp<f32>>,
    /// Acceleration-derate factor 0..=1 (VESC `l_temp_accel_dec`): lerps
    /// the ACCEL copy of each temperature ramp toward 25 °C, so drive
    /// torque rolls off earlier than brake torque. 0 = symmetric.
    pub accel_dec: f32,
}

impl Default for TemperatureDerating {
    fn default() -> Self {
        Self {
            fet_c: Some(Ramp {
                start: 85.0,
                end: 100.0,
            }),
            motor_c: None,
            accel_dec: 0.15,
        }
    }
}

/// Voltage-dependent motoring and regeneration ramps.
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
pub struct BusVoltageDerating {
    /// Battery cutoff: full drive above `start`, none at or below `end`
    /// (`end < start`). `None` = off.
    pub cut: Option<Ramp<ElectricPotential>>,
    /// Regen overvoltage: full regen below `start`, none at or above `end`
    /// (`end > start`). `None` = off.
    pub regen: Option<Ramp<ElectricPotential>>,
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

impl DeratingConfig {
    /// Every set value finite and every set ramp well-formed (the config
    /// server also rejects insane writes loudly via this check). A set
    /// temperature or regen ramp starts above zero, a set cutoff ends above
    /// zero: those are the values the ISR encoding can represent.
    pub fn is_sane(&self) -> bool {
        let speed_ok = |v: Option<ElectricalAngularVelocity>| {
            v.is_none_or(|v| v.is_finite() && v.is_positive())
        };
        let temperature_ok = |ramp: Option<Ramp<f32>>| {
            ramp.is_none_or(|r| {
                r.start.is_finite() && r.end.is_finite() && r.start > 0.0 && r.end > r.start
            })
        };
        // Battery cutoff ramps DOWNWARD in voltage: start (full power)
        // above end (zero).
        let cut_ok = self.bus_voltage.cut.is_none_or(|r| {
            r.start.is_finite() && r.end.is_finite() && r.end.is_positive() && r.start > r.end
        });
        let regen_ok = self.bus_voltage.regen.is_none_or(|r| {
            r.start.is_finite() && r.end.is_finite() && r.start.is_positive() && r.end > r.start
        });
        speed_ok(self.speed.max_forward)
            && speed_ok(self.speed.max_reverse)
            && self.speed.speed_start_frac > 0.0
            && self.speed.speed_start_frac < 1.0
            && (0.0..=1.0).contains(&self.modulation.duty_start_frac)
            && (0.0..=1.0).contains(&self.temperature.accel_dec)
            && temperature_ok(self.temperature.fet_c)
            && temperature_ok(self.temperature.motor_c)
            && cut_ok
            && regen_ok
    }

    /// Compute the live scales. `None` measurements skip their ramps
    /// (a sensor the board does not carry must not derate anything).
    pub fn compute(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
        vbus: ElectricPotential,
    ) -> DeratingScales {
        PreparedDerating::new(self).compute(fet_temp_c, motor_temp_c, vbus)
    }
}

impl PreparedDerating {
    /// Encode a configuration that passed [`DeratingConfig::is_sane`].
    pub(crate) fn new(cfg: &DeratingConfig) -> Self {
        let start_c = |r: Option<Ramp<f32>>| r.map_or(0.0, |r| r.start);
        let end_c = |r: Option<Ramp<f32>>| r.map_or(0.0, |r| r.end);
        let start_v =
            |r: Option<Ramp<ElectricPotential>>| r.map_or(ElectricPotential::ZERO, |r| r.start);
        let end_v =
            |r: Option<Ramp<ElectricPotential>>| r.map_or(ElectricPotential::ZERO, |r| r.end);
        Self {
            speed: PreparedSpeed {
                max_forward: cfg
                    .speed
                    .max_forward
                    .unwrap_or(ElectricalAngularVelocity::ZERO),
                max_reverse: cfg
                    .speed
                    .max_reverse
                    .unwrap_or(ElectricalAngularVelocity::ZERO),
                speed_start_frac: cfg.speed.speed_start_frac,
            },
            modulation: cfg.modulation,
            temperature: PreparedTemperature {
                temp_fet_start_c: start_c(cfg.temperature.fet_c),
                temp_fet_end_c: end_c(cfg.temperature.fet_c),
                temp_motor_start_c: start_c(cfg.temperature.motor_c),
                temp_motor_end_c: end_c(cfg.temperature.motor_c),
                accel_dec: cfg.temperature.accel_dec,
            },
            bus_voltage: PreparedBusVoltage {
                cut_start: start_v(cfg.bus_voltage.cut),
                cut_end: end_v(cfg.bus_voltage.cut),
                regen_start: start_v(cfg.bus_voltage.regen),
                regen_end: end_v(cfg.bus_voltage.regen),
            },
        }
    }

    /// The configuration this encodes.
    pub(crate) fn config(&self) -> DeratingConfig {
        let speed = |v: ElectricalAngularVelocity| Some(v).filter(Sign::is_positive);
        let t = self.temperature;
        let b = self.bus_voltage;
        DeratingConfig {
            speed: SpeedDerating {
                max_forward: speed(self.speed.max_forward),
                max_reverse: speed(self.speed.max_reverse),
                speed_start_frac: self.speed.speed_start_frac,
            },
            modulation: self.modulation,
            temperature: TemperatureDerating {
                fet_c: (t.temp_fet_start_c > 0.0).then_some(Ramp {
                    start: t.temp_fet_start_c,
                    end: t.temp_fet_end_c,
                }),
                motor_c: (t.temp_motor_start_c > 0.0).then_some(Ramp {
                    start: t.temp_motor_start_c,
                    end: t.temp_motor_end_c,
                }),
                accel_dec: t.accel_dec,
            },
            bus_voltage: BusVoltageDerating {
                cut: b.cut_start.is_positive().then_some(Ramp {
                    start: b.cut_start,
                    end: b.cut_end,
                }),
                regen: b.regen_start.is_positive().then_some(Ramp {
                    start: b.regen_start,
                    end: b.regen_end,
                }),
            },
        }
    }
}

/// [`SpeedDerating`] in the per-cycle encoding: a zero ceiling = none.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PreparedSpeed {
    /// Forward speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_forward: ElectricalAngularVelocity,
    /// Reverse speed ceiling in electrical rad/s. Zero disables the ceiling.
    pub max_reverse: ElectricalAngularVelocity,
    /// Fraction of each enabled speed ceiling at which motoring torque tapers.
    pub speed_start_frac: f32,
}

impl PreparedSpeed {
    #[cfg(any(feature = "runtime", test))]
    fn is_sane(self) -> bool {
        let valid_limit = |v: ElectricalAngularVelocity| v.is_finite() && v.is_non_negative();
        valid_limit(self.max_forward)
            && valid_limit(self.max_reverse)
            && self.speed_start_frac > 0.0
            && self.speed_start_frac < 1.0
    }

    /// The speed ceiling for a rider-facing direction (its sign).
    pub(super) fn speed_ceiling(self, rider_direction: impl Sign) -> ElectricalAngularVelocity {
        if rider_direction.is_negative() {
            self.max_reverse
        } else {
            self.max_forward
        }
    }

    /// Evaluate on every FOC cycle. A free rotor can cross the taper
    /// between updates of the slower temperature and bus voltage scales.
    pub(super) fn speed_cut(self, omega: ElectricalAngularVelocity) -> f32 {
        let ceiling = self.speed_ceiling(omega);
        let speed = omega.abs();
        let start = ceiling * self.speed_start_frac;
        if ceiling == ElectricalAngularVelocity::ZERO {
            1.0
        } else if speed >= ceiling {
            0.0
        } else if speed <= start {
            1.0
        } else {
            (ceiling - speed).ratio_to(ceiling - start)
        }
    }
}

/// [`TemperatureDerating`] in the decimated-update encoding: a ramp with a
/// non-positive start is off.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PreparedTemperature {
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

/// [`BusVoltageDerating`] in the decimated-update encoding: a ramp with a
/// non-positive start is off.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct PreparedBusVoltage {
    /// Battery cutoff ramp start (V, full drive above); `0` disables
    pub cut_start: ElectricPotential,
    /// Battery cutoff ramp end (V, zero drive at/below; must be < start)
    pub cut_end: ElectricPotential,
    /// Regen overvoltage ramp start (V, full brake below); `0` disables
    pub regen_start: ElectricPotential,
    /// Regen overvoltage ramp end (V, zero regen at/above; must be > start)
    pub regen_end: ElectricPotential,
}

/// [`DeratingConfig`] as the control ISR evaluates it: the layout and checks
/// the configuration had before its "off" values became `Option`s. The
/// speed taper runs every FOC cycle, the ramps every decimated update;
/// converting outside the ISR keeps that code unchanged.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PreparedDerating {
    pub(crate) speed: PreparedSpeed,
    pub(crate) modulation: ModulationDerating,
    pub(crate) temperature: PreparedTemperature,
    pub(crate) bus_voltage: PreparedBusVoltage,
}

impl PreparedDerating {
    /// The checks [`DeratingConfig::is_sane`] had over this encoding, for
    /// the ISR's command boundary: all fields finite and every enabled ramp
    /// well-formed.
    ///
    /// Out of line like the shared `DeratingConfig::is_sane` it replaces in
    /// the ISR: as the only caller, `run_foc_cycle` would inline its ~550 B
    /// and push `angle_difference_general` out of line instead.
    #[cfg(any(feature = "runtime", test))]
    #[inline(never)]
    pub(crate) fn is_sane(&self) -> bool {
        let f = |v: f32| v.is_finite();
        let all_finite = f(self.temperature.temp_fet_start_c)
            && f(self.temperature.temp_fet_end_c)
            && f(self.temperature.temp_motor_start_c)
            && f(self.temperature.temp_motor_end_c)
            && f(self.temperature.accel_dec)
            && self.bus_voltage.cut_start.is_finite()
            && self.bus_voltage.cut_end.is_finite()
            && self.bus_voltage.regen_start.is_finite()
            && self.bus_voltage.regen_end.is_finite();
        if !all_finite {
            return false;
        }
        let fet_ok = self.temperature.temp_fet_start_c <= 0.0
            || self.temperature.temp_fet_end_c > self.temperature.temp_fet_start_c;
        let motor_ok = self.temperature.temp_motor_start_c <= 0.0
            || self.temperature.temp_motor_end_c > self.temperature.temp_motor_start_c;
        // Battery cutoff ramps DOWNWARD in voltage: start (full power)
        // above end (zero).
        let cut_ok = self.bus_voltage.cut_start.is_non_positive()
            || (self.bus_voltage.cut_end.is_positive()
                && self.bus_voltage.cut_start > self.bus_voltage.cut_end);
        let regen_ok = self.bus_voltage.regen_start.is_non_positive()
            || self.bus_voltage.regen_end > self.bus_voltage.regen_start;
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
    pub(crate) fn compute(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
        vbus: ElectricPotential,
    ) -> DeratingScales {
        let thermal = self.thermal_scales(fet_temp_c, motor_temp_c);
        let bus = self.bus_voltage_scales(vbus);
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

    pub(super) fn bus_voltage_scales(&self, vbus: ElectricPotential) -> DeratingScales {
        let mut drive = 1.0f32;
        let mut regen = 1.0f32;
        if self.bus_voltage.cut_start.is_positive()
            && self.bus_voltage.cut_start > self.bus_voltage.cut_end
        {
            // ramp_down mirrored in voltage: full above start, zero at end.
            drive = drive
                .min(1.0 - ramp_down(vbus, self.bus_voltage.cut_end, self.bus_voltage.cut_start));
        }
        if self.bus_voltage.regen_start.is_positive()
            && self.bus_voltage.regen_end > self.bus_voltage.regen_start
        {
            regen = ramp_down(
                vbus,
                self.bus_voltage.regen_start,
                self.bus_voltage.regen_end,
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
fn ramp_down<T>(x: T, start: T, end: T) -> f32
where
    T: Linear + PartialOrd + core::ops::Sub<Output = T>,
{
    if end <= start {
        return if x >= end { 0.0 } else { 1.0 };
    }
    clamp_f32((end - x).ratio_to(end - start), 0.0, 1.0)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * clamp_f32(t, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{erad_per_s, volts};

    fn cfg() -> DeratingConfig {
        DeratingConfig::default()
    }

    #[test]
    fn cold_board_no_derate() {
        let s = cfg().compute(Some(40.0), None, volts(24.0));
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn fet_ramp_derates_accel_before_brake() {
        // Mid-ramp on the brake curve (85→100): the accel copy (lerped
        // toward 25 °C) must already be lower.
        let s = cfg().compute(Some(92.0), None, volts(24.0));
        assert!(s.brake > 0.4 && s.brake < 0.7, "brake mid-ramp: {s:?}");
        assert!(s.drive < s.brake, "accel must derate earlier: {s:?}");

        // At the ramp end: both at zero — the OverTemp fault right past
        // this point means the ramp already failed.
        let s = cfg().compute(Some(100.0), None, volts(24.0));
        assert_eq!(s.brake, 0.0);
        assert_eq!(s.drive, 0.0);
    }

    #[test]
    fn missing_sensor_skips_ramp() {
        let s = cfg().compute(None, None, volts(24.0));
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn battery_cutoff_sheds_drive_keeps_brake() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                cut: Some(Ramp {
                    start: volts(40.0),
                    end: volts(36.0),
                }),
                ..Default::default()
            },
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, volts(38.0));
        assert!((s.drive - 0.5).abs() < 1e-3, "mid-cut: {s:?}");
        assert_eq!(s.brake, 1.0, "sag must never cost braking");
        let s = c.compute(Some(40.0), None, volts(35.0));
        assert_eq!(s.drive, 0.0, "below cut end: no drive");
    }

    #[test]
    fn regen_ov_sheds_regen_only() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                regen: Some(Ramp {
                    start: volts(56.0),
                    end: volts(58.0),
                }),
                ..Default::default()
            },
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, volts(57.0));
        assert!((s.regen - 0.5).abs() < 1e-3, "mid-regen-cut: {s:?}");
        assert_eq!((s.drive, s.brake), (1.0, 1.0));
        let s = c.compute(Some(40.0), None, volts(58.5));
        assert_eq!(s.regen, 0.0, "full pack: no regen");
        assert_eq!(s.worst(), 0.0, "and the rider is warned");
    }

    #[test]
    fn ramps_compose_via_min() {
        let c = DeratingConfig {
            bus_voltage: BusVoltageDerating {
                cut: Some(Ramp {
                    start: volts(40.0),
                    end: volts(36.0),
                }),
                ..Default::default()
            },
            ..cfg()
        };
        // Hot + sagging: drive takes the tighter of the two ramps.
        let s = c.compute(Some(92.0), None, volts(38.0));
        let thermal_only = c.compute(Some(92.0), None, volts(50.0));
        assert!(s.drive <= thermal_only.drive.min(0.5));
    }

    #[test]
    fn sanity_checks_reject_malformed_ramps() {
        let with_temperature = |temperature| DeratingConfig {
            temperature,
            ..cfg()
        };
        let fet = |start, end| TemperatureDerating {
            fet_c: Some(Ramp { start, end }),
            ..Default::default()
        };
        assert!(cfg().is_sane());
        assert!(
            !with_temperature(fet(85.0, 80.0)).is_sane(),
            "end below start"
        );
        assert!(!with_temperature(fet(f32::NAN, 100.0)).is_sane());
        assert!(
            !with_temperature(fet(0.0, 100.0)).is_sane(),
            "a set ramp starts above zero; `None` turns it off"
        );
        assert!(
            !with_temperature(TemperatureDerating {
                accel_dec: 1.5,
                ..Default::default()
            })
            .is_sane()
        );
        let with_bus = |bus_voltage| DeratingConfig {
            bus_voltage,
            ..cfg()
        };
        assert!(
            !with_bus(BusVoltageDerating {
                cut: Some(Ramp {
                    start: volts(36.0),
                    end: volts(40.0), // cutoff must ramp downward
                }),
                ..Default::default()
            })
            .is_sane()
        );
        assert!(
            !with_bus(BusVoltageDerating {
                regen: Some(Ramp {
                    start: volts(58.0),
                    end: volts(56.0),
                }),
                ..Default::default()
            })
            .is_sane()
        );
        // Off ramps need no thresholds at all.
        assert!(
            with_temperature(TemperatureDerating {
                fet_c: None,
                ..Default::default()
            })
            .is_sane()
        );
    }

    #[test]
    fn the_isr_encoding_round_trips_every_sane_config() {
        let configs = [
            cfg(),
            DeratingConfig {
                speed: SpeedDerating {
                    max_forward: Some(erad_per_s(400.0)),
                    max_reverse: None,
                    speed_start_frac: 0.7,
                },
                temperature: TemperatureDerating {
                    fet_c: None,
                    motor_c: Some(Ramp {
                        start: 90.0,
                        end: 120.0,
                    }),
                    accel_dec: 0.0,
                },
                bus_voltage: BusVoltageDerating {
                    cut: Some(Ramp {
                        start: volts(40.0),
                        end: volts(36.0),
                    }),
                    regen: Some(Ramp {
                        start: volts(56.0),
                        end: volts(58.0),
                    }),
                },
                ..cfg()
            },
        ];
        for config in configs {
            assert!(config.is_sane());
            let prepared = PreparedDerating::new(&config);
            assert!(prepared.is_sane());
            assert_eq!(prepared.config(), config);
        }
    }
}

#[cfg(test)]
mod speed_tests {
    use super::*;
    use crate::units::erad_per_s;

    fn with_speed(speed: SpeedDerating) -> DeratingConfig {
        DeratingConfig {
            speed,
            ..DeratingConfig::default()
        }
    }

    #[test]
    fn speed_taper_can_be_enabled_in_only_one_direction() {
        let config = with_speed(SpeedDerating {
            max_forward: Some(erad_per_s(100.0)),
            ..SpeedDerating::default()
        });
        assert!(config.is_sane());
        let speed = PreparedDerating::new(&config).speed;
        assert_eq!(speed.speed_cut(erad_per_s(80.0)), 1.0);
        assert_eq!(speed.speed_cut(erad_per_s(90.0)), 0.5);
        assert_eq!(speed.speed_cut(erad_per_s(100.0)), 0.0);
        assert_eq!(speed.speed_cut(erad_per_s(-200.0)), 1.0);
    }

    #[test]
    fn invalid_speed_taper_or_ceiling_is_rejected() {
        let mut speed = SpeedDerating::default();
        for start in [0.0, 1.0, f32::NAN] {
            speed.speed_start_frac = start;
            assert!(!with_speed(speed).is_sane());
        }
        speed.speed_start_frac = 0.8;
        for ceiling in [0.0, -1.0, f32::INFINITY, f32::NAN] {
            speed.max_reverse = Some(erad_per_s(ceiling));
            assert!(!with_speed(speed).is_sane(), "{ceiling}");
        }
    }
}
