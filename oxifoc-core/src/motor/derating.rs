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

/// Temperature and bus voltage ramps. A zero start disables that ramp.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct DeratingConfig {
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
    /// Battery cutoff ramp start (V, full drive above); `0` disables
    pub vbus_cut_start_v: f32,
    /// Battery cutoff ramp end (V, zero drive at/below; must be < start)
    pub vbus_cut_end_v: f32,
    /// Regen overvoltage ramp start (V, full brake below); `0` disables
    pub vbus_regen_start_v: f32,
    /// Regen overvoltage ramp end (V, zero regen at/above; must be > start)
    pub vbus_regen_end_v: f32,
}

impl Default for DeratingConfig {
    /// FET thermal protection on (85→100 °C, matching the g431 board
    /// fault threshold), VESC-default accel asymmetry, everything that
    /// needs per-battery/per-vehicle numbers off.
    fn default() -> Self {
        Self {
            temp_fet_start_c: 85.0,
            temp_fet_end_c: 100.0,
            temp_motor_start_c: 0.0,
            temp_motor_end_c: 0.0,
            accel_dec: 0.15,
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
        let all_finite = f(self.temp_fet_start_c)
            && f(self.temp_fet_end_c)
            && f(self.temp_motor_start_c)
            && f(self.temp_motor_end_c)
            && f(self.accel_dec)
            && f(self.vbus_cut_start_v)
            && f(self.vbus_cut_end_v)
            && f(self.vbus_regen_start_v)
            && f(self.vbus_regen_end_v);
        if !all_finite {
            return false;
        }
        let fet_ok = self.temp_fet_start_c <= 0.0 || self.temp_fet_end_c > self.temp_fet_start_c;
        let motor_ok =
            self.temp_motor_start_c <= 0.0 || self.temp_motor_end_c > self.temp_motor_start_c;
        // Battery cutoff ramps DOWNWARD in voltage: start (full power)
        // above end (zero).
        let cut_ok = self.vbus_cut_start_v <= 0.0
            || (self.vbus_cut_end_v > 0.0 && self.vbus_cut_start_v > self.vbus_cut_end_v);
        let regen_ok =
            self.vbus_regen_start_v <= 0.0 || self.vbus_regen_end_v > self.vbus_regen_start_v;
        (0.0..=1.0).contains(&self.accel_dec) && fet_ok && motor_ok && cut_ok && regen_ok
    }

    /// Compute the live scales. `None` measurements skip their ramps
    /// (a sensor the board does not carry must not derate anything).
    pub fn compute(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
        vbus_v: f32,
    ) -> DeratingScales {
        let mut drive = 1.0f32;
        let mut brake = 1.0f32;
        let mut regen = 1.0f32;

        let mut thermal = |temp: Option<f32>, start: f32, end: f32| {
            let Some(t) = temp else { return };
            if start <= 0.0 || end <= start {
                return;
            }
            // brake follows the configured ramp...
            let b = ramp_down(t, start, end);
            // ...the accel copy is lerped toward 25 °C by accel_dec, so it
            // always sits at or below the brake ramp (VESC formula).
            let a_start = lerp(start, 25.0, self.accel_dec);
            let a_end = lerp(end, 25.0, self.accel_dec);
            let d = ramp_down(t, a_start, a_end);
            brake = brake.min(b);
            drive = drive.min(d.min(b));
        };
        thermal(fet_temp_c, self.temp_fet_start_c, self.temp_fet_end_c);
        thermal(motor_temp_c, self.temp_motor_start_c, self.temp_motor_end_c);

        if self.vbus_cut_start_v > 0.0 && self.vbus_cut_start_v > self.vbus_cut_end_v {
            // ramp_down mirrored in voltage: full above start, zero at end.
            drive = drive.min(1.0 - ramp_down(vbus_v, self.vbus_cut_end_v, self.vbus_cut_start_v));
        }
        if self.vbus_regen_start_v > 0.0 && self.vbus_regen_end_v > self.vbus_regen_start_v {
            regen = ramp_down(vbus_v, self.vbus_regen_start_v, self.vbus_regen_end_v);
        }

        DeratingScales {
            drive,
            brake,
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
            vbus_cut_start_v: 40.0,
            vbus_cut_end_v: 36.0,
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
            vbus_regen_start_v: 56.0,
            vbus_regen_end_v: 58.0,
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
            vbus_cut_start_v: 40.0,
            vbus_cut_end_v: 36.0,
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
                temp_fet_end_c: 80.0, // end below start
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                vbus_cut_start_v: 36.0,
                vbus_cut_end_v: 40.0, // cutoff must ramp downward
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                accel_dec: 1.5,
                ..cfg()
            }
            .is_sane()
        );
        assert!(
            !DeratingConfig {
                temp_fet_start_c: f32::NAN,
                ..cfg()
            }
            .is_sane()
        );
        // Disabled ramps don't need well-formed pairs.
        assert!(
            DeratingConfig {
                temp_fet_start_c: 0.0,
                temp_fet_end_c: 0.0,
                ..cfg()
            }
            .is_sane()
        );
    }
}
