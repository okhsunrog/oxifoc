//! Graduated derating: continuous power rolloff BEFORE any fault.
//!
//! VESC's `update_override_limits` lesson (docs/notes/fault-overhaul.md):
//! a stop is the response to "derating has already failed", not to "hot"
//! or "sagging". This module computes two live scale factors from the
//! measurements —
//!
//! - `drive` scales torque in the direction of motion (acceleration),
//! - `brake` scales torque opposing it (incl. regen),
//!
//! each the `min`-composition of independent linear ramps:
//!
//! | Ramp | Side | Shape |
//! |---|---|---|
//! | FET temperature | both, accel earlier | 1 below `start`, 0 at `end`; the accel copy has start/end lerped toward 25 °C by `accel_dec` (VESC `l_temp_accel_dec`) — a hot board loses acceleration first and keeps braking |
//! | Motor temperature | both, accel earlier | same shape, separate thresholds (off when no NTC) |
//! | Battery cutoff | drive | 1 above `vbus_cut_start`, 0 at `vbus_cut_end` (start > end) — a sagging pack sheds drive current before the UV fault can trip |
//! | Regen overvoltage | brake | 1 below `vbus_regen_start`, 0 at `vbus_regen_end` — braking into a full pack sheds regen before the OV fault |
//! | Speed cut | drive | 1 below `frac·max`, 0 at `max_speed_erad_s` — a soft ceiling, NEVER limits braking (descending past the limit must still brake) |
//!
//! The scales are computed ISR-side (decimated — temperatures and bus
//! voltage are slow), so the protection survives an async-executor hang,
//! same philosophy as the Layer-2 deadman. The faults of the severity
//! ladder remain the backstops at the ramp ends.

/// Live derating factors (1.0 = no derate). Applied as multipliers on the
/// iq budget in `step_current_control`, selected by direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeratingScales {
    /// Torque in the direction of motion (acceleration), 0..=1
    pub drive: f32,
    /// Torque opposing motion (braking/regen), 0..=1
    pub brake: f32,
}

impl Default for DeratingScales {
    fn default() -> Self {
        Self {
            drive: 1.0,
            brake: 1.0,
        }
    }
}

impl DeratingScales {
    /// No derate (also usable in const initializers, unlike `default()`).
    pub const IDENTITY: Self = Self {
        drive: 1.0,
        brake: 1.0,
    };

    /// The stronger of the two derates (for the Warning threshold).
    pub fn worst(&self) -> f32 {
        self.drive.min(self.brake)
    }
}

/// Runtime derating configuration (SI units; mirrors
/// `crate::storage::DeratingConfigStored`). Per-ramp `0` disables that
/// ramp — the all-default config derates on FET temperature only.
#[derive(Clone, Copy, Debug, PartialEq)]
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
    /// Speed soft ceiling (electrical rad/s; eRPM = erad/s × 60 / 2π);
    /// `0` disables. Drive-only — braking is never speed-limited.
    pub max_speed_erad_s: f32,
    /// Fraction of `max_speed_erad_s` where the drive rolloff starts
    pub speed_start_frac: f32,
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
            max_speed_erad_s: 0.0,
            speed_start_frac: 0.8,
        }
    }
}

impl DeratingConfig {
    /// Build from the stored (host-writable) form; a missing or non-sane
    /// stored value falls back to [`Default`] — a corrupt config can never
    /// disable the FET thermal rolloff. (The config server additionally
    /// rejects insane writes loudly: `From` + [`Self::is_sane`].)
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&crate::storage::DeratingConfigStored>) -> Self {
        match cfg {
            Some(c) => {
                let candidate = Self::from(c);
                if candidate.is_sane() {
                    candidate
                } else {
                    Self::default()
                }
            }
            None => Self::default(),
        }
    }

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
            && f(self.vbus_regen_end_v)
            && f(self.max_speed_erad_s)
            && f(self.speed_start_frac);
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
        let speed_ok = self.max_speed_erad_s <= 0.0
            || (self.speed_start_frac > 0.0 && self.speed_start_frac <= 1.0);
        (0.0..=1.0).contains(&self.accel_dec)
            && fet_ok
            && motor_ok
            && cut_ok
            && regen_ok
            && speed_ok
    }

    /// Compute the live scales. `None` measurements skip their ramps
    /// (a sensor the board does not carry must not derate anything).
    pub fn compute(
        &self,
        fet_temp_c: Option<f32>,
        motor_temp_c: Option<f32>,
        vbus_v: f32,
        omega_e_rad_s: f32,
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
            brake = brake.min(ramp_down(
                vbus_v,
                self.vbus_regen_start_v,
                self.vbus_regen_end_v,
            ));
        }
        if self.max_speed_erad_s > 0.0 {
            let start = self.max_speed_erad_s * self.speed_start_frac.clamp(0.0, 1.0);
            drive = drive.min(ramp_down(omega_e_rad_s.abs(), start, self.max_speed_erad_s));
        }

        DeratingScales { drive, brake }
    }
}

#[cfg(feature = "storage")]
impl From<&crate::storage::DeratingConfigStored> for DeratingConfig {
    fn from(c: &crate::storage::DeratingConfigStored) -> Self {
        Self {
            temp_fet_start_c: c.temp_fet_start_c,
            temp_fet_end_c: c.temp_fet_end_c,
            temp_motor_start_c: c.temp_motor_start_c,
            temp_motor_end_c: c.temp_motor_end_c,
            accel_dec: c.accel_dec,
            vbus_cut_start_v: c.vbus_cut_start_v,
            vbus_cut_end_v: c.vbus_cut_end_v,
            vbus_regen_start_v: c.vbus_regen_start_v,
            vbus_regen_end_v: c.vbus_regen_end_v,
            max_speed_erad_s: c.max_speed_erad_s,
            speed_start_frac: c.speed_start_frac,
        }
    }
}

/// 1.0 at/below `start`, 0.0 at/above `end`, linear between.
fn ramp_down(x: f32, start: f32, end: f32) -> f32 {
    if end <= start {
        return 1.0; // malformed ramp never derates (is_sane rejects writes)
    }
    ((end - x) / (end - start)).clamp(0.0, 1.0)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> DeratingConfig {
        DeratingConfig::default()
    }

    #[test]
    fn cold_board_no_derate() {
        let s = cfg().compute(Some(40.0), None, 24.0, 100.0);
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn fet_ramp_derates_accel_before_brake() {
        // Mid-ramp on the brake curve (85→100): the accel copy (lerped
        // toward 25 °C) must already be lower.
        let s = cfg().compute(Some(92.0), None, 24.0, 100.0);
        assert!(s.brake > 0.4 && s.brake < 0.7, "brake mid-ramp: {s:?}");
        assert!(s.drive < s.brake, "accel must derate earlier: {s:?}");

        // At the ramp end: both at zero — the OverTemp fault right past
        // this point means the ramp already failed.
        let s = cfg().compute(Some(100.0), None, 24.0, 100.0);
        assert_eq!(s.brake, 0.0);
        assert_eq!(s.drive, 0.0);
    }

    #[test]
    fn missing_sensor_skips_ramp() {
        let s = cfg().compute(None, None, 24.0, 100.0);
        assert_eq!(s, DeratingScales::default());
    }

    #[test]
    fn battery_cutoff_sheds_drive_keeps_brake() {
        let c = DeratingConfig {
            vbus_cut_start_v: 40.0,
            vbus_cut_end_v: 36.0,
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, 38.0, 100.0);
        assert!((s.drive - 0.5).abs() < 1e-3, "mid-cut: {s:?}");
        assert_eq!(s.brake, 1.0, "sag must never cost braking");
        let s = c.compute(Some(40.0), None, 35.0, 100.0);
        assert_eq!(s.drive, 0.0, "below cut end: no drive");
    }

    #[test]
    fn regen_ov_sheds_brake_keeps_drive() {
        let c = DeratingConfig {
            vbus_regen_start_v: 56.0,
            vbus_regen_end_v: 58.0,
            ..cfg()
        };
        let s = c.compute(Some(40.0), None, 57.0, 100.0);
        assert!((s.brake - 0.5).abs() < 1e-3, "mid-regen-cut: {s:?}");
        assert_eq!(s.drive, 1.0);
        let s = c.compute(Some(40.0), None, 58.5, 100.0);
        assert_eq!(s.brake, 0.0, "full pack: no regen");
    }

    #[test]
    fn speed_cut_is_drive_only() {
        let c = DeratingConfig {
            max_speed_erad_s: 1000.0,
            ..cfg()
        };
        // Below the rolloff start: untouched.
        let s = c.compute(Some(40.0), None, 24.0, 700.0);
        assert_eq!(s.drive, 1.0);
        // Mid-rolloff (start 800, end 1000).
        let s = c.compute(Some(40.0), None, 24.0, 900.0);
        assert!((s.drive - 0.5).abs() < 1e-3, "mid speed cut: {s:?}");
        // Past the ceiling (downhill): zero drive, FULL brake.
        let s = c.compute(Some(40.0), None, 24.0, 1100.0);
        assert_eq!(s.drive, 0.0);
        assert_eq!(s.brake, 1.0, "braking past the limit must work");
        // Direction-agnostic.
        let s = c.compute(Some(40.0), None, 24.0, -1100.0);
        assert_eq!(s.drive, 0.0);
    }

    #[test]
    fn ramps_compose_via_min() {
        let c = DeratingConfig {
            max_speed_erad_s: 1000.0,
            vbus_cut_start_v: 40.0,
            vbus_cut_end_v: 36.0,
            ..cfg()
        };
        // Hot + sagging + fast: drive takes the worst of the three.
        let s = c.compute(Some(92.0), None, 38.0, 900.0);
        let thermal_only = c.compute(Some(92.0), None, 50.0, 0.0);
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
