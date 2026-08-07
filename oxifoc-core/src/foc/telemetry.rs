//! Fast-telemetry fixed-point codec + shared raw→engineering enrichment.
//!
//! The compact 18-byte [`FastTelemetry`] frame ships raw ADC counts and
//! fixed-point scalars; the host reconstructs engineering units with the **same
//! `oxifoc-core` code** the firmware uses, so the two cannot desync:
//!
//! - **Scalars** (`vbus`, `vd`, `vq`, `rpm`): encoded/decoded through [`Scale`]
//!   — one LSB constant per field defines *both* directions, so
//!   `build_fast_telemetry`'s encode and [`FastTelemetry::enrich`]'s decode are
//!   structurally inverse (round-trip tested below). `angle` is modular, its own
//!   paired methods over one constant.
//! - **Currents**: shipped as raw ADC counts, decoded only via
//!   [`ShuntCurrentSense`] — the firmware's own converter — then Clarke/Park.
//!
//! See `docs/notes/telemetry-enrichment.md` for the full design.

use core::f32::consts::TAU;

use crate::foc::current_sense::ShuntCurrentSense;
use crate::foc::transforms::{clarke, park};
use crate::types::{BoardCalib, FastTelemetry};

/// Linear fixed-point codec: a single LSB value (physical units per quantum)
/// defines both directions, so encode and decode cannot drift apart.
#[derive(Clone, Copy, Debug)]
pub struct Scale {
    lsb: f32,
}

impl Scale {
    /// Codec with `lsb` physical units per integer quantum.
    pub const fn new(lsb: f32) -> Self {
        Self { lsb }
    }
    /// Physical value → integer quantum (truncates toward zero, ≤1 LSB bias).
    #[inline]
    pub fn enc(self, v: f32) -> i32 {
        (v / self.lsb) as i32
    }
    /// Physical value → saturated unsigned 16-bit quantum.
    #[inline]
    fn enc_u16(self, v: f32) -> u16 {
        self.enc(v).clamp(0, i32::from(u16::MAX)) as u16
    }
    /// Physical value → saturated signed 16-bit quantum.
    #[inline]
    fn enc_i16(self, v: f32) -> i16 {
        self.enc(v).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
    }
    /// Integer quantum → physical value.
    #[inline]
    pub fn dec(self, raw: i32) -> f32 {
        raw as f32 * self.lsb
    }
}

/// Bus / dq voltages: 1 LSB = 2 mV (in volts).
const VOLT: Scale = Scale::new(0.002);
/// Mechanical speed: 1 LSB = 2 RPM.
const RPM: Scale = Scale::new(2.0);
/// Electrical angle: full-scale `u16` over one turn.
const ANGLE_PER_LSB: f32 = TAU / 65536.0;

impl FastTelemetry {
    // ---- encode (firmware `build_fast_telemetry` path) ----

    /// Pack bus voltage (volts) → `vbus` field.
    #[inline]
    pub fn pack_vbus(volts: f32) -> u16 {
        VOLT.enc_u16(volts)
    }
    /// Pack a dq voltage (volts) → `vd`/`vq` field.
    #[inline]
    pub fn pack_volt(volts: f32) -> i16 {
        VOLT.enc_i16(volts)
    }
    /// Pack mechanical speed (RPM) → `rpm` field.
    #[inline]
    pub fn pack_rpm(mech_rpm: f32) -> i16 {
        RPM.enc_i16(mech_rpm)
    }
    /// Pack electrical angle (radians, any range) → full-scale `angle` field.
    /// `rem_euclid` is std-only, so wrap via a truncating cast of the turn count.
    #[inline]
    pub fn pack_angle(rad: f32) -> u16 {
        let turns = rad / TAU;
        let frac = turns - (turns as i32 as f32);
        let frac = if frac < 0.0 { frac + 1.0 } else { frac };
        (frac * 65536.0) as u16
    }

    // ---- decode (host `enrich` path) ----

    /// Bus voltage in volts.
    #[inline]
    pub fn vbus_v(&self) -> f32 {
        VOLT.dec(i32::from(self.vbus))
    }
    /// d-axis applied voltage in volts.
    #[inline]
    pub fn vd_v(&self) -> f32 {
        VOLT.dec(i32::from(self.vd))
    }
    /// q-axis applied voltage in volts.
    #[inline]
    pub fn vq_v(&self) -> f32 {
        VOLT.dec(i32::from(self.vq))
    }
    /// Mechanical speed in RPM.
    #[inline]
    pub fn mech_rpm(&self) -> f32 {
        RPM.dec(i32::from(self.rpm))
    }
    /// Electrical angle in radians, `[0, 2π)`.
    #[inline]
    pub fn angle_rad(&self) -> f32 {
        f32::from(self.angle) * ANGLE_PER_LSB
    }

    /// Reconstruct full engineering units from the raw frame using the same core
    /// math the firmware runs (`ShuntCurrentSense` + Clarke/Park + the scalar
    /// codec). Host-side; the single decode path shared by CLI and GUI.
    pub fn enrich(&self, ctx: &EnrichCtx) -> RichSample {
        let (ia, ib, ic) = ctx.isense.convert_raw(self.ia, self.ib, self.ic);
        let (i_alpha, i_beta) = clarke(ia, ib);
        let angle_rad = self.angle_rad();
        let (id, iq) = park(
            i_alpha,
            i_beta,
            libm::sinf(angle_rad),
            libm::cosf(angle_rad),
        );
        let mech_rpm = self.mech_rpm();
        RichSample {
            ia,
            ib,
            ic,
            i_alpha,
            i_beta,
            id,
            iq,
            vbus_v: self.vbus_v(),
            vd: self.vd_v(),
            vq: self.vq_v(),
            angle_rad,
            mech_rpm,
            erpm: mech_rpm * f32::from(ctx.pole_pairs),
            seq: self.seq,
        }
    }
}

/// Everything the host needs to enrich a frame: the current-sense converter
/// (built from the static [`BoardCalib`] + the calibrated `dc_offsets`) and the
/// pole-pair count (for eRPM). Built once at connect.
pub struct EnrichCtx {
    /// Current converter — the firmware's own `ShuntCurrentSense`, configured
    /// from the device-reported calibration.
    pub isense: ShuntCurrentSense,
    /// Motor pole pairs (0 = unknown; eRPM then reads 0).
    pub pole_pairs: u8,
}

impl EnrichCtx {
    /// Build from the device-reported [`BoardCalib`], the calibrated per-phase
    /// zero-current `dc_offsets` (ADC counts), and `pole_pairs`.
    pub fn new(calib: &BoardCalib, dc_offsets: (f32, f32, f32), pole_pairs: u8) -> Self {
        let mut isense = ShuntCurrentSense::from_calib(calib);
        isense.set_offsets(dc_offsets.0, dc_offsets.1, dc_offsets.2);
        Self { isense, pole_pairs }
    }
}

/// Fully decoded engineering-unit telemetry sample (host-facing).
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize)]
pub struct RichSample {
    /// Phase currents (A).
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    /// Clarke αβ currents (A).
    pub i_alpha: f32,
    pub i_beta: f32,
    /// Park dq currents (A).
    pub id: f32,
    pub iq: f32,
    /// Bus voltage (V).
    pub vbus_v: f32,
    /// Applied dq voltages (V).
    pub vd: f32,
    pub vq: f32,
    /// Electrical angle (rad).
    pub angle_rad: f32,
    /// Mechanical speed (RPM) and electrical RPM (= mech · pole_pairs).
    pub mech_rpm: f32,
    pub erpm: f32,
    /// Sequence number (FOC-cycle counter mod 65536).
    pub seq: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    // A representative board calibration (B-G431B-ESC1-ish, generic values).
    fn calib() -> BoardCalib {
        BoardCalib {
            shunt_ohms: 0.003,
            amp_gain: 16.0,
            adc_vref_mv: 3300,
            adc_max_counts: 4095,
            invert_current_sign: false,
            vbus_divider_ratio: 10.39,
        }
    }

    // ---- scalar codec ----

    #[test]
    fn scale_roundtrip_within_one_lsb() {
        for s in [VOLT, RPM] {
            for &v in &[-65.0, -12.3, -0.5, 0.0, 0.123, 12.0, 48.0, 65.0] {
                let back = s.dec(s.enc(v));
                assert!(
                    (back - v).abs() <= s.lsb + 1e-6,
                    "lsb={} v={v} back={back}",
                    s.lsb
                );
            }
        }
    }

    #[test]
    fn field_scalar_roundtrip() {
        // volts within 2 mV, rpm within 2 RPM
        for &v in &[-48.0f32, -1.234, 0.0, 3.3, 24.0, 48.0] {
            let f = FastTelemetry {
                vbus: FastTelemetry::pack_vbus(v.abs()),
                vd: FastTelemetry::pack_volt(v),
                vq: FastTelemetry::pack_volt(-v),
                ..Default::default()
            };
            assert!((f.vbus_v() - v.abs()).abs() <= 0.002 + 1e-6, "vbus {v}");
            assert!((f.vd_v() - v).abs() <= 0.002 + 1e-6, "vd {v}");
            assert!((f.vq_v() + v).abs() <= 0.002 + 1e-6, "vq {v}");
        }
        for &r in &[-60000.0, -1234.0, 0.0, 700.0, 45000.0, 60000.0] {
            let f = FastTelemetry {
                rpm: FastTelemetry::pack_rpm(r),
                ..Default::default()
            };
            assert!((f.mech_rpm() - r).abs() <= 2.0 + 1e-3, "rpm {r}");
        }
    }

    #[test]
    fn field_scalar_packers_saturate_instead_of_wrapping() {
        assert_eq!(FastTelemetry::pack_vbus(-1.0), 0);
        assert_eq!(FastTelemetry::pack_vbus(f32::NAN), 0);
        assert_eq!(FastTelemetry::pack_vbus(f32::INFINITY), u16::MAX);

        assert_eq!(FastTelemetry::pack_volt(f32::NEG_INFINITY), i16::MIN);
        assert_eq!(FastTelemetry::pack_volt(f32::INFINITY), i16::MAX);
        assert_eq!(FastTelemetry::pack_volt(f32::NAN), 0);

        assert_eq!(FastTelemetry::pack_rpm(-100_000.0), i16::MIN);
        assert_eq!(FastTelemetry::pack_rpm(100_000.0), i16::MAX);
        assert_eq!(FastTelemetry::pack_rpm(f32::NAN), 0);
    }

    #[test]
    fn angle_roundtrip_and_wrap() {
        use core::f32::consts::PI;
        // in-range and out-of-range angles both wrap to [0, 2π) within 1 LSB
        for &a in &[0.0, 0.1, PI, 1.9 * PI, TAU, TAU + 0.3, -0.2, -PI] {
            let packed = FastTelemetry::pack_angle(a);
            let got = FastTelemetry {
                angle: packed,
                ..Default::default()
            }
            .angle_rad();
            let want = {
                let mut w = a % TAU;
                if w < 0.0 {
                    w += TAU;
                }
                w
            };
            // shortest angular distance (handles the 0/2π seam)
            let mut d = got - want;
            if d > PI {
                d -= TAU;
            } else if d < -PI {
                d += TAU;
            }
            assert!(
                d.abs() <= ANGLE_PER_LSB * 2.0,
                "a={a} got={got} want={want}"
            );
        }
    }

    // ---- full enrichment ----

    #[test]
    fn enrich_zero_current_zero_angle() {
        let ctx = EnrichCtx::new(&calib(), (2048.0, 2048.0, 2048.0), 7);
        let f = FastTelemetry {
            ia: 2048,
            ib: 2048,
            ic: 2048,
            vbus: FastTelemetry::pack_vbus(24.0),
            angle: 0,
            vd: FastTelemetry::pack_volt(0.0),
            vq: FastTelemetry::pack_volt(1.0),
            rpm: FastTelemetry::pack_rpm(1000.0),
            seq: 42,
        };
        let r = f.enrich(&ctx);
        assert!(r.ia.abs() < 1e-3 && r.ib.abs() < 1e-3 && r.ic.abs() < 1e-3);
        assert!(r.id.abs() < 1e-3 && r.iq.abs() < 1e-3);
        assert!((r.vbus_v - 24.0).abs() <= 0.002 + 1e-6);
        assert!((r.vq - 1.0).abs() <= 0.002 + 1e-6);
        assert!((r.mech_rpm - 1000.0).abs() <= 2.0);
        assert!((r.erpm - 7000.0).abs() <= 20.0); // 1000 * 7 pp
        assert_eq!(r.seq, 42);
    }

    #[test]
    fn enrich_golden_currents_and_park() {
        // Known: offsets 2048; +10 A on A, -5 A on B, 0 A on C (matches the
        // current_sense golden test); angle 0 → Park(θ=0): id=iα, iq=iβ.
        let c = calib();
        let ctx = EnrichCtx::new(&c, (2048.0, 2048.0, 2048.0), 1);
        let counts_per_mv = f32::from(c.adc_max_counts) / c.adc_vref_mv as f32;
        let d_a = (10.0 * c.shunt_ohms * c.amp_gain * 1000.0 * counts_per_mv) as i32;
        let d_b = (-5.0 * c.shunt_ohms * c.amp_gain * 1000.0 * counts_per_mv) as i32;
        let f = FastTelemetry {
            ia: (2048 + d_a) as u16,
            ib: (2048 + d_b) as u16,
            ic: 2048,
            angle: 0,
            ..Default::default()
        };
        let r = f.enrich(&ctx);
        assert!((r.ia - 10.0).abs() < 0.5, "ia={}", r.ia);
        assert!((r.ib + 5.0).abs() < 0.3, "ib={}", r.ib);
        assert!(r.ic.abs() < 0.1, "ic={}", r.ic);
        // θ=0: id = iα = ia, iq = iβ
        assert!((r.id - r.i_alpha).abs() < 1e-4 && (r.iq - r.i_beta).abs() < 1e-4);
        assert!((r.id - r.ia).abs() < 1e-4);
    }

    #[test]
    fn calib_is_boardconfig_substruct() {
        // BoardConfig::calib() projects the same field values (compile-checked,
        // this guards the value mapping too).
        use crate::foc::config::BoardConfig;
        let bc = BoardConfig {
            calib: calib(),
            max_iq_target_a: 5.0,
            max_phase_current_a: 10.0,
            max_vbus_mv: 45_000,
            min_vbus_mv: 8_000,
            max_fet_temp_c: 85.0,
            max_motor_temp_c: 0.0,
            phase_sense: None,
        };
        // `calib` is a genuine sub-struct field now (no bridge method): the same
        // BoardCalib the firmware sense path and the host enrichment both use.
        assert_eq!(bc.calib, calib());
        assert_eq!(bc.calib.shunt_ohms, 0.003);
        assert_eq!(bc.calib.amp_gain, 16.0);
    }
}
