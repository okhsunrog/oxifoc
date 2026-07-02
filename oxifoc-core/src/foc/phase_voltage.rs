//! Phase-terminal voltage sensing for back-EMF / undriven rotation detection.
//!
//! Converts raw phase-terminal ADC counts to phase voltages and the αβ
//! stationary frame, mirroring [`current_sense::ShuntCurrentSense`]. The board
//! capability lives in [`config::PhaseSense`]; the per-cycle decision of which
//! voltage the observer integrates is [`observer_voltage_source`].
//!
//! # Common mode
//!
//! Unlike the current path — where Kirchhoff guarantees `ia + ib + ic = 0` so
//! the cheap 2-input Clarke is exact — measured phase voltages carry a
//! common-mode term (the floating-neutral DC, plus any matched channel bias).
//! So [`PhaseVoltageSense::alpha_beta`] uses the full 3-input Clarke
//! `((2Va−Vb−Vc)/3, (Vb−Vc)/√3)`, which cancels common mode. A consequence:
//! the αβ projection is correct from the matched component alone, so it
//! degrades gracefully even before the per-phase offsets are calibrated — the
//! offsets only remove *differential* (channel-to-channel) bias.
//!
//! [`current_sense::ShuntCurrentSense`]: crate::foc::current_sense::ShuntCurrentSense
//! [`config::PhaseSense`]: crate::foc::config::PhaseSense

use super::config::BoardConfig;
use super::constants::FRAC_1_SQRT_3;

/// Converts raw phase-terminal ADC counts → phase voltages (V) and αβ.
///
/// Holds the divider ratio, ADC scale, calibrated undriven DC offsets (in ADC
/// counts, like `ShuntCurrentSense`), and whether the board has RC phase
/// filters. Construct via [`from_board`](Self::from_board), which returns
/// `None` on boards without phase sensing — so a no-sensing board simply holds
/// `Option::<PhaseVoltageSense>::None` and pays nothing.
#[derive(Clone, Copy, Debug)]
pub struct PhaseVoltageSense {
    divider_ratio: f32,
    adc_vref_mv: u32,
    adc_max_counts: u16,
    has_filters: bool,
    /// Undriven zero-voltage ADC offsets per phase (counts).
    offsets: [f32; 3],
    /// True once the undriven offsets have been calibrated.
    calibrated: bool,
}

impl PhaseVoltageSense {
    /// Build from a board config, or `None` if the board has no phase sensing.
    pub fn from_board(board: &BoardConfig) -> Option<Self> {
        let ps = board.phase_sense?;
        Some(Self {
            divider_ratio: ps.divider_ratio,
            adc_vref_mv: board.calib.adc_vref_mv,
            adc_max_counts: board.calib.adc_max_counts,
            has_filters: ps.has_filters,
            // Resting terminal ≈ GND; the real per-channel bias comes from
            // undriven calibration. A uniform offset cancels in alpha_beta
            // anyway, so 0 is a safe pre-calibration default.
            offsets: [0.0; 3],
            calibrated: false,
        })
    }

    /// Whether the board has RC phase filters (measurement valid while driving).
    #[inline]
    pub fn has_filters(&self) -> bool {
        self.has_filters
    }

    /// Whether the undriven DC offsets have been calibrated.
    #[inline]
    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Per-phase terminal voltages (V), offset-corrected and divider-scaled.
    #[inline]
    pub fn phase_voltages(&self, raw: [u16; 3]) -> [f32; 3] {
        [
            self.counts_to_volts(raw[0], self.offsets[0]),
            self.counts_to_volts(raw[1], self.offsets[1]),
            self.counts_to_volts(raw[2], self.offsets[2]),
        ]
    }

    /// Measured phase voltages projected to the αβ stationary frame.
    ///
    /// Full 3-input Clarke so the common-mode (floating-neutral) term cancels:
    /// `v_alpha = (2Va − Vb − Vc)/3`, `v_beta = (Vb − Vc)/√3`.
    #[inline]
    pub fn alpha_beta(&self, raw: [u16; 3]) -> (f32, f32) {
        let [va, vb, vc] = self.phase_voltages(raw);
        let v_alpha = (2.0 * va - vb - vc) * (1.0 / 3.0);
        let v_beta = FRAC_1_SQRT_3 * (vb - vc);
        (v_alpha, v_beta)
    }

    /// Current undriven offsets (ADC counts).
    #[inline]
    pub fn offsets(&self) -> [f32; 3] {
        self.offsets
    }

    /// Manually set undriven offsets (ADC counts) and mark calibrated.
    pub fn set_offsets(&mut self, offsets: [f32; 3]) {
        self.offsets = offsets;
        self.calibrated = true;
    }

    /// Streaming undriven-offset calibration: per-phase mean of `count` samples
    /// taken with the bridge OFF and the motor still, from running sums.
    /// Mirrors [`ShuntCurrentSense::calibrate_offsets_from_sums`] so async
    /// callers needn't buffer thousands of samples across await points.
    ///
    /// [`ShuntCurrentSense::calibrate_offsets_from_sums`]: crate::foc::current_sense::ShuntCurrentSense::calibrate_offsets_from_sums
    pub fn calibrate_offsets_from_sums(&mut self, sums: [u32; 3], count: u32) {
        if count == 0 {
            return;
        }
        let count = count as f32;
        self.offsets = [
            sums[0] as f32 / count,
            sums[1] as f32 / count,
            sums[2] as f32 / count,
        ];
        self.calibrated = true;
    }

    #[inline]
    fn counts_to_volts(&self, raw: u16, offset: f32) -> f32 {
        let delta = f32::from(raw) - offset;
        let v_adc = delta * (self.adc_vref_mv as f32 / 1000.0) / f32::from(self.adc_max_counts);
        v_adc * self.divider_ratio
    }
}

/// Which voltage the back-EMF observer integrates on a given cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObserverVoltage {
    /// Commanded modulation × Vbus — no valid phase measurement available.
    Commanded,
    /// Measured phase voltage — undriven back-EMF, or driven with RC filters.
    Measured,
}

/// Pick the observer's voltage source from the board capability and bridge
/// state. The single decision point that collapses all three board classes:
///
/// - no sensing (`None`) → always [`Commanded`](ObserverVoltage::Commanded);
/// - sensing, bridge undriven → [`Measured`](ObserverVoltage::Measured) (back-EMF);
/// - sensing **+ filters**, bridge driven → `Measured`;
/// - sensing, **no** filters, bridge driven → `Commanded` (the measurement is
///   PWM noise without an RC filter to average it).
#[inline]
pub fn observer_voltage_source(
    sensor: Option<&PhaseVoltageSense>,
    bridge_driven: bool,
) -> ObserverVoltage {
    match sensor {
        None => ObserverVoltage::Commanded,
        Some(_) if !bridge_driven => ObserverVoltage::Measured,
        Some(s) if s.has_filters() => ObserverVoltage::Measured,
        Some(_) => ObserverVoltage::Commanded,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::config::{BoardConfig, PhaseSense};

    const fn board(phase_sense: Option<PhaseSense>) -> BoardConfig {
        BoardConfig {
            calib: crate::types::BoardCalib {
                shunt_ohms: 0.0005,
                amp_gain: 10.0,
                adc_vref_mv: 3300,
                adc_max_counts: 4095,
                invert_current_sign: false,
                vbus_divider_ratio: (39.0 + 2.2) / 2.2,
            },
            initial_vbus_volts: 12.0,
            max_iq_target_a: 10.0,
            max_phase_current_a: 60.0,
            max_vbus_mv: 57_000,
            min_vbus_mv: 6_000,
            max_fet_temp_c: 100.0,
            max_motor_temp_c: 120.0,
            phase_sense,
        }
    }

    const CF2_SENSE: PhaseSense = PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: false,
    };

    #[test]
    fn from_board_none_without_sensing() {
        assert!(PhaseVoltageSense::from_board(&board(None)).is_none());
    }

    #[test]
    fn from_board_some_with_sensing() {
        let s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        assert!(!s.has_filters());
        assert!(!s.is_calibrated());
    }

    #[test]
    fn counts_to_volts_scales_by_divider() {
        let s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        // Full-scale counts (4095) → 3.3 V at the ADC × 18.727 divider ≈ 61.8 V.
        let [va, ..] = s.phase_voltages([4095, 0, 0]);
        let expected = 3.3 * (39.0 + 2.2) / 2.2;
        assert!((va - expected).abs() < 0.05, "va={va} expected={expected}");
    }

    #[test]
    fn alpha_beta_cancels_common_mode() {
        let s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        // Equal terminal voltages = pure common mode → αβ must be ~0.
        let (a, b) = s.alpha_beta([1500, 1500, 1500]);
        assert!(a.abs() < 1e-3 && b.abs() < 1e-3, "a={a} b={b}");
    }

    #[test]
    fn alpha_beta_recovers_balanced_set() {
        // Construct a balanced 3-phase voltage set offset by a common mode and
        // confirm the αβ vector matches the full-Clarke of the AC component.
        let s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        let scale = 3.3 * (39.0 + 2.2) / 2.2 / 4095.0; // V per count
        // Phase A on the +alpha axis: Va=+1, Vb=Vc=-0.5 (arbitrary units),
        // plus a 1500-count common mode that must wash out.
        let mid = 1500.0;
        let amp = 800.0;
        let raw = [
            (mid + amp) as u16,
            (mid - amp / 2.0) as u16,
            (mid - amp / 2.0) as u16,
        ];
        let (a, beta) = s.alpha_beta(raw);
        // v_alpha = (2Va − Vb − Vc)/3 = amp counts → volts; v_beta = 0.
        assert!((a - amp * scale).abs() < 0.05, "a={a}");
        assert!(beta.abs() < 1e-3, "beta={beta}");
    }

    #[test]
    fn calibrate_offsets_averages() {
        let mut s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        // 4 samples summing to (8000,8200,8400) → means (2000,2050,2100).
        s.calibrate_offsets_from_sums([8000, 8200, 8400], 4);
        assert!(s.is_calibrated());
        let o = s.offsets();
        assert!((o[0] - 2000.0).abs() < 0.1);
        assert!((o[1] - 2050.0).abs() < 0.1);
        assert!((o[2] - 2100.0).abs() < 0.1);
        // Reading exactly at the offset → zero phase voltage.
        let v = s.phase_voltages([2000, 2050, 2100]);
        assert!(v.iter().all(|x| x.abs() < 1e-3), "v={v:?}");
    }

    #[test]
    fn calibrate_zero_count_is_noop() {
        let mut s = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        s.calibrate_offsets_from_sums([0, 0, 0], 0);
        assert!(!s.is_calibrated());
    }

    #[test]
    fn voltage_source_truth_table() {
        // No sensing → always commanded.
        assert_eq!(
            observer_voltage_source(None, false),
            ObserverVoltage::Commanded
        );
        assert_eq!(
            observer_voltage_source(None, true),
            ObserverVoltage::Commanded
        );

        // Sensing, no filters (CF2): measured undriven, commanded driven.
        let cf2 = PhaseVoltageSense::from_board(&board(Some(CF2_SENSE))).unwrap();
        assert_eq!(
            observer_voltage_source(Some(&cf2), false),
            ObserverVoltage::Measured
        );
        assert_eq!(
            observer_voltage_source(Some(&cf2), true),
            ObserverVoltage::Commanded
        );

        // Sensing + filters: measured in both states.
        let filt = PhaseVoltageSense::from_board(&board(Some(PhaseSense {
            divider_ratio: 18.0,
            has_filters: true,
        })))
        .unwrap();
        assert_eq!(
            observer_voltage_source(Some(&filt), false),
            ObserverVoltage::Measured
        );
        assert_eq!(
            observer_voltage_source(Some(&filt), true),
            ObserverVoltage::Measured
        );
    }
}
