//! Board configuration and ADC utilities
//!
//! Provides shared configuration structures and helper functions for
//! platform-specific board parameters like shunt resistance, amplifier gains,
//! voltage dividers, and calibration settings.

/// Board-specific hardware configuration
///
/// Contains all the hardware parameters needed for current sensing,
/// voltage measurement, and motor control. Platform crates define
/// their own `const` instance with board-specific values.
///
/// # Example
///
/// ```rust
/// use oxifoc_core::foc::config::BoardConfig;
/// use oxifoc_core::types::BoardCalib;
///
/// const BOARD: BoardConfig = BoardConfig {
///     calib: BoardCalib {
///         shunt_ohms: 0.003,           // 3mΩ shunts
///         amp_gain: 16.0,              // 16x OPAMP gain
///         adc_vref_mv: 3300,           // 3.3V reference
///         adc_max_counts: 4095,        // 12-bit ADC
///         invert_current_sign: true,   // Low-side shunts
///         vbus_divider_ratio: 10.39,   // VBUS divider
///     },
///     max_iq_target_a: 10.0,       // Max torque current
///     // Fault thresholds
///     max_phase_current_a: 40.0,   // Peak phase current limit
///     max_vbus_mv: 60_000,         // Overvoltage at 60V
///     min_vbus_mv: 8_000,          // Undervoltage at 8V
///     max_fet_temp_c: 100.0,       // FET overtemp at 100°C
///     max_motor_temp_c: 0.0,       // 0 = no motor NTC wired
///     phase_sense: None,           // no phase-voltage sensing
/// };
/// ```
/// Phase-voltage sensing capability of a board.
///
/// "Sensing" means each phase terminal is routed through a resistor divider to
/// an ADC channel. "Filters" means those same lines additionally carry an RC
/// low-pass, so the measurement is valid *while the bridge is actively PWMing*
/// (the switching node is averaged out). Filters imply sensing — the filter
/// sits on the sense line — which is why `has_filters` lives *inside*
/// `PhaseSense` rather than as a parallel board flag. Without filters
/// (`has_filters == false`) the measurement is only meaningful when the bridge
/// is undriven (all FETs off → terminal voltage = motor back-EMF).
///
/// Three board classes fall out of `Option<PhaseSense>`:
/// - `None` → no sensing (e.g. B-G431B-ESC1, whose phase nets only show the
///   zero-crossing — useless for a full αβ projection);
/// - `Some { has_filters: false }` → sensing, no filters (e.g. Cheap FOCer 2):
///   measured voltage usable only undriven;
/// - `Some { has_filters: true }` → sensing + filters: measured voltage usable
///   while driving too.
///
/// See `foc::phase_voltage` for the converter and the per-cycle source decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseSense {
    /// Divider ratio Vphase / Vadc, e.g. `(39_000.0 + 2_200.0) / 2_200.0` for
    /// the Cheap FOCer 2 (its phase divider matches the Vbus divider).
    pub divider_ratio: f32,
    /// RC phase filters present → measured phase voltage valid while driving.
    /// `false` → only valid undriven (back-EMF).
    pub has_filters: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BoardConfig {
    /// Current-sense / vbus electrical constants (shunt, gain, vref, ADC
    /// resolution, sign, vbus divider). This is also the wire projection sent
    /// to the host for telemetry enrichment — one field list, no duplication.
    /// See [`crate::types::BoardCalib`].
    pub calib: crate::types::BoardCalib,
    /// Maximum q-axis current target in Amperes
    pub max_iq_target_a: f32,

    // Fault thresholds
    /// Maximum peak phase current in Amperes (instantaneous trip)
    pub max_phase_current_a: f32,
    /// Maximum DC bus voltage in millivolts (overvoltage threshold)
    pub max_vbus_mv: u32,
    /// Minimum DC bus voltage in millivolts (undervoltage threshold)
    pub min_vbus_mv: u32,
    /// Maximum FET temperature in Celsius (overtemperature threshold)
    pub max_fet_temp_c: f32,
    /// Maximum motor temperature in Celsius (overtemperature threshold).
    /// 0.0 disables the check (board has no motor NTC wired).
    pub max_motor_temp_c: f32,

    /// Phase-voltage sensing capability. `None` on boards that don't route
    /// phase terminals to the ADC (the observer then always uses commanded
    /// voltage). See [`PhaseSense`].
    pub phase_sense: Option<PhaseSense>,
}

impl BoardConfig {
    /// Convert raw ADC value to bus voltage in millivolts
    ///
    /// Uses the board's voltage divider ratio to scale the ADC reading.
    #[inline]
    pub fn vbus_mv_from_adc(&self, raw: u16) -> u32 {
        let raw = u32::from(raw);
        let vsense_mv = raw * self.calib.adc_vref_mv / u32::from(self.calib.adc_max_counts);
        (vsense_mv as f32 * self.calib.vbus_divider_ratio) as u32
    }

    /// Convert 3 raw ADC readings to phase currents in Amps
    ///
    /// Assumes mid-scale (adc_max_counts/2) is zero current and honors
    /// `invert_current_sign` like the runtime sense path does.
    /// Scale = Vref / (adc_max * R_shunt * gain)
    ///
    /// Note: this uses the *theoretical* mid-scale offset, not measured
    /// calibration offsets — fine for rough conversion, but prefer the
    /// calibrated `CurrentSensor` path for control.
    #[inline]
    pub fn convert_raw_currents(&self, raw_a: u16, raw_b: u16, raw_c: u16) -> (f32, f32, f32) {
        let offset = f32::from(self.calib.adc_max_counts) / 2.0;
        let mut scale = self.calib.adc_vref_mv as f32
            / 1000.0
            / f32::from(self.calib.adc_max_counts)
            / self.calib.shunt_ohms
            / self.calib.amp_gain;
        if self.calib.invert_current_sign {
            scale = -scale;
        }
        (
            (f32::from(raw_a) - offset) * scale,
            (f32::from(raw_b) - offset) * scale,
            (f32::from(raw_c) - offset) * scale,
        )
    }

    /// Convert duty percent (0-100) to target q-axis current
    ///
    /// Linear mapping from duty percentage to current target.
    #[inline]
    pub fn duty_to_iq(&self, duty: u8) -> f32 {
        let duty = duty.min(100);
        f32::from(duty) / 100.0 * self.max_iq_target_a
    }
}

// ============================================================================
// Calibration Constants
// ============================================================================

/// Default number of samples for current sense calibration
pub const DEFAULT_CALIBRATION_SAMPLES: usize = 256;

/// Default delay between calibration samples in microseconds
pub const DEFAULT_CALIBRATION_DELAY_US: u64 = 100;

// ============================================================================
// NTC Temperature Sensing
// ============================================================================

/// NTC circuit topology
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NtcTopology {
    /// NTC on low side: fixed pull-up resistor to Vref, NTC to GND
    /// ADC measures voltage at NTC/pull-up junction
    /// R_ntc = R_pullup / (Vmax/Vadc - 1)
    LowSide,
    /// NTC on high side: NTC to Vref, fixed pull-down resistor to GND
    /// ADC measures voltage at NTC/pull-down junction
    /// R_ntc = R_pulldown * (Vmax/Vadc - 1)
    HighSide,
}

/// NTC thermistor configuration for temperature sensing
///
/// Uses the Beta parameter model for NTC resistance-to-temperature conversion.
/// Supports both high-side and low-side NTC topologies.
#[derive(Clone, Copy, Debug)]
pub struct NtcConfig {
    /// Fixed resistor in voltage divider (Ohms) - pull-up for low-side, pull-down for high-side
    pub r_fixed_ohm: f32,
    /// NTC resistance at reference temperature T0 (Ohms)
    pub r0_ohm: f32,
    /// Beta parameter (K)
    pub beta: f32,
    /// Reference temperature in Kelvin (typically 298.15K = 25°C)
    pub t0_k: f32,
    /// Circuit topology (high-side or low-side NTC)
    pub topology: NtcTopology,
}

impl NtcConfig {
    /// Convert raw ADC value to temperature in 0.1 °C units, saturated to
    /// the i16 range and with non-finite results mapped to 0.
    ///
    /// Signed on purpose: sub-zero ambient is a legitimate reading, and
    /// clamping it to 0 (as the platform ISRs used to) lies in telemetry.
    #[inline]
    pub fn temp_c_x10_from_adc(&self, raw: u16, adc_max_counts: u16) -> i16 {
        let temp_c = self.temp_c_from_adc(raw, adc_max_counts);
        if temp_c.is_finite() {
            // `as` saturates at the i16 bounds (and maps NaN to 0).
            (temp_c * 10.0) as i16
        } else {
            0
        }
    }

    /// Convert raw ADC value to temperature in Celsius
    ///
    /// Uses the Beta model: T = 1 / (ln(R/R0)/Beta + 1/T0) - 273.15
    #[inline]
    pub fn temp_c_from_adc(&self, raw: u16, adc_max_counts: u16) -> f32 {
        let adc = f32::from(raw);
        let adc_max = f32::from(adc_max_counts);

        // Avoid divide-by-zero
        let eps = 0.1;

        // Calculate NTC resistance based on topology
        let r_ntc = match self.topology {
            NtcTopology::LowSide => {
                // NTC to GND, pull-up to Vref
                // R_ntc = R_pullup / (Vmax/Vadc - 1)
                self.r_fixed_ohm / (adc_max / (adc + eps) - 1.0)
            }
            NtcTopology::HighSide => {
                // NTC to Vref, pull-down to GND
                // R_ntc = R_pulldown * (Vmax/Vadc - 1)
                self.r_fixed_ohm * (adc_max / (adc + eps) - 1.0)
            }
        };

        // Beta-model temperature calculation
        // T = 1 / (ln(R/R0)/Beta + 1/T0)
        let ln_term = libm::logf(r_ntc / self.r0_ohm);
        let temp_k = 1.0 / (ln_term / self.beta + 1.0 / self.t0_k);
        temp_k - 273.15 // Convert to Celsius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_BOARD: BoardConfig = BoardConfig {
        calib: crate::types::BoardCalib {
            shunt_ohms: 0.003,
            amp_gain: 16.0,
            adc_vref_mv: 3300,
            adc_max_counts: 4095,
            invert_current_sign: false,
            vbus_divider_ratio: 10.39, // 187/18
        },
        max_iq_target_a: 10.0,
        // Fault thresholds
        max_phase_current_a: 40.0,
        max_vbus_mv: 60_000,
        min_vbus_mv: 8_000,
        max_fet_temp_c: 100.0,
        max_motor_temp_c: 120.0,
        phase_sense: None,
    };

    #[test]
    fn test_vbus_conversion() {
        // Mid-scale ADC (1.65V) with 10.39x divider = ~17.1V
        let vbus = TEST_BOARD.vbus_mv_from_adc(2048);
        assert!(vbus > 17000 && vbus < 17500);
    }

    #[test]
    fn convert_raw_currents_honors_invert_flag() {
        // The B-G431B-ESC1 (the struct's own doc example) needs
        // invert_current_sign: true — without honoring it this helper hands
        // sign-flipped currents to everything sign-sensitive in detection
        // (HFI dq separation, flux-linkage sign, observer input).
        let inverted = BoardConfig {
            calib: crate::types::BoardCalib {
                invert_current_sign: true,
                ..TEST_BOARD.calib
            },
            ..TEST_BOARD
        };
        // Raw above mid-scale = positive voltage at the ADC; with inversion
        // that must read as NEGATIVE motor current (and vice versa).
        let (ia, _, _) = inverted.convert_raw_currents(3000, 2048, 2048);
        assert!(
            ia < 0.0,
            "inverted board: raw>mid must be negative, got {ia}"
        );
        let (ia_n, _, _) = TEST_BOARD.convert_raw_currents(3000, 2048, 2048);
        assert!(ia_n > 0.0, "non-inverted board: raw>mid must be positive");
        // Same magnitude either way
        assert!((ia + ia_n).abs() < 1e-6);
    }

    #[test]
    fn test_duty_to_iq() {
        assert!((TEST_BOARD.duty_to_iq(0) - 0.0).abs() < 0.01);
        assert!((TEST_BOARD.duty_to_iq(50) - 5.0).abs() < 0.01);
        assert!((TEST_BOARD.duty_to_iq(100) - 10.0).abs() < 0.01);
        assert!((TEST_BOARD.duty_to_iq(150) - 10.0).abs() < 0.01); // Clamped
    }

    #[test]
    fn test_ntc_temp_low_side() {
        // Test low-side NTC with 4.7k pull-up (like B-G431B-ESC1)
        let ntc = NtcConfig {
            r_fixed_ohm: 4700.0,
            r0_ohm: 10_000.0,
            beta: 3455.0,
            t0_k: 273.15 + 25.0,
            topology: NtcTopology::LowSide,
        };
        // At 25°C, NTC = 10k, voltage divider gives specific ADC value
        // Low-side: Vadc = Vref * R_ntc / (R_pullup + R_ntc) = 3.3 * 10k / 14.7k = 2.245V
        // ADC = 2.245 / 3.3 * 4095 ≈ 2785
        let temp = ntc.temp_c_from_adc(2785, 4095);
        assert!(temp > 20.0 && temp < 30.0, "temp was {temp}");
    }

    #[test]
    fn test_ntc_temp_high_side() {
        // Test high-side NTC with 10k pull-down (like Cheap FOCer 2 board temp)
        let ntc = NtcConfig {
            r_fixed_ohm: 10_000.0,
            r0_ohm: 10_000.0,
            beta: 3380.0,
            t0_k: 273.15 + 25.0,
            topology: NtcTopology::HighSide,
        };
        // At 25°C, NTC = 10k, voltage divider gives specific ADC value
        // High-side: Vadc = Vref * R_pulldown / (R_ntc + R_pulldown) = 3.3 * 10k / 20k = 1.65V
        // ADC = 1.65 / 3.3 * 4095 ≈ 2048
        let temp = ntc.temp_c_from_adc(2048, 4095);
        assert!(temp > 20.0 && temp < 30.0, "temp was {temp}");
    }
}
