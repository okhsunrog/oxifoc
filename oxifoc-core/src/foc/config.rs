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
///
/// const BOARD: BoardConfig = BoardConfig {
///     shunt_ohms: 0.003,           // 3mΩ shunts
///     amp_gain: 16.0,              // 16x OPAMP gain
///     vbus_divider_ratio: 10.39,   // VBUS divider
///     adc_vref_mv: 3300,           // 3.3V reference
///     adc_max_counts: 4095,        // 12-bit ADC
///     initial_vbus_volts: 12.0,    // Default VBUS assumption
///     max_iq_target_a: 10.0,       // Max torque current
/// };
/// ```
#[derive(Clone, Copy, Debug)]
pub struct BoardConfig {
    /// Shunt resistance in Ohms (e.g., 0.003 for 3mΩ)
    pub shunt_ohms: f32,
    /// Current amplifier gain (OPAMP or DRV8301 gain)
    pub amp_gain: f32,
    /// VBUS voltage divider ratio (Vbus = Vsense * ratio)
    pub vbus_divider_ratio: f32,
    /// ADC reference voltage in millivolts
    pub adc_vref_mv: u32,
    /// Maximum ADC count (e.g., 4095 for 12-bit)
    pub adc_max_counts: u16,
    /// Initial VBUS voltage assumption before ADC readings
    pub initial_vbus_volts: f32,
    /// Maximum q-axis current target in Amperes
    pub max_iq_target_a: f32,
}

impl BoardConfig {
    /// Convert raw ADC value to bus voltage in millivolts
    ///
    /// Uses the board's voltage divider ratio to scale the ADC reading.
    #[inline]
    pub fn vbus_mv_from_adc(&self, raw: u16) -> u32 {
        let raw = raw as u32;
        let vsense_mv = raw * self.adc_vref_mv / self.adc_max_counts as u32;
        (vsense_mv as f32 * self.vbus_divider_ratio) as u32
    }

    /// Convert duty percent (0-100) to target q-axis current
    ///
    /// Linear mapping from duty percentage to current target.
    #[inline]
    pub fn duty_to_iq(&self, duty: u8) -> f32 {
        let duty = duty.min(100);
        duty as f32 / 100.0 * self.max_iq_target_a
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
    /// Convert raw ADC value to temperature in Celsius
    ///
    /// Uses the Beta model: T = 1 / (ln(R/R0)/Beta + 1/T0) - 273.15
    #[inline]
    pub fn temp_c_from_adc(&self, raw: u16, adc_max_counts: u16) -> f32 {
        let adc = raw as f32;
        let adc_max = adc_max_counts as f32;

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
        shunt_ohms: 0.003,
        amp_gain: 16.0,
        vbus_divider_ratio: 10.39, // 187/18
        adc_vref_mv: 3300,
        adc_max_counts: 4095,
        initial_vbus_volts: 12.0,
        max_iq_target_a: 10.0,
    };

    #[test]
    fn test_vbus_conversion() {
        // Mid-scale ADC (1.65V) with 10.39x divider = ~17.1V
        let vbus = TEST_BOARD.vbus_mv_from_adc(2048);
        assert!(vbus > 17000 && vbus < 17500);
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
        assert!(temp > 20.0 && temp < 30.0, "temp was {}", temp);
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
        assert!(temp > 20.0 && temp < 30.0, "temp was {}", temp);
    }
}
