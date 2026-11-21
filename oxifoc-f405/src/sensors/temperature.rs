//! Exact ADC-to-temperature tables in flash: no logarithm in the control ISR.
//!
//! The build script evaluates the same core Beta model for every ADC code.
//! No interpolation, new sampling delay or runtime RAM allocation is involved.

use oxifoc_core::foc::config::NtcConfig;

include!(concat!(env!("OUT_DIR"), "/ntc_tables.rs"));

const _: () = assert!(crate::config::BOARD.calib.adc_max_counts == NTC_ADC_MAX_COUNTS);

#[inline]
pub fn board_c_x10(raw: u16) -> i16 {
    BOARD_NTC
        .get(usize::from(raw))
        .copied()
        .unwrap_or(NtcConfig::INVALID_TEMP_C_X10)
}

#[inline]
pub fn motor_c_x10(raw: u16) -> i16 {
    MOTOR_NTC
        .get(usize::from(raw))
        .copied()
        .unwrap_or(NtcConfig::INVALID_TEMP_C_X10)
}
