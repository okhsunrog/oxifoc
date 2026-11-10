//! Exact ADC-to-temperature tables in flash: no logarithm in the control ISR.
//!
//! The build script evaluates the same core Beta model for every ADC code.
//! No interpolation, new sampling delay or runtime RAM allocation is involved.

use oxifoc_core::foc::config::NtcConfig;

include!(concat!(env!("OUT_DIR"), "/ntc_tables.rs"));

const _: () = assert!(crate::config::BOARD.calib.adc_max_counts == 4095);

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

/// Exhaustively compare the host-generated entries with the MCU's float math
/// before PWM is initialized. A mismatch aborts boot, with the bridge off.
#[cfg(feature = "bench-ntc-check")]
pub fn verify_on_target() {
    use super::temperature_config::{ADC_MAX_COUNTS, NTC_BOARD, NTC_MOTOR};
    for raw in 0..=ADC_MAX_COUNTS {
        let raw = core::hint::black_box(raw);
        defmt::assert_eq!(
            board_c_x10(raw),
            NTC_BOARD.temp_c_x10_from_adc(raw, ADC_MAX_COUNTS)
        );
        defmt::assert_eq!(
            motor_c_x10(raw),
            NTC_MOTOR.temp_c_x10_from_adc(raw, ADC_MAX_COUNTS)
        );
    }
    defmt::assert_eq!(board_c_x10(u16::MAX), NtcConfig::INVALID_TEMP_C_X10);
    defmt::assert_eq!(motor_c_x10(u16::MAX), NtcConfig::INVALID_TEMP_C_X10);
    NTC_CHECK_COUNT.store(8194, core::sync::atomic::Ordering::Release);
}

#[cfg(feature = "bench-ntc-check")]
#[unsafe(no_mangle)]
pub static NTC_CHECK_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
