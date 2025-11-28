//! Configuration constants and structures for oxifoc-g431

/// Conservative default bus voltage used until ADC updates arrive.
pub const INITIAL_VBUS_VOLTS: f32 = 12.0;

/// Maps motor duty percent (0-100) to a target q-axis current in Amps.
pub const MAX_IQ_TARGET_A: f32 = 10.0;

/// Timebase for Hall interpolation (match embassy_time ticks).
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

/// Size of outgoing packet queue
pub const OUT_QUEUE_SIZE: usize = 2048;

/// Maximum size of a single packet
pub const MAX_PACKET_SIZE: usize = 512;

// ========== UART Transport Configuration ==========

#[cfg(feature = "transport-uart")]
pub const UART_BAUD: u32 = 115_200;

#[cfg(feature = "transport-uart")]
pub const UART_BUF_LEN: usize = 1024;

// ========== ADC Conversion Constants ==========

/// ADC maximum counts (12-bit)
pub const ADC_MAX_COUNTS: u32 = 4095;

/// ADC reference voltage in millivolts
pub const ADC_VREF_MV: u32 = 3300;

// B-G431B-ESC1 VBUS divider: 169k (top, R68) and 18k (bottom, R76).
// Vsense = Vbus * 18 / 187  =>  Vbus = Vsense * 187 / 18.
pub const VBUS_DIV_NUM: u32 = 187;
pub const VBUS_DIV_DEN: u32 = 18;

/// Convert raw ADC value to bus voltage in millivolts
pub fn vbus_mv_from_adc(raw: u16) -> u32 {
    let raw = raw as u32;
    let vsense_mv = raw * ADC_VREF_MV / ADC_MAX_COUNTS;
    vsense_mv * VBUS_DIV_NUM / VBUS_DIV_DEN
}

// ========== Temperature Sensing Constants ==========

// Temperature sensing constants for PB14 NTC divider:
//  - 10k NTC to 3.3V
//  - 4.7k resistor to GND
// Using a simple Beta model with Beta = 3455 and R0 = 10k at 25°C.
pub const NTC_R_BOTTOM_OHM: f32 = 4700.0;
pub const NTC_R0_OHM: f32 = 10_000.0;
pub const NTC_BETA: f32 = 3455.0;
pub const NTC_T0_K: f32 = 273.15 + 25.0;
pub const NTC_KELVIN_OFFSET: f32 = 273.15;

/// Convert raw ADC value to FET temperature in Celsius
pub fn fet_temp_c_from_adc(raw: u16) -> f32 {
    let adc = raw as f32;
    // Avoid divide-by-zero when ADC reading is very small.
    let eps = 0.1;
    let r_ntc = NTC_R_BOTTOM_OHM * (4096.0 / (adc + eps) - 1.0);
    // Beta-model temperature calculation.
    let ln_term = libm::logf(NTC_R0_OHM / r_ntc);
    let temp_k = NTC_BETA * NTC_T0_K / (NTC_BETA - NTC_T0_K * ln_term);
    temp_k - NTC_KELVIN_OFFSET
}
