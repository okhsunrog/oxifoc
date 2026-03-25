//! Configuration constants and structures for oxifoc-g474 (NUCLEO-G474RE + X-NUCLEO-IHM08M1)
//!
//! X-NUCLEO-IHM08M1 hardware configuration.

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};
use oxifoc_core::foc::pwm::MotorPwmConfig;

// ============================================================================
// Board Hardware Configuration (X-NUCLEO-IHM08M1)
// ============================================================================

/// NUCLEO-G474RE + X-NUCLEO-IHM08M1 board configuration
///
/// X-NUCLEO-IHM08M1 hardware specs:
/// - Driver: L6398 gate driver + STL220N6F7 MOSFETs
/// - Shunt resistors: 0.33Ω (R7, R8, R12 on schematic)
/// - Current sense: direct ADC (no external op-amp, uses MCU internal OPAMPs)
/// - VBUS divider: 1:19.18 (R5=560k, R6=30.9k) -> ratio ~19.12
/// - Max voltage: 45V (limited by STL220N6F7)
/// - Max current: ~15A peak (limited by shunt power dissipation)
#[allow(dead_code)]
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.33,          // 0.33Ω shunt resistors
    amp_gain: 16.0,            // Using internal OPAMP with 16x gain (PGA mode)
    vbus_divider_ratio: 19.12, // (560k + 30.9k) / 30.9k = 19.12
    adc_vref_mv: 3300,         // 3.3V
    adc_max_counts: 4095,      // 12-bit ADC
    initial_vbus_volts: 12.0,  // Conservative default
    max_iq_target_a: 5.0,      // Conservative default for testing
    invert_current_sign: false, // TODO: verify for this board
    // Fault thresholds
    max_phase_current_a: 10.0, // Conservative limit
    max_vbus_mv: 45_000,       // Max 45V for STL220N6F7
    min_vbus_mv: 8_000,        // Undervoltage at 8V
    max_fet_temp_c: 85.0,      // Conservative overtemp threshold
};

/// NTC configuration for X-NUCLEO-IHM08M1
/// NTC1 on board: 10kΩ @ 25°C, Beta ~3435K (typical NTC)
#[allow(dead_code)]
pub const NTC: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,          // R13 = 10kΩ pull-up
    r0_ohm: 10_000.0,               // NTC = 10kΩ @ 25°C
    beta: 3435.0,                   // Typical NTC beta
    t0_k: 273.15 + 25.0,            // Reference temp 25°C
    topology: NtcTopology::LowSide, // NTC to GND, pull-up to VCC
};

// ============================================================================
// PWM Configuration
// ============================================================================

/// Motor PWM configuration
///
/// Central source of truth for PWM frequency and timing.
/// Used by motor.rs for timer setup and control/foc.rs for dt calculation.
#[allow(dead_code)]
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new();
// To change frequency: MotorPwmConfig::new().with_pwm_freq(25_000)

// ============================================================================
// Timing Configuration
// ============================================================================

/// Timebase for Hall interpolation (match embassy_time ticks)
#[allow(dead_code)]
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue
pub const OUT_QUEUE_SIZE: usize = 2048;

/// Maximum size of a single packet
pub const MAX_PACKET_SIZE: usize = 512;

// ============================================================================
// UART Transport Configuration
// ============================================================================

#[cfg(feature = "transport-uart")]
pub const UART_BAUD: u32 = 115_200;

#[cfg(feature = "transport-uart")]
pub const UART_BUF_LEN: usize = 1024;
