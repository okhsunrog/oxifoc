//! Configuration constants for oxifoc-f405 (Simple FOCer 2 / Cheap FOCer 2)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};
use oxifoc_core::foc::pwm::MotorPwmConfig;

// ============================================================================
// Board Hardware Configuration
// ============================================================================

/// Cheap FOCer 2 / Simple FOCer 2 board configuration
///
/// Hardware specs:
/// - Shunt resistors: 2x 1mΩ in parallel = 0.5mΩ effective
/// - DRV8301 amp gain: 10 V/V (for Cheap FOCer 2 v1.0, 20 V/V for v0.9)
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: 39k / 2.2k = (39 + 2.2) / 2.2 ratio
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.0005,                     // 0.5mΩ (two 1mΩ in parallel)
    amp_gain: 10.0,                         // DRV8301 10 V/V gain
    vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    adc_vref_mv: 3300,                      // 3.3V
    adc_max_counts: 4095,                   // 12-bit
    initial_vbus_volts: 12.0,               // Conservative default
    max_iq_target_a: 10.0,                  // Max torque current
};

/// On-board PCB/FET NTC thermistor configuration (PA3, ADC123_IN3)
/// High-side 10k NTC with 10k pull-down resistor
/// From VESC: NTC_RES(adc_val) = (4095.0 * 10000.0) / adc_val - 10000.0
pub const NTC_BOARD: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,
    r0_ohm: 10_000.0,
    beta: 3380.0,
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::HighSide,
};

/// Motor temperature NTC configuration (PC4, ADC12_IN14)
/// Low-side 10k NTC with 10k pull-up resistor (external motor thermistor)
/// From VESC: NTC_RES_MOTOR(adc_val) = 10000.0 / ((4095.0 / adc_val) - 1.0)
pub const NTC_MOTOR: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,
    r0_ohm: 10_000.0,
    beta: 3380.0, // Default, typically configured per motor
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::LowSide,
};

// ============================================================================
// PWM Configuration
// ============================================================================

/// Motor PWM configuration
///
/// Central source of truth for PWM frequency and timing.
/// Used by motor.rs for timer setup and control/foc.rs for dt calculation.
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new();
// To change frequency: MotorPwmConfig::new().with_pwm_freq(25_000)

// ============================================================================
// Timing Configuration
// ============================================================================

/// Embassy timebase ticks per second
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

/// TIM1 clock frequency in Hz (APB2 timers run at SYSCLK on F405)
/// Used for motor PWM and dead time calculation.
pub const TIM1_CLOCK_HZ: u32 = 168_000_000;

/// TIM6 clock frequency in Hz (APB1 timers run at 2x APB1 when prescaler > 1)
/// Embassy default: SYSCLK=168MHz, APB1=42MHz, timer clock=84MHz
pub const TIM6_CLOCK_HZ: u32 = 84_000_000;

/// TIM6 auto-reload value for Hall sensor polling
/// Computed from: (TIM6_CLOCK_HZ / 1_000_000 * POLL_INTERVAL_US) - 1
/// At 84MHz with 5µs interval: 84 * 5 - 1 = 419
pub const TIM6_ARR: u16 = (TIM6_CLOCK_HZ / 1_000_000
    * oxifoc_core::foc::sensors::hall_polling::POLL_INTERVAL_US) as u16
    - 1;

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue for ergot over USB
pub const OUT_QUEUE_SIZE: usize = 4096;

/// Maximum packet size for ergot framing
pub const MAX_PACKET_SIZE: usize = 512;
