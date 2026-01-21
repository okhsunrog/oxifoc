//! Configuration constants and structures for oxifoc-g474 (NUCLEO-G474RE)
//!
//! Motor control configuration is placeholder until IHM08M1 shield is connected.
//! The board configuration will need to be updated based on the shield's
//! shunt resistors, voltage divider, and other hardware specifics.

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};
use oxifoc_core::foc::pwm::MotorPwmConfig;

// ============================================================================
// Board Hardware Configuration (Placeholder for IHM08M1)
// ============================================================================

/// NUCLEO-G474RE + IHM08M1 board configuration (PLACEHOLDER)
///
/// TODO: Update these values when IHM08M1 shield is connected!
/// Hardware specs will depend on the IHM08M1 shield configuration.
///
/// IHM08M1 typical specs (verify with your specific board):
/// - Shunt resistors: needs verification
/// - OPAMP gain: needs verification
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: needs verification
#[allow(dead_code)]
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.010,        // Placeholder - verify for IHM08M1
    amp_gain: 10.0,           // Placeholder - verify for IHM08M1
    vbus_divider_ratio: 10.0, // Placeholder - verify for IHM08M1
    adc_vref_mv: 3300,        // 3.3V
    adc_max_counts: 4095,     // 12-bit
    initial_vbus_volts: 12.0, // Conservative default
    max_iq_target_a: 5.0,     // Placeholder - conservative default
    // Fault thresholds (conservative placeholders)
    max_phase_current_a: 10.0, // Placeholder - verify for your motor
    max_vbus_mv: 30_000,       // Placeholder - verify for your setup
    min_vbus_mv: 8_000,        // Undervoltage at 8V
    max_fet_temp_c: 85.0,      // Conservative overtemp threshold
};

/// NTC configuration (PLACEHOLDER)
/// TODO: Verify NTC configuration for IHM08M1 shield
#[allow(dead_code)]
pub const NTC: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,
    r0_ohm: 10_000.0,
    beta: 3435.0,
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
#[allow(dead_code)]
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new();
// To change frequency: MotorPwmConfig::new().with_pwm_freq(25_000)

// ============================================================================
// Timing Configuration
// ============================================================================

/// Timebase for Hall interpolation (match embassy_time ticks)
#[allow(dead_code)]
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

/// TIM1 clock frequency in Hz (runs at SYSCLK on G4)
/// Used for motor PWM and dead time calculation.
#[allow(dead_code)]
pub const TIM1_CLOCK_HZ: u32 = 170_000_000;

/// TIM6 clock frequency in Hz (APB1 timers on G4 run at SYSCLK)
/// Embassy default: SYSCLK=170MHz
#[allow(dead_code)]
pub const TIM6_CLOCK_HZ: u32 = 170_000_000;

/// TIM6 auto-reload value for Hall sensor polling
/// Computed from: (TIM6_CLOCK_HZ / 1_000_000 * POLL_INTERVAL_US) - 1
/// At 170MHz with 5µs interval: 170 * 5 - 1 = 849
#[allow(dead_code)]
pub const TIM6_ARR: u16 = (TIM6_CLOCK_HZ / 1_000_000
    * oxifoc_core::foc::sensors::hall_polling::POLL_INTERVAL_US) as u16
    - 1;

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
