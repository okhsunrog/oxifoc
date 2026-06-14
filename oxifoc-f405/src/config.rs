//! Configuration constants for oxifoc-f405 (Simple FOCer 2 / Cheap FOCer 2)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology, PhaseSense};
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
/// - Max continuous current: ~60A (FET rating, IRFS7530)
/// - Max VBUS: 57V (VESC HW_LIM_VIN, FET Vds=60V with margin)
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.0005,                     // 0.5mΩ (two 1mΩ in parallel)
    amp_gain: 10.0,                         // DRV8301 10 V/V gain
    vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    adc_vref_mv: 3300,                      // 3.3V
    adc_max_counts: 4095,                   // 12-bit
    initial_vbus_volts: 12.0,               // Conservative default
    max_iq_target_a: 10.0,                  // Max torque current
    invert_current_sign: false,             // DRV8301: standard polarity
    // Fault thresholds (matched to VESC hwconf)
    max_phase_current_a: 60.0, // Peak phase current limit (FET rating)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN)
    min_vbus_mv: 6_000,        // Undervoltage at 6V (VESC HW_LIM_VIN)
    max_fet_temp_c: 100.0,     // FET overtemp at 100°C
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus divider
    // (39k/2.2k). No RC phase filters on this board → usable only undriven
    // (back-EMF / coasting-rotation detection), not while PWMing.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: false,
    }),
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
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(360);
// VESC default dead time for Cheap FOCer 2: 360ns (HW_DEAD_TIME_NSEC fallback)

// ============================================================================
// Timing Configuration
// ============================================================================

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue for ergot over USB (framed)
pub const USB_OUT_QUEUE_SIZE: usize = 4096;

/// Size of outgoing packet queue for ergot over UART (COBS stream)
pub const UART_OUT_QUEUE_SIZE: usize = 4096;

/// Size of outgoing packet queue for ergot over RTT (COBS stream)
pub const RTT_OUT_QUEUE_SIZE: usize = 4096;

/// Maximum packet size for ergot framing
pub const MAX_PACKET_SIZE: usize = 512;

/// UART baud rate for USART3 (Cheap FOCer 2 external connector)
pub const UART_BAUD: u32 = 921_600;

/// UART TX buffer size
pub const UART_TX_BUF_LEN: usize = 2048;

/// UART RX buffer size
pub const UART_RX_BUF_LEN: usize = 512;
