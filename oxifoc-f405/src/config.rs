//! Configuration constants for oxifoc-f405 (per-board `BOARD` const below)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology, PhaseSense};
use oxifoc_core::foc::pwm::MotorPwmConfig;
use oxifoc_core::types::BoardCalib;

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
#[cfg(feature = "board-cf2")]
pub const BOARD: BoardConfig = BoardConfig {
    calib: BoardCalib {
        shunt_ohms: 0.0005,                     // 0.5mΩ (two 1mΩ in parallel)
        amp_gain: 10.0,                         // DRV8301 10 V/V gain
        adc_vref_mv: 3300,                      // 3.3V
        adc_max_counts: 4095,                   // 12-bit
        invert_current_sign: false,             // DRV8301: standard polarity
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0, // Max torque current
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

/// VESC 6 MK5 (Trampa layout + clones; the bench unit is a Flipsky
/// "Mini V6 MK5") board configuration — facts from VESC hwconf
/// hw_60_core.h + the Flipsky listing, see docs/hw/vesc6-mk5.md.
///
/// Hardware specs:
/// - In-line phase shunts (HW_HAS_PHASE_SHUNTS): 0.5mΩ, standard polarity
/// - DRV8301 amp gain: 20 V/V (VESC CURRENT_AMP_GAIN, all hw60 marks)
/// - VBUS divider: 39k / 2.2k, same on SENS1-3
/// - Flipsky rating: 70A continuous / 200A instantaneous, 14-60V
///   (original Trampa hw60: ±120A, VIN 6-57V) — NB 14V operating minimum,
///   power the bench from ≥14V
#[cfg(feature = "board-vesc6-mk5")]
pub const BOARD: BoardConfig = BoardConfig {
    calib: BoardCalib {
        shunt_ohms: 0.0005, // 0.5mΩ in-line phase shunts
        amp_gain: 20.0,     // DRV8301 20 V/V gain (must match drv8301.rs SHUNT_AMP_GAIN)
        adc_vref_mv: 3300,
        adc_max_counts: 4095,
        invert_current_sign: false, // hw60 does NOT define INVERTED_SHUNT_POLARITY
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0,     // Max torque current
    max_phase_current_a: 70.0, // Flipsky continuous rating (200A "instantaneous" ignored)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN; vendor abs max 60V)
    min_vbus_mv: 6_000,        // UV fault floor (VESC HW_LIM_VIN; operating min is 14V)
    max_fet_temp_c: 100.0,     // Conservative (original hw60 cutoff: 110°C)
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus
    // divider. MK5 has switchable RC phase filters (enabled via PC13 at
    // boot) → usable while PWMing, unlike CF2.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: true,
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
// VESC HW_DEAD_TIME_NSEC fallback: 360ns — neither CF2 nor hw60 overrides it

// ============================================================================
// Timing Configuration
// ============================================================================

/// Core clock after PLL init — MUST match the RCC setup in
/// hardware/peripherals.rs (8 MHz HSE /4 ×168 /2 = 168 MHz).
/// Single source for all cycle-based timing: the ISR cycle budget,
/// busy-wait delays (DRV8301 tWAKE, bit-bang SPI), load percentages.
pub const CPU_HZ: u32 = 168_000_000;

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
