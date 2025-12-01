//! Configuration constants for oxifoc-f405 (Simple FOCer 2 / Cheap FOCer 2)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};

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
// Timing Configuration
// ============================================================================

/// Embassy timebase ticks per second
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue for ergot over USB
pub const OUT_QUEUE_SIZE: usize = 4096;

/// Maximum packet size for ergot framing
pub const MAX_PACKET_SIZE: usize = 512;
