//! Configuration constants and structures for oxifoc-g431 (B-G431B-ESC1)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};
use oxifoc_core::foc::pwm::MotorPwmConfig;

// ============================================================================
// Board Hardware Configuration
// ============================================================================

/// B-G431B-ESC1 board configuration
///
/// Hardware specs:
/// - Shunt resistors: 3mΩ (0.003Ω) on phases A, B, C
/// - OPAMP gain: 16x (configured in hardware/peripherals.rs)
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: 169k (top, R68) / 18k (bottom, R76) = 187/18 ratio
/// - Max continuous current: ~40A (FET rating)
/// - Max VBUS: 45V (FET Vds rating with margin)
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.003,                // 3mΩ
    amp_gain: 16.0,                   // 16x OPAMP gain
    vbus_divider_ratio: 187.0 / 18.0, // 169k + 18k / 18k
    adc_vref_mv: 3300,                // 3.3V
    adc_max_counts: 4095,             // 12-bit
    initial_vbus_volts: 12.0,         // Conservative default
    max_iq_target_a: 10.0,            // Max torque current
    // Fault thresholds
    max_phase_current_a: 40.0, // Peak phase current limit (FET rating)
    max_vbus_mv: 45_000,       // Overvoltage at 45V (FET Vds margin)
    min_vbus_mv: 8_000,        // Undervoltage at 8V
    max_fet_temp_c: 100.0,     // FET overtemp at 100°C
};

/// NTC configuration for FET temperature sensing on PB14
/// High-side 10k NTC (RT1) with 4.7k pull-down (R60) to GND
pub const NTC: NtcConfig = NtcConfig {
    r_fixed_ohm: 4700.0,
    r0_ohm: 10_000.0,
    beta: 3425.0,
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::HighSide,
};

// ============================================================================
// PWM Configuration
// ============================================================================

/// Motor PWM configuration
///
/// Central source of truth for PWM frequency and timing.
/// Used by motor.rs for timer setup and control/foc.rs for dt calculation.
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(500);

// ============================================================================
// Hardware Overcurrent Protection
// ============================================================================

/// Hardware overcurrent trip threshold (amperes, peak per-phase).
/// This is the COMP+DAC hardware last-resort trip point.
/// FETs (STL180N6F7): 120A continuous, 480A pulsed. Shunts: 3mΩ.
/// Set well above software limit (40A) to avoid nuisance trips,
/// but below absolute hardware limits.
pub const HW_OVERCURRENT_A: f32 = 80.0;

/// Convert overcurrent threshold to DAC3 12-bit counts.
/// Raw shunt voltage (no OPAMP gain): V = I × R_shunt.
/// DAC LSB = 3.3V / 4096 = 0.806mV.
pub fn overcurrent_dac_counts(amps: f32) -> u16 {
    let mv = amps * BOARD.shunt_ohms * 1000.0;
    let counts = mv / (3300.0 / 4096.0);
    counts as u16
}

// ============================================================================
// Timing Configuration
// ============================================================================

/// Timebase for Hall interpolation (match embassy_time ticks)
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue
pub const OUT_QUEUE_SIZE: usize = 2048;

/// Maximum size of a single ergot packet (COBS-encoded).
/// Sized for the largest actual payload: FastTelemetryBatch<8> = 8×44 + header ≈ 380 bytes.
/// Smaller MTU reduces the OUTQ grant size (grant_exact requests max_encoding_length(MTU)),
/// leaving more room for concurrent protocol responses while streaming.
pub const MAX_PACKET_SIZE: usize = 400;

// ============================================================================
// UART Transport Configuration
// ============================================================================

#[cfg(feature = "transport-uart")]
pub const UART_BAUD: u32 = 921_600;

#[cfg(feature = "transport-uart")]
pub const UART_TX_BUF_LEN: usize = 512;

#[cfg(feature = "transport-uart")]
pub const UART_RX_BUF_LEN: usize = 1024;
