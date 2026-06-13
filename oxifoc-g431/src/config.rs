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
/// - OPAMP PGA gain: 16x, but shunt sensing circuit has 1.5kΩ series +
///   22kΩ/2.2kΩ divider that attenuates by 4/7 → effective gain = 64/7 ≈ 9.14
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: 169k (top, R68) / 18k (bottom, R76) = 187/18 ratio
/// - Max continuous current: ~40A (FET rating)
/// - Max VBUS: 45V (FET Vds rating with margin)
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.003,                // 3mΩ
    amp_gain: 64.0 / 7.0,             // 16x OPAMP × 4/7 resistor attenuation
    vbus_divider_ratio: 187.0 / 18.0, // 169k + 18k / 18k
    adc_vref_mv: 3300,                // 3.3V
    adc_max_counts: 4095,             // 12-bit
    initial_vbus_volts: 12.0,         // Conservative default
    max_iq_target_a: 10.0,            // Max torque current
    invert_current_sign: true,        // Low-side shunts: positive current → ADC below offset
    // Fault thresholds
    max_phase_current_a: 40.0, // Peak phase current limit (FET rating)
    max_vbus_mv: 45_000,       // Overvoltage at 45V (FET Vds margin)
    min_vbus_mv: 8_000,        // Undervoltage at 8V
    max_fet_temp_c: 100.0,     // FET overtemp at 100°C
    max_motor_temp_c: 0.0,     // no motor NTC wired on B-G431B-ESC1
    // No phase-voltage sensing: the B-G431B-ESC1 BEMF nets are clamped (only
    // the zero-crossing is visible), unusable for a full αβ projection.
    phase_sense: None,
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
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(800);

// ============================================================================
// Hardware Overcurrent Protection
// ============================================================================

/// Hardware overcurrent comparator (COMP1/2/4 + DAC3 → TIM1 BKIN) threshold,
/// in DAC3 12-bit counts. Set near the 3.3 V rail (4083 ≈ 3.29 V), matching ST
/// MCSDK's `M1_DAC_CURRENT_THRESHOLD = 4083` for this exact board.
///
/// WHY NEAR-RAIL (proven on hardware 2026-06-13, see docs/hw/b-g431b-esc1.md):
/// the comparators tap the **raw shunt pad** (PA1/PA7/PB0 = the OPAMP *input*,
/// NOT its output — silicon-confirmed: COMP1 INP0=PA1/INP1=PB1, no OPAMP-output
/// path). A DAC sweep at idle put that node at **128–132 mV** and its current
/// slope is only `R_shunt × 4/7` ≈ **1.71 mV/A** (the ×16 PGA gain is downstream,
/// invisible to the comparator). So a meaningful current threshold (e.g. 60 A →
/// ~231 mV, ~100 mV over idle) sits *inside* the PWM switching-noise envelope on
/// the raw shunt and nuisance-trips — which is exactly what latched a false Kill
/// OverCurrent on earlier runs. There is no usable hardware current threshold on
/// this board; ST parks the DAC at the rail so the comparator only ever fires on
/// a catastrophic pad excursion (dead short), and relies on the **software** OCP
/// — `BOARD.max_phase_current_a` (40 A), read from the ×9.14-amplified ADC signal
/// with good SNR — as the real protection. We do the same.
///
/// `pad_node_dac_counts` documents the (unusable) current→counts mapping.
pub const HW_OCP_DAC_COUNTS: u16 = 4083;

/// Reference only: DAC counts for a pad-node trip at `amps`, showing why no sane
/// current threshold is viable here. V_pad(I) = 128.6 mV (22kΩ→3.3 V / 1.5kΩ /
/// 2.2kΩ→GND bias) + I × R_shunt × 4/7 (≈1.71 mV/A). NOT used for the live
/// threshold — `HW_OCP_DAC_COUNTS` (near rail) is. Kept for documentation.
#[allow(dead_code)]
pub fn pad_node_dac_counts(amps: f32) -> u16 {
    let bias_mv = 3300.0 * (1.0 / 22.0) / (1.0 / 1.5 + 1.0 / 22.0 + 1.0 / 2.2); // ≈128.6mV
    let pad_mv = bias_mv + amps * BOARD.shunt_ohms * 1000.0 * (4.0 / 7.0);
    let counts = pad_mv / (3300.0 / 4096.0);
    if counts >= 4095.0 {
        4095
    } else {
        counts as u16
    }
}

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
