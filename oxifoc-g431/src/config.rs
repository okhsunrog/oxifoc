//! Configuration constants and structures for oxifoc-g431 (B-G431B-ESC1)

use oxifoc_core::foc::config::{BoardConfig, NtcConfig, NtcTopology};
use oxifoc_core::foc::pwm::MotorPwmConfig;
use oxifoc_core::types::BoardCalib;

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
    calib: BoardCalib {
        shunt_ohms: 0.003,                // 3mΩ
        amp_gain: 64.0 / 7.0,             // 16x OPAMP × 4/7 resistor attenuation
        adc_vref_mv: 3300,                // 3.3V
        adc_max_counts: 4095,             // 12-bit
        invert_current_sign: true,        // Low-side shunts: positive current → ADC below offset
        vbus_divider_ratio: 187.0 / 18.0, // 169k + 18k / 18k
    },
    initial_vbus_volts: 12.0, // Conservative default
    max_iq_target_a: 10.0,    // Max torque current
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

/// Sensorless operation: no Hall sensors are wired to this board's H1/H2/H3
/// inputs (the ZD2808 and most drone motors have none). When `true`, the boot
/// angle source is kept OFF Hall — otherwise the floating hall inputs read an
/// invalid state (0 or 7) and the firmware raises a `HallError` warning every
/// FOC cycle. Set `false` for a Hall-sensored motor on this board.
///
/// Detection is unaffected either way (it commands the electrical angle
/// directly via OpenLoop/DirectVoltage, bypassing the angle source).
pub const SENSORLESS: bool = true;

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
    let pad_mv = bias_mv + amps * BOARD.calib.shunt_ohms * 1000.0 * (4.0 / 7.0);
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

/// Size of outgoing packet queue. The cobs_stream Sink reserves
/// `max_encoding_length(MAX_PACKET_SIZE)` ≈ 1030 B per packet regardless of
/// its actual size, so this must hold several such grants for the tx path to
/// pipeline — 2048 held only 1-2 packets in flight and stalled the 20 kHz
/// stream at ~14.6k samples/s (2026-07-05 bench).
/// (2026-07-06 stack→CCM migration: 4096→3328; 3264 when the observer grew
/// the slip-gate + accel-prior state; 3200 for the freq-led commutation
/// filter; 3136 for the phase tracker's damping/feedforward state —
/// still three ~1030 B grants in flight (2048 = 1-2 grants stalled the
/// 20 kHz stream, 2026-07-05); frees 832 B toward fitting the statics
/// into the 22 K SRAM region.)
pub const OUT_QUEUE_SIZE: usize = 3136;

/// Maximum size of a single ergot packet (COBS-encoded). Outbound MTU — the
/// fast-telemetry batch must fit (compile-time assert below).
pub const MAX_PACKET_SIZE: usize = 1024;

/// Inbound (host→device) frame buffer size. Inbound traffic is commands and
/// config writes — small structs, nothing near the outbound MTU; the largest
/// today is a config-group write well under 256 B. Split from
/// MAX_PACKET_SIZE in the stack→CCM migration to halve RECV_BUF.
pub const MAX_RX_PACKET_SIZE: usize = 512;

// A raw-Pod fast-telemetry batch (fixed wire size) must fit one packet with
// room for the ergot header + length varint + COBS overhead. A varint-encoded
// batch could straddle the MTU depending on VALUES and died silently there
// (commit e1f65b5); raw-Pod makes the fit checkable here, at compile time.
const _: () = assert!(
    oxifoc_core::types::FAST_BATCH_BYTES + 64 <= MAX_PACKET_SIZE,
    "fast-telemetry batch does not fit MAX_PACKET_SIZE"
);

// ============================================================================
// UART Transport Configuration
// ============================================================================

#[cfg(feature = "transport-uart")]
pub const UART_BAUD: u32 = 921_600;

#[cfg(feature = "transport-uart")]
pub const UART_TX_BUF_LEN: usize = 512;

#[cfg(feature = "transport-uart")]
pub const UART_RX_BUF_LEN: usize = 1024;
