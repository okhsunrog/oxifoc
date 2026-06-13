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
/// Values derived from the shield schematic
/// (docs/X-NUCLEO-IHM08M1_schematic.pdf, Fig. 5) — see
/// docs/hw/nucleo-g474re-ihm08m1.md for the full pin/analog mapping.
///
/// - Driver: L6398 gate drivers + STL220N6F7 MOSFETs (60 V)
/// - Shunts: 0.010 Ω 1 W (R43/R44/R45)
/// - Current sense: TSV994 difference amp PER PHASE on the shield
///   (NOT the MCU internal OPAMPs — signals arrive conditioned):
///   Vshunt→680Ω→(+) with 6.8k bias to 3V3 via JP1, Kelvin GND→1k→(−),
///   4.7k feedback ⇒ gain ≈ 5.18 V/V, offset ≈ 1.71 V (~2122 counts).
///   ≈ 51.8 mV/A, full scale ≈ ±31 A. Requires JP1+JP2 closed and
///   C3/C5/C7 removed (FOC configuration, UM1996 §2.2.1).
///   BENCH-VERIFY: JP2 alters the feedback network — confirm effective
///   gain via zero-current offset (calibrate()) + one known current.
/// - Hardware OCP: fixed ≈30 A on raw shunts → BKIN (PA6, active low),
///   autonomous (R179/R180 divider, no firmware setup needed).
/// - VBUS divider: 169k / 9.31k ⇒ ratio 19.15
#[allow(dead_code)]
pub const BOARD: BoardConfig = BoardConfig {
    shunt_ohms: 0.01,           // 0.010Ω 1W shunts (R43-45)
    amp_gain: 5.18,             // TSV994 diff amp on shield (verify vs JP2)
    vbus_divider_ratio: 19.15,  // (169k + 9.31k) / 9.31k
    adc_vref_mv: 3300,          // 3.3V
    adc_max_counts: 4095,       // 12-bit ADC
    initial_vbus_volts: 12.0,   // Conservative default
    max_iq_target_a: 5.0,       // Conservative default for testing
    invert_current_sign: false, // Amp is non-inverting (+5.18·Vshunt + 1.71 V); verify on bench
    // Fault thresholds
    max_phase_current_a: 10.0, // Conservative limit (hw OCP trips at ~30 A)
    max_vbus_mv: 45_000,       // Max 45V (board rated 10-48V, leave margin)
    min_vbus_mv: 8_000,        // Undervoltage at 8V
    max_fet_temp_c: 85.0,      // Conservative overtemp threshold
    max_motor_temp_c: 0.0,     // no motor NTC wired
    phase_sense: None,         // X-NUCLEO-IHM08M1: no phase-voltage sensing wired
};

/// NTC configuration for X-NUCLEO-IHM08M1
/// NTC 10kΩ @ 25°C near the power FETs (schematic Fig. 4), output on PC2.
/// BENCH-VERIFY: beta is a typical value, and confirm divider topology /
/// fixed-resistor value against the board at a known temperature.
#[allow(dead_code)]
pub const NTC: NtcConfig = NtcConfig {
    r_fixed_ohm: 10_000.0,          // pull-up per schematic Fig. 4
    r0_ohm: 10_000.0,               // NTC = 10kΩ @ 25°C
    beta: 3435.0,                   // Typical NTC beta (verify)
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

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Maximum size of a single packet
pub const MAX_PACKET_SIZE: usize = 512;

/// USB liveness timeout (ms): mark the USB interface Down if no frame arrives
/// within this window. Shorter than the UART/ICD timeout for faster reaction.
pub const USB_LIVENESS_TIMEOUT_MS: u64 = 3000;

// ============================================================================
// Transport Configuration
// ============================================================================

/// Size of outgoing packet queue for USB (framed)
pub const USB_OUT_QUEUE_SIZE: usize = 2048;

/// Size of outgoing packet queue for UART (COBS stream)
pub const UART_OUT_QUEUE_SIZE: usize = 2048;

/// LPUART1 baud rate (ST-LINK VCP)
pub const UART_BAUD: u32 = 115_200;

/// UART TX/RX buffer size
pub const UART_BUF_LEN: usize = 1024;
