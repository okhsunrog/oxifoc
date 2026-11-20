//! Fixed thermistor networks, shared by firmware and build-time table generation.

use oxifoc_core::foc::config::{NtcConfig, NtcTopology};
use oxifoc_core::units::ohms;

pub const ADC_MAX_COUNTS: u16 = 4095;

/// PA3: high-side 10k NTC with a 10k pull-down (VESC hw60).
pub const NTC_BOARD: NtcConfig = NtcConfig {
    r_fixed: ohms(10_000.0),
    r0: ohms(10_000.0),
    beta: 3380.0,
    t0_k: 273.15 + 25.0,
    topology: NtcTopology::HighSide,
};

/// PC4: low-side 10k motor NTC with a 10k pull-up.
pub const NTC_MOTOR: NtcConfig = NtcConfig {
    topology: NtcTopology::LowSide,
    ..NTC_BOARD
};
