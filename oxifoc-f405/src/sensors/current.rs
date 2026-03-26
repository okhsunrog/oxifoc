//! STM32F405 current sensing implementation
//!
//! Uses the generic `EmbassyCurrentSensor` from oxifoc-core, which reads
//! from the platform's ADC sample atomics.
//!
//! # Hardware Setup (Simple FOCer 2 / Cheap FOCer 2)
//!
//! - **Shunt resistors**: 2x 1mΩ in parallel = 0.5mΩ effective
//! - **DRV8301 gain**: 10 V/V
//! - **ADC**: 12-bit injected channels, synchronized by TIM1_CC4
//! - **Sampling**: Phase A (ADC1 PC0), Phase B (ADC2 PC1), Phase C (ADC3 PC2)

pub use oxifoc_core::foc::sensors::EmbassyCurrentSensor as F405CurrentSensor;
pub use oxifoc_core::foc::sensors::EmbassyCurrentSensorExt as F405CurrentSensorExt;
