//! STM32F405 current sensing implementation
//!
//! Wraps the platform-agnostic `ShuntCurrentSense` with F405-specific
//! ADC reading from injected channels.
//!
//! # Hardware Setup (Simple FOCer 2 / Cheap FOCer 2)
//!
//! - **Shunt resistors**: 2x 1mΩ in parallel = 0.5mΩ effective
//! - **DRV8301 gain**: 10 V/V
//! - **ADC**: 12-bit injected channels, synchronized by TIM1_CC4
//! - **Sampling**: Phase A (ADC1 PC0), Phase B (ADC2 PC1), Phase C (ADC3 PC2)
//!
//! # Calibration
//!
//! Before using current readings, call `calibrate()` with motor disabled
//! (PWM off, no current flow). This measures the zero-current ADC offsets.
//!
//! Uses `ShuntCurrentSense` from oxifoc-core for ADC-to-current conversion,
//! and implements the `CurrentSensor` trait for a unified interface.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::{
    BoardConfig, DEFAULT_CALIBRATION_DELAY_US, DEFAULT_CALIBRATION_SAMPLES,
};
use oxifoc_core::foc::current_sense::ShuntCurrentSense;
use oxifoc_core::foc::sensors::CurrentSensor;

use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

// ============================================================================
// F405 Current Sensor (implements CurrentSensor trait)
// ============================================================================

/// F405 current sensor implementation
///
/// Reads phase currents from ADC via static atomics and converts to Amperes
/// using `ShuntCurrentSense` from oxifoc-core.
pub struct F405CurrentSensor {
    /// Core conversion logic
    converter: ShuntCurrentSense,
}

impl F405CurrentSensor {
    /// Create a new F405 current sensor
    ///
    /// Uses board configuration for hardware parameters.
    pub fn new(config: &BoardConfig) -> Self {
        Self {
            converter: ShuntCurrentSense::new(
                config.shunt_ohms,
                config.amp_gain,
                config.adc_vref_mv,
                config.adc_max_counts,
            ),
        }
    }

    /// Manually set calibration offsets
    ///
    /// Useful for loading saved calibration values.
    pub fn set_offsets(&mut self, offset_a: f32, offset_b: f32, offset_c: f32) {
        self.converter.set_offsets(offset_a, offset_b, offset_c);
    }

    /// Calibrate current sense offsets (async)
    ///
    /// Samples ADC values over time and computes zero-current offsets.
    /// Call this with motor disabled (no current flowing).
    ///
    /// Uses `ShuntCurrentSense::calibrate_offsets()` from oxifoc-core for the
    /// offset calculation algorithm.
    pub async fn calibrate(&mut self) {
        self.calibrate_with_params(DEFAULT_CALIBRATION_SAMPLES, DEFAULT_CALIBRATION_DELAY_US)
            .await;
    }

    /// Calibrate current sense offsets with custom parameters (async)
    ///
    /// # Arguments
    /// * `num_samples` - Number of samples to collect (more = more accurate, slower)
    /// * `delay_us` - Delay between samples in microseconds
    pub async fn calibrate_with_params(&mut self, num_samples: usize, delay_us: u64) {
        defmt::info!(
            "Calibrating current sense: {} samples, {}us delay",
            num_samples,
            delay_us
        );

        // Collect samples
        let mut samples = heapless::Vec::<(u16, u16, u16), 1024>::new();

        for i in 0..num_samples.min(1024) {
            let (raw_a, raw_b, raw_c) = self.read_raw();

            let _ = samples.push((raw_a, raw_b, raw_c));

            if i % 64 == 0 {
                defmt::debug!(
                    "Calibration sample {}: A={} B={} C={}",
                    i,
                    raw_a,
                    raw_b,
                    raw_c
                );
            }

            Timer::after(Duration::from_micros(delay_us)).await;
        }

        // Use shared calibration algorithm from oxifoc-core
        self.converter.calibrate_offsets(&samples);

        let (oa, ob, oc) = self.converter.get_offsets();
        defmt::info!(
            "Current sense calibrated: A={} B={} C={}",
            oa as u16,
            ob as u16,
            oc as u16
        );
    }
}

impl CurrentSensor for F405CurrentSensor {
    fn read_currents(&self) -> (f32, f32, f32) {
        let (adc_a, adc_b, adc_c) = self.read_raw();
        self.converter.convert_raw(adc_a, adc_b, adc_c)
    }

    fn read_raw(&self) -> (u16, u16, u16) {
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        (ia_raw, ib_raw, ic_raw)
    }

    fn is_calibrated(&self) -> bool {
        self.converter.is_calibrated()
    }

    fn get_offsets(&self) -> (f32, f32, f32) {
        self.converter.get_offsets()
    }
}
