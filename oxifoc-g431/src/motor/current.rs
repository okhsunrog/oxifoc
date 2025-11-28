//! STM32G431 current sensing implementation
//!
//! Wraps the platform-agnostic `ShuntCurrentSense` with G431-specific
//! ADC reading from injected channels.
//!
//! # Hardware Setup (B-G431B-ESC1)
//!
//! - **Shunt resistors**: 3mΩ (0.003Ω) on phases A, B, C
//! - **OPAMP gain**: 16x (configured in main.rs)
//! - **ADC**: 12-bit injected channels, synchronized by TIM1_TRGO2
//! - **Sampling**: Phase A (ADC1), Phase B+C (ADC2)
//!
//! # Calibration
//!
//! Before using current readings, call `calibrate()` with motor disabled
//! (PWM off, no current flow). This measures the zero-current ADC offsets.
#![allow(dead_code)]

use core::sync::atomic::Ordering;

use oxifoc_core::foc::current_sense::ShuntCurrentSense;
use oxifoc_core::foc::sensors::CurrentSensor;

use super::super::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

/// G431-specific current sensor
///
/// Reads raw ADC samples from global atomics and converts to calibrated
/// current measurements using the platform-agnostic `ShuntCurrentSense`.
pub struct G431CurrentSensor {
    /// Core conversion logic
    converter: ShuntCurrentSense,
}

impl G431CurrentSensor {
    /// Create a new G431 current sensor
    ///
    /// Uses B-G431B-ESC1 hardware specifications:
    /// - 3mΩ shunt resistors
    /// - 16x OPAMP gain
    /// - 12-bit ADC with 3.3V reference
    pub fn new() -> Self {
        const SHUNT_OHMS: f32 = 0.003; // 3mΩ
        const OPAMP_GAIN: f32 = 16.0; // 16x gain
        const ADC_VREF_MV: u32 = 3300; // 3.3V
        const ADC_MAX: u16 = 4095; // 12-bit

        Self {
            converter: ShuntCurrentSense::new(SHUNT_OHMS, OPAMP_GAIN, ADC_VREF_MV, ADC_MAX),
        }
    }

    /// Get current calibration offsets
    ///
    /// Returns (offset_a, offset_b, offset_c) in ADC counts.
    /// Useful for saving/loading calibration.
    pub fn get_offsets(&self) -> (f32, f32, f32) {
        self.converter.get_offsets()
    }

    /// Manually set calibration offsets
    ///
    /// Useful for loading saved calibration values.
    ///
    /// # Arguments
    /// * `offset_a` - Phase A zero-current ADC value
    /// * `offset_b` - Phase B zero-current ADC value
    /// * `offset_c` - Phase C zero-current ADC value
    pub fn set_offsets(&mut self, offset_a: f32, offset_b: f32, offset_c: f32) {
        self.converter.set_offsets(offset_a, offset_b, offset_c);
    }

    /// Read raw ADC samples from hardware
    ///
    /// Returns (adc_a, adc_b, adc_c)
    fn read_raw_adc(&self) -> (u16, u16, u16) {
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        (ia_raw, ib_raw, ic_raw)
    }
}

impl Default for G431CurrentSensor {
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentSensor for G431CurrentSensor {
    fn read_currents(&mut self) -> (f32, f32, f32) {
        let (adc_a, adc_b, adc_c) = self.read_raw_adc();
        self.converter.convert_raw(adc_a, adc_b, adc_c)
    }

    fn calibrate(&mut self, samples: usize) {
        defmt::info!(
            "Starting current sensor calibration with {} samples",
            samples
        );

        // Collect samples with motor disabled
        let mut sample_buffer = heapless::Vec::<(u16, u16, u16), 1000>::new();

        for i in 0..samples.min(1000) {
            let (adc_a, adc_b, adc_c) = self.read_raw_adc();
            let _ = sample_buffer.push((adc_a, adc_b, adc_c));

            if i % 100 == 0 {
                defmt::debug!(
                    "Calibration sample {}: A={} B={} C={}",
                    i,
                    adc_a,
                    adc_b,
                    adc_c
                );
            }

            // Small delay between samples
            cortex_m::asm::delay(1000); // ~1μs at 170MHz
        }

        // Calibrate using collected samples
        self.converter.calibrate_offsets(&sample_buffer);

        let (oa, ob, oc) = self.converter.get_offsets();
        defmt::info!(
            "Current sensor calibrated: offset_a={=f32} offset_b={=f32} offset_c={=f32}",
            oa,
            ob,
            oc
        );
    }

    fn is_calibrated(&self) -> bool {
        self.converter.is_calibrated()
    }
}
