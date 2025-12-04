//! STM32F405 current sensing implementation
//!
//! Uses the generic `GenericCurrentSensor` from oxifoc-core with an
//! F405-specific raw ADC reader.
//!
//! # Hardware Setup (Simple FOCer 2 / Cheap FOCer 2)
//!
//! - **Shunt resistors**: 2x 1mΩ in parallel = 0.5mΩ effective
//! - **DRV8301 gain**: 10 V/V
//! - **ADC**: 12-bit injected channels, synchronized by TIM1_CC4
//! - **Sampling**: Phase A (ADC1 PC0), Phase B (ADC2 PC1), Phase C (ADC3 PC2)

#![allow(dead_code)]

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::{
    BoardConfig, DEFAULT_CALIBRATION_DELAY_US, DEFAULT_CALIBRATION_SAMPLES,
};
use oxifoc_core::foc::sensors::{GenericCurrentSensor, RawCurrentReader};

use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

// ============================================================================
// F405 Raw ADC Reader
// ============================================================================

/// F405-specific raw ADC reader
///
/// Reads phase currents from static atomics populated by ADC ISR.
#[derive(Clone, Copy)]
pub struct F405AdcReader;

impl RawCurrentReader for F405AdcReader {
    fn read_raw(&self) -> (u16, u16, u16) {
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        (ia_raw, ib_raw, ic_raw)
    }
}

// ============================================================================
// F405 Current Sensor (type alias)
// ============================================================================

/// F405 current sensor - generic sensor with F405-specific ADC reader
pub type F405CurrentSensor = GenericCurrentSensor<F405AdcReader>;

/// Extension trait for F405-specific calibration
pub trait F405CurrentSensorExt {
    /// Create a new F405 current sensor from board config
    fn from_board(config: &BoardConfig) -> Self;

    /// Calibrate current sense offsets (async)
    async fn calibrate(&mut self);

    /// Calibrate current sense offsets with custom parameters (async)
    async fn calibrate_with_params(&mut self, num_samples: usize, delay_us: u64);
}

impl F405CurrentSensorExt for F405CurrentSensor {
    fn from_board(config: &BoardConfig) -> Self {
        GenericCurrentSensor::from_config(config, F405AdcReader)
    }

    async fn calibrate(&mut self) {
        self.calibrate_with_params(DEFAULT_CALIBRATION_SAMPLES, DEFAULT_CALIBRATION_DELAY_US)
            .await;
    }

    async fn calibrate_with_params(&mut self, num_samples: usize, delay_us: u64) {
        defmt::info!(
            "Calibrating current sense: {} samples, {}us delay",
            num_samples,
            delay_us
        );

        // Collect samples
        let mut samples = heapless::Vec::<(u16, u16, u16), 1024>::new();

        for i in 0..num_samples.min(1024) {
            let raw = F405AdcReader.read_raw();

            let _ = samples.push(raw);

            if i % 64 == 0 {
                defmt::debug!(
                    "Calibration sample {}: A={} B={} C={}",
                    i,
                    raw.0,
                    raw.1,
                    raw.2
                );
            }

            Timer::after(Duration::from_micros(delay_us)).await;
        }

        // Use shared calibration algorithm from oxifoc-core
        self.calibrate_offsets(&samples);

        let (oa, ob, oc) = self.converter().get_offsets();
        defmt::info!(
            "Current sense calibrated: A={} B={} C={}",
            oa as u16,
            ob as u16,
            oc as u16
        );
    }
}
