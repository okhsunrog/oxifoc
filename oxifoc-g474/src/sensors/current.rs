//! STM32G474 current sensing implementation for X-NUCLEO-IHM08M1
//!
//! Uses the generic `GenericCurrentSensor` from oxifoc-core with a
//! G474-specific raw ADC reader.
//!
//! # Hardware Setup (X-NUCLEO-IHM08M1)
//!
//! - **Shunt resistors**: 0.33Ω on phases U, V, W
//! - **OPAMP gain**: 16x (using internal OPAMPs in PGA mode)
//! - **ADC**: 12-bit injected channels, synchronized by TIM1_TRGO2
//! - **Sampling**: Phase U (ADC1), Phase V+W (ADC2)

#![allow(dead_code)]

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::{
    BoardConfig, DEFAULT_CALIBRATION_DELAY_US, DEFAULT_CALIBRATION_SAMPLES,
};
use oxifoc_core::foc::sensors::{GenericCurrentSensor, RawCurrentReader};

use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

// ============================================================================
// G474 Raw ADC Reader
// ============================================================================

/// G474-specific raw ADC reader for X-NUCLEO-IHM08M1
///
/// Reads phase currents from static atomics populated by ADC ISR.
#[derive(Clone, Copy)]
pub struct G474AdcReader;

impl RawCurrentReader for G474AdcReader {
    fn read_raw(&self) -> (u16, u16, u16) {
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        (ia_raw, ib_raw, ic_raw)
    }
}

// ============================================================================
// G474 Current Sensor (type alias)
// ============================================================================

/// G474 current sensor - generic sensor with G474-specific ADC reader
pub type G474CurrentSensor = GenericCurrentSensor<G474AdcReader>;

/// Extension trait for G474-specific calibration
pub trait G474CurrentSensorExt {
    /// Create a new G474 current sensor from board config
    fn from_board(config: &BoardConfig) -> Self;

    /// Calibrate current sense offsets (async)
    async fn calibrate(&mut self);

    /// Calibrate current sense offsets with custom parameters (async)
    async fn calibrate_with_params(&mut self, num_samples: usize, delay_us: u64);
}

impl G474CurrentSensorExt for G474CurrentSensor {
    fn from_board(config: &BoardConfig) -> Self {
        GenericCurrentSensor::from_config(config, G474AdcReader)
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
            let raw = G474AdcReader.read_raw();

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
