//! Generic current sensing for STM32G4 platforms
//!
//! Provides [`G4AdcReader`] and [`G4CurrentSensor`] that work with any G4 target
//! by taking references to the platform's ADC sample atomics.

#![allow(dead_code)]

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::{
    BoardConfig, DEFAULT_CALIBRATION_DELAY_US, DEFAULT_CALIBRATION_SAMPLES,
};
use oxifoc_core::foc::sensors::{CurrentSensor, GenericCurrentSensor, RawCurrentReader};

// ============================================================================
// G4 Raw ADC Reader
// ============================================================================

/// G4-family raw ADC reader
///
/// Reads phase currents from static atomics populated by ADC ISR.
/// Stores references to platform-specific atomics.
#[derive(Clone, Copy)]
pub struct G4AdcReader {
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
}

impl G4AdcReader {
    pub const fn new(
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
    ) -> Self {
        Self { ia, ib, ic }
    }
}

impl RawCurrentReader for G4AdcReader {
    fn read_raw(&self) -> (u16, u16, u16) {
        let ia_raw = self.ia.load(Ordering::Relaxed);
        let ib_raw = self.ib.load(Ordering::Relaxed);
        let ic_raw = self.ic.load(Ordering::Relaxed);
        (ia_raw, ib_raw, ic_raw)
    }
}

// ============================================================================
// G4 Current Sensor
// ============================================================================

/// G4 current sensor - generic sensor with G4 ADC reader
pub type G4CurrentSensor = GenericCurrentSensor<G4AdcReader>;

/// Extension trait for G4-family current sensor calibration
#[allow(async_fn_in_trait)]
pub trait G4CurrentSensorExt {
    /// Create a new G4 current sensor from board config and ADC atomics
    fn from_board(
        config: &BoardConfig,
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
    ) -> Self;

    /// Calibrate current sense offsets (async)
    async fn calibrate(&mut self);

    /// Calibrate current sense offsets with custom parameters (async)
    async fn calibrate_with_params(&mut self, num_samples: usize, delay_us: u64);
}

impl G4CurrentSensorExt for G4CurrentSensor {
    fn from_board(
        config: &BoardConfig,
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
    ) -> Self {
        GenericCurrentSensor::from_config(config, G4AdcReader::new(ia, ib, ic))
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
            let raw = self.read_raw();

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
