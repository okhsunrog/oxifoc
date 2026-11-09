//! Prepare estimator state outside the control interrupt, then install it once.

use super::{PhaseManager, PhaseTracker};
use crate::foc::phase::PhaseSource;
#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::phase::observer::{BackEmfObserver, Observer};
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
use crate::storage::RuntimeConfig;

/// Fully initialized estimator model. Construction (including filter setup)
/// belongs to boot or the configuration task, never to the periodic FOC path.
pub struct PreparedPhaseModel {
    reset_hall: Option<crate::foc::hall_sensor::PreparedHallCalibration>,
    observer: Observer,
    #[cfg(feature = "hfi")]
    hfi: Option<HfiObserver>,
    tracker: PhaseTracker,
}

impl PreparedPhaseModel {
    pub fn from_config(config: &RuntimeConfig, vbus: f32) -> Self {
        let mut prepared = Self {
            reset_hall: None,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            tracker: PhaseTracker::default(),
        };
        if let Some(mp) = &config.motor_params
            && mp.is_valid()
            && mp.flux_linkage_wb > 0.0
        {
            let l_avg = (mp.inductance_d_h + mp.inductance_q_h) / 2.0;
            let mut obs = BackEmfObserver::new(mp.resistance_ohm, l_avg, mp.flux_linkage_wb)
                .with_lambda_tracking(crate::foc::phase::observer::DEFAULT_LAMBDA_GAIN);
            if (mp.inductance_q_h - mp.inductance_d_h).abs() > 0.05 * l_avg {
                obs = obs.with_saliency(mp.inductance_d_h, mp.inductance_q_h);
            }
            prepared.observer = Observer::BackEmf(obs);
            prepared.tracker.kp = 100.0 * 100.0;
            prepared.tracker.kd = 2.0 * 1.2 * 100.0;
            #[cfg(feature = "hfi")]
            {
                use crate::foc::phase::observer::{
                    HFI_CARRIER_RIPPLE_TARGET_A, HFI_DEFAULT_AMPLITUDE_RATIO, HFI_DEFAULT_FREQ_HZ,
                    HFI_RIPPLE_RATING_FRACTION,
                };
                let target = mp
                    .rating_current_a()
                    .map_or(HFI_CARRIER_RIPPLE_TARGET_A, |rating| {
                        crate::foc::clamp_f32(
                            HFI_RIPPLE_RATING_FRACTION * rating,
                            0.05,
                            HFI_CARRIER_RIPPLE_TARGET_A,
                        )
                    });
                let amplitude = (target * HFI_DEFAULT_FREQ_HZ * core::f32::consts::TAU * l_avg)
                    .min(vbus * HFI_DEFAULT_AMPLITUDE_RATIO)
                    .max(0.05);
                prepared.hfi = Some(HfiObserver::new(HFI_DEFAULT_FREQ_HZ, amplitude));
            }
            #[cfg(not(feature = "hfi"))]
            let _ = vbus;
        }
        prepared
    }

    pub(crate) fn reset_hall_reference(&mut self) {
        self.reset_hall = Some(crate::foc::hall_sensor::PreparedHallCalibration::nominal());
    }

    pub(crate) fn hall_reset(&self) -> Option<&crate::foc::hall_sensor::PreparedHallCalibration> {
        self.reset_hall.as_ref()
    }
}

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Replace all model-dependent state, retaining hardware sensors and the
    /// selected source whenever it is still available. A partial model must
    /// clear the previous motor's estimators rather than silently retaining it.
    pub fn apply_prepared_model(&mut self, prepared: &mut PreparedPhaseModel) -> PhaseSource {
        self.observer = core::mem::take(&mut prepared.observer);
        #[cfg(feature = "hfi")]
        {
            self.hfi = prepared.hfi.take().map(HfiObserver::with_sincos);
            self.hfi_was_active = false;
        }
        self.tracker = core::mem::take(&mut prepared.tracker);
        let old_source = self.source;
        if self.set_source(old_source).is_err() {
            // Manual is always available and does not consult unwired sensors.
            let _ = self.set_source(PhaseSource::Manual);
        }
        self.hall_control_motion.reset();
        self.source
    }

    /// Compatibility entry point for standalone users; platforms and live
    /// configuration both use the driver's complete prepared-model path.
    pub fn configure_observers_from_config(&mut self, config: &RuntimeConfig, vbus: f32) {
        self.apply_prepared_model(&mut PreparedPhaseModel::from_config(config, vbus));
    }
}
