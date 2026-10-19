//! Arm observers from a stored runtime config.

use super::PhaseManager;
#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
#[cfg(feature = "storage")]
use crate::storage::RuntimeConfig;
#[cfg(feature = "hfi")]
use core::f32::consts::TAU;

// ============================================================================
// Configuration from stored runtime config
// ============================================================================

#[cfg(feature = "storage")]
impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Configure the software estimators from stored motor parameters.
    ///
    /// With valid detected params both slots are armed: a back-EMF observer
    /// (R, L_avg, λ) and an HFI estimator with default carrier settings.
    /// The sources stay untouched — the estimators only run; the host
    /// selects a sensorless source explicitly when it wants one.
    #[cfg_attr(not(feature = "hfi"), allow(unused_variables))]
    pub fn configure_observers_from_config(&mut self, config: &RuntimeConfig, vbus: f32) {
        use crate::foc::phase::observer::{BackEmfObserver, Observer};
        #[cfg(feature = "hfi")]
        use crate::foc::phase::observer::{HFI_DEFAULT_AMPLITUDE_RATIO, HFI_DEFAULT_FREQ_HZ};

        if let Some(ref mp) = config.motor_params
            && mp.is_valid()
            && mp.flux_linkage_wb > 0.0
        {
            let l_avg = (mp.inductance_d_h + mp.inductance_q_h) / 2.0;
            // Detected params arm the full model: salient Lα/Lβ when the
            // motor measurably is (scalar L on an IPM gives load-dependent
            // angle error), and online λ tracking (λ drifts with saturation
            // and magnet temperature; the stored value is one bench point).
            let mut obs = BackEmfObserver::new(mp.resistance_ohm, l_avg, mp.flux_linkage_wb)
                .with_lambda_tracking(crate::foc::phase::observer::DEFAULT_LAMBDA_GAIN);
            let saliency = (mp.inductance_q_h - mp.inductance_d_h).abs();
            if saliency > 0.05 * l_avg {
                obs = obs.with_saliency(mp.inductance_d_h, mp.inductance_q_h);
            }
            self.set_observer(Observer::BackEmf(obs));
            // Carrier amplitude solved from the measured inductance for a
            // target ripple current, ceilinged by the legacy vbus ratio:
            // on a low-L outrunner (25 µH eskate motor) the raw ratio
            // would drive tens of amps of carrier ripple; on high-L motors
            // the solve exceeds the ceiling and the ratio default applies
            // unchanged. The ripple target scales with the motor's RATING
            // when detection stored one (see HFI_RIPPLE_RATING_FRACTION —
            // not the session current limit), absolute default otherwise.
            #[cfg(feature = "hfi")]
            {
                let i_target = match mp.rating_current_a() {
                    Some(rating) => crate::foc::clamp_f32(
                        crate::foc::phase::observer::HFI_RIPPLE_RATING_FRACTION * rating,
                        0.05,
                        crate::foc::phase::observer::HFI_CARRIER_RIPPLE_TARGET_A,
                    ),
                    None => crate::foc::phase::observer::HFI_CARRIER_RIPPLE_TARGET_A,
                };
                let omega_c = HFI_DEFAULT_FREQ_HZ * TAU;
                let amplitude = (i_target * omega_c * l_avg)
                    .min(vbus * HFI_DEFAULT_AMPLITUDE_RATIO)
                    .max(0.05);
                self.set_hfi_observer(
                    HfiObserver::new(HFI_DEFAULT_FREQ_HZ, amplitude).with_sincos(),
                );
            }
        }
    }
}
