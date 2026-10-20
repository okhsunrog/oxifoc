//! Source selection and the small getters and setters.

use super::{DEFAULT_OPENLOOP_TIME, HallHealth, OpenLoopOverride, PhaseFault, PhaseManager};
#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::phase::observer::Observer;
use crate::foc::phase::source::PhaseSource;
use crate::foc::phase::source::PhaseSourceError;
use crate::foc::phase::startup::SensorlessStartup;
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Set the timebase for tick conversions
    pub fn set_ticks_per_sec(&mut self, ticks_per_sec: u64) {
        self.ticks_per_sec = ticks_per_sec.max(1);
    }

    /// Get current phase source
    pub fn source(&self) -> PhaseSource {
        self.source
    }

    /// Set phase source
    ///
    /// Returns error if the requested source is not available.
    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError> {
        // Validate source availability
        if source.requires_hall() && !self.has_hall() {
            return Err(PhaseSourceError::HallNotAvailable);
        }
        if source.requires_encoder() && !self.has_encoder() {
            return Err(PhaseSourceError::EncoderNotAvailable);
        }
        if source.requires_observer() && !self.observer.is_configured() {
            return Err(PhaseSourceError::ObserverNotConfigured);
        }
        // HFI sources need the estimator that actually generates a carrier:
        // without one in the dedicated slot, injection() stays zero and
        // the source would silently never produce an estimate. With HFI
        // compiled out (`hfi` feature off) the variant stays in the wire enum
        // but is always rejected here.
        if source.requires_hfi() {
            #[cfg(feature = "hfi")]
            if self.hfi.is_none() {
                return Err(PhaseSourceError::HfiNotConfigured);
            }
            #[cfg(not(feature = "hfi"))]
            return Err(PhaseSourceError::HfiNotConfigured);
        }

        self.source = source;
        // Crossover memory belongs to the previous source's thresholds.
        self.crossover_latched = false;
        // The open-loop override likewise belongs to the source that armed
        // it (only Hall arms it). Without this, switching to a non-hall
        // source while the override is live strands it: the deactivation
        // paths are all `requires_hall()`-gated, so `angle_trustworthy()`
        // for Encoder/EncoderToObserver — which reads `!override.active` —
        // would stay false forever on a healthy encoder. If the new source
        // still can't produce an angle, `compute_phase_with_fallback`
        // re-arms the override next cycle.
        self.deactivate_open_loop_override();
        // A live cold-start sequence belongs to the source that began it.
        self.startup.deactivate();
        Ok(())
    }

    /// Enable the second-order commutation phase tracker for the pure-Observer source.
    ///
    /// `omega_n` = tracker natural frequency
    /// (el rad/s — wobble above it is rejected quadratically, hunting
    /// below it is followed/damped), `zeta` = damping ratio (1.0 =
    /// critical). `omega_n <= 0` disables (raw observer output).
    pub fn set_phase_tracker(&mut self, omega_n: f32, zeta: f32) {
        if omega_n.is_finite() && zeta.is_finite() && zeta >= 0.0 {
            let wn = omega_n.max(0.0);
            self.tracker.kp = wn * wn;
            self.tracker.kd = 2.0 * zeta * wn;
            self.tracker.active = false;
        }
    }

    /// Configure the back-EMF observer's physics acceleration prior
    /// (no-op without a configured observer) — see
    /// `BackEmfObserver::set_accel_prior`.
    pub fn set_observer_accel_prior(&mut self, floor_el: f32, per_amp_el: f32) {
        self.observer.set_accel_prior(floor_el, per_amp_el);
    }

    /// Override the back-EMF observer's PLL gains (no-op without a
    /// configured observer) — bench tuning of the slip-kick loop gain.
    pub fn set_observer_pll_gains(&mut self, kp: f32, ki: f32) {
        if let Observer::BackEmf(o) = &mut self.observer {
            o.set_pll_gains(kp, ki);
        }
    }

    /// Seed the back-EMF observer from an external (angle, velocity) —
    /// the same trusted-handoff semantics the deadshort catch uses
    /// internally. Sim/bench forensics: injecting a phase kick mid-drive
    /// (seed at `phase + Δ`) emulates one accumulated slip-kick.
    pub fn seed_observer(&mut self, angle: f32, velocity: f32) {
        self.observer.seed(angle, velocity);
    }

    pub fn set_observer(&mut self, observer: Observer) {
        self.observer = observer;
    }

    /// Get observer reference
    pub fn observer(&self) -> &Observer {
        &self.observer
    }

    /// Get mutable observer reference
    pub fn observer_mut(&mut self) -> &mut Observer {
        &mut self.observer
    }

    /// Set the HFI estimator (dedicated low-speed slot)
    #[cfg(feature = "hfi")]
    pub fn set_hfi_observer(&mut self, hfi: HfiObserver<S>) {
        self.hfi = Some(hfi);
    }

    /// Get HFI estimator reference
    #[cfg(feature = "hfi")]
    pub fn hfi_observer(&self) -> Option<&HfiObserver<S>> {
        self.hfi.as_ref()
    }

    /// Get mutable HFI estimator reference
    #[cfg(feature = "hfi")]
    pub fn hfi_observer_mut(&mut self) -> Option<&mut HfiObserver<S>> {
        self.hfi.as_mut()
    }

    /// Check if Hall sensor is available.
    ///
    /// Presence is structural ([`AngleSensor::is_present`]), not "has data":
    /// a healthy hall on a rotor that has not moved yet has no sample and no
    /// errors, and gating on data made `set_source(Hall…)` from stored config
    /// fail spuriously on cold start.
    pub fn has_hall(&self) -> bool {
        self.hall.is_present()
    }

    /// Check if encoder is available (structural presence, see [`Self::has_hall`])
    pub fn has_encoder(&self) -> bool {
        self.encoder.is_present()
    }

    /// Set manual angle (for Manual source)
    pub fn set_manual_angle(&mut self, angle: f32) {
        self.manual_angle = wrap_angle(angle);
    }

    /// Get manual angle
    pub fn manual_angle(&self) -> f32 {
        self.manual_angle
    }

    /// Set open-loop velocity (for OpenLoop source)
    pub fn set_open_loop_velocity(&mut self, velocity: f32) {
        self.open_loop_velocity = velocity;
    }

    /// Get open-loop velocity
    pub fn open_loop_velocity(&self) -> f32 {
        self.open_loop_velocity
    }

    /// Set open-loop angle directly
    pub fn set_open_loop_angle(&mut self, angle: f32) {
        self.open_loop_angle = wrap_angle(angle);
    }

    /// Get open-loop angle
    pub fn open_loop_angle(&self) -> f32 {
        self.open_loop_angle
    }

    /// Get mutable reference to Hall sensor
    pub fn hall_mut(&mut self) -> &mut H {
        &mut self.hall
    }

    /// Get reference to Hall sensor
    pub fn hall(&self) -> &H {
        &self.hall
    }

    /// Get mutable reference to encoder
    pub fn encoder_mut(&mut self) -> &mut E {
        &mut self.encoder
    }

    /// Get reference to encoder
    pub fn encoder(&self) -> &E {
        &self.encoder
    }

    // ========================================================================
    // Hall Health and Fault Tracking
    // ========================================================================

    /// Get Hall sensor health status
    pub fn hall_health(&self) -> HallHealth {
        self.hall_health
    }

    /// Get active faults
    pub fn faults(&self) -> &[PhaseFault] {
        &self.faults
    }

    /// Check if a specific fault is active
    pub fn has_fault(&self, fault: PhaseFault) -> bool {
        self.faults.contains(&fault)
    }

    /// Clear all faults
    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    /// Get open-loop override state
    pub fn open_loop_override(&self) -> &OpenLoopOverride {
        &self.open_loop_override
    }

    /// True while the cold-start sequencer owns commutation.
    pub fn is_starting(&self) -> bool {
        self.startup.is_active()
    }

    /// The cold-start sequencer (diagnostics).
    pub fn startup(&self) -> &SensorlessStartup {
        &self.startup
    }

    /// Check if open-loop override is active
    pub fn is_open_loop_override_active(&self) -> bool {
        self.open_loop_override.active
    }

    /// Activate open-loop override with default parameters
    ///
    /// This is called automatically when Hall fails and observer isn't ready.
    /// Can also be called manually for startup.
    pub fn activate_open_loop_override(&mut self, initial_angle: f32, velocity: f32) {
        self.open_loop_override.active = true;
        self.open_loop_override.angle = wrap_angle(initial_angle);
        self.open_loop_override.velocity = velocity;
        self.open_loop_override.timer = DEFAULT_OPENLOOP_TIME;
    }

    /// Deactivate open-loop override
    pub fn deactivate_open_loop_override(&mut self) {
        self.open_loop_override.active = false;
        self.open_loop_override.timer = 0.0;
    }
}
