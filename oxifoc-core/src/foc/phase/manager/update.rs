//! `PhaseProvider` implementation: sample sensors and publish one phase output.

#[cfg(feature = "hfi")]
use super::{CROSSOVER_HYSTERESIS, HFI_VOLTS_HYSTERESIS};
use super::{HallHealth, PhaseManager};
use crate::foc::fast_math::sqrtf;
use crate::foc::hall_sensor::HallFaultKind;
#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::phase::observer::ObserverInput;
use crate::foc::phase::provider::PhaseInput;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::phase::provider::PhaseProvider;
use crate::foc::phase::provider::StartupStatus;
use crate::foc::phase::source::PhaseSource;
use crate::foc::phase::startup::ConfirmResult;
use crate::foc::phase::startup::DeadshortResult;
use crate::foc::phase::startup::StartupPhase;
use crate::foc::sensors::AngleSample;
use crate::foc::sensors::AngleSensor;
use crate::foc::transforms::park;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;

/// Diagnostic-only reproduction switch, changed while Stopped by the bench.
/// Compiled out together with periodic observer logging in normal images.
#[cfg(feature = "observer-logging")]
#[unsafe(no_mangle)]
pub static OBSERVER_LOG_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(true);

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Whether HFI runs this cycle: carrier injection AND the demod update
    /// — always as a pair (a demod without carrier measures silence, a
    /// carrier without demod is pure loss).
    ///
    /// Above the crossover latch the fast source commutates and HFI is off
    /// entirely — keeping the carrier on at speed only costs losses and
    /// acoustic noise while the saliency response degrades anyway, and
    /// running the demod update without carrier is wasted ISR time (in
    /// non-Hfi sources it never runs at all for the same reason).
    ///
    /// PRE-HEAT: in the latched regime HFI resumes one hysteresis band
    /// EARLY — while the speed (or back-EMF proxy) is still a margin above
    /// the latch-release threshold. The demod filters need several carrier
    /// periods of real current to lock after a carrier-off gap
    /// ([`HfiObserver::restart_demod`]); the margin buys that time for any
    /// controller-bounded deceleration, so by the time the latch releases
    /// and blend weight starts flowing to HFI it is already locked. The
    /// margin deliberately does NOT try to cover mechanically-unbounded
    /// deceleration (a wheel jam crosses any finite band instantly) — that
    /// tail is covered by the trust gate instead: a cold demod reports
    /// zero confidence, `angle_trustworthy()` stays false and the driver
    /// keeps iq at zero for the few ms the lock takes.
    #[cfg(feature = "hfi")]
    fn hfi_active(&self) -> bool {
        match self.source {
            PhaseSource::Hfi => true,
            PhaseSource::HfiToObserver { min_vel, .. } => {
                !self.crossover_latched
                    || self.output.velocity.abs() < min_vel * (1.0 + CROSSOVER_HYSTERESIS)
            }
            PhaseSource::HfiToObserverVolts { toggle_v, .. } => {
                !self.crossover_latched || self.bemf_proxy_v < toggle_v + HFI_VOLTS_HYSTERESIS
            }
            PhaseSource::HfiToHall { switch_vel } | PhaseSource::HfiToEncoder { switch_vel } => {
                !self.crossover_latched
                    || self.output.velocity.abs() < switch_vel * (1.0 + CROSSOVER_HYSTERESIS)
            }
            _ => false,
        }
    }
}

// ============================================================================
// PhaseProvider implementation
// ============================================================================

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseProvider for PhaseManager<H, E, S> {
    fn motion_reference_epoch(&self) -> super::super::MotionReferenceEpoch {
        self.motion_reference_epoch
    }

    fn rotor_velocity(&self) -> Option<super::RotorVelocityEstimate> {
        if self.startup.is_active() || self.open_loop_override.active {
            None
        } else {
            self.rotor_velocity
        }
    }

    fn requires_hall_reference(&self) -> bool {
        self.source.requires_hall()
    }

    fn apply_hall_calibration(
        &mut self,
        prepared: &crate::foc::hall_sensor::PreparedHallCalibration,
    ) -> bool {
        // Only replace a cached frame to which Hall actually contributed.
        // Pure observer/manual/HFI outputs and active fallbacks stay intact.
        let observer_weight = match self.source {
            PhaseSource::Hall => Some(0.0),
            PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            } => {
                let weight = if self.observer.is_ready() {
                    super::select::compute_blend(
                        self.hall_control_motion
                            .velocity()
                            .abs()
                            .max(self.observer.velocity().unwrap_or(0.0).abs()),
                        blend_low,
                        blend_high,
                    )
                } else {
                    0.0
                };
                (weight < 1.0).then_some(weight)
            }
            PhaseSource::HfiToHall { .. } if self.crossover_latched => Some(0.0),
            _ => None,
        };
        let Some(update) = self.hall.apply_prepared_hall_calibration(prepared) else {
            return false;
        };
        self.motion_reference_epoch.advance();
        self.rotor_velocity = None;
        self.hall_control_motion.reset();
        self.hall_motion_witness.reset();
        if let Some(sample) = update.sample {
            if self.hall_health == HallHealth::Ok
                && !self.open_loop_override.active
                && let Some(weight) = observer_weight
            {
                // Preserve the applied hybrid weight, and the independent
                // observer/HFI state. Normal updates own crossover/reseeding.
                self.output = match (
                    weight > 0.0,
                    self.observer.phase(),
                    self.observer.velocity(),
                ) {
                    (true, Some(angle), Some(velocity)) => PhaseOutput {
                        angle: super::select::blend_angles(sample.angle, angle, weight),
                        velocity: velocity * weight,
                    },
                    _ => PhaseOutput {
                        angle: sample.angle,
                        velocity: 0.0,
                    },
                };
            }
        } else if self.source.requires_hall() {
            self.update_hall_health(false, false);
        }
        true
    }

    #[cfg(feature = "storage")]
    fn apply_motor_model(&mut self, model: &mut super::PreparedPhaseModel) -> Option<PhaseSource> {
        // ResetAll owns both the default model and default Hall frame. A
        // structurally absent sensor needs no reset; a present unsupported
        // sensor must reject before any estimator/model state is changed.
        if let Some(reference) = model.hall_reset()
            && self.has_hall()
            && !PhaseProvider::apply_hall_calibration(self, reference)
        {
            return None;
        }
        Some(self.apply_prepared_model(model))
    }

    fn get(&self) -> PhaseOutput {
        self.output
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn update(&mut self, input: &PhaseInput, now_ticks: u64) {
        let previous_hall_health = self.hall_health;
        let previous_crossover = self.crossover_latched;
        let observer_was_ready = matches!(
            self.source,
            PhaseSource::HallToObserver { .. } | PhaseSource::EncoderToObserver { .. }
        ) && self.observer.is_ready();
        // Sample hardware sensors — only for sources that consume them. The
        // stateful path matters for hall: it carries the rate limiter that
        // smooths sector-edge discontinuities. A stale hall (edges stopped
        // while spinning) is treated as having no sample at all, so every
        // consumer below falls back uniformly. On a sensorless (Observer)
        // source both samples are dead weight at 20 kHz — the interpolation
        // math ran every cycle for data nothing read (2026-07-06 ISR
        // profiling); everything downstream is already `requires_*`-gated
        // and treats `None` as "no sensor".
        let (hall_stale, hall_sample) = if self.source.requires_hall() {
            let observer = self
                .observer
                .is_ready()
                .then(|| {
                    self.observer
                        .phase()
                        .zip(self.observer.velocity())
                        .map(|(angle, velocity)| PhaseOutput { angle, velocity })
                })
                .flatten();
            let stale = if let Some(observer) = observer {
                match self.hall.hall_edge_observation() {
                    Some(edge) => self
                        .hall_motion_witness
                        .observe(edge, Some(observer), now_ticks),
                    None => {
                        self.hall_motion_witness.reset();
                        // Preserve the health hook for custom sensor adapters.
                        self.hall.is_stale(now_ticks, observer.velocity)
                    }
                }
            } else {
                self.hall_motion_witness.reset();
                false
            };
            // A rejected observation must not advance the stateful angle
            // limiter: recovery starts from the last accepted Hall frame.
            (
                stale,
                if stale {
                    None
                } else {
                    self.hall.sample_for_control(now_ticks)
                },
            )
        } else {
            self.hall_motion_witness.reset();
            (false, None)
        };
        let encoder_sample = if self.source.requires_encoder() {
            self.encoder.sample_mut(now_ticks)
        } else {
            None
        };
        // Sensor samples describe the rotor at `now_ticks` — this cycle's
        // current sample — but the output commutates the NEXT cycle (see
        // the PhaseProvider timing contract): extrapolate one period.
        // Without it hall/encoder commutation ran one period late.
        let predict = |s: AngleSample| AngleSample {
            angle: wrap_angle(s.angle + s.omega * input.dt),
            ..s
        };
        // Differentiate the stateful, rate-limited Hall angle before the
        // one-period prediction. Use actual elapsed Hall-domain ticks for
        // that measurement; input.dt remains the prediction horizon (also
        // when Stopped calls us at dt * 4).
        let mut hall_velocity = None;
        let hall_sample = if let Some(sample) = hall_sample {
            if self.hall_reference_epoch != Some(sample.reference_epoch) {
                self.hall_reference_epoch = Some(sample.reference_epoch);
                self.motion_reference_epoch.advance();
            }
            let native_velocity = super::RotorVelocityEstimate::new(sample.sample.omega);
            self.hall_control_motion
                .observe(sample, now_ticks, self.output.velocity)
                .map(|sample| {
                    // Capture only an accepted coherent observation; a missing
                    // or nonfinite angle must not leave a usable speed behind.
                    hall_velocity = native_velocity;
                    PhaseOutput {
                        angle: wrap_angle(sample.angle + sample.velocity * input.dt),
                        ..sample
                    }
                })
        } else {
            self.hall_control_motion.unavailable();
            None
        };
        let encoder_sample = encoder_sample.map(predict);

        // Hall health is only meaningful for sources that consume hall data:
        // an idle hall during Manual-angle calibration or pure-observer
        // operation is not a failure.
        if self.source.requires_hall() {
            self.update_hall_health(hall_sample.is_some(), hall_stale);
        }

        // Back-EMF proxy for the voltage-based crossover: park the drive
        // voltage and current onto the current output frame and remove the
        // resistive drop. Only meaningful (and only computed) when the
        // active source uses it.
        if matches!(self.source, PhaseSource::HfiToObserverVolts { .. }) {
            use crate::foc::trig::FastSinCos;
            let r = self.observer.resistance().unwrap_or(0.0);
            let (sin_t, cos_t) = FastSinCos::sin_cos(self.output.angle);
            let (_, vq) = park(input.v_alpha, input.v_beta, sin_t, cos_t);
            let (_, iq) = park(input.i_alpha, input.i_beta, sin_t, cos_t);
            let bemf = vq - r * iq;
            self.bemf_proxy_v = if bemf < 0.0 { -bemf } else { bemf };
        }

        // Update the estimators. The back-EMF observer always runs — it is
        // the fallback/crossover target for every source and has its own
        // signal (back-EMF) regardless of mode. HFI runs only while its
        // carrier is injected (see hfi_active()): without carrier the demod
        // measures silence — pure wasted ISR time, ~10% of the cycle budget
        // in the default hall ride configuration. A rising edge restarts
        // the demod filters so the stale pre-pause state can't masquerade
        // as confidence.
        let obs_input = ObserverInput {
            v_alpha: input.v_alpha,
            v_beta: input.v_beta,
            i_alpha: input.i_alpha,
            i_beta: input.i_beta,
            dt: input.dt,
        };
        let prof_t0 = crate::isr_prof::now();
        self.observer.update(&obs_input);
        crate::isr_prof::add(&crate::isr_prof::EST_OBS, prof_t0, crate::isr_prof::now());

        // Optional ~2.4 Hz observer text diagnostic. Logging from the ADC
        // handler can exceed its deadline even at this low average rate;
        // normal images use telemetry and compile this entire block out.
        #[cfg(feature = "observer-logging")]
        {
            // Plain field, not an atomic: this runs only in the ISR, and a
            // static AtomicU32 here was a per-cycle LDREX/STREX pair (PC
            // sampling, tier-3 shave).
            self.obs_trace_ticks = self.obs_trace_ticks.wrapping_add(1);
            if self.obs_trace_ticks.is_multiple_of(8192)
                && OBSERVER_LOG_ENABLED.load(core::sync::atomic::Ordering::Relaxed)
                && let Some(vel) = self.observer.velocity()
                && vel.abs() > 20.0
            {
                let (bemf_q, travel) = self.observer.validity().unwrap_or((0.0, 0.0));
                info!(
                    "obs: vel={} conf={} e_q={} travel={} lambda={}",
                    vel,
                    self.observer.confidence(),
                    bemf_q,
                    travel,
                    self.observer.lambda().unwrap_or(0.0)
                );
            }
        }
        #[cfg(feature = "hfi")]
        {
            let hfi_active = self.hfi_active();
            if hfi_active && let Some(hfi) = &mut self.hfi {
                if !self.hfi_was_active {
                    hfi.restart_demod();
                }
                hfi.update(&obs_input);
                self.hfi_velocity_observed = true;
            }
            self.hfi_was_active = hfi_active;
        }

        // Advance open-loop angle if in OpenLoop mode
        if matches!(self.source, PhaseSource::OpenLoop) {
            self.open_loop_angle += self.open_loop_velocity * input.dt;
            self.open_loop_angle = wrap_angle(self.open_loop_angle);
        }

        // Advance the sensorless cold-start sequencer (if running): it drives
        // the open-loop angle the Observer arm commutates from, and hands off
        // — deactivates — the moment the observer has actually converged at
        // handoff speed (it keeps running on commanded-v + measured-i the
        // whole time, so by handoff its angle is the true rotor angle).
        let prof_t0 = crate::isr_prof::now();
        // Startup-log rate limit (see startup.rs LOG_TOKENS_PER_WINDOW):
        // tick the window every cycle; when frames were dropped, one
        // summary frame per window keeps the churn visible.
        let log_suppressed = self.startup.log_tick(input.dt);
        if log_suppressed > 0 {
            warn!(
                "startup: {} log frames suppressed (restart churn)",
                log_suppressed
            );
        }
        if self.startup.is_active() {
            let phase_before = self.startup.phase();
            if phase_before == StartupPhase::Deadshort {
                // Flying-restart probe: the driver holds the bridge at zero
                // voltage; feed the back-EMF-driven current to the probe. A
                // spinning rotor → seed the observer and go straight to closed
                // loop; standstill → the probe falls through to the ramp.
                let r = self.observer.resistance().unwrap_or(0.0);
                let l = self.observer.inductance().unwrap_or(0.0);
                let lambda = self.observer.lambda().unwrap_or(0.0);
                if let DeadshortResult::Caught { angle, velocity } =
                    self.startup
                        .feed_deadshort(input.i_alpha, input.i_beta, input.dt, r, l, lambda)
                {
                    if self.startup.log_allow() {
                        info!(
                            "startup: deadshort caught spinning rotor (angle={} vel={}), seeding observer",
                            angle, velocity
                        );
                    }
                    // The probe measured the rotor at this sample; the
                    // observer's phase lives one period ahead.
                    self.observer
                        .seed(wrap_angle(angle + velocity * input.dt), velocity);
                } else if self.startup.phase() == StartupPhase::Ramp && self.startup.log_allow() {
                    info!("startup: deadshort saw standstill, ramp cold start");
                }
            } else if phase_before == StartupPhase::Confirm {
                // Handoff-confirm probe: bridge shorted, the measured
                // back-EMF must corroborate the observer before closed loop
                // engages (a phantom-locked observer passes every internal
                // gate — see BackEmfObserver::is_ready).
                let r = self.observer.resistance().unwrap_or(0.0);
                let l = self.observer.inductance().unwrap_or(0.0);
                let lambda = self.observer.lambda().unwrap_or(0.0);
                // The probe measures the rotor at THIS sample; the
                // observer's phase predicts the next one — compare on the
                // probe's time base.
                let obs_vel = self.observer.velocity().unwrap_or(0.0);
                let claim = PhaseOutput {
                    angle: wrap_angle(self.observer.phase().unwrap_or(0.0) - obs_vel * input.dt),
                    velocity: obs_vel,
                };
                match self.startup.feed_confirm(
                    input.i_alpha,
                    input.i_beta,
                    input.dt,
                    r,
                    l,
                    lambda,
                    claim,
                ) {
                    ConfirmResult::Confirmed { velocity } => {
                        if self.startup.log_allow() {
                            info!(
                                "startup: handoff confirmed by probe (probe_vel={} observer_vel={})",
                                velocity, obs_vel
                            );
                        }
                    }
                    ConfirmResult::Unconfirmed { velocity } => {
                        if self.startup.log_allow() {
                            info!(
                                "startup: handoff unconfirmed (probe_vel={} observer_vel={}), \
                                 holding for retry",
                                velocity, obs_vel
                            );
                        }
                    }
                    ConfirmResult::SeedAndHandoff { angle, velocity } => {
                        // The probe measured a real spinning rotor on
                        // CONFIRM_SEED_PROBES consecutive tries while the
                        // observer's claim kept diverging (hold-ratchet):
                        // trust the measurement, reseed the observer from
                        // it — same as the deadshort catch.
                        if self.startup.log_allow() {
                            warn!(
                                "startup: observer diverged from probed rotor \
                                 (probe_vel={} observer_vel={}), seeding observer from probe",
                                velocity, obs_vel
                            );
                        }
                        self.observer
                            .seed(wrap_angle(angle + velocity * input.dt), velocity);
                    }
                    ConfirmResult::Probing => {}
                }
            } else {
                let i_mag = sqrtf(input.i_alpha * input.i_alpha + input.i_beta * input.i_beta);
                let out = self.startup.tick(
                    input.dt,
                    i_mag,
                    self.observer.is_ready(),
                    self.observer.velocity().unwrap_or(0.0),
                );
                // Transitions are one-shot per start — but starts themselves
                // repeat several times a second during restart churn, hence
                // the token bucket on every frame here.
                let phase_now = self.startup.phase();
                if phase_now != phase_before && self.startup.log_allow() {
                    info!(
                        "startup: {} -> {} (vel={} |i|={})",
                        phase_before.name(),
                        phase_now.name(),
                        out.velocity,
                        i_mag
                    );
                }
                // Hold give-up recycle: the observer failed to confirm for
                // the whole hold window — presume a phantom lock and
                // restart it from scratch while the deadshort→ramp start
                // re-acquires the rotor.
                if self.startup.take_recycled() {
                    if self.startup.log_allow() {
                        warn!(
                            "startup: hold gave up (no confirmed handoff), observer reset + recycle"
                        );
                    }
                    self.observer.reset();
                }
                if out.handoff {
                    if self.startup.log_allow() {
                        info!(
                            "startup: handoff gates passed (openloop_vel={} observer_vel={}), \
                             running confirm probe",
                            out.velocity,
                            self.observer.velocity().unwrap_or(0.0)
                        );
                    }
                } else if phase_now == StartupPhase::Hold {
                    // Waiting on the observer: ~2 Hz convergence trace so a
                    // hold that never hands off (observer incoherent, see
                    // HANDOFF_COHERENCE_FRAC) is diagnosable from the log.
                    self.hold_trace_ticks = self.hold_trace_ticks.wrapping_add(1);
                    if self.hold_trace_ticks.is_multiple_of(8192) && self.startup.log_allow() {
                        info!(
                            "startup: holding (openloop_vel={} observer_vel={} ready={} conf={})",
                            out.velocity,
                            self.observer.velocity().unwrap_or(0.0),
                            self.observer.is_ready(),
                            self.observer.confidence()
                        );
                    }
                }
            }
        }

        let prof_t1 = crate::isr_prof::now();
        crate::isr_prof::add(&crate::isr_prof::EST_STARTUP, prof_t0, prof_t1);

        // Update open-loop override state (for Hall failure recovery)
        self.update_open_loop_override(input.dt);

        // Compute output based on source (with potential fallback)
        self.output = self.compute_phase_with_fallback(
            hall_sample,
            hall_velocity,
            encoder_sample,
            input.dt,
            observer_was_ready,
        );
        let hall_reference_active = matches!(
            self.source,
            PhaseSource::Hall | PhaseSource::HallToObserver { .. }
        ) || (matches!(self.source, PhaseSource::HfiToHall { .. })
            && self.crossover_latched);
        if (hall_reference_active
            && (self.hall_health == HallHealth::Ok) != (previous_hall_health == HallHealth::Ok))
            || self.crossover_latched != previous_crossover
        {
            self.motion_reference_epoch.advance();
        }
        crate::isr_prof::add(&crate::isr_prof::EST_OUT, prof_t1, crate::isr_prof::now());
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        self.set_source(source).is_ok()
    }

    fn begin_cold_start(&mut self, dir: f32) {
        // Only a pure back-EMF source needs the open-loop bootstrap: a sensored
        // source (Hall/Encoder, or a *ToObserver blend) already has an angle at
        // standstill. No-op once the observer is tracking. Starts from the last
        // output angle so the field doesn't jump on engage.
        if !matches!(self.source, PhaseSource::Observer) {
            return;
        }
        if self.observer.is_ready() {
            if self.startup.log_allow() {
                info!(
                    "startup: cold start skipped, observer already tracking (vel={})",
                    self.observer.velocity().unwrap_or(0.0)
                );
            }
            return;
        }
        if self.startup.log_allow() {
            info!(
                "startup: cold start engaged (angle0={} dir={})",
                self.output.angle, dir
            );
        }
        self.startup.begin_cold_start(self.output.angle, dir);
    }

    fn wants_short(&self) -> bool {
        self.startup_status().wants_short()
    }

    fn startup_current_scale(&self) -> f32 {
        self.startup_status().current_scale()
    }

    fn is_starting(&self) -> bool {
        self.startup_status().is_active()
    }

    fn startup_status(&self) -> StartupStatus {
        if self.startup.wants_short() {
            StartupStatus::ShortProbe
        } else if self.startup.is_active() {
            StartupStatus::Driving {
                current_scale: self.startup.current_scale(),
            }
        } else {
            StartupStatus::Inactive
        }
    }

    /// Trustworthy down to standstill when a hardware sensor backs the active
    /// source (Hall/Encoder track to a stop), or when HFI is locked (valid at
    /// zero speed by design). A pure back-EMF observer is only trusted while
    /// `is_ready()` — it drops below its speed floor, so the failsafe brake
    /// coasts the last bit instead of commutating blind.
    ///
    /// While the open-loop recovery override carries commutation (sensor
    /// dead AND observer not ready), the angle is FABRICATED — a rotating
    /// frame with no relation to the rotor. Reporting it trustworthy put
    /// the full commanded iq into that frame (random-direction torque
    /// jolts under a rider at low speed) and let the failsafe commutate
    /// the brake blind. Untrusted, the existing iq gate coasts instead;
    /// recovery comes from physical motion (kick-push → back-EMF →
    /// observer locks → override deactivates → trust returns). The
    /// deliberate cost: no self-start from standstill with a dead hall
    /// until the sensorless promotion lands (fault-overhaul.md phase 6,
    /// variant A decision 2026-06-13).
    fn angle_trustworthy(&self) -> bool {
        match self.source {
            PhaseSource::Hall
            | PhaseSource::Encoder
            | PhaseSource::HallToObserver { .. }
            | PhaseSource::EncoderToObserver { .. } => !self.open_loop_override.active,
            // Calibration/detection sources: the commanded angle IS the
            // reference frame, trusted by construction.
            PhaseSource::Manual | PhaseSource::OpenLoop => true,
            // Pure sensorless: trusted once the observer locks, OR while the
            // cold-start sequencer drives — there the open-loop angle IS the
            // intended I/f reference and torque MUST flow into it to spin the
            // rotor up (unlike the hall-dropout recovery override above, which
            // is a fabricated frame under a moving rider and stays untrusted).
            // Only `begin_cold_start` (a deliberate sensorless start) arms it.
            PhaseSource::Observer => self.observer.is_ready() || self.startup.is_active(),
            #[cfg(feature = "hfi")]
            PhaseSource::Hfi => self.hfi.as_ref().is_some_and(HfiObserver::is_ready),
            #[cfg(feature = "hfi")]
            PhaseSource::HfiToObserver { min_confidence, .. }
            | PhaseSource::HfiToObserverVolts { min_confidence, .. } => {
                // The selector uses HFI until the crossover is COMPLETE,
                // including the blend band. A ready background observer
                // cannot certify the HFI contribution to that angle.
                if self.crossover_latched {
                    self.observer.is_ready() && self.observer.confidence() >= min_confidence
                } else {
                    self.hfi.as_ref().is_some_and(HfiObserver::is_ready)
                }
            }
            // Before the latch, these modes actually commute from HFI, so a
            // present Hall/encoder does not make a cold demodulation estimate
            // safe. After the latch the hardware sensor carries the angle.
            #[cfg(feature = "hfi")]
            PhaseSource::HfiToHall { .. } | PhaseSource::HfiToEncoder { .. } => {
                !self.open_loop_override.active
                    && (self.crossover_latched
                        || self.hfi.as_ref().is_some_and(HfiObserver::is_ready))
            }
            // HFI compiled out: these sources are unreachable (set_source
            // rejects them), but the match must stay exhaustive.
            #[cfg(not(feature = "hfi"))]
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => self.observer.is_ready(),
        }
    }

    #[cfg(feature = "hfi")]
    fn injection(&self) -> (f32, f32) {
        if self.hfi_active()
            && let Some(hfi) = &self.hfi
        {
            hfi.get_injection()
        } else {
            (0.0, 0.0)
        }
    }

    #[cfg(not(feature = "hfi"))]
    fn injection(&self) -> (f32, f32) {
        (0.0, 0.0)
    }

    fn note_torque_current(&mut self, iq_abs: f32, dt: f32) {
        self.observer.note_torque_current(iq_abs, dt);
    }

    fn debug_observer(&self) -> Option<(f32, f32, f32, f32)> {
        match (
            self.observer.phase(),
            self.observer.phase_raw(),
            self.observer.velocity(),
            self.observer.readiness_phase_err(),
        ) {
            (Some(pll), Some(raw), Some(vel), Some(err)) => Some((pll, raw, vel, err)),
            _ => None,
        }
    }

    /// Most specific active hall degradation: the sensor's wire verdicts
    /// (named dead wires / error rate) win over the coarse health kinds —
    /// "wire H2 dead" tells the bench which pin to buzz out, "invalid
    /// state" only says something is wrong. Only sources that consume hall
    /// data report; an idle hall during Manual calibration or pure
    /// sensorless is not a failure.
    fn hall_fault(&self) -> Option<HallFaultKind> {
        if !self.source.requires_hall() {
            return None;
        }
        if let Some(kind) = self.hall.fault_kind() {
            return Some(kind);
        }
        match self.hall_health {
            HallHealth::Stale => Some(HallFaultKind::StaleAtSpeed),
            HallHealth::Invalid => Some(HallFaultKind::InvalidState),
            HallHealth::Ok | HallHealth::NotPresent => None,
        }
    }
}
