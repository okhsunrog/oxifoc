//! Per-cycle source selection, blending, and HFI handoff.

use super::{CROSSOVER_HYSTERESIS, HFI_VOLTS_HYSTERESIS, PhaseFault, PhaseManager};
use crate::foc::angle_difference;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::phase::source::PhaseSource;
use crate::foc::sensors::AngleSample;
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;
use core::f32::consts::TAU;

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Current HFI estimate as a phase output (None if no HFI configured).
    /// Always `None` when HFI is compiled out — the Hfi* `PhaseSource` arms
    /// that call this are then unreachable (rejected by `set_source`).
    #[cfg(feature = "hfi")]
    fn hfi_output(&self) -> Option<PhaseOutput> {
        self.hfi.as_ref().map(|h| PhaseOutput {
            angle: h.phase(),
            velocity: h.velocity(),
        })
    }

    #[cfg(not(feature = "hfi"))]
    fn hfi_output(&self) -> Option<PhaseOutput> {
        None
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub(super) fn compute_phase_with_fallback(
        &mut self,
        hall_sample: Option<AngleSample>,
        encoder_sample: Option<AngleSample>,
        dt: f32,
    ) -> PhaseOutput {
        // A live hall sample retakes commutation from the recovery
        // override. Without this, one glitch-triggered override outlived
        // the glitch: nothing deactivated it when the SENSOR (rather than
        // the observer) came back, and `angle_trustworthy()` stayed false
        // forever on a healthy hall.
        if hall_sample.is_some() && self.open_loop_override.active && self.source.requires_hall() {
            self.deactivate_open_loop_override();
        }

        match self.source {
            PhaseSource::Hall => {
                // VESC-style: try Hall first, fall back to observer if Hall failed
                if let Some(sample) = hall_sample {
                    PhaseOutput {
                        angle: sample.angle,
                        velocity: sample.omega,
                    }
                } else {
                    // Hall failed - try observer fallback
                    self.try_observer_fallback().unwrap_or(self.output)
                }
            }

            PhaseSource::Encoder => sample_to_output(encoder_sample, &self.output),

            PhaseSource::Observer => {
                // Pure sensorless: commutate from a converged observer; else
                // run the cold-start sequencer open-loop (ramp→hold→confirm)
                // if it is active; else hold the last output (and raise the
                // fault). The sequencer is advanced in `update`, which hands
                // off (deactivates) once a probe confirms the observer.
                if self.observer.is_ready() {
                    self.clear_fault(PhaseFault::ObserverNotReady);
                    match (self.observer.phase(), self.observer.velocity()) {
                        (Some(angle), Some(vel)) => {
                            let raw = PhaseOutput {
                                angle,
                                velocity: vel,
                            };
                            // Filter only POST-handoff: while the startup
                            // sequencer is still active a ready observer
                            // takes commutation raw (pre-existing behavior
                            // the hold/confirm dynamics are tuned around);
                            // the filter seeds continuity at the handoff.
                            if self.tracker.kp > 0.0 && !self.startup.is_active() {
                                self.tracker_output(raw, dt)
                            } else {
                                self.tracker.active = false;
                                raw
                            }
                        }
                        _ => self.output,
                    }
                } else if self.startup.is_active() {
                    // The sequencer owns commutation: the tracker re-seeds
                    // continuity at the next handoff.
                    self.tracker.active = false;
                    PhaseOutput {
                        angle: self.startup.angle(),
                        velocity: self.startup.velocity(),
                    }
                } else {
                    self.set_fault(PhaseFault::ObserverNotReady);
                    self.tracker.active = false;
                    self.output
                }
            }

            PhaseSource::Hfi => {
                // HFI estimate straight from the dedicated slot. No readiness
                // gate: HFI is valid from standstill by design. The carrier
                // reaches the motor via PhaseProvider::injection() →
                // FocController::step_with_injection in FocDriver.
                self.hfi_output().unwrap_or(self.output)
            }

            PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            } => {
                // VESC-style full Hall mode:
                // 1. Hall failed (invalid state / stale at speed) → observer
                //    if ready, else the open-loop recovery override.
                // 2. Hall healthy → blend Hall→observer by velocity.
                if hall_sample.is_none() {
                    return self.try_observer_fallback().unwrap_or(self.output);
                }
                let sensor = sample_to_output(hall_sample, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
            }

            PhaseSource::EncoderToObserver {
                blend_low,
                blend_high,
            } => {
                let sensor = sample_to_output(encoder_sample, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
            }

            // Both estimator slots run concurrently; the output blends from
            // HFI to the observer across the velocity band
            // [min_vel·(1−CROSSOVER_HYSTERESIS), min_vel]. A sharp switch is
            // not enough here: the HFI demod/PLL lag grows with speed, so at
            // the crossover the two estimates legitimately disagree by tenths
            // of a radian — blending absorbs that, a latch jump would not.
            // `crossover_latched` marks the fully-blended regime: only there
            // is the carrier injection switched off, and re-entering the band
            // from above reseeds the HFI estimator from the last output
            // (its own estimate drifted while the carrier was off).
            PhaseSource::HfiToObserver {
                min_vel,
                min_confidence,
            } => {
                let mut hfi_out = self.hfi_output().unwrap_or(self.output);
                let obs_ready =
                    self.observer.is_ready() && self.observer.confidence() >= min_confidence;
                if !obs_ready {
                    // Observer can't be trusted at any blend weight yet.
                    if self.crossover_latched {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                        hfi_out = self.output;
                    }
                    return hfi_out;
                }

                // Speed reference: the faster of the two estimates, for the
                // same reason as the hall blend — near the handoff the
                // low-speed source's velocity may already be degrading.
                let obs_vel = self.observer.velocity().unwrap_or(0.0).abs();
                let speed = hfi_out.velocity.abs().max(obs_vel);
                let blend_low = min_vel * (1.0 - CROSSOVER_HYSTERESIS);
                let blend = compute_blend(speed, blend_low, min_vel);

                let was_latched = self.crossover_latched;
                self.crossover_latched = blend >= 1.0;
                if was_latched && !self.crossover_latched {
                    self.seed_hfi_from_output();
                    hfi_out = self.output;
                }

                match (self.observer.phase(), self.observer.velocity()) {
                    (Some(obs_angle), Some(obs_velocity)) => PhaseOutput {
                        angle: blend_angles(hfi_out.angle, obs_angle, blend),
                        velocity: hfi_out.velocity * (1.0 - blend) + obs_velocity * blend,
                    },
                    _ => hfi_out,
                }
            }

            // Voltage-criterion variant of HfiToObserver (MESC): the blend
            // weight comes from the back-EMF share of the drive voltage
            // instead of a velocity threshold. Same latch/reseed mechanics.
            PhaseSource::HfiToObserverVolts {
                toggle_v,
                min_confidence,
            } => {
                let mut hfi_out = self.hfi_output().unwrap_or(self.output);
                let obs_ready =
                    self.observer.is_ready() && self.observer.confidence() >= min_confidence;
                if !obs_ready {
                    if self.crossover_latched {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                        hfi_out = self.output;
                    }
                    return hfi_out;
                }

                let blend_low = (toggle_v - HFI_VOLTS_HYSTERESIS).max(0.0);
                let blend = compute_blend(self.bemf_proxy_v, blend_low, toggle_v);

                let was_latched = self.crossover_latched;
                self.crossover_latched = blend >= 1.0;
                if was_latched && !self.crossover_latched {
                    self.seed_hfi_from_output();
                    hfi_out = self.output;
                }

                match (self.observer.phase(), self.observer.velocity()) {
                    (Some(obs_angle), Some(obs_velocity)) => PhaseOutput {
                        angle: blend_angles(hfi_out.angle, obs_angle, blend),
                        velocity: hfi_out.velocity * (1.0 - blend) + obs_velocity * blend,
                    },
                    _ => hfi_out,
                }
            }

            PhaseSource::HfiToHall { switch_vel } => {
                let hall = sample_to_output(hall_sample, &self.output);
                if self.crossover_latched {
                    // Drop back to HFI only below the hysteresis band or on
                    // hall failure.
                    if hall_sample.is_none()
                        || hall.velocity.abs() < switch_vel * (1.0 - CROSSOVER_HYSTERESIS)
                    {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                    }
                } else if hall_sample.is_some() && hall.velocity.abs() >= switch_vel {
                    self.crossover_latched = true;
                }

                if self.crossover_latched {
                    hall
                } else {
                    self.hfi_output().unwrap_or(self.output)
                }
            }

            PhaseSource::HfiToEncoder { switch_vel } => {
                let enc = sample_to_output(encoder_sample, &self.output);
                if self.crossover_latched {
                    if encoder_sample.is_none()
                        || enc.velocity.abs() < switch_vel * (1.0 - CROSSOVER_HYSTERESIS)
                    {
                        self.crossover_latched = false;
                        self.seed_hfi_from_output();
                    }
                } else if encoder_sample.is_some() && enc.velocity.abs() >= switch_vel {
                    self.crossover_latched = true;
                }

                if self.crossover_latched {
                    enc
                } else {
                    self.hfi_output().unwrap_or(self.output)
                }
            }

            PhaseSource::Manual => PhaseOutput {
                angle: self.manual_angle,
                velocity: 0.0,
            },

            PhaseSource::OpenLoop => PhaseOutput {
                angle: self.open_loop_angle,
                velocity: self.open_loop_velocity,
            },
        }
    }

    /// Reseed the HFI estimator from the last managed output (downward
    /// crossover handoff). The angle comes from a source that was trusted
    /// for commutation, so this also resolves the HFI π ambiguity. No-op when
    /// HFI is compiled out (the callers — Hfi* arms — are then unreachable).
    #[cfg(feature = "hfi")]
    fn seed_hfi_from_output(&mut self) {
        let out = self.output;
        if let Some(hfi) = &mut self.hfi {
            hfi.set_phase(out.angle);
            hfi.set_velocity(out.velocity);
        }
    }

    #[cfg(not(feature = "hfi"))]
    fn seed_hfi_from_output(&mut self) {}

    /// Blend sensor output with observer based on velocity
    fn blend_with_observer(
        &mut self,
        sensor: PhaseOutput,
        blend_low: f32,
        blend_high: f32,
    ) -> PhaseOutput {
        // An unconverged observer must not pull the output anywhere — the
        // sensor stays authoritative until the observer is actually locked.
        if !self.observer.is_ready() {
            return sensor;
        }

        // Speed reference for regime selection: the faster of the two
        // estimates. Using the sensor velocity alone is perverse when the
        // sensor dies at speed — its decaying estimate drags the blend back
        // toward trusting the dying sensor MORE, while the (ready) observer
        // still reports the true speed.
        let obs_speed = self.observer.velocity().unwrap_or(0.0).abs();
        let blend = compute_blend(sensor.velocity.abs().max(obs_speed), blend_low, blend_high);

        // π-flip guard: back-EMF observers carry a half-turn ambiguity from
        // standstill. While the sensor still has any weight it is the truth
        // reference — a >90° disagreement means the observer locked onto the
        // inverted flux vector, and blending toward it collapses torque
        // mid-crossover. Reseed instead of blending.
        if blend < 1.0
            && let Some(obs_angle) = self.observer.phase()
            && angle_difference(obs_angle, sensor.angle).abs() > core::f32::consts::FRAC_PI_2
        {
            self.observer.seed(sensor.angle, sensor.velocity);
        }

        if let (Some(obs_angle), Some(obs_vel)) = (self.observer.phase(), self.observer.velocity())
        {
            PhaseOutput {
                angle: blend_angles(sensor.angle, obs_angle, blend),
                velocity: sensor.velocity * (1.0 - blend) + obs_vel * blend,
            }
        } else {
            sensor
        }
    }
}

fn sample_to_output(sample: Option<AngleSample>, fallback: &PhaseOutput) -> PhaseOutput {
    match sample {
        Some(s) => PhaseOutput {
            angle: s.angle,
            velocity: s.omega,
        },
        None => *fallback,
    }
}

/// Compute blend factor (0.0 = sensor, 1.0 = observer)
pub(super) fn compute_blend(velocity: f32, blend_low: f32, blend_high: f32) -> f32 {
    if velocity <= blend_low {
        0.0
    } else if velocity >= blend_high {
        1.0
    } else {
        (velocity - blend_low) / (blend_high - blend_low)
    }
}

/// Blend two angles with given factor (0.0 = a, 1.0 = b)
pub(super) fn blend_angles(a: f32, b: f32, blend: f32) -> f32 {
    // Handle angle wraparound properly
    let diff = wrap_angle(b - a);
    let diff_signed = if diff > core::f32::consts::PI {
        diff - TAU
    } else {
        diff
    };

    wrap_angle(a + diff_signed * blend)
}
