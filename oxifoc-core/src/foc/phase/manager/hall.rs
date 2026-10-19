//! Hall health, fault flags, and hall-calibration accessors.

use super::{DEFAULT_OPENLOOP_MIN_VEL, HallHealth, PhaseFault, PhaseManager};
use crate::foc::hall_calibration::HallCalibrationResult;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::sensors::AngleSensor;
use crate::foc::sensors::HallSensorTrait;
use crate::foc::trig::SinCos;

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Set a fault (adds if not already present)
    pub(super) fn set_fault(&mut self, fault: PhaseFault) {
        if !self.faults.contains(&fault) {
            let _ = self.faults.push(fault);
        }
    }

    /// Clear a specific fault
    pub(super) fn clear_fault(&mut self, fault: PhaseFault) {
        self.faults.retain(|f| *f != fault);
    }

    /// Update Hall health status from sample availability and staleness.
    pub(super) fn update_hall_health(&mut self, sample_valid: bool, stale: bool) {
        // Skip if Hall is not configured
        if matches!(self.hall_health, HallHealth::NotPresent) {
            return;
        }

        if sample_valid && !stale {
            // Hall is working - clear failures
            if self.hall_health != HallHealth::Ok {
                self.hall_health = HallHealth::Ok;
                self.clear_fault(PhaseFault::HallTimeout);
                self.clear_fault(PhaseFault::HallInvalidState);
            }
        } else if stale {
            // Edges stopped while the rotor was spinning (velocity-adaptive
            // check in the sensor) — dead cable / dead sensor.
            self.hall_health = HallHealth::Stale;
            self.set_fault(PhaseFault::HallTimeout);
        } else {
            // No valid sample at all (invalid 0/7 states, never seen an
            // edge, ...).
            self.hall_health = HallHealth::Invalid;
            self.set_fault(PhaseFault::HallInvalidState);
        }
    }

    /// Try to get observer output for fallback, with open-loop override as last resort
    pub(super) fn try_observer_fallback(&mut self) -> Option<PhaseOutput> {
        // Gate on is_ready(), not phase().is_some(): every *configured*
        // observer returns a phase, including one frozen at 0 with zero
        // confidence. Handing it commutation on a sensor dropout would swap
        // a good angle for garbage.
        let ready_output = if self.observer.is_ready() {
            match (self.observer.phase(), self.observer.velocity()) {
                (Some(angle), Some(vel)) => Some(PhaseOutput {
                    angle,
                    velocity: vel,
                }),
                _ => None,
            }
        } else {
            None
        };

        if let Some(out) = ready_output {
            self.clear_fault(PhaseFault::ObserverNotReady);
            if self.open_loop_override.active {
                self.deactivate_open_loop_override();
            }
            Some(out)
        } else {
            // Observer not ready - use open-loop override if available
            self.set_fault(PhaseFault::ObserverNotReady);

            if !self.open_loop_override.active {
                // Activate open-loop override for recovery, continuing from
                // the last known output angle at minimum velocity. Signed by
                // the last known velocity (VESC-style): a board rolling
                // backward must not be spun forward by the recovery override.
                let dir = if self.output.velocity < 0.0 {
                    -1.0
                } else {
                    1.0
                };
                self.activate_open_loop_override(self.output.angle, dir * DEFAULT_OPENLOOP_MIN_VEL);
            }
            Some(PhaseOutput {
                angle: self.open_loop_override.angle,
                velocity: self.open_loop_override.velocity,
            })
        }
    }
}

// ============================================================================
// Conditional methods for Hall sensor
// ============================================================================

impl<H: HallSensorTrait, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Set Hall calibration table
    pub fn set_hall_calibration(&mut self, table: [f32; 6]) {
        self.hall.set_calibration(table);
    }

    /// Apply Hall calibration result
    pub fn apply_hall_calibration(&mut self, result: &HallCalibrationResult) -> bool {
        self.hall.apply_calibration(result)
    }

    /// Get raw Hall state (0-7)
    pub fn hall_raw_state(&self) -> u8 {
        self.hall.raw_state()
    }

    /// Get logical Hall state (0-5)
    pub fn hall_logical_state(&self) -> u8 {
        self.hall.logical_state()
    }

    /// Set Hall timing advance
    pub fn set_hall_advance(&mut self, advance_rad: f32) {
        self.hall.set_advance(advance_rad);
    }

    /// Get Hall timing advance
    pub fn hall_advance(&self) -> f32 {
        self.hall.advance()
    }

    /// Get Hall electrical velocity
    pub fn hall_velocity(&self) -> f32 {
        self.hall.electrical_velocity()
    }
}
