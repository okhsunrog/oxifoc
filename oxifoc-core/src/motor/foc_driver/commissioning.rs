//! Temporary, ISR-owned commissioning constraints. Installed ride settings
//! remain the restoration baseline and never determine the test-current cap.

use super::actuation::ActuationModel;
use super::{CurrentLimits, FocDriver, OVERCURRENT_HEADROOM, PowerSourceLimits};
use crate::foc::{phase::PhaseProvider, pwm::PhasePwm, sensors::CurrentSensor, trig::SinCos};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommissioningError {
    InvalidProtection,
    InvalidTestCeiling,
    Busy,
    NotCalibrated,
}

/// Validated outside the ISR. Both trip arguments are inverter protections,
/// independent of the previously installed motor's rating and ride limits.
#[derive(Clone, Copy, Debug)]
pub struct PreparedCommissioning {
    limits: CurrentLimits,
    source: PowerSourceLimits,
}

impl PreparedCommissioning {
    pub fn new(
        board_phase_trip_a: f32,
        configured_dq_trip_a: f32,
        source: PowerSourceLimits,
        test_ceiling_a: Option<f32>,
    ) -> Result<Self, CommissioningError> {
        if [board_phase_trip_a, configured_dq_trip_a]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(CommissioningError::InvalidProtection);
        }
        if test_ceiling_a.is_some_and(|v| !v.is_finite() || v <= 0.0) {
            return Err(CommissioningError::InvalidTestCeiling);
        }
        let trip = board_phase_trip_a.min(configured_dq_trip_a);
        let ceiling = trip / OVERCURRENT_HEADROOM;
        let ceiling = test_ceiling_a.map_or(ceiling, |v| v.min(ceiling));
        Ok(Self {
            source,
            limits: CurrentLimits {
                max_current_a: ceiling,
                max_brake_a: ceiling,
                overcurrent_threshold_a: trip,
                bus_in_max_a: source.input_a(),
                bus_regen_max_a: source.regen_a(),
                max_dissipation_w: 0.0,
            },
        })
    }

    pub fn current_ceiling_a(self) -> f32 {
        self.limits.max_current_a
    }
}

pub(super) struct CommissioningContext {
    pub generation: u32,
    restore: CurrentLimits,
    source: PowerSourceLimits,
    pub modulation: ActuationModel,
    filtered_current_a: f32,
    filter_alpha: f32,
    source_constrained: bool,
}

impl CommissioningContext {
    pub fn constrain_current<S: SinCos>(
        &mut self,
        angle: f32,
        current: f32,
        rotating: bool,
    ) -> f32 {
        let (a, b) = self.modulation.applied_voltage();
        let (sin, cos) = S::sin_cos(angle);
        let coefficient = if rotating {
            -a * sin + b * cos
        } else {
            a * cos + b * sin
        };
        let input = coefficient * current;
        if input > self.source.input_a() {
            self.source_constrained = true;
            current * (self.source.input_a() / input)
        } else if input < -self.source.regen_a() {
            self.source_constrained = true;
            current * (self.source.regen_a() / -input)
        } else {
            current
        }
    }
    /// Signed cycle-average inverter-input estimate. Modulation is paired
    /// with the current sample through the same delay model as the observer.
    /// This is not a measurement of current upstream of the DC-link capacitor.
    pub fn source_allows(&mut self, alpha: f32, beta: f32) -> bool {
        let (a, b) = self.modulation.applied_voltage();
        let current = a * alpha + b * beta;
        if !current.is_finite() {
            return false;
        }
        self.filtered_current_a += self.filter_alpha * (current - self.filtered_current_a);
        // PWM/HFI exchanges energy with the DC-link capacitor. Source feedback
        // uses the same 2-ms bandwidth as the existing drive bus limiter.
        // A 1-mA estimate floor rejects zero-vector measurement/rounding noise
        // (MK5 startup capture: below 0.1 mA); it does not authorize commanded
        // regeneration, whose target grant retains the exact zero limit.
        const ESTIMATE_FLOOR_A: f32 = 0.001;
        self.filtered_current_a <= self.source.input_a() + ESTIMATE_FLOOR_A
            && self.filtered_current_a >= -self.source.regen_a() - ESTIMATE_FLOOR_A
    }
}

impl<P: PhasePwm, C: CurrentSensor, Ph: PhaseProvider, S: SinCos> FocDriver<P, C, Ph, S> {
    pub fn begin_commissioning(
        &mut self,
        generation: u32,
        prepared: PreparedCommissioning,
    ) -> Result<(), CommissioningError> {
        if self.mode != super::ControlMode::Stopped
            || self.commissioning.is_some()
            || self.current_offset_active()
            || self.failsafe_active()
            || self.failsafe_latched()
        {
            return Err(CommissioningError::Busy);
        }
        if !self.current_sensor.is_calibrated() {
            return Err(CommissioningError::NotCalibrated);
        }
        let mut modulation = ActuationModel::default();
        modulation.set_delay(self.actuation.delay_cycles());
        self.commissioning = Some(CommissioningContext {
            generation,
            restore: self.current_limits,
            source: prepared.source,
            modulation,
            filtered_current_a: 0.0,
            filter_alpha: self.dt / (0.002 + self.dt),
            source_constrained: false,
        });
        self.current_limits = prepared.limits;
        self.controller.reset();
        self.invalidate_velocity();
        Ok(())
    }

    pub fn commissioning_generation(&self) -> Option<u32> {
        self.commissioning.as_ref().map(|c| c.generation)
    }

    pub fn commissioning_source_constrained(&self) -> bool {
        self.commissioning
            .as_ref()
            .is_some_and(|c| c.source_constrained)
    }

    /// Always disarm; restoring configuration never resumes a previous mode.
    pub fn end_commissioning(&mut self) {
        if self.commissioning.is_some() {
            self.safe_off();
        }
    }

    pub(super) fn restore_commissioning_limits(&mut self) {
        if let Some(context) = self.commissioning.take() {
            self.current_limits = context.restore;
            self.bus_mod_q_filt = 0.0;
        }
    }
}
