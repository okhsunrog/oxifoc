//! Temporary, ISR-owned commissioning constraints. Installed ride settings
//! remain the restoration baseline and never determine the test-current cap.

use super::actuation::ActuationModel;
use super::{CurrentLimits, FocDriver, OVERCURRENT_HEADROOM, PowerSourceLimits};
use crate::foc::{phase::PhaseProvider, pwm::PhasePwm, sensors::CurrentSensor, trig::SinCos};
use crate::units::{
    AngleRepr, ConstZero, ElectricCurrent, ElectricalAngle, Linear, Power, Sign, amps, seconds,
};

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
        board_phase_trip: ElectricCurrent,
        configured_dq_trip: ElectricCurrent,
        source: PowerSourceLimits,
        test_ceiling: Option<ElectricCurrent>,
    ) -> Result<Self, CommissioningError> {
        if [board_phase_trip, configured_dq_trip]
            .iter()
            .any(|v| !v.is_finite() || v.is_non_positive())
        {
            return Err(CommissioningError::InvalidProtection);
        }
        if test_ceiling.is_some_and(|v| !v.is_finite() || v.is_non_positive()) {
            return Err(CommissioningError::InvalidTestCeiling);
        }
        let trip = board_phase_trip.min(configured_dq_trip);
        let ceiling = trip / OVERCURRENT_HEADROOM;
        let ceiling = test_ceiling.map_or(ceiling, |v| v.min(ceiling));
        Ok(Self {
            source,
            limits: CurrentLimits {
                max_current: ceiling,
                max_brake: ceiling,
                overcurrent_threshold: trip,
                bus_in_max: source.input(),
                bus_regen_max: source.regen(),
                max_dissipation: Power::ZERO,
            },
        })
    }

    pub fn current_ceiling(self) -> ElectricCurrent {
        self.limits.max_current
    }
}

pub(super) struct CommissioningContext {
    pub generation: u32,
    restore: CurrentLimits,
    source: PowerSourceLimits,
    /// Duty-fraction αβ commands: the bus current is their dot product with
    /// the phase-current vector.
    pub modulation: ActuationModel<f32>,
    filtered_current: ElectricCurrent,
    filter_alpha: f32,
    source_constrained: bool,
}

impl CommissioningContext {
    pub fn constrain_current<S: SinCos>(
        &mut self,
        angle: ElectricalAngle,
        current: ElectricCurrent,
        rotating: bool,
    ) -> ElectricCurrent {
        let (a, b) = self.modulation.applied_voltage();
        let (sin, cos) = S::sin_cos(angle.radians());
        let coefficient = if rotating {
            -a * sin + b * cos
        } else {
            a * cos + b * sin
        };
        let input = coefficient * current;
        if input > self.source.input() {
            self.source_constrained = true;
            current * self.source.input().ratio_to(input)
        } else if input < -self.source.regen() {
            self.source_constrained = true;
            current * self.source.regen().ratio_to(-input)
        } else {
            current
        }
    }
    /// Signed cycle-average inverter-input estimate. Modulation is paired
    /// with the current sample through the same delay model as the observer.
    /// This is not a measurement of current upstream of the DC-link capacitor.
    pub fn source_allows(&mut self, alpha: ElectricCurrent, beta: ElectricCurrent) -> bool {
        let (a, b) = self.modulation.applied_voltage();
        let current = a * alpha + b * beta;
        if !current.is_finite() {
            return false;
        }
        self.filtered_current += self.filter_alpha * (current - self.filtered_current);
        // PWM/HFI exchanges energy with the DC-link capacitor. Source feedback
        // uses the same 2-ms bandwidth as the existing drive bus limiter.
        // A 1-mA estimate floor rejects zero-vector measurement/rounding noise
        // (MK5 startup capture: below 0.1 mA); it does not authorize commanded
        // regeneration, whose target grant retains the exact zero limit.
        const ESTIMATE_FLOOR: ElectricCurrent = amps(0.001);
        self.filtered_current <= self.source.input() + ESTIMATE_FLOOR
            && self.filtered_current >= -self.source.regen() - ESTIMATE_FLOOR
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
            filtered_current: ElectricCurrent::ZERO,
            filter_alpha: self.dt.ratio_to(seconds(0.002) + self.dt),
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
