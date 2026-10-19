//! ISR-owned current-offset calibration sequenced ahead of the mode arm.

use super::{CURRENT_OFFSET_STANDSTILL_MAX_E_RAD_S, ControlMode, FocDriver};
use crate::foc::controller::FocOutput;
use crate::foc::current_offset::CurrentOffsetError;
use crate::foc::current_offset::CurrentOffsetMethod;
use crate::foc::current_offset::CurrentOffsetReport;
use crate::foc::current_offset::CurrentOffsetRequest;
use crate::foc::current_offset::float_all;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::pwm::PhaseState;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Begin an ISR-driven current-offset measurement.
    pub fn start_current_offset_calibration(
        &mut self,
        request: CurrentOffsetRequest,
    ) -> Result<(), CurrentOffsetError> {
        if !matches!(self.mode, ControlMode::Stopped) || self.failsafe_ctrl.is_active() {
            return Err(CurrentOffsetError::Busy);
        }
        if request.method == CurrentOffsetMethod::AllPhases50 {
            if !request.stationary_confirmed {
                return Err(CurrentOffsetError::UnsafeWhileMoving);
            }
            if !self.current_sensor.is_calibrated() {
                return Err(CurrentOffsetError::RequiresExistingCalibration);
            }
            let velocity = self.phase.get().velocity;
            if !velocity.is_finite() || velocity.abs() > CURRENT_OFFSET_STANDSTILL_MAX_E_RAD_S {
                return Err(CurrentOffsetError::UnsafeWhileMoving);
            }
        }
        self.current_offset.start(request)?;
        self.current_offset_outcome = None;
        // Establish the safe topology synchronously. The first calibration
        // step will select its measurement topology and begin the settle wait.
        self.pwm.set_phase_states(float_all());
        self.pwm_off = true;
        Ok(())
    }

    /// Whether a current-offset maintenance operation owns the bridge.
    pub fn current_offset_active(&self) -> bool {
        self.current_offset.is_active()
    }

    /// Apply offsets supplied by config without exposing the sensor itself to
    /// async code. Rejected while a measurement owns the sensor/PWM.
    pub fn set_current_offsets(&mut self, offsets: (f32, f32, f32)) {
        // Diagnostics accumulate raw ADC counts, so a config write that was
        // already in flight may safely update conversion offsets without
        // perturbing the measurement. The all-phase safety check simply uses
        // the newly supplied (still calibrated) conversion on later cycles.
        self.current_sensor.set_offsets(offsets);
    }

    /// Cancel calibration and float the bridge immediately.
    pub fn cancel_current_offset_calibration(&mut self) {
        if self.current_offset.cancel() {
            self.pwm.set_phase_states(float_all());
            self.pwm_off = true;
            self.current_offset_outcome = Some(Err(CurrentOffsetError::Cancelled));
        }
    }

    /// Consume the one-shot result for delivery to the async requester.
    pub fn take_current_offset_outcome(
        &mut self,
    ) -> Option<Result<CurrentOffsetReport, CurrentOffsetError>> {
        self.current_offset_outcome.take()
    }

    pub(super) fn step_current_offset(&mut self) -> FocOutput {
        let Some(request) = self.current_offset.request() else {
            return FocOutput::default();
        };

        // The alternative topology deliberately retains the previous
        // calibration so software current protection remains meaningful.
        if request.method == CurrentOffsetMethod::AllPhases50 {
            let velocity = self.phase.get().velocity;
            let currents = self.current_sensor.read_currents();
            let limit = self.current_limits.overcurrent_threshold_a;
            let moving =
                !velocity.is_finite() || velocity.abs() > CURRENT_OFFSET_STANDSTILL_MAX_E_RAD_S;
            let overcurrent = limit > 0.0
                && (currents.0.abs() > limit
                    || currents.1.abs() > limit
                    || currents.2.abs() > limit);
            if moving || overcurrent {
                self.current_offset.cancel();
                self.pwm.set_phase_states(float_all());
                self.pwm_off = true;
                self.current_offset_outcome = Some(Err(if overcurrent {
                    CurrentOffsetError::OverCurrent
                } else {
                    CurrentOffsetError::UnsafeWhileMoving
                }));
                return FocOutput::default();
            }
        }

        let raw = self.current_sensor.read_raw();
        let progress = self
            .current_offset
            .tick([raw.0, raw.1, raw.2], self.pwm.max_duty());
        if let Some(states) = progress.phase_states {
            self.pwm.set_phase_states(states);
            self.pwm_off = states
                .iter()
                .all(|state| matches!(state, PhaseState::Float));
        }
        if let Some(report) = progress.report {
            // Float before mutating/publishing anything: completion is never
            // observable while a calibration topology remains energised.
            self.pwm.set_phase_states(float_all());
            self.pwm_off = true;
            if request.apply {
                self.current_sensor.set_offsets((
                    report.offsets[0],
                    report.offsets[1],
                    report.offsets[2],
                ));
            }
            self.current_offset_outcome = Some(Ok(report));
        }
        FocOutput::default()
    }
}
