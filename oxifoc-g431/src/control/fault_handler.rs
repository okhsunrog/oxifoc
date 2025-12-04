//! Fault handler implementation for B-G431B-ESC1
//!
//! Implements platform-specific responses to motor faults.

use oxifoc_core::foc::fault::{FaultAction, FaultHandler, FaultKind};
use oxifoc_protocol::MotorState;

use crate::motor::{MotorPwm, set_motor_state};

/// G431-specific fault handler
///
/// Coordinates fault responses across PWM, state management, and safety features.
#[allow(dead_code)]
pub struct G431FaultHandler<'a, 'd> {
    pwm: &'a mut MotorPwm<'d>,
    /// Optional: brake resistor control (future enhancement)
    _brake_enabled: bool,
}

#[allow(dead_code)]
impl<'a, 'd> G431FaultHandler<'a, 'd> {
    /// Create a new fault handler
    pub fn new(pwm: &'a mut MotorPwm<'d>) -> Self {
        Self {
            pwm,
            _brake_enabled: false,
        }
    }

    /// Emergency stop sequence
    fn emergency_stop(&mut self) {
        defmt::error!("EMERGENCY STOP triggered");
        self.pwm.emergency_stop();
        set_motor_state(MotorState::Error);
        // TODO: Enable brake resistor if available
    }

    /// Disable outputs but keep system alive
    fn disable_output(&mut self) {
        defmt::warn!("Disabling motor outputs");
        self.pwm.emergency_stop();
        set_motor_state(MotorState::Stopped);
    }
}

impl<'a, 'd> FaultHandler for G431FaultHandler<'a, 'd> {
    fn handle_fault(&mut self, fault: FaultKind, action: FaultAction) {
        defmt::warn!("Fault detected: {:?} -> action: {:?}", fault, action);

        match action {
            FaultAction::Log => {
                // Just log, already done above
            }
            FaultAction::DisableOutput => {
                self.disable_output();
            }
            FaultAction::EmergencyStop => {
                self.emergency_stop();
            }
        }
    }

    fn get_action(&self, fault: FaultKind) -> FaultAction {
        // Platform-specific overrides
        match fault {
            // Be extra cautious with overcurrent on this small board
            FaultKind::OverCurrent => FaultAction::EmergencyStop,
            // Everything else uses defaults
            _ => fault.default_action(),
        }
    }
}
