//! Motor control helpers for oxifoc-g431
pub mod fault_handler;
pub mod pwm;

use core::sync::atomic::{AtomicU8, Ordering};
use oxifoc_protocol::{MotorState, MotorStatus};

/// Global motor state (for host telemetry).
static MOTOR_STATE: AtomicU8 = AtomicU8::new(MotorState::Stopped as u8);
static MOTOR_DUTY: AtomicU8 = AtomicU8::new(0);
static MOTOR_STEP: AtomicU8 = AtomicU8::new(0);

pub fn set_motor_state(state: MotorState) {
    MOTOR_STATE.store(state as u8, Ordering::Relaxed);
}

pub fn get_motor_state() -> MotorState {
    match MOTOR_STATE.load(Ordering::Relaxed) {
        0 => MotorState::Stopped,
        1 => MotorState::Running,
        _ => MotorState::Error,
    }
}

pub fn set_motor_duty(duty: u8) {
    MOTOR_DUTY.store(duty, Ordering::Relaxed);
}

pub fn get_motor_duty() -> u8 {
    MOTOR_DUTY.load(Ordering::Relaxed)
}

pub fn set_motor_step(step: u8) {
    MOTOR_STEP.store(step, Ordering::Relaxed);
}

pub fn get_motor_step() -> u8 {
    MOTOR_STEP.load(Ordering::Relaxed)
}

pub fn get_motor_status() -> MotorStatus {
    MotorStatus {
        state: get_motor_state(),
        duty: get_motor_duty(),
        step: get_motor_step(),
    }
}
