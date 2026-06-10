//! Motor state management
//!
//! This module provides centralized state management for motor control.
//! Platforms instantiate the state with their own fault types.

use core::cell::RefCell;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;

use crate::foc::controller::FocOutput;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::foc::trig::SinCos;
use crate::motor::{ControlMode, FocDriver};
use crate::types::MotorState;

// ============================================================================
// Global Communication Channels
// ============================================================================

/// Command for the ISR-owned FocDriver.
///
/// The driver is mutated only inside the FOC ISR; every async-side request
/// to change it goes through [`CMD_CHANNEL`] and is applied in order by
/// [`process_commands`]. One channel (rather than per-purpose signals)
/// keeps the commands sequenced: "set limits, then start" arrives exactly
/// that way.
#[derive(Clone, Copy, Debug)]
pub enum DriverCommand {
    /// Change control mode (start/stop/targets)
    SetMode(ControlMode),
    /// Apply current limits (already clamped to the board ceiling)
    SetCurrentLimits(crate::motor::foc_driver::CurrentLimits),
    /// Apply current-loop PI gains (post-detection tune, config write)
    SetPiGains { kp: f32, ki: f32 },
}

/// Command channel - servers send DriverCommands here, ISR receives them
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, DriverCommand, 8> = Channel::new();

/// Waker for FOC cycle completion — ISR wakes after `update_telemetry()`.
/// Used by calibration/detection to synchronize with individual FOC cycles.
/// The listener reads `last_foc` from the state mutex after waking.
pub static TELEM_WAKER: AtomicWaker = AtomicWaker::new();

// ============================================================================
// State Structure
// ============================================================================

/// Motor control state
///
/// This struct holds all telemetry-related state that servers need to access.
/// It's updated by the platform's ISR and read by the protocol servers.
#[derive(Clone, Debug)]
pub struct MotorControlState {
    /// Current motor state (Stopped/Running/Error)
    pub motor_state: MotorState,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Last Hall sensor snapshot
    pub last_hall: Option<HallSnapshot>,
    /// Last ADC snapshot
    pub last_adc: AdcSnapshot,
    /// Last FOC telemetry
    pub last_foc: FocOutput,
    /// Link active flag (host has connected)
    pub link_active: bool,
}

impl MotorControlState {
    /// Create new state (const for static initialization)
    pub const fn new() -> Self {
        Self {
            motor_state: MotorState::Stopped,
            control_mode: ControlMode::Stopped,
            last_hall: None,
            last_adc: AdcSnapshot::empty(),
            last_foc: FocOutput::empty(),
            link_active: false,
        }
    }

    /// Set motor to stopped state.
    ///
    /// Deliberately does NOT exit [`MotorState::Error`]: a Stop command (or
    /// the link-loss failsafe, which routes through here) must not un-latch
    /// faults — only [`clear_error`](Self::clear_error) may, after the host
    /// explicitly cleared the fault registry.
    pub fn set_stopped(&mut self) {
        if self.motor_state != MotorState::Error {
            self.motor_state = MotorState::Stopped;
        }
        self.control_mode = ControlMode::Stopped;
    }

    /// Set motor to running with given control mode
    pub fn set_running(&mut self, mode: ControlMode) {
        self.motor_state = MotorState::Running;
        self.control_mode = mode;
    }

    /// Set motor to error state
    pub fn set_error(&mut self) {
        self.motor_state = MotorState::Error;
    }

    /// Clear error and return to stopped state
    pub fn clear_error(&mut self) {
        self.motor_state = MotorState::Stopped;
        self.control_mode = ControlMode::Stopped;
    }

    /// Update Hall snapshot
    pub fn update_hall(&mut self, hall: HallSnapshot) {
        self.last_hall = Some(hall);
    }

    /// Update ADC snapshot
    pub fn update_adc(&mut self, adc: AdcSnapshot) {
        self.last_adc = adc;
    }

    /// Update FOC telemetry
    pub fn update_foc(&mut self, foc: FocOutput) {
        self.last_foc = foc;
    }

    /// Mark link as active (host connected)
    pub fn set_link_active(&mut self) {
        self.link_active = true;
    }

    /// Mark link as inactive (host disconnected / liveness timed out).
    /// [`process_commands`] forces [`ControlMode::Stopped`] while inactive.
    pub fn set_link_inactive(&mut self) {
        self.link_active = false;
    }
}

impl Default for MotorControlState {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Platform State - Platforms define this
// ============================================================================

/// Platform must define this macro to provide state globals
///
/// Example usage in platform crate:
/// ```ignore
/// use oxifoc_core::define_platform_state;
/// define_platform_state!(MyFault);
/// ```
#[macro_export]
macro_rules! define_platform_state {
    ($fault_type:ty) => {
        /// Global motor state
        pub static STATE: ::critical_section::Mutex<
            ::core::cell::RefCell<$crate::state::MotorControlState>,
        > = ::critical_section::Mutex::new(::core::cell::RefCell::new(
            $crate::state::MotorControlState::new(),
        ));

        /// Global fault registry
        pub static FAULT_REGISTRY: $crate::foc::fault::FaultRegistry<$fault_type> =
            $crate::foc::fault::FaultRegistry::new();
    };
}

// ============================================================================
// Command Processing
// ============================================================================

/// Process pending commands from the channel
///
/// Call this from the ISR before running FOC. It processes any pending
/// ControlMode commands and applies them to the driver.
///
/// # Arguments
/// * `state_mutex` - Reference to the platform STATE global
/// * `foc` - Mutable reference to the FocDriver
///
/// # Returns
/// The current ControlMode after processing commands
pub fn process_commands<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    foc: &mut FocDriver<P, C, Ph, S>,
    fault_registry: &crate::foc::fault::FaultRegistry<F>,
) -> ControlMode
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: crate::foc::fault::PlatformFault,
{
    // Process all pending commands
    while let Ok(cmd) = CMD_CHANNEL.try_receive() {
        let mode = match cmd {
            DriverCommand::SetMode(mode) => mode,
            DriverCommand::SetCurrentLimits(limits) => {
                foc.set_current_limits(limits);
                continue;
            }
            DriverCommand::SetPiGains { kp, ki } => {
                foc.controller_mut().id_pi.set_gains(kp, ki);
                foc.controller_mut().iq_pi.set_gains(kp, ki);
                continue;
            }
        };
        critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();

            // Exit the Error latch only once the host has explicitly cleared
            // the fault registry (FaultRequest::Clear via fault_server).
            // Critical faults never auto-clear, so the acknowledgement stays
            // mandatory — but after it the next command works without a
            // separate "clear error" verb.
            if state.motor_state == MotorState::Error && !fault_registry.any() {
                state.clear_error();
            }

            // Can't change mode if in error state (must clear faults first),
            // and never start running with an active critical fault even if
            // the Error latch was missed (belt and braces: the latch is set
            // by platform glue, the registry by the fault checkers).
            if mode != ControlMode::Stopped
                && (state.motor_state == MotorState::Error || fault_registry.any_critical())
            {
                return;
            }

            // Apply the control mode
            match mode {
                ControlMode::Stopped => {
                    state.set_stopped();
                }
                _ => {
                    state.set_running(mode);
                }
            }
            foc.set_mode(mode);
        });
    }

    // Fail-safe: while the link is inactive (liveness timed out / host gone),
    // force Stopped regardless of the last commanded mode. After reconnect the
    // host must send a fresh command to run again. This runs in the ISR, so it
    // does not depend on any async task to react to link loss.
    let link_active = critical_section::with(|cs| state_mutex.borrow(cs).borrow().link_active);
    if !link_active && foc.mode() != ControlMode::Stopped {
        critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
        foc.set_mode(ControlMode::Stopped);
    }

    foc.mode()
}

/// One FOC cycle of driver work, shared by every platform ISR.
///
/// Call inside the platform's FOC_DRIVER lock after reading the ADC:
/// applies pending [`DriverCommand`]s, gates on faults, runs the FOC step
/// and checks the measured currents. Returns the cycle telemetry, or None
/// when the step was skipped (faulted) or failed.
pub fn run_foc_cycle<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &crate::foc::fault::FaultRegistry<F>,
    driver: &mut FocDriver<P, C, Ph, S>,
    vbus_v: f32,
    now_ticks: u64,
    board: &crate::foc::config::BoardConfig,
    overcurrent_fault: F,
) -> Option<FocOutput>
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: crate::foc::fault::PlatformFault,
{
    driver.set_vbus(vbus_v);

    let prev_mode = driver.mode();
    let mode = process_commands(state_mutex, driver, fault_registry);

    // Spurious break-input trips during PWM channel enable can latch an
    // OverCurrent fault right at start (seen on G431: COMP→BKIN glitch when
    // MOE re-arms). Scoped to the OverCurrent category: clearing the whole
    // registry here would bypass the host-acknowledged fault latch for
    // unrelated faults. A real latched OverCurrent cannot reach this line —
    // process_commands refuses the Stopped→active transition while any
    // critical fault is registered.
    if matches!(prev_mode, ControlMode::Stopped) && mode != ControlMode::Stopped {
        fault_registry.clear(crate::foc::fault::FaultCategory::OverCurrent);
    }

    // If faulted, disable outputs and skip the FOC step
    if fault_registry.any() {
        if mode != ControlMode::Stopped {
            driver.set_mode(ControlMode::Stopped);
        }
        return None;
    }

    match driver.step(now_ticks) {
        Ok(telem) => {
            // Instantaneous phase-current fault check
            let before = fault_registry.any();
            crate::foc::fault::check_current_faults(
                telem.ia,
                telem.ib,
                telem.ic,
                board,
                fault_registry,
                overcurrent_fault,
            );
            if !before && fault_registry.any() {
                #[cfg(feature = "defmt")]
                defmt::error!(
                    "SW overcurrent FAULT: ia={}, ib={}, ic={}",
                    telem.ia,
                    telem.ib,
                    telem.ic
                );
            }
            Some(telem)
        }
        Err(_e) => {
            #[cfg(feature = "defmt")]
            defmt::error!("FOC step error: {}", _e);
            // Sensor not ready or other error - disable outputs
            if mode != ControlMode::Stopped {
                driver.set_mode(ControlMode::Stopped);
            }
            None
        }
    }
}

/// Update state with new telemetry from ISR
///
/// Call this after running FOC step to update the global state
/// with fresh telemetry data.
pub fn update_telemetry(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    adc: AdcSnapshot,
    hall: Option<HallSnapshot>,
    foc: FocOutput,
) {
    // Update state
    critical_section::with(|cs| {
        let mut state = state_mutex.borrow(cs).borrow_mut();
        state.update_adc(adc);
        if let Some(h) = hall {
            state.update_hall(h);
        }
        state.update_foc(foc);
    });

    // Wake calibration/detection task if waiting for FOC cycle
    TELEM_WAKER.wake();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_does_not_clear_error_latch() {
        // A Stopped command (or the link-loss failsafe, which uses the same
        // path) must not silently un-latch the Error state: only an explicit
        // fault clear may. Otherwise "stop, then run" bypasses every latched
        // fault.
        let mut st = MotorControlState::new();
        st.set_error();
        st.set_stopped();
        assert_eq!(st.motor_state, MotorState::Error, "Stop cleared the latch");
        assert_eq!(st.control_mode, ControlMode::Stopped);

        st.clear_error();
        assert_eq!(st.motor_state, MotorState::Stopped);
    }
}
