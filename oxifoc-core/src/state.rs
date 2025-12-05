//! Motor state management
//!
//! This module provides centralized state management for motor control.
//! Core owns the state, platforms update it via the provided methods.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                         oxifoc-core                             │
//! │  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────┐  │
//! │  │  STATE (global) │  │  CMD_CHANNEL    │  │  TELEMETRY     │  │
//! │  │  - motor_state  │  │  - ControlMode  │  │  - FocTelemetry│  │
//! │  │  - fault        │  │                 │  │                │  │
//! │  │  - hall/adc     │  │                 │  │                │  │
//! │  └────────┬────────┘  └────────┬────────┘  └────────┬───────┘  │
//! │           │                    │                    │          │
//! │           ▼                    ▼                    ▼          │
//! │  ┌─────────────────────────────────────────────────────────┐   │
//! │  │                      Servers                             │   │
//! │  │   (access STATE directly, send to CMD_CHANNEL)          │   │
//! │  └─────────────────────────────────────────────────────────┘   │
//! └─────────────────────────────────────────────────────────────────┘
//!                              │
//!                    platform calls
//!                              ▼
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                         Platform                                │
//! │  ┌──────────────────────────────────────────────────────────┐  │
//! │  │                        ADC ISR                            │  │
//! │  │   1. Read ADC → AdcSample                                 │  │
//! │  │   2. core::process_commands(&mut foc_driver)              │  │
//! │  │   3. foc_driver.step() if running                         │  │
//! │  │   4. core::update_telemetry(...)                          │  │
//! │  └──────────────────────────────────────────────────────────┘  │
//! │  ┌──────────────────────────────────────────────────────────┐  │
//! │  │                     FocDriver<P,C,Ph>                     │  │
//! │  │   (platform-specific, owns hardware)                      │  │
//! │  └──────────────────────────────────────────────────────────┘  │
//! └─────────────────────────────────────────────────────────────────┘
//! ```

use core::cell::RefCell;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;

use crate::foc::controller::FocTelemetry;
use crate::foc::fault::{FaultKind, FaultRegistry};
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::motor::{ControlMode, FocDriver};
use crate::types::{FaultCode, MotorState, MotorStatus};

// ============================================================================
// Global State
// ============================================================================

/// Command channel - servers send ControlMode here, ISR receives them
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, ControlMode, 4> = Channel::new();

/// Telemetry watch - ISR broadcasts, streaming tasks can subscribe
pub static TELEMETRY: Watch<CriticalSectionRawMutex, FocTelemetry, 2> = Watch::new();

/// Global motor state - owned by core, updated by platform ISR
pub static STATE: CriticalSectionMutex<RefCell<MotorControlState>> =
    CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));

/// Global fault registry - atomic, ISR-safe
pub static FAULT_REGISTRY: FaultRegistry = FaultRegistry::new();

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
    /// Active fault (if any)
    pub fault: Option<FaultCode>,
    /// Last Hall sensor snapshot
    pub last_hall: Option<HallSnapshot>,
    /// Last ADC snapshot
    pub last_adc: AdcSnapshot,
    /// Last FOC telemetry
    pub last_foc: FocTelemetry,
    /// Link active flag (host has connected)
    pub link_active: bool,
}

impl MotorControlState {
    /// Create new state (const for static initialization)
    pub const fn new() -> Self {
        Self {
            motor_state: MotorState::Stopped,
            control_mode: ControlMode::Stopped,
            fault: None,
            last_hall: None,
            last_adc: AdcSnapshot::empty(),
            last_foc: FocTelemetry::empty(),
            link_active: false,
        }
    }

    /// Get current motor status for protocol response
    pub fn status(&self) -> MotorStatus {
        let fault_bits = FAULT_REGISTRY.bits();
        // Get primary fault from registry if any
        let fault = FAULT_REGISTRY
            .active_faults()
            .next()
            .map(FaultCode::from)
            .or(self.fault);
        MotorStatus {
            state: self.motor_state,
            mode: self.control_mode,
            fault,
            fault_bits,
        }
    }

    /// Set motor to stopped state
    pub fn set_stopped(&mut self) {
        self.motor_state = MotorState::Stopped;
        self.control_mode = ControlMode::Stopped;
    }

    /// Set motor to running with given control mode
    pub fn set_running(&mut self, mode: ControlMode) {
        self.motor_state = MotorState::Running;
        self.control_mode = mode;
    }

    /// Set motor to error state
    pub fn set_error(&mut self, fault: FaultCode) {
        self.motor_state = MotorState::Error;
        self.fault = Some(fault);
    }

    /// Clear fault and return to stopped state
    pub fn clear_fault(&mut self) {
        self.fault = None;
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
    pub fn update_foc(&mut self, foc: FocTelemetry) {
        self.last_foc = foc;
    }

    /// Mark link as active (host connected)
    pub fn set_link_active(&mut self) {
        self.link_active = true;
    }
}

impl Default for MotorControlState {
    fn default() -> Self {
        Self::new()
    }
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
/// * `foc` - Mutable reference to the FocDriver
///
/// # Returns
/// The current ControlMode after processing commands
pub fn process_commands<P, C, Ph>(foc: &mut FocDriver<P, C, Ph>) -> ControlMode
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
{
    // Process all pending commands
    while let Ok(mode) = CMD_CHANNEL.try_receive() {
        critical_section::with(|cs| {
            let mut state = STATE.borrow(cs).borrow_mut();

            // Can't change mode if in error state (must clear fault first via separate mechanism)
            if state.motor_state == MotorState::Error && mode != ControlMode::Stopped {
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

    foc.mode()
}

/// Update state with new telemetry from ISR
///
/// Call this after running FOC step to update the global state
/// with fresh telemetry data.
pub fn update_telemetry(adc: AdcSnapshot, hall: Option<HallSnapshot>, foc: FocTelemetry) {
    // Update state
    critical_section::with(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        state.update_adc(adc);
        if let Some(h) = hall {
            state.update_hall(h);
        }
        state.update_foc(foc.clone());
    });

    // Broadcast to any subscribers
    TELEMETRY.sender().send(foc);
}

// ============================================================================
// State Access Helpers
// ============================================================================

/// Get current motor status (for server responses)
pub fn motor_status() -> MotorStatus {
    critical_section::with(|cs| STATE.borrow(cs).borrow().status())
}

/// Get last ADC snapshot
pub fn adc_snapshot() -> AdcSnapshot {
    critical_section::with(|cs| STATE.borrow(cs).borrow().last_adc.clone())
}

/// Get last Hall snapshot
pub fn hall_snapshot() -> Option<HallSnapshot> {
    critical_section::with(|cs| STATE.borrow(cs).borrow().last_hall)
}

/// Check if link is active
pub fn is_link_active() -> bool {
    critical_section::with(|cs| STATE.borrow(cs).borrow().link_active)
}

/// Mark link as active
pub fn set_link_active() {
    critical_section::with(|cs| {
        STATE.borrow(cs).borrow_mut().set_link_active();
    });
}

// ============================================================================
// Fault Management
// ============================================================================

/// Check if any fault is active
pub fn any_fault() -> bool {
    FAULT_REGISTRY.any()
}

/// Get current fault bits
pub fn fault_bits() -> u32 {
    FAULT_REGISTRY.bits()
}

/// Clear all faults and return motor to stopped state
pub fn clear_all_faults() {
    FAULT_REGISTRY.clear_all();
    critical_section::with(|cs| {
        STATE.borrow(cs).borrow_mut().clear_fault();
    });
}

/// Clear specific fault
pub fn clear_fault(kind: FaultKind) {
    FAULT_REGISTRY.clear(kind);
    // If no more faults, clear error state
    if !FAULT_REGISTRY.any() {
        critical_section::with(|cs| {
            STATE.borrow(cs).borrow_mut().clear_fault();
        });
    }
}

/// Set a fault (for use from ISR or fault detection)
pub fn set_fault(kind: FaultKind) {
    FAULT_REGISTRY.set(kind);
    critical_section::with(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        state.motor_state = MotorState::Error;
        state.fault = Some(FaultCode::from(kind));
    });
}
