//! Fault handling for motor control systems
//!
//! Provides a shared fault registry and action policies for handling
//! hardware faults consistently across platforms.
//!
//! Platform crates map hardware-specific conditions (overcurrent, driver fault,
//! undervoltage, overtemperature, calibration failure, etc.) into this
//! registry so the control loop and host telemetry can react consistently.

use core::sync::atomic::{AtomicU32, Ordering};

/// Fault categories understood by the control stack.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultKind {
    OverCurrent = 0,
    OverVoltage = 1,
    UnderVoltage = 2,
    OverTemp = 3,
    DriverFault = 4,
    CalibrationFailed = 5,
    CommsTimeout = 6,
    HallSensorError = 7,
    Stall = 8,
    Unknown = 15,
}

/// Action to take when a fault occurs
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultAction {
    /// Log the fault but continue operation
    Log,
    /// Disable PWM outputs but keep control loop running
    DisableOutput,
    /// Emergency stop - halt all operations
    EmergencyStop,
}

impl FaultKind {
    /// Get the default action for this fault type
    ///
    /// Returns the recommended safety action. Platforms can override
    /// specific fault actions via `FaultHandler`.
    pub const fn default_action(self) -> FaultAction {
        match self {
            FaultKind::OverCurrent => FaultAction::EmergencyStop,
            FaultKind::OverVoltage => FaultAction::DisableOutput,
            FaultKind::UnderVoltage => FaultAction::DisableOutput,
            FaultKind::OverTemp => FaultAction::DisableOutput,
            FaultKind::DriverFault => FaultAction::EmergencyStop,
            FaultKind::CalibrationFailed => FaultAction::Log,
            FaultKind::CommsTimeout => FaultAction::Log,
            FaultKind::HallSensorError => FaultAction::DisableOutput,
            FaultKind::Stall => FaultAction::DisableOutput,
            FaultKind::Unknown => FaultAction::DisableOutput,
        }
    }

    /// Returns true if this fault can auto-clear when condition resolves.
    ///
    /// Recoverable faults (undervoltage, comms timeout) will automatically
    /// clear when the fault condition is no longer present.
    /// Non-recoverable faults (overcurrent, overtemp, driver fault) require
    /// explicit host command to clear.
    pub const fn is_recoverable(self) -> bool {
        matches!(self, FaultKind::UnderVoltage | FaultKind::CommsTimeout)
    }
}

// ============================================================================
// Hysteresis constants for auto-recovery
// ============================================================================

/// Voltage hysteresis in millivolts.
/// Undervoltage clears when Vbus > min_vbus_mv + VOLTAGE_HYSTERESIS_MV
pub const VOLTAGE_HYSTERESIS_MV: u32 = 500;

/// Bitmask helper for a fault flag.
const fn bit(kind: FaultKind) -> u32 {
    1u32 << kind as u8
}

/// Platform-specific fault handler trait
///
/// Implement this trait to define how your platform responds to faults.
/// For example, disabling PWM, enabling brake resistor, triggering watchdog, etc.
pub trait FaultHandler {
    /// Handle a fault with the given action
    ///
    /// # Arguments
    /// * `fault` - The fault that occurred
    /// * `action` - The action to take (may be overridden by platform policy)
    fn handle_fault(&mut self, fault: FaultKind, action: FaultAction);

    /// Optional: Get platform-specific action for a fault
    ///
    /// Override this to customize fault actions. Default uses `FaultKind::default_action()`.
    fn get_action(&self, fault: FaultKind) -> FaultAction {
        fault.default_action()
    }
}

/// Atomic fault registry (no_std friendly).
///
/// This can be placed in a `static` and shared across ISRs and tasks.
/// Use with a `FaultHandler` implementation for active fault management.
#[derive(Debug)]
pub struct FaultRegistry {
    flags: AtomicU32,
}

impl FaultRegistry {
    /// Create a new registry with all faults cleared.
    pub const fn new() -> Self {
        Self {
            flags: AtomicU32::new(0),
        }
    }

    /// Set a fault bit.
    pub fn set(&self, kind: FaultKind) {
        let mask = bit(kind);
        self.flags.fetch_or(mask, Ordering::Relaxed);
    }

    /// Set a fault and invoke handler
    ///
    /// This is the preferred method when a handler is available.
    pub fn set_with_handler(&self, kind: FaultKind, handler: &mut impl FaultHandler) {
        self.set(kind);
        let action = handler.get_action(kind);
        handler.handle_fault(kind, action);
    }

    /// Clear a fault bit.
    pub fn clear(&self, kind: FaultKind) {
        let mask = bit(kind);
        self.flags.fetch_and(!mask, Ordering::Relaxed);
    }

    /// Clear all faults.
    pub fn clear_all(&self) {
        self.flags.store(0, Ordering::Relaxed);
    }

    /// Returns `true` if any fault is active.
    pub fn any(&self) -> bool {
        self.flags.load(Ordering::Relaxed) != 0
    }

    /// Returns `true` if the specific fault is set.
    pub fn is_set(&self, kind: FaultKind) -> bool {
        let mask = bit(kind);
        self.flags.load(Ordering::Relaxed) & mask != 0
    }

    /// Check for any faults and invoke handler if needed
    ///
    /// Returns `true` if any fault is active.
    pub fn check_and_handle(&self, handler: &mut impl FaultHandler) -> bool {
        let bits = self.bits();
        if bits == 0 {
            return false;
        }

        // Check each fault and handle active ones
        for fault in Self::ALL_FAULTS {
            if self.is_set(fault) {
                let action = handler.get_action(fault);
                handler.handle_fault(fault, action);
            }
        }

        true
    }

    /// Clear faults by bitmask
    pub fn clear_mask(&self, mask: u32) {
        self.flags.fetch_and(!mask, Ordering::Relaxed);
    }

    /// All fault kinds for iteration
    const ALL_FAULTS: [FaultKind; 10] = [
        FaultKind::OverCurrent,
        FaultKind::OverVoltage,
        FaultKind::UnderVoltage,
        FaultKind::OverTemp,
        FaultKind::DriverFault,
        FaultKind::CalibrationFailed,
        FaultKind::CommsTimeout,
        FaultKind::HallSensorError,
        FaultKind::Stall,
        FaultKind::Unknown,
    ];

    /// Raw fault bitfield (useful for telemetry).
    pub fn bits(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// Get iterator over active faults
    pub fn active_faults(&self) -> impl Iterator<Item = FaultKind> {
        let bits = self.bits();
        Self::ALL_FAULTS.into_iter().filter(move |&fault| {
            let mask = bit(fault);
            bits & mask != 0
        })
    }
}

impl Default for FaultRegistry {
    fn default() -> Self {
        Self::new()
    }
}
