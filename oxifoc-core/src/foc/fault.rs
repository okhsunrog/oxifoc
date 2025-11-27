//! Simple fault registry shared across device targets.
//!
//! Platform crates map hardware-specific conditions (overcurrent, driver fault,
//! undervoltage, overtemperature, calibration failure, etc.) into this
//! registry so the control loop and host telemetry can react consistently.

use core::sync::atomic::{AtomicU32, Ordering};

/// Fault categories understood by the control stack.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    OverCurrent = 0,
    OverVoltage = 1,
    UnderVoltage = 2,
    OverTemp = 3,
    DriverFault = 4,
    CalibrationFailed = 5,
    CommsTimeout = 6,
    Unknown = 7,
}

/// Bitmask helper for a fault flag.
const fn bit(kind: FaultKind) -> u32 {
    1u32 << kind as u8
}

/// Atomic fault registry (no_std friendly).
///
/// This can be placed in a `static` and shared across ISRs and tasks.
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

    /// Raw fault bitfield (useful for telemetry).
    pub fn bits(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }
}
