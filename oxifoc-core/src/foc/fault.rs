//! Fault handling for motor control systems
//!
//! Provides a flexible fault registry that platforms can extend with their own fault types.
//! Platform crates define their own fault enums (with associated data if needed) and
//! the registry stores them for ISR-safe access.
//!
//! # Example Platform Fault Type
//!
//! ```ignore
//! use drv8301_dd::FaultStatus;
//!
//! #[derive(Clone, Copy, PartialEq)]
//! pub enum MyPlatformFault {
//!     OverCurrent,
//!     OverVoltage,
//!     UnderVoltage,
//!     OverTemp,
//!     HallSensorError,
//!     DrvFault(FaultStatus),  // Contains full driver fault details!
//! }
//! ```

#[cfg(feature = "runtime")]
use core::cell::RefCell;
#[cfg(feature = "runtime")]
use embassy_sync::blocking_mutex::CriticalSectionMutex;
#[cfg(feature = "runtime")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "runtime")]
use embassy_sync::signal::Signal;
#[cfg(feature = "runtime")]
use heapless::Vec;

use heapless::String;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// ============================================================================
// Protocol Fault Type (fixed, for wire protocol)
// ============================================================================

/// Protocol-level fault categories
///
/// This is the fixed set of fault categories used in the wire protocol.
/// Platforms map their rich fault types to these categories.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultCategory {
    /// No fault
    #[default]
    None,
    /// Over-current detected
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET or motor)
    OverTemp,
    /// Gate driver fault (DRV8301, etc.)
    DriverFault,
    /// Hall sensor error
    HallError,
    /// Motor stalled
    Stall,
    /// Calibration required or failed
    CalibrationFault,
    /// Communication timeout
    CommTimeout,
}

/// Fault information for protocol transmission
///
/// Contains a category (fixed enum) plus a human-readable detail string
/// that platforms can populate with specific information.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultInfo {
    /// Fault category
    pub category: FaultCategory,
    /// Human-readable details (platform-specific)
    pub details: String<128>,
}

impl FaultInfo {
    /// Create a new fault info with category and details
    pub fn new(category: FaultCategory, details: &str) -> Self {
        let mut s = String::new();
        let _ = s.push_str(details);
        Self {
            category,
            details: s,
        }
    }

    /// Create a fault info with just a category (no details)
    pub fn from_category(category: FaultCategory) -> Self {
        Self {
            category,
            details: String::new(),
        }
    }
}

// ============================================================================
// Platform Fault Trait
// ============================================================================

/// Platform-specific fault type
///
/// Platforms implement this trait for their fault enum, which can contain
/// associated data (e.g., `DrvFault(FaultStatus)`).
///
/// # Example
///
/// ```ignore
/// impl PlatformFault for MyPlatformFault {
///     fn category(&self) -> FaultCategory {
///         match self {
///             MyPlatformFault::OverCurrent => FaultCategory::OverCurrent,
///             MyPlatformFault::DrvFault(_) => FaultCategory::DriverFault,
///             // ...
///         }
///     }
///
///     fn details(&self) -> String<128> {
///         match self {
///             MyPlatformFault::DrvFault(status) => {
///                 let mut s = String::new();
///                 if status.fetha_oc { s.push_str("PhA_OC "); }
///                 if status.otsd { s.push_str("ThermalShutdown "); }
///                 // ...
///                 s
///             }
///             _ => String::new(),
///         }
///     }
///
///     fn is_recoverable(&self) -> bool {
///         matches!(self, MyPlatformFault::UnderVoltage)
///     }
///
///     fn is_critical(&self) -> bool {
///         matches!(self, MyPlatformFault::OverCurrent | MyPlatformFault::DrvFault(_))
///     }
/// }
/// ```
pub trait PlatformFault: Copy + Clone + PartialEq {
    /// Get the protocol-level fault category
    fn category(&self) -> FaultCategory;

    /// Get human-readable details about this fault
    ///
    /// For driver faults, this might include specific flags like
    /// "PhaseA_OC ThermalWarning GVDD_UV"
    fn details(&self) -> String<128>;

    /// Convert to protocol fault info
    fn to_fault_info(&self) -> FaultInfo {
        FaultInfo {
            category: self.category(),
            details: self.details(),
        }
    }

    /// Returns true if this fault can auto-clear when condition resolves
    fn is_recoverable(&self) -> bool;

    /// Returns true if this fault requires immediate motor shutdown
    fn is_critical(&self) -> bool;
}

// ============================================================================
// Fault Registry (requires runtime feature)
// ============================================================================

/// Maximum number of simultaneous faults
#[cfg(feature = "runtime")]
pub const MAX_FAULTS: usize = 16;

/// Fault registry that stores platform-specific faults with their data
///
/// This is ISR-safe and can be used from both interrupts and tasks.
/// Faults are stored in a heapless Vec with their associated data.
#[cfg(feature = "runtime")]
pub struct FaultRegistry<F: PlatformFault> {
    /// Active faults (protected by critical section)
    faults: CriticalSectionMutex<RefCell<Vec<F, MAX_FAULTS>>>,
    /// Signal to wake tasks when faults change
    changed: Signal<CriticalSectionRawMutex, ()>,
}

#[cfg(feature = "runtime")]
impl<F: PlatformFault> FaultRegistry<F> {
    /// Create a new fault registry
    pub const fn new() -> Self {
        Self {
            faults: CriticalSectionMutex::new(RefCell::new(Vec::new())),
            changed: Signal::new(),
        }
    }

    /// Set a fault (adds if not already present, or updates if category matches)
    ///
    /// Returns true if the fault was newly added (not already present).
    pub fn set(&self, fault: F) -> bool {
        let newly_added = self.faults.lock(|cell| {
            let mut faults = cell.borrow_mut();
            let cat = fault.category();

            // Check if this fault category is already present
            if let Some(pos) = faults.iter().position(|f| f.category() == cat) {
                // Update existing fault with new data
                faults[pos] = fault;
                false
            } else {
                // Add new fault
                faults.push(fault).is_ok()
            }
        });

        if newly_added {
            self.changed.signal(());
        }

        newly_added
    }

    /// Clear a specific fault by category
    pub fn clear(&self, category: FaultCategory) {
        let removed = self.faults.lock(|cell| {
            let mut faults = cell.borrow_mut();
            if let Some(pos) = faults.iter().position(|f| f.category() == category) {
                faults.swap_remove(pos);
                true
            } else {
                false
            }
        });

        if removed {
            self.changed.signal(());
        }
    }

    /// Clear a specific fault by value
    pub fn clear_fault(&self, fault: &F) {
        self.clear(fault.category());
    }

    /// Clear all faults
    pub fn clear_all(&self) {
        let had_faults = self.faults.lock(|cell| {
            let mut faults = cell.borrow_mut();
            let had = !faults.is_empty();
            faults.clear();
            had
        });

        if had_faults {
            self.changed.signal(());
        }
    }

    /// Returns true if any fault is active
    pub fn any(&self) -> bool {
        self.faults.lock(|cell| !cell.borrow().is_empty())
    }

    /// Returns true if any critical fault is active
    pub fn any_critical(&self) -> bool {
        self.faults
            .lock(|cell| cell.borrow().iter().any(|f| f.is_critical()))
    }

    /// Check if a specific fault category is active
    pub fn has_category(&self, category: FaultCategory) -> bool {
        self.faults
            .lock(|cell| cell.borrow().iter().any(|f| f.category() == category))
    }

    /// Get a fault by category
    pub fn get_by_category(&self, category: FaultCategory) -> Option<F> {
        self.faults.lock(|cell| {
            cell.borrow()
                .iter()
                .find(|f| f.category() == category)
                .copied()
        })
    }

    /// Get all faults as FaultInfo for protocol transmission
    pub fn to_fault_info_vec(&self) -> Vec<FaultInfo, MAX_FAULTS> {
        self.faults
            .lock(|cell| cell.borrow().iter().map(|f| f.to_fault_info()).collect())
    }

    /// Get count of active faults
    pub fn count(&self) -> usize {
        self.faults.lock(|cell| cell.borrow().len())
    }

    /// Get a snapshot of all active faults
    pub fn snapshot(&self) -> Vec<F, MAX_FAULTS> {
        self.faults.lock(|cell| cell.borrow().clone())
    }

    /// Get the first fault (if any)
    pub fn first(&self) -> Option<F> {
        self.faults.lock(|cell| cell.borrow().first().copied())
    }

    /// Wait for fault state to change
    pub async fn wait_for_change(&self) {
        self.changed.wait().await;
    }

    /// Auto-clear all recoverable faults
    ///
    /// Call this when the fault condition is no longer present
    /// (e.g., voltage back in range).
    pub fn auto_clear_recoverable(&self) {
        let changed = self.faults.lock(|cell| {
            let mut faults = cell.borrow_mut();
            let before = faults.len();
            faults.retain(|f| !f.is_recoverable());
            faults.len() != before
        });

        if changed {
            self.changed.signal(());
        }
    }

    /// Execute a closure with access to the fault list
    ///
    /// Useful for complex operations that need to inspect multiple faults.
    pub fn with_faults<R>(&self, f: impl FnOnce(&Vec<F, MAX_FAULTS>) -> R) -> R {
        self.faults.lock(|cell| f(&cell.borrow()))
    }
}

#[cfg(feature = "runtime")]
impl<F: PlatformFault> Default for FaultRegistry<F> {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Hysteresis constants for auto-recovery
// ============================================================================

/// Voltage hysteresis in millivolts.
/// Undervoltage clears when Vbus > min_vbus_mv + VOLTAGE_HYSTERESIS_MV
pub const VOLTAGE_HYSTERESIS_MV: u32 = 500;
