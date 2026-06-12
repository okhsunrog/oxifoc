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

use crate::foc::hall_sensor::HallFaultKind;
#[cfg(feature = "runtime")]
use crate::types::{FaultResponse, MAX_FAULT_RESPONSE};
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
    // postcard encodes the variant index — append only.
    /// Graduated derating active (power rolloff > 20%)
    Derating,
}

/// What the firmware does about a fault — see `docs/notes/fault-overhaul.md`.
///
/// Ordered: `Warning < GracefulStop < Kill`, so severity comparisons read
/// naturally (`severity() >= GracefulStop` = "stops the motor").
///
/// The remote/host UI keys off this field, never off hardcoded categories —
/// new fault categories must not require a remote firmware update.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, Schema, Default,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultSeverity {
    /// Report only — the motor is untouched. The rider keeps riding.
    Warning,
    /// The inverter is healthy but continuing is unsafe: stop via the
    /// failsafe machinery (ramp / controlled stop per config), restart
    /// blocked while the fault is active. No Error latch — when the
    /// condition clears (auto or host clear), the normal failsafe re-arm
    /// applies (explicit safe-mode acknowledgement).
    GracefulStop,
    /// Inverter-integrity threat: immediate high-Z + Error latch until an
    /// explicit host clear. Default — an unclassified fault is treated as
    /// the worst case.
    #[default]
    Kill,
}

impl FaultCategory {
    /// Central severity policy (one place, shared by every board) — the
    /// rationale table lives in `docs/notes/fault-overhaul.md`. Platforms
    /// can override per-fault via [`PlatformFault::severity`] but should
    /// have a concrete reason to diverge.
    pub fn severity(&self) -> FaultSeverity {
        match self {
            // Inverter integrity — no choice but high-Z.
            Self::OverCurrent | Self::OverVoltage | Self::DriverFault => FaultSeverity::Kill,
            // Inverter healthy, continuing unsafe: stop gracefully. The
            // "must not restart while still hot/sagging" property survives
            // the downgrade from Kill — the start gate blocks while any
            // stopping-class fault is active, and OverTemp has no
            // auto-clear.
            Self::OverTemp | Self::UnderVoltage | Self::CommTimeout | Self::Stall => {
                FaultSeverity::GracefulStop
            }
            // Degradations the vehicle rides through (fallback paths carry
            // commutation); the rider gets informed, nothing else.
            Self::HallError | Self::CalibrationFault | Self::Derating | Self::None => {
                FaultSeverity::Warning
            }
        }
    }
}

/// Fault information for protocol transmission
///
/// Contains a category (fixed enum), the response class, plus a
/// human-readable detail string that platforms can populate with specific
/// information.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultInfo {
    /// Fault category
    pub category: FaultCategory,
    /// What the firmware did about it (drives the remote's vibration/UI)
    pub severity: FaultSeverity,
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
            severity: category.severity(),
            details: s,
        }
    }

    /// Create a fault info with just a category (no details)
    pub fn from_category(category: FaultCategory) -> Self {
        Self {
            category,
            severity: category.severity(),
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
///     // severity() comes from the category by default.
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
            severity: self.severity(),
            details: self.details(),
        }
    }

    /// Response class. Defaults to the central per-category policy
    /// ([`FaultCategory::severity`]); override only with a concrete reason
    /// (e.g. a driver fault whose status bits say "warning only").
    fn severity(&self) -> FaultSeverity {
        self.category().severity()
    }

    /// Construct the platform's hall fault from a degradation kind, for
    /// the sticky `HallError` warning bridge in `run_foc_cycle`. Default
    /// `None` = the platform carries no hall fault (the bridge is a no-op)
    /// — so the bridge needs no extra `run_foc_cycle` parameter.
    fn from_hall_kind(_kind: HallFaultKind) -> Option<Self> {
        None
    }

    /// Construct the platform fault for a payload-free category — how the
    /// shared protection code in core (`run_foc_cycle` / `run_protection`)
    /// raises faults without per-category value parameters. Implement for
    /// every category the board can experience; returning `None` makes
    /// core silently skip raising that category. Payload-carrying faults
    /// (DRV status, hall kind) keep their dedicated constructors.
    fn from_category(_category: FaultCategory) -> Option<Self> {
        None
    }
}

/// Ready-made [`PlatformFault`] for platforms with no extra hardware
/// diagnostics beyond the shared categories (no DRV8301-style gate-driver
/// status to carry). G431, G474 and the virtual device use it directly;
/// F405 keeps its own enum for the DRV8301 status payload.
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StandardFault {
    /// Over-current detected
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET or motor NTC)
    OverTemp,
    /// Hall sensor error (warning class: the ride continues on the
    /// fallback chain; the payload names the degradation, e.g. which wire)
    HallError(HallFaultKind),
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
    /// Graduated derating active (power rolloff > 20%) — warning class
    Derating,
}

impl PlatformFault for StandardFault {
    fn category(&self) -> FaultCategory {
        match self {
            Self::OverCurrent => FaultCategory::OverCurrent,
            Self::OverVoltage => FaultCategory::OverVoltage,
            Self::UnderVoltage => FaultCategory::UnderVoltage,
            Self::OverTemp => FaultCategory::OverTemp,
            Self::HallError(_) => FaultCategory::HallError,
            Self::CommTimeout => FaultCategory::CommTimeout,
            Self::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        match self {
            Self::HallError(kind) => kind.details(),
            _ => String::new(),
        }
    }

    // severity(): central per-category policy (FaultCategory::severity).

    fn from_hall_kind(kind: HallFaultKind) -> Option<Self> {
        Some(Self::HallError(kind))
    }

    /// Payload-free categories the shared core protection can raise.
    fn from_category(category: FaultCategory) -> Option<Self> {
        match category {
            FaultCategory::OverCurrent => Some(Self::OverCurrent),
            FaultCategory::OverVoltage => Some(Self::OverVoltage),
            FaultCategory::UnderVoltage => Some(Self::UnderVoltage),
            FaultCategory::OverTemp => Some(Self::OverTemp),
            FaultCategory::CommTimeout => Some(Self::CommTimeout),
            FaultCategory::Derating => Some(Self::Derating),
            _ => None,
        }
    }
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
    /// Signals `changed` on addition AND on a payload change of an existing
    /// entry (e.g. a sticky HallError refining `InvalidState` →
    /// `WireDead{H2}`) — the fault topic must push the refined details.
    /// Re-setting an identical value stays silent, so a detector calling
    /// `set()` every ISR cycle cannot spam the topic.
    pub fn set(&self, fault: F) -> bool {
        let (newly_added, value_changed) = self.faults.lock(|cell| {
            let mut faults = cell.borrow_mut();
            let cat = fault.category();

            // Check if this fault category is already present
            if let Some(pos) = faults.iter().position(|f| f.category() == cat) {
                // Update existing fault with new data
                let changed = faults[pos] != fault;
                faults[pos] = fault;
                (false, changed)
            } else {
                // Add new fault
                (faults.push(fault).is_ok(), false)
            }
        });

        if newly_added || value_changed {
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

    /// Returns true if any Kill-class fault is active (immediate high-Z)
    pub fn any_kill(&self) -> bool {
        self.faults.lock(|cell| {
            cell.borrow()
                .iter()
                .any(|f| f.severity() == FaultSeverity::Kill)
        })
    }

    /// Returns true if any fault that stops the motor is active
    /// (GracefulStop or Kill) — the start gate blocks on this; warnings
    /// never block.
    pub fn any_stopping(&self) -> bool {
        self.faults.lock(|cell| {
            cell.borrow()
                .iter()
                .any(|f| f.severity() >= FaultSeverity::GracefulStop)
        })
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
        self.faults.lock(|cell| {
            cell.borrow()
                .iter()
                .map(PlatformFault::to_fault_info)
                .collect()
        })
    }

    /// Protocol snapshot (`FaultResponse`): at most `MAX_FAULT_RESPONSE`
    /// entries plus the true total so the consumer can see truncation.
    /// Shared by the fault endpoint server and the fault topic publisher —
    /// both must serialize the registry identically.
    pub fn snapshot_response(&self) -> FaultResponse {
        let infos = self.to_fault_info_vec();
        let total = infos.len() as u8;
        let mut faults = Vec::new();
        for info in infos.iter().take(MAX_FAULT_RESPONSE) {
            let _ = faults.push(info.clone());
        }
        FaultResponse { faults, total }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The central severity policy — the table in
    /// docs/notes/fault-overhaul.md. A new category falling through to a
    /// wrong class is a safety bug, so pin every one.
    #[test]
    fn severity_policy_pinned() {
        use FaultSeverity::*;
        let expected = [
            (FaultCategory::None, Warning),
            (FaultCategory::OverCurrent, Kill),
            (FaultCategory::OverVoltage, Kill),
            (FaultCategory::UnderVoltage, GracefulStop),
            (FaultCategory::OverTemp, GracefulStop),
            (FaultCategory::DriverFault, Kill),
            (FaultCategory::HallError, Warning),
            (FaultCategory::Stall, GracefulStop),
            (FaultCategory::CalibrationFault, Warning),
            (FaultCategory::CommTimeout, GracefulStop),
        ];
        for (cat, sev) in expected {
            assert_eq!(cat.severity(), sev, "{cat:?}");
        }
        // Ordering carries meaning: ">= GracefulStop" must read "stops the
        // motor", and the conservative default is the worst case.
        assert!(Warning < GracefulStop && GracefulStop < Kill);
        assert_eq!(FaultSeverity::default(), Kill);
    }

    #[test]
    fn fault_info_carries_severity() {
        let info = FaultInfo::from_category(FaultCategory::HallError);
        assert_eq!(info.severity, FaultSeverity::Warning);
        let info = FaultInfo::new(FaultCategory::OverCurrent, "test");
        assert_eq!(info.severity, FaultSeverity::Kill);
    }

    /// The fault topic wakes on `changed`: it must fire on a NEW fault and
    /// on a payload refinement of an existing one (sticky HallError
    /// upgrading `InvalidState` → `WireDead`), but a detector re-setting
    /// the identical value every ISR cycle must stay silent — otherwise
    /// the topic spams the link at 20 kHz.
    #[test]
    #[cfg(feature = "runtime")]
    fn set_signals_on_add_and_refinement_only() {
        #[derive(Clone, Copy, PartialEq)]
        struct TF(u8);
        impl PlatformFault for TF {
            fn category(&self) -> FaultCategory {
                FaultCategory::HallError
            }
            fn details(&self) -> String<128> {
                String::new()
            }
        }

        let reg: FaultRegistry<TF> = FaultRegistry::new();
        assert!(!reg.changed.signaled());

        reg.set(TF(1));
        assert!(reg.changed.signaled(), "new fault must signal");
        reg.changed.reset();

        reg.set(TF(1));
        assert!(!reg.changed.signaled(), "identical re-set must stay silent");

        reg.set(TF(2));
        assert!(reg.changed.signaled(), "payload refinement must signal");
        reg.changed.reset();

        reg.clear(FaultCategory::HallError);
        assert!(reg.changed.signaled(), "clearing must signal");
    }
}
