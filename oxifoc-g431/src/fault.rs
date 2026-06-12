//! Platform-specific fault types for B-G431B-ESC1 (STM32G431)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};
use oxifoc_core::foc::hall_sensor::HallFaultKind;

/// G431 platform-specific faults
///
/// The B-G431B-ESC1 doesn't have an external gate driver like DRV8301,
/// so faults are simpler - just the basic categories without associated data.
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum G431Fault {
    /// Over-current detected
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET)
    OverTemp,
    /// Hall sensor error (warning class: the ride continues on the
    /// fallback chain; the payload names the degradation, e.g. which wire)
    HallError(HallFaultKind),
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
    /// Graduated derating active (power rolloff > 20%) — warning class
    Derating,
}

impl PlatformFault for G431Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G431Fault::OverCurrent => FaultCategory::OverCurrent,
            G431Fault::OverVoltage => FaultCategory::OverVoltage,
            G431Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G431Fault::OverTemp => FaultCategory::OverTemp,
            G431Fault::HallError(_) => FaultCategory::HallError,
            G431Fault::CommTimeout => FaultCategory::CommTimeout,
            G431Fault::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        match self {
            G431Fault::HallError(kind) => kind.details(),
            _ => String::new(),
        }
    }

    fn is_recoverable(&self) -> bool {
        // UnderVoltage clears via the voltage hysteresis check; CommTimeout
        // clears in run_foc_cycle when commands flow again.
        matches!(
            self,
            G431Fault::UnderVoltage | G431Fault::CommTimeout | G431Fault::Derating
        )
    }

    // severity(): central per-category policy (FaultCategory::severity).

    fn from_hall_kind(kind: HallFaultKind) -> Option<Self> {
        Some(G431Fault::HallError(kind))
    }

    /// Payload-free categories the shared core protection can raise.
    fn from_category(category: FaultCategory) -> Option<Self> {
        match category {
            FaultCategory::OverCurrent => Some(G431Fault::OverCurrent),
            FaultCategory::OverVoltage => Some(G431Fault::OverVoltage),
            FaultCategory::UnderVoltage => Some(G431Fault::UnderVoltage),
            FaultCategory::OverTemp => Some(G431Fault::OverTemp),
            FaultCategory::CommTimeout => Some(G431Fault::CommTimeout),
            FaultCategory::Derating => Some(G431Fault::Derating),
            _ => None,
        }
    }
}
