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

    fn is_recoverable(&self) -> bool {
        // UnderVoltage clears via the voltage hysteresis check; CommTimeout
        // clears in run_foc_cycle when commands flow again.
        matches!(
            self,
            Self::UnderVoltage | Self::CommTimeout | Self::Derating
        )
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
