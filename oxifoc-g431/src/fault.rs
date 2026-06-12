//! Platform-specific fault types for B-G431B-ESC1 (STM32G431)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};

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
    /// Hall sensor error
    HallError,
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
}

impl PlatformFault for G431Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G431Fault::OverCurrent => FaultCategory::OverCurrent,
            G431Fault::OverVoltage => FaultCategory::OverVoltage,
            G431Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G431Fault::OverTemp => FaultCategory::OverTemp,
            G431Fault::HallError => FaultCategory::HallError,
            G431Fault::CommTimeout => FaultCategory::CommTimeout,
        }
    }

    fn details(&self) -> String<128> {
        // G431 faults don't have additional details
        String::new()
    }

    fn is_recoverable(&self) -> bool {
        // UnderVoltage clears via the voltage hysteresis check; CommTimeout
        // clears in run_foc_cycle when commands flow again.
        matches!(self, G431Fault::UnderVoltage | G431Fault::CommTimeout)
    }

    // severity(): central per-category policy (FaultCategory::severity).
}
