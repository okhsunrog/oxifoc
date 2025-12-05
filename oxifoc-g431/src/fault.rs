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
}

impl PlatformFault for G431Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G431Fault::OverCurrent => FaultCategory::OverCurrent,
            G431Fault::OverVoltage => FaultCategory::OverVoltage,
            G431Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G431Fault::OverTemp => FaultCategory::OverTemp,
            G431Fault::HallError => FaultCategory::HallError,
        }
    }

    fn details(&self) -> String<128> {
        // G431 faults don't have additional details
        String::new()
    }

    fn is_recoverable(&self) -> bool {
        matches!(self, G431Fault::UnderVoltage)
    }

    fn is_critical(&self) -> bool {
        matches!(
            self,
            G431Fault::OverCurrent | G431Fault::OverVoltage | G431Fault::OverTemp
        )
    }
}
