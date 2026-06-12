//! Platform-specific fault types for NUCLEO-G474RE (STM32G474)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};

/// G474 platform-specific faults
///
/// When the IHM08M1 shield is connected, these faults will be triggered
/// by the motor control subsystem based on current, voltage, and temperature
/// measurements.
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum G474Fault {
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

impl PlatformFault for G474Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G474Fault::OverCurrent => FaultCategory::OverCurrent,
            G474Fault::OverVoltage => FaultCategory::OverVoltage,
            G474Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G474Fault::OverTemp => FaultCategory::OverTemp,
            G474Fault::HallError => FaultCategory::HallError,
            G474Fault::CommTimeout => FaultCategory::CommTimeout,
        }
    }

    fn details(&self) -> String<128> {
        // G474 faults don't have additional details
        String::new()
    }

    fn is_recoverable(&self) -> bool {
        // UnderVoltage clears via the voltage hysteresis check; CommTimeout
        // clears in run_foc_cycle when commands flow again.
        matches!(self, G474Fault::UnderVoltage | G474Fault::CommTimeout)
    }

    // severity(): central per-category policy (FaultCategory::severity).
}
