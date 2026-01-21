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
}

impl PlatformFault for G474Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G474Fault::OverCurrent => FaultCategory::OverCurrent,
            G474Fault::OverVoltage => FaultCategory::OverVoltage,
            G474Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G474Fault::OverTemp => FaultCategory::OverTemp,
            G474Fault::HallError => FaultCategory::HallError,
        }
    }

    fn details(&self) -> String<128> {
        // G474 faults don't have additional details
        String::new()
    }

    fn is_recoverable(&self) -> bool {
        matches!(self, G474Fault::UnderVoltage)
    }

    fn is_critical(&self) -> bool {
        matches!(
            self,
            G474Fault::OverCurrent | G474Fault::OverVoltage | G474Fault::OverTemp
        )
    }
}
