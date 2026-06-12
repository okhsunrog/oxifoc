//! Platform-specific fault types for NUCLEO-G474RE (STM32G474)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};
use oxifoc_core::foc::hall_sensor::HallFaultKind;

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
    /// Hall sensor error (warning class: the ride continues on the
    /// fallback chain; the payload names the degradation, e.g. which wire)
    HallError(HallFaultKind),
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
    /// Graduated derating active (power rolloff > 20%) — warning class
    Derating,
}

impl PlatformFault for G474Fault {
    fn category(&self) -> FaultCategory {
        match self {
            G474Fault::OverCurrent => FaultCategory::OverCurrent,
            G474Fault::OverVoltage => FaultCategory::OverVoltage,
            G474Fault::UnderVoltage => FaultCategory::UnderVoltage,
            G474Fault::OverTemp => FaultCategory::OverTemp,
            G474Fault::HallError(_) => FaultCategory::HallError,
            G474Fault::CommTimeout => FaultCategory::CommTimeout,
            G474Fault::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        match self {
            G474Fault::HallError(kind) => kind.details(),
            _ => String::new(),
        }
    }

    fn is_recoverable(&self) -> bool {
        // UnderVoltage clears via the voltage hysteresis check; CommTimeout
        // clears in run_foc_cycle when commands flow again.
        matches!(
            self,
            G474Fault::UnderVoltage | G474Fault::CommTimeout | G474Fault::Derating
        )
    }

    // severity(): central per-category policy (FaultCategory::severity).

    fn from_hall_kind(kind: HallFaultKind) -> Option<Self> {
        Some(G474Fault::HallError(kind))
    }

    /// Payload-free categories the shared core protection can raise.
    fn from_category(category: FaultCategory) -> Option<Self> {
        match category {
            FaultCategory::OverCurrent => Some(G474Fault::OverCurrent),
            FaultCategory::OverVoltage => Some(G474Fault::OverVoltage),
            FaultCategory::UnderVoltage => Some(G474Fault::UnderVoltage),
            FaultCategory::OverTemp => Some(G474Fault::OverTemp),
            FaultCategory::CommTimeout => Some(G474Fault::CommTimeout),
            FaultCategory::Derating => Some(G474Fault::Derating),
            _ => None,
        }
    }
}
