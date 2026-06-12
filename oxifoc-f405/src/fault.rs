//! Platform-specific fault types for Simple FOCer 2 (STM32F405 + DRV8301)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};
use oxifoc_core::foc::hall_sensor::HallFaultKind;

// Import DRV8301 fault status from driver crate
pub use drv8301_dd::FaultStatus as DrvFaultStatus;

/// F405 platform-specific faults
///
/// These faults can contain associated data from the hardware, such as
/// detailed DRV8301 fault status bits.
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum F405Fault {
    /// Over-current detected (from software or DRV8301)
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET)
    OverTemp,
    /// DRV8301 gate driver fault with detailed status
    DrvFault(DrvFaultStatus),
    /// Hall sensor error (warning class: the ride continues on the
    /// fallback chain; the payload names the degradation, e.g. which wire)
    HallError(HallFaultKind),
    /// Command link stale while running (deadman / link-loss)
    CommTimeout,
    /// Graduated derating active (power rolloff > 20%) — warning class
    Derating,
}

impl PlatformFault for F405Fault {
    fn category(&self) -> FaultCategory {
        match self {
            Self::OverCurrent => FaultCategory::OverCurrent,
            Self::OverVoltage => FaultCategory::OverVoltage,
            Self::UnderVoltage => FaultCategory::UnderVoltage,
            Self::OverTemp => FaultCategory::OverTemp,
            Self::DrvFault(_) => FaultCategory::DriverFault,
            Self::HallError(_) => FaultCategory::HallError,
            Self::CommTimeout => FaultCategory::CommTimeout,
            Self::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        let mut s = String::new();
        match self {
            Self::DrvFault(status) => {
                // Build a string with all the DRV8301 fault flags
                if status.fetha_oc {
                    let _ = s.push_str("PhA_H_OC ");
                }
                if status.fetla_oc {
                    let _ = s.push_str("PhA_L_OC ");
                }
                if status.fethb_oc {
                    let _ = s.push_str("PhB_H_OC ");
                }
                if status.fetlb_oc {
                    let _ = s.push_str("PhB_L_OC ");
                }
                if status.fethc_oc {
                    let _ = s.push_str("PhC_H_OC ");
                }
                if status.fetlc_oc {
                    let _ = s.push_str("PhC_L_OC ");
                }
                if status.otsd {
                    let _ = s.push_str("ThermalShutdown ");
                }
                if status.otw {
                    let _ = s.push_str("ThermalWarn ");
                }
                if status.gvdd_uv {
                    let _ = s.push_str("GVDD_UV ");
                }
                if status.gvdd_ov {
                    let _ = s.push_str("GVDD_OV ");
                }
                if status.pvdd_uv {
                    let _ = s.push_str("PVDD_UV ");
                }
            }
            Self::HallError(kind) => {
                return kind.details();
            }
            _ => {
                // No additional details for other faults
            }
        }
        s
    }

    // severity(): central per-category policy (FaultCategory::severity).
    // OverTemp is GracefulStop, not Kill — "must not restart while hot"
    // survives via the any_stopping() start gate (no auto-clear on OT).

    fn from_hall_kind(kind: HallFaultKind) -> Option<Self> {
        Some(Self::HallError(kind))
    }

    /// Payload-free categories the shared core protection can raise.
    // DriverFault carries the DRV status payload — raised by the
    // platform DRV handler, not through this constructor.
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
