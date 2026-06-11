//! Platform-specific fault types for Simple FOCer 2 (STM32F405 + DRV8301)

use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};

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
    /// Hall sensor error
    HallError,
}

impl PlatformFault for F405Fault {
    fn category(&self) -> FaultCategory {
        match self {
            F405Fault::OverCurrent => FaultCategory::OverCurrent,
            F405Fault::OverVoltage => FaultCategory::OverVoltage,
            F405Fault::UnderVoltage => FaultCategory::UnderVoltage,
            F405Fault::OverTemp => FaultCategory::OverTemp,
            F405Fault::DrvFault(_) => FaultCategory::DriverFault,
            F405Fault::HallError => FaultCategory::HallError,
        }
    }

    fn details(&self) -> String<128> {
        let mut s = String::new();
        match self {
            F405Fault::DrvFault(status) => {
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
            _ => {
                // No additional details for other faults
            }
        }
        s
    }

    fn is_recoverable(&self) -> bool {
        matches!(self, F405Fault::UnderVoltage)
    }

    fn is_critical(&self) -> bool {
        // OverTemp is critical like on G431/G474: an overheated board must
        // not be restartable by a plain host command while still hot.
        matches!(
            self,
            F405Fault::OverCurrent
                | F405Fault::OverVoltage
                | F405Fault::OverTemp
                | F405Fault::DrvFault(_)
        )
    }
}
