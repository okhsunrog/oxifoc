use heapless::String;
use oxifoc_core::foc::fault::{FaultCategory, PlatformFault};

/// Minimal fault type for the virtual device.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum VirtualFault {
    OverCurrent,
    OverVoltage,
    UnderVoltage,
    OverTemp,
    CommTimeout,
    Derating,
}

impl PlatformFault for VirtualFault {
    fn category(&self) -> FaultCategory {
        match self {
            Self::OverCurrent => FaultCategory::OverCurrent,
            Self::OverVoltage => FaultCategory::OverVoltage,
            Self::UnderVoltage => FaultCategory::UnderVoltage,
            Self::OverTemp => FaultCategory::OverTemp,
            Self::CommTimeout => FaultCategory::CommTimeout,
            Self::Derating => FaultCategory::Derating,
        }
    }

    fn details(&self) -> String<128> {
        String::new()
    }

    fn is_recoverable(&self) -> bool {
        true
    }

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
