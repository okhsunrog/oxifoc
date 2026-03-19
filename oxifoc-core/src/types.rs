//! Shared types for motor control ICD (Interface Control Document)
//!
//! This module contains types that are shared between embedded firmware and
//! host applications. All types here are serializable via serde/postcard.
//!
//! # Feature: `types`
//!
//! This module is only available when the `types` feature is enabled.
//! Host applications that only need type definitions can use:
//! ```toml
//! oxifoc-core = { version = "0.1", default-features = false, features = ["types"] }
//! ```

use heapless::String;
use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// Re-export Direction from hall_sensor (it has conditional serde derives)
pub use crate::foc::hall_sensor::Direction;

// Re-export fault types for protocol use
pub use crate::foc::fault::{FaultCategory, FaultInfo};

// ============================================================================
// Motor State Types
// ============================================================================

/// Motor operational state
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorState {
    /// Motor is stopped (PWM disabled or zero duty)
    #[default]
    Stopped,
    /// Motor is running (FOC active)
    Running,
    /// Motor is in error state (fault detected)
    Error,
}

/// Control mode for the motor
///
/// This is the unified control mode used by both the protocol layer and the FOC driver.
/// Send this directly via the command channel to control the motor.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlMode {
    /// Motor stopped, PWM disabled
    #[default]
    Stopped,
    /// Current control mode (torque control)
    CurrentControl {
        /// Target q-axis current (torque) in Amps
        iq_target: f32,
        /// Target d-axis current (field weakening) in Amps
        id_target: f32,
    },
    /// Velocity control mode
    VelocityControl {
        /// Target velocity in rad/s
        target_vel: f32,
    },
    /// Position control mode
    PositionControl {
        /// Target position in radians
        target_pos: f32,
    },
    /// Open-loop mode for calibration - locks rotor to specified electrical angle
    ///
    /// Uses commanded angle instead of sensor feedback. Current control still
    /// runs to regulate the applied current. Used for Hall sensor calibration.
    OpenLoop {
        /// Target electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps) - applied as q-current to lock rotor
        current: f32,
    },
    /// Direct voltage mode — apply dq voltages without PI control.
    ///
    /// Bypasses current regulation entirely. Used for measurement modes
    /// (HFI inductance detection), calibration, and board bringup where
    /// PI interference is undesirable.
    DirectVoltage {
        /// d-axis voltage (V)
        vd: f32,
        /// q-axis voltage (V)
        vq: f32,
        /// Electrical angle (radians)
        angle_rad: f32,
    },
    /// Six-step (trapezoidal) commutation mode
    ///
    /// Simple voltage-mode drive for board bringup and testing.
    /// Does not require current sensor calibration.
    /// Sign of duty determines direction: positive = forward,
    /// negative = reverse.
    SixStep {
        /// Duty cycle (-1.0 to 1.0)
        duty: f32,
    },
}

// ============================================================================
// Status/Telemetry Types
// ============================================================================

/// Motor status response
///
/// Note: Fault information is now platform-specific. Use the platform's
/// fault endpoint to get detailed fault information.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorStatus {
    /// Current motor state
    pub state: MotorState,
    /// Current control mode
    pub mode: ControlMode,
    /// Number of active faults
    pub fault_count: u8,
}

/// Hall sensor telemetry data
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallSensorData {
    /// Electrical angle in radians (0 to 2π)
    pub angle_rad: f32,
    /// Direction of rotation
    pub direction: Direction,
    /// Raw Hall state (0-7, where 0 and 7 are invalid)
    pub state: u8,
    /// Cumulative error count (invalid states or transitions)
    pub error_count: u32,
    /// Monotonic sequence number (wraps on overflow)
    pub seq: u32,
}

/// Raw ADC sample data
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcSample {
    /// Phase A current (raw ADC counts)
    pub ia: u16,
    /// Phase B current (raw ADC counts)
    pub ib: u16,
    /// Phase C current (raw ADC counts)
    pub ic: u16,
    /// DC bus voltage in millivolts
    pub vbus_mv: u32,
    /// FET temperature in 0.1°C units
    pub fet_temp_c_x10: u16,
    /// Monotonic sequence number
    pub seq: u32,
}

/// Basic device information
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct DeviceInfo {
    /// Hardware identifier (e.g., "B-G431B-ESC1")
    pub hw: String<32>,
    /// Software version (e.g., "oxifoc-0.1.0")
    pub sw: String<32>,
}

impl Default for DeviceInfo {
    fn default() -> Self {
        Self {
            hw: String::new(),
            sw: String::new(),
        }
    }
}

/// Button events from hardware
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ButtonEvent {
    /// Single button click
    SingleClick,
    /// Double click detected
    DoubleClick,
    /// Button held down
    Hold,
}

// ============================================================================
// Fault Protocol Types
// ============================================================================

/// Maximum number of faults that can be returned in a response
pub const MAX_FAULT_RESPONSE: usize = 8;

/// Fault management request
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultRequest {
    /// Query all active faults
    #[default]
    Query,
    /// Clear a specific fault by category
    Clear(FaultCategory),
    /// Clear all faults
    ClearAll,
}

/// Fault management response
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultResponse {
    /// List of active faults with details
    pub faults: Vec<FaultInfo, MAX_FAULT_RESPONSE>,
}

// ============================================================================
// Configuration Protocol Types (require storage feature)
// ============================================================================

#[cfg(feature = "storage")]
pub use config_types::*;

#[cfg(feature = "storage")]
mod config_types {
    use super::*;

    /// Configuration request from host
    #[derive(Clone, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigRequest {
        /// Read a config group (returns current value or defaults)
        Read(ConfigGroupId),
        /// Write a config group to flash
        Write(ConfigWrite),
        /// Reset all config to defaults (erase flash)
        ResetAll,
    }

    /// Config group identifier for read requests
    #[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigGroupId {
        MotorParams,
        HallCalibration,
        DcOffsets,
        CurrentLimits,
        VoltageLimits,
        PwmConfig,
        PiGains,
        HallTuning,
    }

    /// Config write payload — one variant per group
    #[derive(Clone, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigWrite {
        MotorParams(crate::storage::MotorParamsConfig),
        CurrentLimits(crate::storage::CurrentLimitsConfig),
        VoltageLimits(crate::storage::VoltageLimitsConfig),
        PwmConfig(crate::storage::PwmConfigStored),
        PiGains(crate::storage::PiGainsConfig),
        HallTuning(crate::storage::HallTuningConfig),
    }

    /// Configuration response
    #[derive(Clone, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigResponse {
        /// Operation succeeded
        Ok,
        /// Motor parameters
        MotorParams(crate::storage::MotorParamsConfig),
        /// Current limits
        CurrentLimits(crate::storage::CurrentLimitsConfig),
        /// Voltage limits
        VoltageLimits(crate::storage::VoltageLimitsConfig),
        /// PWM configuration
        PwmConfig(crate::storage::PwmConfigStored),
        /// PI gains
        PiGains(crate::storage::PiGainsConfig),
        /// Hall tuning
        HallTuning(crate::storage::HallTuningConfig),
        /// Hall calibration data
        HallCalibration(crate::storage::HallCalibrationConfig),
        /// DC offsets
        DcOffsets(crate::storage::DcOffsetsConfig),
        /// Requested group has no stored value
        NotFound,
        /// Flash operation failed
        Error,
    }
}

// ============================================================================
// Conversion helpers
// ============================================================================

impl MotorState {
    /// Convert from u8 (for atomic storage)
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => MotorState::Stopped,
            1 => MotorState::Running,
            _ => MotorState::Error,
        }
    }

    /// Convert to u8 (for atomic storage)
    pub fn to_u8(self) -> u8 {
        match self {
            MotorState::Stopped => 0,
            MotorState::Running => 1,
            MotorState::Error => 2,
        }
    }
}

impl HallSensorData {
    /// Create from a HallSnapshot (internal type)
    pub fn from_snapshot(snapshot: &crate::foc::sensors::HallSnapshot, seq: u32) -> Self {
        Self {
            angle_rad: snapshot.angle_rad,
            direction: snapshot.direction,
            state: snapshot.state,
            error_count: snapshot.error_count,
            seq,
        }
    }
}

impl AdcSample {
    /// Create from an AdcSnapshot (internal type)
    pub fn from_snapshot(snapshot: &crate::foc::sensors::AdcSnapshot) -> Self {
        Self {
            ia: snapshot.ia,
            ib: snapshot.ib,
            ic: snapshot.ic,
            vbus_mv: snapshot.vbus_mv,
            fet_temp_c_x10: snapshot.fet_temp_c_x10().unwrap_or(0),
            seq: snapshot.seq,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_state_conversion() {
        assert_eq!(MotorState::from_u8(0), MotorState::Stopped);
        assert_eq!(MotorState::from_u8(1), MotorState::Running);
        assert_eq!(MotorState::from_u8(2), MotorState::Error);
        assert_eq!(MotorState::from_u8(255), MotorState::Error);

        assert_eq!(MotorState::Stopped.to_u8(), 0);
        assert_eq!(MotorState::Running.to_u8(), 1);
        assert_eq!(MotorState::Error.to_u8(), 2);
    }
}
