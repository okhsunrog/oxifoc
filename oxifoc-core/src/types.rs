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
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// Re-export Direction from hall_sensor (it has conditional serde derives)
pub use crate::foc::hall_sensor::Direction;

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
    /// HFI injection mode for inductance measurement
    ///
    /// Locks rotor at angle 0 with a holding current while injecting
    /// high-frequency voltage for inductance measurement.
    HfiInjection {
        /// DC current to hold rotor in place (Amps)
        hold_current: f32,
        /// d-axis voltage to inject (V)
        vd_inject: f32,
        /// q-axis voltage to inject (V)
        vq_inject: f32,
    },
}

/// Motor fault codes
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultCode {
    /// No fault
    None,
    /// Over-current detected
    OverCurrent,
    /// Over-voltage on DC bus
    OverVoltage,
    /// Under-voltage on DC bus
    UnderVoltage,
    /// Over-temperature (FET or motor)
    OverTemperature,
    /// Hall sensor error (invalid state or sequence)
    HallSensorError,
    /// Encoder error
    EncoderError,
    /// Communication timeout
    CommTimeout,
    /// Motor stalled
    Stall,
    /// Calibration required
    CalibrationRequired,
    /// Generic hardware fault
    HardwareFault,
}

// ============================================================================
// Status/Telemetry Types
// ============================================================================

/// Motor status response
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorStatus {
    /// Current motor state
    pub state: MotorState,
    /// Current control mode
    pub mode: ControlMode,
    /// Primary/first active fault code (if any)
    pub fault: Option<FaultCode>,
    /// Full fault bitmask (all active faults)
    pub fault_bits: u32,
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
// Fault Management Types
// ============================================================================

/// Fault management request
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultRequest {
    /// If true, clear all clearable faults
    pub clear_all: bool,
    /// Optional: clear only faults matching this bitmask
    pub clear_mask: Option<u32>,
}

/// Fault management response
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultResponse {
    /// Currently active faults (bitmask)
    pub active_faults: u32,
    /// Primary fault code (first active fault)
    pub primary_fault: Option<FaultCode>,
}

// ============================================================================
// Conversion helpers
// ============================================================================

// FaultKind <-> FaultCode conversions
use crate::foc::fault::FaultKind;

impl From<FaultKind> for FaultCode {
    fn from(kind: FaultKind) -> Self {
        match kind {
            FaultKind::OverCurrent => FaultCode::OverCurrent,
            FaultKind::OverVoltage => FaultCode::OverVoltage,
            FaultKind::UnderVoltage => FaultCode::UnderVoltage,
            FaultKind::OverTemp => FaultCode::OverTemperature,
            FaultKind::DriverFault => FaultCode::HardwareFault,
            FaultKind::CalibrationFailed => FaultCode::CalibrationRequired,
            FaultKind::CommsTimeout => FaultCode::CommTimeout,
            FaultKind::HallSensorError => FaultCode::HallSensorError,
            FaultKind::Stall => FaultCode::Stall,
            FaultKind::Unknown => FaultCode::HardwareFault,
        }
    }
}

impl From<FaultCode> for Option<FaultKind> {
    fn from(code: FaultCode) -> Self {
        match code {
            FaultCode::None => None,
            FaultCode::OverCurrent => Some(FaultKind::OverCurrent),
            FaultCode::OverVoltage => Some(FaultKind::OverVoltage),
            FaultCode::UnderVoltage => Some(FaultKind::UnderVoltage),
            FaultCode::OverTemperature => Some(FaultKind::OverTemp),
            FaultCode::HallSensorError => Some(FaultKind::HallSensorError),
            FaultCode::EncoderError => Some(FaultKind::Unknown), // No encoder fault in FaultKind
            FaultCode::CommTimeout => Some(FaultKind::CommsTimeout),
            FaultCode::Stall => Some(FaultKind::Stall),
            FaultCode::CalibrationRequired => Some(FaultKind::CalibrationFailed),
            FaultCode::HardwareFault => Some(FaultKind::DriverFault),
        }
    }
}

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
