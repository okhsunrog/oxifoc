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
// Delivery / idempotency-key wire types
// ============================================================================

/// A client-chosen request id, stable across retries of the same logical
/// request. The server deduplicates on it so a non-idempotent action runs at
/// most once. See the `delivery` module for the full ladder.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ReqId(pub u64);

/// A request payload tagged with a [`ReqId`] — the wire envelope for a
/// deduplicated (effectively-once) endpoint. Mirrors an HTTP idempotency key.
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct Keyed<T> {
    /// Stable across retries; the dedup key.
    pub id: ReqId,
    /// The actual request.
    pub inner: T,
}

impl<T> Keyed<T> {
    /// Tag a request with an id.
    pub fn new(id: ReqId, inner: T) -> Self {
        Self { id, inner }
    }
}

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
    /// Open-loop mode — drive motor at commanded electrical angle.
    ///
    /// Uses commanded angle instead of sensor feedback. Current control still
    /// runs to regulate the applied current.
    ///
    /// When `velocity_rad_s == 0`: locks rotor at `angle_rad` (calibration use).
    /// When `velocity_rad_s != 0`: firmware advances angle at the given speed
    /// (open-loop spinning without hall sensors).
    OpenLoop {
        /// Initial electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps) - applied as q-current
        current: f32,
        /// Electrical velocity (rad/s) — 0 = lock, nonzero = spin
        velocity_rad_s: f32,
        /// Optional PI gains override (kp, ki). Applied on mode entry.
        /// Used by detection to set conservative gains when motor params are unknown.
        pi_gains: Option<(f32, f32)>,
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
    /// Coast mode — all FETs off (high-impedance), motor spins freely.
    ///
    /// Used during spin-down flux linkage measurement.  Phase voltages
    /// float so back-EMF can be read directly by ADC.
    Coast,
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

// ============================================================================
// Streaming Telemetry Types (push-based via Topics)
// ============================================================================

/// High-frequency motor control telemetry (streamed at configurable rate).
///
/// Contains FOC loop outputs in engineering units. ISR decimates and pushes
/// to a lock-free bbqueue; async task drains and broadcasts batches.
///
/// `#[repr(C)]` with explicit padding for `bytemuck::Pod` (zero-copy ISR→task transfer).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "runtime", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct FastTelemetry {
    /// Phase A current in Amps
    pub ia: f32,
    /// Phase B current in Amps
    pub ib: f32,
    /// Phase C current in Amps
    pub ic: f32,
    /// D-axis current in Amps
    pub id: f32,
    /// Q-axis current in Amps
    pub iq: f32,
    /// D-axis voltage in Volts
    pub vd: f32,
    /// Q-axis voltage in Volts
    pub vq: f32,
    /// Electrical angle in radians
    pub angle_rad: f32,
    /// Electrical RPM
    pub erpm: i32,
    /// Duty cycle in 0.1% units (0-1000)
    pub duty_x10: i16,
    /// Raw Hall sensor state (0-7)
    pub hall_state: u8,
    /// Explicit padding for bytemuck::Pod
    #[serde(skip)]
    pub _pad: u8,
    /// Monotonic sequence number
    pub seq: u32,
}

/// Batch of fast telemetry samples for efficient network transmission.
///
/// Generic over capacity `N`. The wire format is independent of `N` — postcard
/// only encodes actual elements. A sender using `FastTelemetryBatch<8>` is
/// wire-compatible with a receiver using `FastTelemetryBatch<32>`.
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastTelemetryBatch<const N: usize = 32> {
    pub samples: Vec<FastTelemetry, N>,
}

/// Low-frequency system telemetry (streamed at configurable rate, default 10Hz)
///
/// Contains slowly-changing system health data. Firmware pushes this
/// at a lower rate since these values don't change rapidly.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlowTelemetry {
    /// DC bus voltage in millivolts
    pub vbus_mv: u32,
    /// FET temperature in 0.1°C units
    pub fet_temp_c_x10: u16,
    /// Motor temperature in 0.1°C units (0 if not available)
    pub motor_temp_c_x10: u16,
    /// Board temperature in 0.1°C units (0 if not available)
    pub board_temp_c_x10: u16,
    /// Current motor state
    pub motor_state: MotorState,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Number of active faults
    pub fault_count: u8,
    /// Active phase source (Hall / Observer / HFI / crossovers)
    pub phase_source: crate::foc::phase::PhaseSource,
    /// Monotonic sequence number
    pub seq: u32,
}

/// Response to a phase-source change request.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhaseSourceAck {
    /// Whether the command was enqueued to the control ISR. Confirm the
    /// actual switch via `SlowTelemetry::phase_source`.
    pub enqueued: bool,
}

// TODO: EnergyTelemetry — add when energy tracking is implemented
// Will include: amp_hours, amp_hours_charged, watt_hours, watt_hours_charged,
// tachometer (electrical revolutions). Streamed at ~1 Hz.

/// Telemetry rate configuration
///
/// Host sends this to start/stop fast telemetry streaming.
/// Device does not stream until host explicitly requests it.
/// Slow telemetry is poll-based (host controls the rate).
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryConfig {
    /// Desired fast telemetry rate in Hz. 0 = stop streaming.
    /// Device computes the closest achievable rate from FOC frequency.
    pub fast_hz: u16,
}

/// Acknowledgment for telemetry config change
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryConfigAck {
    /// Actual fast rate in Hz after applying the divider
    pub actual_fast_hz: u16,
}

/// Hardware information returned on initial handshake
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct HardwareInfo {
    /// Hardware identifier (e.g., "B-G431B-ESC1")
    pub hw: String<32>,
    /// Software version (e.g., "oxifoc-0.1.0")
    pub sw: String<32>,
    /// MCU chip name (e.g., "STM32G431CB")
    pub mcu: String<32>,
    /// 96-bit unique device ID (STM32 UID, hex-encoded)
    pub uuid: String<32>,
    /// FOC loop frequency in Hz (e.g., 20000)
    pub foc_freq_hz: u32,
    /// Hardware peak current limit in Amps
    pub max_current_a: f32,
}

impl Default for HardwareInfo {
    fn default() -> Self {
        Self {
            hw: String::new(),
            sw: String::new(),
            mcu: String::new(),
            uuid: String::new(),
            foc_freq_hz: 0,
            max_current_a: 0.0,
        }
    }
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
// Motor Detection Protocol Types
// ============================================================================

/// Motor detection request from host.
///
/// Motor detection request — one step per request.
///
/// Each step is independent. GUI provides all required parameters explicitly
/// (e.g., resistance from a previous measurement or from saved config).
/// PI gains are computed on the host side from R and L (like VESC Tool).
// PartialEq: the device-side dedup cache verifies the request payload, not
// just the ReqId — see runtime/detect.rs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectRequest {
    /// Measure phase-to-neutral resistance.
    MeasureResistance {
        /// Max power dissipation during test (W). Controls safe test current.
        max_power_loss_w: f32,
    },
    /// Measure d/q-axis inductance via HFI.
    MeasureInductance {
        /// Max power dissipation during test (W).
        max_power_loss_w: f32,
        /// Previously measured resistance (Ω). GUI provides this.
        resistance_ohm: f32,
    },
    /// Measure flux linkage via open-loop spin.
    MeasureFlux {
        /// Max power dissipation during test (W).
        max_power_loss_w: f32,
        /// Previously measured resistance (Ω). GUI provides this.
        resistance_ohm: f32,
        /// Number of pole pairs.
        pole_pairs: u8,
        /// Open-loop ERPM for spin-up.
        openloop_erpm: f32,
    },
    /// Calibrate Hall sensors by sweeping electrical angle.
    /// No prerequisites — only needs motor connected.
    CalibrateHall,
}

/// Motor detection response — matches the request step.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectResponse {
    /// Resistance measurement result.
    Resistance {
        /// Phase-to-neutral resistance (Ohms)
        resistance_ohm: f32,
    },
    /// Inductance measurement result.
    Inductance {
        /// d-axis inductance (Henries)
        inductance_d_h: f32,
        /// q-axis inductance (Henries)
        inductance_q_h: f32,
    },
    /// Flux linkage measurement result.
    FluxLinkage {
        /// Flux linkage (Weber)
        flux_linkage_wb: f32,
        /// Motor Kv (RPM/V)
        kv_rpm_per_v: f32,
    },
    /// Hall calibration succeeded.
    /// Read calibration angles via config endpoint (HallCalibration group).
    HallCalibrated,
    /// Step failed.
    Error(DetectError),
}

/// Wire-friendly detection error.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectError {
    /// Motor not responding (open circuit, disconnected)
    MotorNotResponding,
    /// Measured value outside physical range
    OutOfRange,
    /// Detection timed out
    Timeout,
    /// Hardware fault during measurement
    HardwareFault,
    /// Not enough valid samples collected
    InsufficientSamples,
    /// Measurement too noisy
    LowConfidence,
    /// Prerequisite measurement not done (e.g., inductance without resistance)
    MissingPrerequisite,
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
        /// Refused: motor is running. Flash writes stall the chip (sector
        /// erase takes up to seconds on F405) and would glitch the FOC loop.
        Busy,
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
