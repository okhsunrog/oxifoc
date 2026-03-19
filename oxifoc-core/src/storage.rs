//! Persistent configuration storage types.
//!
//! Defines the config group structs and key enum used with
//! [`sequential_storage::map`] for flash-backed key-value storage.
//!
//! Platform crates provide the flash driver and storage task;
//! this module provides the shared type definitions.
//!
//! All value types implement [`PostcardValue`] via the marker trait,
//! giving them automatic postcard serialization for sequential-storage.

use sequential_storage::map::{Key, PostcardValue, SerializationError};
use serde::{Deserialize, Serialize};

// ============================================================================
// Storage Keys
// ============================================================================

/// Keys for configuration groups stored in flash.
///
/// Each key maps to a small struct serialized with postcard.
/// New keys can be added without invalidating existing storage.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigKey {
    /// Motor electrical parameters (R, L, λ, pole pairs)
    MotorParams = 1,
    /// Hall sensor calibration (sector angles)
    HallCalibration = 2,
    /// Current sensor DC offsets
    DcOffsets = 3,
    /// Current limits (Iq, phase current)
    CurrentLimits = 4,
    /// Voltage limits (min/max VBUS)
    VoltageLimits = 5,
    /// PWM configuration
    PwmConfig = 6,
    /// PI controller gains
    PiGains = 7,
    /// Hall interpolation tuning
    HallTuning = 8,
}

impl Key for ConfigKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = *self as u8;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        let key = match buffer[0] {
            1 => Self::MotorParams,
            2 => Self::HallCalibration,
            3 => Self::DcOffsets,
            4 => Self::CurrentLimits,
            5 => Self::VoltageLimits,
            6 => Self::PwmConfig,
            7 => Self::PiGains,
            8 => Self::HallTuning,
            _ => return Err(SerializationError::InvalidFormat),
        };
        Ok((key, 1))
    }
}

// ============================================================================
// Config Group Structs
// ============================================================================

/// Motor electrical parameters.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorParamsConfig {
    /// Phase-to-neutral resistance (Ω)
    pub resistance_ohm: f32,
    /// d-axis inductance (H)
    pub inductance_d_h: f32,
    /// q-axis inductance (H)
    pub inductance_q_h: f32,
    /// Permanent-magnet flux linkage (Wb)
    pub flux_linkage_wb: f32,
    /// Number of pole pairs
    pub pole_pairs: u8,
}

impl PostcardValue<'_> for MotorParamsConfig {}

impl MotorParamsConfig {
    /// Check if parameters are valid (have been calibrated)
    pub fn is_valid(&self) -> bool {
        self.resistance_ohm > 0.0 && self.pole_pairs > 0
    }
}

/// Hall sensor calibration data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallCalibrationConfig {
    /// Electrical angle (radians) for each raw Hall state (0-7)
    pub angles: [f32; 8],
    /// Validity flags for each Hall state
    pub valid: [bool; 8],
}

impl PostcardValue<'_> for HallCalibrationConfig {}

impl Default for HallCalibrationConfig {
    fn default() -> Self {
        use core::f32::consts::TAU;
        Self {
            angles: [
                0.0,               // state 0 — invalid
                TAU / 12.0,        // state 1 — 30°
                5.0 * TAU / 12.0,  // state 2 — 150°
                TAU / 4.0,         // state 3 — 90°
                3.0 * TAU / 4.0,   // state 4 — 270°
                11.0 * TAU / 12.0, // state 5 — 330°
                7.0 * TAU / 12.0,  // state 6 — 210°
                0.0,               // state 7 — invalid
            ],
            valid: [false, true, true, true, true, true, true, false],
        }
    }
}

impl HallCalibrationConfig {
    /// Check if all 6 valid Hall states have been calibrated
    pub fn is_calibrated(&self) -> bool {
        self.valid.iter().filter(|&&v| v).count() == 6
    }
}

/// Current sensor DC offset calibration data.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DcOffsetsConfig {
    /// Phase A offset (ADC counts)
    pub phase_a: f32,
    /// Phase B offset (ADC counts)
    pub phase_b: f32,
    /// Phase C offset (ADC counts)
    pub phase_c: f32,
}

impl PostcardValue<'_> for DcOffsetsConfig {}

/// Current limits.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentLimitsConfig {
    /// Maximum q-axis (torque) current target (A)
    pub max_iq_a: f32,
    /// Maximum instantaneous phase current (A)
    pub max_phase_current_a: f32,
}

impl PostcardValue<'_> for CurrentLimitsConfig {}

impl Default for CurrentLimitsConfig {
    fn default() -> Self {
        Self {
            max_iq_a: 10.0,
            max_phase_current_a: 40.0,
        }
    }
}

/// Voltage limits.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VoltageLimitsConfig {
    /// Minimum bus voltage before undervoltage fault (mV)
    pub min_vbus_mv: u32,
    /// Maximum bus voltage before overvoltage fault (mV)
    pub max_vbus_mv: u32,
}

impl PostcardValue<'_> for VoltageLimitsConfig {}

impl Default for VoltageLimitsConfig {
    fn default() -> Self {
        Self {
            min_vbus_mv: 8_000,
            max_vbus_mv: 45_000,
        }
    }
}

/// PWM configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmConfigStored {
    /// PWM switching frequency (Hz)
    pub freq_hz: u32,
    /// Maximum duty cycle (percent, 0-100)
    pub max_duty_percent: u8,
}

impl PostcardValue<'_> for PwmConfigStored {}

impl Default for PwmConfigStored {
    fn default() -> Self {
        Self {
            freq_hz: 20_000,
            max_duty_percent: 95,
        }
    }
}

/// PI controller gains.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PiGainsConfig {
    /// Proportional gain
    pub kp: f32,
    /// Integral gain
    pub ki: f32,
    /// Bandwidth used to compute gains (rad/s), for reference
    pub bandwidth_rad_s: f32,
}

impl PostcardValue<'_> for PiGainsConfig {}

impl Default for PiGainsConfig {
    fn default() -> Self {
        Self {
            kp: 0.4,
            ki: 40.0,
            bandwidth_rad_s: 1000.0,
        }
    }
}

/// Hall sensor interpolation tuning.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallTuningConfig {
    /// Minimum eRPM for angle interpolation
    pub interp_min_erpm: f32,
    /// Drift correction gain (fraction per cycle)
    pub drift_correction_gain: f32,
    /// Rate limit factor (1.0 = no overshoot allowed)
    pub rate_limit_factor: f32,
    /// Hall state timeout (µs)
    pub timeout_us: u32,
}

impl PostcardValue<'_> for HallTuningConfig {}

impl Default for HallTuningConfig {
    fn default() -> Self {
        Self {
            interp_min_erpm: 500.0,
            drift_correction_gain: 0.01,
            rate_limit_factor: 1.5,
            timeout_us: 100_000,
        }
    }
}

// ============================================================================
// Runtime Config Aggregate
// ============================================================================

/// All stored configuration loaded at boot.
///
/// Fields are `None` if not yet stored in flash (use board defaults).
#[derive(Debug, Clone, Default)]
pub struct RuntimeConfig {
    pub motor_params: Option<MotorParamsConfig>,
    pub hall_calibration: Option<HallCalibrationConfig>,
    pub dc_offsets: Option<DcOffsetsConfig>,
    pub current_limits: Option<CurrentLimitsConfig>,
    pub voltage_limits: Option<VoltageLimitsConfig>,
    pub pwm_config: Option<PwmConfigStored>,
    pub pi_gains: Option<PiGainsConfig>,
    pub hall_tuning: Option<HallTuningConfig>,
}

// ============================================================================
// Flash Operation Messages (shared across platforms)
// ============================================================================

/// Messages for flash write operations.
///
/// Sent from protocol servers to the platform storage worker task.
#[derive(Clone, Debug)]
pub enum FlashOperation {
    /// Save a config group to flash
    Save(ConfigKey, ConfigPayload),
    /// Erase all stored configuration
    EraseAll,
}

/// Payload variants for each config group.
#[derive(Clone, Debug)]
pub enum ConfigPayload {
    MotorParams(MotorParamsConfig),
    HallCalibration(HallCalibrationConfig),
    DcOffsets(DcOffsetsConfig),
    CurrentLimits(CurrentLimitsConfig),
    VoltageLimits(VoltageLimitsConfig),
    PwmConfig(PwmConfigStored),
    PiGains(PiGainsConfig),
    HallTuning(HallTuningConfig),
}

// ============================================================================
// Shared Channels (require runtime feature for embassy_sync)
// ============================================================================

#[cfg(feature = "runtime")]
mod channels {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;
    use embassy_sync::signal::Signal;

    use super::{FlashOperation, RuntimeConfig};

    /// Channel for sending flash operations to the storage worker task.
    pub static FLASH_CHANNEL: Channel<CriticalSectionRawMutex, FlashOperation, 4> = Channel::new();

    /// Signal indicating flash operation completion (true = success).
    pub static FLASH_DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

    /// Signal carrying loaded config from storage worker to main task at boot.
    pub static CONFIG_LOADED: Signal<CriticalSectionRawMutex, RuntimeConfig> = Signal::new();
}

#[cfg(feature = "runtime")]
pub use channels::*;
