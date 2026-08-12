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

pub type UncachedStorage<KEY> = sequential_storage::cache::Cache<
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    sequential_storage::cache::Uncached,
    KEY,
>;

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
    /// Command-staleness deadman + failsafe policy
    Failsafe = 9,
    /// Cruise velocity-loop tuning
    Velocity = 10,
    /// Graduated derating ramps (thermal/voltage/speed)
    Derating = 11,
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
            9 => Self::Failsafe,
            10 => Self::Velocity,
            11 => Self::Derating,
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
    /// Continuous current RATING (A) — the motor's thermal ceiling from
    /// the detection solve `√(max_power_loss / R / 1.5)` (VESC's i_max),
    /// uncapped by session limits. Owned by the motor, not the setup:
    /// effective operational limits are clamped to it at apply time
    /// (`CurrentLimits::from_config_clamped`). 0 = unknown (pre-rating
    /// config blob) — no rating clamp applied.
    pub max_current_a: f32,
    /// Power-dissipation class (W) the rating was solved for — the
    /// "motor size" chosen at detection (VESC wizard equivalent),
    /// persisted so re-detection and derived defaults reuse it.
    /// 0 = unknown.
    pub max_power_loss_w: f32,
    /// Fundamental (voltage-pulse) d-axis inductance (H) — the
    /// dq-DECOUPLING value. Distinct from `inductance_d_h`, which is the
    /// AC/high-frequency plateau the ESTIMATION chain and PI gains use
    /// (the two-inductance rule; on eddy-current-heavy motors they
    /// differ several-fold — ZD2808: 86/129 µH fundamental vs ~24 µH
    /// AC). 0 = unknown → decoupling falls back to the AC values.
    pub ld_fundamental_h: f32,
    /// Fundamental (voltage-pulse) q-axis inductance (H) — see
    /// [`Self::ld_fundamental_h`]. 0 = unknown.
    pub lq_fundamental_h: f32,
}

impl PostcardValue<'_> for MotorParamsConfig {}

impl MotorParamsConfig {
    /// Check if parameters are valid (have been calibrated)
    pub fn is_valid(&self) -> bool {
        // `> 0.0` rejects NaN for R/L (NaN comparisons are false); the flux
        // check needs the explicit is_finite. Inductances feed PI tuning
        // directly — a NaN there becomes NaN gains and garbage duties.
        self.resistance_ohm > 0.0
            && self.inductance_d_h > 0.0
            && self.inductance_q_h > 0.0
            && self.flux_linkage_wb.is_finite()
            && self.flux_linkage_wb >= 0.0
            && self.pole_pairs > 0
    }

    /// Fundamental d/q inductances for the dq-decoupling feedforward,
    /// when known (see [`Self::ld_fundamental_h`]): `None` for blobs
    /// written before the fields existed or when detection has not
    /// measured them — callers then fall back to the AC values.
    pub fn fundamental_ld_lq(&self) -> Option<(f32, f32)> {
        if self.ld_fundamental_h.is_finite()
            && self.ld_fundamental_h > 0.0
            && self.lq_fundamental_h.is_finite()
            && self.lq_fundamental_h > 0.0
        {
            Some((self.ld_fundamental_h, self.lq_fundamental_h))
        } else {
            None
        }
    }

    /// The motor's continuous current rating, when known.
    ///
    /// NaN/zero/negative (including blobs written before the rating
    /// fields existed) read as "no rating" — callers then skip the
    /// rating clamp rather than clamping to garbage.
    pub fn rating_current_a(&self) -> Option<f32> {
        if self.max_current_a.is_finite() && self.max_current_a > 0.0 {
            Some(self.max_current_a)
        } else {
            None
        }
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
    /// Check that exactly the six physical Hall states are calibrated and
    /// that every angle which will reach commutation math is finite.
    pub fn is_calibrated(&self) -> bool {
        !self.valid[0]
            && !self.valid[7]
            && self.valid[1..=6].iter().all(|&valid| valid)
            && self.angles[1..=6].iter().all(|angle| angle.is_finite())
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
    /// Maximum supply (bus) current draw (A). `< 0` = unlimited.
    /// VESC's `l_in_current_max`: protects the battery/PSU and wiring —
    /// at low duty a large phase current is a small bus current.
    pub bus_in_max_a: f32,
    /// Maximum regen (charge) current pushed back into the supply (A,
    /// positive magnitude). `< 0` = unlimited, **0 = no regen at all** —
    /// the safe setting for a lab PSU, which cannot absorb reverse
    /// current. VESC's `l_in_current_min` (negated).
    pub bus_regen_max_a: f32,
}

impl PostcardValue<'_> for CurrentLimitsConfig {}

impl CurrentLimitsConfig {
    /// Boundary validation for host writes: every field finite, and when
    /// both phase-side limits are set the overcurrent trip must clear the
    /// iq ceiling by [`OVERCURRENT_HEADROOM`] — otherwise a legitimate
    /// full-throttle command (target + PI overshoot + HFI ripple) lives
    /// inside the Kill band and trips mid-ride. The config server rejects
    /// an incoherent write loudly (`ConfigResponse::Invalid`) so the user
    /// learns the rule; `CurrentLimits::from_config_clamped` additionally
    /// clamps whatever arrives by other paths (baked/boot configs).
    ///
    /// `<= 0` keeps its "not set" / "unlimited" / "no regen" semantics
    /// and is always coherent — the ceilings fill in with proper headroom.
    ///
    /// [`OVERCURRENT_HEADROOM`]: crate::motor::foc_driver::OVERCURRENT_HEADROOM
    pub fn is_coherent(&self) -> bool {
        use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
        let finite = self.max_iq_a.is_finite()
            && self.max_phase_current_a.is_finite()
            && self.bus_in_max_a.is_finite()
            && self.bus_regen_max_a.is_finite();
        let headroom_ok = !(self.max_iq_a > 0.0 && self.max_phase_current_a > 0.0)
            || self.max_phase_current_a >= OVERCURRENT_HEADROOM * self.max_iq_a;
        finite && headroom_ok
    }
}

impl Default for CurrentLimitsConfig {
    fn default() -> Self {
        Self {
            max_iq_a: 10.0,
            max_phase_current_a: 40.0,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
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

impl PiGainsConfig {
    /// Gate for every consumer (config boundary, boot): the gains multiply
    /// the current error straight into the output voltage, and the dq
    /// software overcurrent check on the resulting NaN currents compares
    /// false — a NaN/zero kp latches a dead or unstable loop that no
    /// downstream protection catches.
    pub fn is_sane(&self) -> bool {
        self.kp.is_finite() && self.ki.is_finite() && self.kp > 0.0 && self.ki >= 0.0
    }
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

/// Command-staleness deadman + failsafe reaction (see
/// [`crate::motor::failsafe`]). Stored in ms / u8 for compactness; the driver
/// converts to its runtime form via `FailsafeConfig::from_stored`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FailsafeConfigStored {
    /// Command-staleness threshold (ms) before the failsafe arms.
    pub staleness_timeout_ms: u32,
    /// Reaction policy, as `FailsafePolicy as u8` (0=Coast, 1=RampToZero,
    /// 2=ControlledStop; unknown → Coast).
    pub policy: u8,
    /// Regen-brake current cap (A).
    pub brake_current_a: f32,
    /// q-current slew time (ms).
    pub ramp_ms: f32,
    /// Hard cap on the brake duration (ms); the smart give-up is the
    /// no-progress detector in the controller.
    pub brake_time_ms: f32,
    /// |ω_e| (electrical rad/s) treated as stopped.
    pub standstill_rad_s: f32,
    /// Brake deceleration (electrical rad/s²) for the velocity-ramped stop.
    pub decel_rad_s2: f32,
    /// Clean-stop terminal, as `FailsafeTerminal as u8` (0=HighZ,
    /// 1=ParkBrake; unknown → HighZ).
    pub terminal: u8,
}

impl PostcardValue<'_> for FailsafeConfigStored {}

/// Cruise velocity-loop tuning (see [`crate::foc::velocity`]); mirrors
/// `VelocityLoopConfig` field-for-field. The driver decodes via
/// `VelocityLoopConfig::from_stored` (sane-checked, falls back to default).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VelocityConfigStored {
    /// Proportional gain, A per (electrical rad/s).
    pub kp: f32,
    /// Integral gain, A per (electrical rad/s · s).
    pub ki: f32,
    /// Reference accel/decel ramp limit (electrical rad/s²).
    pub accel_limit: f32,
    /// Acceleration feedforward, A per (electrical rad/s²) of REFERENCE
    /// slew — the measured free-rotor accel-per-amp inverted (ZD2808:
    /// 1/8100 ≈ 1.2e-4). The ramp's torque is delivered open-loop and the
    /// PI only trims, which kills the ramp-lag overshoot. 0 = off.
    pub accel_ff: f32,
}

impl PostcardValue<'_> for VelocityConfigStored {}

impl Default for VelocityConfigStored {
    /// Mirrors `VelocityLoopConfig::default` (soft, hall-edge-rate safe).
    fn default() -> Self {
        Self {
            kp: 0.01,
            ki: 0.2,
            accel_limit: 500.0,
            accel_ff: 0.0,
        }
    }
}

impl Default for FailsafeConfigStored {
    /// Longboard default: brake to a controlled stop on link loss, then hold
    /// the parking brake (mirrors `FailsafeConfig::default`).
    fn default() -> Self {
        Self {
            staleness_timeout_ms: 150,
            policy: 2, // ControlledStop
            brake_current_a: 15.0,
            ramp_ms: 100.0,
            brake_time_ms: 10_000.0,
            standstill_rad_s: 20.0,
            decel_rad_s2: 1_000.0,
            terminal: 1, // ParkBrake
        }
    }
}

/// Graduated derating ramps (see [`crate::motor::derating`]); mirrors
/// `DeratingConfig` field-for-field (same SI units). The driver decodes
/// via `DeratingConfig::from_stored` (sane-checked, falls back to the
/// default = FET thermal rolloff only).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeratingConfigStored {
    /// FET temperature ramp start (°C); 0 disables
    pub temp_fet_start_c: f32,
    /// FET temperature ramp end (°C; scale = 0, the OverTemp fault sits here)
    pub temp_fet_end_c: f32,
    /// Motor temperature ramp start (°C); 0 disables (no NTC)
    pub temp_motor_start_c: f32,
    /// Motor temperature ramp end (°C)
    pub temp_motor_end_c: f32,
    /// Acceleration-derate factor 0..=1 (VESC `l_temp_accel_dec`)
    pub accel_dec: f32,
    /// Battery cutoff ramp start (V, full drive above); 0 disables
    pub vbus_cut_start_v: f32,
    /// Battery cutoff ramp end (V, zero drive at/below; < start)
    pub vbus_cut_end_v: f32,
    /// Regen overvoltage ramp start (V, full brake below); 0 disables
    pub vbus_regen_start_v: f32,
    /// Regen overvoltage ramp end (V, zero regen at/above; > start)
    pub vbus_regen_end_v: f32,
    /// Speed soft ceiling (electrical rad/s; eRPM = erad/s × 60/2π); 0 disables
    pub max_speed_erad_s: f32,
    /// Fraction of the ceiling where the drive rolloff starts
    pub speed_start_frac: f32,
}

impl PostcardValue<'_> for DeratingConfigStored {}

impl Default for DeratingConfigStored {
    /// Mirrors `DeratingConfig::default` (FET 85→100 °C, accel_dec 0.15,
    /// per-vehicle ramps off).
    fn default() -> Self {
        Self {
            temp_fet_start_c: 85.0,
            temp_fet_end_c: 100.0,
            temp_motor_start_c: 0.0,
            temp_motor_end_c: 0.0,
            accel_dec: 0.15,
            vbus_cut_start_v: 0.0,
            vbus_cut_end_v: 0.0,
            vbus_regen_start_v: 0.0,
            vbus_regen_end_v: 0.0,
            max_speed_erad_s: 0.0,
            speed_start_frac: 0.8,
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
    pub failsafe: Option<FailsafeConfigStored>,
    pub velocity: Option<VelocityConfigStored>,
    pub derating: Option<DeratingConfigStored>,
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
    Failsafe(FailsafeConfigStored),
    Velocity(VelocityConfigStored),
    Derating(DeratingConfigStored),
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

// ============================================================================
// Shared Storage Worker (generic over the platform flash driver)
// ============================================================================

/// Storage worker loop: loads all configs at boot, signals
/// [`CONFIG_LOADED`], then serves [`FLASH_CHANNEL`] forever.
///
/// The platform provides the flash driver and the storage range; everything
/// else (key layout, load order, operation handling) is identical across
/// boards and lives here.
#[cfg(all(feature = "runtime", feature = "storage"))]
pub async fn run_storage_worker<F>(
    storage: &mut sequential_storage::map::MapStorage<ConfigKey, F, UncachedStorage<ConfigKey>>,
    buf: &mut [u8],
) -> !
where
    F: embedded_storage_async::nor_flash::NorFlash,
{
    // Boot-time: load all stored configs
    let cfg = load_all(storage, buf).await;
    CONFIG_LOADED.signal(cfg);

    // Runtime: handle write operations
    loop {
        let op = FLASH_CHANNEL.receive().await;

        let success = match op {
            FlashOperation::Save(key, payload) => {
                let result = match payload {
                    ConfigPayload::MotorParams(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallCalibration(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::DcOffsets(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::CurrentLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::VoltageLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PwmConfig(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PiGains(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallTuning(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Failsafe(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Velocity(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Derating(v) => storage.store_item(buf, &key, &v).await,
                };
                if result.is_err() {
                    #[cfg(feature = "defmt")]
                    defmt::error!("Failed to save config");
                }
                result.is_ok()
            }
            FlashOperation::EraseAll => {
                let result = storage.erase_all().await;
                if result.is_err() {
                    #[cfg(feature = "defmt")]
                    defmt::error!("Failed to erase storage");
                }
                result.is_ok()
            }
        };

        FLASH_DONE.signal(success);
    }
}

/// Load all stored configs. Missing keys (or any read error) become None.
#[cfg(all(feature = "runtime", feature = "storage"))]
async fn load_all<F>(
    storage: &mut sequential_storage::map::MapStorage<ConfigKey, F, UncachedStorage<ConfigKey>>,
    buf: &mut [u8],
) -> RuntimeConfig
where
    F: embedded_storage_async::nor_flash::NorFlash,
{
    let mut cfg = RuntimeConfig::default();

    macro_rules! load {
        ($field:ident, $key:ident) => {
            cfg.$field = storage
                .fetch_item(buf, &ConfigKey::$key)
                .await
                .ok()
                .flatten();
        };
    }

    load!(motor_params, MotorParams);
    load!(hall_calibration, HallCalibration);
    load!(dc_offsets, DcOffsets);
    load!(current_limits, CurrentLimits);
    load!(voltage_limits, VoltageLimits);
    load!(pwm_config, PwmConfig);
    load!(pi_gains, PiGains);
    load!(hall_tuning, HallTuning);
    load!(failsafe, Failsafe);
    load!(velocity, Velocity);
    load!(derating, Derating);

    cfg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inductance_model_accessors_gate_on_valid_values() {
        let mut mp = MotorParamsConfig::default();
        assert!(mp.fundamental_ld_lq().is_none());
        mp.ld_fundamental_h = 85.7e-6;
        assert!(
            mp.fundamental_ld_lq().is_none(),
            "one axis alone must not enable decoupling override"
        );
        mp.lq_fundamental_h = 129.4e-6;
        assert_eq!(mp.fundamental_ld_lq(), Some((85.7e-6, 129.4e-6)));
    }

    #[test]
    fn motor_params_validity_rejects_non_finite() {
        let good = MotorParamsConfig {
            resistance_ohm: 0.1,
            inductance_d_h: 1e-4,
            inductance_q_h: 3e-4,
            flux_linkage_wb: 0.02,
            pole_pairs: 7,
            max_current_a: 15.0,
            max_power_loss_w: 50.0,
            ..Default::default()
        };
        assert!(good.is_valid());

        // NaN anywhere numeric must invalidate — these feed PI tuning and
        // the observers directly.
        for f in [
            |p: &mut MotorParamsConfig| p.resistance_ohm = f32::NAN,
            |p: &mut MotorParamsConfig| p.inductance_d_h = f32::NAN,
            |p: &mut MotorParamsConfig| p.inductance_q_h = 0.0,
            |p: &mut MotorParamsConfig| p.flux_linkage_wb = f32::NAN,
            |p: &mut MotorParamsConfig| p.flux_linkage_wb = f32::INFINITY,
        ] {
            let mut bad = good.clone();
            f(&mut bad);
            assert!(!bad.is_valid(), "must reject {bad:?}");
        }
    }

    /// Boundary rule for current-limits writes: finite fields, and the
    /// trip must clear the iq ceiling by the overcurrent headroom.
    #[test]
    fn current_limits_coherence() {
        let base = CurrentLimitsConfig::default();
        assert!(base.is_coherent(), "default 10/40 must pass");

        // Exactly at the headroom boundary: allowed.
        let at = CurrentLimitsConfig {
            max_iq_a: 10.0,
            max_phase_current_a: 13.0,
            ..base.clone()
        };
        assert!(at.is_coherent());

        // Inside the band: rejected (the foot-gun).
        let inside = CurrentLimitsConfig {
            max_iq_a: 40.0,
            max_phase_current_a: 40.0,
            ..base.clone()
        };
        assert!(!inside.is_coherent());

        // "Not set" sides are always coherent — ceilings fill in.
        let unset_iq = CurrentLimitsConfig {
            max_iq_a: 0.0,
            max_phase_current_a: 40.0,
            ..base.clone()
        };
        assert!(unset_iq.is_coherent());
        let unset_phase = CurrentLimitsConfig {
            max_iq_a: 40.0,
            max_phase_current_a: -1.0,
            ..base.clone()
        };
        assert!(unset_phase.is_coherent());

        // Non-finite anywhere: rejected at the boundary (the builder's
        // NaN tolerance is for the boot path, not for explicit writes).
        for f in [
            |c: &mut CurrentLimitsConfig| c.max_iq_a = f32::NAN,
            |c: &mut CurrentLimitsConfig| c.max_phase_current_a = f32::INFINITY,
            |c: &mut CurrentLimitsConfig| c.bus_in_max_a = f32::NAN,
            |c: &mut CurrentLimitsConfig| c.bus_regen_max_a = f32::NAN,
        ] {
            let mut bad = base.clone();
            f(&mut bad);
            assert!(!bad.is_coherent(), "must reject {bad:?}");
        }
    }
    #[test]
    fn pi_gains_sanity_gate() {
        let ok = PiGainsConfig {
            kp: 0.4,
            ki: 40.0,
            bandwidth_rad_s: 1000.0,
        };
        assert!(ok.is_sane());
        assert!(!PiGainsConfig { kp: 0.0, ..ok }.is_sane());
        assert!(!PiGainsConfig { kp: -1.0, ..ok }.is_sane());
        assert!(!PiGainsConfig { ki: -1.0, ..ok }.is_sane());
        assert!(!PiGainsConfig { kp: f32::NAN, ..ok }.is_sane());
        assert!(!PiGainsConfig { ki: f32::NAN, ..ok }.is_sane());
    }

    #[test]
    fn hall_calibration_requires_physical_states_and_finite_angles() {
        let good = HallCalibrationConfig::default();
        assert!(good.is_calibrated());

        let mut missing = good.clone();
        missing.valid[3] = false;
        assert!(!missing.is_calibrated());

        let mut shifted = good.clone();
        shifted.valid[3] = false;
        shifted.valid[0] = true;
        assert!(!shifted.is_calibrated());

        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut non_finite = good.clone();
            non_finite.angles[4] = invalid;
            assert!(!non_finite.is_calibrated());
        }
    }
}
