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

pub use crate::types::VoltageFeedforwardMode;
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
/// New firmware preserves existing key numbers. Older firmware that does not
/// recognize a newly written key may fail to scan this map: use a full backup
/// when rolling back across a storage-schema change.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigKey {
    /// Motor electrical parameters (R, L, λ, pole pairs)
    MotorParams = 1,
    /// Legacy Hall sector angles. Read-only migration source; new saves use 14.
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
    /// Per-device analog measurement corrections (versioned payload).
    BoardCalibration = 12,
    /// Explicit current-loop voltage feedforward policy.
    VoltageFeedforward = 13,
    /// Complete versioned Hall calibration, including optional edge geometry.
    HallCalibrationRecord = 14,
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
            12 => Self::BoardCalibration,
            13 => Self::VoltageFeedforward,
            14 => Self::HallCalibrationRecord,
            _ => return Err(SerializationError::InvalidFormat),
        };
        Ok((key, 1))
    }
}

// ============================================================================
// Config Group Structs
// ============================================================================

/// Persistent corrections for one physical board, independent of its motor.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardCalibrationConfig {
    /// Absolute VBUS multiplier, applied once after ADC/VREF conversion.
    pub vbus_gain: f32,
}

impl BoardCalibrationConfig {
    pub const IDENTITY: Self = Self { vbus_gain: 1.0 };

    pub fn is_sane(&self) -> bool {
        self.vbus_gain.is_finite() && (0.95..=1.05).contains(&self.vbus_gain)
    }

    pub fn correct_vbus_mv(&self, nominal_mv: u32) -> u32 {
        (nominal_mv as f32 * self.vbus_gain + 0.5) as u32
    }
}

impl Default for BoardCalibrationConfig {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Freeze each stored layout: future fields require a new variant and an
/// explicit migration, never extension of the V1 postcard struct in place.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
enum StoredBoardCalibration {
    V1(BoardCalibrationV1),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
struct BoardCalibrationV1 {
    vbus_gain: f32,
}

#[cfg(any(feature = "runtime", test))]
impl PostcardValue<'_> for StoredBoardCalibration {}

#[cfg(any(feature = "runtime", test))]
impl From<BoardCalibrationConfig> for StoredBoardCalibration {
    fn from(value: BoardCalibrationConfig) -> Self {
        Self::V1(BoardCalibrationV1 {
            vbus_gain: value.vbus_gain,
        })
    }
}

#[cfg(any(feature = "runtime", test))]
impl StoredBoardCalibration {
    fn validated(self) -> Option<BoardCalibrationConfig> {
        let Self::V1(v) = self;
        let value = BoardCalibrationConfig {
            vbus_gain: v.vbus_gain,
        };
        value.is_sane().then_some(value)
    }
}

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
        // Positive infinity passes `> 0`; reject it before deriving PI
        // gains and observer state at either the config boundary or boot.
        self.resistance_ohm.is_finite()
            && self.inductance_d_h.is_finite()
            && self.inductance_q_h.is_finite()
            && self.resistance_ohm > 0.0
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

/// Untrusted Hall geometry on the wire. Convert through the validated RAM
/// constructor before installing it in an estimator.
pub use crate::types::measurement::HallTransitionGeometryConfig;

impl HallTransitionGeometryConfig {
    pub fn validated(
        &self,
        centroids: [f32; 8],
    ) -> Option<crate::foc::hall_calibration::HallTransitionGeometry> {
        crate::foc::hall_calibration::HallTransitionGeometry::try_from_raw_parts(
            self.entry_angles,
            self.next_states,
            centroids,
        )
    }
}

impl From<&crate::foc::hall_calibration::HallTransitionGeometry> for HallTransitionGeometryConfig {
    fn from(value: &crate::foc::hall_calibration::HallTransitionGeometry) -> Self {
        Self {
            entry_angles: value.entry_angles(),
            next_states: value.next_states(),
        }
    }
}

/// One complete Hall calibration: centroids and the geometry measured with them.
pub use crate::types::measurement::HallCalibrationConfig;

// These flash layouts are frozen. Evolving the public/wire config requires a
// new record variant and explicit conversions, never adding fields to V1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
struct LegacyHallCalibrationConfig {
    angles: [f32; 8],
    valid: [bool; 8],
}
#[cfg(any(feature = "runtime", test))]
impl PostcardValue<'_> for LegacyHallCalibrationConfig {}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
enum HallCalibrationRecord {
    V1(HallCalibrationV1),
}
#[cfg(any(feature = "runtime", test))]
impl PostcardValue<'_> for HallCalibrationRecord {}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
struct HallCalibrationV1 {
    angles: [f32; 8],
    valid: [bool; 8],
    geometry: Option<HallTransitionGeometryV1>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
struct HallTransitionGeometryV1 {
    entry_angles: [f32; 8],
    next_states: [u8; 8],
}

#[cfg(any(feature = "runtime", test))]
impl From<HallCalibrationConfig> for HallCalibrationRecord {
    fn from(value: HallCalibrationConfig) -> Self {
        Self::V1(HallCalibrationV1 {
            angles: value.angles,
            valid: value.valid,
            geometry: value.geometry.map(|g| HallTransitionGeometryV1 {
                entry_angles: g.entry_angles,
                next_states: g.next_states,
            }),
        })
    }
}

#[cfg(any(feature = "runtime", test))]
impl HallCalibrationRecord {
    fn validated(self) -> Option<HallCalibrationConfig> {
        let Self::V1(value) = self;
        let config = HallCalibrationConfig {
            angles: value.angles,
            valid: value.valid,
            geometry: value.geometry.map(|g| HallTransitionGeometryConfig {
                entry_angles: g.entry_angles,
                next_states: g.next_states,
            }),
        };
        config.is_calibrated().then_some(config)
    }
}

#[cfg(any(feature = "runtime", test))]
impl LegacyHallCalibrationConfig {
    fn validated(self) -> Option<HallCalibrationConfig> {
        let config = HallCalibrationConfig {
            angles: self.angles,
            valid: self.valid,
            geometry: None,
        };
        config.is_calibrated().then_some(config)
    }
}

impl From<&crate::foc::hall_calibration::HallCalibrationResult> for HallCalibrationConfig {
    fn from(result: &crate::foc::hall_calibration::HallCalibrationResult) -> Self {
        Self {
            angles: result.angles,
            valid: result.valid,
            geometry: result
                .geometry
                .as_ref()
                .map(HallTransitionGeometryConfig::from),
        }
    }
}

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
            geometry: None,
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
            && self
                .geometry
                .as_ref()
                .is_none_or(|geometry| geometry.validated(self.angles).is_some())
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
    /// Maximum braking q current (A, magnitude): the torque limit whenever
    /// iq opposes the rotation — remote braking, the velocity loop, the
    /// failsafe stop. Separate from `max_iq_a` because braking has its own
    /// reasons to be softer (rider balance, traction, belt skip), and at low
    /// speed the battery regen limit hardly bounds it. `<= 0` = not set:
    /// same as `max_iq_a`. VESC's `l_current_min`.
    pub max_brake_a: f32,
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
    /// Dissipative braking (W): regen the supply refuses (the
    /// regen-overvoltage derate, `bus_regen_max_a`) is burned in the
    /// windings — up to this much heat from the extra d current,
    /// `1.5·R·id²` — instead of cutting the brake torque
    /// (`motor::dissipation`). The d current stays inside the `max_iq_a`
    /// circle, so the total copper loss never exceeds the motor's rating
    /// (`MotorParamsConfig::max_power_loss_w`); this sets how much of it a
    /// long descent may use. Needs detected motor params (R, λ).
    /// `<= 0` = off.
    pub max_dissipation_w: f32,
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
            && self.max_brake_a.is_finite()
            && self.max_phase_current_a.is_finite()
            && self.bus_in_max_a.is_finite()
            && self.bus_regen_max_a.is_finite()
            && self.max_dissipation_w.is_finite();
        let headroom_ok = !(self.max_iq_a > 0.0 && self.max_phase_current_a > 0.0)
            || self.max_phase_current_a >= OVERCURRENT_HEADROOM * self.max_iq_a;
        finite && headroom_ok
    }
}

impl Default for CurrentLimitsConfig {
    fn default() -> Self {
        Self {
            max_iq_a: 10.0,
            max_brake_a: 0.0,
            max_phase_current_a: 40.0,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
            max_dissipation_w: 0.0,
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

impl VoltageLimitsConfig {
    pub fn is_sane(&self) -> bool {
        self.min_vbus_mv > 0 && self.min_vbus_mv < self.max_vbus_mv
    }
}

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
    /// Rate limit factor (1.0 = no overshoot allowed)
    pub rate_limit_factor: f32,
}

impl PostcardValue<'_> for HallTuningConfig {}

impl Default for HallTuningConfig {
    fn default() -> Self {
        Self {
            interp_min_erpm: 500.0,
            rate_limit_factor: 1.5,
        }
    }
}

impl HallTuningConfig {
    /// Usable values: a finite, non-negative interpolation floor, and a rate
    /// limit that lets the angle keep up with the rotor (a factor below 1
    /// holds the commutation angle behind it at any steady speed).
    pub fn is_sane(&self) -> bool {
        (0.0..=100_000.0).contains(&self.interp_min_erpm)
            && (1.0..=10.0).contains(&self.rate_limit_factor)
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
    /// Brake current of the stop (A), within the general `max_brake_a`;
    /// 0 = the general limit alone.
    pub brake_current_a: f32,
    /// Rate (A/s) at which the stop changes the q current.
    pub slew_a_per_s: f32,
    /// |ω_e| (electrical rad/s) treated as stopped.
    pub standstill_rad_s: f32,
    /// Upper bound on the stop's deceleration (electrical rad/s²).
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
            slew_a_per_s: 150.0,
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

/// Voltage-model activation is independent of measured motor parameters.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VoltageFeedforwardConfig {
    pub mode: VoltageFeedforwardMode,
}

impl PostcardValue<'_> for VoltageFeedforwardConfig {}

/// All stored configuration loaded at boot.
///
/// Fields are `None` if not yet stored in flash (use board defaults).
#[derive(Debug, Clone, Default)]
pub struct RuntimeConfig {
    pub board_calibration: Option<BoardCalibrationConfig>,
    /// Boot read/validation failure; distinct from a missing optional record.
    pub board_calibration_invalid: bool,
    pub motor_params: Option<MotorParamsConfig>,
    pub hall_calibration: Option<HallCalibrationConfig>,
    /// A present but unreadable/invalid Hall record cannot fall back to an
    /// older calibration. Repair is a successful complete live Hall apply.
    pub hall_calibration_invalid: bool,
    pub dc_offsets: Option<DcOffsetsConfig>,
    pub current_limits: Option<CurrentLimitsConfig>,
    pub voltage_limits: Option<VoltageLimitsConfig>,
    pub pwm_config: Option<PwmConfigStored>,
    pub pi_gains: Option<PiGainsConfig>,
    pub hall_tuning: Option<HallTuningConfig>,
    pub failsafe: Option<FailsafeConfigStored>,
    pub velocity: Option<VelocityConfigStored>,
    pub derating: Option<DeratingConfigStored>,
    /// Missing records retain plain PI control.
    pub voltage_feedforward: Option<VoltageFeedforwardConfig>,
}

impl RuntimeConfig {
    pub fn board_calibration_is_valid(&self) -> bool {
        !self.board_calibration_invalid && self.board_calibration.is_none_or(|v| v.is_sane())
    }

    pub fn hall_calibration_is_valid(&self) -> bool {
        !self.hall_calibration_invalid
            && self
                .hall_calibration
                .as_ref()
                .is_none_or(HallCalibrationConfig::is_calibrated)
    }
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
    BoardCalibration(BoardCalibrationConfig),
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
    VoltageFeedforward(VoltageFeedforwardConfig),
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
                    ConfigPayload::BoardCalibration(v) => {
                        storage
                            .store_item(buf, &key, &StoredBoardCalibration::from(v))
                            .await
                    }
                    ConfigPayload::MotorParams(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallCalibration(v) => {
                        storage
                            .store_item(
                                buf,
                                &ConfigKey::HallCalibrationRecord,
                                &HallCalibrationRecord::from(v),
                            )
                            .await
                    }
                    ConfigPayload::DcOffsets(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::CurrentLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::VoltageLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PwmConfig(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PiGains(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallTuning(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Failsafe(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Velocity(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::Derating(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::VoltageFeedforward(v) => storage.store_item(buf, &key, &v).await,
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

/// Load stored configs. Safety-relevant calibration failures remain distinct
/// from missing records and must be repaired before their control path runs.
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
    load!(dc_offsets, DcOffsets);
    load!(current_limits, CurrentLimits);
    load!(voltage_limits, VoltageLimits);
    load!(pwm_config, PwmConfig);
    load!(pi_gains, PiGains);
    load!(hall_tuning, HallTuning);
    load!(failsafe, Failsafe);
    load!(velocity, Velocity);
    load!(derating, Derating);
    load!(voltage_feedforward, VoltageFeedforward);

    match storage
        .fetch_item::<HallCalibrationRecord>(buf, &ConfigKey::HallCalibrationRecord)
        .await
    {
        Ok(Some(record)) => {
            cfg.hall_calibration = record.validated();
            cfg.hall_calibration_invalid = cfg.hall_calibration.is_none();
        }
        Ok(None) => match storage
            .fetch_item::<LegacyHallCalibrationConfig>(buf, &ConfigKey::HallCalibration)
            .await
        {
            Ok(None) => {}
            Ok(Some(record)) => {
                cfg.hall_calibration = record.validated();
                cfg.hall_calibration_invalid = cfg.hall_calibration.is_none();
            }
            Err(_) => cfg.hall_calibration_invalid = true,
        },
        Err(_) => cfg.hall_calibration_invalid = true,
    }

    match storage
        .fetch_item::<StoredBoardCalibration>(buf, &ConfigKey::BoardCalibration)
        .await
    {
        Ok(None) => {}
        Ok(Some(stored)) => match stored.validated() {
            Some(value) => cfg.board_calibration = Some(value),
            None => cfg.board_calibration_invalid = true,
        },
        Err(_) => cfg.board_calibration_invalid = true,
    }

    cfg
}

#[cfg(test)]
mod tests {
    use super::*;

    fn measured_hall_config() -> HallCalibrationConfig {
        use core::f32::consts::TAU;
        HallCalibrationConfig {
            geometry: Some(HallTransitionGeometryConfig {
                entry_angles: [
                    0.0,
                    0.0,
                    TAU / 3.0,
                    TAU / 6.0,
                    2.0 * TAU / 3.0,
                    5.0 * TAU / 6.0,
                    TAU / 2.0,
                    0.0,
                ],
                next_states: [0, 3, 6, 2, 5, 1, 4, 0],
            }),
            ..HallCalibrationConfig::default()
        }
    }

    #[test]
    fn hall_record_freezes_legacy_bytes_and_appends_versioned_storage_without_moving_wire_ids() {
        use crate::types::{ConfigGroupId, ConfigValue, ConfigWrite};
        let legacy = LegacyHallCalibrationConfig {
            angles: [0.0; 8],
            valid: [false, true, true, true, true, true, true, false],
        };
        let mut expected = [0u8; 40];
        expected[33..39].fill(1);
        let mut buf = [0; 128];
        assert_eq!(postcard::to_slice(&legacy, &mut buf).unwrap(), expected);
        let migrated = postcard::from_bytes::<LegacyHallCalibrationConfig>(&expected)
            .unwrap()
            .validated()
            .unwrap();
        assert!(migrated.geometry.is_none());
        assert!(
            postcard::from_bytes::<HallCalibrationConfig>(&expected).is_err(),
            "legacy flash must use its explicit frozen decoder"
        );
        let record = HallCalibrationRecord::from(migrated.clone());
        let mut versioned = [0u8; 42];
        versioned[34..40].fill(1);
        assert_eq!(postcard::to_slice(&record, &mut buf).unwrap(), versioned);
        assert_eq!(
            postcard::from_bytes::<HallCalibrationRecord>(&versioned)
                .unwrap()
                .validated(),
            Some(migrated)
        );
        versioned[0] = 1;
        assert!(postcard::from_bytes::<HallCalibrationRecord>(&versioned).is_err());
        assert_eq!(
            postcard::to_slice(&ConfigGroupId::HallCalibration, &mut buf).unwrap(),
            &[1]
        );
        let config = measured_hall_config();
        assert_eq!(
            postcard::to_slice(&ConfigWrite::HallCalibration(config.clone()), &mut buf).unwrap()[0],
            6
        );
        assert_eq!(
            postcard::to_slice(&ConfigValue::HallCalibration(config.clone()), &mut buf).unwrap()[0],
            6
        );
        let record = HallCalibrationRecord::from(config.clone());
        let bytes = postcard::to_slice(&record, &mut buf).unwrap();
        assert_eq!(
            postcard::from_bytes::<HallCalibrationRecord>(bytes)
                .unwrap()
                .validated(),
            Some(config)
        );
        let mut key = [0];
        ConfigKey::HallCalibration.serialize_into(&mut key).unwrap();
        assert_eq!(key, [2]);
        ConfigKey::HallCalibrationRecord
            .serialize_into(&mut key)
            .unwrap();
        assert_eq!(key, [14]);
    }

    #[test]
    fn hall_geometry_validation_includes_the_centroid_reference() {
        let good = measured_hall_config();
        assert!(good.is_calibrated());
        let result = crate::foc::hall_calibration::HallCalibrationResult {
            angles: good.angles,
            valid: good.valid,
            valid_count: 6,
            geometry: good.geometry.as_ref().unwrap().validated(good.angles),
        };
        assert_eq!(
            HallCalibrationConfig::from(&result),
            good,
            "a detection result must retain the complete geometry on the wire"
        );
        let mut bad = good.clone();
        bad.geometry.as_mut().unwrap().next_states[1] = 1;
        assert!(!bad.is_calibrated());
        assert!(HallCalibrationRecord::from(bad).validated().is_none());
        let mut bad = good.clone();
        bad.geometry.as_mut().unwrap().entry_angles[1] = f32::NAN;
        assert!(!bad.is_calibrated());
        let mut bad = good;
        bad.angles[1] = core::f32::consts::PI;
        assert!(
            !bad.is_calibrated(),
            "a valid edge ring cannot belong to mismatched centroids"
        );
        assert!(
            !RuntimeConfig {
                hall_calibration: Some(bad),
                ..Default::default()
            }
            .hall_calibration_is_valid()
        );
        assert!(
            !RuntimeConfig {
                hall_calibration_invalid: true,
                ..Default::default()
            }
            .hall_calibration_is_valid()
        );
        assert!(RuntimeConfig::default().hall_calibration_is_valid());
    }

    #[cfg(feature = "runtime")]
    #[test]
    fn hall_flash_migrates_legacy_prefers_v1_and_never_falls_back_from_invalid_new_records() {
        use sequential_storage::cache::Cache;
        use sequential_storage::map::{MapConfig, MapStorage};
        use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
        fn ready<T>(future: impl Future<Output = T>) -> T {
            let mut future = core::pin::pin!(future);
            let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
            match future.as_mut().poll(&mut cx) {
                core::task::Poll::Ready(v) => v,
                core::task::Poll::Pending => panic!("mock flash unexpectedly yielded"),
            }
        }
        type Flash = MockFlashBase<4, 4, 256>;
        let mut storage = MapStorage::new(
            Flash::new(WriteCountCheck::TwiceWithZero, None, true),
            MapConfig::new(Flash::FULL_FLASH_RANGE),
            Cache::new_uncached(),
        );
        let mut buf = [0; 128];
        let absent = ready(load_all(&mut storage, &mut buf));
        assert!(absent.hall_calibration.is_none() && absent.hall_calibration_is_valid());
        let old = HallCalibrationConfig::default();
        let legacy = LegacyHallCalibrationConfig {
            angles: old.angles,
            valid: old.valid,
        };
        ready(storage.store_item(&mut buf, &ConfigKey::HallCalibration, &legacy)).unwrap();
        let loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.hall_calibration, Some(old));
        assert!(loaded.hall_calibration_is_valid());
        let new = measured_hall_config();
        ready(storage.store_item(
            &mut buf,
            &ConfigKey::HallCalibrationRecord,
            &HallCalibrationRecord::from(new.clone()),
        ))
        .unwrap();
        assert_eq!(
            ready(load_all(&mut storage, &mut buf)).hall_calibration,
            Some(new.clone())
        );
        // A present unknown version shadows the still-valid legacy key.
        ready(storage.store_item(&mut buf, &ConfigKey::HallCalibrationRecord, &99u8)).unwrap();
        let invalid = ready(load_all(&mut storage, &mut buf));
        assert!(invalid.hall_calibration.is_none() && invalid.hall_calibration_invalid);
        // A decodable record with invalid geometry must fail the same way.
        let mut bad = new;
        bad.geometry.as_mut().unwrap().next_states[3] = 3;
        ready(storage.store_item(
            &mut buf,
            &ConfigKey::HallCalibrationRecord,
            &HallCalibrationRecord::from(bad),
        ))
        .unwrap();
        let invalid = ready(load_all(&mut storage, &mut buf));
        assert!(invalid.hall_calibration.is_none() && invalid.hall_calibration_invalid);
        // A truncated V1 also cannot become a missing record.
        ready(storage.store_item(&mut buf, &ConfigKey::HallCalibrationRecord, &0u8)).unwrap();
        let invalid = ready(load_all(&mut storage, &mut buf));
        assert!(invalid.hall_calibration.is_none() && invalid.hall_calibration_invalid);
        ready(storage.erase_all()).unwrap();
        let erased = ready(load_all(&mut storage, &mut buf));
        assert!(erased.hall_calibration.is_none() && erased.hall_calibration_is_valid());
        ready(storage.store_item(&mut buf, &ConfigKey::HallCalibration, &0u8)).unwrap();
        let legacy_invalid = ready(load_all(&mut storage, &mut buf));
        assert!(
            legacy_invalid.hall_calibration.is_none() && legacy_invalid.hall_calibration_invalid
        );
    }

    #[test]
    fn feedforward_policy_appends_storage_and_wire_ids_and_defaults_disabled() {
        use crate::types::{ConfigGroupId, ConfigValue, ConfigWrite};
        assert_eq!(
            VoltageFeedforwardConfig::default().mode,
            VoltageFeedforwardMode::Disabled
        );
        assert!(RuntimeConfig::default().voltage_feedforward.is_none());
        let mut key = [0; 1];
        ConfigKey::VoltageFeedforward
            .serialize_into(&mut key)
            .unwrap();
        assert_eq!(key, [13]);
        assert_eq!(
            ConfigKey::deserialize_from(&key).unwrap().0,
            ConfigKey::VoltageFeedforward
        );
        let mut buf = [0; 32];
        for (index, group) in ConfigGroupId::ALL.into_iter().enumerate() {
            assert_eq!(
                postcard::to_slice(&group, &mut buf).unwrap(),
                &[u8::try_from(index).unwrap()]
            );
            assert_eq!(group.index(), index);
        }
        for (tag, mode) in [
            VoltageFeedforwardMode::Disabled,
            VoltageFeedforwardMode::CrossCoupling,
            VoltageFeedforwardMode::BackEmf,
            VoltageFeedforwardMode::Both,
        ]
        .into_iter()
        .enumerate()
        {
            let value = VoltageFeedforwardConfig { mode };
            let tag = u8::try_from(tag).unwrap();
            assert_eq!(postcard::to_slice(&value, &mut buf).unwrap(), &[tag]);
            assert_eq!(
                postcard::from_bytes::<VoltageFeedforwardConfig>(&[tag]).unwrap(),
                value
            );
            assert_eq!(
                postcard::to_slice(&ConfigWrite::VoltageFeedforward(value), &mut buf).unwrap(),
                &[12, tag]
            );
            assert_eq!(
                postcard::to_slice(&ConfigValue::VoltageFeedforward(value), &mut buf).unwrap(),
                &[12, tag]
            );
        }
        assert!(postcard::from_bytes::<VoltageFeedforwardConfig>(&[4]).is_err());
    }

    #[test]
    fn board_calibration_validation_and_absolute_scale() {
        let correction = BoardCalibrationConfig {
            vbus_gain: 1.009725,
        };
        assert!(correction.is_sane());
        assert_eq!(correction.correct_vbus_mv(20_000), 20_195);
        assert_eq!(
            BoardCalibrationConfig::default().correct_vbus_mv(20_000),
            20_000
        );
        for gain in [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            0.0,
            -1.0,
            0.949,
            1.051,
        ] {
            assert!(!BoardCalibrationConfig { vbus_gain: gain }.is_sane());
        }
    }

    #[test]
    fn board_calibration_v1_layout_is_frozen() {
        let record = StoredBoardCalibration::from(BoardCalibrationConfig::IDENTITY);
        let mut buf = [0; 16];
        let encoded = postcard::to_slice(&record, &mut buf).unwrap();
        assert_eq!(encoded, &[0, 0, 0, 128, 63]); // V1 tag + f32(1.0), LE
        let decoded: StoredBoardCalibration = postcard::from_bytes(encoded).unwrap();
        assert_eq!(decoded.validated(), Some(BoardCalibrationConfig::IDENTITY));
        assert!(postcard::from_bytes::<StoredBoardCalibration>(&[1, 0, 0, 128, 63]).is_err());
        let bad = StoredBoardCalibration::from(BoardCalibrationConfig {
            vbus_gain: f32::NAN,
        });
        assert!(bad.validated().is_none());
    }

    #[cfg(feature = "runtime")]
    #[test]
    fn board_calibration_flash_load_distinguishes_missing_valid_and_invalid() {
        use sequential_storage::cache::Cache;
        use sequential_storage::map::{MapConfig, MapStorage};
        use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
        // The mock storage futures complete synchronously; no executor/dependency needed.
        fn ready<T>(future: impl Future<Output = T>) -> T {
            let mut future = core::pin::pin!(future);
            let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
            match future.as_mut().poll(&mut cx) {
                core::task::Poll::Ready(v) => v,
                core::task::Poll::Pending => panic!("mock flash unexpectedly yielded"),
            }
        }
        type Flash = MockFlashBase<4, 4, 256>;
        let flash = Flash::new(WriteCountCheck::TwiceWithZero, None, true);
        let mut storage = MapStorage::new(
            flash,
            MapConfig::new(Flash::FULL_FLASH_RANGE),
            Cache::new_uncached(),
        );
        let mut buf = [0; 128];
        let absent = ready(load_all(&mut storage, &mut buf));
        assert_eq!(absent.board_calibration, None);
        assert_eq!(absent.voltage_feedforward, None);
        assert!(!absent.board_calibration_invalid);
        let limits = CurrentLimitsConfig::default();
        ready(storage.store_item(&mut buf, &ConfigKey::CurrentLimits, &limits)).unwrap();
        let policy = VoltageFeedforwardConfig {
            mode: VoltageFeedforwardMode::Both,
        };
        ready(storage.store_item(&mut buf, &ConfigKey::VoltageFeedforward, &policy)).unwrap();
        let value = BoardCalibrationConfig {
            vbus_gain: 1.009725,
        };
        ready(storage.store_item(
            &mut buf,
            &ConfigKey::BoardCalibration,
            &StoredBoardCalibration::from(value),
        ))
        .unwrap();
        let loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.board_calibration, Some(value));
        assert!(!loaded.board_calibration_invalid);
        assert_eq!(loaded.current_limits, Some(limits.clone()));
        assert_eq!(loaded.voltage_feedforward, Some(policy));
        // Unknown version is an error, not an absent record or an identity fallback.
        ready(storage.store_item(&mut buf, &ConfigKey::BoardCalibration, &99u8)).unwrap();
        let invalid = ready(load_all(&mut storage, &mut buf));
        assert_eq!(invalid.board_calibration, None);
        assert!(invalid.board_calibration_invalid);
        assert_eq!(invalid.current_limits, Some(limits));
        ready(storage.erase_all()).unwrap();
        let erased = ready(load_all(&mut storage, &mut buf));
        assert!(!erased.board_calibration_invalid);
        assert_eq!(erased.voltage_feedforward, None);
    }

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
            |p: &mut MotorParamsConfig| p.resistance_ohm = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_d_h = f32::NAN,
            |p: &mut MotorParamsConfig| p.inductance_d_h = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_q_h = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_q_h = 0.0,
            |p: &mut MotorParamsConfig| p.flux_linkage_wb = f32::NAN,
            |p: &mut MotorParamsConfig| p.flux_linkage_wb = f32::INFINITY,
        ] {
            let mut bad = good.clone();
            f(&mut bad);
            assert!(!bad.is_valid(), "must reject {bad:?}");
            #[cfg(feature = "algorithms")]
            {
                use crate::foc::controller::FocController;
                use crate::foc::pwm::SvpwmModulator;
                let mut controller = FocController::<SvpwmModulator>::from_runtime_config(
                    &RuntimeConfig {
                        motor_params: Some(bad),
                        ..Default::default()
                    },
                    24.0,
                );
                let out = controller.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, 0.00005);
                assert!(out.vd.is_finite() && out.vq.is_finite());
                assert!(controller.decoupling().is_none());
            }
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
            |c: &mut CurrentLimitsConfig| c.max_brake_a = f32::NAN,
            |c: &mut CurrentLimitsConfig| c.max_phase_current_a = f32::INFINITY,
            |c: &mut CurrentLimitsConfig| c.bus_in_max_a = f32::NAN,
            |c: &mut CurrentLimitsConfig| c.bus_regen_max_a = f32::NAN,
            |c: &mut CurrentLimitsConfig| c.max_dissipation_w = f32::NAN,
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
    fn hall_tuning_rejects_a_lagging_rate_limit() {
        let ok = HallTuningConfig::default();
        assert!(ok.is_sane());
        // Below 1 the rate limiter holds the angle behind a steady rotor.
        assert!(
            !HallTuningConfig {
                rate_limit_factor: 0.01,
                ..ok
            }
            .is_sane()
        );
        assert!(
            !HallTuningConfig {
                rate_limit_factor: f32::NAN,
                ..ok
            }
            .is_sane()
        );
        assert!(
            !HallTuningConfig {
                interp_min_erpm: -1.0,
                ..ok
            }
            .is_sane()
        );
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
