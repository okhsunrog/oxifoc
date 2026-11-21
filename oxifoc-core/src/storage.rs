//! Persistent configuration storage types.
//!
//! Defines the config group structs and key enum used with
//! [`sequential_storage::map`] for flash-backed key-value storage.
//!
//! Platform crates provide the flash driver and storage task;
//! this module provides the shared type definitions.
//!
//! All value types implement [`PostcardValue`](sequential_storage::map::PostcardValue) via the marker trait,
//! giving them automatic postcard serialization for sequential-storage.

#[cfg(any(feature = "runtime", test))]
use crate::drive_setup::ValidatedDriveSetup;
#[cfg(any(feature = "runtime", test))]
use crate::drive_setup::record::SetupRecord;
use crate::foc::detection::pi_tuning::CurrentIntegralGain;
use crate::foc::hall_calibration::{HallCalibrationResult, HallTransitionGeometry};
use crate::foc::phase::PhaseSource;
pub use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::dissipation::heat_current_limit;
use crate::motor::failsafe::FailsafeConfig;
pub use crate::motor::{derating::DeratingConfig, drive::DriveConfig};
#[cfg(all(feature = "runtime", feature = "storage"))]
use crate::state;
pub use crate::types::VoltageFeedforwardMode;
use crate::types::{ConfigGroupId, ConfigValue};
use crate::units::{
    AngularVelocity, CurrentRate, ElectricCurrent, ElectricalAngle, ElectricalAngularAcceleration,
    ElectricalAngularVelocity, ElectricalResistance, Inductance, MagneticFlux, Power, Sign, amps,
    erad, erad_per_s, erpm, ohms, quantity, rad_per_s, unit::electrical, watts,
};
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

/// Declares [`ConfigKey`] from one tag table, so `tag` and `from_tag` cannot
/// drift apart (a repeated tag is an unreachable pattern in `from_tag`).
macro_rules! config_keys {
    ($($(#[$meta:meta])* $name:ident = $tag:literal,)+) => {
        /// Keys for configuration groups stored in flash.
        ///
        /// Each key maps to a small struct serialized with postcard. Tags are
        /// never reused: a superseded layout keeps its tag as `ReservedN` and is
        /// never read. A tag this firmware does not know (written by an older or
        /// newer firmware) decodes to [`Self::Unknown`] with its byte, so a scan
        /// skips the item instead of failing and flash compaction keeps the
        /// newest item of each such tag. Firmware before this rule (ICD 23 and
        /// older) fails every fetch once the map holds a tag it does not know:
        /// roll back to it only with that firmware's own storage image.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum ConfigKey {
            $($(#[$meta])* $name,)+
            /// A tag this firmware does not know, kept byte-exact.
            Unknown(u8),
        }

        impl ConfigKey {
            /// The one-byte tag stored in flash.
            pub const fn tag(self) -> u8 {
                match self {
                    $(Self::$name => $tag,)+
                    Self::Unknown(tag) => tag,
                }
            }

            /// Decode a stored tag; unknown tags are kept as [`Self::Unknown`].
            pub const fn from_tag(tag: u8) -> Self {
                match tag {
                    $($tag => Self::$name,)+
                    other => Self::Unknown(other),
                }
            }
        }
    };
}

config_keys! {
    /// Superseded motor parameters with zero-valued "unknown" fields. The
    /// tag stays recognizable during flash compaction; the value is never
    /// read, so a firmware with the `Option` layout starts from defaults.
    Reserved1 = 1,
    /// Superseded Hall sector angles without edge geometry. Never read: a
    /// missing [`Self::HallCalibrationRecord`] means "not calibrated", not
    /// an older calibration.
    Reserved2 = 2,
    /// Current sensor DC offsets
    DcOffsets = 3,
    /// Superseded current limits with negative/zero "unset" sentinels.
    /// Never read; see [`Self::Reserved1`].
    Reserved4 = 4,
    /// Voltage limits (min/max VBUS)
    VoltageLimits = 5,
    /// PWM configuration
    PwmConfig = 6,
    /// PI controller gains
    PiGains = 7,
    /// Hall interpolation tuning
    HallTuning = 8,
    /// Superseded failsafe config with a zero "general limit" brake current.
    /// Never read; see [`Self::Reserved1`].
    Reserved9 = 9,
    /// Superseded velocity tuning with a zero/negative "no ramp" sentinel.
    /// Never read; see [`Self::Reserved1`].
    Reserved10 = 10,
    /// Reserved key tag. Keep it recognizable during flash compaction.
    Reserved11 = 11,
    /// Per-device analog measurement corrections (versioned payload).
    BoardCalibration = 12,
    /// Explicit current-loop voltage feedforward policy.
    VoltageFeedforward = 13,
    /// Complete versioned Hall calibration, including optional edge geometry.
    HallCalibrationRecord = 14,
    /// Superseded installed-drive records embedding the sentinel layouts.
    /// Never read; see [`Self::Reserved1`].
    Reserved15 = 15,
    MotorTemperature = 16,
    /// Reserved key tag. Keep it recognizable during flash compaction.
    Reserved17 = 17,
    /// Reserved for the v19 drive and derating layouts.
    Reserved18 = 18,
    Reserved19 = 19,
    Drive = 20,
    /// Superseded derating with zero "off" thresholds. Never read; see
    /// [`Self::Reserved1`].
    Reserved21 = 21,
    MotorFeedback = 22,
    /// Motor electrical parameters (R, L, λ, pole pairs)
    MotorParams = 23,
    /// Current limits (Iq, phase current, supply, dissipation)
    CurrentLimits = 24,
    /// Command-staleness deadman + failsafe policy
    Failsafe = 25,
    InstalledDrive = 26,
    /// Cruise velocity-loop tuning (`accel_limit: None` = no ramp)
    Velocity = 27,
    /// Graduated derating (`None` ramps and ceilings are off)
    Derating = 28,
}

impl Key for ConfigKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let Some(first) = buffer.first_mut() else {
            return Err(SerializationError::BufferTooSmall);
        };
        *first = self.tag();
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        let Some(&tag) = buffer.first() else {
            return Err(SerializationError::BufferTooSmall);
        };
        Ok((Self::from_tag(tag), 1))
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
    pub resistance: ElectricalResistance,
    /// d-axis inductance (H)
    pub inductance_d: Inductance,
    /// q-axis inductance (H)
    pub inductance_q: Inductance,
    /// Permanent-magnet flux linkage (Wb)
    pub flux_linkage: MagneticFlux,
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Optional manual continuous-current ceiling. `None` derives the
    /// rating from resistance and the motor's copper-loss budget, if known.
    pub max_current: Option<ElectricCurrent>,
    /// Power-dissipation class the rating was solved for — the
    /// "motor size" chosen at detection (VESC wizard equivalent),
    /// persisted so re-detection and derived defaults reuse it.
    pub max_power_loss: Option<Power>,
    /// Fundamental (voltage-pulse) d-axis inductance — the
    /// dq-DECOUPLING value. Distinct from `inductance_d`, which is the
    /// AC/high-frequency plateau the ESTIMATION chain and PI gains use
    /// (the two-inductance rule; on eddy-current-heavy motors they
    /// differ several-fold — ZD2808: 86/129 µH fundamental vs ~24 µH
    /// AC). `None` → decoupling falls back to the AC values.
    pub ld_fundamental: Option<Inductance>,
    /// Fundamental (voltage-pulse) q-axis inductance — see
    /// [`Self::ld_fundamental`].
    pub lq_fundamental: Option<Inductance>,
}

impl PostcardValue<'_> for MotorParamsConfig {}

impl MotorParamsConfig {
    /// Check if parameters are valid (have been calibrated)
    pub fn is_valid(&self) -> bool {
        // Positive infinity passes `> 0`; reject it before deriving PI
        // gains and observer state at either the config boundary or boot.
        self.resistance.is_finite()
            && self.inductance_d.is_finite()
            && self.inductance_q.is_finite()
            && self.resistance.is_positive()
            && self.inductance_d.is_positive()
            && self.inductance_q.is_positive()
            && self.flux_linkage.is_finite()
            && self.flux_linkage.is_non_negative()
            && self.pole_pairs > 0
    }

    /// Fundamental d/q inductances for the dq-decoupling feedforward,
    /// when both are measured and usable (see [`Self::ld_fundamental`]);
    /// callers otherwise fall back to the AC values.
    pub fn fundamental_ld_lq(&self) -> Option<(Inductance, Inductance)> {
        let usable = |l: Option<Inductance>| l.filter(|l| l.is_finite() && l.is_positive());
        Some((usable(self.ld_fundamental)?, usable(self.lq_fundamental)?))
    }

    /// The continuous-current estimate from the configured copper-loss budget.
    pub fn thermal_current(&self) -> Option<ElectricCurrent> {
        let loss = self.max_power_loss?;
        if self.resistance.is_finite()
            && self.resistance.is_positive()
            && loss.is_finite()
            && loss.is_positive()
        {
            let current = heat_current_limit(loss, self.resistance);
            (current.is_finite() && current.is_positive()).then_some(current)
        } else {
            None
        }
    }

    /// Continuous current allowed by both the estimate and a manual ceiling.
    pub fn rating_current(&self) -> Option<ElectricCurrent> {
        let thermal = self.thermal_current();
        match self
            .max_current
            .filter(|a| a.is_finite() && a.is_positive())
        {
            Some(ceiling) => Some(thermal.map_or(ceiling, |a| a.min(ceiling))),
            None => thermal,
        }
    }
}

/// Untrusted Hall geometry on the wire. Convert through the validated RAM
/// constructor before installing it in an estimator.
pub use crate::types::measurement::HallTransitionGeometryConfig;

impl HallTransitionGeometryConfig {
    pub fn validated(&self, centroids: [ElectricalAngle; 8]) -> Option<HallTransitionGeometry> {
        HallTransitionGeometry::try_from_raw_parts(self.entry_angles, self.next_states, centroids)
    }
}

impl From<&HallTransitionGeometry> for HallTransitionGeometryConfig {
    fn from(value: &HallTransitionGeometry) -> Self {
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
// A quantity serializes as its f32 SI value, so typed angle fields keep the
// frozen bytes (pinned by `hall_record_freezes_bytes_and_wire_ids`).
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
    angles: [ElectricalAngle; 8],
    valid: [bool; 8],
    geometry: Option<HallTransitionGeometryV1>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg(any(feature = "runtime", test))]
struct HallTransitionGeometryV1 {
    entry_angles: [ElectricalAngle; 8],
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

impl From<&HallCalibrationResult> for HallCalibrationConfig {
    fn from(result: &HallCalibrationResult) -> Self {
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
            ]
            .map(erad),
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
    /// Maximum q-axis (torque) current target. `None` inherits the motor
    /// rating or the board budget.
    pub max_iq: Option<ElectricCurrent>,
    /// Maximum braking q current (magnitude): the torque limit whenever
    /// iq opposes the rotation — remote braking, the velocity loop, the
    /// failsafe stop. Separate from `max_iq` because braking has its own
    /// reasons to be softer (rider balance, traction, belt skip), and at low
    /// speed the battery regen limit hardly bounds it. `None` = same as
    /// `max_iq`. VESC's `l_current_min`.
    pub max_brake: Option<ElectricCurrent>,
    /// Maximum instantaneous phase current. `None` inherits the board's
    /// trip.
    pub max_phase_current: Option<ElectricCurrent>,
    /// Maximum supply (bus) current draw. `None` = unlimited.
    /// VESC's `l_in_current_max`: protects the battery/PSU and wiring —
    /// at low duty a large phase current is a small bus current.
    pub bus_in_max: Option<ElectricCurrent>,
    /// Maximum regen (charge) current pushed back into the supply
    /// (positive magnitude). `None` = unlimited, **zero = no regen at all**
    /// — the safe setting for a lab PSU, which cannot absorb reverse
    /// current. VESC's `l_in_current_min` (negated).
    pub bus_regen_max: Option<ElectricCurrent>,
    /// Dissipative braking: regen the supply refuses (the
    /// regen-overvoltage derate, `bus_regen_max`) is burned in the
    /// windings — up to this much heat from the extra d current,
    /// `1.5·R·id²` — instead of cutting the brake torque
    /// (`motor::dissipation`). The d current stays inside the `max_iq`
    /// circle, so the total copper loss never exceeds the motor's rating
    /// (`MotorParamsConfig::max_power_loss`); this sets how much of it a
    /// long descent may use. Needs detected motor params (R, λ).
    /// Zero = off.
    pub max_dissipation: Power,
}

impl PostcardValue<'_> for CurrentLimitsConfig {}

impl CurrentLimitsConfig {
    /// Boundary validation for host writes: every set limit finite and
    /// positive (the regen limit may be zero), and when both phase-side limits are set the
    /// overcurrent trip must clear the iq ceiling by
    /// [`OVERCURRENT_HEADROOM`] — otherwise a legitimate full-throttle
    /// command (target + PI overshoot + HFI ripple) lives inside the Kill
    /// band and trips mid-ride. The config server rejects an incoherent
    /// write loudly (`ConfigResponse::Invalid`) so the user learns the rule;
    /// `CurrentLimits::from_config_clamped` additionally clamps whatever
    /// arrives by other paths (baked/boot configs).
    ///
    /// Unset limits are always coherent — the ceilings fill in with proper
    /// headroom.
    ///
    /// [`OVERCURRENT_HEADROOM`]: crate::motor::foc_driver::OVERCURRENT_HEADROOM
    pub fn is_coherent(&self) -> bool {
        use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
        // A set limit is a real, positive bound; only the regen limit may
        // be zero (no charging at all).
        let positive =
            |limit: Option<ElectricCurrent>| limit.is_none_or(|a| a.is_finite() && a.is_positive());
        let values = positive(self.max_iq)
            && positive(self.max_brake)
            && positive(self.max_phase_current)
            && positive(self.bus_in_max)
            && self
                .bus_regen_max
                .is_none_or(|a| a.is_finite() && a.is_non_negative())
            && self.max_dissipation.is_finite();
        let headroom_ok = match (self.max_iq, self.max_phase_current) {
            (Some(iq), Some(trip)) => trip >= OVERCURRENT_HEADROOM * iq,
            _ => true,
        };
        values && headroom_ok
    }
}

impl CurrentLimitsConfig {
    /// All configurable limits inherit the board or motor budget.
    pub const AUTO: Self = Self {
        max_iq: None,
        max_brake: None,
        max_phase_current: None,
        bus_in_max: None,
        bus_regen_max: None,
        max_dissipation: watts(0.0),
    };
}

impl Default for CurrentLimitsConfig {
    fn default() -> Self {
        Self {
            max_iq: Some(amps(10.0)),
            max_phase_current: Some(amps(40.0)),
            ..Self::AUTO
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

/// Board limits for the PWM settings that can be stored for the next boot.
#[derive(Clone, Copy)]
pub struct PwmConfigLimits {
    pub frequencies_hz: &'static [u32],
    /// The duty ceiling is fixed by the board profile.
    pub max_duty_percent: u8,
}

impl PwmConfigLimits {
    pub fn accepts(self, config: &PwmConfigStored) -> bool {
        self.frequencies_hz.contains(&config.freq_hz)
            && config.max_duty_percent == self.max_duty_percent
    }
}

/// PI controller gains.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PiGainsConfig {
    /// Proportional gain (V/A)
    pub kp: ElectricalResistance,
    /// Integral gain (V/(A·s))
    pub ki: CurrentIntegralGain,
    /// Bandwidth used to compute gains (rad/s), for reference
    pub bandwidth: AngularVelocity,
}

impl PiGainsConfig {
    /// Gate for every consumer (config boundary, boot): the gains multiply
    /// the current error straight into the output voltage, and the dq
    /// software overcurrent check on the resulting NaN currents compares
    /// false — a NaN/zero kp latches a dead or unstable loop that no
    /// downstream protection catches.
    pub fn is_sane(&self) -> bool {
        self.kp.is_finite()
            && self.ki.is_finite()
            && self.kp.is_positive()
            && self.ki.is_non_negative()
    }
}

impl PostcardValue<'_> for PiGainsConfig {}

impl Default for PiGainsConfig {
    fn default() -> Self {
        Self {
            kp: ohms(0.4),
            ki: quantity(40.0),
            bandwidth: rad_per_s(1000.0),
        }
    }
}

/// Hall sensor interpolation tuning.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallTuningConfig {
    /// Minimum eRPM for angle interpolation
    pub interp_min_speed: ElectricalAngularVelocity,
    /// Rate limit factor (1.0 = no overshoot allowed)
    pub rate_limit_factor: f32,
}

impl PostcardValue<'_> for HallTuningConfig {}

impl Default for HallTuningConfig {
    fn default() -> Self {
        Self {
            interp_min_speed: erpm(500.0),
            rate_limit_factor: 1.5,
        }
    }
}

impl HallTuningConfig {
    /// Usable values: a finite, non-negative interpolation floor, and a rate
    /// limit that lets the angle keep up with the rotor (a factor below 1
    /// holds the commutation angle behind it at any steady speed).
    pub fn is_sane(&self) -> bool {
        (0.0..=100_000.0).contains(&self.interp_min_speed.get::<electrical::erpm>())
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
    /// Brake current of the stop, within the general `max_brake`;
    /// `None` = the general limit alone.
    pub brake_current: Option<ElectricCurrent>,
    /// Rate (A/s) at which the stop changes the q current.
    pub slew: CurrentRate,
    /// |ω_e| (electrical rad/s) treated as stopped.
    pub standstill: ElectricalAngularVelocity,
    /// Upper bound on the stop's deceleration (electrical rad/s²).
    pub decel: ElectricalAngularAcceleration,
    /// Clean-stop terminal, as `FailsafeTerminal as u8` (0=HighZ,
    /// 1=ParkBrake; unknown → HighZ).
    pub terminal: u8,
}

impl PostcardValue<'_> for FailsafeConfigStored {}

impl PostcardValue<'_> for VelocityLoopConfig {}

impl FailsafeConfigStored {
    pub fn is_sane(&self) -> bool {
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy, FailsafeTerminal};
        self.policy <= FailsafePolicy::ControlledStop as u8
            && self.terminal <= FailsafeTerminal::ParkBrake as u8
            && FailsafeConfig::from(self).is_sane()
    }
}

impl Default for FailsafeConfigStored {
    fn default() -> Self {
        let config = FailsafeConfig::default();
        Self {
            staleness_timeout_ms: (config.staleness_timeout_us / 1_000) as u32,
            policy: config.policy as u8,
            brake_current: config.brake_current,
            slew: config.slew,
            standstill: config.standstill,
            decel: config.decel,
            terminal: config.terminal as u8,
        }
    }
}

impl PostcardValue<'_> for DeratingConfig {}

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
/// Motor temperature input selection. The board owns the ADC and sensor curve.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorTemperatureSensor {
    #[default]
    BoardDefault,
    Disabled,
}

#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorTemperatureConfig {
    pub sensor: MotorTemperatureSensor,
}

impl PostcardValue<'_> for MotorTemperatureConfig {}

impl PostcardValue<'_> for DriveConfig {}

/// Commissioned commutation choices. Diagnostic sources remain transient.
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(rename_all = "kebab-case")]
pub enum MotorFeedbackMode {
    #[default]
    Auto,
    // Keep the stored discriminants when renaming these two Hall policies.
    HallWithoutCrossover,
    Observer,
    #[serde(alias = "hall-to-observer")]
    Hall,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, postcard_schema::Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[serde(deny_unknown_fields)]
pub struct MotorFeedbackConfig {
    pub mode: MotorFeedbackMode,
    pub blend_low: ElectricalAngularVelocity,
    pub blend_high: ElectricalAngularVelocity,
}

impl Default for MotorFeedbackConfig {
    fn default() -> Self {
        Self {
            mode: MotorFeedbackMode::Auto,
            blend_low: erad_per_s(150.0),
            blend_high: erad_per_s(300.0),
        }
    }
}

impl MotorFeedbackConfig {
    pub fn is_sane(self) -> bool {
        self.blend_low.is_finite()
            && self.blend_high.is_finite()
            && self.blend_low.is_non_negative()
            && self.blend_high > self.blend_low
    }

    pub(crate) fn source(self, config: &RuntimeConfig) -> Option<PhaseSource> {
        let hall = config.hall_calibration_is_valid()
            && config
                .hall_calibration
                .as_ref()
                .is_some_and(HallCalibrationConfig::is_calibrated);
        self.source_for(hall, config.motor_params.as_ref())
    }

    fn source_for(self, hall: bool, motor: Option<&MotorParamsConfig>) -> Option<PhaseSource> {
        use crate::foc::phase::PhaseSource;
        if !self.is_sane() {
            return None;
        }
        let observer =
            motor.is_some_and(|motor| motor.is_valid() && motor.flux_linkage.is_positive());
        let hall_with_observer = PhaseSource::HallToObserver {
            blend_low: self.blend_low,
            blend_high: self.blend_high,
        };
        match self.mode {
            MotorFeedbackMode::Auto | MotorFeedbackMode::Hall if hall && observer => {
                Some(hall_with_observer)
            }
            MotorFeedbackMode::Auto if hall => Some(PhaseSource::Hall),
            MotorFeedbackMode::Auto if observer => Some(PhaseSource::Observer),
            MotorFeedbackMode::Auto => Some(PhaseSource::Manual),
            MotorFeedbackMode::HallWithoutCrossover if hall => Some(PhaseSource::Hall),
            MotorFeedbackMode::Observer if observer => Some(PhaseSource::Observer),
            _ => None,
        }
    }
}

impl PostcardValue<'_> for MotorFeedbackConfig {}

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
    pub velocity: Option<VelocityLoopConfig>,
    pub derating: Option<DeratingConfig>,
    /// Missing records retain plain PI control.
    pub voltage_feedforward: Option<VoltageFeedforwardConfig>,
    pub motor_temperature: Option<MotorTemperatureConfig>,
    pub drive: Option<DriveConfig>,
    pub drive_invalid: bool,
    pub motor_feedback: Option<MotorFeedbackConfig>,
    pub motor_feedback_invalid: bool,
    /// Groups whose stored record failed to read at boot (one bit per
    /// `ConfigGroupId::index`). Their fields hold the board defaults until
    /// the group is written again.
    pub corrupt_records: u32,
}

impl RuntimeConfig {
    /// Nothing loaded: every group on its board default. Equal to
    /// `Default::default()`, usable in a `static` initializer.
    pub const fn new() -> Self {
        Self {
            board_calibration: None,
            board_calibration_invalid: false,
            motor_params: None,
            hall_calibration: None,
            hall_calibration_invalid: false,
            dc_offsets: None,
            current_limits: None,
            voltage_limits: None,
            pwm_config: None,
            pi_gains: None,
            hall_tuning: None,
            failsafe: None,
            velocity: None,
            derating: None,
            voltage_feedforward: None,
            motor_temperature: None,
            drive: None,
            drive_invalid: false,
            motor_feedback: None,
            motor_feedback_invalid: false,
            corrupt_records: 0,
        }
    }

    /// Groups a corrupt record must not silently replace with defaults: the
    /// motor stays stopped until they are written again.
    const SAFETY_RECORDS: [ConfigGroupId; 3] = [
        ConfigGroupId::CurrentLimits,
        ConfigGroupId::VoltageLimits,
        ConfigGroupId::Failsafe,
    ];

    pub(crate) fn mark_corrupt(&mut self, group: ConfigGroupId) {
        self.corrupt_records |= 1 << group.index();
    }

    /// A successful write replaces the unreadable record.
    pub(crate) fn mark_rewritten(&mut self, group: ConfigGroupId) {
        self.corrupt_records &= !(1 << group.index());
    }

    pub fn record_is_valid(&self, group: ConfigGroupId) -> bool {
        self.corrupt_records & (1 << group.index()) == 0
    }

    pub fn has_corrupt_records(&self) -> bool {
        self.corrupt_records != 0
    }

    /// Current/voltage limits and the failsafe either loaded or are unset.
    pub fn safety_records_valid(&self) -> bool {
        Self::SAFETY_RECORDS
            .iter()
            .all(|&group| self.record_is_valid(group))
    }

    /// Resolve the installed policy after loading the motor and Hall records.
    pub fn boot_feedback_source(&self) -> PhaseSource {
        self.configured_feedback_source().unwrap_or_else(|| {
            MotorFeedbackConfig::default()
                .source(self)
                .expect("automatic feedback always resolves")
        })
    }

    pub(crate) fn configured_feedback_source(&self) -> Option<PhaseSource> {
        use crate::foc::phase::PhaseSource;
        if self.motor_feedback_invalid {
            return Some(PhaseSource::Manual);
        }
        self.motor_feedback
            .map(|policy| policy.source(self).unwrap_or(PhaseSource::Manual))
    }

    pub fn motor_feedback_is_valid(&self) -> bool {
        !self.motor_feedback_invalid
            && self
                .motor_feedback
                .is_none_or(|value| value.source(self).is_some())
    }

    pub fn effective_velocity(&self) -> VelocityLoopConfig {
        self.velocity
            .filter(VelocityLoopConfig::is_sane)
            .unwrap_or_default()
    }

    pub fn effective_derating(&self) -> DeratingConfig {
        self.derating
            .filter(DeratingConfig::is_sane)
            .unwrap_or_default()
    }

    pub fn drive_is_valid(&self) -> bool {
        !self.drive_invalid
    }

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

/// Effective setup and later individually persisted edits live under one key.
/// The legacy keys are read only when an installed drive record is absent.
#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum InstalledDrive {
    V1(InstalledDriveV1),
    V2(InstalledDriveV2),
    V3(InstalledDriveV3),
}

#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InstalledDriveV1 {
    setup: SetupRecord,
    configured_trip: Option<ElectricCurrent>,
    // The legacy records below already have fixed on-flash layouts.
    motor_override: Option<MotorParamsConfig>,
    limits_override: Option<CurrentLimitsConfig>,
    hall_override: Option<HallCalibrationRecord>,
}

/// Canonical installed values. Each edit replaces the field it owns.
#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InstalledDriveV2 {
    motor: MotorParamsConfig,
    hall: Option<HallCalibrationRecord>,
    source: InstalledSourceLimits,
    phase: InstalledPhaseLimits,
    trip: Option<ElectricCurrent>,
}

#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InstalledDriveV3 {
    values: InstalledDriveV2,
    feedback: Option<MotorFeedbackConfig>,
}

#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
struct InstalledSourceLimits {
    input: Option<ElectricCurrent>,
    regen: Option<ElectricCurrent>,
}

#[cfg(any(feature = "runtime", test))]
#[derive(Clone, Debug, Serialize, Deserialize)]
struct InstalledPhaseLimits {
    drive: Option<ElectricCurrent>,
    brake: Option<ElectricCurrent>,
    dissipation: Power,
}

#[cfg(any(feature = "runtime", test))]
impl InstalledDriveV2 {
    fn new(
        motor: MotorParamsConfig,
        hall: Option<HallCalibrationRecord>,
        limits: &CurrentLimitsConfig,
    ) -> Self {
        Self {
            motor,
            hall,
            source: InstalledSourceLimits {
                input: limits.bus_in_max,
                regen: limits.bus_regen_max,
            },
            phase: InstalledPhaseLimits {
                drive: limits.max_iq,
                brake: limits.max_brake,
                dissipation: limits.max_dissipation,
            },
            trip: limits.max_phase_current,
        }
    }

    fn limits(&self) -> CurrentLimitsConfig {
        CurrentLimitsConfig {
            max_iq: self.phase.drive,
            max_brake: self.phase.brake,
            max_phase_current: self.trip,
            bus_in_max: self.source.input,
            bus_regen_max: self.source.regen,
            max_dissipation: self.phase.dissipation,
        }
    }
}

#[cfg(any(feature = "runtime", test))]
impl PostcardValue<'_> for InstalledDrive {}

#[cfg(any(feature = "runtime", test))]
impl InstalledDrive {
    pub fn new(setup: &ValidatedDriveSetup) -> Self {
        Self::V3(InstalledDriveV3 {
            values: InstalledDriveV2::new(
                setup.motor_policy(),
                setup.hall().map(Into::into),
                &setup.current_policy(),
            ),
            feedback: Some(setup.motor_feedback()),
        })
    }

    fn canonical(&self, board_trip: ElectricCurrent) -> Option<InstalledDriveV2> {
        let record = match self {
            Self::V1(record) => record,
            Self::V2(record) => return Some(record.clone()),
            Self::V3(record) => return Some(record.values.clone()),
        };
        let setup = record
            .setup
            .clone()
            .validate(board_trip, record.configured_trip)
            .ok()?;
        Some(InstalledDriveV2::new(
            record
                .motor_override
                .clone()
                .unwrap_or_else(|| setup.motor_policy()),
            record
                .hall_override
                .clone()
                .or_else(|| setup.hall().map(Into::into)),
            &record
                .limits_override
                .clone()
                .unwrap_or_else(|| setup.current_policy()),
        ))
    }

    pub(crate) fn project(&self, config: &mut RuntimeConfig, board_trip: ElectricCurrent) -> bool {
        let Some(record) = self.canonical(board_trip) else {
            return false;
        };
        let limits = record.limits();
        let hall = match record.hall {
            Some(record) => match record.validated() {
                Some(hall) => Some(hall),
                None => return false,
            },
            None => None,
        };
        if !record.motor.is_valid() || !limits.is_coherent() {
            return false;
        }
        config.motor_params = Some(record.motor);
        config.current_limits = Some(limits);
        config.hall_calibration = hall;
        config.hall_calibration_invalid = false;
        config.motor_feedback = self.feedback();
        config.motor_feedback_invalid = false;
        true
    }

    fn feedback(&self) -> Option<MotorFeedbackConfig> {
        match self {
            Self::V3(record) => record.feedback,
            _ => None,
        }
    }

    fn with_edit(&self, payload: &ConfigValue, board_trip: ElectricCurrent) -> Option<Self> {
        let mut next = self.canonical(board_trip)?;
        let mut feedback = self.feedback();
        match payload {
            ConfigValue::MotorParams(value) => next.motor = value.clone(),
            ConfigValue::CurrentLimits(value) => {
                next = InstalledDriveV2::new(next.motor, next.hall, value);
            }
            ConfigValue::HallCalibration(value) => next.hall = Some((*value).into()),
            ConfigValue::MotorFeedback(value) => feedback = Some(*value),
            _ => return None,
        }
        if let Some(policy) = feedback {
            let hall = next.hall.clone().and_then(HallCalibrationRecord::validated);
            policy.source_for(hall.is_some(), Some(&next.motor))?;
        }
        Some(Self::V3(InstalledDriveV3 {
            values: next,
            feedback,
        }))
    }
}

// ============================================================================
// Flash Operation Messages (shared across platforms)
// ============================================================================

/// Messages for flash write operations.
///
/// Sent from protocol servers to the platform storage worker task.
#[derive(Clone, Debug)]
#[cfg_attr(
    feature = "runtime",
    expect(
        clippy::large_enum_variant,
        reason = "The embedded worker queues complete flash records without an allocator."
    )
)]
pub enum FlashOperation {
    /// Save a config group to flash
    Save(ConfigValue),
    /// Erase all stored configuration
    EraseAll,
    #[cfg(feature = "runtime")]
    SaveInstalledDrive(InstalledDrive),
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
    board_trip: ElectricCurrent,
) -> !
where
    F: embedded_storage_async::nor_flash::NorFlash,
{
    // Boot-time: load all stored configs
    let mut cfg = load_all(storage, buf).await;
    let mut installed = storage
        .fetch_item::<InstalledDrive>(buf, &ConfigKey::InstalledDrive)
        .await;
    if !load_installed_drive(&mut cfg, &installed, board_trip) {
        state::set_drive_setup_recovery(true);
    }
    CONFIG_LOADED.signal(cfg);

    // Runtime: handle write operations
    loop {
        let op = FLASH_CHANNEL.receive().await;

        let success = match op {
            FlashOperation::SaveInstalledDrive(record) => {
                if storage
                    .store_item(buf, &ConfigKey::InstalledDrive, &record)
                    .await
                    .is_ok()
                {
                    installed = Ok(Some(record));
                    true
                } else {
                    false
                }
            }
            FlashOperation::Save(value) => {
                if value.group().is_setup_component() && !matches!(&installed, Ok(None)) {
                    let success = if let Ok(Some(current)) = &installed
                        && let Some(next) = current.with_edit(&value, board_trip)
                    {
                        if storage
                            .store_item(buf, &ConfigKey::InstalledDrive, &next)
                            .await
                            .is_ok()
                        {
                            installed = Ok(Some(next));
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    };
                    FLASH_DONE.signal(success);
                    continue;
                }
                let result = match value {
                    ConfigValue::BoardCalibration(v) => {
                        storage
                            .store_item(
                                buf,
                                &ConfigKey::BoardCalibration,
                                &StoredBoardCalibration::from(v),
                            )
                            .await
                    }
                    ConfigValue::MotorParams(v) => {
                        storage.store_item(buf, &ConfigKey::MotorParams, &v).await
                    }
                    ConfigValue::HallCalibration(v) => {
                        storage
                            .store_item(
                                buf,
                                &ConfigKey::HallCalibrationRecord,
                                &HallCalibrationRecord::from(v),
                            )
                            .await
                    }
                    ConfigValue::DcOffsets(v) => {
                        storage.store_item(buf, &ConfigKey::DcOffsets, &v).await
                    }
                    ConfigValue::CurrentLimits(v) => {
                        storage.store_item(buf, &ConfigKey::CurrentLimits, &v).await
                    }
                    ConfigValue::VoltageLimits(v) => {
                        storage.store_item(buf, &ConfigKey::VoltageLimits, &v).await
                    }
                    ConfigValue::PwmConfig(v) => {
                        storage.store_item(buf, &ConfigKey::PwmConfig, &v).await
                    }
                    ConfigValue::PiGains(v) => {
                        storage.store_item(buf, &ConfigKey::PiGains, &v).await
                    }
                    ConfigValue::HallTuning(v) => {
                        storage.store_item(buf, &ConfigKey::HallTuning, &v).await
                    }
                    ConfigValue::Failsafe(v) => {
                        storage.store_item(buf, &ConfigKey::Failsafe, &v).await
                    }
                    ConfigValue::Velocity(v) => {
                        storage.store_item(buf, &ConfigKey::Velocity, &v).await
                    }
                    ConfigValue::Derating(v) => {
                        storage.store_item(buf, &ConfigKey::Derating, &v).await
                    }
                    ConfigValue::Drive(v) => storage.store_item(buf, &ConfigKey::Drive, &v).await,
                    ConfigValue::MotorFeedback(v) => {
                        storage.store_item(buf, &ConfigKey::MotorFeedback, &v).await
                    }
                    ConfigValue::VoltageFeedforward(v) => {
                        storage
                            .store_item(buf, &ConfigKey::VoltageFeedforward, &v)
                            .await
                    }
                    ConfigValue::MotorTemperature(v) => {
                        storage
                            .store_item(buf, &ConfigKey::MotorTemperature, &v)
                            .await
                    }
                };
                if result.is_err() {
                    #[cfg(feature = "defmt")]
                    defmt::error!("Failed to save config");
                }
                result.is_ok()
            }
            FlashOperation::EraseAll => {
                let result = storage.erase_all().await;
                if result.is_ok() {
                    installed = Ok(None);
                }
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

#[cfg(any(feature = "runtime", test))]
fn load_installed_drive<E>(
    config: &mut RuntimeConfig,
    installed: &Result<Option<InstalledDrive>, E>,
    board_trip: ElectricCurrent,
) -> bool {
    match installed {
        Ok(Some(record)) if record.project(config, board_trip) => return true,
        Ok(None) => return true,
        _ => {}
    }
    // A present but unreadable record shadows all three legacy groups.
    config.motor_params = None;
    config.hall_calibration = None;
    config.hall_calibration_invalid = true;
    config.current_limits = None;
    config.motor_feedback = None;
    config.motor_feedback_invalid = true;
    false
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

    // A record that fails to read leaves its field unset (board defaults)
    // and marks the group corrupt; see `RuntimeConfig::safety_records_valid`.
    macro_rules! load {
        ($field:ident, $key:ident) => {
            match storage.fetch_item(buf, &ConfigKey::$key).await {
                Ok(value) => cfg.$field = value,
                Err(_) => cfg.mark_corrupt(ConfigGroupId::$key),
            }
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
    load!(motor_temperature, MotorTemperature);
    match storage
        .fetch_item::<MotorFeedbackConfig>(buf, &ConfigKey::MotorFeedback)
        .await
    {
        Ok(Some(value)) => cfg.motor_feedback = Some(value),
        Ok(None) => {}
        Err(_) => cfg.motor_feedback_invalid = true,
    }
    match storage
        .fetch_item::<DriveConfig>(buf, &ConfigKey::Drive)
        .await
    {
        Ok(Some(value)) => cfg.drive = Some(value),
        Ok(None) => {}
        _ => cfg.drive_invalid = true,
    }

    match storage
        .fetch_item::<HallCalibrationRecord>(buf, &ConfigKey::HallCalibrationRecord)
        .await
    {
        Ok(Some(record)) => {
            cfg.hall_calibration = record.validated();
            cfg.hall_calibration_invalid = cfg.hall_calibration.is_none();
        }
        // The superseded sector-angle record (`Reserved2`) is never read: an
        // older calibration without geometry must not stand in for a missing one.
        Ok(None) => {}
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
    use crate::foc::hall_calibration::HallCalibrationResult;
    #[cfg(feature = "runtime")]
    use crate::motor::derating::{Ramp, TemperatureDerating};
    use crate::units::{
        amps, erad, erad_per_s, erpm, henries, rad_per_s, seconds, volts, watts, webers,
    };
    use crate::units::{ohms, quantity};

    #[test]
    fn feedback_requires_calibration_and_model_and_survives_other_edits() {
        use crate::foc::phase::PhaseSource;
        let feedback = MotorFeedbackConfig {
            mode: MotorFeedbackMode::Hall,
            ..Default::default()
        };
        let mut config = RuntimeConfig {
            motor_feedback: Some(feedback),
            ..Default::default()
        };
        assert!(!config.motor_feedback_is_valid());
        assert_eq!(config.boot_feedback_source(), PhaseSource::Manual);
        let motor = MotorParamsConfig {
            resistance: ohms(0.063),
            inductance_d: henries(31e-6),
            inductance_q: henries(51e-6),
            flux_linkage: webers(0.004),
            pole_pairs: 7,
            ..Default::default()
        };
        let installed = InstalledDrive::V2(InstalledDriveV2::new(
            motor.clone(),
            Some(measured_hall_config().into()),
            &CurrentLimitsConfig::AUTO,
        ));
        let edited = installed
            .with_edit(&ConfigValue::MotorFeedback(feedback), amps(70.0))
            .unwrap()
            .with_edit(&ConfigValue::MotorParams(motor), amps(70.0))
            .unwrap();
        assert!(edited.project(&mut config, amps(70.0)));
        assert!(config.motor_feedback_is_valid());
        assert_eq!(
            config.boot_feedback_source(),
            PhaseSource::HallToObserver {
                blend_low: erad_per_s(150.0),
                blend_high: erad_per_s(300.0),
            }
        );
        assert!(
            !MotorFeedbackConfig {
                blend_high: erad_per_s(100.0),
                ..feedback
            }
            .is_sane()
        );
        config.hall_calibration = None;
        assert!(!config.motor_feedback_is_valid());
        assert_eq!(config.boot_feedback_source(), PhaseSource::Manual);
    }

    #[test]
    fn commissioned_hall_and_auto_resolve_to_the_same_crossover() {
        use crate::drive_setup::{DriveCurrent, DriveSetup, Feedback};
        use crate::foc::phase::PhaseSource;
        use crate::types::commissioning::SourceLimits;

        let setup = DriveSetup {
            motor: MotorParamsConfig {
                resistance: ohms(0.063),
                inductance_d: henries(31e-6),
                inductance_q: henries(51e-6),
                flux_linkage: webers(0.004),
                pole_pairs: 7,
                max_power_loss: Some(watts(120.0)),
                ..Default::default()
            },
            feedback: Feedback::Hall(measured_hall_config()),
            source: SourceLimits {
                input: amps(5.0),
                regen: amps(0.0),
            },
            current: DriveCurrent::Auto,
        }
        .validate(amps(70.0), Some(amps(30.0)))
        .unwrap();
        let mut config = RuntimeConfig::default();
        assert!(InstalledDrive::new(&setup).project(&mut config, amps(70.0)));
        assert_eq!(config.motor_feedback.unwrap().mode, MotorFeedbackMode::Hall);
        let crossover = PhaseSource::HallToObserver {
            blend_low: erad_per_s(150.0),
            blend_high: erad_per_s(300.0),
        };
        assert_eq!(config.boot_feedback_source(), crossover);
        config.motor_feedback = None;
        assert_eq!(config.boot_feedback_source(), crossover);
    }

    #[test]
    fn aggregate_load_shadows_legacy_and_preserves_independently_saved_edits() {
        use crate::drive_setup::{DriveCurrent, DriveSetup, Feedback};
        use crate::types::commissioning::SourceLimits;
        let mut legacy = RuntimeConfig {
            motor_params: Some(MotorParamsConfig {
                resistance: ohms(0.2),
                ..Default::default()
            }),
            hall_calibration: Some(measured_hall_config()),
            current_limits: Some(CurrentLimitsConfig {
                bus_in_max: Some(amps(20.0)),
                ..Default::default()
            }),
            board_calibration: Some(BoardCalibrationConfig { vbus_gain: 1.01 }),
            ..Default::default()
        };
        assert!(load_installed_drive(
            &mut legacy,
            &Ok::<_, ()>(None),
            amps(70.0)
        ));
        assert_eq!(legacy.motor_params.as_ref().unwrap().resistance.value, 0.2);
        let setup = DriveSetup {
            motor: MotorParamsConfig {
                resistance: ohms(0.063),
                inductance_d: henries(31e-6),
                inductance_q: henries(51e-6),
                flux_linkage: webers(0.004),
                pole_pairs: 7,
                max_power_loss: Some(watts(120.0)),
                ..Default::default()
            },
            feedback: Feedback::Sensorless,
            source: SourceLimits {
                input: amps(5.0),
                regen: amps(0.0),
            },
            current: DriveCurrent::Auto,
        }
        .validate(amps(70.0), Some(amps(30.0)))
        .unwrap();
        let installed = InstalledDrive::V1(InstalledDriveV1 {
            setup: (&setup).into(),
            configured_trip: Some(amps(30.0)),
            motor_override: None,
            limits_override: None,
            hall_override: None,
        });
        assert!(load_installed_drive(
            &mut legacy,
            &Ok::<_, ()>(Some(installed.clone())),
            amps(70.0)
        ));
        assert_eq!(legacy.motor_params.as_ref(), Some(&setup.motor_policy()));
        assert_eq!(
            legacy.current_limits.as_ref(),
            Some(&setup.current_policy())
        );
        assert!(legacy.hall_calibration.is_none());
        let mut limits = setup.current_policy();
        limits.bus_in_max = Some(amps(8.0));
        let edited = installed
            .with_edit(&ConfigValue::CurrentLimits(limits.clone()), amps(70.0))
            .unwrap();
        assert!(matches!(edited, InstalledDrive::V3(_)));
        assert!(load_installed_drive(
            &mut legacy,
            &Ok::<_, ()>(Some(edited.clone())),
            amps(70.0)
        ));
        assert_eq!(legacy.current_limits.as_ref(), Some(&limits));
        assert_eq!(legacy.motor_params.as_ref(), Some(&setup.motor_policy()));
        assert!(legacy.hall_calibration.is_none());
        let mut motor = setup.motor_policy();
        motor.resistance.value *= 2.5;
        let edited = edited
            .with_edit(&ConfigValue::MotorParams(motor.clone()), amps(70.0))
            .unwrap();
        assert!(edited.project(&mut legacy, amps(70.0)));
        assert_eq!(legacy.current_limits.as_ref(), Some(&limits));
        let rating = legacy
            .motor_params
            .as_ref()
            .unwrap()
            .rating_current()
            .unwrap();
        assert_near!(
            rating,
            setup.thermal_current() / libm::sqrtf(2.5),
            amps(1e-5)
        );
        #[cfg(feature = "algorithms")]
        {
            use crate::motor::foc_driver::CurrentLimits;
            let effective = CurrentLimits::from_config_clamped(&limits, amps(70.0), Some(rating));
            assert_eq!(effective.max_current(), Some(rating));
            assert_eq!(effective.bus_in_max(), Some(amps(8.0)));
            assert_eq!(effective.bus_regen_max(), Some(amps(0.0)));
            assert_eq!(effective.overcurrent_threshold(), Some(amps(30.0)));
        }
        assert!(!load_installed_drive(&mut legacy, &Err(()), amps(70.0)));
        assert!(legacy.motor_params.is_none() && legacy.current_limits.is_none());
        assert_eq!(legacy.board_calibration.unwrap().vbus_gain, 1.01);
    }

    fn measured_hall_config() -> HallCalibrationConfig {
        use core::f32::consts::TAU;
        HallCalibrationConfig {
            geometry: Some(HallTransitionGeometryConfig {
                entry_angles: [
                    erad(0.0),
                    erad(0.0),
                    erad(TAU / 3.0),
                    erad(TAU / 6.0),
                    erad(2.0 * TAU / 3.0),
                    erad(5.0 * TAU / 6.0),
                    erad(TAU / 2.0),
                    erad(0.0),
                ],
                next_states: [0, 3, 6, 2, 5, 1, 4, 0],
            }),
            ..HallCalibrationConfig::default()
        }
    }

    #[test]
    fn hall_record_freezes_bytes_and_wire_ids() {
        use crate::types::{ConfigGroupId, ConfigValue};
        let uncalibrated_geometry = HallCalibrationConfig {
            angles: [erad(0.0); 8],
            valid: [false, true, true, true, true, true, true, false],
            geometry: None,
        };
        let mut buf = [0; 128];
        let record = HallCalibrationRecord::from(uncalibrated_geometry);
        let mut versioned = [0u8; 42];
        versioned[34..40].fill(1);
        assert_eq!(postcard::to_slice(&record, &mut buf).unwrap(), versioned);
        assert_eq!(
            postcard::from_bytes::<HallCalibrationRecord>(&versioned)
                .unwrap()
                .validated(),
            Some(uncalibrated_geometry)
        );
        versioned[0] = 1;
        assert!(postcard::from_bytes::<HallCalibrationRecord>(&versioned).is_err());
        assert_eq!(
            postcard::to_slice(&ConfigGroupId::HallCalibration, &mut buf).unwrap(),
            &[1]
        );
        let config = measured_hall_config();
        assert_eq!(
            postcard::to_slice(&ConfigValue::HallCalibration(config), &mut buf).unwrap()[0],
            6
        );
        assert_eq!(
            postcard::to_slice(&ConfigValue::HallCalibration(config), &mut buf).unwrap()[0],
            6
        );
        let record = HallCalibrationRecord::from(config);
        let bytes = postcard::to_slice(&record, &mut buf).unwrap();
        assert_eq!(
            postcard::from_bytes::<HallCalibrationRecord>(bytes)
                .unwrap()
                .validated(),
            Some(config)
        );
        let mut key = [0];
        ConfigKey::Reserved2.serialize_into(&mut key).unwrap();
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
        let result = HallCalibrationResult {
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
        let mut bad = good;
        bad.geometry.as_mut().unwrap().next_states[1] = 1;
        assert!(!bad.is_calibrated());
        assert!(HallCalibrationRecord::from(bad).validated().is_none());
        let mut bad = good;
        bad.geometry.as_mut().unwrap().entry_angles[1] = erad(f32::NAN);
        assert!(!bad.is_calibrated());
        let mut bad = good;
        bad.angles[1] = erad(core::f32::consts::PI);
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
    fn hall_flash_ignores_superseded_angles_and_never_falls_back_from_invalid_records() {
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
        // A complete, valid superseded record (eight f32 angles, eight valid
        // flags) under the reserved key: never read, so still not calibrated.
        let old = HallCalibrationConfig::default();
        let mut superseded = [0u8; 40];
        let (angle_words, _) = superseded[..32].as_chunks_mut::<4>();
        for (word, angle) in angle_words.iter_mut().zip(old.angles) {
            *word = angle.value.to_le_bytes();
        }
        for (flag, valid) in superseded[32..].iter_mut().zip(old.valid) {
            *flag = u8::from(valid);
        }
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved2, &superseded)).unwrap();
        let loaded = ready(load_all(&mut storage, &mut buf));
        assert!(loaded.hall_calibration.is_none() && loaded.hall_calibration_is_valid());
        let new = measured_hall_config();
        ready(storage.store_item(
            &mut buf,
            &ConfigKey::HallCalibrationRecord,
            &HallCalibrationRecord::from(new),
        ))
        .unwrap();
        assert_eq!(
            ready(load_all(&mut storage, &mut buf)).hall_calibration,
            Some(new)
        );
        // A present record of an unknown version reads as invalid, not absent.
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
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved2, &0u8)).unwrap();
        let superseded_only = ready(load_all(&mut storage, &mut buf));
        assert!(
            superseded_only.hall_calibration.is_none()
                && superseded_only.hall_calibration_is_valid()
        );
    }

    #[test]
    fn every_tag_round_trips_and_unknown_tags_keep_their_byte() {
        for tag in 0..=u8::MAX {
            let key = ConfigKey::from_tag(tag);
            assert_eq!(key.tag(), tag);
            let mut stored = [0u8];
            assert_eq!(key.serialize_into(&mut stored), Ok(1));
            assert_eq!(ConfigKey::deserialize_from(&stored), Ok((key, 1)));
            let known = !matches!(key, ConfigKey::Unknown(_));
            assert_eq!(known, (1..=28).contains(&tag), "tag {tag}");
        }
    }

    #[cfg(feature = "runtime")]
    #[test]
    fn foreign_tags_neither_block_loading_nor_vanish_in_compaction() {
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
        type Flash = MockFlashBase<4, 4, 64>;
        let mut storage = MapStorage::new(
            Flash::new(WriteCountCheck::TwiceWithZero, None, true),
            MapConfig::new(Flash::FULL_FLASH_RANGE),
            Cache::new_uncached(),
        );
        let mut buf = [0; 128];
        let hall = measured_hall_config();
        ready(storage.store_item(
            &mut buf,
            &ConfigKey::HallCalibrationRecord,
            &HallCalibrationRecord::from(hall),
        ))
        .unwrap();
        // Items of a newer firmware: two tags, one of them rewritten.
        let (newer, other) = (ConfigKey::from_tag(200), ConfigKey::from_tag(201));
        ready(storage.store_item(&mut buf, &newer, &[1u8, 1, 1, 1])).unwrap();
        ready(storage.store_item(&mut buf, &other, &[2u8, 2, 2, 2])).unwrap();
        ready(storage.store_item(&mut buf, &newer, &[3u8, 3, 3, 3])).unwrap();
        // Churn one known key until every page has been compacted repeatedly.
        for value in 0..200u32 {
            ready(storage.store_item(&mut buf, &ConfigKey::PwmConfig, &value)).unwrap();
        }
        let fetch = |storage: &mut MapStorage<_, _, _>, key| {
            ready(storage.fetch_item::<[u8; 4]>(&mut [0; 128], &key)).unwrap()
        };
        assert_eq!(fetch(&mut storage, newer), Some([3; 4]));
        assert_eq!(fetch(&mut storage, other), Some([2; 4]));
        assert_eq!(
            ready(storage.fetch_item::<u32>(&mut buf, &ConfigKey::PwmConfig)).unwrap(),
            Some(199)
        );
        assert_eq!(
            ready(load_all(&mut storage, &mut buf)).hall_calibration,
            Some(hall)
        );
    }

    #[test]
    fn feedforward_policy_appends_storage_and_wire_ids_and_defaults_disabled() {
        use crate::types::{ConfigGroupId, ConfigValue};
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
                postcard::to_slice(&ConfigValue::VoltageFeedforward(value), &mut buf).unwrap(),
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
    fn drive_and_derating_use_only_their_current_flash_keys() {
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
        let mut old_buf = [0; 64];
        let old_derating = [
            77.0f32, 95.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 400.0, 0.9,
        ];
        let bytes: &[u8] = postcard::to_slice(&old_derating, &mut old_buf).unwrap();
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved11, &bytes)).unwrap();
        let old_drive = (true, Some(200.0f32), None::<f32>, 0.85f32);
        let bytes: &[u8] = postcard::to_slice(&old_drive, &mut old_buf).unwrap();
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved17, &bytes)).unwrap();
        let old_drive = (true, 300.0f32, 150.0f32, 0.8f32, 0.85f32);
        let bytes: &[u8] = postcard::to_slice(&old_drive, &mut old_buf).unwrap();
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved18, &bytes)).unwrap();
        let old_derating = [77.0f32, 95.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0];
        let bytes: &[u8] = postcard::to_slice(&old_derating, &mut old_buf).unwrap();
        ready(storage.store_item(&mut buf, &ConfigKey::Reserved19, &bytes)).unwrap();
        let voltage = VoltageLimitsConfig::default();
        ready(storage.store_item(&mut buf, &ConfigKey::VoltageLimits, &voltage)).unwrap();
        let loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.voltage_limits, Some(voltage));
        assert_eq!(loaded.drive, None);
        assert!(loaded.drive_is_valid());
        assert_eq!(loaded.derating, None);
        assert_eq!(loaded.effective_derating(), DeratingConfig::default());

        let drive = DriveConfig {
            invert_direction: true,
        };
        let derating = DeratingConfig {
            temperature: TemperatureDerating {
                fet_c: Some(Ramp {
                    start: 80.0,
                    end: 100.0,
                }),
                ..Default::default()
            },
            ..DeratingConfig::default()
        };
        ready(storage.store_item(&mut buf, &ConfigKey::Drive, &drive)).unwrap();
        ready(storage.store_item(&mut buf, &ConfigKey::Derating, &derating)).unwrap();
        let loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.drive, Some(drive));
        assert!(loaded.drive_is_valid());
        assert_eq!(loaded.effective_derating(), derating);
    }

    /// An unreadable record loads as defaults and marks its group corrupt
    /// until rewritten; limits and failsafe are the safety records.
    #[cfg(feature = "runtime")]
    #[test]
    fn unreadable_records_are_marked_corrupt() {
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
        assert!(!absent.has_corrupt_records() && absent.safety_records_valid());

        // A truncated record cannot decode as its type.
        ready(storage.store_item(&mut buf, &ConfigKey::PiGains, &0u8)).unwrap();
        let mut loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.pi_gains, None);
        assert!(!loaded.record_is_valid(ConfigGroupId::PiGains));
        assert!(loaded.has_corrupt_records() && loaded.safety_records_valid());

        ready(storage.store_item(&mut buf, &ConfigKey::Failsafe, &0u8)).unwrap();
        loaded = ready(load_all(&mut storage, &mut buf));
        assert_eq!(loaded.failsafe, None);
        assert!(!loaded.safety_records_valid());

        loaded.mark_rewritten(ConfigGroupId::Failsafe);
        assert!(loaded.safety_records_valid() && loaded.has_corrupt_records());
        loaded.mark_rewritten(ConfigGroupId::PiGains);
        assert!(!loaded.has_corrupt_records());
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
        mp.ld_fundamental = Some(henries(85.7e-6));
        assert!(
            mp.fundamental_ld_lq().is_none(),
            "one axis alone must not enable decoupling override"
        );
        mp.lq_fundamental = Some(henries(129.4e-6));
        assert_eq!(
            mp.fundamental_ld_lq(),
            Some((henries(85.7e-6), henries(129.4e-6)))
        );
    }

    #[test]
    fn motor_params_validity_rejects_non_finite() {
        let good = MotorParamsConfig {
            resistance: ohms(0.1),
            inductance_d: henries(1e-4),
            inductance_q: henries(3e-4),
            flux_linkage: webers(0.02),
            pole_pairs: 7,
            max_current: Some(amps(15.0)),
            max_power_loss: Some(watts(50.0)),
            ..Default::default()
        };
        assert!(good.is_valid());

        // NaN anywhere numeric must invalidate — these feed PI tuning and
        // the observers directly.
        for f in [
            |p: &mut MotorParamsConfig| p.resistance.value = f32::NAN,
            |p: &mut MotorParamsConfig| p.resistance.value = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_d.value = f32::NAN,
            |p: &mut MotorParamsConfig| p.inductance_d.value = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_q.value = f32::INFINITY,
            |p: &mut MotorParamsConfig| p.inductance_q.value = 0.0,
            |p: &mut MotorParamsConfig| p.flux_linkage.value = f32::NAN,
            |p: &mut MotorParamsConfig| p.flux_linkage.value = f32::INFINITY,
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
                    volts(24.0),
                );
                let out = controller.step(
                    (amps(0.0), amps(0.0), amps(0.0)),
                    erad(0.0),
                    amps(0.0),
                    amps(1.0),
                    1000,
                    seconds(0.00005),
                );
                assert!(out.vd.value.is_finite() && out.vq.value.is_finite());
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
            max_iq: Some(amps(10.0)),
            max_phase_current: Some(amps(13.0)),
            ..base.clone()
        };
        assert!(at.is_coherent());

        // Inside the band: rejected (the foot-gun).
        let inside = CurrentLimitsConfig {
            max_iq: Some(amps(40.0)),
            max_phase_current: Some(amps(40.0)),
            ..base.clone()
        };
        assert!(!inside.is_coherent());

        // "Not set" sides are always coherent — ceilings fill in.
        let unset_iq = CurrentLimitsConfig {
            max_iq: None,
            max_phase_current: Some(amps(40.0)),
            ..base.clone()
        };
        assert!(unset_iq.is_coherent());
        let unset_phase = CurrentLimitsConfig {
            max_iq: Some(amps(40.0)),
            max_phase_current: None,
            ..base.clone()
        };
        assert!(unset_phase.is_coherent());

        // Non-finite anywhere: rejected at the boundary (the builder's
        // NaN tolerance is for the boot path, not for explicit writes).
        for f in [
            |c: &mut CurrentLimitsConfig| c.max_iq = Some(amps(f32::NAN)),
            |c: &mut CurrentLimitsConfig| c.max_brake = Some(amps(f32::NAN)),
            |c: &mut CurrentLimitsConfig| c.max_phase_current = Some(amps(f32::INFINITY)),
            |c: &mut CurrentLimitsConfig| c.bus_in_max = Some(amps(f32::NAN)),
            |c: &mut CurrentLimitsConfig| c.bus_regen_max = Some(amps(f32::NAN)),
            |c: &mut CurrentLimitsConfig| c.max_dissipation = watts(f32::NAN),
        ] {
            let mut bad = base.clone();
            f(&mut bad);
            assert!(!bad.is_coherent(), "must reject {bad:?}");
        }
    }
    #[test]
    fn pi_gains_sanity_gate() {
        let ok = PiGainsConfig {
            kp: ohms(0.4),
            ki: quantity(40.0),
            bandwidth: rad_per_s(1000.0),
        };
        assert!(ok.is_sane());
        assert!(
            !PiGainsConfig {
                kp: ohms(0.0),
                ..ok
            }
            .is_sane()
        );
        assert!(
            !PiGainsConfig {
                kp: ohms(-1.0),
                ..ok
            }
            .is_sane()
        );
        assert!(
            !PiGainsConfig {
                ki: quantity(-1.0),
                ..ok
            }
            .is_sane()
        );
        assert!(
            !PiGainsConfig {
                kp: ohms(f32::NAN),
                ..ok
            }
            .is_sane()
        );
        assert!(
            !PiGainsConfig {
                ki: quantity(f32::NAN),
                ..ok
            }
            .is_sane()
        );
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
                interp_min_speed: erpm(-1.0),
                ..ok
            }
            .is_sane()
        );
    }

    #[test]
    fn hall_calibration_requires_physical_states_and_finite_angles() {
        let good = HallCalibrationConfig::default();
        assert!(good.is_calibrated());

        let mut missing = good;
        missing.valid[3] = false;
        assert!(!missing.is_calibrated());

        let mut shifted = good;
        shifted.valid[3] = false;
        shifted.valid[0] = true;
        assert!(!shifted.is_calibrated());

        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut non_finite = good;
            non_finite.angles[4] = erad(invalid);
            assert!(!non_finite.is_calibrated());
        }
    }
}
