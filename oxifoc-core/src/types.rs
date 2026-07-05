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

use crate::foc::phase::PhaseSource;
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
    /// Brake mode — all low-side FETs on, windings shorted (parking brake).
    ///
    /// Resists motion with speed-proportional torque and dissipates the
    /// energy in the motor (no regen, no bus pumping); draws nothing at
    /// standstill, so it can be held indefinitely. It is a *viscous* brake,
    /// not a position hold — on a slope the board creeps slowly.
    ///
    /// Entry is speed-gated at the command boundary: shorting the windings
    /// at speed dumps an uncontrolled current (~λ/L) through the FETs, so
    /// `process_commands` rejects it above a small velocity threshold.
    Brake,
}

// ============================================================================
// Status/Telemetry Types
// ============================================================================

/// Motor status response
///
/// Note: Fault information is now platform-specific. Use the platform's
/// fault endpoint to get detailed fault information.
impl ControlMode {
    /// All f32 payload fields are finite.
    ///
    /// Wire input is arbitrary bits: a NaN/inf target doesn't panic (float
    /// math never does), but it propagates through the PI loop into a
    /// garbage SVPWM vector. Commands failing this check must be dropped
    /// at the boundary.
    pub fn is_finite(&self) -> bool {
        match *self {
            Self::Stopped | Self::Coast | Self::Brake => true,
            Self::CurrentControl {
                iq_target,
                id_target,
            } => iq_target.is_finite() && id_target.is_finite(),
            Self::VelocityControl { target_vel } => target_vel.is_finite(),
            Self::PositionControl { target_pos } => target_pos.is_finite(),
            Self::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                pi_gains,
            } => {
                angle_rad.is_finite()
                    && current.is_finite()
                    && velocity_rad_s.is_finite()
                    && pi_gains.is_none_or(|(kp, ki)| kp.is_finite() && ki.is_finite())
            }
            Self::DirectVoltage { vd, vq, angle_rad } => {
                vd.is_finite() && vq.is_finite() && angle_rad.is_finite()
            }
        }
    }

    /// Whether the bridge is high-impedance (all FETs off, phases floating) in
    /// this mode. In a high-Z state the phase terminals show the motor's
    /// back-EMF, so a board with phase sensing can feed the *measured* terminal
    /// voltage to the observer (coasting-rotation tracking / flying start).
    ///
    /// `Brake` is deliberately excluded: it shorts the low sides (terminals ≈
    /// 0 V, not floating), so it keeps its own `(0 V, measured-i)` observer
    /// feed rather than the measured-BEMF path.
    pub fn is_high_z(&self) -> bool {
        matches!(self, Self::Stopped | Self::Coast)
    }
}

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

/// High-frequency motor diagnostic telemetry (streamed at configurable rate).
///
/// **Raw-ADC diagnostic frame.** Carries the three phase-current shunt readings
/// as uncalibrated 12-bit ADC counts — no calibration, sign correction, or
/// anti-alias decimation — straight from the converter. The host applies
/// offset/sign/scale (per `BoardConfig`) and reconstructs iα/iβ/id/iq in
/// post-processing. The third shunt is kept (not reconstructed from
/// `−(ia+ib)`) so a per-phase current-sense fault stays visible in the raw
/// data. Frame/field rationale: docs/notes/rtt-telemetry-throughput.md §6.
///
/// `#[repr(C)]`, 9×u16/i16 = **18 bytes**, align 2, no padding → `Pod` clean.
/// The in-memory layout IS the wire layout: batches ship the little-endian
/// Pod bytes verbatim (see [`FastTelemetryBatch`]), so this struct doubles as
/// the wire contract — reorder/resize fields only together with the host.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "pod", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub struct FastTelemetry {
    /// Phase A current — raw ADC counts (uncalibrated). Host reconstructs amps
    /// via `BoardConfig` (vref/counts/amp_gain/shunt) and the per-phase
    /// zero-current `dc_offsets`; `iα/iβ/id/iq` then follow from Clarke/Park.
    pub ia: u16,
    /// Phase B current — raw ADC counts
    pub ib: u16,
    /// Phase C current — raw ADC counts
    pub ic: u16,
    /// Bus voltage in **2 mV** units (u16 → 0..131 V, covers VESC classes).
    pub vbus: u16,
    /// Electrical angle, full-scale `u16` (0..2π). Needed host-side for Park.
    pub angle: u16,
    /// D-axis applied voltage (PI output) in **2 mV** units (i16 → ±65 V).
    /// Not reconstructable from currents (depends on PI integrator state).
    pub vd: i16,
    /// Q-axis applied voltage (PI output) in **2 mV** units.
    pub vq: i16,
    /// Mechanical speed in **2 RPM** units (i16 → ±65534 RPM). The ACTIVE
    /// angle source's velocity (hall / observer / HFI / startup ramp — see
    /// `FocOutput::velocity_rad_s`); before 2026-07-06 this read the hall
    /// estimator unconditionally and showed 0 on sensorless boards while
    /// spinning. Still hard-0 whenever `pole_pairs` is unknown (0).
    pub rpm: i16,
    /// Sequence number — `u16` (FOC-cycle counter mod 65536). At 20 kHz wraps
    /// every ~3.3 s; loss detection uses `wrapping_sub` (unambiguous for gaps
    /// < 1.6 s); host accumulates deltas for the time axis on longer captures.
    pub seq: u16,
}

/// Samples per fast-telemetry batch. Sized so the encoded batch statically
/// fits every board's `MAX_PACKET_SIZE` (1024): 576 B payload + ~15 B
/// header/len/COBS. Batch size does not move throughput on the byte-rate-bound
/// debug links (docs/notes/rtt-telemetry-throughput.md §4.7) — it only has to
/// fit the MTU.
pub const FAST_BATCH_SAMPLES: usize = 32;

/// Byte capacity of a fast-telemetry batch: samples × 18 B Pod frame.
pub const FAST_BATCH_BYTES: usize = FAST_BATCH_SAMPLES * size_of::<FastTelemetry>();

/// Batch of fast telemetry samples for efficient network transmission.
///
/// **Raw-Pod encoding**: `data` holds the concatenated little-endian
/// [`FastTelemetry`] Pod bytes (18 B each), NOT a postcard element sequence.
/// postcard serializes `Vec<u8>` as varint(len) + verbatim bytes, so:
/// - device-side encode is a memcpy (no per-field varint work in the hot
///   telemetry path — measured CPU relief on a core that also runs a 20 kHz
///   FOC ISR);
/// - the wire size is a compile-time constant (18 B/sample regardless of
///   values), so an encoded batch can never straddle the MTU — varint frames
///   grew with motor activity and silently exceeded it (see the BATCH=64 bug,
///   commit e1f65b5);
/// - both ends are little-endian (Cortex-M, x86); the Pod layout is the wire
///   contract, round-trip covered by `batch_roundtrip` below.
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastTelemetryBatch {
    /// `18 × n` raw frame bytes for `n` samples, `n ≤ FAST_BATCH_SAMPLES`.
    pub data: Vec<u8, FAST_BATCH_BYTES>,
}

impl FastTelemetryBatch {
    pub const fn new() -> Self {
        Self { data: Vec::new() }
    }

    /// Number of whole samples currently in the batch.
    pub fn len(&self) -> usize {
        self.data.len() / size_of::<FastTelemetry>()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// True when another 18 B frame no longer fits.
    pub fn is_full(&self) -> bool {
        self.data.len() + size_of::<FastTelemetry>() > FAST_BATCH_BYTES
    }

    /// Append one frame's raw Pod bytes. Returns `false` (batch unchanged) if
    /// `frame` is not exactly one 18 B frame or the batch is full.
    pub fn push_bytes(&mut self, frame: &[u8]) -> bool {
        frame.len() == size_of::<FastTelemetry>() && self.data.extend_from_slice(frame).is_ok()
    }

    /// Append one sample (device-side convenience over [`Self::push_bytes`]).
    #[cfg(feature = "pod")]
    pub fn push(&mut self, sample: &FastTelemetry) -> bool {
        self.push_bytes(bytemuck::bytes_of(sample))
    }

    /// Decode the batch back into samples (host-side).
    #[cfg(feature = "pod")]
    pub fn samples(&self) -> impl Iterator<Item = FastTelemetry> + '_ {
        self.data
            .chunks_exact(size_of::<FastTelemetry>())
            .map(bytemuck::pod_read_unaligned)
    }
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
    pub fet_temp_c_x10: i16,
    /// Motor temperature in 0.1°C units (0 if not available)
    pub motor_temp_c_x10: i16,
    /// Board temperature in 0.1°C units (0 if not available)
    pub board_temp_c_x10: i16,
    /// Current motor state
    pub motor_state: MotorState,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Number of active faults
    pub fault_count: u8,
    /// Active phase source (Hall / Observer / HFI / crossovers)
    pub phase_source: PhaseSource,
    /// Monotonic sequence number
    pub seq: u32,
    // postcard struct fields are positional — append only.
    /// Live drive-side derating (percent of the configured limit, 100 =
    /// no derate) — "why does the board feel weak" at a glance
    pub derate_drive_pct: u8,
    /// Live brake-side derating (percent, 100 = no derate)
    pub derate_brake_pct: u8,
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

/// Application protocol version. Bump on ANY breaking ICD change — wire *shape*
/// (already enforced fail-closed by ergot's schema-hashed keys) OR *semantics*
/// (units/meaning of a field whose type is unchanged, which the key hash cannot
/// see). The device reports it in [`HardwareInfo::proto_version`]; the host
/// compares it to its own value at the connect handshake and warns on mismatch
/// — the human-readable gate that also covers topics (whose key change would
/// otherwise fail as *silent absence*). See `docs/notes/protocol-versioning.md`.
pub const ICD_PROTO_VERSION: u16 = 1;

/// Hardware information returned on initial handshake
#[derive(Clone, Debug, Default, Serialize, Deserialize, Schema)]
pub struct HardwareInfo {
    /// Application protocol version ([`ICD_PROTO_VERSION`]) — host compares to
    /// its own and warns on mismatch.
    pub proto_version: u16,
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
    /// Static current-sense / vbus constants for host-side telemetry
    /// enrichment (raw frame → engineering units). See [`BoardCalib`].
    pub calib: BoardCalib,
}

/// Static board electrical constants the host needs to reconstruct engineering
/// units from the raw [`FastTelemetry`] frame (ADC counts → amps, raw bus →
/// volts). Compile-time per board; carried once at connect. Combined host-side
/// with the (dynamic) `dc_offsets` calibration and `pole_pairs` to build the
/// enrichment context. See [`crate::foc::telemetry`] and the design note
/// `docs/notes/telemetry-enrichment.md`.
///
/// This is the wire projection of the current-sense/vbus fields of
/// [`crate::foc::config::BoardConfig`] (its literal `calib` sub-struct field —
/// one definition, no bridging copy); it is intentionally NOT the whole
/// `BoardConfig` (fault thresholds, phase-sense, etc. are firmware-internal).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardCalib {
    /// Shunt resistance in Ohms.
    pub shunt_ohms: f32,
    /// Current amplifier gain (V/V).
    pub amp_gain: f32,
    /// ADC reference voltage in millivolts.
    pub adc_vref_mv: u32,
    /// Maximum ADC count (e.g. 4095 for 12-bit).
    pub adc_max_counts: u16,
    /// Current-sense sign inversion (low-side shunts, MCSDK convention).
    pub invert_current_sign: bool,
    /// VBUS divider ratio (Vbus = Vsense · ratio). Not needed to decode the
    /// frame's `vbus` (already post-divider mV) — carried for completeness.
    pub vbus_divider_ratio: f32,
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
    /// List of active faults with details (at most `MAX_FAULT_RESPONSE`)
    pub faults: Vec<FaultInfo, MAX_FAULT_RESPONSE>,
    /// Total active faults in the registry; `total > faults.len()` means
    /// the list above is truncated.
    pub total: u8,
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
        /// Previously measured average inductance (H), 0.0 = unknown.
        /// Trims the ωL·i reactance term of the back-EMF-vector method.
        inductance_h: f32,
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
    use crate::storage::{
        CurrentLimitsConfig, DcOffsetsConfig, DeratingConfigStored, FailsafeConfigStored,
        HallCalibrationConfig, HallTuningConfig, MotorParamsConfig, PiGainsConfig, PwmConfigStored,
        VelocityConfigStored, VoltageLimitsConfig,
    };

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
        Failsafe,
        Velocity,
        // postcard encodes the variant index — append only.
        Derating,
    }

    /// Config write payload — one variant per group
    #[derive(Clone, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigWrite {
        MotorParams(MotorParamsConfig),
        CurrentLimits(CurrentLimitsConfig),
        VoltageLimits(VoltageLimitsConfig),
        PwmConfig(PwmConfigStored),
        PiGains(PiGainsConfig),
        HallTuning(HallTuningConfig),
        /// Hall calibration result (written by the host after CalibrateHall)
        HallCalibration(HallCalibrationConfig),
        /// Current-sensor DC offsets
        DcOffsets(DcOffsetsConfig),
        /// Command-staleness deadman + failsafe policy
        Failsafe(FailsafeConfigStored),
        /// Cruise velocity-loop tuning
        Velocity(VelocityConfigStored),
        // postcard encodes the variant index — append only.
        /// Graduated derating ramps (thermal/voltage/speed)
        Derating(DeratingConfigStored),
    }

    /// Configuration response
    #[derive(Clone, Debug, Serialize, Deserialize, Schema)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ConfigResponse {
        /// Operation succeeded
        Ok,
        /// Motor parameters
        MotorParams(MotorParamsConfig),
        /// Current limits
        CurrentLimits(CurrentLimitsConfig),
        /// Voltage limits
        VoltageLimits(VoltageLimitsConfig),
        /// PWM configuration
        PwmConfig(PwmConfigStored),
        /// PI gains
        PiGains(PiGainsConfig),
        /// Hall tuning
        HallTuning(HallTuningConfig),
        /// Hall calibration data
        HallCalibration(HallCalibrationConfig),
        /// DC offsets
        DcOffsets(DcOffsetsConfig),
        /// Failsafe (deadman + policy)
        Failsafe(FailsafeConfigStored),
        /// Cruise velocity-loop tuning
        Velocity(VelocityConfigStored),
        /// Requested group has no stored value
        NotFound,
        /// Flash operation failed
        Error,
        /// Refused: motor is running. Flash writes stall the chip (sector
        /// erase takes up to seconds on F405) and would glitch the FOC loop.
        Busy,
        // postcard encodes the variant index — append new variants below,
        // never reorder the ones above.
        /// Refused: the written value fails boundary validation (e.g. the
        /// current-limits headroom rule, `CurrentLimitsConfig::is_coherent`).
        Invalid,
        /// Graduated derating ramps
        Derating(DeratingConfigStored),
    }
}

// ============================================================================
// Conversion helpers
// ============================================================================

impl MotorState {
    /// Convert from u8 (for atomic storage)
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::Stopped,
            1 => Self::Running,
            _ => Self::Error,
        }
    }

    /// Convert to u8 (for atomic storage)
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Stopped => 0,
            Self::Running => 1,
            Self::Error => 2,
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

    /// Raw-Pod batch: push → postcard wire → decode must reproduce the exact
    /// samples, and the wire size must be value-independent (the whole point
    /// of raw-Pod vs per-field varints).
    #[cfg(feature = "pod")]
    #[test]
    fn batch_roundtrip() {
        let samples: Vec<FastTelemetry, FAST_BATCH_SAMPLES> = (0..FAST_BATCH_SAMPLES)
            .map(|i| FastTelemetry {
                ia: 2500 + i as u16,
                ib: 40_000,
                ic: 3,
                vbus: 6000,
                angle: (i as u16).wrapping_mul(2048),
                vd: -1200,
                vq: 32000,
                rpm: -15000,
                seq: 65_500 + i as u16, // wraps
            })
            .collect();

        let mut batch = FastTelemetryBatch::new();
        for s in &samples {
            assert!(batch.push(s));
        }
        assert!(batch.is_full());
        assert!(!batch.push(&FastTelemetry::default()));
        assert_eq!(batch.len(), FAST_BATCH_SAMPLES);

        let mut buf = [0u8; FAST_BATCH_BYTES + 8];
        let wire = postcard::to_slice(&batch, &mut buf).unwrap();
        // varint(len=576) = 2 B + payload; the container adds nothing else.
        assert_eq!(wire.len(), 2 + FAST_BATCH_BYTES);

        let decoded: FastTelemetryBatch = postcard::from_bytes(wire).unwrap();
        let out: Vec<FastTelemetry, FAST_BATCH_SAMPLES> = decoded.samples().collect();
        assert_eq!(out, samples);
    }
}
