//! Interface Control Document (ICD) - Ergot endpoint and topic definitions
//!
//! This module defines the communication protocol between host applications
//! and motor controller firmware using the ergot framework.
//!
//! # Feature: `icd`
//!
//! This module is only available when the `icd` feature is enabled.
//! It automatically enables the `types` feature for shared type definitions.
//!
//! # Topics (push-based streaming, device → host)
//!
//! | Topic | Message | Description |
//! |-------|---------|-------------|
//! | `RawTelemetryBlockTopic` | `RawTelemetryBlock` | Packed28 fast telemetry blocks, unicast to the session owner |
//! | `FaultTopic` | `FaultSnapshot` | Revisioned snapshot pushed on every registry change |
//! | `PairingEventTopic` | `PairingEvent` | Pairing progress and bond changes (bridge) |
//!
//! # Endpoints (request/response)
//!
//! | Endpoint | Request | Response | Description |
//! |----------|---------|----------|-------------|
//! | `HardwareInfoEndpoint` | `()` | `HardwareInfo` | Hardware information query |
//! | `MotorEndpoint` | `MotorRequest` | `MotorStatus` | Setpoint, affirmation, stop, and phase-source selection |
//! | `SlowTelemetryEndpoint` | `()` | `SlowTelemetry` | System health poll (~10 Hz, doubles as heartbeat) |
//! | `RawTelemetryEndpoint` | `RawTelemetryRequest` | `RawTelemetryAck` | Arm, stop and query a fast telemetry session |
//! | `FaultEndpoint` | `FaultRequest` | `FaultResponse` | Fault query/clear |
//! | `DiagEndpoint` | `DiagRequest` | `DiagResponse` | ISR timing, totals since boot, reset cause, crash record |
//! | `BridgeDiagEndpoint` | `BridgeDiagRequest` | `BridgeDiagResponse` | Link counters and the crash record (served by the bridge) |
//! | `DetectEndpoint` | `MeasurementRequest` | `MeasurementReply` | Motor detection |
//! | `ConfigEndpoint` | `ConfigRequest` | `ConfigResponse` | Revisioned config apply/persist (`storage` feature) |
//! | `PairingEndpoint` | `PairingRequest` | `PairingResponse` | BLE pairing windows and bonds (served by the bridge) |

use ergot::endpoint;

// Re-export all types for convenience
pub use crate::types::*;

use crate::raw_telemetry;
use crate::raw_telemetry::{RawTelemetryAck, RawTelemetryRequest, VelocityDebugArm};
use crate::types::bridge_ota::{OtaRequest, OtaResponse};
use crate::types::swd_probe::{ProbeRequest, ProbeResponse};

// ============================================================================
// Protocol Constants
// ============================================================================

/// Liveness timeout in milliseconds for all ergot transports.
/// If no frames are received within this period, the interface transitions
/// to Inactive (COBS streams) or Down (UDP).
///
/// The ISR-resident command-staleness deadman (≈150 ms, transport-agnostic)
/// is the real fast safety net now, so this transport-level liveness only
/// needs to be tight enough to drop a dead link reasonably; 1 s is the
/// conservative interim (500 ms risks BLE flapping — this constant is shared
/// across all transports). See docs/safety.md (Layer 1 vs Layer 2).
pub const LIVENESS_TIMEOUT_MS: u64 = 1000;

// ============================================================================
// Topic Definitions (push-based streaming)
// ============================================================================

/// One raw-session envelope: a block with its versioned header or END, and
/// in velocity-debug sessions a velocity companion block after it.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, postcard_schema::Schema)]
pub struct RawTelemetryBlock {
    #[serde(serialize_with = "serialize_raw_block_bytes")]
    pub data: heapless::Vec<u8, { raw_telemetry::RAW_MAX_BLOCK_BYTES }>,
}

fn serialize_raw_block_bytes<S: serde::Serializer>(
    bytes: &[u8],
    serializer: S,
) -> Result<S::Ok, S::Error> {
    serializer.serialize_bytes(bytes)
}

/// Borrowed sender representation: serialization copies directly into Ergot's
/// framed queue without first copying the block into a second owned buffer.
#[derive(serde::Serialize)]
pub struct RawTelemetryBlockRef<'a> {
    #[serde(serialize_with = "serialize_raw_block_bytes")]
    pub data: &'a [u8],
}

impl postcard_schema::Schema for RawTelemetryBlockRef<'_> {
    const SCHEMA: &'static postcard_schema::schema::NamedType = RawTelemetryBlock::SCHEMA;
}

ergot::topic!(
    RawTelemetryBlockTopic,
    RawTelemetryBlock,
    "telemetry/raw_block"
);

/// Ergot frame bytes around a raw block: the largest Ergot header and the
/// postcard length prefix of a byte string up to `u16::MAX` bytes.
pub const RAW_FRAME_OVERHEAD: usize = ergot::wire_frames::MAX_HDR_ENCODED_SIZE + 3;

/// Largest raw block one Ergot frame of `mtu` bytes carries, capped at
/// [`raw_telemetry::RAW_MAX_BLOCK_BYTES`]. A link whose result is below
/// [`raw_telemetry::RAW_MIN_BLOCK_BYTES`] cannot carry sessions.
pub const fn raw_block_limit(mtu: u16) -> u16 {
    let budget = (mtu as usize).saturating_sub(RAW_FRAME_OVERHEAD);
    if budget < raw_telemetry::RAW_MAX_BLOCK_BYTES {
        budget as u16
    } else {
        raw_telemetry::RAW_MAX_BLOCK_BYTES as u16
    }
}

pub struct RawTelemetryBlockRefTopic<'a>(core::marker::PhantomData<&'a ()>);

impl<'a> ergot::traits::Topic for RawTelemetryBlockRefTopic<'a> {
    type Message = RawTelemetryBlockRef<'a>;
    const PATH: &'static str = <RawTelemetryBlockTopic as ergot::traits::Topic>::PATH;
    const TOPIC_KEY: ergot::traits::Key =
        <RawTelemetryBlockTopic as ergot::traits::Topic>::TOPIC_KEY;
}

#[cfg(test)]
#[path = "icd_raw_tests.rs"]
mod raw_block_tests;

// Slow telemetry endpoint (host polls device)
// Host periodically requests system health data (~10Hz).
// Serves as both telemetry and heartbeat for device-side liveness tracking.
endpoint!(
    SlowTelemetryEndpoint,
    (),
    SlowTelemetry,
    "req/telemetry_slow"
);

// Fault topic (device → host/remote): the FULL fault snapshot, broadcast
// on every registry change (fault raised, payload refined, cleared) plus
// once at stream start. Snapshot-not-delta on purpose: topics are
// fire-and-forget (no ack), so a lost packet must cost staleness, not a
// wrong state — the next change resends everything, and the remote's
// SlowTelemetry poll (`fault_generation`) detects a stale view and re-queries
// via FaultEndpoint, including same-count refinements. The remote keys vibration/UI off `FaultInfo::severity`,
// never off hardcoded categories.
ergot::topic!(FaultTopic, FaultSnapshot, "telemetry/faults");

// TODO: EnergyTelemetryTopic — add when energy tracking is implemented
// topic!(EnergyTelemetryTopic, EnergyTelemetry, "telemetry/energy");

// ============================================================================
// Endpoint Definitions (request/response)
// ============================================================================

// Hardware info endpoint (host → device)
endpoint!(HardwareInfoEndpoint, (), HardwareInfo, "req/hardware_info");

// Motor control endpoint (host → device). Safe modes and EmergencyStop are
// universal; active setpoints are source-owned and sequence checked.
endpoint!(MotorEndpoint, MotorRequest, MotorStatus, "cmd/motor");

endpoint!(
    RawTelemetryEndpoint,
    RawTelemetryRequest,
    RawTelemetryAck,
    "cmd/raw_telemetry"
);
endpoint!(
    VelocityDebugArmEndpoint,
    VelocityDebugArm,
    RawTelemetryAck,
    "cmd/raw_velocity_debug"
);

// Fault management endpoint (host → device)
endpoint!(FaultEndpoint, FaultRequest, FaultResponse, "cmd/fault");

// Controller diagnostics: what an assembled board can report without a probe.
pub use crate::types::diag::{DiagRequest, DiagResponse};
endpoint!(DiagEndpoint, DiagRequest, DiagResponse, "req/diag");
pub use crate::types::diag::{BridgeDiagRequest, BridgeDiagResponse};
endpoint!(
    BridgeDiagEndpoint,
    BridgeDiagRequest,
    BridgeDiagResponse,
    "req/bridge_diag"
);

// Session-scoped actions: retries preserve CAS session or operation identity.
pub use crate::types::measurement::{Reply as MeasurementReply, Request as MeasurementRequest};
endpoint!(
    DetectEndpoint,
    MeasurementRequest,
    MeasurementReply,
    "cmd/detect"
);

// BLE pairing management, served by the bridge rather than the motor
// controller. See docs/notes/ble-pairing.md.
pub use crate::types::pairing::{PairingEvent, PairingRequest, PairingResponse};
endpoint!(
    PairingEndpoint,
    PairingRequest,
    PairingResponse,
    "cmd/pairing"
);
ergot::topic!(PairingEventTopic, PairingEvent, "events/pairing");

endpoint!(
    SwdProbeEndpoint,
    ProbeRequest,
    ProbeResponse,
    "cmd/swd_probe"
);

endpoint!(BridgeOtaEndpoint, OtaRequest, OtaResponse, "cmd/bridge_ota");

// Configuration endpoint (host → device)
#[cfg(feature = "storage")]
endpoint!(ConfigEndpoint, ConfigRequest, ConfigResponse, "cmd/config");

// ============================================================================
// Delivery semantics classification (requires `delivery` feature)
// ============================================================================
//
// Every command declares where it sits on the delivery ladder. Almost all
// endpoints are idempotent by construction — reads and absolute setpoints —
// so the host may `at_least_once` them (retry on timeout is safe). The one
// action, `DetectEndpoint`, is `Deduplicated`: session CAS and operation IDs make retries safe.
// `ConfigEndpoint` remains retry-idempotent at the endpoint boundary because
// its action variants embed stable keys and replay cached responses; Read and
// ResetAll are naturally idempotent.
#[cfg(feature = "delivery")]
mod delivery_classes {
    use super::*;
    use crate::delivery::{AtMostOnce, Command, Deduplicated, Idempotent};

    impl Command for HardwareInfoEndpoint {
        type Delivery = Idempotent;
    }
    // Detection is the one action: deduplicated, not idempotent.
    impl Command for DetectEndpoint {
        type Delivery = Deduplicated;
    }
    impl Command for SlowTelemetryEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for MotorEndpoint {
        // Retrying an unconfirmed setpoint could acquire ownership later.
        type Delivery = AtMostOnce;
    }
    impl Command for RawTelemetryEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for VelocityDebugArmEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for FaultEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for DiagEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for BridgeDiagEndpoint {
        type Delivery = Idempotent;
    }
    #[cfg(feature = "storage")]
    impl Command for ConfigEndpoint {
        type Delivery = Idempotent;
    }
}
