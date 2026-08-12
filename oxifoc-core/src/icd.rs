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
//! | `FastTelemetryTopic` | `FastTelemetryBatch` | High-frequency motor data (default 1kHz) |
//! | `FaultTopic` | `FaultSnapshot` | Revisioned snapshot pushed on every registry change |
//!
//! # Endpoints (request/response)
//!
//! | Endpoint | Request | Response | Description |
//! |----------|---------|----------|-------------|
//! | `HardwareInfoEndpoint` | `()` | `HardwareInfo` | Hardware information query |
//! | `MotorEndpoint` | `MotorRequest` | `MotorStatus` | Sequenced motor control and emergency stop |
//! | `SlowTelemetryEndpoint` | `()` | `SlowTelemetry` | System health poll (~10 Hz, doubles as heartbeat) |
//! | `TelemetryConfigEndpoint` | `TelemetryConfig` | `TelemetryConfigAck` | Configure streaming rates |
//! | `FaultEndpoint` | `FaultRequest` | `FaultResponse` | Fault query/clear |
//! | `DetectEndpoint` | `Keyed<DetectRequest>` | `DetectResponse` | Motor detection |
//! | `PhaseSourceEndpoint` | `PhaseSource` | `PhaseSourceAck` | Angle source selection |
//! | `ConfigEndpoint` | `ConfigRequest` | `ConfigResponse` | Revisioned config apply/persist (`storage` feature) |

use ergot::endpoint;

// Re-export all types for convenience
pub use crate::types::*;

use crate::foc::phase::PhaseSource;

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

// Fast telemetry topic (device → host)
// Firmware pushes batched motor control data at configurable rate.
// Host must send TelemetryConfig to enable streaming.
// The batch is raw-Pod encoded with a fixed capacity — see FastTelemetryBatch.
pub struct FastTelemetryTopic {
    _priv: core::marker::PhantomData<()>,
}

impl ergot::traits::Topic for FastTelemetryTopic {
    type Message = FastTelemetryBatch;
    const PATH: &'static str = "telemetry/fast";
    const TOPIC_KEY: ergot::traits::Key =
        ergot::traits::Key::for_path::<FastTelemetryBatch>("telemetry/fast");
}

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

// Telemetry rate configuration endpoint (host → device)
// Host configures fast/slow telemetry streaming rates.
endpoint!(
    TelemetryConfigEndpoint,
    TelemetryConfig,
    TelemetryConfigAck,
    "cmd/telemetry_config"
);

// Fault management endpoint (host → device)
endpoint!(FaultEndpoint, FaultRequest, FaultResponse, "cmd/fault");

// Motor detection endpoint (host → device).
//
// Detection is a non-idempotent ACTION, so its request carries a `ReqId`
// (`Keyed<DetectRequest>`): the device deduplicates on it (see
// `oxifoc_core::runtime::detect`), making it safely retryable as
// effectively-once. Classified `Deduplicated` below. The `endpoint!` macro
// takes a single type token, hence the alias.
pub type KeyedDetectRequest = Keyed<DetectRequest>;
endpoint!(
    DetectEndpoint,
    KeyedDetectRequest,
    DetectResponse,
    "cmd/detect"
);

// Phase source selection endpoint (host → device).
// Response reports whether the command was enqueued; the actually-active
// source is read back via SlowTelemetry.phase_source (the ISR-side switch
// can still reject an invalid source, leaving it unchanged).
endpoint!(
    PhaseSourceEndpoint,
    PhaseSource,
    PhaseSourceAck,
    "cmd/phase_source"
);

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
// action, `DetectEndpoint`, is `Deduplicated`: its whole request is keyed.
// `ConfigEndpoint` remains retry-idempotent at the endpoint boundary because
// its action variants embed stable keys and replay cached responses; Read and
// ResetAll are naturally idempotent.
#[cfg(feature = "delivery")]
mod delivery_classes {
    use super::*;
    use crate::delivery::{Command, Deduplicated, Idempotent};

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
        type Delivery = Idempotent;
    }
    impl Command for PhaseSourceEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for TelemetryConfigEndpoint {
        type Delivery = Idempotent;
    }
    impl Command for FaultEndpoint {
        type Delivery = Idempotent;
    }
    #[cfg(feature = "storage")]
    impl Command for ConfigEndpoint {
        type Delivery = Idempotent;
    }
}
