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
//! | `FastTelemetryTopic` | `FastTelemetry` | High-frequency motor data (default 1kHz) |
//! | `SlowTelemetryTopic` | `SlowTelemetry` | System health data (default 10Hz) |
//!
//! # Endpoints (request/response)
//!
//! | Endpoint | Request | Response | Description |
//! |----------|---------|----------|-------------|
//! | `HardwareInfoEndpoint` | `()` | `HardwareInfo` | Hardware information query |
//! | `MotorEndpoint` | `ControlMode` | `MotorStatus` | Motor control commands |
//! | `TelemetryConfigEndpoint` | `TelemetryConfig` | `TelemetryConfigAck` | Configure streaming rates |
//! | `FaultEndpoint` | `FaultRequest` | `FaultResponse` | Fault query/clear |
//! | `DetectEndpoint` | `DetectRequest` | `DetectResponse` | Motor detection |

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
pub const LIVENESS_TIMEOUT_MS: u64 = 5000;

// ============================================================================
// Topic Definitions (push-based streaming)
// ============================================================================

// Fast telemetry topic (device → host)
// Firmware pushes batched motor control data at configurable rate.
// Host must send TelemetryConfig to enable streaming.
// Generic over batch capacity N — wire format is identical regardless of N.
pub struct FastTelemetryTopic<const N: usize = 32> {
    _priv: core::marker::PhantomData<()>,
}

impl<const N: usize> ergot::traits::Topic for FastTelemetryTopic<N> {
    type Message = FastTelemetryBatch<N>;
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

// TODO: EnergyTelemetryTopic — add when energy tracking is implemented
// topic!(EnergyTelemetryTopic, EnergyTelemetry, "telemetry/energy");

// ============================================================================
// Endpoint Definitions (request/response)
// ============================================================================

// Hardware info endpoint (host → device)
endpoint!(HardwareInfoEndpoint, (), HardwareInfo, "req/hardware_info");

// Motor control endpoint (host → device)
endpoint!(MotorEndpoint, ControlMode, MotorStatus, "cmd/motor");

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
// Every command declares where it sits on the delivery ladder. All endpoints
// below are idempotent by construction — reads and absolute setpoints — so the
// host may `at_least_once` them (retry on timeout is safe). The one action,
// `DetectEndpoint`, is intentionally NOT classified here yet: it becomes
// `Deduplicated` (with a `Keyed<DetectRequest>` payload) when the server-side
// dedup lands, so the compiler keeps it off the blind-retry path until then.
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
