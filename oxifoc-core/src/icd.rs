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
//! | `InfoEndpoint` | `()` | `DeviceInfo` | Device information query |
//! | `MotorEndpoint` | `ControlMode` | `MotorStatus` | Motor control commands |
//! | `TelemetryConfigEndpoint` | `TelemetryConfig` | `TelemetryConfigAck` | Configure streaming rates |
//! | `FaultEndpoint` | `FaultRequest` | `FaultResponse` | Fault query/clear |
//! | `DetectEndpoint` | `DetectRequest` | `DetectResponse` | Motor detection |
//! | `ButtonEndpoint` | `ButtonEvent` | `()` | Button events from device |

use ergot::endpoint;

// Re-export all types for convenience
pub use crate::types::*;

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

// Button event endpoint (device → host)
endpoint!(ButtonEndpoint, ButtonEvent, (), "event/button");

// Device info endpoint (host → device)
endpoint!(InfoEndpoint, (), DeviceInfo, "req/device_info");

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

// Motor detection endpoint (host → device)
endpoint!(DetectEndpoint, DetectRequest, DetectResponse, "cmd/detect");

// Configuration endpoint (host → device)
#[cfg(feature = "storage")]
endpoint!(ConfigEndpoint, ConfigRequest, ConfigResponse, "cmd/config");
