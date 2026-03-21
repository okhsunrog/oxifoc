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

use ergot::{endpoint, topic};

// Re-export all types for convenience
pub use crate::types::*;

// ============================================================================
// Topic Definitions (push-based streaming)
// ============================================================================

// Fast telemetry topic (device → host)
// Firmware pushes motor control data at configurable rate (default 1kHz).
// Contains phase currents, dq currents/voltages, angle, RPM, hall state.
topic!(FastTelemetryTopic, FastTelemetry, "telemetry/fast");

// Slow telemetry topic (device → host)
// Firmware pushes system health data at lower rate (default 10Hz).
// Contains bus voltage, temperatures, motor state, fault count.
topic!(SlowTelemetryTopic, SlowTelemetry, "telemetry/slow");

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
endpoint!(TelemetryConfigEndpoint, TelemetryConfig, TelemetryConfigAck, "cmd/telemetry_config");

// Fault management endpoint (host → device)
endpoint!(FaultEndpoint, FaultRequest, FaultResponse, "cmd/fault");

// Motor detection endpoint (host → device)
endpoint!(DetectEndpoint, DetectRequest, DetectResponse, "cmd/detect");

// Configuration endpoint (host → device)
#[cfg(feature = "storage")]
endpoint!(ConfigEndpoint, ConfigRequest, ConfigResponse, "cmd/config");

// Legacy endpoints — kept for backward compatibility during migration
// TODO: Remove once all host tools use topic-based telemetry
endpoint!(AdcSampleEndpoint, (), AdcSample, "req/adc");
endpoint!(HallSensorEndpoint, (), HallSensorData, "req/hall");
