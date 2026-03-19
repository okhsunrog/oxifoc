//! Interface Control Document (ICD) - Ergot endpoint definitions
//!
//! This module defines the communication protocol between host applications
//! and motor controller firmware using the ergot framework.
//!
//! # Feature: `icd`
//!
//! This module is only available when the `icd` feature is enabled.
//! It automatically enables the `types` feature for shared type definitions.
//!
//! # Endpoints
//!
//! | Endpoint | Request | Response | Topic | Description |
//! |----------|---------|----------|-------|-------------|
//! | `ButtonEndpoint` | `ButtonEvent` | `()` | `event/button` | Button events from device |
//! | `InfoEndpoint` | `()` | `DeviceInfo` | `req/device_info` | Device information query |
//! | `MotorEndpoint` | `ControlMode` | `MotorStatus` | `cmd/motor` | Motor control commands |
//! | `AdcSampleEndpoint` | `()` | `AdcSample` | `req/adc` | ADC sample poll |
//! | `HallSensorEndpoint` | `()` | `HallSensorData` | `req/hall` | Hall sensor data poll |
//! | `FaultEndpoint` | `FaultRequest` | `FaultResponse` | `cmd/fault` | Fault query/clear |

use ergot::endpoint;

// Re-export all types for convenience
pub use crate::types::*;

// ============================================================================
// Endpoint Definitions
// ============================================================================

// Button event endpoint (device → host)
// Device sends button events to host when user presses hardware button.
endpoint!(ButtonEndpoint, ButtonEvent, (), "event/button");

// Device info endpoint (host → device)
// Host queries device for hardware and software version information.
endpoint!(InfoEndpoint, (), DeviceInfo, "req/device_info");

// Motor control endpoint (host → device)
// Host sends control mode, device responds with current status.
endpoint!(MotorEndpoint, ControlMode, MotorStatus, "cmd/motor");

// ADC sample endpoint (host → device)
// Host polls device for current ADC readings (phase currents, voltage, temperature).
endpoint!(AdcSampleEndpoint, (), AdcSample, "req/adc");

// Hall sensor endpoint (host → device)
// Host polls device for Hall sensor data (angle, direction, state).
endpoint!(HallSensorEndpoint, (), HallSensorData, "req/hall");

// Fault management endpoint (host → device)
// Host queries/clears faults. Responds with current fault state.
endpoint!(FaultEndpoint, FaultRequest, FaultResponse, "cmd/fault");

// Configuration endpoint (host → device)
// Host reads/writes persistent configuration stored in flash.
#[cfg(feature = "storage")]
endpoint!(ConfigEndpoint, ConfigRequest, ConfigResponse, "cmd/config");
