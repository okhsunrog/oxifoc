//! Signal definitions for Dart <-> Rust communication

use rinf::{DartSignal, RustSignal, SignalPiece};
use serde::{Deserialize, Serialize};

// ============================================================================
// Commands: Dart -> Rust
// ============================================================================

/// Request list of available serial ports
#[derive(Deserialize, DartSignal)]
pub struct ListSerialPorts;

/// Request list of available debug probes
#[derive(Deserialize, DartSignal)]
pub struct ListProbes;

/// Connect to device via serial port
#[derive(Deserialize, DartSignal)]
pub struct ConnectSerial {
    pub port_path: String,
    pub baud_rate: u32,
}

/// Connect to device via RTT debug probe
#[derive(Deserialize, DartSignal)]
pub struct ConnectRtt {
    pub probe_id: String,
    pub chip: String,
}

/// Disconnect from device
#[derive(Deserialize, DartSignal)]
pub struct Disconnect;

/// Motor control command
#[derive(Deserialize, DartSignal)]
pub struct MotorCommand {
    pub command: MotorCommandType,
}

#[derive(Deserialize, SignalPiece)]
pub enum MotorCommandType {
    Stop,
    Start { iq_target: f32 },
}

// ============================================================================
// Responses/Events: Rust -> Dart
// ============================================================================

/// Serial port information
#[derive(Serialize, SignalPiece, Clone)]
pub struct SerialPortInfo {
    pub path: String,
    pub product: Option<String>,
    pub manufacturer: Option<String>,
}

/// List of available serial ports
#[derive(Serialize, RustSignal)]
pub struct SerialPortsList {
    pub ports: Vec<SerialPortInfo>,
}

/// Debug probe information
#[derive(Serialize, SignalPiece, Clone)]
pub struct ProbeInfo {
    pub identifier: String,
    pub vid: u16,
    pub pid: u16,
    pub serial_number: Option<String>,
    pub probe_type: String,
}

/// List of available debug probes
#[derive(Serialize, RustSignal)]
pub struct ProbesList {
    pub probes: Vec<ProbeInfo>,
}

/// Connection status update
#[derive(Serialize, RustSignal)]
pub struct ConnectionStatus {
    pub connected: bool,
    pub message: Option<String>,
}

/// ADC sample from device
#[derive(Serialize, RustSignal)]
pub struct AdcSample {
    pub ia: u16,
    pub ib: u16,
    pub ic: u16,
    pub vbus_mv: u32,
    pub fet_temp_c_x10: u16,
    pub seq: u32,
}

impl From<oxifoc_core::types::AdcSample> for AdcSample {
    fn from(s: oxifoc_core::types::AdcSample) -> Self {
        Self {
            ia: s.ia,
            ib: s.ib,
            ic: s.ic,
            vbus_mv: s.vbus_mv,
            fet_temp_c_x10: s.fet_temp_c_x10,
            seq: s.seq,
        }
    }
}
