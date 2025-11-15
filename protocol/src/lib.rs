#![no_std]

use ergot::endpoint;
use heapless::String;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

/// Button events from the B-G431B-ESC1 board
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub enum ButtonEvent {
    SingleClick,
    DoubleClick,
    Hold,
}

// Define endpoint for button communication
endpoint!(ButtonEndpoint, ButtonEvent, (), "event/button");

/// Basic device info returned on request
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct DeviceInfo {
    pub hw: String<32>,
    pub sw: String<32>,
}

// Host -> Device info query endpoint (unit request, returns DeviceInfo)
endpoint!(InfoEndpoint, (), DeviceInfo, "req/device_info");

/// Motor control commands
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub enum MotorCommand {
    Stop,
    Start { duty: u8 },      // duty: 0-100%
    SetSpeed { duty: u8 },   // duty: 0-100% (adjust while running)
}

/// Motor operational state
#[derive(Clone, Schema, Serialize, Deserialize, Debug, PartialEq, Eq)]
pub enum MotorState {
    Stopped,
    Running,
    Error,
}

/// Motor status response
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct MotorStatus {
    pub state: MotorState,
    pub duty: u8,           // Current duty cycle (0-100%)
    pub step: u8,           // Current commutation step (0-5)
}

// Host -> Device motor control endpoint (command in, status out)
endpoint!(MotorEndpoint, MotorCommand, MotorStatus, "cmd/motor");
