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
    Start { duty: u8 },    // duty: 0-100%
    SetSpeed { duty: u8 }, // duty: 0-100% (adjust while running)
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
    pub duty: u8, // Current duty cycle (0-100%)
    pub step: u8, // Current commutation step (0-5)
}

// Host -> Device motor control endpoint (command in, status out)
endpoint!(MotorEndpoint, MotorCommand, MotorStatus, "cmd/motor");

/// Raw ADC sample (phase currents, voltage, temperature)
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct AdcSample {
    pub ia: u16,
    pub ib: u16,
    pub ic: u16,
    /// Measured DC bus voltage in millivolts.
    pub vbus_mv: u32,
    /// Estimated FET temperature in 0.1°C units.
    pub fet_temp_c_x10: u16,
    /// Monotonic sequence number (wraps on overflow)
    pub seq: u32,
}

// Host polls device for current ADC sample (request-response)
endpoint!(AdcSampleEndpoint, (), AdcSample, "req/adc");

/// Direction of rotation from Hall sensors
#[derive(Clone, Copy, Schema, Serialize, Deserialize, Debug, PartialEq, Eq)]
pub enum HallDirection {
    Clockwise,
    CounterClockwise,
    Stopped,
}

/// Hall sensor telemetry
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct HallSensorData {
    /// Electrical angle in radians (0 to 2π)
    pub angle_rad: f32,
    /// Direction of rotation
    pub direction: HallDirection,
    /// Raw Hall state (0-5 for valid states)
    pub state: u8,
    /// Error count (invalid states or transitions)
    pub error_count: u32,
    /// Monotonic sequence number (wraps on overflow)
    pub seq: u32,
}

// Host polls device for Hall sensor data (request-response)
endpoint!(HallSensorEndpoint, (), HallSensorData, "req/hall");
