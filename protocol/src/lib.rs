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
