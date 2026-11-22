//! USB control of the bridge's wireless SWD probe.

use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

pub const MAX_ARM_MINUTES: u8 = 30;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum ProbeRequest {
    Arm { minutes: u8 },
    Disarm,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum ProbeResponse {
    Armed { ipv4: [u8; 4], minutes: u8 },
    Disarmed,
    UsbRequired,
    InvalidDuration,
    WifiUnavailable,
}
