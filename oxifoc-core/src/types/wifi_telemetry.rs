//! Retrieve a recently forwarded raw telemetry block from the Wi-Fi bridge.

use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize, Schema)]
pub struct TelemetryReplayRequest {
    pub session: u64,
    pub block: u32,
}
