#![no_std]

use ergot::endpoint;
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
