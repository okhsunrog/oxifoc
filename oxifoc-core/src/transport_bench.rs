//! Opt-in transport qualification protocol. Contains no motor commands.
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

pub const MAX_PAYLOAD: usize = 1536;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
pub struct Config {
    pub session: u32,
    pub payload_bytes: u16,
    /// Zero requests maximum sustainable throughput.
    pub frames_per_second: u32,
    pub frames: u32,
    /// Independent device stop, including when host requests cannot get through.
    pub duration_ms: u32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
pub enum Request {
    Start(Config),
    Status,
    Stop,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, Schema)]
pub struct Stats {
    pub session: u32,
    pub produced: u32,
    pub queued: u32,
    pub dropped: u32,
    pub complete: bool,
}

ergot::endpoint!(ControlEndpoint, Request, Stats, "bench/transport/control");

#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct Frame {
    pub session: u32,
    pub seq: u32,
    #[serde(serialize_with = "serialize_bytes")]
    pub data: heapless::Vec<u8, MAX_PAYLOAD>,
}

fn serialize_bytes<S: serde::Serializer>(data: &[u8], serializer: S) -> Result<S::Ok, S::Error> {
    serializer.serialize_bytes(data)
}

ergot::topic!(FrameTopic, Frame, "bench/transport/frame");

pub fn pattern(session: u32, seq: u32, i: usize) -> u8 {
    (session ^ seq.rotate_left((i % 31) as u32) ^ (i as u32 * 29)) as u8
}
