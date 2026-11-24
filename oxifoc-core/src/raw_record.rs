//! A fast telemetry session recorded on the bridge, to be fetched later.
//!
//! The bridge opens the session at the controller as it does for a host, and
//! keeps the blocks in its own memory instead of sending them on. A
//! recording is the blocks one after another, Delta28 where that is smaller,
//! so [`crate::raw_telemetry::RawStreamDecoder`] reads it as it reads a
//! stream. It has no END block: it ends where it was stopped.
//!
//! [`RecordRequest`] and [`RecordResponse`] are the bridge's `cmd/raw_record`
//! endpoint. [`Recording`] is the memory itself.

use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use crate::{raw_telemetry::raw_block_len, types::TelemetryConfigAck};

/// Most bytes of a recording one reply carries.
pub const RECORD_CHUNK_BYTES: usize = 2048;

/// What a recording does when its memory is full.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RecordMode {
    /// Stops. The recording is the session's beginning.
    UntilFull,
    /// Drops its oldest blocks. The recording is the time before it was
    /// stopped, by a request or by a fault that stops the motor.
    Ring,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RecordRequest {
    Status,
    /// Open a session at the controller and record it. `session` is the
    /// requester's token, never zero: a repeated Start with the token of the
    /// recording that runs is answered with its status.
    Start {
        session: u64,
        fast_hz: u16,
        mode: RecordMode,
        /// The requester's clock, milliseconds since the Unix epoch. The
        /// bridge has no clock of its own and only keeps the value.
        started_unix_ms: u64,
    },
    Stop,
    /// Up to `max_bytes` of a stopped recording from byte `offset`, and no
    /// more than [`RECORD_CHUNK_BYTES`]. A reply shorter than asked is the
    /// recording's end.
    Read {
        offset: u32,
        max_bytes: u16,
    },
    /// Forget a stopped recording.
    Clear,
}

/// Why a recording stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RecordEnd {
    /// A Stop request.
    Request,
    /// [`RecordMode::UntilFull`] ran out of memory.
    Full,
    /// A fault stopped the motor; a ring keeps a second more.
    Fault,
    /// The controller ended the session.
    SessionEnded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RecordState {
    /// Nothing is kept.
    Empty,
    Recording,
    Stopped(RecordEnd),
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RecordStatus {
    pub state: RecordState,
    pub mode: RecordMode,
    pub session: u64,
    pub started_unix_ms: u64,
    /// The session as the controller acknowledged it.
    pub timing: TelemetryConfigAck,
    /// Bytes kept, and the memory there is for them.
    pub bytes: u32,
    pub capacity: u32,
    /// Blocks taken into the recording.
    pub blocks: u32,
    /// Blocks missing from the session's sequence when the next arrived.
    pub missed: u32,
    /// Bytes a ring dropped from its old end.
    pub overwritten: u32,
}

/// Why a request was not carried out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RecordRefusal {
    /// This bridge has no memory for recordings.
    NoMemory,
    /// A recording runs: stop it first.
    Recording,
    /// A stopped recording is kept: clear it first.
    Kept,
    /// A host holds the fast telemetry session.
    Busy,
    /// The controller did not open the session.
    Controller,
    Invalid,
}

#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub enum RecordResponse {
    Status(RecordStatus),
    Data(RecordChunk),
    Refused(RecordRefusal),
}

#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct RecordChunk {
    #[serde(serialize_with = "serialize_bytes")]
    pub data: heapless::Vec<u8, RECORD_CHUNK_BYTES>,
}

fn serialize_bytes<S: serde::Serializer>(bytes: &[u8], serializer: S) -> Result<S::Ok, S::Error> {
    serializer.serialize_bytes(bytes)
}

/// The blocks of a recording in one piece of memory.
///
/// A block is never split: when the next does not fit after the newest, a
/// ring continues at the start of the memory and drops whole blocks from its
/// old end to make room.
pub struct Recording<'a> {
    memory: &'a mut [u8],
    /// The oldest block.
    start: usize,
    /// One past the newest block.
    end: usize,
    /// Where the older blocks end, once the newer ones have continued at the
    /// start of the memory: `start..wrap` is then followed by `0..end`.
    wrap: Option<usize>,
    overwritten: u32,
}

impl<'a> Recording<'a> {
    pub fn new(memory: &'a mut [u8]) -> Self {
        Self {
            memory,
            start: 0,
            end: 0,
            wrap: None,
            overwritten: 0,
        }
    }

    pub fn clear(&mut self) {
        (self.start, self.end, self.wrap, self.overwritten) = (0, 0, None, 0);
    }

    pub fn capacity(&self) -> usize {
        self.memory.len()
    }

    pub fn len(&self) -> usize {
        match self.wrap {
            Some(wrap) => wrap - self.start + self.end,
            None => self.end - self.start,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Bytes a ring has dropped from its old end.
    pub fn overwritten(&self) -> u32 {
        self.overwritten
    }

    /// Appends an envelope: one block, or a block and its velocity
    /// companion. Returns false, storing nothing, when there is no room and
    /// the mode does not make any, or the envelope exceeds the memory.
    pub fn push(&mut self, envelope: &[u8], mode: RecordMode) -> bool {
        let len = envelope.len();
        let capacity = self.memory.len();
        if len > capacity || (mode == RecordMode::UntilFull && self.end + len > capacity) {
            return false;
        }
        loop {
            match self.wrap {
                None if self.end + len <= capacity => break,
                None => (self.wrap, self.end) = (Some(self.end), 0),
                Some(_) if self.end + len <= self.start => break,
                Some(wrap) => {
                    // A header that does not parse drops the rest of the old
                    // blocks, so the loop always ends.
                    let old = &self.memory[self.start..wrap];
                    let block = raw_block_len(old).map_or(old.len(), |block| block.min(old.len()));
                    self.start += block;
                    self.overwritten = self.overwritten.wrapping_add(block as u32);
                    if self.start == wrap {
                        (self.start, self.wrap) = (0, None);
                    }
                }
            }
        }
        self.memory[self.end..self.end + len].copy_from_slice(envelope);
        self.end += len;
        true
    }

    /// Copies the recording from byte `offset` into `out` and returns the
    /// bytes copied: fewer than `out` holds at the recording's end.
    pub fn read(&self, offset: usize, out: &mut [u8]) -> usize {
        let (older, newer) = match self.wrap {
            Some(wrap) => (&self.memory[self.start..wrap], &self.memory[..self.end]),
            None => (&self.memory[self.start..self.end], &[][..]),
        };
        let mut copied = 0;
        for (part, skip) in [(older, offset), (newer, offset.saturating_sub(older.len()))] {
            let Some(rest) = part.get(skip..) else {
                continue;
            };
            let take = rest.len().min(out.len() - copied);
            out[copied..copied + take].copy_from_slice(&rest[..take]);
            copied += take;
        }
        copied
    }
}

#[cfg(test)]
#[path = "raw_record_tests.rs"]
mod tests;
