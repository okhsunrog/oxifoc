//! Versioned, independently decoded telemetry blocks. All packing is lossless.
//! The only native record layout is Packed28 (in-block tag 3): the three phase
//! currents and the optional phase voltages as 12-bit pairs, an enable byte
//! whose top bit marks phase voltages present, eight 16-bit words (vbus, FOC
//! angle, vd, vq, rpm, three CCRs) and a two-byte Hall angle with explicit
//! absence. Sequence and ARR come from the header, so a block holds one
//! contiguous, constant-ARR run; a frame that does not pack (an ADC word above
//! 12 bits, a mixed phase-voltage sentinel, an enable bit above 6) is refused
//! and the session fails rather than changing format.
//!
//! The 32-byte header contains magic/version/encoding/flags/count (8 bytes),
//! session (u64), block index and cumulative producer drops (u32 each), then
//! seq/stride/ARR/payload length (u16 each). END replaces the final eight bytes
//! with total emitted records. Timestamps remain sequence-derived, as in the
//! original frame; a task-side send timestamp is not an acquisition timestamp.
//! Velocity companion blocks share the header, tagged Packed28 with the
//! VELOCITY flag, and carry fixed velocity records instead.

use crate::types::{FastTelemetry, HallAngle, TelemetryConfigAck};
use crate::velocity_telemetry::{VELOCITY_RECORD_BYTES, VelocityDebugFrame};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

pub const RAW_MAGIC: [u8; 4] = *b"OXTR";
pub const RAW_HEADER_BYTES: usize = 32;
pub const RAW_MAX_RECORDS: usize = 64;
const PACKED28_BYTES: usize = 28;
/// Velocity records a full native block leaves room for in the same envelope.
const COMPANION_RECORDS: usize = 11;
/// One envelope: a full Packed28 block, then a velocity companion block. The
/// 2208 bytes equal the former full-width (34-byte record) bound, so the
/// companion keeps the room it had; the largest single block, 64 velocity
/// records after Stop, also fits.
pub const RAW_MAX_BLOCK_BYTES: usize = RAW_HEADER_BYTES
    + RAW_MAX_RECORDS * PACKED28_BYTES
    + RAW_HEADER_BYTES
    + COMPANION_RECORDS * VELOCITY_RECORD_BYTES;
const _: () =
    assert!(RAW_MAX_BLOCK_BYTES >= RAW_HEADER_BYTES + RAW_MAX_RECORDS * VELOCITY_RECORD_BYTES);
/// Smallest block limit an Arm may request: a header and one record.
pub const RAW_MIN_BLOCK_BYTES: usize = RAW_HEADER_BYTES + PACKED28_BYTES;
const VERSION: u8 = 1;
const PACKED28_TAG: u8 = 3;
const END: u8 = 1;
const VELOCITY: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RawEncoding {
    /// Fixed production layout; see the module documentation.
    Packed28,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RawTelemetryRequest {
    Status,
    Arm {
        session: u64,
        /// Latest token returned by Status. Fences delayed Arm requests from
        /// earlier captures without retaining an unbounded replay cache.
        previous_session: u64,
        encoding: RawEncoding,
        fast_hz: u16,
        /// Largest block, in bytes, the requester's link carries in one
        /// Ergot frame. The device fills blocks up to the smaller of this,
        /// its own link limit and [`RAW_MAX_BLOCK_BYTES`]; below
        /// [`RAW_MIN_BLOCK_BYTES`] the Arm is invalid.
        max_block_bytes: u16,
    },
    Stop {
        session: u64,
    },
}

/// Separate Arm endpoint preserves the existing raw control ICD key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct VelocityDebugArm {
    pub session: u64,
    pub previous_session: u64,
    pub fast_hz: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RawTelemetryStatus {
    Armed,
    Stopping,
    Stopped,
    Busy,
    Invalid,
    Unavailable,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawTelemetryAck {
    pub session: u64,
    pub encoding: RawEncoding,
    pub status: RawTelemetryStatus,
    pub timing: TelemetryConfigAck,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawBlockHeader {
    pub session: u64,
    pub block_index: u32,
    pub producer_drops: u32,
    pub count: u8,
    pub first_seq: u16,
    pub stride: u16,
    pub pwm_arr: u16,
    /// Meaningful only for END, whose count and payload are zero.
    pub total_records: u64,
    flags: u8,
    payload_len: u16,
}
impl RawBlockHeader {
    pub const fn is_end(&self) -> bool {
        self.flags & END != 0
    }
    pub const fn is_velocity(&self) -> bool {
        self.flags & VELOCITY != 0
    }
    pub const fn payload_bytes(&self) -> usize {
        self.payload_len as usize
    }

    pub fn parse(bytes: &[u8]) -> Result<Self, RawDecodeError> {
        let b = bytes
            .get(..RAW_HEADER_BYTES)
            .ok_or(RawDecodeError::Truncated)?;
        if b[..4] != RAW_MAGIC || b[4] != VERSION || b[5] != PACKED28_TAG {
            return Err(RawDecodeError::Header);
        }
        let flags = b[6];
        let count = b[7];
        if flags & !(END | VELOCITY) != 0 || count as usize > RAW_MAX_RECORDS {
            return Err(RawDecodeError::Header);
        }
        let end = flags & END != 0;
        let payload_len = if end { 0 } else { read16(b, 30) };
        if end {
            if count != 0 {
                return Err(RawDecodeError::Header);
            }
        } else {
            let record = if flags & VELOCITY != 0 {
                VELOCITY_RECORD_BYTES
            } else {
                PACKED28_BYTES
            };
            if count == 0 || usize::from(payload_len) != count as usize * record {
                return Err(RawDecodeError::Header);
            }
        }
        let header = Self {
            session: read64(b, 8),
            block_index: read32(b, 16),
            producer_drops: read32(b, 20),
            flags,
            count,
            first_seq: if end { 0 } else { read16(b, 24) },
            stride: if end { 0 } else { read16(b, 26) },
            pwm_arr: if end { 0 } else { read16(b, 28) },
            total_records: if end { read64(b, 24) } else { 0 },
            payload_len,
        };
        if header.session == 0 {
            return Err(RawDecodeError::Header);
        }
        Ok(header)
    }

    /// Encode a diagnostic block header. Block indexes share the fast session sequence.
    pub fn velocity(
        session: u64,
        block_index: u32,
        count: u8,
        drops: u32,
        end_total: Option<u64>,
    ) -> Self {
        Self {
            session,
            block_index,
            producer_drops: drops,
            count,
            first_seq: 0,
            stride: 0,
            pwm_arr: 0,
            flags: VELOCITY | if end_total.is_some() { END } else { 0 },
            total_records: end_total.unwrap_or(0),
            payload_len: u16::from(count) * VELOCITY_RECORD_BYTES as u16,
        }
    }

    pub fn write(self, b: &mut [u8]) {
        b[..4].copy_from_slice(&RAW_MAGIC);
        b[4] = VERSION;
        b[5] = PACKED28_TAG;
        b[6] = self.flags;
        b[7] = self.count;
        b[8..16].copy_from_slice(&self.session.to_le_bytes());
        b[16..20].copy_from_slice(&self.block_index.to_le_bytes());
        b[20..24].copy_from_slice(&self.producer_drops.to_le_bytes());
        if self.is_end() {
            b[24..32].copy_from_slice(&self.total_records.to_le_bytes());
        } else {
            b[24..26].copy_from_slice(&self.first_seq.to_le_bytes());
            b[26..28].copy_from_slice(&self.stride.to_le_bytes());
            b[28..30].copy_from_slice(&self.pwm_arr.to_le_bytes());
            b[30..32].copy_from_slice(&(self.payload_bytes() as u16).to_le_bytes());
        }
    }
}

fn read16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}
fn read32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(core::array::from_fn(|i| b[at + i]))
}
fn read64(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(core::array::from_fn(|i| b[at + i]))
}

fn packable(f: &FastTelemetry) -> bool {
    [f.ia, f.ib, f.ic].iter().all(|&v| v <= 4095)
        && (f.vphase_adc == [u16::MAX; 3] || f.vphase_adc.iter().all(|&v| v <= 4095))
        && f.pwm_enables & !127 == 0
}

fn pack28(f: &FastTelemetry, out: &mut [u8]) {
    let present = f.vphase_adc != [u16::MAX; 3];
    let p = if present { f.vphase_adc } else { [0; 3] };
    for (encoded, (a, b)) in
        out[..9]
            .as_chunks_mut::<3>()
            .0
            .iter_mut()
            .zip([(f.ia, f.ib), (f.ic, p[0]), (p[1], p[2])])
    {
        *encoded = [a as u8, ((a >> 8) | (b << 4)) as u8, (b >> 4) as u8];
    }
    out[9] = f.pwm_enables as u8 | (u8::from(present) << 7);
    let words = [
        f.vbus,
        f.angle,
        f.vd as u16,
        f.vq as u16,
        f.rpm as u16,
        f.pwm_ccr[0],
        f.pwm_ccr[1],
        f.pwm_ccr[2],
        f.hall_angle.encoded(),
    ];
    for (encoded, value) in out[10..28].as_chunks_mut::<2>().0.iter_mut().zip(words) {
        *encoded = value.to_le_bytes();
    }
}

fn unpack28(b: &[u8], header: RawBlockHeader, index: usize) -> FastTelemetry {
    let mut adc = [0; 6];
    for (i, p) in b[..9].as_chunks::<3>().0.iter().enumerate() {
        adc[i * 2] = u16::from(p[0]) | (u16::from(p[1] & 15) << 8);
        adc[i * 2 + 1] = u16::from(p[1] >> 4) | (u16::from(p[2]) << 4);
    }
    let word = |i: usize| read16(b, 10 + i * 2);
    FastTelemetry {
        ia: adc[0],
        ib: adc[1],
        ic: adc[2],
        vbus: word(0),
        angle: word(1),
        hall_angle: HallAngle::from_encoded(word(8)),
        vd: word(2) as i16,
        vq: word(3) as i16,
        rpm: word(4) as i16,
        seq: header
            .first_seq
            .wrapping_add((index as u16).wrapping_mul(header.stride)),
        vphase_adc: if b[9] & 128 != 0 {
            [adc[3], adc[4], adc[5]]
        } else {
            [u16::MAX; 3]
        },
        pwm_ccr: [word(5), word(6), word(7)],
        pwm_arr: header.pwm_arr,
        pwm_enables: u16::from(b[9] & 127),
    }
}

/// Reusable fixed storage. A refused push leaves the complete block unchanged.
pub struct RawBlockEncoder {
    bytes: [u8; RAW_MAX_BLOCK_BYTES],
    header: RawBlockHeader,
    max_records: u8,
}
impl Default for RawBlockEncoder {
    fn default() -> Self {
        Self::new()
    }
}
impl RawBlockEncoder {
    pub const fn new() -> Self {
        Self {
            bytes: [0; RAW_MAX_BLOCK_BYTES],
            header: RawBlockHeader {
                session: 0,
                block_index: 0,
                producer_drops: 0,
                count: 0,
                first_seq: 0,
                stride: 0,
                pwm_arr: 0,
                total_records: 0,
                flags: 0,
                payload_len: 0,
            },
            max_records: RAW_MAX_RECORDS as u8,
        }
    }
    /// Records per block that fit in `max_block_bytes` (at least one, at
    /// most [`RAW_MAX_RECORDS`]). Applies to the following blocks.
    pub fn set_block_limit(&mut self, max_block_bytes: usize) {
        let records = max_block_bytes.saturating_sub(RAW_HEADER_BYTES) / PACKED28_BYTES;
        self.max_records = records.clamp(1, RAW_MAX_RECORDS) as u8;
    }
    pub fn begin(&mut self, session: u64, block_index: u32, stride: u16) {
        self.header = RawBlockHeader {
            session,
            block_index,
            producer_drops: 0,
            count: 0,
            first_seq: 0,
            stride,
            pwm_arr: 0,
            total_records: 0,
            flags: 0,
            payload_len: 0,
        };
    }
    pub const fn len(&self) -> usize {
        self.header.count as usize
    }
    pub const fn is_empty(&self) -> bool {
        self.header.count == 0
    }
    pub const fn is_full(&self) -> bool {
        self.header.count >= self.max_records
    }
    /// Refuses a frame that does not pack, or that breaks the block's
    /// sequence run or ARR; the caller starts a new block or fails the session.
    pub fn push(&mut self, f: &FastTelemetry) -> bool {
        if self.is_full() || self.header.is_end() || !packable(f) {
            return false;
        }
        if self.is_empty() {
            self.header.first_seq = f.seq;
            self.header.pwm_arr = f.pwm_arr;
        } else if f.pwm_arr != self.header.pwm_arr
            || f.seq
                != self
                    .header
                    .first_seq
                    .wrapping_add(u16::from(self.header.count).wrapping_mul(self.header.stride))
        {
            return false;
        }
        let at = RAW_HEADER_BYTES + self.header.payload_bytes();
        pack28(f, &mut self.bytes[at..at + PACKED28_BYTES]);
        self.header.payload_len += PACKED28_BYTES as u16;
        self.header.count += 1;
        true
    }
    pub fn finish(&mut self, producer_drops: u32) -> &[u8] {
        self.header.producer_drops = producer_drops;
        self.header.write(&mut self.bytes[..RAW_HEADER_BYTES]);
        &self.bytes[..RAW_HEADER_BYTES + self.header.payload_bytes()]
    }

    #[cfg(feature = "fast-telemetry")]
    pub(crate) fn finish_with_suffix(
        &mut self,
        producer_drops: u32,
        append: impl FnOnce(&mut [u8]) -> usize,
    ) -> &[u8] {
        let len = self.finish(producer_drops).len();
        let added = append(&mut self.bytes[len..]);
        &self.bytes[..len + added]
    }

    #[cfg(feature = "fast-telemetry")]
    pub(crate) fn empty_scratch(&mut self) -> &mut [u8] {
        debug_assert!(self.is_empty());
        &mut self.bytes
    }
    pub fn end(
        &mut self,
        session: u64,
        block_index: u32,
        total_records: u64,
        producer_drops: u32,
    ) -> &[u8] {
        self.begin(session, block_index, 0);
        self.header.flags = END;
        self.header.total_records = total_records;
        self.finish(producer_drops)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawDecodeError {
    Header,
    Truncated,
    Length,
}

#[derive(Debug)]
pub struct DecodedRawBlock {
    pub header: RawBlockHeader,
    pub samples: heapless::Vec<FastTelemetry, RAW_MAX_RECORDS>,
    pub velocity: heapless::Vec<VelocityDebugFrame, RAW_MAX_RECORDS>,
}

pub fn decode_raw_block(bytes: &[u8]) -> Result<DecodedRawBlock, RawDecodeError> {
    let header = RawBlockHeader::parse(bytes)?;
    if bytes.len() != RAW_HEADER_BYTES + header.payload_bytes() {
        return Err(RawDecodeError::Length);
    }
    let mut samples = heapless::Vec::new();
    let mut velocity = heapless::Vec::new();
    let payload = &bytes[RAW_HEADER_BYTES..];
    if header.is_velocity() {
        for record in payload.as_chunks::<VELOCITY_RECORD_BYTES>().0 {
            velocity
                .push(VelocityDebugFrame::decode(record).ok_or(RawDecodeError::Length)?)
                .map_err(|_| RawDecodeError::Length)?;
        }
    } else {
        for (index, record) in payload.as_chunks::<PACKED28_BYTES>().0.iter().enumerate() {
            samples
                .push(unpack28(record, header, index))
                .map_err(|_| RawDecodeError::Length)?;
        }
    }
    Ok(DecodedRawBlock {
        header,
        samples,
        velocity,
    })
}

/// Strict incremental parser; reset after transport/session abandonment. The
/// host may seek RAW_MAGIC before its first matching-session block, then use
/// this strict parser so corruption cannot be silently hidden mid-capture.
pub struct RawStreamDecoder {
    bytes: heapless::Vec<u8, RAW_MAX_BLOCK_BYTES>,
}
impl Default for RawStreamDecoder {
    fn default() -> Self {
        Self::new()
    }
}
impl RawStreamDecoder {
    pub const fn new() -> Self {
        Self {
            bytes: heapless::Vec::new(),
        }
    }
    pub fn reset(&mut self) {
        self.bytes.clear();
    }
    pub fn push(
        &mut self,
        mut input: &[u8],
        mut on_block: impl FnMut(DecodedRawBlock),
    ) -> Result<(), RawDecodeError> {
        let result = (|| {
            while !input.is_empty() {
                let expected = if self.bytes.len() < RAW_HEADER_BYTES {
                    RAW_HEADER_BYTES
                } else {
                    RAW_HEADER_BYTES + RawBlockHeader::parse(&self.bytes)?.payload_bytes()
                };
                let take = (expected - self.bytes.len()).min(input.len());
                self.bytes
                    .extend_from_slice(&input[..take])
                    .map_err(|_| RawDecodeError::Length)?;
                input = &input[take..];
                if self.bytes.len() >= RAW_HEADER_BYTES {
                    let total =
                        RAW_HEADER_BYTES + RawBlockHeader::parse(&self.bytes)?.payload_bytes();
                    if self.bytes.len() == total {
                        on_block(decode_raw_block(&self.bytes)?);
                        self.bytes.clear();
                    }
                }
            }
            Ok(())
        })();
        if result.is_err() {
            self.reset();
        }
        result
    }
}

#[cfg(test)]
#[path = "raw_telemetry_tests.rs"]
mod tests;
