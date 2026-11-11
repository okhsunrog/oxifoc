//! Versioned, independently decoded telemetry blocks. All packing is lossless.
//! Production tag3 is fixed Packed28: tag1's 26 bytes plus a two-byte Hall angle
//! with explicit absence. Tags0..2 retain their historical decoding contract;
//! they reject new samples containing Hall rather than silently discarding it.
//!
//! The 32-byte header contains magic/version/encoding/flags/count (8 bytes),
//! session (u64), block index and cumulative producer drops (u32 each), then
//! seq/stride/ARR/payload length (u16 each). END replaces the final eight bytes
//! with total emitted records. Timestamps remain sequence-derived, as in the
//! original frame; a task-side send timestamp is not an acquisition timestamp.
//! Encoding tag2 adds byte deltas over tag1's packed layout: a full 26-byte
//! seed, then tag0+26 full bytes or tag1+u32 changed-byte mask+replacement
//! bytes in offset order. Only mask bits0..25 are valid. Every block starts
//! afresh; incompressible frames use full encoding and invalid packed input
//! uses the existing explicit Raw34 block fallback.

use crate::types::{FastTelemetry, TelemetryConfigAck};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

pub const RAW_MAGIC: [u8; 4] = *b"OXTR";
pub const RAW_HEADER_BYTES: usize = 32;
pub const RAW_MAX_RECORDS: usize = 64;
pub const RAW_MAX_BLOCK_BYTES: usize = RAW_HEADER_BYTES + RAW_MAX_RECORDS * 34;
const VERSION: u8 = 1;
const END: u8 = 1;
const FALLBACK: u8 = 2;
const VELOCITY: u8 = 4;
use crate::velocity_telemetry::{VELOCITY_RECORD_BYTES, VelocityDebugFrame};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RawEncoding {
    Raw34,
    Packed26,
    /// Independent 26-byte seed, then tagged full frames or byte replacements.
    Packed26Delta,
    /// Fixed production layout: Packed26 followed by an independent Hall angle.
    Packed28,
}
impl RawEncoding {
    const fn bytes(self) -> usize {
        match self {
            Self::Raw34 => 34,
            Self::Packed26 | Self::Packed26Delta => 26,
            Self::Packed28 => 28,
        }
    }
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
    pub encoding: RawEncoding,
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
    pub const fn is_fallback(&self) -> bool {
        self.flags & FALLBACK != 0
    }
    pub const fn payload_bytes(&self) -> usize {
        self.payload_len as usize
    }

    pub fn parse(bytes: &[u8]) -> Result<Self, RawDecodeError> {
        let b = bytes
            .get(..RAW_HEADER_BYTES)
            .ok_or(RawDecodeError::Truncated)?;
        if b[..4] != RAW_MAGIC || b[4] != VERSION {
            return Err(RawDecodeError::Header);
        }
        let encoding = match b[5] {
            0 => RawEncoding::Raw34,
            1 => RawEncoding::Packed26,
            2 => RawEncoding::Packed26Delta,
            3 => RawEncoding::Packed28,
            _ => return Err(RawDecodeError::Header),
        };
        let flags = b[6];
        let count = b[7];
        if flags & !(END | FALLBACK | VELOCITY) != 0
            || flags & VELOCITY != 0 && (encoding != RawEncoding::Packed28 || flags & FALLBACK != 0)
            || count as usize > RAW_MAX_RECORDS
            || flags & FALLBACK != 0 && encoding != RawEncoding::Raw34
        {
            return Err(RawDecodeError::Header);
        }
        let end = flags & END != 0;
        let payload_len = if end { 0 } else { read16(b, 30) };
        if end {
            if count != 0 || flags & !VELOCITY != END {
                return Err(RawDecodeError::Header);
            }
        } else {
            if count == 0 {
                return Err(RawDecodeError::Header);
            }
            let length = usize::from(payload_len);
            let valid = if flags & VELOCITY != 0 {
                length == count as usize * VELOCITY_RECORD_BYTES
            } else if encoding == RawEncoding::Packed26Delta {
                // The seed is fixed; each later frame is a tag+full frame or
                // a tag+four-byte mask followed by its changed bytes.
                (26 + (count as usize - 1) * 5..=26 + (count as usize - 1) * 27).contains(&length)
            } else {
                length == count as usize * encoding.bytes()
            };
            if !valid {
                return Err(RawDecodeError::Header);
            }
        }
        let header = Self {
            session: read64(b, 8),
            block_index: read32(b, 16),
            producer_drops: read32(b, 20),
            encoding,
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
            encoding: RawEncoding::Packed28,
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
        b[5] = match self.encoding {
            RawEncoding::Raw34 => 0,
            RawEncoding::Packed26 => 1,
            RawEncoding::Packed26Delta => 2,
            RawEncoding::Packed28 => 3,
        };
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

fn words(f: &FastTelemetry) -> [u16; 17] {
    [
        f.ia,
        f.ib,
        f.ic,
        f.vbus,
        f.angle,
        f.vd as u16,
        f.vq as u16,
        f.rpm as u16,
        f.seq,
        f.vphase_adc[0],
        f.vphase_adc[1],
        f.vphase_adc[2],
        f.pwm_ccr[0],
        f.pwm_ccr[1],
        f.pwm_ccr[2],
        f.pwm_arr,
        f.pwm_enables,
    ]
}
fn frame(w: [u16; 17]) -> FastTelemetry {
    FastTelemetry {
        ia: w[0],
        ib: w[1],
        ic: w[2],
        vbus: w[3],
        angle: w[4],
        hall_angle: crate::types::HallAngle::default(),
        vd: w[5] as i16,
        vq: w[6] as i16,
        rpm: w[7] as i16,
        seq: w[8],
        vphase_adc: [w[9], w[10], w[11]],
        pwm_ccr: [w[12], w[13], w[14]],
        pwm_arr: w[15],
        pwm_enables: w[16],
    }
}
fn packable(f: &FastTelemetry) -> bool {
    [f.ia, f.ib, f.ic].iter().all(|&v| v <= 4095)
        && (f.vphase_adc == [u16::MAX; 3] || f.vphase_adc.iter().all(|&v| v <= 4095))
        && f.pwm_enables & !127 == 0
}

fn pack26(f: &FastTelemetry, out: &mut [u8]) {
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
    ];
    for (encoded, value) in out[10..26].as_chunks_mut::<2>().0.iter_mut().zip(words) {
        *encoded = value.to_le_bytes();
    }
}

fn unpack26(b: &[u8], header: RawBlockHeader, index: usize) -> FastTelemetry {
    let mut w = [0; 17];
    let mut adc = [0; 6];
    for (i, p) in b[..9].chunks_exact(3).enumerate() {
        adc[i * 2] = u16::from(p[0]) | (u16::from(p[1] & 15) << 8);
        adc[i * 2 + 1] = u16::from(p[1] >> 4) | (u16::from(p[2]) << 4);
    }
    w[..3].copy_from_slice(&adc[..3]);
    w[9..12].copy_from_slice(if b[9] & 128 != 0 {
        &adc[3..]
    } else {
        &[u16::MAX; 3]
    });
    for (i, value) in w[3..8].iter_mut().enumerate() {
        *value = read16(b, 10 + i * 2);
    }
    for (i, value) in w[12..15].iter_mut().enumerate() {
        *value = read16(b, 20 + i * 2);
    }
    w[8] = header
        .first_seq
        .wrapping_add((index as u16).wrapping_mul(header.stride));
    w[15] = header.pwm_arr;
    w[16] = u16::from(b[9] & 127);
    frame(w)
}

/// Reusable fixed storage. A refused push leaves the complete block unchanged.
pub struct RawBlockEncoder {
    bytes: [u8; RAW_MAX_BLOCK_BYTES],
    header: RawBlockHeader,
    requested: RawEncoding,
    previous: [u8; 26],
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
            requested: RawEncoding::Raw34,
            previous: [0; 26],
            header: RawBlockHeader {
                session: 0,
                block_index: 0,
                encoding: RawEncoding::Raw34,
                producer_drops: 0,
                count: 0,
                first_seq: 0,
                stride: 0,
                pwm_arr: 0,
                total_records: 0,
                flags: 0,
                payload_len: 0,
            },
        }
    }
    pub fn begin(&mut self, session: u64, block_index: u32, encoding: RawEncoding, stride: u16) {
        self.requested = encoding;
        self.header = RawBlockHeader {
            session,
            block_index,
            encoding,
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
        self.len() == RAW_MAX_RECORDS
    }
    pub fn push(&mut self, f: &FastTelemetry) -> bool {
        if self.is_full() || self.header.is_end() {
            return false;
        }
        // Historical formats cannot represent Hall. Never silently drop it,
        // and never fall back from the production format to a different codec.
        let fits_packed = self.requested == RawEncoding::Raw34 || packable(f);
        if (self.requested != RawEncoding::Packed28 && f.hall_angle.encoded() != 0)
            || (self.requested == RawEncoding::Packed28 && !fits_packed)
        {
            return false;
        }
        if self.is_empty() {
            self.header.first_seq = f.seq;
            self.header.pwm_arr = f.pwm_arr;
            if !fits_packed {
                self.header.encoding = RawEncoding::Raw34;
                self.header.flags = FALLBACK;
            }
        }
        let packed = self.header.encoding != RawEncoding::Raw34;
        if packed
            && (!fits_packed
                || f.pwm_arr != self.header.pwm_arr
                || f.seq
                    != self.header.first_seq.wrapping_add(
                        u16::from(self.header.count).wrapping_mul(self.header.stride),
                    ))
        {
            return false;
        }
        let at = RAW_HEADER_BYTES + self.header.payload_bytes();
        let length = if self.header.encoding == RawEncoding::Packed26Delta {
            let mut current = [0_u8; 26];
            pack26(f, &mut current);
            let length = if self.is_empty() {
                self.bytes[at..at + 26].copy_from_slice(&current);
                26
            } else {
                let mut mask = 0_u32;
                for (i, (&new, &old)) in current.iter().zip(&self.previous).enumerate() {
                    mask |= u32::from(new != old) << i;
                }
                let length = 5 + mask.count_ones() as usize;
                if length < 27 {
                    self.bytes[at] = 1;
                    self.bytes[at + 1..at + 5].copy_from_slice(&mask.to_le_bytes());
                    let mut to = at + 5;
                    while mask != 0 {
                        let i = mask.trailing_zeros() as usize;
                        self.bytes[to] = current[i];
                        to += 1;
                        mask &= mask - 1;
                    }
                    length
                } else {
                    self.bytes[at] = 0;
                    self.bytes[at + 1..at + 27].copy_from_slice(&current);
                    27
                }
            };
            self.previous = current;
            length
        } else {
            let length = self.header.encoding.bytes();
            let out = &mut self.bytes[at..at + length];
            if packed {
                pack26(f, out);
                if self.header.encoding == RawEncoding::Packed28 {
                    out[26..28].copy_from_slice(&f.hall_angle.encoded().to_le_bytes());
                }
            } else {
                for (i, value) in words(f).iter().enumerate() {
                    out[i * 2..i * 2 + 2].copy_from_slice(&value.to_le_bytes());
                }
            }
            length
        };
        // At most 26+63*27 bytes for delta, or 64*34 for full fallback.
        self.header.payload_len += length as u16;
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
        debug_assert!(self.len() == 0);
        &mut self.bytes
    }
    pub fn end(
        &mut self,
        session: u64,
        block_index: u32,
        encoding: RawEncoding,
        total_records: u64,
        producer_drops: u32,
    ) -> &[u8] {
        self.begin(session, block_index, encoding, 0);
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
    if header.is_velocity() {
        for record in bytes[RAW_HEADER_BYTES..].chunks_exact(VELOCITY_RECORD_BYTES) {
            velocity
                .push(VelocityDebugFrame::decode(record).ok_or(RawDecodeError::Length)?)
                .map_err(|_| RawDecodeError::Length)?;
        }
        return Ok(DecodedRawBlock {
            header,
            samples,
            velocity,
        });
    }
    if header.encoding == RawEncoding::Packed26Delta && !header.is_end() {
        let mut data = &bytes[RAW_HEADER_BYTES..];
        let mut previous = [0_u8; 26];
        previous.copy_from_slice(data.get(..26).ok_or(RawDecodeError::Truncated)?);
        data = &data[26..];
        samples
            .push(unpack26(&previous, header, 0))
            .map_err(|_| RawDecodeError::Length)?;
        for index in 1..header.count as usize {
            let (&tag, rest) = data.split_first().ok_or(RawDecodeError::Truncated)?;
            data = rest;
            match tag {
                0 => {
                    previous.copy_from_slice(data.get(..26).ok_or(RawDecodeError::Truncated)?);
                    data = &data[26..];
                }
                1 => {
                    let bytes = data.get(..4).ok_or(RawDecodeError::Truncated)?;
                    let mut mask = read32(bytes, 0);
                    if mask >> 26 != 0 {
                        return Err(RawDecodeError::Header);
                    }
                    data = &data[4..];
                    while mask != 0 {
                        let (&replacement, rest) =
                            data.split_first().ok_or(RawDecodeError::Truncated)?;
                        previous[mask.trailing_zeros() as usize] = replacement;
                        data = rest;
                        mask &= mask - 1;
                    }
                }
                _ => return Err(RawDecodeError::Header),
            }
            samples
                .push(unpack26(&previous, header, index))
                .map_err(|_| RawDecodeError::Length)?;
        }
        if !data.is_empty() {
            return Err(RawDecodeError::Length);
        }
        return Ok(DecodedRawBlock {
            header,
            samples,
            velocity,
        });
    }
    for (index, b) in bytes[RAW_HEADER_BYTES..]
        .chunks_exact(header.encoding.bytes())
        .enumerate()
    {
        let mut sample = if header.encoding == RawEncoding::Raw34 {
            let mut w = [0; 17];
            for (i, value) in w.iter_mut().enumerate() {
                *value = read16(b, i * 2);
            }
            frame(w)
        } else {
            unpack26(b, header, index)
        };
        if header.encoding == RawEncoding::Packed28 {
            sample.hall_angle = crate::types::HallAngle::from_encoded(read16(b, 26));
        }
        samples.push(sample).map_err(|_| RawDecodeError::Length)?;
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
