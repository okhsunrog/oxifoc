//! Delta28: a Packed28 block with its records stored as differences.
//!
//! The controller packs fast telemetry as Packed28 and has no time to spare
//! for more. A node that relays or stores the stream (the bridge) rewrites
//! each block as Delta28, and a reader expands it to the identical Packed28
//! block. Each block stands alone, so a missing block costs only its own
//! records.
//!
//! A Packed28 record is sixteen fields in one little-endian bit string: six
//! 12-bit ADC words, the enable byte and nine 16-bit words. A Delta28 block
//! keeps the 32-byte header with tag 4 and its own payload length, then:
//!
//! - the first record, as Packed28 (28 bytes);
//! - sixteen 5-bit widths, one per field (10 bytes);
//! - for each field in turn, `count - 1` differences from the previous
//!   record, each `width` bits, zero-padded to a whole byte at the end.
//!
//! A difference is taken modulo the field's size, so an angle that wraps
//! costs no more than one that does not, and is stored zigzag-coded. A field
//! that does not change takes no bits. On an unloaded motor at +120 rad/s a
//! 20 kHz stream comes to about 10 bytes a record.
//!
//! A block that would not shrink stays Packed28: a stream may mix both.

use crate::raw_telemetry::{RAW_HEADER_BYTES, RAW_MAGIC, RAW_MAX_RECORDS, RawDecodeError};

const RECORD_BYTES: usize = 28;
const VERSION: u8 = 1;
const PACKED28_TAG: u8 = 3;
const DELTA28_TAG: u8 = 4;
const FIELDS: usize = 16;
/// Bits of each field, in record order.
const FIELD_BITS: [u8; FIELDS] = [
    12, 12, 12, 12, 12, 12, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16,
];
const WIDTH_BITS: usize = 5;
const WIDTH_BYTES: usize = (FIELDS * WIDTH_BITS).div_ceil(8);
const _: () = {
    let mut bits = 0;
    let mut field = 0;
    while field < FIELDS {
        bits += FIELD_BITS[field] as usize;
        field += 1;
    }
    assert!(bits == RECORD_BYTES * 8);
};

/// Largest Packed28 block: what [`expand`] may write.
pub const PLAIN_BLOCK_BYTES: usize = RAW_HEADER_BYTES + RAW_MAX_RECORDS * RECORD_BYTES;

/// Up to 16 bits starting at bit `at`: they span at most three bytes.
fn read_bits(bytes: &[u8], at: usize, bits: usize) -> u32 {
    let mut word = 0;
    for (index, &byte) in bytes[at / 8..].iter().take(3).enumerate() {
        word |= u32::from(byte) << (8 * index);
    }
    (word >> (at % 8)) & ((1 << bits) - 1)
}

/// Up to 16 bits starting at bit `at`. The target bits must be zero.
fn write_bits(bytes: &mut [u8], at: usize, value: u32) {
    let word = value << (at % 8);
    for (index, byte) in bytes[at / 8..].iter_mut().take(3).enumerate() {
        *byte |= (word >> (8 * index)) as u8;
    }
}

fn field_offset(field: usize) -> usize {
    FIELD_BITS[..field]
        .iter()
        .map(|&bits| usize::from(bits))
        .sum()
}

/// The zigzag code of `current - previous` taken modulo the field's size.
fn difference(previous: u32, current: u32, bits: usize) -> u32 {
    let mask = (1_u32 << bits) - 1;
    let delta = current.wrapping_sub(previous) & mask;
    if delta >> (bits - 1) == 0 {
        delta << 1
    } else {
        ((mask - delta) << 1) | 1
    }
}

fn apply(previous: u32, code: u32, bits: usize) -> u32 {
    let mask = (1_u32 << bits) - 1;
    let delta = if code & 1 == 0 {
        code >> 1
    } else {
        mask - (code >> 1)
    };
    previous.wrapping_add(delta) & mask
}

/// True for a block in the Delta28 layout. Says nothing about its validity.
pub fn is_delta(block: &[u8]) -> bool {
    block.len() >= RAW_HEADER_BYTES && block[..4] == RAW_MAGIC && block[5] == DELTA28_TAG
}

struct Shape {
    count: usize,
    payload: usize,
}

/// Checks a Delta28 header and returns its record count and payload length.
fn delta_shape(block: &[u8]) -> Result<Shape, RawDecodeError> {
    let header = block
        .get(..RAW_HEADER_BYTES)
        .ok_or(RawDecodeError::Truncated)?;
    if header[..4] != RAW_MAGIC || header[4] != VERSION || header[5] != DELTA28_TAG {
        return Err(RawDecodeError::Header);
    }
    let count = usize::from(header[7]);
    let payload = usize::from(u16::from_le_bytes([header[30], header[31]]));
    // Flags mark END and velocity blocks, which are never rewritten. One
    // record has no differences, and a rewrite always shrinks the block.
    if header[6] != 0
        || !(2..=RAW_MAX_RECORDS).contains(&count)
        || payload < RECORD_BYTES + WIDTH_BYTES
        || payload >= count * RECORD_BYTES
    {
        return Err(RawDecodeError::Header);
    }
    Ok(Shape { count, payload })
}

/// Bytes of the block that starts with this Delta28 header.
pub fn delta_block_len(header: &[u8]) -> Result<usize, RawDecodeError> {
    Ok(RAW_HEADER_BYTES + delta_shape(header)?.payload)
}

/// Rewrites one Packed28 block as Delta28 in `out` and returns its length.
///
/// Returns `None`, leaving the caller to pass the block on as it is, for
/// anything but a complete Packed28 block of two or more records, and for a
/// block that Delta28 would not make smaller.
pub fn compress(block: &[u8], out: &mut [u8]) -> Option<usize> {
    let header = block.get(..RAW_HEADER_BYTES)?;
    let count = usize::from(header[7]);
    if header[..4] != RAW_MAGIC
        || header[4] != VERSION
        || header[5] != PACKED28_TAG
        || header[6] != 0
        || !(2..=RAW_MAX_RECORDS).contains(&count)
        || block.len() != RAW_HEADER_BYTES + count * RECORD_BYTES
    {
        return None;
    }
    let records = &block[RAW_HEADER_BYTES..];
    let record = |index: usize| &records[index * RECORD_BYTES..][..RECORD_BYTES];

    let mut widths = [0_usize; FIELDS];
    let mut bits = 0;
    for (field, width) in widths.iter_mut().enumerate() {
        let (at, size) = (field_offset(field), usize::from(FIELD_BITS[field]));
        let mut previous = read_bits(record(0), at, size);
        let mut widest = 0;
        for index in 1..count {
            let current = read_bits(record(index), at, size);
            widest |= difference(previous, current, size);
            previous = current;
        }
        *width = (u32::BITS - widest.leading_zeros()) as usize;
        bits += *width * (count - 1);
    }
    let payload = RECORD_BYTES + WIDTH_BYTES + bits.div_ceil(8);
    if payload >= count * RECORD_BYTES {
        return None;
    }
    let out = out.get_mut(..RAW_HEADER_BYTES + payload)?;
    out.fill(0);
    out[..RAW_HEADER_BYTES].copy_from_slice(header);
    out[5] = DELTA28_TAG;
    out[30..32].copy_from_slice(&(payload as u16).to_le_bytes());
    let body = &mut out[RAW_HEADER_BYTES..];
    body[..RECORD_BYTES].copy_from_slice(record(0));
    for (field, &width) in widths.iter().enumerate() {
        write_bits(&mut body[RECORD_BYTES..], field * WIDTH_BITS, width as u32);
    }
    let mut cursor = (RECORD_BYTES + WIDTH_BYTES) * 8;
    for (field, &width) in widths.iter().enumerate() {
        if width == 0 {
            continue;
        }
        let (at, size) = (field_offset(field), usize::from(FIELD_BITS[field]));
        let mut previous = read_bits(record(0), at, size);
        for index in 1..count {
            let current = read_bits(record(index), at, size);
            write_bits(body, cursor, difference(previous, current, size));
            cursor += width;
            previous = current;
        }
    }
    Some(RAW_HEADER_BYTES + payload)
}

/// Rewrites a Delta28 block as the Packed28 block it was made from, in `out`,
/// and returns its length. `block` must be exactly one block.
///
/// Every byte of the block is accounted for: a length that does not follow
/// from the widths, a width wider than its field and set padding bits are
/// all refused, so a damaged block cannot pass for a different one.
pub fn expand(block: &[u8], out: &mut [u8]) -> Result<usize, RawDecodeError> {
    let Shape { count, payload } = delta_shape(block)?;
    if block.len() != RAW_HEADER_BYTES + payload {
        return Err(RawDecodeError::Length);
    }
    let body = &block[RAW_HEADER_BYTES..];
    let mut widths = [0_usize; FIELDS];
    let mut bits = 0;
    for (field, width) in widths.iter_mut().enumerate() {
        *width = read_bits(&body[RECORD_BYTES..], field * WIDTH_BITS, WIDTH_BITS) as usize;
        if *width > usize::from(FIELD_BITS[field]) {
            return Err(RawDecodeError::Header);
        }
        bits += *width * (count - 1);
    }
    let end = (RECORD_BYTES + WIDTH_BYTES) * 8 + bits;
    if payload != end.div_ceil(8) || read_bits(body, end, payload * 8 - end) != 0 {
        return Err(RawDecodeError::Length);
    }

    let plain = RAW_HEADER_BYTES + count * RECORD_BYTES;
    let out = out.get_mut(..plain).ok_or(RawDecodeError::Length)?;
    out.fill(0);
    out[..RAW_HEADER_BYTES].copy_from_slice(&block[..RAW_HEADER_BYTES]);
    out[5] = PACKED28_TAG;
    out[30..32].copy_from_slice(&((count * RECORD_BYTES) as u16).to_le_bytes());
    let records = &mut out[RAW_HEADER_BYTES..];
    records[..RECORD_BYTES].copy_from_slice(&body[..RECORD_BYTES]);
    let mut cursor = (RECORD_BYTES + WIDTH_BYTES) * 8;
    for (field, &width) in widths.iter().enumerate() {
        let (at, size) = (field_offset(field), usize::from(FIELD_BITS[field]));
        let mut previous = read_bits(records, at, size);
        for index in 1..count {
            let code = read_bits(body, cursor, width);
            cursor += width;
            previous = apply(previous, code, size);
            write_bits(records, index * RECORD_BYTES * 8 + at, previous);
        }
    }
    Ok(plain)
}

#[cfg(test)]
#[path = "raw_delta_tests.rs"]
mod tests;
