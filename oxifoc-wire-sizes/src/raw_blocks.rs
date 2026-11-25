//! Raw telemetry blocks for the `telemetry/raw_block` rows. Their Delta28
//! size depends on the signals, so they come from captured records instead
//! of the schema.

use std::fmt;

use oxifoc_core::{
    icd::raw_block_limit,
    raw_delta::compress_envelope,
    raw_telemetry::{
        RAW_HEADER_BYTES, RAW_MAX_BLOCK_BYTES, RAW_MIN_BLOCK_BYTES, RawBlockEncoder,
        RawDecodeError, decode_raw_block, raw_block_header, raw_block_len,
    },
};

/// The bundled fixture: 8 Packed28 blocks of 64 consecutive control-cycle
/// records each (1824 bytes a block), rebuilt from the decoded columns of
/// bench captures of a driven motor, with a valid header and a placeholder
/// session id. No test data in `oxifoc-core` stands in for it: its
/// raw-delta tests use synthetic sine waves, which compress differently
/// from a real motor.
pub const BUNDLED_FIXTURE: &[u8] = include_bytes!("../fixture/packed28-blocks.bin");

/// MTU of the BLE link (`oxifoc-host-lib/src/transport/ble.rs`, `BLE_MTU`).
/// A BLE host arms sessions with the raw block limit of this MTU.
const BLE_MTU: u16 = 505;
const RECORD_BYTES: usize = RAW_MIN_BLOCK_BYTES - RAW_HEADER_BYTES;

pub struct RawBlocks {
    /// Packed28 length of each fixture block.
    pub packed_full: Vec<usize>,
    /// Each fixture block, Delta28-compressed.
    pub delta_full: Vec<Vec<u8>>,
    /// The fixture's records re-cut at the BLE block limit: Packed28 length
    /// of each full block (a short tail is dropped).
    pub packed_ble: Vec<usize>,
    /// Those blocks, Delta28-compressed.
    pub delta_ble: Vec<Vec<u8>>,
    /// The END block that closes the re-cut session.
    pub end: Vec<u8>,
}

impl RawBlocks {
    /// Splits concatenated Packed28 blocks and compresses each.
    pub fn from_fixture(data: &[u8], session: u64) -> Result<Self, RawDecodeError> {
        let ble_limit = usize::from(ble_block_limit());
        let ble_records = ble_block_records();
        let mut blocks = Self {
            packed_full: Vec::new(),
            delta_full: Vec::new(),
            packed_ble: Vec::new(),
            delta_ble: Vec::new(),
            end: Vec::new(),
        };
        let mut compressed = vec![0; RAW_MAX_BLOCK_BYTES];
        let mut compress = |block: &[u8]| -> Result<Vec<u8>, RawDecodeError> {
            let len = compress_envelope(block, &mut compressed)?;
            Ok(compressed[..len].to_vec())
        };
        let mut encoder = RawBlockEncoder::new();
        let mut ble_index = 0;
        let mut records = 0;
        let mut rest = data;
        while rest.len() >= RAW_HEADER_BYTES {
            let (block, tail) = rest
                .split_at_checked(raw_block_len(rest)?)
                .ok_or(RawDecodeError::Truncated)?;
            rest = tail;
            blocks.packed_full.push(block.len());
            blocks.delta_full.push(compress(block)?);

            let header = raw_block_header(block)?;
            let samples = decode_raw_block(block)?.samples;
            records += samples.len();
            let mut next = samples.iter().peekable();
            while next.peek().is_some() {
                encoder.set_block_limit(ble_limit);
                encoder.begin(header.session, ble_index, header.stride);
                ble_index += 1;
                while next.next_if(|sample| encoder.push(sample)).is_some() {}
                let ble_block = encoder.finish(0);
                if ble_block.len() < RAW_HEADER_BYTES + ble_records * RECORD_BYTES {
                    continue;
                }
                blocks.packed_ble.push(ble_block.len());
                blocks.delta_ble.push(compress(ble_block)?);
            }
        }
        blocks.end = encoder.end(session, ble_index, records as u64, 0).to_vec();
        Ok(blocks)
    }
}

pub fn ble_block_limit() -> u16 {
    raw_block_limit(BLE_MTU)
}

/// Packed28 records in a block at the BLE limit.
pub fn ble_block_records() -> usize {
    (usize::from(ble_block_limit()) - RAW_HEADER_BYTES) / RECORD_BYTES
}

/// The smallest, the median and the largest of `blocks` by length.
pub struct Spread<'a> {
    pub smallest: &'a [u8],
    pub median: &'a [u8],
    pub largest: &'a [u8],
}

impl<'a> Spread<'a> {
    pub fn of(blocks: &'a [Vec<u8>]) -> Option<Self> {
        let mut sorted: Vec<&[u8]> = blocks.iter().map(Vec::as_slice).collect();
        sorted.sort_by_key(|b| b.len());
        Some(Self {
            smallest: sorted.first()?,
            median: sorted[sorted.len() / 2],
            largest: sorted.last()?,
        })
    }
}

/// Sorted lengths, for the fixture summary.
pub struct Lengths<'a>(pub &'a [Vec<u8>]);

impl fmt::Display for Lengths<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut lengths: Vec<usize> = self.0.iter().map(Vec::len).collect();
        lengths.sort_unstable();
        write!(f, "{lengths:?}")
    }
}
