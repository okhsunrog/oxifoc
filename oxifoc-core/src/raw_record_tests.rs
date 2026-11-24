use super::*;
use crate::raw_delta::compress_envelope;
use crate::raw_telemetry::{RawBlockEncoder, RawStreamDecoder};
use crate::types::FastTelemetry;
use std::vec::Vec;

/// Block `index` of a session: `count` records whose values depend on the
/// index, rewritten as Delta28 when that is smaller.
fn block(index: u32, count: u16) -> Vec<u8> {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(7, index, 1);
    for record in 0..count {
        let frame = FastTelemetry {
            seq: (index as u16).wrapping_mul(64).wrapping_add(record),
            ia: 2000 + (index as u16 * 37 + record * 11) % 97,
            angle: (index as u16).wrapping_mul(4099).wrapping_add(record * 300),
            pwm_arr: 4200,
            ..FastTelemetry::default()
        };
        assert!(encoder.push(&frame));
    }
    let plain = encoder.finish(0).to_vec();
    let mut relayed = std::vec![0; plain.len()];
    let len = compress_envelope(&plain, &mut relayed).unwrap();
    relayed.truncate(len);
    relayed
}

fn contents(recording: &Recording<'_>, chunk: usize) -> Vec<u8> {
    let mut all = Vec::new();
    let mut part = std::vec![0; chunk];
    loop {
        let copied = recording.read(all.len(), &mut part);
        all.extend(&part[..copied]);
        if copied < chunk {
            return all;
        }
    }
}

fn indices(stream: &[u8]) -> Vec<u32> {
    let mut found = Vec::new();
    RawStreamDecoder::new()
        .push(stream, |block| found.push(block.header.block_index))
        .unwrap();
    found
}

#[test]
fn until_full_keeps_the_beginning_and_refuses_the_block_that_does_not_fit() {
    let mut memory = [0xee; 3000];
    let mut recording = Recording::new(&mut memory);
    let mut expected = Vec::new();
    let mut stored = 0;
    for index in 0..40 {
        let next = block(index, 2 + (index as u16 * 7) % 60);
        if !recording.push(&next, RecordMode::UntilFull) {
            assert!(expected.len() + next.len() > 3000);
            break;
        }
        expected.extend(next);
        stored += 1;
    }
    assert!(stored > 3 && stored < 40);
    assert_eq!(recording.len(), expected.len());
    assert_eq!(recording.blocks(), stored);
    assert_eq!(recording.overwritten(), 0);
    for chunk in [1, 7, 480, 2048, 4000] {
        assert_eq!(contents(&recording, chunk), expected);
    }
    assert_eq!(indices(&expected), (0..stored).collect::<Vec<_>>());
    assert_eq!(recording.read(expected.len(), &mut [0; 8]), 0);
    assert_eq!(recording.read(expected.len() + 99, &mut [0; 8]), 0);
}

#[test]
fn a_ring_keeps_the_newest_whole_blocks() {
    for capacity in [700, 1500, 2600, 4096, 9000] {
        let mut memory = std::vec![0xee; capacity];
        let mut recording = Recording::new(&mut memory);
        let mut pushed: Vec<Vec<u8>> = Vec::new();
        for index in 0..300 {
            let next = block(index, 2 + (index as u16 * 13) % 62);
            assert!(next.len() <= capacity);
            assert!(recording.push(&next, RecordMode::Ring));
            pushed.push(next);

            let kept = contents(&recording, 333);
            assert_eq!(kept.len(), recording.len());
            assert!(kept.len() <= capacity);
            // The recording is the last blocks pushed, whole, in order.
            let tail: Vec<u8> = {
                let mut blocks = 0;
                let mut bytes = 0;
                while blocks < pushed.len()
                    && bytes + pushed[pushed.len() - 1 - blocks].len() <= kept.len()
                {
                    bytes += pushed[pushed.len() - 1 - blocks].len();
                    blocks += 1;
                }
                pushed[pushed.len() - blocks..].concat()
            };
            assert_eq!(kept, tail, "capacity {capacity}, block {index}");
            let found = indices(&kept);
            assert_eq!(found.len(), recording.blocks() as usize);
            assert_eq!(found.last(), Some(&index));
            assert!(found.windows(2).all(|pair| pair[1] == pair[0] + 1));
            let total: usize = pushed.iter().map(Vec::len).sum();
            assert_eq!(recording.overwritten() as usize, total - kept.len());
        }
    }
}

#[test]
fn a_ring_wastes_no_more_than_the_block_that_did_not_fit_at_the_end() {
    let mut memory = [0; 6000];
    let mut recording = Recording::new(&mut memory);
    let mut least = usize::MAX;
    for index in 0..500 {
        let next = block(index, 64);
        assert!(recording.push(&next, RecordMode::Ring));
        if index > 50 {
            least = least.min(recording.len());
        }
    }
    let largest = block(0, 64).len();
    assert!(least + 2 * largest > 6000, "kept only {least} of 6000");
}

#[test]
fn an_envelope_larger_than_the_memory_is_refused_and_clear_forgets() {
    let mut memory = [0; 200];
    let mut recording = Recording::new(&mut memory);
    let small = block(0, 2);
    let large = block(1, 64);
    assert!(small.len() <= 200 && large.len() > 200);
    assert!(recording.push(&small, RecordMode::Ring));
    assert!(!recording.push(&large, RecordMode::Ring));
    assert!(!recording.push(&large, RecordMode::UntilFull));
    assert_eq!(contents(&recording, 64), small);
    recording.clear();
    assert!(recording.is_empty());
    assert_eq!(recording.read(0, &mut [0; 8]), 0);
    assert!(recording.push(&small, RecordMode::UntilFull));
    assert_eq!(contents(&recording, 64), small);
}

#[test]
fn a_reply_with_a_full_chunk_fits_one_frame() {
    let response = RecordResponse::Data(RecordChunk {
        data: heapless::Vec::from_slice(&[0xff; RECORD_CHUNK_BYTES]).unwrap(),
    });
    let mut frame = [0; 2304];
    let len = postcard::to_slice(&response, &mut frame).unwrap().len();
    // The bridge's links carry 2304-byte frames with a header of up to 24.
    assert!(len + 24 <= 2304, "{len}");
    let RecordResponse::Data(chunk) = postcard::from_bytes(&frame[..len]).unwrap() else {
        panic!("not data");
    };
    assert_eq!(chunk.data.len(), RECORD_CHUNK_BYTES);
}
