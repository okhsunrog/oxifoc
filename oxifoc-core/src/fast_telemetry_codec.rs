//! Lossless, independently decodable telemetry batches. Runs in the async
//! stream task; the ADC ISR still writes its original 34-byte Pod frames.
use super::{FAST_BATCH_BYTES, FAST_BATCH_SAMPLES, FastTelemetry, FastTelemetryBatch};
use core::mem::MaybeUninit;

const FRAME: usize = size_of::<FastTelemetry>();
const MASK_BYTES: usize = FRAME.div_ceil(8);
const _: () = assert!(FRAME < 64);

/// Compare native words, then visit only their changed bytes in wire order.
/// The final partial word is handled bytewise, without an unaligned cast.
fn encode_delta(
    previous: &[u8; FRAME],
    frame: &[u8; FRAME],
    encoded: &mut [MaybeUninit<u8>; 1 + FRAME],
) -> Option<usize> {
    let (header, replacements) = encoded.split_at_mut(1 + MASK_BYTES);
    let header = header.write_copy_of_slice(&[0; 1 + MASK_BYTES]);
    let mut count = 0;
    let (words, tail) = frame.as_chunks::<4>();
    let (previous_words, previous_tail) = previous.as_chunks::<4>();
    for (index, (&new, &old)) in words.iter().zip(previous_words).enumerate() {
        let new = u32::from_le_bytes(new);
        let mut changed = new ^ u32::from_le_bytes(old);
        let mut mask = 0u8;
        while changed != 0 {
            let byte = changed.trailing_zeros() / 8;
            let shift = byte * 8;
            replacements.get_mut(count)?.write((new >> shift) as u8);
            count += 1;
            mask |= 1 << byte;
            changed &= !(0xff << shift);
        }
        header[1 + index / 2] |= mask << ((index % 2) * 4);
    }
    for (offset, (&new, &old)) in tail.iter().zip(previous_tail).enumerate() {
        if new != old {
            replacements.get_mut(count)?.write(new);
            count += 1;
            let index = words.len() * 4 + offset;
            header[1 + index / 8] |= 1 << (index % 8);
        }
    }
    Some(1 + MASK_BYTES + count)
}

/// Write only into spare storage, then publish the initialized frame in one
/// operation. A refused reservation leaves the payload unchanged.
fn append_frame(
    data: &mut heapless::Vec<u8, FAST_BATCH_BYTES>,
    previous: Option<&[u8; FRAME]>,
    frame: &[u8; FRAME],
) -> bool {
    let start = data.len();
    let Some(encoded) = data.spare_capacity_mut().first_chunk_mut::<{ 1 + FRAME }>() else {
        return false;
    };
    let delta_len = previous
        .and_then(|previous| encode_delta(previous, frame, encoded))
        .filter(|&len| len < 1 + FRAME);
    let len = if let Some(len) = delta_len {
        encoded[0].write(1);
        len
    } else {
        encoded[0].write(0);
        encoded[1..].write_copy_of_slice(frame);
        encoded.len()
    };
    // SAFETY: the reservation above proves start + len <= capacity. For a
    // delta, encode_delta initialized its complete header and each contiguous
    // replacement byte through len. Full fallback overwrites all 1 + FRAME
    // bytes, including any partially written delta. The old prefix is intact.
    unsafe {
        data.set_len(start + len);
    }
    true
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastBatchDecodeError {
    Truncated,
    UnknownTag,
    DeltaWithoutBase,
    InvalidMask,
    TooManySamples,
}

/// Encoding state is local to the producer, never serialized in the DTO.
pub struct FastTelemetryBatchEncoder {
    batch: FastTelemetryBatch,
    previous: [u8; FRAME],
    samples: usize,
}

impl Default for FastTelemetryBatchEncoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FastTelemetryBatchEncoder {
    pub const fn new() -> Self {
        Self {
            batch: FastTelemetryBatch::new(),
            previous: [0; FRAME],
            samples: 0,
        }
    }

    pub fn batch(&self) -> &FastTelemetryBatch {
        &self.batch
    }
    pub fn len(&self) -> usize {
        self.samples
    }
    pub fn is_empty(&self) -> bool {
        self.samples == 0
    }

    /// Reserve a full-frame fallback before consuming another ISR queue grant.
    /// Compression ratio can never cause an already-consumed sample to fail.
    pub fn is_full(&self) -> bool {
        self.samples == FAST_BATCH_SAMPLES || self.batch.data.len() + 1 + FRAME > FAST_BATCH_BYTES
    }

    pub fn clear(&mut self) {
        self.batch.data.clear();
        self.samples = 0;
    }

    /// False leaves both the payload and encoder history unchanged.
    pub fn push_bytes(&mut self, frame: &[u8]) -> bool {
        let Ok(frame) = <&[u8; FRAME]>::try_from(frame) else {
            return false;
        };
        if self.samples == FAST_BATCH_SAMPLES {
            return false;
        }
        if !append_frame(
            &mut self.batch.data,
            (self.samples != 0).then_some(&self.previous),
            frame,
        ) {
            return false;
        }
        self.previous.copy_from_slice(frame);
        self.samples += 1;
        true
    }

    pub fn push(&mut self, frame: &FastTelemetry) -> bool {
        self.push_bytes(bytemuck::bytes_of(frame))
    }
}

impl FastTelemetryBatch {
    /// Validate the whole batch before exposing any samples. Malformed input
    /// cannot yield a plausible prefix followed by silently truncated data.
    pub fn samples(
        &self,
    ) -> Result<heapless::Vec<FastTelemetry, FAST_BATCH_SAMPLES>, FastBatchDecodeError> {
        use FastBatchDecodeError::*;
        let mut samples = heapless::Vec::new();
        let mut previous = [0u8; FRAME];
        let mut data = self.data.as_slice();
        while let Some((&tag, rest)) = data.split_first() {
            if samples.len() == FAST_BATCH_SAMPLES {
                return Err(TooManySamples);
            }
            data = rest;
            match tag {
                0 => {
                    let frame = data.get(..FRAME).ok_or(Truncated)?;
                    previous.copy_from_slice(frame);
                    data = &data[FRAME..];
                }
                1 => {
                    if samples.is_empty() {
                        return Err(DeltaWithoutBase);
                    }
                    let bytes = data.get(..MASK_BYTES).ok_or(Truncated)?;
                    let mut mask_bytes = [0u8; 8];
                    mask_bytes[..MASK_BYTES].copy_from_slice(bytes);
                    let mask = u64::from_le_bytes(mask_bytes);
                    if mask >> FRAME != 0 {
                        return Err(InvalidMask);
                    }
                    data = &data[MASK_BYTES..];
                    let count = mask.count_ones() as usize;
                    let mut changed = data.get(..count).ok_or(Truncated)?.iter();
                    for (i, byte) in previous.iter_mut().enumerate() {
                        if mask & (1 << i) != 0 {
                            *byte = *changed.next().ok_or(Truncated)?;
                        }
                    }
                    data = &data[count..];
                }
                _ => return Err(UnknownTag),
            }
            samples
                .push(bytemuck::pod_read_unaligned(&previous))
                .map_err(|_| TooManySamples)?;
        }
        Ok(samples)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec as StdVec;

    // Deliberately bytewise wire oracle, independent of the optimized word
    // comparisons and reserved-tail writes in the encoder.
    fn reference_frame(previous: Option<&[u8; FRAME]>, frame: &[u8; FRAME]) -> StdVec<u8> {
        let mut full = StdVec::from([0]);
        full.extend_from_slice(frame);
        let Some(previous) = previous else {
            return full;
        };
        let mut mask = [0; MASK_BYTES];
        let mut replacements = StdVec::new();
        for (index, (&old, &new)) in previous.iter().zip(frame).enumerate() {
            if old != new {
                mask[index / 8] |= 1 << (index % 8);
                replacements.push(new);
            }
        }
        if MASK_BYTES + replacements.len() >= FRAME {
            return full;
        }
        let mut delta = StdVec::from([1]);
        delta.extend_from_slice(&mask);
        delta.extend_from_slice(&replacements);
        delta
    }

    fn random_byte(seed: &mut u32) -> u8 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        *seed as u8
    }

    fn change_bytes(previous: &[u8; FRAME], count: usize, seed: &mut u32) -> [u8; FRAME] {
        let mut offsets = core::array::from_fn::<_, FRAME, _>(|index| index);
        for index in (1..FRAME).rev() {
            offsets.swap(index, usize::from(random_byte(seed)) % (index + 1));
        }
        let mut frame = *previous;
        for &offset in &offsets[..count] {
            frame[offset] ^= random_byte(seed).max(1);
        }
        frame
    }

    #[test]
    fn exact_wire_matches_bytewise_oracle_for_every_change_count_and_random_batches() {
        let mut seed = 0xd53e82c1;
        for changed in 0..=FRAME {
            for _ in 0..64 {
                let previous = core::array::from_fn(|_| random_byte(&mut seed));
                let frame = change_bytes(&previous, changed, &mut seed);
                let mut encoder = FastTelemetryBatchEncoder::new();
                assert!(encoder.push_bytes(&previous));
                assert!(encoder.push_bytes(&frame));
                let mut expected = reference_frame(None, &previous);
                expected.extend(reference_frame(Some(&previous), &frame));
                assert_eq!(
                    encoder.batch().data.as_slice(),
                    expected,
                    "changed={changed}"
                );
                assert_eq!(encoder.previous, frame);
            }
        }

        let mut encoder = FastTelemetryBatchEncoder::new();
        let mut expected = StdVec::new();
        let mut previous = None;
        let mut frame = core::array::from_fn(|_| random_byte(&mut seed));
        for index in 0..2_000 {
            if encoder.is_full() {
                encoder.clear();
                expected.clear();
                previous = None;
            }
            let changed = usize::from(random_byte(&mut seed)) % (FRAME + 1);
            frame = change_bytes(&frame, changed, &mut seed);
            assert!(encoder.push_bytes(&frame));
            expected.extend(reference_frame(previous.as_ref(), &frame));
            assert_eq!(encoder.batch().data.as_slice(), expected, "frame={index}");
            previous = Some(frame);
        }
    }

    #[test]
    fn reserved_tail_preserves_prefix_at_every_capacity_and_change_count() {
        let previous = [0x5a; FRAME];
        for used in 0..=FAST_BATCH_BYTES {
            for changed in 0..=FRAME {
                let mut data = heapless::Vec::new();
                data.resize(used, 0xa5).unwrap();
                let mut frame = previous;
                frame[..changed].fill(0x3c);
                let expected = reference_frame(Some(&previous), &frame);
                let appended = append_frame(&mut data, Some(&previous), &frame);
                let fits = used + 1 + FRAME <= FAST_BATCH_BYTES;
                assert_eq!(appended, fits, "used={used}, changed={changed}");
                assert!(data[..used].iter().all(|&byte| byte == 0xa5));
                assert_eq!(&data[used..], if fits { &expected[..] } else { &[] });
            }
        }
    }

    #[test]
    fn refused_push_preserves_payload_history_and_following_valid_frame() {
        let mut encoder = FastTelemetryBatchEncoder::new();
        let previous = [0x69; FRAME];
        assert!(encoder.push_bytes(&previous));
        for wrong_length in [0, FRAME - 1, FRAME + 1] {
            let payload = encoder.batch().data.clone();
            assert!(!encoder.push_bytes(&[0x5a; FRAME + 1][..wrong_length]));
            assert_eq!(encoder.batch().data, payload);
            assert_eq!(encoder.previous, previous);
            assert_eq!(encoder.len(), 1);
        }
        let mut next = previous;
        next[FRAME - 1] ^= 0xff;
        assert!(encoder.push_bytes(&next));
        let mut expected = reference_frame(None, &previous);
        expected.extend(reference_frame(Some(&previous), &next));
        assert_eq!(encoder.batch().data.as_slice(), expected);

        // Exercise both limits independently: full frames exhaust byte
        // capacity; identical frames reach the sample cap with room to spare.
        for dense in [false, true] {
            encoder.clear();
            let mut frame = [0; FRAME];
            while !encoder.is_full() {
                if dense {
                    for byte in &mut frame {
                        *byte ^= 0xff;
                    }
                }
                assert!(encoder.push_bytes(&frame));
            }
            let payload = encoder.batch().data.clone();
            let history = encoder.previous;
            let samples = encoder.len();
            assert_eq!(samples == FAST_BATCH_SAMPLES, !dense);
            // Even a zero-change delta must reserve a complete full frame.
            assert!(!encoder.push_bytes(&frame));
            assert!(!encoder.push_bytes(&[0; FRAME - 1]));
            assert_eq!(encoder.batch().data, payload);
            assert_eq!(encoder.previous, history);
            assert_eq!(encoder.len(), samples);
            encoder.clear();
            assert!(encoder.push_bytes(&next));
            assert_eq!(
                encoder.batch().data.as_slice(),
                reference_frame(None, &next)
            );
        }
    }

    #[test]
    fn dense_fallback_refreshes_complete_history_for_unaligned_sparse_successors() {
        let mut seed = 0x716b_92cdu32;
        for alignment in 0..4 {
            for changes in FRAME - MASK_BYTES..=FRAME {
                let base = core::array::from_fn(|_| random_byte(&mut seed));
                let dense = change_bytes(&base, changes, &mut seed);
                let mut sparse = dense;
                sparse[0] ^= 0x5a;
                sparse[FRAME - 2] ^= 0x81;
                sparse[FRAME - 1] ^= 0xff;
                let frames = [base, dense, sparse, sparse];
                let mut encoder = FastTelemetryBatchEncoder::new();
                let mut expected = StdVec::new();
                let mut previous = None;
                for frame in frames {
                    let mut storage = [0; FRAME + 3];
                    storage[alignment..alignment + FRAME].copy_from_slice(&frame);
                    assert!(encoder.push_bytes(&storage[alignment..alignment + FRAME]));
                    expected.extend(reference_frame(previous.as_ref(), &frame));
                    assert_eq!(encoder.batch().data.as_slice(), expected);
                    assert_eq!(encoder.previous, frame);
                    previous = Some(frame);
                }
                let decoded = encoder.batch().samples().unwrap();
                for (actual, expected) in decoded.iter().zip(frames) {
                    assert_eq!(bytemuck::bytes_of(actual), expected);
                }
            }
        }
    }

    #[test]
    fn sparse_mask_matches_wire_bytes_across_32_bit_boundary() {
        let mut encoder = FastTelemetryBatchEncoder::new();
        let mut frame = [0u8; FRAME];
        assert!(encoder.push_bytes(&frame));
        for (i, offset) in [0, 7, 8, 31, 32, 33].into_iter().enumerate() {
            frame[offset] = i as u8 + 1;
        }
        assert!(encoder.push_bytes(&frame));
        let mut expected = StdVec::from([0; 1 + FRAME]);
        expected.extend_from_slice(&[1, 0x81, 0x01, 0, 0x80, 0x03, 1, 2, 3, 4, 5, 6]);
        assert_eq!(encoder.batch().data.as_slice(), expected.as_slice());
    }

    #[test]
    fn delta_payload_switches_to_full_at_the_existing_size_boundary() {
        for changed in [0, 1, 8, 16, 28, 29, 30, FRAME] {
            let mut encoder = FastTelemetryBatchEncoder::new();
            let mut frame = [0; FRAME];
            assert!(encoder.push_bytes(&frame));
            frame[..changed].fill(0xa5);
            assert!(encoder.push_bytes(&frame));
            let second = &encoder.batch().data[1 + FRAME..];
            let uses_delta = 1 + MASK_BYTES + changed < 1 + FRAME;
            assert_eq!(second[0], u8::from(uses_delta));
            let expected_len = if uses_delta {
                1 + MASK_BYTES + changed
            } else {
                1 + FRAME
            };
            assert_eq!(second.len(), expected_len);
            let decoded = encoder.batch().samples().unwrap();
            assert_eq!(bytemuck::bytes_of(&decoded[1]), frame);
        }
    }

    #[test]
    fn arbitrary_bits_roundtrip_with_full_fallback_and_independent_batches() {
        let mut seed = 0x13579bdfu32;
        let mut frames = StdVec::new();
        for _ in 0..1000 {
            let bytes = core::array::from_fn::<_, FRAME, _>(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                seed as u8
            });
            frames.push(bytemuck::pod_read_unaligned::<FastTelemetry>(&bytes));
        }
        let mut encoder = FastTelemetryBatchEncoder::new();
        let mut decoded = StdVec::new();
        for frame in &frames {
            if encoder.is_full() {
                assert!(encoder.batch().data.len() <= FAST_BATCH_BYTES);
                decoded.extend(encoder.batch().samples().unwrap());
                encoder.clear();
            }
            assert!(encoder.push(frame));
        }
        decoded.extend(encoder.batch().samples().unwrap());
        assert_eq!(decoded, frames);
    }

    #[test]
    fn sparse_roundtrip_preserves_wraps_sentinels_and_pwm_edges() {
        let mut encoder = FastTelemetryBatchEncoder::new();
        let mut expected = StdVec::new();
        for i in 0..FAST_BATCH_SAMPLES {
            let frame = FastTelemetry {
                seq: 65530u16.wrapping_add(i as u16),
                ia: 2048 + i as u16,
                angle: 65530u16.wrapping_add(i as u16),
                vd: -100 + i as i16,
                vphase_adc: [65535, 0, 4095],
                pwm_ccr: [0, 4199, 2100],
                pwm_arr: 4199,
                pwm_enables: if i < 12 { 64 } else { 127 },
                ..Default::default()
            };
            assert!(encoder.push(&frame));
            expected.push(frame);
        }
        assert!(encoder.batch().data.len() < expected.len() * FRAME);
        assert_eq!(
            encoder.batch().samples().unwrap().as_slice(),
            expected.as_slice()
        );
        let payload = encoder.batch().data.clone();
        assert!(!encoder.push(&FastTelemetry::default()));
        assert!(!encoder.push_bytes(&[0; 2]));
        assert_eq!(encoder.batch().data, payload);
        encoder.clear();
        assert!(encoder.push(&expected[3]));
        assert_eq!(encoder.batch().data[0], 0);
        assert_eq!(
            encoder.batch().samples().unwrap().as_slice(),
            &expected[3..4]
        );
    }

    #[test]
    fn malformed_batches_are_rejected_without_a_partial_result() {
        let bad = |bytes: &[u8]| FastTelemetryBatch {
            data: heapless::Vec::from_slice(bytes).unwrap(),
        };
        assert_eq!(bad(&[2]).samples(), Err(FastBatchDecodeError::UnknownTag));
        assert_eq!(bad(&[0, 1]).samples(), Err(FastBatchDecodeError::Truncated));
        assert_eq!(
            bad(&[1, 0, 0, 0, 0, 0]).samples(),
            Err(FastBatchDecodeError::DeltaWithoutBase)
        );
        let mut encoder = FastTelemetryBatchEncoder::new();
        assert!(encoder.push(&FastTelemetry::default()));
        let mut p = encoder.batch().clone();
        p.data.extend_from_slice(&[1, 0, 0, 0, 0, 0x80]).unwrap();
        assert_eq!(p.samples(), Err(FastBatchDecodeError::InvalidMask));
        p.data.truncate(1 + FRAME);
        p.data.extend_from_slice(&[1, 1, 0, 0, 0, 0]).unwrap();
        assert_eq!(p.samples(), Err(FastBatchDecodeError::Truncated));
        encoder.clear();
        for _ in 0..FAST_BATCH_SAMPLES {
            assert!(encoder.push(&FastTelemetry::default()));
        }
        let mut p = encoder.batch().clone();
        p.data.extend_from_slice(&[1, 0, 0, 0, 0, 0]).unwrap();
        assert_eq!(p.samples(), Err(FastBatchDecodeError::TooManySamples));
    }
}
