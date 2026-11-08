//! Lossless, independently decodable telemetry batches. Runs in the async
//! stream task; the ADC ISR still writes its original 34-byte Pod frames.
use super::{FAST_BATCH_BYTES, FAST_BATCH_SAMPLES, FastTelemetry, FastTelemetryBatch};

const FRAME: usize = size_of::<FastTelemetry>();
const MASK_BYTES: usize = FRAME.div_ceil(8);
const _: () = assert!(FRAME < 64);

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
        if frame.len() != FRAME || self.is_full() {
            return false;
        }
        let mut mask = 0u64;
        for (i, (&new, &old)) in frame.iter().zip(&self.previous).enumerate() {
            if new != old {
                mask |= 1 << i;
            }
        }
        let sparse = self.samples != 0 && MASK_BYTES + (mask.count_ones() as usize) < FRAME;
        // This temporary always fits the reservation above; commit in one
        // operation so a failed append cannot leave a partial encoded frame.
        let mut encoded = [0u8; 1 + FRAME];
        let len = if sparse {
            encoded[0] = 1;
            encoded[1..1 + MASK_BYTES].copy_from_slice(&mask.to_le_bytes()[..MASK_BYTES]);
            let mut pos = 1 + MASK_BYTES;
            for (i, &byte) in frame.iter().enumerate() {
                if mask & (1 << i) != 0 {
                    encoded[pos] = byte;
                    pos += 1;
                }
            }
            pos
        } else {
            encoded[1..].copy_from_slice(frame);
            encoded.len()
        };
        if self.batch.data.extend_from_slice(&encoded[..len]).is_err() {
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
