//! Typed, fixed-record ISR queue. Only complete telemetry records are published.

use bbqueue::{nicknames::Churrasco, traits::coordination::WriteGrantError};

use crate::types::{FastTelemetry, FastTelemetryBatchEncoder};

const FRAME_BYTES: usize = size_of::<FastTelemetry>();
const _: () = assert!(FRAME_BYTES == 34);

pub(super) struct FastTelemetryQueue<const N: usize> {
    queue: Churrasco<N>,
}

impl<const N: usize> FastTelemetryQueue<N> {
    pub(super) const fn new() -> Self {
        Self {
            queue: Churrasco::new(),
        }
    }

    /// One nonblocking reservation; the producer never publishes a partial
    /// record. An unusable ring tail is handled by bbqueue's exact-grant API.
    pub(super) fn push(&self, sample: &FastTelemetry) -> Result<(), WriteGrantError> {
        let mut grant = self.queue.stream_producer().grant_exact(FRAME_BYTES)?;
        grant.copy_from_slice(bytemuck::bytes_of(sample));
        grant.commit(FRAME_BYTES);
        Ok(())
    }

    /// Encode at most one bounded batch, amortizing coordination over each
    /// contiguous chunk. No queue grant survives this synchronous call.
    pub(super) fn drain_into(&self, batch: &mut FastTelemetryBatchEncoder) -> u32 {
        let consumer = self.queue.stream_consumer();
        let mut accepted = 0;
        while !batch.is_full()
            && let Ok(grant) = consumer.read()
        {
            // The private producer commits complete, contiguous records, and
            // consumers release only whole-record prefixes, including at wrap.
            debug_assert!(grant.len().is_multiple_of(FRAME_BYTES));
            let mut records = 0;
            for frame in grant.chunks_exact(FRAME_BYTES) {
                if batch.is_full() || !batch.push_bytes(frame) {
                    break;
                }
                records += 1;
            }
            grant.release(records * FRAME_BYTES);
            accepted += records as u32;
            if records == 0 {
                // A refused encode must retain that record, not spin or drop
                // it. Normally is_full() establishes space before each push.
                break;
            }
        }
        accepted
    }

    /// Called under the stream/session gate after disabling the producer.
    pub(super) fn discard(&self) {
        let consumer = self.queue.stream_consumer();
        while let Ok(grant) = consumer.read() {
            let bytes = grant.len();
            grant.release(bytes);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{FAST_BATCH_BYTES, FAST_BATCH_SAMPLES};
    use std::{collections::VecDeque, vec::Vec};

    fn sample(seq: u16) -> FastTelemetry {
        FastTelemetry {
            seq,
            ..Default::default()
        }
    }

    #[test]
    fn partial_release_retains_suffix_and_encoder_history_until_clear() {
        let queue = FastTelemetryQueue::<{ FRAME_BYTES * 26 + 7 }>::new();
        for seq in 0..=FAST_BATCH_SAMPLES as u16 {
            queue.push(&sample(seq)).unwrap();
        }
        let mut batch = FastTelemetryBatchEncoder::new();
        assert_eq!(queue.drain_into(&mut batch), FAST_BATCH_SAMPLES as u32);
        let bytes = batch.batch().data.clone();
        assert_eq!(queue.drain_into(&mut batch), 0);
        assert_eq!(batch.batch().data, bytes);
        for (index, actual) in batch.batch().samples().unwrap().iter().enumerate() {
            assert_eq!(*actual, sample(index as u16));
        }
        batch.clear();
        assert_eq!(queue.drain_into(&mut batch), 1);
        assert_eq!(batch.batch().samples().unwrap()[0], sample(24));
        batch.clear();
        assert_eq!(queue.drain_into(&mut batch), 0);
    }

    #[test]
    fn incompressible_batch_keeps_unconsumed_records_at_the_byte_cap() {
        let queue = FastTelemetryQueue::<{ FRAME_BYTES * 20 + 11 }>::new();
        let expected: Vec<_> = (0..18)
            .map(|index| {
                bytemuck::pod_read_unaligned::<FastTelemetry>(
                    &[if index % 2 == 0 { 0x55 } else { 0xaa }; FRAME_BYTES],
                )
            })
            .collect();
        for sample in &expected {
            queue.push(sample).unwrap();
        }
        let mut batch = FastTelemetryBatchEncoder::new();
        let first = FAST_BATCH_BYTES / (1 + FRAME_BYTES);
        assert_eq!(queue.drain_into(&mut batch), first as u32);
        assert_eq!(
            batch.batch().samples().unwrap().as_slice(),
            &expected[..first]
        );
        let bytes = batch.batch().data.clone();
        assert_eq!(queue.drain_into(&mut batch), 0);
        assert_eq!(batch.batch().data, bytes);
        batch.clear();
        assert_eq!(
            queue.drain_into(&mut batch),
            (expected.len() - first) as u32
        );
        assert_eq!(
            batch.batch().samples().unwrap().as_slice(),
            &expected[first..]
        );
    }

    #[test]
    fn nonmultiple_capacity_wrap_and_backpressure_preserve_exact_records() {
        let queue = FastTelemetryQueue::<{ FRAME_BYTES * 5 + 7 }>::new();
        let mut expected = VecDeque::new();
        let mut batch = FastTelemetryBatchEncoder::new();
        let mut next = 0;
        for cycle in 0..1000 {
            // Vary burst lengths and allow reservations to fail without retry;
            // only accepted samples belong to the downstream sequence.
            for _ in 0..cycle % 7 + 1 {
                let value = sample(next);
                next += 1;
                if queue.push(&value).is_ok() {
                    expected.push_back(value);
                }
            }
            queue.drain_into(&mut batch);
            for actual in batch.batch().samples().unwrap() {
                assert_eq!(Some(actual), expected.pop_front());
            }
            assert!(expected.is_empty());
            batch.clear();
        }
    }

    #[test]
    fn held_read_grant_keeps_records_immutable_and_producer_nonblocking() {
        let queue = FastTelemetryQueue::<{ FRAME_BYTES * 4 + 7 }>::new();
        queue.push(&sample(1)).unwrap();
        let consumer = queue.queue.stream_consumer();
        let grant = consumer.read().unwrap();
        queue.push(&sample(2)).unwrap();
        queue.push(&sample(3)).unwrap();
        queue.push(&sample(4)).unwrap();
        assert!(queue.push(&sample(5)).is_err());
        assert_eq!(&*grant, bytemuck::bytes_of(&sample(1)));
        drop(grant); // Keep all data, releasing only read ownership.
        let mut batch = FastTelemetryBatchEncoder::new();
        assert_eq!(queue.drain_into(&mut batch), 4);
        assert_eq!(
            batch.batch().samples().unwrap().as_slice(),
            &[sample(1), sample(2), sample(3), sample(4)]
        );
    }

    #[test]
    fn discard_covers_both_ring_chunks_before_the_next_session() {
        let queue = FastTelemetryQueue::<{ FRAME_BYTES * 5 + 7 }>::new();
        for seq in 0..4 {
            queue.push(&sample(seq)).unwrap();
        }
        let consumer = queue.queue.stream_consumer();
        consumer.read().unwrap().release(3 * FRAME_BYTES);
        queue.push(&sample(4)).unwrap();
        queue.push(&sample(5)).unwrap(); // Wrap; both sides now hold records.
        queue.discard();
        queue.push(&sample(100)).unwrap();
        let mut batch = FastTelemetryBatchEncoder::new();
        assert_eq!(queue.drain_into(&mut batch), 1);
        assert_eq!(batch.batch().samples().unwrap().as_slice(), &[sample(100)]);
    }
}
