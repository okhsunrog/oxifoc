use std::{
    ops::Range,
    sync::{
        Mutex,
        atomic::{AtomicU64, Ordering},
    },
};

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

/// Interleaved frames protected by a short lock, so snapshots include a
/// consistent write position. No GPU operations run while holding this lock.
pub struct PlotBuffer {
    id: u64,
    inner: Mutex<Inner>,
    pub num_channels: usize,
    pub capacity: usize,
}
struct Inner {
    data: Vec<f32>,
    written: u64,
    generation: u64,
    epoch: u64,
}
#[derive(Default)]
pub(crate) struct Snapshot {
    source: u64,
    pub generation: u64,
    pub write_pos: u32,
    pub available: u32,
    written: u64,
    epoch: u64,
    initialized: bool,
}
impl PlotBuffer {
    pub fn new(num_channels: usize, capacity: usize) -> Self {
        assert!((1..=crate::MAX_CHANNELS).contains(&num_channels));
        assert!((2..=u32::MAX as usize).contains(&capacity));
        Self {
            id: NEXT_ID.fetch_add(1, Ordering::Relaxed),
            inner: Mutex::new(Inner {
                data: vec![f32::NAN; capacity.checked_mul(num_channels).unwrap()],
                written: 0,
                generation: 0,
                epoch: 0,
            }),
            num_channels,
            capacity,
        }
    }
    pub fn push_frame(&self, values: &[f32]) {
        assert_eq!(values.len(), self.num_channels);
        self.push_batch(values);
    }
    pub fn push_batch(&self, frames: &[f32]) {
        assert_eq!(frames.len() % self.num_channels, 0);
        if frames.is_empty() {
            return;
        }
        let mut inner = self.inner.lock().unwrap();
        for frame in frames.chunks_exact(self.num_channels) {
            let base = (inner.written % self.capacity as u64) as usize * self.num_channels;
            inner.data[base..base + self.num_channels].copy_from_slice(frame);
            inner.written += 1;
        }
        inner.generation += 1;
    }
    pub fn clear(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.data.fill(f32::NAN);
        inner.written = 0;
        inner.epoch += 1;
        inner.generation += 1;
    }
    pub fn write_pos(&self) -> u32 {
        (self.inner.lock().unwrap().written % self.capacity as u64) as u32
    }
    pub fn available_samples(&self) -> u32 {
        self.inner.lock().unwrap().written.min(self.capacity as u64) as u32
    }
    pub fn read_back(&self, frames_back: usize, dst: &mut [f32]) {
        assert_eq!(dst.len(), self.num_channels);
        let inner = self.inner.lock().unwrap();
        if frames_back == 0 || frames_back as u64 > inner.written.min(self.capacity as u64) {
            dst.fill(f32::NAN);
            return;
        }
        let base = ((inner.written - frames_back as u64) % self.capacity as u64) as usize
            * self.num_channels;
        dst.copy_from_slice(&inner.data[base..base + self.num_channels]);
    }
    pub fn copy_to(&self, dst: &mut Vec<f32>) {
        dst.clone_from(&self.inner.lock().unwrap().data);
    }
    pub(crate) fn sync_to(&self, dst: &mut Vec<f32>, snapshot: &mut Snapshot) -> Vec<Range<usize>> {
        let inner = self.inner.lock().unwrap();
        if snapshot.source == self.id
            && snapshot.initialized
            && snapshot.generation == inner.generation
        {
            return Vec::new();
        }
        let count = inner.written.saturating_sub(snapshot.written);
        let full = snapshot.source != self.id
            || !snapshot.initialized
            || snapshot.epoch != inner.epoch
            || count >= self.capacity as u64;
        let mut ranges = Vec::with_capacity(2);
        if full {
            dst.clone_from(&inner.data);
            ranges.push(0..inner.data.len());
        } else if count > 0 {
            let start = (snapshot.written % self.capacity as u64) as usize;
            let first = (count as usize).min(self.capacity - start);
            ranges.push(start * self.num_channels..(start + first) * self.num_channels);
            if first < count as usize {
                ranges.push(0..(count as usize - first) * self.num_channels);
            }
            for range in &ranges {
                dst[range.clone()].copy_from_slice(&inner.data[range.clone()]);
            }
        }
        *snapshot = Snapshot {
            source: self.id,
            generation: inner.generation,
            write_pos: (inner.written % self.capacity as u64) as u32,
            available: inner.written.min(self.capacity as u64) as u32,
            written: inner.written,
            epoch: inner.epoch,
            initialized: true,
        };
        ranges
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn incremental_snapshot_wrap_overrun_and_clear() {
        let buffer = PlotBuffer::new(1, 4);
        let mut snapshot = Snapshot::default();
        let mut data = Vec::new();
        assert_eq!(buffer.sync_to(&mut data, &mut snapshot), vec![0..4]);
        buffer.push_batch(&[1., 2., 3.]);
        assert_eq!(buffer.sync_to(&mut data, &mut snapshot), vec![0..3]);
        buffer.push_batch(&[4., 5.]);
        assert_eq!(buffer.sync_to(&mut data, &mut snapshot), vec![3..4, 0..1]);
        assert_eq!(data, vec![5., 2., 3., 4.]);
        assert_eq!(snapshot.write_pos, 1);
        buffer.push_batch(&[6., 7., 8., 9., 10.]);
        assert_eq!(buffer.sync_to(&mut data, &mut snapshot), vec![0..4]);
        assert_eq!(data, vec![9., 10., 7., 8.]);
        assert!(buffer.sync_to(&mut data, &mut snapshot).is_empty());
        buffer.clear();
        assert_eq!(buffer.sync_to(&mut data, &mut snapshot), vec![0..4]);
        assert!(data.iter().all(|v| v.is_nan()));
        assert_eq!(snapshot.available, 0);
    }
    #[test]
    fn concurrent_snapshots_match_their_write_position() {
        let buffer = std::sync::Arc::new(PlotBuffer::new(2, 64));
        let other = buffer.clone();
        let writer = std::thread::spawn(move || {
            for i in 0..2000 {
                other.push_frame(&[i as f32, i as f32]);
            }
        });
        let mut snapshot = Snapshot::default();
        let mut data = Vec::new();
        for _ in 0..2000 {
            buffer.sync_to(&mut data, &mut snapshot);
            for i in 0..snapshot.available {
                let index = (snapshot.write_pos as usize + 64 - 1 - i as usize) % 64;
                let expected = (snapshot.written - 1 - u64::from(i)) as f32;
                assert_eq!(&data[index * 2..index * 2 + 2], &[expected, expected]);
            }
        }
        writer.join().unwrap();
    }
}
