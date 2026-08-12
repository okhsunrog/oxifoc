use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Lock-free SPSC ring buffer for real-time visualization.
///
/// Samples are stored interleaved: `[ch0_f0, ch1_f0, …, chN_f0, ch0_f1, …]`
///
/// # Concurrency contract
/// - Exactly **one** writer thread calls [`push_frame`] / [`push_batch`].
/// - Exactly **one** reader thread calls [`write_pos`] / [`copy_to`].
/// - Torn f32 reads at the ring boundary are visually imperceptible and
///   accepted as the cost of a zero-lock design.
///
/// [`push_frame`]: PlotBuffer::push_frame
/// [`push_batch`]: PlotBuffer::push_batch
/// [`write_pos`]: PlotBuffer::write_pos
/// [`copy_to`]: PlotBuffer::copy_to
pub struct PlotBuffer {
    /// Interleaved sample data stored as bit-identical u32s for atomic access.
    data: Vec<AtomicU32>,
    /// Index of the *next* frame slot to write; wraps at `capacity`.
    write_pos: AtomicU32,
    /// Monotonic content generation used by render caches. Unlike write_pos,
    /// this changes across complete ring wraps and clear operations.
    generation: AtomicU64,
    pub num_channels: usize,
    pub capacity: usize,
}

impl PlotBuffer {
    pub fn new(num_channels: usize, capacity: usize) -> Self {
        assert!(
            (1..=crate::MAX_CHANNELS).contains(&num_channels),
            "num_channels must be 1..={}",
            crate::MAX_CHANNELS
        );
        assert!(capacity >= 2, "capacity must be at least 2");

        let nan_bits = f32::NAN.to_bits();
        let data = (0..capacity * num_channels)
            .map(|_| AtomicU32::new(nan_bits))
            .collect();

        Self {
            data,
            write_pos: AtomicU32::new(0),
            generation: AtomicU64::new(0),
            num_channels,
            capacity,
        }
    }

    /// Push one frame (one `f32` value per channel).
    ///
    /// `values.len()` must equal `num_channels`.
    #[inline]
    pub fn push_frame(&self, values: &[f32]) {
        debug_assert_eq!(values.len(), self.num_channels);
        let pos = self.write_pos.load(Ordering::Relaxed) as usize;
        let base = pos * self.num_channels;
        for (ch, &v) in values.iter().enumerate() {
            self.data[base + ch].store(v.to_bits(), Ordering::Relaxed);
        }
        self.write_pos
            .store(((pos + 1) % self.capacity) as u32, Ordering::Release);
        self.generation.fetch_add(1, Ordering::Release);
    }

    /// Push a contiguous slice of frames.
    ///
    /// `frames.len()` must be a multiple of `num_channels`; values are
    /// ordered as `[ch0_f0, ch1_f0, …, ch0_f1, …]`.
    pub fn push_batch(&self, frames: &[f32]) {
        debug_assert_eq!(frames.len() % self.num_channels, 0);
        let n = frames.len() / self.num_channels;
        let mut pos = self.write_pos.load(Ordering::Relaxed) as usize;
        for i in 0..n {
            let src = i * self.num_channels;
            let dst = pos * self.num_channels;
            for ch in 0..self.num_channels {
                self.data[dst + ch].store(frames[src + ch].to_bits(), Ordering::Relaxed);
            }
            pos = (pos + 1) % self.capacity;
        }
        // Update write_pos once after all frames, with Release ordering so
        // the reader's Acquire on write_pos synchronises with all the
        // Relaxed sample stores above.
        self.write_pos.store(pos as u32, Ordering::Release);
        if n > 0 {
            self.generation.fetch_add(1, Ordering::Release);
        }
    }

    /// Reset the buffer: fill with NaN and reset write position.
    ///
    /// NaN values are rendered as transparent by the shader, avoiding
    /// visible steps when the buffer is partially filled.
    pub fn clear(&self) {
        let nan_bits = f32::NAN.to_bits();
        for atom in self.data.iter() {
            atom.store(nan_bits, Ordering::Relaxed);
        }
        self.write_pos.store(0, Ordering::Release);
        self.generation.fetch_add(1, Ordering::Release);
    }

    /// Current write position for use in the GPU shader.
    #[inline]
    pub fn write_pos(&self) -> u32 {
        self.write_pos.load(Ordering::Acquire)
    }

    /// Monotonic version of the buffer contents for renderer cache invalidation.
    #[inline]
    pub(crate) fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Copy the entire ring buffer into `dst` as `f32` values for GPU upload.
    ///
    /// `dst` is resized to `capacity * num_channels` and overwritten.
    pub fn copy_to(&self, dst: &mut Vec<f32>) {
        // Acquire on write_pos synchronises-with the Release in push_frame /
        // push_batch, ensuring all previously written data is visible here.
        let _wp = self.write_pos.load(Ordering::Acquire);
        dst.resize(self.capacity * self.num_channels, 0.0);
        for (i, atom) in self.data.iter().enumerate() {
            dst[i] = f32::from_bits(atom.load(Ordering::Relaxed));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::PlotBuffer;

    #[test]
    fn generation_changes_across_full_wrap_and_clear() {
        let buffer = PlotBuffer::new(1, 2);
        let initial = buffer.generation();

        buffer.push_batch(&[1.0, 2.0]);
        assert_eq!(buffer.write_pos(), 0, "batch completed a full ring wrap");
        let wrapped = buffer.generation();
        assert!(wrapped > initial);

        buffer.clear();
        assert_eq!(buffer.write_pos(), 0);
        assert!(buffer.generation() > wrapped);
    }

    #[test]
    fn empty_batch_does_not_invalidate_renderer_cache() {
        let buffer = PlotBuffer::new(1, 2);
        let generation = buffer.generation();
        buffer.push_batch(&[]);
        assert_eq!(buffer.generation(), generation);
    }
}
