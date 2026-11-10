//! ADC-owned finite histograms. Publish once after Stop; no per-cycle I/O or CS.
//! Bins describe the timed body, excluding this recorder and the statistics tail.
use core::sync::atomic::{AtomicU32, Ordering};

pub const BINS: usize = 256;
pub const BIN_CYCLES: u32 = 64;
// Frozen header words: schema, CPU Hz, bin cycles, body count, gap count,
// body max, gap max, over-budget count, epoch changes, first/last/worst seq,
// body cycle sum low/high, initial DWT epoch, budget cycles.
pub const HEADER: usize = 16;
// 0 disabled, 1 arm request (handled while inactive), 2 armed/recording, 3 frozen.
#[unsafe(no_mangle)]
pub static FOC_DISTRIBUTION_CONTROL: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static FOC_DISTRIBUTION_DATA: [AtomicU32; HEADER + 2 * BINS] =
    [const { AtomicU32::new(0) }; HEADER + 2 * BINS];

pub struct Recorder {
    active: bool,
    header: [u32; HEADER],
    body: [u32; BINS],
    interval: [u32; BINS],
    previous: Option<(u32, u32)>,
    sum: u64,
}
impl Recorder {
    pub const fn new() -> Self {
        Self {
            active: false,
            header: [0; HEADER],
            body: [0; BINS],
            interval: [0; BINS],
            previous: None,
            sum: 0,
        }
    }

    pub fn observe(
        &mut self,
        active: bool,
        seq: u32,
        start: u32,
        generation: u32,
        elapsed: u32,
        budget: u32,
    ) {
        let control = FOC_DISTRIBUTION_CONTROL.load(Ordering::Relaxed);
        if control == 1 && !active {
            *self = Self::new();
            self.header[0] = 1; // schema
            self.header[1] = crate::config::CPU_HZ;
            self.header[2] = BIN_CYCLES;
            self.header[15] = budget;
            FOC_DISTRIBUTION_CONTROL.store(2, Ordering::Release);
            return;
        }
        if control != 2 {
            return;
        }
        if !active {
            if self.active {
                self.header[12] = self.sum as u32;
                self.header[13] = (self.sum >> 32) as u32;
                for (dst, src) in FOC_DISTRIBUTION_DATA.iter().zip(
                    self.header
                        .iter()
                        .chain(self.body.iter())
                        .chain(self.interval.iter()),
                ) {
                    dst.store(*src, Ordering::Relaxed);
                }
                FOC_DISTRIBUTION_CONTROL.store(3, Ordering::Release);
            }
            return;
        }
        if !self.active {
            self.active = true;
            self.header[9] = seq;
            self.header[14] = generation;
        }
        self.header[3] += 1;
        self.header[10] = seq;
        self.sum += u64::from(elapsed);
        self.body[(elapsed / BIN_CYCLES).min(BINS as u32 - 1) as usize] += 1;
        if elapsed > self.header[5] {
            self.header[5] = elapsed;
            self.header[11] = seq;
        }
        self.header[7] += u32::from(elapsed > budget);
        if let Some((previous, old_generation)) = self.previous {
            if old_generation == generation {
                let gap = start.wrapping_sub(previous);
                self.interval[(gap / BIN_CYCLES).min(BINS as u32 - 1) as usize] += 1;
                self.header[4] += 1;
                self.header[6] = self.header[6].max(gap);
            } else {
                self.header[8] += 1;
            }
        }
        self.previous = Some((start, generation));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_window_rearms_and_distinguishes_wrap_from_epoch_change() {
        let mut r = Recorder::new();
        FOC_DISTRIBUTION_CONTROL.store(1, Ordering::Relaxed);
        r.observe(false, 0, 0, 1, 0, 100);
        assert_eq!(FOC_DISTRIBUTION_CONTROL.load(Ordering::Relaxed), 2);
        r.observe(true, 10, u32::MAX - 9, 1, 64, 100);
        r.observe(true, 11, 10, 1, 101, 100);
        r.observe(true, 12, 30, 2, 20000, 100);
        r.observe(false, 13, 40, 2, 0, 100);
        assert_eq!(FOC_DISTRIBUTION_CONTROL.load(Ordering::Relaxed), 3);
        assert_eq!(r.header[3], 3);
        assert_eq!(r.header[4], 1);
        assert_eq!(r.header[6], 20);
        assert_eq!(r.header[7], 2);
        assert_eq!(r.header[8], 1);
        assert_eq!(r.body[1], 2);
        assert_eq!(r.body[255], 1);
        assert_eq!(r.sum, 20165);
        r.observe(true, 14, 50, 2, 10, 100);
        assert_eq!(r.header[3], 3); // frozen means frozen
        FOC_DISTRIBUTION_CONTROL.store(1, Ordering::Relaxed);
        r.observe(false, 15, 60, 2, 0, 100);
        assert_eq!(r.header[3], 0);
        assert_eq!(r.sum, 0);
    }
}
