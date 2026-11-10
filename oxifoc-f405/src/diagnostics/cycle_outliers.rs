//! Finite diagnostic: one active velocity/current interval, then frozen RAM.
//! ADC owns Recorder. Only publication after the interval touches shared RAM;
//! there is no logging, I/O or interrupt masking here. Detailed profiling and
//! counter observations perturb execution, so compare a minimal-timing image.

use core::sync::atomic::{AtomicU32, Ordering};

pub const SECTIONS: usize = oxifoc_core::isr_prof::Section::ObserverTail as usize + 1;
pub const WORDS: usize = 10 + SECTIONS;
const KEEP: usize = 8;
pub type Record = [u32; WORDS];

// Header: active count, body sum/max/over, thermal-cycle count/sum/max/over,
// first/last sequence, DWT generation, generation changes (invalid if nonzero).
// Sums are u32: readers must require count * max < 2^32 for both populations
// before reporting means. The finite five-second experiment satisfies this.
#[unsafe(no_mangle)]
pub static BENCH_CYCLE_STATUS: AtomicU32 = AtomicU32::new(0); // 0 waiting, 1 active, 2 frozen
#[unsafe(no_mangle)]
pub static BENCH_CYCLE_SUMMARY: [AtomicU32; 12] = [const { AtomicU32::new(0) }; 12];
// First 8 over-budget cycles followed by the 8 longest, not chronological.
// Record: seq, body cycles, entry interval, flags(temp bit0, motion bit1),
// velocity f32 bits, ADC, snapshot, FOC, publication, remainder, core sections.
#[unsafe(no_mangle)]
pub static BENCH_CYCLE_RECORDS: [[AtomicU32; WORDS]; KEEP * 2] =
    [const { [const { AtomicU32::new(0) }; WORDS] }; KEEP * 2];

pub struct Recorder {
    state: u8,
    summary: [u32; 12],
    first: [Record; KEEP],
    worst: [Record; KEEP],
    first_n: usize,
    worst_n: usize,
    minimum: u32,
    last_start: Option<(u32, u32)>,
}

impl Recorder {
    pub const fn new() -> Self {
        Self {
            state: 0,
            summary: [0; 12],
            first: [[0; WORDS]; KEEP],
            worst: [[0; WORDS]; KEEP],
            first_n: 0,
            worst_n: 0,
            minimum: 0,
            last_start: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        active: bool,
        seq: u32,
        start: u32,
        generation: u32,
        elapsed: u32,
        budget: u32,
        flags: u32,
        velocity_bits: u32,
        platform: [u32; 4],
        sections: [u32; SECTIONS],
    ) {
        if self.state == 2 {
            return;
        }
        if !active {
            if self.state == 1 {
                self.publish();
                self.state = 2;
            }
            return;
        }
        if self.state == 0 {
            self.summary[8] = seq;
            self.summary[10] = generation;
            self.state = 1;
            BENCH_CYCLE_STATUS.store(1, Ordering::Release);
        }
        let interval = match self.last_start.replace((start, generation)) {
            Some((previous, previous_generation)) if previous_generation == generation => {
                start.wrapping_sub(previous)
            }
            Some(_) => {
                self.summary[11] += 1;
                0
            }
            None => 0,
        };
        self.summary[0] += 1;
        self.summary[1] = self.summary[1].wrapping_add(elapsed);
        self.summary[2] = self.summary[2].max(elapsed);
        self.summary[3] += u32::from(elapsed > budget);
        self.summary[9] = seq;
        if flags & 1 != 0 {
            self.summary[4] += 1;
            self.summary[5] = self.summary[5].wrapping_add(elapsed);
            self.summary[6] = self.summary[6].max(elapsed);
            self.summary[7] += u32::from(elapsed > budget);
        }
        if elapsed <= budget || (self.first_n == KEEP && elapsed <= self.minimum) {
            return;
        }
        let mut record = [0; WORDS];
        record[..5].copy_from_slice(&[seq, elapsed, interval, flags, velocity_bits]);
        record[5..9].copy_from_slice(&platform);
        record[9] = elapsed.saturating_sub(platform.iter().sum());
        record[10..].copy_from_slice(&sections);
        if self.first_n < KEEP {
            self.first[self.first_n] = record;
            self.first_n += 1;
        }
        if self.worst_n < KEEP {
            self.worst[self.worst_n] = record;
            self.worst_n += 1;
        } else if let Some((index, _)) = self.worst.iter().enumerate().min_by_key(|(_, r)| r[1]) {
            self.worst[index] = record;
        }
        if self.worst_n == KEEP {
            self.minimum = self.worst.iter().map(|r| r[1]).min().unwrap_or(0);
        }
    }

    fn publish(&self) {
        for (slot, value) in BENCH_CYCLE_SUMMARY.iter().zip(self.summary) {
            slot.store(value, Ordering::Relaxed);
        }
        for (slots, record) in BENCH_CYCLE_RECORDS
            .iter()
            .zip(self.first.iter().chain(&self.worst))
        {
            for (slot, &value) in slots.iter().zip(record) {
                slot.store(value, Ordering::Relaxed);
            }
        }
        BENCH_CYCLE_STATUS.store(2, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cycle(r: &mut Recorder, seq: u32, elapsed: u32, thermal: bool) {
        r.observe(
            true,
            seq,
            seq.wrapping_mul(100),
            0,
            elapsed,
            100,
            u32::from(thermal),
            0,
            [1, 2, 3, 4],
            [7; SECTIONS],
        );
    }

    #[test]
    fn first_and_longest_are_bounded_and_only_contain_overruns() {
        let mut r = Recorder::new();
        for seq in 1..=100 {
            cycle(&mut r, seq, seq * 2, seq % 2 == 0);
        }
        assert_eq!(r.summary[..4], [100, 10100, 200, 50]);
        assert_eq!(r.summary[4], 50);
        assert_eq!(r.summary[7], 25);
        assert_eq!(
            r.first.map(|record| record[0]),
            [51, 52, 53, 54, 55, 56, 57, 58]
        );
        let mut worst = r.worst.map(|record| record[1]);
        worst.sort();
        assert_eq!(worst, [186, 188, 190, 192, 194, 196, 198, 200]);
        assert!(r.worst.iter().all(|record| record[10..] == [7; SECTIONS]));
    }

    #[test]
    fn wrap_and_generation_changes_are_distinguished() {
        let mut r = Recorder::new();
        r.observe(
            true,
            1,
            u32::MAX - 9,
            0,
            150,
            100,
            0,
            0,
            [0; 4],
            [0; SECTIONS],
        );
        r.observe(true, 2, 20, 0, 160, 100, 0, 0, [0; 4], [0; SECTIONS]);
        assert_eq!(r.first[1][2], 30);
        r.observe(true, 3, 30, 1, 170, 100, 0, 0, [0; 4], [0; SECTIONS]);
        assert_eq!(r.first[2][2], 0);
        assert_eq!(r.summary[11], 1);
    }
}
