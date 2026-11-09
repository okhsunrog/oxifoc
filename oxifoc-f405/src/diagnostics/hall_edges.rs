//! Finite Hall capture evidence, read over SWD only after stopping the bridge.
//!
//! One TIM3 writer (or boot before it is enabled), no new critical sections.
//! Keep the first edges rather than overwriting startup evidence. The ADC
//! sequence is the latest observed loop, not a simultaneous ADC capture.

use core::sync::atomic::{AtomicU32, Ordering};

const CAPACITY: usize = 128;
static ADC_SEQUENCE: AtomicU32 = AtomicU32::new(0);

#[repr(C)]
pub struct Edge {
    ticks_lo: AtomicU32,
    ticks_hi: AtomicU32,
    adc_sequence: AtomicU32,
    /// Raw state bits 0..2, hardware overcapture bit 8, boot seed bit 9.
    flags: AtomicU32,
}

impl Edge {
    const fn new() -> Self {
        Self {
            ticks_lo: AtomicU32::new(0),
            ticks_hi: AtomicU32::new(0),
            adc_sequence: AtomicU32::new(0),
            flags: AtomicU32::new(0),
        }
    }
}

#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGES: [Edge; CAPACITY] = [const { Edge::new() }; CAPACITY];
#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGE_DROPPED: AtomicU32 = AtomicU32::new(0);

#[inline]
pub fn observe_adc(sequence: u32) {
    ADC_SEQUENCE.store(sequence, Ordering::Relaxed);
}

/// `ticks` uses the same logical 1-MHz, delay-corrected domain as the estimator.
pub fn record(ticks: u64, raw: u8, overcapture: bool, boot_seed: bool) {
    let index = BENCH_HALL_EDGE_COUNT.load(Ordering::Relaxed) as usize;
    let Some(slot) = BENCH_HALL_EDGES.get(index) else {
        BENCH_HALL_EDGE_DROPPED.fetch_add(1, Ordering::Relaxed);
        return;
    };
    slot.ticks_lo.store(ticks as u32, Ordering::Relaxed);
    slot.ticks_hi.store((ticks >> 32) as u32, Ordering::Relaxed);
    slot.adc_sequence
        .store(ADC_SEQUENCE.load(Ordering::Relaxed), Ordering::Relaxed);
    slot.flags.store(
        u32::from(raw) | (u32::from(overcapture) << 8) | (u32::from(boot_seed) << 9),
        Ordering::Relaxed,
    );
    BENCH_HALL_EDGE_COUNT.store(index as u32 + 1, Ordering::Release);
}
