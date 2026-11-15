//! Shared bounded DWT measurements. Print only after all measured windows.

use core::hint::black_box;
use core::sync::atomic::{Ordering, compiler_fence};

pub fn enable() {
    let demcr = 0xE000_EDFC as *mut u32;
    // SAFETY: Cortex-M4 debug register, read during unique test initialization.
    let demcr_value = unsafe { core::ptr::read_volatile(demcr) };
    // SAFETY: unique test initialization, Cortex-M4 debug register DEMCR.
    unsafe { core::ptr::write_volatile(demcr, demcr_value | (1 << 24)) };
    // SAFETY: unique test initialization, Cortex-M4 DWT CYCCNT register.
    unsafe { core::ptr::write_volatile(0xE000_1004 as *mut u32, 0) };
    let ctrl = 0xE000_1000 as *mut u32;
    // SAFETY: Cortex-M4 DWT CTRL, read during unique test initialization.
    let ctrl_value = unsafe { core::ptr::read_volatile(ctrl) };
    // SAFETY: unique test initialization, Cortex-M4 DWT CTRL register.
    unsafe { core::ptr::write_volatile(ctrl, ctrl_value | 1) };
}

pub fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

#[inline(never)]
pub fn batch(function: fn(u32) -> u64, seed: u32) -> u32 {
    let function = black_box(function);
    let seed = black_box(seed);
    cortex_m::interrupt::free(|_| {
        compiler_fence(Ordering::SeqCst);
        let start = cycles();
        black_box(function(seed));
        compiler_fence(Ordering::SeqCst);
        cycles().wrapping_sub(start)
    })
}

#[derive(Clone, Copy)]
pub struct Stats {
    min: u32,
    max: u32,
    total: u64,
}

impl Stats {
    pub const fn new() -> Self {
        Self {
            min: u32::MAX,
            max: 0,
            total: 0,
        }
    }

    pub fn record(&mut self, cycles: u32) {
        defmt::assert!(cycles > 0 && cycles < 1_000_000, "unexpected batch length");
        self.min = self.min.min(cycles);
        self.max = self.max.max(cycles);
        self.total += u64::from(cycles);
    }

    pub fn report(&self, label: &str, rounds: u32, calls: u32) {
        defmt::info!(
            "BENCHV1 name={} calls={} rounds={} min={} max={} total={}",
            label,
            calls,
            rounds,
            self.min,
            self.max,
            self.total
        );
    }

    pub fn total(&self) -> u64 {
        self.total
    }
}
