//! Isolated scheduling primitives, not a measurement of the linked ADC ISR.

use core::hint::black_box;
use core::sync::atomic::{AtomicU32, Ordering};
use oxifoc_core::foc::capture_timebase::{CaptureTimebase, scale_capture_ticks};

use super::dwt::{self, Stats};

const CALLS: u32 = 64;
const ROUNDS: u32 = 32;
static TIMEBASE: CaptureTimebase<u16> = CaptureTimebase::new();
static FRACTION: AtomicU32 = AtomicU32::new(0);

macro_rules! counter {
    ($name:ident, $body:expr) => {
        #[inline(never)]
        fn $name(seed: u32) -> u64 {
            let mut c = seed;
            let mut out = 0_u32;
            for _ in 0..CALLS {
                c = c.wrapping_add(1);
                out ^= black_box(($body)(&mut c));
            }
            u64::from(out)
        }
    };
}

counter!(base32, |c: &mut u32| *c);
counter!(mod15, |c: &mut u32| *c % 15);
counter!(mod20, |c: &mut u32| *c % 20);
counter!(mod16, |c: &mut u32| *c % 16);
counter!(mod32, |c: &mut u32| *c % 32);
counter!(mask16, |c: &mut u32| *c & 15);
counter!(mask32, |c: &mut u32| *c & 31);
#[inline(never)]
fn reset20(seed: u32) -> u64 {
    let mut c = seed % 20;
    let mut out = 0;
    for _ in 0..CALLS {
        c = c.wrapping_add(1);
        if c == 20 {
            c = 0;
        }
        out ^= black_box(c);
    }
    u64::from(out)
}

#[inline(never)]
fn runtime_mod20(seed: u32) -> u64 {
    let period = black_box(20);
    let mut c = seed;
    let mut out = 0;
    for _ in 0..CALLS {
        c = c.wrapping_add(1);
        out ^= black_box(c % period);
    }
    u64::from(out)
}

#[inline(never)]
fn runtime_reset20(seed: u32) -> u64 {
    let period = black_box(20);
    let mut c = seed % period;
    let mut out = 0;
    for _ in 0..CALLS {
        c = c.wrapping_add(1);
        if c == period {
            c = 0;
        }
        out ^= black_box(c);
    }
    u64::from(out)
}

#[inline(never)]
fn base64(seed: u32) -> u64 {
    let mut c = u64::from(seed);
    let mut out = 0;
    for _ in 0..CALLS {
        c = c.wrapping_add(1);
        out ^= black_box(c);
    }
    out
}

macro_rules! clock {
    ($name:ident, $body:expr) => {
        #[inline(never)]
        fn $name(_: u32) -> u64 {
            let mut out = 0;
            for _ in 0..CALLS {
                out ^= black_box($body);
            }
            out
        }
    };
}

clock!(embassy_ticks, embassy_time::Instant::now().as_ticks());
clock!(embassy_micros, embassy_time::Instant::now().as_micros());
clock!(
    tim3_raw,
    u64::from(embassy_stm32::pac::TIM3.cnt().read().cnt())
);
// Same public extension/scaling primitives and register order as Hall now_ticks.
// This is an adapter microbenchmark; the production Hall function is not linked.
#[inline(never)]
fn tim3_extended(_: u32) -> u64 {
    // Opaque once per batch: otherwise LTO knows no capture IRQ writes this
    // test-only timebase and folds its overflow count to zero.
    let timebase = black_box(&TIMEBASE);
    let mut out = 0;
    for _ in 0..CALLS {
        let raw = timebase.now(|| {
            let cnt = embassy_stm32::pac::TIM3.cnt().read().cnt();
            let uif = embassy_stm32::pac::TIM3.sr().read().uif();
            (cnt, uif)
        });
        out ^= black_box(scale_capture_ticks(raw, FRACTION.load(Ordering::Relaxed)));
    }
    out
}

pub fn run() {
    // No PWM, Hall capture ISR, or motor commands in this image.
    let timer = embassy_stm32::pac::TIM3;
    timer.cr1().modify(|w| w.set_cen(false));
    timer.cnt().write(|w| w.set_cnt(0));
    timer.sr().write_value(Default::default());
    timer.cr1().modify(|w| w.set_cen(true));
    defmt::assert!(cortex_m::peripheral::DWT::cycle_counter_enabled());

    let cases = black_box([
        ("base32", base32 as fn(u32) -> u64),
        ("base64", base64),
        ("mod15", mod15),
        ("mod20", mod20),
        ("mod16", mod16),
        ("mod32", mod32),
        ("runtime_mod20", runtime_mod20),
        ("reset20", reset20),
        ("runtime_reset20", runtime_reset20),
        ("mask16", mask16),
        ("mask32", mask32),
        ("embassy_ticks", embassy_ticks),
        ("embassy_micros", embassy_micros),
        ("tim3_raw", tim3_raw),
        ("tim3_extended", tim3_extended),
    ]);
    for (fraction, seed) in [
        (0, 0),
        (0, 0x1234_5600),
        (4_200_000, 0),
        (4_200_000, 0x1234_5600),
    ] {
        FRACTION.store(fraction, Ordering::Relaxed);
        let mut stats = [Stats::new(); 15];
        for (_, function) in cases {
            for _ in 0..4 {
                black_box(dwt::batch(function, seed));
            }
        }
        for round in 0..ROUNDS {
            // Alternate direction and rotate, so one case is not always first.
            for slot in 0..cases.len() {
                let i = if round & 1 == 0 {
                    slot
                } else {
                    cases.len() - 1 - slot
                };
                let i = (i + round as usize) % cases.len();
                stats[i].record(dwt::batch(cases[i].1, seed));
            }
        }
        // All logs are outside the measured windows.
        defmt::info!(
            "SCHEDULEV1 fraction_q32={} irq_masked=true seed={}",
            fraction,
            seed
        );
        for ((name, _), stat) in cases.iter().zip(stats) {
            stat.report(name, ROUNDS, CALLS);
        }
    }
}
