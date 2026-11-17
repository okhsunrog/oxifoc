//! ADC-owned work classes and over-budget records, exported after Stop.
//! Updates follow the body's end timestamp and contribute to ISR entry gaps.
use core::sync::atomic::{AtomicU32, Ordering};

pub const CLASSES: usize = 256;
pub const CLASS_WORDS: usize = 5; // count, max, over, sum low/high
pub const CAPACITY: usize = 1024;
pub const RECORD_WORDS: usize = 3; // seq, body cycles, full event mask

/// Event bits forming the class index, least significant first. The decoder
/// reads this table, so class summaries always carry real event masks.
#[cfg(not(feature = "bench-work-class-wraps"))]
const CLASS_BITS: [u8; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
/// PI, command, deadman, derating, authority-ahead, authority-fallback,
/// Hall slew wrap and observer-error wrap.
#[cfg(feature = "bench-work-class-wraps")]
const CLASS_BITS: [u8; 8] = [0, 1, 2, 3, 14, 15, 8, 11];
#[used]
#[unsafe(no_mangle)]
pub static FOC_WORK_CLASS_BITS: [AtomicU32; 2] = [
    AtomicU32::new(u32::from_le_bytes([
        CLASS_BITS[0],
        CLASS_BITS[1],
        CLASS_BITS[2],
        CLASS_BITS[3],
    ])),
    AtomicU32::new(u32::from_le_bytes([
        CLASS_BITS[4],
        CLASS_BITS[5],
        CLASS_BITS[6],
        CLASS_BITS[7],
    ])),
];

#[inline(always)]
fn class_of(mask: u32) -> usize {
    let mut class = 0;
    let mut bit = 0;
    while bit < CLASS_BITS.len() {
        class |= ((mask >> CLASS_BITS[bit]) & 1) << bit;
        bit += 1;
    }
    class as usize
}

#[unsafe(no_mangle)]
pub static FOC_WORK_HEADER: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];
#[unsafe(no_mangle)]
pub static FOC_WORK_CLASSES: [AtomicU32; CLASSES * CLASS_WORDS] =
    [const { AtomicU32::new(0) }; CLASSES * CLASS_WORDS];
#[unsafe(no_mangle)]
pub static FOC_WORK_RECORDS: [AtomicU32; CAPACITY * RECORD_WORDS] =
    [const { AtomicU32::new(0) }; CAPACITY * RECORD_WORDS];

pub fn arm() {
    for word in FOC_WORK_CLASSES.iter().chain(FOC_WORK_RECORDS.iter()) {
        word.store(0, Ordering::Relaxed);
    }
    for (word, value) in FOC_WORK_HEADER.iter().zip([
        1,
        oxifoc_core::isr_work::ISR_WORK_ENABLED.load(Ordering::Relaxed),
        CLASSES as u32,
        CLASS_WORDS as u32,
        RECORD_WORDS as u32,
        CAPACITY as u32,
        0,
        0,
    ]) {
        word.store(value, Ordering::Relaxed);
    }
}

pub fn observe(seq: u32, elapsed: u32, budget: u32) {
    let mask = oxifoc_core::isr_work::events();
    let index = class_of(mask) * CLASS_WORDS;
    let words = &FOC_WORK_CLASSES[index..index + CLASS_WORDS];
    words[0].store(words[0].load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    words[1].store(
        words[1].load(Ordering::Relaxed).max(elapsed),
        Ordering::Relaxed,
    );
    let previous = words[3].load(Ordering::Relaxed);
    let (sum, carry) = previous.overflowing_add(elapsed);
    words[3].store(sum, Ordering::Relaxed);
    words[4].store(
        words[4].load(Ordering::Relaxed) + u32::from(carry),
        Ordering::Relaxed,
    );
    if elapsed > budget {
        words[2].store(words[2].load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        let n = FOC_WORK_HEADER[6].load(Ordering::Relaxed) as usize;
        if n < CAPACITY {
            for (word, value) in FOC_WORK_RECORDS[n * RECORD_WORDS..][..RECORD_WORDS]
                .iter()
                .zip([seq, elapsed, mask])
            {
                word.store(value, Ordering::Relaxed);
            }
            FOC_WORK_HEADER[6].store(n as u32 + 1, Ordering::Relaxed);
        } else {
            FOC_WORK_HEADER[7].store(
                FOC_WORK_HEADER[7].load(Ordering::Relaxed) + 1,
                Ordering::Relaxed,
            );
        }
    }
}
