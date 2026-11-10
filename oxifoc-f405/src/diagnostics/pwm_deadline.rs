//! Observed driven PWM commits versus the next TIM1 peak, without logging.
//! TIM5 timestamps precede the TIM1 entry-phase read: reported margin is a
//! conservative lower estimate, excluding that read's positive latency.
//! Only the ADC writes these counters; host reset/export occurs while Stopped.
use super::foc_distribution::FOC_DISTRIBUTION_CONTROL;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_stm32::pac;

// schema, writes, minimum signed margin, maximum elapsed (TIM1 ticks),
// minimum/maximum entry phase, late writes, ambiguous phase, worst sequence,
// writes outside ADC, writes whose caller still masks updates.
#[unsafe(no_mangle)]
pub static PWM_DEADLINE_DATA: [AtomicU32; 11] = [const { AtomicU32::new(0) }; 11];
static VALID: AtomicBool = AtomicBool::new(false);
static START: AtomicU32 = AtomicU32::new(0);
static REMAINING: AtomicU32 = AtomicU32::new(0);
static PHASE: AtomicU32 = AtomicU32::new(0);
static SEQ: AtomicU32 = AtomicU32::new(0);

pub fn reset() {
    for v in &PWM_DEADLINE_DATA {
        v.store(0, Ordering::Relaxed);
    }
    PWM_DEADLINE_DATA[0].store(1, Ordering::Relaxed);
    PWM_DEADLINE_DATA[2].store(i32::MAX as u32, Ordering::Relaxed);
    PWM_DEADLINE_DATA[4].store(u32::MAX, Ordering::Relaxed);
}

fn armed() -> bool {
    matches!(FOC_DISTRIBUTION_CONTROL.load(Ordering::Relaxed), 2 | 4)
}

#[inline(always)]
pub fn begin(seq: u32) {
    VALID.store(false, Ordering::Relaxed);
    if !armed() {
        return;
    }
    let start = pac::TIM5.cnt().read();
    let tim = pac::TIM1;
    let before = tim.cr1().read().dir().to_bits();
    let cnt = u32::from(tim.cnt().read().cnt());
    let after = tim.cr1().read().dir().to_bits();
    let arr = u32::from(tim.arr().read().arr());
    let phase = if after != 0 { arr - cnt } else { arr + cnt };
    START.store(start, Ordering::Relaxed);
    REMAINING.store(
        if before == after {
            2 * arr - phase
        } else {
            u32::MAX
        },
        Ordering::Relaxed,
    );
    PHASE.store(phase, Ordering::Relaxed);
    SEQ.store(seq, Ordering::Relaxed);
    VALID.store(true, Ordering::Release);
}

#[inline(always)]
pub fn end() {
    VALID.store(false, Ordering::Release);
}

fn increment(i: usize) {
    PWM_DEADLINE_DATA[i].store(
        PWM_DEADLINE_DATA[i].load(Ordering::Relaxed) + 1,
        Ordering::Relaxed,
    );
}

/// Called after restoring UDIS, not between the three CCR stores.
#[inline]
pub fn committed() {
    if !armed() {
        return;
    }
    let tim = pac::TIM1;
    if tim.ccer().read().0 & 0x555 == 0 {
        return;
    }
    if tim.cr1().read().udis() {
        increment(10);
        return;
    }
    if !VALID.load(Ordering::Acquire) {
        increment(9);
        return;
    }
    // Reading CR1 above observes UDIS cleared before this timestamp.
    let now = pac::TIM5.cnt().read();
    let remaining = REMAINING.load(Ordering::Relaxed);
    if remaining == u32::MAX {
        increment(7);
        return;
    }
    // RCC readback checks TIM1/TIM5's 2:1 timer-clock ratio before excitation.
    let elapsed = now
        .wrapping_sub(START.load(Ordering::Relaxed))
        .saturating_mul(2);
    let margin = remaining as i32 - elapsed as i32;
    increment(1);
    if margin < PWM_DEADLINE_DATA[2].load(Ordering::Relaxed) as i32 {
        PWM_DEADLINE_DATA[2].store(margin as u32, Ordering::Relaxed);
        PWM_DEADLINE_DATA[8].store(SEQ.load(Ordering::Relaxed), Ordering::Relaxed);
    }
    if elapsed > PWM_DEADLINE_DATA[3].load(Ordering::Relaxed) {
        PWM_DEADLINE_DATA[3].store(elapsed, Ordering::Relaxed);
    }
    let phase = PHASE.load(Ordering::Relaxed);
    if phase < PWM_DEADLINE_DATA[4].load(Ordering::Relaxed) {
        PWM_DEADLINE_DATA[4].store(phase, Ordering::Relaxed);
    }
    if phase > PWM_DEADLINE_DATA[5].load(Ordering::Relaxed) {
        PWM_DEADLINE_DATA[5].store(phase, Ordering::Relaxed);
    }
    if margin < 0 {
        increment(6);
    }
}
