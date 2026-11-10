//! Minimal deadline witness: TIM5 deadline established at ADC entry; the PWM
//! hook keeps only driven-write count, minimum margin and missed deadlines.
//! One ADC writer; host reads only after Stop. Normal images omit this module.
use super::foc_distribution::FOC_DISTRIBUTION_CONTROL;
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::pac;
#[unsafe(no_mangle)]
pub static PWM_DEADLINE_DATA: [AtomicU32; 11] = [const { AtomicU32::new(0) }; 11];
// 0: outside armed ADC; 1: known entry phase; 2: ambiguous phase.
static VALID: AtomicU32 = AtomicU32::new(0);
static DEADLINE: AtomicU32 = AtomicU32::new(0);

/// Called by the ADC when it consumes the arm request, before publishing Armed.
pub fn reset() {
    VALID.store(0, Ordering::Relaxed);
    for value in &PWM_DEADLINE_DATA {
        value.store(0, Ordering::Relaxed);
    }
    PWM_DEADLINE_DATA[0].store(3, Ordering::Relaxed);
    PWM_DEADLINE_DATA[2].store(i32::MAX as u32, Ordering::Relaxed);
}

#[inline(always)]
pub fn begin() {
    VALID.store(0, Ordering::Relaxed);
    let control = FOC_DISTRIBUTION_CONTROL.load(Ordering::Relaxed);
    if !matches!(control, 2 | 4) {
        return;
    }
    let now = pac::TIM5.cnt().read();
    let tim = pac::TIM1;
    let before = tim.cr1().read().dir().to_bits();
    let count = u32::from(tim.cnt().read().cnt());
    let after = tim.cr1().read().dir().to_bits();
    let arr = u32::from(tim.arr().read().arr());
    if before != after {
        VALID.store(2, Ordering::Relaxed);
        return;
    }
    let remaining = if after != 0 { arr + count } else { arr - count };
    // TIM1 runs at twice TIM5. Floor rounding and the timestamp before phase
    // reads make this a lower bound on the next-peak deadline.
    DEADLINE.store(now.wrapping_add(remaining / 2), Ordering::Relaxed);
    VALID.store(1, Ordering::Relaxed);
}
#[inline(always)]
pub fn end() {
    VALID.store(0, Ordering::Relaxed);
}
#[inline(always)]
fn increment(index: usize) {
    PWM_DEADLINE_DATA[index].store(
        PWM_DEADLINE_DATA[index].load(Ordering::Relaxed) + 1,
        Ordering::Relaxed,
    );
}
#[inline(always)]
pub fn committed() {
    let valid = VALID.load(Ordering::Relaxed);
    if valid == 0 {
        return;
    }
    let tim = pac::TIM1;
    if tim.ccer().read().0 & 0x555 == 0 {
        return;
    }
    if valid == 2 {
        increment(7);
        return;
    }
    if tim.cr1().read().udis() {
        increment(10);
        return;
    }
    let now = pac::TIM5.cnt().read();
    let margin = (DEADLINE.load(Ordering::Relaxed).wrapping_sub(now) as i32) * 2;
    increment(1);
    if margin < PWM_DEADLINE_DATA[2].load(Ordering::Relaxed) as i32 {
        PWM_DEADLINE_DATA[2].store(margin as u32, Ordering::Relaxed);
        // Schema 3 stores the driven-write ordinal, not the ADC sequence.
        PWM_DEADLINE_DATA[8].store(
            PWM_DEADLINE_DATA[1].load(Ordering::Relaxed),
            Ordering::Relaxed,
        );
    }
    if margin < 0 {
        increment(6);
    }
}
