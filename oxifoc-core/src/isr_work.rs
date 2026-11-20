//! Default-off ADC work tags. Change the runtime switch only while Stopped.
//! Runtime on/off comparisons retain identical instructions and addresses.
use crate::units::AngleRepr;
use core::sync::atomic::{AtomicU32, Ordering};

#[unsafe(no_mangle)]
pub static ISR_WORK_ENABLED: AtomicU32 = AtomicU32::new(1);
static EVENTS: AtomicU32 = AtomicU32::new(0);

pub const VELOCITY_PI: u32 = 1 << 0;
pub const COMMAND: u32 = 1 << 1;
pub const DEADMAN: u32 = 1 << 2;
pub const DERATING: u32 = 1 << 3;
pub const HALL_NEW_EDGE: u32 = 1 << 4;
pub const HALL_SLEW_CLIPPED: u32 = 1 << 5;
pub const TEMPERATURE: u32 = 1 << 6;
pub const STARTUP: u32 = 1 << 7;
pub const HALL_SLEW_WRAP: u32 = 1 << 8;
pub const HALL_VELOCITY_WRAP: u32 = 1 << 9;
pub const HALL_WITNESS_WRAP: u32 = 1 << 10;
pub const OBSERVER_ERROR_WRAP: u32 = 1 << 11;
pub const ANGLE_WRAP: u32 = 1 << 12;
pub const CIRCLE_SEARCH: u32 = 1 << 13;
pub const AUTHORITY_AHEAD: u32 = 1 << 14;
pub const AUTHORITY_FALLBACK: u32 = 1 << 15;

#[inline(always)]
pub fn begin() {
    EVENTS.store(0, Ordering::Relaxed);
}

#[inline(always)]
pub fn mark(event: u32) {
    if ISR_WORK_ENABLED.load(Ordering::Relaxed) != 0 {
        EVENTS.fetch_or(event, Ordering::Relaxed);
    }
}

/// The exact branch condition used by the angle-difference fast path.
#[inline(always)]
pub fn angle_difference<A: AngleRepr>(event: u32, a: A, b: A) {
    if (a.radians() - b.radians()).to_bits() & 0x7fff_ffff >= core::f32::consts::PI.to_bits() {
        mark(event);
    }
}

#[inline(always)]
pub fn events() -> u32 {
    EVENTS.load(Ordering::Relaxed)
}
