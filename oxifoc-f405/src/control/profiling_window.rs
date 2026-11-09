//! DWT enable-bit updates and validity of consecutive profiling-clock samples.

use core::num::NonZeroU32;

pub(crate) const TRACE_ENABLE: u32 = 1 << 24;
pub(crate) const COUNTER_ENABLE: u32 = 1;

pub(crate) const fn dwt_enabled(demcr: u32, ctrl: u32) -> bool {
    demcr & TRACE_ENABLE != 0 && ctrl & COUNTER_ENABLE != 0
}

pub(crate) fn enable_trace(value: u32) -> u32 {
    value | TRACE_ENABLE
}

pub(crate) fn enable_counter(value: u32) -> u32 {
    value | COUNTER_ENABLE
}

#[derive(Clone, Copy)]
pub(crate) struct DwtSample {
    pub cycles: u32,
    pub generation: u32,
}

impl DwtSample {
    pub fn elapsed_since(self, previous: Self) -> Option<NonZeroU32> {
        if self.generation != previous.generation {
            return None;
        }
        NonZeroU32::new(self.cycles.wrapping_sub(previous.cycles))
    }
}

#[derive(Default)]
pub(crate) struct DwtWindow {
    previous: Option<DwtSample>,
}

impl DwtWindow {
    /// Missing, frozen and cross-generation windows have no denominator.
    /// A healthy current sample still primes the next complete window.
    pub fn observe(&mut self, sample: Option<DwtSample>) -> Option<NonZeroU32> {
        let previous = core::mem::replace(&mut self.previous, sample);
        sample?.elapsed_since(previous?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(cycles: u32, generation: u32) -> Option<DwtSample> {
        Some(DwtSample { cycles, generation })
    }

    #[test]
    fn enabling_either_missing_bit_preserves_all_other_debug_bits() {
        for other in [0, u32::MAX, 0xa55a_5aa5, 0x5aa5_a55a] {
            for trace in [false, true] {
                for counter in [false, true] {
                    let demcr = (other & !TRACE_ENABLE) | if trace { TRACE_ENABLE } else { 0 };
                    let ctrl = (other & !COUNTER_ENABLE) | u32::from(counter);
                    assert_eq!(dwt_enabled(demcr, ctrl), trace && counter);
                    assert!(dwt_enabled(enable_trace(demcr), enable_counter(ctrl)));
                    assert_eq!(enable_trace(demcr) & !TRACE_ENABLE, demcr & !TRACE_ENABLE);
                    assert_eq!(
                        enable_counter(ctrl) & !COUNTER_ENABLE,
                        ctrl & !COUNTER_ENABLE
                    );
                    if trace && counter {
                        assert_eq!(enable_trace(demcr), demcr);
                        assert_eq!(enable_counter(ctrl), ctrl);
                    }
                }
            }
        }
    }

    #[test]
    fn startup_missing_and_frozen_windows_cannot_produce_a_denominator() {
        let mut window = DwtWindow::default();
        assert_eq!(window.observe(sample(100, 0)), None);
        assert_eq!(
            window.observe(sample(200, 0)).map(NonZeroU32::get),
            Some(100)
        );
        assert_eq!(window.observe(sample(200, 0)), None);
        assert_eq!(
            window.observe(sample(300, 0)).map(NonZeroU32::get),
            Some(100)
        );
        assert_eq!(window.observe(None), None);
        assert_eq!(window.observe(sample(500, 0)), None);
        assert_eq!(
            window.observe(sample(600, 0)).map(NonZeroU32::get),
            Some(100)
        );
    }

    #[test]
    fn rearm_discards_partial_window_then_restores_normalization_and_usb_delta() {
        let mut window = DwtWindow::default();
        assert_eq!(window.observe(sample(1_000, 7)), None);
        let repaired = DwtSample {
            cycles: 20_000,
            generation: 8,
        };
        assert_eq!(window.observe(Some(repaired)), None);
        let complete = DwtSample {
            cycles: 168_020_000,
            generation: 8,
        };
        let denominator = window.observe(Some(complete)).unwrap().get();
        assert_eq!(denominator, 168_000_000);
        assert_eq!(complete.elapsed_since(repaired).unwrap().get(), denominator);
        let usb_frames = 992_u16.wrapping_sub(2040) & 0x7ff;
        assert_eq!(usb_frames, 1_000);
        assert_eq!(denominator / u32::from(usb_frames), 168_000);
    }

    #[test]
    fn healthy_counter_wrap_keeps_the_window_valid() {
        let mut window = DwtWindow::default();
        assert_eq!(window.observe(sample(u32::MAX - 4, 2)), None);
        assert_eq!(window.observe(sample(5, 2)).map(NonZeroU32::get), Some(10));
        assert_eq!(window.observe(sample(15, 3)), None);
        assert_eq!(window.observe(sample(25, 3)).map(NonZeroU32::get), Some(10));
    }
}
