//! 16/32-bit timer capture → 64-bit timestamp extension.
//!
//! Hall edges are timestamped by a timer's input capture (CCR latches CNT
//! in hardware). To turn those wrapping counter values into a monotonic
//! 64-bit tick count, software counts update events (overflows) and splices
//! the two — which has two classic races this module encodes the rules for:
//!
//! 1. **Capture vs. pending overflow.** One ISR invocation can observe both
//!    CC1IF and UIF in the same SR snapshot. Whether the capture happened
//!    before or after the wrap is decided by the captured value itself: a
//!    value in the upper half of the range means "just before the wrap"
//!    (use the old overflow count), lower half means "just after" (use the
//!    incremented one). Edges are spaced far wider than half a timer period
//!    apart in practice, so the midpoint test is unambiguous.
//!
//! 2. **Reading "now" concurrently with the overflow ISR.** A reader that
//!    samples CNT while UIF is pending but not yet serviced must add the
//!    unaccounted overflow itself — but only if CNT already wrapped (lower
//!    half). For this test to be sound, the writer must make its
//!    increment-and-clear-UIF step atomic with respect to readers:
//!    [`CaptureTimebase::overflow`] and [`CaptureTimebase::capture`] wrap it
//!    in a critical section, so no reader can ever observe "UIF cleared but
//!    counter not yet incremented" or the reverse.
//!
//! The counter width is a type parameter: G431/F405 capture on 16-bit
//! timers (TIM4/TIM3, wrap every 65.5 ms at 1 MHz), G474 captures on the
//! 32-bit TIM2 (wrap every ~71.6 min). The pure decision functions are
//! kept separate from the stateful wrapper so the wrap rules are
//! host-testable without hardware or atomics.

/// A timer counter/capture word: the wrapping low part of the timestamp.
pub trait CaptureWord: Copy {
    /// Register width in bits (shift for the overflow count).
    const BITS: u32;
    /// Whether the value lies in the upper half of the range — i.e. it
    /// precedes a concurrently-pending overflow.
    fn upper_half(self) -> bool;
    /// Zero-extend to u64.
    fn as_u64(self) -> u64;
}

impl CaptureWord for u16 {
    const BITS: u32 = 16;
    #[inline]
    fn upper_half(self) -> bool {
        self >= 0x8000
    }
    #[inline]
    fn as_u64(self) -> u64 {
        u64::from(self)
    }
}

impl CaptureWord for u32 {
    const BITS: Self = 32;
    #[inline]
    fn upper_half(self) -> bool {
        self >= 0x8000_0000
    }
    #[inline]
    fn as_u64(self) -> u64 {
        u64::from(self)
    }
}

/// Splice an overflow count and a counter value into 64-bit ticks.
#[inline]
pub fn compose<W: CaptureWord>(overflows: u32, low: W) -> u64 {
    (u64::from(overflows) << W::BITS) | low.as_u64()
}

/// Extend a captured value observed in the same SR snapshot as `uif_pending`.
///
/// Returns `(timestamp, new_overflow_count)`. When an overflow is pending,
/// the captured value decides ordering: upper half → capture preceded the
/// wrap (timestamp uses the old count, but the count still advances); lower
/// half → capture followed it. The caller must store the returned count and
/// clear UIF atomically with respect to concurrent readers.
#[inline]
pub fn extend_capture<W: CaptureWord>(
    overflows: u32,
    captured: W,
    uif_pending: bool,
) -> (u64, u32) {
    if uif_pending {
        let next = overflows.wrapping_add(1);
        if captured.upper_half() {
            (compose(overflows, captured), next)
        } else {
            (compose(next, captured), next)
        }
    } else {
        (compose(overflows, captured), overflows)
    }
}

/// Extend an instantaneous CNT read taken while `uif_pending` overflows may
/// not have been serviced yet. Lower-half CNT with a pending UIF means the
/// wrap already happened but wasn't counted — account for it locally.
#[inline]
pub fn extend_now<W: CaptureWord>(overflows: u32, cnt: W, uif_pending: bool) -> u64 {
    if uif_pending && !cnt.upper_half() {
        compose(overflows.wrapping_add(1), cnt)
    } else {
        compose(overflows, cnt)
    }
}

/// Stateful overflow accounting shared between a timer ISR (single writer)
/// and any number of readers, including ones at higher interrupt priority.
///
/// Contract: only one ISR calls [`capture`](Self::capture) /
/// [`overflow`](Self::overflow); both perform the increment-and-clear-UIF
/// step inside a critical section so readers at any priority see it as
/// atomic. [`now`](Self::now) retries on torn reads (thread-context readers
/// can be preempted by the writer between loads).
#[cfg(feature = "embassy")]
pub struct CaptureTimebase<W> {
    overflows: core::sync::atomic::AtomicU32,
    _word: core::marker::PhantomData<W>,
}

#[cfg(feature = "embassy")]
impl<W: CaptureWord> CaptureTimebase<W> {
    pub const fn new() -> Self {
        Self {
            overflows: core::sync::atomic::AtomicU32::new(0),
            _word: core::marker::PhantomData,
        }
    }

    /// Extend a captured value from the timer ISR. `uif_pending` is the UIF
    /// bit from the same SR snapshot as the capture flag; `clear_uif` runs
    /// inside the critical section together with the counter update.
    pub fn capture(&self, captured: W, uif_pending: bool, clear_uif: impl FnOnce()) -> u64 {
        use core::sync::atomic::Ordering;
        if uif_pending {
            critical_section::with(|_| {
                let (ts, next) =
                    extend_capture(self.overflows.load(Ordering::Relaxed), captured, true);
                self.overflows.store(next, Ordering::Release);
                clear_uif();
                ts
            })
        } else {
            extend_capture(self.overflows.load(Ordering::Acquire), captured, false).0
        }
    }

    /// Account for an overflow (UIF observed with no capture pending).
    pub fn overflow(&self, clear_uif: impl FnOnce()) {
        use core::sync::atomic::Ordering;
        critical_section::with(|_| {
            let next = self.overflows.load(Ordering::Relaxed).wrapping_add(1);
            self.overflows.store(next, Ordering::Release);
            clear_uif();
        });
    }

    /// Compose "now" from a coherent `(CNT, UIF)` read. `read_cnt_uif` must
    /// read CNT *before* UIF: if the wrap lands between the two reads, UIF
    /// is seen set with an upper-half CNT, which [`extend_now`] correctly
    /// leaves un-incremented.
    pub fn now(&self, read_cnt_uif: impl Fn() -> (W, bool)) -> u64 {
        use core::sync::atomic::Ordering;
        loop {
            let before = self.overflows.load(Ordering::Acquire);
            let (cnt, uif) = read_cnt_uif();
            let after = self.overflows.load(Ordering::Acquire);
            if before == after {
                return extend_now(before, cnt, uif);
            }
        }
    }
}

#[cfg(feature = "embassy")]
impl<W: CaptureWord> Default for CaptureTimebase<W> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_overflow_passthrough() {
        assert_eq!(
            extend_capture(3, 0x1234u16, false),
            (compose(3, 0x1234u16), 3)
        );
        assert_eq!(extend_now(3, 0x1234u16, false), compose(3, 0x1234u16));
    }

    #[test]
    fn capture_before_wrap_uses_old_count_but_advances_it() {
        // Edge latched at 0xFFF0, then the counter wrapped before the ISR ran:
        // the timestamp belongs to the old epoch, the count moves on.
        let (ts, next) = extend_capture(7, 0xFFF0u16, true);
        assert_eq!(ts, compose(7, 0xFFF0u16));
        assert_eq!(next, 8);
    }

    #[test]
    fn capture_after_wrap_uses_new_count() {
        let (ts, next) = extend_capture(7, 0x0010u16, true);
        assert_eq!(ts, compose(8, 0x0010u16));
        assert_eq!(next, 8);
    }

    #[test]
    fn now_accounts_pending_wrap_only_after_cnt_wrapped() {
        // CNT read just before the wrap, UIF set between CNT and UIF reads:
        // timestamp must stay in the old epoch.
        assert_eq!(extend_now(7, 0xFFFEu16, true), compose(7, 0xFFFEu16));
        // CNT already wrapped, overflow ISR not yet run: count it locally.
        assert_eq!(extend_now(7, 0x0002u16, true), compose(8, 0x0002u16));
    }

    #[test]
    fn monotonic_across_wrap_sequence() {
        // Simulated event sequence around a wrap, mixing captures and reads.
        let mut ovf = 41u32;
        let mut last = 0u64;
        let events: [(u16, bool); 5] = [
            (0x8000, false), // mid-range capture
            (0xFFF8, false), // late capture
            (0xFFFC, true),  // capture latched pre-wrap, UIF pending
            (0x0004, false), // next capture, overflow already counted
            (0x7000, false),
        ];
        for (ccr, uif) in events {
            let (ts, next) = extend_capture(ovf, ccr, uif);
            assert!(ts > last, "timestamps must be monotonic: {ts} after {last}");
            last = ts;
            ovf = next;
        }
    }

    #[test]
    fn overflow_count_wraps_without_panic() {
        let (ts, next) = extend_capture(u32::MAX, 0x0001u16, true);
        assert_eq!(next, 0);
        assert_eq!(ts, compose(0, 0x0001u16));
    }

    // ── u32 captures (32-bit timers, e.g. TIM2 on G474) ──────────────────

    #[test]
    fn u32_compose_uses_full_width() {
        assert_eq!(compose(1, 0u32), 1u64 << 32);
        assert_eq!(compose(2, 0xDEAD_BEEFu32), (2u64 << 32) | 0xDEAD_BEEFu64);
    }

    #[test]
    fn u32_capture_wrap_disambiguation() {
        // Pre-wrap capture: old epoch, count advances.
        let (ts, next) = extend_capture(5, 0xFFFF_FFF0u32, true);
        assert_eq!(ts, compose(5, 0xFFFF_FFF0u32));
        assert_eq!(next, 6);
        // Post-wrap capture: new epoch.
        let (ts, next) = extend_capture(5, 0x0000_0010u32, true);
        assert_eq!(ts, compose(6, 0x0000_0010u32));
        assert_eq!(next, 6);
    }

    #[test]
    fn u32_now_wrap_rules() {
        assert_eq!(
            extend_now(5, 0xFFFF_FFFEu32, true),
            compose(5, 0xFFFF_FFFEu32)
        );
        assert_eq!(extend_now(5, 0x2u32, true), compose(6, 0x2u32));
    }
}
