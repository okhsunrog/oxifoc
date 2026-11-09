//! App-local NetStack locking: mask its executors while keeping control IRQs live.

use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use cortex_m::interrupt::InterruptNumber;
use cortex_m::peripheral::{NVIC, SCB, scb::VectActive};
use cortex_m::register::{basepri, basepri_max};
use embassy_stm32::interrupt;
use mutex::{ConstInit, ScopedRawMutex};

use crate::config::USB_EXECUTOR_PRIORITY;

const CEILING: u8 = USB_EXECUTOR_PRIORITY as u8;
const _: () = assert!(embassy_stm32::NVIC_PRIO_BITS == 4 && CEILING == 0x60);

#[cfg(feature = "bench-netstack-primask")]
type PolicyRawMutex = mutex::raw_impls::cs::CriticalSectionRawMutex;
#[cfg(not(feature = "bench-netstack-primask"))]
type PolicyRawMutex = CeilingRawMutex;

#[cfg(feature = "bench-netstack-trace")]
pub(super) type StackRawMutex = TraceRawMutex<PolicyRawMutex>;
#[cfg(not(feature = "bench-netstack-trace"))]
pub(super) type StackRawMutex = PolicyRawMutex;

/// Only thread tasks and the USART1 USB executor may call NetStack. OTG_FS
/// and USART3 hardware handlers just wake tasks; ADC/Hall/fault logging queues
/// frames for later forwarding. Wakers/serializers must not poll tasks inline.
pub struct CeilingRawMutex {
    taken: AtomicBool,
}

impl ConstInit for CeilingRawMutex {
    const INIT: Self = Self {
        taken: AtomicBool::new(false),
    };
}

/// BASEPRI is per-core, so save and restore it for each scoped acquisition.
struct CeilingGuard(u8);

impl CeilingGuard {
    #[inline]
    fn enter() -> Self {
        let previous = basepri::read();
        // cortex-m's BASEPRI assembly uses `nomem`; these fences keep data
        // accesses inside the mask even when this guard is fully inlined.
        compiler_fence(Ordering::SeqCst);
        basepri_max::write(CEILING);
        compiler_fence(Ordering::SeqCst);
        Self(previous)
    }
}

impl Drop for CeilingGuard {
    #[inline]
    fn drop(&mut self) {
        compiler_fence(Ordering::SeqCst);
        // SAFETY: restore precisely the inherited mask after the lock flag is
        // released. BASEPRI_MAX on entry never weakens a caller's stricter mask.
        unsafe { basepri::write(self.0) };
        // Synchronize any lowering of execution priority before continuing.
        cortex_m::asm::isb();
        compiler_fence(Ordering::SeqCst);
    }
}

struct Unlock<'a>(&'a AtomicBool);

impl Drop for Unlock<'_> {
    #[inline]
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

#[inline]
fn assert_context() {
    // F405 implements four high priority bits. PRIGROUP 0..=3 leaves all four
    // as preemption bits; a subpriority split would invalidate the P6 ceiling.
    // SAFETY: AIRCR is an atomic, read-only observation of the system register.
    let grouping = (unsafe { (*SCB::PTR).aircr.read() } >> 8) & 7;
    assert!(
        grouping <= 3,
        "NetStack requires four preemption priority bits"
    );
    let allowed = match SCB::vect_active() {
        VectActive::ThreadMode => true,
        VectActive::Interrupt { irqn } => {
            u16::from(irqn) == interrupt::USART1.number()
                && NVIC::get_priority(interrupt::USART1) == CEILING
        }
        VectActive::Exception(_) => false,
    };
    assert!(
        allowed,
        "NetStack called outside its thread/P6 USB executors"
    );
}

// SAFETY: the per-instance atomic CAS admits exactly one callback. This also
// prevents aliasing if an unexpected higher-priority caller were introduced;
// the context assertion rejects that policy violation before acquiring. On the
// single F405 core, BASEPRI masks all permitted preempting callers. Reentry
// returns None immediately, and RAII releases the flag before restoring masks.
unsafe impl ScopedRawMutex for CeilingRawMutex {
    #[inline]
    fn try_with_lock<R>(&self, f: impl FnOnce() -> R) -> Option<R> {
        assert_context();
        let _ceiling = CeilingGuard::enter();
        if self
            .taken
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        // Locals drop in reverse order: release the flag, then restore BASEPRI.
        let _unlock = Unlock(&self.taken);
        Some(f())
    }

    #[inline]
    #[expect(
        clippy::panic,
        reason = "a held lock cannot be released by this executor"
    )]
    fn with_lock<R>(&self, f: impl FnOnce() -> R) -> R {
        match self.try_with_lock(f) {
            Some(result) => result,
            None => panic!("Deadlocked"),
        }
    }

    #[inline]
    fn is_locked(&self) -> bool {
        self.taken.load(Ordering::Relaxed)
    }
}

#[cfg(feature = "bench-netstack-trace")]
pub struct TraceRawMutex<M> {
    inner: M,
}

#[cfg(feature = "bench-netstack-trace")]
impl<M: ConstInit> ConstInit for TraceRawMutex<M> {
    const INIT: Self = Self { inner: M::INIT };
}

#[cfg(feature = "bench-netstack-trace")]
impl<M> TraceRawMutex<M> {
    #[inline]
    fn instrument<R>(&self, f: impl FnOnce() -> R) -> R {
        use crate::diagnostics::trace;

        // NetStack's inner mutex and WaitQueue use separate instances.
        let id = core::ptr::from_ref(self) as u32;
        // Both markers stay INSIDE the selected policy's protected callback.
        // With the ceiling policy ADC may preempt between them; elapsed spans
        // include that work. Release bookkeeping follows the end marker.
        trace::record(trace::NETSTACK_LOCK_BEGIN, id);
        let result = f();
        trace::record(trace::NETSTACK_LOCK_END, id);
        result
    }
}

#[cfg(feature = "bench-netstack-trace")]
// SAFETY: the delegate alone admits/releases callbacks. Instrumentation only
// surrounds a successful callback and cannot access protected data itself.
unsafe impl<M: ScopedRawMutex> ScopedRawMutex for TraceRawMutex<M> {
    #[inline]
    fn try_with_lock<R>(&self, f: impl FnOnce() -> R) -> Option<R> {
        self.inner.try_with_lock(|| self.instrument(f))
    }

    #[inline]
    fn with_lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.with_lock(|| self.instrument(f))
    }

    #[inline]
    fn is_locked(&self) -> bool {
        self.inner.is_locked()
    }
}

/// Boot-only exercise of the ceiling primitive's actual CPU mask instructions,
/// before USB polling or motor initialization. Runs in both traced A/B images;
/// the witness's policy field identifies selection, not the primitive tested.
#[cfg(feature = "bench-netstack-trace")]
pub(super) fn self_test() {
    self_test::run();
}

#[cfg(feature = "bench-netstack-trace")]
mod self_test {
    use core::cell::Cell;
    use core::sync::atomic::AtomicU32;

    use cortex_m::register::primask;

    use super::*;

    // SWD-readable u32 words: magic, version, status (1 running/2 pass/3 fail),
    // passed bits, expected bits, initial/final BASEPRI, initial/final PRIMASK,
    // ceiling, selected policy (0 ceiling/1 PRIMASK). Retained until reset.
    #[used]
    #[unsafe(no_mangle)]
    static NETSTACK_MUTEX_SELFTEST: [AtomicU32; 11] = [const { AtomicU32::new(0) }; 11];

    fn record(index: usize, value: u32) {
        NETSTACK_MUTEX_SELFTEST[index].store(value, Ordering::Relaxed);
    }

    fn write_basepri(value: u8) {
        compiler_fence(Ordering::SeqCst);
        // SAFETY: boot thread only, before any competing executor or motor ISR
        // starts. Each case restores the original mask before returning.
        unsafe { basepri::write(value) };
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        compiler_fence(Ordering::SeqCst);
    }

    pub(super) fn run() {
        const EXPECTED: u32 = 0x3f;
        let initial_basepri = basepri::read();
        let initial_primask = primask::read_raw() & 1;
        record(0, 0x4e53_4d58);
        record(1, 1);
        record(2, 1);
        record(4, EXPECTED);
        record(5, u32::from(initial_basepri));
        record(7, initial_primask);
        record(9, u32::from(CEILING));
        record(10, u32::from(cfg!(feature = "bench-netstack-primask")));
        // The test deliberately lowers masks between cases. Never weaken an
        // outer caller's critical section if the boot call site is moved.
        if initial_basepri != 0 || initial_primask != 0 {
            record(2, 3);
        }
        assert!(
            initial_basepri == 0 && initial_primask == 0,
            "NetStack mutex self-test requires unmasked boot thread"
        );

        let a = CeilingRawMutex::INIT;
        let b = CeilingRawMutex::INIT;
        let mut passed = 0;
        for (bit, inherited, effective) in [(0, 0, CEILING), (1, 0x30, 0x30), (2, 0xa0, CEILING)] {
            write_basepri(inherited);
            let correct_inside = a.with_lock(|| {
                a.is_locked()
                    && basepri::read() == effective
                    && primask::read_raw() & 1 == initial_primask
            });
            if correct_inside && !a.is_locked() && basepri::read() == inherited {
                passed |= 1 << bit;
            }
        }

        write_basepri(0);
        let independent = a.with_lock(|| {
            let nested =
                b.with_lock(|| a.is_locked() && b.is_locked() && basepri::read() == CEILING);
            nested && a.is_locked() && !b.is_locked() && basepri::read() == CEILING
        });
        if independent && !a.is_locked() && basepri::read() == 0 {
            passed |= 1 << 3;
        }

        let called = Cell::new(false);
        let rejected = a.with_lock(|| {
            a.try_with_lock(|| called.set(true)).is_none()
                && !called.get()
                && a.is_locked()
                && basepri::read() == CEILING
        });
        if rejected && !a.is_locked() && basepri::read() == 0 {
            passed |= 1 << 4;
        }

        let inherited_primask = critical_section::with(|_| {
            a.with_lock(|| primask::read_raw() & 1 == 1) && primask::read_raw() & 1 == 1
        });
        write_basepri(initial_basepri);
        if inherited_primask
            && basepri::read() == initial_basepri
            && primask::read_raw() & 1 == initial_primask
        {
            passed |= 1 << 5;
        }
        record(3, passed);
        record(6, u32::from(basepri::read()));
        record(8, primask::read_raw() & 1);
        record(2, if passed == EXPECTED { 2 } else { 3 });
        assert!(
            passed == EXPECTED,
            "NetStack ceiling mutex boot self-test failed"
        );
    }
}
