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

pub(super) type StackRawMutex = CeilingRawMutex;

/// Only thread tasks and the USART1 USB executor may call NetStack. OTG_FS
/// hardware handlers just wake tasks; ADC/Hall/fault logging queues
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
