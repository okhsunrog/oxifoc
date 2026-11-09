//! Bench-only observations of the existing NetStack synchronization policy.

use mutex::{ConstInit, ScopedRawMutex, raw_impls::cs::CriticalSectionRawMutex};

use crate::diagnostics::trace;

pub struct TraceRawMutex {
    inner: CriticalSectionRawMutex,
}

impl ConstInit for TraceRawMutex {
    const INIT: Self = Self {
        inner: CriticalSectionRawMutex::INIT,
    };
}

impl TraceRawMutex {
    #[inline]
    fn instrument<R>(&self, f: impl FnOnce() -> R) -> R {
        // NetStack owns both its inner mutex and a WaitQueue mutex. Identify
        // each instance; a single "stack" ID would conflate their spans.
        let id = core::ptr::from_ref(self) as u32;
        // Both markers run INSIDE the delegate's successful locked callback.
        // The same critical section therefore orders timestamps and excludes
        // ADC preemption between the markers. Release bookkeeping follows end.
        trace::record(trace::NETSTACK_LOCK_BEGIN, id);
        let result = f();
        trace::record(trace::NETSTACK_LOCK_END, id);
        result
    }
}

// SAFETY: the delegate alone implements acquisition, rejection on reentry,
// release and interrupt masking. Instrumentation only wraps a successful
// callback and never accesses the protected data independently.
unsafe impl ScopedRawMutex for TraceRawMutex {
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
