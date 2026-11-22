//! Controller diagnostics store and its `req/diag` server.
//!
//! The platform's once-per-second statistics task calls [`publish_window`];
//! boot code calls [`set_reset_flags`] and [`set_crash`]. Readers only copy,
//! so polling from several clients cannot disturb the counts.

use core::cell::RefCell;
use core::pin::pin;

use critical_section::Mutex;
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

use crate::icd::DiagEndpoint;
use crate::types::diag::{
    CrashRecord, DiagRequest, DiagResponse, DiagSummary, IsrWindow, ResetFlags,
};

struct Store {
    summary: DiagSummary,
    last_window: Option<IsrWindow>,
    crash: Option<CrashRecord>,
    /// Erases the platform's surviving copy of the crash record.
    clear_crash: Option<fn()>,
}

static STORE: Mutex<RefCell<Store>> = Mutex::new(RefCell::new(Store {
    summary: DiagSummary {
        boot_epoch: 0,
        uptime_ms: 0,
        reset: ResetFlags {
            power_on: false,
            brown_out: false,
            pin: false,
            software: false,
            independent_watchdog: false,
            window_watchdog: false,
            low_power: false,
        },
        windows: 0,
        isr_overruns: 0,
        isr_max_cycles: 0,
        isr_long_intervals: 0,
        isr_short_intervals: 0,
        telemetry_pushed: 0,
        telemetry_dropped: 0,
        hall_edges: 0,
        hall_overcaptures: 0,
    },
    last_window: None,
    crash: None,
    clear_crash: None,
}));

/// Hand over the crash record found at boot; `clear` erases the platform's
/// copy when a client clears it.
pub fn set_crash(crash: Option<CrashRecord>, clear: fn()) {
    critical_section::with(|cs| {
        let mut store = STORE.borrow(cs).borrow_mut();
        store.crash = crash;
        store.clear_crash = Some(clear);
    });
}

/// Record the reset cause read at boot.
pub fn set_reset_flags(reset: ResetFlags) {
    critical_section::with(|cs| STORE.borrow(cs).borrow_mut().summary.reset = reset);
}

/// Publish one completed window (its `window` number is assigned here) and
/// fold it into the totals. `uptime_ms` is the platform clock at the end of
/// the window.
pub fn publish_window(mut window: IsrWindow, uptime_ms: u64) {
    critical_section::with(|cs| {
        let mut store = STORE.borrow(cs).borrow_mut();
        window.window = store.summary.windows.wrapping_add(1);
        store.summary.absorb(&window);
        store.summary.uptime_ms = uptime_ms;
        store.last_window = Some(window);
    });
}

fn respond(request: DiagRequest, boot_epoch: u64) -> DiagResponse {
    critical_section::with(|cs| {
        let store = STORE.borrow(cs).borrow();
        match request {
            DiagRequest::Summary => DiagResponse::Summary(DiagSummary {
                boot_epoch,
                ..store.summary
            }),
            DiagRequest::IsrWindow => store
                .last_window
                .map_or(DiagResponse::Unavailable, DiagResponse::IsrWindow),
            DiagRequest::Crash => DiagResponse::Crash(store.crash.clone()),
            DiagRequest::ClearCrash => {
                drop(store);
                let mut store = STORE.borrow(cs).borrow_mut();
                store.crash = None;
                if let Some(clear) = store.clear_crash {
                    clear();
                }
                DiagResponse::Crash(None)
            }
        }
    })
}

/// Serve `req/diag` from the store.
pub async fn diag_server<NS, const N: usize>(endpoints: Endpoints<NS>, boot_epoch: u64)
where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<DiagEndpoint, N>(Some("diag"));
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve(|request: &DiagRequest| {
                let response = respond(*request, boot_epoch);
                async move { response }
            })
            .await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::diag::CycleStats;

    #[test]
    fn windows_are_numbered_and_summed() {
        let before = match respond(DiagRequest::Summary, 7) {
            DiagResponse::Summary(summary) => summary,
            other => panic!("{other:?}"),
        };
        assert_eq!(before.boot_epoch, 7);
        let window = IsrWindow {
            overruns: 1,
            isr: CycleStats {
                samples: 15_000,
                sum_cycles: 90_000_000,
                max_cycles: 7_100,
            },
            ..Default::default()
        };
        publish_window(window, 1_000);
        publish_window(window, 2_000);
        let DiagResponse::IsrWindow(last) = respond(DiagRequest::IsrWindow, 7) else {
            panic!("no window");
        };
        assert_eq!(last.window, before.windows + 2);
        assert_eq!(last.isr.average(), Some(6_000));
        let DiagResponse::Summary(after) = respond(DiagRequest::Summary, 7) else {
            panic!("no summary");
        };
        assert_eq!(after.windows, before.windows + 2);
        assert_eq!(after.isr_overruns, before.isr_overruns + 2);
        assert_eq!(after.uptime_ms, 2_000);
    }

    #[test]
    fn crash_record_is_reported_until_cleared() {
        use crate::types::diag::{CrashKind, CrashRecord};
        use core::sync::atomic::{AtomicBool, Ordering};
        static CLEARED: AtomicBool = AtomicBool::new(false);
        let record = CrashRecord {
            kind: CrashKind::Panic,
            boots_since: 1,
            uptime_ms: 1_234,
            pc: 0,
            lr: 0,
            cfsr: 0,
            hfsr: 0,
            file: heapless::String::try_from("state.rs").unwrap(),
            line: 42,
            message: heapless::String::try_from("index out of bounds").unwrap(),
        };
        fn clear() {
            CLEARED.store(true, Ordering::Relaxed);
        }
        set_crash(Some(record.clone()), clear);
        assert_eq!(
            respond(DiagRequest::Crash, 0),
            DiagResponse::Crash(Some(record))
        );
        assert_eq!(
            respond(DiagRequest::ClearCrash, 0),
            DiagResponse::Crash(None)
        );
        assert!(CLEARED.load(Ordering::Relaxed));
        assert_eq!(respond(DiagRequest::Crash, 0), DiagResponse::Crash(None));
    }
}
