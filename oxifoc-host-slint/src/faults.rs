//! Faults UI integration.

use super::{App, FaultRow, SharedRuntime};
use oxifoc_core::types::{FaultCategory, FaultClearTarget, FaultSnapshot};
use oxifoc_host_lib::{HostRuntime, ops};
use slint::ComponentHandle;
use slint::{ModelRc, SharedString, VecModel};
use std::sync::Arc;
use std::thread;

/// FaultCategory ↔ stable discriminant (postcard variant index): used to
/// round-trip a category through the Slint model's `category-id` int so a
/// per-row "Clear" can rebuild the request. Append-only, matching the wire
/// enum order in `oxifoc_core::foc::fault::FaultCategory`.
fn fault_category_id(cat: FaultCategory) -> i32 {
    match cat {
        FaultCategory::None => 0,
        FaultCategory::OverCurrent => 1,
        FaultCategory::OverVoltage => 2,
        FaultCategory::UnderVoltage => 3,
        FaultCategory::OverTemp => 4,
        FaultCategory::DriverFault => 5,
        FaultCategory::HallError => 6,
        FaultCategory::Stall => 7,
        FaultCategory::CalibrationFault => 8,
        FaultCategory::CommTimeout => 9,
        FaultCategory::Derating => 10,
    }
}

fn fault_category_from_id(id: i32) -> Option<FaultCategory> {
    Some(match id {
        1 => FaultCategory::OverCurrent,
        2 => FaultCategory::OverVoltage,
        3 => FaultCategory::UnderVoltage,
        4 => FaultCategory::OverTemp,
        5 => FaultCategory::DriverFault,
        6 => FaultCategory::HallError,
        7 => FaultCategory::Stall,
        8 => FaultCategory::CalibrationFault,
        9 => FaultCategory::CommTimeout,
        10 => FaultCategory::Derating,
        _ => return None,
    })
}

/// Render a device fault snapshot into Slint rows + the active total.
fn faults_to_rows(resp: &FaultSnapshot) -> (Vec<FaultRow>, i32) {
    let rows = resp
        .faults
        .iter()
        .map(|f| FaultRow {
            category: SharedString::from(format!("{:?}", f.category)),
            severity: SharedString::from(format!("{:?}", f.severity)),
            details: SharedString::from(f.details.as_str()),
            category_id: fault_category_id(f.category),
        })
        .collect();
    (rows, i32::from(resp.total))
}

/// Push a fault snapshot into the UI model (call from the event loop).
pub(super) fn apply_faults(app: &App, resp: &FaultSnapshot) {
    let (rows, total) = faults_to_rows(resp);
    app.set_faults(ModelRc::new(VecModel::from(rows)));
    app.set_fault_total(total);
}

/// Issue a fault query/clear off-thread (no runtime mutex held during the
/// wait) and apply the device's reply snapshot to the UI model.
fn send_fault_request(
    rt: &Arc<std::sync::Mutex<Option<HostRuntime>>>,
    weak: &slint::Weak<App>,
    clear: Option<FaultClearTarget>,
) {
    let cmd_tx = {
        let guard = rt.lock().unwrap();
        match guard.as_ref() {
            Some(r) => r.cmd_tx.clone(),
            None => return,
        }
    };
    let weak = weak.clone();
    thread::spawn(move || {
        let result = match clear {
            Some(target) => ops::fault::clear(&cmd_tx, target),
            None => ops::fault::query(&cmd_tx),
        };
        match result {
            Ok(snapshot) => {
                let _ = weak.upgrade_in_event_loop(move |app| apply_faults(&app, &snapshot));
            }
            Err(error) => {
                tracing::warn!("Fault operation failed: {error:#}");
                // A conflict means nothing was cleared; refresh so the UI
                // shows the concurrent fault that protected the operation.
                if let Ok(snapshot) = ops::fault::query(&cmd_tx) {
                    let _ = weak.upgrade_in_event_loop(move |app| apply_faults(&app, &snapshot));
                }
            }
        }
    });
}

pub(super) fn bind(app: &App, runtime: &SharedRuntime) {
    // ── Faults: refresh / clear-all / clear-one ──────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_faults_refresh(move || {
            send_fault_request(&rt, &weak, None);
        });
    }
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_clear_faults(move || {
            send_fault_request(&rt, &weak, Some(FaultClearTarget::All));
        });
    }
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_clear_fault_category(move |id| {
            if let Some(cat) = fault_category_from_id(id) {
                send_fault_request(&rt, &weak, Some(FaultClearTarget::Category(cat)));
            }
        });
    }
}
