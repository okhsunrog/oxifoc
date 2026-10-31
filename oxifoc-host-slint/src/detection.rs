//! Detection UI integration.

use super::{App, SharedRuntime, parse_field};
use oxifoc_host_lib::ops;
use slint::ComponentHandle;
use slint::SharedString;
use std::thread;

pub(super) fn bind(app: &App, runtime: &SharedRuntime) {
    // ── Detect start ─────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_detect_start(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();

            let pole_pairs = app.get_pole_pairs().max(1) as u8;
            let mut parse_err: Option<String> = None;
            let max_loss: f32 =
                parse_field("max power loss", &app.get_detect_max_loss(), &mut parse_err);
            let openloop_erpm: f32 = parse_field(
                "open-loop ERPM",
                &app.get_detect_openloop_erpm(),
                &mut parse_err,
            );
            if let Some(msg) = parse_err {
                app.set_detect_status(SharedString::from(msg));
                return;
            }

            // Full R → L → flux → hall sequence via the shared op (same code
            // path the CLI uses). Runs on a cloned command sender, NOT under
            // the runtime mutex, so Stop/Coast stay responsive during the
            // ~minute-long detection.
            let cmd_tx = runtime.cmd_tx.clone();
            app.set_detect_status(SharedString::from("Running..."));

            thread::spawn(move || {
                let result =
                    ops::detect::run_sequence(&cmd_tx, pole_pairs, max_loss, openloop_erpm);
                let _ = weak.upgrade_in_event_loop(move |app| match result {
                    Ok(out) => {
                        app.set_detect_resistance(SharedString::from(format!(
                            "{:.4}",
                            out.resistance_ohm
                        )));
                        app.set_detect_inductance_d(SharedString::from(format!(
                            "{:.6}",
                            out.inductance_d_h
                        )));
                        app.set_detect_inductance_q(SharedString::from(format!(
                            "{:.6}",
                            out.inductance_q_h
                        )));
                        app.set_detect_flux_linkage(SharedString::from(format!(
                            "{:.6}",
                            out.flux_linkage_wb
                        )));
                        app.set_detect_kv(SharedString::from(format!("{:.1}", out.kv_rpm_per_v)));
                        // The gains the device WILL compute from these params on
                        // a write (display only — apply does not write them).
                        let (kp, ki) =
                            ops::detect::suggested_pi_gains(out.resistance_ohm, out.l_avg());
                        app.set_detect_kp(SharedString::from(format!("{kp:.4}")));
                        app.set_detect_ki(SharedString::from(format!("{ki:.2}")));
                        app.set_detect_status(SharedString::from(if out.hall_ok {
                            "OK (with Hall)"
                        } else {
                            "OK (no Hall)"
                        }));
                    }
                    Err(e) => {
                        tracing::error!("Detection failed: {e:#}");
                        app.set_detect_status(SharedString::from(format!("Error: {e}")));
                    }
                });
            });
        });
    }

    // ── Detect apply to config ──────────────────────────────────────────────
    // Writes the measured motor params (+ thermal current rating) via the
    // shared op. PI gains are NOT written — the device retunes the current
    // loop from the motor params on write (single source of truth).
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_detect_apply(move || {
            let cmd_tx = {
                let guard = rt.lock().unwrap();
                let Some(ref runtime) = *guard else {
                    return;
                };
                runtime.cmd_tx.clone()
            };
            let weak = weak.clone();
            let app = weak.unwrap();

            let mut parse_err: Option<String> = None;
            let err = &mut parse_err;
            let outcome = ops::detect::DetectionOutcome {
                resistance_ohm: parse_field("resistance", &app.get_detect_resistance(), err),
                inductance_d_h: parse_field("inductance d", &app.get_detect_inductance_d(), err),
                inductance_q_h: parse_field("inductance q", &app.get_detect_inductance_q(), err),
                flux_linkage_wb: parse_field("flux linkage", &app.get_detect_flux_linkage(), err),
                kv_rpm_per_v: 0.0,
                hall_ok: false,
            };
            let pole_pairs = app.get_pole_pairs().max(1) as u8;
            let max_loss: f32 = parse_field("max power loss", &app.get_detect_max_loss(), err);
            if let Some(msg) = parse_err {
                app.set_detect_status(SharedString::from(msg));
                return;
            }

            thread::spawn(move || {
                let result =
                    ops::detect::apply_motor_params(&cmd_tx, &outcome, pole_pairs, max_loss);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    let status = match result {
                        Ok(()) => "Applied to config".to_string(),
                        Err(e) => format!("Failed to apply: {e}"),
                    };
                    app.set_detect_status(SharedString::from(status));
                });
            });
        });
    }
}
