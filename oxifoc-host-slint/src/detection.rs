//! Motor commissioning UI.

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};

use oxifoc_core::types::measurement::{OperationId, Request};
use oxifoc_host_lib::CommandSender;
use oxifoc_host_lib::ops::detect::{self, DetectionOutcome, DetectionPlan, PRESETS};
use slint::{ComponentHandle, SharedString};

use super::{App, SharedRuntime, parse_field};

pub fn bind(app: &App, runtime: &SharedRuntime) {
    let result = Arc::new(Mutex::new(
        None::<(CommandSender, DetectionPlan, DetectionOutcome)>,
    ));
    let cancelled = Arc::new(AtomicBool::new(false));
    let active = Arc::new(Mutex::new(None::<(CommandSender, OperationId)>));

    {
        let weak = app.as_weak();
        let result = result.clone();
        app.on_detect_source_continue(move || {
            let app = weak.unwrap();
            match source_limits(&app) {
                Ok(_) => {
                    *result.lock().unwrap() = None;
                    app.set_detect_resistance("".into());
                    app.set_detect_status("".into());
                    app.set_detect_page(1);
                }
                Err(e) => app.set_detect_status(format!("{e:#}").into()),
            }
        });
    }
    {
        let weak = app.as_weak();
        app.on_preset_changed(move |name| {
            if let Some(app) = weak.upgrade()
                && let Some(preset) = PRESETS.iter().find(|p| p.name == name.as_str())
            {
                let plan = DetectionPlan::from(preset);
                app.set_detect_pole_pairs(i32::from(plan.pole_pairs));
                app.set_detect_flux_erpm(plan.flux_target_erpm.to_string().into());
                app.set_detect_current_limit(SharedString::default());
                app.set_detect_resistance(SharedString::default());
                app.set_detect_status(SharedString::default());
            }
        });
    }

    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        let result = result.clone();
        let cancelled = cancelled.clone();
        let active = active.clone();
        app.on_detect_start(move || {
            let app = weak.unwrap();
            if app.get_detect_running() {
                return;
            }
            let Some(cmd) = rt.lock().unwrap().as_ref().map(|r| r.cmd_tx.clone()) else {
                app.set_detect_status("Connect a controller first".into());
                return;
            };
            let Some(preset) = PRESETS.get(app.get_selected_preset() as usize) else {
                return;
            };
            let mut plan = DetectionPlan::from(preset);
            plan.source = match source_limits(&app) {
                Ok(source) => Some(source),
                Err(e) => {
                    app.set_detect_status(format!("{e:#}").into());
                    return;
                }
            };
            plan.calibrate_hall = app.get_detect_hall();
            if app.get_detect_advanced() {
                let mut error = None;
                plan.pole_pairs = app.get_detect_pole_pairs() as u8;
                plan.flux_target_erpm =
                    parse_field("flux spin ERPM", &app.get_detect_flux_erpm(), &mut error);
                let limit = app.get_detect_current_limit();
                if !limit.trim().is_empty() {
                    plan.current_limit_a = Some(parse_field("current ceiling", &limit, &mut error));
                }
                if let Some(error) = error {
                    app.set_detect_status(error.into());
                    return;
                }
            }

            *result.lock().unwrap() = None;
            {
                let mut operation = active.lock().unwrap();
                cancelled.store(false, Ordering::Release);
                *operation = None;
            }
            app.set_detect_resistance(SharedString::default());
            app.set_detect_running(true);
            app.set_detect_cancellable(true);
            app.set_detect_status("Checking controller".into());
            let weak = weak.clone();
            let result = result.clone();
            let cancelled = cancelled.clone();
            let active = active.clone();
            std::thread::spawn(move || {
                let measured = detect::run_sequence(&cmd, plan, |stage, request| {
                    {
                        let mut operation = active.lock().unwrap();
                        if cancelled.load(Ordering::Acquire) {
                            return Err(std::io::Error::other("detection cancelled").into());
                        }
                        *operation = Some((cmd.clone(), request.id));
                    }
                    let _ = weak.upgrade_in_event_loop(move |app| {
                        app.set_detect_status(format!("Measuring {stage}").into());
                    });
                    detect::run_measurement(&cmd, request)
                });
                let _ = weak.upgrade_in_event_loop(move |app| {
                    app.set_detect_running(false);
                    app.set_detect_cancellable(false);
                    if cancelled.load(Ordering::Acquire) {
                        app.set_detect_status("Cancelled".into());
                        return;
                    }
                    match measured {
                        Ok(out) => {
                            *result.lock().unwrap() = Some((cmd, plan, out));
                            show_result(&app, plan, out);
                        }
                        Err(error) => app.set_detect_status(format!("{error:#}").into()),
                    }
                });
            });
        });
    }

    {
        app.on_detect_cancel(move || {
            let operation = {
                let operation = active.lock().unwrap();
                cancelled.store(true, Ordering::Release);
                operation.clone()
            };
            if let Some((cmd, id)) = operation {
                std::thread::spawn(move || {
                    let _ = oxifoc_host_lib::measurement_request(&cmd, Request::Cancel { id });
                });
            }
        });
    }

    let rt = runtime.clone();
    let weak = app.as_weak();
    app.on_detect_apply(move || {
        let app = weak.unwrap();
        if app.get_detect_running() {
            return;
        }
        let Some((cmd, plan, out)) = result.lock().unwrap().clone() else {
            return;
        };
        if !rt
            .lock()
            .unwrap()
            .as_ref()
            .is_some_and(|r| r.cmd_tx.same_channel(&cmd))
        {
            app.set_detect_status("Controller changed. Run detection again.".into());
            return;
        }
        app.set_detect_running(true);
        app.set_detect_status("Saving configuration".into());
        let weak = weak.clone();
        std::thread::spawn(move || {
            let saved = detect::apply_sequence(&cmd, &out, plan);
            let _ = weak.upgrade_in_event_loop(move |app| {
                app.set_detect_running(false);
                app.set_detect_status(match saved {
                    Ok(()) => "Saved to controller".into(),
                    Err(error) => format!("Save failed: {error:#}").into(),
                });
            });
        });
    });
}

fn show_result(app: &App, plan: DetectionPlan, out: DetectionOutcome) {
    app.set_detect_resistance(format!("{:.2}", out.resistance_ohm * 1e3).into());
    app.set_detect_inductance_d(format!("{:.2}", out.inductance_d_h * 1e6).into());
    app.set_detect_inductance_q(format!("{:.2}", out.inductance_q_h * 1e6).into());
    app.set_detect_flux_linkage(format!("{:.3}", out.flux_linkage_wb * 1e3).into());
    app.set_detect_kv(format!("{:.1}", out.kv_rpm_per_v).into());
    let rating = detect::rating_from_loss(out.resistance_ohm, plan.max_power_loss_w);
    app.set_detect_max_current(format!("{rating:.1}").into());
    let (kp, ki) = detect::suggested_pi_gains(out.resistance_ohm, out.l_avg());
    app.set_detect_kp(format!("{kp:.4}").into());
    app.set_detect_ki(format!("{ki:.2}").into());
    app.set_detect_status(
        if out.hall_ok {
            "Ready to save · Hall calibrated"
        } else {
            "Ready to save · Sensorless"
        }
        .into(),
    );
}

fn source_limits(app: &App) -> Result<oxifoc_core::types::commissioning::SourceLimits, String> {
    let source = oxifoc_core::types::commissioning::SourceLimits {
        input_a: app
            .get_detect_source_input()
            .trim()
            .parse()
            .map_err(|_| "Enter input current limit (A)")?,
        regen_a: app
            .get_detect_source_regen()
            .trim()
            .parse()
            .map_err(|_| "Enter regenerative current limit (A), or select No regeneration")?,
    };
    if !source.is_valid() {
        return Err("Input current must be positive; regenerative current must be nonnegative. Both must be finite.".into());
    }
    Ok(source)
}
