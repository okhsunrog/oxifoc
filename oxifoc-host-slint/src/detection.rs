//! Motor commissioning UI.

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};

use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::ops::detect::{self, DetectionOutcome, DetectionPlan, PRESETS};
use oxifoc_host_lib::{CommandSender, HostCommand};
use slint::{ComponentHandle, SharedString};

use super::{App, SharedRuntime, parse_field};

pub fn bind(app: &App, runtime: &SharedRuntime) {
    let result = Arc::new(Mutex::new(
        None::<(CommandSender, DetectionPlan, DetectionOutcome)>,
    ));
    let cancelled = Arc::new(AtomicBool::new(false));

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
            cancelled.store(false, Ordering::Release);
            app.set_detect_resistance(SharedString::default());
            app.set_detect_running(true);
            app.set_detect_cancellable(true);
            app.set_detect_status("Checking controller".into());
            let weak = weak.clone();
            let result = result.clone();
            let cancelled = cancelled.clone();
            std::thread::spawn(move || {
                let measured = detect::run_sequence(&cmd, plan, |stage, request| {
                    if cancelled.load(Ordering::Acquire) {
                        return Err(std::io::Error::other("detection cancelled").into());
                    }
                    let _ = weak.upgrade_in_event_loop(move |app| {
                        app.set_detect_status(format!("Measuring {stage}").into());
                    });
                    detect::run_step(&cmd, request)
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
        let rt = runtime.clone();
        app.on_detect_cancel(move || {
            cancelled.store(true, Ordering::Release);
            if let Some(device) = rt.lock().unwrap().as_ref() {
                let _ = device.cmd_tx.send(HostCommand::Motor(ControlMode::Stopped));
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
