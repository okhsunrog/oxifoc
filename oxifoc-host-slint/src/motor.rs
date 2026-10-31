//! Motor UI integration.

use super::{App, SharedRuntime, parse_id_target};
use oxifoc_core::types::{ControlMode, MotorState};
use oxifoc_host_lib::{HostCommand, HostRuntime, motor_channel, ops};
use slint::ComponentHandle;
use slint::SharedString;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;

pub(super) fn apply_motor_state(app: &App, state: MotorState) {
    app.set_motor_running(state == MotorState::Running);
    app.set_motor_state_text(SharedString::from(format!("{state:?}")));
}

pub(super) fn begin_motor_request(generation: &AtomicU64) -> u64 {
    generation.fetch_add(1, Ordering::Relaxed) + 1
}

pub(super) fn motor_request_is_current(generation: &AtomicU64, request: u64) -> bool {
    generation.load(Ordering::Relaxed) == request
}

pub(super) fn send_motor_request(
    rt: &Arc<std::sync::Mutex<Option<HostRuntime>>>,
    weak: &slint::Weak<App>,
    request_generation: &Arc<AtomicU64>,
    mode: ControlMode,
    pending: &'static str,
) {
    let cmd_tx = {
        let guard = rt.lock().unwrap();
        match guard.as_ref() {
            Some(r) => r.cmd_tx.clone(),
            None => {
                if let Some(app) = weak.upgrade() {
                    app.set_motor_status(SharedString::from("Not connected"));
                }
                return;
            }
        }
    };

    let generation = begin_motor_request(request_generation);
    if let Some(app) = weak.upgrade() {
        app.set_motor_command_pending(true);
        app.set_motor_status(SharedString::from(pending));
    }

    let weak = weak.clone();
    let request_generation = request_generation.clone();
    thread::spawn(move || {
        let (tx, rx) = motor_channel();
        let send_result = cmd_tx.send(HostCommand::MotorAck(mode, tx));
        let result = match send_result {
            Ok(()) => rx.blocking_recv(),
            Err(_) => {
                let _ = weak.upgrade_in_event_loop(move |app| {
                    if motor_request_is_current(&request_generation, generation) {
                        app.set_motor_command_pending(false);
                        app.set_motor_status(SharedString::from("Failed to queue motor command"));
                    }
                });
                return;
            }
        };

        let _ = weak.upgrade_in_event_loop(move |app| {
            if !motor_request_is_current(&request_generation, generation) {
                return;
            }
            app.set_motor_command_pending(false);
            match result {
                Ok(Ok(status)) => {
                    apply_motor_state(&app, status.state);
                    if status.fault_count > 0 {
                        app.set_motor_status(SharedString::from(format!(
                            "Device reported {} fault(s)",
                            status.fault_count
                        )));
                        app.set_fault_text(SharedString::from(format!("{}", status.fault_count)));
                    } else {
                        app.set_motor_status(SharedString::from("OK"));
                    }
                }
                Ok(Err(e)) => {
                    app.set_motor_status(SharedString::from(format!("Motor command failed: {e}")));
                }
                Err(_) => {
                    app.set_motor_status(SharedString::from("No motor response"));
                }
            }
        });
    });
}

pub(super) fn bind(
    app: &App,
    runtime: &SharedRuntime,
    motor_request_generation: &Arc<AtomicU64>,
    motor_update_pending: &Arc<AtomicBool>,
) {
    // ── Motor start ───────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        let generation = motor_request_generation.clone();
        app.on_motor_start(move || {
            let app = weak.unwrap();
            let iq_target = app.get_iq_target();
            let id_target = match parse_id_target(&app.get_id_target_text()) {
                Ok(v) => v,
                Err(msg) => {
                    app.set_motor_status(SharedString::from(msg));
                    return;
                }
            };
            tracing::info!("Motor start: iq_target={iq_target:.2}A id_target={id_target:.2}A");
            send_motor_request(
                &rt,
                &weak,
                &generation,
                ControlMode::CurrentControl {
                    iq_target,
                    id_target,
                },
                "Starting...",
            );
        });
    }

    // ── Motor stop ────────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        let generation = motor_request_generation.clone();
        app.on_motor_stop(move || {
            tracing::info!("Motor stop");
            send_motor_request(&rt, &weak, &generation, ControlMode::Stopped, "Stopping...");
        });
    }

    // ── Motor coast (gates off, free spin) ───────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        let generation = motor_request_generation.clone();
        app.on_motor_coast(move || {
            tracing::info!("Motor coast");
            send_motor_request(&rt, &weak, &generation, ControlMode::Coast, "Coasting...");
        });
    }

    // ── Motor brake (short windings, parking brake) ──────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        let generation = motor_request_generation.clone();
        app.on_motor_brake(move || {
            tracing::info!("Motor brake");
            send_motor_request(&rt, &weak, &generation, ControlMode::Brake, "Braking...");
        });
    }

    // ── Motor update (live slider changes while running) ─────────────────────
    // Just sets a flag; the actual send happens in BeforeRendering (~60Hz throttle)
    {
        let pending = motor_update_pending.clone();
        app.on_motor_update(move || {
            pending.store(true, Ordering::Relaxed);
        });
    }

    // ── Phase source switch ──────────────────────────────────────────────────
    // Fire-and-forget like the CLI `source` command: the device validates
    // (sensor present, estimators configured) and the actually-active source
    // reads back via SlowTelemetry.phase_source ("Src:" in the dashboard).
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_phase_source_changed(move || {
            let app = weak.unwrap();
            // Shared mapping + defaults (identical to the CLI `source` command).
            let Some(kind) = ops::phase::PhaseSourceKind::from_index(app.get_phase_source_index())
            else {
                return;
            };
            let ps = ops::phase::preset(
                kind,
                ops::phase::DEFAULT_SWITCH_VEL,
                ops::phase::DEFAULT_TOGGLE_V,
            );
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Phase source request: {ps:?}");
                if let Err(e) = runtime.cmd_tx.send(HostCommand::SetPhaseSource(ps)) {
                    tracing::error!("Failed to send phase source command: {e}");
                }
            } else {
                tracing::warn!("Phase source changed but no runtime");
            }
        });
    }

    // ── Stream start ─────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_stream_start(move || {
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                let app = weak.unwrap();
                let hz = ops::stream_rate_hz(app.get_stream_rate_index() as usize);
                tracing::info!("Starting fast telemetry at {} Hz", hz);
                let _ = runtime.cmd_tx.send(HostCommand::SetTelemetryConfig(
                    oxifoc_core::icd::TelemetryConfig { fast_hz: hz },
                    None,
                ));
            }
        });
    }

    // ── Stream stop ──────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        app.on_stream_stop(move || {
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Stopping fast telemetry");
                let _ = runtime.cmd_tx.send(HostCommand::SetTelemetryConfig(
                    oxifoc_core::icd::TelemetryConfig { fast_hz: 0 },
                    None,
                ));
            }
        });
    }
}
