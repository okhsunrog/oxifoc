// Slint-generated code doesn't satisfy our stricter lint set.
#[allow(
    unused_qualifications,
    clippy::use_self,
    clippy::semicolon_if_nothing_returned
)]
mod generated {
    slint::include_modules!();
}
pub use generated::*;

mod presets;

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: slint::android::AndroidApp) {
    slint::android::init(app).unwrap();
    main();
}

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use oxifoc_core::types::{ControlMode, FaultCategory, FaultRequest, FaultResponse};
use oxifoc_host_lib::{
    BleDeviceInfo, HostCommand, HostConfig, HostRuntime, TransportType, config_channel,
    fault_channel, ops, scan_ble_devices, start_host,
};
#[cfg(feature = "desktop")]
use oxifoc_host_lib::{ProbeInfo, SerialPortInfo, list_probes, list_serial_ports};
use slint::wgpu_28::WGPUConfiguration;
use slint::{
    GraphicsAPI, Image, Model, ModelRc, RenderingState, SharedString, StandardListViewItem,
    VecModel,
};
use slint_wgpu_plot::{PlotBuffer, PlotConfig, PlotRenderer, required_wgpu_settings};
use tracing_subscriber::Layer;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const CAPACITY: usize = 32768;
const MAX_LOG_LINES: usize = 2000;
const BAUD_RATES: [u32; 6] = [115200, 230400, 460800, 921600, 1_000_000, 2_000_000];

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
fn faults_to_rows(resp: &FaultResponse) -> (Vec<FaultRow>, i32) {
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
fn apply_faults(app: &App, resp: &FaultResponse) {
    let (rows, total) = faults_to_rows(resp);
    app.set_faults(ModelRc::new(VecModel::from(rows)));
    app.set_fault_total(total);
}

/// Issue a fault query/clear off-thread (no runtime mutex held during the
/// wait) and apply the device's reply snapshot to the UI model.
fn send_fault_request(
    rt: &Arc<std::sync::Mutex<Option<HostRuntime>>>,
    weak: &slint::Weak<App>,
    req: FaultRequest,
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
        let (tx, rx) = fault_channel();
        if cmd_tx.send(HostCommand::Fault(req, tx)).is_err() {
            return;
        }
        if let Ok(Ok(resp)) = rx.blocking_recv() {
            let _ = weak.upgrade_in_event_loop(move |app| apply_faults(&app, &resp));
        }
    });
}

/// Parse a numeric text field for a config/detect write. A typo must abort
/// the write with a visible error, not silently become 0 (and end up in
/// flash as e.g. 0 Ω). Records the first failing field in `err`; the
/// returned default never reaches the device because the caller bails out
/// when `err` is set.
fn parse_field<T: std::str::FromStr + Default>(
    name: &str,
    value: &SharedString,
    err: &mut Option<String>,
) -> T {
    match value.trim().parse() {
        Ok(v) => v,
        Err(_) => {
            if err.is_none() {
                *err = Some(format!("Invalid {name}: '{}'", value.as_str()));
            }
            T::default()
        }
    }
}

/// Tracing layer that sends log messages to a crossbeam channel for the UI.
struct UiLogLayer {
    tx: crossbeam_channel::Sender<(String, i32)>,
}

impl<S: tracing::Subscriber> Layer<S> for UiLogLayer {
    fn on_event(
        &self,
        event: &tracing::Event<'_>,
        _ctx: tracing_subscriber::layer::Context<'_, S>,
    ) {
        let level = match *event.metadata().level() {
            tracing::Level::TRACE => 0,
            tracing::Level::DEBUG => 1,
            tracing::Level::INFO => 2,
            tracing::Level::WARN => 3,
            tracing::Level::ERROR => 4,
        };

        // Format the message
        let mut visitor = MessageVisitor(String::new());
        event.record(&mut visitor);

        let target = event.metadata().target();
        let prefix = match level {
            3 => "WARN ",
            4 => "ERROR",
            1 => "DEBUG",
            0 => "TRACE",
            _ => "INFO ",
        };
        let line = if target == "device" {
            format!("[{prefix}] [device] {}", visitor.0)
        } else {
            format!("[{prefix}] {}", visitor.0)
        };

        let _ = self.tx.try_send((line, level));
    }
}

struct MessageVisitor(String);

impl tracing::field::Visit for MessageVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        if field.name() == "message" {
            self.0 = value.to_string();
        }
    }
}

pub fn main() {
    // Set up tracing with both stderr output and UI channel
    let (log_tx, log_rx) = crossbeam_channel::bounded::<(String, i32)>(512);

    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));

    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .with(UiLogLayer { tx: log_tx })
        .init();

    // Configure the WGPU backend (required for GPU chart rendering).
    // The largest chart has 3 channels (phase currents).
    let wgpu_settings = required_wgpu_settings(CAPACITY, 3);
    slint::BackendSelector::new()
        .require_wgpu_28(WGPUConfiguration::Automatic(wgpu_settings))
        .select()
        .expect("Failed to initialise WGPU backend");

    let app = App::new().unwrap();

    // ── Log + faults models ──────────────────────────────────────────────────
    app.set_log_messages(ModelRc::new(VecModel::<LogMessage>::default()));
    app.set_faults(ModelRc::new(VecModel::<FaultRow>::default()));

    {
        let weak = app.as_weak();
        thread::spawn(move || {
            while let Ok((text, level)) = log_rx.recv() {
                let text = SharedString::from(&text);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    let model = app.get_log_messages();
                    let vec_model = model
                        .as_any()
                        .downcast_ref::<VecModel<LogMessage>>()
                        .unwrap();
                    vec_model.push(LogMessage { text, level });
                    // Batch-trim old messages to avoid O(n) per removal
                    if vec_model.row_count() > MAX_LOG_LINES {
                        let to_remove = vec_model.row_count() - MAX_LOG_LINES + MAX_LOG_LINES / 4;
                        for _ in 0..to_remove {
                            vec_model.remove(0);
                        }
                    }
                });
            }
        });
    }

    {
        let weak = app.as_weak();
        app.on_clear_log(move || {
            if let Some(app) = weak.upgrade() {
                app.set_log_messages(ModelRc::new(VecModel::<LogMessage>::default()));
            }
        });
    }

    // ── Motor presets ─────────────────────────────────────────────────────────
    {
        let names: Vec<SharedString> = presets::preset_names()
            .into_iter()
            .map(SharedString::from)
            .collect();
        let model = ModelRc::new(VecModel::from(names));
        app.set_preset_names(model);
        // Default to first preset's pole pairs
        app.set_pole_pairs(i32::from(presets::PRESETS[0].pole_pairs));
    }
    {
        let weak = app.as_weak();
        app.on_preset_changed(move |name| {
            if let Some(app) = weak.upgrade()
                && let Some(preset) = presets::PRESETS.iter().find(|p| p.name == name.as_str())
            {
                app.set_pole_pairs(i32::from(preset.pole_pairs));
                app.set_detect_max_loss(SharedString::from(format!("{}", preset.max_power_loss_w)));
                app.set_detect_openloop_erpm(SharedString::from(format!(
                    "{}",
                    preset.openloop_erpm
                )));
                app.set_detect_sensorless_erpm(SharedString::from(format!(
                    "{}",
                    preset.sensorless_erpm
                )));
            }
        });
    }

    // ── Shared state ──────────────────────────────────────────────────────────
    #[cfg(feature = "desktop")]
    let ports_list: Arc<std::sync::Mutex<Vec<SerialPortInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    #[cfg(feature = "desktop")]
    let probes_list: Arc<std::sync::Mutex<Vec<ProbeInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let ble_devices_list: Arc<std::sync::Mutex<Vec<BleDeviceInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let runtime: Arc<std::sync::Mutex<Option<HostRuntime>>> = Arc::new(std::sync::Mutex::new(None));
    let stop_adc = Arc::new(AtomicBool::new(false));

    // ── Ring buffers shared between data thread and render notifier ───────────
    let currents_buf = Arc::new(PlotBuffer::new(3, CAPACITY)); // ia, ib, ic
    let vbus_buf = Arc::new(PlotBuffer::new(1, CAPACITY)); // V
    let temp_buf = Arc::new(PlotBuffer::new(1, CAPACITY)); // °C
    let hall_buf = Arc::new(PlotBuffer::new(2, CAPACITY)); // angle_rad, erpm/1000

    // Raw→engineering enrichment context (device BoardCalib + dc_offsets +
    // pole_pairs). Built by the device-info listener on connect; read by the
    // telemetry drain to turn raw ADC frames into amps / dq / volts.
    let enrich_slot: Arc<std::sync::Mutex<Option<oxifoc_core::foc::telemetry::EnrichCtx>>> =
        Arc::new(std::sync::Mutex::new(None));

    // Actual fast telemetry rate — set by HostRuntime after device ack
    let fast_hz: Arc<std::sync::atomic::AtomicU16> = Arc::new(std::sync::atomic::AtomicU16::new(0));

    // Pending motor update — set by slider callback, consumed in BeforeRendering (~60Hz throttle)
    let motor_update_pending = Arc::new(AtomicBool::new(false));

    // Shared telemetry receivers — set on connect, read in BeforeRendering
    let fast_rx_slot: Arc<
        std::sync::Mutex<Option<crossbeam_channel::Receiver<oxifoc_core::types::FastTelemetry>>>,
    > = Arc::new(std::sync::Mutex::new(None));
    let slow_rx_slot: Arc<
        std::sync::Mutex<Option<crossbeam_channel::Receiver<oxifoc_core::types::SlowTelemetry>>>,
    > = Arc::new(std::sync::Mutex::new(None));
    let connected_flag: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));

    #[cfg(feature = "desktop")]
    refresh_serial_ports(&app, &ports_list);
    #[cfg(feature = "desktop")]
    refresh_probes(&app, &probes_list);

    // ── Rendering notifier: creates PlotRenderers on setup, renders on every frame ──
    {
        let app_weak = app.as_weak();
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();
        let hb = hall_buf.clone();
        let esl = enrich_slot.clone();
        let fhz = fast_hz.clone();
        let frx = fast_rx_slot.clone();
        let srx = slow_rx_slot.clone();
        let conn = connected_flag.clone();
        let motor_pending = motor_update_pending.clone();
        let motor_rt = runtime.clone();

        let mut cr: Option<PlotRenderer> = None;
        let mut vr: Option<PlotRenderer> = None;
        let mut tr: Option<PlotRenderer> = None;
        let mut hr: Option<PlotRenderer> = None;

        // Measured fast-telemetry arrival rate: a configured rate above the
        // link bandwidth used to drop frames silently while the UI displayed
        // the requested Hz.
        let mut rate_count: u32 = 0;
        let mut rate_t0 = std::time::Instant::now();
        let mut measured_hz: f32 = 0.0;

        app.window()
            .set_rendering_notifier(move |state, graphics_api| match state {
                RenderingState::RenderingSetup => {
                    tracing::info!("RenderingSetup: graphics_api = {:?}", graphics_api);
                    if let GraphicsAPI::WGPU28 { device, queue, .. } = graphics_api {
                        cr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 3,
                                capacity: CAPACITY,
                                y_min: -1.0,
                                y_max: 1.0,
                                auto_range: true,
                                channel_colors: vec![
                                    [0.133, 0.827, 0.933, 1.0], // cyan  – Phase A
                                    [0.545, 0.361, 0.965, 1.0], // violet – Phase B
                                    [0.976, 0.451, 0.086, 1.0], // orange – Phase C
                                ],
                            },
                        ));
                        vr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 1,
                                capacity: CAPACITY,
                                y_min: 0.0,
                                y_max: 60.0,
                                auto_range: true,
                                channel_colors: vec![[0.918, 0.702, 0.031, 1.0]], // yellow
                            },
                        ));
                        tr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 1,
                                capacity: CAPACITY,
                                y_min: 0.0,
                                y_max: 150.0,
                                auto_range: true,
                                channel_colors: vec![[0.937, 0.267, 0.267, 1.0]], // red
                            },
                        ));
                        hr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 2,
                                capacity: CAPACITY,
                                y_min: 0.0,
                                y_max: 7.0,
                                auto_range: true,
                                channel_colors: vec![
                                    [0.290, 0.871, 0.502, 1.0], // green – Hall state
                                    [0.376, 0.647, 0.980, 1.0], // blue – eRPM/1000
                                ],
                            },
                        ));
                    }
                }
                RenderingState::BeforeRendering => {
                    // Read pause states before draining
                    let currents_paused = app_weak
                        .upgrade()
                        .map(|a| a.get_currents_paused())
                        .unwrap_or(false);
                    let vbus_paused = app_weak
                        .upgrade()
                        .map(|a| a.get_vbus_paused())
                        .unwrap_or(false);
                    let temp_paused = app_weak
                        .upgrade()
                        .map(|a| a.get_temp_paused())
                        .unwrap_or(false);
                    let hall_paused = app_weak
                        .upgrade()
                        .map(|a| a.get_hall_paused())
                        .unwrap_or(false);

                    // Drain telemetry — always consume from channel, only write to buffer if not paused
                    let mut last_fast = None;
                    let mut last_rich = None;
                    if let Ok(guard) = frx.try_lock()
                        && let Some(ref rx) = *guard
                    {
                        // Lock the enrichment ctx once per poll (it's set on
                        // connect and then stable for the session).
                        let eguard = esl.lock().ok();
                        let ectx = eguard.as_ref().and_then(|g| g.as_ref());
                        while let Ok(sample) = rx.try_recv() {
                            let rich = ectx.map(|c| sample.enrich(c));
                            if !currents_paused {
                                match &rich {
                                    // amps once calibrated, else raw ADC counts
                                    Some(r) => cb.push_frame(&[r.ia, r.ib, r.ic]),
                                    None => cb.push_frame(&[
                                        f32::from(sample.ia),
                                        f32::from(sample.ib),
                                        f32::from(sample.ic),
                                    ]),
                                }
                            }
                            if !hall_paused {
                                let (ang, erpm) =
                                    rich.map_or((0.0, 0.0), |r| (r.angle_rad, r.erpm / 1000.0));
                                hb.push_frame(&[ang, erpm]);
                            }
                            rate_count += 1;
                            last_fast = Some(sample);
                            last_rich = rich;
                        }
                    }
                    let elapsed = rate_t0.elapsed();
                    if elapsed >= Duration::from_secs(1) {
                        measured_hz = rate_count as f32 / elapsed.as_secs_f32();
                        rate_count = 0;
                        rate_t0 = std::time::Instant::now();
                    }
                    let mut last_slow = None;
                    if let Ok(guard) = srx.try_lock()
                        && let Some(ref rx) = *guard
                    {
                        while let Ok(sample) = rx.try_recv() {
                            if !vbus_paused {
                                vb.push_frame(&[sample.vbus_mv as f32 / 1000.0]);
                            }
                            if !temp_paused {
                                tb.push_frame(&[f32::from(sample.fet_temp_c_x10) / 10.0]);
                            }
                            last_slow = Some(sample);
                        }
                    }

                    if let (Some(app), Some(cr), Some(vr), Some(tr), Some(hr)) = (
                        app_weak.upgrade(),
                        cr.as_mut(),
                        vr.as_mut(),
                        tr.as_mut(),
                        hr.as_mut(),
                    ) {
                        // Throttled motor update: send at most once per frame (~60Hz)
                        if motor_pending.swap(false, Ordering::Relaxed) {
                            let iq_target = app.get_iq_target();
                            let id_target = app
                                .get_id_target_text()
                                .trim()
                                .parse::<f32>()
                                .unwrap_or(0.0);
                            if let Ok(guard) = motor_rt.try_lock()
                                && let Some(ref rt) = *guard
                            {
                                let _ = rt.cmd_tx.send(HostCommand::Motor(
                                    ControlMode::CurrentControl {
                                        iq_target,
                                        id_target,
                                    },
                                ));
                            }
                        }

                        // Update connection status + text from latest samples
                        app.set_is_connected(conn.load(Ordering::Relaxed));
                        if let Some(s) = last_fast {
                            if let Some(r) = last_rich {
                                // Engineering units (device-calibrated).
                                app.set_ia_text(SharedString::from(format!("{:.2} A", r.ia)));
                                app.set_ib_text(SharedString::from(format!("{:.2} A", r.ib)));
                                app.set_ic_text(SharedString::from(format!("{:.2} A", r.ic)));
                                app.set_id_text(SharedString::from(format!("{:.2} A", r.id)));
                                app.set_iq_text(SharedString::from(format!("{:.2} A", r.iq)));
                                app.set_erpm_text(SharedString::from(format!("{:.0}", r.erpm)));
                                app.set_rpm_text(SharedString::from(format!("{:.0}", r.mech_rpm)));
                            } else {
                                // No calibration yet — show raw ADC counts.
                                app.set_ia_text(SharedString::from(format!("{} cnt", s.ia)));
                                app.set_ib_text(SharedString::from(format!("{} cnt", s.ib)));
                                app.set_ic_text(SharedString::from(format!("{} cnt", s.ic)));
                            }
                            app.set_seq_text(SharedString::from(format!("{}", s.seq)));
                        }
                        if let Some(s) = last_slow {
                            app.set_vbus_text(SharedString::from(format!(
                                "{:.2} V",
                                s.vbus_mv as f32 / 1000.0
                            )));
                            app.set_temp_text(SharedString::from(format!(
                                "{:.1} °C",
                                f32::from(s.fet_temp_c_x10) / 10.0
                            )));
                            let state_str = format!("{:?}", s.motor_state);
                            app.set_motor_state_text(SharedString::from(state_str));
                            app.set_phase_source_text(SharedString::from(ops::phase::label(
                                s.phase_source,
                            )));
                            if s.fault_count > 0 {
                                app.set_fault_text(SharedString::from(format!(
                                    "{}",
                                    s.fault_count
                                )));
                            } else {
                                app.set_fault_text(SharedString::from(""));
                            }
                        }

                        // Set sample rate for plot interaction
                        let actual_hz = fhz.load(Ordering::Relaxed);
                        let fast_rate = if actual_hz > 0 {
                            f32::from(actual_hz)
                        } else {
                            1000.0
                        };
                        app.set_fast_sample_rate(fast_rate);
                        app.set_streaming(actual_hz > 0);

                        // Surface silent frame drops: warn when the measured
                        // arrival rate is well below the device-acked rate
                        // (link bandwidth exceeded — pick a lower stream rate).
                        if actual_hz > 0 && measured_hz > 0.0 && measured_hz < 0.8 * fast_rate {
                            app.set_rate_warning(SharedString::from(format!(
                                "link drops frames: {measured_hz:.0} Hz of {actual_hz} Hz arriving"
                            )));
                        } else {
                            app.set_rate_warning(SharedString::from(""));
                        }

                        // Per-plot time windows and view offsets (each plot can be independently paused/zoomed)
                        let c_tw = app.get_currents_time_window();
                        let c_vis = (c_tw * fast_rate) as u32;
                        let c_off = app.get_currents_view_offset().max(0) as u32;

                        let v_tw = app.get_vbus_time_window();
                        let v_vis = (v_tw * 10.0) as u32;
                        let v_off = app.get_vbus_view_offset().max(0) as u32;

                        let t_tw = app.get_temp_time_window();
                        let t_vis = (t_tw * 10.0) as u32;
                        let t_off = app.get_temp_view_offset().max(0) as u32;

                        let h_tw = app.get_hall_time_window();
                        let h_vis = (h_tw * fast_rate) as u32;
                        let h_off = app.get_hall_view_offset().max(0) as u32;

                        let (tex, y_lo, y_hi) = cr.render(
                            &cb,
                            app.get_currents_w() as u32,
                            app.get_currents_h() as u32,
                            c_vis,
                            c_off,
                        );
                        app.set_currents_texture(Image::try_from(tex).unwrap());
                        app.set_currents_y_min(y_lo);
                        app.set_currents_y_max(y_hi);

                        let (tex, y_lo, y_hi) = vr.render(
                            &vb,
                            app.get_vbus_w() as u32,
                            app.get_vbus_h() as u32,
                            v_vis,
                            v_off,
                        );
                        app.set_vbus_texture(Image::try_from(tex).unwrap());
                        app.set_vbus_y_min(y_lo);
                        app.set_vbus_y_max(y_hi);

                        let (tex, y_lo, y_hi) = tr.render(
                            &tb,
                            app.get_temp_w() as u32,
                            app.get_temp_h() as u32,
                            t_vis,
                            t_off,
                        );
                        app.set_temp_texture(Image::try_from(tex).unwrap());
                        app.set_temp_y_min(y_lo);
                        app.set_temp_y_max(y_hi);

                        let (tex, y_lo, y_hi) = hr.render(
                            &hb,
                            app.get_hall_w() as u32,
                            app.get_hall_h() as u32,
                            h_vis,
                            h_off,
                        );
                        app.set_hall_texture(Image::try_from(tex).unwrap());
                        app.set_hall_y_min(y_lo);
                        app.set_hall_y_max(y_hi);

                        // Keep rendering continuously so charts update with data.
                        app.window().request_redraw();
                    }
                }
                RenderingState::RenderingTeardown => {
                    drop(cr.take());
                    drop(vr.take());
                    drop(tr.take());
                    // hr too — leaking it kept a wgpu Device/Queue alive past
                    // the graphics context and rendered with a stale device
                    // after a context recreate.
                    drop(hr.take());
                }
                _ => {}
            })
            .expect("Unable to set rendering notifier");
    }

    // ── Refresh serial ports ──────────────────────────────────────────────────
    #[cfg(feature = "desktop")]
    {
        let weak = app.as_weak();
        let ports = ports_list.clone();
        app.on_refresh_serial_ports(move || {
            refresh_serial_ports(&weak.unwrap(), &ports);
        });
    }

    // ── Refresh probes ────────────────────────────────────────────────────────
    #[cfg(feature = "desktop")]
    {
        let weak = app.as_weak();
        let probes = probes_list.clone();
        app.on_refresh_probes(move || {
            refresh_probes(&weak.unwrap(), &probes);
        });
    }

    // ── Scan BLE ─────────────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let ble_devs = ble_devices_list.clone();
        app.on_scan_ble(move || {
            let weak = weak.clone();
            let ble_devs = ble_devs.clone();
            // Set scanning flag
            if let Some(app) = weak.upgrade() {
                app.set_ble_scanning(true);
            }
            thread::spawn(move || {
                let rt = tokio::runtime::Runtime::new().unwrap();
                let devices = rt.block_on(scan_ble_devices(Duration::from_secs(4)));
                let items: Vec<StandardListViewItem> = devices
                    .iter()
                    .map(|d| SharedString::from(d.to_string()).into())
                    .collect();
                *ble_devs.lock().unwrap() = devices;
                let _ = weak.upgrade_in_event_loop(move |app| {
                    app.set_ble_devices(ModelRc::new(VecModel::from(items)));
                    app.set_selected_ble(-1);
                    app.set_ble_scanning(false);
                });
            });
        });
    }

    //── Connect device ────────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        #[cfg(feature = "desktop")]
        let ports = ports_list.clone();
        #[cfg(feature = "desktop")]
        let probes = probes_list.clone();
        let ble_devs = ble_devices_list.clone();
        let rt = runtime.clone();
        let stop = stop_adc.clone();
        let frx_slot = fast_rx_slot.clone();
        let srx_slot = slow_rx_slot.clone();
        let conn_flag = connected_flag.clone();
        let esl_connect = enrich_slot.clone();
        let fhz = fast_hz.clone();

        app.on_connect_device(move || {
            let app = weak.unwrap();
            stop.store(false, Ordering::Relaxed);

            let mode = app.get_transport_mode();
            let config = match mode {
                #[cfg(feature = "desktop")]
                0 => {
                    // Serial
                    let idx = app.get_selected_serial();
                    let guard = ports.lock().unwrap();
                    if idx < 0 || idx as usize >= guard.len() {
                        app.set_error_text("No serial port selected".into());
                        return;
                    }
                    let port = &guard[idx as usize];
                    let baud = BAUD_RATES[app.get_baud_index().clamp(0, 5) as usize];
                    HostConfig {
                        transport: Some(TransportType::Serial),
                        serial_path: Some(port.path.clone()),
                        serial_baud: Some(baud),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                #[cfg(feature = "desktop")]
                1 => {
                    // RTT
                    let idx = app.get_selected_probe();
                    let guard = probes.lock().unwrap();
                    if idx < 0 || idx as usize >= guard.len() {
                        app.set_error_text("No probe selected".into());
                        return;
                    }
                    let probe = &guard[idx as usize];
                    let chip = app.get_chip_name().to_string();
                    if chip.is_empty() {
                        app.set_error_text("Chip name required".into());
                        return;
                    }
                    HostConfig {
                        transport: Some(TransportType::Rtt),
                        probe: Some(probe.identifier.clone()),
                        chip: Some(chip),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                2 => {
                    // TCP
                    let host = app.get_tcp_host().to_string();
                    let port_str = app.get_tcp_port().to_string();
                    let port: u16 = match port_str.parse() {
                        Ok(p) => p,
                        Err(_) => {
                            app.set_error_text("Invalid TCP port".into());
                            return;
                        }
                    };
                    HostConfig {
                        transport: Some(TransportType::Tcp),
                        tcp_host: Some(host),
                        tcp_port: Some(port),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                3 => {
                    // UDP
                    let host = app.get_udp_host().to_string();
                    let port_str = app.get_udp_port().to_string();
                    let port: u16 = match port_str.parse() {
                        Ok(p) => p,
                        Err(_) => {
                            app.set_error_text("Invalid UDP port".into());
                            return;
                        }
                    };
                    HostConfig {
                        transport: Some(TransportType::Udp),
                        udp_host: Some(host),
                        udp_port: Some(port),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                4 => {
                    // USB
                    HostConfig {
                        transport: Some(TransportType::Usb),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                5 => {
                    // BLE
                    let idx = app.get_selected_ble();
                    let guard = ble_devs.lock().unwrap();
                    if idx < 0 || idx as usize >= guard.len() {
                        app.set_error_text("No BLE device selected".into());
                        return;
                    }
                    let device = &guard[idx as usize];
                    HostConfig {
                        transport: Some(TransportType::Ble),
                        ble_device: Some(device.device.clone()),
                        stream_defmt: Some(true),
                        stream_ergot: Some(true),
                        ..Default::default()
                    }
                }
                _ => return,
            };

            // Set fast_hz so the backend enables telemetry at connect time
            let hz = ops::stream_rate_hz(app.get_stream_rate_index() as usize);
            let config = HostConfig {
                fast_hz: Some(hz),
                ..config
            };

            app.set_error_text("".into());

            // Shut down any previous backend first (and drop it, releasing
            // the port/probe) before the new transport claims the device —
            // previously the old runtime was silently overwritten and leaked
            // its tokio runtime + thread, still holding the port.
            if let Some(old) = rt.lock().unwrap().take() {
                old.shutdown();
                drop(old);
                // Give the old transport thread a moment to actually close.
                thread::sleep(Duration::from_millis(100));
            }

            let host_runtime = start_host(config);
            let fast_rx = host_runtime.fast_rx.clone();
            let slow_rx = host_runtime.slow_rx.clone();
            let info_rx = host_runtime.device_info_rx.clone();
            let fault_rx = host_runtime.fault_rx.clone();
            let connected = host_runtime.connected.clone();
            let runtime_fast_hz = host_runtime.fast_hz.clone();
            *rt.lock().unwrap() = Some(host_runtime);

            // Store receivers for BeforeRendering to drain into PlotBuffers
            *frx_slot.lock().unwrap() = Some(fast_rx);
            *srx_slot.lock().unwrap() = Some(slow_rx);

            // Propagate connected + fast_hz flags for rendering notifier
            conn_flag.store(true, Ordering::Relaxed);
            fhz.store(runtime_fast_hz.load(Ordering::Relaxed), Ordering::Relaxed);

            // Continuously propagate flags in background
            let conn_src = connected;
            let conn_dst = conn_flag.clone();
            let fhz_src = runtime_fast_hz;
            let fhz_dst = fhz.clone();
            let stop2 = stop.clone();
            thread::spawn(move || {
                while !stop2.load(Ordering::Relaxed) {
                    conn_dst.store(conn_src.load(Ordering::Relaxed), Ordering::Relaxed);
                    fhz_dst.store(fhz_src.load(Ordering::Relaxed), Ordering::Relaxed);
                    thread::sleep(Duration::from_millis(100));
                }
            });

            // Fault listener — the device pushes a full snapshot on every
            // registry change (raise / refine / clear); mirror it into the UI
            // model. Initial population also happens when the Faults tab is
            // opened (it issues a Query).
            {
                let weak_f = weak.clone();
                let stop_f = stop.clone();
                thread::spawn(move || {
                    while !stop_f.load(Ordering::Relaxed) {
                        match fault_rx.recv_timeout(Duration::from_millis(200)) {
                            Ok(resp) => {
                                let _ = weak_f
                                    .upgrade_in_event_loop(move |app| apply_faults(&app, &resp));
                            }
                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                        }
                    }
                });
            }

            // Device info listener — runs once on connection: mirror identity to
            // the UI AND build the raw→engineering enrichment context. Clone
            // cmd_tx out of the runtime mutex first, so the blocking config reads
            // (DcOffsets, MotorParams) don't hold the lock.
            let weak_info = weak.clone();
            let rt_info = rt.clone();
            let esl_info = esl_connect.clone();
            thread::spawn(move || {
                if let Ok(info) = info_rx.recv() {
                    let cmd_tx = rt_info.lock().unwrap().as_ref().map(|r| r.cmd_tx.clone());
                    if let Some(cmd_tx) = cmd_tx
                        && let Some(ctx) = oxifoc_host_lib::build_enrich_ctx(&cmd_tx, Some(&info))
                    {
                        *esl_info.lock().unwrap() = Some(ctx);
                    }
                    let _ = weak_info.upgrade_in_event_loop(move |app| {
                        app.set_device_hw(info.hw.as_str().into());
                        app.set_device_sw(info.sw.as_str().into());
                        app.set_device_mcu(info.mcu.as_str().into());
                        app.set_device_uuid(info.uuid.as_str().into());
                        app.set_device_foc_hz(info.foc_freq_hz as i32);
                        app.set_device_max_current(info.max_current_a);
                    });
                }
            });

            // Pole pairs from the device's stored MotorParams (when present):
            // the RPM display must not depend on the GUI preset matching the
            // motor that's actually connected.
            {
                let rt = rt.clone();
                let weak = weak.clone();
                let connected = conn_flag.clone();
                thread::spawn(move || {
                    for _ in 0..100 {
                        if connected.load(Ordering::Relaxed) {
                            break;
                        }
                        thread::sleep(Duration::from_millis(100));
                    }
                    if !connected.load(Ordering::Relaxed) {
                        return;
                    }
                    let Some(cmd_tx) = rt.lock().unwrap().as_ref().map(|r| r.cmd_tx.clone()) else {
                        return;
                    };
                    let (tx, rx) = config_channel();
                    if cmd_tx
                        .send(HostCommand::ConfigRead(
                            oxifoc_core::types::ConfigGroupId::MotorParams,
                            tx,
                        ))
                        .is_err()
                    {
                        return;
                    }
                    if let Ok(Ok(oxifoc_core::types::ConfigResponse::MotorParams(p))) =
                        rx.blocking_recv()
                        && p.pole_pairs > 0
                    {
                        tracing::info!(
                            "Using device pole pairs: {} (stored MotorParams)",
                            p.pole_pairs
                        );
                        let _ = weak.upgrade_in_event_loop(move |app| {
                            app.set_pole_pairs(i32::from(p.pole_pairs));
                        });
                    }
                });
            }

            app.set_page("main".into());
        });
    }

    // ── Disconnect device ─────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let rt = runtime.clone();
        let stop = stop_adc.clone();
        let frx_slot = fast_rx_slot.clone();
        let srx_slot = slow_rx_slot.clone();
        let conn_flag = connected_flag.clone();
        app.on_disconnect_device(move || {
            stop.store(true, Ordering::Relaxed);
            conn_flag.store(false, Ordering::Relaxed);
            *frx_slot.lock().unwrap() = None;
            *srx_slot.lock().unwrap() = None;
            if let Some(runtime) = rt.lock().unwrap().take() {
                runtime.shutdown();
            }
            let app = weak.unwrap();
            app.set_page("connect".into());
            app.set_is_connected(false);
            // The device's link-loss failsafe stops (and latches) the motor
            // on its own — mirror that in the UI instead of leaving a stale
            // "running" Start/Stop state for the next session.
            app.set_motor_running(false);
        });
    }

    // ── Motor start ───────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_start(move || {
            let app = weak.unwrap();
            let iq_target = app.get_iq_target();
            let id_target = app
                .get_id_target_text()
                .trim()
                .parse::<f32>()
                .unwrap_or(0.0);
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor start: iq_target={iq_target:.2}A id_target={id_target:.2}A");
                match runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::CurrentControl {
                        iq_target,
                        id_target,
                    })) {
                    Ok(()) => {
                        drop(guard);
                        app.set_motor_running(true);
                    }
                    Err(e) => tracing::error!("Failed to send motor command: {e}"),
                }
            } else {
                tracing::warn!("Motor start clicked but no runtime");
            }
        });
    }

    // ── Motor stop ────────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_stop(move || {
            let app = weak.unwrap();
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor stop");
                match runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::Stopped))
                {
                    Ok(()) => {
                        drop(guard);
                        app.set_motor_running(false);
                    }
                    Err(e) => tracing::error!("Failed to send stop command: {e}"),
                }
            } else {
                tracing::warn!("Motor stop clicked but no runtime");
            }
        });
    }

    // ── Motor coast (gates off, free spin) ───────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_coast(move || {
            let app = weak.unwrap();
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor coast");
                if runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::Coast))
                    .is_ok()
                {
                    drop(guard);
                    app.set_motor_running(false);
                }
            }
        });
    }

    // ── Motor brake (short windings, parking brake) ──────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_brake(move || {
            let app = weak.unwrap();
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor brake");
                if runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::Brake))
                    .is_ok()
                {
                    drop(guard);
                    app.set_motor_running(false);
                }
            }
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
                ));
            }
        });
    }

    // ── Config read ──────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_read(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                tracing::warn!("Config read clicked but no runtime");
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();
            let group_idx = app.get_config_group();

            use oxifoc_core::types::ConfigGroupId;
            let group_id = match group_idx {
                0 => ConfigGroupId::MotorParams,
                1 => ConfigGroupId::CurrentLimits,
                2 => ConfigGroupId::VoltageLimits,
                3 => ConfigGroupId::PiGains,
                4 => ConfigGroupId::Velocity,
                5 => ConfigGroupId::Failsafe,
                _ => return,
            };

            let (tx, rx) = config_channel();
            if runtime
                .cmd_tx
                .send(HostCommand::ConfigRead(group_id, tx))
                .is_err()
            {
                return;
            }

            thread::spawn(move || {
                let result = rx.blocking_recv();
                let _ = weak.upgrade_in_event_loop(move |app| {
                    use oxifoc_core::types::ConfigResponse;
                    match result {
                        Ok(Ok(resp)) => {
                            match resp {
                                ConfigResponse::MotorParams(p) => {
                                    app.set_cfg_resistance(SharedString::from(format!(
                                        "{}",
                                        p.resistance_ohm
                                    )));
                                    app.set_cfg_inductance_d(SharedString::from(format!(
                                        "{}",
                                        p.inductance_d_h
                                    )));
                                    app.set_cfg_inductance_q(SharedString::from(format!(
                                        "{}",
                                        p.inductance_q_h
                                    )));
                                    app.set_cfg_flux_linkage(SharedString::from(format!(
                                        "{}",
                                        p.flux_linkage_wb
                                    )));
                                    app.set_cfg_pole_pairs(SharedString::from(format!(
                                        "{}",
                                        p.pole_pairs
                                    )));
                                    app.set_cfg_motor_rating(SharedString::from(format!(
                                        "{}",
                                        p.max_current_a
                                    )));
                                    app.set_cfg_motor_power_loss(SharedString::from(format!(
                                        "{}",
                                        p.max_power_loss_w
                                    )));
                                }
                                ConfigResponse::CurrentLimits(c) => {
                                    app.set_cfg_max_iq(SharedString::from(format!(
                                        "{}",
                                        c.max_iq_a
                                    )));
                                    app.set_cfg_max_phase_current(SharedString::from(format!(
                                        "{}",
                                        c.max_phase_current_a
                                    )));
                                    app.set_cfg_bus_in_max(SharedString::from(format!(
                                        "{}",
                                        c.bus_in_max_a
                                    )));
                                    app.set_cfg_bus_regen_max(SharedString::from(format!(
                                        "{}",
                                        c.bus_regen_max_a
                                    )));
                                }
                                ConfigResponse::VoltageLimits(v) => {
                                    app.set_cfg_min_vbus(SharedString::from(format!(
                                        "{}",
                                        v.min_vbus_mv
                                    )));
                                    app.set_cfg_max_vbus(SharedString::from(format!(
                                        "{}",
                                        v.max_vbus_mv
                                    )));
                                }
                                ConfigResponse::PiGains(g) => {
                                    app.set_cfg_kp(SharedString::from(format!("{}", g.kp)));
                                    app.set_cfg_ki(SharedString::from(format!("{}", g.ki)));
                                    app.set_cfg_bandwidth(SharedString::from(format!(
                                        "{}",
                                        g.bandwidth_rad_s
                                    )));
                                }
                                ConfigResponse::Velocity(v) => {
                                    app.set_cfg_vel_kp(SharedString::from(format!("{}", v.kp)));
                                    app.set_cfg_vel_ki(SharedString::from(format!("{}", v.ki)));
                                    app.set_cfg_vel_accel(SharedString::from(format!(
                                        "{}",
                                        v.accel_limit
                                    )));
                                }
                                ConfigResponse::Failsafe(f) => {
                                    app.set_cfg_fs_staleness(SharedString::from(format!(
                                        "{}",
                                        f.staleness_timeout_ms
                                    )));
                                    app.set_cfg_fs_policy(SharedString::from(format!(
                                        "{}",
                                        f.policy
                                    )));
                                    app.set_cfg_fs_brake_current(SharedString::from(format!(
                                        "{}",
                                        f.brake_current_a
                                    )));
                                    app.set_cfg_fs_ramp(SharedString::from(format!(
                                        "{}",
                                        f.ramp_ms
                                    )));
                                    app.set_cfg_fs_brake_time(SharedString::from(format!(
                                        "{}",
                                        f.brake_time_ms
                                    )));
                                    app.set_cfg_fs_standstill(SharedString::from(format!(
                                        "{}",
                                        f.standstill_rad_s
                                    )));
                                    app.set_cfg_fs_decel(SharedString::from(format!(
                                        "{}",
                                        f.decel_rad_s2
                                    )));
                                    app.set_cfg_fs_terminal(SharedString::from(format!(
                                        "{}",
                                        f.terminal
                                    )));
                                }
                                ConfigResponse::NotFound => {
                                    app.set_config_status(SharedString::from("Not stored"));
                                    return;
                                }
                                _ => {}
                            }
                            app.set_config_status(SharedString::from("OK"));
                        }
                        Ok(Err(e)) => {
                            app.set_config_status(SharedString::from(format!("Error: {e}")));
                        }
                        Err(_) => {
                            app.set_config_status(SharedString::from("No response"));
                        }
                    }
                });
            });
        });
    }

    // ── Config write ─────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_write(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                tracing::warn!("Config write clicked but no runtime");
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();
            let group_idx = app.get_config_group();

            use oxifoc_core::storage::*;
            use oxifoc_core::types::ConfigWrite;

            let mut parse_err: Option<String> = None;
            let err = &mut parse_err;
            let write = match group_idx {
                0 => {
                    let r: f32 = parse_field("resistance", &app.get_cfg_resistance(), err);
                    let ld: f32 = parse_field("inductance d", &app.get_cfg_inductance_d(), err);
                    let lq: f32 = parse_field("inductance q", &app.get_cfg_inductance_q(), err);
                    let fl: f32 = parse_field("flux linkage", &app.get_cfg_flux_linkage(), err);
                    let pp: u8 = parse_field("pole pairs", &app.get_cfg_pole_pairs(), err);
                    let rating: f32 = parse_field("motor rating", &app.get_cfg_motor_rating(), err);
                    let ploss: f32 =
                        parse_field("motor power loss", &app.get_cfg_motor_power_loss(), err);
                    ConfigWrite::MotorParams(MotorParamsConfig {
                        resistance_ohm: r,
                        inductance_d_h: ld,
                        inductance_q_h: lq,
                        flux_linkage_wb: fl,
                        pole_pairs: pp,
                        max_current_a: rating,
                        max_power_loss_w: ploss,
                    })
                }
                1 => {
                    let iq: f32 = parse_field("max iq", &app.get_cfg_max_iq(), err);
                    let ph: f32 =
                        parse_field("max phase current", &app.get_cfg_max_phase_current(), err);
                    let bus_in: f32 = parse_field("bus in max", &app.get_cfg_bus_in_max(), err);
                    let bus_regen: f32 =
                        parse_field("bus regen max", &app.get_cfg_bus_regen_max(), err);
                    ConfigWrite::CurrentLimits(CurrentLimitsConfig {
                        max_iq_a: iq,
                        max_phase_current_a: ph,
                        bus_in_max_a: bus_in,
                        bus_regen_max_a: bus_regen,
                    })
                }
                2 => {
                    let min: u32 = parse_field("min vbus", &app.get_cfg_min_vbus(), err);
                    let max: u32 = parse_field("max vbus", &app.get_cfg_max_vbus(), err);
                    ConfigWrite::VoltageLimits(VoltageLimitsConfig {
                        min_vbus_mv: min,
                        max_vbus_mv: max,
                    })
                }
                3 => {
                    let kp: f32 = parse_field("kp", &app.get_cfg_kp(), err);
                    let ki: f32 = parse_field("ki", &app.get_cfg_ki(), err);
                    let bw: f32 = parse_field("bandwidth", &app.get_cfg_bandwidth(), err);
                    ConfigWrite::PiGains(PiGainsConfig {
                        kp,
                        ki,
                        bandwidth_rad_s: bw,
                    })
                }
                4 => {
                    let kp: f32 = parse_field("velocity kp", &app.get_cfg_vel_kp(), err);
                    let ki: f32 = parse_field("velocity ki", &app.get_cfg_vel_ki(), err);
                    let accel: f32 = parse_field("accel limit", &app.get_cfg_vel_accel(), err);
                    ConfigWrite::Velocity(VelocityConfigStored {
                        kp,
                        ki,
                        accel_limit: accel,
                    })
                }
                5 => {
                    let staleness: u32 = parse_field("staleness", &app.get_cfg_fs_staleness(), err);
                    let policy: u8 = parse_field("policy", &app.get_cfg_fs_policy(), err);
                    let brake: f32 =
                        parse_field("brake current", &app.get_cfg_fs_brake_current(), err);
                    let ramp: f32 = parse_field("ramp", &app.get_cfg_fs_ramp(), err);
                    let brake_time: f32 =
                        parse_field("brake time", &app.get_cfg_fs_brake_time(), err);
                    let standstill: f32 =
                        parse_field("standstill", &app.get_cfg_fs_standstill(), err);
                    let decel: f32 = parse_field("decel", &app.get_cfg_fs_decel(), err);
                    let terminal: u8 = parse_field("terminal", &app.get_cfg_fs_terminal(), err);
                    ConfigWrite::Failsafe(FailsafeConfigStored {
                        staleness_timeout_ms: staleness,
                        policy,
                        brake_current_a: brake,
                        ramp_ms: ramp,
                        brake_time_ms: brake_time,
                        standstill_rad_s: standstill,
                        decel_rad_s2: decel,
                        terminal,
                    })
                }
                _ => return,
            };

            // A typo must never reach flash as a zero — abort the write.
            if let Some(msg) = parse_err {
                app.set_config_status(SharedString::from(msg));
                return;
            }

            let (tx, rx) = config_channel();
            if runtime
                .cmd_tx
                .send(HostCommand::ConfigWrite(write, tx))
                .is_err()
            {
                return;
            }

            thread::spawn(move || {
                let result = rx.blocking_recv();
                let _ = weak.upgrade_in_event_loop(move |app| match result {
                    Ok(Ok(_)) => {
                        app.set_config_status(SharedString::from("Written OK"));
                    }
                    Ok(Err(e)) => {
                        app.set_config_status(SharedString::from(format!("Error: {e}")));
                    }
                    Err(_) => {
                        app.set_config_status(SharedString::from("No response"));
                    }
                });
            });
        });
    }

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

    // ── Config reset (factory reset) ─────────────────────────────────────────
    // Guarded by the UI "confirm" toggle (the button is disabled otherwise).
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_reset(move || {
            let app = weak.unwrap();
            if !app.get_config_reset_confirm() {
                return;
            }
            let cmd_tx = {
                let guard = rt.lock().unwrap();
                let Some(ref runtime) = *guard else {
                    app.set_config_status(SharedString::from("Not connected"));
                    return;
                };
                runtime.cmd_tx.clone()
            };
            app.set_config_status(SharedString::from("Resetting..."));
            let weak = weak.clone();
            thread::spawn(move || {
                let result = ops::config::reset_all(&cmd_tx);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    app.set_config_reset_confirm(false);
                    match result {
                        Ok(()) => app.set_config_status(SharedString::from("Reset to defaults")),
                        Err(e) => app.set_config_status(SharedString::from(format!("Error: {e}"))),
                    }
                });
            });
        });
    }

    // ── Faults: refresh / clear-all / clear-one ──────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_faults_refresh(move || {
            send_fault_request(&rt, &weak, FaultRequest::Query);
        });
    }
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_clear_faults(move || {
            send_fault_request(&rt, &weak, FaultRequest::ClearAll);
        });
    }
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_clear_fault_category(move |id| {
            if let Some(cat) = fault_category_from_id(id) {
                send_fault_request(&rt, &weak, FaultRequest::Clear(cat));
            }
        });
    }

    app.run().unwrap();

    stop_adc.store(true, Ordering::Relaxed);
    if let Some(rt) = runtime.lock().unwrap().take() {
        rt.shutdown();
    }
}

#[cfg(feature = "desktop")]
fn refresh_serial_ports(app: &App, ports: &Arc<std::sync::Mutex<Vec<SerialPortInfo>>>) {
    let usb_only = app.get_usb_only();
    let all_ports = list_serial_ports();
    let filtered: Vec<SerialPortInfo> = if usb_only {
        all_ports
            .into_iter()
            .filter(|p| {
                let path = p.path.to_lowercase();
                path.contains("ttyacm") || path.contains("ttyusb") || path.contains("ttyama")
            })
            .collect()
    } else {
        all_ports
    };
    let items: Vec<StandardListViewItem> = filtered
        .iter()
        .map(|p| SharedString::from(p.to_string()).into())
        .collect();
    *ports.lock().unwrap() = filtered;
    app.set_serial_ports(ModelRc::new(VecModel::from(items)));
    app.set_selected_serial(-1);
}

#[cfg(feature = "desktop")]
fn refresh_probes(app: &App, probes: &Arc<std::sync::Mutex<Vec<ProbeInfo>>>) {
    let all_probes = list_probes();
    let items: Vec<StandardListViewItem> = all_probes
        .iter()
        .map(|p| SharedString::from(p.to_string()).into())
        .collect();
    *probes.lock().unwrap() = all_probes;
    app.set_probes(ModelRc::new(VecModel::from(items)));
    app.set_selected_probe(-1);
}
