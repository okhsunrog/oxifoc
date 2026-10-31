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

mod configuration;
mod detection;
mod faults;
mod logging;
mod motor;
mod presets;

use faults::apply_faults;
use logging::UiLogLayer;

type SharedRuntime = Arc<std::sync::Mutex<Option<HostRuntime>>>;

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: slint::android::AndroidApp) {
    slint::android::init(app).unwrap();
    main();
}

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread;
use std::time::Duration;

use oxifoc_core::types::{ConfigResponse, ConfigValue, ControlMode, MotorState};
use oxifoc_host_lib::{
    BleDeviceInfo, HostCommand, HostConfig, HostRuntime, TransportType, config_channel, ops,
    scan_ble_devices, start_host,
};
#[cfg(feature = "desktop")]
use oxifoc_host_lib::{ProbeInfo, SerialPortInfo, list_probes, list_serial_ports};
use slint::wgpu_30::WGPUConfiguration;
use slint::{
    GraphicsAPI, Image, ModelRc, RenderingState, SharedString, StandardListViewItem, VecModel,
};
use slint_wgpu_plot::{PlotBuffer, PlotConfig, PlotRenderer, required_wgpu_settings};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const CAPACITY: usize = 32768;
const MAX_LOG_LINES: usize = 2000;
const BAUD_RATES: [u32; 6] = [115200, 230400, 460800, 921600, 1_000_000, 2_000_000];

fn cursor_readout(
    buffer: &PlotBuffer,
    fraction: f32,
    visible_samples: u32,
    view_offset: u32,
    sample_rate: f32,
    channels: &[&str],
    visible_channels: Option<&[bool]>,
) -> SharedString {
    if buffer.available_samples() == 0 {
        return SharedString::default();
    }
    debug_assert_eq!(channels.len(), buffer.num_channels);
    let visible = visible_samples.clamp(2, buffer.capacity as u32);
    let offset = view_offset.min(buffer.available_samples().saturating_sub(visible));
    let index =
        (oxifoc_core::foc::clamp_f32(fraction, 0.0, 1.0) * (visible - 1) as f32).round() as u32;
    let frames_back = (offset + visible - index) as usize;
    let mut frame = vec![0.0; buffer.num_channels];
    buffer.read_back(frames_back, &mut frame);
    let mut text = format!(
        "t: -{:.0} ms",
        (frames_back - 1) as f32 * 1000.0 / sample_rate
    );
    for (index, (name, value)) in channels.iter().zip(frame).enumerate() {
        if visible_channels.is_some_and(|visible| !visible[index]) {
            continue;
        }
        text.push_str(&format!(
            "  {name} {}",
            if value.is_finite() {
                format!("{value:.2}")
            } else {
                "—".to_string()
            }
        ));
    }
    SharedString::from(text)
}

/// Parse a config/detection field, preserving the first error. Callers abort
/// the write when an error is set, so the fallback never reaches the device.
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

fn parse_id_target(value: &SharedString) -> Result<f32, String> {
    let trimmed = value.trim();
    match trimmed.parse::<f32>() {
        Ok(v) if v.is_finite() => Ok(v),
        _ => Err(format!("Invalid Id: '{}'", value.as_str())),
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
    // The current chart overlays five signals on the same ampere axis.
    let wgpu_settings = required_wgpu_settings(CAPACITY, 5);
    slint::BackendSelector::new()
        .require_wgpu_30(WGPUConfiguration::Automatic(wgpu_settings))
        .select()
        .expect("Failed to initialise WGPU backend");

    let app = App::new().unwrap();

    // ── Log + faults models ──────────────────────────────────────────────────
    app.set_log_messages(ModelRc::new(VecModel::<LogMessage>::default()));
    app.set_faults(ModelRc::new(VecModel::<FaultRow>::default()));

    logging::bind(&app, log_rx);

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
    let motor_request_generation = Arc::new(AtomicU64::new(0));
    let stop_adc = Arc::new(AtomicBool::new(false));

    // ── Ring buffers shared between data thread and render notifier ───────────
    let currents_buf = Arc::new(PlotBuffer::new(5, CAPACITY)); // ia, ib, ic, id, iq
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
                    if let GraphicsAPI::WGPU30 { device, queue, .. } = graphics_api {
                        cr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 5,
                                capacity: CAPACITY,
                                y_min: -1.0,
                                y_max: 1.0,
                                auto_range: true,
                                channel_colors: vec![
                                    [0.133, 0.827, 0.933, 1.0], // cyan  – Phase A
                                    [0.545, 0.361, 0.965, 1.0], // violet – Phase B
                                    [0.976, 0.451, 0.086, 1.0], // orange – Phase C
                                    [0.141, 0.655, 0.349, 1.0], // green – Id
                                    [0.918, 0.271, 0.463, 1.0], // rose – Iq
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
                                    Some(r) => cb.push_frame(&[r.ia, r.ib, r.ic, r.id, r.iq]),
                                    None => cb.push_frame(&[
                                        f32::from(sample.ia),
                                        f32::from(sample.ib),
                                        f32::from(sample.ic),
                                        f32::NAN,
                                        f32::NAN,
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
                        let current_visibility = [
                            app.get_show_ia(),
                            app.get_show_ib(),
                            app.get_show_ic(),
                            app.get_show_id(),
                            app.get_show_iq(),
                        ];
                        cr.set_channel_visibility(&current_visibility);
                        // Throttled motor update: send at most once per frame (~60Hz)
                        if motor_pending.swap(false, Ordering::Relaxed) {
                            let iq_target = app.get_iq_target();
                            match parse_id_target(&app.get_id_target_text()) {
                                Ok(id_target) => {
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
                                Err(msg) => {
                                    app.set_motor_status(SharedString::from(msg));
                                }
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
                                if r.erpm.abs() < 0.5 && r.mech_rpm.abs() > 1.0 {
                                    app.set_erpm_text(SharedString::from("--"));
                                } else {
                                    app.set_erpm_text(SharedString::from(format!("{:.0}", r.erpm)));
                                }
                                app.set_rpm_text(SharedString::from(format!("{:.0}", r.mech_rpm)));
                            } else {
                                // No calibration yet — show raw ADC counts.
                                app.set_ia_text(SharedString::from(format!("{} cnt", s.ia)));
                                app.set_ib_text(SharedString::from(format!("{} cnt", s.ib)));
                                app.set_ic_text(SharedString::from(format!("{} cnt", s.ic)));
                                app.set_id_text(SharedString::from("--"));
                                app.set_iq_text(SharedString::from("--"));
                                app.set_erpm_text(SharedString::from("--"));
                                app.set_rpm_text(SharedString::from(format!(
                                    "{:.0}",
                                    s.mech_rpm()
                                )));
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
                            app.set_motor_running(s.motor_state == MotorState::Running);
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

                        app.set_currents_available(cb.available_samples() as i32);
                        app.set_vbus_available(vb.available_samples() as i32);
                        app.set_temp_available(tb.available_samples() as i32);
                        app.set_hall_available(hb.available_samples() as i32);
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

                        let current_cursor = if app.get_currents_cursor_active() {
                            cursor_readout(
                                &cb,
                                app.get_currents_cursor_frac(),
                                c_vis,
                                c_off,
                                fast_rate,
                                &["Ia", "Ib", "Ic", "Id", "Iq"],
                                Some(&current_visibility),
                            )
                        } else {
                            SharedString::default()
                        };
                        if app.get_currents_cursor_text() != current_cursor {
                            app.set_currents_cursor_text(current_cursor);
                        }
                        let vbus_cursor = if app.get_vbus_cursor_active() {
                            cursor_readout(
                                &vb,
                                app.get_vbus_cursor_frac(),
                                v_vis,
                                v_off,
                                10.0,
                                &["V"],
                                None,
                            )
                        } else {
                            SharedString::default()
                        };
                        if app.get_vbus_cursor_text() != vbus_cursor {
                            app.set_vbus_cursor_text(vbus_cursor);
                        }
                        let temp_cursor = if app.get_temp_cursor_active() {
                            cursor_readout(
                                &tb,
                                app.get_temp_cursor_frac(),
                                t_vis,
                                t_off,
                                10.0,
                                &["°C"],
                                None,
                            )
                        } else {
                            SharedString::default()
                        };
                        if app.get_temp_cursor_text() != temp_cursor {
                            app.set_temp_cursor_text(temp_cursor);
                        }
                        let hall_cursor = if app.get_hall_cursor_active() {
                            cursor_readout(
                                &hb,
                                app.get_hall_cursor_frac(),
                                h_vis,
                                h_off,
                                fast_rate,
                                &["Angle", "eRPM/1000"],
                                None,
                            )
                        } else {
                            SharedString::default()
                        };
                        if app.get_hall_cursor_text() != hall_cursor {
                            app.set_hall_cursor_text(hall_cursor);
                        }

                        let scale_factor = app.window().scale_factor();
                        let layout = app.get_chart_layout();
                        let primary = app.get_primary_chart();
                        let secondary = app.get_secondary_chart();
                        let chart_visible = |index| {
                            app.get_active_tab() == 0
                                && match layout {
                                    0 => index == primary,
                                    1 => index == primary || index == secondary,
                                    2 => index != 3,
                                    _ => true,
                                }
                        };
                        if chart_visible(0) {
                            let output = cr.render(
                                &cb,
                                app.get_currents_w() as u32,
                                app.get_currents_h() as u32,
                                c_vis,
                                c_off,
                                scale_factor,
                            );
                            if output.rendered {
                                app.set_currents_texture(Image::try_from(output.texture).unwrap());
                                app.set_currents_y_min(output.y_min);
                                app.set_currents_y_max(output.y_max);
                                app.set_currents_y_divisions(output.y_divisions as i32);
                            }
                        }

                        if chart_visible(1) {
                            let output = vr.render(
                                &vb,
                                app.get_vbus_w() as u32,
                                app.get_vbus_h() as u32,
                                v_vis,
                                v_off,
                                scale_factor,
                            );
                            if output.rendered {
                                app.set_vbus_texture(Image::try_from(output.texture).unwrap());
                                app.set_vbus_y_min(output.y_min);
                                app.set_vbus_y_max(output.y_max);
                                app.set_vbus_y_divisions(output.y_divisions as i32);
                            }
                        }

                        if chart_visible(2) {
                            let output = tr.render(
                                &tb,
                                app.get_temp_w() as u32,
                                app.get_temp_h() as u32,
                                t_vis,
                                t_off,
                                scale_factor,
                            );
                            if output.rendered {
                                app.set_temp_texture(Image::try_from(output.texture).unwrap());
                                app.set_temp_y_min(output.y_min);
                                app.set_temp_y_max(output.y_max);
                                app.set_temp_y_divisions(output.y_divisions as i32);
                            }
                        }

                        if chart_visible(3) {
                            let output = hr.render(
                                &hb,
                                app.get_hall_w() as u32,
                                app.get_hall_h() as u32,
                                h_vis,
                                h_off,
                                scale_factor,
                            );
                            if output.rendered {
                                app.set_hall_texture(Image::try_from(output.texture).unwrap());
                                app.set_hall_y_min(output.y_min);
                                app.set_hall_y_max(output.y_max);
                                app.set_hall_y_divisions(output.y_divisions as i32);
                            }
                        }

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

            // Device info listener — mirror every successful handshake to
            // the UI AND build the raw→engineering enrichment context. Clone
            // cmd_tx out of the runtime mutex first, so the blocking config reads
            // (DcOffsets, MotorParams) don't hold the lock.
            let weak_info = weak.clone();
            let rt_info = rt.clone();
            let esl_info = esl_connect.clone();
            let stop_info = stop.clone();
            thread::spawn(move || {
                while !stop_info.load(Ordering::Relaxed) {
                    match info_rx.recv_timeout(Duration::from_millis(200)) {
                        Ok(info) => {
                            let cmd_tx = rt_info.lock().unwrap().as_ref().map(|r| r.cmd_tx.clone());
                            if let Some(cmd_tx) = cmd_tx
                                && let Some(ctx) =
                                    oxifoc_host_lib::build_enrich_ctx(&cmd_tx, Some(&info))
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
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                    }
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
                    if let Ok(Ok(ConfigResponse::Snapshot(snapshot))) = rx.blocking_recv()
                        && let Some(ConfigValue::MotorParams(p)) = snapshot.value
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
            app.set_motor_command_pending(false);
            app.set_motor_status(SharedString::from(""));
        });
    }

    motor::bind(
        &app,
        &runtime,
        &motor_request_generation,
        &motor_update_pending,
    );
    configuration::bind(&app, &runtime);
    detection::bind(&app, &runtime);
    faults::bind(&app, &runtime);

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

#[cfg(test)]
mod tests {
    use super::motor::{begin_motor_request, motor_request_is_current};
    use std::sync::atomic::AtomicU64;

    #[test]
    fn newer_motor_request_supersedes_late_response() {
        let generation = AtomicU64::new(0);
        let start = begin_motor_request(&generation);
        assert!(motor_request_is_current(&generation, start));

        let stop = begin_motor_request(&generation);
        assert!(!motor_request_is_current(&generation, start));
        assert!(motor_request_is_current(&generation, stop));
    }
}
