slint::include_modules!();

mod presets;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ProbeInfo, SerialPortInfo, TransportType, config_channel,
    list_probes, list_serial_ports, start_host,
};
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
            self.0 = format!("{:?}", value);
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        if field.name() == "message" {
            self.0 = value.to_string();
        }
    }
}

fn main() {
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

    // ── Log model ───────────────────────────────────────────────────────────
    app.set_log_messages(ModelRc::new(VecModel::<LogMessage>::default()));

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
        app.set_pole_pairs(presets::PRESETS[0].pole_pairs as i32);
    }
    {
        let weak = app.as_weak();
        app.on_preset_changed(move |name| {
            if let Some(app) = weak.upgrade()
                && let Some(preset) = presets::PRESETS.iter().find(|p| p.name == name.as_str())
            {
                app.set_pole_pairs(preset.pole_pairs as i32);
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
    let ports_list: Arc<std::sync::Mutex<Vec<SerialPortInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let probes_list: Arc<std::sync::Mutex<Vec<ProbeInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let runtime: Arc<std::sync::Mutex<Option<HostRuntime>>> = Arc::new(std::sync::Mutex::new(None));
    let stop_adc = Arc::new(AtomicBool::new(false));

    // ── Ring buffers shared between data thread and render notifier ───────────
    let currents_buf = Arc::new(PlotBuffer::new(3, CAPACITY)); // ia, ib, ic
    let vbus_buf = Arc::new(PlotBuffer::new(1, CAPACITY)); // V
    let temp_buf = Arc::new(PlotBuffer::new(1, CAPACITY)); // °C

    // Actual fast telemetry rate — set by HostRuntime after device ack
    let fast_hz: Arc<std::sync::atomic::AtomicU16> = Arc::new(std::sync::atomic::AtomicU16::new(0));

    // Shared telemetry receivers — set on connect, read in BeforeRendering
    let fast_rx_slot: Arc<
        std::sync::Mutex<Option<crossbeam_channel::Receiver<oxifoc_core::types::FastTelemetry>>>,
    > = Arc::new(std::sync::Mutex::new(None));
    let slow_rx_slot: Arc<
        std::sync::Mutex<Option<crossbeam_channel::Receiver<oxifoc_core::types::SlowTelemetry>>>,
    > = Arc::new(std::sync::Mutex::new(None));
    let connected_flag: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));

    refresh_serial_ports(&app, &ports_list);
    refresh_probes(&app, &probes_list);

    // ── Rendering notifier: creates PlotRenderers on setup, renders on every frame ──
    {
        let app_weak = app.as_weak();
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();
        let fhz = fast_hz.clone();
        let frx = fast_rx_slot.clone();
        let srx = slow_rx_slot.clone();
        let conn = connected_flag.clone();

        let mut cr: Option<PlotRenderer> = None;
        let mut vr: Option<PlotRenderer> = None;
        let mut tr: Option<PlotRenderer> = None;

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

                    // Drain telemetry — always consume from channel, only write to buffer if not paused
                    let mut last_fast = None;
                    if let Ok(guard) = frx.try_lock()
                        && let Some(ref rx) = *guard
                    {
                        while let Ok(sample) = rx.try_recv() {
                            if !currents_paused {
                                cb.push_frame(&[sample.ia, sample.ib, sample.ic]);
                            }
                            last_fast = Some(sample);
                        }
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
                                tb.push_frame(&[sample.fet_temp_c_x10 as f32 / 10.0]);
                            }
                            last_slow = Some(sample);
                        }
                    }

                    if let (Some(app), Some(cr), Some(vr), Some(tr)) =
                        (app_weak.upgrade(), cr.as_mut(), vr.as_mut(), tr.as_mut())
                    {
                        // Update connection status + text from latest samples
                        app.set_is_connected(conn.load(std::sync::atomic::Ordering::Relaxed));
                        if let Some(s) = last_fast {
                            app.set_ia_text(SharedString::from(format!("{:.2} A", s.ia)));
                            app.set_ib_text(SharedString::from(format!("{:.2} A", s.ib)));
                            app.set_ic_text(SharedString::from(format!("{:.2} A", s.ic)));
                            app.set_erpm_text(SharedString::from(format!("{}", s.erpm)));
                            let pole_pairs = app.get_pole_pairs().max(1);
                            let rpm = s.erpm / pole_pairs;
                            app.set_rpm_text(SharedString::from(format!("{}", rpm)));
                            app.set_seq_text(SharedString::from(format!("{}", s.seq)));
                        }
                        if let Some(s) = last_slow {
                            app.set_vbus_text(SharedString::from(format!(
                                "{:.2} V",
                                s.vbus_mv as f32 / 1000.0
                            )));
                            app.set_temp_text(SharedString::from(format!(
                                "{:.1} °C",
                                s.fet_temp_c_x10 as f32 / 10.0
                            )));
                        }

                        // Set sample rate for plot interaction
                        let actual_hz = fhz.load(std::sync::atomic::Ordering::Relaxed);
                        let fast_rate = if actual_hz > 0 {
                            actual_hz as f32
                        } else {
                            1000.0
                        };
                        app.set_fast_sample_rate(fast_rate);
                        app.set_streaming(actual_hz > 0);

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

                        // Keep rendering continuously so charts update with data.
                        app.window().request_redraw();
                    }
                }
                RenderingState::RenderingTeardown => {
                    drop(cr.take());
                    drop(vr.take());
                    drop(tr.take());
                }
                _ => {}
            })
            .expect("Unable to set rendering notifier");
    }

    // ── Refresh serial ports ──────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let ports = ports_list.clone();
        app.on_refresh_serial_ports(move || {
            refresh_serial_ports(&weak.unwrap(), &ports);
        });
    }

    // ── Refresh probes ────────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let probes = probes_list.clone();
        app.on_refresh_probes(move || {
            refresh_probes(&weak.unwrap(), &probes);
        });
    }

    // ── Connect device ────────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let ports = ports_list.clone();
        let probes = probes_list.clone();
        let rt = runtime.clone();
        let stop = stop_adc.clone();
        let frx_slot = fast_rx_slot.clone();
        let srx_slot = slow_rx_slot.clone();
        let conn_flag = connected_flag.clone();
        let fhz = fast_hz.clone();

        app.on_connect_device(move || {
            let app = weak.unwrap();
            stop.store(false, Ordering::Relaxed);

            let mode = app.get_transport_mode();
            let config = match mode {
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
                _ => return,
            };

            // Set fast_hz so the backend enables telemetry at connect time
            let rates = [100u16, 500, 1000, 2000, 5000, 10000, 20000];
            let idx = app.get_stream_rate_index() as usize;
            let hz = rates.get(idx).copied().unwrap_or(1000);
            let config = HostConfig {
                fast_hz: Some(hz),
                ..config
            };

            app.set_error_text("".into());

            let host_runtime = start_host(config);
            let fast_rx = host_runtime.fast_rx.clone();
            let slow_rx = host_runtime.slow_rx.clone();
            let info_rx = host_runtime.device_info_rx.clone();
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

            // Device info listener — runs once on connection
            let weak_info = weak.clone();
            thread::spawn(move || {
                if let Ok(info) = info_rx.recv() {
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
        });
    }

    // ── Motor start ───────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_start(move || {
            let app = weak.unwrap();
            let duty = app.get_duty();
            let iq_target = duty * 0.1;
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor start: duty={duty:.0}%, iq_target={iq_target:.2}A");
                match runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::CurrentControl {
                        iq_target,
                        id_target: 0.0,
                    })) {
                    Ok(()) => tracing::debug!("Motor command sent"),
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
        app.on_motor_stop(move || {
            let guard = rt.lock().unwrap();
            if let Some(ref runtime) = *guard {
                tracing::info!("Motor stop");
                match runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::Stopped))
                {
                    Ok(()) => tracing::debug!("Stop command sent"),
                    Err(e) => tracing::error!("Failed to send stop command: {e}"),
                }
            } else {
                tracing::warn!("Motor stop clicked but no runtime");
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
                let rates = [100u16, 500, 1000, 2000, 5000, 10000, 20000];
                let idx = app.get_stream_rate_index() as usize;
                let hz = rates.get(idx).copied().unwrap_or(1000);
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

            let write = match group_idx {
                0 => {
                    let r: f32 = app.get_cfg_resistance().parse().unwrap_or(0.0);
                    let ld: f32 = app.get_cfg_inductance_d().parse().unwrap_or(0.0);
                    let lq: f32 = app.get_cfg_inductance_q().parse().unwrap_or(0.0);
                    let fl: f32 = app.get_cfg_flux_linkage().parse().unwrap_or(0.0);
                    let pp: u8 = app.get_cfg_pole_pairs().parse().unwrap_or(1);
                    ConfigWrite::MotorParams(MotorParamsConfig {
                        resistance_ohm: r,
                        inductance_d_h: ld,
                        inductance_q_h: lq,
                        flux_linkage_wb: fl,
                        pole_pairs: pp,
                    })
                }
                1 => {
                    let iq: f32 = app.get_cfg_max_iq().parse().unwrap_or(0.0);
                    let ph: f32 = app.get_cfg_max_phase_current().parse().unwrap_or(0.0);
                    ConfigWrite::CurrentLimits(CurrentLimitsConfig {
                        max_iq_a: iq,
                        max_phase_current_a: ph,
                    })
                }
                2 => {
                    let min: u32 = app.get_cfg_min_vbus().parse().unwrap_or(0);
                    let max: u32 = app.get_cfg_max_vbus().parse().unwrap_or(0);
                    ConfigWrite::VoltageLimits(VoltageLimitsConfig {
                        min_vbus_mv: min,
                        max_vbus_mv: max,
                    })
                }
                3 => {
                    let kp: f32 = app.get_cfg_kp().parse().unwrap_or(0.0);
                    let ki: f32 = app.get_cfg_ki().parse().unwrap_or(0.0);
                    let bw: f32 = app.get_cfg_bandwidth().parse().unwrap_or(0.0);
                    ConfigWrite::PiGains(PiGainsConfig {
                        kp,
                        ki,
                        bandwidth_rad_s: bw,
                    })
                }
                _ => return,
            };

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
            let max_loss: f32 = app.get_detect_max_loss().parse().unwrap_or(120.0);
            let openloop_erpm: f32 = app.get_detect_openloop_erpm().parse().unwrap_or(700.0);
            let sensorless_erpm: f32 = app.get_detect_sensorless_erpm().parse().unwrap_or(4000.0);

            let req = oxifoc_core::types::DetectRequest {
                pole_pairs,
                max_power_loss_w: max_loss,
                openloop_erpm,
                sensorless_erpm,
            };

            app.set_detect_status(SharedString::from("Running..."));

            let (tx, rx) = oxifoc_host_lib::detect_channel();
            if runtime.cmd_tx.send(HostCommand::Detect(req, tx)).is_err() {
                return;
            }

            thread::spawn(move || {
                let result = rx.blocking_recv();
                let _ = weak.upgrade_in_event_loop(move |app| {
                    use oxifoc_core::types::DetectResponse;
                    match result {
                        Ok(Ok(DetectResponse::Ok {
                            resistance_ohm,
                            inductance_d_h,
                            inductance_q_h,
                            flux_linkage_wb,
                            kv_rpm_per_v,
                            max_current_a,
                            kp_current,
                            ki_current,
                        })) => {
                            app.set_detect_resistance(SharedString::from(format!(
                                "{resistance_ohm:.4}"
                            )));
                            app.set_detect_inductance_d(SharedString::from(format!(
                                "{inductance_d_h:.6}"
                            )));
                            app.set_detect_inductance_q(SharedString::from(format!(
                                "{inductance_q_h:.6}"
                            )));
                            app.set_detect_flux_linkage(SharedString::from(format!(
                                "{flux_linkage_wb:.6}"
                            )));
                            app.set_detect_kv(SharedString::from(format!("{kv_rpm_per_v:.1}")));
                            app.set_detect_max_current(SharedString::from(format!(
                                "{max_current_a:.2}"
                            )));
                            app.set_detect_kp(SharedString::from(format!("{kp_current:.4}")));
                            app.set_detect_ki(SharedString::from(format!("{ki_current:.2}")));
                            app.set_detect_status(SharedString::from("OK"));
                        }
                        Ok(Ok(DetectResponse::Error(e))) => {
                            app.set_detect_status(SharedString::from(format!("Error: {e:?}")));
                        }
                        Ok(Err(e)) => {
                            app.set_detect_status(SharedString::from(format!("Error: {e}")));
                        }
                        Err(_) => {
                            app.set_detect_status(SharedString::from("No response"));
                        }
                    }
                });
            });
        });
    }

    // ── Detect apply to config ──────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_detect_apply(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();

            use oxifoc_core::storage::{MotorParamsConfig, PiGainsConfig};
            use oxifoc_core::types::ConfigWrite;

            let r: f32 = app.get_detect_resistance().parse().unwrap_or(0.0);
            let ld: f32 = app.get_detect_inductance_d().parse().unwrap_or(0.0);
            let lq: f32 = app.get_detect_inductance_q().parse().unwrap_or(0.0);
            let fl: f32 = app.get_detect_flux_linkage().parse().unwrap_or(0.0);
            let pp = app.get_pole_pairs().max(1) as u8;
            let kp: f32 = app.get_detect_kp().parse().unwrap_or(0.0);
            let ki: f32 = app.get_detect_ki().parse().unwrap_or(0.0);

            // Write motor params
            let (tx1, rx1) = config_channel();
            let _ = runtime.cmd_tx.send(HostCommand::ConfigWrite(
                ConfigWrite::MotorParams(MotorParamsConfig {
                    resistance_ohm: r,
                    inductance_d_h: ld,
                    inductance_q_h: lq,
                    flux_linkage_wb: fl,
                    pole_pairs: pp,
                }),
                tx1,
            ));

            // Write PI gains
            let (tx2, rx2) = config_channel();
            let _ = runtime.cmd_tx.send(HostCommand::ConfigWrite(
                ConfigWrite::PiGains(PiGainsConfig {
                    kp,
                    ki,
                    bandwidth_rad_s: 0.0,
                }),
                tx2,
            ));

            thread::spawn(move || {
                let r1 = rx1.blocking_recv();
                let r2 = rx2.blocking_recv();
                let _ = weak.upgrade_in_event_loop(move |app| {
                    if r1.is_ok() && r2.is_ok() {
                        app.set_detect_status(SharedString::from("Applied to config"));
                    } else {
                        app.set_detect_status(SharedString::from("Failed to apply"));
                    }
                });
            });
        });
    }

    app.run().unwrap();

    stop_adc.store(true, Ordering::Relaxed);
    if let Some(rt) = runtime.lock().unwrap().take() {
        rt.shutdown();
    }
}

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
