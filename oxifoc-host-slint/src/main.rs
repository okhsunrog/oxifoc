slint::include_modules!();

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::RecvTimeoutError;
use oxifoc_core::types::{AdcSample, ControlMode};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ProbeInfo, SerialPortInfo, TransportType, list_probes,
    list_serial_ports, start_host,
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
const UI_UPDATE_HZ: u64 = 30;
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
    let log_model = std::rc::Rc::new(VecModel::<LogMessage>::default());
    app.set_log_messages(ModelRc::from(log_model.clone()));

    {
        let weak = app.as_weak();
        thread::spawn(move || {
            while let Ok((text, level)) = log_rx.recv() {
                let text = SharedString::from(&text);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    let model = app.get_log_messages();
                    model
                        .as_any()
                        .downcast_ref::<VecModel<LogMessage>>()
                        .unwrap()
                        .push(LogMessage { text, level });
                    // Trim old messages to prevent unbounded growth
                    while model.row_count() > MAX_LOG_LINES {
                        model
                            .as_any()
                            .downcast_ref::<VecModel<LogMessage>>()
                            .unwrap()
                            .remove(0);
                    }
                });
            }
        });
    }

    {
        let model = log_model.clone();
        app.on_clear_log(move || {
            while model.row_count() > 0 {
                model.remove(0);
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

    refresh_serial_ports(&app, &ports_list);
    refresh_probes(&app, &probes_list);

    // ── Rendering notifier: creates PlotRenderers on setup, renders on every frame ──
    {
        let app_weak = app.as_weak();
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();

        let mut cr: Option<PlotRenderer> = None;
        let mut vr: Option<PlotRenderer> = None;
        let mut tr: Option<PlotRenderer> = None;

        app.window()
            .set_rendering_notifier(move |state, graphics_api| match state {
                RenderingState::RenderingSetup => {
                    if let GraphicsAPI::WGPU28 { device, queue, .. } = graphics_api {
                        cr = Some(PlotRenderer::new(
                            device,
                            queue,
                            PlotConfig {
                                num_channels: 3,
                                capacity: CAPACITY,
                                y_min: 0.0,
                                y_max: 4095.0,
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
                                channel_colors: vec![[0.937, 0.267, 0.267, 1.0]], // red
                            },
                        ));
                    }
                }
                RenderingState::BeforeRendering => {
                    if let (Some(app), Some(cr), Some(vr), Some(tr)) =
                        (app_weak.upgrade(), cr.as_mut(), vr.as_mut(), tr.as_mut())
                    {
                        let vis = CAPACITY as u32;

                        let tex = cr.render(
                            &cb,
                            app.get_currents_w() as u32,
                            app.get_currents_h() as u32,
                            vis,
                        );
                        app.set_currents_texture(Image::try_from(tex).unwrap());

                        let tex =
                            vr.render(&vb, app.get_vbus_w() as u32, app.get_vbus_h() as u32, vis);
                        app.set_vbus_texture(Image::try_from(tex).unwrap());

                        let tex =
                            tr.render(&tb, app.get_temp_w() as u32, app.get_temp_h() as u32, vis);
                        app.set_temp_texture(Image::try_from(tex).unwrap());

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
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();

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

            app.set_error_text("".into());

            let host_runtime = start_host(config);
            let adc_rx = host_runtime.adc_rx.clone();
            let connected = host_runtime.connected.clone();
            *rt.lock().unwrap() = Some(host_runtime);

            let weak2 = weak.clone();
            let stop2 = stop.clone();
            let cb2 = cb.clone();
            let vb2 = vb.clone();
            let tb2 = tb.clone();
            thread::spawn(move || {
                adc_poll_loop(weak2, adc_rx, connected, stop2, cb2, vb2, tb2);
            });

            app.set_page("main".into());
        });
    }

    // ── Disconnect device ─────────────────────────────────────────────────────
    {
        let weak = app.as_weak();
        let rt = runtime.clone();
        let stop = stop_adc.clone();
        app.on_disconnect_device(move || {
            stop.store(true, Ordering::Relaxed);
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
            if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::CurrentControl {
                        iq_target,
                        id_target: 0.0,
                    }));
            }
        });
    }

    // ── Motor stop ────────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        app.on_motor_stop(move || {
            if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime
                    .cmd_tx
                    .send(HostCommand::Motor(ControlMode::Stopped));
            }
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

fn adc_poll_loop(
    weak: slint::Weak<App>,
    adc_rx: crossbeam_channel::Receiver<AdcSample>,
    connected: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    currents_buf: Arc<PlotBuffer>,
    vbus_buf: Arc<PlotBuffer>,
    temp_buf: Arc<PlotBuffer>,
) {
    let mut last_ui_update = Instant::now();
    let ui_interval = Duration::from_millis(1000 / UI_UPDATE_HZ);

    while !stop.load(Ordering::Relaxed) {
        match adc_rx.recv_timeout(Duration::from_millis(50)) {
            Ok(sample) => {
                // Push to ring buffers on every sample — no rate limiting.
                currents_buf.push_frame(&[sample.ia as f32, sample.ib as f32, sample.ic as f32]);
                vbus_buf.push_frame(&[sample.vbus_mv as f32 / 1000.0]);
                temp_buf.push_frame(&[sample.fet_temp_c_x10 as f32 / 10.0]);

                // Throttle the text telemetry updates to UI_UPDATE_HZ.
                if last_ui_update.elapsed() >= ui_interval {
                    last_ui_update = Instant::now();
                    let is_conn = connected.load(Ordering::Relaxed);
                    let s = sample;
                    let _ = weak.upgrade_in_event_loop(move |app| {
                        app.set_is_connected(is_conn);
                        app.set_ia_text(format!("{}", s.ia).into());
                        app.set_ib_text(format!("{}", s.ib).into());
                        app.set_ic_text(format!("{}", s.ic).into());
                        app.set_vbus_text(format!("{:.2} V", s.vbus_mv as f32 / 1000.0).into());
                        app.set_temp_text(
                            format!("{:.1} °C", s.fet_temp_c_x10 as f32 / 10.0).into(),
                        );
                        app.set_seq_text(format!("{}", s.seq).into());
                    });
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                let is_conn = connected.load(Ordering::Relaxed);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    app.set_is_connected(is_conn);
                });
            }
            Err(RecvTimeoutError::Disconnected) => break,
        }
    }
}
