slint::include_modules!();

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::RecvTimeoutError;
use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::pi_controller::PIController;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::types::{AdcSample, ControlMode};
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ProbeInfo, SerialPortInfo, TransportType, init_tracing,
    list_probes, list_serial_ports, start_host,
};
use slint::wgpu_28::WGPUConfiguration;
use slint::{GraphicsAPI, Image, ModelRc, RenderingState, SharedString, StandardListViewItem,
            VecModel};
use slint_wgpu_plot::{PlotBuffer, PlotConfig, PlotRenderer, required_wgpu_settings};

const CAPACITY: usize = 32768;
const UI_UPDATE_HZ: u64 = 30;
const BAUD_RATES: [u32; 6] = [115200, 230400, 460800, 921600, 1_000_000, 2_000_000];

/// Default motor parameters for simulation (small hobby BLDC, 24 V, ~100 W).
const SIM_PARAMS: MotorParams = MotorParams {
    r: 0.5,
    ld: 5e-4,
    lq: 5e-4,
    lambda: 0.01,
    pole_pairs: 7,
    j: 1e-4,
    friction_b: 1e-4,
};
const SIM_VBUS: f32 = 24.0;

fn main() {
    init_tracing();

    // Configure the WGPU backend (required for GPU chart rendering).
    // The largest chart has 3 channels (phase currents).
    let wgpu_settings = required_wgpu_settings(CAPACITY, 3);
    slint::BackendSelector::new()
        .require_wgpu_28(WGPUConfiguration::Automatic(wgpu_settings))
        .select()
        .expect("Failed to initialise WGPU backend");

    let app = App::new().unwrap();

    // ── Shared state ──────────────────────────────────────────────────────────
    let ports_list: Arc<std::sync::Mutex<Vec<SerialPortInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let probes_list: Arc<std::sync::Mutex<Vec<ProbeInfo>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    let runtime: Arc<std::sync::Mutex<Option<HostRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));
    let stop_adc = Arc::new(AtomicBool::new(false));

    // Simulation-specific state
    let sim_mode = Arc::new(AtomicBool::new(false));
    let sim_control: Arc<std::sync::Mutex<ControlMode>> =
        Arc::new(std::sync::Mutex::new(ControlMode::Stopped));

    // ── Ring buffers shared between data thread and render notifier ───────────
    let currents_buf = Arc::new(PlotBuffer::new(3, CAPACITY)); // ia, ib, ic
    let vbus_buf = Arc::new(PlotBuffer::new(1, CAPACITY));     // V
    let temp_buf = Arc::new(PlotBuffer::new(1, CAPACITY));     // °C

    refresh_serial_ports(&app, &ports_list);
    refresh_probes(&app, &probes_list);

    // ── Rendering notifier: creates PlotRenderers on setup, renders on every frame ──
    {
        let app_weak = app.as_weak();
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();
        let sim = sim_mode.clone();

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
                    if let (Some(app), Some(cr), Some(vr), Some(tr)) = (
                        app_weak.upgrade(),
                        cr.as_mut(),
                        vr.as_mut(),
                        tr.as_mut(),
                    ) {
                        let vis = CAPACITY as u32;

                        // Adjust currents chart y-range based on mode
                        if sim.load(Ordering::Relaxed) {
                            cr.set_y_range(-20.0, 20.0); // Amps
                        } else {
                            cr.set_y_range(0.0, 4095.0); // ADC counts
                        }

                        let tex = cr.render(
                            &cb,
                            app.get_currents_w() as u32,
                            app.get_currents_h() as u32,
                            vis,
                        );
                        app.set_currents_texture(Image::try_from(tex).unwrap());

                        let tex = vr.render(
                            &vb,
                            app.get_vbus_w() as u32,
                            app.get_vbus_h() as u32,
                            vis,
                        );
                        app.set_vbus_texture(Image::try_from(tex).unwrap());

                        let tex = tr.render(
                            &tb,
                            app.get_temp_w() as u32,
                            app.get_temp_h() as u32,
                            vis,
                        );
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
        let sim = sim_mode.clone();
        let sim_ctl = sim_control.clone();
        let cb = currents_buf.clone();
        let vb = vbus_buf.clone();
        let tb = temp_buf.clone();

        app.on_connect_device(move || {
            let app = weak.unwrap();
            stop.store(false, Ordering::Relaxed);

            if app.get_use_simulate() {
                // ── Simulation mode ────────────────────────────────────────
                sim.store(true, Ordering::Relaxed);
                *sim_ctl.lock().unwrap() = ControlMode::Stopped;

                let weak2 = weak.clone();
                let stop2 = stop.clone();
                let ctl2 = sim_ctl.clone();
                let cb2 = cb.clone();
                let vb2 = vb.clone();
                let tb2 = tb.clone();
                thread::spawn(move || {
                    sim_loop(weak2, ctl2, stop2, cb2, vb2, tb2);
                });

                app.set_error_text("".into());
                app.set_page("main".into());
                return;
            }

            // ── Hardware mode ──────────────────────────────────────────────
            sim.store(false, Ordering::Relaxed);

            let config = if app.get_use_serial() {
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
                    probe: None,
                    chip: None,
                    elf: None,
                    stream_defmt: Some(true),
                    stream_ergot: Some(true),
                }
            } else {
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
                    serial_path: None,
                    serial_baud: None,
                    probe: Some(probe.identifier.clone()),
                    chip: Some(chip),
                    elf: None,
                    stream_defmt: Some(true),
                    stream_ergot: Some(true),
                }
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
        let sim = sim_mode.clone();
        app.on_disconnect_device(move || {
            stop.store(true, Ordering::Relaxed);
            sim.store(false, Ordering::Relaxed);
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
        let sim = sim_mode.clone();
        let sim_ctl = sim_control.clone();
        app.on_motor_start(move || {
            let app = weak.unwrap();
            let duty = app.get_duty();
            let iq_target = duty * 0.1;
            if sim.load(Ordering::Relaxed) {
                *sim_ctl.lock().unwrap() =
                    ControlMode::CurrentControl { iq_target, id_target: 0.0 };
            } else if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime.cmd_tx.send(HostCommand::Motor(
                    ControlMode::CurrentControl { iq_target, id_target: 0.0 },
                ));
            }
        });
    }

    // ── Motor stop ────────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let sim = sim_mode.clone();
        let sim_ctl = sim_control.clone();
        app.on_motor_stop(move || {
            if sim.load(Ordering::Relaxed) {
                *sim_ctl.lock().unwrap() = ControlMode::Stopped;
            } else if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime.cmd_tx.send(HostCommand::Motor(ControlMode::Stopped));
            }
        });
    }

    app.run().unwrap();

    stop_adc.store(true, Ordering::Relaxed);
    sim_mode.store(false, Ordering::Relaxed);
    if let Some(rt) = runtime.lock().unwrap().take() {
        rt.shutdown();
    }
}

fn refresh_serial_ports(
    app: &App,
    ports: &Arc<std::sync::Mutex<Vec<SerialPortInfo>>>,
) {
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
    let items: Vec<StandardListViewItem> =
        filtered.iter().map(|p| SharedString::from(p.to_string()).into()).collect();
    *ports.lock().unwrap() = filtered;
    app.set_serial_ports(ModelRc::new(VecModel::from(items)));
    app.set_selected_serial(-1);
}

fn refresh_probes(app: &App, probes: &Arc<std::sync::Mutex<Vec<ProbeInfo>>>) {
    let all_probes = list_probes();
    let items: Vec<StandardListViewItem> =
        all_probes.iter().map(|p| SharedString::from(p.to_string()).into()).collect();
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
                currents_buf.push_frame(&[
                    sample.ia as f32,
                    sample.ib as f32,
                    sample.ic as f32,
                ]);
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
                        app.set_vbus_text(
                            format!("{:.2} V", s.vbus_mv as f32 / 1000.0).into(),
                        );
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

/// Simulation loop: runs FocController + VirtualMotor at ~20 kHz.
///
/// Pushes physical currents (A) directly into `currents_buf` so the chart
/// shows Amps on the y-axis (−20 … 20 A range set in BeforeRendering).
fn sim_loop(
    weak: slint::Weak<App>,
    control: Arc<std::sync::Mutex<ControlMode>>,
    stop: Arc<AtomicBool>,
    currents_buf: Arc<PlotBuffer>,
    vbus_buf: Arc<PlotBuffer>,
    temp_buf: Arc<PlotBuffer>,
) {
    const DT: f32 = 1.0 / 20_000.0;
    const MAX_DUTY: u16 = 1000;
    // Run 100 steps then sleep ~5 ms to approximate 20 kHz wall-clock time.
    const BATCH: usize = 100;

    // Current-loop bandwidth ≈ 1000 rad/s → Kp = L·ω, Ki = R·ω
    let kp = SIM_PARAMS.ld * 1000.0;
    let ki = SIM_PARAMS.r * 1000.0;
    let v_lim = SIM_VBUS;

    let mut foc = FocController::<SvpwmModulator>::new(SIM_VBUS);
    foc.id_pi = PIController::new(kp, ki).with_limits(-v_lim, v_lim);
    foc.iq_pi = PIController::new(kp, ki).with_limits(-v_lim, v_lim);

    let mut motor = VirtualMotor::new(SIM_PARAMS);
    let mut out = VirtualMotorOutput::default();

    let mut seq: u32 = 0;
    let mut last_ui_update = Instant::now();
    let ui_interval = Duration::from_millis(1000 / UI_UPDATE_HZ);

    while !stop.load(Ordering::Relaxed) {
        let mode = *control.lock().unwrap();
        let (id_target, iq_target) = match mode {
            ControlMode::Stopped => {
                foc.reset();
                (0.0, 0.0)
            }
            ControlMode::CurrentControl { iq_target, id_target } => (id_target, iq_target),
            _ => (0.0, 0.0),
        };

        for _ in 0..BATCH {
            let telem = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle_rad,
                id_target,
                iq_target,
                MAX_DUTY,
                DT,
            );
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

            currents_buf.push_frame(&[out.ia, out.ib, out.ic]);
            vbus_buf.push_frame(&[SIM_VBUS]);
            temp_buf.push_frame(&[25.0]);
            seq += 1;
        }

        // Throttle text telemetry to UI_UPDATE_HZ.
        if last_ui_update.elapsed() >= ui_interval {
            last_ui_update = Instant::now();
            let ia = out.ia;
            let ib = out.ib;
            let ic = out.ic;
            let omega = out.omega_e;
            let s = seq;
            let _ = weak.upgrade_in_event_loop(move |app| {
                app.set_is_connected(true);
                app.set_ia_text(format!("{:.3} A", ia).into());
                app.set_ib_text(format!("{:.3} A", ib).into());
                app.set_ic_text(format!("{:.3} A", ic).into());
                app.set_vbus_text(format!("{:.1} V", SIM_VBUS).into());
                app.set_temp_text("25.0 °C".into());
                // Show electrical RPM in seq field for simulation
                let erpm = omega * 60.0 / (2.0 * core::f32::consts::PI);
                app.set_seq_text(format!("{:.0} eRPM", erpm).into());
                let _ = s; // used for sequencing, not displayed in sim
            });
        }

        // Sleep to approximate 20 kHz: BATCH steps × 50 µs = 5 ms.
        thread::sleep(Duration::from_millis(5));
    }
}
