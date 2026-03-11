slint::include_modules!();

use std::collections::VecDeque;
use std::fmt::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::RecvTimeoutError;
use oxifoc_core::types::{AdcSample, ControlMode};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ProbeInfo, SerialPortInfo, TransportType, init_tracing,
    list_probes, list_serial_ports, start_host,
};
use slint::{ModelRc, SharedString, StandardListViewItem, VecModel};

const MAX_SAMPLES: usize = 500;
const UI_UPDATE_HZ: u64 = 30;
const BAUD_RATES: [u32; 6] = [115200, 230400, 460800, 921600, 1_000_000, 2_000_000];

fn main() {
    init_tracing();

    let app = App::new().unwrap();

    // Shared state
    let ports_list: Arc<Mutex<Vec<SerialPortInfo>>> = Arc::new(Mutex::new(Vec::new()));
    let probes_list: Arc<Mutex<Vec<ProbeInfo>>> = Arc::new(Mutex::new(Vec::new()));
    let runtime: Arc<Mutex<Option<HostRuntime>>> = Arc::new(Mutex::new(None));
    let stop_adc = Arc::new(AtomicBool::new(false));

    // Populate initial lists
    refresh_serial_ports(&app, &ports_list);
    refresh_probes(&app, &probes_list);

    // ── Refresh serial ports ──
    {
        let weak = app.as_weak();
        let ports = ports_list.clone();
        app.on_refresh_serial_ports(move || {
            let app = weak.unwrap();
            refresh_serial_ports(&app, &ports);
        });
    }

    // ── Refresh probes ──
    {
        let weak = app.as_weak();
        let probes = probes_list.clone();
        app.on_refresh_probes(move || {
            let app = weak.unwrap();
            refresh_probes(&app, &probes);
        });
    }

    // ── Connect device ──
    {
        let weak = app.as_weak();
        let ports = ports_list.clone();
        let probes = probes_list.clone();
        let rt = runtime.clone();
        let stop = stop_adc.clone();
        app.on_connect_device(move || {
            let app = weak.unwrap();

            let config = if app.get_use_serial() {
                let idx = app.get_selected_serial();
                let ports_guard = ports.lock().unwrap();
                if idx < 0 || idx as usize >= ports_guard.len() {
                    app.set_error_text("No serial port selected".into());
                    return;
                }
                let port = &ports_guard[idx as usize];
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
                let probes_guard = probes.lock().unwrap();
                if idx < 0 || idx as usize >= probes_guard.len() {
                    app.set_error_text("No probe selected".into());
                    return;
                }
                let probe = &probes_guard[idx as usize];
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
            stop.store(false, Ordering::Relaxed);

            // Start ADC polling thread
            let weak2 = weak.clone();
            let stop2 = stop.clone();
            thread::spawn(move || {
                adc_poll_loop(weak2, adc_rx, connected, stop2);
            });

            app.set_page("main".into());
        });
    }

    // ── Disconnect device ──
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

    // ── Motor start ──
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_motor_start(move || {
            let app = weak.unwrap();
            let duty = app.get_duty();
            let iq_target = duty * 0.1;
            if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime.cmd_tx.send(HostCommand::Motor(ControlMode::CurrentControl {
                    iq_target,
                    id_target: 0.0,
                }));
            }
        });
    }

    // ── Motor stop ──
    {
        let rt = runtime.clone();
        app.on_motor_stop(move || {
            if let Some(ref runtime) = *rt.lock().unwrap() {
                let _ = runtime.cmd_tx.send(HostCommand::Motor(ControlMode::Stopped));
            }
        });
    }

    app.run().unwrap();

    // Cleanup on exit
    stop_adc.store(true, Ordering::Relaxed);
    if let Some(rt) = runtime.lock().unwrap().take() {
        rt.shutdown();
    }
}

fn refresh_serial_ports(app: &App, ports: &Arc<Mutex<Vec<SerialPortInfo>>>) {
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

fn refresh_probes(app: &App, probes: &Arc<Mutex<Vec<ProbeInfo>>>) {
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
) {
    let mut samples: VecDeque<AdcSample> = VecDeque::with_capacity(MAX_SAMPLES);
    let mut last_update = Instant::now();
    let update_interval = Duration::from_millis(1000 / UI_UPDATE_HZ);

    while !stop.load(Ordering::Relaxed) {
        match adc_rx.recv_timeout(Duration::from_millis(50)) {
            Ok(sample) => {
                if samples.len() >= MAX_SAMPLES {
                    samples.pop_front();
                }
                samples.push_back(sample);

                if last_update.elapsed() >= update_interval {
                    last_update = Instant::now();

                    let is_conn = connected.load(Ordering::Relaxed);
                    let latest = samples.back().cloned();
                    let count = samples.len() as i32;

                    // Generate chart path commands
                    let ia_cmd = chart_commands(&samples, |s| s.ia as f64);
                    let ib_cmd = chart_commands(&samples, |s| s.ib as f64);
                    let ic_cmd = chart_commands(&samples, |s| s.ic as f64);
                    let vbus_cmd = chart_commands(&samples, |s| s.vbus_mv as f64 / 1000.0);
                    let temp_cmd = chart_commands(&samples, |s| s.fet_temp_c_x10 as f64 / 10.0);

                    let _ = weak.upgrade_in_event_loop(move |app| {
                        app.set_is_connected(is_conn);
                        app.set_sample_count(count);

                        if let Some(s) = latest {
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
                        }

                        app.set_chart_ia_cmd(SharedString::from(ia_cmd));
                        app.set_chart_ib_cmd(SharedString::from(ib_cmd));
                        app.set_chart_ic_cmd(SharedString::from(ic_cmd));
                        app.set_chart_vbus_cmd(SharedString::from(vbus_cmd));
                        app.set_chart_temp_cmd(SharedString::from(temp_cmd));
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

/// Generate SVG path commands for a line chart.
/// Maps data to a 1000x1000 viewbox with 50px margin top/bottom.
fn chart_commands<F>(samples: &VecDeque<AdcSample>, f: F) -> String
where
    F: Fn(&AdcSample) -> f64,
{
    if samples.len() < 2 {
        return String::new();
    }

    let values: Vec<f64> = samples.iter().map(&f).collect();
    let min = values.iter().copied().fold(f64::INFINITY, f64::min);
    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let range = max - min;

    let n = values.len();
    let mut cmd = String::with_capacity(n * 16);

    for (i, &v) in values.iter().enumerate() {
        let x = (i as f64 / (n - 1) as f64) * 1000.0;
        let y = if range < 0.001 {
            500.0 // flat line centered
        } else {
            950.0 - ((v - min) / range) * 900.0 // maps to 50..950
        };

        if i == 0 {
            let _ = write!(cmd, "M {} {}", x as i32, y as i32);
        } else {
            let _ = write!(cmd, " L {} {}", x as i32, y as i32);
        }
    }

    cmd
}
