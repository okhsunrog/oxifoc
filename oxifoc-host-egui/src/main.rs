use eframe::{App, egui};
use egui_plot::{Line, Plot, PlotPoints};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ProbeInfo, SerialPortInfo, TransportType, init_tracing,
    list_probes, list_serial_ports, start_host,
};
use oxifoc_protocol::{AdcSample, MotorCommand};
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::time::Duration;

/// Transport selection for connection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SelectedTransport {
    Serial,
    Rtt,
}

/// Connection screen state
struct ConnectionState {
    transport: SelectedTransport,
    serial_ports: Vec<SerialPortInfo>,
    debug_probes: Vec<ProbeInfo>,
    selected_serial: Option<usize>,
    selected_probe: Option<usize>,
    baud_rate: u32,
    chip: String,
    error: Option<String>,
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self {
            transport: SelectedTransport::Serial,
            serial_ports: list_serial_ports(),
            debug_probes: list_probes(),
            selected_serial: None,
            selected_probe: None,
            baud_rate: 921600,
            chip: "STM32G431CBUx".to_string(),
            error: None,
        }
    }
}

/// Connected state
struct ConnectedState {
    runtime: HostRuntime,
    duty: f32,
    samples: VecDeque<AdcSample>,
    max_samples: usize,
}

/// Application state machine
enum AppState {
    Connecting(ConnectionState),
    Connected(ConnectedState),
}

struct OxifocApp {
    state: AppState,
}

impl Default for OxifocApp {
    fn default() -> Self {
        Self {
            state: AppState::Connecting(ConnectionState::default()),
        }
    }
}

impl OxifocApp {
    fn show_connection_screen(&mut self, ctx: &egui::Context) {
        let conn = match &mut self.state {
            AppState::Connecting(c) => c,
            _ => return,
        };

        let mut connect_action: Option<HostConfig> = None;

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(40.0);
                ui.heading("Connect to Device");
                ui.add_space(20.0);
            });

            // Transport selection
            ui.horizontal(|ui| {
                ui.label("Transport:");
                ui.selectable_value(
                    &mut conn.transport,
                    SelectedTransport::Serial,
                    "Serial (UART)",
                );
                ui.selectable_value(
                    &mut conn.transport,
                    SelectedTransport::Rtt,
                    "RTT (Debug Probe)",
                );
            });

            ui.add_space(10.0);
            ui.separator();
            ui.add_space(10.0);

            // Show error if any
            if let Some(err) = &conn.error {
                ui.colored_label(egui::Color32::RED, err.as_str());
                ui.add_space(10.0);
            }

            match conn.transport {
                SelectedTransport::Serial => {
                    ui.horizontal(|ui| {
                        ui.heading("Serial Ports");
                        if ui.button("↻ Refresh").clicked() {
                            conn.serial_ports = list_serial_ports();
                        }
                    });

                    ui.add_space(5.0);

                    if conn.serial_ports.is_empty() {
                        ui.label("No serial ports found");
                    } else {
                        egui::ScrollArea::vertical()
                            .max_height(200.0)
                            .show(ui, |ui| {
                                for (idx, port) in conn.serial_ports.iter().enumerate() {
                                    let is_selected = conn.selected_serial == Some(idx);
                                    let response =
                                        ui.selectable_label(is_selected, port.to_string());
                                    if response.clicked() {
                                        conn.selected_serial = Some(idx);
                                    }
                                }
                            });
                    }

                    ui.add_space(10.0);

                    ui.horizontal(|ui| {
                        ui.label("Baud Rate:");
                        egui::ComboBox::from_id_salt("baud_rate")
                            .selected_text(format!("{}", conn.baud_rate))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut conn.baud_rate, 115200, "115200");
                                ui.selectable_value(&mut conn.baud_rate, 230400, "230400");
                                ui.selectable_value(&mut conn.baud_rate, 460800, "460800");
                                ui.selectable_value(&mut conn.baud_rate, 921600, "921600");
                                ui.selectable_value(&mut conn.baud_rate, 1000000, "1000000");
                                ui.selectable_value(&mut conn.baud_rate, 2000000, "2000000");
                            });
                    });
                }
                SelectedTransport::Rtt => {
                    ui.horizontal(|ui| {
                        ui.heading("Debug Probes");
                        if ui.button("↻ Refresh").clicked() {
                            conn.debug_probes = list_probes();
                        }
                    });

                    ui.add_space(5.0);

                    if conn.debug_probes.is_empty() {
                        ui.label("No debug probes found");
                    } else {
                        egui::ScrollArea::vertical()
                            .max_height(200.0)
                            .show(ui, |ui| {
                                for (idx, probe) in conn.debug_probes.iter().enumerate() {
                                    let is_selected = conn.selected_probe == Some(idx);
                                    let response =
                                        ui.selectable_label(is_selected, probe.to_string());
                                    if response.clicked() {
                                        conn.selected_probe = Some(idx);
                                    }
                                }
                            });
                    }

                    ui.add_space(10.0);

                    ui.horizontal(|ui| {
                        ui.label("Target Chip:");
                        ui.text_edit_singleline(&mut conn.chip);
                    });
                }
            }

            ui.add_space(20.0);

            // Connect button
            let can_connect = match conn.transport {
                SelectedTransport::Serial => conn.selected_serial.is_some(),
                SelectedTransport::Rtt => conn.selected_probe.is_some() && !conn.chip.is_empty(),
            };

            ui.horizontal(|ui| {
                if ui
                    .add_enabled(can_connect, egui::Button::new("Connect"))
                    .clicked()
                {
                    // Build config
                    let config = match conn.transport {
                        SelectedTransport::Serial => {
                            let port = &conn.serial_ports[conn.selected_serial.unwrap()];
                            HostConfig {
                                transport: Some(TransportType::Serial),
                                serial_path: Some(port.path.clone()),
                                serial_baud: Some(conn.baud_rate),
                                probe: None,
                                chip: None,
                                elf: None,
                                stream_defmt: Some(true),
                                stream_ergot: Some(true),
                            }
                        }
                        SelectedTransport::Rtt => {
                            let probe = &conn.debug_probes[conn.selected_probe.unwrap()];
                            HostConfig {
                                transport: Some(TransportType::Rtt),
                                serial_path: None,
                                serial_baud: None,
                                probe: Some(probe.identifier.clone()),
                                chip: Some(conn.chip.clone()),
                                elf: None,
                                stream_defmt: Some(true),
                                stream_ergot: Some(true),
                            }
                        }
                    };
                    connect_action = Some(config);
                }
            });
        });

        // Apply connect action after closure
        if let Some(config) = connect_action {
            let runtime = start_host(config);
            self.state = AppState::Connected(ConnectedState {
                runtime,
                duty: 10.0,
                samples: VecDeque::with_capacity(1024),
                max_samples: 1024,
            });
        }
    }

    fn show_main_ui(&mut self, ctx: &egui::Context) {
        let connected = match &mut self.state {
            AppState::Connected(c) => c,
            _ => return,
        };

        // Drain incoming samples
        while let Ok(sample) = connected.runtime.adc_rx.try_recv() {
            if connected.samples.len() >= connected.max_samples {
                connected.samples.pop_front();
            }
            connected.samples.push_back(sample);
        }

        let mut disconnect = false;

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let is_connected = connected.runtime.connected.load(Ordering::Relaxed);
                let status_text = if is_connected {
                    "Connected"
                } else {
                    "Connecting..."
                };
                let status_color = if is_connected {
                    egui::Color32::GREEN
                } else {
                    egui::Color32::YELLOW
                };
                ui.colored_label(status_color, status_text);

                ui.separator();

                // Disconnect button
                if ui.button("Disconnect").clicked() {
                    disconnect = true;
                }
            });

            ui.horizontal(|ui| {
                ui.label("Duty (%)");
                ui.add(
                    egui::Slider::new(&mut connected.duty, 0.0..=100.0)
                        .clamping(egui::SliderClamping::Always),
                );
                if ui.button("Start").clicked() {
                    let duty = connected.duty as u8;
                    let _ = connected
                        .runtime
                        .cmd_tx
                        .send(HostCommand::Motor(MotorCommand::Start { duty }));
                }
                if ui.button("Stop").clicked() {
                    let _ = connected
                        .runtime
                        .cmd_tx
                        .send(HostCommand::Motor(MotorCommand::Stop));
                }
            });

            if let Some(last) = connected.samples.back() {
                ui.label(format!("Vbus: {:.2} V", last.vbus_mv as f32 / 1000.0));
                ui.label(format!(
                    "FET temp: {:.1} °C",
                    last.fet_temp_c_x10 as f32 / 10.0
                ));
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let points_ia: PlotPoints = connected
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ia as f64])
                .collect();
            let points_ib: PlotPoints = connected
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ib as f64])
                .collect();
            let points_ic: PlotPoints = connected
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ic as f64])
                .collect();
            let points_vbus: PlotPoints = connected
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.vbus_mv as f64])
                .collect();
            let points_temp: PlotPoints = connected
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.fet_temp_c_x10 as f64 / 10.0])
                .collect();

            Plot::new("adc_plot")
                .legend(egui_plot::Legend::default())
                .show(ui, |plot_ui| {
                    plot_ui.line(Line::new("ia", points_ia));
                    plot_ui.line(Line::new("ib", points_ib));
                    plot_ui.line(Line::new("ic", points_ic));
                    plot_ui.line(Line::new("vbus_mv", points_vbus));
                    plot_ui.line(Line::new("fet_temp_c", points_temp));
                });
        });

        ctx.request_repaint_after(Duration::from_millis(16));

        // Apply disconnect action after closure
        if disconnect {
            self.state = AppState::Connecting(ConnectionState::default());
        }
    }
}

impl App for OxifocApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        match &self.state {
            AppState::Connecting(_) => self.show_connection_screen(ctx),
            AppState::Connected(_) => self.show_main_ui(ctx),
        }
    }
}

fn main() -> eframe::Result<()> {
    init_tracing();

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Oxifoc Host",
        native_options,
        Box::new(move |_cc| Ok(Box::new(OxifocApp::default()))),
    )
}
