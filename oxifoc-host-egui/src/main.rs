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
    show_usb_only: bool,
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
            show_usb_only: true,
        }
    }
}

impl ConnectionState {
    /// Filter serial ports to show only USB-Serial devices (ttyACM*, ttyUSB*, ttyAMA*)
    fn filtered_serial_ports(&self) -> Vec<(usize, SerialPortInfo)> {
        self.serial_ports
            .iter()
            .enumerate()
            .filter(|(_, port)| {
                if !self.show_usb_only {
                    return true;
                }
                let path = port.path.to_lowercase();
                path.contains("ttyacm") || path.contains("ttyusb") || path.contains("ttyama")
            })
            .map(|(idx, port)| (idx, port.clone()))
            .collect()
    }
}

/// Connected state
struct ConnectedState {
    runtime: HostRuntime,
    duty: f32,
    samples: VecDeque<AdcSample>,
    max_samples: usize,
    window_samples: usize,
    show_numeric: bool,
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
                        ui.checkbox(&mut conn.show_usb_only, "USB-Serial only");
                        if ui.button("↻ Refresh").clicked() {
                            conn.serial_ports = list_serial_ports();
                        }
                    });

                    ui.add_space(5.0);

                    let filtered = conn.filtered_serial_ports();

                    if conn.serial_ports.is_empty() {
                        ui.label("No serial ports found");
                    } else if filtered.is_empty() {
                        ui.label("No USB-Serial devices found");
                    } else {
                        egui::ScrollArea::vertical()
                            .max_height(200.0)
                            .show(ui, |ui| {
                                for (idx, port) in &filtered {
                                    let is_selected = conn.selected_serial == Some(*idx);
                                    let response =
                                        ui.selectable_label(is_selected, port.to_string());
                                    if response.clicked() {
                                        conn.selected_serial = Some(*idx);
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
                samples: VecDeque::with_capacity(2000),
                max_samples: 2000,
                window_samples: 120, // ~2 seconds at 60Hz
                show_numeric: true,
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

            ui.horizontal(|ui| {
                ui.label(format!("Samples: {}", connected.samples.len()));
                ui.separator();
                ui.label("View:");
                ui.checkbox(&mut connected.show_numeric, "Numeric");
                ui.separator();
                ui.label("Window:");
                ui.add(
                    egui::Slider::new(&mut connected.window_samples, 30..=1000).suffix(" samples"),
                );
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // Numeric display
                if connected.show_numeric
                    && let Some(last) = connected.samples.back()
                {
                    ui.group(|ui| {
                        ui.heading("Real-Time Telemetry");
                        ui.add_space(10.0);

                        // Phase Currents (raw ADC)
                        ui.label(egui::RichText::new("Phase Currents (Raw ADC)").strong());
                        ui.horizontal(|ui| {
                            ui.label(format!("Phase A: {}", last.ia));
                            ui.separator();
                            ui.label(format!("Phase B: {}", last.ib));
                            ui.separator();
                            ui.label(format!("Phase C: {}", last.ic));
                        });
                        ui.add_space(10.0);

                        // Voltage & Temperature
                        ui.label(egui::RichText::new("Voltage & Temperature").strong());
                        ui.horizontal(|ui| {
                            ui.label(format!("DC Bus: {:.2} V", last.vbus_mv as f32 / 1000.0));
                            ui.separator();
                            ui.label(format!(
                                "FET Temp: {:.1} °C",
                                last.fet_temp_c_x10 as f32 / 10.0
                            ));
                        });
                        ui.add_space(10.0);

                        // Sequence
                        ui.label(format!("Sequence: {}", last.seq));
                    });
                    ui.add_space(10.0);
                }

                // Get windowed samples
                let window_start = connected
                    .samples
                    .len()
                    .saturating_sub(connected.window_samples);
                let windowed: Vec<&AdcSample> =
                    connected.samples.iter().skip(window_start).collect();

                // Phase Currents Plot
                ui.group(|ui| {
                    ui.heading("Phase Currents");
                    let points_ia: PlotPoints = windowed
                        .iter()
                        .map(|s| [s.seq as f64, s.ia as f64])
                        .collect();
                    let points_ib: PlotPoints = windowed
                        .iter()
                        .map(|s| [s.seq as f64, s.ib as f64])
                        .collect();
                    let points_ic: PlotPoints = windowed
                        .iter()
                        .map(|s| [s.seq as f64, s.ic as f64])
                        .collect();

                    Plot::new("phase_currents_plot")
                        .height(200.0)
                        .legend(egui_plot::Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new("Phase A", points_ia)
                                    .color(egui::Color32::from_rgb(34, 211, 238)),
                            );
                            plot_ui.line(
                                Line::new("Phase B", points_ib)
                                    .color(egui::Color32::from_rgb(139, 92, 246)),
                            );
                            plot_ui.line(
                                Line::new("Phase C", points_ic)
                                    .color(egui::Color32::from_rgb(249, 115, 22)),
                            );
                        });
                });
                ui.add_space(10.0);

                // Voltage & Temperature Plot
                ui.group(|ui| {
                    ui.heading("Voltage & Temperature");
                    let points_vbus: PlotPoints = windowed
                        .iter()
                        .map(|s| [s.seq as f64, s.vbus_mv as f64 / 1000.0])
                        .collect();
                    let points_temp: PlotPoints = windowed
                        .iter()
                        .map(|s| [s.seq as f64, s.fet_temp_c_x10 as f64 / 10.0])
                        .collect();

                    Plot::new("voltage_temp_plot")
                        .height(200.0)
                        .legend(egui_plot::Legend::default())
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new("Voltage (V)", points_vbus)
                                    .color(egui::Color32::from_rgb(234, 179, 8)),
                            );
                            plot_ui.line(
                                Line::new("Temperature (°C)", points_temp)
                                    .color(egui::Color32::from_rgb(239, 68, 68)),
                            );
                        });
                });
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
