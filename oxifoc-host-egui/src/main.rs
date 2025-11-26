use crossbeam_channel::{Receiver, Sender};
use eframe::{App, egui};
use egui_plot::{Line, Plot, PlotPoints};
use oxifoc_host_lib::{HostCommand, HostConfig, HostRuntime, init_tracing, start_host};
use oxifoc_protocol::{AdcSample, MotorCommand};
use std::collections::VecDeque;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;

struct OxifocApp {
    connected: Arc<AtomicBool>,
    adc_rx: Receiver<AdcSample>,
    cmd_tx: Sender<HostCommand>,
    duty: f32,
    samples: VecDeque<AdcSample>,
    max_samples: usize,
}

impl OxifocApp {
    fn new(runtime: HostRuntime) -> Self {
        Self {
            connected: runtime.connected,
            adc_rx: runtime.adc_rx,
            cmd_tx: runtime.cmd_tx,
            duty: 10.0,
            samples: VecDeque::with_capacity(1024),
            max_samples: 1024,
        }
    }
}

impl App for OxifocApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Ok(sample) = self.adc_rx.try_recv() {
            if self.samples.len() >= self.max_samples {
                self.samples.pop_front();
            }
            self.samples.push_back(sample);
        }

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            let is_connected = self.connected.load(Ordering::Relaxed);
            ui.label(if is_connected {
                "Connected"
            } else {
                "Not connected"
            });

            ui.horizontal(|ui| {
                ui.label("Duty (%)");
                ui.add(
                    egui::Slider::new(&mut self.duty, 0.0..=100.0)
                        .clamping(egui::SliderClamping::Always),
                );
                if ui.button("Start").clicked() {
                    let duty = self.duty as u8;
                    let _ = self
                        .cmd_tx
                        .send(HostCommand::Motor(MotorCommand::Start { duty }));
                }
                if ui.button("Stop").clicked() {
                    let _ = self.cmd_tx.send(HostCommand::Motor(MotorCommand::Stop));
                }
            });

            if let Some(last) = self.samples.back() {
                ui.label(format!("Vbus: {:.2} V", last.vbus_mv as f32 / 1000.0));
                ui.label(format!(
                    "FET temp: {:.1} °C",
                    last.fet_temp_c_x10 as f32 / 10.0
                ));
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let points_ia: PlotPoints = self
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ia as f64])
                .collect();
            let points_ib: PlotPoints = self
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ib as f64])
                .collect();
            let points_ic: PlotPoints = self
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.ic as f64])
                .collect();
            let points_vbus: PlotPoints = self
                .samples
                .iter()
                .map(|s| [s.seq as f64, s.vbus_mv as f64])
                .collect();
            let points_temp: PlotPoints = self
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
    }
}

fn main() -> eframe::Result<()> {
    init_tracing();

    let cfg = HostConfig::load_default().unwrap_or_default();
    let runtime = start_host(cfg);

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Oxifoc Host",
        native_options,
        Box::new(move |_cc| Ok(Box::new(OxifocApp::new(runtime)))),
    )
}
