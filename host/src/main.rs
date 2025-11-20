mod config;

use anyhow::{Context, Result};
use cobs_acc::{CobsAccumulator, FeedResult};
use config::HostConfig;
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender, unbounded};
use defmt_decoder::{DecodeError, Table};
use eframe::{App, egui};
use egui_plot::{Line, Plot, PlotPoints};
use ergot::interface_manager::interface_impls::tokio_serial_cobs::TokioSerialInterface;
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
use ergot::interface_manager::utils::std::new_std_queue;
use ergot::interface_manager::InterfaceState;
use ergot::net_stack::ArcNetStack;
use ergot::well_known::ErgotDefmtRxOwnedTopic;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{
    AdcSample, AdcSampleEndpoint, ButtonEndpoint, ButtonEvent, MotorCommand, MotorEndpoint,
};
use std::collections::VecDeque;
use std::fs;
use std::path::Path;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::runtime::Runtime;
use tokio_serial::SerialPortBuilderExt;
use tracing::{error, info};

fn init_tracing() {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(true)
        .with_level(true)
        .compact()
        .try_init();
}

#[derive(Clone)]
enum HostCommand {
    Motor(MotorCommand),
}

fn spawn_backend(
    config: HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: Receiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let rt = Runtime::new().expect("Failed to create tokio runtime");
        if let Err(e) = rt.block_on(backend_main(config, adc_tx, cmd_rx, connected_flag)) {
            error!("backend_main error: {:?}", e);
        }
    });
}

async fn backend_main(
    cfg: HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: Receiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
) -> Result<()> {
    const ERGOT_MTU: u16 = 512;

    let serial_path = cfg.serial_path();
    let baud = cfg.serial_baud();

    info!(
        "Oxifoc Host backend - USART2 over ST-Link VCP (serial={}, baud={})",
        serial_path, baud
    );

    if !cfg.stream_ergot() {
        info!("stream_ergot disabled in config; backend not starting transport");
        return Ok(());
    }

    let port = tokio_serial::new(serial_path.clone(), baud)
        .open_native_async()
        .with_context(|| format!("Failed to open serial port {}", serial_path))?;
    let (mut serial_rx, mut serial_tx) = tokio::io::split(port);

    type EdgeProfile = DirectEdge<TokioSerialInterface>;
    type EdgeStack = ArcNetStack<CriticalSectionRawMutex, EdgeProfile>;
    let queue = new_std_queue(4096);

    let stack: EdgeStack = ArcNetStack::new_with_profile(DirectEdge::new_controller(
        ErgotSink::new_from_handle(queue.clone(), ERGOT_MTU),
        InterfaceState::Active {
            net_id: 1,
            node_id: 1,
        },
    ));

    // Serial RX worker
    tokio::spawn({
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        async move {
            let mut buf = vec![0u8; 2048];
            let mut cobs_acc = CobsAccumulator::new_boxslice((ERGOT_MTU as usize) + 64);
            let mut net_id = Some(1u16);
            loop {
                match serial_rx.read(&mut buf).await {
                    Ok(0) => {
                        error!("Serial port closed");
                        connected_flag.store(false, Ordering::Relaxed);
                        break;
                    }
                    Ok(count) => {
                        let mut window = &mut buf[..count];
                        while !window.is_empty() {
                            window = match cobs_acc.feed_raw(window) {
                                FeedResult::Consumed => break,
                                FeedResult::OverFull(rem) | FeedResult::DecodeError(rem) => rem,
                                FeedResult::Success { data, remaining }
                                | FeedResult::SuccessInput { data, remaining } => {
                                    ergot_edge_process_frame(&mut net_id, data, &stack, ());
                                    remaining
                                }
                            };
                        }
                    }
                    Err(e) => {
                        error!("Serial read error: {:?}", e);
                        connected_flag.store(false, Ordering::Relaxed);
                        break;
                    }
                }
            }
        }
    });

    // Serial TX worker
    let tx_queue: &'static _ = Box::leak(Box::new(queue.clone()));
    tokio::spawn({
        let tx_consumer = tx_queue.stream_consumer();
        let connected_flag = connected_flag.clone();
        async move {
            loop {
                let frame = tx_consumer.wait_read().await;
                let len = frame.len();
                if len == 0 {
                    frame.release(len);
                    continue;
                }

                if let Err(e) = serial_tx.write_all(&frame[..len]).await {
                    error!("Serial write error: {:?}", e);
                    connected_flag.store(false, Ordering::Relaxed);
                    frame.release(len);
                    break;
                }
                frame.release(len);
            }
        }
    });

    // Button events
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let server = stack
                .endpoints()
                .bounded_server::<ButtonEndpoint, 8>(Some("button"));
            let server = pin!(server);
            let mut h = server.attach();
            loop {
                let _ = h
                    .serve(|event: &ButtonEvent| {
                        let ev = event.clone();
                        async move {
                            match ev {
                                ButtonEvent::SingleClick => tracing::info!("Button: SINGLE"),
                                ButtonEvent::DoubleClick => tracing::info!("Button: DOUBLE"),
                                ButtonEvent::Hold => tracing::info!("Button: HOLD"),
                            }
                        }
                    })
                    .await;
            }
        }
    });

    // ADC samples from device
    tokio::spawn({
        let stack = stack.clone();
        let adc_tx = adc_tx.clone();
        async move {
            let server = stack
                .endpoints()
                .bounded_server::<AdcSampleEndpoint, 64>(Some("adc"));
            let server = pin!(server);
            let mut h = server.attach();
            loop {
                let _ = h
                    .serve(|sample: &AdcSample| {
                        let s = sample.clone();
                        let adc_tx = adc_tx.clone();
                        async move {
                            let _ = adc_tx.send(s);
                        }
                    })
                    .await;
            }
        }
    });

    // Motor command handler
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        async move {
            while let Ok(HostCommand::Motor(mc)) = cmd_rx.recv() {
                let device_addr = Address {
                    network_id: 1,
                    node_id: 2,
                    port_id: 0,
                };
                let res = stack
                    .endpoints()
                    .request::<MotorEndpoint>(device_addr, &mc, Some("motor"))
                    .await;
                if let Err(e) = res {
                    tracing::warn!("Motor command failed: {:?}", e);
                }
            }
        }
    });

    // Handshake: DeviceInfo
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        async move {
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };
            let mut backoff = Duration::from_millis(100);
            for attempt in 1..=10u32 {
                let fut = stack.endpoints().request::<oxifoc_protocol::InfoEndpoint>(
                    device_addr,
                    &(),
                    Some("device_info"),
                );
                match tokio::time::timeout(Duration::from_millis(800), fut).await {
                    Ok(Ok(info)) => {
                        let hw = info.hw.as_str();
                        let sw = info.sw.as_str();
                        tracing::info!("Device connected: hw='{}' sw='{}'", hw, sw);
                        connected_flag.store(true, Ordering::Relaxed);
                        return;
                    }
                    Ok(Err(e)) => {
                        tracing::warn!("DeviceInfo attempt {} failed: {:?}", attempt, e);
                    }
                    Err(_) => {
                        tracing::warn!("DeviceInfo attempt {} timed out", attempt);
                    }
                }
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(Duration::from_secs(2));
            }
            tracing::warn!("Device info not received after retries; continuing without it");
        }
    });

    // Prepare defmt decoder (frames arrive over ergot defmt sink)
    if cfg.stream_defmt() {
        let default_elf = {
            let p = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../device/target/thumbv7em-none-eabihf/release/oxifoc");
            p.to_string_lossy().into_owned()
        };
        let elf_path = cfg.elf.clone().unwrap_or(default_elf);
        let elf_bytes =
            fs::read(&elf_path).with_context(|| format!("Failed to read ELF at {}", elf_path))?;
        let table = Table::parse(&elf_bytes)
            .context("Parsing defmt table from ELF failed")?
            .ok_or_else(|| anyhow::anyhow!("No .defmt section in ELF; build device with defmt"))?;

        tokio::spawn({
            let stack = stack.clone();
            async move {
                let sub = stack
                    .topics()
                    .heap_bounded_receiver::<ErgotDefmtRxOwnedTopic>(32, Some("defmt"));
                let sub = pin!(sub);
                let mut hdl = sub.subscribe();

                loop {
                    let msg = hdl.recv().await;
                    match table.decode(&msg.t.frame) {
                        Ok((frame, _consumed)) => {
                            println!("{}", frame.display(true));
                        }
                        Err(DecodeError::UnexpectedEof) => {
                            error!("Unexpected EOF while decoding defmt frame");
                        }
                        Err(DecodeError::Malformed) => {
                            error!("Malformed defmt frame");
                        }
                    }
                }
            }
        });
    }

    // Keep backend alive
    loop {
        tokio::time::sleep(Duration::from_secs(60)).await;
    }
}

struct OxifocApp {
    connected: Arc<AtomicBool>,
    adc_rx: Receiver<AdcSample>,
    cmd_tx: Sender<HostCommand>,
    duty: f32,
    samples: VecDeque<AdcSample>,
    max_samples: usize,
}

impl OxifocApp {
    fn new(
        adc_rx: Receiver<AdcSample>,
        cmd_tx: Sender<HostCommand>,
        connected: Arc<AtomicBool>,
    ) -> Self {
        Self {
            connected,
            adc_rx,
            cmd_tx,
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

    let (adc_tx, adc_rx) = unbounded::<AdcSample>();
    let (cmd_tx, cmd_rx) = unbounded::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));

    // Spawn backend immediately; UI will reflect connection state.
    let cfg = HostConfig::load_default().unwrap_or_default();
    spawn_backend(cfg, adc_tx, cmd_rx, connected.clone());

    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Oxifoc Host",
        native_options,
        Box::new(|_cc| Ok(Box::new(OxifocApp::new(adc_rx, cmd_tx, connected)))),
    )
}
