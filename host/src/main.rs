mod config;

use anyhow::{Context, Result};
use cobs_acc::{CobsAccumulator, FeedResult};
use config::HostConfig;
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender, unbounded};
use defmt_decoder::{DecodeError, StreamDecoder, Table};
use eframe::{App, egui};
use egui_plot::{Line, Plot, PlotPoints};
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
use ergot::interface_manager::utils::std::StdQueue as ErgotStdQueue;
use ergot::interface_manager::utils::std::new_std_queue;
use ergot::interface_manager::{Interface, InterfaceState};
use ergot::net_stack::ArcNetStack;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{
    AdcSample, AdcSampleEndpoint, ButtonEndpoint, ButtonEvent, MotorCommand, MotorEndpoint,
};
use probe_rs::Permissions;
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{Rtt, ScanRegion};
use std::collections::VecDeque;
use std::fs;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;
use tokio::runtime::Runtime;
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
    info!(
        "Oxifoc Host backend - RTT (chip={:?}, probe={:?})",
        cfg.chip, cfg.probe
    );

    let lister = Lister::new();
    let probes = lister.list_all();
    if probes.is_empty() {
        error!("No debug probes found! Make sure ST-Link is connected.");
        return Ok(());
    }

    let probe = if let Some(sel) = cfg.probe.clone() {
        let mut parts = sel.split(':');
        let vid = parts.next();
        let pid = parts.next();
        let serial = parts.next();
        let chosen = probes
            .iter()
            .find(|p| {
                let ok_vid = vid
                    .and_then(|v| u16::from_str_radix(v, 16).ok())
                    .map(|v| p.vendor_id == v)
                    .unwrap_or(true);
                let ok_pid = pid
                    .and_then(|v| u16::from_str_radix(v, 16).ok())
                    .map(|v| p.product_id == v)
                    .unwrap_or(true);
                let ok_ser = serial
                    .map(|s| p.serial_number.as_deref() == Some(s))
                    .unwrap_or(true);
                ok_vid && ok_pid && ok_ser
            })
            .ok_or_else(|| anyhow::anyhow!("Configured probe not found: {}", sel))?;
        chosen.open().context("Failed to open selected probe")?
    } else {
        probes[0].open().context("Failed to open probe")?
    };

    let ts = match cfg.chip.clone() {
        Some(name) => probe_rs::config::TargetSelector::from(name),
        None => probe_rs::config::TargetSelector::Auto,
    };
    let mut session = probe
        .attach(ts, Permissions::default())
        .context("Failed to attach to target")?;

    info!("Successfully attached to STM32G431");

    let mut core = session.core(0)?;
    let mut rtt =
        Rtt::attach_region(&mut core, &ScanRegion::Ram).context("Failed to attach RTT")?;

    info!("RTT attached successfully");
    info!("Available RTT up channels:");
    for (idx, channel) in rtt.up_channels().iter().enumerate() {
        info!("  up{}: {}", idx, channel.name().unwrap_or("unnamed"));
    }
    info!("Available RTT down channels:");
    for (idx, channel) in rtt.down_channels().iter().enumerate() {
        info!("  down{}: {}", idx, channel.name().unwrap_or("unnamed"));
    }

    let mut find_by_name = |name: &str| -> Option<usize> {
        rtt.up_channels().iter().enumerate().find_map(|(i, ch)| {
            if ch.name().map(|n| n == name).unwrap_or(false) {
                Some(i)
            } else {
                None
            }
        })
    };
    let ergot_up_idx = if cfg.stream_ergot() {
        find_by_name("ergot").or(Some(1))
    } else {
        None
    };
    let defmt_up_idx = if cfg.stream_defmt() {
        find_by_name("defmt").or(Some(0))
    } else {
        None
    };
    info!(
        "Using channels: ergot={:?}, defmt={:?}",
        ergot_up_idx, defmt_up_idx
    );

    struct RttInterface;
    impl Interface for RttInterface {
        type Sink = ErgotSink<ErgotStdQueue>;
    }
    type EdgeProfile = DirectEdge<RttInterface>;
    type EdgeStack = ArcNetStack<CriticalSectionRawMutex, EdgeProfile>;
    const ERGOT_MTU: u16 = 1024;
    let queue = new_std_queue(4096);

    let stack: EdgeStack = ArcNetStack::new_with_profile(DirectEdge::new_controller(
        ErgotSink::new_from_handle(queue.clone(), ERGOT_MTU),
        InterfaceState::Active {
            net_id: 1,
            node_id: 1,
        },
    ));

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

    // Prepare defmt decoder
    let default_elf = {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../device/target/thumbv7em-none-eabihf/release/oxifoc");
        p.to_string_lossy().into_owned()
    };
    let defmt_table: Option<Table> = if defmt_up_idx.is_some() {
        let elf_path = cfg.elf.clone().unwrap_or(default_elf);
        let elf_bytes =
            fs::read(&elf_path).with_context(|| format!("Failed to read ELF at {}", elf_path))?;
        Some(
            Table::parse(&elf_bytes)
                .context("Parsing defmt table from ELF failed")?
                .ok_or_else(|| {
                    anyhow::anyhow!("No .defmt section in ELF; build device with defmt")
                })?,
        )
    } else {
        None
    };
    let mut defmt_stream: Option<Box<dyn StreamDecoder + '_>> =
        defmt_table.as_ref().map(|t| t.new_stream_decoder());

    let mut buf = vec![0u8; 4096];
    let mut defbuf = vec![0u8; 2048];
    let mut cobs_acc = CobsAccumulator::new_boxslice(1024 * 4);
    let mut net_id = Some(1u16);
    let down_idx = {
        let mut find_down = |name: &str| -> Option<usize> {
            rtt.down_channels().iter().enumerate().find_map(|(i, ch)| {
                if ch.name().map(|n| n == name).unwrap_or(false) {
                    Some(i)
                } else {
                    None
                }
            })
        };
        find_down("ergot-down").or(Some(0))
    };
    let tx_consumer = queue.stream_consumer();

    connected_flag.store(true, Ordering::Relaxed);

    loop {
        if let Some(up_idx) = ergot_up_idx
            && let Some(channel) = rtt.up_channels().get_mut(up_idx)
        {
            let count = channel.read(&mut core, &mut buf)?;
            if count > 0 {
                let mut window = &mut buf[..count];
                while !window.is_empty() {
                    window = match cobs_acc.feed_raw(window) {
                        FeedResult::Consumed => break,
                        FeedResult::OverFull(new_w) => new_w,
                        FeedResult::DecodeError(new_w) => new_w,
                        FeedResult::Success { data, remaining }
                        | FeedResult::SuccessInput { data, remaining } => {
                            ergot_edge_process_frame(&mut net_id, data, &stack, ());
                            remaining
                        }
                    };
                }
            }
        }

        if let (Some(up_idx), Some(stream)) = (defmt_up_idx, defmt_stream.as_mut())
            && let Some(channel) = rtt.up_channels().get_mut(up_idx)
        {
            let count = channel.read(&mut core, &mut defbuf)?;
            if count > 0 {
                stream.received(&defbuf[..count]);
                loop {
                    match stream.decode() {
                        Ok(frame) => {
                            println!("{}", frame.display(true));
                        }
                        Err(DecodeError::UnexpectedEof) => break,
                        Err(DecodeError::Malformed) => {
                            error!("Malformed defmt frame");
                            break;
                        }
                    }
                }
            }
        }

        if let Some(di) = down_idx
            && let Some(channel) = rtt.down_channels().get_mut(di)
        {
            for _ in 0..8 {
                match tokio::time::timeout(Duration::from_millis(1), tx_consumer.wait_read()).await
                {
                    Ok(frame) => {
                        let len = frame.len();
                        if len == 0 {
                            break;
                        }
                        let data = &frame[..len];
                        let _ = channel.write(&mut core, data);
                        frame.release(len);
                    }
                    Err(_) => break,
                }
            }
        }

        tokio::time::sleep(Duration::from_millis(10)).await;
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
