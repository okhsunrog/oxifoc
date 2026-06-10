use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand, ValueEnum};
use oxifoc_core::types::{ControlMode, DetectRequest};
use oxifoc_host_lib::{
    HostCommand, HostConfig, TransportType, init_tracing, list_probes, list_serial_ports,
    start_host,
};

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Transport {
    Serial,
    Rtt,
    Tcp,
    Udp,
    Usb,
}

/// A motor-detection step with no prerequisites (drivable standalone).
#[derive(Debug, Clone, Copy, ValueEnum)]
enum DetectStep {
    /// Measure phase-to-neutral resistance
    Resistance,
    /// Calibrate Hall sensors
    Hall,
}

#[derive(Parser)]
#[command(
    name = "oxifoc-host-cli",
    about = "CLI wrapper over the Oxifoc host backend (shared with egui app)",
    version
)]
struct Cli {
    #[arg(
        long,
        default_value_t = 3,
        value_parser = clap::value_parser!(u64).range(0..=30),
        help = "Seconds to wait for the device handshake before proceeding"
    )]
    wait_secs: u64,

    /// Transport type (serial or rtt). If not specified, uses config file.
    #[arg(short, long, value_enum)]
    transport: Option<Transport>,

    /// Serial port path (e.g., /dev/ttyACM0). Required for serial transport if not in config.
    #[arg(long)]
    serial_path: Option<String>,

    /// Serial baud rate (overrides config file; default: config value, else 921600)
    #[arg(long)]
    baud: Option<u32>,

    /// Debug probe identifier (VID:PID or VID:PID:SERIAL). Required for RTT transport if not in config.
    #[arg(long)]
    probe: Option<String>,

    /// Target chip name (e.g., STM32G431CBUx). Required for RTT transport.
    #[arg(long)]
    chip: Option<String>,

    /// TCP host (for TCP transport, default: 127.0.0.1)
    #[arg(long)]
    tcp_host: Option<String>,

    /// TCP port (for TCP transport, default: 2025)
    #[arg(long)]
    tcp_port: Option<u16>,

    /// UDP host (for UDP transport, default: 127.0.0.1)
    #[arg(long)]
    udp_host: Option<String>,

    /// UDP port (for UDP transport, default: 2025)
    #[arg(long)]
    udp_port: Option<u16>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List available devices (serial ports and debug probes)
    List,
    /// Start the motor at the specified duty cycle
    Start {
        #[arg(
            short,
            long,
            default_value_t = 10,
            value_parser = clap::value_parser!(u8).range(0..=100),
            help = "Duty cycle percentage"
        )]
        duty: u8,
    },
    /// Stop the motor
    Stop,
    /// Select the angle source for commutation
    Source {
        #[arg(value_enum)]
        source: SourceArg,
        #[arg(
            long,
            default_value_t = 150.0,
            help = "Crossover velocity for blended sources (electrical rad/s)"
        )]
        switch_vel: f32,
    },
    /// Monitor telemetry for a duration
    Monitor {
        #[arg(
            short,
            long,
            default_value_t = 10,
            help = "How long to stream telemetry (seconds)"
        )]
        seconds: u64,
        #[arg(
            long,
            default_value_t = 1000,
            help = "Fast telemetry rate in Hz (0 = disabled)"
        )]
        fast_hz: u16,
    },
    /// Run a motor-detection step (effectively-once: retried under a stable id,
    /// the device dedups so the measurement runs at most once)
    Detect {
        #[arg(value_enum)]
        step: DetectStep,
        #[arg(
            long,
            default_value_t = 10.0,
            help = "Max power dissipation during the test (W)"
        )]
        max_power_w: f32,
    },
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum SourceArg {
    /// Hall sensors only
    Hall,
    /// Hall with observer fallback + velocity blend (default sensored mode)
    HallFallback,
    /// Back-EMF observer only (needs spin-up)
    Observer,
    /// HFI only (zero/low speed, salient motors)
    Hfi,
    /// HFI at standstill, blend to the back-EMF observer at speed
    HfiObserver,
}

fn main() -> Result<()> {
    init_tracing();

    let cli = Cli::parse();

    // Handle list command separately (doesn't need connection)
    if matches!(cli.command, Command::List) {
        return list_devices();
    }

    // Build config from CLI args or load from file
    let mut cfg = build_config(&cli)?;

    // Set fast_hz in config so backend enables telemetry at connect time
    if let Command::Monitor { fast_hz, .. } = &cli.command
        && *fast_hz > 0
    {
        cfg.fast_hz = Some(*fast_hz);
    }

    let runtime = start_host(cfg);

    let wait = Duration::from_secs(cli.wait_secs);
    if cli.wait_secs > 0 && !runtime.wait_for_connection(wait) {
        eprintln!(
            "Device did not report connected within {}s; continuing anyway",
            cli.wait_secs
        );
    }

    match cli.command {
        Command::List => unreachable!(), // Handled above
        Command::Start { duty } => {
            // Convert duty percentage to iq_target (0-100% → 0-10A)
            let iq_target = duty as f32 * 0.1;
            runtime
                .cmd_tx
                .send(HostCommand::Motor(ControlMode::CurrentControl {
                    iq_target,
                    id_target: 0.0,
                }))
                .context("send start command")?;
            println!(
                "Start command sent with duty {}% (iq={:.1}A)",
                duty, iq_target
            );
            // cmd_tx is async; give the backend time to deliver the command
            // (at_least_once round-trip) before the process exits.
            std::thread::sleep(Duration::from_millis(800));
        }
        Command::Stop => {
            runtime
                .cmd_tx
                .send(HostCommand::Motor(ControlMode::Stopped))
                .context("send stop command")?;
            println!("Stop command sent");
            std::thread::sleep(Duration::from_millis(800));
        }
        Command::Source { source, switch_vel } => {
            use oxifoc_core::foc::phase::PhaseSource;
            let ps = match source {
                SourceArg::Hall => PhaseSource::Hall,
                SourceArg::HallFallback => PhaseSource::HallWithFallback {
                    blend_low: switch_vel,
                    blend_high: switch_vel * 2.0,
                    timeout_us: 100_000,
                },
                SourceArg::Observer => PhaseSource::Observer,
                SourceArg::Hfi => PhaseSource::Hfi,
                SourceArg::HfiObserver => PhaseSource::HfiToObserver {
                    min_vel: switch_vel,
                    min_confidence: 0.5,
                },
            };
            runtime
                .cmd_tx
                .send(HostCommand::SetPhaseSource(ps))
                .context("send phase source command")?;
            println!(
                "Phase source command sent: {:?} (confirm via monitor — telemetry reports the active source)",
                ps
            );
            std::thread::sleep(Duration::from_millis(800));
        }
        Command::Monitor { seconds, .. } => {
            run_monitor(&runtime, Duration::from_secs(seconds))?;
        }
        Command::Detect { step, max_power_w } => {
            let req = match step {
                DetectStep::Resistance => DetectRequest::MeasureResistance {
                    max_power_loss_w: max_power_w,
                },
                DetectStep::Hall => DetectRequest::CalibrateHall,
            };
            println!("Detection started: {:?}", req);
            let (tx, mut rx) = oxifoc_host_lib::detect_channel();
            runtime
                .cmd_tx
                .send(HostCommand::Detect(req, tx))
                .context("send detect command")?;
            // The detect oneshot is async; poll it (no tokio runtime on this
            // thread). Bounded by a generous deadline (detection can be slow).
            let deadline = Instant::now() + Duration::from_secs(70);
            loop {
                if let Ok(res) = rx.try_recv() {
                    match res {
                        Ok(resp) => println!("Detect result: {:?}", resp),
                        Err(e) => eprintln!("Detect failed: {:?}", e),
                    }
                    break;
                }
                if Instant::now() >= deadline {
                    bail!("Detection timed out");
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    Ok(())
}

fn build_config(cli: &Cli) -> Result<HostConfig> {
    // Start with defaults from config file
    let mut cfg = HostConfig::load_default().unwrap_or_default();

    // Override with CLI arguments
    if let Some(transport) = cli.transport {
        cfg.transport = Some(match transport {
            Transport::Serial => TransportType::Serial,
            Transport::Rtt => TransportType::Rtt,
            Transport::Tcp => TransportType::Tcp,
            Transport::Udp => TransportType::Udp,
            Transport::Usb => TransportType::Usb,
        });
    }

    if let Some(ref path) = cli.serial_path {
        cfg.serial_path = Some(path.clone());
    }

    // Only an explicitly passed --baud overrides the config file; the
    // host-lib accessor falls back to 921600 when neither is set.
    if let Some(baud) = cli.baud {
        cfg.serial_baud = Some(baud);
    }

    if let Some(ref probe) = cli.probe {
        cfg.probe = Some(probe.clone());
    }

    if let Some(ref chip) = cli.chip {
        cfg.chip = Some(chip.clone());
    }

    if let Some(ref host) = cli.tcp_host {
        cfg.tcp_host = Some(host.clone());
    }

    if let Some(port) = cli.tcp_port {
        cfg.tcp_port = Some(port);
    }

    if let Some(ref host) = cli.udp_host {
        cfg.udp_host = Some(host.clone());
    }

    if let Some(port) = cli.udp_port {
        cfg.udp_port = Some(port);
    }

    Ok(cfg)
}

fn list_devices() -> Result<()> {
    println!("=== Serial Ports ===");
    let ports = list_serial_ports();
    if ports.is_empty() {
        println!("  (none found)");
    } else {
        for port in ports {
            println!("  {}", port);
            if let (Some(vid), Some(pid)) = (port.vid, port.pid) {
                println!("    VID:PID = {:04x}:{:04x}", vid, pid);
            }
            if let Some(ref serial) = port.serial_number {
                println!("    Serial: {}", serial);
            }
            if let Some(ref mfr) = port.manufacturer {
                println!("    Manufacturer: {}", mfr);
            }
        }
    }

    println!();
    println!("=== Debug Probes (RTT) ===");
    let probes = list_probes();
    if probes.is_empty() {
        println!("  (none found)");
    } else {
        for probe in probes {
            println!("  {}", probe);
            println!("    Identifier: {}", probe.identifier);
            if let Some(ref serial) = probe.serial_number {
                println!("    Serial: {}", serial);
            }
        }
    }

    Ok(())
}

fn run_monitor(runtime: &oxifoc_host_lib::HostRuntime, duration: Duration) -> Result<()> {
    use crossbeam_channel::RecvTimeoutError;

    println!("Streaming telemetry for {:?}...", duration);
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        // Print fast telemetry
        match runtime.fast_rx.recv_timeout(Duration::from_millis(500)) {
            Ok(sample) => {
                println!(
                    "#{:>5} ia:{:>7.2}A ib:{:>7.2}A ic:{:>7.2}A id:{:>7.2}A iq:{:>7.2}A erpm:{:>6}",
                    sample.seq, sample.ia, sample.ib, sample.ic, sample.id, sample.iq, sample.erpm,
                );
            }
            Err(RecvTimeoutError::Timeout) => {
                if !runtime.connected.load(Ordering::Relaxed) {
                    eprintln!("Waiting for device connection...");
                }
            }
            Err(RecvTimeoutError::Disconnected) => {
                bail!("Telemetry channel disconnected");
            }
        }
        // Print slow telemetry when available
        if let Ok(slow) = runtime.slow_rx.try_recv() {
            println!(
                "  [sys] vbus:{:.1}V fet:{:.1}°C motor:{:.1}°C faults:{}",
                slow.vbus_mv as f32 / 1000.0,
                slow.fet_temp_c_x10 as f32 / 10.0,
                slow.motor_temp_c_x10 as f32 / 10.0,
                slow.fault_count,
            );
        }
    }

    Ok(())
}
