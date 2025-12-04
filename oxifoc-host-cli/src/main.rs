use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand, ValueEnum};
use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::{
    HostCommand, HostConfig, TransportType, init_tracing, list_probes, list_serial_ports,
    start_host,
};

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Transport {
    Serial,
    Rtt,
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

    /// Serial baud rate
    #[arg(long, default_value_t = 921600)]
    baud: u32,

    /// Debug probe identifier (VID:PID or VID:PID:SERIAL). Required for RTT transport if not in config.
    #[arg(long)]
    probe: Option<String>,

    /// Target chip name (e.g., STM32G431CBUx). Required for RTT transport.
    #[arg(long)]
    chip: Option<String>,

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
    /// Monitor ADC samples for a duration
    Monitor {
        #[arg(
            short,
            long,
            default_value_t = 10,
            help = "How long to stream ADC samples (seconds)"
        )]
        seconds: u64,
    },
}

fn main() -> Result<()> {
    init_tracing();

    let cli = Cli::parse();

    // Handle list command separately (doesn't need connection)
    if matches!(cli.command, Command::List) {
        return list_devices();
    }

    // Build config from CLI args or load from file
    let cfg = build_config(&cli)?;
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
        }
        Command::Stop => {
            runtime
                .cmd_tx
                .send(HostCommand::Motor(ControlMode::Stopped))
                .context("send stop command")?;
            println!("Stop command sent");
        }
        Command::Monitor { seconds } => run_monitor(&runtime, Duration::from_secs(seconds))?,
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
        });
    }

    if let Some(ref path) = cli.serial_path {
        cfg.serial_path = Some(path.clone());
    }

    cfg.serial_baud = Some(cli.baud);

    if let Some(ref probe) = cli.probe {
        cfg.probe = Some(probe.clone());
    }

    if let Some(ref chip) = cli.chip {
        cfg.chip = Some(chip.clone());
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

    println!("Streaming ADC samples for {:?}...", duration);
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        match runtime.adc_rx.recv_timeout(Duration::from_millis(500)) {
            Ok(sample) => {
                println!(
                    "#{:>5} ia:{:>6} ib:{:>6} ic:{:>6} vbus:{:>6.1}V fet:{:>5.1}C",
                    sample.seq,
                    sample.ia,
                    sample.ib,
                    sample.ic,
                    sample.vbus_mv as f32 / 1000.0,
                    sample.fet_temp_c_x10 as f32 / 10.0,
                );
            }
            Err(RecvTimeoutError::Timeout) => {
                if !runtime.connected.load(Ordering::Relaxed) {
                    eprintln!("Waiting for device connection...");
                }
            }
            Err(RecvTimeoutError::Disconnected) => {
                bail!("ADC channel disconnected");
            }
        }
    }

    Ok(())
}
