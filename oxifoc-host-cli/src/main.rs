use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use oxifoc_host_lib::{HostCommand, HostConfig, HostRuntime, init_tracing, start_host};
use oxifoc_protocol::MotorCommand;

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

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
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
    Stop,
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
    let cfg = HostConfig::load_default().unwrap_or_default();
    let runtime = start_host(cfg);

    let wait = Duration::from_secs(cli.wait_secs);
    if cli.wait_secs > 0 && !runtime.wait_for_connection(wait) {
        eprintln!(
            "Device did not report connected within {}s; continuing anyway",
            cli.wait_secs
        );
    }

    match cli.command {
        Command::Start { duty } => {
            runtime
                .cmd_tx
                .send(HostCommand::Motor(MotorCommand::Start { duty }))
                .context("send start command")?;
            println!("Start command sent with duty {}%", duty);
        }
        Command::Stop => {
            runtime
                .cmd_tx
                .send(HostCommand::Motor(MotorCommand::Stop))
                .context("send stop command")?;
            println!("Stop command sent");
        }
        Command::Monitor { seconds } => run_monitor(runtime, Duration::from_secs(seconds))?,
    }

    Ok(())
}

fn run_monitor(runtime: HostRuntime, duration: Duration) -> Result<()> {
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
