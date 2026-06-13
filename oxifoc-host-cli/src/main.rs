//! Oxifoc host CLI — scriptable control, diagnostics and telemetry capture.
//!
//! Designed to be equally usable by a human and by an AI agent driving the
//! bench: every command has a `--json` mode with a stable machine-readable
//! envelope (one JSON document on stdout, nonzero exit on failure), config
//! is fully readable/writable field-by-field, and `record` captures fast
//! telemetry into parquet files with full provenance metadata.

mod config_cli;
mod detect;
mod maneuver;
mod record;
mod watch;

use oxifoc_core::types::{ConfigResponse, MotorStatus};
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand, ValueEnum};
use oxifoc_core::types::{ControlMode, FaultCategory, FaultRequest};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, TransportType, fault_channel, init_tracing, list_probes,
    list_serial_ports, motor_channel, start_host,
};
use serde_json::json;

/// Send a motor command and wait for the device's acknowledgement — the
/// process must exit nonzero when the command never reached the device.
pub(crate) fn send_motor_acked(runtime: &HostRuntime, mode: ControlMode) -> Result<MotorStatus> {
    let (tx, rx) = motor_channel();
    runtime
        .cmd_tx
        .send(HostCommand::MotorAck(mode, tx))
        .context("send motor command")?;
    rx.blocking_recv()
        .context("backend dropped the motor command")?
        .context("motor command not acknowledged by the device")
}

/// Print a command result: one JSON document in `--json` mode, the human
/// line otherwise.
pub(crate) fn emit(json_mode: bool, value: serde_json::Value, human: String) {
    if json_mode {
        println!("{value:#}");
    } else {
        println!("{human}");
    }
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Transport {
    Serial,
    Rtt,
    Tcp,
    Udp,
    Usb,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum DetectStep {
    /// Measure phase-to-neutral resistance
    Resistance,
    /// Measure d/q inductance (HFI with pulse fallback; needs R)
    Inductance,
    /// Measure flux linkage (needs R, L, pole pairs)
    Flux,
    /// Calibrate Hall sensors
    Hall,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum FaultCategoryArg {
    OverCurrent,
    OverVoltage,
    UnderVoltage,
    OverTemp,
    DriverFault,
    HallError,
    Stall,
    CalibrationFault,
    CommTimeout,
}

impl From<FaultCategoryArg> for FaultCategory {
    fn from(v: FaultCategoryArg) -> Self {
        match v {
            FaultCategoryArg::OverCurrent => Self::OverCurrent,
            FaultCategoryArg::OverVoltage => Self::OverVoltage,
            FaultCategoryArg::UnderVoltage => Self::UnderVoltage,
            FaultCategoryArg::OverTemp => Self::OverTemp,
            FaultCategoryArg::DriverFault => Self::DriverFault,
            FaultCategoryArg::HallError => Self::HallError,
            FaultCategoryArg::Stall => Self::Stall,
            FaultCategoryArg::CalibrationFault => Self::CalibrationFault,
            FaultCategoryArg::CommTimeout => Self::CommTimeout,
        }
    }
}

#[derive(Parser)]
#[command(
    name = "oxifoc-host-cli",
    about = "CLI over the Oxifoc host backend — control, config, diagnostics, telemetry capture",
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

    /// Machine-readable output: one JSON document on stdout per command
    #[arg(long, global = true)]
    json: bool,

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
    /// Show device hardware info from the handshake
    Info,
    /// One system-health snapshot (mode, state, vbus, temps, faults, source)
    Status,
    /// Query active faults; --clear / --clear-category to clear
    Faults {
        #[arg(long, help = "Clear all faults")]
        clear: bool,
        #[arg(long, value_enum, help = "Clear one fault category")]
        clear_category: Option<FaultCategoryArg>,
        #[arg(
            long,
            help = "Watch the fault topic: print a snapshot on every change \
                    (device pushes on raise/refine/clear)"
        )]
        watch: bool,
        #[arg(
            long,
            default_value_t = 0,
            help = "Stop watching after this many seconds (0 = until Ctrl-C)"
        )]
        seconds: u64,
    },
    /// Start the motor in current (torque) control
    Start {
        #[arg(
            short,
            long,
            default_value_t = 1.0,
            allow_hyphen_values = true,
            help = "Target q-axis current in Amps (sign = direction)"
        )]
        iq: f32,
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            help = "Target d-axis current in Amps (field weakening)"
        )]
        id: f32,
    },
    /// Stop the motor (PWM off)
    Stop,
    /// Coast — all FETs off, motor spins freely
    Coast,
    /// Engage the parking brake (short the windings; near-standstill only)
    Brake,
    /// Run velocity control at the given electrical rad/s (sign = direction)
    Velocity {
        #[arg(allow_hyphen_values = true, help = "Target velocity, electrical rad/s")]
        rad_s: f32,
    },
    /// Open-loop drive at a commanded angle/velocity (no sensor feedback)
    Openloop {
        #[arg(long, default_value_t = 1.0, help = "Current magnitude (A)")]
        current: f32,
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            help = "Electrical velocity (rad/s); 0 = lock rotor at --angle"
        )]
        velocity: f32,
        #[arg(long, default_value_t = 0.0, help = "Initial electrical angle (rad)")]
        angle: f32,
    },
    /// Direct dq voltage, no PI (measurement/bringup; no current regulation!)
    Voltage {
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            help = "d-axis voltage (V)"
        )]
        vd: f32,
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            help = "q-axis voltage (V)"
        )]
        vq: f32,
        #[arg(long, default_value_t = 0.0, help = "Electrical angle (rad)")]
        angle: f32,
    },
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
        #[arg(
            long,
            default_value_t = 2.0,
            help = "Drive-voltage threshold for hfi-observer-volts (V, ≈5% of vbus)"
        )]
        toggle_v: f32,
    },
    /// Monitor telemetry for a duration (JSONL per sample with --json)
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
    /// Record fast telemetry into a parquet file (with provenance metadata)
    Record {
        #[arg(short, long, help = "Output parquet file path")]
        out: String,
        #[arg(
            short,
            long,
            default_value_t = 10.0,
            help = "Capture duration (seconds)"
        )]
        seconds: f64,
        #[arg(long, default_value_t = 5000, help = "Fast telemetry rate (Hz)")]
        fast_hz: u16,
        #[arg(
            long,
            help = "Exit 0 even when frames were dropped (default: gaps fail the capture)"
        )]
        allow_gaps: bool,
    },
    /// Run or validate a scripted experiment (timed commands + capture)
    Maneuver {
        #[command(subcommand)]
        action: ManeuverAction,
    },
    /// Device configuration operations
    Config {
        #[command(subcommand)]
        action: ConfigAction,
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
        #[arg(long, help = "Resistance (Ω); default: stored motor-params")]
        resistance: Option<f32>,
        #[arg(long, help = "Avg inductance (H); default: stored motor-params")]
        inductance: Option<f32>,
        #[arg(long, help = "Pole pairs; default: stored motor-params")]
        pole_pairs: Option<u8>,
        #[arg(
            long,
            default_value_t = 700.0,
            help = "Open-loop ERPM for flux spin-up"
        )]
        erpm: f32,
        #[arg(
            long,
            help = "Record raw fast telemetry during the step into a parquet \
                    file (fast_hz = FOC frequency, M=1: decimated rates would \
                    CIC-null the HFI carrier). Written on failure too. NOTE: \
                    the virtual device runs detection in a private sim that \
                    this capture cannot see; meaningful on hardware."
        )]
        record: Option<String>,
        #[arg(
            long,
            help = "Write the measured values into the motor-params config group"
        )]
        apply: bool,
    },
}

#[derive(Subcommand)]
enum ManeuverAction {
    /// Execute a maneuver file against the device, recording to parquet
    Run {
        #[arg(help = "Maneuver JSON file")]
        file: String,
        #[arg(short, long, help = "Output parquet file path")]
        out: String,
        #[arg(long, help = "Skip the device current-limits check")]
        force: bool,
        #[arg(
            long,
            help = "Exit 0 even when frames were dropped (default: gaps fail the capture)"
        )]
        allow_gaps: bool,
    },
    /// Validate a maneuver file offline (no device needed)
    Validate {
        #[arg(help = "Maneuver JSON file")]
        file: String,
    },
}

#[derive(Subcommand)]
enum ConfigAction {
    /// Read every config group and print it.
    /// --rust emits a ready-to-paste `baked_config.rs` body.
    Dump {
        #[arg(long, help = "Emit Rust code for src/baked_config.rs")]
        rust: bool,
    },
    /// Read one config group (prints defaults when nothing is stored)
    Get {
        #[arg(help = "Group name, e.g. motor-params, current-limits, failsafe")]
        group: String,
    },
    /// Set fields of one config group: field=value pairs (read-modify-write)
    Set {
        #[arg(help = "Group name, e.g. current-limits")]
        group: String,
        #[arg(required = true, help = "field=value pairs, e.g. max_iq_a=5.0")]
        fields: Vec<String>,
    },
    /// Erase ALL stored config groups (factory reset; requires --yes)
    Reset {
        #[arg(long, help = "Confirm the erase")]
        yes: bool,
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
    /// Like hfi-observer, but the crossover criterion is drive voltage
    /// (MESC-style |vq − R·iq| > --toggle-v), self-normalizing per motor
    HfiObserverVolts,
}

fn main() -> Result<()> {
    init_tracing();

    let cli = Cli::parse();
    let json = cli.json;

    // Handle list command separately (doesn't need connection)
    if matches!(cli.command, Command::List) {
        return list_devices(json);
    }

    // Maneuver validation is offline too.
    if let Command::Maneuver {
        action: ManeuverAction::Validate { ref file },
    } = cli.command
    {
        let m = maneuver::load(file)?;
        emit(
            json,
            json!({"valid": true, "name": m.name, "steps": m.timeline.len()}),
            format!("{file}: ok — '{}', {} step(s)", m.name, m.timeline.len()),
        );
        return Ok(());
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
        Command::Info => {
            let info = record::latest_hw_info(&runtime)
                .context("no hardware info received — device not connected?")?;
            emit(
                json,
                json!({
                    "connected": runtime.connected.load(Ordering::Relaxed),
                    "hw": info.hw.as_str(),
                    "sw": info.sw.as_str(),
                    "mcu": info.mcu.as_str(),
                    "uuid": info.uuid.as_str(),
                    "foc_freq_hz": info.foc_freq_hz,
                    "max_current_a": info.max_current_a,
                }),
                format!(
                    "{} ({}) sw={} uuid={} foc={}Hz max_i={}A",
                    info.hw, info.mcu, info.sw, info.uuid, info.foc_freq_hz, info.max_current_a
                ),
            );
        }
        Command::Status => {
            let slow = runtime
                .slow_rx
                .recv_timeout(Duration::from_secs(3))
                .context("no slow telemetry within 3 s — device not connected?")?;
            emit(
                json,
                serde_json::to_value(slow)?,
                format!(
                    "state={:?} mode={:?} source={:?} vbus={:.2}V fet={:.1}°C motor={:.1}°C faults={}",
                    slow.motor_state,
                    slow.control_mode,
                    slow.phase_source,
                    slow.vbus_mv as f32 / 1000.0,
                    f32::from(slow.fet_temp_c_x10) / 10.0,
                    f32::from(slow.motor_temp_c_x10) / 10.0,
                    slow.fault_count,
                ),
            );
        }
        Command::Faults {
            clear,
            clear_category,
            watch,
            seconds,
        } => {
            if watch {
                watch::watch_faults(&runtime, seconds, json)?;
                return Ok(());
            }
            let req = if clear {
                FaultRequest::ClearAll
            } else if let Some(cat) = clear_category {
                FaultRequest::Clear(cat.into())
            } else {
                FaultRequest::Query
            };
            let (tx, rx) = fault_channel();
            runtime
                .cmd_tx
                .send(HostCommand::Fault(req, tx))
                .context("send fault request")?;
            let resp = rx
                .blocking_recv()
                .context("backend dropped the fault request")?
                .context("fault request failed")?;
            let human = if resp.faults.is_empty() {
                format!("no active faults (total={})", resp.total)
            } else {
                let lines: Vec<String> = resp
                    .faults
                    .iter()
                    .map(|f| format!("{:?} [{:?}]: {}", f.category, f.severity, f.details))
                    .collect();
                format!("{} active fault(s):\n  {}", resp.total, lines.join("\n  "))
            };
            emit(json, serde_json::to_value(&resp)?, human);
        }
        Command::Start { iq, id } => {
            let status = send_motor_acked(
                &runtime,
                ControlMode::CurrentControl {
                    iq_target: iq,
                    id_target: id,
                },
            )?;
            emit(
                json,
                json!({"sent": {"iq": iq, "id": id}, "device": format!("{status:?}")}),
                format!("Motor started at iq={iq:.1} A — device: {status:?}"),
            );
        }
        Command::Stop => {
            let status = send_motor_acked(&runtime, ControlMode::Stopped)?;
            emit(
                json,
                json!({"sent": "stopped", "device": format!("{status:?}")}),
                format!("Motor stopped — device: {status:?}"),
            );
        }
        Command::Coast => {
            let status = send_motor_acked(&runtime, ControlMode::Coast)?;
            emit(
                json,
                json!({"sent": "coast", "device": format!("{status:?}")}),
                format!("Coasting (gates off) — device: {status:?}"),
            );
        }
        Command::Brake => {
            let status = send_motor_acked(&runtime, ControlMode::Brake)?;
            emit(
                json,
                json!({"sent": "brake", "device": format!("{status:?}")}),
                format!("Brake engaged (ramps to standstill first if moving) — device: {status:?}"),
            );
        }
        Command::Velocity { rad_s } => {
            let status =
                send_motor_acked(&runtime, ControlMode::VelocityControl { target_vel: rad_s })?;
            emit(
                json,
                json!({"sent": {"velocity_rad_s": rad_s}, "device": format!("{status:?}")}),
                format!("Velocity target {rad_s} erad/s — device: {status:?}"),
            );
        }
        Command::Openloop {
            current,
            velocity,
            angle,
        } => {
            let status = send_motor_acked(
                &runtime,
                ControlMode::OpenLoop {
                    angle_rad: angle,
                    current,
                    velocity_rad_s: velocity,
                    pi_gains: None,
                },
            )?;
            emit(
                json,
                json!({"sent": {"current": current, "velocity_rad_s": velocity, "angle_rad": angle}, "device": format!("{status:?}")}),
                format!("Open loop: {current} A at {velocity} erad/s — device: {status:?}"),
            );
        }
        Command::Voltage { vd, vq, angle } => {
            let status = send_motor_acked(
                &runtime,
                ControlMode::DirectVoltage {
                    vd,
                    vq,
                    angle_rad: angle,
                },
            )?;
            emit(
                json,
                json!({"sent": {"vd": vd, "vq": vq, "angle_rad": angle}, "device": format!("{status:?}")}),
                format!("Direct voltage vd={vd} vq={vq} V — device: {status:?}"),
            );
        }
        Command::Source {
            source,
            switch_vel,
            toggle_v,
        } => {
            use oxifoc_core::foc::phase::PhaseSource;
            let ps = match source {
                SourceArg::Hall => PhaseSource::Hall,
                SourceArg::HallFallback => PhaseSource::HallToObserver {
                    blend_low: switch_vel,
                    blend_high: switch_vel * 2.0,
                },
                SourceArg::Observer => PhaseSource::Observer,
                SourceArg::Hfi => PhaseSource::Hfi,
                SourceArg::HfiObserver => PhaseSource::HfiToObserver {
                    min_vel: switch_vel,
                    min_confidence: 0.5,
                },
                SourceArg::HfiObserverVolts => PhaseSource::HfiToObserverVolts {
                    toggle_v,
                    min_confidence: 0.5,
                },
            };
            runtime
                .cmd_tx
                .send(HostCommand::SetPhaseSource(ps))
                .context("send phase source command")?;
            emit(
                json,
                json!({"sent": format!("{ps:?}"), "note": "confirm via status — telemetry reports the active source"}),
                format!(
                    "Phase source command sent: {ps:?} (confirm via status — telemetry reports the active source)"
                ),
            );
            std::thread::sleep(Duration::from_millis(800));
        }
        Command::Monitor { seconds, .. } => {
            watch::run_monitor(&runtime, Duration::from_secs(seconds), json)?;
        }
        Command::Record {
            out,
            seconds,
            fast_hz,
            allow_gaps,
        } => {
            let snapshot = config_cli::config_snapshot(&runtime);
            let summary = record::record(&runtime, &out, seconds, fast_hz, snapshot)?;
            emit(
                json,
                serde_json::to_value(&summary)?,
                format!(
                    "{}: {} rows at {} Hz (M={}), {:.2} s, {} gap(s), {} sample(s) lost",
                    summary.path,
                    summary.rows,
                    summary.fast_hz_actual,
                    summary.decimation_m,
                    summary.duration_s,
                    summary.gaps,
                    summary.samples_lost,
                ),
            );
            if summary.samples_lost > 0 && !allow_gaps {
                bail!(
                    "capture has {} dropped sample(s) across {} gap(s); \
                     rerun at a lower --fast-hz or pass --allow-gaps",
                    summary.samples_lost,
                    summary.gaps
                );
            }
        }
        Command::Maneuver { action } => match action {
            ManeuverAction::Validate { .. } => unreachable!(), // handled above
            ManeuverAction::Run {
                file,
                out,
                force,
                allow_gaps,
            } => {
                let m = maneuver::load(&file)?;
                let snapshot = config_cli::config_snapshot(&runtime);
                let summary = maneuver::run(&runtime, &m, &out, force, snapshot)?;
                emit(
                    json,
                    maneuver::summary_json(&summary),
                    format!(
                        "{}: '{}' done — {} event(s), {} rows at {} Hz, {} gap(s), {} sample(s) lost",
                        summary.record.path,
                        summary.maneuver,
                        summary.events.len(),
                        summary.record.rows,
                        summary.record.fast_hz_actual,
                        summary.record.gaps,
                        summary.record.samples_lost,
                    ),
                );
                if summary.record.samples_lost > 0 && !allow_gaps {
                    bail!(
                        "capture has {} dropped sample(s); rerun at a lower fast_hz or pass --allow-gaps",
                        summary.record.samples_lost
                    );
                }
            }
        },
        Command::Config { action } => match action {
            ConfigAction::Dump { rust } => config_cli::dump_config(&runtime, rust, json)?,
            ConfigAction::Get { group } => {
                let g = config_cli::parse_group(&group)?;
                let (value, stored) = config_cli::current_value(&runtime, g)?;
                emit(
                    json,
                    json!({"group": group, "stored": stored, "value": value}),
                    format!(
                        "{group}{}: {value:#}",
                        if stored {
                            ""
                        } else {
                            " (defaults, not stored)"
                        }
                    ),
                );
            }
            ConfigAction::Set { group, fields } => {
                let g = config_cli::parse_group(&group)?;
                let value = config_cli::set_fields(&runtime, g, &fields)?;
                emit(
                    json,
                    json!({"group": group, "written": true, "value": value}),
                    format!("{group} written: {value:#}"),
                );
            }
            ConfigAction::Reset { yes } => {
                if !yes {
                    bail!("config reset erases every stored group; pass --yes to confirm");
                }
                let (tx, rx) = oxifoc_host_lib::config_channel();
                runtime
                    .cmd_tx
                    .send(HostCommand::ConfigResetAll(tx))
                    .context("send config reset")?;
                let resp = rx
                    .blocking_recv()
                    .context("backend dropped the config reset")?
                    .context("config reset failed")?;
                match resp {
                    ConfigResponse::Ok => emit(
                        json,
                        json!({"reset": true}),
                        "all stored config groups erased".to_string(),
                    ),
                    other => bail!("config reset rejected: {other:?}"),
                }
            }
        },
        Command::Detect {
            step,
            max_power_w,
            resistance,
            inductance,
            pole_pairs,
            erpm,
            apply,
            record,
        } => {
            detect::run_detect(
                &runtime,
                step,
                max_power_w,
                resistance,
                inductance,
                pole_pairs,
                erpm,
                apply,
                record,
                json,
            )?;
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

fn list_devices(json: bool) -> Result<()> {
    let ports = list_serial_ports();
    let probes = list_probes();

    if json {
        let ports_json: Vec<_> = ports
            .iter()
            .map(|p| {
                json!({
                    "port": p.to_string(),
                    "vid": p.vid,
                    "pid": p.pid,
                    "serial": p.serial_number,
                    "manufacturer": p.manufacturer,
                })
            })
            .collect();
        let probes_json: Vec<_> = probes
            .iter()
            .map(|p| {
                json!({
                    "probe": p.to_string(),
                    "identifier": p.identifier.to_string(),
                    "serial": p.serial_number,
                })
            })
            .collect();
        println!(
            "{:#}",
            json!({"serial_ports": ports_json, "probes": probes_json})
        );
        return Ok(());
    }

    println!("=== Serial Ports ===");
    if ports.is_empty() {
        println!("  (none found)");
    } else {
        for port in ports {
            println!("  {port}");
            if let (Some(vid), Some(pid)) = (port.vid, port.pid) {
                println!("    VID:PID = {vid:04x}:{pid:04x}");
            }
            if let Some(ref serial) = port.serial_number {
                println!("    Serial: {serial}");
            }
            if let Some(ref mfr) = port.manufacturer {
                println!("    Manufacturer: {mfr}");
            }
        }
    }

    println!();
    println!("=== Debug Probes (RTT) ===");
    if probes.is_empty() {
        println!("  (none found)");
    } else {
        for probe in probes {
            println!("  {probe}");
            println!("    Identifier: {}", probe.identifier);
            if let Some(ref serial) = probe.serial_number {
                println!("    Serial: {serial}");
            }
        }
    }

    Ok(())
}
