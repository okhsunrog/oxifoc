pub mod logging;

use crossbeam_channel::TryRecvError;
use logging::{LogEvent, LogLevel};
use oxifoc_host_lib::{
    list_probes, list_serial_ports, start_host, HostCommand, HostConfig, HostRuntime, TransportType,
};
use oxifoc_protocol::MotorCommand;
use serde::{Deserialize, Serialize};
use std::sync::atomic::Ordering;
use std::sync::Mutex;
use tauri::ipc::Channel;
use tauri::State;
use tracing::{debug, info, warn};

/// State wrapper for the Oxifoc host runtime
struct OxifocState(Mutex<Option<HostRuntime>>);

/// ADC sample with specta derives for TypeScript bindings.
/// Mirrors oxifoc_protocol::AdcSample but with specta support.
#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct AdcSample {
    pub ia: u16,
    pub ib: u16,
    pub ic: u16,
    pub vbus_mv: u32,
    pub fet_temp_c_x10: u16,
    pub seq: u32,
}

impl From<oxifoc_protocol::AdcSample> for AdcSample {
    fn from(s: oxifoc_protocol::AdcSample) -> Self {
        Self {
            ia: s.ia,
            ib: s.ib,
            ic: s.ic,
            vbus_mv: s.vbus_mv,
            fet_temp_c_x10: s.fet_temp_c_x10,
            seq: s.seq,
        }
    }
}

/// Motor state enum for TypeScript
#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum MotorState {
    Stopped,
    Running,
    Error,
}

// ============================================================================
// Device Discovery Types
// ============================================================================

/// Serial port information for TypeScript
#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SerialPort {
    pub path: String,
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub serial_number: Option<String>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub display_name: String,
}

impl From<oxifoc_host_lib::SerialPortInfo> for SerialPort {
    fn from(p: oxifoc_host_lib::SerialPortInfo) -> Self {
        let display_name = if let Some(ref product) = p.product {
            format!("{} ({})", p.path, product)
        } else if let (Some(vid), Some(pid)) = (p.vid, p.pid) {
            format!("{} [{:04x}:{:04x}]", p.path, vid, pid)
        } else {
            p.path.clone()
        };
        Self {
            path: p.path,
            vid: p.vid,
            pid: p.pid,
            serial_number: p.serial_number,
            manufacturer: p.manufacturer,
            product: p.product,
            display_name,
        }
    }
}

/// Debug probe information for TypeScript (RTT transport)
#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct DebugProbe {
    pub identifier: String,
    pub vid: u16,
    pub pid: u16,
    pub serial_number: Option<String>,
    pub probe_type: String,
    pub display_name: String,
}

impl From<oxifoc_host_lib::ProbeInfo> for DebugProbe {
    fn from(p: oxifoc_host_lib::ProbeInfo) -> Self {
        let display_name = if let Some(ref serial) = p.serial_number {
            format!("{} [{:04x}:{:04x}:{}]", p.probe_type, p.vid, p.pid, serial)
        } else {
            format!("{} [{:04x}:{:04x}]", p.probe_type, p.vid, p.pid)
        };
        Self {
            identifier: p.identifier,
            vid: p.vid,
            pid: p.pid,
            serial_number: p.serial_number,
            probe_type: p.probe_type,
            display_name,
        }
    }
}

/// Connection configuration from frontend
#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ConnectionConfig {
    /// Transport type: "serial" or "rtt"
    pub transport: String,
    /// Serial port path (for serial transport)
    pub serial_path: Option<String>,
    /// Baud rate (for serial transport)
    pub baud_rate: Option<u32>,
    /// Probe identifier (for RTT transport)
    pub probe: Option<String>,
    /// Chip name (for RTT transport)
    pub chip: Option<String>,
}

// ============================================================================
// Discovery Commands
// ============================================================================

/// List all available serial ports.
#[tauri::command]
#[specta::specta]
fn list_serial_ports_cmd() -> Vec<SerialPort> {
    info!("Listing serial ports...");
    let ports: Vec<SerialPort> = list_serial_ports().into_iter().map(Into::into).collect();
    info!("Found {} serial ports", ports.len());
    ports
}

/// List all available debug probes (for RTT transport).
#[tauri::command]
#[specta::specta]
fn list_probes_cmd() -> Vec<DebugProbe> {
    info!("Listing debug probes...");
    let probes: Vec<DebugProbe> = list_probes().into_iter().map(Into::into).collect();
    info!("Found {} debug probes", probes.len());
    probes
}

/// Connect to a device with the specified configuration.
#[tauri::command]
#[specta::specta]
fn connect_device(config: ConnectionConfig, state: State<OxifocState>) -> Result<(), String> {
    let mut guard = state.0.lock().map_err(|e| e.to_string())?;

    // Disconnect existing connection if any
    if guard.is_some() {
        info!("Disconnecting existing connection...");
        *guard = None;
    }

    info!("Connecting with config: {:?}", config);

    // Build HostConfig from ConnectionConfig
    let host_config = HostConfig {
        transport: Some(match config.transport.as_str() {
            "rtt" => TransportType::Rtt,
            _ => TransportType::Serial,
        }),
        serial_path: config.serial_path,
        serial_baud: config.baud_rate,
        probe: config.probe,
        chip: config.chip,
        elf: None,
        stream_defmt: Some(true),
        stream_ergot: Some(true),
    };

    let runtime = start_host(host_config);
    *guard = Some(runtime);
    info!("Device connection started");
    Ok(())
}

/// Disconnect from the current device.
#[tauri::command]
#[specta::specta]
fn disconnect_device(state: State<OxifocState>) -> Result<(), String> {
    let mut guard = state.0.lock().map_err(|e| e.to_string())?;

    if guard.is_none() {
        info!("No device connected");
        return Ok(());
    }

    info!("Disconnecting device...");
    *guard = None;
    info!("Device disconnected");
    Ok(())
}

// ============================================================================
// Connection State Commands
// ============================================================================

/// Check if the device is connected.
#[tauri::command]
#[specta::specta]
fn is_device_connected(state: State<OxifocState>) -> bool {
    state
        .0
        .lock()
        .ok()
        .and_then(|guard| {
            guard
                .as_ref()
                .map(|rt| rt.connected.load(Ordering::Relaxed))
        })
        .unwrap_or(false)
}

/// Wait for device connection with timeout (in seconds).
#[tauri::command]
#[specta::specta]
fn wait_for_device(state: State<OxifocState>, timeout_secs: u64) -> bool {
    let guard = match state.0.lock() {
        Ok(g) => g,
        Err(_) => return false,
    };

    match guard.as_ref() {
        Some(runtime) => {
            let timeout = std::time::Duration::from_secs(timeout_secs);
            runtime.wait_for_connection(timeout)
        }
        None => false,
    }
}

/// Start the motor at the specified duty cycle (0-100%).
#[tauri::command]
#[specta::specta]
fn motor_start(state: State<OxifocState>, duty: u8) -> Result<(), String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    let duty = duty.min(100);
    info!("Starting motor at {}% duty", duty);

    runtime
        .cmd_tx
        .send(HostCommand::Motor(MotorCommand::Start { duty }))
        .map_err(|e| format!("Failed to send command: {}", e))
}

/// Stop the motor.
#[tauri::command]
#[specta::specta]
fn motor_stop(state: State<OxifocState>) -> Result<(), String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    info!("Stopping motor");

    runtime
        .cmd_tx
        .send(HostCommand::Motor(MotorCommand::Stop))
        .map_err(|e| format!("Failed to send command: {}", e))
}

/// Set motor speed (duty cycle 0-100%) while running.
#[tauri::command]
#[specta::specta]
fn motor_set_speed(state: State<OxifocState>, duty: u8) -> Result<(), String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    let duty = duty.min(100);
    debug!("Setting motor speed to {}%", duty);

    runtime
        .cmd_tx
        .send(HostCommand::Motor(MotorCommand::SetSpeed { duty }))
        .map_err(|e| format!("Failed to send command: {}", e))
}

/// Set ADC poll rate in Hz (0 = disabled, 1-255 = rate).
/// Controls how often the host polls the device for ADC samples.
#[tauri::command]
#[specta::specta]
fn set_adc_poll_rate(state: State<OxifocState>, rate_hz: u8) -> Result<(), String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    info!("Setting ADC poll rate to {}Hz", rate_hz);

    runtime
        .cmd_tx
        .send(HostCommand::SetAdcPollRate(rate_hz))
        .map_err(|e| format!("Failed to send command: {}", e))
}

/// Start streaming ADC samples to the frontend via the provided channel.
/// Spawns a background thread that forwards samples from the host runtime.
#[tauri::command]
#[specta::specta]
fn start_adc_stream(channel: Channel<AdcSample>, state: State<OxifocState>) -> Result<(), String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    // Clone the receiver - note: crossbeam channels support multiple receivers
    let adc_rx = runtime.adc_rx.clone();

    info!("Starting ADC stream to frontend");

    // Spawn a thread to forward ADC samples
    std::thread::spawn(move || {
        loop {
            match adc_rx.recv() {
                Ok(sample) => {
                    let tauri_sample: AdcSample = sample.into();
                    if channel.send(tauri_sample).is_err() {
                        // Channel closed (frontend disconnected)
                        debug!("ADC stream channel closed");
                        break;
                    }
                }
                Err(_) => {
                    // Sender dropped (host backend stopped)
                    warn!("ADC receiver disconnected");
                    break;
                }
            }
        }
    });

    Ok(())
}

/// Get a single ADC sample (non-blocking).
/// Returns None if no sample is available.
#[tauri::command]
#[specta::specta]
fn get_adc_sample(state: State<OxifocState>) -> Result<Option<AdcSample>, String> {
    let guard = state.0.lock().map_err(|e| e.to_string())?;
    let runtime = guard.as_ref().ok_or("Host not initialized")?;

    match runtime.adc_rx.try_recv() {
        Ok(sample) => Ok(Some(sample.into())),
        Err(TryRecvError::Empty) => Ok(None),
        Err(TryRecvError::Disconnected) => Err("ADC receiver disconnected".to_string()),
    }
}

// ============================================================================
// Logging Commands
// ============================================================================

/// Set the host log level at runtime
#[tauri::command]
#[specta::specta]
fn set_host_log_level(level: LogLevel) -> Result<(), String> {
    logging::set_host_log_level(level)
}

/// Set the device log level at runtime
#[tauri::command]
#[specta::specta]
fn set_device_log_level(level: LogLevel) -> Result<(), String> {
    logging::set_device_log_level(level)
}

/// Get current log levels (host, device)
#[tauri::command]
#[specta::specta]
fn get_log_levels() -> (LogLevel, LogLevel) {
    logging::get_log_levels()
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    // Initialize Specta builder
    let specta_builder = tauri_specta::Builder::<tauri::Wry>::new()
        .commands(tauri_specta::collect_commands![
            // Discovery
            list_serial_ports_cmd,
            list_probes_cmd,
            // Connection
            connect_device,
            disconnect_device,
            is_device_connected,
            wait_for_device,
            // Motor control
            motor_start,
            motor_stop,
            motor_set_speed,
            // ADC streaming
            start_adc_stream,
            get_adc_sample,
            set_adc_poll_rate,
            // Logging
            set_host_log_level,
            set_device_log_level,
            get_log_levels,
        ])
        .events(tauri_specta::collect_events![LogEvent]);

    // Export TypeScript bindings in debug mode
    #[cfg(debug_assertions)]
    {
        specta_builder
            .export(
                specta_typescript::Typescript::default()
                    .formatter(specta_typescript::formatter::prettier)
                    .header("/* eslint-disable */\n// @ts-nocheck")
                    .bigint(specta_typescript::BigIntExportBehavior::Number),
                "../src/bindings.ts",
            )
            .expect("Failed to export Specta typescript bindings");
    }

    // Build the Tauri application
    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .plugin(tauri_plugin_log::Builder::new().skip_logger().build())
        .manage(OxifocState(Mutex::new(None)))
        .invoke_handler(specta_builder.invoke_handler())
        .setup(move |app| {
            // Initialize our custom tracing/logging
            logging::init_tracing(app.handle().clone());

            // Mount Specta events
            specta_builder.mount_events(app);

            // Delay startup logs to give frontend time to initialize
            std::thread::spawn(|| {
                std::thread::sleep(std::time::Duration::from_millis(2000));
                info!("Logging initialized.");
                info!("Specta events mounted.");
                debug!("Application is starting up...");
                info!("Welcome to Oxifoc!");
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("Error while running Tauri application");
}
