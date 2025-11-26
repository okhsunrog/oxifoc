pub mod logging;

use crossbeam_channel::TryRecvError;
use logging::LogEvent;
use oxifoc_host_lib::{start_host, HostCommand, HostConfig, HostRuntime};
use oxifoc_protocol::MotorCommand;
use serde::Serialize;
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

/// Initialize connection to the Oxifoc device.
/// This starts the host backend which connects to the serial port.
#[tauri::command]
#[specta::specta]
fn init_device_connection(state: State<OxifocState>) -> Result<(), String> {
    let mut guard = state.0.lock().map_err(|e| e.to_string())?;

    if guard.is_some() {
        info!("Device connection already initialized");
        return Ok(());
    }

    info!("Initializing device connection...");
    let cfg = HostConfig::load_default().unwrap_or_default();
    let runtime = start_host(cfg);
    *guard = Some(runtime);
    info!("Host backend started");
    Ok(())
}

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

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    // Initialize Specta builder
    let specta_builder = tauri_specta::Builder::<tauri::Wry>::new()
        .commands(tauri_specta::collect_commands![
            init_device_connection,
            is_device_connected,
            wait_for_device,
            motor_start,
            motor_stop,
            motor_set_speed,
            start_adc_stream,
            get_adc_sample,
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
