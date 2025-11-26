use serde::{Deserialize, Serialize};
use std::sync::OnceLock;
use tauri::{AppHandle, Emitter};
use tracing_subscriber::{
    filter::LevelFilter,
    layer::SubscriberExt,
    reload::{self, Handle},
    util::SubscriberInitExt,
    EnvFilter, Registry,
};

// Define the event structure for sending logs to the frontend
#[derive(Debug, Clone, Serialize, specta::Type, tauri_specta::Event)]
pub struct LogEvent {
    pub message: String,
}

/// Log level enum for frontend/backend communication
#[derive(Debug, Clone, Copy, Serialize, Deserialize, specta::Type, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Off,
}

impl LogLevel {
    fn to_level_filter(self) -> LevelFilter {
        match self {
            LogLevel::Trace => LevelFilter::TRACE,
            LogLevel::Debug => LevelFilter::DEBUG,
            LogLevel::Info => LevelFilter::INFO,
            LogLevel::Warn => LevelFilter::WARN,
            LogLevel::Error => LevelFilter::ERROR,
            LogLevel::Off => LevelFilter::OFF,
        }
    }
}

// Custom writer that forwards logs to the Tauri frontend via events
struct TauriWriter {
    app_handle: AppHandle,
}

impl std::io::Write for TauriWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        // Attempt to convert bytes to a UTF-8 string
        if let Ok(message) = String::from_utf8(buf.to_vec()) {
            // Check if the message is non-empty after trimming whitespace
            // (This prevents sending empty lines or lines with only whitespace)
            if !message.trim().is_empty() {
                // Emit the log event to the frontend with the ORIGINAL message
                // which should include the newline added by the formatter.
                let _ = self.app_handle.emit(
                    "log-event", // Event name must match the one defined in specta/frontend
                    LogEvent { message },
                );
            }
        }
        // Always report that all bytes were "written"
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        // No buffering, so flush is a no-op
        Ok(())
    }
}

// Global state for filter reload handles
struct FilterHandles {
    host_level: LogLevel,
    device_level: LogLevel,
    reload_handle: Handle<EnvFilter, Registry>,
}

static FILTER_HANDLES: OnceLock<std::sync::Mutex<FilterHandles>> = OnceLock::new();

/// Build an EnvFilter based on current log levels
fn build_filter(host_level: LogLevel, device_level: LogLevel) -> EnvFilter {
    let host_filter = host_level.to_level_filter();
    let device_filter = device_level.to_level_filter();

    // Start with environment variable overrides
    let mut filter = EnvFilter::from_default_env();

    // Add host-related directives
    if host_filter != LevelFilter::OFF {
        filter = filter
            .add_directive(
                format!("oxifoc_host_tauri_lib={}", host_filter)
                    .parse()
                    .unwrap(),
            )
            .add_directive(format!("oxifoc_host_lib={}", host_filter).parse().unwrap());
    }

    // Add device directive
    if device_filter != LevelFilter::OFF {
        filter = filter.add_directive(format!("device={}", device_filter).parse().unwrap());
    }

    // Limit noisy USB enumeration crates to info level max
    filter = filter
        .add_directive("nusb=info".parse().unwrap())
        .add_directive("probe_rs=info".parse().unwrap());

    // Suppress some framework noise
    filter = filter
        .add_directive("webview=warn".parse().unwrap())
        .add_directive("tauri=info".parse().unwrap())
        .add_directive("tao=warn".parse().unwrap())
        .add_directive("wry=warn".parse().unwrap());

    // Catch-all based on the more verbose of the two levels
    let catch_all = std::cmp::max(host_filter, device_filter);
    if catch_all != LevelFilter::OFF {
        filter = filter.add_directive(catch_all.into());
    }

    filter
}

/// Set the host log level at runtime
pub fn set_host_log_level(level: LogLevel) -> Result<(), String> {
    let handles = FILTER_HANDLES
        .get()
        .ok_or("Logging not initialized")?
        .lock()
        .map_err(|e| e.to_string())?;

    let mut handles = handles;
    handles.host_level = level;

    let new_filter = build_filter(handles.host_level, handles.device_level);
    handles
        .reload_handle
        .reload(new_filter)
        .map_err(|e| e.to_string())?;

    tracing::info!("Host log level changed to {:?}", level);
    Ok(())
}

/// Set the device log level at runtime
pub fn set_device_log_level(level: LogLevel) -> Result<(), String> {
    let handles = FILTER_HANDLES
        .get()
        .ok_or("Logging not initialized")?
        .lock()
        .map_err(|e| e.to_string())?;

    let mut handles = handles;
    handles.device_level = level;

    let new_filter = build_filter(handles.host_level, handles.device_level);
    handles
        .reload_handle
        .reload(new_filter)
        .map_err(|e| e.to_string())?;

    tracing::info!("Device log level changed to {:?}", level);
    Ok(())
}

/// Get current log levels
pub fn get_log_levels() -> (LogLevel, LogLevel) {
    FILTER_HANDLES
        .get()
        .and_then(|h| h.lock().ok())
        .map(|h| (h.host_level, h.device_level))
        .unwrap_or((LogLevel::Info, LogLevel::Info))
}

/// Initializes the tracing subscriber based on build configuration and platform.
///
/// Sets up filtering with reload capability and multiple writers:
/// - TauriWriter: Sends logs to the frontend via events.
/// - Stdout (non-Android): Writes logs to the console.
/// - Logcat (Android): Writes logs to Android's logcat.
pub fn init_tracing(app_handle: AppHandle) {
    // Default log levels
    #[cfg(debug_assertions)]
    let (host_level, device_level) = (LogLevel::Debug, LogLevel::Trace);
    #[cfg(not(debug_assertions))]
    let (host_level, device_level) = (LogLevel::Info, LogLevel::Info);

    // Build initial filter
    let initial_filter = build_filter(host_level, device_level);

    // Create reloadable filter layer
    let (filter_layer, reload_handle) = reload::Layer::new(initial_filter);

    // Store the reload handle globally
    if FILTER_HANDLES
        .set(std::sync::Mutex::new(FilterHandles {
            host_level,
            device_level,
            reload_handle,
        }))
        .is_err()
    {
        panic!("Filter handles already initialized");
    }

    // Create a writer factory for Tauri with ANSI colors enabled
    // Cloning app_handle is cheap (it's Arc-based)
    let tauri_writer_factory = move || TauriWriter {
        app_handle: app_handle.clone(),
    };

    #[cfg(target_os = "android")]
    {
        use tracing_logcat::{LogcatMakeWriter, LogcatTag};

        // Logcat layer for Android logging (viewable via `adb logcat`)
        let tag = LogcatTag::Fixed("Oxifoc".to_owned());
        let logcat_writer = LogcatMakeWriter::new(tag).expect("Failed to initialize logcat writer");
        let logcat_layer = tracing_subscriber::fmt::layer()
            .with_ansi(false) // Logcat doesn't support ANSI
            .with_writer(logcat_writer);

        // Tauri layer for frontend terminal (with ANSI colors)
        let tauri_layer = tracing_subscriber::fmt::layer()
            .with_ansi(true)
            .with_writer(tauri_writer_factory);

        tracing_subscriber::registry()
            .with(filter_layer)
            .with(logcat_layer)
            .with(tauri_layer)
            .init();
    }

    #[cfg(not(target_os = "android"))]
    {
        // Layer for writing to standard output (with ANSI colors)
        let stdout_layer = tracing_subscriber::fmt::layer()
            .with_ansi(true)
            .with_writer(std::io::stdout);

        // Layer for sending logs to the Tauri frontend (with ANSI colors)
        let tauri_layer = tracing_subscriber::fmt::layer()
            .with_ansi(true)
            .with_writer(tauri_writer_factory);

        // Combine layers and initialize the subscriber
        tracing_subscriber::registry()
            .with(filter_layer)
            .with(stdout_layer)
            .with(tauri_layer)
            .init();
    }
}
