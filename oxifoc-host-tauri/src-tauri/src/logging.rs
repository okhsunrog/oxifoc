use serde::Serialize;
use tauri::{AppHandle, Emitter}; // Added Manager for potential future use
use tracing_subscriber::{prelude::*, EnvFilter};

// Define the event structure for sending logs to the frontend
#[derive(Debug, Clone, Serialize, specta::Type, tauri_specta::Event)]
pub struct LogEvent {
    pub message: String,
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

/// Initializes the tracing subscriber based on build configuration and platform.
///
/// Sets up filtering and multiple writers:
/// - TauriWriter: Sends logs to the frontend via events.
/// - Stdout (non-Android): Writes logs to the console.
/// - Logcat (Android): Writes logs to Android's logcat.
pub fn init_tracing(app_handle: AppHandle) {
    // Configure the filter with different log levels based on build type
    // Start with default environment variable settings (RUST_LOG)
    let filter = EnvFilter::from_default_env();

    // Debug build configuration (more verbose)
    #[cfg(debug_assertions)]
    let filter = filter
        .add_directive("oxifoc_host_tauri_lib=trace".parse().unwrap())
        .add_directive("webview=debug".parse().unwrap()) // Less verbose webview usually
        .add_directive("tauri=info".parse().unwrap()) // Less verbose tauri usually
        .add_directive("debug".parse().unwrap()); // Catch-all for other debug logs

    // Release build configuration (less verbose)
    #[cfg(not(debug_assertions))]
    let filter = filter
        .add_directive("oxifoc_host_tauri_lib=info".parse().unwrap())
        .add_directive("warn".parse().unwrap()); // Catch-all for warn and error

    // Create a writer factory for Tauri with ANSI colors enabled
    // Cloning app_handle is cheap (it's Arc-based)
    let tauri_writer_factory = move || TauriWriter {
        app_handle: app_handle.clone(),
    };

    #[cfg(target_os = "android")]
    {
        // Temporarily disable logcat integration until tracing_logcat is added.
        // Uncomment the block below once the dependency is available.
        /*
        use tracing_logcat::{LogcatMakeWriter, LogcatTag};
        let tag = LogcatTag::Fixed("FwupdGui".to_owned());
        let logcat_writer = LogcatMakeWriter::new(tag).expect("Failed to initialize logcat writer");
        let logcat_layer = tracing_subscriber::fmt::layer()
            .with_ansi(false)
            .with_writer(logcat_writer);
        */

        let tauri_layer = tracing_subscriber::fmt::layer()
            .with_ansi(true)
            .with_writer(tauri_writer_factory);

        tracing_subscriber::registry()
            .with(filter)
            // .with(logcat_layer) // Re-enable when tracing_logcat is added
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
            .with(filter)
            .with(stdout_layer)
            .with(tauri_layer)
            .init();
    }
}
