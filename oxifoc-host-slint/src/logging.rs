//! Logging UI integration.

use super::{App, LogMessage, MAX_LOG_LINES};
use slint::ComponentHandle;
use slint::Model;
use slint::{ModelRc, SharedString, VecModel};
use std::thread;
use tracing_subscriber::Layer;

/// Tracing layer that sends log messages to a crossbeam channel for the UI.
pub(super) struct UiLogLayer {
    pub(super) tx: crossbeam_channel::Sender<(String, i32)>,
}

impl<S: tracing::Subscriber> Layer<S> for UiLogLayer {
    fn on_event(
        &self,
        event: &tracing::Event<'_>,
        _ctx: tracing_subscriber::layer::Context<'_, S>,
    ) {
        let level = match *event.metadata().level() {
            tracing::Level::TRACE => 0,
            tracing::Level::DEBUG => 1,
            tracing::Level::INFO => 2,
            tracing::Level::WARN => 3,
            tracing::Level::ERROR => 4,
        };

        // Format the message
        let mut visitor = MessageVisitor(String::new());
        event.record(&mut visitor);

        let target = event.metadata().target();
        let prefix = match level {
            3 => "WARN ",
            4 => "ERROR",
            1 => "DEBUG",
            0 => "TRACE",
            _ => "INFO ",
        };
        let line = if target == "device" {
            format!("[{prefix}] [device] {}", visitor.0)
        } else {
            format!("[{prefix}] {}", visitor.0)
        };

        let _ = self.tx.try_send((line, level));
    }
}

struct MessageVisitor(String);

impl tracing::field::Visit for MessageVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        if field.name() == "message" {
            self.0 = value.to_string();
        }
    }
}

pub(super) fn bind(app: &App, log_rx: crossbeam_channel::Receiver<(String, i32)>) {
    {
        let weak = app.as_weak();
        thread::spawn(move || {
            while let Ok((text, level)) = log_rx.recv() {
                let text = SharedString::from(&text);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    let model = app.get_log_messages();
                    let vec_model = model
                        .as_any()
                        .downcast_ref::<VecModel<LogMessage>>()
                        .unwrap();
                    vec_model.push(LogMessage { text, level });
                    // Batch-trim old messages to avoid O(n) per removal
                    if vec_model.row_count() > MAX_LOG_LINES {
                        let to_remove = vec_model.row_count() - MAX_LOG_LINES + MAX_LOG_LINES / 4;
                        for _ in 0..to_remove {
                            vec_model.remove(0);
                        }
                    }
                });
            }
        });
    }

    {
        let weak = app.as_weak();
        app.on_clear_log(move || {
            if let Some(app) = weak.upgrade() {
                app.set_log_messages(ModelRc::new(VecModel::<LogMessage>::default()));
            }
        });
    }
}
