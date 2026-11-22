//! `diag` subcommand: controller diagnostics over `req/diag` (ISR timing,
//! totals since boot, reset cause) — what an assembled board can report
//! without a debug probe.

use std::time::{Duration, Instant};

use anyhow::Result;
use clap::Subcommand;
use oxifoc_core::types::diag::{BridgeDiag, CrashRecord, DiagSummary, IsrWindow};
use oxifoc_host_lib::{HostRuntime, ops::diag as ops};

use crate::emit;

#[derive(Subcommand)]
pub enum DiagAction {
    /// Totals since boot and the reset cause
    Summary,
    /// The last completed one-second ISR window
    Isr {
        /// Keep printing each new window for this many seconds
        #[arg(long, default_value_t = 0)]
        watch: u64,
    },
    /// The bridge's own link counters and crash record; works without the controller
    Bridge {
        /// Forget the bridge's crash record after printing it
        #[arg(long)]
        clear_crash: bool,
    },
    /// The last panic/HardFault record (kept across resets until cleared)
    Crash {
        /// Forget the record after printing it
        #[arg(long)]
        clear: bool,
    },
}

pub fn run(runtime: &HostRuntime, action: DiagAction, json: bool) -> Result<()> {
    match action {
        DiagAction::Summary => {
            let summary = ops::summary(&runtime.cmd_tx)?;
            emit(
                json,
                serde_json::to_value(summary)?,
                format_summary(&summary),
            );
        }
        DiagAction::Bridge { clear_crash } => {
            let bridge = ops::bridge(&runtime.cmd_tx)?;
            let crash = ops::bridge_crash(&runtime.cmd_tx)?;
            let mut value = serde_json::to_value(bridge)?;
            value["crash"] = serde_json::to_value(&crash)?;
            let human = format!(
                "{}\ncrash: {}",
                format_bridge(&bridge),
                crash
                    .as_ref()
                    .map_or_else(|| "none".to_string(), format_bridge_crash)
            );
            emit(json, value, human);
            if clear_crash && crash.is_some() {
                ops::clear_bridge_crash(&runtime.cmd_tx)?;
            }
        }
        DiagAction::Crash { clear } => {
            let record = ops::crash(&runtime.cmd_tx)?;
            let human = record
                .as_ref()
                .map_or_else(|| "no crash recorded".to_string(), format_crash);
            emit(json, serde_json::to_value(&record)?, human);
            if clear && record.is_some() {
                ops::clear_crash(&runtime.cmd_tx)?;
            }
        }
        DiagAction::Isr { watch } => {
            let deadline = Instant::now() + Duration::from_secs(watch);
            let mut last = None;
            loop {
                match ops::isr_window(&runtime.cmd_tx)? {
                    Some(window) if last != Some(window.window) => {
                        last = Some(window.window);
                        emit(json, serde_json::to_value(window)?, format_window(&window));
                    }
                    Some(_) => {}
                    None if watch == 0 => {
                        emit(json, serde_json::Value::Null, "no ISR window yet".into());
                    }
                    None => {}
                }
                if Instant::now() >= deadline {
                    break;
                }
                std::thread::sleep(Duration::from_millis(400));
            }
        }
    }
    Ok(())
}

fn format_summary(s: &DiagSummary) -> String {
    let r = &s.reset;
    let reset: Vec<&str> = [
        (r.power_on, "power-on"),
        (r.brown_out, "brown-out"),
        (r.pin, "pin"),
        (r.software, "software"),
        (r.independent_watchdog, "IWDG"),
        (r.window_watchdog, "WWDG"),
        (r.low_power, "low-power"),
    ]
    .into_iter()
    .filter_map(|(set, name)| set.then_some(name))
    .collect();
    format!(
        "boot_epoch={:#x} uptime={:.1}s reset=[{}]\n\
         windows={} isr: overruns={} max={} long_intervals={} short_intervals={}\n\
         telemetry: pushed={} dropped={}  hall: edges={} overcaptures={}",
        s.boot_epoch,
        s.uptime_ms as f64 / 1000.0,
        reset.join(","),
        s.windows,
        s.isr_overruns,
        s.isr_max_cycles,
        s.isr_long_intervals,
        s.isr_short_intervals,
        s.telemetry_pushed,
        s.telemetry_dropped,
        s.hall_edges,
        s.hall_overcaptures,
    )
}

fn format_bridge(b: &BridgeDiag) -> String {
    let mut out = format!(
        "bridge uptime={:.1}s reset_reason={:#04x}\n\
         spi: slots={} errors={} (length={} crc={} header={}) controller_boots={}\n\
         usb: dropped={} write_errors={} max_grant={}",
        b.uptime_ms as f64 / 1000.0,
        b.reset_reason,
        b.spi.slots,
        b.spi.errors,
        b.spi.length_errors,
        b.spi.crc_errors,
        b.spi.header_errors,
        b.spi.peer_boots,
        b.usb.dropped,
        b.usb.write_errors,
        b.usb.max_grant,
    );
    for (link, l) in b.ble.iter().enumerate() {
        out.push_str(&format!(
            "\nble{link}: {} connects={} disconnects={} last_reason={:#04x} \
             untrusted_drops={} notify_errors={} dropped={}",
            match (l.connected, l.trusted) {
                (false, _) => "free",
                (true, false) => "connected",
                (true, true) => "trusted",
            },
            l.connects,
            l.disconnects,
            l.last_disconnect_reason,
            l.untrusted_drops,
            l.notify_errors,
            l.dropped,
        ));
    }
    out.push_str(&format!(
        "\nwifi: {} connects={} dropped={} max_backlog={}",
        if b.wifi.connected {
            "connected"
        } else {
            "free"
        },
        b.wifi.connects,
        b.wifi.dropped,
        b.wifi.max_backlog,
    ));
    out
}

fn format_crash(c: &CrashRecord) -> String {
    format!(
        "{:?} {} boot(s) ago at uptime {:.3}s\n  {}:{} {}\n  pc={:#010x} lr={:#010x} cfsr={:#010x} hfsr={:#010x}",
        c.kind,
        c.boots_since,
        f64::from(c.uptime_ms) / 1000.0,
        c.file,
        c.line,
        c.message,
        c.pc,
        c.lr,
        c.cfsr,
        c.hfsr,
    )
}

/// The bridge records panics only, so the fault registers are left out.
fn format_bridge_crash(c: &CrashRecord) -> String {
    format!(
        "{:?} {} boot(s) ago at uptime {:.3}s\n  {}:{} {}",
        c.kind,
        c.boots_since,
        f64::from(c.uptime_ms) / 1000.0,
        c.file,
        c.line,
        c.message,
    )
}

fn format_window(w: &IsrWindow) -> String {
    let avg = |v: Option<u32>| v.map_or_else(|| "-".to_string(), |v| v.to_string());
    let load = w
        .load_percent()
        .map_or_else(|| "-".to_string(), |p| format!("{p}%"));
    format!(
        "#{} isr: n={} avg={} max={} over={} load={} | motion: n={} avg={} max={} | \
         interval: avg={} min={} max={} long={} short={} budget={} | \
         telem: push={} drop={} read={} | hall: state={:03b} edges={} overcap={}",
        w.window,
        w.isr.samples,
        avg(w.isr.average()),
        w.isr.max_cycles,
        w.overruns,
        load,
        w.motion.samples,
        avg(w.motion.average()),
        w.motion.max_cycles,
        avg(w.intervals.average()),
        w.intervals.min_cycles,
        w.intervals.max_cycles,
        w.intervals.long,
        w.intervals.short,
        w.budget_cycles,
        w.telemetry.pushed,
        w.telemetry.dropped,
        w.telemetry.read,
        w.hall.state,
        w.hall.edges,
        w.hall.overcaptures,
    )
}
