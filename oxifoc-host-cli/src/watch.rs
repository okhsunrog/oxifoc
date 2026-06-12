//! Live telemetry watchers: `faults --watch` and `monitor`.

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use oxifoc_host_lib::HostRuntime;

/// `faults --watch`: print every fault snapshot the device pushes on the
/// fault topic (raise / payload refinement / clear). The device sends the
/// FULL list each time, so each printed line is a self-contained state.
pub fn watch_faults(runtime: &HostRuntime, seconds: u64, json: bool) -> Result<()> {
    use crossbeam_channel::RecvTimeoutError;

    if !json {
        if seconds == 0 {
            println!("Watching fault events (Ctrl-C to stop)...");
        } else {
            println!("Watching fault events for {seconds}s...");
        }
    }
    let deadline = (seconds > 0).then(|| Instant::now() + Duration::from_secs(seconds));
    loop {
        if let Some(d) = deadline
            && Instant::now() >= d
        {
            break;
        }
        match runtime.fault_rx.recv_timeout(Duration::from_millis(500)) {
            Ok(snapshot) => {
                if json {
                    // JSONL: one compact object per event
                    println!("{}", serde_json::to_string(&snapshot)?);
                } else if snapshot.faults.is_empty() {
                    println!("faults cleared (total=0)");
                } else {
                    let lines: Vec<String> = snapshot
                        .faults
                        .iter()
                        .map(|f| format!("{:?} [{:?}]: {}", f.category, f.severity, f.details))
                        .collect();
                    println!(
                        "{} active fault(s):\n  {}",
                        snapshot.total,
                        lines.join("\n  ")
                    );
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                if !runtime.connected.load(Ordering::Relaxed) {
                    eprintln!("Waiting for device connection...");
                }
            }
            Err(RecvTimeoutError::Disconnected) => {
                bail!("Fault event channel disconnected");
            }
        }
    }
    Ok(())
}

pub fn run_monitor(runtime: &HostRuntime, duration: Duration, json: bool) -> Result<()> {
    use crossbeam_channel::RecvTimeoutError;

    if !json {
        println!("Streaming telemetry for {duration:?}...");
    }
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        // Print fast telemetry
        match runtime.fast_rx.recv_timeout(Duration::from_millis(500)) {
            Ok(sample) => {
                if json {
                    // JSONL: one compact object per sample
                    println!("{}", serde_json::to_string(&sample)?);
                } else {
                    println!(
                        "#{:>5} ia:{:>7.2}A ib:{:>7.2}A ic:{:>7.2}A id:{:>7.2}A iq:{:>7.2}A erpm:{:>6}",
                        sample.seq,
                        sample.ia,
                        sample.ib,
                        sample.ic,
                        sample.id,
                        sample.iq,
                        sample.erpm,
                    );
                }
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
            if json {
                println!("{}", serde_json::to_string(&slow)?);
            } else {
                println!(
                    "  [sys] vbus:{:.1}V fet:{:.1}°C motor:{:.1}°C faults:{}",
                    slow.vbus_mv as f32 / 1000.0,
                    f32::from(slow.fet_temp_c_x10) / 10.0,
                    f32::from(slow.motor_temp_c_x10) / 10.0,
                    slow.fault_count,
                );
            }
        }
    }

    Ok(())
}
