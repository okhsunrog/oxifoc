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
        // Poll at 500 ms (deadline re-check + connection heartbeat), but
        // never block past the deadline: the fault channel is event-driven
        // and usually idle, so a flat 500 ms wait would overshoot a bounded
        // watch by nearly that on every run.
        let wait = match deadline {
            Some(d) => match d.checked_duration_since(Instant::now()) {
                Some(rem) => rem.min(Duration::from_millis(500)),
                None => break,
            },
            None => Duration::from_millis(500),
        };
        match runtime.fault_rx.recv_timeout(wait) {
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
    // Enrichment context (device BoardCalib + dc_offsets + pole_pairs). When
    // absent (no handshake/calib) we fall back to printing raw ADC counts.
    let hw = crate::record::latest_hw_info(runtime);
    let ctx = oxifoc_host_lib::build_enrich_ctx(&runtime.cmd_tx, hw.as_ref());
    let deadline = Instant::now() + duration;
    // Clamp each wait to the time left so a stalled stream can't overshoot
    // the requested window by up to the 500 ms poll interval.
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        let wait = remaining.min(Duration::from_millis(500));
        // Print fast telemetry
        match runtime.fast_rx.recv_timeout(wait) {
            Ok(sample) => {
                let rich = ctx.as_ref().map(|c| sample.enrich(c));
                if json {
                    // JSONL: enriched when we have calibration, else the raw frame.
                    match &rich {
                        Some(r) => println!("{}", serde_json::to_string(r)?),
                        None => println!("{}", serde_json::to_string(&sample)?),
                    }
                } else if let Some(r) = &rich {
                    println!(
                        "#{:>5} i[a{:+.2} b{:+.2} c{:+.2}] dq[{:+.2} {:+.2}]A  \
                         vbus{:.1} vdq[{:+.2} {:+.2}]V  θ{:+.2} {:.0}rpm",
                        r.seq, r.ia, r.ib, r.ic, r.id, r.iq, r.vbus_v, r.vd, r.vq, r.angle_rad,
                        r.mech_rpm,
                    );
                } else {
                    println!(
                        "#{:>5} ia:{:>5} ib:{:>5} ic:{:>5}  (raw ADC counts — no calib)",
                        sample.seq, sample.ia, sample.ib, sample.ic,
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
