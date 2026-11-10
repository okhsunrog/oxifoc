//! Motor-inhibited full-route telemetry and simultaneous command qualification.
use anyhow::{Result, ensure};
use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::{HostCommand, HostConfig, TransportType, motor_channel, start_host};
use serde_json::json;
use std::time::{Duration, Instant};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 3,
        "usage: bridge_handshake SERIAL_PATH SECONDS FAST_HZ"
    );
    let seconds: u64 = args[1].parse()?;
    let hz: u16 = args[2].parse()?;
    ensure!(
        (1..=120).contains(&seconds),
        "duration must be 1..120 seconds"
    );
    ensure!([0, 5000, 20000].contains(&hz), "use 0, 5000 or 20000 Hz");
    let runtime = start_host(HostConfig {
        transport: Some(TransportType::UsbSerialJtag),
        serial_path: Some(args[0].clone()),
        fast_hz: Some(0),
        ..HostConfig::default()
    });
    ensure!(
        runtime.wait_for_connection(Duration::from_secs(12)),
        "handshake failed"
    );
    let initial = runtime.slow_rx.recv_timeout(Duration::from_secs(3))?;
    ensure!(
        initial.control_mode == ControlMode::Stopped && initial.fault_count == 0,
        "controller must be stopped without faults"
    );
    let ack = oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, hz)?;
    let step = u16::try_from(ack.decimation_m)?;
    // Report the ordinary stream-enable transient separately; the existing
    // recorder uses the same 150-ms settling interval for the legacy stream.
    let warmup = Instant::now();
    let mut warmup_rows = 0u64;
    let mut warmup_gaps = 0u64;
    let mut warmup_lost = 0u64;
    let mut last: Option<u16> = None;
    while warmup.elapsed() < Duration::from_millis(150) {
        for sample in runtime.fast_rx.try_iter() {
            if let Some(previous) = last {
                let delta = sample.seq.wrapping_sub(previous);
                if delta != step {
                    warmup_gaps += 1;
                    warmup_lost += u64::from(delta.saturating_sub(step) / step.max(1));
                }
            }
            last = Some(sample.seq);
            warmup_rows += 1;
        }
        std::thread::sleep(Duration::from_micros(100));
    }
    let started = Instant::now();
    let mut next_command = started;
    let mut pending = None;
    let mut commands = 0u64;
    let mut max_ack_us = 0u128;
    let mut rows = 0u64;
    let mut gaps = 0u64;
    let mut lost = 0u64;
    let mut previous: Option<u16> = last;
    let result: Result<()> = (|| {
        while started.elapsed() < Duration::from_secs(seconds) || pending.is_some() {
            for sample in runtime.fast_rx.try_iter() {
                if let Some(last) = previous {
                    let delta = sample.seq.wrapping_sub(last);
                    if delta != step {
                        gaps += 1;
                        lost += u64::from(delta.saturating_sub(step) / step.max(1));
                    }
                }
                previous = Some(sample.seq);
                rows += 1;
            }
            for sample in runtime.slow_rx.try_iter() {
                ensure!(
                    sample.control_mode == ControlMode::Stopped && sample.fault_count == 0,
                    "controller left Stopped or reported a fault"
                );
            }
            if pending.is_none()
                && Instant::now() >= next_command
                && started.elapsed() < Duration::from_secs(seconds)
            {
                let (tx, rx) = motor_channel();
                runtime
                    .cmd_tx
                    .send(HostCommand::MotorAck(ControlMode::Stopped, tx))?;
                pending = Some((Instant::now(), rx));
                next_command += Duration::from_millis(50);
            }
            if let Some((sent, rx)) = &mut pending {
                if let Ok(reply) = rx.try_recv() {
                    reply.map_err(|e| anyhow::anyhow!("Stop rejected: {e:?}"))?;
                    max_ack_us = max_ack_us.max(sent.elapsed().as_micros());
                    commands += 1;
                    pending = None;
                } else {
                    ensure!(sent.elapsed() < Duration::from_secs(2), "Stop ACK timeout");
                }
            }
            std::thread::sleep(Duration::from_micros(100));
        }
        Ok(())
    })();
    // Always request a disabled stream, including when a measurement fails.
    let stopped = oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, 0);
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "seconds": seconds, "fast_hz": ack.actual_fast_hz, "decimation_m": step, "rows": rows,
            "gaps": gaps, "samples_lost": lost, "commands": commands,
        "warmup_rows": warmup_rows, "warmup_gaps": warmup_gaps, "warmup_samples_lost": warmup_lost,
            "max_ack_us": max_ack_us, "stream_stopped": stopped.is_ok(),
            "error": result.as_ref().err().map(ToString::to_string)
        }))?
    );
    result?;
    stopped?;
    ensure!(gaps == 0, "telemetry gaps detected");
    ensure!(
        hz == 0 || rows >= u64::from(hz) * seconds * 9 / 10,
        "too few samples"
    );
    Ok(())
}
