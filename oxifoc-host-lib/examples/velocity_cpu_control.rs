//! No-fast-stream negative control for the secured MK5 velocity benchmark.
//! Explicit --run is required. Same +40 erad/s, 200-ms refresh, Coast and Stop
//! timeline as the recorded five-second hold; never writes configuration.

use anyhow::{Result, bail, ensure};
use oxifoc_core::types::{ControlMode, MotorStatus};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, TransportType, motor_channel, start_host,
};
use serde_json::json;
use std::time::{Duration, Instant};

fn send(runtime: &HostRuntime, mode: ControlMode) -> Result<MotorStatus> {
    let (tx, mut rx) = motor_channel();
    runtime.cmd_tx.send(HostCommand::MotorAck(mode, tx))?;
    let until = Instant::now() + Duration::from_millis(200);
    loop {
        if let Ok(result) = rx.try_recv() {
            return result.map_err(|error| anyhow::anyhow!("{error:?}"));
        }
        ensure!(Instant::now() < until, "motor ACK exceeded 200 ms");
        std::thread::sleep(Duration::from_micros(100));
    }
}

struct StopGuard<'a>(&'a HostRuntime);
impl Drop for StopGuard<'_> {
    fn drop(&mut self) {
        if let Err(error) = send(self.0, ControlMode::Stopped) {
            eprintln!("terminal Stop failed: {error:#}");
        }
    }
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 2 || args[0] != "--run" {
        bail!(
            "Usage: velocity_cpu_control --run EXACT_ELF; energizes secured MK5 at +40 erad/s without fast streaming"
        );
    }
    oxifoc_host_lib::init_tracing();
    let runtime = start_host(HostConfig {
        transport: Some(TransportType::Usb),
        elf: Some(args[1].clone()),
        fast_hz: Some(0),
        ..HostConfig::default()
    });
    ensure!(
        runtime.wait_for_connection(Duration::from_secs(8)),
        "USB handshake failed"
    );
    let initial = runtime.slow_rx.recv_timeout(Duration::from_secs(3))?;
    ensure!(
        initial.control_mode == ControlMode::Stopped
            && initial.fault_count == 0
            && (22_000..=26_000).contains(&initial.vbus_mv),
        "unexpected starting state: {initial:?}"
    );
    let _stop = StopGuard(&runtime);
    let started = Instant::now();
    let mut events = Vec::new();
    let mut slow = Vec::new();
    let result: Result<()> = (|| {
        for step in 0..28 {
            let (at, mode) = match step {
                0..=25 => (
                    0.3 + f64::from(step) * 0.2,
                    ControlMode::VelocityControl { target_vel: 40.0 },
                ),
                26 => (5.5, ControlMode::Coast),
                _ => (7.5, ControlMode::Stopped),
            };
            let deadline = started + Duration::from_secs_f64(at);
            while Instant::now() < deadline {
                for sample in runtime.slow_rx.try_iter() {
                    ensure!(sample.fault_count == 0, "device reported fault: {sample:?}");
                    slow.push(json!({"t": started.elapsed().as_secs_f64(), "sample": sample}));
                }
                std::thread::sleep(Duration::from_micros(100));
            }
            let sent = started.elapsed().as_secs_f64();
            ensure!(
                sent - at <= 0.005,
                "late timeline command by {} seconds",
                sent - at
            );
            let status = send(&runtime, mode)?;
            events.push(json!({"planned": at, "sent": sent, "ack": started.elapsed().as_secs_f64(), "status": format!("{status:?}")}));
        }
        Ok(())
    })();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"fast_hz": 0, "events": events, "slow": slow,
        "error": result.as_ref().err().map(|error| format!("{error:#}"))}))?
    );
    result
}
