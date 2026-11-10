//! Synthetic Ergot telemetry and concurrent-control qualification. No motor commands.
use anyhow::{Result, ensure};
use ergot::{
    Address,
    interface_manager::{InterfaceState, Profile},
    toolkits::tokio_serial_v5 as kit,
};
use oxifoc_core::transport_bench::{Config, ControlEndpoint, FrameTopic, Request, Stats, pattern};
use serde_json::json;
use std::{
    pin::pin,
    time::{Duration, Instant},
};

async fn request(stack: &kit::RouterStack, peer: Address, req: Request) -> Result<Stats> {
    tokio::time::timeout(
        Duration::from_secs(2),
        stack
            .endpoints()
            .request::<ControlEndpoint>(peer, &req, None),
    )
    .await?
    .map_err(|e| anyhow::anyhow!("control: {e:?}"))
}

#[tokio::main]
async fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(
        args.len() == 5,
        "usage: usb_serial_jtag_bench PORT FPS PAYLOAD_BYTES SECONDS"
    );
    let fps: u32 = args[2].parse()?;
    let bytes: u16 = args[3].parse()?;
    let seconds: u32 = args[4].parse()?;
    ensure!(
        seconds > 0
            && seconds <= 600
            && bytes > 0
            && usize::from(bytes) <= oxifoc_core::transport_bench::MAX_PAYLOAD,
        "invalid duration or payload size"
    );
    let stack = kit::RouterStack::new();
    let iface = kit::register_router_interface(&stack, &args[1], 921600, 2048, 65536)
        .await
        .map_err(|e| anyhow::anyhow!("serial registration: {e:?}"))?;
    let receiver = stack
        .topics()
        .heap_bounded_receiver::<FrameTopic>(256, None);
    let receiver = pin!(receiver);
    let mut handle = receiver.subscribe();
    let peer = Address {
        network_id: stack.manage_profile(|p| match p.interface_state(iface) {
            Some(InterfaceState::Active { net_id, .. }) => net_id,
            _ => 0,
        }),
        node_id: 2,
        port_id: 0,
    };
    let mut ready = false;
    for _ in 0..10 {
        if request(&stack, peer, Request::Status).await.is_ok() {
            ready = true;
            break;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    ensure!(ready, "C5 Ergot handshake failed");
    let session = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)?
        .subsec_nanos()
        .max(1);
    let frames = if fps == 0 {
        10_000_000
    } else {
        fps.checked_mul(seconds)
            .ok_or_else(|| anyhow::anyhow!("frame count overflow"))?
    };
    request(
        &stack,
        peer,
        Request::Start(Config {
            session,
            payload_bytes: bytes,
            frames_per_second: fps,
            frames,
            duration_ms: seconds * 1000,
        }),
    )
    .await?;
    let controls = {
        let stack = stack.clone();
        tokio::spawn(async move {
            let start = Instant::now();
            let mut latency = Vec::new();
            let mut errors = 0;
            while start.elapsed() < Duration::from_secs(u64::from(seconds)) {
                let at = Instant::now();
                match request(&stack, peer, Request::Status).await {
                    Ok(_) => latency.push(at.elapsed().as_micros() as u64),
                    Err(_) => errors += 1,
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            (latency, errors)
        })
    };
    let start = Instant::now();
    let mut received = 0u64;
    let mut corrupt = 0u64;
    let mut gaps = 0u64;
    let mut next = 0;
    let mut last = start;
    while start.elapsed() < Duration::from_secs(u64::from(seconds) + 2) {
        match tokio::time::timeout(Duration::from_millis(200), handle.recv()).await {
            Ok(message) if message.t.session == session => {
                let frame = &message.t;
                if frame.seq != next {
                    gaps += u64::from(frame.seq.saturating_sub(next));
                }
                next = frame.seq + 1;
                if frame.data.len() != usize::from(bytes)
                    || frame
                        .data
                        .iter()
                        .enumerate()
                        .any(|(i, b)| *b != pattern(session, frame.seq, i))
                {
                    corrupt += 1;
                }
                received += 1;
                last = Instant::now();
                if fps != 0 && received + gaps >= u64::from(frames) {
                    break;
                }
            }
            _ if start.elapsed() >= Duration::from_secs(u64::from(seconds)) => break,
            _ => {}
        }
    }
    let stats = request(&stack, peer, Request::Status).await?;
    request(&stack, peer, Request::Stop).await?;
    let (latency, control_errors) = controls.await?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"fps":fps,"payload_bytes":bytes,"seconds":seconds,
        "received_frames":received,"gaps":gaps,"corrupt":corrupt,"elapsed_s":last.duration_since(start).as_secs_f64(),
        "payload_Bps": received as f64 * f64::from(bytes) / last.duration_since(start).as_secs_f64(),
        "device_produced":stats.produced,"device_queued":stats.queued,"device_dropped":stats.dropped,
        "device_complete":stats.complete,"controls":latency.len(),"control_errors":control_errors,
        "control_max_us":latency.iter().max(),"control_latencies_us":latency}))?
    );
    Ok(())
}
