//! Read the dedicated RTT channel without resetting, halting or resuming firmware.

use crate::{
    HostConfig,
    raw_telemetry::{RawIngress, RawTelemetryState},
};
use anyhow::{Context, Result, anyhow, bail};
use object::{Object, ObjectSymbol};
use probe_rs::{
    MemoryInterface, Permissions,
    probe::list::Lister,
    rtt::{Rtt, ScanRegion},
};
use std::{
    sync::Arc,
    thread,
    time::{Duration, Instant},
};
use tokio_util::sync::CancellationToken;

const RAW_CHANNEL: usize = 2;
const RAW_NAME: &str = "raw-fast";
const MAX_READ: usize = 8192;

pub(crate) struct RawRttWorker {
    cancel: CancellationToken,
    thread: Option<thread::JoinHandle<()>>,
}

impl Drop for RawRttWorker {
    fn drop(&mut self) {
        self.cancel.cancel();
        if let Some(handle) = self.thread.take() {
            let deadline = Instant::now() + Duration::from_secs(3);
            while !handle.is_finished() && Instant::now() < deadline {
                thread::sleep(Duration::from_millis(5));
            }
            if handle.is_finished() {
                let _ = handle.join();
            } else {
                tracing::error!("raw RTT reader still owns the probe after shutdown timeout");
            }
        }
    }
}

fn exact_rtt_address(elf: &str) -> Result<u64> {
    let bytes = std::fs::read(elf).with_context(|| format!("read exact raw RTT ELF {elf}"))?;
    let file = object::File::parse(&*bytes).context("parse raw RTT ELF")?;
    if file.is_64() {
        bail!("raw RTT reader requires the 32-bit firmware ELF");
    }
    file.symbols()
        .chain(file.dynamic_symbols())
        .find(|s| s.name() == Ok("_SEGGER_RTT"))
        .map(|s| s.address())
        .ok_or_else(|| anyhow!("exact ELF has no _SEGGER_RTT symbol; no RAM scan fallback"))
}

pub(crate) fn spawn(
    cfg: &HostConfig,
    state: Arc<RawTelemetryState>,
    host_cancel: CancellationToken,
) -> Result<RawRttWorker> {
    let chip = cfg.chip.clone().context("rtt-raw requires --chip")?;
    let address = exact_rtt_address(
        cfg.elf
            .as_deref()
            .context("rtt-raw requires the exact running --elf")?,
    )?;
    let selector = cfg.probe.clone();
    let cancel = host_cancel.child_token();
    let worker_cancel = cancel.clone();
    let handle = thread::Builder::new()
        .name("raw-rtt-reader".into())
        .spawn(move || {
            if let Err(error) = run(&chip, selector.as_deref(), address, &state, &worker_cancel) {
                tracing::error!("raw RTT reader failed: {error:#}");
                state.connection_lost(error);
            }
        })?;
    Ok(RawRttWorker {
        cancel,
        thread: Some(handle),
    })
}

fn run(
    chip: &str,
    selector: Option<&str>,
    rtt_address: u64,
    state: &Arc<RawTelemetryState>,
    cancel: &CancellationToken,
) -> Result<()> {
    let probes = Lister::new().list_all();
    let mut matching = probes.into_iter().filter(|probe| {
        selector.is_none_or(|selector| {
            let parts: Vec<_> = selector.split(':').collect();
            (parts.len() == 2 || parts.len() == 3)
                && u16::from_str_radix(parts[0], 16) == Ok(probe.vendor_id)
                && u16::from_str_radix(parts[1], 16) == Ok(probe.product_id)
                && (parts.len() == 2 || probe.serial_number.as_deref() == Some(parts[2]))
        })
    });
    let info = matching.next().context("no matching raw RTT debug probe")?;
    if matching.next().is_some() {
        bail!("ambiguous raw RTT probe; select VID:PID:SERIAL");
    }
    let mut probe = info.open().context("open raw RTT probe")?;
    let requested = std::env::var("OXIFOC_SWD_KHZ")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(24_000);
    let speed = probe
        .set_speed(requested)
        .context("set raw RTT SWD speed")?;
    tracing::info!(
        speed_khz = speed,
        address = rtt_address,
        "attaching raw RTT without reset/halt/run"
    );
    let mut session = probe
        .attach(chip, Permissions::default())
        .context("attach raw RTT probe")?;
    let mut core = session.core(0)?;
    if core.core_halted()? {
        bail!("target is halted; raw RTT reader will not resume it");
    }
    let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Exact(rtt_address))
        .context("attach exact live RTT control block")?;
    let channel = rtt
        .up_channel(RAW_CHANNEL)
        .context("firmware has no RTT raw channel 2")?;
    if channel.name() != Some(RAW_NAME) {
        bail!("RTT channel 2 is not {RAW_NAME:?}");
    }
    let expected_size = channel.buffer_size();
    // SEGGER's 32-bit control block: 16-byte ID, two u32 counts, then
    // 24-byte channel descriptors (name, buffer, size, write, read, flags).
    let descriptor = rtt_address + 24 + RAW_CHANNEL as u64 * 24;
    let mut fields = [0u32; 6];
    core.read_32(descriptor, &mut fields)?;
    let buffer = u64::from(fields[1]);
    let size = fields[2] as usize;
    if size != expected_size
        || !(4..=1 << 20).contains(&size)
        || !size.is_multiple_of(4)
        || buffer % 4 != 0
    {
        bail!("invalid or unaligned raw RTT ring descriptor");
    }
    drop(rtt);
    let mut scratch = vec![0u8; MAX_READ + 4];
    let mut ingress = RawIngress::new(state.clone());
    let mut expected_read = None;
    state.reader_ready();
    while !cancel.is_cancelled() {
        let mut offsets = [0u32; 2];
        core.read_32(descriptor + 12, &mut offsets)?;
        let write = offsets[0] as usize;
        let read = offsets[1] as usize;
        if write >= size || read >= size || expected_read.is_some_and(|last| last != read) {
            bail!("raw RTT ring offsets changed unexpectedly (reset or another reader)");
        }
        let available = if write >= read {
            write - read
        } else {
            size - read + write
        };
        if available == 0 {
            state.check()?;
            thread::sleep(Duration::from_micros(50));
            continue;
        }
        let count = available.min(size - read).min(MAX_READ);
        let aligned = read & !3;
        let prefix = read - aligned;
        let span = (prefix + count).next_multiple_of(4);
        // Native 32-bit bulk reads avoid probe-rs RTT's read_8 chunk path.
        // Only unread ring bytes are consumed; rounded boundary bytes are ignored.
        core.read_mem_32bit(buffer + aligned as u64, &mut scratch[..span])?;
        ingress.push(&scratch[prefix..prefix + count])?;
        let next = (read + count) % size;
        core.write_word_32(descriptor + 16, next as u32)?;
        expected_read = Some(next);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn aligned_read_ranges_cover_only_the_allocated_ring() {
        for read in 0..64_usize {
            for count in 1..=64 - read {
                let aligned = read & !3;
                let prefix = read - aligned;
                let span = (prefix + count).next_multiple_of(4);
                assert!(aligned + span <= 64);
                assert_eq!(span % 4, 0);
                assert!(prefix + count <= span);
            }
        }
    }
}
