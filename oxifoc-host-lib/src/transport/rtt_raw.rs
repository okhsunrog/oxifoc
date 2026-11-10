//! Read the dedicated RTT channel without resetting, halting or resuming firmware.

use crate::{
    HostConfig,
    raw_telemetry::{RawIngress, RawTelemetryState},
};
use anyhow::{Context, Result, anyhow, bail};
use object::{Object, ObjectSymbol};
use probe_rs::{
    architecture::arm::{FullyQualifiedApAddress, memory::ArmMemoryInterface},
    config::{DebugSequence, Registry},
    probe::list::Lister,
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
const SRAM: std::ops::Range<u64> = 0x2000_0000..0x2002_0000;
const FLASH: std::ops::Range<u64> = 0x0800_0000..0x0810_0000;
const RTT_ID: [u8; 16] = *b"SEGGER RTT\0\0\0\0\0\0";
const LATENCY_BOUNDS_US: [u64; 13] = [
    50, 100, 250, 500, 1_000, 2_000, 5_000, 10_000, 15_000, 20_000, 30_000, 50_000, 100_000,
];

#[derive(Default)]
struct OperationTiming {
    count: u64,
    total_ns: u128,
    max_ns: u128,
    buckets: [u64; 14],
}

impl OperationTiming {
    fn record(&mut self, elapsed: Duration) {
        let ns = elapsed.as_nanos();
        self.count += 1;
        self.total_ns += ns;
        self.max_ns = self.max_ns.max(ns);
        let us = ns.div_ceil(1_000);
        let bucket = LATENCY_BOUNDS_US
            .iter()
            .position(|&bound| us <= u128::from(bound))
            .unwrap_or(LATENCY_BOUNDS_US.len());
        self.buckets[bucket] += 1;
    }

    fn report(&self, operation: &str) {
        tracing::info!(operation, count=self.count,
            mean_us=if self.count == 0 {0.0} else {self.total_ns as f64 / self.count as f64 / 1_000.0},
            max_us=self.max_ns as f64 / 1_000.0, buckets=?self.buckets,
            "raw RTT reader latency (bucket upper bounds in summary)");
    }
}

struct ReaderStats {
    started: Instant,
    cursor_read: OperationTiming,
    bulk_read: OperationTiming,
    decode: OperationTiming,
    cursor_write: OperationTiming,
    poll_gap: OperationTiming,
    release_gap: OperationTiming,
    last_poll: Option<Instant>,
    last_release: Option<Instant>,
    empty_polls: u64,
    bytes: u64,
    native_bytes: u64,
    batch_min: usize,
    batch_max: usize,
    available_min: usize,
    available_max: usize,
    free_min: usize,
    free_max: usize,
    wraps: u64,
}

impl ReaderStats {
    fn new(size: usize) -> Self {
        Self {
            started: Instant::now(),
            cursor_read: OperationTiming::default(),
            bulk_read: OperationTiming::default(),
            decode: OperationTiming::default(),
            cursor_write: OperationTiming::default(),
            poll_gap: OperationTiming::default(),
            release_gap: OperationTiming::default(),
            last_poll: None,
            last_release: None,
            empty_polls: 0,
            bytes: 0,
            native_bytes: 0,
            batch_min: usize::MAX,
            batch_max: 0,
            available_min: size,
            available_max: 0,
            free_min: size,
            free_max: 0,
            wraps: 0,
        }
    }

    fn report(&self, size: usize, failed: bool) {
        tracing::info!(failed, elapsed_s=self.started.elapsed().as_secs_f64(), ring_bytes=size, bytes=self.bytes,
            native_bytes=self.native_bytes, empty_polls=self.empty_polls, batch_min=if self.batch_min==usize::MAX {0}else{self.batch_min},
            batch_max=self.batch_max, available_min=self.available_min, available_max=self.available_max,
            free_min=self.free_min, free_max=self.free_max, wraps=self.wraps, bucket_upper_us=?LATENCY_BOUNDS_US,
            "raw RTT reader summary (final latency bucket is overflow)");
        for (operation, timing) in [
            ("cursor_read", &self.cursor_read),
            ("bulk_read", &self.bulk_read),
            ("decode_delivery", &self.decode),
            ("cursor_write", &self.cursor_write),
            ("poll_gap", &self.poll_gap),
            ("release_gap", &self.release_gap),
        ] {
            timing.report(operation);
        }
    }
}

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
    // This direct MEM-AP reader deliberately supports the F405 bench layout.
    // Other targets must supply their own validated RAM/AP/device contract.
    if !chip.starts_with("STM32F405") {
        bail!("non-halting raw RTT currently supports STM32F405 only");
    }
    let target = Registry::from_builtin_families()
        .get_target_by_name(chip)
        .context("resolve raw RTT target")?;
    let DebugSequence::Arm(sequence) = target.debug_sequence else {
        bail!("raw RTT requires an ARM target");
    };
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
    // Session::new and Session::drop clear breakpoints through halted_access.
    // Own the public MEM-AP interface directly so neither lifecycle can halt
    // or resume a running motor controller, including an ingress error exit.
    probe
        .attach_to_unspecified()
        .context("attach raw RTT debug port")?;
    let mut arm = probe
        .try_into_arm_debug_interface(sequence)
        .map_err(|(_probe, error)| anyhow!("open raw RTT ARM interface: {error}"))?;
    let result = match arm.memory_interface(&FullyQualifiedApAddress::v1_with_default_dp(0)) {
        Ok(mut memory) => read_live(&mut *memory, rtt_address, state, cancel),
        Err(error) => Err(anyhow!("open raw RTT MEM-AP0: {error}")),
    };
    let mut probe = arm.close();
    let detached = probe.detach().context("detach raw RTT debug port");
    if result.is_err() {
        if let Err(error) = detached {
            tracing::warn!("raw RTT detach also failed: {error:#}");
        }
        result
    } else {
        detached
    }
}

fn ram_range(address: u64, bytes: usize) -> Result<()> {
    let end = address
        .checked_add(bytes as u64)
        .context("raw RTT range overflow")?;
    if bytes == 0 || address < SRAM.start || end > SRAM.end {
        bail!("raw RTT address range is outside STM32F405 SRAM");
    }
    Ok(())
}

fn validate_device(dhcsr: u32, cpuid: u32, device: u32) -> Result<()> {
    if dhcsr & (1 << 17) != 0 {
        bail!("target is halted; raw RTT reader will not resume it");
    }
    if dhcsr & (1 << 19) != 0 {
        bail!("raw RTT target is locked up");
    }
    if cpuid & 0xff0f_fff0 != 0x410f_c240 || device & 0xfff != 0x413 {
        bail!(
            "raw RTT target is not the expected Cortex-M4 STM32F405/407 family (CPUID={cpuid:#x}, ID={device:#x})"
        );
    }
    Ok(())
}

fn validate_header(address: u64, header: &[u32; 6]) -> Result<()> {
    let id = core::array::from_fn::<_, 16, _>(|i| header[i / 4].to_le_bytes()[i % 4]);
    if id != RTT_ID {
        bail!("exact ELF does not identify a live SEGGER RTT control block");
    }
    if header[4] <= RAW_CHANNEL as u32 || header[4] > 255 || header[5] > 255 {
        bail!("firmware has no valid physical RTT raw channel2 descriptor");
    }
    ram_range(address, 24 + (header[4] as usize + header[5] as usize) * 24)
}

fn validate_ring(fields: &[u32; 6]) -> Result<(u64, usize)> {
    let buffer = u64::from(fields[1]);
    let size = fields[2] as usize;
    if size < 4
        || !size.is_multiple_of(4)
        || !buffer.is_multiple_of(4)
        || fields[3] >= fields[2]
        || fields[4] >= fields[2]
        || fields[5] & 3 != 0
    {
        bail!("invalid, unaligned or blocking raw RTT ring descriptor");
    }
    ram_range(buffer, size)?;
    Ok((buffer, size))
}

fn read_live(
    core: &mut dyn ArmMemoryInterface,
    rtt_address: u64,
    state: &Arc<RawTelemetryState>,
    cancel: &CancellationToken,
) -> Result<()> {
    let dhcsr = core.read_word_32(0xe000_edf0)?;
    let cpuid = core.read_word_32(0xe000_ed00)?;
    let device = core.read_word_32(0xe004_2000)?;
    validate_device(dhcsr, cpuid, device)?;
    ram_range(rtt_address, 24)?;
    if !rtt_address.is_multiple_of(4) {
        bail!("raw RTT control block is unaligned");
    }
    let mut header = [0_u32; 6];
    core.read_32(rtt_address, &mut header)?;
    validate_header(rtt_address, &header)?;
    // SEGGER's 32-bit control block: 16-byte ID, two u32 counts, then
    // 24-byte channel descriptors (name, buffer, size, write, read, flags).
    let descriptor = rtt_address + 24 + RAW_CHANNEL as u64 * 24;
    let mut fields = [0u32; 6];
    core.read_32(descriptor, &mut fields)?;
    let (buffer, size) = validate_ring(&fields)?;
    let name = u64::from(fields[0]);
    let aligned_name = name & !3;
    let prefix = (name - aligned_name) as usize;
    let span = (prefix + RAW_NAME.len() + 1).next_multiple_of(4);
    if !FLASH.contains(&aligned_name) || aligned_name + span as u64 > FLASH.end {
        ram_range(aligned_name, span).context("raw RTT channel name is outside flash/SRAM")?;
    }
    let mut name_bytes = [0_u8; 12];
    core.read_mem_32bit(aligned_name, &mut name_bytes[..span])?;
    if &name_bytes[prefix..prefix + RAW_NAME.len()] != RAW_NAME.as_bytes()
        || name_bytes[prefix + RAW_NAME.len()] != 0
    {
        bail!("RTT physical channel2 is not {RAW_NAME:?}");
    }
    let mut scratch = vec![0u8; MAX_READ + 4];
    let mut ingress = RawIngress::new(state.clone());
    let mut expected_read = None;
    let mut stats = ReaderStats::new(size);
    state.reader_ready();
    let result = (|| -> Result<()> {
        while !cancel.is_cancelled() {
            let poll = Instant::now();
            if let Some(previous) = stats.last_poll {
                stats.poll_gap.record(poll.duration_since(previous));
            }
            stats.last_poll = Some(poll);
            let mut offsets = [0u32; 2];
            let operation = Instant::now();
            let cursor = core.read_32(descriptor + 12, &mut offsets);
            stats.cursor_read.record(operation.elapsed());
            cursor?;
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
                stats.empty_polls += 1;
                state.check()?;
                thread::sleep(Duration::from_micros(50));
                continue;
            }
            stats.available_min = stats.available_min.min(available);
            stats.available_max = stats.available_max.max(available);
            let free = size - 1 - available;
            stats.free_min = stats.free_min.min(free);
            stats.free_max = stats.free_max.max(free);
            let count = available.min(size - read).min(MAX_READ);
            let aligned = read & !3;
            let prefix = read - aligned;
            let span = (prefix + count).next_multiple_of(4);
            // Native 32-bit bulk reads avoid probe-rs RTT's read_8 chunk path.
            // Only unread ring bytes are consumed; rounded boundary bytes are ignored.
            let operation = Instant::now();
            let bulk = core.read_mem_32bit(buffer + aligned as u64, &mut scratch[..span]);
            stats.bulk_read.record(operation.elapsed());
            bulk?;
            stats.bytes += count as u64;
            stats.native_bytes += span as u64;
            stats.batch_min = stats.batch_min.min(count);
            stats.batch_max = stats.batch_max.max(count);
            let operation = Instant::now();
            let decoded = ingress.push(&scratch[prefix..prefix + count]);
            stats.decode.record(operation.elapsed());
            decoded?;
            let next = (read + count) % size;
            let operation = Instant::now();
            let written = core.write_word_32(descriptor + 16, next as u32);
            stats.cursor_write.record(operation.elapsed());
            written?;
            let released = Instant::now();
            if let Some(previous) = stats.last_release {
                stats.release_gap.record(released.duration_since(previous));
            }
            stats.last_release = Some(released);
            stats.wraps += u64::from(next == 0);
            expected_read = Some(next);
        }
        Ok(())
    })();
    // Summaries precede debug-port detach, keeping lifecycle time separate
    // from reader/decoder latency. No Session or core-control writes exist.
    stats.report(size, result.is_err());
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn memory_only_target_gate_rejects_halt_lockup_or_the_wrong_core_and_family() {
        assert!(validate_device(0, 0x410f_c241, 0x1000_0413).is_ok());
        assert!(validate_device(0, 0x412f_c24f, 0x2000_0413).is_ok());
        for (dhcsr, cpuid, device) in [
            (1 << 17, 0x410f_c241, 0x413),
            (1 << 19, 0x410f_c241, 0x413),
            (0, 0x410f_c231, 0x413),
            (0, 0x410f_c241, 0x469),
        ] {
            assert!(validate_device(dhcsr, cpuid, device).is_err());
        }
    }

    #[test]
    fn live_rtt_header_and_physical_ring_are_bounded_before_access() {
        let mut header = [0_u32; 6];
        for (word, bytes) in header[..4].iter_mut().zip(RTT_ID.chunks_exact(4)) {
            *word = u32::from_le_bytes(bytes.try_into().unwrap());
        }
        header[4] = 3;
        assert!(validate_header(SRAM.start, &header).is_ok());
        assert!(validate_header(SRAM.end - 24, &header).is_err());
        header[4] = 2;
        assert!(validate_header(SRAM.start, &header).is_err());
        header[4] = 3;
        header[0] ^= 1;
        assert!(validate_header(SRAM.start, &header).is_err());
        let fields = [0x0800_8000, 0x2000_4000, 16_384, 16_383, 0, 0];
        assert_eq!(validate_ring(&fields).unwrap(), (0x2000_4000, 16_384));
        for (index, bad) in [
            (1, 0x2000_4001),
            (1, 0x2001_f000),
            (2, 0),
            (2, 16_383),
            (3, 16_384),
            (4, 16_384),
            (5, 2),
        ] {
            let mut invalid = fields;
            invalid[index] = bad;
            assert!(validate_ring(&invalid).is_err(), "field{index}={bad:#x}");
        }
        assert!(ram_range(u64::MAX, 4).is_err());
    }

    #[test]
    fn latency_histogram_retains_boundary_and_overflow_samples() {
        let mut timing = OperationTiming::default();
        for ns in [0, 50_000, 50_001, 100_000, 100_000_001] {
            timing.record(Duration::from_nanos(ns));
        }
        assert_eq!(timing.count, 5);
        assert_eq!(timing.total_ns, 100_200_002);
        assert_eq!(timing.max_ns, 100_000_001);
        assert_eq!(timing.buckets[0], 2);
        assert_eq!(timing.buckets[1], 2);
        assert_eq!(timing.buckets[13], 1);
        assert_eq!(timing.buckets.iter().sum::<u64>(), timing.count);
    }

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
