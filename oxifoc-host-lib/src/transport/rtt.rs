//! RTT transport implementation via probe-rs.
//!
//! probe-rs RTT is synchronous and Core/Rtt refs aren't Send, so all RTT I/O
//! runs in a dedicated blocking thread. Channels bridge to the async world via
//! StreamReader (AsyncRead) and SinkWriter+CopyToBytes+PollSender (AsyncWrite).

use super::CobsStreamTransport;
use anyhow::{Context, Result, anyhow};
use bytes::Bytes;
use futures::SinkExt;
use probe_rs::{
    MemoryInterface, Permissions,
    probe::list::Lister,
    rtt::{Rtt, ScanRegion},
};
use std::io;
use std::time::Duration;
use tokio_stream::wrappers::ReceiverStream;
use tokio_util::io::{CopyToBytes, SinkWriter, StreamReader};
use tokio_util::sync::PollSender;
use tracing::{debug, error, info, warn};

/// RTT channel indices (must match device firmware)
const RTT_UP_CHANNEL_DEFMT: usize = 0; // Device -> Host (defmt logs)
const RTT_UP_CHANNEL_ERGOT: usize = 1; // Device -> Host (ergot data)
const RTT_DOWN_CHANNEL_ERGOT: usize = 0; // Host -> Device (ergot data)

/// JoinHandle of the active blocking RTT I/O thread (one per process — the
/// probe is exclusive). The thread owns the probe-rs `Session` and busy-polls
/// USB transactions; if the process exits while it is mid-transfer, the
/// ST-Link firmware is left with a torn command and the NEXT open times out
/// on `GET_CURRENT_MODE` (measured: ~alternating open failures, "USB error:
/// timed out" ~2.6 s into connect). [`join_rtt_io_thread`] lets the shutdown
/// path wait for the thread to finish its current poll and drop the Session
/// cleanly (probe detach → ST-Link back to idle mode).
static RTT_IO_THREAD: std::sync::Mutex<Option<std::thread::JoinHandle<()>>> =
    std::sync::Mutex::new(None);

/// Join the RTT I/O thread if one is (still) running, bounded by `timeout`.
///
/// Called from the backend shutdown path (and from `connect` itself to reap
/// a stale thread before re-opening the probe). No-op when no RTT transport
/// was ever connected — safe to call unconditionally.
pub fn join_rtt_io_thread(timeout: Duration) {
    let handle = RTT_IO_THREAD
        .lock()
        .expect("RTT thread registry poisoned")
        .take();
    let Some(h) = handle else { return };
    // std has no join-with-timeout; poll `is_finished` (the thread notices a
    // closed channel within one ~ms poll iteration, so this converges fast).
    let deadline = std::time::Instant::now() + timeout;
    while !h.is_finished() && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    if h.is_finished() {
        let _ = h.join();
        info!("RTT I/O thread joined — probe session closed cleanly");
    } else {
        warn!(
            "RTT I/O thread still busy after {timeout:?}; \
             exiting anyway may leave the ST-Link wedged for the next open"
        );
    }
}

/// Address of the live RTT control block, read from the firmware ELF's
/// `_SEGGER_RTT` symbol. Pinning the scan to this avoids the "multiple control
/// blocks" failure: a `ScanRegion::Ram` sweep also finds STALE control blocks
/// that previous firmware images left in uninitialized RAM (CCMRAM/SRAM2 — a
/// reset does not clear them), and probe-rs cannot pick among them. This is
/// how `probe-rs run` resolves it too.
fn segger_rtt_addr(elf_path: &str) -> Option<u64> {
    use object::{Object, ObjectSymbol};
    // Failures here are LOUD: a typo'd --elf silently degrading to a full-RAM
    // scan both resurrects the stale-control-block race (the magic wipe below
    // is skipped without an address) and can latch a leftover block in
    // CCMRAM/SRAM2 — the exact intermittent NoRouteToDest this pin exists to
    // prevent.
    let bytes = match std::fs::read(elf_path) {
        Ok(b) => b,
        Err(e) => {
            warn!("RTT: cannot read ELF '{elf_path}' ({e}); falling back to a full-RAM scan");
            return None;
        }
    };
    let file = match object::File::parse(&*bytes) {
        Ok(f) => f,
        Err(e) => {
            warn!("RTT: cannot parse ELF '{elf_path}' ({e}); falling back to a full-RAM scan");
            return None;
        }
    };
    let addr = file
        .symbols()
        .chain(file.dynamic_symbols())
        .find(|s| s.name() == Ok("_SEGGER_RTT"))
        .map(|s| s.address());
    if addr.is_none() {
        warn!(
            "RTT: no _SEGGER_RTT symbol in '{elf_path}' (wrong image?); falling back to a full-RAM scan"
        );
    }
    addr
}

/// Connect to a target via RTT through a debug probe.
///
/// Spawns a dedicated blocking thread that owns the probe-rs Session/Core/Rtt
/// and communicates with the async world via channels. `elf` is the running
/// firmware image; when given, the RTT control block is located from its
/// `_SEGGER_RTT` symbol instead of a full-RAM scan (see [`segger_rtt_addr`]).
#[allow(clippy::single_range_in_vec_init)]
pub fn connect(
    probe_selector: Option<&str>,
    chip: &str,
    elf: Option<&str>,
) -> Result<CobsStreamTransport> {
    info!("Connecting via RTT to chip: {}", chip);

    // Pin the RTT control block to the live `_SEGGER_RTT` address (avoids the
    // stale-block scan ambiguity). Computed once, used by BOTH the verification
    // attach below and the worker thread's attach.
    let rtt_addr = elf.and_then(segger_rtt_addr);
    let scan_region = match rtt_addr {
        Some(addr) => {
            info!("RTT control block pinned to _SEGGER_RTT @ {addr:#x} (from ELF)");
            ScanRegion::Exact(addr)
        }
        None => {
            info!("RTT: no ELF symbol available, scanning all RAM");
            ScanRegion::Ram
        }
    };

    let lister = Lister::new();

    // Find and open the probe
    let mut probe = if let Some(selector) = probe_selector {
        info!("Using probe selector: {}", selector);
        let probes = lister.list_all();
        debug!("Available probes: {:?}", probes);

        let parts: Vec<&str> = selector.split(':').collect();
        let (vid, pid, serial) = match parts.len() {
            2 => {
                let vid = u16::from_str_radix(parts[0], 16)
                    .with_context(|| format!("Invalid VID: {}", parts[0]))?;
                let pid = u16::from_str_radix(parts[1], 16)
                    .with_context(|| format!("Invalid PID: {}", parts[1]))?;
                (vid, pid, None)
            }
            3 => {
                let vid = u16::from_str_radix(parts[0], 16)
                    .with_context(|| format!("Invalid VID: {}", parts[0]))?;
                let pid = u16::from_str_radix(parts[1], 16)
                    .with_context(|| format!("Invalid PID: {}", parts[1]))?;
                (vid, pid, Some(parts[2]))
            }
            _ => {
                return Err(anyhow!(
                    "Invalid probe selector format. Use 'VID:PID' or 'VID:PID:SERIAL'"
                ));
            }
        };

        let probe_info = probes
            .into_iter()
            .find(|p| {
                p.vendor_id == vid
                    && p.product_id == pid
                    && (serial.is_none() || p.serial_number.as_deref() == serial)
            })
            .ok_or_else(|| anyhow!("No matching probe found for selector: {selector}"))?;

        probe_info.open().context("Failed to open probe")?
    } else {
        let probes = lister.list_all();
        if probes.is_empty() {
            return Err(anyhow!("No debug probes found"));
        }
        info!("Auto-selecting first probe: {:?}", probes[0]);
        probes[0].open().context("Failed to open probe")?
    };

    // Push the SWD clock as high as the probe allows before attaching: RTT is
    // polled by the host reading the target's RAM buffers over SWD, so on a
    // slow default clock the debug-port throughput — not the link — caps the
    // telemetry rate. Request the STLink-V3 SWD ceiling (24 MHz, per spec); the
    // probe clamps to its real max, so a STLink V2-1 still lands on its own
    // 4.6 MHz cap. Measured on a NUCLEO-G474RE onboard STLINK-V3E: raising the
    // request from 4.6→24 MHz lifts the raw SWD read ceiling from ~125 KB/s to
    // ~620 KB/s (3.7× the V2-1). Read is round-trip-latency bound, so it
    // plateaus well below the write rate (~940 KB/s) — telemetry is the read
    // direction.
    let req_khz: u32 = std::env::var("OXIFOC_SWD_KHZ")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24_000);
    match probe.set_speed(req_khz) {
        Ok(khz) => info!("SWD clock set to {} kHz", khz),
        Err(e) => info!("Could not raise SWD clock (using default): {e}"),
    }

    // Attach to the target and verify RTT channels
    let mut session = probe
        .attach(chip, Permissions::default())
        .context("Failed to attach to target")?;

    info!("Reset-and-halt, clearing stale RTT control block...");
    {
        let mut core = session.core(0).context("Failed to get core 0")?;
        // Reset-and-HALT so the firmware has NOT re-run `rtt_init!` yet, then wipe
        // the pre-reset `_SEGGER_RTT` magic while the core is halted. This is the
        // race-free ordering: the stale control block (magic + dead channel
        // pointers that survive a reset in un-cleared RAM) is invalidated BEFORE
        // the firmware boots, so the poll below can only ever attach to the FRESH
        // block the booting firmware writes — never the stale one. Latching the
        // stale block desyncs the channel pointers, the device never sees our
        // down-channel frames, and the link never activates (the intermittent
        // NoRouteToDest). Mirrors probe-rs's own clear-block-then-poll-retry RTT
        // client; a fixed boot-wait alone was still racy (broke at larger RTT
        // buffer sizes whose init timing shifted).
        core.reset_and_halt(Duration::from_millis(500))
            .context("Failed to reset/halt target")?;
        if let Some(addr) = rtt_addr {
            let _ = core.write_8(addr, &[0u8; 16]); // zero the 16-byte RTT magic
        }
        core.run().context("Failed to resume target")?;
    }

    info!("Polling for RTT until the firmware re-inits the control block...");
    let mut rtt = {
        let deadline = std::time::Instant::now() + Duration::from_secs(4);
        let mut attempts = 0u32;
        let mut last_err_str = String::new();
        loop {
            let mut core = session.core(0).context("Failed to get core 0")?;
            match Rtt::attach_region(&mut core, &scan_region) {
                // A found control block can still be MID-INIT: the magic can
                // land before the full channel table (observed on the bench:
                // attach 23 ms post-reset saw defmt but no ergot channel).
                // Treat missing expected channels as retryable, same as a
                // missing block — only a complete table breaks the loop.
                Ok(mut rtt) => {
                    let complete = rtt.up_channel(RTT_UP_CHANNEL_DEFMT).is_some()
                        && rtt.up_channel(RTT_UP_CHANNEL_ERGOT).is_some()
                        && rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT).is_some();
                    if complete {
                        break rtt;
                    }
                    if std::time::Instant::now() >= deadline {
                        return Err(anyhow!(
                            "RTT control block found but the expected channels never appeared \
                             ({attempts} poll attempts) — wrong firmware image for this ELF?"
                        ));
                    }
                    attempts += 1;
                    let s = "control block found, channel table incomplete".to_string();
                    if s != last_err_str {
                        info!("RTT attach attempt {attempts}: {s}");
                        last_err_str = s;
                    }
                    drop(core);
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(e) if std::time::Instant::now() < deadline => {
                    attempts += 1;
                    // Log each DISTINCT underlying error once — the poll is
                    // expected to fail with "control block not found" until
                    // the firmware re-inits it, but any OTHER error (memory
                    // read failures, probe faults) is the real diagnostic
                    // when the whole attach eventually times out.
                    let s = format!("{e:?}");
                    if s != last_err_str {
                        info!("RTT attach attempt {attempts}: {s}");
                        last_err_str = s;
                    }
                    drop(core);
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(e) => {
                    return Err(e).with_context(|| {
                        format!(
                            "Failed to attach to RTT after reset \
                             ({attempts} poll attempts; last polled error: {last_err_str})"
                        )
                    });
                }
            }
        }
    };

    {
        let mut core = session.core(0).context("Failed to get core 0")?;
        info!("RTT attached successfully");

        for (idx, channel) in rtt.up_channels().iter().enumerate() {
            info!(
                "RTT up channel {}: {} (size: {})",
                idx,
                channel.name().unwrap_or("unnamed"),
                channel.buffer_size()
            );
        }
        for (idx, channel) in rtt.down_channels().iter().enumerate() {
            info!(
                "RTT down channel {}: {} (size: {})",
                idx,
                channel.name().unwrap_or("unnamed"),
                channel.buffer_size()
            );
        }

        if rtt.up_channel(RTT_UP_CHANNEL_DEFMT).is_none() {
            return Err(anyhow!(
                "RTT up channel {RTT_UP_CHANNEL_DEFMT} (defmt) not found"
            ));
        }
        if rtt.up_channel(RTT_UP_CHANNEL_ERGOT).is_none() {
            return Err(anyhow!(
                "RTT up channel {RTT_UP_CHANNEL_ERGOT} (ergot) not found"
            ));
        }
        let down_ch = rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT).ok_or_else(|| {
            anyhow!("RTT down channel {RTT_DOWN_CHANNEL_ERGOT} (ergot) not found")
        })?;

        // Send a COBS frame boundary so the device can flush any stale
        // partial frame from a previous session and sync cleanly.
        let _ = down_ch.write(&mut core, &[0]);
    }

    // Set up channels between blocking RTT thread and async world.
    //
    // Ergot RX (device→host): blocking_send → StreamReader (AsyncRead)
    // Ergot TX (host→device): SinkWriter (AsyncWrite) → try_recv
    // Defmt RX (device→host): blocking_send → StreamReader (AsyncRead)
    let (ergot_rx_tx, ergot_rx_rx) = tokio::sync::mpsc::channel::<io::Result<Bytes>>(64);
    let (ergot_tx_tx, mut ergot_tx_rx) = tokio::sync::mpsc::channel::<Bytes>(64);
    let (defmt_rx_tx, defmt_rx_rx) = tokio::sync::mpsc::channel::<io::Result<Bytes>>(64);

    // Blocking RTT I/O thread — owns Session, Core, Rtt; never re-attaches.
    // Failures must surface as a broken link (io::Error through the reader
    // channel), not a silent thread death: the old expect()s killed the
    // link with no signal to the reconnect logic.
    let io_thread = std::thread::spawn(move || {
        let fail = |ergot_rx_tx: &tokio::sync::mpsc::Sender<io::Result<Bytes>>, msg: String| {
            error!("{msg}");
            let _ = ergot_rx_tx.blocking_send(Err(io::Error::other(msg)));
        };
        let mut core = match session.core(0) {
            Ok(c) => c,
            Err(e) => {
                fail(
                    &ergot_rx_tx,
                    format!("RTT thread: failed to get core 0: {e}"),
                );
                return;
            }
        };
        // Same pinned region as the verification attach (rtt_addr is Copy).
        let thread_region = match rtt_addr {
            Some(addr) => ScanRegion::Exact(addr),
            None => ScanRegion::Ram,
        };
        let mut rtt = match Rtt::attach_region(&mut core, &thread_region) {
            Ok(r) => r,
            Err(e) => {
                fail(&ergot_rx_tx, format!("RTT thread: failed to attach: {e}"));
                return;
            }
        };

        // Large read buffer: each `channel.read` is one SWD round-trip, so
        // reading the whole available RTT buffer per transaction (instead of
        // capping at 2 KiB) cuts the transaction count — the dominant cost if
        // RTT throughput is transaction-latency-bound rather than SWD-clock-bound.
        let mut ergot_rx_buf = [0u8; 49152];
        let mut defmt_buf = [0u8; 4096];

        // Diagnostic tee: OXIFOC_RTT_DUMP=<path> writes the raw ergot up-channel
        // byte stream to a file for offline COBS/frame analysis (wire-level
        // ground truth when attributing telemetry loss to a pipeline stage).
        let mut dump = std::env::var("OXIFOC_RTT_DUMP")
            .ok()
            .and_then(|p| std::fs::File::create(p).ok());

        // Idle backoff between polls when no bytes moved. RTT is polled (each
        // `channel.read` is one SWD round-trip), so any sleep here caps how fast
        // we re-poll after a momentary drain. This thread is dedicated and
        // blocking and a capture lasts only seconds, so the default is a pure
        // busy-spin (idle_us = 0): burn one core for minimum latency = maximum
        // throughput on this latency-bound link. Set `OXIFOC_RTT_IDLE_US` > 0 to
        // trade throughput for CPU on a long-running attach.
        let idle_us: u64 = std::env::var("OXIFOC_RTT_IDLE_US")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);

        // Poll the defmt channel only every Nth iteration. Each `channel.read`
        // is a SWD round-trip, and on a latency-bound link the defmt read
        // competes with the ergot read for that bandwidth — reading defmt every
        // iteration roughly halves the ergot poll rate under a high-rate stream.
        // defmt is low-rate diagnostic, so polling it 1/N as often costs nothing
        // but frees the SWD bus for telemetry (env `OXIFOC_RTT_DEFMT_EVERY`).
        let defmt_every: u32 = std::env::var("OXIFOC_RTT_DEFMT_EVERY")
            .ok()
            .and_then(|s| s.parse().ok())
            .filter(|&n| n > 0)
            .unwrap_or(64);
        let mut tick: u32 = 0;

        // 1 Hz down-channel write stats, enabled after the first write of the
        // session (idle attaches stay quiet). Pairs with the device's `rx/s:`
        // defmt line to bracket the host→device command path: writes here with
        // zero `motor_reqs` there = frames lost inside the device; no writes
        // here while the host is affirming = frames lost in the host stack.
        let mut down_writes: u32 = 0;
        let mut down_bytes: u64 = 0;
        let mut down_seen_any = false;
        let mut down_report_at = std::time::Instant::now();

        loop {
            // Exit promptly on teardown: the receiver side dropping is the
            // shutdown signal. Without this check the thread only noticed on
            // the next blocking_send — i.e. never while the device is quiet —
            // and got killed by process exit mid-USB-transaction instead,
            // wedging the ST-Link for the next open (see RTT_IO_THREAD).
            if ergot_rx_tx.is_closed() {
                info!("Ergot rx channel closed, stopping RTT thread");
                return;
            }

            let mut did_work = false;
            tick = tick.wrapping_add(1);

            // 1. Read ergot data from device
            if let Some(channel) = rtt.up_channel(RTT_UP_CHANNEL_ERGOT) {
                match channel.read(&mut core, &mut ergot_rx_buf) {
                    Ok(n) if n > 0 => {
                        did_work = true;
                        if let Some(f) = dump.as_mut() {
                            use std::io::Write;
                            let _ = f.write_all(&ergot_rx_buf[..n]);
                        }
                        if ergot_rx_tx
                            .blocking_send(Ok(Bytes::copy_from_slice(&ergot_rx_buf[..n])))
                            .is_err()
                        {
                            info!("Ergot rx channel closed, stopping RTT thread");
                            return;
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        fail(
                            &ergot_rx_tx,
                            format!("RTT ergot read error (probe gone?): {e}"),
                        );
                        return;
                    }
                }
            }

            // 2. Write ergot data to device
            while let Ok(data) = ergot_tx_rx.try_recv() {
                if let Some(channel) = rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT) {
                    let mut offset = 0;
                    while offset < data.len() {
                        match channel.write(&mut core, &data[offset..]) {
                            Ok(n) => offset += n,
                            Err(e) => {
                                fail(
                                    &ergot_rx_tx,
                                    format!("RTT ergot write error (probe gone?): {e}"),
                                );
                                return;
                            }
                        }
                    }
                    down_writes += 1;
                    down_bytes += data.len() as u64;
                    down_seen_any = true;
                    did_work = true;
                }
            }
            if down_seen_any && down_report_at.elapsed() >= Duration::from_secs(1) {
                info!("rtt down/s: writes={down_writes} bytes={down_bytes}");
                down_writes = 0;
                down_bytes = 0;
                down_report_at = std::time::Instant::now();
            }

            // 3. Read defmt data from device (rate-limited; see defmt_every)
            if tick.is_multiple_of(defmt_every)
                && let Some(channel) = rtt.up_channel(RTT_UP_CHANNEL_DEFMT)
            {
                match channel.read(&mut core, &mut defmt_buf) {
                    Ok(n) if n > 0 => {
                        did_work = true;
                        // try_send, NOT blocking_send: defmt is diagnostic and
                        // droppable. A blocking send here lets the defmt decoder
                        // task (starved under a high-rate ergot stream) back up
                        // until its 64-slot mpsc fills, then BLOCK this RTT worker
                        // thread — which stops it reading the ergot channel too,
                        // stalling the whole telemetry stream mid-capture. Drop
                        // defmt on a full queue instead so ergot never starves.
                        use tokio::sync::mpsc::error::TrySendError;
                        match defmt_rx_tx.try_send(Ok(Bytes::copy_from_slice(&defmt_buf[..n]))) {
                            Ok(()) | Err(TrySendError::Full(_)) => {}
                            Err(TrySendError::Closed(_)) => {
                                info!("Defmt rx channel closed, stopping RTT thread");
                                return;
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        fail(
                            &ergot_rx_tx,
                            format!("RTT defmt read error (probe gone?): {e}"),
                        );
                        return;
                    }
                }
            }

            if !did_work {
                if idle_us == 0 {
                    std::hint::spin_loop();
                } else {
                    std::thread::sleep(Duration::from_micros(idle_us));
                }
            }
        }
    });

    // Register the thread for the shutdown join (see RTT_IO_THREAD). A
    // previous entry can only be a finished thread (the probe is exclusive:
    // a live one would have made this connect fail) — reap it.
    if let Some(old) = RTT_IO_THREAD
        .lock()
        .expect("RTT thread registry poisoned")
        .replace(io_thread)
        && old.is_finished()
    {
        let _ = old.join();
    }

    // Build AsyncRead/AsyncWrite from channels
    let reader = StreamReader::new(ReceiverStream::new(ergot_rx_rx));
    let writer = SinkWriter::new(CopyToBytes::new(
        PollSender::new(ergot_tx_tx).sink_map_err(|e| io::Error::new(io::ErrorKind::BrokenPipe, e)),
    ));
    let defmt_reader = StreamReader::new(ReceiverStream::new(defmt_rx_rx));

    Ok(CobsStreamTransport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: Some(Box::new(defmt_reader)),
    })
}
