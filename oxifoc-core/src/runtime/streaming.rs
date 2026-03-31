//! Fast telemetry streaming via lock-free bbqueue.
//!
//! ISR/sim writes `FastTelemetry` structs into a bbqueue at the decimated rate.
//! The async streaming task wakes on a timer, drains the queue, and broadcasts
//! batches of up to 8 samples as a single ergot topic message.
//!
//! Streaming is disabled by default — the host must send `TelemetryConfig`
//! with a nonzero `fast_hz` to start.

use core::mem::size_of;
use core::sync::atomic::{AtomicU32, Ordering};

use bbqueue::nicknames::Churrasco;
use ergot::net_stack::NetStackHandle;
use heapless::Vec;

use crate::foc::controller::FocOutput;
use crate::icd::FastTelemetryTopic;
use crate::timer::Timer;
use crate::types::FastTelemetry;

/// Lock-free queue for ISR → async telemetry transfer.
///
/// ~22 frames of 46 bytes (44 data + 2 header) fit in 1024 bytes.
/// Churrasco = Inline + Atomic + Polling + Static reference.
pub static FAST_TELEM_Q: Churrasco<2048> = Churrasco::new();

/// Decimation period: ISR writes to bbqueue every `period` FOC cycles.
/// 0 = streaming disabled. Set by `telemetry_config_server` when host
/// sends `TelemetryConfig { fast_hz }`.
pub static FAST_TELEM_PERIOD: AtomicU32 = AtomicU32::new(0);

/// Default maximum samples per batch (fits within 2048-byte MTU for TCP/serial).
/// Devices can use a smaller batch size via the const generic on `fast_telemetry_stream`.
pub const DEFAULT_BATCH_SIZE: usize = 32;

/// Build a `FastTelemetry` from FOC output and sensor state.
///
/// Callable from ISR (no allocation, pure computation).
pub fn build_fast_telemetry(
    foc: &FocOutput,
    hall_state: u8,
    velocity_rad_s: f32,
    seq: u32,
) -> FastTelemetry {
    FastTelemetry {
        ia: foc.ia,
        ib: foc.ib,
        ic: foc.ic,
        id: foc.id,
        iq: foc.iq,
        vd: foc.vd,
        vq: foc.vq,
        angle_rad: foc.angle_rad,
        erpm: (velocity_rad_s * 60.0 / core::f32::consts::TAU) as i32,
        duty_x10: 0, // TODO: compute from duties when available
        hall_state,
        _pad: 0,
        seq,
    }
}

/// Push a `FastTelemetry` sample into the bbqueue.
///
/// Called from ISR or sim loop after decimation check.
/// If the queue is full, the sample is silently dropped (correct for real-time).
pub fn push_fast_telemetry(telem: &FastTelemetry) {
    let prod = FAST_TELEM_Q.framed_producer();
    if let Ok(mut grant) = prod.grant(size_of::<FastTelemetry>() as u16) {
        grant.copy_from_slice(bytemuck::bytes_of(telem));
        grant.commit(size_of::<FastTelemetry>() as u16);
    }
}

/// Run the fast telemetry streaming task.
///
/// Drains `FAST_TELEM_Q` on a timer and broadcasts `FastTelemetryBatch`
/// via the ergot topic system. When streaming is disabled (`FAST_TELEM_PERIOD == 0`),
/// polls periodically waiting for the host to enable it.
///
/// The const generic `BATCH` controls the maximum samples per broadcast.
/// Smaller values reduce stack usage at the cost of more frequent broadcasts.
/// The wire format is compatible regardless of batch size — postcard only
/// encodes actual elements, and the host deserializes into `Vec<_, 32>`.
pub async fn fast_telemetry_stream<NS, const BATCH: usize, T: Timer>(stack: NS, foc_freq_hz: u32)
where
    NS: NetStackHandle + Clone,
{
    let cons = FAST_TELEM_Q.framed_consumer();

    #[cfg(feature = "log")]
    log::info!("fast_telemetry_stream: waiting for host to enable streaming");

    loop {
        let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);

        if period == 0 {
            // Streaming disabled — check again in 100ms
            T::after_millis(100).await;
            continue;
        }

        // Compute sleep interval: BATCH samples at (foc_freq_hz / period) Hz
        let sample_hz = foc_freq_hz / period;
        let interval_us = if sample_hz > 0 {
            (BATCH as u64 * 1_000_000) / sample_hz as u64
        } else {
            100_000 // fallback 100ms
        };

        T::after_micros(interval_us).await;

        // Drain bbqueue into batches and broadcast
        loop {
            let mut samples: Vec<FastTelemetry, BATCH> = Vec::new();

            while samples.len() < BATCH {
                match cons.read() {
                    Ok(grant) => {
                        if grant.len() == size_of::<FastTelemetry>() {
                            let telem: FastTelemetry = bytemuck::pod_read_unaligned(&grant);
                            let _ = samples.push(telem);
                        }
                        grant.release();
                    }
                    Err(_) => break, // queue empty
                }
            }

            if samples.is_empty() {
                break; // nothing left to send
            }

            let batch_full = samples.len() == BATCH;
            let batch = crate::types::FastTelemetryBatch { samples };
            let _result = stack
                .stack()
                .topics()
                .broadcast::<FastTelemetryTopic<BATCH>>(&batch, None);

            #[cfg(feature = "log")]
            if _result.is_err() {
                log::warn!("fast_telemetry broadcast failed: {:?}", _result);
            }

            // If we got fewer than BATCH, the queue is drained
            if !batch_full {
                break;
            }
        }
    }
}
