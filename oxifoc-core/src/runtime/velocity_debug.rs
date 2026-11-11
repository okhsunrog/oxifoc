//! Optional SPSC queue, owned by the same gate and session as the raw stream.
//! The ISR emits already encoded records; no async work or mutex in the producer.

use crate::velocity_telemetry::{SESSION_START, VELOCITY_RECORD_BYTES, VelocityDebugFrame};
use bbqueue::nicknames::Churrasco;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

const BYTES: usize = VELOCITY_RECORD_BYTES;
static QUEUE: Churrasco<2048> = Churrasco::new();
static ENABLED: AtomicBool = AtomicBool::new(false);
static START_PENDING: AtomicBool = AtomicBool::new(false);
static AVAILABLE: AtomicBool = AtomicBool::new(false);
pub(super) static DROPS: AtomicU32 = AtomicU32::new(0);

/// Register a board that supplies the native sequence and exact velocity update.
pub fn register_producer() {
    AVAILABLE.store(true, Ordering::Release);
}

pub(super) fn available() -> bool {
    AVAILABLE.load(Ordering::Acquire)
}

pub fn enabled() -> bool {
    ENABLED.load(Ordering::Acquire)
}

pub fn start_pending() -> bool {
    START_PENDING.load(Ordering::Relaxed)
}

/// Called only by the ADC ISR, after the control step and before fast publish.
pub fn push(mut frame: VelocityDebugFrame) {
    let start = start_pending();
    if start {
        frame.flags |= SESSION_START;
    }
    match QUEUE.stream_producer().grant_exact(BYTES) {
        Ok(mut grant) => {
            grant.copy_from_slice(&frame.encode());
            grant.commit(BYTES);
            if start {
                START_PENDING.store(false, Ordering::Relaxed);
            }
        }
        Err(_) => {
            DROPS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

pub(super) fn enable() {
    START_PENDING.store(true, Ordering::Relaxed);
    ENABLED.store(true, Ordering::Release);
}

pub(super) fn disable() {
    ENABLED.store(false, Ordering::Release);
}

/// Consumers are serialized by FAST_TELEM_GATE. No grant lives across an await.
pub(super) fn drain(output: &mut [u8]) -> usize {
    debug_assert!(output.len().is_multiple_of(BYTES));
    let consumer = QUEUE.stream_consumer();
    let mut written = 0;
    while written < output.len()
        && let Ok(grant) = consumer.read()
    {
        let n = grant.len().min(output.len() - written);
        debug_assert!(n.is_multiple_of(BYTES));
        output[written..written + n].copy_from_slice(&grant[..n]);
        grant.release(n);
        written += n;
    }
    written
}

/// Only while producer is disabled and the task holds the session gate.
pub(super) fn discard() {
    let consumer = QUEUE.stream_consumer();
    while let Ok(grant) = consumer.read() {
        let n = grant.len();
        grant.release(n);
    }
    DROPS.store(0, Ordering::Relaxed);
}
