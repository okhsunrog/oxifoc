//! Bounded ADC logging with thread-mode encoding and nonblocking RTT output.
//!
//! Each defmt `write` supplies serialized, owned bytes. ADC stores those bytes
//! in one framed SPSC grant. The consumer performs rzcobs encoding afterwards.
//! Direct and fatal calls have separate encoder state and share a try-only
//! RTT writer. No borrowed format arguments survive a logging call.

use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use bbqueue::{
    nicknames::Texas, prod_cons::framed::FramedGrantW, traits::notifier::maitake::MaiNotSpsc,
};
use rtt_target::UpChannel;

const RAW_FRAME_BYTES: usize = 256;
const ENCODED_FRAME_BYTES: usize = 512;
const NO_OWNER: u32 = u32::MAX;
const ADC_CONTEXT: u32 = 16 + embassy_stm32::interrupt::ADC as u32;
type AdcQueue = Texas<2048, MaiNotSpsc>;

static ADC_QUEUE: AdcQueue = AdcQueue::new();
static ADC_LOGGER: AdcLogger = AdcLogger {
    busy: AtomicBool::new(false),
    frame: UnsafeCell::new(AdcFrame {
        grant: None,
        len: 0,
        overflow: false,
    }),
};
static DIRECT: DirectLogger = DirectLogger::new();
static EMERGENCY: DirectLogger = DirectLogger::new();
static FATAL: AtomicBool = AtomicBool::new(false);
static RTT: RttSink = RttSink {
    busy: AtomicBool::new(false),
    channel: UnsafeCell::new(None),
};

/// Event counters in field order, also readable through SWD.
#[repr(C)]
pub struct LogStats {
    pub adc_queued: AtomicU32,
    pub queue_full: AtomicU32,
    pub oversize: AtomicU32,
    pub contention: AtomicU32,
    pub rtt_full: AtomicU32,
    pub written: AtomicU32,
    pub uninitialized: AtomicU32,
}

#[unsafe(no_mangle)]
pub static LOG_STATS: LogStats = LogStats {
    adc_queued: AtomicU32::new(0),
    queue_full: AtomicU32::new(0),
    oversize: AtomicU32::new(0),
    contention: AtomicU32::new(0),
    rtt_full: AtomicU32::new(0),
    written: AtomicU32::new(0),
    uninitialized: AtomicU32::new(0),
};

struct Buffer {
    bytes: [u8; ENCODED_FRAME_BYTES],
    len: usize,
    overflow: bool,
}

impl Buffer {
    const fn new() -> Self {
        Self {
            bytes: [0; ENCODED_FRAME_BYTES],
            len: 0,
            overflow: false,
        }
    }

    fn clear(&mut self) {
        self.len = 0;
        self.overflow = false;
    }

    fn extend(&mut self, bytes: &[u8]) {
        if self.overflow || bytes.len() > self.bytes.len() - self.len {
            self.overflow = true;
            return;
        }
        self.bytes[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
    }

    fn send(&self) {
        if self.overflow {
            LOG_STATS.oversize.fetch_add(1, Ordering::Relaxed);
        } else {
            RTT.write(&self.bytes[..self.len]);
        }
    }
}

struct AdcFrame {
    grant: Option<FramedGrantW<&'static AdcQueue, u16>>,
    len: usize,
    overflow: bool,
}

struct AdcLogger {
    busy: AtomicBool,
    frame: UnsafeCell<AdcFrame>,
}

// SAFETY: only ADC accesses frame. Its busy flag rejects reentrant calls.
// Higher-priority fatal exceptions use EMERGENCY instead, including panics in ADC.
unsafe impl Sync for AdcLogger {}

impl AdcLogger {
    fn acquire(&self) {
        assert!(
            !self.busy.swap(true, Ordering::Relaxed),
            "reentrant ADC logger"
        );
        // SAFETY: ADC is the sole producer and this call acquired its frame.
        let frame = unsafe { &mut *self.frame.get() };
        frame.len = 0;
        frame.overflow = false;
        frame.grant = ADC_QUEUE
            .framed_producer()
            .grant(RAW_FRAME_BYTES as u16)
            .ok();
        if frame.grant.is_none() {
            LOG_STATS.queue_full.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn write(&self, bytes: &[u8]) {
        // SAFETY: paired acquire/write/release calls run in the sole ADC producer.
        let frame = unsafe { &mut *self.frame.get() };
        let Some(grant) = frame.grant.as_mut() else {
            return;
        };
        if frame.overflow || bytes.len() > RAW_FRAME_BYTES - frame.len {
            frame.overflow = true;
            return;
        }
        grant[frame.len..frame.len + bytes.len()].copy_from_slice(bytes);
        frame.len += bytes.len();
    }

    fn release(&self) {
        // SAFETY: ADC still owns the frame until busy is cleared below.
        let frame = unsafe { &mut *self.frame.get() };
        if let Some(grant) = frame.grant.take() {
            if frame.overflow {
                LOG_STATS.oversize.fetch_add(1, Ordering::Relaxed);
                drop(grant);
            } else {
                grant.commit(frame.len as u16);
                LOG_STATS.adc_queued.fetch_add(1, Ordering::Relaxed);
            }
        }
        self.busy.store(false, Ordering::Relaxed);
    }
}

struct DirectFrame {
    encoder: defmt::Encoder,
    buffer: Buffer,
}

struct DirectLogger {
    owner: AtomicU32,
    frame: UnsafeCell<DirectFrame>,
}

// SAFETY: owner grants exclusive access to exactly one Cortex-M context.
// Cooperative thread-mode logging contains no await or task switch.
unsafe impl Sync for DirectLogger {}

impl DirectLogger {
    const fn new() -> Self {
        Self {
            owner: AtomicU32::new(NO_OWNER),
            frame: UnsafeCell::new(DirectFrame {
                encoder: defmt::Encoder::new(),
                buffer: Buffer::new(),
            }),
        }
    }

    fn acquire(&self, context: u32) {
        assert_ne!(
            self.owner.load(Ordering::Relaxed),
            context,
            "reentrant direct logger"
        );
        if self
            .owner
            .compare_exchange(NO_OWNER, context, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            LOG_STATS.contention.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // SAFETY: compare_exchange acquired this encoder for the current context.
        let DirectFrame { encoder, buffer } = unsafe { &mut *self.frame.get() };
        buffer.clear();
        encoder.start_frame(|bytes| buffer.extend(bytes));
    }

    fn write(&self, context: u32, bytes: &[u8]) {
        if self.owner.load(Ordering::Relaxed) != context {
            return;
        }
        // SAFETY: this context owns the encoder and cannot be polled concurrently.
        let DirectFrame { encoder, buffer } = unsafe { &mut *self.frame.get() };
        encoder.write(bytes, |bytes| buffer.extend(bytes));
    }

    fn release(&self, context: u32) {
        if self.owner.load(Ordering::Relaxed) != context {
            return;
        }
        // SAFETY: ownership extends through frame completion and RTT publication.
        let DirectFrame { encoder, buffer } = unsafe { &mut *self.frame.get() };
        encoder.end_frame(|bytes| buffer.extend(bytes));
        buffer.send();
        self.owner.store(NO_OWNER, Ordering::Release);
    }
}

struct RttSink {
    busy: AtomicBool,
    channel: UnsafeCell<Option<UpChannel>>,
}

// SAFETY: every channel access is protected by busy. Interrupts only try once.
unsafe impl Sync for RttSink {}

impl RttSink {
    fn write(&self, bytes: &[u8]) {
        if self
            .busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            LOG_STATS.contention.fetch_add(1, Ordering::Relaxed);
            return;
        }
        // SAFETY: this context exclusively owns the channel until busy is cleared.
        match unsafe { &mut *self.channel.get() } {
            Some(channel) => {
                if channel.write_frame(bytes) {
                    LOG_STATS.written.fetch_add(1, Ordering::Relaxed);
                } else {
                    LOG_STATS.rtt_full.fetch_add(1, Ordering::Relaxed);
                }
            }
            None => {
                LOG_STATS.uninitialized.fetch_add(1, Ordering::Relaxed);
            }
        }
        self.busy.store(false, Ordering::Release);
    }
}

fn context() -> u32 {
    let ipsr;
    // SAFETY: reading IPSR has no side effects and identifies the active exception.
    unsafe {
        core::arch::asm!("mrs {}, ipsr", out(reg) ipsr, options(nomem, nostack, preserves_flags));
    }
    ipsr
}

fn direct(context: u32) -> &'static DirectLogger {
    // NMI and HardFault always bypass ADC and normal encoder state.
    if FATAL.load(Ordering::Relaxed) || context == 2 || context == 3 {
        &EMERGENCY
    } else {
        &DIRECT
    }
}

#[defmt::global_logger]
struct Logger;

// SAFETY: ADC owns its SPSC producer. Other contexts use atomic encoder ownership.
// Fatal exceptions have separate state. Each published RTT frame is indivisible.
unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let context = context();
        if context == ADC_CONTEXT && !FATAL.load(Ordering::Relaxed) {
            ADC_LOGGER.acquire();
        } else {
            direct(context).acquire(context);
        }
    }

    unsafe fn write(bytes: &[u8]) {
        let context = context();
        if context == ADC_CONTEXT && !FATAL.load(Ordering::Relaxed) {
            ADC_LOGGER.write(bytes);
        } else {
            direct(context).write(context, bytes);
        }
    }

    unsafe fn release() {
        let context = context();
        if context == ADC_CONTEXT && !FATAL.load(Ordering::Relaxed) {
            ADC_LOGGER.release();
        } else {
            direct(context).release(context);
        }
    }

    // Direct frames are published on release. ADC frames remain asynchronous.
    // Never wait for a host or for the thread-mode consumer here.
    unsafe fn flush() {}
}

/// Install the one defmt RTT channel during boot, before enabling control IRQs.
pub fn init(channel: UpChannel) {
    assert!(
        RTT.busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    );
    // SAFETY: the initialization caller holds busy and owns the channel handle.
    unsafe {
        *RTT.channel.get() = Some(channel);
    }
    RTT.busy.store(false, Ordering::Release);
}

/// Called after gate shutdown and before logging from the panic/HardFault handler.
pub fn enter_fatal() {
    FATAL.store(true, Ordering::Relaxed);
}

/// One thread-mode consumer. The producer wakes this task when it commits a frame.
#[embassy_executor::task]
pub async fn worker() {
    let consumer = ADC_QUEUE.framed_consumer();
    let mut encoder = defmt::Encoder::new();
    let mut buffer = Buffer::new();
    loop {
        let frame = consumer.wait_read().await;
        buffer.clear();
        encoder.start_frame(|bytes| buffer.extend(bytes));
        encoder.write(&frame, |bytes| buffer.extend(bytes));
        encoder.end_frame(|bytes| buffer.extend(bytes));
        frame.release();
        buffer.send();
        // Give other thread-mode work a turn even during a sustained log burst.
        embassy_futures::yield_now().await;
    }
}
