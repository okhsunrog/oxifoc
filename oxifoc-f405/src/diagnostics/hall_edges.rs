//! Finite Hall capture evidence, read over SWD only after stopping the bridge.
//!
//! One TIM3 writer (or boot before it is enabled), no new critical sections.
//! Keep the first edges rather than overwriting startup evidence. The ADC
//! sequence is the latest observed loop, not a simultaneous ADC capture.

use core::sync::atomic::{AtomicU32, Ordering};

// Covers a short 120 electrical rad/s velocity/braking capture, including
// coast. These CPU-only records cost 8 KiB in bench images only.
const CAPACITY: usize = 512;
static ADC_SEQUENCE: AtomicU32 = AtomicU32::new(0);

// ADC-owned, finite evidence of Hall/observer handoffs. No text formatting or
// transport work in the control interrupt; exported only after Stop.
#[cfg(feature = "bench-hall-phase")]
const PHASE_CAPACITY: usize = 64;
#[cfg(feature = "bench-hall-phase")]
const PHASE_WORDS: usize = 14;
#[cfg(feature = "bench-hall-phase")]
#[unsafe(no_mangle)]
pub static BENCH_HALL_PHASE_EVENTS: [[AtomicU32; PHASE_WORDS]; PHASE_CAPACITY] =
    [const { [const { AtomicU32::new(0) }; PHASE_WORDS] }; PHASE_CAPACITY];
#[cfg(feature = "bench-hall-phase")]
#[unsafe(no_mangle)]
pub static BENCH_HALL_PHASE_COUNT: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "bench-hall-phase")]
#[unsafe(no_mangle)]
pub static BENCH_VELOCITY_SAMPLES: [[AtomicU32; 8]; 128] =
    [const { [const { AtomicU32::new(0) }; 8] }; 128];
#[cfg(feature = "bench-hall-phase")]
#[unsafe(no_mangle)]
pub static BENCH_VELOCITY_COUNT: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "bench-hall-phase")]
pub struct PhaseRecorder {
    previous_health: Option<oxifoc_core::foc::phase::HallHealth>,
    previous_angle: f32,
    last_velocity_sequence: u32,
}

#[cfg(feature = "bench-hall-phase")]
impl PhaseRecorder {
    pub const fn new() -> Self {
        Self {
            previous_health: None,
            previous_angle: 0.0,
            last_velocity_sequence: 0,
        }
    }

    /// Native ADC sequence spacing gives ~40 Hz at the nominal 20 kHz loop.
    /// The finite prefix covers a short maneuver; no transport or formatting.
    pub fn observe_velocity(
        &mut self,
        sequence: u32,
        snapshot: impl FnOnce() -> Option<oxifoc_core::motor::foc_driver::VelocityControlSnapshot>,
    ) {
        if sequence.wrapping_sub(self.last_velocity_sequence) < 500 {
            return;
        }
        self.last_velocity_sequence = sequence;
        let index = BENCH_VELOCITY_COUNT.load(Ordering::Relaxed) as usize;
        let Some(slot) = BENCH_VELOCITY_SAMPLES.get(index) else {
            return;
        };
        let Some(sample) = snapshot() else {
            return;
        };
        let values = [
            sequence,
            sample.target_rad_s.to_bits(),
            sample.reference_rad_s.to_bits(),
            sample.feedback_rad_s.unwrap_or(f32::NAN).to_bits(),
            sample.requested_iq_a.to_bits(),
            sample.granted_iq_a.to_bits(),
            sample.measured_iq_a.to_bits(),
            sample.granted_id_a.to_bits(),
        ];
        for (word, value) in slot.iter().zip(values) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_VELOCITY_COUNT.store(index as u32 + 1, Ordering::Release);
    }

    pub fn observe<
        H: oxifoc_core::foc::sensors::AngleSensor,
        E: oxifoc_core::foc::sensors::AngleSensor,
        S: oxifoc_core::foc::trig::SinCos,
    >(
        &mut self,
        sequence: u32,
        phase: &oxifoc_core::foc::phase::PhaseManager<H, E, S>,
        hall: Option<oxifoc_core::foc::sensors::HallSnapshot>,
    ) {
        use oxifoc_core::foc::phase::{HallHealth, PhaseProvider};
        let health = phase.hall_health();
        let output = phase.get();
        let previous_angle = self.previous_angle;
        self.previous_angle = output.angle;
        if self.previous_health.replace(health) == Some(health) {
            return;
        }
        let index = BENCH_HALL_PHASE_COUNT.load(Ordering::Relaxed) as usize;
        let Some(slot) = BENCH_HALL_PHASE_EVENTS.get(index) else {
            return;
        };
        let health_code = match health {
            HallHealth::Ok => 0,
            HallHealth::Stale => 1,
            HallHealth::Invalid => 2,
            HallHealth::NotPresent => 3,
        };
        let observer = phase
            .debug_observer()
            .unwrap_or((f32::NAN, f32::NAN, f32::NAN, f32::NAN));
        let validity = phase.observer().validity().unwrap_or((f32::NAN, f32::NAN));
        let values = [
            sequence,
            health_code,
            u32::from(phase.angle_trustworthy()),
            previous_angle.to_bits(),
            output.angle.to_bits(),
            output.velocity.to_bits(),
            observer.0.to_bits(),
            observer.2.to_bits(),
            observer.3.to_bits(),
            hall.map_or(f32::NAN, |h| h.angle_rad).to_bits(),
            hall.map_or(f32::NAN, |h| h.velocity_rad_s).to_bits(),
            hall.map_or(f32::NAN, |h| h.edge_age_s).to_bits(),
            validity.0.to_bits(),
            validity.1.to_bits(),
        ];
        for (word, value) in slot.iter().zip(values) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_HALL_PHASE_COUNT.store(index as u32 + 1, Ordering::Release);
    }
}

#[repr(C)]
pub struct Edge {
    ticks_lo: AtomicU32,
    ticks_hi: AtomicU32,
    adc_sequence: AtomicU32,
    /// Raw state bits 0..2, hardware overcapture bit 8, boot seed bit 9.
    flags: AtomicU32,
}

impl Edge {
    const fn new() -> Self {
        Self {
            ticks_lo: AtomicU32::new(0),
            ticks_hi: AtomicU32::new(0),
            adc_sequence: AtomicU32::new(0),
            flags: AtomicU32::new(0),
        }
    }
}

#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGES: [Edge; CAPACITY] = [const { Edge::new() }; CAPACITY];
#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGE_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static BENCH_HALL_EDGE_DROPPED: AtomicU32 = AtomicU32::new(0);

/// Wrapping totals latched by the 1-Hz stats task: windows, producer drops,
/// malformed queue frames, broadcast failures. Retained when log lines drop.
/// This adds no control-cycle work; the current partial window is excluded.
#[unsafe(no_mangle)]
pub static BENCH_HALL_STREAM_LOSSES: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];

pub fn observe_stream_stats(producer: u32, malformed: u32, broadcast: u32) {
    for (counter, delta) in BENCH_HALL_STREAM_LOSSES
        .iter()
        .zip([1, producer, malformed, broadcast])
    {
        counter.fetch_add(delta, Ordering::Relaxed);
    }
}

#[inline]
pub fn observe_adc(sequence: u32) {
    ADC_SEQUENCE.store(sequence, Ordering::Relaxed);
}

/// `ticks` uses the same logical 1-MHz, delay-corrected domain as the estimator.
pub fn record(ticks: u64, raw: u8, overcapture: bool, boot_seed: bool) {
    let index = BENCH_HALL_EDGE_COUNT.load(Ordering::Relaxed) as usize;
    let Some(slot) = BENCH_HALL_EDGES.get(index) else {
        BENCH_HALL_EDGE_DROPPED.fetch_add(1, Ordering::Relaxed);
        return;
    };
    slot.ticks_lo.store(ticks as u32, Ordering::Relaxed);
    slot.ticks_hi.store((ticks >> 32) as u32, Ordering::Relaxed);
    slot.adc_sequence
        .store(ADC_SEQUENCE.load(Ordering::Relaxed), Ordering::Relaxed);
    slot.flags.store(
        u32::from(raw) | (u32::from(overcapture) << 8) | (u32::from(boot_seed) << 9),
        Ordering::Relaxed,
    );
    BENCH_HALL_EDGE_COUNT.store(index as u32 + 1, Ordering::Release);
}
