//! Finite Hall capture evidence, read over SWD only after stopping the bridge.
//!
//! One TIM3 writer (or boot before it is enabled), no new critical sections.
//! Keep the first edges rather than overwriting startup evidence. The ADC
//! sequence is the latest observed loop, not a simultaneous ADC capture.

use core::sync::atomic::{AtomicU32, Ordering};
use oxifoc_core::foc::phase::{HallHealth, PhaseManager, PhaseProvider, RotorVelocityEstimate};
use oxifoc_core::foc::sensors::{AngleSensor, HallSnapshot};
use oxifoc_core::foc::trig::SinCos;
use oxifoc_core::motor::foc_driver::VelocityControlSnapshot;
use oxifoc_core::units::{ConstZero, ElectricalAngle, ElectricalAngularVelocity, ToBits};

// Covers a short 120 electrical rad/s velocity/braking capture, including
// coast. These CPU-only records cost 8 KiB in hall-diagnostics images only.
const CAPACITY: usize = 512;
static ADC_SEQUENCE: AtomicU32 = AtomicU32::new(0);

// ADC-owned, finite evidence of Hall/observer handoffs. No text formatting or
// transport work in the control interrupt; exported only after Stop.
const PHASE_CAPACITY: usize = 64;
const PHASE_WORDS: usize = 14;
#[unsafe(no_mangle)]
pub static BENCH_HALL_PHASE_EVENTS: [[AtomicU32; PHASE_WORDS]; PHASE_CAPACITY] =
    [const { [const { AtomicU32::new(0) }; PHASE_WORDS] }; PHASE_CAPACITY];
#[unsafe(no_mangle)]
pub static BENCH_HALL_PHASE_COUNT: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
pub static BENCH_VELOCITY_SAMPLES: [[AtomicU32; 8]; 128] =
    [const { [const { AtomicU32::new(0) }; 8] }; 128];
#[unsafe(no_mangle)]
pub static BENCH_VELOCITY_COUNT: AtomicU32 = AtomicU32::new(0);

#[unsafe(no_mangle)]
pub static BENCH_CROSSOVER_SAMPLES: [[AtomicU32; 8]; 320] =
    [const { [const { AtomicU32::new(0) }; 8] }; 320];
#[unsafe(no_mangle)]
pub static BENCH_CROSSOVER_COUNT: AtomicU32 = AtomicU32::new(0);

// Finite registry changes survive a warning that clears before the host polls.
// Fields: ADC sequence, generation, active category bitmask, rotor speed bits.
#[unsafe(no_mangle)]
pub static BENCH_FAULT_EVENTS: [[AtomicU32; 4]; 16] =
    [const { [const { AtomicU32::new(0) }; 4] }; 16];
#[unsafe(no_mangle)]
pub static BENCH_FAULT_COUNT: AtomicU32 = AtomicU32::new(0);

pub struct PhaseRecorder {
    previous_health: Option<HallHealth>,
    previous_angle: ElectricalAngle,
    last_velocity_sequence: u32,
    last_crossover_sequence: u32,
    fault_generation: u32,
}

impl PhaseRecorder {
    pub const fn new() -> Self {
        Self {
            previous_health: None,
            previous_angle: ElectricalAngle::ZERO,
            last_velocity_sequence: 0,
            last_crossover_sequence: 0,
            fault_generation: 0,
        }
    }

    /// Sample the selected phase and physical speed alongside the observer at 40 Hz.
    pub fn observe_crossover<H: AngleSensor, E: AngleSensor, S: SinCos>(
        &mut self,
        sequence: u32,
        phase: &PhaseManager<H, E, S>,
    ) {
        if sequence.wrapping_sub(self.last_crossover_sequence) < 500 {
            return;
        }
        self.last_crossover_sequence = sequence;
        let index = BENCH_CROSSOVER_COUNT.load(Ordering::Relaxed) as usize;
        let Some(slot) = BENCH_CROSSOVER_SAMPLES.get(index) else {
            return;
        };
        let observer = phase.observer();
        let output = phase.get();
        let flags = u32::from(observer.is_ready())
            | (u32::from(phase.angle_trustworthy()) << 1)
            | (u32::from(phase.hall_health() == HallHealth::Ok) << 2);
        let values = [
            sequence,
            flags,
            phase.bench_observer_weight().unwrap_or(f32::NAN).to_bits(),
            output.angle.to_bits(),
            output.velocity.to_bits(),
            bits(phase.rotor_velocity().map(RotorVelocityEstimate::velocity)),
            bits(observer.phase()),
            bits(observer.velocity()),
        ];
        for (word, value) in slot.iter().zip(values) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_CROSSOVER_COUNT.store(index as u32 + 1, Ordering::Release);
    }

    pub fn observe_faults(
        &mut self,
        sequence: u32,
        speed: impl FnOnce() -> Option<ElectricalAngularVelocity>,
    ) {
        use oxifoc_core::foc::fault::PlatformFault;
        let registry = &crate::FAULT_REGISTRY;
        let generation = registry.generation();
        if generation == self.fault_generation {
            return;
        }
        self.fault_generation = generation;
        let index = BENCH_FAULT_COUNT.load(Ordering::Relaxed) as usize;
        let Some(slot) = BENCH_FAULT_EVENTS.get(index) else {
            return;
        };
        // ADC owns execution here; no lower-priority registry writer can
        // intervene. No strings, allocation or transport in the snapshot.
        let mask = registry
            .snapshot()
            .iter()
            .fold(0_u32, |mask, fault| mask | (1 << fault.category() as u32));
        for (word, value) in slot.iter().zip([sequence, generation, mask, bits(speed())]) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_FAULT_COUNT.store(index as u32 + 1, Ordering::Release);
    }

    /// Native ADC sequence spacing gives ~40 Hz at the nominal 20 kHz loop.
    /// The finite prefix covers a short maneuver; no transport or formatting.
    pub fn observe_velocity(
        &mut self,
        sequence: u32,
        snapshot: impl FnOnce() -> Option<VelocityControlSnapshot>,
    ) {
        if sequence.wrapping_sub(self.last_velocity_sequence) < 1000 {
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
            sample.target.to_bits(),
            sample.reference.to_bits(),
            bits(sample.feedback),
            sample.requested_iq.to_bits(),
            sample.granted_iq.to_bits(),
            sample.measured_iq.to_bits(),
            sample.granted_id.to_bits(),
        ];
        for (word, value) in slot.iter().zip(values) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_VELOCITY_COUNT.store(index as u32 + 1, Ordering::Release);
    }

    pub fn observe<H: AngleSensor, E: AngleSensor, S: SinCos>(
        &mut self,
        sequence: u32,
        phase: &PhaseManager<H, E, S>,
        hall: Option<HallSnapshot>,
    ) {
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
        let observer = phase.debug_observer();
        let validity = phase.observer().validity();
        let values = [
            sequence,
            health_code,
            u32::from(phase.angle_trustworthy()),
            previous_angle.to_bits(),
            output.angle.to_bits(),
            output.velocity.to_bits(),
            bits(observer.map(|o| o.0)),
            bits(observer.map(|o| o.2)),
            bits(observer.map(|o| o.3)),
            bits(hall.map(|h| h.angle)),
            bits(hall.map(|h| h.velocity)),
            bits(hall.map(|h| h.edge_age)),
            bits(validity.map(|v| v.0)),
            bits(validity.map(|v| v.1)),
        ];
        for (word, value) in slot.iter().zip(values) {
            word.store(value, Ordering::Relaxed);
        }
        BENCH_HALL_PHASE_COUNT.store(index as u32 + 1, Ordering::Release);
    }
}

/// The capture word of a sample; a missing sample reads back as NaN.
fn bits(sample: Option<impl ToBits>) -> u32 {
    sample.map_or(f32::NAN.to_bits(), ToBits::to_bits)
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

/// Wrapping totals latched by the 1-Hz stats task: windows and producer
/// drops. Retained when log lines drop. This adds no control-cycle work; the
/// current partial window is excluded.
#[unsafe(no_mangle)]
pub static BENCH_HALL_STREAM_LOSSES: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

pub fn observe_stream_stats(producer: u32) {
    for (counter, delta) in BENCH_HALL_STREAM_LOSSES.iter().zip([1, producer]) {
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
