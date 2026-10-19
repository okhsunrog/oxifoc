//! `PhaseManager` constructors and sensor-binding conversions.

use super::{HallHealth, OpenLoopOverride, PhaseManager, PhaseTracker};
#[cfg(feature = "hfi")]
use crate::foc::phase::observer::HfiObserver;
use crate::foc::phase::observer::Observer;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::phase::source::PhaseSource;
use crate::foc::phase::startup::SensorlessStartup;
use crate::foc::sensors::AngleSensor;
use crate::foc::sensors::NoSensor;
use crate::foc::trig::SinCos;
use heapless::Vec as HeaplessVec;

// ============================================================================
// Constructors
// ============================================================================

impl PhaseManager<NoSensor, NoSensor> {
    /// Create a sensorless phase manager (observer only)
    pub fn sensorless() -> Self {
        Self {
            hall: NoSensor,
            encoder: NoSensor,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Manual,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }
}

impl<H: AngleSensor> PhaseManager<H, NoSensor> {
    /// Create a phase manager with Hall sensor
    pub fn with_hall(hall: H) -> Self {
        Self {
            hall,
            encoder: NoSensor,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Hall,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::Ok,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }

    /// Add an encoder to the phase manager
    pub fn with_encoder<E2: AngleSensor>(self, encoder: E2) -> PhaseManager<H, E2> {
        PhaseManager {
            hall: self.hall,
            encoder,
            observer: self.observer,
            #[cfg(feature = "hfi")]
            hfi: self.hfi,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            open_loop_override: self.open_loop_override,
            startup: self.startup,
            tracker: self.tracker,
            crossover_latched: self.crossover_latched,
            #[cfg(feature = "hfi")]
            hfi_was_active: self.hfi_was_active,
            bemf_proxy_v: self.bemf_proxy_v,
            obs_trace_ticks: self.obs_trace_ticks,
            hold_trace_ticks: self.hold_trace_ticks,
            faults: self.faults,
        }
    }
}

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Rebind the HFI sin/cos backend (state-preserving). Firmware calls
    /// this right after construction: CORDIC on G4, FastSinCos on F405.
    pub fn with_sincos<S2: SinCos>(self) -> PhaseManager<H, E, S2> {
        PhaseManager {
            hall: self.hall,
            encoder: self.encoder,
            observer: self.observer,
            #[cfg(feature = "hfi")]
            hfi: self.hfi.map(HfiObserver::with_sincos),
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: self.source,
            output: self.output,
            manual_angle: self.manual_angle,
            open_loop_angle: self.open_loop_angle,
            open_loop_velocity: self.open_loop_velocity,
            ticks_per_sec: self.ticks_per_sec,
            hall_health: self.hall_health,
            open_loop_override: self.open_loop_override,
            startup: self.startup,
            tracker: self.tracker,
            crossover_latched: self.crossover_latched,
            #[cfg(feature = "hfi")]
            hfi_was_active: self.hfi_was_active,
            bemf_proxy_v: self.bemf_proxy_v,
            obs_trace_ticks: self.obs_trace_ticks,
            hold_trace_ticks: self.hold_trace_ticks,
            faults: self.faults,
        }
    }
}

impl<E: AngleSensor> PhaseManager<NoSensor, E> {
    /// Create a phase manager with encoder only
    pub fn with_encoder_only(encoder: E) -> Self {
        Self {
            hall: NoSensor,
            encoder,
            observer: Observer::None,
            #[cfg(feature = "hfi")]
            hfi: None,
            #[cfg(not(feature = "hfi"))]
            _sincos: core::marker::PhantomData,
            source: PhaseSource::Encoder,
            output: PhaseOutput::default(),
            manual_angle: 0.0,
            open_loop_angle: 0.0,
            open_loop_velocity: 0.0,
            ticks_per_sec: 1_000_000,
            hall_health: HallHealth::NotPresent,
            open_loop_override: OpenLoopOverride::default(),
            startup: SensorlessStartup::default(),
            tracker: PhaseTracker::default(),
            crossover_latched: false,
            #[cfg(feature = "hfi")]
            hfi_was_active: false,
            bemf_proxy_v: 0.0,
            obs_trace_ticks: 0,
            hold_trace_ticks: 0,
            faults: HeaplessVec::new(),
        }
    }
}
