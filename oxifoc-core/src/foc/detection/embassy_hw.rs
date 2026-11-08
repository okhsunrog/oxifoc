//! Embassy-based detection hardware implementation
//!
//! Provides [`EmbassyDetectionHardware`] and [`EmbassyHallReader`] for platforms
//! using embassy-time and critical-section mutexes for motor parameter detection.

#![allow(dead_code)]

use core::cell::RefCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::Poll;

use critical_section::Mutex as CriticalSectionMutex;

use crate::foc::config::BoardConfig;
use crate::foc::controller::FocOutput;
use crate::foc::detection::DetectionError;
use crate::foc::detection::sweep::{self, DetectionHardware, HallReader};
use crate::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use crate::foc::trig::SinCos;
use crate::motor::ControlMode;
use crate::state::{self, MotorControlState};
use crate::timer::EmbassyTimer;

// Re-export types from core for convenience
pub use crate::foc::detection::sweep::{DetectionParams, DetectionResult};
pub use crate::foc::detection::{FluxLinkageParams, InductanceParams, ResistanceParams};

// ============================================================================
// Hardware Implementation
// ============================================================================

/// Embassy-based hardware abstraction for motor detection.
///
/// Generic implementation usable on any platform with embassy-time,
/// critical-section mutexes for state, and atomic ADC samples.
pub struct EmbassyDetectionHardware {
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
}

impl EmbassyDetectionHardware {
    /// Create a new detection hardware instance.
    pub fn new(
        state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
        board: &'static BoardConfig,
    ) -> Self {
        Self {
            state_mutex,
            ia,
            ib,
            ic,
            board,
        }
    }
}

impl DetectionHardware for EmbassyDetectionHardware {
    fn current_sense_topology(&self) -> crate::types::CurrentSenseTopology {
        self.board.current_sense_topology
    }

    async fn send_command(&self, mode: ControlMode) {
        // send().await, not try_send: the ISR drains the channel every FOC
        // cycle, so this parks for at most ~50 µs — and a silently dropped
        // command would corrupt the measurement (the HFI loop pairs each
        // current sample with the voltage commanded one cycle earlier).
        state::CMD_CHANNEL
            .send(state::DriverCommand::SetMode(mode))
            .await;
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        wait_next_telemetry(self.state_mutex).await
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        let ia_raw = self.ia.load(Ordering::Relaxed);
        let ib_raw = self.ib.load(Ordering::Relaxed);
        let ic_raw = self.ic.load(Ordering::Relaxed);
        self.board.convert_raw_currents(ia_raw, ib_raw, ic_raw)
    }
}

async fn wait_next_telemetry(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
) -> FocOutput {
    let previous_seq = critical_section::with(|cs| state_mutex.borrow(cs).borrow().last_adc.seq);
    poll_fn(|cx| {
        // Re-polling alone is not proof of a new FOC cycle: the enclosing task
        // can wake for another future. Register before checking the counter so
        // a publication between the initial read and registration is not lost.
        state::TELEM_WAKER.register(cx.waker());
        critical_section::with(|cs| {
            let state = state_mutex.borrow(cs).borrow();
            if state.last_adc.seq != previous_seq {
                // ADC sequence and FOC output are published in one critical
                // section. Copy the matching output under the same guard.
                Poll::Ready(state.last_foc)
            } else {
                Poll::Pending
            }
        })
    })
    .await
}

// ============================================================================
// Hall Sensor Reader
// ============================================================================

/// Hall sensor reader that delegates to a platform-provided function.
pub struct EmbassyHallReader {
    read_fn: fn() -> u8,
}

impl EmbassyHallReader {
    /// Create a new Hall reader with a platform-specific GPIO read function.
    pub fn new(read_fn: fn() -> u8) -> Self {
        Self { read_fn }
    }
}

impl HallReader for EmbassyHallReader {
    fn read_hall_state(&self) -> u8 {
        (self.read_fn)()
    }
}

// ============================================================================
// Public API
// ============================================================================

/// Measure motor phase resistance.
pub async fn measure_resistance(
    params: &ResistanceParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = EmbassyDetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::measure_resistance::<_, EmbassyTimer>(&mut hw, params).await
}

/// Measure motor inductance: rotating HFI with voltage-pulse fallback
/// (see [`sweep::measure_inductance_auto`]) — high-resistance motors whose
/// HFI ripple sinks below the ADC floor still get a result.
pub async fn measure_inductance<S: SinCos>(
    params: &InductanceParams,
    pwm_freq_hz: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<(f32, f32), DetectionError> {
    let mut hw = EmbassyDetectionHardware::new(state_mutex, ia, ib, ic, board);
    // Experiment build (`impedance-sweep`): replace the normal L step with a
    // one-lock R(f)/L(f) frequency sweep, logged to RTT. Same safe lock + probe.
    #[cfg(feature = "impedance-sweep")]
    return sweep::measure_impedance_sweep::<_, EmbassyTimer, S>(&mut hw, params, pwm_freq_hz)
        .await;
    #[cfg(not(feature = "impedance-sweep"))]
    sweep::measure_inductance_auto::<_, EmbassyTimer, S>(&mut hw, params, pwm_freq_hz).await
}

/// Measure motor flux linkage via open-loop spinning.
///
/// Routes through [`sweep::measure_flux_linkage_auto`]: spin-down when the
/// hardware reads phase voltages during coast (none of the current boards
/// do — `EmbassyDetectionHardware` keeps the default `false`), otherwise
/// the back-EMF-vector driven method (load-angle invariant, unlike the
/// q-axis method which is biased by up to −90% in open loop);
/// `params.inductance_h` (0.0 if unknown) trims its `ωL·i` reactance term.
pub async fn measure_flux_linkage(
    params: &FluxLinkageParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = EmbassyDetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::measure_flux_linkage_auto::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection<S: SinCos>(
    params: DetectionParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<DetectionResult, DetectionError> {
    let mut hw = EmbassyDetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::run_full_detection::<_, EmbassyTimer, S>(&mut hw, params).await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
    read_hall_fn: fn() -> u8,
) -> Result<HallCalibrationResult, DetectionError> {
    let mut hw = EmbassyDetectionHardware::new(state_mutex, ia, ib, ic, board);
    let reader = EmbassyHallReader::new(read_hall_fn);
    sweep::calibrate_hall::<_, EmbassyTimer, _>(&mut hw, &reader, params).await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default(
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
    read_hall_fn: fn() -> u8,
) -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(
        HallCalibrationParams::default(),
        state_mutex,
        ia,
        ib,
        ic,
        board,
        read_hall_fn,
    )
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::sensors::AdcSnapshot;
    use core::{future::Future, pin::pin, task::Context};

    fn publish(state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>, seq: u32, vd: f32) {
        state::update_telemetry(
            state_mutex,
            AdcSnapshot::new(2048, 2048, 2048, 24_000, seq),
            None,
            FocOutput {
                vd,
                ..FocOutput::empty()
            },
        );
    }

    #[test]
    fn repeated_polls_without_a_new_adc_cycle_remain_pending() {
        let state = CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        publish(&state, 17, 0.1);
        let mut future = pin!(wait_next_telemetry(&state));
        let mut context = Context::from_waker(core::task::Waker::noop());
        assert!(future.as_mut().poll(&mut context).is_pending());
        assert!(future.as_mut().poll(&mut context).is_pending());
        publish(&state, 18, 0.2);
        let Poll::Ready(output) = future.as_mut().poll(&mut context) else {
            panic!("new ADC cycle did not complete the wait");
        };
        assert_eq!(output.vd, 0.2);
    }

    #[test]
    fn counter_wrap_returns_the_latest_matching_output() {
        let state = CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        publish(&state, u32::MAX, 0.1);
        let mut future = pin!(wait_next_telemetry(&state));
        let mut context = Context::from_waker(core::task::Waker::noop());
        assert!(future.as_mut().poll(&mut context).is_pending());
        publish(&state, 0, 0.2);
        publish(&state, 1, 0.3);
        let Poll::Ready(output) = future.as_mut().poll(&mut context) else {
            panic!("wrapped ADC counter did not complete the wait");
        };
        assert_eq!(output.vd, 0.3);
    }
}
