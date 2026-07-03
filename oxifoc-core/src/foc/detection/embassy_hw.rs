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
        // Wait for ISR to complete a FOC cycle.
        // First poll: register waker and return Pending.
        // ISR calls TELEM_WAKER.wake() → executor re-polls → Ready.
        let mut registered = false;
        poll_fn(|cx| {
            if registered {
                Poll::Ready(())
            } else {
                state::TELEM_WAKER.register(cx.waker());
                registered = true;
                Poll::Pending
            }
        })
        .await;
        // Read latest FOC output from shared state
        critical_section::with(|cs| self.state_mutex.borrow(cs).borrow().last_foc)
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        let ia_raw = self.ia.load(Ordering::Relaxed);
        let ib_raw = self.ib.load(Ordering::Relaxed);
        let ic_raw = self.ic.load(Ordering::Relaxed);
        self.board.convert_raw_currents(ia_raw, ib_raw, ic_raw)
    }
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
