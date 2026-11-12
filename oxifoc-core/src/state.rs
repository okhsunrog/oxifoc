//! Motor state management
//!
//! This module provides centralized state management for motor control.
//! Platforms instantiate the state with their own fault types.

#[cfg(feature = "storage")]
pub mod model;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;

use crate::foc::controller::{Decoupling, FocOutput};
#[cfg(feature = "offset-diagnostics")]
use crate::foc::current_offset::CurrentOffsetRequest;
use crate::foc::current_offset::{CurrentOffsetError, CurrentOffsetReport};
use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
use crate::foc::phase::{PhaseProvider, PhaseSource};
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::foc::trig::SinCos;
use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::derating::{DeratingConfig, DeratingScales};
use crate::motor::failsafe::{BRAKE_ENTRY_MAX_E_RAD_S, FailsafeConfig};
use crate::motor::foc_driver::{CurrentLimits, PreparedBoardConfig, StepError};
use crate::motor::{ControlMode, FocDriver};
use crate::types::{
    DriveOwnership, DriveSessionId, MotorCommand, MotorCommandOutcome, MotorRequest, MotorState,
};

// ============================================================================
// Global Communication Channels
// ============================================================================

#[cfg(feature = "storage")]
static MOTOR_TEMPERATURE_INPUT_ENABLED: AtomicBool = AtomicBool::new(true);

/// Board samplers consult this before publishing a motor temperature sample.
#[cfg(feature = "storage")]
pub fn motor_temperature_input_enabled() -> bool {
    MOTOR_TEMPERATURE_INPUT_ENABLED.load(Ordering::Relaxed)
}

#[cfg(feature = "storage")]
pub fn set_motor_temperature_input(config: crate::storage::MotorTemperatureConfig) {
    MOTOR_TEMPERATURE_INPUT_ENABLED.store(
        config.sensor == crate::storage::MotorTemperatureSensor::BoardDefault,
        Ordering::Relaxed,
    );
}

/// Command for the ISR-owned FocDriver.
///
/// The driver is mutated only inside the FOC ISR; every async-side request
/// to change it goes through [`CMD_CHANNEL`] and is applied in order by
/// [`process_commands`]. One channel (rather than per-purpose signals)
/// keeps the commands sequenced: "set limits, then start" arrives exactly
/// that way.
#[derive(Clone, Copy, Debug)]
pub enum DriverCommand {
    BeginCommissioning {
        token: DetectionCommandToken,
        prepared: crate::motor::foc_driver::PreparedCommissioning,
    },
    EndCommissioning {
        token: DetectionCommandToken,
    },
    /// Device-side excitation belongs to one detection generation.
    DetectionMode {
        token: DetectionCommandToken,
        mode: ControlMode,
    },
    /// Change control mode (start/stop/targets)
    SetMode(ControlMode),
    /// Source-owned, sequenced protocol command.
    Motor(MotorRequest),
    /// Apply current limits (already clamped to the board ceiling)
    SetCurrentLimits(CurrentLimits),
    /// Apply current-loop PI gains (post-detection tune, config write)
    SetPiGains {
        kp: f32,
        ki: f32,
    },
    /// Apply dq-decoupling/back-EMF feedforward params (post-detection)
    SetDecoupling(Decoupling),
    /// Small token referencing the single-flight prepared-model mailbox.
    #[cfg(feature = "storage")]
    ApplyMotorModel(u32),
    /// Switch the angle source (hall / observer / HFI / crossovers)
    SetPhaseSource(PhaseSource),
    /// Apply failsafe tuning (deadman timeout + reaction policy + brake params)
    SetFailsafe(FailsafeConfig),
    /// Apply cruise velocity-loop tuning (gains + accel limit)
    SetVelocityConfig(VelocityLoopConfig),
    /// Apply graduated derating ramps (thermal/voltage/speed)
    SetDerating(DeratingConfig),
    /// Begin a single-flight, ISR-owned current-offset measurement.
    #[cfg(feature = "offset-diagnostics")]
    StartCurrentOffsetCalibration(CurrentOffsetRequest),
    /// Apply persisted/host-supplied offsets to the live current sensor.
    SetCurrentOffsets([f32; 3]),
}

impl DriverCommand {
    /// All numeric payloads are finite (and gains/limits positive where
    /// zero makes no sense). Wire input is arbitrary bits — non-finite
    /// commands are dropped at the boundary instead of feeding NaN into
    /// the control loop.
    pub fn is_sane(&self) -> bool {
        match *self {
            Self::BeginCommissioning { .. } | Self::EndCommissioning { .. } => true,
            Self::DetectionMode { mode, .. } => mode.is_finite(),
            Self::SetMode(mode) => mode.is_finite(),
            Self::Motor(request) => request.command.is_finite(),
            Self::SetCurrentLimits(limits) => {
                limits.max_current_a.is_finite()
                    && limits.overcurrent_threshold_a.is_finite()
                    && limits.bus_in_max_a.is_finite()
                    && limits.bus_regen_max_a.is_finite()
            }
            Self::SetPiGains { kp, ki } => {
                kp.is_finite() && ki.is_finite() && kp > 0.0 && ki >= 0.0
            }
            Self::SetDecoupling(d) => d.is_valid(),
            #[cfg(feature = "storage")]
            Self::ApplyMotorModel(_) => true,
            Self::SetPhaseSource(source) => source.is_finite(),
            Self::SetFailsafe(cfg) => cfg.is_sane(),
            Self::SetVelocityConfig(cfg) => cfg.is_sane(),
            Self::SetDerating(cfg) => cfg.is_sane(),
            #[cfg(feature = "offset-diagnostics")]
            Self::StartCurrentOffsetCalibration(request) => request.is_valid(),
            Self::SetCurrentOffsets(offsets) => offsets.iter().all(|value| value.is_finite()),
        }
    }
}

/// Command channel - servers send DriverCommands here, ISR receives them
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, DriverCommand, 8> = Channel::new();

#[derive(Clone, Copy, PartialEq, Eq)]
enum CommissioningTransition {
    Begin,
    End,
}

struct CommissioningAck {
    generation: u32,
    transition: CommissioningTransition,
    accepted: bool,
}

static COMMISSIONING_DONE: Signal<CriticalSectionRawMutex, CommissioningAck> = Signal::new();
static DETECTION_SOURCE_LIMITED: AtomicBool = AtomicBool::new(false);
static DETECTION_SOURCE_CONSTRAINED: AtomicBool = AtomicBool::new(false);

pub fn detection_source_constrained() -> bool {
    DETECTION_SOURCE_CONSTRAINED.load(Ordering::Acquire)
}

pub fn detection_source_limited() -> bool {
    DETECTION_SOURCE_LIMITED.load(Ordering::Acquire)
}

/// Completion of the single serialized protocol motor request. The motor
/// server resets this before enqueue; `Signal` retains an ISR completion that
/// arrives before the async waiter is polled.
pub static MOTOR_COMMAND_DONE: Signal<
    CriticalSectionRawMutex,
    (DriveSessionId, u32, MotorCommandOutcome),
> = Signal::new();

/// Single-flight completion for current-offset calibration. Boot starts the
/// driver operation directly; runtime diagnostics enqueue the typed command.
pub static CURRENT_OFFSET_DONE: Signal<
    CriticalSectionRawMutex,
    Result<CurrentOffsetReport, CurrentOffsetError>,
> = Signal::new();

/// True until platform boot calibration completes. F405 starts protocol
/// servers before FOC initialisation, so runtime offset requests use this to
/// fail Busy without resetting/consuming the boot completion signal.
pub static BOOT_CURRENT_OFFSET_PENDING: AtomicBool = AtomicBool::new(true);

pub fn boot_current_offset_pending() -> bool {
    BOOT_CURRENT_OFFSET_PENDING.load(Ordering::Acquire)
}

/// Closes the async-command TOCTOU window before the ISR mirrors a runtime
/// calibration into [`MotorControlState::driver_operation`].
#[cfg(feature = "offset-diagnostics")]
pub static CURRENT_OFFSET_REQUEST_PENDING: AtomicBool = AtomicBool::new(false);

pub fn current_offset_request_pending() -> bool {
    #[cfg(feature = "offset-diagnostics")]
    {
        CURRENT_OFFSET_REQUEST_PENDING.load(Ordering::Acquire)
    }
    #[cfg(not(feature = "offset-diagnostics"))]
    {
        false
    }
}

/// A model replacement owns the stopped bridge until its durable commit.
pub fn model_apply_pending() -> bool {
    #[cfg(feature = "storage")]
    {
        model::model_apply_pending()
    }
    #[cfg(not(feature = "storage"))]
    {
        false
    }
}

/// Waker for FOC cycle completion — ISR wakes after `update_telemetry()`.
/// Used by calibration/detection to synchronize with individual FOC cycles.
/// The listener reads `last_foc` from the state mutex after waking.
pub static TELEM_WAKER: CycleNotification = CycleNotification::new();

/// A single-cycle subscription, unlike AtomicWaker's persistent registration.
pub struct CycleNotification {
    waiting: AtomicBool,
    waker: AtomicWaker,
}

impl CycleNotification {
    const fn new() -> Self {
        Self {
            waiting: AtomicBool::new(false),
            waker: AtomicWaker::new(),
        }
    }

    pub fn register(&self, waker: &core::task::Waker) {
        self.waker.register(waker);
        self.waiting.store(true, Ordering::Release);
    }

    pub fn cancel(&self) {
        self.waiting.store(false, Ordering::Release);
    }

    pub fn wake(&self) {
        // Only an idle hint: the subscriber rechecks its sequence after
        // registering. The exchange still owns wake duty on the armed path.
        if self.waiting.load(Ordering::Relaxed) && self.waiting.swap(false, Ordering::AcqRel) {
            self.waker.wake();
        }
    }
}

/// True while a flash operation is queued or running. Internal-flash erase
/// stalls the whole chip (code executes from the same flash), so it must
/// never overlap an energized motor.
///
/// Closes the TOCTOU gap in the config server's Busy check from both ends:
/// the server arms this flag *before* re-checking the motor state, and
/// [`process_commands`] refuses to start the motor while it is set — so
/// whichever side wins the race, the unsafe overlap is impossible.
pub static FLASH_OP_PENDING: AtomicBool = AtomicBool::new(false);

static DRIVE_SETUP_RECOVERY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "storage")]
pub(crate) fn set_drive_setup_recovery(required: bool) {
    DRIVE_SETUP_RECOVERY.store(required, Ordering::Release);
}

#[cfg(feature = "storage")]
pub(crate) fn drive_setup_recovery_required() -> bool {
    DRIVE_SETUP_RECOVERY.load(Ordering::Acquire)
}

/// Set while a device-side detection step is actively driving the motor.
///
/// Detection is host-initiated but device-driven: the host then blocks on the
/// result and sends no frames, so the ergot interface liveness times out within
/// `LIVENESS_TIMEOUT_MS` and the **link-loss** failsafe would otherwise cut the
/// drive mid-measurement (current collapses, the failsafe latches) and fight the
/// (bounded, current-limited, rotor-locked/slow) detection. The detect server
/// raises this around each measurement so the link-loss path is suspended.
///
/// The command-staleness deadman still covers detection, but with the LONG
/// [`crate::motor::foc_driver::BENCH_STALENESS_TIMEOUT_US`] bound that
/// OpenLoop/DirectVoltage always get (detection dwells up to ~5 s between
/// device-side `SetMode`s — settle waits, sample windows, the flux-spin
/// collection). A truly hung measurement (telemetry dead, no commands
/// flowing) is cut ~10 s in. Historical note: until 2026-07-06 those modes
/// were exempt from the deadman entirely, so this flag used to disable BOTH
/// comm failsafes despite this comment claiming otherwise. The instantaneous
/// over-current check was never gated by it.
pub static DETECTION_ACTIVE: AtomicBool = AtomicBool::new(false);
static DETECTION_CANCELLED: AtomicBool = AtomicBool::new(false);

pub fn detection_cancelled() -> bool {
    DETECTION_ACTIVE.load(Ordering::Acquire) && DETECTION_CANCELLED.load(Ordering::Acquire)
}

fn cancel_detection() {
    critical_section::with(|_| {
        if DETECTION_ACTIVE.load(Ordering::Acquire) {
            DETECTION_CANCELLED.store(true, Ordering::Release);
        }
    });
}

/// RAII guard for [`DETECTION_ACTIVE`]: clears the flag on every exit path.
/// Without it, any future `select`/timeout wrapper around the measurement
/// future would strand the flag on cancellation and leave the link-loss
/// failsafe disabled forever.
pub struct DetectionActiveGuard {
    generation: u32,
}

static DETECTION_GENERATION: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// A capability captured when a detection backend is constructed. The field
/// is private: an old backend cannot mint authority for a subsequent run.
#[derive(Clone, Copy, Debug)]
pub struct DetectionCommandToken(u32);

impl DetectionCommandToken {
    #[cfg(any(feature = "embassy", test))]
    pub(crate) fn capture() -> Option<Self> {
        critical_section::with(|_| {
            (DETECTION_ACTIVE.load(Ordering::Acquire)
                && !DETECTION_CANCELLED.load(Ordering::Acquire))
            .then(|| Self(DETECTION_GENERATION.load(Ordering::Acquire)))
        })
    }

    pub(crate) fn is_current(self) -> bool {
        critical_section::with(|_| {
            DETECTION_ACTIVE.load(Ordering::Acquire)
                && !DETECTION_CANCELLED.load(Ordering::Acquire)
                && DETECTION_GENERATION.load(Ordering::Acquire) == self.0
        })
    }
}

impl DetectionActiveGuard {
    /// Revoke only this generation; ISR cleanup rejects its queued work.
    pub(crate) fn cancel(&self) {
        critical_section::with(|_| {
            if DETECTION_GENERATION.load(Ordering::Acquire) == self.generation {
                DETECTION_CANCELLED.store(true, Ordering::Release);
            }
        });
    }

    pub(crate) fn command_token(&self) -> DetectionCommandToken {
        DetectionCommandToken(self.generation)
    }

    pub(crate) async fn begin<T: crate::timer::Timer>(
        &self,
        prepared: crate::motor::foc_driver::PreparedCommissioning,
    ) -> bool {
        self.change_context::<T>(Some(prepared)).await
    }

    pub(crate) async fn end<T: crate::timer::Timer>(&self) -> bool {
        self.change_context::<T>(None).await
    }

    async fn change_context<T: crate::timer::Timer>(
        &self,
        prepared: Option<crate::motor::foc_driver::PreparedCommissioning>,
    ) -> bool {
        use embassy_futures::select::{Either, select};
        let transition = if prepared.is_some() {
            CommissioningTransition::Begin
        } else {
            CommissioningTransition::End
        };
        let command = match prepared {
            Some(prepared) => DriverCommand::BeginCommissioning {
                token: self.command_token(),
                prepared,
            },
            None => DriverCommand::EndCommissioning {
                token: self.command_token(),
            },
        };
        let operation = async {
            CMD_CHANNEL.send(command).await;
            loop {
                let ack = COMMISSIONING_DONE.wait().await;
                if ack.generation == self.generation && ack.transition == transition {
                    return ack.accepted;
                }
            }
        };
        match select(operation, T::after_millis(500)).await {
            Either::First(accepted) => accepted,
            Either::Second(()) => false,
        }
    }
    pub fn arm() -> Self {
        Self::try_arm().expect("one detection operation owns the device")
    }

    pub fn try_arm() -> Option<Self> {
        critical_section::with(|_| {
            if DETECTION_ACTIVE.load(Ordering::Acquire) {
                return None;
            }
            #[cfg(feature = "storage")]
            if model_apply_pending()
                || FLASH_OP_PENDING.load(Ordering::Acquire)
                || DRIVE_SETUP_RECOVERY.load(Ordering::Acquire)
            {
                return None;
            }
            let generation = DETECTION_GENERATION.load(Ordering::Relaxed).wrapping_add(1);
            DETECTION_GENERATION.store(generation, Ordering::Release);
            DETECTION_CANCELLED.store(false, Ordering::Release);
            DETECTION_SOURCE_LIMITED.store(false, Ordering::Release);
            DETECTION_SOURCE_CONSTRAINED.store(false, Ordering::Release);
            DETECTION_ACTIVE.store(true, Ordering::Release);
            Some(Self { generation })
        })
    }
}

impl Drop for DetectionActiveGuard {
    fn drop(&mut self) {
        critical_section::with(|_| {
            if DETECTION_GENERATION.load(Ordering::Acquire) == self.generation {
                DETECTION_ACTIVE.store(false, Ordering::Release);
            }
        });
    }
}

/// RAII guard for [`FLASH_OP_PENDING`]: clears the flag on every exit
/// path, including the early returns on flash errors.
pub struct FlashPendingGuard(());

impl FlashPendingGuard {
    pub fn arm() -> Self {
        FLASH_OP_PENDING.store(true, Ordering::SeqCst);
        Self(())
    }
}

impl Drop for FlashPendingGuard {
    fn drop(&mut self) {
        FLASH_OP_PENDING.store(false, Ordering::SeqCst);
    }
}

// ============================================================================
// State Structure
// ============================================================================

/// Apply the active board correction exactly once at the measurement source.
/// Control, protection and all telemetry consume this same voltage afterwards.
#[cfg(feature = "storage")]
pub fn calibrated_vbus_mv(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    nominal_mv: u32,
) -> u32 {
    critical_section::with(|cs| {
        state_mutex
            .borrow(cs)
            .borrow()
            .board_calibration
            .correct_vbus_mv(nominal_mv)
    })
}

/// Internal actuator ownership outside the public motor control modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverOperation {
    Idle,
    CurrentOffsetCalibration,
}

/// Motor control state
///
/// This struct holds all telemetry-related state that servers need to access.
/// It's updated by the platform's ISR and read by the protocol servers.
#[derive(Clone, Debug)]
pub struct MotorControlState {
    #[cfg(feature = "storage")]
    pub board_calibration: crate::storage::BoardCalibrationConfig,
    #[cfg(feature = "storage")]
    pub board_calibration_valid: bool,
    /// Stored Hall reference validity; gates only Hall-dependent actuation.
    #[cfg(feature = "storage")]
    pub hall_calibration_valid: bool,
    /// Current motor state (Stopped/Running/Error)
    pub motor_state: MotorState,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Last Hall sensor snapshot
    pub last_hall: Option<HallSnapshot>,
    /// Last ADC snapshot
    pub last_adc: AdcSnapshot,
    /// Last FOC telemetry
    pub last_foc: FocOutput,
    /// Link active flag (host has connected)
    pub link_active: bool,
    /// Active phase source (mirrors the driver's PhaseManager; updated when
    /// a SetPhaseSource command is applied)
    pub phase_source: PhaseSource,
    /// Pole-pair count from the actually installed model, shared by telemetry.
    pub motor_pole_pairs: u8,
    /// Last successfully applied live model transaction (0 at boot).
    pub motor_model_generation: u32,
    /// Live derating scales (mirrored from the driver each cycle so the
    /// slow-telemetry server can report them without touching the driver)
    pub derating: DeratingScales,
    /// ISR-owned maintenance operation. Configuration/flash and motor starts
    /// use this to avoid racing an energised calibration topology.
    pub driver_operation: DriverOperation,
    /// Source currently allowed to send active setpoints. Safe-mode and
    /// emergency commands are never owner-gated.
    drive_ownership: DriveOwnership,
    #[cfg(feature = "detection")]
    pub(crate) excitation: crate::foc::detection::excitation::Excitation,
}

impl MotorControlState {
    /// Create new state (const for static initialization)
    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "storage")]
            board_calibration: crate::storage::BoardCalibrationConfig::IDENTITY,
            #[cfg(feature = "storage")]
            board_calibration_valid: true,
            #[cfg(feature = "storage")]
            hall_calibration_valid: true,
            motor_state: MotorState::Stopped,
            control_mode: ControlMode::Stopped,
            last_hall: None,
            last_adc: AdcSnapshot::empty(),
            last_foc: FocOutput::empty(),
            link_active: false,
            phase_source: PhaseSource::Hall,
            motor_pole_pairs: 0,
            motor_model_generation: 0,
            derating: DeratingScales::IDENTITY,
            driver_operation: DriverOperation::Idle,
            drive_ownership: DriveOwnership::new(),
            #[cfg(feature = "detection")]
            excitation: crate::foc::detection::excitation::Excitation::new(),
        }
    }

    /// Restore calibration before starting the ADC/control loop. Missing is
    /// identity; unreadable/invalid is a persistent start interlock until repaired.
    #[cfg(feature = "storage")]
    pub fn load_board_calibration(&mut self, config: &crate::storage::RuntimeConfig) {
        let value = config.board_calibration.unwrap_or_default();
        self.board_calibration_valid = config.board_calibration_is_valid();
        self.board_calibration = if self.board_calibration_valid {
            value
        } else {
            crate::storage::BoardCalibrationConfig::IDENTITY
        };
    }

    /// Load the stored Hall-reference interlock before control/command tasks.
    /// Missing legacy configuration permits the nominal reference; corrupt or
    /// invalid stored data requires a successful detection/config repair.
    #[cfg(feature = "storage")]
    pub fn load_hall_calibration(&mut self, config: &crate::storage::RuntimeConfig) {
        self.hall_calibration_valid = config.hall_calibration_is_valid();
    }

    /// Called under the same critical section as the live config/revision
    /// update. The ADC ISR sees the complete new value at its next boundary.
    #[cfg(feature = "storage")]
    pub fn can_apply_board_calibration(&self) -> bool {
        self.motor_state == MotorState::Stopped
            && self.control_mode == ControlMode::Stopped
            && self.driver_operation == DriverOperation::Idle
            && !boot_current_offset_pending()
            && !current_offset_request_pending()
            && !model_apply_pending()
            && !DETECTION_ACTIVE.load(Ordering::Acquire)
            && crate::runtime::streaming::FAST_TELEM_PERIOD.load(Ordering::Acquire) == 0
    }

    /// Set motor to stopped state.
    ///
    /// Deliberately does NOT exit [`MotorState::Error`]: a Stop command (or
    /// the link-loss failsafe, which routes through here) must not un-latch
    /// faults — only [`clear_error`](Self::clear_error) may, after the host
    /// explicitly cleared the fault registry.
    pub fn set_stopped(&mut self) {
        if self.motor_state != MotorState::Error {
            self.motor_state = MotorState::Stopped;
        }
        self.control_mode = ControlMode::Stopped;
        self.drive_ownership.release();
    }

    /// Set motor to running with given control mode
    pub fn set_running(&mut self, mode: ControlMode) {
        self.motor_state = MotorState::Running;
        self.control_mode = mode;
    }

    /// Set motor to error state
    pub fn set_error(&mut self) {
        self.motor_state = MotorState::Error;
    }

    /// Clear error and return to stopped state
    pub fn clear_error(&mut self) {
        self.motor_state = MotorState::Stopped;
        self.control_mode = ControlMode::Stopped;
        self.drive_ownership.release();
    }

    /// Update Hall snapshot
    pub fn update_hall(&mut self, hall: HallSnapshot) {
        self.last_hall = Some(hall);
    }

    /// Update ADC snapshot
    pub fn update_adc(&mut self, adc: AdcSnapshot) {
        self.last_adc = adc;
    }

    /// Update FOC telemetry
    pub fn update_foc(&mut self, foc: FocOutput) {
        self.last_foc = foc;
    }

    /// Mark link as active (host connected)
    pub fn set_link_active(&mut self) {
        self.link_active = true;
    }

    /// Mark link as inactive (host disconnected / liveness timed out).
    /// [`process_commands`] forces [`ControlMode::Stopped`] while inactive.
    pub fn set_link_inactive(&mut self) {
        self.link_active = false;
        self.drive_ownership.release();
    }

    /// Whether any operation currently owns or energises the bridge.
    pub fn actuator_busy(&self) -> bool {
        self.motor_state == MotorState::Running
            || self.driver_operation != DriverOperation::Idle
            || boot_current_offset_pending()
            || current_offset_request_pending()
            || model_apply_pending()
    }
}

impl Default for MotorControlState {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Platform State - Platforms define this
// ============================================================================

/// Platform must define this macro to provide state globals
///
/// Example usage in platform crate:
/// ```ignore
/// use oxifoc_core::define_platform_state;
/// define_platform_state!(MyFault);
/// ```
/// Optional attributes apply only to STATE, for platform-specific memory placement.
#[macro_export]
macro_rules! define_platform_state {
    ($fault_type:ty $(, #[$state_attr:meta])*) => {
        /// Global motor state
        $(#[$state_attr])*
        pub static STATE: ::critical_section::Mutex<
            ::core::cell::RefCell<$crate::state::MotorControlState>,
        > = ::critical_section::Mutex::new(::core::cell::RefCell::new(
            $crate::state::MotorControlState::new(),
        ));

        /// Global fault registry
        pub static FAULT_REGISTRY: $crate::foc::fault::FaultRegistry<$fault_type> =
            $crate::foc::fault::FaultRegistry::new();
    };
}

// ============================================================================
// Command Processing
// ============================================================================

/// Process pending commands from the channel
///
/// Call this from the ISR before running FOC. It processes any pending
/// ControlMode commands and applies them to the driver.
///
/// # Arguments
/// * `state_mutex` - Reference to the platform STATE global
/// * `foc` - Mutable reference to the FocDriver
///
/// # Returns
/// The current ControlMode after processing commands
pub fn process_commands<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    foc: &mut FocDriver<P, C, Ph, S>,
    fault_registry: &FaultRegistry<F>,
) -> ControlMode
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: PlatformFault,
{
    let mut saw_set_mode = false;
    let link_active = critical_section::with(|cs| state_mutex.borrow(cs).borrow().link_active);
    process_commands_inner(
        state_mutex,
        foc,
        fault_registry,
        &mut saw_set_mode,
        link_active,
    )
}

fn signal_motor_completion(request: MotorRequest, outcome: MotorCommandOutcome) {
    MOTOR_COMMAND_DONE.signal((request.source_session, request.seq, outcome));
}

/// Like [`process_commands`] but reports whether a `SetMode` was drained on
/// this call via `saw_set_mode` — the command-staleness deadman's "fresh
/// affirmation" signal (set even for a `SetMode` the gates reject: a rejected
/// command still proves the host is alive). [`run_foc_cycle`] uses this entry
/// point; external callers and tests keep using [`process_commands`].
pub fn process_commands_inner<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    foc: &mut FocDriver<P, C, Ph, S>,
    fault_registry: &FaultRegistry<F>,
    saw_set_mode: &mut bool,
    link_active: bool,
) -> ControlMode
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: PlatformFault,
{
    // Process all pending commands
    if let Some(generation) = foc.commissioning_generation()
        && !DetectionCommandToken(generation).is_current()
    {
        foc.end_commissioning();
        critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
    }
    while let Ok(cmd) = CMD_CHANNEL.try_receive() {
        let detection_authority = match cmd {
            DriverCommand::DetectionMode { token, .. } => Some(token),
            _ => None,
        };
        let protocol_request = match cmd {
            DriverCommand::Motor(request) => Some(request),
            _ => None,
        };
        // NaN/inf payloads die here, not in the PI loop.
        if !cmd.is_sane() {
            #[cfg(feature = "defmt")]
            defmt::warn!("Dropping non-finite driver command");
            if let Some(request) = protocol_request {
                signal_motor_completion(request, MotorCommandOutcome::RejectedInvalid);
            }
            continue;
        }
        // A concurrent configuration update ends the temporary operating
        // context before touching its restoration baseline. It cannot silently
        // change the conditions of an in-progress measurement.
        if foc.commissioning_generation().is_some()
            && matches!(
                cmd,
                DriverCommand::SetCurrentLimits(_)
                    | DriverCommand::SetPiGains { .. }
                    | DriverCommand::SetDecoupling(_)
                    | DriverCommand::SetPhaseSource(_)
                    | DriverCommand::SetFailsafe(_)
                    | DriverCommand::SetVelocityConfig(_)
                    | DriverCommand::SetDerating(_)
                    | DriverCommand::SetCurrentOffsets(_)
            )
        {
            cancel_detection();
            foc.end_commissioning();
            critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
        }
        let mode = match cmd {
            DriverCommand::BeginCommissioning { token, prepared } => {
                let accepted = critical_section::with(|cs| {
                    let state = state_mutex.borrow(cs).borrow();
                    token.is_current()
                        && state.motor_state == MotorState::Stopped
                        && !fault_registry.any_stopping()
                        && !BOOT_CURRENT_OFFSET_PENDING.load(Ordering::Acquire)
                        && !current_offset_request_pending()
                        && !FLASH_OP_PENDING.load(Ordering::Acquire)
                        && !model_apply_pending()
                        && foc.begin_commissioning(token.0, prepared).is_ok()
                });
                COMMISSIONING_DONE.signal(CommissioningAck {
                    generation: token.0,
                    transition: CommissioningTransition::Begin,
                    accepted,
                });
                continue;
            }
            DriverCommand::EndCommissioning { token } => {
                if foc.commissioning_generation() == Some(token.0) {
                    foc.end_commissioning();
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
                }
                COMMISSIONING_DONE.signal(CommissioningAck {
                    generation: token.0,
                    transition: CommissioningTransition::End,
                    accepted: true,
                });
                continue;
            }
            DriverCommand::DetectionMode { token, mode } => {
                if !token.is_current() {
                    continue;
                }
                *saw_set_mode = true;
                crate::runtime::streaming::cmd_stats::SETMODE_DRAINED
                    .fetch_add(1, Ordering::Relaxed);
                mode
            }
            DriverCommand::SetMode(mode) => {
                // Fresh setpoint from the host — feed the deadman. Set even if
                // the gates below reject the mode: liveness ≠ acceptance.
                *saw_set_mode = true;
                crate::runtime::streaming::cmd_stats::SETMODE_DRAINED
                    .fetch_add(1, Ordering::Relaxed);
                mode
            }
            DriverCommand::Motor(request) => match request.command {
                MotorCommand::EmergencyStop => {
                    cancel_detection();
                    foc.end_commissioning();
                    foc.emergency_stop();
                    critical_section::with(|cs| {
                        state_mutex.borrow(cs).borrow_mut().set_stopped();
                    });
                    publish_current_offset_outcome(state_mutex, foc);
                    signal_motor_completion(request, MotorCommandOutcome::Applied);
                    continue;
                }
                MotorCommand::SetMode(mode) => {
                    let authorization = critical_section::with(|cs| {
                        let state = state_mutex.borrow(cs).borrow();
                        state.drive_ownership.authorize(request)
                    });
                    if let Err(outcome) = authorization {
                        signal_motor_completion(request, outcome);
                        continue;
                    }
                    // An authorized host override cancels the whole operation,
                    // including PI holds between finite excitation packets.
                    cancel_detection();
                    foc.end_commissioning();
                    // Only an authorized, non-duplicate request affirms the
                    // deadman. Busy/stale traffic cannot keep old torque alive.
                    *saw_set_mode = true;
                    crate::runtime::streaming::cmd_stats::SETMODE_DRAINED
                        .fetch_add(1, Ordering::Relaxed);
                    mode
                }
            },
            #[cfg(feature = "storage")]
            DriverCommand::ApplyMotorModel(generation) => {
                let allowed = critical_section::with(|cs| {
                    let state = state_mutex.borrow(cs).borrow();
                    state.motor_state == MotorState::Stopped
                        && state.control_mode == ControlMode::Stopped
                        && state.driver_operation == DriverOperation::Idle
                        && !boot_current_offset_pending()
                        && !current_offset_request_pending()
                        && !FLASH_OP_PENDING.load(Ordering::Acquire)
                        && !fault_registry.any_stopping()
                });
                model::apply(generation, foc, state_mutex, allowed);
                continue;
            }
            DriverCommand::SetCurrentLimits(limits) => {
                foc.set_current_limits(limits);
                continue;
            }
            DriverCommand::SetPiGains { kp, ki } => {
                foc.set_current_pi_gains(kp, ki);
                continue;
            }
            DriverCommand::SetDecoupling(d) => {
                foc.set_decoupling(Some(d));
                continue;
            }
            DriverCommand::SetPhaseSource(source) => {
                #[cfg(feature = "storage")]
                if source.requires_hall()
                    && !critical_section::with(|cs| {
                        state_mutex.borrow(cs).borrow().hall_calibration_valid
                    })
                {
                    continue;
                }
                // The provider validates (sensor present, estimators
                // configured). On success mirror the active source into the
                // shared state so the host can read it back via telemetry —
                // an invalid request simply leaves the source unchanged.
                if foc.request_phase_source(source) {
                    critical_section::with(|cs| {
                        state_mutex.borrow(cs).borrow_mut().phase_source = source;
                    });
                } else {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("Phase source change rejected");
                }
                continue;
            }
            DriverCommand::SetFailsafe(cfg) => {
                // Config, not a setpoint — does NOT affirm the deadman.
                foc.set_failsafe(cfg);
                continue;
            }
            DriverCommand::SetVelocityConfig(cfg) => {
                // Config, not a setpoint — does NOT affirm the deadman.
                foc.set_velocity_config(cfg);
                continue;
            }
            DriverCommand::SetDerating(cfg) => {
                // Config, not a setpoint — does NOT affirm the deadman.
                foc.set_derating(cfg);
                continue;
            }
            #[cfg(feature = "offset-diagnostics")]
            DriverCommand::StartCurrentOffsetCalibration(request) => {
                let allowed = critical_section::with(|cs| {
                    let state = state_mutex.borrow(cs).borrow();
                    state.driver_operation == DriverOperation::Idle
                        && state.motor_state == MotorState::Stopped
                        && !fault_registry.any_stopping()
                        && !FLASH_OP_PENDING.load(Ordering::SeqCst)
                        && !model_apply_pending()
                });
                let result = if allowed {
                    foc.start_current_offset_calibration(request)
                } else {
                    Err(CurrentOffsetError::Busy)
                };
                match result {
                    Ok(()) => critical_section::with(|cs| {
                        state_mutex.borrow(cs).borrow_mut().driver_operation =
                            DriverOperation::CurrentOffsetCalibration;
                    }),
                    Err(error) => {
                        CURRENT_OFFSET_REQUEST_PENDING.store(false, Ordering::Release);
                        CURRENT_OFFSET_DONE.signal(Err(error));
                    }
                }
                continue;
            }
            DriverCommand::SetCurrentOffsets(offsets) => {
                foc.set_current_offsets((offsets[0], offsets[1], offsets[2]));
                continue;
            }
        };

        // A mode command cannot take the bridge away from calibration. Stop
        // is the exception: it is the universal cancellation/safe-off verb.
        if foc.current_offset_active() {
            // Boot owns initialisation: even an early Stop from the F405's
            // already-running protocol server must not turn calibration into
            // a partially initialised device.
            if BOOT_CURRENT_OFFSET_PENDING.load(Ordering::Relaxed) {
                #[cfg(feature = "defmt")]
                defmt::warn!("Mode rejected: boot current calibration active");
                if let Some(request) = protocol_request {
                    signal_motor_completion(request, MotorCommandOutcome::RejectedSafety);
                }
                continue;
            }
            if mode != ControlMode::Stopped {
                #[cfg(feature = "defmt")]
                defmt::warn!("Mode rejected: current-offset calibration active");
                if let Some(request) = protocol_request {
                    signal_motor_completion(request, MotorCommandOutcome::RejectedSafety);
                }
                continue;
            }
            foc.cancel_current_offset_calibration();
            publish_current_offset_outcome(state_mutex, foc);
        }
        let outcome = critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();
            // Recheck at the application boundary, serialized with guard
            // cancellation/drop, not merely when dequeuing the command.
            if detection_authority.is_some_and(|token| !token.is_current()) {
                return MotorCommandOutcome::RejectedSafety;
            }

            // Exit the Error latch only once the host has explicitly cleared
            // the fault registry (FaultRequest::Clear via fault_server).
            // Critical faults never auto-clear, so the acknowledgement stays
            // mandatory — but after it the next command works without a
            // separate "clear error" verb.
            if state.motor_state == MotorState::Error && !fault_registry.any() {
                state.clear_error();
            }

            // Can't change mode if in error state (must clear faults first),
            // and never start running while any stopping-class fault is
            // active even if the Error latch was missed (belt and braces:
            // the latch is set by platform glue, the registry by the fault
            // checkers). Warnings never block a start — the vehicle rides
            // through them by design.
            if mode != ControlMode::Stopped
                && (state.motor_state == MotorState::Error
                    || fault_registry.any_stopping()
                    || detection_cancelled())
            {
                return MotorCommandOutcome::RejectedSafety;
            }

            // A queued flash write/erase would stall the chip mid-spin —
            // refuse to start until it finishes (see FLASH_OP_PENDING).
            if mode != ControlMode::Stopped
                && (FLASH_OP_PENDING.load(Ordering::SeqCst)
                    || model_apply_pending()
                    || DRIVE_SETUP_RECOVERY.load(Ordering::Acquire))
            {
                #[cfg(feature = "defmt")]
                defmt::warn!("Motor start rejected: flash operation in flight");
                return MotorCommandOutcome::RejectedSafety;
            }

            #[cfg(feature = "storage")]
            if mode != ControlMode::Stopped && !state.board_calibration_valid {
                return MotorCommandOutcome::RejectedSafety;
            }

            #[cfg(feature = "storage")]
            if !state.hall_calibration_valid
                && foc.phase().requires_hall_reference()
                && (matches!(
                    mode,
                    ControlMode::CurrentControl { .. }
                        | ControlMode::VelocityControl { .. }
                        | ControlMode::PositionControl { .. }
                ) || mode == ControlMode::Brake
                    && foc.torque_speed().abs() > BRAKE_ENTRY_MAX_E_RAD_S)
            {
                return MotorCommandOutcome::RejectedSafety;
            }

            // Brake (windings shorted) is only safe to enter near standstill:
            // at speed the short-circuit current is set by back-EMF against
            // the motor impedance (→ λ/L), outside any control loop. Above
            // the gate, substitute the controlled-stop ramp ending in Brake
            // (decel-limited regen to standstill, then the short) — same
            // machinery as the failsafe but user-commanded, so it does not
            // set the re-arm latch. Falls back to rejecting when the current
            // sensor is uncalibrated (can't current-brake).
            if mode == ControlMode::Brake && foc.torque_speed().abs() > BRAKE_ENTRY_MAX_E_RAD_S {
                if foc.enter_brake_ramp() {
                    #[cfg(feature = "defmt")]
                    defmt::info!("Brake at speed: ramping to standstill first");
                    // The stop sequencer drives PWM even when the previous
                    // driver mode was Stopped. Keep flash/config gates busy
                    // for the entire ramp and its parking-brake terminal.
                    state.set_running(ControlMode::Brake);
                    state.drive_ownership.release();
                    return MotorCommandOutcome::Applied;
                } else {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("Brake rejected: at speed and current sensor uncalibrated");
                    return MotorCommandOutcome::RejectedSafety;
                }
            }

            // After a failsafe engagement (deadman or link loss) the host
            // must acknowledge with an explicit safe mode before a running
            // mode is accepted again — "throttle back to neutral". Without
            // this, a reconnecting host replaying its last setpoint (or a
            // wedged app that resumes affirming) would relaunch the board
            // right after the failsafe stopped it. The rejected SetMode
            // still stamped the deadman above (liveness ≠ acceptance).
            if foc.failsafe_latched() {
                if !matches!(
                    mode,
                    ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
                ) {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("Mode rejected: failsafe latched, acknowledge with Stopped first");
                    return MotorCommandOutcome::RejectedSafety;
                }
                // The safe-mode command IS the acknowledgement — released
                // here at the host-command boundary only. Internal stop
                // paths (fault kill, step error) call `set_mode` directly
                // and keep the latch, so a failsafe brake that trips a
                // fault mid-stop still demands this acknowledgement.
                foc.acknowledge_failsafe();
            }

            // Apply the control mode
            match mode {
                ControlMode::Stopped => {
                    state.set_stopped();
                }
                _ => {
                    state.set_running(mode);
                }
            }
            if let Some(request) = protocol_request {
                state.drive_ownership.record_applied(request);
            }
            foc.set_mode(mode);
            MotorCommandOutcome::Applied
        });
        if let Some(request) = protocol_request {
            signal_motor_completion(request, outcome);
        }
    }

    // Fail-safe: while the link is inactive (liveness timed out / host gone),
    // bring the motor down via the configured failsafe policy regardless of
    // the last commanded mode (Coast policy reproduces the legacy hard Stop;
    // ControlledStop regen-brakes the board to rest). Runs in the ISR, so it
    // doesn't depend on any async task to react to link loss; the faster
    // command-staleness deadman in `run_foc_cycle` arms the same path. After
    // reconnect the host must acknowledge with Stopped before running again
    // (failsafe latch).
    //
    // The safe standing states are exempt, same as the deadman: a parked
    // board must stay braked through link loss (Brake), and a commanded
    // free-wheel stays a free-wheel (Coast) — "braking" Coast would drive
    // the current loop into floated phases anyway. The shared state is NOT
    // forced to stopped here: the failsafe is still actively driving the
    // motor (until standstill) — it syncs at the failsafe terminal in
    // `run_foc_cycle`, so e.g. the config server's motor-running gate can't
    // admit a flash stall mid-brake.
    if !link_active
        && !DETECTION_ACTIVE.load(Ordering::Relaxed)
        && !matches!(
            foc.mode(),
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        )
    {
        // Edge-detect on the failsafe not yet running: this gate re-fires
        // every cycle while the link stays down and would spam the ISR log.
        if !foc.failsafe_active() {
            warn!("failsafe: host link lost while running");
        }
        foc.enter_failsafe();
    }

    foc.mode()
}

/// One FOC cycle of driver work, shared by every platform ISR.
///
/// Call inside the platform's FOC_DRIVER lock after reading the ADC:
/// applies pending [`DriverCommand`]s, gates on faults, runs the FOC step
/// and checks the measured currents. Returns the cycle telemetry, or None
/// when the step was skipped (faulted) or failed.
// Faults are raised via `PlatformFault::from_category` /
// `from_hall_kind` — no per-category value parameters.
#[cfg_attr(feature = "isr-speed", optimize(speed))]
#[cfg_attr(
    all(feature = "ram-foc", target_arch = "arm", target_os = "none"),
    unsafe(link_section = ".data.ram_foc")
)]
pub fn run_foc_cycle<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &FaultRegistry<F>,
    driver: &mut FocDriver<P, C, Ph, S>,
    vbus_v: f32,
    now_ticks: u64,
    prepared_board: &PreparedBoardConfig,
) -> Option<FocOutput>
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: PlatformFault,
{
    let board = prepared_board.board();
    driver.set_vbus(vbus_v);

    let prof_t0 = crate::isr_prof::now();

    struct CycleInputs {
        link_active: bool,
        fet_t: Option<i16>,
        motor_t: Option<i16>,
        #[cfg(feature = "detection")]
        excitation_present: bool,
    }

    // One consolidated CS for everything this cycle needs from / mirrors
    // into the shared state (link flag, previous-cycle temperatures,
    // derating scales). These used to be four separate CS + RefCell
    // entries per cycle — pure lock overhead in the ISR. The derating
    // mirror is the PREVIOUS cycle's value (recomputed every 256 ticks
    // anyway; the slow-telemetry consumer cannot tell).
    let CycleInputs {
        link_active,
        fet_t,
        motor_t,
        #[cfg(feature = "detection")]
        excitation_present,
    } = critical_section::with(|cs| {
        let mut st = state_mutex.borrow(cs).borrow_mut();
        #[cfg(feature = "storage")]
        if !st.board_calibration_valid
            && let Some(fault) = F::from_category(FaultCategory::CalibrationFault)
        {
            fault_registry.set(fault);
        }
        st.derating = driver.derating();
        CycleInputs {
            link_active: st.link_active,
            fet_t: st.last_adc.fet_temp_c_x10(),
            motor_t: st.last_adc.motor_temp_c_x10(),
            // Admit a packet at the cycle boundary, including preparing,
            // completed and cancelled reservations that still need cleanup.
            // A packet installed later must wait for the next full cycle.
            #[cfg(feature = "detection")]
            excitation_present: st.excitation.program.is_some(),
        }
    });

    let prev_mode = driver.mode();
    let mut saw_set_mode = false;
    let mode = process_commands_inner(
        state_mutex,
        driver,
        fault_registry,
        &mut saw_set_mode,
        link_active,
    );
    publish_current_offset_outcome(state_mutex, driver);

    // Command-staleness deadman (ISR-resident — survives an async-executor
    // hang): stamp on a fresh setpoint; a stale link while running raises
    // the CommTimeout fault, whose GracefulStop severity the gate below
    // turns into the configured failsafe. See docs/safety.md (Layer 2) and
    // docs/notes/fault-overhaul.md. The deadman/link gate are DETECTORS;
    // the severity gate is the executor. A drained SetMode — accepted or
    // not — proves commands are flowing again and clears the fault (the
    // post-failsafe re-arm latch still demands an explicit safe mode, so
    // this auto-clear cannot relaunch the motor by itself).
    if saw_set_mode {
        driver.note_command_tick(now_ticks);
        fault_registry.clear(FaultCategory::CommTimeout);
    }
    // Layer-1 mirror: the link gate inside process_commands already armed
    // the failsafe directly (it predates the fault path and stays as belt
    // and braces); raising the fault here makes the event visible to the
    // host/remote instead of silent.
    let link_lost = !link_active;
    let in_safe_mode = matches!(
        mode,
        ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
    );
    // Link-loss is suspended during a device-side detection (see
    // `DETECTION_ACTIVE`); the command-staleness deadman is not, so a hung
    // detection still raises CommTimeout.
    let link_lost_unacked = link_lost && !in_safe_mode && !DETECTION_ACTIVE.load(Ordering::Relaxed);
    let staleness = driver.command_staleness_us(now_ticks);
    let deadman_expired = driver.deadman_expired_with(staleness, now_ticks);
    if (deadman_expired || link_lost_unacked)
        && let Some(f) = F::from_category(FaultCategory::CommTimeout)
    {
        // Log the raise edge only (the condition persists until a SetMode
        // drains), and say which detector fired — the two have very
        // different bench remedies (host affirm cadence vs transport drop).
        if !fault_registry.has_category(FaultCategory::CommTimeout) {
            warn!(
                "CommTimeout raised: deadman_expired={} link_lost={}",
                deadman_expired, link_lost_unacked
            );
        }
        fault_registry.set(f);
    }

    // Shared protection: voltage-excursion integrators, temperature
    // thresholds, graduated derating (one core implementation — the
    // per-platform ISR copies are gone). Temperatures come from the
    // previous cycle's ADC snapshot in the shared state — one cycle of
    // staleness is nothing on a thermal time scale.
    let prof_t1 = crate::isr_prof::now();
    crate::isr_prof::add(&crate::isr_prof::CYCLE_CMD, prof_t0, prof_t1);

    driver.run_protection(fault_registry, prepared_board, vbus_v, fet_t, motor_t);
    if fault_registry.any_stopping() {
        cancel_detection();
        driver.end_commissioning();
    }

    let prof_t2 = crate::isr_prof::now();
    crate::isr_prof::add(&crate::isr_prof::CYCLE_PROT, prof_t1, prof_t2);

    // Spurious break-input trips during PWM channel enable can latch an
    // OverCurrent fault right at start (seen on G431: COMP→BKIN glitch when
    // MOE re-arms). Scoped to the OverCurrent category: clearing the whole
    // registry here would bypass the host-acknowledged fault latch for
    // unrelated faults. A real latched OverCurrent cannot reach this line —
    // process_commands refuses the Stopped→active transition while any
    // stopping-class fault is registered (`any_stopping`).
    if matches!(prev_mode, ControlMode::Stopped) && mode != ControlMode::Stopped {
        fault_registry.clear(FaultCategory::OverCurrent);
    }

    // Severity gate (docs/notes/fault-overhaul.md): Kill cuts PWM now,
    // GracefulStop routes through the failsafe machinery, warnings are
    // report-only and never touch the motor.
    if fault_registry.any_kill() {
        #[cfg(feature = "detection")]
        let packet_aborted = critical_section::with(|cs| {
            state_mutex
                .borrow(cs)
                .borrow_mut()
                .excitation
                .abort(crate::foc::detection::DetectionError::HardwareFault)
        });
        #[cfg(feature = "detection")]
        if packet_aborted {
            crate::foc::detection::excitation::DONE_WAKER.wake();
        }
        // Edge log: safe_off() latches Stopped, so this fires once per trip.
        if !matches!(driver.mode(), ControlMode::Stopped) {
            error!("KILL fault active: cutting gate drive");
        }
        // Cut the gate drive NOW, re-asserted every cycle the Kill stays
        // latched. `set_mode(Stopped)` alone only floats the bridge on the
        // next step() in Stopped mode, which the early return below skips —
        // and a software-only Kill (integrated OverVoltage) has no hardware
        // break to do it for us, so without this the timer holds its last
        // commanded duty and the phases stay energized.
        driver.safe_off();
        // A Kill fault takes over from any in-progress failsafe brake (e.g.
        // the OverVoltage that regen braking can itself raise) — drop to
        // high-Z and don't resume braking into the same fault after it
        // clears.
        driver.failsafe_reset();
        // Mirror into the shared state: without this the host kept seeing
        // Running after a fault stop (and the config server's motor-running
        // gate stayed Busy forever). The Error latch is released by
        // process_commands once the host clears the registry.
        critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_error());
        publish_current_offset_outcome(state_mutex, driver);
        return None;
    }
    // GracefulStop-class: the inverter is healthy — bring the vehicle down
    // via the configured failsafe (ramp / controlled stop) instead of
    // dropping torque instantly. Safe standing states are exempt: a parked
    // Brake must persist through a fault, Stopped/Coast have nothing to
    // stop. No Error latch — restart stays blocked by the any_stopping()
    // start gate while the fault is active, plus the failsafe re-arm latch
    // (explicit safe-mode acknowledgement) after it clears.
    if fault_registry.any_stopping()
        && !matches!(
            driver.mode(),
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        )
    {
        if !driver.failsafe_active() {
            warn!("failsafe: stopping-class fault active");
        }
        driver.enter_failsafe();
    }

    let prof_t3 = crate::isr_prof::now();
    crate::isr_prof::add(&crate::isr_prof::CYCLE_CMD, prof_t2, prof_t3);

    #[cfg(feature = "detection")]
    if excitation_present {
        use crate::foc::detection::DetectionError;
        use crate::foc::detection::excitation::BurstAction;
        let next = critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();
            let cycle = state.last_adc.seq.wrapping_add(1);
            let ex = &mut state.excitation;
            if ex.cancelled && ex.program.is_some() {
                ex.program = None;
                return Some(BurstAction::Stop);
            }
            if !ex.running() {
                return None;
            }
            let program = ex.program?;
            let starting_in_hold = matches!(driver.mode(), ControlMode::OpenLoop {
                velocity_rad_s: 0.0, current, angle_rad, ..
            } if current == program.hold_current && angle_rad == program.angle);
            if fault_registry.any_stopping()
                || driver.failsafe_active()
                || saw_set_mode
                || !DETECTION_ACTIVE.load(Ordering::Relaxed)
                || ex.authority.is_some_and(|token| !token.is_current())
                || detection_cancelled()
                || (ex.index == 0 && !starting_in_hold)
            {
                ex.abort(DetectionError::HardwareFault);
                cancel_detection();
                return Some(BurstAction::Stop);
            }
            if let Err(e) = ex.check_timing(cycle, now_ticks as u32, driver.dt()) {
                ex.abort(e);
                cancel_detection();
                return Some(BurstAction::Stop);
            }
            // Copy only a small program descriptor while IRQs are masked.
            Some(if ex.index < usize::from(program.samples) {
                let (vd, vq) = ex.voltages[ex.index];
                BurstAction::Voltage {
                    vd: program.vd_hold + vd,
                    vq: program.vq_hold + vq,
                    angle: program.angle,
                    compensation_current: ex.compensation_current,
                }
            } else {
                BurstAction::Hold(program)
            })
        });
        if let Some(action) = next {
            if matches!(&action, BurstAction::Stop) {
                crate::foc::detection::excitation::DONE_WAKER.wake();
            }
            let voltage_packet = matches!(&action, BurstAction::Voltage { .. });
            let next = match action {
                BurstAction::Stop => ControlMode::Stopped,
                BurstAction::Voltage {
                    vd,
                    vq,
                    angle,
                    compensation_current,
                } => {
                    driver.set_detection_voltage(vd, vq, angle, compensation_current);
                    driver.mode()
                }
                BurstAction::Hold(program) => program.hold_mode(),
            };
            if !voltage_packet {
                driver.set_mode(next);
            }
            critical_section::with(|cs| {
                let mut state = state_mutex.borrow(cs).borrow_mut();
                if next == ControlMode::Stopped {
                    state.set_stopped();
                } else {
                    state.set_running(next);
                }
            });
        }
    }

    let was_failsafe = driver.failsafe_active();
    let step_result = driver.step(now_ticks);
    if driver.commissioning_source_constrained() {
        DETECTION_SOURCE_CONSTRAINED.store(true, Ordering::Release);
    }
    publish_current_offset_outcome(state_mutex, driver);
    let prof_t4 = crate::isr_prof::now();
    crate::isr_prof::add(&crate::isr_prof::CYCLE_STEP, prof_t3, prof_t4);
    let result = match step_result {
        Ok(telem) => {
            // Instantaneous per-phase overcurrent check against the board
            // ABS line (the dq-magnitude trip inside step() is the other
            // Kill detector; both deliberately single-sample — current
            // moves fast, unlike voltage).
            let limit = board.max_phase_current_a;
            if (telem.ia.abs() > limit || telem.ib.abs() > limit || telem.ic.abs() > limit)
                && !fault_registry.has_category(FaultCategory::OverCurrent)
                && let Some(f) = F::from_category(FaultCategory::OverCurrent)
            {
                fault_registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!(
                    "SW overcurrent FAULT: ia={}, ib={}, ic={}",
                    telem.ia,
                    telem.ib,
                    telem.ic
                );
            }
            Some(telem)
        }
        Err(_e) => {
            if matches!(_e, StepError::SourceLimit) {
                DETECTION_SOURCE_LIMITED.store(true, Ordering::Release);
                cancel_detection();
                driver.end_commissioning();
            }
            #[cfg(feature = "defmt")]
            defmt::error!("FOC step error: {}", _e);
            if matches!(_e, StepError::Overcurrent) {
                // The driver tripped its dq-magnitude protection (and cut
                // PWM itself). Surface it: latch the fault so the host sees
                // the cause and restart stays blocked until an explicit
                // clear — previously this trip was silent (the per-phase
                // registry check below only runs on Ok-telemetry, and its
                // threshold differs from the dq one).
                if let Some(f) = F::from_category(FaultCategory::OverCurrent) {
                    fault_registry.set(f);
                }
                critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_error());
            } else {
                // Couldn't run (sensor not calibrated / unimplemented
                // mode): mirror the stop so the host doesn't see a phantom
                // Running.
                critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
            }
            if mode != ControlMode::Stopped {
                driver.set_mode(ControlMode::Stopped);
            }
            driver.failsafe_reset();
            None
        }
    };

    if fault_registry.any_stopping() {
        cancel_detection();
    }
    #[cfg(feature = "detection")]
    let (excitation_done, excitation_failed) = if excitation_present {
        critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();
            let cycle = state.last_adc.seq.wrapping_add(1);
            let ex = &mut state.excitation;
            if !ex.running() {
                return (false, false);
            }
            let Some(telem) = result.filter(|_| !fault_registry.any_stopping()) else {
                ex.abort(crate::foc::detection::DetectionError::HardwareFault);
                cancel_detection();
                return (true, true);
            };
            let Some(program) = ex.program else {
                return (false, false);
            };
            let (alpha, beta) = board
                .current_sense_topology
                .clarke((telem.ia, telem.ib, telem.ic));
            if !program.current_in_budget(alpha, beta) {
                ex.abort(crate::foc::detection::DetectionError::HardwareFault);
                cancel_detection();
                return (true, true);
            }
            if ex.index == usize::from(program.samples) {
                // The retained PI hold was actually stepped before completion.
                ex.outcome = Some(Ok(()));
                return (true, false);
            } else {
                ex.samples[ex.index] = crate::foc::detection::excitation::BurstSample {
                    alpha,
                    beta,
                    cycle,
                    tick_us: now_ticks as u32,
                };
                ex.index += 1;
            }
            (false, false)
        })
    } else {
        (false, false)
    };
    #[cfg(feature = "detection")]
    if excitation_done {
        crate::foc::detection::excitation::DONE_WAKER.wake();
    }
    #[cfg(feature = "detection")]
    if excitation_failed {
        driver.safe_off();
        critical_section::with(|cs| state_mutex.borrow(cs).borrow_mut().set_stopped());
    }

    // Hall-degradation bridge — after the step, which is where the phase
    // manager refreshed its health. STICKY by design: set-only, never
    // cleared here. A sensor that lied once mid-ride stays on record until
    // the host clears it (`faults clear`), even though the live fallback
    // behavior recovers the moment the hall does; partial-failure health
    // flapping (Ok↔Invalid every revolution) must not buzz the remote at
    // rev rate. Warning class: the vehicle rides through on the fallback
    // chain (docs/notes/fault-overhaul.md §5).
    if let Some(kind) = driver.phase().hall_fault()
        && let Some(fault) = F::from_hall_kind(kind)
    {
        fault_registry.set(fault);
    }

    // The failsafe terminal transition (brake finished / aborted → Stopped,
    // or clean-stop → parking Brake) happens inside step(); mirror it into
    // the shared state so telemetry and the config server's motor-running
    // gate see the truth. While the brake is still running the state stays
    // Running — a flash stall must not be admitted mid-brake.
    if was_failsafe && !driver.failsafe_active() {
        let terminal_mode = driver.mode();
        critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();
            match terminal_mode {
                ControlMode::Stopped => state.set_stopped(),
                // ParkBrake terminal (mirrors how a direct Brake command is
                // recorded: running with mode = Brake).
                other => state.set_running(other),
            }
        });
    }

    crate::isr_prof::add(
        &crate::isr_prof::CYCLE_TAIL,
        prof_t4,
        crate::isr_prof::now(),
    );

    result
}

/// Enqueue a diagnostic current-offset measurement and await its ISR-owned
/// completion. The operation itself continues safely if this future is
/// cancelled; its state machine always floats the bridge before publishing.
#[cfg(feature = "offset-diagnostics")]
pub async fn measure_current_offsets(
    request: CurrentOffsetRequest,
) -> Result<CurrentOffsetReport, CurrentOffsetError> {
    if BOOT_CURRENT_OFFSET_PENDING.load(Ordering::Acquire) {
        return Err(CurrentOffsetError::Busy);
    }
    if CURRENT_OFFSET_REQUEST_PENDING.swap(true, Ordering::AcqRel) {
        return Err(CurrentOffsetError::Busy);
    }
    struct EnqueueGuard(bool);
    impl Drop for EnqueueGuard {
        fn drop(&mut self) {
            if self.0 {
                CURRENT_OFFSET_REQUEST_PENDING.store(false, Ordering::Release);
            }
        }
    }
    let mut guard = EnqueueGuard(true);
    CURRENT_OFFSET_DONE.reset();
    CMD_CHANNEL
        .send(DriverCommand::StartCurrentOffsetCalibration(request))
        .await;
    // From here the queued ISR operation owns clearing the pending flag,
    // even if this waiter is cancelled.
    guard.0 = false;
    CURRENT_OFFSET_DONE.wait().await
}

fn publish_current_offset_outcome<P, C, Ph, S>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    driver: &mut FocDriver<P, C, Ph, S>,
) where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
{
    if let Some(outcome) = driver.take_current_offset_outcome() {
        #[cfg(feature = "offset-diagnostics")]
        CURRENT_OFFSET_REQUEST_PENDING.store(false, Ordering::Release);
        critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();
            state.driver_operation = DriverOperation::Idle;
        });
        CURRENT_OFFSET_DONE.signal(outcome);
    }
}

/// Update state with new telemetry from ISR
///
/// Call this after running FOC step to update the global state
/// with fresh telemetry data.
pub fn update_telemetry(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    adc: AdcSnapshot,
    hall: Option<HallSnapshot>,
    foc: FocOutput,
) -> u8 {
    // Update state and return the same cycle's applied pole-pair count.
    let pole_pairs = critical_section::with(|cs| {
        let mut state = state_mutex.borrow(cs).borrow_mut();
        state.update_adc(adc);
        // None is a current unavailable/invalid reading, not permission to
        // retain an old valid speed beside a fresh ADC snapshot.
        state.last_hall = hall;
        state.update_foc(foc);
        state.motor_pole_pairs
    });

    // Wake calibration/detection task if waiting for FOC cycle
    TELEM_WAKER.wake();
    pole_pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, atomic::AtomicUsize};

    struct Counter(AtomicUsize);
    impl std::task::Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn publish_cycle(
        notification: &CycleNotification,
        state: &CriticalSectionMutex<RefCell<MotorControlState>>,
        seq: u32,
    ) {
        let mut foc = FocOutput::empty();
        foc.iq = seq as f32;
        critical_section::with(|cs| {
            let mut state = state.borrow(cs).borrow_mut();
            state.update_adc(AdcSnapshot::new(0, 0, 0, 24_000, seq));
            state.update_foc(foc);
        });
        notification.wake();
    }

    struct PublishOnWake {
        notification: Arc<CycleNotification>,
        state: Arc<CriticalSectionMutex<RefCell<MotorControlState>>>,
    }

    impl std::task::Wake for PublishOnWake {
        fn wake(self: Arc<Self>) {
            self.wake_by_ref();
        }
        fn wake_by_ref(self: &Arc<Self>) {
            publish_cycle(&self.notification, &self.state, 1);
        }
    }

    #[test]
    fn completed_or_cancelled_cycle_wait_does_not_leave_adc_rate_wakeups() {
        let counter = Arc::new(Counter(AtomicUsize::new(0)));
        let waker = core::task::Waker::from(counter.clone());
        let notification = CycleNotification::new();
        notification.register(&waker);
        for _ in 0..100 {
            notification.wake();
        }
        assert_eq!(counter.0.load(Ordering::Relaxed), 1);
        notification.register(&waker);
        notification.cancel();
        notification.wake();
        assert_eq!(counter.0.load(Ordering::Relaxed), 1);
        notification.register(&waker);
        notification.wake();
        assert_eq!(counter.0.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn publication_at_registration_boundaries_is_observed_without_lingering_wakes() {
        #[derive(Clone, Copy, Debug)]
        enum Publication {
            BeforeRegister,
            DuringRegister,
            AfterRegister,
            AfterPending,
        }
        for point in [
            Publication::BeforeRegister,
            Publication::DuringRegister,
            Publication::AfterRegister,
            Publication::AfterPending,
        ] {
            let notification = Arc::new(CycleNotification::new());
            let state = Arc::new(CriticalSectionMutex::new(RefCell::new(
                MotorControlState::new(),
            )));
            let counter = Arc::new(Counter(AtomicUsize::new(0)));
            let waker = core::task::Waker::from(counter.clone());
            let previous_seq = critical_section::with(|cs| state.borrow(cs).borrow().last_adc.seq);
            match point {
                Publication::BeforeRegister => publish_cycle(&notification, &state, 1),
                Publication::DuringRegister => {
                    // AtomicWaker retains a cancelled registration. Replacing
                    // it invokes this callback INSIDE register(), before the
                    // new CycleNotification subscription has been armed.
                    let publisher = core::task::Waker::from(Arc::new(PublishOnWake {
                        notification: notification.clone(),
                        state: state.clone(),
                    }));
                    notification.register(&publisher);
                    notification.cancel();
                }
                _ => {}
            }
            notification.register(&waker);
            match point {
                Publication::AfterRegister => publish_cycle(&notification, &state, 1),
                Publication::AfterPending => {
                    assert_eq!(
                        critical_section::with(|cs| state.borrow(cs).borrow().last_adc.seq),
                        previous_seq
                    );
                    std::thread::scope(|scope| {
                        scope.spawn(|| publish_cycle(&notification, &state, 1));
                    });
                }
                _ => {}
            }
            // This is the consumer's register-before-sequence-recheck
            // protocol: a missed notification is harmless only if the
            // recheck observes the complete, matching publication.
            critical_section::with(|cs| {
                let state = state.borrow(cs).borrow();
                assert_ne!(state.last_adc.seq, previous_seq, "{point:?}");
                assert_eq!((state.last_adc.seq, state.last_foc.iq), (1, 1.0));
            });
            let expected_wakes = usize::from(matches!(
                point,
                Publication::AfterRegister | Publication::AfterPending
            ));
            assert_eq!(
                counter.0.load(Ordering::Relaxed),
                expected_wakes,
                "{point:?}"
            );
            // Completion drops the subscription, including when publication
            // happened during register() and left a newly armed flag behind.
            notification.cancel();
            for seq in 2..=100 {
                publish_cycle(&notification, &state, seq);
            }
            assert_eq!(
                counter.0.load(Ordering::Relaxed),
                expected_wakes,
                "{point:?}"
            );

            notification.register(&waker);
            notification.cancel();
            publish_cycle(&notification, &state, 101);
            assert_eq!(
                counter.0.load(Ordering::Relaxed),
                expected_wakes,
                "cancelled {point:?}"
            );
        }
    }

    #[cfg(feature = "storage")]
    #[test]
    fn board_calibration_boot_validation_and_measurement_scale() {
        let mut state = MotorControlState::new();
        let mut config = crate::storage::RuntimeConfig::default();
        state.load_board_calibration(&config);
        assert!(state.board_calibration_valid);
        config.board_calibration = Some(crate::storage::BoardCalibrationConfig { vbus_gain: 1.01 });
        state.load_board_calibration(&config);
        let mutex = CriticalSectionMutex::new(RefCell::new(state.clone()));
        assert_eq!(calibrated_vbus_mv(&mutex, 24_000), 24_240);
        config.board_calibration_invalid = true;
        state.load_board_calibration(&config);
        assert!(!state.board_calibration_valid);
        assert_eq!(
            state.board_calibration,
            crate::storage::BoardCalibrationConfig::IDENTITY
        );
        config.board_calibration_invalid = false;
        config.board_calibration.as_mut().unwrap().vbus_gain = f32::NAN;
        state.load_board_calibration(&config);
        assert!(!state.board_calibration_valid);
    }

    #[test]
    fn stop_does_not_clear_error_latch() {
        // A Stopped command (or the link-loss failsafe, which uses the same
        // path) must not silently un-latch the Error state: only an explicit
        // fault clear may. Otherwise "stop, then run" bypasses every latched
        // fault.
        let mut st = MotorControlState::new();
        st.set_error();
        st.set_stopped();
        assert_eq!(st.motor_state, MotorState::Error, "Stop cleared the latch");
        assert_eq!(st.control_mode, ControlMode::Stopped);

        st.clear_error();
        assert_eq!(st.motor_state, MotorState::Stopped);
    }
}
