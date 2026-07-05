//! Motor state management
//!
//! This module provides centralized state management for motor control.
//! Platforms instantiate the state with their own fault types.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;

use crate::foc::config::BoardConfig;
use crate::foc::controller::{Decoupling, FocOutput};
use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
use crate::foc::phase::{PhaseProvider, PhaseSource};
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::sensors::{AdcSnapshot, HallSnapshot};
use crate::foc::trig::SinCos;
use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::derating::{DeratingConfig, DeratingScales};
use crate::motor::failsafe::FailsafeConfig;
use crate::motor::foc_driver::{CurrentLimits, StepError};
use crate::motor::{ControlMode, FocDriver};
use crate::types::MotorState;

/// Maximum |ω_e| (electrical rad/s) at which a `Brake` (windings-short)
/// command is accepted — above this the short-circuit current is governed by
/// back-EMF against the motor impedance, outside any control loop. Kept a
/// little above the failsafe standstill threshold (20 rad/s default) so a
/// rider standing next to the board can always engage it. Bench-tune.
pub const BRAKE_ENTRY_MAX_E_RAD_S: f32 = 50.0;

// ============================================================================
// Global Communication Channels
// ============================================================================

/// Command for the ISR-owned FocDriver.
///
/// The driver is mutated only inside the FOC ISR; every async-side request
/// to change it goes through [`CMD_CHANNEL`] and is applied in order by
/// [`process_commands`]. One channel (rather than per-purpose signals)
/// keeps the commands sequenced: "set limits, then start" arrives exactly
/// that way.
#[derive(Clone, Copy, Debug)]
pub enum DriverCommand {
    /// Change control mode (start/stop/targets)
    SetMode(ControlMode),
    /// Apply current limits (already clamped to the board ceiling)
    SetCurrentLimits(CurrentLimits),
    /// Apply current-loop PI gains (post-detection tune, config write)
    SetPiGains { kp: f32, ki: f32 },
    /// Apply dq-decoupling/back-EMF feedforward params (post-detection)
    SetDecoupling(Decoupling),
    /// Switch the angle source (hall / observer / HFI / crossovers)
    SetPhaseSource(PhaseSource),
    /// Apply failsafe tuning (deadman timeout + reaction policy + brake params)
    SetFailsafe(FailsafeConfig),
    /// Apply cruise velocity-loop tuning (gains + accel limit)
    SetVelocityConfig(VelocityLoopConfig),
    /// Apply graduated derating ramps (thermal/voltage/speed)
    SetDerating(DeratingConfig),
}

impl DriverCommand {
    /// All numeric payloads are finite (and gains/limits positive where
    /// zero makes no sense). Wire input is arbitrary bits — non-finite
    /// commands are dropped at the boundary instead of feeding NaN into
    /// the control loop.
    pub fn is_sane(&self) -> bool {
        match *self {
            Self::SetMode(mode) => mode.is_finite(),
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
            Self::SetPhaseSource(source) => source.is_finite(),
            Self::SetFailsafe(cfg) => cfg.is_sane(),
            Self::SetVelocityConfig(cfg) => cfg.is_sane(),
            Self::SetDerating(cfg) => cfg.is_sane(),
        }
    }
}

/// Command channel - servers send DriverCommands here, ISR receives them
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, DriverCommand, 8> = Channel::new();

/// Waker for FOC cycle completion — ISR wakes after `update_telemetry()`.
/// Used by calibration/detection to synchronize with individual FOC cycles.
/// The listener reads `last_foc` from the state mutex after waking.
pub static TELEM_WAKER: AtomicWaker = AtomicWaker::new();

/// True while a flash operation is queued or running. Internal-flash erase
/// stalls the whole chip (code executes from the same flash), so it must
/// never overlap an energized motor.
///
/// Closes the TOCTOU gap in the config server's Busy check from both ends:
/// the server arms this flag *before* re-checking the motor state, and
/// [`process_commands`] refuses to start the motor while it is set — so
/// whichever side wins the race, the unsafe overlap is impossible.
pub static FLASH_OP_PENDING: AtomicBool = AtomicBool::new(false);

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

/// RAII guard for [`DETECTION_ACTIVE`]: clears the flag on every exit path.
/// Without it, any future `select`/timeout wrapper around the measurement
/// future would strand the flag on cancellation and leave the link-loss
/// failsafe disabled forever.
pub struct DetectionActiveGuard(());

impl DetectionActiveGuard {
    pub fn arm() -> Self {
        DETECTION_ACTIVE.store(true, Ordering::Relaxed);
        Self(())
    }
}

impl Drop for DetectionActiveGuard {
    fn drop(&mut self) {
        DETECTION_ACTIVE.store(false, Ordering::Relaxed);
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

/// Motor control state
///
/// This struct holds all telemetry-related state that servers need to access.
/// It's updated by the platform's ISR and read by the protocol servers.
#[derive(Clone, Debug)]
pub struct MotorControlState {
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
    /// Live derating scales (mirrored from the driver each cycle so the
    /// slow-telemetry server can report them without touching the driver)
    pub derating: DeratingScales,
}

impl MotorControlState {
    /// Create new state (const for static initialization)
    pub const fn new() -> Self {
        Self {
            motor_state: MotorState::Stopped,
            control_mode: ControlMode::Stopped,
            last_hall: None,
            last_adc: AdcSnapshot::empty(),
            last_foc: FocOutput::empty(),
            link_active: false,
            phase_source: PhaseSource::Hall,
            derating: DeratingScales::IDENTITY,
        }
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
#[macro_export]
macro_rules! define_platform_state {
    ($fault_type:ty) => {
        /// Global motor state
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
    process_commands_inner(state_mutex, foc, fault_registry, &mut saw_set_mode)
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
) -> ControlMode
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: PlatformFault,
{
    // Process all pending commands
    while let Ok(cmd) = CMD_CHANNEL.try_receive() {
        // NaN/inf payloads die here, not in the PI loop.
        if !cmd.is_sane() {
            #[cfg(feature = "defmt")]
            defmt::warn!("Dropping non-finite driver command");
            continue;
        }
        let mode = match cmd {
            DriverCommand::SetMode(mode) => {
                // Fresh setpoint from the host — feed the deadman. Set even if
                // the gates below reject the mode: liveness ≠ acceptance.
                *saw_set_mode = true;
                mode
            }
            DriverCommand::SetCurrentLimits(limits) => {
                foc.set_current_limits(limits);
                continue;
            }
            DriverCommand::SetPiGains { kp, ki } => {
                foc.controller_mut().id_pi.set_gains(kp, ki);
                foc.controller_mut().iq_pi.set_gains(kp, ki);
                continue;
            }
            DriverCommand::SetDecoupling(d) => {
                foc.controller_mut().set_decoupling(Some(d));
                continue;
            }
            DriverCommand::SetPhaseSource(source) => {
                // The provider validates (sensor present, estimators
                // configured). On success mirror the active source into the
                // shared state so the host can read it back via telemetry —
                // an invalid request simply leaves the source unchanged.
                if foc.phase_mut().request_source(source) {
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
        };
        critical_section::with(|cs| {
            let mut state = state_mutex.borrow(cs).borrow_mut();

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
                && (state.motor_state == MotorState::Error || fault_registry.any_stopping())
            {
                return;
            }

            // A queued flash write/erase would stall the chip mid-spin —
            // refuse to start until it finishes (see FLASH_OP_PENDING).
            if mode != ControlMode::Stopped && FLASH_OP_PENDING.load(Ordering::SeqCst) {
                #[cfg(feature = "defmt")]
                defmt::warn!("Motor start rejected: flash operation in flight");
                return;
            }

            // Brake (windings shorted) is only safe to enter near standstill:
            // at speed the short-circuit current is set by back-EMF against
            // the motor impedance (→ λ/L), outside any control loop. Above
            // the gate, substitute the controlled-stop ramp ending in Brake
            // (decel-limited regen to standstill, then the short) — same
            // machinery as the failsafe but user-commanded, so it does not
            // set the re-arm latch. Falls back to rejecting when the current
            // sensor is uncalibrated (can't current-brake).
            if mode == ControlMode::Brake
                && foc.phase().get().velocity.abs() > BRAKE_ENTRY_MAX_E_RAD_S
            {
                if foc.enter_brake_ramp() {
                    #[cfg(feature = "defmt")]
                    defmt::info!("Brake at speed: ramping to standstill first");
                } else {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("Brake rejected: at speed and current sensor uncalibrated");
                }
                return;
            }

            // After a failsafe engagement (deadman or link loss) the host
            // must acknowledge with an explicit safe mode before a running
            // mode is accepted again — "throttle back to neutral". Without
            // this, a reconnecting host replaying its last setpoint (or a
            // wedged app that resumes affirming) would relaunch the board
            // right after the failsafe stopped it. The rejected SetMode
            // still stamped the deadman above (liveness ≠ acceptance).
            if foc.failsafe_latched()
                && !matches!(
                    mode,
                    ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
                )
            {
                #[cfg(feature = "defmt")]
                defmt::warn!("Mode rejected: failsafe latched, acknowledge with Stopped first");
                return;
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
            foc.set_mode(mode);
        });
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
    // motor (up to brake_time_s) — it syncs at the failsafe terminal in
    // `run_foc_cycle`, so e.g. the config server's motor-running gate can't
    // admit a flash stall mid-brake.
    let link_active = critical_section::with(|cs| state_mutex.borrow(cs).borrow().link_active);
    if !link_active
        && !DETECTION_ACTIVE.load(Ordering::Relaxed)
        && !matches!(
            foc.mode(),
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        )
    {
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
pub fn run_foc_cycle<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &FaultRegistry<F>,
    driver: &mut FocDriver<P, C, Ph, S>,
    vbus_v: f32,
    now_ticks: u64,
    board: &BoardConfig,
) -> Option<FocOutput>
where
    P: PhasePwm,
    C: CurrentSensor,
    Ph: PhaseProvider,
    S: SinCos,
    F: PlatformFault,
{
    driver.set_vbus(vbus_v);

    let prev_mode = driver.mode();
    let mut saw_set_mode = false;
    let mode = process_commands_inner(state_mutex, driver, fault_registry, &mut saw_set_mode);

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
    let link_lost = !critical_section::with(|cs| state_mutex.borrow(cs).borrow().link_active);
    let in_safe_mode = matches!(
        mode,
        ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
    );
    // Link-loss is suspended during a device-side detection (see
    // `DETECTION_ACTIVE`); the command-staleness deadman is not, so a hung
    // detection still raises CommTimeout.
    let link_lost_unacked = link_lost && !in_safe_mode && !DETECTION_ACTIVE.load(Ordering::Relaxed);
    if (driver.deadman_expired(now_ticks) || link_lost_unacked)
        && let Some(f) = F::from_category(FaultCategory::CommTimeout)
    {
        fault_registry.set(f);
    }

    // Shared protection: voltage-excursion integrators, temperature
    // thresholds, graduated derating (one core implementation — the
    // per-platform ISR copies are gone). Temperatures come from the
    // previous cycle's ADC snapshot in the shared state — one cycle of
    // staleness is nothing on a thermal time scale.
    let (fet_t, motor_t) = critical_section::with(|cs| {
        let st = state_mutex.borrow(cs).borrow();
        (st.last_adc.fet_temp_c_x10(), st.last_adc.motor_temp_c_x10())
    });
    driver.run_protection(fault_registry, board, vbus_v, fet_t, motor_t);
    critical_section::with(|cs| {
        state_mutex.borrow(cs).borrow_mut().derating = driver.derating();
    });

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
        driver.enter_failsafe();
    }

    let was_failsafe = driver.failsafe_active();
    let result = match driver.step(now_ticks) {
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

    result
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
) {
    // Update state
    critical_section::with(|cs| {
        let mut state = state_mutex.borrow(cs).borrow_mut();
        state.update_adc(adc);
        if let Some(h) = hall {
            state.update_hall(h);
        }
        state.update_foc(foc);
    });

    // Wake calibration/detection task if waiting for FOC cycle
    TELEM_WAKER.wake();
}

#[cfg(test)]
mod tests {
    use super::*;

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
