//! Reusable FOC motor driver
//!
//! High-level driver integrating the FOC controller, current sensing, a
//! phase provider, and PWM. The hot path, control modes, protection,
//! failsafe, and current-offset calibration live in sibling modules; they
//! are one `FocDriver` type split across files.
//!
//! Platform code provides the trait implementations and calls [`FocDriver::step`].

mod actuation;
mod calibration;
mod driver;
mod failsafe;
mod modes;
mod protection;
mod step;
mod watchdog;

#[cfg(test)]
mod tests;

pub use crate::types::ControlMode;
pub use protection::{CurrentLimits, OVERCURRENT_HEADROOM};

use crate::foc::controller::FocController;
use crate::foc::current_offset::CurrentOffsetCalibrator;
use crate::foc::current_offset::CurrentOffsetError;
use crate::foc::current_offset::CurrentOffsetReport;
use crate::foc::phase::PhaseProvider;
use crate::foc::phase_voltage::PhaseVoltageSense;
use crate::foc::position::Position;
use crate::foc::position::PositionLoopConfig;
use crate::foc::position::PositionTracker;
use crate::foc::pwm::PhasePwm;
use crate::foc::pwm::SvpwmModulator;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::LibmSinCos;
use crate::foc::trig::SinCos;
use crate::foc::velocity::VelocityLoop;
use actuation::ActuationModel;
use protection::ProtectionMonitor;
use watchdog::CommandWatchdog;

use crate::motor::failsafe::FailsafeConfig;
use crate::motor::failsafe::FailsafeController;

/// Command-staleness bound for the bench/calibration modes
/// (OpenLoop/DirectVoltage) — see [`FocDriver::deadman_expired`]. Sized for
/// the longest legitimate device-side detection dwell between `SetMode`s
/// (~5 s flux-spin sample collection) with 2× margin; a hung measurement
/// (dead telemetry, no commands) is cut this long in even though
/// `DETECTION_ACTIVE` suspends the link gate.
pub const BENCH_STALENESS_TIMEOUT_US: u64 = 10_000_000;

/// Command-staleness bound while the sensorless startup sequencer owns
/// commutation — see [`FocDriver::deadman_expired`]. The startup is a
/// bounded (~0.5 s) device-local automaton, and its ramp saturates the
/// ISR enough on the g431 bench (87-100% of the 20 kHz budget under a
/// 1 kHz capture) that the thread-mode command pump can starve past the
/// 150 ms drive bound while affirms sit undrained in the transport —
/// 2026-07-06: ~4/5 cold starts tripped CommTimeout at engage with the
/// motor spinning up fine. Bounded relaxation, same spirit as the bench
/// modes' [`BENCH_STALENESS_TIMEOUT_US`]: a real link loss during a
/// cold start is still cut well inside the startup itself, and the
/// tight bound re-arms the moment the observer takes over.
/// 2026-07-07: 400 → 700 ms — the host's command/affirm task is single and
/// strictly ordered, so while a Start's ACK round-trip is in flight (up to
/// ~400+ ms on a loaded host) no affirms leave the machine; the measured
/// engage gap sat exactly at the old bound (407 ms passed, 410 tripped —
/// RampToZero silently zeroed the ramp current and every hold gave up).
pub const STARTUP_STALENESS_TIMEOUT_US: u64 = 700_000;

/// Sensorless restart on trust loss: how long (s) the angle must be
/// continuously untrustworthy — torque already gated to zero by the
/// `angle_trustworthy` backstop — while a nonzero drive command is active,
/// before the driver re-enters the sensorless cold start (deadshort →
/// ramp). Without this, a mid-drive estimator collapse is a DEADLOCK: no
/// torque → no motion → no back-EMF → no trust (bench 2026-07-06
/// staircase: 15 s of iq = 0 with the rotor standing, gate flicker
/// whining, until the maneuver gave up). The deadshort probe makes the
/// restart safe in both worlds: a rotor that is actually still spinning
/// is re-caught and seeded, a stalled one falls through to the soft ramp.
/// Above the observer's revoke debounce (0.2 s) plus transient dips, so a
/// brief validity wobble at speed re-locks instead of restarting.
pub const SENSORLESS_RESTART_AFTER_S: f32 = 0.5;

/// Maximum measured electrical speed accepted by the all-phases-at-50%
/// diagnostic. The operator acknowledgement remains mandatory because a
/// sensorless estimator cannot prove that an unenergised rotor is stationary.
pub const CURRENT_OFFSET_STANDSTILL_MAX_E_RAD_S: f32 = 5.0;

/// Minimum |iq| (A) driven while the sensorless startup sequencer owns
/// commutation (ramp/hold). A mid-cruise restart inherits the cruise
/// command, and the open-loop ramp cannot capture the rotor with it: bench
/// 2026-07-07 (spin-punch-15, 0.3 A cruise) — restart ramps left the
/// capture to luck (|i| = 0.10 at ramp exit, the hold watched a standing
/// rotor at conf 0.95, gave up and recycled; the confirmed handoffs that
/// day all rode 1.5 A ramps). The floor replaces a weak command's
/// magnitude, keeps its sign, and still passes through the align
/// soft-start scale; commands at or above it are untouched. Restores the
/// commanded torque the moment the sequencer hands off.
pub const STARTUP_MIN_DRIVE_A: f32 = 1.5;

/// Why a [`FocDriver::step`] cycle could not run. Typed so `run_foc_cycle`
/// can route each cause differently: an overcurrent trip must latch a
/// host-visible fault (it already cut PWM), a calibration gap must not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepError {
    /// Current sensor offsets not calibrated yet.
    NotCalibrated,
    /// Measured dq current magnitude exceeded
    /// [`CurrentLimits::overcurrent_threshold_a`]; PWM was disabled and the
    /// mode forced to Stopped before returning.
    Overcurrent,
}

/// Default actuation delay (PWM periods), see
/// [`FocDriver::set_actuation_delay`]: currents sampled at the peak of a
/// center-aligned timer that latches new compare values at BOTH ends of
/// the count, with an ISR that writes the duties before the next (valley)
/// update event — the new duties act from half a period after the sample.
/// A board whose ISR can miss the valley update must configure 1.0.
pub const DEFAULT_ACTUATION_DELAY_CYCLES: f32 = 0.5;

/// Upper bound of [`FocDriver::set_actuation_delay`] (PWM periods).
pub const MAX_ACTUATION_DELAY_CYCLES: f32 = 2.0;

/// While the current loop is out of voltage, the cruise loop may ask for
/// this much (A) beyond the q current actually flowing — enough to follow
/// the deliverable current up as the voltage frees, not enough to bank the
/// current limit.
const VOLTAGE_LIMITED_IQ_MARGIN_A: f32 = 1.0;

/// Reusable FOC driver
///
/// # Type Parameters
/// * `P` - PWM output implementing `PhasePwm`
/// * `C` - Current sensor implementing `CurrentSensor`
/// * `Phase` - Phase provider implementing `PhaseProvider`
///
/// # Example (platform code)
/// ```rust,ignore
/// use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
/// use oxifoc_core::foc::pwm::MotorPwmConfig;
///
/// static FOC_DRIVER: Mutex<NoopRawMutex, RefCell<Option<FocDriver<MotorPwm, CurrentSense, PhaseManager<HallSensor>>>>> =
///     Mutex::new(RefCell::new(None));
///
/// // During init - dt comes from PWM config:
/// let motor_pwm = MotorPwm::new(resources, config::PWM_CONFIG);
/// let driver = FocDriver::new(
///     controller,
///     motor_pwm,
///     current_sensor,
///     phase,
///     config::PWM_CONFIG.dt_s(),
/// );
///
/// #[interrupt]
/// fn ADC1_2() {
///     // Run FOC - dt is stored in driver
///     FOC_DRIVER.lock(|cell| {
///         if let Some(driver) = cell.borrow_mut().as_mut() {
///             if let Ok(telem) = driver.step(now_ticks) {
///                 MOTOR_TELEM.set(telem);
///             }
///         }
///     });
/// }
/// ```
pub struct FocDriver<P, C, Phase, S: SinCos = LibmSinCos>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
{
    /// FOC controller
    controller: FocController<SvpwmModulator, S>,
    /// PWM output
    pwm: P,
    /// Current sensor
    current_sensor: C,
    /// Phase provider (manages angle sources)
    phase: Phase,
    /// Current control mode
    mode: ControlMode,
    /// Bus voltage (V)
    vbus: f32,
    /// Control loop period in seconds (1/pwm_freq)
    dt: f32,
    /// Current limiting configuration
    current_limits: CurrentLimits,
    /// Accumulated angle for open-loop velocity mode
    open_loop_angle: f32,
    /// Applied-voltage history and output-vector timing compensation.
    actuation: ActuationModel,
    /// Command freshness and startup/engage grace windows.
    watchdog: CommandWatchdog,
    /// PWM already disabled by the Stopped arm — skip the per-cycle
    /// `pwm.disable()` (measured ~430 cycles/ISR on G431 for a timer that
    /// is already off). Cleared whenever `set_mode` re-enables the bridge;
    /// the Stopped arm still re-asserts the disable periodically as a
    /// backstop.
    pwm_off: bool,
    /// Previous cycle's `phase.wants_short()` — detects the probe-short →
    /// commutation transition so the current-loop PI can restart from zero
    /// (see `step_inner`).
    was_short: bool,
    /// How long (s) the angle has been continuously untrustworthy under an
    /// active nonzero drive command — drives the
    /// [`SENSORLESS_RESTART_AFTER_S`] restart.
    untrusted_time: f32,
    /// Stopped-arm housekeeping counter: paces the periodic PWM-off
    /// re-assert and the estimator-update decimation.
    stopped_housekeeping: u8,
    /// Cached failsafe tuning (timeout + reaction policy). Host-tunable.
    failsafe_cfg: FailsafeConfig,
    /// Self-contained failsafe sequence, armed when the deadman/link-loss
    /// fires, stepped every cycle until it cuts PWM.
    failsafe_ctrl: FailsafeController,
    /// Set on every failsafe engagement; while set, `process_commands`
    /// rejects running modes — the host must acknowledge with an explicit
    /// safe mode (Stopped / Coast / Brake, "throttle back to neutral")
    /// first, so a reconnecting host replaying a stale setpoint can't
    /// relaunch the board. Cleared by `set_mode` applying a safe mode.
    failsafe_latched: bool,
    /// Over-voltage trip (V) for the proactive regen-brake derate (0 = off,
    /// rely on the OV fault backstop). Set from the board config at boot.
    ov_threshold_v: f32,
    /// Cruise velocity loop for `ControlMode::VelocityControl`, and the
    /// inner loop of `PositionControl`: slew-limited ω reference + clamped
    /// PI → iq, routed through the normal current loop. Host-tunable
    /// (motor/load dependent); reset bumpless on mode entry.
    ///
    /// `ControlledStop` does not use this loop. The failsafe slews a brake
    /// current in [`crate::motor::failsafe::FailsafeController`] and feeds
    /// that current back through the current loop, so a second velocity PI
    /// cannot fight the cruise loop or wind up against the brake.
    velocity_loop: VelocityLoop,
    /// The rotor's unwrapped electrical position, followed every cycle in
    /// every mode (the frame of `PositionControl` targets: 0 at boot).
    position: PositionTracker,
    /// Position P law feeding `velocity_loop` in `PositionControl`.
    position_cfg: PositionLoopConfig,
    /// `PositionControl`'s target, split into turns + angle once at mode
    /// entry.
    position_target: Position,
    /// Measured q current of the last current-loop step — the current the
    /// loop could actually deliver while it was out of voltage (see
    /// `step_velocity_control`).
    last_iq_meas: f32,
    /// Parking brake (`ControlMode::Brake`) regulating instead of shorting:
    /// the brake q current being commanded (see `step_park_brake`). `None`
    /// = the plain windings short.
    park_iq: Option<f32>,
    /// Low-passed q-axis modulation (vq·1.5/vbus) of the previous cycles —
    /// the duty factor relating phase current to bus current
    /// (`i_bus ≈ iq·mod_q`, VESC mcpwm_foc.c:3632). Filtered (τ ≈ 2 ms) so
    /// the bus-limit clamp derived from it doesn't chatter on per-cycle
    /// voltage ripple.
    bus_mod_q_filt: f32,
    /// Voltage/temperature protection and graduated derating state.
    protection: ProtectionMonitor,
    /// Phase-voltage converter, `Some` only on boards with phase sensing
    /// (`BoardConfig::phase_sense`). Drives the measured-voltage observer
    /// feed; `None` → the observer always integrates the commanded voltage,
    /// exactly as before this existed.
    phase_voltage: Option<PhaseVoltageSense>,
    /// This cycle's raw phase-terminal ADC counts, pushed in by the ISR
    /// (`set_phase_voltage_raw`) from the ADC snapshot. `None` until a sensing
    /// board provides them.
    vphase_raw: Option<[u16; 3]>,
    /// Current-offset calibration is an ISR-owned maintenance operation, not
    /// a public motor mode. It runs before the normal mode arm in `step()`.
    current_offset: CurrentOffsetCalibrator,
    /// One-shot completion consumed by `run_foc_cycle`, which wakes the async
    /// requester without giving it ownership of PWM sequencing.
    current_offset_outcome: Option<Result<CurrentOffsetReport, CurrentOffsetError>>,
}
