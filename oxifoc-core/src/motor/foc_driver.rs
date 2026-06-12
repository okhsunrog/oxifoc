//! Reusable FOC motor driver
//!
//! This module provides a high-level FOC driver that integrates:
//! - FOC controller (Clarke/Park transforms, PI control, SVPWM)
//! - Current sensing
//! - Phase provider (angle sensing/estimation)
//! - PWM output
//! - Control mode management
//! - Current limiting (target clamping + measured overcurrent protection)
//!
//! Platform code just needs to provide trait implementations and call `step()`.

use crate::foc::clamp_f32;
#[cfg(feature = "runtime")]
use crate::foc::config::BoardConfig;
use crate::foc::controller::{FocController, FocOutput};
use crate::foc::fast_math::sqrtf;
#[cfg(feature = "runtime")]
use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault, VOLTAGE_HYSTERESIS_MV};
use crate::foc::phase::{PhaseInput, PhaseProvider};
use crate::foc::pwm::{PhasePwm, PhaseState, SvpwmModulator};
use crate::foc::sensors::CurrentSensor;
use crate::foc::transforms::{clarke, park};
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::foc::velocity::{VelocityLoop, VelocityLoopConfig};
use crate::motor::derating::{DeratingConfig, DeratingScales};
use crate::motor::failsafe::{
    FailsafeAction, FailsafeConfig, FailsafeController, FailsafePolicy, FailsafeTerminal,
};
use crate::motor::six_step;
#[cfg(feature = "storage")]
use crate::storage::CurrentLimitsConfig;

// Re-export ControlMode from types (single source of truth)
pub use crate::types::ControlMode;

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
    /// Requested control mode is not implemented yet (position control).
    NotImplemented,
}

/// Default commutation phase advance (PWM cycles): ADC samples at the PWM
/// center, the resulting duties latch at the next update event and act
/// (on average) at the middle of that period — one full cycle later. At
/// 20 kHz and 10k eRPM skipping this costs ~3° of angle (cos-loss torque
/// plus d-axis current); VESC compensates the same way (foc_observer_offset).
pub const DEFAULT_PHASE_ADVANCE_CYCLES: f32 = 1.0;

/// Current limiting configuration for the FOC driver.
///
/// Three layers of protection:
/// 1. **Target clamp**: limits what the PI controller is asked to do (prevents
///    absurd commands). Uses circular clamp with d-axis priority.
/// 2. **Bus (supply) current clamp**: VESC-style `i_bus ≈ iq·mod_q` bound on
///    the iq target — phase and bus current differ by the duty factor, so a
///    battery/PSU limit cannot be expressed as a phase limit. Regen side is
///    separate: `bus_regen_max_a = 0` forbids pushing energy into the supply
///    at all (a lab PSU cannot absorb reverse current).
/// 3. **Measured overcurrent**: checks actual dq current magnitude after the
///    FOC step. If it exceeds `overcurrent_threshold`, PWM is disabled and
///    an error is returned. This is the software equivalent of hardware
///    overcurrent protection for boards that lack it.
#[derive(Clone, Copy, Debug)]
pub struct CurrentLimits {
    /// Maximum current target magnitude (A). The PI controller will never
    /// be asked to produce more than this. Set from BoardConfig or
    /// CurrentLimitsConfig. 0 = no limit.
    pub max_current_a: f32,
    /// Hard overcurrent threshold on measured current (A). If actual
    /// sqrt(id² + iq²) exceeds this, PWM is immediately disabled.
    /// Typically set to 1.2-1.5× max_current_a. 0 = no limit.
    pub overcurrent_threshold_a: f32,
    /// Maximum supply (bus) current draw (A). `< 0` = unlimited.
    /// Note the different "off" sentinel from the phase limits: here 0 is a
    /// meaningful value (no draw allowed), so "unlimited" must be negative.
    pub bus_in_max_a: f32,
    /// Maximum regen (charge) current into the supply (A, positive
    /// magnitude). `< 0` = unlimited; **0 = no regen** (lab-PSU safe:
    /// current braking degrades to what winding losses absorb, the
    /// windings-short parking brake is unaffected — it never touches the
    /// bus).
    pub bus_regen_max_a: f32,
}

impl Default for CurrentLimits {
    fn default() -> Self {
        // Conservative bench-safe fallback, NOT "protection off": a platform
        // that forgets set_current_limits() gets a 5 A motor, not an
        // unprotected one. Explicit zeros still mean "no limit" where a
        // test or sim genuinely wants that.
        Self::from_max_current(5.0)
    }
}

/// Required ratio between the overcurrent trip and the soft iq ceiling.
/// The band between them absorbs what a legitimate full-throttle command
/// adds on top of its target: PI overshoot on steps, the HFI carrier
/// ripple (~2 A target), measurement noise. A config that places the Kill
/// line inside that band turns full throttle into a nuisance trip — the
/// config server rejects such writes (`CurrentLimitsConfig::is_coherent`)
/// and `from_config_clamped` clamps whatever arrives by other paths.
pub const OVERCURRENT_HEADROOM: f32 = 1.3;

impl CurrentLimits {
    /// Create current limits from a maximum current value. Sets the
    /// overcurrent threshold [`OVERCURRENT_HEADROOM`] above the max;
    /// bus limits off.
    pub fn from_max_current(max_a: f32) -> Self {
        Self {
            max_current_a: max_a,
            overcurrent_threshold_a: max_a * OVERCURRENT_HEADROOM,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
        }
    }

    /// Build limits from a host-written config, clamped to the board's
    /// hardware ceiling AND the motor's continuous-current rating.
    ///
    /// Layered semantics (the VESC override-matrix idea): the RATING is a
    /// ceiling owned by the motor (detection's thermal solve, stored in
    /// the MotorParams group), the OPERATIONAL config is the session's
    /// choice below it — it can lower limits but never raise them above
    /// either the board hardware or what the motor tolerates. Zero or
    /// negative config values mean "not set" and fall back to the ceiling
    /// itself (VESC applies `l_current_max = i_max` the same way), so a
    /// detected motor gets sane defaults with no extra configuration. The
    /// overcurrent trip ceiling is 1.5× the rating (VESC's
    /// `l_abs_current_max = i_max·1.5`), still capped by the board.
    ///
    /// `hw_max_a` is the board's ABS trip line — the same line the
    /// per-phase OC check in `run_foc_cycle` kills at — NOT an iq
    /// budget: the iq ceiling sits [`OVERCURRENT_HEADROOM`] below it.
    /// (Until 2026-06-12 the iq ceiling WAS the line, so a board-limit
    /// config met the per-phase Kill exactly at full throttle.) The same
    /// headroom is enforced across the config fields: protection wins
    /// over torque, an incoherent pair lowers iq rather than raising the
    /// trip.
    ///
    /// # Arguments
    /// * `cfg` - the host-written limits config
    /// * `hw_max_a` - board hardware phase-current ABS trip (A)
    /// * `rating_a` - motor continuous-current rating (A); `<= 0`/NaN =
    ///   unknown (no rating clamp)
    #[cfg(feature = "storage")]
    pub fn from_config_clamped(cfg: &CurrentLimitsConfig, hw_max_a: f32, rating_a: f32) -> Self {
        let rating_ok = rating_a.is_finite() && rating_a > 0.0;
        let iq_ceiling_board = hw_max_a / OVERCURRENT_HEADROOM;
        let iq_ceiling = if rating_ok {
            iq_ceiling_board.min(rating_a)
        } else {
            iq_ceiling_board
        };
        let trip_ceiling = if rating_ok {
            hw_max_a.min(1.5 * rating_a)
        } else {
            hw_max_a
        };
        // Bus limits have no board ceiling (they protect the supply, not
        // the board); NaN → unlimited (the boundary sanity check rejects
        // NaN commands, this is boot-path defense).
        let bus = |v: f32| if v.is_finite() { v } else { -1.0 };
        let trip = if cfg.max_phase_current_a > 0.0 {
            cfg.max_phase_current_a.min(trip_ceiling)
        } else {
            trip_ceiling
        };
        let iq = if cfg.max_iq_a > 0.0 {
            cfg.max_iq_a.min(iq_ceiling)
        } else {
            iq_ceiling
        };
        Self {
            max_current_a: iq.min(trip / OVERCURRENT_HEADROOM),
            overcurrent_threshold_a: trip,
            bus_in_max_a: bus(cfg.bus_in_max_a),
            bus_regen_max_a: bus(cfg.bus_regen_max_a),
        }
    }

    /// Limits for boot: the stored config (clamped) when present, board
    /// defaults otherwise. `rating_a` as in
    /// [`from_config_clamped`](Self::from_config_clamped) — pass the
    /// stored motor rating (or `0.0` when absent) so a detected motor's
    /// thermal ceiling holds even with no limits group written.
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&CurrentLimitsConfig>, hw_max_a: f32, rating_a: f32) -> Self {
        match cfg {
            Some(c) => Self::from_config_clamped(c, hw_max_a, rating_a),
            // No limits group stored: the ceilings ARE the limits (an
            // all-unset config, NOT CurrentLimitsConfig::default() — that
            // one carries concrete 10 A/40 A values).
            None if rating_a.is_finite() && rating_a > 0.0 => Self::from_config_clamped(
                &CurrentLimitsConfig {
                    max_iq_a: 0.0,
                    max_phase_current_a: 0.0,
                    bus_in_max_a: -1.0,
                    bus_regen_max_a: -1.0,
                },
                hw_max_a,
                rating_a,
            ),
            None => Self::from_max_current(hw_max_a),
        }
    }

    /// Check if current limiting is enabled
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.max_current_a > 0.0
    }

    /// Clamp id/iq targets to the current limit circle.
    ///
    /// Uses d-axis priority: id is clamped first to the full budget,
    /// then iq gets the remaining circular margin. This is correct for
    /// IPM motors where id is used for field weakening.
    #[inline]
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn clamp_targets(&self, id_target: f32, iq_target: f32) -> (f32, f32) {
        if !(self.max_current_a > 0.0) {
            return (id_target, iq_target);
        }
        let limit = self.max_current_a;
        // D-axis priority: clamp id first
        let id = clamp_f32(id_target, -limit, limit);
        // Q-axis gets the remaining circular budget
        let iq_budget_sq = limit * limit - id * id;
        let iq_budget = if iq_budget_sq > 0.0 {
            // .max(0.0) lets the compiler prove -iq_budget <= iq_budget,
            // eliminating clamp's panic branch (sqrtf can't return NaN here,
            // but LLVM can't prove it).
            sqrtf(iq_budget_sq).max(0.0)
        } else {
            0.0
        };
        let iq = clamp_f32(iq_target, -iq_budget, iq_budget);
        (id, iq)
    }

    /// Check if measured current exceeds the overcurrent threshold.
    /// Returns true if overcurrent is detected.
    #[inline]
    pub fn is_overcurrent(&self, id: f32, iq: f32) -> bool {
        if self.overcurrent_threshold_a <= 0.0 {
            return false;
        }
        let mag_sq = id * id + iq * iq;
        let threshold_sq = self.overcurrent_threshold_a * self.overcurrent_threshold_a;
        mag_sq > threshold_sq
    }
}

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
    /// Commanded αβ voltage of the PREVIOUS cycle — what was physically
    /// applied while this cycle's currents were flowing. The estimators
    /// must integrate this one, not the voltage computed this cycle
    /// (which only acts during the next PWM period).
    v_alpha_prev: f32,
    v_beta_prev: f32,
    /// Commutation phase advance in PWM cycles (VESC's observer-offset
    /// idea, applied source-independently): the angle estimate is from the
    /// ADC sample instant, but the voltage acts ~one period later (duty
    /// registers latch at the next update event; ZOH centroid at that
    /// period's middle). Advance the Park angle by vel·dt·k so the vector
    /// lands where the rotor will be. 0 = off.
    phase_advance_cycles: f32,
    /// Tick (in the ISR `now_ticks` domain) of the last fresh setpoint drained
    /// from the command channel. `None` until the first command. The
    /// command-staleness deadman compares `now_ticks - last_cmd_tick` against
    /// `failsafe_cfg.staleness_timeout_us`.
    last_cmd_tick: Option<u64>,
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
    /// Cruise velocity loop for `ControlMode::VelocityControl`: slew-limited
    /// ω reference + clamped PI → iq, routed through the normal current
    /// loop. Host-tunable (motor/load dependent); reset bumpless on mode
    /// entry. The failsafe does NOT use this instance (own gains, planned).
    velocity_loop: VelocityLoop,
    /// Low-passed q-axis modulation (vq·1.5/vbus) of the previous cycles —
    /// the duty factor relating phase current to bus current
    /// (`i_bus ≈ iq·mod_q`, VESC mcpwm_foc.c:3632). Filtered (τ ≈ 2 ms) so
    /// the bus-limit clamp derived from it doesn't chatter on per-cycle
    /// voltage ripple.
    bus_mod_q_filt: f32,
    /// Graduated derating ramps (host-tunable; see `motor::derating`).
    derating_cfg: DeratingConfig,
    /// Live derating scales, recomputed (decimated) by the protection
    /// update in `run_foc_cycle`; applied per-direction on the iq budget
    /// in `step_current_control`.
    derating: DeratingScales,
    /// Decimation counter for the protection update (temps/vbus are slow).
    /// (Protection fields are only read by `run_protection`, which needs
    /// the `runtime` feature for the fault registry.)
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    protection_tick: u16,
    /// Over/under-voltage excursion integrals (V·s) — the voltage faults
    /// trip on sustained excursion, not single samples (VESC's
    /// `wrong_voltage_integrator`): a regen spike or an EMI blip on the
    /// vbus sense must not cost torque mid-ride.
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    ov_integral_vs: f32,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    uv_integral_vs: f32,
}

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Create a new FOC driver
    ///
    /// # Arguments
    /// * `controller` - FOC controller instance
    /// * `pwm` - PWM output
    /// * `current_sensor` - Current sensor (should be calibrated)
    /// * `phase` - Phase provider (manages angle sources)
    /// * `dt` - Control loop period in seconds (from `MotorPwmConfig::dt_s()`)
    pub fn new(
        controller: FocController<SvpwmModulator, S>,
        pwm: P,
        current_sensor: C,
        phase: Phase,
        dt: f32,
    ) -> Self {
        let vbus = controller.vbus();
        Self {
            controller,
            pwm,
            current_sensor,
            phase,
            mode: ControlMode::Stopped,
            vbus,
            dt,
            current_limits: CurrentLimits::default(),
            open_loop_angle: 0.0,
            v_alpha_prev: 0.0,
            v_beta_prev: 0.0,
            phase_advance_cycles: DEFAULT_PHASE_ADVANCE_CYCLES,
            last_cmd_tick: None,
            failsafe_cfg: FailsafeConfig::default(),
            failsafe_ctrl: FailsafeController::new(),
            failsafe_latched: false,
            ov_threshold_v: 0.0,
            velocity_loop: VelocityLoop::new(VelocityLoopConfig::default()),
            bus_mod_q_filt: 0.0,
            derating_cfg: DeratingConfig::default(),
            derating: DeratingScales::default(),
            protection_tick: 0,
            ov_integral_vs: 0.0,
            uv_integral_vs: 0.0,
        }
    }

    /// Apply derating configuration (boot / live config write).
    pub fn set_derating(&mut self, cfg: DeratingConfig) {
        self.derating_cfg = cfg;
    }

    /// Current derating configuration.
    pub fn derating_cfg(&self) -> &DeratingConfig {
        &self.derating_cfg
    }

    /// Live derating scales (1.0/1.0 = no derate).
    pub fn derating(&self) -> DeratingScales {
        self.derating
    }

    /// Store freshly computed derating scales (protection update).
    pub fn set_derating_scales(&mut self, scales: DeratingScales) {
        self.derating = scales;
    }

    /// Shared protection update, called by `run_foc_cycle` every cycle
    /// (moved here from per-platform ISR copies — one implementation for
    /// every board):
    ///
    /// - **Voltage faults via excursion integrals** (V·s) instead of
    ///   single samples (VESC `wrong_voltage_integrator`): a regen spike
    ///   or an EMI blip on the vbus sense must not cost torque mid-ride.
    ///   Trip threshold ≈ VESC-equivalent `5e-5 · max_vbus` V·s (≈3 ms at
    ///   1 V over on a 57 V board); in-range the integral decays with
    ///   τ ≈ 5 ms, so oscillation around the limit still accumulates.
    ///   UnderVoltage keeps its hysteresis auto-clear.
    /// - **Temperature faults**: single-sample against the board
    ///   thresholds — the readings are already slow and filtered.
    /// - **Graduated derating** (decimated /256 ≈ 78 Hz at 20 kHz —
    ///   temps and vbus are slow): recomputes the live scales and
    ///   maintains the `Derating` warning with set/clear hysteresis
    ///   (set < 0.8, clear > 0.95) so the rider learns WHY the board
    ///   feels weak.
    ///
    /// Faults are raised through [`PlatformFault::from_category`]; a
    /// platform returning `None` for a category silently skips it.
    #[cfg(feature = "runtime")]
    pub fn run_protection<F: PlatformFault>(
        &mut self,
        registry: &FaultRegistry<F>,
        board: &BoardConfig,
        vbus_v: f32,
        fet_temp_c_x10: Option<i16>,
        motor_temp_c_x10: Option<i16>,
    ) {
        // --- Voltage excursion integrals (every cycle) ---
        let max_v = board.max_vbus_mv as f32 / 1000.0;
        let min_v = board.min_vbus_mv as f32 / 1000.0;
        let trip_vs = 5e-5 * max_v;
        let decay = 1.0 - (self.dt / 0.005).min(1.0);
        if vbus_v > max_v {
            self.ov_integral_vs += (vbus_v - max_v) * self.dt;
            if self.ov_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::OverVoltage)
                && let Some(f) = F::from_category(FaultCategory::OverVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("OverVoltage FAULT: vbus = {} V (integrated)", vbus_v);
            }
        } else {
            self.ov_integral_vs *= decay;
        }
        if vbus_v < min_v {
            self.uv_integral_vs += (min_v - vbus_v) * self.dt;
            if self.uv_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::UnderVoltage)
                && let Some(f) = F::from_category(FaultCategory::UnderVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("UnderVoltage FAULT: vbus = {} V (integrated)", vbus_v);
            }
        } else {
            self.uv_integral_vs *= decay;
            if vbus_v * 1000.0 > (board.min_vbus_mv + VOLTAGE_HYSTERESIS_MV) as f32
                && registry.has_category(FaultCategory::UnderVoltage)
            {
                registry.clear(FaultCategory::UnderVoltage);
            }
        }

        // --- Temperatures (board thresholds; <= 0 = sensor not wired) ---
        let fet_c = fet_temp_c_x10.map(|t| f32::from(t) / 10.0);
        let motor_c = motor_temp_c_x10.map(|t| f32::from(t) / 10.0);
        let fet_over =
            fet_c.is_some_and(|t| board.max_fet_temp_c > 0.0 && t > board.max_fet_temp_c);
        let motor_over =
            motor_c.is_some_and(|t| board.max_motor_temp_c > 0.0 && t > board.max_motor_temp_c);
        if (fet_over || motor_over)
            && !registry.has_category(FaultCategory::OverTemp)
            && let Some(f) = F::from_category(FaultCategory::OverTemp)
        {
            registry.set(f);
            #[cfg(feature = "defmt")]
            defmt::error!(
                "OverTemp FAULT (fet_over={}, motor_over={})",
                fet_over,
                motor_over
            );
        }

        // --- Graduated derating (decimated) ---
        self.protection_tick = self.protection_tick.wrapping_add(1);
        if self.protection_tick.is_multiple_of(256) {
            let omega = self.phase.get().velocity;
            self.derating = self.derating_cfg.compute(fet_c, motor_c, vbus_v, omega);
            let worst = self.derating.worst();
            if worst < 0.8 {
                if let Some(f) = F::from_category(FaultCategory::Derating) {
                    registry.set(f);
                }
            } else if worst > 0.95 && registry.has_category(FaultCategory::Derating) {
                registry.clear(FaultCategory::Derating);
            }
        }
    }

    /// Override the commutation phase advance (in PWM cycles; 0 = off).
    /// See the field docs — default 1.0 for center-sampled, next-period
    /// applied pipelines.
    pub fn set_phase_advance(&mut self, cycles: f32) {
        if cycles.is_finite() {
            self.phase_advance_cycles = cycles;
        }
    }

    /// Feed the phase provider with this cycle's measurements and the
    /// causally-matching voltage: the one commanded LAST cycle, which was
    /// acting while these currents were measured. Stores the new command
    /// for the next cycle.
    fn update_phase_with_prev_voltage(
        &mut self,
        v_alpha_new: f32,
        v_beta_new: f32,
        i_alpha: f32,
        i_beta: f32,
        dt: f32,
        now_ticks: u64,
    ) {
        self.phase.update(
            &PhaseInput {
                v_alpha: self.v_alpha_prev,
                v_beta: self.v_beta_prev,
                i_alpha,
                i_beta,
                dt,
            },
            now_ticks,
        );
        self.v_alpha_prev = v_alpha_new;
        self.v_beta_prev = v_beta_new;
    }

    /// Get the control loop period (dt) in seconds
    pub fn dt(&self) -> f32 {
        self.dt
    }

    /// Set current limits.
    pub fn set_current_limits(&mut self, limits: CurrentLimits) {
        self.current_limits = limits;
    }

    /// Get current limits.
    pub fn current_limits(&self) -> &CurrentLimits {
        &self.current_limits
    }

    /// Cache the failsafe tuning (deadman timeout + reaction policy + brake
    /// params). Sane values only — garbage is ignored so a bad config can't
    /// disable the deadman or the brake.
    pub fn set_failsafe(&mut self, cfg: FailsafeConfig) {
        if cfg.is_sane() {
            self.failsafe_cfg = cfg;
        }
    }

    /// Active failsafe config.
    pub fn failsafe_config(&self) -> FailsafeConfig {
        self.failsafe_cfg
    }

    /// Tune the cruise velocity loop (gains + accel limit). Sane values
    /// only — garbage is ignored. Takes effect immediately; the loop state
    /// (ramp/integrator) is preserved, so retuning mid-ride is bumpy only
    /// to the extent the gains differ.
    pub fn set_velocity_config(&mut self, cfg: VelocityLoopConfig) {
        self.velocity_loop.set_config(cfg);
    }

    /// Active velocity-loop tuning.
    pub fn velocity_config(&self) -> VelocityLoopConfig {
        self.velocity_loop.config()
    }

    /// Set the over-voltage trip (V) used for the proactive regen-brake
    /// derate (0 = off → rely on the OV fault backstop). From the board
    /// config at boot.
    pub fn set_ov_threshold(&mut self, volts: f32) {
        self.ov_threshold_v = if volts.is_finite() && volts > 0.0 {
            volts
        } else {
            0.0
        };
    }

    /// Stamp a fresh setpoint arrival (called from the ISR when a `SetMode`
    /// is drained). The deadman's "positive affirmation".
    pub fn note_command_tick(&mut self, now_ticks: u64) {
        self.last_cmd_tick = Some(now_ticks);
    }

    /// Whether the command link has gone stale while the motor is running —
    /// the deadman trigger.
    ///
    /// Only the *drive* modes are covered (CurrentControl, plus the velocity/
    /// position loops once they exist) — those are what a vehicle rides on,
    /// and the host re-affirms them every 50 ms. Exempt:
    /// - Stopped/Coast/Brake — safe standing states: nothing to fail safe
    ///   toward, and a parking brake must persist through link loss.
    /// - OpenLoop/DirectVoltage/SixStep — bench/calibration modes: on-device
    ///   detection sets one and then dwells up to ~1 s between `SetMode`s
    ///   (e.g. the R-measurement settle), which a 150 ms deadman would cut
    ///   mid-measurement. The Layer-1 link gate (1 s liveness) still covers
    ///   them against a dead host.
    ///
    /// Also false while the failsafe already runs.
    pub fn deadman_expired(&self, now_ticks: u64) -> bool {
        if matches!(
            self.mode,
            ControlMode::Stopped
                | ControlMode::Coast
                | ControlMode::Brake
                | ControlMode::OpenLoop { .. }
                | ControlMode::DirectVoltage { .. }
                | ControlMode::SixStep { .. }
        ) {
            return false;
        }
        if self.failsafe_ctrl.is_active() {
            return false;
        }
        match self.last_cmd_tick {
            // Running implies a SetMode was drained this-or-an-earlier cycle
            // (which stamps), so this arm is effectively unreachable; treat a
            // missing stamp as not-stale (defensive).
            None => false,
            Some(t) => now_ticks.wrapping_sub(t) > self.failsafe_cfg.staleness_timeout_us,
        }
    }

    /// Arm the failsafe sequence (deadman or link-loss). Idempotent. Falls
    /// back to Coast when the current sensor isn't calibrated — the brake
    /// needs the current loop, which refuses to run uncalibrated.
    ///
    /// Also latches `failsafe_latched`: after any failsafe engagement the
    /// host must explicitly acknowledge with a safe mode (Stopped / Coast /
    /// Brake) before a running mode is accepted again — see the gate in
    /// `process_commands`. Defense in depth against a reconnecting host
    /// replaying a stale throttle.
    pub fn enter_failsafe(&mut self) {
        self.failsafe_latched = true;
        self.arm_failsafe_with(self.failsafe_cfg.policy, self.failsafe_cfg.terminal);
    }

    /// User-commanded ramp-into-parking-brake: a `Brake` command above the
    /// standstill gate is substituted with the ControlledStop sequence ending
    /// in `ControlMode::Brake`, instead of being rejected. Same machinery as
    /// the failsafe but **not a failsafe event** — it does not set the re-arm
    /// latch (the user asked for this; no "back to neutral" owed). Returns
    /// false when the current sensor is uncalibrated (can't current-brake;
    /// the caller keeps rejecting).
    pub fn enter_brake_ramp(&mut self) -> bool {
        if !self.current_sensor.is_calibrated() {
            return false;
        }
        if self.failsafe_ctrl.is_active() {
            // A stop is already in progress (failsafe brake) — `arm` would
            // be a no-op and silently drop the user's parking-brake intent.
            // Adopt the terminal; the running sequence finishes into Brake.
            self.failsafe_ctrl.set_terminal(FailsafeTerminal::ParkBrake);
            return true;
        }
        self.arm_failsafe_with(FailsafePolicy::ControlledStop, FailsafeTerminal::ParkBrake);
        true
    }

    /// Common arm body for the failsafe and the user brake ramp. Idempotent
    /// while a sequence is active. Falls back to Coast when the current
    /// sensor isn't calibrated — the brake needs the current loop, which
    /// refuses to run uncalibrated.
    fn arm_failsafe_with(&mut self, policy: FailsafePolicy, terminal: FailsafeTerminal) {
        if self.failsafe_ctrl.is_active() {
            return;
        }
        let policy = if self.current_sensor.is_calibrated() {
            policy
        } else {
            FailsafePolicy::Coast
        };
        // The failsafe drives through the normal current-control path, which
        // assumes all three phases are PWM-able — but six-step commutation
        // floats one phase per sector. Restore them (mirrors the
        // SixStep-exit housekeeping in `set_mode`, which the failsafe
        // bypasses).
        if matches!(self.mode, ControlMode::SixStep { .. }) {
            self.pwm
                .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
        }
        // Bumpless: seed the ramp from the q-current currently commanded
        // (OpenLoop applies its `current` as the q-target; VelocityControl's
        // last PI output is what the current loop was just driven with).
        let current_iq = match self.mode {
            ControlMode::CurrentControl { iq_target, .. } => iq_target,
            ControlMode::OpenLoop { current, .. } => current,
            ControlMode::VelocityControl { .. } => self.velocity_loop.last_iq(),
            _ => 0.0,
        };
        // Bound the seed: mode payloads are finite-checked at the command
        // boundary but not clamped, and the RampDown duration scales with
        // the seed (slew = brake_current_a/ramp_s) — an absurd value must
        // not stretch the failsafe ramp toward forever. The physically
        // delivered current was never above the limit anyway, so clamping
        // keeps the ramp bumpless. NaN (nothing upstream should produce
        // one, but the failsafe must not hinge on that) seeds from zero.
        let seed_bound = if self.current_limits.max_current_a > 0.0 {
            self.current_limits.max_current_a
        } else {
            10.0 * self.failsafe_cfg.brake_current_a
        };
        let current_iq = if current_iq.is_finite() {
            clamp_f32(current_iq, -seed_bound, seed_bound)
        } else {
            0.0
        };
        self.failsafe_ctrl.arm(current_iq, policy, terminal);
    }

    /// Clear the failsafe sequence (a fresh command re-armed control, or a
    /// fault took over). Does NOT clear the re-arm latch — only an explicit
    /// safe-mode command does (see `set_mode`).
    pub fn failsafe_reset(&mut self) {
        self.failsafe_ctrl.reset();
    }

    /// Whether the failsafe is currently carrying the motor.
    pub fn failsafe_active(&self) -> bool {
        self.failsafe_ctrl.is_active()
    }

    /// Whether a failsafe engagement is awaiting an explicit safe-mode
    /// acknowledgement (Stopped / Coast / Brake) before running modes are
    /// accepted again. Checked by the `process_commands` gate.
    pub fn failsafe_latched(&self) -> bool {
        self.failsafe_latched
    }

    /// Set control mode
    ///
    /// When leaving SixStep mode, re-enables all PWM channels that may
    /// have been disabled (floated) during six-step commutation.
    pub fn set_mode(&mut self, mode: ControlMode) {
        // Any explicit mode set is an authoritative override: a fresh host
        // command (re-arm after link loss) or a fault-stop both cancel an
        // in-progress failsafe brake. (The failsafe's own terminal Stop sets
        // `self.mode` directly, not through here, so it doesn't self-cancel.)
        self.failsafe_ctrl.reset();

        // A safe-mode command is the explicit "throttle back to neutral"
        // acknowledgement that releases the post-failsafe re-arm latch.
        if matches!(
            mode,
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        ) {
            self.failsafe_latched = false;
        }

        let was_stopped = matches!(self.mode, ControlMode::Stopped);
        let will_be_active = !matches!(mode, ControlMode::Stopped);

        if matches!(self.mode, ControlMode::SixStep { .. })
            && !matches!(mode, ControlMode::SixStep { .. })
        {
            // Re-enable all phases so set_duties() in FOC modes works
            self.pwm
                .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
        }

        // Re-enable PWM outputs when leaving Stopped mode
        if was_stopped && will_be_active {
            self.pwm.enable();
        }

        // Entering velocity mode from another mode: re-arm the loop bumpless
        // — reference ramp seeded at the measured velocity, integrator
        // cleared. A retarget *within* velocity mode keeps the loop state
        // (the ramp carries the reference to the new target).
        if matches!(mode, ControlMode::VelocityControl { .. })
            && !matches!(self.mode, ControlMode::VelocityControl { .. })
        {
            let omega = self.phase.get().velocity;
            self.velocity_loop.reset(omega);
        }

        // Apply PI gains override if provided (detection uses this)
        if let ControlMode::OpenLoop {
            pi_gains: Some((kp, ki)),
            ..
        } = mode
        {
            self.controller.id_pi.set_gains(kp, ki);
            self.controller.iq_pi.set_gains(kp, ki);
            self.controller.reset();
        }

        self.mode = mode;
    }

    /// Get current control mode
    pub fn mode(&self) -> ControlMode {
        self.mode
    }

    /// Update bus voltage
    pub fn set_vbus(&mut self, vbus: f32) {
        self.controller.set_vbus(vbus);
        self.vbus = vbus;
    }

    /// Get bus voltage
    pub fn vbus(&self) -> f32 {
        self.vbus
    }

    /// Execute one FOC control step
    ///
    /// Call this from your ADC ISR synchronized with PWM.
    /// Uses the stored dt (set via `with_dt()` or defaulting to 20kHz).
    ///
    /// # Arguments
    /// * `now_ticks` - Monotonic ticks for phase sampling (sensor-defined timebase)
    ///
    /// # Returns
    /// * `Ok(FocOutput)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready or overcurrent detected
    pub fn step(&mut self, now_ticks: u64) -> Result<FocOutput, StepError> {
        let dt = self.dt;
        // Failsafe overrides the commanded mode while it runs (ramp-down /
        // regen-brake / coast), then cuts PWM and clears itself.
        if self.failsafe_ctrl.is_active() {
            return self.step_failsafe(dt, now_ticks);
        }
        match self.mode {
            ControlMode::Stopped => {
                // Safe-off: `disable()` is the platform's emergency-stop
                // (all channels off / high-Z on the current boards).
                self.pwm.disable();
                // The previous cycle's command WAS applied before this stop
                // took effect — let the estimators integrate it, then decay
                // to zero volts.
                self.update_phase_with_prev_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            ControlMode::VelocityControl { target_vel } => {
                self.step_velocity_control(target_vel, dt, now_ticks)
            }
            ControlMode::PositionControl { .. } => {
                // TODO: Implement position PI controller
                Err(StepError::NotImplemented)
            }
            ControlMode::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                ..
            } => self.step_open_loop(angle_rad, current, velocity_rad_s, dt, now_ticks),
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                self.step_direct_voltage(vd, vq, angle_rad, dt, now_ticks)
            }
            ControlMode::Coast => {
                // High-Z: all FETs off, motor spins freely.
                self.pwm.set_phase_states([
                    PhaseState::Float,
                    PhaseState::Float,
                    PhaseState::Float,
                ]);
                self.controller.reset();
                self.update_phase_with_prev_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
            ControlMode::SixStep { duty } => self.step_six_step(duty, dt, now_ticks),
            ControlMode::Brake => {
                // Parking brake: all low-side FETs on — windings shorted to
                // ground. Speed-proportional drag, energy dissipates in the
                // motor; zero draw at standstill. Entry is speed-gated at the
                // command boundary (see `process_commands`).
                self.pwm
                    .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
                self.controller.reset();
                self.current_sensor.update_duties([0; 3]);

                // The short-circuit current is real and measurable (the low
                // sides conduct continuously, so low-side shunts see the
                // phase currents and the ADC keeps triggering): read it, so
                // the brake is not a protection blind spot — a parked board
                // shoved fast (downhill runaway) must still trip — and so
                // telemetry/estimators see the truth instead of zeros.
                let (ia, ib, ic) = if self.current_sensor.is_calibrated() {
                    self.current_sensor.read_currents()
                } else {
                    (0.0, 0.0, 0.0)
                };
                let (i_alpha, i_beta) = clarke(ia, ib);
                // αβ magnitude equals the dq magnitude (Park preserves it);
                // no current loop runs here, so this check is the only
                // software protection the mode has. Trip → high-Z: a coast
                // is safer than an uncontrolled short at whatever speed
                // produced this much current.
                if self.current_limits.is_overcurrent(i_alpha, i_beta) {
                    self.pwm.disable();
                    self.mode = ControlMode::Stopped;
                    return Err(StepError::Overcurrent);
                }

                // Terminal voltage is zero while shorted; the estimators
                // integrate that with the real circulating currents.
                self.update_phase_with_prev_voltage(0.0, 0.0, i_alpha, i_beta, dt, now_ticks);
                Ok(FocOutput {
                    ia,
                    ib,
                    ic,
                    i_alpha,
                    i_beta,
                    angle_rad: self.phase.get().angle,
                    ..Default::default()
                })
            }
        }
    }

    /// Run one cycle of the active failsafe sequence (see
    /// [`crate::motor::failsafe`]). `Drive` actions are routed back through
    /// the normal current-control path so the current-limit clamp and the
    /// measured-overcurrent trip still apply — the brake never bypasses
    /// protection. The terminal `Stop` cuts PWM and clears the controller.
    fn step_failsafe(&mut self, dt: f32, now_ticks: u64) -> Result<FocOutput, StepError> {
        let omega_e = self.phase.get().velocity;
        let angle_trustworthy = self.phase.angle_trustworthy();
        let action = self.failsafe_ctrl.step(
            omega_e,
            dt,
            &self.failsafe_cfg,
            self.current_limits.max_current_a,
            self.vbus,
            self.ov_threshold_v,
            angle_trustworthy,
        );
        match action {
            FailsafeAction::Drive {
                id_target,
                iq_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            FailsafeAction::Stop => {
                self.pwm.disable();
                self.controller.reset();
                self.mode = ControlMode::Stopped;
                self.failsafe_ctrl.reset();
                self.update_phase_with_prev_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
            // Clean stop with the parking-brake terminal: hand over to
            // ControlMode::Brake (its step arm re-asserts the low-side short
            // every cycle from here on). Only ever emitted at standstill.
            FailsafeAction::EngageBrake => {
                self.pwm
                    .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
                self.controller.reset();
                self.mode = ControlMode::Brake;
                self.failsafe_ctrl.reset();
                self.update_phase_with_prev_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
        }
    }

    /// Execute one velocity-control cycle: the cruise loop turns the ω
    /// target into an iq command (slew-limited reference + clamped PI, see
    /// [`crate::foc::velocity`]), routed through the normal current loop so
    /// the current-limit clamp and overcurrent trip still apply.
    fn step_velocity_control(
        &mut self,
        target_vel: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Velocity needs a usable estimate; a source that can't track (e.g.
        // a back-EMF observer below its speed floor) makes the loop
        // integrate garbage. Degrade to zero torque while staying in the
        // mode instead of hard-stopping: a sensorless cruise that dips
        // below the floor coasts, and resumes by itself once motion (push,
        // downhill) brings the observer back into lock — an Err here would
        // kick the driver to Stopped and require a fresh host command.
        // Hall/encoder sources are always trustworthy, so a sensored ride
        // never takes this branch.
        if !self.phase.angle_trustworthy() {
            // Keep the loop bumpless for re-entry: reference parked at the
            // (unreliable, but best-known) measured velocity, integrator
            // cleared — no stale torque step when lock returns.
            self.velocity_loop.reset(self.phase.get().velocity);
            return self.step_current_control(0.0, 0.0, dt, now_ticks);
        }
        let omega = self.phase.get().velocity;
        // The derating speed ceiling also caps the cruise reference: the
        // current-side rolloff alone would leave the PI winding up against
        // an unreachable target.
        let max_speed = self.derating_cfg.max_speed_erad_s;
        let target_vel = if max_speed > 0.0 {
            clamp_f32(target_vel, -max_speed, max_speed)
        } else {
            target_vel
        };
        let iq_target =
            self.velocity_loop
                .step(target_vel, omega, self.current_limits.max_current_a, dt);
        self.step_current_control(iq_target, 0.0, dt, now_ticks)
    }

    /// Execute current control step
    fn step_current_control(
        &mut self,
        iq_target: f32,
        id_target: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err(StepError::NotCalibrated);
        }

        // Layer 1: Clamp current targets (prevents absurd commands)
        let (id_target, iq_target) = self.current_limits.clamp_targets(id_target, iq_target);

        // An untrustworthy angle may be π-flipped (pure HFI before its
        // polarity probe resolves) or frozen (back-EMF observer below its
        // speed floor) — iq there is torque in an unknown direction. Zero it;
        // id is polarity-symmetric, and commutation keeps following the
        // estimate so the HFI carrier/probe stays frame-aligned and the gate
        // self-clears once the source locks. Velocity/failsafe paths handle
        // this themselves; this is the backstop for direct current commands.
        let iq_target = if self.phase.angle_trustworthy() {
            iq_target
        } else {
            0.0
        };

        // Graduated derating (motor/derating.rs): scale the iq budget by
        // direction — iq·ω ≥ 0 is motoring (standstill counts as motoring:
        // pulling away is acceleration), opposing is braking. Drive derates
        // for heat/sag/speed; brake only for heat/regen-OV — and never for
        // speed. Every torque path funnels through here, so the failsafe
        // brake inherits the regen-OV rolloff too.
        let iq_target = if self.current_limits.max_current_a > 0.0 {
            let omega = self.phase.get().velocity;
            let scale = if iq_target * omega >= 0.0 {
                self.derating.drive
            } else {
                self.derating.brake
            };
            if scale < 1.0 {
                let lim = self.current_limits.max_current_a * scale;
                clamp_f32(iq_target, -lim, lim)
            } else {
                iq_target
            }
        } else {
            iq_target
        };

        // Bus (supply) current limits: every regen-capable path (host
        // current commands, the velocity loop, the failsafe brake) funnels
        // through here, so they all inherit the clamp.
        let iq_target = self.clamp_iq_for_bus(iq_target);

        // Get phase from provider (uses previous update's estimate). The
        // estimate is sample-time truth and is used as-is for the current
        // Park — the pipeline delay is compensated on the ACTUATION side
        // only (output-vector rotation, see set_actuation_advance).
        // Advancing this angle instead, as the code originally did, also
        // advanced the measurement frame: the PI then regulated the
        // current vector `ωe·dt·cycles` off the true q axis
        // (id_true = −iq·sin(δ) — ~29% of iq parasitic d-current for a
        // Flipsky-class motor at full speed). Found by the sim's
        // actuation-delay plant upgrade.
        let phase_out = self.phase.get();
        let angle_rad = phase_out.angle;
        self.controller
            .set_actuation_advance(phase_out.velocity * dt * self.phase_advance_cycles);

        // HFI carrier for this cycle (zero for non-HFI sources). Must be
        // read between get() and update(): the estimator demodulates the
        // currents fed to update() against this exact carrier sample.
        let (vd_inject, vq_inject) = self.phase.injection();

        // Read currents and run FOC controller. The estimated electrical
        // velocity drives the dq-decoupling feedforward (no-op when no
        // motor params are configured).
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        let out = self.controller.step_with_injection(
            currents,
            angle_rad,
            phase_out.velocity,
            id_target,
            iq_target,
            vd_inject,
            vq_inject,
            max_duty,
            dt,
        );

        // Layer 2: Check measured current against hard overcurrent limit
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        // Track the q-axis modulation for the bus-limit clamp (next cycle).
        self.update_bus_mod_q(out.vq, dt);

        // Set PWM duties and feed to current sensor for next-cycle reconstruction
        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Update phase provider for next step (feeds observer if present).
        // The observer gets the PREVIOUS command — the voltage that was
        // actually acting while these currents were measured.
        self.update_phase_with_prev_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );

        Ok(out)
    }

    /// Low-pass the q-axis modulation (`mod_q = 1.5·vq/vbus`, the duty
    /// factor in `i_bus ≈ iq·mod_q`). τ ≈ 2 ms — slow enough to ignore
    /// per-cycle voltage ripple, fast enough to track real speed changes.
    fn update_bus_mod_q(&mut self, vq: f32, dt: f32) {
        const TAU_S: f32 = 0.002;
        let mod_q = 1.5 * vq / self.vbus.max(0.5);
        let alpha = (dt / TAU_S).min(1.0);
        self.bus_mod_q_filt += alpha * (mod_q - self.bus_mod_q_filt);
    }

    /// Clamp the iq target so the implied bus current stays inside
    /// `[-bus_regen_max_a, bus_in_max_a]` (VESC mcpwm_foc.c:3629-3637).
    ///
    /// `i_bus ≈ iq·mod_q` (electrical power over vbus, q-axis only — the
    /// d-axis share is losses/HFI carrier, small and consumption-side).
    /// With `bus_regen_max_a = 0` no energy is ever pushed into the supply:
    /// current braking degrades to ~zero torque in the regen zone (a lab
    /// PSU cannot absorb reverse current), and a ControlledStop failsafe
    /// then exits via its no-progress watchdog to a coast. The
    /// windings-short parking brake is unaffected — it never sees the bus.
    fn clamp_iq_for_bus(&self, iq_target: f32) -> f32 {
        let in_max = self.current_limits.bus_in_max_a;
        let regen_max = self.current_limits.bus_regen_max_a;
        if in_max < 0.0 && regen_max < 0.0 {
            return iq_target; // both unlimited (the default)
        }
        let mod_q = self.bus_mod_q_filt;
        // Near-zero modulation: bus current is negligible whatever iq is,
        // and dividing by it would explode. Same dead-band as VESC.
        if mod_q.abs() < 1e-3 {
            return iq_target;
        }
        let hi_bus = if in_max >= 0.0 { in_max } else { f32::INFINITY };
        let lo_bus = if regen_max >= 0.0 {
            -regen_max
        } else {
            f32::NEG_INFINITY
        };
        let (lo_iq, hi_iq) = if mod_q > 0.0 {
            (lo_bus / mod_q, hi_bus / mod_q)
        } else {
            (hi_bus / mod_q, lo_bus / mod_q)
        };
        clamp_f32(iq_target, lo_iq, hi_iq)
    }

    /// Execute open-loop control step (for calibration)
    ///
    /// Uses commanded angle instead of sensor feedback.
    /// Current feedback is still used to regulate the applied current.
    ///
    /// When `velocity_rad_s == 0`: locks rotor at `angle_rad`.
    /// When `velocity_rad_s != 0`: advances angle at the given velocity,
    /// enabling sensorless open-loop spinning.
    fn step_open_loop(
        &mut self,
        angle_rad: f32,
        current: f32,
        velocity_rad_s: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err(StepError::NotCalibrated);
        }

        // Advance angle if velocity is set, otherwise use commanded angle
        let angle = if velocity_rad_s != 0.0 {
            self.open_loop_angle += velocity_rad_s * dt;
            self.open_loop_angle %= core::f32::consts::TAU;
            if self.open_loop_angle < 0.0 {
                self.open_loop_angle += core::f32::consts::TAU;
            }
            self.open_loop_angle
        } else {
            self.open_loop_angle = angle_rad;
            angle_rad
        };

        // Clamp open-loop current to the target limit
        let current = if self.current_limits.max_current_a > 0.0 {
            clamp_f32(
                current,
                -self.current_limits.max_current_a,
                self.current_limits.max_current_a,
            )
        } else {
            current
        };

        // Read currents and run FOC controller with commanded angle.
        // When stationary (velocity=0): current on d-axis to lock rotor (resistance measurement).
        // When spinning (velocity≠0): current on q-axis to produce torque.
        let (id_target, iq_target) = if velocity_rad_s == 0.0 {
            (current, 0.0)
        } else {
            (0.0, current)
        };
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        // Same pipeline delay as closed loop; the commanded velocity is the
        // exact frame rate (also clears any stale advance from a previous
        // closed-loop cycle when velocity is 0).
        self.controller
            .set_actuation_advance(velocity_rad_s * dt * self.phase_advance_cycles);
        let out = self
            .controller
            .step(currents, angle, id_target, iq_target, max_duty, dt);

        // Check measured overcurrent
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        // Set PWM duties and feed to current sensor for next-cycle reconstruction
        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Update phase provider (for sensor tracking, even in open-loop)
        self.update_phase_with_prev_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );

        Ok(out)
    }

    /// Execute direct voltage step — no PI control.
    ///
    /// Applies the given dq voltages via `FocController::apply_dq`, reads
    /// currents for telemetry, and feeds the phase observer. Used for
    /// measurement modes (HFI inductance) and direct voltage control.
    fn step_direct_voltage(
        &mut self,
        vd: f32,
        vq: f32,
        angle_rad: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let max_duty = self.pwm.max_duty();
        // Latest measured currents feed the dead-time compensation signs;
        // without a PI loop the distortion is otherwise uncompensated and
        // eats the commanded voltage (see `apply_dq` docs). Uncalibrated
        // sensor → zeros → compensation self-cancels.
        let currents = if self.current_sensor.is_calibrated() {
            Some(self.current_sensor.read_currents())
        } else {
            None
        };
        let (i_alpha_m, i_beta_m) = match currents {
            Some(c) => clarke(c.0, c.1),
            None => (0.0, 0.0),
        };
        let mut out = self
            .controller
            .apply_dq(vd, vq, angle_rad, i_alpha_m, i_beta_m, max_duty);

        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Read currents for telemetry and phase observer
        if let Some(currents) = currents {
            out.ia = currents.0;
            out.ib = currents.1;
            out.ic = currents.2;
            let (i_alpha, i_beta) = (i_alpha_m, i_beta_m);
            out.i_alpha = i_alpha;
            out.i_beta = i_beta;
            let (sin_a, cos_a) = S::sin_cos(angle_rad);
            let (id, iq) = park(i_alpha, i_beta, sin_a, cos_a);
            out.id = id;
            out.iq = iq;

            // No PI loop reins the current in here — the commanded voltage
            // is applied verbatim — so the measured check is the only
            // software protection this mode has.
            if self.current_limits.is_overcurrent(out.id, out.iq) {
                self.pwm.disable();
                self.controller.reset();
                self.mode = ControlMode::Stopped;
                return Err(StepError::Overcurrent);
            }
        }

        self.update_phase_with_prev_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );

        Ok(out)
    }

    /// Execute six-step (trapezoidal) commutation step
    ///
    /// Pure voltage-mode drive: no current loop, no Clarke/Park transforms.
    /// Derives commutation sector from the PhaseProvider angle and applies
    /// the appropriate phase states via `set_phase_states()`.
    ///
    /// Does NOT require current sensor calibration, making it suitable
    /// for initial board bringup.
    fn step_six_step(
        &mut self,
        duty: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Get current electrical angle from phase provider
        let phase_out = self.phase.get();
        let sector = six_step::angle_to_sector(phase_out.angle);

        // Duty sign determines direction
        let forward = duty >= 0.0;
        let duty_abs = clamp_f32(duty.abs(), 0.0, 1.0);
        let raw_duty = (duty_abs * f32::from(self.pwm.max_duty())) as u16;

        // Generate and apply phase states
        let states = six_step::commutate(sector, raw_duty, forward);
        self.pwm.set_phase_states(states);
        // Feed duties to current sensor for reconstruction (Float/Low → 0)
        let duties = states.map(|s| match s {
            PhaseState::Pwm(d) => d,
            PhaseState::Low | PhaseState::Float => 0,
        });
        self.current_sensor.update_duties(duties);

        // Update phase provider (keep sensor tracking active; six-step has
        // no αβ voltage command for the observer — zeros, as before)
        self.update_phase_with_prev_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);

        // Read currents opportunistically (if sensor is calibrated)
        let (ia, ib, ic) = if self.current_sensor.is_calibrated() {
            self.current_sensor.read_currents()
        } else {
            (0.0, 0.0, 0.0)
        };

        // Six-step has no current loop at all; check the measured magnitude
        // in αβ (same magnitude as dq — Park preserves it).
        let (i_alpha, i_beta) = clarke(ia, ib);
        if self.current_limits.is_overcurrent(i_alpha, i_beta) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        Ok(FocOutput {
            ia,
            ib,
            ic,
            angle_rad: phase_out.angle,
            duties,
            ..Default::default()
        })
    }

    /// Get mutable reference to current sensor (for calibration)
    pub fn current_sensor_mut(&mut self) -> &mut C {
        &mut self.current_sensor
    }

    /// Get reference to current sensor
    pub fn current_sensor(&self) -> &C {
        &self.current_sensor
    }

    /// Get mutable reference to phase provider
    pub fn phase_mut(&mut self) -> &mut Phase {
        &mut self.phase
    }

    /// Get reference to phase provider
    pub fn phase(&self) -> &Phase {
        &self.phase
    }

    /// Get mutable reference to FOC controller (for tuning)
    pub fn controller_mut(&mut self) -> &mut FocController<SvpwmModulator, S> {
        &mut self.controller
    }

    /// Get reference to FOC controller
    pub fn controller(&self) -> &FocController<SvpwmModulator, S> {
        &self.controller
    }

    /// Get reference to the PWM output (tests / diagnostics).
    pub fn pwm(&self) -> &P {
        &self.pwm
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::phase::PhaseManager;
    #[cfg(feature = "virtual-motor")]
    use crate::foc::{angle_difference, wrap_angle};
    #[cfg(feature = "runtime")]
    use crate::state::CMD_CHANNEL;
    #[cfg(feature = "virtual-motor")]
    use crate::virtual_motor::VirtualMotorOutput;

    struct MockPwm {
        duties: [u16; 3],
    }

    impl PhasePwm for MockPwm {
        fn max_duty(&self) -> u16 {
            1000
        }
        fn set_duties(&mut self, duties: [u16; 3]) {
            self.duties = duties;
        }
    }

    struct MockCurrentSensor {
        currents: (f32, f32, f32),
    }

    impl CurrentSensor for MockCurrentSensor {
        fn read_currents(&self) -> (f32, f32, f32) {
            self.currents
        }
        fn read_raw(&self) -> (u16, u16, u16) {
            (0, 0, 0)
        }
        fn is_calibrated(&self) -> bool {
            true
        }
        fn get_offsets(&self) -> (f32, f32, f32) {
            (0.0, 0.0, 0.0)
        }
    }

    /// CMD_CHANNEL and FLASH_OP_PENDING are process-wide globals — tests
    /// that touch them must not run concurrently. Also drains any stale
    /// commands a previous test left behind.
    #[cfg(feature = "runtime")]
    fn cmd_channel_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let guard = LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        while CMD_CHANNEL.try_receive().is_ok() {}
        guard
    }

    /// SetPhaseSource through the command channel: a valid source switches
    /// the manager (and mirrors into shared state), an invalid one is
    /// rejected and leaves everything unchanged.
    #[test]
    #[cfg(feature = "runtime")]
    fn process_commands_switches_phase_source() {
        let _serial = cmd_channel_lock();
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
        use crate::foc::trig::LibmSinCos;
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();

        let mut mgr = PhaseManager::sensorless();
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            1.0 / 20_000.0,
        );
        // The driver must stay linked so process_commands doesn't force Stopped.
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        // Valid switch: HFI estimator is configured.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hfi));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.phase().source(), PhaseSource::Hfi);
        let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
        assert_eq!(mirrored, PhaseSource::Hfi);

        // Invalid switch: no hall sensor on a sensorless manager.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hall));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.phase().source(),
            PhaseSource::Hfi,
            "invalid source must be rejected, not applied"
        );
        let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
        assert_eq!(mirrored, PhaseSource::Hfi);
    }

    /// Non-finite numbers must die at the command boundary: a NaN target
    /// reaching the PI loop turns the SVPWM output into a garbage voltage
    /// vector (bounded by the saturating casts, but still a mechanical
    /// jolt until the overcurrent check reacts).
    #[test]
    #[cfg(feature = "runtime")]
    fn process_commands_rejects_non_finite_payloads() {
        let _serial = cmd_channel_lock();
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        // NaN current target must be dropped — driver stays Stopped.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: f32::NAN,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "NaN current target must be rejected"
        );

        // Infinite direct voltage likewise.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::DirectVoltage {
            vd: f32::INFINITY,
            vq: 0.0,
            angle_rad: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), ControlMode::Stopped);

        // NaN PI gains must not reach the controller.
        let gains_before = driver.controller().id_pi.gains();
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPiGains {
            kp: f32::NAN,
            ki: 100.0,
        });
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.controller().id_pi.gains(),
            gains_before,
            "NaN gains must be rejected"
        );

        // A finite command still works.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0
            }
        );
    }

    /// Motor start must be refused while a flash operation is in flight:
    /// internal-flash erase stalls the chip, which must never overlap an
    /// energized motor. This is the ISR half of the config server's
    /// Busy-gate TOCTOU fix (the server arms FLASH_OP_PENDING before
    /// re-checking the motor state).
    #[test]
    #[cfg(feature = "runtime")]
    fn process_commands_blocks_start_during_flash_op() {
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::state::{
            CMD_CHANNEL, DriverCommand, FlashPendingGuard, MotorControlState, process_commands,
        };
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        let _serial = cmd_channel_lock();

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        let run_cmd = DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        });

        // Flash op in flight: the start must be refused.
        let pending = FlashPendingGuard::arm();
        let _ = CMD_CHANNEL.try_send(run_cmd);
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "start must be blocked while a flash operation is pending"
        );

        // Stop must stay allowed mid-operation — it is the safe direction.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), ControlMode::Stopped);

        // Flag cleared (guard dropped): the same start goes through.
        drop(pending);
        let _ = CMD_CHANNEL.try_send(run_cmd);
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0
            }
        );
    }

    /// Link loss must route through the configured failsafe policy (arm the
    /// controller), not the legacy instant hard-Stop — `process_commands`
    /// forces this whenever `link_active` is false and the motor is running.
    #[test]
    #[cfg(feature = "runtime")]
    fn link_loss_arms_failsafe_policy() {
        let _serial = cmd_channel_lock();
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_failsafe(FailsafeConfig {
            policy: FailsafePolicy::ControlledStop,
            ..FailsafeConfig::default()
        });

        // Link up, start running.
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert!(!driver.failsafe_active());

        // Link drops → next drain arms the failsafe controller (not a hard Stop).
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_inactive());
        process_commands(&state, &mut driver, &registry);
        assert!(
            driver.failsafe_active(),
            "link loss must arm the configured failsafe policy"
        );

        // …and latches: with the link back, a replayed running setpoint (a
        // reconnecting host's stale throttle) must be rejected until an
        // explicit safe-mode acknowledgement.
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
        assert!(driver.failsafe_latched());
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 5.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert!(
            driver.failsafe_active(),
            "running mode while latched must be rejected, not cancel the brake"
        );
        assert!(driver.failsafe_latched());

        // Stopped acknowledges ("throttle back to neutral"): latch clears…
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
        process_commands(&state, &mut driver, &registry);
        assert!(!driver.failsafe_latched());
        assert!(!driver.failsafe_active());
        assert_eq!(driver.mode(), ControlMode::Stopped);

        // …and a fresh running command is accepted again.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert!(matches!(driver.mode(), ControlMode::CurrentControl { .. }));
    }

    /// Bench/calibration modes are exempt from the deadman: on-device
    /// detection dwells up to ~1 s between `SetMode`s, which a 150 ms
    /// deadman would cut mid-measurement. Drive modes stay covered.
    #[test]
    fn deadman_exempts_bench_modes() {
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_failsafe(FailsafeConfig {
            policy: FailsafePolicy::Coast,
            staleness_timeout_us: 1_000,
            ..FailsafeConfig::default()
        });
        driver.note_command_tick(0);

        for mode in [
            ControlMode::OpenLoop {
                angle_rad: 0.0,
                current: 3.0,
                velocity_rad_s: 0.0,
                pi_gains: None,
            },
            ControlMode::DirectVoltage {
                vd: 1.0,
                vq: 0.0,
                angle_rad: 0.0,
            },
            ControlMode::SixStep { duty: 0.2 },
        ] {
            driver.set_mode(mode);
            assert!(
                !driver.deadman_expired(1_000_000),
                "bench mode {mode:?} must be deadman-exempt"
            );
        }

        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        });
        assert!(
            driver.deadman_expired(1_000_000),
            "drive mode stays covered"
        );
    }

    /// `enter_failsafe` from SixStep must restore the floated phases before
    /// driving the current loop (mirrors the SixStep-exit housekeeping in
    /// `set_mode`, which the failsafe bypasses).
    #[test]
    fn enter_failsafe_restores_six_step_phases() {
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

        /// MockPwm that records the last forced phase states.
        struct StatePwm {
            states: Option<[PhaseState; 3]>,
        }
        impl PhasePwm for StatePwm {
            fn max_duty(&self) -> u16 {
                1000
            }
            fn set_duties(&mut self, _duties: [u16; 3]) {}
            fn set_phase_states(&mut self, states: [PhaseState; 3]) {
                self.states = Some(states);
            }
        }

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            StatePwm { states: None },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_failsafe(FailsafeConfig {
            policy: FailsafePolicy::ControlledStop,
            ..FailsafeConfig::default()
        });
        driver.set_mode(ControlMode::SixStep { duty: 0.2 });
        // Run a six-step cycle so the commutation floats a phase.
        let _ = driver.step(1_000);

        driver.enter_failsafe();
        assert_eq!(
            driver.pwm().states,
            Some([PhaseState::Low, PhaseState::Low, PhaseState::Low]),
            "failsafe from SixStep must un-float all phases"
        );
    }

    /// Brake (windings short) is speed-gated at the command boundary, and
    /// once engaged it is a safe standing state: exempt from the deadman and
    /// from the link-loss failsafe — a parked board must stay braked.
    #[test]
    #[cfg(feature = "runtime")]
    fn brake_speed_gated_and_survives_link_loss() {
        let _serial = cmd_channel_lock();
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::{PhaseInput, PhaseOutput, PhaseProvider};
        use crate::foc::trig::LibmSinCos;
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
        }

        /// Phase provider with a directly settable velocity estimate.
        struct MockPhase {
            vel: f32,
        }
        impl PhaseProvider for MockPhase {
            fn get(&self) -> PhaseOutput {
                PhaseOutput {
                    angle: 0.0,
                    velocity: self.vel,
                }
            }
            fn update(&mut self, _input: &PhaseInput, _now_ticks: u64) {}
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            MockPhase { vel: 300.0 },
            1.0 / 20_000.0,
        );

        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);

        // Spinning fast → Brake is rejected, mode unchanged.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Brake));
        process_commands(&state, &mut driver, &registry);
        assert!(
            matches!(driver.mode(), ControlMode::CurrentControl { .. }),
            "Brake at speed must be rejected, got {:?}",
            driver.mode()
        );

        // Near standstill → accepted.
        driver.phase_mut().vel = 5.0;
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Brake));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), ControlMode::Brake);

        // Safe standing state: no deadman, however stale the command link.
        assert!(!driver.deadman_expired(u64::MAX / 2));

        // Link loss must NOT kick a parked board out of the brake.
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_inactive());
        process_commands(&state, &mut driver, &registry);
        assert!(!driver.failsafe_active(), "brake is exempt from link-loss");
        assert_eq!(driver.mode(), ControlMode::Brake);

        // And the Brake step drives all-low-side without erroring.
        assert!(driver.step(1_000).is_ok());
    }

    #[test]
    #[cfg(feature = "storage")]
    fn config_limits_clamp_to_hardware_ceiling() {
        use CurrentLimitsConfig;
        // Stored config must never raise limits above the board's hardware
        // ceiling, and zero/negative config values (= "not set") fall back
        // to the board defaults instead of disabling protection.
        let hw_max = 10.0;
        let cfg = |iq: f32, ph: f32, bus_in: f32, bus_regen: f32| CurrentLimitsConfig {
            max_iq_a: iq,
            max_phase_current_a: ph,
            bus_in_max_a: bus_in,
            bus_regen_max_a: bus_regen,
        };

        // Normal config below the ceiling: passes through.
        let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, -1.0, -1.0), hw_max, 0.0);
        assert_eq!(l.max_current_a, 5.0);
        assert_eq!(l.overcurrent_threshold_a, 8.0);
        assert_eq!(l.bus_in_max_a, -1.0);
        assert_eq!(l.bus_regen_max_a, -1.0);

        // Config above the ceiling: clamped to the board limits. The board
        // value is the ABS trip line; the iq ceiling keeps the headroom
        // below it (NOT hw_max itself — that put full throttle exactly on
        // the per-phase Kill line).
        let l = CurrentLimits::from_config_clamped(&cfg(50.0, 100.0, -1.0, -1.0), hw_max, 0.0);
        assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
        assert_eq!(l.overcurrent_threshold_a, hw_max);

        // Zeroed config: board defaults, NOT disabled protection.
        let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0, -1.0, -1.0), hw_max, 0.0);
        assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
        assert_eq!(l.overcurrent_threshold_a, hw_max);

        // Bus limits pass through (no board ceiling — they protect the
        // supply); zero is meaningful (no regen), NaN → unlimited.
        let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, 20.0, 0.0), hw_max, 0.0);
        assert_eq!(l.bus_in_max_a, 20.0);
        assert_eq!(l.bus_regen_max_a, 0.0);
        let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, f32::NAN, f32::NAN), hw_max, 0.0);
        assert_eq!(l.bus_in_max_a, -1.0);
        assert_eq!(l.bus_regen_max_a, -1.0);
    }

    /// The 2026-06-12 foot-gun (notes/fault-overhaul.md §4): a config that
    /// places the overcurrent trip at (or inside the headroom band of) the
    /// soft iq ceiling must not survive into the live limits — full
    /// throttle plus PI overshoot / HFI ripple would nuisance-Kill
    /// mid-ride. Protection wins over torque: iq is lowered, the trip is
    /// never raised.
    #[test]
    #[cfg(feature = "storage")]
    fn cross_field_headroom_enforced() {
        use CurrentLimitsConfig;
        // Board ABS line far above so it is not the binding constraint.
        let hw_max = 80.0;
        let l = CurrentLimits::from_config_clamped(
            &CurrentLimitsConfig {
                max_iq_a: 40.0,
                max_phase_current_a: 40.0,
                bus_in_max_a: -1.0,
                bus_regen_max_a: -1.0,
            },
            hw_max,
            0.0,
        );
        assert_eq!(l.overcurrent_threshold_a, 40.0, "trip is never raised");
        assert_eq!(
            l.max_current_a,
            40.0 / OVERCURRENT_HEADROOM,
            "iq must drop one headroom factor below the trip"
        );
        // The invariant holds whatever the inputs.
        assert!(l.overcurrent_threshold_a >= OVERCURRENT_HEADROOM * l.max_current_a);
    }

    /// Motor RATING is a ceiling above the operational config (layered
    /// semantics): the session can only lower limits below it, an unset
    /// config defaults to the rating itself, and the overcurrent trip
    /// ceiling is 1.5× the rating (VESC `l_abs_current_max`) — all still
    /// capped by the board hardware.
    #[cfg(feature = "storage")]
    #[test]
    fn rating_caps_operational_limits() {
        let hw_max = 30.0;
        let cfg = |iq: f32, phase: f32| CurrentLimitsConfig {
            max_iq_a: iq,
            max_phase_current_a: phase,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
        };
        let rating = 10.0;

        // Operational asks for more than the motor tolerates: rating wins.
        let l = CurrentLimits::from_config_clamped(&cfg(25.0, 28.0), hw_max, rating);
        assert_eq!(l.max_current_a, rating);
        assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);

        // Operational below the rating: passes through untouched.
        let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0), hw_max, rating);
        assert_eq!(l.max_current_a, 5.0);
        assert_eq!(l.overcurrent_threshold_a, 8.0);

        // Unset operational: the rating IS the default (VESC applies
        // l_current_max = i_max the same way).
        let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0), hw_max, rating);
        assert_eq!(l.max_current_a, rating);
        assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);

        // Board hardware still caps a huge rating: trip at the board ABS
        // line, iq one headroom factor below it.
        let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0), hw_max, 500.0);
        assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
        assert_eq!(l.overcurrent_threshold_a, hw_max);

        // NaN / zero rating: no rating clamp (pre-rating config blobs).
        // 25/28 is an incoherent pair (28 < 1.3·25): the cross-field clamp
        // lowers iq to trip/headroom on top of the board iq ceiling.
        let l = CurrentLimits::from_config_clamped(&cfg(25.0, 28.0), hw_max, f32::NAN);
        assert_eq!(
            l.max_current_a,
            (28.0 / OVERCURRENT_HEADROOM).min(hw_max / OVERCURRENT_HEADROOM)
        );
        assert_eq!(l.overcurrent_threshold_a, 28.0);

        // No limits group stored at all: rating still holds at boot.
        let l = CurrentLimits::from_stored(None, hw_max, rating);
        assert_eq!(l.max_current_a, rating);
        assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);
        // …and bus limits stay off (boot default).
        assert_eq!(l.bus_in_max_a, -1.0);
        assert_eq!(l.bus_regen_max_a, -1.0);
    }

    /// Bus (supply) current limits: `i_bus ≈ iq·mod_q`, so the allowed iq
    /// window scales inversely with the modulation. With regen = 0 no
    /// negative-power iq is allowed at all (lab-PSU safety); with both
    /// limits negative (the default) nothing is clamped.
    #[test]
    fn bus_limit_clamps_iq() {
        use crate::foc::trig::LibmSinCos;

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );

        // Default: unlimited, pass-through whatever the modulation.
        driver.bus_mod_q_filt = 0.5;
        assert_eq!(driver.clamp_iq_for_bus(-30.0), -30.0);
        assert_eq!(driver.clamp_iq_for_bus(30.0), 30.0);

        // bus_in 10 A, regen 0, mod_q = 0.5 (driving forward at half duty):
        // draw bound = 10/0.5 = 20 A, regen bound = 0 → no negative iq.
        driver.current_limits.bus_in_max_a = 10.0;
        driver.current_limits.bus_regen_max_a = 0.0;
        assert_eq!(driver.clamp_iq_for_bus(30.0), 20.0);
        assert_eq!(driver.clamp_iq_for_bus(-5.0), 0.0, "regen must be denied");
        assert_eq!(driver.clamp_iq_for_bus(15.0), 15.0);

        // Reverse rotation (mod_q < 0): bounds mirror.
        driver.bus_mod_q_filt = -0.5;
        assert_eq!(driver.clamp_iq_for_bus(-30.0), -20.0);
        assert_eq!(driver.clamp_iq_for_bus(5.0), 0.0, "regen must be denied");

        // Near-zero modulation: dead-band, no clamp (bus current ≈ 0).
        driver.bus_mod_q_filt = 0.0;
        assert_eq!(driver.clamp_iq_for_bus(-30.0), -30.0);

        // Finite regen allowance: -2 A bus at mod_q 0.4 → iq ≥ -5 A.
        driver.bus_mod_q_filt = 0.4;
        driver.current_limits.bus_regen_max_a = 2.0;
        assert_eq!(driver.clamp_iq_for_bus(-30.0), -5.0);
    }

    /// Every mode that energizes the motor and can read currents must trip
    /// the measured-overcurrent protection. DirectVoltage has no PI loop to
    /// rein the current in, and six-step has no current loop at all — a
    /// shorted phase in those modes must not cook the FETs just because
    /// the mode is "simple".
    #[test]
    fn direct_voltage_trips_overcurrent() {
        use crate::foc::trig::LibmSinCos;

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                // ~26 A magnitude — far above the 13 A threshold below.
                currents: (20.0, -10.0, -10.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        driver.set_mode(ControlMode::DirectVoltage {
            vd: 1.0,
            vq: 0.0,
            angle_rad: 0.0,
        });

        let res = driver.step(0);
        assert!(
            res.is_err(),
            "overcurrent must abort the DirectVoltage step"
        );
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "overcurrent must latch Stopped"
        );
    }

    #[test]
    fn six_step_trips_overcurrent() {
        use crate::foc::trig::LibmSinCos;

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (20.0, -10.0, -10.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        driver.set_mode(ControlMode::SixStep { duty: 0.2 });

        let res = driver.step(0);
        assert!(res.is_err(), "overcurrent must abort the six-step step");
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "overcurrent must latch Stopped"
        );
    }

    /// Closed-loop HFI through the full runtime path: FocDriver in
    /// CurrentControl with a PhaseManager(HfiObserver) source must apply
    /// the estimator's carrier injection itself — no detection-mode help.
    /// The rotor is parked on the far side (2.5 rad: the PLL's nearest
    /// saliency lock is the π-flipped one), so this exercises BOTH the
    /// carrier path and the saturation polarity probe under an active PI
    /// loop that partially fights the probe pulses.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn current_control_drives_hfi_estimator() {
        use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const ROTOR_ANGLE: f32 = 2.5;
        // Same IPM as the estimator-level HFI tests: 3:1 saliency, heavy
        // rotor + friction so the injection itself doesn't move it, plus
        // d-axis saturation so the polarity probe has a signal.
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-2,
            friction_b: 5e-2,
            hall_offset: 0.0,
            sat_k: 0.05,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(ROTOR_ANGLE);

        // 1000 rad/s current loop — well below the 1 kHz carrier, so the
        // PI regulates the fundamental without eating the carrier response.
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );
        let mut mgr = PhaseManager::sensorless();
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::Hfi).unwrap();

        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        // Carrier + polarity-probe currents exceed the bench-safe default
        // limits — give the test the board-scale ceiling explicitly.
        driver.set_current_limits(CurrentLimits::from_max_current(40.0));
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 0.0,
            id_target: 0.0,
        });

        let mut out = VirtualMotorOutput::default();
        for step in 1..20_000u64 {
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            let telem = driver.step(step * 50).expect("FOC step failed");
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        // Full-circle match: the polarity probe must have corrected the
        // π-flipped initial lock through the driver path.
        let true_angle = wrap_angle(out.angle_rad);
        let err = angle_difference(driver.phase().get().angle, true_angle).abs();
        assert!(
            err < 0.15,
            "HFI did not converge through FocDriver: est {} vs rotor {} (err {} full-circle)",
            driver.phase().get().angle,
            true_angle,
            err
        );
        let hfi = driver.phase().hfi_observer().unwrap();
        assert!(
            hfi.is_ready(),
            "HFI must be ready (locked + polarity resolved), confidence {}",
            hfi.confidence()
        );
        assert!(
            angle_difference(out.angle_rad, ROTOR_ANGLE).abs() < 0.15,
            "injection moved the rotor: {} rad",
            out.angle_rad
        );
    }

    /// The ISR-resident deadman: a running driver with no fresh setpoint for
    /// longer than the staleness timeout arms the failsafe; a fresh
    /// `note_command_tick` (and `set_mode`) re-arms normal control.
    #[test]
    fn deadman_arms_failsafe_on_stale_command() {
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_failsafe(FailsafeConfig {
            policy: FailsafePolicy::Coast,
            staleness_timeout_us: 1_000, // 1 ms
            ..FailsafeConfig::default()
        });

        // Stopped: deadman never fires, however stale.
        assert!(!driver.deadman_expired(1_000_000));

        // Running + affirmed: not stale yet.
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        });
        driver.note_command_tick(0);
        assert!(!driver.deadman_expired(500));
        assert!(!driver.failsafe_active());

        // Past the timeout with no fresh command → arm.
        assert!(driver.deadman_expired(2_000));
        driver.enter_failsafe();
        assert!(driver.failsafe_active());

        // A fresh command re-arms control: set_mode clears the failsafe, and
        // the new stamp un-stales the deadman.
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 2.0,
            id_target: 0.0,
        });
        driver.note_command_tick(2_000);
        assert!(!driver.failsafe_active());
        assert!(!driver.deadman_expired(2_500));
    }

    /// Closed-loop: spin a VirtualMotor up on Hall, stop affirming, and the
    /// ControlledStop policy must regen-brake it to a near standstill and end
    /// Stopped. Drives the failsafe through the real current-control path.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn controlled_stop_brakes_spinning_motor_to_standstill() {
        use crate::foc::hall_sensor::HallSensor;
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        // Light rotor + some friction so it both spins up and brakes quickly.
        let params = MotorParams {
            friction_b: 2e-3,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);

        // Prime the hall estimator with the initial rotor state.
        let mut out = motor.step(0.0, 0.0, 0.0, DT);
        let mut hall = HallSensor::new(1_000_000); // µs timebase
        hall.update(out.hall_state, 0);
        let mut last_hall = out.hall_state;

        let mgr = PhaseManager::with_hall(hall); // default source = Hall
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(40.0));
        driver.set_failsafe(FailsafeConfig {
            policy: FailsafePolicy::ControlledStop,
            staleness_timeout_us: 5_000,
            brake_current_a: 20.0,
            ramp_s: 0.02,
            brake_time_s: 2.0,
            standstill_rad_s: 20.0,
            decel_rad_s2: 800.0,
            terminal: FailsafeTerminal::HighZ,
        });
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 3.0,
            id_target: 0.0,
        });

        const STOP_AFFIRMING_AT: u64 = 30_000; // 1.5 s of drive, then go silent
        let mut peak_omega = 0.0f32;
        for step in 1..70_000u64 {
            let now = step * 50; // µs

            if out.hall_state != last_hall {
                driver.phase_mut().hall_mut().update(out.hall_state, now);
                last_hall = out.hall_state;
            }
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);

            // Emulate run_foc_cycle's deadman (host affirms, then stops).
            if step < STOP_AFFIRMING_AT {
                driver.note_command_tick(now);
            }
            if driver.deadman_expired(now) {
                driver.enter_failsafe();
            }

            let telem = driver.step(now).unwrap_or_default();
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

            if step < STOP_AFFIRMING_AT {
                peak_omega = peak_omega.max(out.omega_e.abs());
            }
        }

        assert!(
            peak_omega > 50.0,
            "motor should have spun up, peak ωe = {peak_omega}"
        );
        assert!(
            out.omega_e.abs() < 25.0,
            "failsafe must brake to ~standstill, ωe = {}",
            out.omega_e
        );
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "ends Stopped after the brake"
        );
        assert!(
            !driver.failsafe_active(),
            "failsafe cleared at the terminal"
        );
    }

    /// Closed-loop ramp-into-brake: a user Brake command at speed substitutes
    /// the controlled-stop ramp (decel-limited, via the failsafe machinery)
    /// and ends parked in `ControlMode::Brake` — without setting the
    /// failsafe re-arm latch (user-commanded, no "back to neutral" owed).
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn brake_at_speed_ramps_to_standstill_then_parks() {
        use crate::foc::hall_sensor::HallSensor;
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams {
            friction_b: 2e-3,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);

        let mut out = motor.step(0.0, 0.0, 0.0, DT);
        let mut hall = HallSensor::new(1_000_000);
        hall.update(out.hall_state, 0);
        let mut last_hall = out.hall_state;

        let mgr = PhaseManager::with_hall(hall);
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(40.0));
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 3.0,
            id_target: 0.0,
        });

        const BRAKE_AT: u64 = 20_000; // 1 s of drive, then the user brakes
        let mut peak_omega = 0.0f32;
        for step in 1..80_000u64 {
            let now = step * 50;
            if step == BRAKE_AT {
                assert!(peak_omega > 50.0, "should be spinning, peak {peak_omega}");
                // What the process_commands gate does for Brake at speed.
                assert!(driver.enter_brake_ramp());
                assert!(
                    !driver.failsafe_latched(),
                    "user brake must not set the re-arm latch"
                );
            }
            if out.hall_state != last_hall {
                driver.phase_mut().hall_mut().update(out.hall_state, now);
                last_hall = out.hall_state;
            }
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            if step < BRAKE_AT {
                driver.note_command_tick(now);
                peak_omega = peak_omega.max(out.omega_e.abs());
            }
            let telem = driver.step(now).unwrap_or_default();
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        assert_eq!(
            driver.mode(),
            ControlMode::Brake,
            "ends parked in Brake after the ramp"
        );
        assert!(!driver.failsafe_active());
        assert!(!driver.failsafe_latched());
        assert!(
            out.omega_e.abs() < 25.0,
            "must be ~standstill, ωe = {}",
            out.omega_e
        );
    }

    /// Closed-loop velocity control: spin a VirtualMotor on Hall via
    /// `ControlMode::VelocityControl`, assert it tracks the target, then
    /// retarget (no loop reset — the ramp carries it) and track again.
    /// Exercises the full cascade: velocity loop → current loop → SVPWM.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn velocity_control_tracks_target_on_virtual_motor() {
        use crate::foc::hall_sensor::HallSensor;
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::foc::velocity::VelocityLoopConfig;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default();
        let mut motor = VirtualMotor::new(params);

        // Prime the hall estimator with the initial rotor state.
        let mut out = motor.step(0.0, 0.0, 0.0, DT);
        let mut hall = HallSensor::new(1_000_000); // µs timebase
        hall.update(out.hall_state, 0);
        let mut last_hall = out.hall_state;

        let mgr = PhaseManager::with_hall(hall); // default source = Hall
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        // Soft gains + a moderate ramp: hall only updates the velocity
        // estimate at edges (~7 ms apart at these speeds), so the loop must
        // not change the speed much within one edge interval — aggressive
        // gains turn the stale estimate into a limit cycle (seen with
        // kp=0.05: ±100 rad/s oscillation around the target).
        driver.set_velocity_config(VelocityLoopConfig {
            kp: 0.008,
            ki: 0.2,
            accel_limit: 400.0, // erad/s²
        });

        // Track 300 electrical rad/s from standstill (1.5 s), then retarget
        // downward within velocity mode (ramped, no reset) for another 1.5 s.
        const RETARGET_AT: u64 = 30_000;
        driver.set_mode(ControlMode::VelocityControl { target_vel: 300.0 });
        for step in 1..=60_000u64 {
            let now = step * 50; // µs
            if step == RETARGET_AT {
                assert!(
                    (out.omega_e - 300.0).abs() < 30.0,
                    "should track 300 erad/s, got {}",
                    out.omega_e
                );
                // Stay well clear of zero: hall velocity is degenerate
                // through standstill/reversal (edge intervals blow up) —
                // a cruise loop never legitimately crosses zero anyway.
                driver.set_mode(ControlMode::VelocityControl { target_vel: 150.0 });
            }
            if out.hall_state != last_hall {
                driver.phase_mut().hall_mut().update(out.hall_state, now);
                last_hall = out.hall_state;
            }
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            let telem = driver.step(now).unwrap_or_default();
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }
        assert!(
            (out.omega_e - 150.0).abs() < 25.0,
            "should track 150 erad/s after retarget, got {}",
            out.omega_e
        );
    }

    /// Severity gate in `run_foc_cycle` (docs/notes/fault-overhaul.md):
    /// Warning never touches the motor, GracefulStop routes through the
    /// failsafe machinery, Kill cuts PWM and latches Error; the deadman
    /// raises CommTimeout and a fresh SetMode clears it.
    #[cfg(feature = "runtime")]
    mod severity_gate {
        use super::*;
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::hall_sensor::HallFaultKind;
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::state::{
            CMD_CHANNEL, DriverCommand, MotorControlState, process_commands, run_foc_cycle,
        };
        use crate::types::MotorState;
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq, Debug)]
        enum SevFault {
            OverCurrent,
            OverVoltage,
            OverTemp,
            Hall,
            CommTimeout,
            Derating,
        }
        impl PlatformFault for SevFault {
            fn category(&self) -> FaultCategory {
                match self {
                    Self::OverCurrent => FaultCategory::OverCurrent,
                    Self::OverVoltage => FaultCategory::OverVoltage,
                    Self::OverTemp => FaultCategory::OverTemp,
                    Self::Hall => FaultCategory::HallError,
                    Self::CommTimeout => FaultCategory::CommTimeout,
                    Self::Derating => FaultCategory::Derating,
                }
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
            fn from_hall_kind(_kind: HallFaultKind) -> Option<Self> {
                Some(Self::Hall)
            }
            fn from_category(category: FaultCategory) -> Option<Self> {
                match category {
                    FaultCategory::OverCurrent => Some(Self::OverCurrent),
                    FaultCategory::OverVoltage => Some(Self::OverVoltage),
                    FaultCategory::OverTemp => Some(Self::OverTemp),
                    FaultCategory::CommTimeout => Some(Self::CommTimeout),
                    FaultCategory::Derating => Some(Self::Derating),
                    _ => None,
                }
            }
        }

        const BOARD: BoardConfig = BoardConfig {
            shunt_ohms: 0.003,
            amp_gain: 16.0,
            vbus_divider_ratio: 10.39,
            adc_vref_mv: 3300,
            adc_max_counts: 4095,
            initial_vbus_volts: 24.0,
            max_iq_target_a: 10.0,
            invert_current_sign: false,
            max_phase_current_a: 40.0,
            max_vbus_mv: 60_000,
            min_vbus_mv: 8_000,
            max_fet_temp_c: 100.0,
            max_motor_temp_c: 120.0,
        };

        struct Harness {
            state: CriticalSectionMutex<RefCell<MotorControlState>>,
            registry: FaultRegistry<SevFault>,
            driver: FocDriver<MockPwm, MockCurrentSensor, PhaseManager, LibmSinCos>,
        }

        /// Driver running CurrentControl (entered via the command channel at
        /// t=0, which also stamps the deadman), link active, Manual angle.
        fn running_harness() -> Harness {
            let state: CriticalSectionMutex<RefCell<MotorControlState>> =
                CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
            let registry: FaultRegistry<SevFault> = FaultRegistry::new();
            let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
            let driver = FocDriver::new(
                foc,
                MockPwm { duties: [0; 3] },
                MockCurrentSensor {
                    currents: (0.0, 0.0, 0.0),
                },
                PhaseManager::sensorless(),
                1.0 / 20_000.0,
            );
            critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
            let mut h = Harness {
                state,
                registry,
                driver,
            };
            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            }));
            let out = h.cycle(0);
            assert!(out.is_some(), "harness must start cleanly");
            assert!(matches!(
                h.driver.mode(),
                ControlMode::CurrentControl { .. }
            ));
            h
        }

        impl Harness {
            fn cycle(&mut self, now_ticks: u64) -> Option<FocOutput> {
                run_foc_cycle(
                    &self.state,
                    &self.registry,
                    &mut self.driver,
                    24.0,
                    now_ticks,
                    &BOARD,
                )
            }

            fn motor_state(&self) -> MotorState {
                critical_section::with(|cs| self.state.borrow(cs).borrow().motor_state)
            }
        }

        #[test]
        fn warning_fault_keeps_motor_running() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();

            h.registry.set(SevFault::Hall);
            let out = h.cycle(50);

            assert!(out.is_some(), "warning must not skip the FOC step");
            assert!(matches!(
                h.driver.mode(),
                ControlMode::CurrentControl { .. }
            ));
            assert!(!h.driver.failsafe_active(), "warning must not arm failsafe");
            assert_eq!(h.motor_state(), MotorState::Running);
        }

        #[test]
        fn warning_fault_does_not_block_start() {
            let _serial = cmd_channel_lock();
            let state: CriticalSectionMutex<RefCell<MotorControlState>> =
                CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
            let registry: FaultRegistry<SevFault> = FaultRegistry::new();
            registry.set(SevFault::Hall);
            let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
            let mut driver = FocDriver::new(
                foc,
                MockPwm { duties: [0; 3] },
                MockCurrentSensor {
                    currents: (0.0, 0.0, 0.0),
                },
                PhaseManager::sensorless(),
                1.0 / 20_000.0,
            );
            critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            }));
            process_commands(&state, &mut driver, &registry);
            assert!(
                matches!(driver.mode(), ControlMode::CurrentControl { .. }),
                "a warning-class fault must not block starting"
            );
        }

        #[test]
        fn graceful_stop_fault_arms_failsafe_not_high_z() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();

            h.registry.set(SevFault::OverTemp);
            let out = h.cycle(50);

            assert!(h.driver.failsafe_active(), "GracefulStop must arm failsafe");
            assert!(out.is_some(), "failsafe drives through the normal step");
            assert_ne!(
                h.motor_state(),
                MotorState::Error,
                "GracefulStop must not latch Error"
            );

            // Restart stays blocked while the fault is active (start gate).
            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
            h.cycle(100);
            assert!(matches!(h.driver.mode(), ControlMode::Stopped));
            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            }));
            h.cycle(150);
            assert!(
                matches!(h.driver.mode(), ControlMode::Stopped),
                "stopping-class fault must block restart"
            );
        }

        #[test]
        fn kill_fault_cuts_pwm_and_latches_error() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();

            h.registry.set(SevFault::OverCurrent);
            let out = h.cycle(50);

            assert!(out.is_none(), "Kill must skip the FOC step");
            assert!(matches!(h.driver.mode(), ControlMode::Stopped));
            assert_eq!(h.motor_state(), MotorState::Error);
        }

        /// The hall bridge is STICKY: the warning lands in the registry on
        /// degradation, the motor keeps running (warning class), and the
        /// record survives the hall's recovery — only a host clear removes
        /// it, while the live fallback behavior recovers immediately.
        #[test]
        fn hall_degradation_bridges_sticky_warning() {
            let _serial = cmd_channel_lock();
            use crate::foc::hall_sensor::HallSensor;

            let state: CriticalSectionMutex<RefCell<MotorControlState>> =
                CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
            let registry: FaultRegistry<SevFault> = FaultRegistry::new();
            let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
            let mut hall = HallSensor::new(1_000_000);
            hall.update(1, 0); // boot seed: healthy, valid sample from t=0
            let mut driver = FocDriver::new(
                foc,
                MockPwm { duties: [0; 3] },
                MockCurrentSensor {
                    currents: (0.0, 0.0, 0.0),
                },
                PhaseManager::with_hall(hall), // default source = Hall
                1.0 / 20_000.0,
            );
            critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

            let cycle = |driver: &mut FocDriver<_, _, _, _>, now: u64| {
                run_foc_cycle(&state, &registry, driver, 24.0, now, &BOARD)
            };

            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            }));
            cycle(&mut driver, 0);
            assert!(!registry.has_category(FaultCategory::HallError));

            // Hall dies: invalid state (cable cut, pull-ups → 0b111).
            driver.phase_mut().hall_mut().update(0b111, 10);
            cycle(&mut driver, 50);
            assert!(
                registry.has_category(FaultCategory::HallError),
                "degradation must bridge into the registry"
            );
            assert!(
                matches!(driver.mode(), ControlMode::CurrentControl { .. }),
                "warning class: the motor keeps running"
            );

            // Hall recovers: behavior follows, the record stays.
            driver.phase_mut().hall_mut().update(1, 100);
            cycle(&mut driver, 150);
            assert!(
                registry.has_category(FaultCategory::HallError),
                "the warning is sticky — only a host clear removes it"
            );
            assert!(matches!(driver.mode(), ControlMode::CurrentControl { .. }));
        }

        #[test]
        fn deadman_raises_comm_timeout_and_fresh_command_clears_it() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();

            // Quiet link until past the staleness timeout (default 150 ms;
            // ticks are µs in this domain).
            h.cycle(100_000);
            assert!(!h.registry.has_category(FaultCategory::CommTimeout));

            h.cycle(200_000);
            assert!(
                h.registry.has_category(FaultCategory::CommTimeout),
                "stale command link must raise CommTimeout"
            );
            assert!(
                h.driver.failsafe_active(),
                "CommTimeout severity must arm the failsafe via the gate"
            );
            assert_ne!(h.motor_state(), MotorState::Error);

            // Commands flowing again (even just an ack) clear the fault; the
            // re-arm latch still demands the explicit safe mode it received.
            let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
            h.cycle(200_050);
            assert!(
                !h.registry.has_category(FaultCategory::CommTimeout),
                "a drained SetMode proves liveness and clears CommTimeout"
            );
            assert!(matches!(h.driver.mode(), ControlMode::Stopped));
        }

        /// Voltage faults trip on INTEGRATED excursion, not single samples
        /// (run_protection): one over-voltage cycle (a regen spike / sense
        /// blip) must ride through; a sustained excursion must Kill.
        #[test]
        fn voltage_fault_integrates_instead_of_single_sample() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();

            // One 50 µs cycle at +10 V over the 60 V board limit:
            // 10 V · 50 µs = 0.5 mV·s, under the 3 mV·s trip — no fault.
            run_cycle_at_vbus(&mut h, 70.0, 50);
            assert!(
                !h.registry.has_category(FaultCategory::OverVoltage),
                "a single-sample excursion must not trip"
            );
            // Back in range, the integral decays (τ ≈ 5 ms ≫ one cycle).
            h.cycle(100);

            // Sustained excursion: trips within ~0.3 ms (VESC-equivalent).
            let mut now = 150;
            for _ in 0..20 {
                run_cycle_at_vbus(&mut h, 70.0, now);
                now += 50;
            }
            assert!(
                h.registry.has_category(FaultCategory::OverVoltage),
                "sustained overvoltage must trip"
            );
            assert!(matches!(h.driver.mode(), ControlMode::Stopped), "OV = Kill");
            assert_eq!(h.motor_state(), MotorState::Error);
        }

        fn run_cycle_at_vbus(h: &mut Harness, vbus: f32, now: u64) {
            run_foc_cycle(&h.state, &h.registry, &mut h.driver, vbus, now, &BOARD);
        }

        /// The derating warning follows the live scales with hysteresis
        /// (set < 0.8, clear > 0.95), and the warning never stops the motor.
        #[test]
        fn derating_warning_sets_and_clears_with_hysteresis() {
            let _serial = cmd_channel_lock();
            let mut h = running_harness();
            h.driver.set_derating(DeratingConfig {
                vbus_cut_start_v: 22.0,
                vbus_cut_end_v: 18.0,
                ..Default::default()
            });

            // Sagged bus (drive scale 0.25 at 19 V): the decimated update
            // fires within 256 cycles.
            let mut now = 50;
            for _ in 0..300 {
                run_cycle_at_vbus(&mut h, 19.0, now);
                now += 50;
            }
            assert!(
                h.registry.has_category(FaultCategory::Derating),
                "deep derate must raise the warning (scales {:?})",
                h.driver.derating()
            );
            assert!(
                matches!(h.driver.mode(), ControlMode::CurrentControl { .. }),
                "warning class: the motor keeps running"
            );
            assert!(h.driver.derating().drive < 0.5);

            // Bus recovered: warning auto-clears (a live state, unlike the
            // sticky hall record).
            for _ in 0..300 {
                run_cycle_at_vbus(&mut h, 24.0, now);
                now += 50;
            }
            assert!(
                !h.registry.has_category(FaultCategory::Derating),
                "derating warning must auto-clear on recovery"
            );
            assert_eq!(h.driver.derating(), DeratingScales::IDENTITY);
        }
    }

    /// Derating scales clamp the iq budget by DIRECTION: drive (iq·ω ≥ 0)
    /// takes `drive`, opposing torque takes `brake` — a speed/sag derate
    /// must never weaken the brakes.
    #[test]
    #[cfg(feature = "runtime")]
    fn derating_clamps_drive_and_brake_separately() {
        use crate::foc::phase::{PhaseManager, PhaseSource};
        use crate::foc::trig::LibmSinCos;

        let make = |scales: DeratingScales, iq: f32| {
            let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
            let mut mgr = PhaseManager::sensorless();
            mgr.set_source(PhaseSource::OpenLoop).unwrap();
            mgr.set_open_loop_velocity(100.0);
            let mut driver = FocDriver::new(
                foc,
                MockPwm { duties: [0; 3] },
                MockCurrentSensor {
                    currents: (0.0, 0.0, 0.0),
                },
                mgr,
                1.0 / 20_000.0,
            );
            driver.set_current_limits(CurrentLimits::from_max_current(10.0));
            driver.set_derating_scales(scales);
            // Warmup at zero target: latches the phase velocity into the
            // managed output without accumulating PI state.
            driver.set_mode(ControlMode::CurrentControl {
                iq_target: 0.0,
                id_target: 0.0,
            });
            driver.step(50).unwrap();
            driver.set_mode(ControlMode::CurrentControl {
                iq_target: iq,
                id_target: 0.0,
            });
            driver.step(100).unwrap().vq
        };

        let full = make(DeratingScales::IDENTITY, 10.0);
        let derated = make(
            DeratingScales {
                drive: 0.5,
                brake: 1.0,
            },
            10.0,
        );
        assert!(
            (derated - 0.5 * full).abs() < 0.05 * full.abs(),
            "drive iq must be halved: full vq {full}, derated vq {derated}"
        );

        // Braking (iq opposes ω): only the brake scale applies — full here.
        let full_brake = make(DeratingScales::IDENTITY, -10.0);
        let brake_with_drive_derate = make(
            DeratingScales {
                drive: 0.5,
                brake: 1.0,
            },
            -10.0,
        );
        assert!(
            (brake_with_drive_derate - full_brake).abs() < 0.05 * full_brake.abs(),
            "a drive derate must not weaken the brake: {full_brake} vs {brake_with_drive_derate}"
        );
    }
}
