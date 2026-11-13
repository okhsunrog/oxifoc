//! Phase-current limits, the bus clamp, derating, and the shared protection update.

use super::FocDriver;
use crate::foc::clamp_f32;
#[cfg(any(feature = "storage", feature = "runtime"))]
use crate::foc::config::BoardConfig;
use crate::foc::fast_math::sqrtf;
#[cfg(feature = "runtime")]
use crate::foc::fault::FaultCategory;
#[cfg(feature = "runtime")]
use crate::foc::fault::FaultRegistry;
#[cfg(feature = "runtime")]
use crate::foc::fault::PlatformFault;
#[cfg(feature = "runtime")]
use crate::foc::fault::VOLTAGE_HYSTERESIS_MV;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::{PhasePwm, QAxisModulation};
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
use crate::motor::derating::DeratingConfig;
use crate::motor::derating::DeratingScales;
use crate::motor::dissipation::RegenBudget;
use crate::motor::dissipation::WindingModel;
use crate::motor::dissipation::dissipate_with_id;
use crate::motor::drive::DriveConfig;

struct BusIqBounds {
    lo: f32,
    hi: f32,
}

impl BusIqBounds {
    const IDENTITY: Self = Self {
        lo: f32::NEG_INFINITY,
        hi: f32::INFINITY,
    };

    #[inline(always)]
    fn clamp(&self, iq: f32) -> f32 {
        clamp_f32(iq, self.lo, self.hi)
    }
}

/// One cycle's velocity-loop authority and final torque-grant constraints.
/// Construct after selecting physical torque speed and reuse only within that
/// cycle, before protection/model/modulation state changes. The nonlinear
/// dissipative brake remains request-dependent and is solved for each request.
pub(super) struct TorqueConstraints {
    bus: BusIqBounds,
    omega: f32,
    drive_limit: f32,
    plain_brake_limit: f32,
}

/// Immutable board configuration and its voltage-protection constants.
///
/// Prepare once before entering the control loop. The source configuration
/// is owned and exposed read-only, so changing a board requires preparing a
/// new value rather than leaving derived thresholds behind.
#[cfg(feature = "runtime")]
#[derive(Clone, Debug)]
pub struct PreparedBoardConfig {
    board: BoardConfig,
    voltage: PreparedVoltageLimits,
}

/// Voltage thresholds and excursion constants are replaced as one value.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(not(any(feature = "storage", feature = "runtime")), allow(dead_code))]
pub struct PreparedVoltageLimits {
    min_mv: u32,
    #[cfg(feature = "storage")]
    max_mv: u32,
    max_vbus_v: f32,
    min_vbus_v: f32,
    trip_vs: f32,
    integral_ceiling: f32,
}

#[cfg(feature = "storage")]
impl<P: PhasePwm, C: CurrentSensor, Ph: PhaseProvider, S: SinCos> FocDriver<P, C, Ph, S> {
    /// Boot and live writes share the same validation and installation path.
    /// Invalid stored thresholds abort initialization before the bridge starts.
    pub fn configure_voltage_protection(
        &mut self,
        board: &BoardConfig,
        stored: Option<&crate::storage::VoltageLimitsConfig>,
    ) -> Result<(), super::MotorModelError> {
        if self.mode() != crate::motor::ControlMode::Stopped || self.current_offset_active() {
            return Err(super::MotorModelError::Busy);
        }
        let bounds = PreparedVoltageLimits::from_bounds(board.min_vbus_mv, board.max_vbus_mv);
        let voltage = match stored {
            Some(value) => {
                let prepared = PreparedVoltageLimits::from_config(value)
                    .filter(|v| v.fits(bounds))
                    .ok_or(super::MotorModelError::Invalid)?;
                Some(prepared)
            }
            None => None,
        };
        self.protection.board_voltage = Some(bounds);
        self.protection.voltage = voltage;
        Ok(())
    }

    pub fn apply_voltage_limits(
        &mut self,
        prepared: PreparedVoltageLimits,
    ) -> Result<(), super::MotorModelError> {
        if self.mode() != crate::motor::ControlMode::Stopped || self.current_offset_active() {
            return Err(super::MotorModelError::Busy);
        }
        let bounds = self
            .protection
            .board_voltage
            .ok_or(super::MotorModelError::Unsupported)?;
        if !prepared.fits(bounds) {
            return Err(super::MotorModelError::Invalid);
        }
        // Preserve accumulated excursions and latched faults: configuration
        // is not a way to acknowledge or erase an existing voltage fault.
        self.protection.voltage = Some(prepared);
        Ok(())
    }

    pub(crate) fn reset_voltage_limits(&mut self) {
        self.protection.voltage = None;
    }
}

impl PreparedVoltageLimits {
    #[cfg(any(feature = "storage", feature = "runtime"))]
    const fn from_bounds(min_mv: u32, max_mv: u32) -> Self {
        let max_vbus_v = max_mv as f32 / 1000.0;
        let min_vbus_v = min_mv as f32 / 1000.0;
        let trip_vs = 5e-5 * max_vbus_v;
        let integral_ceiling = trip_vs * 4.0;
        Self {
            min_mv,
            #[cfg(feature = "storage")]
            max_mv,
            max_vbus_v,
            min_vbus_v,
            trip_vs,
            integral_ceiling,
        }
    }

    #[cfg(feature = "storage")]
    pub fn from_config(config: &crate::storage::VoltageLimitsConfig) -> Option<Self> {
        config
            .is_sane()
            .then(|| Self::from_bounds(config.min_vbus_mv, config.max_vbus_mv))
    }

    #[cfg(feature = "storage")]
    fn fits(self, board: Self) -> bool {
        self.min_mv >= board.min_mv && self.max_mv <= board.max_mv
    }
}

#[cfg(feature = "runtime")]
impl PreparedBoardConfig {
    pub const fn new(board: BoardConfig) -> Self {
        Self {
            voltage: PreparedVoltageLimits::from_bounds(board.min_vbus_mv, board.max_vbus_mv),
            board,
        }
    }

    pub const fn board(&self) -> &BoardConfig {
        &self.board
    }
}

/// Slow-sensor fault detection and derating, independent of PWM and modes.
pub(super) struct ProtectionMonitor {
    board_voltage: Option<PreparedVoltageLimits>,
    voltage: Option<PreparedVoltageLimits>,
    config: DeratingConfig,
    drive: DriveConfig,
    #[cfg(feature = "storage")]
    drive_config_valid: bool,
    environment_drive: f32,
    #[cfg(feature = "fast-telemetry")]
    temperature_drive: f32,
    #[cfg(feature = "fast-telemetry")]
    bus_voltage_drive: f32,
    duty_utilization: f32,
    scales: DeratingScales,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    protection_tick: u16,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    ov_integral_vs: f32,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    uv_integral_vs: f32,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    voltage_decay: f32,
}

#[cfg(feature = "runtime")]
pub(super) struct ProtectionSample {
    pub dt: f32,
    pub vbus_v: f32,
    pub velocity: f32,
    pub fet_temp_c_x10: Option<i16>,
    pub motor_temp_c_x10: Option<i16>,
}

impl ProtectionMonitor {
    pub(super) fn overvoltage_threshold(&self) -> f32 {
        self.voltage
            .or(self.board_voltage)
            .map_or(0.0, |v| v.max_vbus_v)
    }

    pub(super) fn new(dt: f32) -> Self {
        Self {
            board_voltage: None,
            voltage: None,
            config: DeratingConfig::default(),
            drive: DriveConfig::default(),
            #[cfg(feature = "storage")]
            drive_config_valid: true,
            environment_drive: 1.0,
            #[cfg(feature = "fast-telemetry")]
            temperature_drive: 1.0,
            #[cfg(feature = "fast-telemetry")]
            bus_voltage_drive: 1.0,
            duty_utilization: 0.0,
            scales: DeratingScales::default(),
            protection_tick: 0,
            ov_integral_vs: 0.0,
            uv_integral_vs: 0.0,
            voltage_decay: 1.0 - (dt / 0.005).min(1.0),
        }
    }

    pub(super) fn configure(&mut self, config: DeratingConfig) {
        self.config = config;
    }

    pub(super) fn set_control_period(&mut self, dt: f32) {
        self.voltage_decay = 1.0 - (dt / 0.005).min(1.0);
    }

    pub(super) fn config(&self) -> &DeratingConfig {
        &self.config
    }
    pub(super) fn scales(&self) -> DeratingScales {
        self.scales
    }
    pub(super) fn set_scales(&mut self, scales: DeratingScales) {
        self.environment_drive = scales.drive;
        self.scales = scales;
    }

    fn speed_ceiling(&self, physical_direction: f32) -> f32 {
        self.config
            .speed
            .speed_ceiling(self.drive.rider_sign(physical_direction))
    }

    fn drive_cut(&self, omega: f32) -> f32 {
        self.config
            .speed
            .speed_cut(self.drive.rider_sign(omega))
            .min(self.duty_cut())
    }

    fn duty_cut(&self) -> f32 {
        let start = self.config.modulation.duty_start_frac;
        if start >= 1.0 || self.duty_utilization <= start {
            1.0
        } else {
            clamp_f32((1.0 - self.duty_utilization) / (1.0 - start), 0.0, 1.0)
        }
    }

    /// Integrate voltage excursions each cycle, check temperature and update
    /// derating every 256 cycles. Fault registry hysteresis stays here too.
    #[cfg(feature = "runtime")]
    pub(super) fn update<F: PlatformFault>(
        &mut self,
        registry: &FaultRegistry<F>,
        prepared: &PreparedBoardConfig,
        sample: ProtectionSample,
    ) {
        let board = prepared.board();
        // --- Voltage excursion integrals (every cycle) ---
        // Leaky-bucket trip: integrate the excursion, fire past `trip_vs`,
        // bleed it off in-range with τ≈5 ms. The integral is capped at a few
        // × `trip_vs` so a long excursion can't bank unbounded "credit" and
        // make a later marginal spike trip instantly; the cap sits above the
        // trip point, so first-trip latency is unchanged.
        //
        // A non-finite `vbus_v` leaves both comparisons false → neither fault
        // trips and the integrals just decay. Acceptable here: vbus is a
        // linear ADC divider with no division, so NaN is not physically
        // reachable — mirrors the host-boundary "reject NaN" philosophy (the
        // ISR trusts its own sense path).
        let voltage = self.voltage.unwrap_or(prepared.voltage);
        let max_v = voltage.max_vbus_v;
        let min_v = voltage.min_vbus_v;
        let trip_vs = voltage.trip_vs;
        let integral_ceiling = voltage.integral_ceiling;
        let decay = self.voltage_decay;
        if sample.vbus_v > max_v {
            self.ov_integral_vs =
                (self.ov_integral_vs + (sample.vbus_v - max_v) * sample.dt).min(integral_ceiling);
            if self.ov_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::OverVoltage)
                && let Some(f) = F::from_category(FaultCategory::OverVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("OverVoltage FAULT: vbus = {} V (integrated)", sample.vbus_v);
            }
        } else {
            self.ov_integral_vs *= decay;
        }
        if sample.vbus_v < min_v {
            self.uv_integral_vs =
                (self.uv_integral_vs + (min_v - sample.vbus_v) * sample.dt).min(integral_ceiling);
            if self.uv_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::UnderVoltage)
                && let Some(f) = F::from_category(FaultCategory::UnderVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!(
                    "UnderVoltage FAULT: vbus = {} V (integrated)",
                    sample.vbus_v
                );
            }
        } else {
            self.uv_integral_vs *= decay;
            if sample.vbus_v * 1000.0 > voltage.min_mv.saturating_add(VOLTAGE_HYSTERESIS_MV) as f32
                && registry.has_category(FaultCategory::UnderVoltage)
            {
                registry.clear(FaultCategory::UnderVoltage);
            }
        }

        // --- Temperatures (board thresholds; <= 0 = sensor not wired) ---
        let fet_c = sample.fet_temp_c_x10.map(|t| f32::from(t) / 10.0);
        let motor_c = sample.motor_temp_c_x10.map(|t| f32::from(t) / 10.0);
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
            let omega = sample.velocity;
            let temperature = self.config.thermal_scales(fet_c, motor_c);
            let bus_voltage = self.config.bus_voltage_scales(sample.vbus_v);
            self.environment_drive = temperature.drive.min(bus_voltage.drive);
            #[cfg(feature = "fast-telemetry")]
            {
                self.temperature_drive = temperature.drive;
                self.bus_voltage_drive = bus_voltage.drive;
            }
            self.scales = DeratingScales {
                drive: self.environment_drive,
                brake: temperature.brake,
                regen: bus_voltage.regen,
            };
            self.scales.drive = self.scales.drive.min(self.drive_cut(omega));
            let worst = self.scales.worst();
            if worst < 0.8 {
                if let Some(f) = F::from_category(FaultCategory::Derating) {
                    registry.set(f);
                }
            } else if worst > 0.95 && registry.has_category(FaultCategory::Derating) {
                registry.clear(FaultCategory::Derating);
            }
        }
    }
}

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Apply derating configuration (boot / live config write).
    pub fn set_derating(&mut self, cfg: DeratingConfig) {
        if self.protection.config.modulation != cfg.modulation {
            self.reset_duty_utilization();
        }
        self.protection.configure(cfg);
        self.velocity_schedule.clear_request();
    }

    /// Install the rider-facing configuration at boot or at the stopped apply boundary.
    pub fn set_drive_config(&mut self, cfg: DriveConfig) -> Result<(), super::MotorModelError> {
        if self.mode() != crate::types::ControlMode::Stopped || self.current_offset_active() {
            return Err(super::MotorModelError::Busy);
        }
        self.protection.drive = cfg;
        #[cfg(feature = "storage")]
        {
            self.protection.drive_config_valid = true;
        }
        self.reset_duty_utilization();
        Ok(())
    }

    #[cfg(feature = "storage")]
    pub fn load_drive_config(&mut self, config: &crate::storage::RuntimeConfig) {
        self.protection.drive_config_valid = config.drive_is_valid();
        self.protection.drive = if self.protection.drive_config_valid {
            config.drive.unwrap_or_default()
        } else {
            DriveConfig::DEFAULT
        };
        self.reset_duty_utilization();
    }

    #[cfg(all(feature = "runtime", feature = "storage"))]
    pub(crate) fn drive_config_valid(&self) -> bool {
        self.protection.drive_config_valid
    }

    #[cfg(feature = "runtime")]
    pub(crate) fn orient_drive_command(
        &self,
        mode: crate::types::ControlMode,
    ) -> crate::types::ControlMode {
        self.protection.drive.orient(mode)
    }

    pub(super) fn speed_ceiling(&self, physical_direction: f32) -> f32 {
        self.protection.speed_ceiling(physical_direction)
    }

    pub(super) fn reset_duty_utilization(&mut self) {
        self.protection.duty_utilization = 0.0;
    }

    pub(super) fn update_duty_utilization(&mut self, vd: f32, vq: f32, dt: f32) {
        if self.protection.config.modulation.duty_start_frac < 1.0 {
            let voltage_limit = self.controller.voltage_limit();
            if voltage_limit > 0.0 {
                let utilization = (sqrtf(vd * vd + vq * vq) / voltage_limit).min(1.0);
                let alpha = (dt / 0.002).min(1.0);
                self.protection.duty_utilization +=
                    alpha * (utilization - self.protection.duty_utilization);
            }
        }
    }

    pub(super) fn throttle_current(&self, fraction: f32) -> f32 {
        let omega = self.torque_speed();
        let limit = if fraction * omega >= 0.0 {
            self.current_limits.max_current_a
        } else {
            self.current_limits.max_brake_a
        };
        let (_, full_request) = self
            .current_limits
            .clamp_targets(0.0, limit.copysign(fraction));
        let (_, available) = self.grant_torque(0.0, full_request, omega);
        available * fraction.abs()
    }

    /// Current derating configuration.
    pub fn derating_cfg(&self) -> &DeratingConfig {
        self.protection.config()
    }

    /// Live derating scales (1.0/1.0 = no derate).
    pub fn derating(&self) -> DeratingScales {
        self.protection.scales()
    }

    /// Store freshly computed derating scales (protection update).
    pub fn set_derating_scales(&mut self, scales: DeratingScales) {
        self.protection.set_scales(scales);
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
        board: &PreparedBoardConfig,
        vbus_v: f32,
        fet_temp_c_x10: Option<i16>,
        motor_temp_c_x10: Option<i16>,
    ) {
        // Platforms initialize this at boot. Keep the storage-free runtime
        // usable too, without copying the constants on every ADC interrupt.
        if self.protection.board_voltage.is_none() {
            self.protection.board_voltage = Some(board.voltage);
        }
        self.protection.update(
            registry,
            board,
            ProtectionSample {
                dt: self.dt,
                vbus_v,
                velocity: self.torque_speed(),
                fet_temp_c_x10,
                motor_temp_c_x10,
            },
        );
    }

    /// Low-pass the final PWM q-axis modulation (the duty
    /// factor in `i_bus ≈ iq·mod_q`). τ ≈ 2 ms — slow enough to ignore
    /// per-cycle voltage ripple, fast enough to track real speed changes.
    pub(super) fn update_bus_mod_q(&mut self, mod_q: QAxisModulation, dt: f32) {
        const TAU_S: f32 = 0.002;
        let alpha = (dt / TAU_S).min(1.0);
        self.bus_mod_q_filt += alpha * (mod_q.coefficient() - self.bus_mod_q_filt);
    }

    /// The (id, iq) targets a torque request is granted this cycle. Every
    /// torque path funnels through here (remote, velocity loop, failsafe,
    /// the regulated parking brake), and the velocity loop takes its
    /// anti-windup bounds from it.
    ///
    /// - iq·ω ≥ 0 is motoring (standstill counts: pulling away is
    ///   acceleration), limited by `max_current_a` × the drive derate
    ///   (heat, sag, speed). The speed cut is re-evaluated FRESH each cycle
    ///   on top of the decimated scales — a 78 Hz-stale cut bang-bangs the
    ///   torque around the ceiling on a fast free rotor (see
    ///   [`crate::motor::derating::SpeedDerating::speed_cut`]).
    /// - Opposing is braking: `max_brake_a` × the brake derate (heat only,
    ///   never speed). The regen the supply refuses (the regen-OV derate,
    ///   `bus_regen_max_a`) is burned in the windings when dissipation is
    ///   on ([`dissipation`](crate::motor::dissipation)); with it off, the
    ///   regen derate scales the torque instead.
    /// - The bus-current clamp last — except for a brake the dissipation
    ///   already shaped: its bus estimate ignores the d current.
    pub(super) fn grant_torque(&self, id_target: f32, iq_target: f32, omega: f32) -> (f32, f32) {
        let limits = &self.current_limits;
        if !limits.is_enabled() {
            return (id_target, self.clamp_iq_for_bus(iq_target));
        }
        if iq_target * omega >= 0.0 {
            let lim = limits.max_current_a
                * self
                    .protection
                    .environment_drive
                    .min(self.protection.drive_cut(omega));
            return (
                id_target,
                self.clamp_iq_for_bus(clamp_f32(iq_target, -lim, lim)),
            );
        }
        let brake = self.derating().brake;
        let Some(model) = self.dissipation_model() else {
            let lim = limits.max_brake_a * brake.min(self.derating().regen);
            return (
                id_target,
                self.clamp_iq_for_bus(clamp_f32(iq_target, -lim, lim)),
            );
        };
        self.grant_dissipative_brake(id_target, iq_target, omega, brake, model, None)
    }

    #[cfg(feature = "fast-telemetry")]
    #[inline(never)]
    pub(super) fn current_limit_flags(
        &self,
        requested: f32,
        (clamped, before_grant): (f32, f32),
        granted: f32,
        omega: f32,
    ) -> u16 {
        use crate::velocity_telemetry as trace;
        if (requested - granted).abs() <= 1e-4 {
            return 0;
        }
        let mut flags = trace::GRANT_CLIPPED;
        if (requested - clamped).abs() > 1e-4 {
            flags |= trace::LIMIT_PHASE_CURRENT;
        }
        if (clamped - before_grant).abs() > 1e-4 {
            flags |= if !self.phase.angle_trustworthy() {
                trace::LIMIT_ANGLE
            } else {
                trace::STARTUP_CURRENT_SHAPING
            };
        }
        if (before_grant - granted).abs() <= 1e-4 {
            return flags;
        }
        let limits = &self.current_limits;
        let mut phase = before_grant;
        if limits.is_enabled() && before_grant * omega >= 0.0 {
            let p = &self.protection;
            let speed = p.config.speed.speed_cut(p.drive.rider_sign(omega));
            let duty = p.duty_cut();
            let scale = p.environment_drive.min(speed.min(duty));
            let ceiling = limits.max_current_a * scale;
            phase = clamp_f32(before_grant, -ceiling, ceiling);
            if (before_grant - phase).abs() > 1e-4 {
                for (value, reason) in [
                    (speed, trace::LIMIT_SPEED),
                    (duty, trace::LIMIT_MODULATION),
                    (p.temperature_drive, trace::LIMIT_TEMPERATURE),
                    (p.bus_voltage_drive, trace::LIMIT_BUS_VOLTAGE),
                ] {
                    if value < 1.0 && value <= scale {
                        flags |= reason;
                    }
                }
            }
        } else if limits.is_enabled() {
            let thermal = self.protection.scales.brake;
            let regen = self.protection.scales.regen;
            let ceiling = limits.max_brake_a * thermal.min(regen);
            if self.dissipation_model().is_some() {
                // The dissipative solver may change both axes. Do not infer
                // individual limits from its nonlinear grant.
                return flags;
            }
            phase = clamp_f32(before_grant, -ceiling, ceiling);
            if (before_grant - phase).abs() > 1e-4 {
                if thermal <= regen && thermal < 1.0 {
                    flags |= trace::LIMIT_TEMPERATURE;
                }
                if regen <= thermal && regen < 1.0 {
                    flags |= trace::LIMIT_BUS_VOLTAGE;
                }
                if thermal == 1.0 && regen == 1.0 {
                    flags |= trace::LIMIT_PHASE_CURRENT;
                }
            }
        }
        if (phase - granted).abs() > 1e-4 {
            flags |= trace::LIMIT_BUS_CURRENT;
        }
        flags
    }

    /// Prepare shared scalar bounds for the velocity PI's two authority probes
    /// and its actual current request. Ordinary current control keeps the
    /// direct grant path, without constructing this local value.
    pub(super) fn torque_constraints(&self, omega: f32) -> TorqueConstraints {
        let limits = &self.current_limits;
        let (drive_limit, plain_brake_limit) = if limits.is_enabled() {
            (
                limits.max_current_a
                    * self
                        .protection
                        .environment_drive
                        .min(self.protection.drive_cut(omega)),
                limits.max_brake_a * self.derating().brake.min(self.derating().regen),
            )
        } else {
            (f32::INFINITY, f32::INFINITY)
        };
        TorqueConstraints {
            bus: self.bus_iq_bounds().unwrap_or(BusIqBounds::IDENTITY),
            omega,
            drive_limit,
            plain_brake_limit,
        }
    }

    pub(super) fn grant_torque_constrained(
        &self,
        id_target: f32,
        iq_target: f32,
        constraints: &TorqueConstraints,
    ) -> (f32, f32) {
        if !self.current_limits.is_enabled() {
            return (id_target, constraints.bus.clamp(iq_target));
        }
        if iq_target * constraints.omega >= 0.0 {
            let lim = constraints.drive_limit;
            return (
                id_target,
                constraints.bus.clamp(clamp_f32(iq_target, -lim, lim)),
            );
        }
        let Some(model) = self.dissipation_model() else {
            let lim = constraints.plain_brake_limit;
            return (
                id_target,
                constraints.bus.clamp(clamp_f32(iq_target, -lim, lim)),
            );
        };
        self.grant_dissipative_brake(
            id_target,
            iq_target,
            constraints.omega,
            self.derating().brake,
            model,
            Some(&constraints.bus),
        )
    }

    // Keep the optional nonlinear solve out of the common torque grant: its
    // register requirements otherwise spill into motoring and plain braking.
    #[inline(never)]
    fn grant_dissipative_brake(
        &self,
        id_target: f32,
        iq_target: f32,
        omega: f32,
        brake: f32,
        model: WindingModel,
        bus: Option<&BusIqBounds>,
    ) -> (f32, f32) {
        let limits = &self.current_limits;
        let lim = limits.max_brake_a * brake;
        let (id_target, iq) = limits.clamp_targets(id_target, clamp_f32(iq_target, -lim, lim));
        let budget = RegenBudget {
            frac: self.derating().regen,
            max_w: if limits.bus_regen_max_a >= 0.0 {
                self.vbus * limits.bus_regen_max_a
            } else {
                f32::INFINITY
            },
        };
        // The heat budget buys id_max = √(W / 1.5·R), and follows the
        // thermal brake derate: a hot motor or board burns less (and so
        // brakes less on a full pack).
        let id_max = sqrtf(limits.max_dissipation_w * brake / (1.5 * model.r_ohm));
        match dissipate_with_id(
            id_target,
            iq,
            omega,
            &model,
            &budget,
            id_max,
            limits.max_current_a,
        ) {
            // The solver preserves an explicit d-axis direction and uses
            // aligning current when it introduces a new heat request.
            Some(grant) => grant,
            // Plugging: the brake draws from the supply, nothing to shape.
            None => (
                id_target,
                match bus {
                    Some(bounds) => bounds.clamp(iq),
                    None => self.clamp_iq_for_bus(iq),
                },
            ),
        }
    }

    /// The winding model dissipative braking runs on: `Some` when it is on
    /// (`max_dissipation_w > 0`) and the motor params carry R and λ.
    fn dissipation_model(&self) -> Option<WindingModel> {
        let d = self
            .controller
            .decoupling()
            .filter(|_| self.current_limits.max_dissipation_w > 0.0)?;
        (d.resistance_ohm > 0.0 && d.flux_linkage_wb > 0.0).then_some(WindingModel {
            r_ohm: d.resistance_ohm,
            flux_wb: d.flux_linkage_wb,
            saliency_h: (d.lq_h - d.ld_h).abs(),
        })
    }

    /// Clamp the iq target so the implied bus current stays inside
    /// `[-bus_regen_max_a, bus_in_max_a]` (VESC mcpwm_foc.c:3629-3637).
    ///
    /// `i_bus ≈ iq·mod_q` (electrical power over vbus, q-axis only — the
    /// d-axis share is losses/HFI carrier, small and consumption-side).
    /// With `bus_regen_max_a = 0` no energy is ever pushed into the supply:
    /// without dissipation, current braking degrades to ~zero torque in the
    /// regen zone (a lab PSU cannot absorb reverse current). The
    /// windings-short parking brake is unaffected — it never sees the bus.
    pub(super) fn clamp_iq_for_bus(&self, iq_target: f32) -> f32 {
        match self.bus_iq_bounds() {
            Some(bounds) => bounds.clamp(iq_target),
            None => iq_target,
        }
    }

    #[inline(always)]
    fn bus_iq_bounds(&self) -> Option<BusIqBounds> {
        let in_max = self.current_limits.bus_in_max_a;
        let regen_max = self.current_limits.bus_regen_max_a;
        if in_max < 0.0 && regen_max < 0.0 {
            return None; // both unlimited (the default)
        }
        let mod_q = self.bus_mod_q_filt;
        // Near-zero modulation: bus current is negligible whatever iq is,
        // and dividing by it would explode. Same dead-band as VESC.
        if mod_q.abs() < 1e-3 {
            return None;
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
        Some(BusIqBounds {
            lo: lo_iq,
            hi: hi_iq,
        })
    }
}

#[cfg(all(test, feature = "runtime"))]
mod tests {
    use super::*;
    use crate::foc::config::NtcConfig;
    use crate::foc::fault::StandardFault;

    fn board(max_vbus_mv: u32, min_vbus_mv: u32) -> BoardConfig {
        BoardConfig {
            calib: Default::default(),
            current_sense_topology: crate::types::CurrentSenseTopology::TwoShunt,
            max_iq_target_a: 10.0,
            max_phase_current_a: 40.0,
            max_vbus_mv,
            min_vbus_mv,
            max_fet_temp_c: 100.0,
            max_motor_temp_c: 120.0,
            phase_sense: None,
        }
    }

    /// Original voltage algorithm, deliberately deriving constants on every
    /// sample. Compare accumulated bits and fault edges, not just end states.
    #[derive(Default)]
    struct LegacyVoltage {
        ov: f32,
        uv: f32,
        over_voltage: bool,
        under_voltage: bool,
    }

    impl LegacyVoltage {
        fn update(&mut self, board: &BoardConfig, dt: f32, vbus_v: f32) {
            let max_v = board.max_vbus_mv as f32 / 1000.0;
            let min_v = board.min_vbus_mv as f32 / 1000.0;
            let trip_vs = 5e-5 * max_v;
            let integral_ceiling = trip_vs * 4.0;
            let decay = 1.0 - (dt / 0.005).min(1.0);
            if vbus_v > max_v {
                self.ov = (self.ov + (vbus_v - max_v) * dt).min(integral_ceiling);
                if self.ov > trip_vs {
                    self.over_voltage = true;
                }
            } else {
                self.ov *= decay;
            }
            if vbus_v < min_v {
                self.uv = (self.uv + (min_v - vbus_v) * dt).min(integral_ceiling);
                if self.uv > trip_vs {
                    self.under_voltage = true;
                }
            } else {
                self.uv *= decay;
                if vbus_v * 1000.0 > (board.min_vbus_mv + VOLTAGE_HYSTERESIS_MV) as f32 {
                    self.under_voltage = false;
                }
            }
        }
    }

    fn compare_sample(
        monitor: &mut ProtectionMonitor,
        registry: &FaultRegistry<StandardFault>,
        legacy: &mut LegacyVoltage,
        prepared: &PreparedBoardConfig,
        dt: f32,
        vbus_v: f32,
    ) {
        legacy.update(prepared.board(), dt, vbus_v);
        monitor.update(
            registry,
            prepared,
            ProtectionSample {
                dt,
                vbus_v,
                velocity: 0.0,
                fet_temp_c_x10: None,
                motor_temp_c_x10: None,
            },
        );
        assert_eq!(
            monitor.ov_integral_vs.to_bits(),
            legacy.ov.to_bits(),
            "OV dt={dt} v={vbus_v}"
        );
        assert_eq!(
            monitor.uv_integral_vs.to_bits(),
            legacy.uv.to_bits(),
            "UV dt={dt} v={vbus_v}"
        );
        assert_eq!(
            registry.has_category(FaultCategory::OverVoltage),
            legacy.over_voltage
        );
        assert_eq!(
            registry.has_category(FaultCategory::UnderVoltage),
            legacy.under_voltage
        );
    }

    #[test]
    fn prepared_voltage_matches_every_trip_cycle_decay_and_hysteresis_edge() {
        for (max_mv, min_mv) in [(57_000, 8_000), (60_001, 12_001), (1_000, 0)] {
            let prepared = PreparedBoardConfig::new(board(max_mv, min_mv));
            for dt in [0.000025, 0.00005, 0.0001, 0.001, 0.005, 0.01] {
                for over_voltage in [false, true] {
                    let mut monitor = ProtectionMonitor::new(dt);
                    let registry = FaultRegistry::new();
                    let mut legacy = LegacyVoltage::default();
                    let excursion = if over_voltage {
                        max_mv as f32 / 1000.0 + 1.25
                    } else {
                        min_mv as f32 / 1000.0 - 0.75
                    };
                    let category = if over_voltage {
                        FaultCategory::OverVoltage
                    } else {
                        FaultCategory::UnderVoltage
                    };
                    let mut actual_trip = None;
                    let mut reference_trip = None;
                    for cycle in 1..=1_000 {
                        compare_sample(
                            &mut monitor,
                            &registry,
                            &mut legacy,
                            &prepared,
                            dt,
                            excursion,
                        );
                        if registry.has_category(category) {
                            actual_trip.get_or_insert(cycle);
                        }
                        if if over_voltage {
                            legacy.over_voltage
                        } else {
                            legacy.under_voltage
                        } {
                            reference_trip.get_or_insert(cycle);
                        }
                    }
                    assert_eq!(actual_trip, reference_trip);
                    assert!(actual_trip.is_some(), "the excursion must cross the budget");

                    let clear_v = (min_mv + VOLTAGE_HYSTERESIS_MV) as f32 / 1000.0;
                    let recovery = [
                        min_mv as f32 / 1000.0,
                        f32::from_bits(clear_v.to_bits() - 1),
                        clear_v,
                        f32::from_bits(clear_v.to_bits() + 1),
                        f32::NAN,
                        (max_mv as f32 + min_mv as f32) / 2000.0,
                    ];
                    for vbus in recovery.into_iter().cycle().take(300) {
                        compare_sample(&mut monitor, &registry, &mut legacy, &prepared, dt, vbus);
                    }
                    if !over_voltage {
                        assert!(!registry.has_category(FaultCategory::UnderVoltage));
                    }
                }
            }
        }
    }

    #[test]
    fn replacing_prepared_board_preserves_integrator_history_and_uses_new_thresholds() {
        let dt = 0.00005;
        let mut source = board(60_000, 8_000);
        let first = PreparedBoardConfig::new(source);
        let mut monitor = ProtectionMonitor::new(dt);
        let registry = FaultRegistry::new();
        let mut legacy = LegacyVoltage::default();
        for _ in 0..100 {
            compare_sample(&mut monitor, &registry, &mut legacy, &first, dt, 70.0);
        }
        let accumulated = monitor.ov_integral_vs;
        source.max_vbus_mv = 20_001;
        source.min_vbus_mv = 16_001;
        let replacement = PreparedBoardConfig::new(source);
        assert_eq!(first.board().max_vbus_mv, 60_000, "the source is owned");
        registry.clear(FaultCategory::OverVoltage);
        legacy.over_voltage = false;
        compare_sample(&mut monitor, &registry, &mut legacy, &replacement, dt, 17.0);
        assert_eq!(
            monitor.ov_integral_vs.to_bits(),
            (accumulated * (1.0 - dt / 0.005)).to_bits()
        );
        assert!(
            monitor.ov_integral_vs > replacement.voltage.integral_ceiling,
            "healthy samples decay history instead of resetting or clamping it"
        );
        compare_sample(&mut monitor, &registry, &mut legacy, &replacement, dt, 21.0);
        assert!(registry.has_category(FaultCategory::OverVoltage));
        for vbus in [0.0, 16.0, 16.5, 16.501, 17.0, f32::NAN] {
            compare_sample(&mut monitor, &registry, &mut legacy, &replacement, dt, vbus);
        }
    }

    #[test]
    fn prepared_constants_keep_degenerate_board_and_timestep_arithmetic() {
        for (max_mv, min_mv) in [(0, 0), (8_000, 12_000), (u32::MAX, 0)] {
            let prepared = PreparedBoardConfig::new(board(max_mv, min_mv));
            for dt in [0.0, -0.0, -0.001, f32::NAN, f32::INFINITY] {
                let mut monitor = ProtectionMonitor::new(dt);
                let registry = FaultRegistry::new();
                let mut legacy = LegacyVoltage::default();
                for vbus in [24.0, 0.0, -1.0, f32::INFINITY, f32::NEG_INFINITY, f32::NAN] {
                    compare_sample(&mut monitor, &registry, &mut legacy, &prepared, dt, vbus);
                }
            }
        }
        // The old UV hysteresis sum overflows only when its healthy branch
        // executes. Preparation must not eagerly add it for malformed boards.
        let prepared = PreparedBoardConfig::new(board(60_000, u32::MAX));
        let mut monitor = ProtectionMonitor::new(0.00005);
        let registry = FaultRegistry::new();
        compare_sample(
            &mut monitor,
            &registry,
            &mut LegacyVoltage::default(),
            &prepared,
            0.00005,
            0.0,
        );
        assert!(registry.has_category(FaultCategory::UnderVoltage));
    }

    #[test]
    fn prepared_board_keeps_temperature_sentinels_thresholds_and_derating_cadence() {
        let readings = [
            None,
            Some(-300),
            Some(1000),
            Some(1001),
            Some(1201),
            Some(NtcConfig::INVALID_TEMP_C_X10),
        ];
        for (fet_limit, motor_limit) in [(100.0, 120.0), (0.0, -1.0), (f32::NAN, f32::INFINITY)] {
            let mut source = board(60_000, 8_000);
            source.max_fet_temp_c = fet_limit;
            source.max_motor_temp_c = motor_limit;
            let prepared = PreparedBoardConfig::new(source);
            for fet in readings {
                for motor in readings {
                    let mut monitor = ProtectionMonitor::new(0.00005);
                    let registry = FaultRegistry::<StandardFault>::new();
                    for cycle in 1..=256 {
                        monitor.update(
                            &registry,
                            &prepared,
                            ProtectionSample {
                                dt: 0.00005,
                                vbus_v: 24.0,
                                velocity: 0.0,
                                fet_temp_c_x10: fet,
                                motor_temp_c_x10: motor,
                            },
                        );
                        if cycle < 256 {
                            assert_eq!(monitor.scales, DeratingScales::IDENTITY);
                        }
                    }
                    let fet_c = fet.map(|t| f32::from(t) / 10.0);
                    let motor_c = motor.map(|t| f32::from(t) / 10.0);
                    let expected_hot = fet_c.is_some_and(|t| fet_limit > 0.0 && t > fet_limit)
                        || motor_c.is_some_and(|t| motor_limit > 0.0 && t > motor_limit);
                    assert_eq!(registry.has_category(FaultCategory::OverTemp), expected_hot);
                    assert_eq!(
                        monitor.scales,
                        DeratingConfig::default().compute(fet_c, motor_c, 24.0)
                    );
                }
            }
        }
    }
}
