//! Phase-current limits, the bus clamp, derating, and the shared protection update.

use super::FocDriver;
use crate::foc::clamp_f32;
#[cfg(any(feature = "storage", feature = "runtime"))]
use crate::foc::config::BoardConfig;
use crate::foc::fast_math::sqrtf;
#[cfg(feature = "runtime")]
use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault, VOLTAGE_HYSTERESIS_MV};
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::{PhasePwm, QAxisModulation};
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
#[cfg(feature = "isr-profiling")]
use crate::isr_prof;
#[cfg(all(feature = "isr-work-events", feature = "runtime"))]
use crate::isr_work;
use crate::motor::derating::{DeratingConfig, DeratingScales};
#[cfg(feature = "iq-authority")]
use crate::motor::dissipation;
use crate::motor::dissipation::{RegenBudget, WindingModel, dissipate_with_id, heat_current_limit};
use crate::motor::drive::DriveConfig;
#[cfg(feature = "storage")]
use crate::storage::{RuntimeConfig, VoltageLimitsConfig};
use crate::units::{
    ConstZero, ElectricCurrent, ElectricPotential, ElectricalAngularVelocity, Fraction, Frequency,
    Linear, MagneticFlux, Power, Scalar, Sign, Time, per_second, seconds,
};
#[cfg(any(feature = "storage", feature = "runtime"))]
use crate::units::{quantity, volts};

/// `V·s`: the integrated bus-voltage excursion of the OV/UV protection.
type VoltSeconds = MagneticFlux;

struct BusIqBounds {
    lo: ElectricCurrent,
    hi: ElectricCurrent,
}

impl BusIqBounds {
    const IDENTITY: Self = Self {
        lo: ElectricCurrent::NEG_INFINITY,
        hi: ElectricCurrent::INFINITY,
    };

    #[inline(always)]
    fn clamp(&self, iq: ElectricCurrent) -> ElectricCurrent {
        iq.clamp_to(self.lo, self.hi)
    }
}

/// One cycle's velocity-loop authority and final torque-grant constraints.
/// Construct after selecting physical torque speed and reuse only within that
/// cycle, before protection/model/modulation state changes. The nonlinear
/// dissipative brake remains request-dependent and is solved for each request.
pub(super) struct TorqueConstraints {
    bus: BusIqBounds,
    omega: ElectricalAngularVelocity,
    drive_limit: ElectricCurrent,
    plain_brake_limit: ElectricCurrent,
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
// Host storage tools prepare these values without running the protection loop.
#[cfg_attr(not(feature = "runtime"), allow(dead_code))]
pub struct PreparedVoltageLimits {
    min_mv: u32,
    #[cfg(feature = "storage")]
    max_mv: u32,
    max_vbus: ElectricPotential,
    min_vbus: ElectricPotential,
    trip: VoltSeconds,
    integral_ceiling: VoltSeconds,
}

#[cfg(feature = "storage")]
impl<P: PhasePwm, C: CurrentSensor, Ph: PhaseProvider, S: SinCos> FocDriver<P, C, Ph, S> {
    /// Boot and live writes share the same validation and installation path.
    /// Invalid stored thresholds abort initialization before the bridge starts.
    pub fn configure_voltage_protection(
        &mut self,
        board: &BoardConfig,
        stored: Option<&VoltageLimitsConfig>,
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

    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    pub(crate) fn reset_voltage_limits(&mut self) {
        self.protection.voltage = None;
    }
}

impl PreparedVoltageLimits {
    #[cfg(any(feature = "storage", feature = "runtime"))]
    const fn from_bounds(min_mv: u32, max_mv: u32) -> Self {
        // Plain f32 arithmetic: quantity operators are not `const`.
        let max_vbus = max_mv as f32 / 1000.0;
        let min_vbus = min_mv as f32 / 1000.0;
        let trip = 5e-5 * max_vbus;
        let integral_ceiling_vs = trip * 4.0;
        Self {
            min_mv,
            #[cfg(feature = "storage")]
            max_mv,
            max_vbus: volts(max_vbus),
            min_vbus: volts(min_vbus),
            trip: quantity(trip),
            integral_ceiling: quantity(integral_ceiling_vs),
        }
    }

    #[cfg(feature = "storage")]
    pub fn from_config(config: &VoltageLimitsConfig) -> Option<Self> {
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
    #[cfg(feature = "service-slots")]
    pub(super) service_slots: super::velocity_schedule::ServiceSlots,
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
    ov_integral: VoltSeconds,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    uv_integral: VoltSeconds,
    #[cfg_attr(not(feature = "runtime"), allow(dead_code))]
    voltage_decay: f32,
}

#[cfg(feature = "runtime")]
pub(super) struct ProtectionSample {
    pub dt: Time,
    pub vbus: ElectricPotential,
    pub velocity: ElectricalAngularVelocity,
    pub fet_temp_c_x10: Option<i16>,
    pub motor_temp_c_x10: Option<i16>,
}

impl ProtectionMonitor {
    pub(super) fn overvoltage_threshold(&self) -> ElectricPotential {
        self.voltage
            .or(self.board_voltage)
            .map_or(ElectricPotential::ZERO, |v| v.max_vbus)
    }

    pub(super) fn new(dt: Time) -> Self {
        Self {
            #[cfg(feature = "service-slots")]
            service_slots: super::velocity_schedule::ServiceSlots::new(dt),
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
            ov_integral: VoltSeconds::ZERO,
            uv_integral: VoltSeconds::ZERO,
            voltage_decay: voltage_decay(dt),
        }
    }

    pub(super) fn configure(&mut self, config: DeratingConfig) {
        self.config = config;
    }

    pub(super) fn set_control_period(&mut self, dt: Time) {
        #[cfg(feature = "service-slots")]
        {
            self.service_slots = super::velocity_schedule::ServiceSlots::new(dt);
        }
        self.voltage_decay = voltage_decay(dt);
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

    fn speed_ceiling(
        &self,
        physical_direction: ElectricalAngularVelocity,
    ) -> ElectricalAngularVelocity {
        self.config
            .speed
            .speed_ceiling(self.drive.rider_sign(physical_direction))
    }

    fn drive_cut(&self, omega: ElectricalAngularVelocity) -> f32 {
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
        // Leaky-bucket trip: integrate the excursion, fire past `trip`,
        // bleed it off in-range with τ≈5 ms. The integral is capped at a few
        // × `trip` so a long excursion can't bank unbounded "credit" and
        // make a later marginal spike trip instantly; the cap sits above the
        // trip point, so first-trip latency is unchanged.
        //
        // A non-finite `vbus_v` leaves both comparisons false → neither fault
        // trips and the integrals just decay. Acceptable here: vbus is a
        // linear ADC divider with no division, so NaN is not physically
        // reachable — mirrors the host-boundary "reject NaN" philosophy (the
        // ISR trusts its own sense path).
        let voltage = self.voltage.unwrap_or(prepared.voltage);
        let max_v = voltage.max_vbus;
        let min_v = voltage.min_vbus;
        let trip = voltage.trip;
        let integral_ceiling = voltage.integral_ceiling;
        let decay = self.voltage_decay;
        if sample.vbus > max_v {
            self.ov_integral =
                (self.ov_integral + (sample.vbus - max_v) * sample.dt).min(integral_ceiling);
            if self.ov_integral > trip
                && !registry.has_category(FaultCategory::OverVoltage)
                && let Some(f) = F::from_category(FaultCategory::OverVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("OverVoltage FAULT: vbus = {} V (integrated)", sample.vbus);
            }
        } else {
            self.ov_integral *= decay;
        }
        if sample.vbus < min_v {
            self.uv_integral =
                (self.uv_integral + (min_v - sample.vbus) * sample.dt).min(integral_ceiling);
            if self.uv_integral > trip
                && !registry.has_category(FaultCategory::UnderVoltage)
                && let Some(f) = F::from_category(FaultCategory::UnderVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("UnderVoltage FAULT: vbus = {} V (integrated)", sample.vbus);
            }
        } else {
            self.uv_integral *= decay;
            if sample.vbus * 1000.0
                > volts(voltage.min_mv.saturating_add(VOLTAGE_HYSTERESIS_MV) as f32)
                && registry.has_category(FaultCategory::UnderVoltage)
            {
                registry.clear(FaultCategory::UnderVoltage);
            }
        }

        // --- Temperatures (board thresholds; <= 0 = sensor not wired) ---
        let fet_c = sample.fet_temp_c_x10.map(temperature_c);
        let motor_c = sample.motor_temp_c_x10.map(temperature_c);
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
        #[cfg(feature = "service-slots")]
        let refresh_derating = self.service_slots.derating_due();
        #[cfg(not(feature = "service-slots"))]
        let refresh_derating = self.protection_tick.is_multiple_of(256);
        if refresh_derating {
            #[cfg(feature = "isr-work-events")]
            isr_work::mark(isr_work::DERATING);
            let omega = sample.velocity;
            let temperature = self.config.thermal_scales(fet_c, motor_c);
            let bus_voltage = self.config.bus_voltage_scales(sample.vbus);
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
    /// Poll command expiry once per service frame, approximately 1 kHz.
    /// The legacy schedule uses 16 native protection updates per poll.
    /// Detection takes up to one polling interval while the ISR runs.
    /// Link loss, command stamps and Stop stay native-rate.
    #[cfg(feature = "runtime")]
    #[inline]
    pub fn deadman_check_due(&self) -> bool {
        #[cfg(feature = "service-slots")]
        {
            self.protection.service_slots.deadman_due()
        }
        #[cfg(not(feature = "service-slots"))]
        {
            self.protection.protection_tick & 15 == 0
        }
    }

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
    pub fn load_drive_config(&mut self, config: &RuntimeConfig) {
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

    pub(super) fn speed_ceiling(
        &self,
        physical_direction: ElectricalAngularVelocity,
    ) -> ElectricalAngularVelocity {
        self.protection.speed_ceiling(physical_direction)
    }

    pub(super) fn reset_duty_utilization(&mut self) {
        self.protection.duty_utilization = 0.0;
    }

    // Per-cycle on the ISR path; keep it inlined as the f32 version was.
    #[inline(always)]
    pub(super) fn update_duty_utilization(
        &mut self,
        vd: ElectricPotential,
        vq: ElectricPotential,
        dt: Time,
    ) {
        if self.protection.config.modulation.duty_start_frac < 1.0 {
            let voltage_limit = self.controller.voltage_limit();
            if voltage_limit.is_positive() {
                let utilization = (vd * vd + vq * vq)
                    .sqrt_with(sqrtf)
                    .ratio_to(voltage_limit)
                    .min(1.0);
                let alpha = dt.ratio_to(DUTY_UTILIZATION_TAU).min(1.0);
                self.protection.duty_utilization +=
                    alpha * (utilization - self.protection.duty_utilization);
            }
        }
    }

    pub(super) fn throttle_current(&self, fraction: f32) -> ElectricCurrent {
        let omega = self.torque_speed();
        let limit = if (fraction * omega).is_non_negative() {
            self.current_limits.max_current
        } else {
            self.current_limits.max_brake
        };
        let (_, full_request) = self
            .current_limits
            .clamp_targets(ElectricCurrent::ZERO, limit.copysign(fraction));
        let (_, available) = self.grant_torque(ElectricCurrent::ZERO, full_request, omega);
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
        #[cfg(feature = "pi-bounds-ahead")]
        self.velocity_schedule.drop_authority();
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
        vbus: ElectricPotential,
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
                vbus,
                velocity: self.torque_speed(),
                fet_temp_c_x10,
                motor_temp_c_x10,
            },
        );
    }

    /// Low-pass the final PWM q-axis modulation (the duty
    /// factor in `i_bus ≈ iq·mod_q`). τ ≈ 2 ms — slow enough to ignore
    /// per-cycle voltage ripple, fast enough to track real speed changes.
    pub(super) fn update_bus_mod_q(&mut self, mod_q: QAxisModulation, dt: Time) {
        // `dt · (1/τ)`: a constant reciprocal, no divide per cycle.
        const INV_TAU: Frequency = per_second(1.0 / 0.002);
        let alpha = (dt * INV_TAU).fraction().min(1.0);
        self.bus_mod_q_filt += alpha * (mod_q.coefficient() - self.bus_mod_q_filt);
    }

    /// The (id, iq) targets a torque request is granted this cycle. Every
    /// torque path funnels through here (remote, velocity loop, failsafe,
    /// the regulated parking brake), and the velocity loop takes its
    /// anti-windup bounds from it.
    ///
    /// - iq·ω ≥ 0 is motoring (standstill counts: pulling away is
    ///   acceleration), limited by `max_current` × the drive derate
    ///   (heat, sag, speed). The speed cut is re-evaluated FRESH each cycle
    ///   on top of the decimated scales — a 78 Hz-stale cut bang-bangs the
    ///   torque around the ceiling on a fast free rotor (see
    ///   [`crate::motor::derating::SpeedDerating::speed_cut`]).
    /// - Opposing is braking: `max_brake` × the brake derate (heat only,
    ///   never speed). The regen the supply refuses (the regen-OV derate,
    ///   `bus_regen_max`) is burned in the windings when dissipation is
    ///   on ([`dissipation`](crate::motor::dissipation)); with it off, the
    ///   regen derate scales the torque instead.
    /// - The bus-current clamp last — except for a brake the dissipation
    ///   already shaped: its bus estimate ignores the d current.
    #[cfg_attr(
        all(
            feature = "compact-foc",
            target_arch = "arm",
            target_os = "none",
            not(feature = "ram-foc")
        ),
        unsafe(link_section = ".text.foc_grant")
    )]
    pub(super) fn grant_torque(
        &self,
        id_target: ElectricCurrent,
        iq_target: ElectricCurrent,
        omega: ElectricalAngularVelocity,
    ) -> (ElectricCurrent, ElectricCurrent) {
        let limits = &self.current_limits;
        if !limits.is_enabled() {
            return (id_target, self.clamp_iq_for_bus(iq_target));
        }
        if (iq_target * omega).is_non_negative() {
            let lim = limits.max_current
                * self
                    .protection
                    .environment_drive
                    .min(self.protection.drive_cut(omega));
            return (
                id_target,
                self.clamp_iq_for_bus(iq_target.clamp_to(-lim, lim)),
            );
        }
        let brake = self.derating().brake;
        let Some(model) = self.dissipation_model() else {
            let lim = limits.max_brake * brake.min(self.derating().regen);
            return (
                id_target,
                self.clamp_iq_for_bus(iq_target.clamp_to(-lim, lim)),
            );
        };
        self.grant_dissipative_brake(id_target, iq_target, omega, brake, model, None)
    }

    #[cfg(feature = "fast-telemetry")]
    #[inline(never)]
    pub(super) fn current_limit_flags(
        &self,
        requested: ElectricCurrent,
        (clamped, before_grant): (ElectricCurrent, ElectricCurrent),
        granted: ElectricCurrent,
        omega: ElectricalAngularVelocity,
    ) -> u16 {
        use crate::units::amps;
        use crate::velocity_telemetry as trace;
        const EPS: ElectricCurrent = amps(1e-4);
        if (requested - granted).abs() <= EPS {
            return 0;
        }
        let mut flags = trace::GRANT_CLIPPED;
        if (requested - clamped).abs() > EPS {
            flags |= trace::LIMIT_PHASE_CURRENT;
        }
        if (clamped - before_grant).abs() > EPS {
            flags |= if !self.phase.angle_trustworthy() {
                trace::LIMIT_ANGLE
            } else {
                trace::STARTUP_CURRENT_SHAPING
            };
        }
        if (before_grant - granted).abs() <= EPS {
            return flags;
        }
        let limits = &self.current_limits;
        let mut phase = before_grant;
        if limits.is_enabled() && (before_grant * omega).is_non_negative() {
            let p = &self.protection;
            let speed = p.config.speed.speed_cut(p.drive.rider_sign(omega));
            let duty = p.duty_cut();
            let scale = p.environment_drive.min(speed.min(duty));
            let ceiling = limits.max_current * scale;
            phase = before_grant.clamp_to(-ceiling, ceiling);
            if (before_grant - phase).abs() > EPS {
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
            let ceiling = limits.max_brake * thermal.min(regen);
            if self.dissipation_model().is_some() {
                // The dissipative solver may change both axes. Do not infer
                // individual limits from its nonlinear grant.
                return flags;
            }
            phase = before_grant.clamp_to(-ceiling, ceiling);
            if (before_grant - phase).abs() > EPS {
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
        if (phase - granted).abs() > EPS {
            flags |= trace::LIMIT_BUS_CURRENT;
        }
        flags
    }

    /// Prepare shared scalar bounds for the velocity PI's two authority probes
    /// and its actual current request. Ordinary current control keeps the
    /// direct grant path, without constructing this local value.
    pub(super) fn torque_constraints(&self, omega: ElectricalAngularVelocity) -> TorqueConstraints {
        let limits = &self.current_limits;
        let (drive_limit, plain_brake_limit) = if limits.is_enabled() {
            (
                limits.max_current
                    * self
                        .protection
                        .environment_drive
                        .min(self.protection.drive_cut(omega)),
                limits.max_brake * self.derating().brake.min(self.derating().regen),
            )
        } else {
            (ElectricCurrent::INFINITY, ElectricCurrent::INFINITY)
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
        id_target: ElectricCurrent,
        iq_target: ElectricCurrent,
        constraints: &TorqueConstraints,
    ) -> (ElectricCurrent, ElectricCurrent) {
        if !self.current_limits.is_enabled() {
            return (id_target, constraints.bus.clamp(iq_target));
        }
        if (iq_target * constraints.omega).is_non_negative() {
            let lim = constraints.drive_limit;
            return (
                id_target,
                constraints.bus.clamp(iq_target.clamp_to(-lim, lim)),
            );
        }
        let Some(model) = self.dissipation_model() else {
            let lim = constraints.plain_brake_limit;
            return (
                id_target,
                constraints.bus.clamp(iq_target.clamp_to(-lim, lim)),
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

    #[cfg(feature = "iq-authority")]
    pub(super) fn grant_iq_authority(
        &self,
        iq_target: ElectricCurrent,
        constraints: &TorqueConstraints,
    ) -> ElectricCurrent {
        let limits = &self.current_limits;
        if !limits.is_enabled() || (iq_target * constraints.omega).is_non_negative() {
            return self
                .grant_torque_constrained(ElectricCurrent::ZERO, iq_target, constraints)
                .1;
        }
        let Some(model) = self.dissipation_model() else {
            return self
                .grant_torque_constrained(ElectricCurrent::ZERO, iq_target, constraints)
                .1;
        };
        let brake = self.derating().brake;
        let (_, iq) = limits.clamp_targets(
            ElectricCurrent::ZERO,
            iq_target.clamp_to(-limits.max_brake * brake, limits.max_brake * brake),
        );
        let budget = self.regen_budget();
        let id_max = heat_current_limit(limits.max_dissipation * brake, model.resistance);
        dissipation::dissipate_iq(
            iq,
            constraints.omega,
            &model,
            &budget,
            id_max,
            limits.max_current,
        )
        .unwrap_or_else(|| constraints.bus.clamp(iq))
    }

    // Keep the optional nonlinear solve out of the common torque grant: its
    // register requirements otherwise spill into motoring and plain braking.
    #[cfg_attr(feature = "bench-dissipative-inline", inline(always))]
    #[cfg_attr(not(feature = "bench-dissipative-inline"), inline(never))]
    fn grant_dissipative_brake(
        &self,
        id_target: ElectricCurrent,
        iq_target: ElectricCurrent,
        omega: ElectricalAngularVelocity,
        brake: f32,
        model: WindingModel,
        bus: Option<&BusIqBounds>,
    ) -> (ElectricCurrent, ElectricCurrent) {
        let limits = &self.current_limits;
        let lim = limits.max_brake * brake;
        let (id_target, iq) = limits.clamp_targets(id_target, iq_target.clamp_to(-lim, lim));
        let budget = self.regen_budget();
        // The heat budget buys id_max = √(W / 1.5·R), and follows the
        // thermal brake derate: a hot motor or board burns less (and so
        // brakes less on a full pack).
        let id_max = heat_current_limit(limits.max_dissipation * brake, model.resistance);
        #[cfg(feature = "isr-profiling")]
        let solve_start = isr_prof::now();
        let result = dissipate_with_id(
            id_target,
            iq,
            omega,
            &model,
            &budget,
            id_max,
            limits.max_current,
        );
        #[cfg(feature = "isr-profiling")]
        isr_prof::add(&isr_prof::DISSIPATIVE_SOLVE, solve_start, isr_prof::now());
        match result {
            // The solver preserves an explicit d-axis direction and uses
            // aligning current when it introduces a new heat request.
            Some((id, iq)) => (id, iq),
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
    /// (`max_dissipation > 0`) and the motor params carry R and λ.
    fn dissipation_model(&self) -> Option<WindingModel> {
        let d = self
            .controller
            .decoupling()
            .filter(|_| self.current_limits.max_dissipation.is_positive())?;
        (d.resistance.is_positive() && d.flux_linkage.is_positive()).then_some(WindingModel {
            resistance: d.resistance,
            flux_linkage: d.flux_linkage,
            saliency: (d.lq - d.ld).abs(),
        })
    }

    /// What the supply accepts this cycle: the regen derate share and the
    /// bus regen-current limit as a charging-power ceiling.
    fn regen_budget(&self) -> RegenBudget {
        let limits = &self.current_limits;
        RegenBudget {
            frac: self.derating().regen,
            max_power: if limits.bus_regen_max.is_non_negative() {
                self.vbus * limits.bus_regen_max
            } else {
                Power::INFINITY
            },
        }
    }

    /// Clamp the iq target so the implied bus current stays inside
    /// `[-bus_regen_max, bus_in_max]` (VESC mcpwm_foc.c:3629-3637).
    ///
    /// `i_bus ≈ iq·mod_q` (electrical power over vbus, q-axis only — the
    /// d-axis share is losses/HFI carrier, small and consumption-side).
    /// With `bus_regen_max = 0` no energy is ever pushed into the supply:
    /// without dissipation, current braking degrades to ~zero torque in the
    /// regen zone (a lab PSU cannot absorb reverse current). The
    /// windings-short parking brake is unaffected — it never sees the bus.
    pub(super) fn clamp_iq_for_bus(&self, iq_target: ElectricCurrent) -> ElectricCurrent {
        match self.bus_iq_bounds() {
            Some(bounds) => bounds.clamp(iq_target),
            None => iq_target,
        }
    }

    /// Sign of the filtered q modulation outside the bus-bound dead band.
    /// The bus bounds map to the opposite iq direction when it changes.
    #[cfg(feature = "pi-bounds-ahead")]
    pub(super) fn bus_modulation_class(&self) -> i8 {
        let mod_q = self.bus_mod_q_filt;
        if mod_q.abs() < 1e-3 {
            0
        } else if mod_q > 0.0 {
            1
        } else {
            -1
        }
    }

    #[inline(always)]
    fn bus_iq_bounds(&self) -> Option<BusIqBounds> {
        let in_max = self.current_limits.bus_in_max;
        let regen_max = self.current_limits.bus_regen_max;
        if in_max.is_negative() && regen_max.is_negative() {
            return None; // both unlimited (the default)
        }
        let mod_q = self.bus_mod_q_filt;
        // Near-zero modulation: bus current is negligible whatever iq is,
        // and dividing by it would explode. Same dead-band as VESC.
        if mod_q.abs() < 1e-3 {
            return None;
        }
        let hi_bus = if in_max.is_non_negative() {
            in_max
        } else {
            ElectricCurrent::INFINITY
        };
        let lo_bus = if regen_max.is_non_negative() {
            -regen_max
        } else {
            ElectricCurrent::NEG_INFINITY
        };
        let inv_mod_q = 1.0 / mod_q;
        let (lo_iq, hi_iq) = if mod_q > 0.0 {
            (lo_bus * inv_mod_q, hi_bus * inv_mod_q)
        } else {
            (hi_bus * inv_mod_q, lo_bus * inv_mod_q)
        };
        Some(BusIqBounds {
            lo: lo_iq,
            hi: hi_iq,
        })
    }
}

/// Duty utilization low-pass time constant.
const DUTY_UTILIZATION_TAU: Time = seconds(0.002);

/// Per-cycle decay of the OV/UV integrals: a 5 ms time constant.
fn voltage_decay(dt: Time) -> f32 {
    1.0 - dt.ratio_to(seconds(0.005)).min(1.0)
}

#[cfg(feature = "runtime")]
#[inline]
fn temperature_c(tenths: i16) -> f32 {
    f32::from(tenths) * 0.1
}

#[cfg(all(test, feature = "runtime"))]
mod tests {
    use super::*;
    use crate::foc::config::NtcConfig;
    use crate::foc::fault::StandardFault;
    use crate::types::CurrentSenseTopology;
    use crate::units::{ToBits, erad_per_s};

    #[test]
    fn temperature_multiply_error_is_below_sensor_quantization() {
        for raw in i16::MIN..=i16::MAX {
            let actual = temperature_c(raw);
            let exact = f64::from(raw) / 10.0;
            // Full i16 domain, including temperatures far outside the board
            // range: <0.00025 C versus the input's 0.1 C quantum.
            assert!((f64::from(actual) - exact).abs() < 0.00025);
        }
        for threshold in [0.1, 85.0, 85.1, 100.0, 100.1, 120.0, 120.3] {
            let old_trip = (0..=i16::MAX)
                .find(|t| f32::from(*t) / 10.0 > threshold)
                .unwrap();
            let new_trip = (0..=i16::MAX)
                .find(|t| temperature_c(*t) > threshold)
                .unwrap();
            // At an exact threshold, rounding can trip one input quantum
            // earlier. It must never postpone the original protection.
            assert!((0..=1).contains(&(old_trip - new_trip)));
        }
    }

    fn board(max_vbus_mv: u32, min_vbus_mv: u32) -> BoardConfig {
        BoardConfig {
            calib: Default::default(),
            current_sense_topology: CurrentSenseTopology::TwoShunt,
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
            let trip = 5e-5 * max_v;
            let integral_ceiling = trip * 4.0;
            let decay = 1.0 - (dt / 0.005).min(1.0);
            if vbus_v > max_v {
                self.ov = (self.ov + (vbus_v - max_v) * dt).min(integral_ceiling);
                if self.ov > trip {
                    self.over_voltage = true;
                }
            } else {
                self.ov *= decay;
            }
            if vbus_v < min_v {
                self.uv = (self.uv + (min_v - vbus_v) * dt).min(integral_ceiling);
                if self.uv > trip {
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
                dt: seconds(dt),
                vbus: volts(vbus_v),
                velocity: erad_per_s(0.0),
                fet_temp_c_x10: None,
                motor_temp_c_x10: None,
            },
        );
        assert_eq!(
            monitor.ov_integral.to_bits(),
            legacy.ov.to_bits(),
            "OV dt={dt} v={vbus_v}"
        );
        assert_eq!(
            monitor.uv_integral.to_bits(),
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
                    let mut monitor = ProtectionMonitor::new(seconds(dt));
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
        let mut monitor = ProtectionMonitor::new(seconds(dt));
        let registry = FaultRegistry::new();
        let mut legacy = LegacyVoltage::default();
        for _ in 0..100 {
            compare_sample(&mut monitor, &registry, &mut legacy, &first, dt, 70.0);
        }
        let accumulated = monitor.ov_integral;
        source.max_vbus_mv = 20_001;
        source.min_vbus_mv = 16_001;
        let replacement = PreparedBoardConfig::new(source);
        assert_eq!(first.board().max_vbus_mv, 60_000, "the source is owned");
        registry.clear(FaultCategory::OverVoltage);
        legacy.over_voltage = false;
        compare_sample(&mut monitor, &registry, &mut legacy, &replacement, dt, 17.0);
        assert_eq!(
            monitor.ov_integral.to_bits(),
            (accumulated * (1.0 - dt / 0.005)).to_bits()
        );
        assert!(
            monitor.ov_integral > replacement.voltage.integral_ceiling,
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
                let mut monitor = ProtectionMonitor::new(seconds(dt));
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
        let mut monitor = ProtectionMonitor::new(seconds(0.00005));
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
                    let mut monitor = ProtectionMonitor::new(seconds(0.00005));
                    let registry = FaultRegistry::<StandardFault>::new();
                    #[cfg(feature = "service-slots")]
                    let first_refresh = 11;
                    #[cfg(not(feature = "service-slots"))]
                    let first_refresh = 256;
                    for cycle in 1..=256 {
                        monitor.update(
                            &registry,
                            &prepared,
                            ProtectionSample {
                                dt: seconds(0.00005),
                                vbus: volts(24.0),
                                velocity: erad_per_s(0.0),
                                fet_temp_c_x10: fet,
                                motor_temp_c_x10: motor,
                            },
                        );
                        if cycle < first_refresh {
                            assert_eq!(monitor.scales, DeratingScales::IDENTITY);
                        }
                        // Protection precedes the driver step, which advances
                        // the shared service frame once per native cycle.
                        #[cfg(feature = "service-slots")]
                        monitor.service_slots.advance();
                    }
                    let fet_c = fet.map(|t| f32::from(t) / 10.0);
                    let motor_c = motor.map(|t| f32::from(t) / 10.0);
                    let expected_hot = fet_c.is_some_and(|t| fet_limit > 0.0 && t > fet_limit)
                        || motor_c.is_some_and(|t| motor_limit > 0.0 && t > motor_limit);
                    assert_eq!(registry.has_category(FaultCategory::OverTemp), expected_hot);
                    assert_eq!(
                        monitor.scales,
                        DeratingConfig::default().compute(fet_c, motor_c, volts(24.0))
                    );
                }
            }
        }
    }

    #[cfg(feature = "service-slots")]
    #[test]
    fn slotted_derating_refreshes_every_eight_frames_at_both_pwm_rates() {
        let prepared = PreparedBoardConfig::new(board(60_000, 8_000));
        for hz in [15_000_u32, 20_000] {
            let frame_length = hz / 1_000;
            let dt = 1.0 / hz as f32;
            let mut monitor = ProtectionMonitor::new(seconds(dt));
            let registry = FaultRegistry::<StandardFault>::new();
            let retained = DeratingScales {
                drive: 0.123,
                brake: 0.234,
                regen: 0.345,
            };
            let refreshed = DeratingConfig::default().compute(Some(100.0), None, volts(24.0));
            assert_ne!(retained, refreshed);
            // Include the eight-bit frame-counter wrap.
            for cycle in 0..frame_length * 264 {
                monitor.scales = retained;
                monitor.update(
                    &registry,
                    &prepared,
                    ProtectionSample {
                        dt: seconds(dt),
                        vbus: volts(24.0),
                        velocity: erad_per_s(0.0),
                        fet_temp_c_x10: Some(1000),
                        motor_temp_c_x10: None,
                    },
                );
                let due = cycle % frame_length == frame_length / 2
                    && (cycle / frame_length).is_multiple_of(8);
                assert_eq!(
                    monitor.scales,
                    if due { refreshed } else { retained },
                    "rate={hz}, cycle={cycle}"
                );
                monitor.service_slots.advance();
            }
        }
    }
}
