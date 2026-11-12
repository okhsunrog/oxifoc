//! Shared closed-loop rig for the sim examples: VirtualMotor + the firmware
//! control stack built the way the f405 boots it (`RuntimeConfig` motor
//! params → `FocController::from_runtime_config` +
//! `configure_observers_from_config` + the commutation phase tracker), with
//! hall edges time-stamped at the true boundary crossing (like the capture
//! timer) and the plant's actuation delay matching the driver's.

use std::f32::consts::{FRAC_PI_3, FRAC_PI_6, TAU};

use oxifoc_core::foc::angle_difference;
use oxifoc_core::foc::clamp_f32;
use oxifoc_core::foc::controller::{FocController, FocOutput};
use oxifoc_core::foc::hall_sensor::{HallSensor, PreparedHallCalibration};
use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::sensors::CurrentSensor;
use oxifoc_core::foc::transforms::{clarke, park};
use oxifoc_core::foc::trig::LibmSinCos;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::storage::{
    CurrentLimitsConfig, HallCalibrationConfig, HallTransitionGeometryConfig, MotorParamsConfig,
    RuntimeConfig,
};
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

pub const FOC_HZ: u32 = 20_000;
pub const DT: f32 = 1.0 / FOC_HZ as f32;
const TICKS: u64 = 1_000_000 / FOC_HZ as u64;

pub use oxifoc_core::virtual_motor::io::VirtualPwm as Pwm;

pub struct Currents(pub (f32, f32, f32));

impl CurrentSensor for Currents {
    fn read_currents(&self) -> (f32, f32, f32) {
        self.0
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
    fn set_offsets(&mut self, _offsets: (f32, f32, f32)) {}
}

pub type Driver = FocDriver<Pwm, Currents, PhaseManager<HallSensor>, LibmSinCos>;

/// Plant + config preset: (plant, bus voltage, current limit).
pub fn preset(name: &str) -> (MotorParams, f32, f32) {
    match name {
        // ZD2808 700 KV bench motor (2026-07-05 detection values).
        "zd2808" => (
            MotorParams {
                r: 0.127,
                ld: 86e-6,
                lq: 129e-6,
                lambda: 1.145e-3,
                pole_pairs: 7,
                j: 5e-5,
                friction_b: 4e-6,
                ..MotorParams::default()
            },
            12.0,
            10.0,
        ),
        // Flipsky 5065 270 KV: Y-equivalent of the LCR line-to-line
        // values, λ from KV.
        _ => (
            MotorParams {
                r: 0.036,
                ld: 22e-6,
                lq: 22e-6,
                lambda: 60.0 / (3f32.sqrt() * TAU * 270.0 * 7.0),
                pole_pairs: 7,
                j: 4e-4,
                friction_b: 2e-5,
                ..MotorParams::default()
            },
            24.0,
            30.0,
        ),
    }
}

pub struct RigConfig {
    pub motor: String,
    /// Extra viscous load (N·m·s/rad, mechanical).
    pub visc: f32,
    pub substeps: u32,
    /// Actuation delay (PWM periods), plant and driver.
    pub delay: f32,
    /// Phase-tracker ωn (rad/s; 0 = off).
    pub tracker: f32,
    /// Rotor inertia override (kg·m², mechanical; 0 = the preset's). A
    /// rider on a 5065 / 36:15 / 83 mm board reflects ~0.024.
    pub j: f32,
    pub start_angle: f32,
    pub motor_params: Option<MotorParamsConfig>,
    pub hall_calibration: Option<HallCalibrationConfig>,
    pub current_limits: Option<CurrentLimitsConfig>,
}

impl Default for RigConfig {
    fn default() -> Self {
        Self {
            motor: "flipsky".to_owned(),
            visc: 0.0,
            substeps: 8,
            delay: 0.5,
            tracker: 100.0,
            j: 0.0,
            start_angle: 0.0,
            motor_params: None,
            hall_calibration: None,
            current_limits: None,
        }
    }
}

/// One FOC period as the driver saw it: the plant state it sampled and what
/// it computed.
pub struct StepRecord {
    pub sample: VirtualMotorOutput,
    pub telem: FocOutput,
}

impl StepRecord {
    /// TRUE dq current: the sampled phase currents in the plant's rotor
    /// frame.
    pub fn true_dq(&self) -> (f32, f32) {
        let (i_alpha, i_beta) = clarke(self.sample.ia, self.sample.ib);
        let (s, c) = self.sample.angle_rad.sin_cos();
        park(i_alpha, i_beta, s, c)
    }

    /// Commutation frame error vs the rotor at the sample instant (rad).
    pub fn frame_err(&self) -> f32 {
        angle_difference(self.telem.angle_rad, self.sample.angle_rad)
    }
}

pub struct Rig {
    pub params: MotorParams,
    pub motor: VirtualMotor,
    pub out: VirtualMotorOutput,
    pub driver: Driver,
    /// Coulomb load magnitude (N·m), opposes motion.
    pub load: f32,
    /// Signed extra load torque (N·m, positive = braking) — disturbance
    /// injection.
    pub torque_disturbance: f32,
    /// Unwrapped mechanical rotor angle (rad).
    pub mech_angle: f64,
    last_hall: u8,
    hall_geometry: Option<HallTransitionGeometryConfig>,
    prev_angle: f32,
    step: u64,
}

impl Rig {
    pub fn new(cfg: &RigConfig) -> Self {
        let (mut params, vbus, i_max) = preset(&cfg.motor);
        if let Some(model) = &cfg.motor_params {
            params.r = model.resistance_ohm;
            params.ld = model.inductance_d_h;
            params.lq = model.inductance_q_h;
            params.lambda = model.flux_linkage_wb;
            params.pole_pairs = model.pole_pairs;
        }
        if cfg.j > 0.0 {
            params.j = cfg.j;
        }
        params.friction_b += cfg.visc;
        params.substeps = cfg.substeps;
        params.actuation_delay = cfg.delay;
        let config = RuntimeConfig {
            motor_params: Some(MotorParamsConfig {
                resistance_ohm: params.r,
                inductance_d_h: params.ld,
                inductance_q_h: params.lq,
                flux_linkage_wb: params.lambda,
                pole_pairs: params.pole_pairs,
                ..MotorParamsConfig::default()
            }),
            ..RuntimeConfig::default()
        };
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(cfg.start_angle);
        let mut out = motor.step(0.0, 0.0, 0.0, DT);
        let mut hall = HallSensor::new(1_000_000);
        let hall_geometry = cfg.hall_calibration.as_ref().and_then(|config| {
            let prepared = PreparedHallCalibration::from_config(config)
                .expect("invalid Hall reference in sim config");
            hall.install_prepared_calibration(&prepared);
            config.geometry
        });
        if let Some(geometry) = &hall_geometry {
            out.hall_state = measured_hall_state(geometry, out.angle_rad);
        }
        hall.update(out.hall_state, 0);
        let mut phase = PhaseManager::with_hall(hall);
        phase.configure_observers_from_config(&config, vbus);
        phase.set_phase_tracker(cfg.tracker, 1.2);
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_runtime_config(&config, vbus);
        let pwm = Pwm {
            duties: [0; 3],
            enabled: false,
        };
        let mut driver = FocDriver::new(foc, pwm, Currents((0.0, 0.0, 0.0)), phase, DT);
        let limits = cfg.current_limits.as_ref().map_or_else(
            || CurrentLimits::from_max_current(i_max),
            |limits| CurrentLimits::from_config_clamped(limits, i_max, 0.0),
        );
        driver.set_current_limits(limits);
        driver.set_actuation_delay(cfg.delay);
        Self {
            params,
            motor,
            out,
            driver,
            load: 0.0,
            torque_disturbance: 0.0,
            mech_angle: 0.0,
            last_hall: out.hall_state,
            hall_geometry,
            prev_angle: out.angle_rad,
            step: 0,
        }
    }

    pub fn set_source(&mut self, source: PhaseSource) {
        self.driver
            .phase_mut()
            .set_source(source)
            .expect("source configured");
    }

    /// Model time (s) of the next step's sample.
    pub fn t(&self) -> f32 {
        self.step as f32 * DT
    }

    /// Mechanical rpm → electrical rad/s.
    pub fn rpm_to_el(&self, rpm: f32) -> f32 {
        rpm * TAU / 60.0 * f32::from(self.params.pole_pairs)
    }

    /// Advance one FOC period.
    pub fn step(&mut self) -> StepRecord {
        let omega_m = self.out.omega_e / f32::from(self.params.pole_pairs);
        self.step_with_load(self.load * (omega_m / 2.0).tanh() + self.torque_disturbance)
    }

    /// Advance one FOC period with an externally calculated load torque (N·m).
    pub fn step_with_load(&mut self, load: f32) -> StepRecord {
        self.step += 1;
        let now = self.step * TICKS;
        if self.out.hall_state != self.last_hall {
            // Interpolate the crossing time between the two plant samples.
            let a0 = self.prev_angle;
            let a1 = a0 + angle_difference(self.out.angle_rad, a0);
            let boundary = if let Some(geometry) = &self.hall_geometry {
                let entered = if a1 > a0 {
                    self.out.hall_state
                } else {
                    self.last_hall
                };
                Some(a0 + angle_difference(geometry.entry_angles[usize::from(entered)], a0))
            } else {
                let k0 = ((a0 - FRAC_PI_6) / FRAC_PI_3).floor();
                let k1 = ((a1 - FRAC_PI_6) / FRAC_PI_3).floor();
                (k0 != k1).then_some(FRAC_PI_6 + FRAC_PI_3 * k0.max(k1))
            };
            let frac = boundary.filter(|_| a1 != a0).map_or(1.0, |boundary| {
                clamp_f32((boundary - a0) / (a1 - a0), 0.0, 1.0)
            });
            let edge = now - ((1.0 - frac) * TICKS as f32) as u64;
            self.driver
                .phase_mut()
                .hall_mut()
                .update(self.out.hall_state, edge);
            self.last_hall = self.out.hall_state;
        }
        let sample = self.out;
        self.driver.current_sensor_mut().0 = (sample.ia, sample.ib, sample.ic);
        let telem = self.driver.step(now).unwrap_or_default();
        self.prev_angle = sample.angle_rad;
        let omega_m = sample.omega_e / f32::from(self.params.pole_pairs);
        self.out = if self.driver.pwm().enabled {
            self.motor.step(telem.v_alpha, telem.v_beta, load, DT)
        } else {
            self.motor.step_coast(load, DT)
        };
        if let Some(geometry) = &self.hall_geometry {
            self.out.hall_state = measured_hall_state(geometry, self.out.angle_rad);
        }
        self.mech_angle += f64::from(omega_m * DT);
        StepRecord { sample, telem }
    }
}

fn measured_hall_state(geometry: &HallTransitionGeometryConfig, angle: f32) -> u8 {
    // The most recent forward entry boundary identifies the active state.
    (1_usize..=6)
        .min_by(|&a, &b| {
            let distance = |state: usize| (angle - geometry.entry_angles[state]).rem_euclid(TAU);
            distance(a).total_cmp(&distance(b))
        })
        .expect("nonempty Hall state set") as u8
}
