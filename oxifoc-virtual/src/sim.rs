//! Network device simulation through the firmware command/protection/control path.

use core::cell::RefCell;
use std::f32::consts::{FRAC_PI_3, FRAC_PI_6};
use std::time::Duration;

use critical_section::Mutex as CriticalSectionMutex;
use oxifoc_core::foc::angle_difference;
use oxifoc_core::foc::clamp_f32;
use oxifoc_core::foc::config::BoardConfig;
use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::current_sense::ShuntCurrentSense;
use oxifoc_core::foc::fault::{FaultRegistry, StandardFault};
use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::sensors::{AdcSnapshot, CurrentSensor, HallSnapshot, TempSensorId};
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::derating::DeratingConfig;
use oxifoc_core::motor::failsafe::FailsafeConfig;
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use oxifoc_core::state::{MotorControlState, run_foc_cycle};
use oxifoc_core::storage::RuntimeConfig;
use oxifoc_core::virtual_motor::io::VirtualPwm;
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};
use tracing::info;

/// Ideal phase currents encoded into a quantised ADC, then decoded by the
/// firmware converter. Applied offset writes affect control and telemetry in
/// the same units as a physical board; the plant's zero offset stays fixed.
struct Currents {
    raw: [u16; 3],
    converter: ShuntCurrentSense,
}

impl Currents {
    fn new(config: &RuntimeConfig) -> Self {
        let mut converter = ShuntCurrentSense::from_calib(&crate::VIRTUAL_CALIB);
        let offset = f32::from(crate::VIRTUAL_CALIB.adc_max_counts) / 2.0;
        let offsets = config
            .dc_offsets
            .as_ref()
            .map_or([offset; 3], |c| [c.phase_a, c.phase_b, c.phase_c]);
        converter.set_offsets(offsets[0], offsets[1], offsets[2]);
        Self {
            raw: [offset as u16; 3],
            converter,
        }
    }

    fn sample(&mut self, out: VirtualMotorOutput) {
        let calib = crate::VIRTUAL_CALIB;
        let offset = f32::from(calib.adc_max_counts) / 2.0;
        let scale = calib.adc_vref_mv as f32
            / 1000.0
            / f32::from(calib.adc_max_counts)
            / calib.shunt_ohms
            / calib.amp_gain;
        self.raw = [out.ia, out.ib, out.ic].map(|i| {
            let raw = if calib.invert_current_sign {
                offset - i / scale
            } else {
                offset + i / scale
            };
            clamp_f32(raw, 0.0, f32::from(calib.adc_max_counts)).round() as u16
        });
    }
}

impl CurrentSensor for Currents {
    fn read_currents(&self) -> (f32, f32, f32) {
        self.converter
            .convert_raw(self.raw[0], self.raw[1], self.raw[2])
    }
    fn read_raw(&self) -> (u16, u16, u16) {
        (self.raw[0], self.raw[1], self.raw[2])
    }
    fn is_calibrated(&self) -> bool {
        self.converter.is_calibrated()
    }
    fn get_offsets(&self) -> (f32, f32, f32) {
        self.converter.get_offsets()
    }
    fn set_offsets(&mut self, offsets: (f32, f32, f32)) {
        self.converter.set_offsets(offsets.0, offsets.1, offsets.2);
    }
}

/// Synthetic board thresholds. Unlike the motor model/config, these represent
/// the inverter's fixed hardware limits and cannot be raised by config writes.
pub fn board(max_current_a: f32) -> BoardConfig {
    BoardConfig {
        calib: crate::VIRTUAL_CALIB,
        max_iq_target_a: max_current_a,
        max_phase_current_a: max_current_a,
        max_vbus_mv: 60_000,
        min_vbus_mv: 8_000,
        max_fet_temp_c: 100.0,
        max_motor_temp_c: 0.0,
        phase_sense: None,
    }
}

/// Run the same command, deadman, protection and FOC cycle as the board ISR.
#[allow(clippy::too_many_arguments)]
pub async fn foc_loop(
    foc_freq: u32,
    batch: usize,
    vbus: f32,
    load_torque: f32,
    params: MotorParams,
    board: BoardConfig,
    config: RuntimeConfig,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) {
    let dt = 1.0 / foc_freq as f32;
    let mut motor = VirtualMotor::new(params);
    let mut out = motor.step(0.0, 0.0, 0.0, dt);
    let mut hall = HallSensor::new(1_000_000);
    hall.update(out.hall_state, 0);
    if let Some(calib) = &config.hall_calibration {
        hall.set_calibration_raw(calib.angles);
    }
    let mut phase = PhaseManager::with_hall(hall);
    phase.configure_observers_from_config(&config, vbus);
    phase.set_phase_tracker(100.0, 1.2);
    let controller = FocController::<SvpwmModulator>::from_runtime_config(&config, vbus);
    let pwm = VirtualPwm {
        duties: [0; 3],
        enabled: false,
    };
    let mut driver = FocDriver::new(controller, pwm, Currents::new(&config), phase, dt);
    driver.set_actuation_delay(params.actuation_delay);
    driver.set_current_limits(CurrentLimits::from_stored(
        config.current_limits.as_ref(),
        board.max_phase_current_a,
        config
            .motor_params
            .as_ref()
            .and_then(oxifoc_core::storage::MotorParamsConfig::rating_current_a)
            .unwrap_or(0.0),
    ));
    driver.set_failsafe(FailsafeConfig::from_stored(config.failsafe.as_ref()));
    driver.set_ov_threshold(board.max_vbus_mv as f32 / 1000.0);
    driver.set_velocity_config(VelocityLoopConfig::from_stored(config.velocity.as_ref()));
    driver.set_derating(DeratingConfig::from_stored(config.derating.as_ref()));

    let mut seq = 0u32;
    let mut step = 0u64;
    let mut previous_angle = out.angle_rad;
    let mut last_hall = out.hall_state;
    let sleep_us = (batch as u64 * 1_000_000) / u64::from(foc_freq);
    let mut interval = tokio::time::interval(Duration::from_micros(sleep_us));
    info!("Simulation started: {foc_freq}Hz, batch={batch}, vbus={vbus}V, sleep={sleep_us}µs");

    loop {
        interval.tick().await;
        for _ in 0..batch {
            step += 1;
            let now = step * 1_000_000 / u64::from(foc_freq);
            if out.hall_state != last_hall {
                // Timestamp the true sector crossing within the previous
                // plant period, as the examples' capture-timer model does.
                let a0 = previous_angle;
                let a1 = a0 + angle_difference(out.angle_rad, a0);
                let k0 = ((a0 - FRAC_PI_6) / FRAC_PI_3).floor();
                let k1 = ((a1 - FRAC_PI_6) / FRAC_PI_3).floor();
                let fraction = if k0 == k1 {
                    1.0
                } else {
                    clamp_f32(
                        (FRAC_PI_6 + FRAC_PI_3 * k0.max(k1) - a0) / (a1 - a0),
                        0.0,
                        1.0,
                    )
                };
                let edge = now - ((1.0 - fraction) * dt * 1_000_000.0) as u64;
                driver.phase_mut().hall_mut().update(out.hall_state, edge);
                last_hall = out.hall_state;
            }
            let sample = out;
            driver.current_sensor_mut().sample(sample);
            seq = seq.wrapping_add(1);
            let [ia, ib, ic] = driver.current_sensor().raw;
            let adc = AdcSnapshot::new(ia, ib, ic, (vbus * 1000.0) as u32, seq)
                .with_temp(TempSensorId::Fet, 250);
            // Protection consumes the previous ADC snapshot on boards. Publish
            // this cycle's ADC together with FOC below, preserving that order.
            let telem = run_foc_cycle(state_mutex, fault_registry, &mut driver, vbus, now, &board)
                .unwrap_or_default();
            let hall = HallSnapshot {
                angle_rad: sample.angle_rad,
                velocity_rad_s: sample.omega_e,
                direction: if sample.omega_e > 0.1 {
                    Direction::Clockwise
                } else if sample.omega_e < -0.1 {
                    Direction::CounterClockwise
                } else {
                    Direction::Stopped
                },
                state: sample.hall_state,
                error_count: 0,
            };
            publish_cycle_telemetry(state_mutex, adc, Some(hall), telem, params.pole_pairs, seq);
            previous_angle = sample.angle_rad;
            // Load opposes motion, including after a direction reversal.
            let omega_m = sample.omega_e / f32::from(params.pole_pairs);
            let load = load_torque * (omega_m / 2.0).tanh();
            out = if driver.pwm().enabled {
                motor.step(telem.v_alpha, telem.v_beta, load, dt)
            } else {
                motor.step_coast(load, dt)
            };
        }
    }
}
