//! FOC simulation loop — FocController + VirtualMotor running on tokio.

use core::cell::RefCell;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use std::time::Duration;

use critical_section::Mutex as CriticalSectionMutex;
use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::foc::hall_sensor::Direction;
use oxifoc_core::foc::pi_controller::PIController;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::sensors::HallSnapshot;
use oxifoc_core::motor::ControlMode;
use oxifoc_core::state::{self, CMD_CHANNEL, MotorControlState};
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};
use tracing::info;

use oxifoc_core::foc::fault::StandardFault;

/// Run the FOC simulation loop.
pub async fn foc_loop(
    foc_freq: u32,
    batch: usize,
    vbus: f32,
    load_torque: f32,
    params: MotorParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
) {
    let _ = fault_registry; // available for future fault injection

    let dt = 1.0 / foc_freq as f32;

    let kp = params.ld * 1000.0;
    let ki = params.r * 1000.0;

    let mut foc = FocController::<SvpwmModulator>::new(vbus);
    foc.id_pi = PIController::new(kp, ki);
    foc.iq_pi = PIController::new(kp, ki);

    let mut motor = VirtualMotor::new(params);
    let mut out = VirtualMotorOutput::default();
    let mut control_mode = ControlMode::Stopped;
    let mut seq: u32 = 0;

    let sleep_us = (batch as u64 * 1_000_000) / u64::from(foc_freq);
    let mut interval = tokio::time::interval(Duration::from_micros(sleep_us));

    info!(
        "Simulation started: {}Hz, batch={}, vbus={}V, sleep={}µs",
        foc_freq, batch, vbus, sleep_us
    );

    loop {
        interval.tick().await;

        // Process commands from protocol servers
        while let Ok(cmd) = CMD_CHANNEL.try_receive() {
            // The virtual device has no FocDriver — only mode commands
            // affect the simulation; limits/gains commands are accepted
            // and ignored.
            let state::DriverCommand::SetMode(mode) = cmd else {
                continue;
            };
            control_mode = mode;
            critical_section::with(|cs| {
                let mut s = state_mutex.borrow(cs).borrow_mut();
                match mode {
                    ControlMode::Stopped => {
                        s.set_stopped();
                        foc.reset();
                    }
                    _ => s.set_running(mode),
                }
            });
        }

        let (id_target, iq_target) = match control_mode {
            ControlMode::CurrentControl {
                id_target,
                iq_target,
            } => (id_target, iq_target),
            _ => (0.0, 0.0),
        };

        // Run batch of simulation steps through the SAME telemetry path the
        // platform ISRs use (publish_cycle_telemetry: state update + the
        // anti-alias CIC decimator + bbqueue push) — the virtual device must
        // exercise what the firmware ships, not a parallel re-implementation.
        use oxifoc_core::foc::sensors::{AdcSnapshot, TempSensorId};
        let vbus_mv = (vbus * 1000.0) as u32;
        // Encode phase currents back into raw ADC counts (the inverse of the
        // host's `ShuntCurrentSense::convert_raw`) with `VIRTUAL_CALIB` + a
        // mid-scale offset — so the host, which falls back to mid-scale when the
        // device stores no dc_offsets, reconstructs the sim's real currents.
        let calib = crate::VIRTUAL_CALIB;
        let offset = f32::from(calib.adc_max_counts) / 2.0;
        let scale = calib.adc_vref_mv as f32
            / 1000.0
            / f32::from(calib.adc_max_counts)
            / calib.shunt_ohms
            / calib.amp_gain;
        let to_adc = |i: f32| -> u16 {
            let raw = if calib.invert_current_sign {
                offset - i / scale
            } else {
                offset + i / scale
            };
            raw.clamp(0.0, f32::from(calib.adc_max_counts)) as u16
        };
        for _ in 0..batch {
            let last_foc_out = foc.step(
                (out.ia, out.ib, out.ic),
                out.angle_rad,
                id_target,
                iq_target,
                1000,
                dt,
            );
            out = motor.step(last_foc_out.v_alpha, last_foc_out.v_beta, load_torque, dt);
            seq = seq.wrapping_add(1);

            let adc =
                AdcSnapshot::new(to_adc(out.ia), to_adc(out.ib), to_adc(out.ic), vbus_mv, seq)
                    .with_temp(TempSensorId::Fet, 250); // 25.0°C
            let hall = HallSnapshot {
                angle_rad: out.angle_rad,
                velocity_rad_s: out.omega_e,
                direction: if out.omega_e > 0.1 {
                    Direction::Clockwise
                } else if out.omega_e < -0.1 {
                    Direction::CounterClockwise
                } else {
                    Direction::Stopped
                },
                state: out.hall_state,
                error_count: 0,
            };
            publish_cycle_telemetry(
                state_mutex,
                adc,
                Some(hall),
                last_foc_out,
                params.pole_pairs,
                seq,
            );
        }
    }
}
