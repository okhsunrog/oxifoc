//! Hall sensor calibration for G431 platform
//!
//! Implements the async calibration sweep using open-loop FOC control.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::f32::consts::TAU;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::hall_calibration::{
    CalibrationError, HallCalibrationParams, HallCalibrationResult, HallCalibrator, HallReader,
};
use oxifoc_core::motor::ControlMode;

use crate::control::foc::send_command;
use crate::sensors::hall::read_hall_state_raw;

/// Hall sensor reader implementation for G431
pub struct G431HallReader;

impl HallReader for G431HallReader {
    fn read_hall_state(&self) -> u8 {
        read_hall_state_raw()
    }
}

/// Run Hall sensor calibration
///
/// This function sends open-loop commands to the FOC driver to sweep the motor
/// through electrical angles while recording Hall sensor states.
///
/// # Requirements
/// - FOC driver must be initialized and running
/// - Motor should be unloaded (free to rotate)
/// - Current sensor must be calibrated
///
/// # Arguments
/// * `params` - Calibration parameters (current, timing, sweep count)
///
/// # Returns
/// * `Ok(HallCalibrationResult)` - Calibration completed successfully
/// * `Err(CalibrationError)` - Calibration failed
pub async fn calibrate_hall(
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, CalibrationError> {
    let reader = G431HallReader;
    let mut calibrator = HallCalibrator::new();

    defmt::info!(
        "Starting Hall calibration: {}A, {} sweeps, {}us step delay",
        params.current_amps,
        params.sweep_count,
        params.step_delay_us
    );

    // Step 1: Ramp up current at angle 0 to lock rotor
    defmt::info!("Ramping up current...");
    let ramp_steps = 100u32;
    let ramp_delay = Duration::from_millis(params.ramp_time_ms as u64 / ramp_steps as u64);

    for i in 1..=ramp_steps {
        let current = params.current_amps * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    // Hold at full current briefly to let rotor settle
    Timer::after(Duration::from_millis(200)).await;

    // Step 2: Perform sweeps
    let step_delay = Duration::from_micros(params.step_delay_us as u64);
    let degrees_per_sweep = 360u32;

    for sweep in 0..params.sweep_count {
        let forward = sweep % 2 == 0;
        defmt::info!(
            "Sweep {}/{} ({})",
            sweep + 1,
            params.sweep_count,
            if forward { "forward" } else { "reverse" }
        );

        for deg in 0..degrees_per_sweep {
            let actual_deg = if forward {
                deg
            } else {
                degrees_per_sweep - 1 - deg
            };
            let angle_rad = actual_deg as f32 * TAU / 360.0;

            // Command motor to this angle
            send_command(ControlMode::OpenLoop {
                angle_rad,
                current: params.current_amps,
            });

            // Wait for rotor to settle
            Timer::after(step_delay).await;

            // Read and record Hall state
            let hall_state = reader.read_hall_state();
            calibrator.record(angle_rad, hall_state);
        }
    }

    // Step 3: Ramp down and stop
    defmt::info!("Ramping down current...");
    for i in (0..ramp_steps).rev() {
        let current = params.current_amps * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    // Stop motor
    send_command(ControlMode::Stopped);
    Timer::after(Duration::from_millis(100)).await;

    // Step 4: Compute result
    defmt::info!("Computing calibration result...");
    let result = calibrator.finish()?;

    if result.is_valid() {
        defmt::info!("Hall calibration successful!");
        // Log the angles for each raw state
        for raw in 1..=6u8 {
            if let Some(angle) = result.angle_for_raw_state(raw) {
                let deg = angle * 180.0 / core::f32::consts::PI;
                defmt::info!("  Raw state {}: {} deg", raw, deg as i32);
            }
        }
    } else {
        defmt::error!("Hall calibration failed: invalid state count");
    }

    Ok(result)
}

/// Calibrate Hall sensors with default parameters
pub async fn calibrate_hall_default() -> Result<HallCalibrationResult, CalibrationError> {
    calibrate_hall(HallCalibrationParams::default()).await
}
