//! Offline replay of a stopped `dump-hall-mk5` capture through the real Hall
//! estimator. No device access. The output is JSONL at the supplied sample rate.
//! This isolates Hall estimation; it does not simulate the motor or observer.

use anyhow::{Result, bail, ensure};
use oxifoc_core::{
    foc::{
        hall_sensor::{HallSensor, PreparedHallCalibration},
        sensors::AngleSensor,
    },
    storage::HallCalibrationConfig,
};
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufReader, BufWriter, Write},
};

#[derive(Deserialize)]
struct Edge {
    ticks: u64,
    raw_state: u8,
    overcapture: bool,
}

#[derive(Deserialize)]
struct Capture {
    tick_hz: u64,
    dropped_after: u32,
    edges: Vec<Edge>,
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !(4..=5).contains(&args.len()) {
        bail!("Usage: replay_hall EDGES.json CONFIG.json OUTPUT.jsonl SAMPLE_HZ [INTERP_MIN_ERPM]");
    }
    let capture: Capture = serde_json::from_reader(BufReader::new(File::open(&args[0])?))?;
    let config: serde_json::Value = serde_json::from_reader(BufReader::new(File::open(&args[1])?))?;
    let calibration: HallCalibrationConfig =
        serde_json::from_value(config["hall-calibration"].clone())?;
    let prepared = PreparedHallCalibration::from_config(&calibration)
        .ok_or_else(|| anyhow::anyhow!("invalid Hall calibration"))?;
    let sample_hz: f64 = args[3].parse()?;
    ensure!(
        sample_hz.is_finite() && sample_hz > 0.0 && sample_hz <= capture.tick_hz as f64,
        "sample rate must be positive and no greater than the capture clock"
    );
    ensure!(
        capture.edges.len() >= 2
            && capture.dropped_after == 0
            && capture.edges.iter().all(|edge| !edge.overcapture)
            && capture
                .edges
                .windows(2)
                .all(|pair| pair[0].ticks < pair[1].ticks),
        "replay requires a complete, ordered capture without overcapture"
    );
    let mut hall = HallSensor::new(capture.tick_hz);
    hall.apply_prepared_hall_calibration(&prepared);
    if let Some(threshold) = args.get(4) {
        let threshold: f32 = threshold.parse()?;
        ensure!(
            threshold.is_finite() && threshold >= 0.0,
            "invalid interpolation threshold"
        );
        hall.set_interp_min_erpm(threshold);
    }
    let mut output = BufWriter::new(File::create(&args[2])?);
    let start = capture.edges[0].ticks;
    let end = capture.edges.last().unwrap().ticks;
    let mut next = 0;
    let mut sample_index = 0_u64;
    loop {
        let ticks =
            start + (sample_index as f64 * capture.tick_hz as f64 / sample_hz).round() as u64;
        if ticks > end {
            break;
        }
        while next < capture.edges.len() && capture.edges[next].ticks <= ticks {
            let edge = &capture.edges[next];
            hall.update(edge.raw_state, edge.ticks);
            next += 1;
        }
        if let (Some(raw), Some(smoothed)) = (hall.sample_at(ticks), hall.sample_at_mut(ticks)) {
            serde_json::to_writer(
                &mut output,
                &serde_json::json!({
                    "ticks": ticks, "state": capture.edges[next - 1].raw_state,
                    "angle": smoothed.angle, "raw_angle": raw.angle, "omega": smoothed.omega,
                    "interpolating": smoothed.omega.abs() * 60.0 / core::f32::consts::TAU >= hall.interp_min_erpm(),
                }),
            )?;
            writeln!(output)?;
        }
        sample_index += 1;
    }
    output.flush()?;
    Ok(())
}
