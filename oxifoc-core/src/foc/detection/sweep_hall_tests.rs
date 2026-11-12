use super::*;
use core::cell::{Cell, RefCell};
use core::task::{Context, Poll, Waker};
use std::vec::Vec;

#[derive(Debug, PartialEq)]
enum Event {
    Command(ControlMode),
    Millis(u64),
    Micros(u64),
    HallRead(f32),
    CurrentRead,
}

std::thread_local! {
    static EVENTS: RefCell<Vec<Event>> = const { RefCell::new(Vec::new()) };
    static ANGLE: Cell<f32> = const { Cell::new(0.0) };
}

fn record(event: Event) {
    EVENTS.with(|events| events.borrow_mut().push(event));
}

struct RecordingTimer;

impl Timer for RecordingTimer {
    async fn after_millis(ms: u64) {
        record(Event::Millis(ms));
    }

    async fn after_micros(us: u64) {
        record(Event::Micros(us));
    }
}

#[derive(Default)]
struct Hardware {
    current: Cell<f32>,
}

impl DetectionHardware for Hardware {
    async fn send_command(&self, mode: ControlMode) {
        if let ControlMode::OpenLoop {
            angle_rad, current, ..
        } = mode
        {
            ANGLE.with(|angle| angle.set(angle_rad));
            self.current.set(current);
        }
        record(Event::Command(mode));
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        record(Event::CurrentRead);
        FocOutput {
            id: self.current.get(),
            ..FocOutput::empty()
        }
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        panic!("Hall calibration must not depend on raw current readings");
    }
}

struct Reader;

impl HallReader for Reader {
    fn read_hall_state(&self) -> u8 {
        let angle = ANGLE.with(Cell::get);
        record(Event::HallRead(angle));
        // Six static 60-degree sectors; the rotor follows each command exactly.
        [1, 3, 2, 6, 4, 5][((angle * 360.0 / core::f32::consts::TAU).round() as usize / 60).min(5)]
    }
}

fn open_loop(angle_rad: f32, current: f32, pi_gains: Option<(f32, f32)>) -> Event {
    Event::Command(ControlMode::OpenLoop {
        angle_rad,
        current,
        velocity_rad_s: 0.0,
        pi_gains,
    })
}

fn poll_calibration(
    reader: &impl HallReader,
    params: HallCalibrationParams,
) -> Poll<Result<HallCalibrationResult, DetectionError>> {
    let mut hw = Hardware::default();
    let mut future = core::pin::pin!(calibrate_hall::<_, RecordingTimer, _>(
        &mut hw, reader, params
    ));
    future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn hall_sweeps_group_directions_and_visit_successive_motor_arcs() {
    let cases: [(&[bool], i32); 8] = [
        (&[], 0),
        (&[true], 719),
        (&[true, false], 719),
        (&[true, true, false], 1079),
        (&[true, true, false, false], 1079),
        (&[true, true, true, false, false], 1439),
        (&[true, true, true, false, false, false], 1439),
        (&[true, true, true, true, false, false, false], 1799),
    ];
    assert_eq!(HallCalibrationParams::default().sweep_count, 6);

    for (directions, expected_arc_degrees) in cases {
        let params = HallCalibrationParams {
            sweep_count: directions.len() as u8,
            ..Default::default()
        };
        EVENTS.with(|events| events.borrow_mut().clear());
        ANGLE.with(|angle| angle.set(0.0));
        let Poll::Ready(result) = poll_calibration(&Reader, params) else {
            panic!("recording backend unexpectedly yielded");
        };
        if directions.is_empty() || matches!(directions.len(), 2 | 3) {
            assert_eq!(result.unwrap_err(), DetectionError::LowConfidence);
        } else {
            let result = result.unwrap();
            assert!(result.is_valid());
            assert_eq!(result.geometry.is_some(), directions.len() >= 4);
        }

        let mut events =
            EVENTS.with(|events| core::mem::take(&mut *events.borrow_mut()).into_iter());
        for step in 1..=100 {
            let gains = (step == 1).then_some((DETECTION_PI_KP, DETECTION_PI_KI));
            assert_eq!(
                events.next(),
                Some(open_loop(
                    0.0,
                    params.current_amps * (step as f32 / 100.0),
                    gains
                ))
            );
            assert_eq!(events.next(), Some(Event::Millis(10)));
        }
        assert_eq!(events.next(), Some(Event::Millis(200)));

        let mut previous_degrees = 0i32;
        let mut unwrapped_degrees = 0;
        let mut maximum_degrees = 0;
        let mut physical_directions = Vec::new();
        for (index, &forward) in directions.iter().enumerate() {
            if index == 0 || directions[index - 1] != forward {
                physical_directions.push(forward); // unmeasured settling turn
            }
            physical_directions.push(forward);
        }
        for forward in physical_directions {
            for step in 0..360 {
                let degrees = if forward { step } else { 359 - step };
                let angle = degrees as f32 * core::f32::consts::TAU / 360.0;
                assert_eq!(
                    events.next(),
                    Some(open_loop(angle, params.current_amps, None))
                );
                assert_eq!(events.next(), Some(Event::Micros(5000)));
                assert_eq!(events.next(), Some(Event::HallRead(angle)));

                // Unwrap the actual commanded angles using the shortest step.
                // Consecutive forward turns must advance into a new motor arc.
                let mut delta = degrees - previous_degrees;
                if delta < -180 {
                    delta += 360;
                } else if delta > 180 {
                    delta -= 360;
                }
                assert!(delta.abs() <= 1);
                unwrapped_degrees += delta;
                maximum_degrees = maximum_degrees.max(unwrapped_degrees);
                previous_degrees = degrees;
            }
        }
        assert_eq!(maximum_degrees, expected_arc_degrees);

        let mut previous = params.current_amps;
        loop {
            let Some(Event::Command(ControlMode::OpenLoop {
                angle_rad: 0.0,
                current,
                velocity_rad_s: 0.0,
                pi_gains: None,
            })) = events.next()
            else {
                panic!("expected stationary current release");
            };
            assert!((0.0..=0.004).contains(&(previous - current)));
            assert_eq!(events.next(), Some(Event::Millis(1)));
            if current <= 0.5 {
                assert_eq!(events.next(), Some(Event::CurrentRead));
                break;
            }
            previous = current;
        }
        assert_eq!(events.next(), Some(Event::Command(ControlMode::Stopped)));
        assert_eq!(events.next(), Some(Event::Millis(100)));
        assert_eq!(events.next(), None);
    }
}

/// Simulate a directional tracking transient only during the first turn. An
/// optional additional transient in a measured turn must still fail quality.
struct SettlingReader {
    reads: Cell<usize>,
    measured_outlier: bool,
}

impl HallReader for SettlingReader {
    fn read_hall_state(&self) -> u8 {
        let index = self.reads.get();
        self.reads.set(index + 1);
        let turn = index / 360;
        let positive = turn < 4;
        let within_direction = turn % 4;
        let transient = within_direction == 0 || (self.measured_outlier && turn == 2);
        let lag = if transient { 30 } else { 0 } * if positive { 1 } else { -1 };
        let angle = ANGLE.with(Cell::get);
        let degrees = (angle * 360.0 / core::f32::consts::TAU).round() as i32;
        let physical = (degrees - lag).rem_euclid(360) as usize;
        [1, 3, 2, 6, 4, 5][physical / 60]
    }
}

#[test]
fn settling_turns_do_not_contaminate_measured_centroids_or_transition_geometry() {
    let reader = SettlingReader {
        reads: Cell::new(0),
        measured_outlier: false,
    };
    EVENTS.with(|events| events.borrow_mut().clear());
    ANGLE.with(|angle| angle.set(0.0));
    let Poll::Ready(result) = poll_calibration(&reader, HallCalibrationParams::default()) else {
        panic!("recording backend unexpectedly yielded");
    };
    let result = result.unwrap();
    assert_eq!(reader.reads.get(), 8 * 360);
    let geometry = result.geometry.unwrap();
    for (index, raw) in [1, 3, 2, 6, 4, 5].into_iter().enumerate() {
        let entry = index as f32 * core::f32::consts::TAU / 6.0;
        let centroid = entry + core::f32::consts::TAU / 12.0;
        let bin = core::f32::consts::TAU / 360.0;
        assert!(
            crate::foc::angle_difference(geometry.boundary_for_positive_entry(raw).unwrap(), entry,)
                .abs() <= bin
        );
        assert!(
            crate::foc::angle_difference(result.angles[usize::from(raw)], centroid).abs() <= bin
        );
    }
}

#[test]
fn inconsistent_measured_turn_is_rejected_after_settling() {
    let reader = SettlingReader {
        reads: Cell::new(0),
        measured_outlier: true,
    };
    EVENTS.with(|events| events.borrow_mut().clear());
    ANGLE.with(|angle| angle.set(0.0));
    assert!(matches!(
        poll_calibration(&reader, HallCalibrationParams::default()),
        Poll::Ready(Err(DetectionError::LowConfidence))
    ));
    assert_eq!(reader.reads.get(), 8 * 360);
}
