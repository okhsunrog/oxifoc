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

struct Hardware;

impl DetectionHardware for Hardware {
    async fn send_command(&self, mode: ControlMode) {
        if let ControlMode::OpenLoop { angle_rad, .. } = mode {
            ANGLE.with(|angle| angle.set(angle_rad));
        }
        record(Event::Command(mode));
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        panic!("Hall calibration must use its Hall reader, not FOC telemetry");
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
        ((angle * 360.0 / core::f32::consts::TAU).round() as u16 / 60 + 1) as u8
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

#[test]
fn hall_sweeps_group_directions_and_visit_successive_motor_arcs() {
    let cases: [(&[bool], i32); 8] = [
        (&[], 0),
        (&[true], 359),
        (&[true, false], 359),
        (&[true, true, false], 719),
        (&[true, true, false, false], 719),
        (&[true, true, true, false, false], 1079),
        (&[true, true, true, false, false, false], 1079),
        (&[true, true, true, true, false, false, false], 1439),
    ];
    assert_eq!(HallCalibrationParams::default().sweep_count, 6);

    for (directions, expected_arc_degrees) in cases {
        let params = HallCalibrationParams {
            sweep_count: directions.len() as u8,
            ..Default::default()
        };
        EVENTS.with(|events| events.borrow_mut().clear());
        ANGLE.with(|angle| angle.set(0.0));
        let result = {
            let mut hw = Hardware;
            let mut future = core::pin::pin!(calibrate_hall::<_, RecordingTimer, _>(
                &mut hw, &Reader, params
            ));
            match future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
            {
                Poll::Ready(result) => result,
                Poll::Pending => panic!("recording backend unexpectedly yielded"),
            }
        };
        if directions.is_empty() {
            assert_eq!(result.unwrap_err(), DetectionError::LowConfidence);
        } else {
            assert!(result.unwrap().is_valid());
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
        for &forward in directions {
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

        for step in (0..100).rev() {
            assert_eq!(
                events.next(),
                Some(open_loop(
                    0.0,
                    params.current_amps * (step as f32 / 100.0),
                    None
                ))
            );
            assert_eq!(events.next(), Some(Event::Millis(10)));
        }
        assert_eq!(events.next(), Some(Event::Command(ControlMode::Stopped)));
        assert_eq!(events.next(), Some(Event::Millis(100)));
        assert_eq!(events.next(), None);
    }
}
