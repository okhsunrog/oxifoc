use super::*;
use core::cell::Cell;
use core::task::{Context, Poll, Waker};

struct ImmediateTimer;

impl Timer for ImmediateTimer {
    async fn after_millis(_: u64) {}
    async fn after_micros(_: u64) {}
}

struct Hardware {
    current: Cell<f32>,
    stopped: Cell<bool>,
    source: ResistanceVoltageSource,
    pi_slope: f32,
    measured_offset: f32,
    samples: u32,
    mix_sources: bool,
}

impl Hardware {
    fn new(source: ResistanceVoltageSource, pi_slope: f32) -> Self {
        Self {
            current: Cell::new(0.0),
            stopped: Cell::new(true),
            source,
            pi_slope,
            measured_offset: 0.030,
            samples: 0,
            mix_sources: false,
        }
    }
}

impl DetectionHardware for Hardware {
    async fn send_command(&self, mode: ControlMode) {
        match mode {
            ControlMode::OpenLoop { current, .. } => {
                self.current.set(current);
                self.stopped.set(false);
            }
            ControlMode::Stopped => {
                self.current.set(0.0);
                self.stopped.set(true);
            }
            _ => panic!("unexpected mode in resistance procedure"),
        }
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        FocOutput {
            vd: self.pi_slope * self.current.get(),
            id: self.current.get(),
            ..FocOutput::empty()
        }
    }

    fn resistance_voltage_source(&self) -> ResistanceVoltageSource {
        self.source
    }

    async fn wait_resistance_sample(&mut self) -> Result<ResistanceSample, DetectionError> {
        self.samples += 1;
        let voltage_v = match self.source {
            ResistanceVoltageSource::FilteredPhaseAdc => {
                0.031 * self.current.get() + self.measured_offset
            }
            ResistanceVoltageSource::ControllerCommand => self.pi_slope * self.current.get(),
        };
        Ok(ResistanceSample {
            voltage_v,
            current_a: self.current.get(),
            source: if self.mix_sources && self.samples == 1000 {
                ResistanceVoltageSource::ControllerCommand
            } else {
                self.source
            },
        })
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        (
            self.current.get(),
            -self.current.get() / 2.0,
            -self.current.get() / 2.0,
        )
    }
}

#[expect(
    clippy::panic_in_result_fn,
    reason = "a pending immediate test backend is a broken fixture"
)]
fn run(hw: &mut Hardware) -> Result<f32, DetectionError> {
    let params = ResistanceParams {
        current_max: 3.0,
        ..ResistanceParams::default()
    };
    let mut future = core::pin::pin!(measure_resistance::<_, ImmediateTimer>(hw, &params,));
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(result) => result,
        Poll::Pending => panic!("immediate test backend unexpectedly yielded"),
    }
}

#[test]
fn filtered_resistance_cancels_offset_without_using_negative_pi_slope() {
    for offset in [0.0, 0.030, -0.040] {
        let mut hw = Hardware::new(ResistanceVoltageSource::FilteredPhaseAdc, -0.12);
        hw.measured_offset = offset;
        let resistance = run(&mut hw).unwrap();
        assert!((resistance - 0.031).abs() < 0.00001, "R={resistance}");
        assert!(hw.stopped.get());
    }
}

#[test]
fn negative_controller_slope_is_rejected_instead_of_hidden_by_absolute_value() {
    let mut hw = Hardware::new(ResistanceVoltageSource::ControllerCommand, -0.12);
    assert_eq!(run(&mut hw), Err(DetectionError::OutOfRange));
    assert!(hw.stopped.get());
}

#[test]
fn positive_controller_model_remains_supported() {
    let mut hw = Hardware::new(ResistanceVoltageSource::ControllerCommand, 0.07);
    assert!((run(&mut hw).unwrap() - 0.07).abs() < 0.00001);
    assert!(hw.stopped.get());
}

#[test]
fn mixed_voltage_definitions_abort_and_stop_the_motor() {
    let mut hw = Hardware::new(ResistanceVoltageSource::FilteredPhaseAdc, -0.12);
    hw.mix_sources = true;
    assert_eq!(run(&mut hw), Err(DetectionError::HardwareFault));
    assert!(hw.stopped.get());
}
