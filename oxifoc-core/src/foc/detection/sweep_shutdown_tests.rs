use super::*;
use crate::foc::trig::LibmSinCos;
use core::cell::Cell;
use core::task::{Context, Poll, Waker};

struct ImmediateTimer;
impl Timer for ImmediateTimer {
    async fn after_millis(_: u64) {}
    async fn after_micros(_: u64) {}
}

/// Steady-state winding with an effective positive voltage correction. The PI
/// holds positive current using a negative command. Tiny diagnostic pulses are
/// deliberately below the detection floor, exercising failure cleanup too.
struct CompensatedHold {
    current: Cell<f32>,
    vd: Cell<f32>,
    peak: Cell<f32>,
    stopped: Cell<bool>,
    saw_negative_voltage: Cell<bool>,
}

impl DetectionHardware for CompensatedHold {
    async fn send_command(&self, mode: ControlMode) {
        let (current, vd, stopped) = match mode {
            ControlMode::OpenLoop { current, .. } => (current, 0.03 * current - 0.22, false),
            ControlMode::DirectVoltage { vd, .. } => {
                self.saw_negative_voltage
                    .set(self.saw_negative_voltage.get() || vd < 0.0);
                ((vd + 0.22) / 0.03, vd, false)
            }
            ControlMode::Stopped => (0.0, 0.0, true),
            _ => panic!("unexpected mode in pulse detection"),
        };
        self.current.set(current);
        self.vd.set(vd);
        self.stopped.set(stopped);
        self.peak.set(self.peak.get().max(current.abs()));
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        FocOutput {
            vd: self.vd.get(),
            id: self.current.get(),
            ..FocOutput::empty()
        }
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        let i = self.current.get();
        (i, -i / 2.0, -i / 2.0)
    }
}

#[test]
fn failed_inductance_pulse_removes_negative_voltage_hold_without_current_runaway() {
    let mut hw = CompensatedHold {
        current: Cell::new(0.0),
        vd: Cell::new(0.0),
        peak: Cell::new(0.0),
        stopped: Cell::new(true),
        saw_negative_voltage: Cell::new(false),
    };
    let params = VoltagePulseParams {
        hold_current_a: 1.2,
        resistance_ohm: 0.03,
        pulse_voltage_v: 0.0001,
        current_target_a: 0.0,
        num_pulses: 2,
        settle_time_ms: 0,
    };
    let result = {
        let mut future = core::pin::pin!(
            measure_inductance_pulse::<_, ImmediateTimer, LibmSinCos>(&mut hw, &params, 20_000.0)
        );
        match future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
        {
            Poll::Ready(result) => result,
            Poll::Pending => panic!("immediate backend unexpectedly yielded"),
        }
    };
    assert!(
        result.is_err(),
        "subthreshold pulses must not invent an inductance"
    );
    assert!(
        hw.saw_negative_voltage.get(),
        "exercise the compensated negative hold"
    );
    assert!(hw.stopped.get());
    assert_eq!(hw.current.get(), 0.0);
    // The old voltage ramp ended at command zero -> 0.22 / 0.03 = 7.33 A.
    assert!(
        hw.peak.get() < 1.22,
        "cleanup increased current to {} A",
        hw.peak.get()
    );
}
