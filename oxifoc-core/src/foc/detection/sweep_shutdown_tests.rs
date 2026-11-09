use super::*;
#[cfg(feature = "hfi-detect")]
use crate::foc::detection::excitation::{BurstSample, Waveform};
use crate::foc::trig::LibmSinCos;
use core::cell::Cell;
use core::task::{Context, Poll, Waker};

struct ImmediateTimer;
impl Timer for ImmediateTimer {
    async fn after_millis(_: u64) {}
    async fn after_micros(_: u64) {}
}

#[test]
fn flux_settle_keeps_command_deadman_fed_and_observes_hardware_health() {
    std::thread_local! { static NOW_MS: Cell<u64> = const { Cell::new(0) }; }
    struct Clock;
    impl Timer for Clock {
        async fn after_millis(ms: u64) {
            NOW_MS.with(|t| t.set(t.get() + ms));
        }
        async fn after_micros(us: u64) {
            Self::after_millis(us / 1000).await;
        }
    }
    struct Hardware {
        last_command: Cell<u64>,
        failed: bool,
    }
    impl DetectionHardware for Hardware {
        fn check_health(&self) -> Result<(), DetectionError> {
            if self.failed || NOW_MS.with(|t| t.get() - self.last_command.get()) > 250 {
                Err(DetectionError::HardwareFault)
            } else {
                Ok(())
            }
        }
        async fn capture_voltage_burst<S: SinCos>(
            &mut self,
            _: VoltageBurst,
        ) -> Result<BurstCapture, DetectionError> {
            Err(DetectionError::MissingPrerequisite)
        }
        async fn send_command(&self, mode: ControlMode) {
            assert_eq!(
                mode,
                ControlMode::OpenLoop {
                    angle_rad: 0.0,
                    current: 2.0,
                    velocity_rad_s: 200.0,
                    pi_gains: None
                }
            );
            self.last_command.set(NOW_MS.with(Cell::get));
        }
        async fn wait_telemetry(&mut self) -> FocOutput {
            FocOutput::empty()
        }
        fn read_phase_currents(&self) -> (f32, f32, f32) {
            (0.0, 0.0, 0.0)
        }
    }
    for failed in [false, true] {
        NOW_MS.with(|t| t.set(0));
        let mut hw = Hardware {
            last_command: Cell::new(0),
            failed,
        };
        let params = FluxLinkageParams {
            settle_time_ms: 3000,
            current_a: 2.0,
            ..Default::default()
        };
        let mut future = core::pin::pin!(settle_open_loop::<_, Clock>(&mut hw, &params, 200.0));
        let result = future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()));
        assert_eq!(
            result,
            Poll::Ready(if failed {
                Err(DetectionError::HardwareFault)
            } else {
                Ok(())
            })
        );
        assert_eq!(NOW_MS.with(Cell::get), if failed { 0 } else { 3000 });
    }
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
    fn capture_voltage_burst<S: SinCos>(
        &mut self,
        program: VoltageBurst,
    ) -> impl Future<Output = Result<BurstCapture, DetectionError>> {
        capture_model_voltage_burst::<_, S>(self, program)
    }

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
            vq: 0.15,
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

#[cfg(feature = "hfi-detect")]
#[test]
fn hfi_fault_timeout_or_missing_cycles_cannot_fall_back_or_report_success() {
    struct BrokenPacket {
        hold: CompensatedHold,
        error: DetectionError,
        bursts: usize,
    }
    impl DetectionHardware for BrokenPacket {
        async fn send_command(&self, mode: ControlMode) {
            self.hold.send_command(mode).await;
        }
        async fn wait_telemetry(&mut self) -> FocOutput {
            self.hold.wait_telemetry().await
        }
        fn read_phase_currents(&self) -> (f32, f32, f32) {
            self.hold.read_phase_currents()
        }
        async fn capture_voltage_burst<S: SinCos>(
            &mut self,
            program: VoltageBurst,
        ) -> Result<BurstCapture, DetectionError> {
            assert!(
                (program.vq_hold - 0.15).abs() < 1e-5,
                "preserve the q-axis holding bias"
            );
            self.bursts += 1;
            Err(self.error)
        }
    }
    for error in [
        DetectionError::HardwareFault,
        DetectionError::Timeout,
        DetectionError::InsufficientSamples,
    ] {
        let mut hw = BrokenPacket {
            hold: CompensatedHold {
                current: Cell::new(0.0),
                vd: Cell::new(0.0),
                peak: Cell::new(0.0),
                stopped: Cell::new(true),
                saw_negative_voltage: Cell::new(false),
            },
            error,
            bursts: 0,
        };
        let params = InductanceParams {
            resistance_ohm: 0.03,
            hold_current_a: 1.2,
            ..Default::default()
        };
        {
            let mut future =
                core::pin::pin!(measure_inductance_auto::<_, ImmediateTimer, LibmSinCos>(
                    &mut hw, &params, 20_000.0
                ));
            assert_eq!(
                future
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop())),
                Poll::Ready(Err(error))
            );
        }
        assert_eq!(
            hw.bursts, 1,
            "no second excitation method after a failed packet"
        );
        assert!(hw.hold.stopped.get());
    }
}

#[cfg(feature = "hfi-detect")]
#[test]
fn missing_holding_current_refuses_all_excitation_methods() {
    struct OpenBridge {
        stopped: Cell<bool>,
    }
    impl DetectionHardware for OpenBridge {
        async fn send_command(&self, mode: ControlMode) {
            self.stopped.set(mode == ControlMode::Stopped);
        }
        async fn wait_telemetry(&mut self) -> FocOutput {
            FocOutput {
                vd: 10.0,
                ..FocOutput::empty()
            }
        }
        fn read_phase_currents(&self) -> (f32, f32, f32) {
            (0.0, 0.0, 0.0)
        }
        async fn capture_voltage_burst<S: SinCos>(
            &mut self,
            _: VoltageBurst,
        ) -> Result<BurstCapture, DetectionError> {
            panic!("no HFI/pulse is permitted without a measured DC hold")
        }
    }
    let mut hw = OpenBridge {
        stopped: Cell::new(true),
    };
    let params = InductanceParams {
        resistance_ohm: 0.03,
        hold_current_a: 1.2,
        ..Default::default()
    };
    {
        let mut future = core::pin::pin!(measure_inductance_auto::<_, ImmediateTimer, LibmSinCos>(
            &mut hw, &params, 20_000.0
        ));
        assert_eq!(
            future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop())),
            Poll::Ready(Err(DetectionError::MotorNotResponding))
        );
    }
    assert!(hw.stopped.get());
}

#[cfg(feature = "hfi-detect")]
#[test]
fn weak_main_response_cannot_bypass_voltage_and_growth_limits() {
    // The short lag probe sees a 25-uH winding. Subsequent packets deliberately
    // have a 5-mH-looking response, reproducing the inflated interim estimate
    // that previously changed a 0.25-A target into the old 2-A ripple clamp.
    struct WeakResponse {
        hold: CompensatedHold,
        bursts: usize,
        max_run_voltage: f32,
        previous_amplitude: f32,
    }
    impl DetectionHardware for WeakResponse {
        async fn send_command(&self, mode: ControlMode) {
            self.hold.send_command(mode).await;
        }
        async fn wait_telemetry(&mut self) -> FocOutput {
            self.hold.wait_telemetry().await
        }
        fn read_phase_currents(&self) -> (f32, f32, f32) {
            self.hold.read_phase_currents()
        }
        async fn capture_voltage_burst<S: SinCos>(
            &mut self,
            program: VoltageBurst,
        ) -> Result<BurstCapture, DetectionError> {
            let Waveform::RotatingHfi { amplitude, .. } = program.waveform else {
                panic!("expected HFI only");
            };
            if self.previous_amplitude > 0.0 {
                assert!(amplitude <= self.previous_amplitude * 1.701);
            }
            self.previous_amplitude = amplitude;
            if self.bursts > 0 {
                self.max_run_voltage = self.max_run_voltage.max(amplitude);
            }
            let l = if self.bursts == 0 { 25e-6 } else { 5e-3 };
            self.bursts += 1;
            let mut capture = BurstCapture {
                len: usize::from(program.samples),
                ..Default::default()
            };
            let (mut alpha, mut beta) = (program.hold_current, 0.0);
            for k in 0..capture.len {
                if k >= 2 {
                    let (va, vb) = program.injection::<S>(k - 2);
                    alpha += va / (20_000.0 * l);
                    beta += vb / (20_000.0 * l);
                }
                capture.samples[k] = BurstSample {
                    alpha,
                    beta,
                    cycle: k as u32,
                    tick_us: k as u32 * 50,
                };
            }
            Ok(capture)
        }
    }
    let mut hw = WeakResponse {
        hold: CompensatedHold {
            current: Cell::new(0.0),
            vd: Cell::new(0.0),
            peak: Cell::new(0.0),
            stopped: Cell::new(true),
            saw_negative_voltage: Cell::new(false),
        },
        bursts: 0,
        max_run_voltage: 0.0,
        previous_amplitude: 0.0,
    };
    let params = InductanceParams {
        resistance_ohm: 0.03,
        hold_current_a: 1.0,
        ..Default::default()
    };
    {
        let mut future = core::pin::pin!(measure_inductance::<_, ImmediateTimer, LibmSinCos>(
            &mut hw, &params, 20_000.0
        ));
        assert_eq!(
            future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop())),
            Poll::Ready(Err(DetectionError::LowConfidence))
        );
    }
    assert!(hw.bursts >= 3, "exercise the probe and pilot packets");
    assert!(hw.max_run_voltage > 0.0);
    assert!(
        hw.max_run_voltage <= params.hfi_voltage_v,
        "inflated HFI voltage: {}",
        hw.max_run_voltage
    );
    assert!(hw.hold.stopped.get());
}
