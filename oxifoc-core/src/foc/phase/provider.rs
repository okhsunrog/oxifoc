//! Phase provider trait for FOC control
//!
//! Abstracts electrical phase angle provision, allowing FocDriver to work
//! with any phase source (Hall, Encoder, Observer, or combinations).

use crate::foc::hall_sensor::HallFaultKind;

/// Output from phase provider
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseOutput {
    /// Electrical angle (radians, 0 to 2π)
    pub angle: f32,
    /// Electrical velocity (rad/s)
    pub velocity: f32,
}

/// Input for phase provider update
#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseInput {
    /// Commanded α voltage (for observer, V)
    pub v_alpha: f32,
    /// Commanded β voltage (for observer, V)
    pub v_beta: f32,
    /// Measured α current (A)
    pub i_alpha: f32,
    /// Measured β current (A)
    pub i_beta: f32,
    /// Time step (seconds)
    pub dt: f32,
}

/// Provides electrical phase angle to FOC controller
///
/// This trait abstracts the source of electrical angle, allowing FocDriver
/// to work with:
/// - Hardware sensors (Hall, Encoder)
/// - Software observers (Back-EMF, HFI)
/// - Hybrid modes (sensor + observer blending)
/// - Manual/open-loop control
///
/// ## Control Flow
///
/// ```text
/// ┌──────────────────────────────────────────────────────────┐
/// │                     FOC Control Step                      │
/// │                                                          │
/// │  1. phase.get()           ← Get angle for transforms     │
/// │  2. Read currents         ← ADC sampling                 │
/// │  3. Run FOC math          ← Clarke/Park/PI/SVPWM         │
/// │  4. Apply PWM duties      ← Hardware output              │
/// │  5. phase.update(input)   ← Feed back for next step      │
/// │                                                          │
/// └──────────────────────────────────────────────────────────┘
/// ```
pub trait PhaseProvider {
    /// Get current phase estimate
    ///
    /// Called at the START of each control step to obtain the electrical
    /// angle for Park/Clarke transforms.
    fn get(&self) -> PhaseOutput;

    /// Update with latest measurements
    ///
    /// Called at the END of each control step with commanded voltages
    /// and measured currents. This information is used by observers
    /// to estimate phase for the next step.
    ///
    /// # Arguments
    /// * `input` - Voltages and currents from the current step
    /// * `now_ticks` - Current timestamp in sensor's tick timebase
    fn update(&mut self, input: &PhaseInput, now_ticks: u64);

    /// dq voltage to inject this cycle (HFI carrier), in the rotor frame
    /// at [`get`](Self::get)'s angle.
    ///
    /// The control loop must read this BETWEEN `get()` and `update()` and
    /// add it to the PI outputs (`FocController::step_with_injection`):
    /// the estimator demodulates the currents fed to the next `update()`
    /// against this exact carrier sample, and `update()` then advances the
    /// carrier. Default: no injection (sources without HFI).
    fn injection(&self) -> (f32, f32) {
        (0.0, 0.0)
    }

    /// Request a switch of the angle source (host command).
    ///
    /// Returns whether the request was applied. The default declines:
    /// simple providers have exactly one source. `PhaseManager` overrides
    /// this with its validated [`set_source`](super::PhaseManager::set_source).
    fn request_source(&mut self, _source: super::source::PhaseSource) -> bool {
        false
    }

    /// Whether the angle estimate can be trusted down to (near) standstill.
    ///
    /// Used by the failsafe regen-brake, which must not commutate blind: a
    /// hardware sensor tracks the rotor to a full stop, but a pure back-EMF
    /// observer loses lock below its speed floor, so braking on it near zero
    /// speed would mis-commutate. Default `true` (direct-sensor providers).
    fn angle_trustworthy(&self) -> bool {
        true
    }

    /// Active hall degradation, if any — bridged into the fault registry as
    /// a sticky `HallError` warning by `run_foc_cycle` (set-only: recovery
    /// clears the behavior, not the record). Default: nothing to report.
    fn hall_fault(&self) -> Option<HallFaultKind> {
        None
    }

    /// Begin a sensorless cold start in `dir` (sign of the commanded
    /// torque/velocity) on the Stopped/Coast → drive transition.
    ///
    /// A pure back-EMF observer cannot commutate from standstill (no
    /// back-EMF), so `PhaseManager` overrides this to run an open-loop
    /// align→ramp→handoff sequence until the observer locks. The default is a
    /// no-op: sensored providers already have an angle at zero speed.
    fn begin_cold_start(&mut self, _dir: f32) {}
}
