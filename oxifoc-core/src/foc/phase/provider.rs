//! Phase provider trait for FOC control
//!
//! Abstracts electrical phase angle provision, allowing FocDriver to work
//! with any phase source (Hall, Encoder, Observer, or combinations).

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
}
