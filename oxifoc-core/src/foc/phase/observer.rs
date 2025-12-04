//! Sensorless observers for FOC control
//!
//! Provides software-based angle estimation for sensorless motor control.
//! Includes back-EMF observer for medium/high speed and HFI for low/zero speed.

use core::f32::consts::TAU;

/// Input for observer update
#[derive(Clone, Copy, Debug, Default)]
pub struct ObserverInput {
    /// α-axis voltage (V)
    pub v_alpha: f32,
    /// β-axis voltage (V)
    pub v_beta: f32,
    /// α-axis current (A)
    pub i_alpha: f32,
    /// β-axis current (A)
    pub i_beta: f32,
    /// Time step (seconds)
    pub dt: f32,
}

/// Runtime-switchable observer implementations
#[derive(Clone, Debug)]
pub enum Observer {
    /// No observer configured
    None,
    /// Back-EMF flux observer (VESC-style)
    BackEmf(BackEmfObserver),
    /// High-frequency injection observer
    Hfi(HfiObserver),
}

#[allow(clippy::derivable_impls)] // Other variants have data, can't derive
impl Default for Observer {
    fn default() -> Self {
        Observer::None
    }
}

impl Observer {
    /// Update observer with new measurements
    pub fn update(&mut self, input: &ObserverInput) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => o.update(input),
            Observer::Hfi(o) => o.update(input),
        }
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.phase()),
            Observer::Hfi(o) => Some(o.phase()),
        }
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> Option<f32> {
        match self {
            Observer::None => None,
            Observer::BackEmf(o) => Some(o.velocity()),
            Observer::Hfi(o) => Some(o.velocity()),
        }
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        match self {
            Observer::None => 0.0,
            Observer::BackEmf(o) => o.confidence(),
            Observer::Hfi(o) => o.confidence(),
        }
    }

    /// Check if observer is configured
    pub fn is_configured(&self) -> bool {
        !matches!(self, Observer::None)
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        match self {
            Observer::None => {}
            Observer::BackEmf(o) => o.reset(),
            Observer::Hfi(o) => o.reset(),
        }
    }
}

// ============================================================================
// Back-EMF Observer (VESC-style flux observer)
// ============================================================================

/// Back-EMF flux observer for sensorless FOC
///
/// Based on VESC's flux observer implementation. Estimates rotor position
/// by integrating back-EMF voltage to obtain flux linkage, then uses
/// a PLL to extract phase and velocity.
///
/// Works well at medium to high speeds where back-EMF is measurable.
/// At low speeds, HFI should be used instead.
#[derive(Clone, Debug)]
pub struct BackEmfObserver {
    // Flux integrator state
    x1: f32, // α-axis flux estimate
    x2: f32, // β-axis flux estimate

    // PLL state
    phase_pll: f32,    // PLL-filtered phase
    velocity_pll: f32, // PLL-filtered velocity

    // Motor parameters
    r: f32,      // Phase resistance (Ω)
    l: f32,      // Phase inductance (H)
    lambda: f32, // Flux linkage (Wb)

    // Observer tuning
    gain: f32,   // Observer gain
    pll_kp: f32, // PLL proportional gain
    pll_ki: f32, // PLL integral gain

    // State
    confidence: f32, // Confidence estimate (0-1)
}

impl BackEmfObserver {
    /// Create a new back-EMF observer with motor parameters
    ///
    /// # Arguments
    /// * `r` - Phase resistance (Ω)
    /// * `l` - Phase inductance (H)
    /// * `lambda` - Flux linkage (Wb)
    pub fn new(r: f32, l: f32, lambda: f32) -> Self {
        // Observer gain: 1e-3 / λ² (VESC formula)
        let gain = if lambda > 0.0 {
            (1e-3 / (lambda * lambda)).clamp(1e3, 1e9)
        } else {
            1e6
        };

        Self {
            x1: 0.0,
            x2: 0.0,
            phase_pll: 0.0,
            velocity_pll: 0.0,
            r,
            l,
            lambda: lambda.max(1e-6),
            gain,
            pll_kp: 1000.0,
            pll_ki: 20000.0,
            confidence: 0.0,
        }
    }

    /// Update observer with new measurements
    pub fn update(&mut self, input: &ObserverInput) {
        let dt = input.dt;
        if dt <= 0.0 {
            return;
        }

        // Back-EMF estimation: e = v - R*i - L*di/dt
        // For flux observer: dψ/dt = v - R*i
        // ψ = ∫(v - R*i)dt

        // Flux integrator (simplified - full implementation includes correction term)
        let e_alpha = input.v_alpha - self.r * input.i_alpha;
        let e_beta = input.v_beta - self.r * input.i_beta;

        self.x1 += e_alpha * dt;
        self.x2 += e_beta * dt;

        // Limit flux magnitude to prevent runaway
        let flux_mag = libm::sqrtf(self.x1 * self.x1 + self.x2 * self.x2);
        let max_flux = self.lambda * 2.0;
        if flux_mag > max_flux {
            let scale = max_flux / flux_mag;
            self.x1 *= scale;
            self.x2 *= scale;
        }

        // Extract phase from flux
        let phase_raw = libm::atan2f(self.x2, self.x1);

        // PLL tracking
        let phase_error = wrap_angle(phase_raw - self.phase_pll);
        self.velocity_pll += self.pll_ki * phase_error * dt;
        self.phase_pll += (self.velocity_pll + self.pll_kp * phase_error) * dt;
        self.phase_pll = wrap_angle(self.phase_pll);

        // Update confidence based on flux magnitude
        let flux_ratio = flux_mag / self.lambda;
        self.confidence = flux_ratio.clamp(0.0, 1.0);
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> f32 {
        self.phase_pll
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> f32 {
        self.velocity_pll
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.phase_pll = 0.0;
        self.velocity_pll = 0.0;
        self.confidence = 0.0;
    }

    /// Set motor parameters
    pub fn set_motor_params(&mut self, r: f32, l: f32, lambda: f32) {
        self.r = r;
        self.l = l;
        self.lambda = lambda.max(1e-6);
        self.gain = (1e-3 / (self.lambda * self.lambda)).clamp(1e3, 1e9);
    }

    /// Set PLL gains
    pub fn set_pll_gains(&mut self, kp: f32, ki: f32) {
        self.pll_kp = kp;
        self.pll_ki = ki;
    }

    /// Force phase to specific value (for testing or handoff from other source)
    pub fn force_phase(&mut self, phase: f32) {
        self.phase_pll = wrap_angle(phase);
        // Also set flux state to match
        self.x1 = self.lambda * libm::cosf(phase);
        self.x2 = self.lambda * libm::sinf(phase);
    }

    /// Set velocity estimate (for testing or handoff from other source)
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity_pll = velocity;
    }
}

// ============================================================================
// HFI Observer (High-Frequency Injection)
// ============================================================================

/// High-frequency injection observer for low/zero speed sensorless
///
/// Injects a high-frequency voltage signal and measures the current response
/// to estimate rotor position based on magnetic saliency (Ld ≠ Lq).
///
/// Works at zero and low speeds where back-EMF is too small to measure.
#[derive(Clone, Debug)]
pub struct HfiObserver {
    // Injection parameters
    frequency: f32, // Injection frequency (Hz)
    amplitude: f32, // Injection amplitude (V)
    phase_inj: f32, // Current injection phase

    // Demodulation state
    phase_est: f32,    // Estimated rotor phase
    velocity_est: f32, // Estimated velocity

    // PLL state
    pll_integrator: f32,

    // Tuning
    pll_kp: f32,
    pll_ki: f32,

    // State
    confidence: f32,
}

impl HfiObserver {
    /// Create a new HFI observer
    ///
    /// # Arguments
    /// * `frequency` - Injection frequency (Hz), typically 500-2000 Hz
    /// * `amplitude` - Injection voltage amplitude (V)
    pub fn new(frequency: f32, amplitude: f32) -> Self {
        Self {
            frequency,
            amplitude,
            phase_inj: 0.0,
            phase_est: 0.0,
            velocity_est: 0.0,
            pll_integrator: 0.0,
            pll_kp: 100.0,
            pll_ki: 2000.0,
            confidence: 0.0,
        }
    }

    /// Update observer with new measurements
    ///
    /// Note: Full HFI implementation requires:
    /// 1. Injecting Vd = amplitude * sin(2π * frequency * t)
    /// 2. Measuring Id response
    /// 3. Demodulating to extract position error
    ///
    /// This is a placeholder - actual implementation needs integration
    /// with the FOC controller's injection mode.
    pub fn update(&mut self, input: &ObserverInput) {
        let dt = input.dt;
        if dt <= 0.0 {
            return;
        }

        // Advance injection phase
        self.phase_inj += TAU * self.frequency * dt;
        if self.phase_inj > TAU {
            self.phase_inj -= TAU;
        }

        // TODO: Full HFI demodulation requires:
        // - Current injection from FOC controller
        // - Band-pass filtering of current response
        // - Demodulation to extract position error signal
        // - PLL tracking of position

        // Placeholder: just advance based on velocity
        self.phase_est += self.velocity_est * dt;
        self.phase_est = wrap_angle(self.phase_est);
    }

    /// Get estimated electrical phase (radians)
    pub fn phase(&self) -> f32 {
        self.phase_est
    }

    /// Get estimated electrical velocity (rad/s)
    pub fn velocity(&self) -> f32 {
        self.velocity_est
    }

    /// Get observer confidence (0.0-1.0)
    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    /// Reset observer state
    pub fn reset(&mut self) {
        self.phase_inj = 0.0;
        self.phase_est = 0.0;
        self.velocity_est = 0.0;
        self.pll_integrator = 0.0;
        self.confidence = 0.0;
    }

    /// Get injection voltage for current step
    ///
    /// Returns (vd_inject, vq_inject) to be added to FOC output.
    /// Call this at the PWM frequency to get the injection signal.
    pub fn get_injection(&self) -> (f32, f32) {
        let vd = self.amplitude * libm::sinf(self.phase_inj);
        (vd, 0.0)
    }

    /// Set injection parameters
    pub fn set_injection(&mut self, frequency: f32, amplitude: f32) {
        self.frequency = frequency;
        self.amplitude = amplitude;
    }

    /// Set PLL gains
    pub fn set_pll_gains(&mut self, kp: f32, ki: f32) {
        self.pll_kp = kp;
        self.pll_ki = ki;
    }

    /// Set initial phase estimate (for handoff from other source)
    pub fn set_phase(&mut self, phase: f32) {
        self.phase_est = wrap_angle(phase);
    }

    /// Set initial velocity estimate (for handoff from other source)
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity_est = velocity;
    }
}

// ============================================================================
// Utility functions
// ============================================================================

/// Wrap angle to [0, 2π)
#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a < 0.0 {
        a += TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observer_none() {
        let obs = Observer::None;
        assert!(!obs.is_configured());
        assert!(obs.phase().is_none());
        assert!(obs.velocity().is_none());
        assert_eq!(obs.confidence(), 0.0);
    }

    #[test]
    fn test_back_emf_observer_creation() {
        let obs = BackEmfObserver::new(0.1, 0.0001, 0.01);
        assert_eq!(obs.phase(), 0.0);
        assert_eq!(obs.velocity(), 0.0);
        assert_eq!(obs.confidence(), 0.0);
    }

    #[test]
    fn test_hfi_observer_creation() {
        let obs = HfiObserver::new(1000.0, 3.0);
        assert_eq!(obs.phase(), 0.0);
        assert_eq!(obs.velocity(), 0.0);
    }

    #[test]
    fn test_observer_enum() {
        let mut obs = Observer::BackEmf(BackEmfObserver::new(0.1, 0.0001, 0.01));
        assert!(obs.is_configured());
        assert!(obs.phase().is_some());

        obs.reset();
        assert_eq!(obs.phase(), Some(0.0));
    }
}
