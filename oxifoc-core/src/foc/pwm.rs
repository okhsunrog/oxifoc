//! Abstractions for phase PWM hardware
//!
//! Hardware crates implement these traits to expose a shared API to the
//! `FocController` while keeping platform-specific register writes outside of
//! `oxifoc-core`.
use super::svpwm;

/// Generic three-phase PWM interface
pub trait PhasePwm {
    /// Maximum duty value supported by the timer (e.g. ARR value)
    fn max_duty(&self) -> u16;

    /// Apply three phase duties
    ///
    /// Duties are expected to be pre-clamped to `0..=max_duty()`.
    fn set_duties(&mut self, duties: [u16; 3]);

    /// Disable all phases (optional)
    ///
    /// Implementers can override to turn off outputs or enter a safe state.
    fn disable(&mut self) {
        // Default: no-op for platforms that handle safety elsewhere.
    }
}

/// Modulation strategy (SVPWM, sine PWM, etc.)
pub trait Modulator {
    /// Convert stationary-frame voltages (normalized to ±1.0) into duty cycles.
    fn to_duties(alpha: f32, beta: f32, max_duty: u16) -> [u16; 3];
}

/// Default SVPWM modulator using the VESC geometric sector detector.
pub struct SvpwmModulator;

impl Modulator for SvpwmModulator {
    #[inline]
    fn to_duties(alpha: f32, beta: f32, max_duty: u16) -> [u16; 3] {
        svpwm::space_vector_pwm(alpha, beta, max_duty)
    }
}
