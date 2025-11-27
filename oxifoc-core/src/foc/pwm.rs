//! Abstractions for phase PWM hardware
//!
//! Hardware crates implement these traits to expose a shared API to the
//! `FocController` while keeping platform-specific register writes outside of
//! `oxifoc-core`.

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
