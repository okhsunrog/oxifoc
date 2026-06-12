//! Abstractions for phase PWM hardware
//!
//! Hardware crates implement these traits to expose a shared API to the
//! `FocController` while keeping platform-specific register writes outside of
//! `oxifoc-core`.
//!
//! # Configuration
//!
//! [`MotorPwmConfig`] provides shared PWM configuration with board-specific
//! defaults via the builder pattern. Platform crates use [`dead_time_ticks`]
//! to convert nanoseconds to timer ticks.

use super::svpwm;

/// Phase output state for six-step commutation
///
/// Each phase can be PWM-modulated, forced low, or floating (high-impedance).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PhaseState {
    /// PWM modulated output at given duty cycle count.
    /// Both high-side and low-side are active (with dead time).
    Pwm(u16),
    /// Forced low: low-side ON, high-side OFF (current sink path).
    Low,
    /// Floating / high-impedance: both high-side and low-side OFF.
    Float,
}

/// PWM configuration for motor control
///
/// Provides common configuration parameters for 3-phase complementary PWM.
/// Use the builder methods to customize, or call `default()` for safe defaults.
#[derive(Clone, Copy, Debug)]
pub struct MotorPwmConfig {
    /// PWM switching frequency in Hz (default: 20 kHz)
    pub pwm_freq_hz: u32,
    /// Dead time in nanoseconds (default: 1000 ns = 1 µs)
    pub dead_time_ns: u32,
    /// Maximum allowed duty cycle percentage 0-100 (default: 95%)
    pub max_duty_percent: u8,
}

impl Default for MotorPwmConfig {
    fn default() -> Self {
        Self {
            pwm_freq_hz: 20_000,  // 20 kHz - common for BLDC/PMSM
            dead_time_ns: 1000,   // 1 µs - conservative for most gate drivers
            max_duty_percent: 95, // 95% max to ensure bootstrap cap charging
        }
    }
}

impl MotorPwmConfig {
    /// Create a new config with default values
    pub const fn new() -> Self {
        Self {
            pwm_freq_hz: 20_000,
            dead_time_ns: 1000,
            max_duty_percent: 95,
        }
    }

    /// Get the control loop period (dt) in seconds
    ///
    /// This is the inverse of the PWM frequency, used for PI controller updates.
    #[inline]
    pub const fn dt_s(&self) -> f32 {
        1.0 / self.pwm_freq_hz as f32
    }

    /// Set PWM switching frequency in Hz
    pub const fn with_pwm_freq(mut self, freq_hz: u32) -> Self {
        self.pwm_freq_hz = freq_hz;
        self
    }

    /// Set dead time in nanoseconds
    pub const fn with_dead_time_ns(mut self, ns: u32) -> Self {
        self.dead_time_ns = ns;
        self
    }

    /// Set maximum duty cycle percentage (0-100)
    pub const fn with_max_duty_percent(mut self, percent: u8) -> Self {
        self.max_duty_percent = if percent > 100 { 100 } else { percent };
        self
    }
}

/// Calculate dead-time in timer ticks from nanoseconds
///
/// # Arguments
/// * `dead_time_ns` - Dead time in nanoseconds
/// * `timer_clock_hz` - Timer clock frequency in Hz (e.g., 168_000_000 for STM32F4)
///
/// # Returns
/// Dead time value in timer ticks, suitable for passing to `set_dead_time()`
#[inline]
pub fn dead_time_ticks(dead_time_ns: u32, timer_clock_hz: u32) -> u16 {
    ((u64::from(dead_time_ns) * u64::from(timer_clock_hz)) / 1_000_000_000) as u16
}

/// Calculate duty limit from max_duty and percentage
///
/// # Arguments
/// * `max_duty` - Maximum duty value from timer (ARR)
/// * `max_duty_percent` - Maximum allowed percentage (0-100)
///
/// # Returns
/// Clamped duty value
#[inline]
pub fn duty_limit(max_duty: u16, max_duty_percent: u8) -> u16 {
    (u32::from(max_duty) * u32::from(max_duty_percent.min(100)) / 100) as u16
}

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

    /// Re-enable phase outputs after disable().
    ///
    /// Called when transitioning from Stopped to an active control mode.
    /// Default: no-op (platforms where disable is a no-op don't need this).
    fn enable(&mut self) {
        // Default: no-op for platforms that handle safety elsewhere.
    }

    /// Set individual phase states for six-step commutation.
    ///
    /// Each element controls one phase: \[A, B, C\].
    /// Platform crates should override this for proper floating (hi-Z)
    /// support using `enable(Channel)` / `disable(Channel)`.
    ///
    /// The default implementation falls back to `set_duties()` which
    /// cannot produce true high-impedance.
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        let duties = states.map(|s| match s {
            PhaseState::Pwm(duty) => duty,
            PhaseState::Low | PhaseState::Float => 0,
        });
        self.set_duties(duties);
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
