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

/// Direct per-phase output state (brake / coast / float control)
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

/// q-axis PWM modulation used to estimate supply current as `iq * mod_q`.
/// Its only constructor reconstructs the final duty command, so a PI voltage
/// in volts cannot accidentally be passed to the supply-current limiter.
#[derive(Clone, Copy, Debug, Default)]
pub struct QAxisModulation(f32);

impl QAxisModulation {
    pub(crate) fn from_pwm(duties: [u16; 3], period: u16, sin: f32, cos: f32) -> Self {
        let [a, b, c] = duties.map(i32::from);
        // Fold 1.5 * Clarke into the projection. The signed duty differences
        // fit exactly in i32 and f32, including the full u16 timer range.
        Self(
            (super::constants::SQRT_3_DIV_2 * cos * (b - c) as f32
                - 0.5 * sin * (2 * a - b - c) as f32)
                / f32::from(period.max(1)),
        )
    }

    pub fn coefficient(self) -> f32 {
        self.0
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

    /// Highest duty `set_duties` lets through (≤ `max_duty()`): boards cap
    /// the high-side on-time to keep a low-side window for bootstrap
    /// charging and shunt sampling. `FocDriver::new` hands it to the
    /// controller, which limits the voltage and shifts the common mode to
    /// stay under it instead of having the board clip one leg silently.
    /// Default: no cap.
    fn duty_limit(&self) -> u16 {
        self.max_duty()
    }

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

    /// Set individual phase states directly (brake / coast / float).
    ///
    /// Each element controls one phase: \[A, B, C\].
    /// Implementations must provide real high-impedance behavior for
    /// [`PhaseState::Float`]. Duty zero is not equivalent on a complementary
    /// bridge: it normally holds the low-side FET on.
    fn set_phase_states(&mut self, states: [PhaseState; 3]);
}

/// Modulation strategy (SVPWM, sine PWM, etc.)
pub trait Modulator {
    /// Convert a stationary-frame MODULATION vector (`1.5 × volts / vbus`;
    /// linear up to |m| = √3/2) into duty cycles.
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

#[cfg(test)]
mod tests {
    use super::*;

    fn previous_modulation(duties: [u16; 3], period: u16, sin: f32, cos: f32) -> f32 {
        let [a, b, c] = duties.map(f32::from);
        let (alpha, beta) = super::super::transforms::clarke_three(a, b, c);
        1.5 * (cos * beta - sin * alpha) / f32::from(period.max(1))
    }

    fn check_terminal_power(duties: [u16; 3], period: u16, sin: f32, cos: f32) -> (f64, f64) {
        let actual = f64::from(QAxisModulation::from_pwm(duties, period, sin, cos).coefficient());
        let previous = f64::from(previous_modulation(duties, period, sin, cos));
        let [a, b, c] = duties.map(f64::from);
        let period = f64::from(period.max(1));
        let span = (a.max(b).max(c) - a.min(b).min(c)) / period;
        // Absolute coefficient error, scaled by the duty span: <= 4 f32
        // epsilons for any valid duty command, including cancellation at q=0.
        let tolerance = 4.0 * f64::from(f32::EPSILON) * span;
        let difference = (actual - previous).abs();
        assert!(difference <= tolerance, "{duties:?}: old/new {difference}");

        let mut max_power_error = 0.0_f64;
        for iq in [-100.0, -0.01, 0.01, 100.0] {
            // Independently reconstruct q-axis phase currents in f64 and
            // sum their terminal powers, with zero d current/zero sequence.
            let ia = -f64::from(sin) * iq;
            let ib = (0.5 * f64::from(sin) + 0.5 * 3.0_f64.sqrt() * f64::from(cos)) * iq;
            let ic = -ia - ib;
            let bus_current = (a * ia + b * ib + c * ic) / period;
            let error = (actual * iq - bus_current).abs() / iq.abs();
            // f64 terminal summation can leave tiny common-mode residuals.
            assert!(
                error <= tolerance + 1e-14,
                "{duties:?}, period={period}, iq={iq}: power error {error}"
            );
            max_power_error = max_power_error.max(error);
        }
        (max_power_error, difference)
    }

    #[test]
    fn final_pwm_modulation_matches_terminal_power() {
        let mut max_power_error = 0.0_f64;
        let mut max_difference = 0.0_f64;
        for period in [1, 2, 3, 31, 255, 256, 4_199, 10_000, 32_768, u16::MAX] {
            let levels = [0, 1, period / 4, period / 2, period - 1, period];
            for degrees in (0..360).step_by(5) {
                let angle = (degrees as f32).to_radians();
                let (sin, cos) = (libm::sinf(angle), libm::cosf(angle));
                // All duty orderings cover all six voltage sectors, their
                // boundaries, common modes and rail-clamped commands.
                for a in levels {
                    for b in levels {
                        for c in levels {
                            let (error, difference) =
                                check_terminal_power([a, b, c], period, sin, cos);
                            max_power_error = max_power_error.max(error);
                            max_difference = max_difference.max(difference);
                        }
                    }
                }
            }
        }
        println!("max coefficient error {max_power_error:e}; old/new delta {max_difference:e}");
    }

    #[test]
    fn every_timer_period_preserves_normalization() {
        let mut state = 0x73c9_54a1_u32;
        for period in 0..=u16::MAX {
            let duties = core::array::from_fn(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state % (u32::from(period) + 1)) as u16
            });
            let angle = f32::from(period).to_radians();
            check_terminal_power(duties, period, libm::sinf(angle), libm::cosf(angle));
        }
    }

    #[test]
    fn common_mode_does_not_change_modulation() {
        for duties in [[0, 1, 2], [0, 8_192, 42], [129, 17, 8_192]] {
            for degrees in (0..360).step_by(5) {
                let angle = (degrees as f32).to_radians();
                let (sin, cos) = (libm::sinf(angle), libm::cosf(angle));
                let expected = QAxisModulation::from_pwm(duties, u16::MAX, sin, cos).coefficient();
                for shift in [1, 1_000, 32_768, 57_343] {
                    let shifted = duties.map(|duty| duty + shift);
                    let actual =
                        QAxisModulation::from_pwm(shifted, u16::MAX, sin, cos).coefficient();
                    assert_eq!(actual.to_bits(), expected.to_bits());
                }
            }
        }
    }

    #[test]
    fn zero_period_uses_one_tick_normalization() {
        for duties in [[0; 3], [u16::MAX; 3], [0, u16::MAX, 1], [u16::MAX, 0, 1]] {
            let actual = QAxisModulation::from_pwm(duties, 0, -0.6, 0.8).coefficient();
            let expected = QAxisModulation::from_pwm(duties, 1, -0.6, 0.8).coefficient();
            assert_eq!(actual.to_bits(), expected.to_bits());
        }
    }

    #[test]
    fn supply_limit_reference_coefficients_remain_exact() {
        // The limiter's dead-band boundary and its 0.5 modulation examples,
        // reconstructed from final duties in both power-flow directions.
        for (duty, expected) in [(5, 1e-3_f32), (2_500, 0.5)] {
            for direction in [-1.0, 1.0] {
                let actual =
                    QAxisModulation::from_pwm([duty, 0, 0], 5_000, -direction, 0.0).coefficient();
                assert_eq!(actual.to_bits(), (expected * direction).to_bits());
                check_terminal_power([duty, 0, 0], 5_000, -direction, 0.0);
            }
        }
    }
}
