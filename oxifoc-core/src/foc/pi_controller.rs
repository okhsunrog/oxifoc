//! PI Controller for Field-Oriented Control
//!
//! Two controller types for different anti-windup strategies:
//!
//! - [`PIController`](crate::foc::pi_controller::PIController) — raw P+I output with external anti-windup via
//!   [`apply_anti_windup()`](crate::foc::pi_controller::PIController::apply_anti_windup). Used inside
//!   [`FocController`](crate::foc::controller::FocController) where circular
//!   voltage clamping handles saturation.
//!
//! - [`ClampedPI`](crate::foc::pi_controller::ClampedPI) — self-contained PI with rectangular output limits and
//!   conditional integration and bounded integral state. For standalone use (velocity
//!   loops, thermal controllers, etc.).
//!
//! The two types exist to prevent dual anti-windup: if a PI controller
//! had both internal limits *and* external circular clamping, the integral
//! would be double-corrected, causing sluggish or oscillatory behavior.
//!
//! Based on foc-calebfletcher reference implementation, adapted to f32.

/// PI Controller with external anti-windup
///
/// Returns raw (unclamped) output. Anti-windup is applied externally
/// via [`apply_anti_windup()`](Self::apply_anti_windup) after the caller's
/// own saturation logic (e.g., circular voltage clamping in FOC).
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::pi_controller::PIController;
///
/// let mut pi = PIController::new(0.5, 10.0);
/// let raw = pi.update(5.0, 4.8, 0.0001);
///
/// // Caller applies its own clamping, then feeds back saturation:
/// let clamped = raw.clamp(-24.0, 24.0);
/// pi.apply_anti_windup(clamped - raw);
/// ```
use core::fmt::Debug;
use core::ops::{Add, Sub};

use crate::units::{ConstZero, Scalar};

/// Units of a PI regulator: the error and output types, the time step and
/// the gains. The two products are methods, so each implementation checks
/// its own dimension algebra (for example A per electrical rad/s times
/// electrical rad/s is A) while the regulator code stays generic.
pub trait PiUnits {
    type Error: Scalar + Debug + Sub<Output = Self::Error> + Add<Output = Self::Error>;
    type Output: Scalar + Debug + Add<Output = Self::Output> + Sub<Output = Self::Output>;
    type Time: Copy + Debug;
    type Kp: Copy + Debug;
    type Ki: Scalar + Debug;
    /// `kp · error`
    fn proportional(kp: Self::Kp, error: Self::Error) -> Self::Output;
    /// One trapezoidal integration step: `ki · (e + e_prev) · 0.5 · dt`.
    fn integral(ki: Self::Ki, error_sum: Self::Error, dt: Self::Time) -> Self::Output;
}

/// Unitless `f32` regulator (current loop, PLLs).
#[derive(Clone, Copy, Debug)]
pub enum PlainPi {}

impl PiUnits for PlainPi {
    type Error = f32;
    type Output = f32;
    type Time = f32;
    type Kp = f32;
    type Ki = f32;
    #[inline(always)]
    fn proportional(kp: f32, error: f32) -> f32 {
        kp * error
    }
    #[inline(always)]
    fn integral(ki: f32, error_sum: f32, dt: f32) -> f32 {
        ki * error_sum * 0.5 * dt
    }
}

#[derive(Debug)]
pub struct PIController<U: PiUnits = PlainPi> {
    /// Proportional gain
    kp: U::Kp,
    /// Integral gain
    ki: U::Ki,
    /// Accumulated integral
    integral: U::Output,
    /// Previous error for trapezoidal integration
    prev_error: U::Error,
}

impl PIController {
    /// Create a new PI controller with the given gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub fn new(kp: f32, ki: f32) -> Self {
        Self::with_gains(kp, ki)
    }
}

impl<U: PiUnits> PIController<U> {
    /// [`new`](PIController::new) with typed gains.
    pub fn with_gains(kp: U::Kp, ki: U::Ki) -> Self {
        Self {
            kp,
            ki,
            integral: U::Output::ZERO,
            prev_error: U::Error::ZERO,
        }
    }

    /// Compute one PI step, returning the raw (unclamped) output.
    ///
    /// Uses trapezoidal (Tustin) integration for the integral term.
    ///
    /// # Arguments
    /// * `setpoint` - Desired value
    /// * `measurement` - Current measured value
    /// * `dt` - Time step (seconds) since last update
    pub fn update(&mut self, setpoint: U::Error, measurement: U::Error, dt: U::Time) -> U::Output {
        let error = setpoint - measurement;

        // Proportional term
        let p_term = U::proportional(self.kp, error);

        // Integral term (trapezoidal / Tustin integration)
        if self.ki == U::Ki::ZERO {
            self.integral = U::Output::ZERO;
        } else {
            self.integral = self.integral + U::integral(self.ki, error + self.prev_error, dt);
        }
        self.prev_error = error;

        p_term + self.integral
    }

    /// Apply external anti-windup correction to the integral term.
    ///
    /// Call this after your saturation logic with the difference between
    /// the clamped and unclamped output: `v_clamped - v_raw`.
    ///
    /// Used by FOC circular voltage clamping.
    #[inline]
    pub fn apply_anti_windup(&mut self, saturation: U::Output) {
        if self.ki != U::Ki::ZERO {
            self.integral = self.integral + saturation;
        }
    }

    /// Reset the integral term to zero
    ///
    /// Call this when:
    /// - Motor is disabled
    /// - Starting a new control sequence
    /// - Switching control modes
    pub fn reset(&mut self) {
        self.integral = U::Output::ZERO;
        self.prev_error = U::Error::ZERO;
    }

    /// Get current integral value (for debugging/telemetry)
    pub fn get_integral(&self) -> U::Output {
        self.integral
    }

    /// Restart with the integral at `integral`: at zero error the output
    /// continues from there (bumpless takeover of an existing command).
    /// Ignored when `ki == 0`: proportional-only control has no stored bias.
    pub fn preload(&mut self, integral: U::Output) {
        self.integral = if self.ki == U::Ki::ZERO {
            U::Output::ZERO
        } else {
            integral
        };
        self.prev_error = U::Error::ZERO;
    }

    /// Update controller gains at runtime, clearing state when I is disabled.
    pub fn set_gains(&mut self, kp: U::Kp, ki: U::Ki) {
        self.kp = kp;
        self.ki = ki;
        if ki == U::Ki::ZERO {
            self.reset();
        }
    }

    /// Current gains (kp, ki)
    pub fn gains(&self) -> (U::Kp, U::Ki) {
        (self.kp, self.ki)
    }
}

/// PI Controller with internal rectangular clamping and anti-windup
///
/// Wraps [`PIController`] and adds output limits with conditional integration.
/// Use this for standalone control loops (velocity, thermal, etc.)
/// where there is no external saturation logic.
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::pi_controller::ClampedPI;
///
/// let mut pi = ClampedPI::new(0.5, 10.0, -24.0, 24.0);
/// let output = pi.update(5.0, 4.8, 0.0001);
/// assert!(output >= -24.0 && output <= 24.0);
/// ```
#[derive(Debug)]
pub struct ClampedPI<U: PiUnits = PlainPi> {
    pi: PIController<U>,
    min: U::Output,
    max: U::Output,
}

impl ClampedPI {
    /// Create a new clamped PI controller
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `min` - Minimum output value
    /// * `max` - Maximum output value
    pub fn new(kp: f32, ki: f32, min: f32, max: f32) -> Self {
        Self::with_gains(kp, ki, min, max)
    }
}

impl<U: PiUnits> ClampedPI<U> {
    /// [`new`](ClampedPI::new) with typed gains and limits.
    pub fn with_gains(kp: U::Kp, ki: U::Ki, min: U::Output, max: U::Output) -> Self {
        Self {
            pi: PIController::with_gains(kp, ki),
            min,
            max,
        }
    }

    /// Compute one PI step with clamping and internal anti-windup.
    ///
    /// # Arguments
    /// * `setpoint` - Desired value
    /// * `measurement` - Current measured value
    /// * `dt` - Time step (seconds) since last update
    ///
    /// # Returns
    /// Controller output clamped to `[min, max]`
    pub fn update(&mut self, setpoint: U::Error, measurement: U::Error, dt: U::Time) -> U::Output {
        let integral_before = self.pi.integral;
        let raw = self.pi.update(setpoint, measurement, dt);
        let min = if Scalar::is_nan(self.min) {
            U::Output::NEG_INFINITY
        } else {
            self.min
        };
        let max = if Scalar::is_nan(self.max) {
            U::Output::INFINITY
        } else {
            self.max
        };
        let (min, max) = if min <= max { (min, max) } else { (max, min) };
        // Do not integrate further into a saturated output. In particular,
        // clipping a negative P term at a zero braking limit must not CREATE
        // positive integral to cancel it: declining overspeed then requests
        // motoring torque while the rotor is still above its target.
        let delta = self.pi.integral - integral_before;
        if (raw > max && delta > U::Output::ZERO) || (raw < min && delta < U::Output::ZERO) {
            self.pi.integral = integral_before;
        }
        // A shrinking actuator budget must also shed already stored authority.
        // Pure P control never stores state, including with nonzero limits.
        if self.pi.ki != U::Ki::ZERO {
            self.pi.integral = self.pi.integral.clamp_to(min, max);
        }
        (U::proportional(self.pi.kp, setpoint - measurement) + self.pi.integral).clamp_to(min, max)
    }

    /// Update output limits
    pub fn set_limits(&mut self, min: U::Output, max: U::Output) {
        self.min = min;
        self.max = max;
    }

    /// Track an external actuator's output residual once per controller step.
    pub(crate) fn track_output(&mut self, correction: U::Output) {
        self.pi.apply_anti_windup(correction);
    }

    /// Reset the integral term to zero
    pub fn reset(&mut self) {
        self.pi.reset();
    }

    /// See [`PIController::preload`].
    pub fn preload(&mut self, integral: U::Output) {
        self.pi.preload(integral);
    }

    /// Get current integral value (for debugging/telemetry)
    pub fn get_integral(&self) -> U::Output {
        self.pi.get_integral()
    }

    /// Update controller gains at runtime
    pub fn set_gains(&mut self, kp: U::Kp, ki: U::Ki) {
        self.pi.set_gains(kp, ki);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.001;
    const DT: f32 = 0.0001; // 10kHz control loop

    #[test]
    fn unavailable_braking_does_not_create_motoring_integral() {
        for direction in [-1.0, 1.0] {
            let (min, max) = if direction > 0.0 {
                (0.0, 3.0)
            } else {
                (-3.0, 0.0)
            };
            let mut pi = ClampedPI::new(0.01, 0.2, min, max);
            // The load coasts faster than the request. Smaller overspeed is
            // still overspeed, not a request to accelerate back to its peak.
            for speed in [80.0, 60.0, 70.0, 50.0, 45.0, 40.0] {
                let output = pi.update(direction * 40.0, direction * speed, 0.001);
                assert_eq!(output, 0.0, "speed={speed}, integral={}", pi.get_integral());
            }
            assert_eq!(pi.get_integral(), 0.0);
        }
    }

    #[test]
    fn shrinking_authority_removes_unreachable_integral() {
        let mut pi = ClampedPI::new(0.01, 0.2, -3.0, 3.0);
        pi.preload(2.0);
        pi.set_limits(0.0, 0.5);
        assert_eq!(pi.update(40.0, 40.0, 0.001), 0.5);
        assert!(pi.get_integral() <= 0.5);
    }

    #[test]
    fn proportional_only_saturation_does_not_store_a_bias() {
        let mut pi = ClampedPI::new(1.0, 0.0, -1.0, 1.0);
        assert_eq!(pi.update(10.0, 0.0, DT), 1.0);
        for _ in 0..10 {
            assert_eq!(pi.update(10.0, 10.0, DT), 0.0);
        }
        assert_eq!(pi.get_integral(), 0.0);
    }

    #[test]
    fn disabling_integral_gain_clears_preload_and_anti_windup() {
        let mut pi = PIController::new(1.0, 1.0);
        pi.preload(5.0);
        pi.set_gains(1.0, 0.0);
        pi.preload(3.0);
        pi.apply_anti_windup(-10.0);
        assert_eq!(pi.get_integral(), 0.0);
        assert_eq!(pi.update(1.0, 1.0, DT), 0.0);
    }

    #[test]
    fn test_pi_zero_error() {
        let mut controller = PIController::new(0.5, 10.0);

        // Zero error should give zero output
        let output = controller.update(5.0, 5.0, DT);
        assert_near!(output, 0.0, EPSILON);
    }

    #[test]
    fn test_pi_proportional_response() {
        let mut controller = PIController::new(0.5, 0.0); // No integral

        let error = 2.0;
        let output = controller.update(5.0, 3.0, DT);

        // Should be kp * error
        assert_near!(output, 0.5 * error, EPSILON);
    }

    #[test]
    fn test_pi_integral_accumulation() {
        let mut controller = PIController::new(0.0, 10.0); // No proportional

        let error = 1.0;
        let steps = 10;

        for _ in 0..steps {
            controller.update(6.0, 5.0, DT);
        }

        // Trapezoidal integration with constant error:
        // First step: ki * (error + 0) / 2 * dt = half contribution
        // Steps 2..N: ki * (error + error) / 2 * dt = full contribution
        // Total = ki * error * dt * (steps - 0.5)
        let expected = 10.0 * error * DT * (steps as f32 - 0.5);
        assert_near!(controller.get_integral(), expected, EPSILON);
    }

    #[test]
    fn test_pi_convergence() {
        // Use well-tuned gains for a first-order system
        let mut controller = PIController::new(2.0, 50.0);

        let setpoint = 10.0;
        let mut measurement = 0.0;

        // First-order plant with time constant
        let tau = 0.01; // Time constant

        // Simulate first-order system
        for _ in 0..3000 {
            let output = controller.update(setpoint, measurement, DT);
            // First-order plant: dm/dt = (output - measurement) / tau
            measurement += (output - measurement) / tau * DT;
        }

        // Should converge close to setpoint with well-tuned gains
        assert!(
            (measurement - setpoint).abs() < 0.2,
            "PI should converge to setpoint, got {measurement}"
        );
    }

    #[test]
    fn test_clamped_pi_output_limits() {
        let mut controller = ClampedPI::new(1.0, 100.0, -5.0, 5.0);

        // Large error should saturate output
        let output = controller.update(100.0, 0.0, DT);

        assert!(
            (-5.0..=5.0).contains(&output),
            "Output should be clamped, got {output}"
        );
    }

    #[test]
    fn test_clamped_pi_anti_windup() {
        let mut controller = ClampedPI::new(0.5, 100.0, -10.0, 10.0);

        // Apply large error for many steps (would cause windup without anti-windup)
        for _ in 0..100 {
            controller.update(50.0, 0.0, DT);
        }

        let integral_with_antiwindup = controller.get_integral();

        // Now test without limits (windup will occur)
        let mut controller_no_limit = PIController::new(0.5, 100.0);
        for _ in 0..100 {
            controller_no_limit.update(50.0, 0.0, DT);
        }

        let integral_without_antiwindup = controller_no_limit.get_integral();

        // Anti-windup should keep integral much smaller
        assert!(
            integral_with_antiwindup.abs() < integral_without_antiwindup.abs(),
            "Anti-windup should prevent integral buildup: {integral_with_antiwindup} vs {integral_without_antiwindup}"
        );
    }

    #[test]
    fn test_pi_reset() {
        let mut controller = PIController::new(0.5, 10.0);

        // Accumulate some integral
        for _ in 0..10 {
            controller.update(5.0, 0.0, DT);
        }

        assert!(controller.get_integral() > 0.0);

        // Reset should clear integral
        controller.reset();
        assert_near!(controller.get_integral(), 0.0, EPSILON);
    }

    #[test]
    fn test_pi_negative_error() {
        let mut controller = PIController::new(0.5, 10.0);

        // Negative error (measurement > setpoint)
        let output = controller.update(3.0, 5.0, DT);

        // Output should be negative
        assert!(output < 0.0, "Negative error should give negative output");
    }

    #[test]
    fn test_pi_gain_update() {
        let mut controller = PIController::new(0.5, 10.0);

        let output1 = controller.update(5.0, 3.0, DT);

        // Change gains and reset
        controller.set_gains(1.0, 20.0);
        controller.reset();

        let output2 = controller.update(5.0, 3.0, DT);

        // Should be different with new gains
        assert!(
            (output1 - output2).abs() > EPSILON,
            "Gain change should affect output"
        );

        // With reset integral (prev_error=0), output = kp*error + ki*(error+0)/2*dt
        // = 1.0*2.0 + 20.0*2.0/2*0.0001 = 2.0 + 0.002 = 2.002
        let expected = 1.0 * 2.0 + 20.0 * 2.0 * 0.5 * DT;
        assert!(
            (output2 - expected).abs() < 0.01,
            "Expected ~{expected}, got {output2}"
        );
    }

    #[test]
    fn test_clamped_pi_realistic_current_control() {
        // Simulate realistic current control scenario with well-tuned gains
        let mut controller = ClampedPI::new(2.0, 100.0, -24.0, 24.0); // 24V bus

        let target_current = 5.0; // Amps
        let mut actual_current = 0.0;

        // Motor electrical time constant ~ 1ms (L/R)
        let tau = 0.001;

        // Run longer to allow convergence
        for _ in 0..500 {
            let voltage = controller.update(target_current, actual_current, DT);

            // Simple L/R plant: di/dt = (V - R*i) / L
            // Simplified: di/dt = (voltage - current) / tau
            actual_current += (voltage - actual_current) / tau * DT;
        }

        // Should reach target current after 500 steps (50ms)
        // Allow ~7% error due to simplified plant model
        assert!(
            (actual_current - target_current).abs() < 0.35,
            "Current control should converge, got {actual_current} vs {target_current}"
        );
    }
}
