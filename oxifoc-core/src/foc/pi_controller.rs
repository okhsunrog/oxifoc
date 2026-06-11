//! PI Controller for Field-Oriented Control
//!
//! Two controller types for different anti-windup strategies:
//!
//! - [`PIController`] — raw P+I output with external anti-windup via
//!   [`apply_anti_windup()`](PIController::apply_anti_windup). Used inside
//!   [`FocController`](super::controller::FocController) where circular
//!   voltage clamping handles saturation.
//!
//! - [`ClampedPI`] — self-contained PI with rectangular output limits and
//!   internal back-calculation anti-windup. For standalone use (velocity
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
#[derive(Debug)]
pub struct PIController {
    /// Proportional gain
    kp: f32,
    /// Integral gain
    ki: f32,
    /// Accumulated integral
    integral: f32,
    /// Previous error for trapezoidal integration
    prev_error: f32,
}

impl PIController {
    /// Create a new PI controller with the given gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            prev_error: 0.0,
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
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term (trapezoidal / Tustin integration)
        self.integral += self.ki * (error + self.prev_error) * 0.5 * dt;
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
    pub fn apply_anti_windup(&mut self, saturation: f32) {
        self.integral += saturation;
    }

    /// Reset the integral term to zero
    ///
    /// Call this when:
    /// - Motor is disabled
    /// - Starting a new control sequence
    /// - Switching control modes
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// Get current integral value (for debugging/telemetry)
    pub fn get_integral(&self) -> f32 {
        self.integral
    }

    /// Update controller gains at runtime
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Current gains (kp, ki)
    pub fn gains(&self) -> (f32, f32) {
        (self.kp, self.ki)
    }
}

/// PI Controller with internal rectangular clamping and anti-windup
///
/// Wraps [`PIController`] and adds output limits with back-calculation
/// anti-windup. Use this for standalone control loops (velocity, thermal, etc.)
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
pub struct ClampedPI {
    pi: PIController,
    min: f32,
    max: f32,
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
        Self {
            pi: PIController::new(kp, ki),
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
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let raw = self.pi.update(setpoint, measurement, dt);
        let min = if self.min.is_nan() {
            f32::NEG_INFINITY
        } else {
            self.min
        };
        let max = if self.max.is_nan() {
            f32::INFINITY
        } else {
            self.max
        };
        let (min, max) = if min <= max { (min, max) } else { (max, min) };
        let clamped = crate::foc::clamp_f32(raw, min, max);

        // Back-calculation anti-windup
        if raw != clamped {
            self.pi.apply_anti_windup(clamped - raw);
        }

        clamped
    }

    /// Update output limits
    pub fn set_limits(&mut self, min: f32, max: f32) {
        self.min = min;
        self.max = max;
    }

    /// Reset the integral term to zero
    pub fn reset(&mut self) {
        self.pi.reset();
    }

    /// Get current integral value (for debugging/telemetry)
    pub fn get_integral(&self) -> f32 {
        self.pi.get_integral()
    }

    /// Update controller gains at runtime
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.pi.set_gains(kp, ki);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.001;
    const DT: f32 = 0.0001; // 10kHz control loop

    #[test]
    fn test_pi_zero_error() {
        let mut controller = PIController::new(0.5, 10.0);

        // Zero error should give zero output
        let output = controller.update(5.0, 5.0, DT);
        assert!((output - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_pi_proportional_response() {
        let mut controller = PIController::new(0.5, 0.0); // No integral

        let error = 2.0;
        let output = controller.update(5.0, 3.0, DT);

        // Should be kp * error
        assert!((output - 0.5 * error).abs() < EPSILON);
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
        assert!((controller.get_integral() - expected).abs() < EPSILON);
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
            "PI should converge to setpoint, got {}",
            measurement
        );
    }

    #[test]
    fn test_clamped_pi_output_limits() {
        let mut controller = ClampedPI::new(1.0, 100.0, -5.0, 5.0);

        // Large error should saturate output
        let output = controller.update(100.0, 0.0, DT);

        assert!(
            (-5.0..=5.0).contains(&output),
            "Output should be clamped, got {}",
            output
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
            "Anti-windup should prevent integral buildup: {} vs {}",
            integral_with_antiwindup,
            integral_without_antiwindup
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
        assert!((controller.get_integral() - 0.0).abs() < EPSILON);
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
            "Expected ~{}, got {}",
            expected,
            output2
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
            "Current control should converge, got {} vs {}",
            actual_current,
            target_current
        );
    }
}
