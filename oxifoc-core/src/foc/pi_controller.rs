//! PI Controller for Field-Oriented Control
//!
//! Implements a Proportional-Integral controller with anti-windup protection.
//! Used for both current control (d-axis, q-axis) and velocity control.
//!
//! Anti-windup is critical for motor control to prevent integral buildup
//! during saturation (e.g., when voltage limits are reached).
//!
//! Based on foc-calebfletcher reference implementation, adapted to f32.

/// PI Controller with anti-windup
///
/// The controller implements:
/// - Proportional term: kp * error
/// - Integral term: ki * ∫error dt
/// - Anti-windup: Back-calculation method to prevent integral buildup
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::pi_controller::PIController;
///
/// let mut controller = PIController::new(0.5, 10.0)
///     .with_limits(-24.0, 24.0); // Voltage limits
///
/// let dt = 0.0001; // 10kHz control loop
/// let setpoint = 5.0; // Target current
/// let measurement = 4.8; // Measured current
///
/// let output = controller.update(setpoint, measurement, dt);
/// assert!(output > 0.0 && output <= 24.0);
/// ```
pub struct PIController {
    /// Proportional gain
    kp: f32,
    /// Integral gain
    ki: f32,
    /// Accumulated integral
    integral: f32,
    /// Previous error for trapezoidal integration
    prev_error: f32,
    /// Minimum output limit (None = no limit)
    min_limit: Option<f32>,
    /// Maximum output limit (None = no limit)
    max_limit: Option<f32>,
}

impl PIController {
    /// Create a new PI controller with the given gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    ///
    /// # Example
    /// ```rust
    /// use oxifoc_core::foc::pi_controller::PIController;
    ///
    /// // Current controller gains (typical values)
    /// let current_pi = PIController::new(0.5, 10.0);
    ///
    /// // Velocity controller gains (typical values)
    /// let velocity_pi = PIController::new(0.01, 0.1);
    ///
    /// assert_eq!(current_pi.get_integral(), 0.0);
    /// assert_eq!(velocity_pi.get_integral(), 0.0);
    /// ```
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            prev_error: 0.0,
            min_limit: None,
            max_limit: None,
        }
    }

    /// Set output limits for anti-windup protection
    ///
    /// When output saturates, the integral term will be adjusted to prevent
    /// windup using the back-calculation method.
    ///
    /// # Arguments
    /// * `min` - Minimum output value
    /// * `max` - Maximum output value
    ///
    /// # Example
    /// ```rust
    /// use oxifoc_core::foc::pi_controller::PIController;
    ///
    /// let mut controller = PIController::new(0.5, 10.0)
    ///     .with_limits(-24.0, 24.0); // ±24V for a 24V bus
    ///
    /// let limited_output = controller.update(0.0, -100.0, 0.001);
    /// assert!(limited_output >= -24.0 && limited_output <= 24.0);
    /// ```
    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.min_limit = Some(min);
        self.max_limit = Some(max);
        self
    }

    /// Update the PI controller with a new measurement
    ///
    /// # Arguments
    /// * `setpoint` - Desired value
    /// * `measurement` - Current measured value
    /// * `dt` - Time step (seconds) since last update
    ///
    /// # Returns
    /// Controller output (clamped to limits if set)
    ///
    /// # Example
    /// ```rust
    /// use oxifoc_core::foc::pi_controller::PIController;
    ///
    /// let mut controller = PIController::new(0.5, 10.0);
    /// let dt = 0.0001; // 10kHz loop
    /// let output = controller.update(5.0, 4.8, dt);
    /// assert!(output > 0.0);
    /// ```
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        // Calculate error
        let error = setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term (trapezoidal / Tustin integration)
        self.integral += self.ki * (error + self.prev_error) * 0.5 * dt;
        self.prev_error = error;

        // Calculate raw output (before limiting)
        let output_raw = p_term + self.integral;

        // Apply limits and anti-windup
        match (self.min_limit, self.max_limit) {
            (Some(min), Some(max)) => {
                let output_clamped = output_raw.clamp(min, max);

                // Anti-windup: back-calculation method
                // If output saturated, reduce integral by the saturation amount
                if output_raw != output_clamped {
                    let saturation = output_clamped - output_raw;
                    self.integral += saturation;
                }

                output_clamped
            }
            _ => output_raw,
        }
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

    /// Apply external anti-windup correction to the integral term.
    ///
    /// Used when voltage limiting is applied externally (e.g., circular
    /// voltage clamping in FOC). The saturation signal is the difference
    /// between the clamped and unclamped output: `v_clamped - v_raw`.
    #[inline]
    pub fn apply_anti_windup(&mut self, saturation: f32) {
        self.integral += saturation;
    }

    /// Update controller gains at runtime
    ///
    /// Useful for:
    /// - Adaptive control
    /// - Gain scheduling based on operating point
    /// - Manual tuning via telemetry
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
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
    fn test_pi_output_limits() {
        let mut controller = PIController::new(1.0, 100.0).with_limits(-5.0, 5.0);

        // Large error should saturate output
        let output = controller.update(100.0, 0.0, DT);

        assert!(
            (-5.0..=5.0).contains(&output),
            "Output should be clamped, got {}",
            output
        );
    }

    #[test]
    fn test_pi_anti_windup() {
        let mut controller = PIController::new(0.5, 100.0).with_limits(-10.0, 10.0);

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
    fn test_pi_realistic_current_control() {
        // Simulate realistic current control scenario with well-tuned gains
        let mut controller = PIController::new(2.0, 100.0).with_limits(-24.0, 24.0); // 24V bus

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
