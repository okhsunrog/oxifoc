//! Space Vector Pulse Width Modulation (SVPWM)
//!
//! SVPWM is the most efficient PWM technique for 3-phase motor control.
//! It provides better DC bus utilization (~15% more) compared to sinusoidal PWM.
//!
//! The algorithm:
//! 1. Determine which of 6 sectors the voltage vector falls in (geometric)
//! 2. Calculate switching times (t1, t2) for active vectors
//! 3. Center the PWM waveforms symmetrically
//! 4. Clamp to valid duty cycle range
//!
//! Based on VESC firmware implementation.

use super::constants::{FRAC_1_SQRT_3 as ONE_BY_SQRT3, FRAC_2_SQRT_3 as TWO_BY_SQRT3};

/// Space Vector PWM modulation
///
/// Converts α-β frame voltages to 3-phase PWM duty cycles.
///
/// # Arguments
/// * `alpha` - Alpha axis voltage (normalized -1.0 to 1.0)
/// * `beta` - Beta axis voltage (normalized -1.0 to 1.0)
/// * `max_duty` - Maximum duty cycle value (e.g., 1000 for TIM ARR=1000)
///
/// # Returns
/// Array of [duty_a, duty_b, duty_c] where each value is 0 to max_duty
///
/// # Example
/// ```rust
/// use oxifoc_core::foc::svpwm::space_vector_pwm;
///
/// let vbus = 24.0; // volts
/// let v_alpha = 2.0; // desired α-axis voltage
/// let v_beta = 3.0;  // desired β-axis voltage
///
/// // Normalize to -1..1 range
/// let alpha_norm = v_alpha / vbus;
/// let beta_norm = v_beta / vbus;
///
/// let duties = space_vector_pwm(alpha_norm, beta_norm, 1000);
/// assert!(duties.iter().all(|&duty| duty <= 1000));
/// ```
pub fn space_vector_pwm(alpha: f32, beta: f32, max_duty: u16) -> [u16; 3] {
    // Determine sector using geometric method (VESC algorithm)
    // Uses >= for robust boundary handling
    let sector = if beta >= 0.0 {
        if alpha >= 0.0 {
            // Quadrant I
            if ONE_BY_SQRT3 * beta > alpha { 2 } else { 1 }
        } else {
            // Quadrant II
            if -ONE_BY_SQRT3 * beta > alpha { 3 } else { 2 }
        }
    } else if alpha >= 0.0 {
        // Quadrant IV
        if -ONE_BY_SQRT3 * beta > alpha { 5 } else { 6 }
    } else {
        // Quadrant III
        if ONE_BY_SQRT3 * beta > alpha { 4 } else { 5 }
    };

    // Calculate PWM timings per sector
    let pwm_full = max_duty as i32;
    let (ta, tb, tc) = match sector {
        1 => {
            // Vector on-times
            let t1 = ((alpha - ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let t2 = (TWO_BY_SQRT3 * beta * pwm_full as f32) as i32;
            // PWM timings with symmetrical centering
            let ta = (pwm_full + t1 + t2) / 2;
            let tb = ta - t1;
            let tc = tb - t2;
            (ta, tb, tc)
        }
        2 => {
            let t2 = ((alpha + ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let t3 = ((-alpha + ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let tb = (pwm_full + t2 + t3) / 2;
            let ta = tb - t3;
            let tc = ta - t2;
            (ta, tb, tc)
        }
        3 => {
            let t3 = (TWO_BY_SQRT3 * beta * pwm_full as f32) as i32;
            let t4 = ((-alpha - ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let tb = (pwm_full + t3 + t4) / 2;
            let tc = tb - t3;
            let ta = tc - t4;
            (ta, tb, tc)
        }
        4 => {
            let t4 = ((-alpha + ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let t5 = (-TWO_BY_SQRT3 * beta * pwm_full as f32) as i32;
            let tc = (pwm_full + t4 + t5) / 2;
            let tb = tc - t5;
            let ta = tb - t4;
            (ta, tb, tc)
        }
        5 => {
            let t5 = ((-alpha - ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let t6 = ((alpha - ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let tc = (pwm_full + t5 + t6) / 2;
            let ta = tc - t5;
            let tb = ta - t6;
            (ta, tb, tc)
        }
        6 => {
            let t6 = (-TWO_BY_SQRT3 * beta * pwm_full as f32) as i32;
            let t1 = ((alpha + ONE_BY_SQRT3 * beta) * pwm_full as f32) as i32;
            let ta = (pwm_full + t6 + t1) / 2;
            let tc = ta - t1;
            let tb = tc - t6;
            (ta, tb, tc)
        }
        _ => unreachable!("Invalid sector"),
    };

    // Clamp to valid duty cycle range
    [
        ta.clamp(0, pwm_full) as u16,
        tb.clamp(0, pwm_full) as u16,
        tc.clamp(0, pwm_full) as u16,
    ]
}

/// Get the sector number for a given α-β voltage
///
/// Useful for debugging and visualization.
/// Uses the same geometric method as the main SVPWM function.
pub fn get_sector(alpha: f32, beta: f32) -> u8 {
    if beta >= 0.0 {
        if alpha >= 0.0 {
            // Quadrant I
            if ONE_BY_SQRT3 * beta > alpha { 2 } else { 1 }
        } else {
            // Quadrant II
            if -ONE_BY_SQRT3 * beta > alpha { 3 } else { 2 }
        }
    } else if alpha >= 0.0 {
        // Quadrant IV
        if -ONE_BY_SQRT3 * beta > alpha { 5 } else { 6 }
    } else {
        // Quadrant III
        if ONE_BY_SQRT3 * beta > alpha { 4 } else { 5 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_DUTY: u16 = 1000;

    #[test]
    fn test_svpwm_zero_voltage() {
        // Zero voltage should give 50% duty cycle on all phases
        let duties = space_vector_pwm(0.0, 0.0, MAX_DUTY);

        // All phases should be at mid-point
        let expected = MAX_DUTY / 2;
        for duty in duties {
            assert!(
                (duty as i32 - expected as i32).abs() <= 1,
                "Zero voltage duty should be ~{}, got {}",
                expected,
                duty
            );
        }
    }

    #[test]
    fn test_svpwm_duties_in_range() {
        // Test various angles and magnitudes
        for angle_deg in (0..360).step_by(15) {
            let angle_rad = (angle_deg as f32).to_radians();

            for magnitude in [0.3, 0.5, 0.7, 0.9] {
                let alpha = magnitude * libm::cosf(angle_rad);
                let beta = magnitude * libm::sinf(angle_rad);

                let duties = space_vector_pwm(alpha, beta, MAX_DUTY);

                // All duties should be in valid range
                for (i, &duty) in duties.iter().enumerate() {
                    assert!(
                        duty <= MAX_DUTY,
                        "Duty cycle {} out of range at angle={}, mag={}: got {}",
                        i,
                        angle_deg,
                        magnitude,
                        duty
                    );
                }
            }
        }
    }

    #[test]
    fn test_svpwm_all_sectors() {
        // Test that all 6 sectors are covered
        // Using VESC geometric sector detection (robust at boundaries)
        let test_vectors = [
            (0.5, 0.1, 1),   // ~11° - sector 1
            (0.1, 0.5, 2),   // ~79° - sector 2
            (-0.4, 0.4, 3),  // ~135° - sector 3
            (-0.5, -0.1, 4), // ~191° - sector 4
            (-0.1, -0.5, 5), // ~259° - sector 5
            (0.4, -0.4, 6),  // ~315° - sector 6
        ];

        for (alpha, beta, expected_sector) in test_vectors {
            let sector = get_sector(alpha, beta);
            assert_eq!(
                sector, expected_sector,
                "Expected sector {} for α={}, β={}, got {}",
                expected_sector, alpha, beta, sector
            );

            // Also verify SVPWM produces valid output
            let duties = space_vector_pwm(alpha, beta, MAX_DUTY);
            assert!(duties[0] <= MAX_DUTY);
            assert!(duties[1] <= MAX_DUTY);
            assert!(duties[2] <= MAX_DUTY);
        }
    }

    #[test]
    fn test_sector_boundaries() {
        // Test angles at exact sector boundaries
        let boundary_angles: [f32; 6] = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0];

        for angle_deg in boundary_angles {
            let angle_rad = angle_deg.to_radians();
            let alpha = 0.5 * libm::cosf(angle_rad);
            let beta = 0.5 * libm::sinf(angle_rad);

            // Should not panic and should produce valid duties
            let duties = space_vector_pwm(alpha, beta, MAX_DUTY);

            for duty in duties {
                assert!(duty <= MAX_DUTY, "Duty out of range at boundary");
            }
        }
    }

    #[test]
    fn test_svpwm_sector_transitions() {
        // Test smooth transition between sectors
        for base_angle in [0, 60, 120, 180, 240, 300] {
            let mut prev_duties: Option<[u16; 3]> = None;

            for offset in 0..10 {
                let angle_deg = base_angle + offset * 6; // 6° steps
                let angle_rad = (angle_deg as f32).to_radians();
                let alpha = 0.5 * libm::cosf(angle_rad);
                let beta = 0.5 * libm::sinf(angle_rad);

                let duties = space_vector_pwm(alpha, beta, MAX_DUTY);

                if let Some(prev) = prev_duties {
                    // Check no huge jumps between consecutive angles
                    for i in 0..3 {
                        let diff = (duties[i] as i32 - prev[i] as i32).abs();
                        assert!(
                            diff < 200,
                            "Large duty change at sector boundary: {} -> {} (diff={})",
                            prev[i],
                            duties[i],
                            diff
                        );
                    }
                }

                prev_duties = Some(duties);
            }
        }
    }

    #[test]
    fn test_svpwm_saturation() {
        // Test that excessive voltage requests are clamped
        let duties = space_vector_pwm(2.0, 2.0, MAX_DUTY); // Way over range

        for duty in duties {
            assert!(duty <= MAX_DUTY, "Duty should be clamped to max");
        }
    }

    #[test]
    fn test_vesc_algorithm_zero_voltage() {
        // Zero voltage should give 50% duty cycle (centered PWM)
        let duties = space_vector_pwm(0.0, 0.0, 1000);

        // All phases should be at mid-point (500 ± tolerance for rounding)
        for duty in duties {
            assert!(
                (duty as i32 - 500).abs() <= 2,
                "Zero voltage should give ~500 duty, got {}",
                duty
            );
        }
    }

    #[test]
    fn test_svpwm_phase_balance() {
        // For balanced 3-phase SVPWM, verify duties are reasonable
        // Note: The exact sum varies by sector due to SVPWM algorithm
        for angle_deg in (0..360).step_by(30) {
            let angle_rad = (angle_deg as f32).to_radians();
            let alpha = 0.5 * libm::cosf(angle_rad);
            let beta = 0.5 * libm::sinf(angle_rad);

            let duties = space_vector_pwm(alpha, beta, MAX_DUTY);
            let sum: u32 = duties.iter().map(|&d| d as u32).sum();

            // Sum should be reasonable (between 1.0x and 2.0x max_duty)
            // SVPWM centering means it won't be exactly 1.5x everywhere
            assert!(
                sum >= MAX_DUTY as u32 && sum <= (MAX_DUTY as u32 * 2),
                "Duty sum {} outside reasonable range at angle {}",
                sum,
                angle_deg
            );
        }
    }
}
