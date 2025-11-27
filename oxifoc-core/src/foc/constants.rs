//! Mathematical constants for FOC algorithms
//!
//! All constants are f32 with appropriate precision for hardware FPU.

/// √3 = 1.732050807568877...
///
/// Used in inverse Clarke transform
pub const SQRT_3: f32 = 1.732_050_8;

/// 1/√3 = 0.577350269189626...
///
/// Used in Clarke transform and SVPWM sector detection
pub const FRAC_1_SQRT_3: f32 = 0.577_350_27;

/// 2/√3 = 1.154700538379252...
///
/// Used in SVPWM switching time calculations
pub const FRAC_2_SQRT_3: f32 = 1.154_700_5;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sqrt3_precision() {
        // Verify constants are within f32 precision
        let sqrt3_actual = 3.0_f32.sqrt();
        assert!((SQRT_3 - sqrt3_actual).abs() < 1e-6);

        let frac_1_sqrt3_actual = 1.0 / 3.0_f32.sqrt();
        assert!((FRAC_1_SQRT_3 - frac_1_sqrt3_actual).abs() < 1e-6);

        let frac_2_sqrt3_actual = 2.0 / 3.0_f32.sqrt();
        assert!((FRAC_2_SQRT_3 - frac_2_sqrt3_actual).abs() < 1e-6);
    }

    #[test]
    fn test_sqrt3_relationships() {
        // Verify mathematical relationships
        assert!((FRAC_2_SQRT_3 - 2.0 * FRAC_1_SQRT_3).abs() < 1e-6);
        assert!((SQRT_3 * FRAC_1_SQRT_3 - 1.0).abs() < 1e-6);
    }
}
