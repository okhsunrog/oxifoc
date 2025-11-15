//! 6-step commutation logic for BLDC motor control

/// 6-step commutation state
///
/// Each step energizes 2 of the 3 phases:
/// - One phase driven high
/// - One phase driven low
/// - One phase floating (high-Z)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommutationStep {
    Step0 = 0,  // A+, B-, C floating
    Step1 = 1,  // A+, C-, B floating
    Step2 = 2,  // B+, C-, A floating
    Step3 = 3,  // B+, A-, C floating
    Step4 = 4,  // C+, A-, B floating
    Step5 = 5,  // C+, B-, A floating
}

impl CommutationStep {
    /// Advance to the next commutation step
    pub fn next(self) -> Self {
        match self {
            Self::Step0 => Self::Step1,
            Self::Step1 => Self::Step2,
            Self::Step2 => Self::Step3,
            Self::Step3 => Self::Step4,
            Self::Step4 => Self::Step5,
            Self::Step5 => Self::Step0,
        }
    }

    /// Get the step number (0-5)
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// Get phase enable/disable pattern for this step
    ///
    /// Returns (ph_a_en, ph_b_en, ph_c_en, ph_a_high, ph_b_high, ph_c_high)
    /// - enable flags: true = phase active, false = floating
    /// - high flags: true = high-side on, false = low-side on
    pub fn get_phase_states(self) -> (bool, bool, bool, bool, bool, bool) {
        match self {
            // A+, B-, C floating
            Self::Step0 => (true, true, false, true, false, false),
            // A+, C-, B floating
            Self::Step1 => (true, false, true, true, false, false),
            // B+, C-, A floating
            Self::Step2 => (false, true, true, false, true, false),
            // B+, A-, C floating
            Self::Step3 => (true, true, false, false, true, false),
            // C+, A-, B floating
            Self::Step4 => (true, false, true, false, false, true),
            // C+, B-, A floating
            Self::Step5 => (false, true, true, false, false, true),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_step_sequence() {
        let mut step = CommutationStep::Step0;
        for i in 1..=6 {
            step = step.next();
            assert_eq!(step.as_u8(), i % 6);
        }
    }
}
