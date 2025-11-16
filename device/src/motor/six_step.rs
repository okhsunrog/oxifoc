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
    Step0 = 0, // A+, B-, C floating
    Step1 = 1, // A+, C-, B floating
    Step2 = 2, // B+, C-, A floating
    Step3 = 3, // B+, A-, C floating
    Step4 = 4, // C+, A-, B floating
    Step5 = 5, // C+, B-, A floating
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

    /// Get phase drive pattern for this step.
    ///
    /// Returns the desired drive state for (phase A, phase B, phase C).
    pub fn get_phase_states(self) -> (PhaseState, PhaseState, PhaseState) {
        use PhaseState::*;
        match self {
            // A+, B-, C floating
            Self::Step0 => (High, Low, Off),
            // A+, C-, B floating
            Self::Step1 => (High, Off, Low),
            // B+, C-, A floating
            Self::Step2 => (Off, High, Low),
            // B+, A-, C floating
            Self::Step3 => (Low, High, Off),
            // C+, A-, B floating
            Self::Step4 => (Low, Off, High),
            // C+, B-, A floating
            Self::Step5 => (Off, Low, High),
        }
    }
}

/// Desired drive state for a single motor phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseState {
    /// Both FETs off (phase floating).
    Off,
    /// High-side FET active, low-side off.
    High,
    /// Low-side FET active, high-side off.
    Low,
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
