//! Simulation adapters shared by control examples and the network device.

use crate::foc::pwm::{PhasePwm, PhaseState};

/// PWM stage with the boards' channel gating: `disable()` or an all-Float
/// topology turns the bridge off (the rig then coasts the plant — high-Z,
/// not the shorted windings a zero voltage would model), and only
/// `enable()` or a driven topology turns it back on.
pub struct VirtualPwm {
    pub duties: [u16; 3],
    pub enabled: bool,
}

impl PhasePwm for VirtualPwm {
    fn max_duty(&self) -> u16 {
        1000
    }
    fn set_duties(&mut self, duties: [u16; 3]) {
        self.duties = duties;
    }
    fn disable(&mut self) {
        self.enabled = false;
    }
    fn enable(&mut self) {
        self.enabled = true;
    }
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        self.enabled = !states.iter().all(|s| matches!(s, PhaseState::Float));
        self.duties = states.map(|s| match s {
            PhaseState::Pwm(d) => d,
            PhaseState::Low | PhaseState::Float => 0,
        });
    }
}
