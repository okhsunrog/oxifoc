//! Current-sensor offset calibration driven from the FOC interrupt.
//!
//! The state machine is deliberately synchronous and allocation-free: one
//! fresh ADC sample is consumed per FOC cycle, while the caller owns the PWM
//! and applies the requested phase-state transitions. This keeps boot and
//! bench calibration on the same path and, unlike an async PWM routine, cannot
//! strand an energised phase when its waiter is cancelled.

use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use super::pwm::PhaseState;

/// Current-offset measurement topology.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CurrentOffsetMethod {
    /// Float every phase and measure all current channels.
    Undriven,
    /// VESC's normal routine: drive one phase at 50%, float the other two,
    /// and measure only the driven phase before moving to the next phase.
    PerPhase50,
    /// VESC's alternative routine: drive all phases at 50% and measure all
    /// channels. Bench-only; the rotor must be stationary.
    AllPhases50,
}

/// Completed current-offset measurement, in raw ADC counts.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentOffsetReport {
    pub offsets: [f32; 3],
}

/// Why an offset calibration request could not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurrentOffsetError {
    Busy,
    InvalidRequest,
    UnsafeWhileMoving,
    RequiresExistingCalibration,
    OverCurrent,
    Cancelled,
}

/// Internal request sent to the ISR-owned driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurrentOffsetRequest {
    pub method: CurrentOffsetMethod,
    pub samples_per_channel: u16,
    pub settle_cycles: u16,
    /// Apply the measured offsets to the live sensor when complete.
    pub apply: bool,
    /// Explicit operator acknowledgement required by [`CurrentOffsetMethod::AllPhases50`].
    pub stationary_confirmed: bool,
}

impl CurrentOffsetRequest {
    /// VESC-normal boot calibration at a 20 kHz FOC rate: 10 ms settling
    /// after each topology change and 1000 fresh samples per phase.
    pub const fn boot() -> Self {
        Self {
            method: CurrentOffsetMethod::PerPhase50,
            samples_per_channel: 1000,
            settle_cycles: 200,
            apply: true,
            stationary_confirmed: false,
        }
    }

    /// Non-mutating diagnostic request.
    pub const fn diagnostic(
        method: CurrentOffsetMethod,
        samples_per_channel: u16,
        stationary_confirmed: bool,
    ) -> Self {
        Self {
            method,
            samples_per_channel,
            settle_cycles: 200,
            apply: false,
            stationary_confirmed,
        }
    }

    pub const fn is_valid(self) -> bool {
        self.samples_per_channel >= 16
            && self.samples_per_channel <= 4096
            && self.settle_cycles <= 4000
    }
}

/// Output from one state-machine tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurrentOffsetProgress {
    /// A topology change the PWM owner must apply before the next sample.
    pub phase_states: Option<[PhaseState; 3]>,
    /// Present exactly once, after every requested sample has been consumed.
    pub report: Option<CurrentOffsetReport>,
}

/// Allocation-free current-offset calibration state machine.
#[derive(Clone, Copy, Debug)]
pub struct CurrentOffsetCalibrator {
    request: Option<CurrentOffsetRequest>,
    phase: u8,
    configured: bool,
    settling: u16,
    collected: u16,
    sums: [u32; 3],
}

impl CurrentOffsetCalibrator {
    pub const fn new() -> Self {
        Self {
            request: None,
            phase: 0,
            configured: false,
            settling: 0,
            collected: 0,
            sums: [0; 3],
        }
    }

    pub fn start(&mut self, request: CurrentOffsetRequest) -> Result<(), CurrentOffsetError> {
        if self.request.is_some() {
            return Err(CurrentOffsetError::Busy);
        }
        if !request.is_valid() {
            return Err(CurrentOffsetError::InvalidRequest);
        }
        *self = Self {
            request: Some(request),
            ..Self::new()
        };
        Ok(())
    }

    pub const fn is_active(&self) -> bool {
        self.request.is_some()
    }

    pub fn cancel(&mut self) -> bool {
        let was_active = self.is_active();
        *self = Self::new();
        was_active
    }

    pub fn request(&self) -> Option<CurrentOffsetRequest> {
        self.request
    }

    /// Consume one fresh raw ADC sample.
    pub fn tick(&mut self, raw: [u16; 3], max_duty: u16) -> CurrentOffsetProgress {
        let Some(request) = self.request else {
            return CurrentOffsetProgress {
                phase_states: None,
                report: None,
            };
        };

        if !self.configured {
            self.configured = true;
            self.settling = request.settle_cycles;
            return CurrentOffsetProgress {
                phase_states: Some(states_for(request.method, self.phase, max_duty)),
                report: None,
            };
        }

        if self.settling != 0 {
            self.settling -= 1;
            return CurrentOffsetProgress {
                phase_states: None,
                report: None,
            };
        }

        match request.method {
            CurrentOffsetMethod::PerPhase50 => {
                self.record(self.phase as usize, raw[self.phase as usize]);
            }
            CurrentOffsetMethod::Undriven | CurrentOffsetMethod::AllPhases50 => {
                for (channel, value) in raw.into_iter().enumerate() {
                    self.record(channel, value);
                }
            }
        }
        self.collected += 1;

        if self.collected < request.samples_per_channel {
            return CurrentOffsetProgress {
                phase_states: None,
                report: None,
            };
        }

        if request.method == CurrentOffsetMethod::PerPhase50 && self.phase < 2 {
            self.phase += 1;
            self.collected = 0;
            self.settling = request.settle_cycles;
            return CurrentOffsetProgress {
                phase_states: Some(states_for(request.method, self.phase, max_duty)),
                report: None,
            };
        }

        let count = f32::from(request.samples_per_channel);
        let report = CurrentOffsetReport {
            offsets: self.sums.map(|sum| sum as f32 / count),
        };
        *self = Self::new();
        CurrentOffsetProgress {
            phase_states: None,
            report: Some(report),
        }
    }

    fn record(&mut self, channel: usize, value: u16) {
        self.sums[channel] += u32::from(value);
    }
}

impl Default for CurrentOffsetCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

pub const fn float_all() -> [PhaseState; 3] {
    [PhaseState::Float; 3]
}

fn states_for(method: CurrentOffsetMethod, phase: u8, max_duty: u16) -> [PhaseState; 3] {
    let half = max_duty / 2;
    match method {
        CurrentOffsetMethod::Undriven => float_all(),
        CurrentOffsetMethod::AllPhases50 => [PhaseState::Pwm(half); 3],
        CurrentOffsetMethod::PerPhase50 => {
            let mut states = float_all();
            states[phase as usize] = PhaseState::Pwm(half);
            states
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn per_phase_uses_only_the_driven_channel() {
        let mut cal = CurrentOffsetCalibrator::new();
        cal.start(CurrentOffsetRequest {
            method: CurrentOffsetMethod::PerPhase50,
            samples_per_channel: 16,
            settle_cycles: 0,
            apply: false,
            stationary_confirmed: false,
        })
        .unwrap();

        let mut progress = cal.tick([0; 3], 1000);
        let mut report = None;
        for phase in 0..3 {
            let mut expected = float_all();
            expected[phase] = PhaseState::Pwm(500);
            assert_eq!(progress.phase_states, Some(expected));
            for _ in 0..16 {
                progress = cal.tick([101, 202, 303], 1000);
                report = progress.report.or(report);
            }
        }
        let report = report.unwrap();
        assert_eq!(report.offsets, [101.0, 202.0, 303.0]);
        assert!(!cal.is_active());
    }

    #[test]
    fn all_phase_and_undriven_measure_every_channel() {
        for method in [
            CurrentOffsetMethod::Undriven,
            CurrentOffsetMethod::AllPhases50,
        ] {
            let mut cal = CurrentOffsetCalibrator::new();
            cal.start(CurrentOffsetRequest {
                method,
                samples_per_channel: 16,
                settle_cycles: 1,
                apply: false,
                stationary_confirmed: method == CurrentOffsetMethod::AllPhases50,
            })
            .unwrap();
            let first = cal.tick([1, 2, 3], 1000);
            assert!(first.phase_states.is_some());
            assert!(cal.tick([9, 9, 9], 1000).report.is_none());
            let mut report = None;
            for _ in 0..16 {
                report = cal.tick([10, 20, 30], 1000).report.or(report);
            }
            assert_eq!(report.unwrap().offsets, [10.0, 20.0, 30.0]);
        }
    }
}
