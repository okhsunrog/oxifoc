//! Bounded voltage programs. Only waveform generation and paired current
//! capture run in the control ISR; fitting/FFT stays on the async executor.

use super::types::DetectionError;
use crate::foc::trig::SinCos;
use crate::motor::ControlMode;

pub const MAX_BURST_SAMPLES: usize = 256;

#[cfg(feature = "runtime")]
pub(crate) static DONE_WAKER: embassy_sync::waitqueue::AtomicWaker =
    embassy_sync::waitqueue::AtomicWaker::new();

#[derive(Clone, Copy, Debug)]
pub enum Waveform {
    RotatingHfi {
        amplitude: f32,
        phase_step: f32,
    },
    Pulse {
        vd: f32,
        vq: f32,
        start: u16,
        periods: u16,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct VoltageBurst {
    pub waveform: Waveform,
    pub samples: u16,
    pub vd_hold: f32,
    pub vq_hold: f32,
    pub hold_current: f32,
    pub angle: f32,
}

impl VoltageBurst {
    pub fn is_valid(&self) -> bool {
        if self.samples == 0
            || usize::from(self.samples) > MAX_BURST_SAMPLES
            || !self.vd_hold.is_finite()
            || !self.vq_hold.is_finite()
            || !self.hold_current.is_finite()
            || self.hold_current <= 0.0
            || !self.angle.is_finite()
        {
            return false;
        }
        match self.waveform {
            Waveform::RotatingHfi {
                amplitude,
                phase_step,
            } => {
                amplitude.is_finite()
                    && amplitude > 0.0
                    && phase_step.is_finite()
                    && phase_step > 0.0
                    && phase_step < core::f32::consts::PI
            }
            Waveform::Pulse {
                vd,
                vq,
                start,
                periods,
            } => {
                vd.is_finite()
                    && vq.is_finite()
                    && periods > 0
                    && usize::from(start) + usize::from(periods) < usize::from(self.samples)
            }
        }
    }

    pub fn injection<S: SinCos>(&self, index: usize) -> (f32, f32) {
        match self.waveform {
            Waveform::RotatingHfi {
                amplitude,
                phase_step,
            } => {
                let (carrier, _) = S::sin_cos((index as f32 * phase_step) % core::f32::consts::TAU);
                let (sin, cos) = S::sin_cos((index % 32) as f32 * (core::f32::consts::TAU / 32.0));
                (amplitude * carrier * cos, amplitude * carrier * sin)
            }
            Waveform::Pulse {
                vd,
                vq,
                start,
                periods,
            } => {
                if (usize::from(start)..usize::from(start) + usize::from(periods)).contains(&index)
                {
                    (vd, vq)
                } else {
                    (0.0, 0.0)
                }
            }
        }
    }

    pub fn command<S: SinCos>(&self, index: usize) -> ControlMode {
        let (vd, vq) = self.injection::<S>(index);
        ControlMode::DirectVoltage {
            vd: self.vd_hold + vd,
            vq: self.vq_hold + vq,
            angle_rad: self.angle,
        }
    }

    pub fn hold_mode(&self) -> ControlMode {
        ControlMode::OpenLoop {
            angle_rad: self.angle,
            current: self.hold_current,
            velocity_rad_s: 0.0,
            pi_gains: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BurstSample {
    pub alpha: f32,
    pub beta: f32,
    pub cycle: u32,
    pub tick_us: u32,
}

#[derive(Clone, Debug)]
pub struct BurstCapture {
    pub samples: [BurstSample; MAX_BURST_SAMPLES],
    pub len: usize,
}

#[cfg(feature = "runtime")]
pub(crate) enum BurstAction {
    Stop,
    Voltage {
        vd: f32,
        vq: f32,
        angle: f32,
        compensation_current: (f32, f32),
    },
    Hold(VoltageBurst),
}

impl Default for BurstCapture {
    fn default() -> Self {
        Self {
            samples: [BurstSample::default(); MAX_BURST_SAMPLES],
            len: 0,
        }
    }
}

impl BurstCapture {
    pub fn validate(&self, expected: usize) -> Result<(), DetectionError> {
        if self.len != expected || self.len == 0 || self.len > MAX_BURST_SAMPLES {
            return Err(DetectionError::InsufficientSamples);
        }
        if self.samples[..self.len]
            .iter()
            .any(|s| !s.alpha.is_finite() || !s.beta.is_finite())
        {
            return Err(DetectionError::HardwareFault);
        }
        if self.samples[..self.len]
            .windows(2)
            .any(|s| s[1].cycle.wrapping_sub(s[0].cycle) != 1)
        {
            return Err(DetectionError::InsufficientSamples);
        }
        Ok(())
    }
}

/// Single flight, owned by MotorControlState. Async callers only arm, cancel
/// and read frozen samples. One small record is written per ISR invocation.
#[cfg(feature = "runtime")]
#[derive(Clone, Debug)]
pub(crate) struct Excitation {
    pub program: Option<VoltageBurst>,
    pub outcome: Option<Result<(), DetectionError>>,
    pub cancelled: bool,
    pub index: usize,
    pub preparing: bool,
    pub voltages: [(f32, f32); MAX_BURST_SAMPLES],
    pub compensation_current: (f32, f32),
    last_cycle: Option<u32>,
    last_tick: u32,
    first_tick: u32,
    first_cycle: u32,
    pub samples: [BurstSample; MAX_BURST_SAMPLES],
}

#[cfg(feature = "runtime")]
impl Excitation {
    pub const fn new() -> Self {
        Self {
            program: None,
            outcome: None,
            cancelled: false,
            index: 0,
            preparing: false,
            voltages: [(0.0, 0.0); MAX_BURST_SAMPLES],
            compensation_current: (0.0, 0.0),
            last_cycle: None,
            last_tick: 0,
            first_tick: 0,
            first_cycle: 0,
            samples: [BurstSample {
                alpha: 0.0,
                beta: 0.0,
                cycle: 0,
                tick_us: 0,
            }; MAX_BURST_SAMPLES],
        }
    }

    #[cfg(any(feature = "embassy", test))]
    pub fn arm(&mut self, program: VoltageBurst) -> Result<(), DetectionError> {
        if self.program.is_some() {
            return Err(DetectionError::HardwareFault);
        }
        if !program.is_valid() {
            return Err(DetectionError::OutOfRange);
        }
        self.program = Some(program);
        self.outcome = None;
        self.cancelled = false;
        self.index = 0;
        self.preparing = true;
        self.last_cycle = None;
        Ok(())
    }

    pub fn running(&self) -> bool {
        self.program.is_some() && self.outcome.is_none() && !self.preparing
    }

    pub fn abort(&mut self, error: DetectionError) -> bool {
        if self.running() {
            self.outcome = Some(Err(error));
            true
        } else {
            false
        }
    }

    /// Sequence continuity detects skipped/corrupt software samples. The
    /// independent timestamp also rejects a stalled control ISR (>1.9 periods).
    /// Entry jitter alone is allowed; this is not a hardware trigger counter.
    pub fn check_timing(&mut self, cycle: u32, tick: u32, dt: f32) -> Result<(), DetectionError> {
        if let Some(last) = self.last_cycle {
            let elapsed = tick.wrapping_sub(self.last_tick);
            let span = tick.wrapping_sub(self.first_tick) as f32;
            let expected = cycle.wrapping_sub(self.first_cycle) as f32 * dt * 1_000_000.0;
            if cycle.wrapping_sub(last) != 1
                || elapsed == 0
                || elapsed as f32 > dt * 1_900_000.0 + 1.0
                || (span - expected).abs() > dt * 1_000_000.0 + 1.0
            {
                return Err(DetectionError::InsufficientSamples);
            }
        } else {
            self.first_tick = tick;
            self.first_cycle = cycle;
        }
        self.last_cycle = Some(cycle);
        self.last_tick = tick;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captures_reject_missing_duplicate_and_nonfinite_samples() {
        let mut capture = BurstCapture {
            len: 2,
            ..Default::default()
        };
        capture.samples[0].cycle = u32::MAX;
        capture.samples[1].cycle = 0;
        assert!(capture.validate(2).is_ok(), "counter wrap is contiguous");
        assert_eq!(
            capture.validate(3),
            Err(DetectionError::InsufficientSamples)
        );
        capture.samples[1].cycle = u32::MAX;
        assert_eq!(
            capture.validate(2),
            Err(DetectionError::InsufficientSamples)
        );
        capture.samples[1].cycle = 0;
        capture.samples[1].alpha = f32::NAN;
        assert_eq!(capture.validate(2), Err(DetectionError::HardwareFault));
    }

    #[cfg(feature = "runtime")]
    #[test]
    fn independent_timer_wrap_is_valid_but_missing_cycle_is_not() {
        let mut engine = Excitation::new();
        assert!(
            engine
                .check_timing(u32::MAX, u32::MAX - 25, 0.00005)
                .is_ok()
        );
        assert!(engine.check_timing(0, 24, 0.00005).is_ok());
        assert_eq!(
            engine.check_timing(2, 74, 0.00005),
            Err(DetectionError::InsufficientSamples)
        );
    }
}
