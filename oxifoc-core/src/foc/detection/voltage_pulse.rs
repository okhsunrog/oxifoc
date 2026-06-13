//! Inductance measurement via voltage pulse (di/dt method).
//!
//! Applies a voltage step to a locked rotor and measures the resulting
//! current change over one PWM period:
//!
//!   `L = (V_pulse − R × i_avg) × dt / di`
//!
//! Simpler and more robust than HFI on high-resistance motors where
//! the inductive impedance is small relative to R at the HFI frequency.
//! Used as a fallback when the HFI method produces suspicious results.

use super::types::{DetectionError, VoltagePulseParams};

/// Minimum measurable current change (A).  Below this the pulse is
/// indistinguishable from ADC noise.
const MIN_DI: f32 = 0.005;

/// Accumulator for voltage-pulse inductance measurement on a single axis.
///
/// Call [`record_pulse`] once per pulse with the current before and after
/// the voltage step.  After `num_pulses` calls, [`finish`] returns the
/// average inductance.
#[derive(Clone, Debug)]
pub struct VoltagePulseMeasurement {
    /// Total d-axis voltage applied *during* the pulse: the holding voltage
    /// (`R·I_hold`) plus the pulse step. Stored absolute, not as the step
    /// alone, so the resistive term can use the absolute current — see
    /// [`record_pulse`].
    applied_voltage_v: f32,
    resistance_ohm: f32,
    dt: f32,
    target_pulses: u32,
    inductance_sum: f32,
    valid_count: u32,
}

impl VoltagePulseMeasurement {
    /// Create a new single-axis pulse measurement.
    ///
    /// `hold_voltage_v` is the steady d-axis voltage that holds the rotor at
    /// `I_hold` (i.e. `R·I_hold` plus any dead-time make-up) — the baseline
    /// the pulse step adds to. The total `hold + step` is the voltage
    /// actually across the winding while di is measured.
    pub fn new(params: &VoltagePulseParams, pwm_freq_hz: f32, hold_voltage_v: f32) -> Self {
        Self {
            applied_voltage_v: hold_voltage_v + params.pulse_voltage_v,
            resistance_ohm: params.resistance_ohm,
            dt: 1.0 / pwm_freq_hz,
            target_pulses: params.num_pulses,
            inductance_sum: 0.0,
            valid_count: 0,
        }
    }

    /// Record one voltage-pulse result.
    ///
    /// # Arguments
    /// * `i_before` — d-axis current just before the pulse (A)
    /// * `i_after`  — d-axis current one PWM period after the pulse (A)
    ///
    /// # Returns
    /// `true` when enough pulses have been collected.
    pub fn record_pulse(&mut self, i_before: f32, i_after: f32) -> bool {
        let di = i_after - i_before;
        if di.abs() < MIN_DI {
            // Pulse too weak to measure — skip but don't fail
            return self.valid_count >= self.target_pulses;
        }

        // Locked-rotor d-axis equation over one period: V = R·i + L·di/dt, so
        //   L = (V_applied − R·i_avg) · dt / di
        // with i_avg = (i_before + i_after)/2 (trapezoidal).
        //
        // Crucially the resistive drop uses the ABSOLUTE current, not di/2.
        // The open-loop holding voltage does not discharge the winding between
        // pulses (L/R ≫ the inter-pulse gap of a few PWM periods), so i_before
        // ratchets above I_hold pulse after pulse. A `pulse_step − R·di/2` term
        // (which silently assumes every pulse starts from the steady I_hold)
        // then over-credits the inductor as the baseline climbs → a systematic
        // L overestimate that grows with L/R. Anchoring on the absolute current
        // makes the estimate immune to that drift.
        let i_avg = (i_before + i_after) / 2.0;
        let v_inductive = self.applied_voltage_v - self.resistance_ohm * i_avg;

        if v_inductive.abs() > 0.01 {
            let l = v_inductive * self.dt / di;
            if l > 0.0 && l.is_finite() {
                self.inductance_sum += l;
                self.valid_count += 1;
            }
        }

        self.valid_count >= self.target_pulses
    }

    /// Check if enough pulses have been collected.
    pub fn is_complete(&self) -> bool {
        self.valid_count >= self.target_pulses
    }

    /// Compute the average inductance from accumulated pulses.
    pub fn finish(self) -> Result<f32, DetectionError> {
        if self.valid_count < self.target_pulses.min(5) {
            return Err(DetectionError::InsufficientSamples);
        }

        let l = self.inductance_sum / self.valid_count as f32;

        if !(1e-7..=0.1).contains(&l) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(l)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_inductor_pulse() {
        // Simulate a pure inductor (R=0): L = V * dt / di
        let params = VoltagePulseParams {
            pulse_voltage_v: 5.0,
            resistance_ohm: 0.0,
            num_pulses: 10,
            ..Default::default()
        };
        let l_actual = 0.0005; // 500 µH
        let dt = 1.0 / 20_000.0;
        let i_before = 2.0;
        // R = 0 → holding voltage is zero.
        let mut meas = VoltagePulseMeasurement::new(&params, 20_000.0, 0.0);

        for _ in 0..10 {
            // di = V * dt / L
            let di = params.pulse_voltage_v * dt / l_actual;
            let i_after = i_before + di;
            meas.record_pulse(i_before, i_after);
        }

        let l = meas.finish().unwrap();
        let err = (l - l_actual).abs() / l_actual;
        assert!(
            err < 0.01,
            "error {:.1}%: {:.2} µH vs {:.2} µH",
            err * 100.0,
            l * 1e6,
            l_actual * 1e6
        );
    }

    #[test]
    fn with_resistance_compensation() {
        // Simulate the correct physics: the pulse voltage is additional
        // to the holding voltage.  di = (V_pulse - R*di/2) * dt / L.
        // For short pulses R*di/2 is small so di ≈ V_pulse * dt / L.
        let r = 0.5;
        let params = VoltagePulseParams {
            pulse_voltage_v: 5.0,
            resistance_ohm: r,
            num_pulses: 10,
            ..Default::default()
        };
        let l_actual = 0.0005;
        let dt = 1.0 / 20_000.0;
        let i_before = 2.0;
        // Holding voltage R·I_hold balances the baseline current, so the pulse
        // step alone drives di — the steady-baseline case where the absolute
        // and the di/2 resistive terms coincide.
        let mut meas = VoltagePulseMeasurement::new(&params, 20_000.0, r * i_before);

        for _ in 0..10 {
            // Exact first-order: V_pulse = R*di/2 + L*di/dt
            // di = V_pulse * dt / (L + R*dt/2)
            let di = params.pulse_voltage_v * dt / (l_actual + r * dt / 2.0);
            meas.record_pulse(i_before, i_before + di);
        }

        let l = meas.finish().unwrap();
        let err = (l - l_actual).abs() / l_actual;
        assert!(
            err < 0.01,
            "error {:.1}%: {:.2} µH vs {:.2} µH",
            err * 100.0,
            l * 1e6,
            l_actual * 1e6
        );
    }

    #[test]
    fn high_resistance_motor() {
        // Gimbal-class: R=8Ω, L=3mH — the case where HFI fails.
        let r = 8.0;
        let l_actual = 0.003;
        let params = VoltagePulseParams {
            pulse_voltage_v: 5.0,
            resistance_ohm: r,
            num_pulses: 10,
            ..Default::default()
        };
        let dt = 1.0 / 20_000.0;
        let i_before = 0.3;
        let mut meas = VoltagePulseMeasurement::new(&params, 20_000.0, r * i_before);

        for _ in 0..10 {
            let di = params.pulse_voltage_v * dt / (l_actual + r * dt / 2.0);
            meas.record_pulse(i_before, i_before + di);
        }

        let l = meas.finish().unwrap();
        let err = (l - l_actual).abs() / l_actual;
        assert!(
            err < 0.01,
            "error {:.1}%: {:.2} µH vs {:.2} µH",
            err * 100.0,
            l * 1e6,
            l_actual * 1e6
        );
    }
}
