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
use crate::units::{
    ConstZero, ElectricCurrent, ElectricPotential, ElectricalResistance, Frequency, Inductance,
    Time, amps, henries, volts,
};

/// Minimum measurable current change.  Below this the pulse is
/// indistinguishable from ADC noise.
const MIN_DI: ElectricCurrent = amps(0.005);
/// Smallest inductive voltage the period's di is credited to.
const MIN_INDUCTIVE_V: ElectricPotential = volts(0.01);
/// Plausible winding inductance: 0.1 µH to 100 mH.
const L_RANGE: core::ops::RangeInclusive<Inductance> = henries(1e-7)..=henries(0.1);

/// Accumulator for voltage-pulse inductance measurement on a single axis.
///
/// Call [`record_pulse`](VoltagePulseMeasurement::record_pulse) once per pulse with the current before and after
/// the voltage step.  After `num_pulses` calls, [`finish`](VoltagePulseMeasurement::finish) returns the
/// average inductance.
///
/// # Dead-time immunity
///
/// The step is measured *relative to the settled hold*, which cancels the
/// inverter dead-time. The open-loop PI raises the holding voltage to
/// `R·I_hold + V_dt`, where `V_dt` is the dead-time loss the firmware's
/// compensation does not fully cancel (the real dead-time runs larger than the
/// ideal `t_dt·f_pwm` because of FET switching dynamics). Through the pulse the
/// phase-current direction never changes, so the *same* `V_dt` applies and
/// drops out of the step:
///
/// ```text
/// V_dt        = vd_hold − R·I_hold                       (recovered in `new`)
/// V_inductive = V_applied − R·i_avg − V_dt = pulse − R·(i_avg − I_hold)
/// L           = V_inductive · dt / di
/// ```
///
/// Folding the *absolute* hold current in (not `di/2`) also keeps it correct if
/// the baseline ratchets above `I_hold` between pulses. Leaving `V_dt` in (the
/// old absolute form) inflated low-inductance motors several-fold, because for
/// them the pulse step is small and `V_dt` (~0.2–0.4 V) dominates it.
#[derive(Clone, Debug)]
pub struct VoltagePulseMeasurement {
    /// Total d-axis voltage applied *during* the pulse: the settled hold plus
    /// the pulse step.
    applied_voltage: ElectricPotential,
    resistance: ElectricalResistance,
    /// Dead-time make-up baked into the settled hold (`vd_hold − R·I_hold`),
    /// subtracted from every pulse so the inductance is dead-time-immune.
    v_deadtime: ElectricPotential,
    dt: Time,
    target_pulses: u32,
    inductance_sum: Inductance,
    valid_count: u32,
}

impl VoltagePulseMeasurement {
    /// Create a new single-axis pulse measurement.
    ///
    /// `hold_voltage` is the *settled* d-axis voltage holding the rotor at
    /// `params.hold_current_a` (the open-loop PI output, `R·I_hold` plus the
    /// residual dead-time `V_dt`). `V_dt = hold_voltage − R·I_hold` is
    /// recovered here and removed from every pulse — see the type docs.
    pub fn new(
        params: &VoltagePulseParams,
        pwm_freq: Frequency,
        hold_voltage: ElectricPotential,
    ) -> Self {
        let resistance = params.resistance;
        // This is a signed command-side bias. Overcompensation can require a
        // negative PI hold; clipping its residual to zero fails to cancel it.
        let v_deadtime = hold_voltage - resistance * params.hold_current;
        Self {
            applied_voltage: hold_voltage + params.pulse_voltage,
            resistance,
            v_deadtime,
            dt: pwm_freq.recip(),
            target_pulses: params.num_pulses,
            inductance_sum: Inductance::ZERO,
            valid_count: 0,
        }
    }

    /// Measured residual dead-time voltage (`vd_hold − R·I_hold`) — the part of
    /// the inverter dead-time the firmware's compensation did not cancel.
    /// Exposed for diagnostics and for self-calibrating the compensation factor.
    pub fn dead_time(&self) -> ElectricPotential {
        self.v_deadtime
    }

    /// Record one voltage-pulse result.
    ///
    /// # Arguments
    /// * `i_before` — d-axis current just before the pulse
    /// * `i_after`  — d-axis current one PWM period after the pulse
    ///
    /// # Returns
    /// `true` when enough pulses have been collected.
    pub fn record_pulse(&mut self, i_before: ElectricCurrent, i_after: ElectricCurrent) -> bool {
        let di = i_after - i_before;
        if di.abs() < MIN_DI {
            // Pulse too weak to measure — skip but don't fail
            return self.valid_count >= self.target_pulses;
        }

        // Locked-rotor d-axis equation over one period, measured relative to the
        // settled hold so the dead-time make-up `V_dt` cancels (see type docs):
        //   V_inductive = V_applied − R·i_avg − V_dt  (= pulse − R·(i_avg − I_hold))
        //   L = V_inductive · dt / di
        let i_avg = (i_before + i_after) / 2.0;
        let v_inductive = self.applied_voltage - self.resistance * i_avg - self.v_deadtime;

        if v_inductive.abs() > MIN_INDUCTIVE_V {
            let l: Inductance = v_inductive * self.dt / di;
            if l > Inductance::ZERO && l.is_finite() {
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
    pub fn finish(self) -> Result<Inductance, DetectionError> {
        if self.valid_count < self.target_pulses.min(5) {
            return Err(DetectionError::InsufficientSamples);
        }

        let l = self.inductance_sum / self.valid_count as f32;

        if !L_RANGE.contains(&l) {
            return Err(DetectionError::OutOfRange);
        }

        Ok(l)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{hertz, ohms};

    /// Plain-SI-float view of the measurement for the numeric checks below.
    struct Meas(VoltagePulseMeasurement);

    impl Meas {
        fn new(params: &VoltagePulseParams, pwm_freq_hz: f32, hold_voltage_v: f32) -> Self {
            Self(VoltagePulseMeasurement::new(
                params,
                hertz(pwm_freq_hz),
                volts(hold_voltage_v),
            ))
        }
        fn record_pulse(&mut self, i_before: f32, i_after: f32) -> bool {
            self.0.record_pulse(amps(i_before), amps(i_after))
        }
        fn finish(self) -> Result<f32, DetectionError> {
            self.0.finish().map(|l| l.value)
        }
    }

    #[test]
    fn pure_inductor_pulse() {
        // Simulate a pure inductor (R=0): L = V * dt / di
        let params = VoltagePulseParams {
            pulse_voltage: volts(5.0),
            resistance: ohms(0.0),
            hold_current: amps(2.0),
            num_pulses: 10,
            ..Default::default()
        };
        let l_actual = 0.0005; // 500 µH
        let dt = 1.0 / 20_000.0;
        let i_before = 2.0;
        // R = 0 → holding voltage and dead-time make-up are both zero.
        let mut meas = Meas::new(&params, 20_000.0, 0.0);

        for _ in 0..10 {
            // di = V * dt / L
            let di = params.pulse_voltage.value * dt / l_actual;
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
            pulse_voltage: volts(5.0),
            resistance: ohms(r),
            hold_current: amps(2.0),
            num_pulses: 10,
            ..Default::default()
        };
        let l_actual = 0.0005;
        let dt = 1.0 / 20_000.0;
        let i_before = 2.0;
        // Settled hold = R·I_hold exactly (no dead-time in this ideal case), so
        // `V_dt = hold − R·I_hold = 0` and the step `pulse − R·(i_avg − I_hold)`
        // drives di.
        let mut meas = Meas::new(&params, 20_000.0, r * i_before);

        for _ in 0..10 {
            // Exact first-order: V_pulse = R*di/2 + L*di/dt
            // di = V_pulse * dt / (L + R*dt/2)
            let di = params.pulse_voltage.value * dt / (l_actual + r * dt / 2.0);
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
            pulse_voltage: volts(5.0),
            resistance: ohms(r),
            hold_current: amps(0.3),
            num_pulses: 10,
            ..Default::default()
        };
        let dt = 1.0 / 20_000.0;
        let i_before = 0.3;
        let mut meas = Meas::new(&params, 20_000.0, r * i_before);

        for _ in 0..10 {
            let di = params.pulse_voltage.value * dt / (l_actual + r * dt / 2.0);
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
    fn signed_holding_bias_cancels_for_overcompensated_inverter() {
        let r = 0.03;
        let actual_l = 25e-6;
        let dt = 50e-6;
        for hold_voltage in [-0.13, 0.036, 0.24] {
            let params = VoltagePulseParams {
                pulse_voltage: volts(0.05),
                resistance: ohms(r),
                hold_current: amps(1.2),
                num_pulses: 8,
                ..Default::default()
            };
            let mut measurement = Meas::new(&params, 20_000.0, hold_voltage);
            let di = params.pulse_voltage.value * dt / (actual_l + r * dt * 0.5);
            for _ in 0..params.num_pulses {
                measurement.record_pulse(params.hold_current.value, params.hold_current.value + di);
            }
            assert!((measurement.finish().unwrap() / actual_l - 1.0).abs() < 1e-5);
        }
    }
}
