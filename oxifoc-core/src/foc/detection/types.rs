//! Common types for motor parameter detection.
//!
//! Provides motor size classification, detected parameter storage,
//! and error types used across all detection algorithms.

#[cfg(feature = "detection")]
use crate::foc::detection::flux_linkage;
use crate::foc::fast_math::sqrtf;
use crate::motor::dissipation;
use crate::units::{
    AngularVelocity, ConstZero, ElectricCurrent, ElectricPotential, ElectricalAngularVelocity,
    ElectricalResistance, Frequency, Inductance, MagneticFlux, Power, Sign, amps, erad_per_s, erpm,
    hertz, rpm, volts, watts,
};

/// Motor size classification for determining safe test currents.
///
/// Based on VESC's motor size presets, each size maps to a maximum
/// power loss that's safe for detection measurements.
/// This prevents overheating during resistance/inductance measurements.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorSize {
    /// Mini outrunner (~75g), 20W max power loss
    Mini,
    /// Small motor (~200g), 50W max power loss
    Small,
    /// Medium motor (~750g), 120W max power loss
    #[default]
    Medium,
    /// Large motor (~2kg), 400W max power loss
    Large,
    /// Custom power loss limit
    Custom(Power),
}

impl MotorSize {
    /// Returns the maximum power loss safe for this motor size.
    ///
    /// This value is used to determine the maximum test current:
    /// - Start with current_max/50
    /// - Increase by 1.5× until I²R × 1.5 >= max_power_loss/5
    /// - Final max_current = sqrt(max_power_loss / R / 1.5)
    #[inline]
    pub fn max_power_loss(&self) -> Power {
        match self {
            Self::Mini => watts(20.0),
            Self::Small => watts(50.0),
            Self::Medium => watts(120.0),
            Self::Large => watts(400.0),
            Self::Custom(power) => *power,
        }
    }

    /// Suggested open-loop speed for flux linkage measurement.
    ///
    /// Smaller motors can spin faster during detection.
    #[inline]
    pub fn suggested_open_loop_speed(&self) -> ElectricalAngularVelocity {
        match self {
            Self::Mini | Self::Small => erpm(1400.0),
            Self::Medium | Self::Large | Self::Custom(_) => erpm(700.0),
        }
    }

    /// Suggested sensorless transition speed.
    #[inline]
    pub fn suggested_sensorless_speed(&self) -> ElectricalAngularVelocity {
        erpm(4000.0)
    }
}

/// Test currents use the thermal budget as a scale, independently of the
/// application's drive limits. The caller supplies the active current ceiling.
#[derive(Clone, Copy, Debug)]
pub struct DetectionCurrents {
    pub resistance: ElectricCurrent,
    pub inductance_bias: ElectricCurrent,
    pub inductance_rms: ElectricCurrent,
    pub inductance_peak: ElectricCurrent,
    pub flux: ElectricCurrent,
    pub hall: ElectricCurrent,
}

impl DetectionCurrents {
    pub fn new(
        power_loss: Power,
        resistance: ElectricalResistance,
        vbus: ElectricPotential,
        ceiling: ElectricCurrent,
    ) -> Result<Self, DetectionError> {
        let usable = [
            power_loss.is_positive() && power_loss.is_finite(),
            resistance.is_positive() && resistance.is_finite(),
            vbus.is_positive() && vbus.is_finite(),
            ceiling.is_positive() && ceiling.is_finite(),
        ];
        if usable.contains(&false) {
            return Err(DetectionError::OutOfRange);
        }
        let thermal = dissipation::heat_current_limit(power_loss, resistance);
        let dc_ceiling = ceiling.min(vbus * 0.577 * 0.85 / resistance);
        // VESC stops raising its R/L test current when copper loss reaches P/5.
        let resistance_current = (thermal * sqrtf(0.2)).min(dc_ceiling);
        // HFI uses a biased, rotating AC vector. Split the R/L current scale
        // between the DC bias and AC RMS target, leaving room for AC peaks.
        let inductance_peak = thermal.min(ceiling);
        let inductance_bias = 0.4 * resistance_current;
        let inductance_rms =
            (0.16 * resistance_current).min(0.5 * (inductance_peak - inductance_bias));
        Ok(Self {
            resistance: resistance_current,
            inductance_bias,
            inductance_rms,
            inductance_peak,
            flux: (thermal / 2.5).min(dc_ceiling),
            hall: (thermal / 3.0).min(dc_ceiling),
        })
    }
}

/// Detected motor electrical parameters.
///
/// Contains all parameters that can be measured during motor detection:
/// resistance, inductance (d/q axes), flux linkage, and derived values.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorParams {
    /// Phase-to-neutral resistance
    pub resistance: ElectricalResistance,

    /// d-axis inductance
    pub inductance_d: Inductance,

    /// q-axis inductance
    pub inductance_q: Inductance,

    /// Average inductance (Ld + Lq) / 2
    pub inductance_avg: Inductance,

    /// Ld - Lq difference (for IPMSM saliency)
    pub inductance_diff: Inductance,

    /// Flux linkage (lambda)
    pub flux_linkage: MagneticFlux,

    /// Motor Kv rating in RPM/V (calculated from flux linkage)
    pub kv_rpm_per_v: f32,

    /// Maximum safe continuous current, from resistance and the motor size
    /// power limit
    pub max_current: ElectricCurrent,

    /// Number of pole pairs (must be set externally)
    pub pole_pairs: u8,
}

impl MotorParams {
    /// Calculate Kv (RPM per Volt) from flux linkage and pole pairs.
    ///
    /// Formula: Kv = 60 / (√3 × 2π × λ × pole_pairs). The √3 converts the
    /// per-phase λ (amplitude-invariant Park) to the line-to-line Kv that
    /// nameplates/VESC use; see [`flux_linkage::calculate_kv`].
    #[cfg(feature = "detection")]
    pub fn calculate_kv(&mut self) {
        if self.flux_linkage.is_positive() && self.pole_pairs > 0 {
            self.kv_rpm_per_v = flux_linkage::calculate_kv(self.flux_linkage, self.pole_pairs);
        }
    }

    /// Calculate maximum safe current from resistance and power limit.
    ///
    /// Formula: I_max = sqrt(max_power_loss / R / 1.5)
    pub fn calculate_max_current(&mut self, motor_size: MotorSize) {
        if self.resistance.is_positive() {
            self.max_current =
                dissipation::heat_current_limit(motor_size.max_power_loss(), self.resistance);
        }
    }

    /// Check if resistance has been measured.
    #[inline]
    pub fn has_resistance(&self) -> bool {
        self.resistance.is_positive()
    }

    /// Check if inductance has been measured.
    #[inline]
    pub fn has_inductance(&self) -> bool {
        self.inductance_avg.is_positive()
    }

    /// Check if flux linkage has been measured.
    #[inline]
    pub fn has_flux_linkage(&self) -> bool {
        self.flux_linkage.is_positive()
    }

    /// Check if all primary parameters have been measured.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.has_resistance() && self.has_inductance() && self.has_flux_linkage()
    }
}

/// Errors that can occur during motor parameter detection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DetectionError {
    /// Current sensor not calibrated (DC offsets not measured)
    CurrentNotCalibrated,

    /// Motor not responding (open circuit, disconnected, or locked rotor)
    MotorNotResponding,

    /// Measured value outside expected physical range
    OutOfRange,

    /// Detection operation timed out
    Timeout,

    /// Motor moving during static measurement (resistance/inductance)
    UnexpectedMotion,

    /// Measurement quality too low (noisy or unstable)
    LowConfidence,

    /// Hardware fault detected during measurement
    HardwareFault,

    /// Insufficient samples collected
    InsufficientSamples,

    /// Required prerequisite measurement not done
    /// (e.g., flux linkage requires resistance)
    MissingPrerequisite,
}

/// Parameters for resistance measurement.
#[derive(Clone, Copy, Debug)]
pub struct ResistanceParams {
    /// Motor size for determining safe test current
    pub motor_size: MotorSize,

    /// Maximum hardware current limit
    pub current_max: ElectricCurrent,

    /// Minimum detectable current
    pub current_min: ElectricCurrent,

    /// Time to ramp up current (milliseconds)
    pub ramp_time_ms: u32,

    /// Time to wait for settling after ramp (milliseconds)
    pub settle_time_ms: u32,

    /// Number of samples to average for final measurement
    pub num_samples: u32,

    /// Interval between samples (microseconds)
    pub sample_interval_us: u32,
}

impl Default for ResistanceParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            current_max: amps(10.0),
            current_min: amps(0.5),
            ramp_time_ms: 500,
            settle_time_ms: 200,
            num_samples: 100,
            sample_interval_us: 1000,
        }
    }
}

/// Parameters for inductance measurement using HFI.
#[derive(Clone, Copy, Debug)]
pub struct InductanceParams {
    /// Motor size for determining safe test current
    pub motor_size: MotorSize,

    /// HFI injection frequency
    pub hfi_frequency: Frequency,

    /// Maximum live HFI injection voltage amplitude.
    pub hfi_voltage: ElectricPotential,

    /// DC current to hold rotor in place
    pub hold_current: ElectricCurrent,

    /// Target RMS magnitude of the AC alpha-beta current, excluding DC hold.
    pub hfi_current: ElectricCurrent,

    /// Maximum sampled alpha-beta current magnitude, including the DC hold.
    /// Enforced each ISR cycle during HFI packets, independently of the trip.
    pub max_current: ElectricCurrent,

    /// Number of HFI cycles to measure
    pub num_cycles: u32,

    /// Time to wait for settling (milliseconds)
    pub settle_time_ms: u32,

    /// Previously measured phase resistance.
    /// Used to compensate for the resistive voltage drop in the 1/L
    /// calculation. Offline replay permits 0; live HFI needs measured R to
    /// limit its first probe before inductance is known.
    pub resistance: ElectricalResistance,

    /// DC bus voltage, zero = unknown.
    ///
    /// When known, the injection amplitude is clamped to the voltage
    /// headroom left above the holding voltage (`vbus·0.577 − R·I_hold`),
    /// so the commanded vector never saturates against the bus mid-window.
    pub vbus: ElectricPotential,

    /// Command→apply pipeline depth in FOC cycles. `< 0` (default) =
    /// measure it in place with a correlation probe before the HFI run
    /// (see `probe_hfi_pipeline_lag`); `≥ 1` = use this value (tests,
    /// platforms with a known pipeline). One mispaired cycle rotates the
    /// demod carrier reference by `ω_c·T_pwm` — 90° at the default
    /// 5 kHz/20 kHz — which corrupts the inductance estimate entirely,
    /// so this must match the real latency, not be guessed.
    pub pipeline_lag: i8,
}

impl Default for InductanceParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            hfi_frequency: hertz(5000.0),
            hfi_voltage: volts(3.0),
            hold_current: amps(2.0),
            hfi_current: amps(0.5),
            max_current: amps(3.0),
            num_cycles: 100,
            settle_time_ms: 200,
            resistance: ElectricalResistance::ZERO,
            vbus: ElectricPotential::ZERO,
            pipeline_lag: -1,
        }
    }
}

impl InductanceParams {
    /// Derive separate DC alignment and AC excitation from the motor class.
    pub fn commissioning(
        motor_size: MotorSize,
        resistance: ElectricalResistance,
        vbus: ElectricPotential,
        current_limit: ElectricCurrent,
    ) -> Result<Self, DetectionError> {
        let currents =
            DetectionCurrents::new(motor_size.max_power_loss(), resistance, vbus, current_limit)?;
        let hold_current = currents.inductance_bias;
        let hfi_current = currents.inductance_rms;
        if hold_current < amps(0.05) || hfi_current < amps(0.01) {
            return Err(DetectionError::OutOfRange);
        }
        Ok(Self {
            motor_size,
            resistance,
            vbus,
            hold_current,
            hfi_current,
            max_current: currents.inductance_peak,
            hfi_voltage: vbus * 0.4,
            ..Self::default()
        })
    }
}

/// Parameters for flux linkage measurement.
#[derive(Clone, Copy, Debug)]
pub struct FluxLinkageParams {
    /// Motor size for determining test current
    pub motor_size: MotorSize,

    /// Previously measured resistance — required for driven method,
    /// ignored by spin-down method.
    pub resistance: ElectricalResistance,

    /// Previously measured average inductance, zero = unknown.
    ///
    /// Used by the driven back-EMF-vector method for the `ωL·i` reactance
    /// term. With zero the method still works (load-angle invariance does
    /// not depend on it); the term only trims a small residual that grows
    /// with `L·I/λ`. Ignored by the q-axis and spin-down methods.
    pub inductance: Inductance,

    /// Maximum mechanical speed for the spin-up ramp. With a positive
    /// `v_target` the ramp usually stops earlier (see below); this is the
    /// hard cap.
    pub spin_speed: AngularVelocity,

    /// Open-loop current during spin-up
    pub current: ElectricCurrent,

    /// Voltage-magnitude target for the spin-up ramp, zero = disabled.
    ///
    /// The driven flux methods degrade when the resistive drop R·I is
    /// comparable to the back-EMF ω·λ, so instead of spinning to a fixed
    /// RPM the ramp continues until |V| reaches this target — the same
    /// idea as VESC ramping its open loop to duty 0.3 (≈ 0.2·vbus of
    /// phase voltage, conf_general.c) rather than to a speed. Callers
    /// that know vbus should set ≈ `0.2 × vbus`.
    pub v_target: ElectricPotential,

    /// Time to ramp up to target RPM (milliseconds)
    pub ramp_time_ms: u32,

    /// Time to wait at target RPM before sampling (milliseconds)
    pub settle_time_ms: u32,

    /// Number of samples to average
    pub num_samples: u32,

    /// Number of pole pairs (for eRPM calculation)
    pub pole_pairs: u8,

    /// Minimum electrical angular velocity to accept a sample during
    /// spin-down measurement.  Below this, back-EMF is too small for
    /// accurate ADC readings.  Default: 50 rad/s (~475 eRPM at 7pp).
    pub min_coast_speed: ElectricalAngularVelocity,
}

impl Default for FluxLinkageParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            resistance: ElectricalResistance::ZERO,
            inductance: Inductance::ZERO,
            // Generous cap: with v_target set, the voltage target normally
            // ends the ramp well before this speed is reached.
            spin_speed: rpm(1500.0),
            current: amps(2.0),
            v_target: ElectricPotential::ZERO,
            ramp_time_ms: 2000,
            // In open-loop drive the rotor hunts around the synchronous
            // point with a decay time of ~2J/b — seconds for high-inertia
            // low-friction motors. Averaging the e⃗ components over an
            // oscillating load angle shrinks |e⃗| (cos factor), biasing λ
            // low (−12% on an IPM traction sim at 1 s/100 ms). VESC settles
            // 1 s and then averages for 10 s; 3 s + a 1 s window gets within
            // ~2% at a fraction of that.
            settle_time_ms: 3000,
            num_samples: 2000,
            pole_pairs: 7,
            min_coast_speed: erad_per_s(50.0),
        }
    }
}

/// Parameters for voltage-pulse inductance measurement (fallback for HFI).
#[derive(Clone, Copy, Debug)]
pub struct VoltagePulseParams {
    /// DC holding current to lock rotor
    pub hold_current: ElectricCurrent,
    /// Previously measured phase resistance
    pub resistance: ElectricalResistance,
    /// Voltage step amplitude — the *ceiling*. With a positive
    /// `current_target` the step is ramped up from small only as far as
    /// needed to reach the target di (and never past this), so for a
    /// low-inductance motor the actual step lands well below it.  Typically
    /// the bus headroom `vbus·0.577 − R·I_hold`.
    pub pulse_voltage: ElectricPotential,
    /// Current excursion budget per pulse: the peak stays within
    /// `I_hold + current_target` regardless of L. Positive enables the
    /// VESC-style current-limited ramp; since a pulse may stay applied for
    /// up to `PIPELINE_LAG_MAX + 2` periods (command latency + edge
    /// detection), the step is sized so ONE period's di is that fraction of
    /// the budget — sizing it to the whole budget per period let the peak
    /// reach several times it. Zero (default) = no limit, use
    /// `pulse_voltage` as-is.
    pub current_target: ElectricCurrent,
    /// Number of pulses to average per axis (default: 20)
    pub num_pulses: u32,
    /// Settling time after locking rotor (milliseconds)
    pub settle_time_ms: u32,
}

impl Default for VoltagePulseParams {
    fn default() -> Self {
        Self {
            hold_current: amps(2.0),
            resistance: ElectricalResistance::ZERO,
            pulse_voltage: volts(5.0),
            current_target: ElectricCurrent::ZERO,
            num_pulses: 20,
            settle_time_ms: 200,
        }
    }
}

/// Parameters for DC offset calibration.
#[derive(Clone, Copy, Debug)]
pub struct DcOffsetParams {
    /// Number of samples to average per phase
    pub num_samples: u32,

    /// Settling time between PWM state changes (milliseconds)
    pub settle_time_ms: u32,
}

impl Default for DcOffsetParams {
    fn default() -> Self {
        Self {
            num_samples: 256,
            settle_time_ms: 50,
        }
    }
}

/// DC offset calibration results.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DcOffsets {
    /// Phase A current sensor offset (ADC counts or Amps depending on stage)
    pub phase_a: f32,

    /// Phase B current sensor offset
    pub phase_b: f32,

    /// Phase C current sensor offset
    pub phase_c: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{amps, henries, ohms, volts, watts, webers};

    #[test]
    fn detection_profile_scales_with_class_and_respects_the_active_ceiling() {
        let low =
            InductanceParams::commissioning(MotorSize::Medium, ohms(0.031), volts(24.0), amps(3.0))
                .unwrap();
        let high = InductanceParams::commissioning(
            MotorSize::Medium,
            ohms(0.031),
            volts(24.0),
            amps(30.7),
        )
        .unwrap();
        assert!(low.hold_current < high.hold_current);
        assert!(low.hfi_current < high.hfi_current);
        let mini =
            DetectionCurrents::new(watts(20.0), ohms(0.0624), volts(24.0), amps(20.0)).unwrap();
        let medium =
            DetectionCurrents::new(watts(120.0), ohms(0.0624), volts(24.0), amps(20.0)).unwrap();
        assert!(medium.resistance > mini.resistance);
        assert!(medium.flux > mini.flux && medium.hall > mini.hall);
        let copper_loss = 1.5 * ohms(0.0624) * medium.resistance * medium.resistance;
        assert!((copper_loss - watts(24.0)).abs() < watts(0.01));
        assert!(medium.inductance_bias + 2.0 * medium.inductance_rms <= amps(20.0));
        let capped =
            DetectionCurrents::new(watts(120.0), ohms(0.0624), volts(24.0), amps(6.0)).unwrap();
        assert_eq!(capped.resistance, amps(6.0));
        assert_eq!(capped.hall, amps(6.0));
        let small =
            InductanceParams::commissioning(MotorSize::Mini, ohms(8.0), volts(12.0), amps(0.5))
                .unwrap();
        assert!(small.hold_current < amps(0.5));
        assert!(small.hold_current + 2.0 * small.hfi_current <= small.max_current);
        for limit in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(
                InductanceParams::commissioning(
                    MotorSize::Medium,
                    ohms(0.03),
                    volts(24.0),
                    amps(limit)
                )
                .is_err()
            );
        }
    }

    #[test]
    fn test_motor_size_power_loss() {
        assert_eq!(MotorSize::Mini.max_power_loss(), watts(20.0));
        assert_eq!(MotorSize::Small.max_power_loss(), watts(50.0));
        assert_eq!(MotorSize::Medium.max_power_loss(), watts(120.0));
        assert_eq!(MotorSize::Large.max_power_loss(), watts(400.0));
        assert_eq!(MotorSize::Custom(watts(75.0)).max_power_loss(), watts(75.0));
    }

    #[cfg(feature = "detection")]
    #[test]
    fn test_motor_params_kv_calculation() {
        let mut params = MotorParams {
            flux_linkage: webers(0.01), // 10 mWb
            pole_pairs: 7,
            ..Default::default()
        };
        params.calculate_kv();

        // Kv = 60 / (√3 × 2π × 0.01 × 7) ≈ 78.8 RPM/V (line-to-line)
        assert!((params.kv_rpm_per_v - 78.8).abs() < 1.0);
    }

    #[test]
    fn test_motor_params_max_current() {
        let mut params = MotorParams {
            resistance: ohms(0.1), // 100 mΩ
            ..Default::default()
        };
        params.calculate_max_current(MotorSize::Medium);

        // I_max = sqrt(120 / 0.1 / 1.5) = sqrt(800) ≈ 28.3A
        assert!((params.max_current - amps(28.3)).abs() < amps(0.5));
    }

    #[test]
    fn test_motor_params_completeness() {
        let mut params = MotorParams::default();
        assert!(!params.is_complete());

        params.resistance = ohms(0.1);
        assert!(!params.is_complete());

        params.inductance_avg = henries(0.0001);
        assert!(!params.is_complete());

        params.flux_linkage = webers(0.01);
        assert!(params.is_complete());
    }
}
