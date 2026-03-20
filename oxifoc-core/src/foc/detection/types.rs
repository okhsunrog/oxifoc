//! Common types for motor parameter detection.
//!
//! Provides motor size classification, detected parameter storage,
//! and error types used across all detection algorithms.

/// Motor size classification for determining safe test currents.
///
/// Based on VESC's motor size presets, each size maps to a maximum
/// power loss (in Watts) that's safe for detection measurements.
/// This prevents overheating during resistance/inductance measurements.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorSize {
    /// Mini outrunner (~75g), 20W max power loss
    Mini,
    /// Small motor (~200g), 50W max power loss
    Small,
    /// Medium motor (~750g), 120W max power loss
    Medium,
    /// Large motor (~2kg), 400W max power loss
    Large,
    /// Custom power loss limit in Watts
    Custom(f32),
}

impl MotorSize {
    /// Returns the maximum power loss in Watts safe for this motor size.
    ///
    /// This value is used to determine the maximum test current:
    /// - Start with current_max/50
    /// - Increase by 1.5× until I²R × 1.5 >= max_power_loss/5
    /// - Final max_current = sqrt(max_power_loss / R / 1.5)
    #[inline]
    pub fn max_power_loss_w(&self) -> f32 {
        match self {
            MotorSize::Mini => 20.0,
            MotorSize::Small => 50.0,
            MotorSize::Medium => 120.0,
            MotorSize::Large => 400.0,
            MotorSize::Custom(w) => *w,
        }
    }

    /// Suggested open-loop eRPM for flux linkage measurement.
    ///
    /// Smaller motors can spin faster during detection.
    #[inline]
    pub fn suggested_open_loop_erpm(&self) -> f32 {
        match self {
            MotorSize::Mini | MotorSize::Small => 1400.0,
            MotorSize::Medium => 700.0,
            MotorSize::Large => 700.0,
            MotorSize::Custom(_) => 700.0,
        }
    }

    /// Suggested sensorless transition eRPM.
    #[inline]
    pub fn suggested_sensorless_erpm(&self) -> f32 {
        match self {
            MotorSize::Mini | MotorSize::Small => 4000.0,
            MotorSize::Medium | MotorSize::Large => 4000.0,
            MotorSize::Custom(_) => 4000.0,
        }
    }
}

#[allow(clippy::derivable_impls)] // Can't derive Default with Custom(f32) variant
impl Default for MotorSize {
    fn default() -> Self {
        MotorSize::Medium
    }
}

/// Detected motor electrical parameters.
///
/// Contains all parameters that can be measured during motor detection:
/// resistance, inductance (d/q axes), flux linkage, and derived values.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorParams {
    /// Phase-to-neutral resistance in Ohms
    pub resistance_ohm: f32,

    /// d-axis inductance in Henries
    pub inductance_d_h: f32,

    /// q-axis inductance in Henries
    pub inductance_q_h: f32,

    /// Average inductance (Ld + Lq) / 2 in Henries
    pub inductance_avg_h: f32,

    /// Ld - Lq difference in Henries (for IPMSM saliency)
    pub inductance_diff_h: f32,

    /// Flux linkage (lambda) in Weber
    pub flux_linkage_wb: f32,

    /// Motor Kv rating in RPM/V (calculated from flux linkage)
    pub kv_rpm_per_v: f32,

    /// Maximum safe continuous current in Amps
    /// Calculated from resistance and motor size power limit
    pub max_current_a: f32,

    /// Number of pole pairs (must be set externally)
    pub pole_pairs: u8,
}

impl MotorParams {
    /// Calculate Kv (RPM per Volt) from flux linkage and pole pairs.
    ///
    /// Formula: Kv = 60 / (2π × λ × pole_pairs)
    pub fn calculate_kv(&mut self) {
        if self.flux_linkage_wb > 0.0 && self.pole_pairs > 0 {
            self.kv_rpm_per_v =
                60.0 / (core::f32::consts::TAU * self.flux_linkage_wb * self.pole_pairs as f32);
        }
    }

    /// Calculate maximum safe current from resistance and power limit.
    ///
    /// Formula: I_max = sqrt(max_power_loss / R / 1.5)
    pub fn calculate_max_current(&mut self, motor_size: MotorSize) {
        if self.resistance_ohm > 0.0 {
            let max_power = motor_size.max_power_loss_w();
            self.max_current_a = libm::sqrtf(max_power / self.resistance_ohm / 1.5);
        }
    }

    /// Check if resistance has been measured.
    #[inline]
    pub fn has_resistance(&self) -> bool {
        self.resistance_ohm > 0.0
    }

    /// Check if inductance has been measured.
    #[inline]
    pub fn has_inductance(&self) -> bool {
        self.inductance_avg_h > 0.0
    }

    /// Check if flux linkage has been measured.
    #[inline]
    pub fn has_flux_linkage(&self) -> bool {
        self.flux_linkage_wb > 0.0
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

    /// Maximum hardware current limit (Amps)
    pub current_max: f32,

    /// Minimum detectable current (Amps)
    pub current_min: f32,

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
            current_max: 10.0,
            current_min: 0.5,
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

    /// HFI injection frequency in Hz
    pub hfi_frequency_hz: f32,

    /// HFI injection voltage amplitude in Volts
    pub hfi_voltage_v: f32,

    /// DC current to hold rotor in place (Amps)
    pub hold_current_a: f32,

    /// Number of HFI cycles to measure
    pub num_cycles: u32,

    /// Time to wait for settling (milliseconds)
    pub settle_time_ms: u32,

    /// Previously measured phase resistance (Ohms).
    /// Used to compensate for the resistive voltage drop in the 1/L
    /// calculation.  Set to 0.0 when unknown.
    pub resistance_ohm: f32,
}

impl Default for InductanceParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            hfi_frequency_hz: 5000.0,
            hfi_voltage_v: 3.0,
            hold_current_a: 2.0,
            num_cycles: 100,
            settle_time_ms: 200,
            resistance_ohm: 0.0,
        }
    }
}

/// Parameters for flux linkage measurement.
#[derive(Clone, Copy, Debug)]
pub struct FluxLinkageParams {
    /// Motor size for determining test current
    pub motor_size: MotorSize,

    /// Previously measured resistance (Ohms) — required for driven method,
    /// ignored by spin-down method.
    pub resistance_ohm: f32,

    /// Target mechanical RPM for measurement
    pub spin_rpm: f32,

    /// Open-loop current during spin-up (Amps)
    pub current_a: f32,

    /// Time to ramp up to target RPM (milliseconds)
    pub ramp_time_ms: u32,

    /// Time to wait at target RPM before sampling (milliseconds)
    pub settle_time_ms: u32,

    /// Number of samples to average
    pub num_samples: u32,

    /// Number of pole pairs (for eRPM calculation)
    pub pole_pairs: u8,

    /// Minimum electrical angular velocity (rad/s) to accept a sample
    /// during spin-down measurement.  Below this, back-EMF is too small
    /// for accurate ADC readings.  Default: 50 rad/s (~475 eRPM at 7pp).
    pub min_coast_omega_e: f32,
}

impl Default for FluxLinkageParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            resistance_ohm: 0.0,
            spin_rpm: 500.0,
            current_a: 2.0,
            ramp_time_ms: 2000,
            settle_time_ms: 1000,
            num_samples: 200,
            pole_pairs: 7,
            min_coast_omega_e: 50.0,
        }
    }
}

/// Parameters for voltage-pulse inductance measurement (fallback for HFI).
#[derive(Clone, Copy, Debug)]
pub struct VoltagePulseParams {
    /// DC holding current to lock rotor (Amps)
    pub hold_current_a: f32,
    /// Previously measured phase resistance (Ohms)
    pub resistance_ohm: f32,
    /// Voltage step amplitude (Volts).  Typically 20-30% of Vbus.
    pub pulse_voltage_v: f32,
    /// Number of pulses to average per axis (default: 20)
    pub num_pulses: u32,
    /// Settling time after locking rotor (milliseconds)
    pub settle_time_ms: u32,
}

impl Default for VoltagePulseParams {
    fn default() -> Self {
        Self {
            hold_current_a: 2.0,
            resistance_ohm: 0.0,
            pulse_voltage_v: 5.0,
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

    #[test]
    fn test_motor_size_power_loss() {
        assert_eq!(MotorSize::Mini.max_power_loss_w(), 20.0);
        assert_eq!(MotorSize::Small.max_power_loss_w(), 50.0);
        assert_eq!(MotorSize::Medium.max_power_loss_w(), 120.0);
        assert_eq!(MotorSize::Large.max_power_loss_w(), 400.0);
        assert_eq!(MotorSize::Custom(75.0).max_power_loss_w(), 75.0);
    }

    #[test]
    fn test_motor_params_kv_calculation() {
        let mut params = MotorParams {
            flux_linkage_wb: 0.01, // 10 mWb
            pole_pairs: 7,
            ..Default::default()
        };
        params.calculate_kv();

        // Kv = 60 / (2π × 0.01 × 7) ≈ 136.5 RPM/V
        assert!((params.kv_rpm_per_v - 136.5).abs() < 1.0);
    }

    #[test]
    fn test_motor_params_max_current() {
        let mut params = MotorParams {
            resistance_ohm: 0.1, // 100 mΩ
            ..Default::default()
        };
        params.calculate_max_current(MotorSize::Medium);

        // I_max = sqrt(120 / 0.1 / 1.5) = sqrt(800) ≈ 28.3A
        assert!((params.max_current_a - 28.3).abs() < 0.5);
    }

    #[test]
    fn test_motor_params_completeness() {
        let mut params = MotorParams::default();
        assert!(!params.is_complete());

        params.resistance_ohm = 0.1;
        assert!(!params.is_complete());

        params.inductance_avg_h = 0.0001;
        assert!(!params.is_complete());

        params.flux_linkage_wb = 0.01;
        assert!(params.is_complete());
    }
}
