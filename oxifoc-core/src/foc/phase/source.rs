//! Phase source selection for FOC control
//!
//! Defines which angle source to use and how to blend between sources.

/// Phase source selection
///
/// Specifies where the electrical angle comes from for FOC control.
/// Supports hardware sensors, software observers, and hybrid modes.
#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    PartialEq,
    serde::Serialize,
    serde::Deserialize,
    postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseSource {
    // =========================================================================
    // Direct hardware sensor
    // =========================================================================
    /// Use Hall sensor for angle
    #[default]
    Hall,

    /// Use encoder for angle
    Encoder,

    // =========================================================================
    // Software estimation
    // =========================================================================
    /// Back-EMF observer (sensorless)
    Observer,

    /// High-frequency injection (for low/zero speed sensorless)
    Hfi,

    // =========================================================================
    // Hybrid modes (sensor + observer blending)
    // =========================================================================
    /// Hall primary, observer assist (the default sensored ride mode)
    ///
    /// Two orthogonal jobs in one source:
    /// - blends linearly Hall→observer between `blend_low` and `blend_high`
    ///   velocity (hall quantization dominates at speed, observer is exact);
    /// - automatic failure fallback: on hall loss (invalid state / stale at
    ///   speed) commutation falls to the observer if ready, else to the
    ///   open-loop recovery override.
    HallToObserver {
        /// Start blending (electrical rad/s)
        blend_low: f32,
        /// Full observer (electrical rad/s)
        blend_high: f32,
    },

    /// Encoder at low speed, transition to observer at high speed
    EncoderToObserver {
        /// Start blending (electrical rad/s)
        blend_low: f32,
        /// Full observer (electrical rad/s)
        blend_high: f32,
    },

    /// HFI at startup, transition to back-EMF observer
    HfiToObserver {
        /// Minimum velocity for observer (rad/s)
        min_vel: f32,
        /// Minimum observer confidence (0.0-1.0)
        min_confidence: f32,
    },

    /// HFI at startup, transition to Hall sensor
    HfiToHall {
        /// Velocity threshold for switching (rad/s)
        switch_vel: f32,
    },

    /// HFI at startup, transition to encoder
    HfiToEncoder {
        /// Velocity threshold for switching (rad/s)
        switch_vel: f32,
    },

    // =========================================================================
    // Manual control (calibration, testing)
    // =========================================================================
    /// Use manually set angle (for calibration)
    ///
    /// Angle is set via `PhaseManager::set_manual_angle()`.
    /// Motor locks at the specified electrical angle.
    Manual,

    /// Open-loop angle ramp (sensorless startup, calibration)
    ///
    /// Angle advances automatically based on `set_open_loop_velocity()`.
    OpenLoop,

    // =========================================================================
    // NOTE: postcard encodes the variant index — append new variants HERE,
    // never reorder the ones above. (One deliberate renumber 2026-06-12:
    // HallWithFallback merged into HallToObserver. Safe because PhaseSource
    // is never persisted and host + firmware build from the same tree.)
    // =========================================================================
    /// HFI at low drive voltage, blend to the back-EMF observer above —
    /// MESC-style criterion: |vq − R·iq| (the back-EMF share of the drive
    /// voltage) replaces the velocity threshold. Self-normalizing: no
    /// per-motor eRPM tuning, the threshold is in volts.
    ///
    /// Blend band: [toggle_v − 1 V hysteresis, toggle_v].
    HfiToObserverVolts {
        /// Drive-voltage threshold (V) above which the observer carries
        /// commutation alone. MESC default ≈ 5% of vbus, min 1.5 V.
        toggle_v: f32,
        /// Minimum observer confidence (0.0-1.0)
        min_confidence: f32,
    },
}

/// Error when setting phase source
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseSourceError {
    /// Hall sensor not available
    HallNotAvailable,
    /// Encoder not available
    EncoderNotAvailable,
    /// Observer not configured
    ObserverNotConfigured,
    /// HFI not configured
    HfiNotConfigured,
}

impl PhaseSource {
    /// All f32 payload fields are finite (see `ControlMode::is_finite`).
    pub fn is_finite(&self) -> bool {
        match *self {
            Self::Hall
            | Self::Encoder
            | Self::Observer
            | Self::Hfi
            | Self::Manual
            | Self::OpenLoop => true,
            Self::HallToObserver {
                blend_low,
                blend_high,
            }
            | Self::EncoderToObserver {
                blend_low,
                blend_high,
            } => blend_low.is_finite() && blend_high.is_finite(),
            Self::HfiToObserver {
                min_vel,
                min_confidence,
            } => min_vel.is_finite() && min_confidence.is_finite(),
            Self::HfiToObserverVolts {
                toggle_v,
                min_confidence,
            } => toggle_v.is_finite() && toggle_v > 0.0 && min_confidence.is_finite(),
            Self::HfiToHall { switch_vel } | Self::HfiToEncoder { switch_vel } => {
                switch_vel.is_finite()
            }
        }
    }

    /// Check if this source requires a Hall sensor
    pub fn requires_hall(&self) -> bool {
        matches!(
            self,
            Self::Hall | Self::HallToObserver { .. } | Self::HfiToHall { .. }
        )
    }

    /// Check if this source requires an encoder
    pub fn requires_encoder(&self) -> bool {
        matches!(
            self,
            Self::Encoder | Self::EncoderToObserver { .. } | Self::HfiToEncoder { .. }
        )
    }

    /// Check if this source requires an observer
    pub fn requires_observer(&self) -> bool {
        matches!(
            self,
            Self::Observer
                | Self::HallToObserver { .. }
                | Self::EncoderToObserver { .. }
                | Self::HfiToObserver { .. }
                | Self::HfiToObserverVolts { .. }
        )
    }

    /// Check if this source requires HFI
    pub fn requires_hfi(&self) -> bool {
        matches!(
            self,
            Self::Hfi
                | Self::HfiToObserver { .. }
                | Self::HfiToObserverVolts { .. }
                | Self::HfiToHall { .. }
                | Self::HfiToEncoder { .. }
        )
    }

    /// Check if this is a manual/open-loop mode
    pub fn is_manual(&self) -> bool {
        matches!(self, Self::Manual | Self::OpenLoop)
    }
}
