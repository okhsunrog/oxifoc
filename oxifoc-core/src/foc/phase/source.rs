//! Phase source selection for FOC control
//!
//! Defines which angle source to use and how to blend between sources.

/// Phase source selection
///
/// Specifies where the electrical angle comes from for FOC control.
/// Supports hardware sensors, software observers, and hybrid modes.
#[derive(
    Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseSource {
    // =========================================================================
    // Direct hardware sensor
    // =========================================================================
    /// Use Hall sensor for angle
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
    /// Hall at low speed, transition to observer at high speed
    ///
    /// Blends linearly between `blend_low` and `blend_high` velocity.
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

    /// Hall sensor with automatic observer fallback (VESC-style)
    ///
    /// Full-featured Hall mode with:
    /// - Hall at low speed
    /// - Blends to observer at high speed
    /// - Automatic fallback to observer if Hall fails
    /// - Open-loop override if observer not ready (TODO)
    HallWithFallback {
        /// Start blending Hall→Observer (electrical rad/s)
        blend_low: f32,
        /// Full observer (electrical rad/s)
        blend_high: f32,
        /// Hall timeout before fallback (microseconds)
        timeout_us: u32,
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
    // never reorder the ones above.
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

#[allow(clippy::derivable_impls)] // Other variants have data, can't derive
impl Default for PhaseSource {
    fn default() -> Self {
        PhaseSource::Hall
    }
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
            PhaseSource::Hall
            | PhaseSource::Encoder
            | PhaseSource::Observer
            | PhaseSource::Hfi
            | PhaseSource::Manual
            | PhaseSource::OpenLoop => true,
            PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            }
            | PhaseSource::EncoderToObserver {
                blend_low,
                blend_high,
            } => blend_low.is_finite() && blend_high.is_finite(),
            PhaseSource::HallWithFallback {
                blend_low,
                blend_high,
                timeout_us: _,
            } => blend_low.is_finite() && blend_high.is_finite(),
            PhaseSource::HfiToObserver {
                min_vel,
                min_confidence,
            } => min_vel.is_finite() && min_confidence.is_finite(),
            PhaseSource::HfiToObserverVolts {
                toggle_v,
                min_confidence,
            } => toggle_v.is_finite() && toggle_v > 0.0 && min_confidence.is_finite(),
            PhaseSource::HfiToHall { switch_vel } | PhaseSource::HfiToEncoder { switch_vel } => {
                switch_vel.is_finite()
            }
        }
    }

    /// Check if this source requires a Hall sensor
    pub fn requires_hall(&self) -> bool {
        matches!(
            self,
            PhaseSource::Hall
                | PhaseSource::HallToObserver { .. }
                | PhaseSource::HallWithFallback { .. }
                | PhaseSource::HfiToHall { .. }
        )
    }

    /// Check if this source requires an encoder
    pub fn requires_encoder(&self) -> bool {
        matches!(
            self,
            PhaseSource::Encoder
                | PhaseSource::EncoderToObserver { .. }
                | PhaseSource::HfiToEncoder { .. }
        )
    }

    /// Check if this source requires an observer
    pub fn requires_observer(&self) -> bool {
        matches!(
            self,
            PhaseSource::Observer
                | PhaseSource::HallToObserver { .. }
                | PhaseSource::HallWithFallback { .. }
                | PhaseSource::EncoderToObserver { .. }
                | PhaseSource::HfiToObserver { .. }
                | PhaseSource::HfiToObserverVolts { .. }
        )
    }

    /// Check if this source requires HFI
    pub fn requires_hfi(&self) -> bool {
        matches!(
            self,
            PhaseSource::Hfi
                | PhaseSource::HfiToObserver { .. }
                | PhaseSource::HfiToObserverVolts { .. }
                | PhaseSource::HfiToHall { .. }
                | PhaseSource::HfiToEncoder { .. }
        )
    }

    /// Check if this is a manual/open-loop mode
    pub fn is_manual(&self) -> bool {
        matches!(self, PhaseSource::Manual | PhaseSource::OpenLoop)
    }
}
