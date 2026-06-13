//! Angle-source presets shared by the front-ends.
//!
//! The user picks from a short list of commutation sources; this maps that
//! choice onto the richer wire [`PhaseSource`], filling the crossover
//! parameters from one set of defaults. Keeping the mapping (and the magic
//! numbers) here is the whole point — the CLI and GUI used to carry separate
//! copies that could drift.

use oxifoc_core::foc::phase::PhaseSource;

/// Default crossover velocity for blended sources (electrical rad/s).
pub const DEFAULT_SWITCH_VEL: f32 = 150.0;
/// Default drive-voltage threshold for `HfiObserverVolts` (≈5 % of vbus).
pub const DEFAULT_TOGGLE_V: f32 = 2.0;
/// Default minimum observer confidence for an HFI→observer crossover.
pub const DEFAULT_MIN_CONFIDENCE: f32 = 0.5;

/// Simplified, UI-facing selection of an angle source. Variant order matches
/// the combo-box / CLI value-enum order so an index round-trips.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseSourceKind {
    /// Hall sensors only.
    Hall,
    /// Hall with observer fallback + velocity blend (default sensored mode).
    HallFallback,
    /// Back-EMF observer only (needs spin-up).
    Observer,
    /// HFI only (zero/low speed, salient motors).
    Hfi,
    /// HFI at standstill, blend to the back-EMF observer at speed.
    HfiObserver,
    /// Like `HfiObserver`, but the crossover criterion is drive voltage.
    HfiObserverVolts,
}

impl PhaseSourceKind {
    /// All kinds in selector order; the position is the combo-box index.
    pub const ALL: [Self; 6] = [
        Self::Hall,
        Self::HallFallback,
        Self::Observer,
        Self::Hfi,
        Self::HfiObserver,
        Self::HfiObserverVolts,
    ];

    /// Kind for a selector index (None out of range).
    #[must_use]
    pub fn from_index(index: i32) -> Option<Self> {
        usize::try_from(index)
            .ok()
            .and_then(|i| Self::ALL.get(i).copied())
    }
}

/// Map a simplified kind to the wire `PhaseSource`, filling crossover
/// parameters from the shared defaults / the supplied thresholds.
#[must_use]
pub fn preset(kind: PhaseSourceKind, switch_vel: f32, toggle_v: f32) -> PhaseSource {
    match kind {
        PhaseSourceKind::Hall => PhaseSource::Hall,
        PhaseSourceKind::HallFallback => PhaseSource::HallToObserver {
            blend_low: switch_vel,
            blend_high: switch_vel * 2.0,
        },
        PhaseSourceKind::Observer => PhaseSource::Observer,
        PhaseSourceKind::Hfi => PhaseSource::Hfi,
        PhaseSourceKind::HfiObserver => PhaseSource::HfiToObserver {
            min_vel: switch_vel,
            min_confidence: DEFAULT_MIN_CONFIDENCE,
        },
        PhaseSourceKind::HfiObserverVolts => PhaseSource::HfiToObserverVolts {
            toggle_v,
            min_confidence: DEFAULT_MIN_CONFIDENCE,
        },
    }
}

/// Short display label for the active source read back from telemetry.
#[must_use]
pub fn label(src: PhaseSource) -> &'static str {
    use PhaseSource as P;
    match src {
        P::Hall => "Hall",
        P::Encoder => "Encoder",
        P::Observer => "Observer",
        P::Hfi => "HFI",
        P::HallToObserver { .. } => "Hall→Obs",
        P::EncoderToObserver { .. } => "Enc→Obs",
        P::HfiToObserver { .. } => "HFI→Obs",
        P::HfiToObserverVolts { .. } => "HFI→Obs(V)",
        P::HfiToHall { .. } => "HFI→Hall",
        P::HfiToEncoder { .. } => "HFI→Enc",
        P::Manual => "Manual",
        P::OpenLoop => "OpenLoop",
    }
}
