//! Hall sensor angle estimation for BLDC/FOC motor control
//!
//! Hall sensors provide 6 discrete positions per electrical revolution.
//! This module converts Hall sensor states to electrical angles and can
//! interpolate between edges using the measured electrical speed (similar
//! to VESC `mcpwm_foc` low-speed handling).
//!
//! ## VESC-compatible features:
//! - **Rate limiting**: Limits angle rate of change to prevent current spikes
//! - **Explicit direction tracking**: Handles direction reversals cleanly
//! - **Low-speed threshold**: Disables interpolation below configurable velocity

use core::f32::consts::TAU;

use super::hall_calibration::HallTransitionGeometry;
use super::sensors::{
    AngleSample, AngleSensor, HallInterpolationInfo, HallSensorTrait, HallSnapshot,
};
use super::{angle_difference, wrap_angle};
use crate::foc::clamp_f32;

/// Hall state lookup table: maps raw sensor reading (0-7) to logical state (0-5)
///
/// Valid Hall sequence (CW rotation): 1 → 3 → 2 → 6 → 4 → 5 (repeat)
/// This corresponds to: 001 → 011 → 010 → 110 → 100 → 101 in binary
///
/// The table maps raw 3-bit Hall reading to normalized state:
/// - Invalid states (0, 7) map to 0 with error flag
/// - Valid states (1,2,3,4,5,6) map to their position in the sequence
pub const HALL_STATE_TABLE: [u8; 8] = [
    0, // 000 (invalid - all sensors low)
    0, // 001 (H1 only)
    2, // 010 (H2 only)
    1, // 011 (H1+H2)
    4, // 100 (H3 only)
    5, // 101 (H3+H1)
    3, // 110 (H3+H2)
    0, // 111 (invalid - all sensors high)
];

/// Direction of rotation
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Default,
    serde::Serialize,
    serde::Deserialize,
    postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Clockwise rotation
    Clockwise,
    /// Counter-clockwise rotation
    CounterClockwise,
    /// Motor stopped or direction unknown
    #[default]
    Stopped,
}

/// Default minimum eRPM for interpolation
/// Below this, use nearest Hall sector angle only.
/// VESC default is 500 eRPM
pub const DEFAULT_INTERP_MIN_ERPM: f32 = 500.0;

/// Default rate limit factor for angle changes
/// Limits angle step to max_rate = (velocity * dt * RATE_LIMIT_FACTOR)
/// VESC uses 1.5 (allows 50% overshoot for transients)
pub const DEFAULT_RATE_LIMIT_FACTOR: f32 = 1.5;

/// Sector-time multiple for staleness detection
/// ([`HallSensor::is_stale_at_speed`]): with a reference speed an edge is
/// expected every `current_sector_width / |ω|` seconds; this many sector times
/// with no edge means the sensor stopped reporting. Only consulted with
/// INDEPENDENT evidence of rotation (the caller's `omega_ref`), which is
/// what makes a tight window safe: past the sector end the interpolated
/// angle is clamped, and on a low-impedance motor the frame error grows
/// into amps within a sector — the old 8× window (5.6 ms at 1500 rad/s)
/// let a dying hall reach +32 A of true id and the overcurrent trip first.
pub const STALE_SECTOR_MULTIPLE: f32 = 1.5;

/// Absolute floor for the staleness window (seconds): a few PWM periods,
/// so capture/ISR jitter at very high speed can't trip it.
pub const STALE_MIN_WINDOW_S: f32 = 0.000_3;

/// Maximum electrical speed a hall edge pair is allowed to claim (rad/s).
///
/// 30 000 el rad/s ≈ 286 kERPM — several times any supported motor's
/// ceiling (a Flipsky 5065 at 60 V tops out near 12 k el rad/s), yet ~30×
/// below what a connector-bounce edge pair produces (`dt_ticks.max(1)` at a
/// 1 MHz timebase reads a 1 µs "sector" as ~10⁶ rad/s). Edges whose implied
/// speed exceeds this are glitches: the velocity estimate keeps its
/// previous value instead of adopting them.
pub const MAX_PLAUSIBLE_E_RAD_S: f32 = 30_000.0;

/// Validated, normalized Hall reference, prepared outside the control ISR.
pub struct PreparedHallCalibration {
    calibration: HallCalibration,
    geometry: Option<HallTransitionGeometry>,
}

impl PreparedHallCalibration {
    pub(crate) fn nominal() -> Self {
        Self {
            calibration: HallCalibration::default(),
            geometry: None,
        }
    }

    #[cfg(feature = "storage")]
    pub fn from_config(config: &crate::storage::HallCalibrationConfig) -> Option<Self> {
        if !config.is_calibrated() {
            return None;
        }
        let geometry = match &config.geometry {
            Some(geometry) => Some(geometry.validated(config.angles)?),
            None => None,
        };
        Some(Self {
            calibration: HallCalibration::from_raw_table(config.angles, 0.0),
            geometry,
        })
    }

    pub fn from_result(result: &super::hall_calibration::HallCalibrationResult) -> Option<Self> {
        if !(result.is_valid()
            && result.valid[1..=6].iter().all(|valid| *valid)
            && result.angles[1..=6].iter().all(|angle| angle.is_finite()))
        {
            return None;
        }
        // Results expose their centroid table publicly. Revalidate a measured
        // geometry against the current table instead of trusting its old frame.
        let geometry = match result.geometry {
            Some(geometry) => Some(HallTransitionGeometry::try_from_raw_parts(
                geometry.entry_angles(),
                geometry.next_states(),
                result.angles,
            )?),
            None => None,
        };
        Some(Self {
            calibration: HallCalibration::from_raw_table(result.angles, 0.0),
            geometry,
        })
    }
}

/// One capture-clock rate and its floating-point representation. Keeping the
/// conversion here avoids repeated software u64-to-f32 helpers on Cortex-M4.
struct HallTimebase {
    ticks_per_sec: core::num::NonZeroU64,
    hz: f32,
}

impl HallTimebase {
    fn new(ticks_per_sec: u64) -> Self {
        let ticks_per_sec =
            core::num::NonZeroU64::new(ticks_per_sec).unwrap_or(core::num::NonZeroU64::MIN);
        Self {
            ticks_per_sec,
            hz: ticks_per_sec.get() as f32,
        }
    }

    #[inline]
    fn seconds(&self, ticks: u64) -> f32 {
        crate::foc::fast_math::u64_to_f32(ticks) / self.hz
    }
}

/// Platform-agnostic Hall sensor angle estimator
///
/// Tracks Hall sensor state transitions to estimate electrical angle
/// and direction of rotation.
///
/// Returns electrical angle (0 to 2π per electrical revolution),
/// which completes every 6 Hall states regardless of motor pole count.
///
/// ## Features (VESC-compatible):
/// - Rate limiting (prevents current spikes on Hall transitions)
/// - Explicit direction reversal tracking
/// - Low-speed interpolation threshold
pub struct HallSensor {
    /// Electrical angle increment per Hall state change (TAU / 6)
    angle_per_state: f32,
    /// Optional measured boundary topology; legacy centroid-only tables retain
    /// the nominal sector equations.
    geometry: Option<HallTransitionGeometry>,
    /// Width of the current valid sector, cached at accepted edges/configuration.
    current_sector_width: f32,
    /// Calibration table: electrical angle (rad) for each logical Hall state (0-5)
    calib: HallCalibration,
    /// Interpolation base angle at the last Hall edge (radians, 0 to 2π).
    ///
    /// A measured transition boundary when available, otherwise the legacy
    /// midpoint of adjacent centroids. Unequal sector widths cannot in general
    /// be recovered from centroids alone. Falls back to the new centroid when
    /// no usable previous sector establishes an edge.
    angle: f32,
    /// Whether `angle` is a sector boundary rather than a boot centroid.
    /// Gates the measured-traversal velocity estimate: the distance
    /// between two boundaries is a real sector width; the distance from a
    /// centroid fallback to a boundary is not.
    base_is_boundary: bool,
    /// Raw 3-bit Hall state (0-7)
    raw_state: u8,
    /// Logical Hall state (0-5)
    logical_state: u8,
    /// Current direction of rotation
    direction: Direction,
    /// Previous direction (for reversal detection)
    prev_direction: Direction,
    /// Direction reversal detected on last update
    direction_reversed: bool,
    /// Last edge timestamp (ticks of a caller-provided clock)
    last_edge_ticks: Option<u64>,
    /// Electrical velocity estimate (rad/s) from last edge
    elec_velocity: f32,
    /// Last VALID raw state (1..=6): the sector the rotor was last seen in.
    /// Survives an invalid-state glitch. 0 before the first valid reading.
    valid_raw: u8,
    /// An invalid state (0/7) interrupted the edge stream since the last
    /// valid reading.
    in_glitch: bool,
    /// A skipped measured transition has no trusted boundary timestamp. Wait
    /// for an adjacent captured edge before publishing a frame again.
    geometry_resync: bool,
    /// When that glitch began (ticks).
    glitch_start_ticks: u64,
    /// Upcoming edges whose timing is unusable for a speed estimate (the
    /// edge a glitch hid, and the next one measured from it): the
    /// pre-glitch speed is kept through them.
    velocity_hold_edges: u8,
    /// Error counter for invalid states or transitions
    error_count: u32,
    /// Timebase (ticks per second)
    timebase: HallTimebase,
    /// Minimum eRPM for interpolation. Below this, use sector angle only.
    interp_min_erpm: f32,
    /// The same configured minimum in rad/s, used by the angle rate limiter.
    /// Keep the eRPM comparison separate to preserve its rounded boundary.
    interp_min_rad_s: f32,
    /// Rate limit factor (1.5 = allow 50% overshoot, VESC default)
    rate_limit_factor: f32,
    /// Rate-limited angle (tracked across samples for smooth limiting)
    rate_limited_angle: f32,
    /// Last sample timestamp for rate limiting dt calculation
    last_sample_ticks: Option<u64>,

    // === Wire-health observation window (see note_wire_activity) ===
    /// Per-bit transition counts within the current window (saturating)
    wire_bit_changes: [u8; 3],
    /// Observed transitions (update calls) in the current window
    wire_window_updates: u8,
    /// Invalid-state (0/7) events in the current window
    wire_window_invalids: u8,
    /// `error_count` snapshot at the window start (for the rate delta)
    errors_at_window_start: u32,
    /// Latest window verdict: bitmask of dead wires (0 = none)
    wire_dead_mask: u8,
    /// Latest window verdict: error rate above threshold
    error_rate_flag: bool,
    /// Whether at least one state has been observed (the POR prev state
    /// must not feed the bit counters — see note_wire_activity)
    wire_primed: bool,
}

/// Snapshot of the Hall estimator state after a valid edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HallReading {
    pub angle_rad: f32,
    pub direction: Direction,
    pub state: u8,
    pub elec_velocity: f32,
    pub t_ticks: u64,
}

/// Error when processing a Hall edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HallError {
    InvalidState(u8),
}

/// What is wrong with the hall sensor — the payload of the `HallError`
/// warning fault (docs/notes/fault-overhaul.md §5). Produced by the wire
/// detectors below ([`HallSensor::fault_kind`]) and by the health tracking
/// in `PhaseManager` (stale / invalid).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HallFaultKind {
    /// Invalid state (0b000/0b111) observed — wiring or pull-up trouble.
    InvalidState,
    /// Edges stopped while the rotor was spinning (velocity-adaptive
    /// staleness) — dead cable or dead sensor.
    StaleAtSpeed,
    /// Specific hall wires identified as dead (bit i set = wire H{i+1},
    /// the H1/H2/H3 inputs). See the per-bit detector in
    /// [`HallSensor::note_wire_activity`].
    WireDead {
        /// Bitmask of dead wires (bit 0 = H1, bit 1 = H2, bit 2 = H3)
        mask: u8,
    },
    /// Error rate over the observation window exceeded the threshold —
    /// stochastic glitches (contact bounce, EMI) rather than a clean break.
    ErrorRate,
}

impl HallFaultKind {
    /// Human-readable details for the protocol `FaultInfo` — shared by the
    /// platform fault enums so the wording stays identical everywhere.
    pub fn details(&self) -> heapless::String<128> {
        let mut s = heapless::String::new();
        match self {
            Self::InvalidState => {
                let _ = s.push_str("invalid state (0/7) - wiring/pull-up");
            }
            Self::StaleAtSpeed => {
                let _ = s.push_str("edges stopped at speed - dead cable/sensor");
            }
            Self::WireDead { mask } => {
                let _ = s.push_str("dead wire:");
                for bit in 0..3u8 {
                    if mask & (1 << bit) != 0 {
                        let _ = s.push_str(match bit {
                            0 => " H1",
                            1 => " H2",
                            _ => " H3",
                        });
                    }
                }
            }
            Self::ErrorRate => {
                let _ = s.push_str("high error rate - bounce/EMI");
            }
        }
        s
    }
}

/// Wire-health observation window length, in observed hall transitions
/// (`update()` calls). 24 transitions = 4 electrical revolutions on a
/// healthy sensor (6 edges/rev), 6 revolutions with one dead wire (4
/// observed transitions/rev — two real edges are masked, one lands on an
/// invalid state), 12 with two dead wires. Long enough that every live
/// bit must show activity, short enough to flag a break within roughly
/// one wheel revolution at riding speed.
pub const WIRE_WINDOW_UPDATES: u8 = 24;

/// Minimum invalid-state events inside a window for a `WireDead` verdict
/// (≈ one per electrical revolution with a dead wire — see
/// [`HallSensor::note_wire_activity`]); two = debounce against a single
/// boot/connect glitch.
pub const WIRE_WINDOW_MIN_INVALIDS: u8 = 2;

/// `error_count` delta over one observation window that flags
/// [`HallFaultKind::ErrorRate`]. A healthy sensor contributes zero; a
/// single EMI glitch contributes one or two — the threshold sits above
/// both, while bounce or a flaky contact (errors every few edges) clears
/// it within a window.
pub const ERROR_RATE_WINDOW_THRESHOLD: u32 = 4;

impl Default for HallSensor {
    fn default() -> Self {
        Self::new(1)
    }
}

impl HallSensor {
    /// Create a new Hall sensor estimator
    ///
    /// Returns **electrical angle** (0 to 2π per electrical revolution),
    /// which completes every 6 Hall states.
    ///
    /// To convert to mechanical angle, divide by pole pairs:
    /// `mechanical_angle = electrical_angle / pole_pairs`
    pub fn new(ticks_per_sec: u64) -> Self {
        // Electrical angle completes 2π every 6 Hall states (one electrical revolution)
        let angle_per_state = TAU / 6.0;

        Self {
            angle_per_state,
            geometry: None,
            current_sector_width: angle_per_state,
            calib: HallCalibration::default(),
            angle: 0.0,
            base_is_boundary: false,
            raw_state: 0,
            logical_state: 0,
            direction: Direction::Stopped,
            prev_direction: Direction::Stopped,
            direction_reversed: false,
            last_edge_ticks: None,
            elec_velocity: 0.0,
            valid_raw: 0,
            in_glitch: false,
            geometry_resync: false,
            glitch_start_ticks: 0,
            velocity_hold_edges: 0,
            error_count: 0,
            timebase: HallTimebase::new(ticks_per_sec),
            interp_min_erpm: DEFAULT_INTERP_MIN_ERPM,
            interp_min_rad_s: DEFAULT_INTERP_MIN_ERPM * TAU / 60.0,
            rate_limit_factor: DEFAULT_RATE_LIMIT_FACTOR,
            rate_limited_angle: 0.0,
            last_sample_ticks: None,
            wire_bit_changes: [0; 3],
            wire_window_updates: 0,
            wire_window_invalids: 0,
            errors_at_window_start: 0,
            wire_dead_mask: 0,
            error_rate_flag: false,
            wire_primed: false,
        }
    }

    /// Set minimum eRPM for interpolation.
    ///
    /// Below this eRPM, interpolation is disabled and the nearest
    /// Hall sector angle is used. This prevents oscillation during
    /// slow direction reversals. Default: 500 eRPM (VESC default).
    pub fn set_interp_min_erpm(&mut self, erpm: f32) {
        self.interp_min_erpm = erpm.abs();
        self.interp_min_rad_s = self.interp_min_erpm * TAU / 60.0;
    }

    /// Get minimum interpolation eRPM
    pub fn interp_min_erpm(&self) -> f32 {
        self.interp_min_erpm
    }

    /// Convert electrical velocity (rad/s) to eRPM
    #[inline]
    fn vel_to_erpm(vel_rad_s: f32) -> f32 {
        vel_rad_s.abs() * 60.0 / TAU
    }

    /// Set rate limit factor for angle changes.
    ///
    /// Limits angle step per sample to `velocity * dt * factor`.
    /// VESC uses 1.5 (allows 50% overshoot for transients).
    /// Higher = less limiting, lower = smoother but slower response.
    pub fn set_rate_limit_factor(&mut self, factor: f32) {
        self.rate_limit_factor = factor.max(0.1);
    }

    /// Get rate limit factor
    pub fn rate_limit_factor(&self) -> f32 {
        self.rate_limit_factor
    }

    /// Staleness against an independent speed estimate: edges stopped
    /// arriving although the rotor demonstrably turns at `omega_ref`.
    ///
    /// Edge timing alone cannot separate a stopped rotor from a dead
    /// sensor (a coasting stop leaves the last edge's speed behind), so the
    /// caller must supply evidence of rotation — the back-EMF observer's
    /// speed. The expected sector time uses the SLOWER of the two speeds:
    /// the observer's velocity lags a deceleration, and the hall's last
    /// edge speed is the better bound there. `omega_ref = 0` never trips.
    pub fn is_stale_at_speed(&self, now_ticks: u64, omega_ref: f32) -> bool {
        let Some(dt_ticks) = self.time_since_edge(now_ticks) else {
            // No edge ever seen: "no data yet", not "data stopped".
            return false;
        };
        let vel = self.elec_velocity.abs().min(omega_ref.abs());
        if vel < 1.0 {
            return false;
        }
        let dt_s = self.timebase.seconds(dt_ticks);
        let expected_sector_s = self.current_sector_width / vel;
        dt_s > (expected_sector_s * STALE_SECTOR_MULTIPLE).max(STALE_MIN_WINDOW_S)
    }

    /// Get time since last Hall edge in ticks.
    ///
    /// Returns `None` if no edge has been received yet.
    pub fn time_since_edge(&self, now_ticks: u64) -> Option<u64> {
        // saturating, not wrapping: an edge timestamped just ahead of the
        // caller's "now" snapshot (capture ISR vs FOC ISR race) must read as
        // dt = 0, not as a huge wrapped interval.
        self.last_edge_ticks
            .map(|last| now_ticks.saturating_sub(last))
    }

    /// Age of the last captured edge in seconds, in this sensor's clock domain.
    pub fn time_since_edge_s(&self, now_ticks: u64) -> Option<f32> {
        self.time_since_edge(now_ticks)
            .map(|ticks| self.timebase.seconds(ticks))
    }

    /// Get time since last Hall edge in microseconds.
    ///
    /// Returns `None` if no edge has been received yet.
    pub fn time_since_edge_us(&self, now_ticks: u64) -> Option<u32> {
        self.time_since_edge(now_ticks)
            .map(|ticks| ((ticks * 1_000_000) / self.timebase.ticks_per_sec.get()) as u32)
    }

    /// Per-bit wire-break detector — accounting half (the verdict is taken
    /// at the end of each observation window).
    ///
    /// Physics: over one electrical revolution every live hall input
    /// toggles exactly twice (the six states form a Gray-code cycle — one
    /// bit changes per edge, each bit every third edge). A broken wire is
    /// therefore a bit whose transition counter stays at ZERO while the
    /// other counters tick — and the zero bit NAMES the dead wire for the
    /// fault details ("hall wire H2 dead" → that's the pin to buzz out).
    /// Works for one or two dead wires, at any rotation speed, with no
    /// platform support: we already receive `raw_state` on every edge, so
    /// XOR against the previous state is all the accounting needed.
    ///
    /// THE INVALID-STATE GATE (why counters alone are not enough): rocking
    /// the rotor back and forth across ONE sector boundary — vibration at
    /// standstill, a board nudged on a slope — toggles a single bit
    /// legitimately, leaving the other two counters at zero: by counters
    /// alone that reads as "two wires dead". But rocking never produces an
    /// invalid state, while real rotation with a stuck bit ALWAYS does,
    /// once per electrical revolution regardless of stuck polarity: the
    /// sector where the other two bits are both low reads 0b000 (bit stuck
    /// low) or the one where both are high reads 0b111 (stuck high). So a
    /// `WireDead` verdict additionally requires invalid-state events in
    /// the same window — rocking produces none and stays verdict-free.
    ///
    /// Limits (accepted): a rotor that never moves gives no edges and no
    /// verdict (nothing to detect, nothing at risk); a sensor frozen at a
    /// VALID state since boot is indistinguishable from a parked rotor
    /// here — that case needs the hall-vs-observer consistency check
    /// (TODO.md, saliency-monitor ladder).
    fn note_wire_activity(&mut self, prev_raw: u8, raw_state: u8) {
        // The power-on `prev_raw` (0) is not an observed hall state:
        // counting the first transition against it credits phantom bit
        // activity — a stuck-HIGH wire would show one "toggle" crossing
        // from POR and dodge the verdict for a whole window.
        if !self.wire_primed {
            self.wire_primed = true;
            return;
        }
        let changed = (prev_raw ^ raw_state) & 0x07;
        for bit in 0..3 {
            if changed & (1 << bit) != 0 {
                self.wire_bit_changes[bit] = self.wire_bit_changes[bit].saturating_add(1);
            }
        }
        if raw_state == 0 || raw_state > 6 {
            self.wire_window_invalids = self.wire_window_invalids.saturating_add(1);
        }
        self.wire_window_updates += 1;
        if self.wire_window_updates >= WIRE_WINDOW_UPDATES {
            // Verdicts are per-window snapshots, not latches: the sticky
            // memory lives in the fault registry (set-only bridge in
            // run_foc_cycle); here a clean window clears the flags so the
            // live behavior (and the fault details) track the present.
            self.wire_dead_mask = if self.wire_window_invalids >= WIRE_WINDOW_MIN_INVALIDS {
                let mut mask = 0u8;
                for bit in 0..3 {
                    if self.wire_bit_changes[bit] == 0 {
                        mask |= 1 << bit;
                    }
                }
                mask
            } else {
                0
            };
            self.error_rate_flag = self.error_count.wrapping_sub(self.errors_at_window_start)
                >= ERROR_RATE_WINDOW_THRESHOLD;
            self.wire_bit_changes = [0; 3];
            self.wire_window_updates = 0;
            self.wire_window_invalids = 0;
            self.errors_at_window_start = self.error_count;
        }
    }

    /// Latest wire-health verdict (most specific first): named dead wires,
    /// else an excessive error rate. `None` = the last window looked
    /// healthy. Stale/invalid health kinds are layered on top by
    /// `PhaseManager::hall_fault`.
    pub fn fault_kind(&self) -> Option<HallFaultKind> {
        if self.wire_dead_mask != 0 {
            Some(HallFaultKind::WireDead {
                mask: self.wire_dead_mask,
            })
        } else if self.error_rate_flag {
            Some(HallFaultKind::ErrorRate)
        } else {
            None
        }
    }

    /// Check if direction reversal was detected on last update
    pub fn direction_reversed(&self) -> bool {
        self.direction_reversed
    }

    /// Get previous direction (before last update)
    pub fn prev_direction(&self) -> Direction {
        self.prev_direction
    }

    /// Update angle based on new Hall sensor reading and timestamp
    ///
    /// # Arguments
    /// * `raw_state` - 3-bit Hall sensor value (H3<<2 | H2<<1 | H1)
    /// * `t_ticks` - Monotonic time in ticks (of `ticks_per_sec`)
    ///
    /// # Returns
    /// * `Some(angle)` - Electrical angle at the edge in radians (0 to 2π)
    /// * `None` - Invalid Hall state (0 or 7)
    pub fn update(&mut self, raw_state: u8, t_ticks: u64) -> Option<f32> {
        // Previous raw state (for the boundary midpoint) before overwriting
        let prev_raw = self.raw_state;
        // Store raw state
        self.raw_state = raw_state;

        // Wire-health accounting runs BEFORE the invalid early-return:
        // transitions into/out of 0/7 are bit activity too, and the
        // invalid events themselves are the detector's gating signal.
        self.note_wire_activity(prev_raw, raw_state);

        // Check for invalid states (all low or all high)
        if raw_state == 0 || raw_state > 6 {
            self.error_count += 1;
            // A glitch (connector bounce, EMI) or a disconnect. Keep the
            // speed, direction, last edge and base angle — VESC keeps its
            // speed estimate across invalid readings too. Resetting them
            // made the hall report ω = 0 at full speed once the signal came
            // back (interpolation stopped, the back-EMF feedforward
            // vanished: iq −22 A), and the glitch's END then counted as an
            // edge mid-sector, so the next speed estimate came out 1.6×
            // high (+38 A, overcurrent) — a 0.2 ms glitch at speed tripped
            // 10 of 16 closed-loop timings. A disconnect still shows:
            // sample() is None while the state is invalid, and staleness /
            // the speed decay take over from there.
            if !self.in_glitch {
                self.in_glitch = true;
                self.glitch_start_ticks = t_ticks;
            }
            return None;
        }

        // Back from a glitch in the SAME sector: no edge happened.
        let mut t_ticks = t_ticks;
        if core::mem::take(&mut self.in_glitch) {
            if raw_state == self.valid_raw {
                return if self.geometry_resync {
                    None
                } else {
                    Some(self.angle)
                };
            }
            // The glitch hid an edge. The new sector is right, the time is
            // not: the rotor crossed the boundary somewhere inside the
            // glitch, and anchoring the boundary at the glitch's END put
            // the frame up to a sector behind the rotor (−0.9 rad after a
            // 1 ms glitch at 1500 rad/s — overcurrent in a few periods).
            // Place the crossing where the pre-glitch speed says it was,
            // inside the glitch window, and keep that speed for this edge
            // and the next (measured from this estimated time).
            if let Some(last) = self.last_edge_ticks {
                let speed = self.elec_velocity.abs();
                let t_cross = if speed > 1.0 {
                    let sector_s = self.current_sector_width / speed;
                    last.saturating_add((sector_s * self.timebase.hz) as u64)
                } else {
                    t_ticks
                };
                t_ticks = t_cross.clamp(self.glitch_start_ticks, t_ticks);
            }
            self.velocity_hold_edges = 2;
        }
        let prev_raw = self.valid_raw;
        let resuming_geometry = self.geometry_resync;

        if let Some(geometry) = self.geometry.as_ref() {
            if prev_raw == raw_state {
                // A same-state capture is not a new boundary or speed sample.
                return if self.geometry_resync {
                    None
                } else {
                    Some(self.angle)
                };
            }
            if prev_raw != 0
                && geometry
                    .boundary_for_transition(prev_raw, raw_state)
                    .is_none()
            {
                // A skipped transition does not identify a single crossed
                // boundary. Preserve the trusted motion, but withhold samples
                // until a real adjacent edge supplies a known anchor and time.
                self.error_count += 1;
                self.valid_raw = raw_state;
                self.logical_state = HALL_STATE_TABLE[usize::from(raw_state)];
                self.base_is_boundary = false;
                self.geometry_resync = true;
                return None;
            }
            if self.geometry_resync {
                self.geometry_resync = false;
                // This captured edge has a real timestamp; the next adjacent
                // edge completes the first fully known new-frame interval.
                self.velocity_hold_edges = 1;
            }
        }

        let prev_state = self.logical_state;
        let current_state = HALL_STATE_TABLE[raw_state as usize];

        // Detect direction based on state transitions
        let is_wrap_cw = prev_state == 5 && current_state == 0;
        let is_wrap_ccw = prev_state == 0 && current_state == 5;
        if prev_raw != 0 && current_state.abs_diff(prev_state) > 1 && !is_wrap_cw && !is_wrap_ccw {
            // Non-adjacent transition (not wraparound) => error
            // The first valid sector seeds position; there was no prior edge.
            self.error_count += 1;
        }

        // Save previous direction before updating
        self.prev_direction = self.direction;

        // Measured geometry records commanded positive orientation explicitly.
        // Even valid adjacent centroids can be >π apart in wide sectors, so
        // their shortest difference must not override that topology. Legacy
        // tables retain their calibrated-centroid direction calculation.
        let new_direction = if let Some(geometry) = self.geometry.as_ref() {
            let next = geometry.next_states();
            if next[usize::from(prev_raw)] == raw_state {
                Direction::Clockwise
            } else if next[usize::from(raw_state)] == prev_raw {
                Direction::CounterClockwise
            } else {
                self.direction
            }
        } else if self.calib.is_valid_state(prev_raw) {
            let step = angle_difference(
                self.calib.angle_for_state(raw_state),
                self.calib.angle_for_state(prev_raw),
            );
            if step > 0.0 {
                Direction::Clockwise
            } else if step < 0.0 {
                Direction::CounterClockwise
            } else {
                self.direction
            }
        } else {
            self.direction // no previous state to compare with
        };

        // Explicit direction reversal detection (VESC-style)
        self.direction_reversed = matches!(
            (self.prev_direction, new_direction),
            (Direction::Clockwise, Direction::CounterClockwise)
                | (Direction::CounterClockwise, Direction::Clockwise)
        );

        self.direction = new_direction;
        self.logical_state = current_state;
        if resuming_geometry
            || (self.geometry.is_some() && self.velocity_hold_edges > 0 && self.direction_reversed)
        {
            // The adjacent edge restores direction too. Keep the last trusted
            // speed magnitude, with that direction, until a full known sector
            // interval can replace it; never interpolate the new frame forward
            // after a reverse recovery, including a reverse edge hidden by an
            // invalid-state glitch whose timing deliberately remains held.
            self.elec_velocity =
                self.angle_step_signed(new_direction).signum() * self.elec_velocity.abs();
        }

        // An adjacent edge identifies its measured boundary. Legacy tables
        // retain VESC's centroid-midpoint estimate; equal-looking centroids
        // alone cannot identify alternating narrow/wide sectors. The centroid
        // remains the low-speed/boot fallback, with control-angle rate limiting.
        let prev_base = self.angle;
        let prev_base_is_boundary = self.base_is_boundary;
        let centroid_new = self.calib.angle_for_state(raw_state);
        let had_prev_edge = self.last_edge_ticks.is_some();
        let crossed_boundary =
            had_prev_edge && raw_state != prev_raw && self.calib.is_valid_state(prev_raw);
        let base = if crossed_boundary {
            if let Some(boundary) = self
                .geometry
                .as_ref()
                .and_then(|g| g.boundary_for_transition(prev_raw, raw_state))
            {
                wrap_angle(boundary + self.calib.advance_rad)
            } else {
                let centroid_prev = self.calib.angle_for_state(prev_raw);
                wrap_angle(centroid_prev + angle_difference(centroid_new, centroid_prev) / 2.0)
            }
        } else {
            centroid_new
        };

        // Velocity estimate from last edge (not across a glitch — see above)
        if self.velocity_hold_edges > 0 {
            self.velocity_hold_edges -= 1;
        } else if let Some(last_t) = self.last_edge_ticks {
            let dt_ticks = t_ticks.wrapping_sub(last_t).max(1);
            let dt = self.timebase.seconds(dt_ticks);

            // Direction reversal: the rotor left the sector through the
            // boundary it entered by — no sector width was traversed, and
            // the speed passed through zero inside the interval. A rotor
            // that turns around symmetrically leaves about as fast as it
            // came in, and cannot have averaged more than one sector over
            // the interval: take the smaller, with the new sign. The
            // interval alone read a slow rotor rocking across a boundary
            // as hundreds of rad/s (5 ms → 209 rad/s at 5 rad/s) — a
            // velocity spike at every low-speed turnaround.
            let candidate = if self.geometry.is_some() && !prev_base_is_boundary {
                // Boot/configuration knows a sector centroid, not when its
                // entry boundary was crossed. The next real interval is the
                // first one that can measure a full sector's speed.
                self.elec_velocity
            } else if self.direction_reversed {
                let angle_step = self.angle_step_signed(self.direction);
                let speed_bound = if self.geometry.is_some() {
                    self.current_sector_width / dt
                } else {
                    (angle_step / dt).abs()
                };
                angle_step.signum() * self.elec_velocity.abs().min(speed_bound)
            } else if prev_base_is_boundary && crossed_boundary {
                // Boundary-to-boundary distance is the MEASURED width of the
                // sector just traversed (handles asymmetric Hall placement —
                // MESC carries the same per-state width for its angle step).
                angle_difference(base, prev_base) / dt
            } else {
                self.angle_step_signed(self.direction) / dt
            };
            // Plausibility gate: an edge-bounce pair from a flaky connector
            // arrives ticks apart and would read as ~10⁵–10⁶ rad/s — an
            // impossible speed that then feeds the dq-decoupling
            // feedforward and the actuation advance as a full-bus torque
            // pulse (the bench-observed OC-from-hall-bounce class). Keep
            // the previous estimate; the angle/timestamp update above
            // stands, and real edges re-converge the velocity immediately.
            if candidate.abs() <= MAX_PLAUSIBLE_E_RAD_S {
                self.elec_velocity = candidate;
            }
        }

        self.last_edge_ticks = Some(t_ticks);
        self.angle = base;
        self.base_is_boundary = crossed_boundary;
        self.valid_raw = raw_state;
        self.current_sector_width = self
            .geometry
            .as_ref()
            .and_then(|g| g.sector_width(raw_state))
            .unwrap_or(self.angle_per_state);

        // Deliberately NOT snapping rate_limited_angle here: the edge is
        // where the up-to-60° discontinuity happens, i.e. exactly what the
        // rate limiter exists to smooth. VESC keeps its rate-limited angle
        // across transitions and always slews it by at most one step
        // (foc_math.c:665-672). Only the first smoothed sample seeds an initial
        // frame; every later edge retains the rate-limiter history.

        Some(self.angle)
    }

    /// Update and return a full reading (angle, dir, velocity) or an error.
    pub fn update_sample(&mut self, raw_state: u8, t_ticks: u64) -> Result<HallReading, HallError> {
        match self.update(raw_state, t_ticks) {
            Some(angle) => Ok(HallReading {
                angle_rad: angle,
                direction: self.direction,
                state: self.logical_state,
                elec_velocity: self.elec_velocity,
                t_ticks,
            }),
            None => Err(HallError::InvalidState(raw_state)),
        }
    }

    /// Get current electrical angle in radians (0 to 2π)
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Get current rotation direction
    pub fn direction(&self) -> Direction {
        self.direction
    }

    /// Get error count (invalid states or transitions)
    pub fn error_count(&self) -> u32 {
        self.error_count
    }

    /// Reset error counter
    pub fn reset_errors(&mut self) {
        self.error_count = 0;
        // Restart the wire-health window too: the verdicts derive from the
        // same evidence the host just acknowledged/cleared.
        self.wire_bit_changes = [0; 3];
        self.wire_window_updates = 0;
        self.wire_window_invalids = 0;
        self.errors_at_window_start = 0;
        self.wire_dead_mask = 0;
        self.error_rate_flag = false;
    }

    /// Get the logical Hall state (0-5)
    pub fn state(&self) -> u8 {
        self.logical_state
    }

    /// Get the raw Hall state (0-7)
    pub fn raw_state(&self) -> u8 {
        self.raw_state
    }

    /// Get current timing advance (radians)
    pub fn advance(&self) -> f32 {
        self.calib.advance_rad
    }

    /// Get ticks per second
    pub fn ticks_per_sec(&self) -> u64 {
        self.timebase.ticks_per_sec.get()
    }

    /// Calibrate Hall table using raw-state angles (8-entry table).
    ///
    /// This is the preferred method as it works with any Hall sensor wiring.
    /// Angles should be in [0, 2π). States 0 and 7 are invalid.
    pub fn set_calibration_raw(&mut self, raw_table: [f32; 8]) {
        self.calib = HallCalibration::from_raw_table(raw_table, self.calib.advance_rad);
        self.geometry = None;
        self.current_sector_width = self.angle_per_state;
        self.geometry_resync = false;
    }

    /// Calibrate Hall table using logical-state angles (6-entry table).
    ///
    /// For backwards compatibility. Angles should be in [0, 2π) for the
    /// clockwise logical sequence 0→1→2→3→4→5.
    pub fn set_calibration(&mut self, table_rad: [f32; 6]) {
        self.calib = HallCalibration::from_logical_table(table_rad, self.calib.advance_rad);
        self.geometry = None;
        self.current_sector_width = self.angle_per_state;
        self.geometry_resync = false;
    }

    /// Apply calibration result from `HallCalibrator`
    ///
    /// Uses the raw-state angles directly from calibration result.
    /// Returns `true` if calibration was valid and applied, `false` otherwise.
    pub fn apply_calibration(
        &mut self,
        result: &super::hall_calibration::HallCalibrationResult,
    ) -> bool {
        if let Some(prepared) = PreparedHallCalibration::from_result(result) {
            self.install_prepared_calibration(&prepared);
            true
        } else {
            false
        }
    }

    /// Reanchor the angle frame without inventing a physical edge. A parked
    /// valid sector stays observable; speed resumes after two new boundaries.
    pub fn install_prepared_calibration(
        &mut self,
        prepared: &PreparedHallCalibration,
    ) -> super::sensors::HallReferenceUpdate {
        let advance = self.calib.advance_rad;
        self.calib = prepared.calibration;
        self.calib.advance_rad = advance;
        self.geometry = prepared.geometry;
        self.current_sector_width = self
            .geometry
            .and_then(|geometry| geometry.sector_width(self.valid_raw))
            .unwrap_or(self.angle_per_state);
        self.geometry_resync = false;
        self.angle = self.calib.angle_for_state(self.valid_raw);
        self.base_is_boundary = false;
        self.elec_velocity = 0.0;
        self.direction = Direction::Stopped;
        self.prev_direction = Direction::Stopped;
        self.direction_reversed = false;
        self.velocity_hold_edges = 1;
        self.last_sample_ticks = None;
        self.rate_limited_angle = self.angle;
        super::sensors::HallReferenceUpdate {
            sample: (self.last_edge_ticks.is_some() && !self.in_glitch).then_some(AngleSample {
                angle: self.angle,
                omega: 0.0,
                direction: Direction::Stopped,
            }),
        }
    }

    /// Set additional electrical advance (radians) applied to calibrated angles.
    pub fn set_advance(&mut self, advance_rad: f32) {
        self.calib.advance_rad = advance_rad;
    }

    /// Interpolated sample at `now_ticks` (immutable version).
    ///
    /// For the rate-limited VESC-style angle, use `sample_at_mut()` instead.
    ///
    /// This version applies:
    /// - Below `interp_min_vel`: returns sector angle only (no interpolation)
    /// - Above threshold: linear interpolation from the last boundary, at
    ///   most one sector (see `decayed_velocity`)
    pub fn sample_at(&self, now_ticks: u64) -> Option<AngleSample> {
        self.sample_with_edge_age_at(now_ticks)
            .map(|(sample, _)| sample)
    }

    /// Raw immutable telemetry snapshot, with the exact edge age used to
    /// calculate its angle and velocity. This avoids a second time conversion.
    pub fn snapshot_at(&self, now_ticks: u64) -> Option<HallSnapshot> {
        self.sample_with_edge_age_at(now_ticks)
            .map(|(sample, edge_age_s)| HallSnapshot {
                angle_rad: sample.angle,
                velocity_rad_s: sample.omega,
                direction: sample.direction,
                state: self.logical_state,
                error_count: self.error_count,
                edge_age_s,
            })
    }

    #[inline]
    fn sample_with_edge_age_at(&self, now_ticks: u64) -> Option<(AngleSample, f32)> {
        let t0 = self.last_edge_ticks?;
        if self.in_glitch || self.geometry_resync {
            // Invalid state right now: no angle (the caller falls back).
            return None;
        }
        // saturating: a wrapped "negative" dt as f32 would be ~2^64 and
        // collapse the decayed velocity to zero for the cycle.
        let dt = self.timebase.seconds(now_ticks.saturating_sub(t0));

        // Decayed velocity: see decayed_velocity() — a stopping motor must
        // not keep reporting (and integrating) the last edge speed forever.
        let velocity = self.decayed_velocity(dt);

        // Check if velocity is below interpolation threshold (compare in eRPM)
        if Self::vel_to_erpm(velocity) < self.interp_min_erpm {
            // Low speed: use sector angle directly, no interpolation
            // This prevents oscillation during direction reversals
            return Some((
                AngleSample {
                    angle: self.calib.angle_for_state(self.raw_state),
                    omega: velocity,
                    direction: self.direction,
                },
                dt,
            ));
        }

        // High speed: interpolate using velocity
        Some((
            AngleSample {
                angle: wrap_angle(self.angle + velocity * dt),
                omega: velocity,
                direction: self.direction,
            },
            dt,
        ))
    }

    /// Velocity estimate bounded by the time already waited since the last
    /// edge.
    ///
    /// If the motor were really still doing `|elec_velocity|`, an edge would
    /// have arrived within one sector time — so once `dt_from_edge` exceeds
    /// that, the estimate is `current_sector_width / dt_from_edge`,
    /// which decays toward zero at standstill. VESC uses growing edge age in
    /// its interpolation threshold, but integrates the last edge speed with
    /// a centroid-relative correction instead (foc_math.c:645–659). Measured
    /// geometry must use the current width here, even though edge speed came
    /// from the previous sector. Centroid-only tables keep the nominal60° cap.
    #[inline]
    fn decayed_velocity(&self, dt_from_edge: f32) -> f32 {
        let vel_limit = self.current_sector_width / dt_from_edge.max(1e-9);
        clamp_f32(self.elec_velocity, -vel_limit, vel_limit)
    }

    /// Interpolated sample at `now_ticks` with full VESC-style processing.
    ///
    /// Applies:
    /// - Below `interp_min_vel`: targets the calibrated sector centroid
    /// - Above threshold: targets bounded linear interpolation
    /// - At every speed: limits the angle's rate of change toward that target
    ///
    /// The first valid sample establishes the frame directly. A nonfinite
    /// target is reported unchanged; a later finite target starts
    /// a new frame because there is no finite previous angle to slew from.
    ///
    /// This version mutates internal state to track rate limiting across calls.
    pub fn sample_at_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        let t0 = self.last_edge_ticks?;
        if self.in_glitch || self.geometry_resync {
            // Invalid state right now: no angle (the caller falls back).
            return None;
        }
        // saturating: see sample_at().
        let dt_from_edge = self.timebase.seconds(now_ticks.saturating_sub(t0));

        // Calculate dt since last sample for rate limiting. A stale snapshot
        // must not wrap into a huge interval or move the clock watermark back.
        let initialized = self.last_sample_ticks.is_some();
        let dt_sample_ticks = if let Some(last) = self.last_sample_ticks {
            let dt_ticks = now_ticks.saturating_sub(last).max(1);
            self.last_sample_ticks = Some(now_ticks.max(last));
            dt_ticks
        } else {
            self.last_sample_ticks = Some(now_ticks);
            0 // The first sample anchors directly, without a rate calculation.
        };

        // Decayed velocity (see decayed_velocity()): also moves us to the
        // low-speed path once the edges stop coming.
        let velocity = self.decayed_velocity(dt_from_edge);

        let target_angle = if Self::vel_to_erpm(velocity) < self.interp_min_erpm {
            self.calib.angle_for_state(self.raw_state)
        } else {
            // The raw target starts at the boundary just crossed and advances
            // at most one sector. The limiter may lag behind this target.
            wrap_angle(self.angle + velocity * dt_from_edge)
        };

        if !initialized || !self.rate_limited_angle.is_finite() || !target_angle.is_finite() {
            self.rate_limited_angle = target_angle;
            return Some(AngleSample {
                angle: target_angle,
                omega: velocity,
                direction: self.direction,
            });
        }

        // A settled centroid needs no slew calculation, but its timestamp must
        // still advance: the next Hall edge gets only one sample's allowance.
        // Nonfinite factors retain the full arithmetic (0 * infinity is NaN).
        if target_angle == self.rate_limited_angle && self.rate_limit_factor.is_finite() {
            // The normal zero-step addition also turns a -0 frame into +0.
            self.rate_limited_angle = target_angle + 0.0;
            return Some(AngleSample {
                angle: self.rate_limited_angle,
                omega: velocity,
                direction: self.direction,
            });
        }

        // === RATE LIMITING (VESC-style) ===
        // Limit how fast angle can change per sample to prevent current spikes
        // max_step = max(|velocity|, min_vel_from_erpm) * dt * rate_limit_factor
        let dt_sample = self.timebase.seconds(dt_sample_ticks);
        let effective_vel = velocity.abs().max(self.interp_min_rad_s);
        let max_step = effective_vel * dt_sample * self.rate_limit_factor;

        // Calculate desired step from current rate-limited angle
        let desired_step = angle_difference(target_angle, self.rate_limited_angle);

        // Apply rate limiting
        let actual_step = if desired_step.abs() <= max_step {
            desired_step
        } else {
            // Clamp to max step, preserving sign
            if desired_step > 0.0 {
                max_step
            } else {
                -max_step
            }
        };

        self.rate_limited_angle = wrap_angle(self.rate_limited_angle + actual_step);

        Some(AngleSample {
            angle: self.rate_limited_angle,
            omega: velocity,
            direction: self.direction,
        })
    }

    /// Electrical velocity estimate (rad/s) from last transition
    pub fn electrical_velocity(&self) -> f32 {
        self.elec_velocity
    }

    #[inline]
    fn angle_step_signed(&self, dir: Direction) -> f32 {
        match dir {
            Direction::Clockwise => self.angle_per_state,
            Direction::CounterClockwise => -self.angle_per_state,
            Direction::Stopped => 0.0,
        }
    }
}

/// Calibration data: electrical angle per raw Hall state (0-7) plus advance.
///
/// Uses 8-entry raw-state table for direct lookup without logical state conversion.
/// This is more flexible as it works with any Hall sensor wiring.
/// States 0 and 7 are invalid (all sensors low/high).
#[derive(Clone, Copy)]
pub struct HallCalibration {
    /// Electrical angle (rad) for each raw Hall state (0-7)
    /// Invalid states (0, 7) store 0.0 but should not be used
    raw_table: [f32; 8],
    /// Validity flags for each raw state
    valid: [bool; 8],
    /// Additional timing advance (radians)
    advance_rad: f32,
}

impl HallCalibration {
    /// Create calibration from raw-state angles (8-entry table)
    pub fn from_raw_table(raw_table: [f32; 8], advance_rad: f32) -> Self {
        let mut valid = [false; 8];
        let mut normalized = [0.0_f32; 8];
        for i in 1..=6 {
            normalized[i] = wrap_angle(raw_table[i]);
            valid[i] = true;
        }
        Self {
            raw_table: normalized,
            valid,
            advance_rad,
        }
    }

    /// Create calibration from logical-state angles (6-entry table)
    /// Maps logical states back to raw states using HALL_STATE_TABLE
    pub fn from_logical_table(logical_table: [f32; 6], advance_rad: f32) -> Self {
        let mut raw_table = [0.0_f32; 8];
        let mut valid = [false; 8];

        // Map logical state index to raw state
        // HALL_STATE_TABLE[raw] = logical, so we need the inverse
        // raw 1 -> logical 0, raw 3 -> logical 1, raw 2 -> logical 2,
        // raw 6 -> logical 3, raw 4 -> logical 4, raw 5 -> logical 5
        const LOGICAL_TO_RAW: [u8; 6] = [1, 3, 2, 6, 4, 5];

        for (logical, &raw) in LOGICAL_TO_RAW.iter().enumerate() {
            raw_table[raw as usize] = wrap_angle(logical_table[logical]);
            valid[raw as usize] = true;
        }

        Self {
            raw_table,
            valid,
            advance_rad,
        }
    }

    /// Get angle for a raw Hall state (returns None for invalid states)
    #[inline]
    pub fn angle_for_raw_state(&self, raw_state: u8) -> Option<f32> {
        let idx = (raw_state & 0x07) as usize;
        if self.valid[idx] {
            Some(wrap_angle(self.raw_table[idx] + self.advance_rad))
        } else {
            None
        }
    }

    /// Get angle for a raw Hall state, panicking on invalid (for internal use)
    #[inline]
    pub fn angle_for_state(&self, raw_state: u8) -> f32 {
        self.angle_for_raw_state(raw_state).unwrap_or(0.0) // Fallback for invalid states
    }

    /// Check if a raw state is valid
    #[inline]
    pub fn is_valid_state(&self, raw_state: u8) -> bool {
        self.valid[(raw_state & 0x07) as usize]
    }
}

impl Default for HallCalibration {
    fn default() -> Self {
        // Default: evenly spaced angles for raw states 1-6
        // Raw state sequence (CW): 1 → 3 → 2 → 6 → 4 → 5
        // Assign angles in sequence order
        let step = TAU / 6.0;
        let mut raw_table = [0.0_f32; 8];
        let mut valid = [false; 8];

        // Map raw states to their position in the CW sequence
        const RAW_TO_SEQUENCE: [(u8, u8); 6] = [
            (1, 0), // raw 1 is 1st in sequence
            (3, 1), // raw 3 is 2nd in sequence
            (2, 2), // raw 2 is 3rd in sequence
            (6, 3), // raw 6 is 4th in sequence
            (4, 4), // raw 4 is 5th in sequence
            (5, 5), // raw 5 is 6th in sequence
        ];

        for (raw, seq_pos) in RAW_TO_SEQUENCE {
            raw_table[raw as usize] = step * f32::from(seq_pos);
            valid[raw as usize] = true;
        }

        Self {
            raw_table,
            valid,
            advance_rad: 0.0,
        }
    }
}

impl AngleSensor for HallSensor {
    fn apply_prepared_hall_calibration(
        &mut self,
        prepared: &PreparedHallCalibration,
    ) -> Option<super::sensors::HallReferenceUpdate> {
        Some(self.install_prepared_calibration(prepared))
    }

    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        self.sample_at(now_ticks)
    }

    fn sample_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        // Full VESC-style path: rate limiting smooths the up-to-60° angle
        // discontinuities at hall edges, including centroid steps below the
        // interpolation threshold. The immutable sample_at() has no limiter state.
        self.sample_at_mut(now_ticks)
    }

    fn is_stale(&self, now_ticks: u64, omega_ref: f32) -> bool {
        self.is_stale_at_speed(now_ticks, omega_ref)
    }

    fn error_count(&self) -> u32 {
        self.error_count
    }

    fn reset_errors(&mut self) {
        Self::reset_errors(self);
    }

    fn fault_kind(&self) -> Option<HallFaultKind> {
        Self::fault_kind(self)
    }
}

impl HallSensorTrait for HallSensor {
    fn raw_state(&self) -> u8 {
        self.raw_state
    }

    fn logical_state(&self) -> u8 {
        self.logical_state
    }

    fn last_edge_ticks(&self) -> Option<u64> {
        self.last_edge_ticks
    }

    fn electrical_velocity(&self) -> f32 {
        self.elec_velocity
    }

    fn set_calibration(&mut self, table: [f32; 6]) {
        Self::set_calibration(self, table);
    }

    fn set_calibration_raw(&mut self, raw_table: [f32; 8]) {
        Self::set_calibration_raw(self, raw_table);
    }

    fn apply_calibration(
        &mut self,
        result: &super::hall_calibration::HallCalibrationResult,
    ) -> bool {
        Self::apply_calibration(self, result)
    }

    fn set_advance(&mut self, advance_rad: f32) {
        Self::set_advance(self, advance_rad);
    }

    fn advance(&self) -> f32 {
        self.calib.advance_rad
    }

    fn interpolation_info(&self, now_ticks: u64) -> HallInterpolationInfo {
        // The actual interpolation base (sector boundary since the
        // boundary-anchor fix), so base + offset matches the estimate.
        let base_angle = self.angle;
        let (interpolation_offset, time_since_edge_us) = if let Some(t0) = self.last_edge_ticks {
            let dt_ticks = now_ticks.wrapping_sub(t0);
            let dt_sec = self.timebase.seconds(dt_ticks);
            let offset = self.elec_velocity * dt_sec;
            let time_us = (dt_sec * 1_000_000.0) as u32;
            (offset, time_us)
        } else {
            (0.0, 0)
        };

        HallInterpolationInfo {
            base_angle,
            interpolation_offset,
            estimated_velocity: self.elec_velocity,
            time_since_edge_us,
        }
    }
}

#[cfg(test)]
mod geometry_tests;

#[cfg(test)]
mod tests {
    #[cfg(feature = "storage")]
    #[test]
    fn prepared_reference_reanchors_raw_and_control_without_old_geometry() {
        use super::*;
        let mut hall = HallSensor::new(1_000_000);
        hall.set_interp_min_erpm(0.0);
        hall.set_advance(0.2);
        hall.update(1, 1_000);
        hall.update(3, 2_000);
        hall.update(2, 3_000);
        assert!(hall.sample_at_mut(3_050).unwrap().omega > 100.0);
        let before = (
            hall.raw_state,
            hall.logical_state,
            hall.last_edge_ticks,
            hall.error_count,
        );
        let mut config = crate::storage::HallCalibrationConfig::default();
        for angle in &mut config.angles[1..=6] {
            *angle += 0.7;
        }
        let prepared = PreparedHallCalibration::from_config(&config).unwrap();
        hall.install_prepared_calibration(&prepared);
        assert_eq!(
            (
                hall.raw_state,
                hall.logical_state,
                hall.last_edge_ticks,
                hall.error_count
            ),
            before
        );
        let expected = wrap_angle(config.angles[2] + 0.2);
        for sample in [
            hall.sample_at(3_100).unwrap(),
            hall.sample_at_mut(3_100).unwrap(),
        ] {
            assert_eq!(sample.angle.to_bits(), expected.to_bits());
            assert_eq!(sample.omega, 0.0);
            assert_eq!(sample.direction, Direction::Stopped);
        }
        assert!(!hall.base_is_boundary);
        hall.update(6, 4_000);
        assert_eq!(
            hall.electrical_velocity(),
            0.0,
            "old edge frame is not a speed baseline"
        );
        hall.update(4, 5_000);
        assert!(
            hall.electrical_velocity() > 100.0,
            "two new boundaries restore measured speed"
        );
        hall.update(7, 5_100);
        hall.install_prepared_calibration(&prepared);
        assert!(
            hall.sample_at(5_150).is_none(),
            "reference apply cannot hide a wire glitch"
        );
        let mut accumulator = crate::foc::hall_calibration::HallCalibrator::new();
        for state in 1..=6 {
            for _ in 0..30 {
                accumulator.record(config.angles[state], state as u8);
            }
        }
        let mut forged = accumulator.finish().unwrap();
        forged.valid[3] = false;
        assert_eq!(forged.valid_count, 6);
        assert!(
            PreparedHallCalibration::from_result(&forged).is_none(),
            "claimed count cannot override a missing physical state"
        );
        let mut invalid = config;
        invalid.angles[3] = f32::NAN;
        assert!(PreparedHallCalibration::from_config(&invalid).is_none());
        let mut cold = HallSensor::new(1_000_000);
        cold.install_prepared_calibration(&prepared);
        assert!(
            cold.sample_at(0).is_none(),
            "reference apply must not fabricate an edge"
        );
    }

    use super::*;
    use core::f32::consts::FRAC_PI_3;

    fn assert_raw_snapshot_matches_separate_sample_and_age(hall: &HallSensor, now: u64) {
        match (hall.snapshot_at(now), hall.sample_at(now)) {
            (None, None) => {}
            (Some(snapshot), Some(sample)) => {
                assert_eq!(snapshot.angle_rad.to_bits(), sample.angle.to_bits());
                assert_eq!(snapshot.velocity_rad_s.to_bits(), sample.omega.to_bits());
                assert_eq!(snapshot.direction, sample.direction);
                assert_eq!(snapshot.state, hall.logical_state());
                assert_eq!(snapshot.error_count, hall.error_count());
                assert_eq!(
                    snapshot.edge_age_s.to_bits(),
                    hall.time_since_edge_s(now).unwrap().to_bits()
                );
            }
            pair => panic!("raw snapshot availability differs: {pair:?}"),
        }
    }

    #[test]
    fn raw_snapshot_reuses_same_edge_age_across_motion_glitches_and_clock_ranges() {
        for ticks_per_sec in [1, 1_000_000, 170_000_000] {
            let mut hall = HallSensor::new(ticks_per_sec);
            let interval = (ticks_per_sec / 100).max(1);
            assert_raw_snapshot_matches_separate_sample_and_age(&hall, 0);
            let mut now = 0;
            for raw in [1, 3, 2, 6, 4, 5, 1, 5, 4, 6, 2, 3, 1] {
                hall.update(raw, now);
                for observed in [
                    now.saturating_sub(1),
                    now,
                    now + 1,
                    now + interval / 2,
                    now + 2 * interval,
                    u64::MAX,
                ] {
                    assert_raw_snapshot_matches_separate_sample_and_age(&hall, observed);
                }
                now += interval;
            }
            hall.update(7, now);
            assert!(hall.snapshot_at(now).is_none());
            assert_raw_snapshot_matches_separate_sample_and_age(&hall, now + interval);
            hall.update(1, now + interval);
            assert_raw_snapshot_matches_separate_sample_and_age(&hall, now + interval);
            assert!(hall.snapshot_at(now + interval).unwrap().error_count > 0);
        }
    }

    // Keep the pre-optimization raw sampling equations as a differential oracle.
    // Edge processing is exercised through the public API on both estimators.
    fn reference_raw_sample(hall: &HallSensor, now: u64) -> Option<AngleSample> {
        let t0 = hall.last_edge_ticks?;
        if hall.in_glitch {
            return None;
        }
        let age = hall.timebase.seconds(now.saturating_sub(t0));
        let sector = hall.calib.angle_for_state(hall.raw_state);
        let limit = hall.angle_per_state / age.max(1e-9);
        let velocity = clamp_f32(hall.elec_velocity, -limit, limit);
        let angle = if velocity.abs() * 60.0 / TAU < hall.interp_min_erpm {
            sector
        } else {
            wrap_angle(hall.angle + velocity * age)
        };
        Some(AngleSample {
            angle,
            omega: velocity,
            direction: hall.direction,
        })
    }

    // The all-speed limiter before its settled-target fast path. In particular,
    // keep the eager dt conversion and IEEE arithmetic of a zero desired step.
    fn reference_smoothed_sample(hall: &mut HallSensor, now: u64) -> Option<AngleSample> {
        let t0 = hall.last_edge_ticks?;
        if hall.in_glitch {
            return None;
        }
        let age = hall.timebase.seconds(now.saturating_sub(t0));
        let initialized = hall.last_sample_ticks.is_some();
        let dt = if let Some(last) = hall.last_sample_ticks {
            hall.last_sample_ticks = Some(now.max(last));
            hall.timebase.seconds(now.saturating_sub(last).max(1))
        } else {
            hall.last_sample_ticks = Some(now);
            age
        };
        let velocity = hall.decayed_velocity(age);
        let target = if HallSensor::vel_to_erpm(velocity) < hall.interp_min_erpm {
            hall.calib.angle_for_state(hall.raw_state)
        } else {
            wrap_angle(hall.angle + velocity * age)
        };
        if !initialized || !hall.rate_limited_angle.is_finite() || !target.is_finite() {
            hall.rate_limited_angle = target;
        } else {
            let max_step = velocity.abs().max(hall.interp_min_rad_s) * dt * hall.rate_limit_factor;
            let desired = angle_difference(target, hall.rate_limited_angle);
            let step = if desired.abs() <= max_step {
                desired
            } else if desired > 0.0 {
                max_step
            } else {
                -max_step
            };
            hall.rate_limited_angle = wrap_angle(hall.rate_limited_angle + step);
        }
        Some(AngleSample {
            angle: hall.rate_limited_angle,
            omega: velocity,
            direction: hall.direction,
        })
    }

    fn assert_same_sample(actual: Option<AngleSample>, expected: Option<AngleSample>) {
        match (actual, expected) {
            (None, None) => {}
            (Some(actual), Some(expected)) => {
                assert_eq!(actual.direction, expected.direction);
                for (actual, expected) in [
                    (actual.angle, expected.angle),
                    (actual.omega, expected.omega),
                ] {
                    if expected.is_nan() {
                        assert!(actual.is_nan());
                    } else {
                        assert_eq!(actual.to_bits(), expected.to_bits());
                    }
                }
            }
            (actual, expected) => panic!("sample availability changed: {actual:?} vs {expected:?}"),
        }
    }

    #[test]
    fn raw_geometry_and_both_sample_speeds_survive_edges_and_live_configuration() {
        let edges = [1, 3, 2, 6, 4, 5, 1, 0, 1, 3, 7, 2, 6, 2, 3, 1, 5, 4, 6];
        for rate_limited in [false, true] {
            for threshold in [0.0, 1.0, 500.0, -500.0, 1e6, f32::INFINITY, f32::NAN] {
                let mut actual = HallSensor::new(1_000_000);
                let mut expected = HallSensor::new(1_000_000);
                actual.set_interp_min_erpm(threshold);
                expected.set_interp_min_erpm(threshold);
                for (i, raw) in edges.into_iter().enumerate() {
                    if i == 5 {
                        // Reversed wiring and asymmetric sectors; applying a new
                        // calibration must not re-anchor the current live base.
                        let table = [0.0, 5.4, 1.5, 0.6, 4.0, 5.0, 2.6, 0.0];
                        actual.set_calibration_raw(table);
                        expected.set_calibration_raw(table);
                    }
                    if i == 9 {
                        actual.set_advance(-TAU * 1.25);
                        expected.set_advance(-TAU * 1.25);
                    }
                    if i == 15 {
                        let table = [0.1, 1.2, 2.3, 3.4, 4.5, 5.6];
                        actual.set_calibration(table);
                        expected.set_calibration(table);
                    }
                    let now = i as u64 * 12_347;
                    actual.update(raw, now);
                    expected.update(raw, now);
                    for sample_time in [
                        now.saturating_sub(1),
                        now,
                        now + 1,
                        now + 999,
                        now + 1_000_000,
                    ] {
                        let sample = if rate_limited {
                            actual.sample_at_mut(sample_time)
                        } else {
                            actual.sample_at(sample_time)
                        };
                        let reference = reference_raw_sample(&expected, sample_time);
                        if rate_limited {
                            // The control angle intentionally changes. Its speed,
                            // direction and availability must retain raw semantics.
                            let motion_only = |mut s: AngleSample| {
                                s.angle = 0.0;
                                s
                            };
                            assert_same_sample(sample.map(motion_only), reference.map(motion_only));
                        } else {
                            assert_same_sample(sample, reference);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn interpolation_settings_preserve_threshold_neighbor_samples() {
        for rate_limited in [false, true] {
            for erpm in [
                0.0,
                -0.0,
                1.0,
                500.0,
                -500.0,
                f32::from_bits(1),
                f32::MIN_POSITIVE,
                f32::MAX,
                f32::INFINITY,
                f32::NEG_INFINITY,
                f32::NAN,
            ] {
                let threshold = erpm.abs() * TAU / 60.0;
                for velocity in [
                    threshold.next_down(),
                    threshold,
                    threshold.next_up(),
                    -threshold.next_down(),
                    -threshold,
                    -threshold.next_up(),
                ] {
                    let mut actual = HallSensor::new(1_000_000);
                    let mut expected = HallSensor::new(1_000_000);
                    actual.update(1, 123);
                    expected.update(1, 123);
                    // Changing the setting after edges have arrived must
                    // update the limiter cache without resetting live state.
                    actual.set_interp_min_erpm(erpm);
                    expected.set_interp_min_erpm(erpm);
                    actual.elec_velocity = velocity;
                    expected.elec_velocity = velocity;
                    let sample = if rate_limited {
                        actual.sample_at_mut(123)
                    } else {
                        actual.sample_at(123)
                    };
                    assert_same_sample(sample, reference_raw_sample(&expected, 123));
                }
            }
        }
    }

    #[test]
    fn hall_timebase_preserves_wide_tick_conversion_and_zero_rate_fallback() {
        for hz in [0, 1, 999_023, 1_000_000, u64::from(u32::MAX), u64::MAX] {
            let timebase = HallTimebase::new(hz);
            for ticks in [
                0,
                1,
                65_535,
                u64::from(u32::MAX),
                u64::from(u32::MAX) + 1,
                u64::MAX,
            ] {
                let previous = ticks as f32 / hz.max(1) as f32;
                assert_eq!(timebase.seconds(ticks).to_bits(), previous.to_bits());
            }
        }
    }

    #[test]
    fn test_hall_state_table() {
        assert_eq!(HALL_STATE_TABLE[0], 0); // Invalid
        assert_eq!(HALL_STATE_TABLE[1], 0); // State 0
        assert_eq!(HALL_STATE_TABLE[2], 2); // State 2
        assert_eq!(HALL_STATE_TABLE[3], 1); // State 1
        assert_eq!(HALL_STATE_TABLE[4], 4); // State 4
        assert_eq!(HALL_STATE_TABLE[5], 5); // State 5
        assert_eq!(HALL_STATE_TABLE[6], 3); // State 3
        assert_eq!(HALL_STATE_TABLE[7], 0); // Invalid
    }

    #[test]
    fn test_hall_sensor_creation() {
        let hall = HallSensor::new(1);
        assert_eq!(hall.angle(), 0.0);
        assert_eq!(hall.direction(), Direction::Stopped);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_invalid_states() {
        let mut hall = HallSensor::new(1);

        // All low (0b000)
        assert!(hall.update(0, 0).is_none());
        assert_eq!(hall.error_count(), 1);

        // All high (0b111)
        assert!(hall.update(7, 0).is_none());
        assert_eq!(hall.error_count(), 2);
    }

    #[test]
    fn first_valid_hall_sector_is_an_initial_position_not_a_skipped_edge() {
        for raw in 1..=6 {
            let mut hall = HallSensor::new(1_000_000);
            assert!(hall.update(raw, 100).is_some());
            assert_eq!(hall.error_count(), 0, "initial raw sector {raw}");
            assert_eq!(hall.direction(), Direction::Stopped);
            assert_eq!(hall.elec_velocity, 0.0);

            // Real skipped transitions remain errors after initialization.
            let skipped = (1..=6)
                .find(|next| {
                    HALL_STATE_TABLE[*next as usize] == (HALL_STATE_TABLE[raw as usize] + 2) % 6
                })
                .unwrap();
            hall.update(skipped, 1100);
            assert_eq!(hall.error_count(), 1);
        }
    }

    /// A connector-bounce edge pair (adjacent edges ticks apart) used to
    /// read as ~10^6 rad/s and feed the decoupling feedforward / actuation
    /// advance as a full-bus pulse. Implausible edge speeds must keep the
    /// previous velocity estimate.
    #[test]
    fn edge_bounce_does_not_explode_velocity() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz timebase
        // Steady forward rotation: one sector per millisecond
        // (~pi/3 / 1e-3 = 1047 el rad/s).
        let sequence = [1u8, 3, 2, 6, 4, 5, 1, 3];
        let mut t = 0u64;
        for &state in &sequence {
            hall.update(state, t);
            t += 1_000;
        }
        let steady = hall.elec_velocity;
        assert!(
            (steady.abs() - 1047.0).abs() < 60.0,
            "unexpected steady velocity {steady}"
        );
        // Bounce: the next edge and its return arrive 1 tick apart.
        hall.update(2, t);
        hall.update(3, t + 1);
        hall.update(2, t + 2);
        assert!(
            hall.elec_velocity.abs() <= MAX_PLAUSIBLE_E_RAD_S,
            "bounce produced implausible velocity {}",
            hall.elec_velocity
        );
        assert!(
            (hall.elec_velocity.abs() - steady.abs()).abs() < 60.0,
            "bounce should keep the previous estimate, got {}",
            hall.elec_velocity
        );
    }

    #[test]
    fn test_forward_sequence() {
        let mut hall = HallSensor::new(1_000);

        // Valid CW sequence: 1 → 3 → 2 → 6 → 4 → 5 → (wrap to 1).
        // The first edge has no usable previous sector → centroid (0°);
        // every following edge returns the SECTOR BOUNDARY just crossed
        // (midpoint of adjacent centroids) = centroid − 30° for forward.
        let sequence = [1, 3, 2, 6, 4, 5, 1];
        let expected_angles = [
            0.0,               // centroid fallback (first edge)
            TAU / 12.0,        // boundary 0°|60° = 30°
            TAU / 4.0,         // boundary 60°|120° = 90°
            5.0 * TAU / 12.0,  // 150°
            7.0 * TAU / 12.0,  // 210°
            3.0 * TAU / 4.0,   // 270°
            11.0 * TAU / 12.0, // boundary 300°|360° = 330°
        ];

        for (i, &state) in sequence.iter().enumerate() {
            let angle = hall.update(state, i as u64).unwrap();
            assert!(
                (angle - expected_angles[i]).abs() < 1e-5,
                "Step {}: expected {}, got {}",
                i,
                expected_angles[i],
                angle
            );
        }

        // After full sequence, should be CW
        assert_eq!(hall.direction(), Direction::Clockwise);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_reverse_sequence() {
        let mut hall = HallSensor::new(1_000);

        // Start from state 5
        hall.update(5, 0).unwrap();

        // Valid CCW sequence: 5 → 4 → 6 → 2 → 3 → 1 → (wrap to 5)
        let sequence = [4, 6, 2, 3, 1, 5];

        for (i, &state) in sequence.iter().enumerate() {
            hall.update(state, (i + 1) as u64).unwrap();
        }

        // Should detect CCW direction
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_electrical_angle_increment() {
        let mut hall = HallSensor::new(1_000);

        // First edge: centroid fallback (0°). Second edge: the boundary
        // between the 0° and 60° sectors = 30°.
        let angle1 = hall.update(1, 0).unwrap();
        assert!((angle1 - 0.0).abs() < 1e-5);

        let angle2 = hall.update(3, 1).unwrap();
        assert!((angle2 - TAU / 12.0).abs() < 1e-5);

        // From a boundary base, each further edge advances one full sector.
        let angle3 = hall.update(2, 2).unwrap();
        assert!((angle3 - (TAU / 12.0 + TAU / 6.0)).abs() < 1e-5);
    }

    #[test]
    fn test_error_detection() {
        let mut hall = HallSensor::new(1_000);

        // Valid state
        hall.update(1, 0).unwrap();

        // Invalid jump (should increment error)
        let initial_errors = hall.error_count();
        hall.update(6, 0).unwrap(); // Jumping from 1 to 6 (state 0 to 3)
        assert!(hall.error_count() > initial_errors);
    }

    #[test]
    fn test_reset_errors() {
        let mut hall = HallSensor::new(1_000);

        hall.update(0, 0).unwrap_or(0.0); // Generate error
        assert!(hall.error_count() > 0);

        hall.reset_errors();
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_interpolation_and_velocity() {
        let mut hall = HallSensor::new(10_000); // Higher tick rate for shorter dt
        hall.update(1, 0).unwrap();
        hall.update(3, 10).unwrap(); // 1 ms later with 10 kHz ticks

        let expected_vel = (TAU / 6.0) / 0.001;
        assert!((hall.electrical_velocity() - expected_vel).abs() < expected_vel * 0.01);

        // Sample shortly after the edge (0.1ms): interpolation runs from
        // the boundary just crossed (30°), not the sector centroid.
        let interp = hall.sample_at(11).unwrap();
        let expected_angle = (TAU / 12.0) + expected_vel * 0.0001;
        let diff = (wrap_angle(interp.angle) - wrap_angle(expected_angle)).abs();
        assert!(diff < 1e-3);
    }

    #[test]
    fn test_low_speed_no_interpolation() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz ticks
        hall.update(1, 0).unwrap();
        // Very slow: 100ms between edges = ~10 rad/s (below 52 rad/s threshold)
        hall.update(3, 100_000).unwrap();

        let sector_angle = TAU / 6.0; // Angle for logical state 1
        let sample = hall.sample_at(150_000).unwrap();

        // At low speed, should return sector angle exactly (no interpolation)
        assert!(
            (sample.angle - sector_angle).abs() < 1e-5,
            "Low speed should use sector angle, got {} expected {}",
            sample.angle,
            sector_angle
        );
    }

    #[test]
    fn first_smoothed_sample_anchors_the_observed_sector_not_zero() {
        for raw in 1..=6 {
            let mut hall = HallSensor::new(1_000_000);
            hall.set_calibration_raw([0.0, 5.4474, 1.5422, 0.5691, 4.0420, 4.9824, 2.6073, 0.0]);
            hall.set_advance(-TAU * 1.25);
            assert!(hall.sample_at_mut(0).is_none());
            hall.update(0, 50);
            assert!(hall.sample_at_mut(50).is_none());
            hall.update(raw, 100);
            let expected = hall.sample_at(100);
            assert_same_sample(hall.sample_at_mut(100), expected);
        }

        // First sampling can also begin after rotation is already established.
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0);
        hall.update(3, 1_000);
        let expected = hall.sample_at(1_050);
        assert_same_sample(hall.sample_at_mut(1_050), expected);
    }

    #[test]
    fn slow_hall_edge_slews_the_control_frame_instead_of_rotating_it_sixty_degrees() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0);
        let mut previous = 0.0;
        for now in (0..100_000).step_by(50) {
            previous = hall.sample_at_mut(now).unwrap().angle;
        }
        hall.update(3, 100_000);
        let accepted_speed = hall.electrical_velocity().to_bits();
        let raw = hall.sample_at(100_000).unwrap();
        let smoothed = hall.sample_at_mut(100_000).unwrap();
        let raw_step = angle_difference(raw.angle, previous).abs();
        let control_step = angle_difference(smoothed.angle, previous).abs();
        let bound = DEFAULT_INTERP_MIN_ERPM * TAU / 60.0 * 50e-6 * DEFAULT_RATE_LIMIT_FACTOR;
        assert!(
            raw_step > 1.0,
            "the raw sector really changed by about 60 degrees"
        );
        assert!(
            control_step <= bound + 1e-6,
            "unlimited low-speed step {control_step}"
        );

        // This is the geometrical change of a 3-A reference vector, not a
        // prediction of the physical current transient in a particular motor.
        assert!(6.0 * libm::sinf(raw_step * 0.5) > 2.9);
        assert!(6.0 * libm::sinf(control_step * 0.5) < 0.012);
        previous = smoothed.angle;
        for now in (100_050..=114_000).step_by(50) {
            let sample = hall.sample_at_mut(now).unwrap();
            assert!(angle_difference(sample.angle, previous).abs() <= bound + 1e-6);
            previous = sample.angle;
        }
        assert!(angle_difference(previous, raw.angle).abs() < 1e-5);
        assert_eq!(hall.electrical_velocity().to_bits(), accepted_speed);
    }

    #[test]
    fn slow_forward_reverse_and_wrap_edges_obey_one_continuous_angle_bound() {
        let default = HallCalibration::default();
        let mut reverse = [0.0; 8];
        for (raw, angle) in reverse.iter_mut().enumerate().take(7).skip(1) {
            *angle = wrap_angle(-default.angle_for_state(raw as u8));
        }
        for table in [
            default.raw_table,
            reverse,
            [0.0, 5.4474, 1.5422, 0.5691, 4.0420, 4.9824, 2.6073, 0.0],
        ] {
            let mut hall = HallSensor::new(1_000_000);
            hall.set_calibration_raw(table);
            let mut previous = None;
            let mut forward = false;
            let mut backward = false;
            for (sector, raw) in [5, 1, 3, 2, 6, 4, 5, 1, 5, 4, 6, 2, 3, 1]
                .into_iter()
                .enumerate()
            {
                let edge = sector as u64 * 100_000;
                hall.update(raw, edge);
                let accepted_speed = hall.electrical_velocity();
                forward |= accepted_speed > 1.0;
                backward |= accepted_speed < -1.0;
                for now in (edge..edge + 100_000).step_by(50) {
                    let raw_sample = hall.sample_at(now).unwrap();
                    assert_eq!(
                        raw_sample.angle.to_bits(),
                        hall.calib.angle_for_state(raw).to_bits()
                    );
                    let sample = hall.sample_at_mut(now).unwrap();
                    assert_eq!(sample.omega.to_bits(), raw_sample.omega.to_bits());
                    if let Some(angle) = previous {
                        let bound = sample.omega.abs().max(hall.interp_min_rad_s)
                            * 50e-6
                            * hall.rate_limit_factor;
                        assert!(
                            angle_difference(sample.angle, angle).abs() <= bound + 2e-6,
                            "sector={sector}, time={now}, previous={angle}, sample={sample:?}"
                        );
                    }
                    previous = Some(sample.angle);
                }
                assert_eq!(
                    hall.electrical_velocity().to_bits(),
                    accepted_speed.to_bits()
                );
            }
            assert!(forward && backward);
        }
    }

    #[test]
    fn interpolation_threshold_and_live_advance_changes_do_not_reset_the_angle_limiter() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0);
        hall.update(3, 20_500); // ~488 eRPM: initially the centroid branch.
        let mut previous = hall.sample_at_mut(20_500).unwrap().angle;
        for (now, threshold, advance) in [
            (20_550, 400.0, 0.0), // interpolation target moves toward the boundary
            (20_600, 500.0, 0.0), // centroid target returns
            (20_650, 500.0, 0.7), // live centroid calibration changes
            (20_700, 400.0, 0.7),
        ] {
            hall.set_interp_min_erpm(threshold);
            hall.set_advance(advance);
            let sample = hall.sample_at_mut(now).unwrap();
            let bound =
                sample.omega.abs().max(hall.interp_min_rad_s) * 50e-6 * hall.rate_limit_factor;
            assert!(angle_difference(sample.angle, previous).abs() <= bound + 1e-6);
            previous = sample.angle;
        }
    }

    #[test]
    fn stale_sample_timestamps_cannot_authorize_a_large_angle_step() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0);
        let mut previous = hall.sample_at_mut(100_000).unwrap().angle;
        hall.update(3, 100_000);
        for now in [99_999, 100_000, 100_050] {
            let sample = hall.sample_at_mut(now).unwrap();
            let ticks = if now <= 100_000 { 1.0 } else { 50.0 };
            let bound = hall.interp_min_rad_s * ticks * 1e-6 * hall.rate_limit_factor;
            assert!(angle_difference(sample.angle, previous).abs() <= bound + 1e-6);
            assert_eq!(hall.last_sample_ticks, Some(now.max(100_000)));
            previous = sample.angle;
        }
    }

    #[test]
    fn nonfinite_calibration_is_reported_and_a_finite_reconfiguration_reanchors() {
        for advance in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut hall = HallSensor::new(1_000_000);
            hall.update(3, 0);
            hall.sample_at_mut(0).unwrap();
            hall.set_advance(advance);
            assert!(!hall.sample_at(50).unwrap().angle.is_finite());
            assert!(!hall.sample_at_mut(50).unwrap().angle.is_finite());
            hall.set_advance(0.1);
            let expected = hall.sample_at(100);
            assert_same_sample(hall.sample_at_mut(100), expected);
        }
    }

    #[test]
    fn nonfinite_tuning_retains_comparison_and_rate_bound_arithmetic() {
        for (threshold, factor) in [
            (f32::NAN, 1.5),
            (f32::INFINITY, 1.5),
            (f32::NEG_INFINITY, 1.5),
            (500.0, f32::INFINITY),
            (500.0, f32::NAN),
            (500.0, f32::NEG_INFINITY),
        ] {
            let mut hall = HallSensor::new(1_000_000);
            hall.set_interp_min_erpm(threshold);
            hall.set_rate_limit_factor(factor);
            hall.update(1, 0);
            let before = hall.sample_at_mut(99_950).unwrap();
            hall.update(3, 100_000);
            let raw = hall.sample_at(100_000).unwrap();
            let sample = hall.sample_at_mut(100_000).unwrap();
            assert_eq!(sample.omega.to_bits(), raw.omega.to_bits());
            assert!(sample.angle.is_finite());
            if threshold.is_infinite() || factor == f32::INFINITY {
                assert!(angle_difference(sample.angle, raw.angle).abs() < 1e-6);
            } else {
                let bound =
                    raw.omega.abs().max(hall.interp_min_rad_s) * 50e-6 * hall.rate_limit_factor;
                assert!(angle_difference(sample.angle, before.angle).abs() <= bound + 1e-6);
            }
        }
    }

    #[test]
    fn settled_target_fast_path_preserves_full_limiter_trajectories() {
        let edges = [1, 1, 3, 2, 6, 4, 5, 1, 0, 1, 3, 7, 2, 6, 2, 3, 1, 5, 4, 6];
        for hz in [0, 1_000_000, u64::MAX] {
            for threshold in [
                0.0,
                f32::from_bits(1),
                500.0_f32.next_down(),
                500.0,
                500.0_f32.next_up(),
                f32::INFINITY,
                f32::NAN,
            ] {
                for factor in [
                    0.0,
                    1.5,
                    f32::MAX,
                    f32::INFINITY,
                    f32::NEG_INFINITY,
                    f32::NAN,
                ] {
                    let mut actual = HallSensor::new(hz);
                    let mut expected = HallSensor::new(hz);
                    for hall in [&mut actual, &mut expected] {
                        hall.set_interp_min_erpm(threshold);
                        hall.set_rate_limit_factor(factor);
                    }
                    for (i, raw) in edges.into_iter().enumerate() {
                        for hall in [&mut actual, &mut expected] {
                            match i {
                                5 => hall.set_calibration_raw([
                                    0.0, 5.4473944, 1.5422314, 0.5690834, 4.0419855, 4.982443,
                                    2.6073236, 0.0,
                                ]),
                                9 => hall.set_advance(-TAU * 1.25),
                                11 => hall.set_advance(f32::NAN),
                                13 => hall.set_advance(0.0),
                                _ => {}
                            }
                            hall.update(raw, i as u64 * 12_347);
                        }
                        let now = i as u64 * 12_347;
                        for time in [now.saturating_sub(1), now, now + 1, now + 50, now + 51] {
                            assert_same_sample(
                                actual.sample_at_mut(time),
                                reference_smoothed_sample(&mut expected, time),
                            );
                            assert_eq!(actual.last_sample_ticks, expected.last_sample_ticks);
                        }
                    }
                    // Both the short native conversion and a full-width idle
                    // interval must leave exactly the same settled frame.
                    for time in [1_000_000, u64::from(u32::MAX) + 1, u64::MAX] {
                        assert_same_sample(
                            actual.sample_at_mut(time),
                            reference_smoothed_sample(&mut expected, time),
                        );
                        assert_eq!(actual.last_sample_ticks, expected.last_sample_ticks);
                    }
                }
            }
        }
    }

    #[test]
    fn equal_angles_keep_signed_zero_and_nonfinite_factor_arithmetic() {
        for threshold in [0.0, 500.0, f32::INFINITY, f32::NAN] {
            for factor in [0.0, 1.5, f32::INFINITY, f32::NAN] {
                for negative_zero in [false, true] {
                    let mut actual = HallSensor::new(1_000_000);
                    let mut expected = HallSensor::new(1_000_000);
                    for hall in [&mut actual, &mut expected] {
                        let zero = if negative_zero { -0.0 } else { 0.0 };
                        hall.set_calibration_raw([zero; 8]);
                        hall.set_advance(zero);
                        hall.set_interp_min_erpm(threshold);
                        hall.set_rate_limit_factor(factor);
                        hall.update(1, 0);
                    }
                    for time in [0, 0, 50, 49, 100, u64::MAX] {
                        assert_same_sample(
                            actual.sample_at_mut(time),
                            reference_smoothed_sample(&mut expected, time),
                        );
                        assert_eq!(actual.last_sample_ticks, expected.last_sample_ticks);
                    }
                }
            }
        }
    }

    /// Interpolation runs from the boundary just crossed at the measured
    /// speed, capped at one sector: 1 ms after the edge at 60°/ms it sits
    /// at the next boundary (90°), and it stays there if no edge comes.
    #[test]
    fn interpolation_runs_one_sector_from_the_boundary() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0).unwrap();
        hall.update(3, 1000).unwrap(); // 1 ms per sector = 1047 rad/s
        let next_boundary = wrap_angle(TAU / 12.0 + TAU / 6.0); // 90°
        for t in [1_500, 2_000, 3_000] {
            let expected = if t == 1_500 {
                wrap_angle(TAU / 12.0 + TAU / 12.0)
            } else {
                next_boundary
            };
            let angle = hall.sample_at(t).unwrap().angle;
            assert!(
                angle_difference(angle, expected).abs() < 1e-4,
                "t={t}: {angle}"
            );
        }
    }

    #[test]
    fn test_rate_limiting() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0).unwrap();
        hall.update(3, 1000).unwrap(); // 1ms = 1047 rad/s (high speed)

        // First sample establishes baseline
        let sample1 = hall.sample_at_mut(1001).unwrap();

        // Second sample 10µs later - rate limiting should constrain step
        let sample2 = hall.sample_at_mut(1011).unwrap();

        let step = angle_difference(sample2.angle, sample1.angle).abs();

        // Max step should be: velocity * dt * rate_limit_factor
        // = 1047 * 0.00001 * 1.5 ≈ 0.0157 rad
        let expected_max_step = 1047.0 * 0.00001 * DEFAULT_RATE_LIMIT_FACTOR;

        assert!(
            step <= expected_max_step * 1.1, // Allow 10% margin for floating point
            "Rate limiting should constrain step to {expected_max_step}, got {step} rad"
        );
    }

    #[test]
    fn test_direction_reversal_detection() {
        let mut hall = HallSensor::new(1_000);

        // Move forward: 1 → 3 (CW)
        hall.update(1, 0).unwrap();
        hall.update(3, 1).unwrap();
        assert_eq!(hall.direction(), Direction::Clockwise);
        assert!(!hall.direction_reversed());

        // Reverse: 3 → 1 (CCW) - this is a reversal
        hall.update(1, 2).unwrap();
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert!(hall.direction_reversed());
        assert_eq!(hall.prev_direction(), Direction::Clockwise);

        // Continue CCW: 1 → 5 - not a reversal
        hall.update(5, 3).unwrap();
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert!(!hall.direction_reversed());
    }

    /// A slow rotor that turns around just past a boundary (a hold, a
    /// reversal through zero) crosses it twice within milliseconds. The
    /// interval then bounds nothing useful: one sector over 5 ms read as
    /// 209 rad/s for a rotor moving at 5 — the spikes the sim showed at
    /// every low-speed reversal.
    #[test]
    fn slow_turnaround_past_a_boundary_reads_slow() {
        let mut hall = HallSensor::new(1_000_000); // µs ticks
        let sector_us = 209_440; // 5 el rad/s
        let mut t = 0u64;
        for &state in &CW_SEQ[..4] {
            hall.update(state, t).unwrap();
            t += sector_us;
        }
        let omega = hall.electrical_velocity();
        assert!((omega - 5.0).abs() < 0.1, "ω {omega}");
        // Into the next sector, and straight back out 5 ms later.
        hall.update(CW_SEQ[4], t).unwrap();
        hall.update(CW_SEQ[3], t + 5_000).unwrap();
        assert!(hall.direction_reversed());
        let omega = hall.electrical_velocity();
        assert!(omega < 0.0 && omega > -10.0, "ω {omega}");
    }

    #[test]
    fn test_time_since_edge() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz

        // No edge yet
        assert!(hall.time_since_edge(1000).is_none());
        assert!(hall.time_since_edge_us(1000).is_none());

        // After edge
        hall.update(1, 1_000_000).unwrap(); // Edge at t=1s

        // 500ms later
        let elapsed = hall.time_since_edge(1_500_000).unwrap();
        assert_eq!(elapsed, 500_000); // 500_000 ticks

        let elapsed_us = hall.time_since_edge_us(1_500_000).unwrap();
        assert_eq!(elapsed_us, 500_000); // 500ms = 500_000µs
    }

    /// CW raw-state sequence (one electrical revolution = 6 edges).
    const CW_SEQ: [u8; 6] = [1, 3, 2, 6, 4, 5];

    #[test]
    fn rate_limiter_smooths_early_hall_edge() {
        // An edge arriving earlier than interpolation predicted is exactly the
        // discontinuity the rate limiter exists for. VESC never snaps its
        // rate-limited angle on an edge (foc_math.c:665-672: it always slews
        // by at most angle_step toward the target). Snapping in update()
        // disables the limiter at the worst possible moment.
        let mut hall = HallSensor::new(1_000_000); // µs ticks
        let mut idx = 0usize;
        let mut t = 0u64;
        let mut last_out = 0.0f32;

        // Steady rotation: edge every 500 µs (≈20k eRPM), 20 kHz sampling.
        for _ in 0..11 {
            hall.update(CW_SEQ[idx % 6], t).unwrap();
            idx += 1;
            for k in 1..10u64 {
                last_out = hall.sample_at_mut(t + k * 50).unwrap().angle;
            }
            t += 500;
        }
        // Final steady edge, sampled only up to +200 µs…
        hall.update(CW_SEQ[idx % 6], t).unwrap();
        idx += 1;
        for k in 1..=4u64 {
            last_out = hall.sample_at_mut(t + k * 50).unwrap().angle;
        }
        // …then the next edge arrives at +250 µs — twice as early as expected.
        hall.update(CW_SEQ[idx % 6], t + 250).unwrap();
        let out = hall.sample_at_mut(t + 300).unwrap().angle;

        let step = angle_difference(out, last_out).abs();
        // The limiter bound uses the (now doubled) post-edge velocity estimate
        // over the 100 µs between the two samples (t+200 → t+300).
        let max_step = hall.electrical_velocity().abs() * 100e-6 * DEFAULT_RATE_LIMIT_FACTOR + 1e-3;
        assert!(
            step <= max_step,
            "angle jumped {step} rad across an early hall edge (rate limit {max_step})"
        );
    }

    #[test]
    fn stale_at_speed_trips_after_missing_edges() {
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        for s in CW_SEQ.iter().cycle().take(12) {
            hall.update(*s, t).unwrap();
            t += 1000; // 1 ms per sector
        }
        // The last edge was at t − 1 ms; one sector takes 1 ms (1047 rad/s)
        // and the observer confirms that speed.
        let w = FRAC_PI_3 / 1e-3;
        assert!(!hall.is_stale_at_speed(t, w));
        assert!(!hall.is_stale_at_speed(t + 400, w));
        // Cable cut: 1.5 expected sector times with no edge.
        assert!(hall.is_stale_at_speed(t + 600, w));
    }

    /// Half the ways to connect three hall wires reverse the nominal state
    /// order. The calibration table absorbs that; direction and speed must
    /// follow the calibrated angles, not the wiring order (the second edge
    /// after boot used to get its speed with the wrong sign).
    #[test]
    fn reversed_wiring_reads_forward_rotation_forward() {
        let mut hall = HallSensor::new(1_000_000);
        // The nominal forward sequence wired in reverse: forward rotation
        // now walks it backwards, one 60° sector per state.
        let seq = [5u8, 4, 6, 2, 3, 1];
        let mut table = [0.0f32; 8];
        for (k, &raw) in seq.iter().enumerate() {
            table[raw as usize] = k as f32 * FRAC_PI_3;
        }
        hall.set_calibration_raw(table);
        for (k, &raw) in seq.iter().cycle().take(12).enumerate() {
            hall.update(raw, k as u64 * 1_000);
            if k > 0 {
                assert_eq!(hall.direction(), Direction::Clockwise, "edge {k}");
                let v = hall.electrical_velocity();
                assert!((v - FRAC_PI_3 / 1e-3).abs() < 1.0, "edge {k}: {v} rad/s");
            }
        }
    }

    #[test]
    fn stale_at_speed_needs_evidence_of_rotation() {
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        for s in CW_SEQ.iter().cycle().take(12) {
            hall.update(*s, t).unwrap();
            t += 1000;
        }
        // Coasted to a stop: no independent rotation evidence → the rotor
        // stopped, however stale the last edge's speed looks.
        assert!(!hall.is_stale_at_speed(t + 1_000_000, 0.0));
        // A slower reference (decelerating; the observer lags) stretches the
        // window: the slower of the two speeds sets the expected cadence.
        assert!(!hall.is_stale_at_speed(t + 5_000, 100.0));
    }

    #[test]
    fn stale_at_speed_never_trips_at_standstill() {
        // A motor that was never moved produces no edges — not staleness,
        // whatever the reference claims.
        let hall = HallSensor::new(1_000_000);
        assert!(!hall.is_stale_at_speed(10_000_000, 1000.0));

        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0).unwrap(); // single state, velocity 0
        assert!(!hall.is_stale_at_speed(10_000_000, 1000.0));
    }

    #[test]
    fn invalid_state_glitch_keeps_the_speed_estimate() {
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        for s in CW_SEQ.iter().cycle().take(12) {
            hall.update(*s, t).unwrap();
            t += 1000; // 1 ms per sector
        }
        let v = hall.electrical_velocity();
        assert!(v > 500.0);
        let last = CW_SEQ[11 % CW_SEQ.len()];
        // A 200 µs all-low glitch mid-sector: no angle while it lasts, but
        // the speed and direction survive it.
        assert!(hall.update(0, t - 700).is_none());
        assert!(hall.sample_at(t - 600).is_none());
        assert_eq!(hall.electrical_velocity(), v);
        assert_eq!(hall.direction(), Direction::Clockwise);
        // Back in the SAME sector: not an edge, interpolation resumes.
        hall.update(last, t - 500).unwrap();
        assert_eq!(hall.electrical_velocity(), v);
        assert!(hall.sample_at(t - 400).is_some());
        // A glitch that hides an edge: the next sector shows up at the
        // glitch's end, off the real crossing time. Neither that edge nor
        // the next one (measured from it) may corrupt the speed.
        let next = CW_SEQ[12 % CW_SEQ.len()];
        let after = CW_SEQ[13 % CW_SEQ.len()];
        assert!(hall.update(0, t - 100).is_none());
        hall.update(next, t + 300).unwrap(); // real crossing was at t
        assert_eq!(hall.electrical_velocity(), v);
        hall.update(after, t + 1000).unwrap(); // only 0.7 ms after the fake edge
        assert_eq!(hall.electrical_velocity(), v);
    }

    /// The estimate must track an INDEPENDENT continuous rotor angle — not
    /// the stored sector angle. The older tests defined ground truth as
    /// `calib.angle_for_state(...)` itself, so an anchor-convention bias
    /// (seeding interpolation at the sector CENTER instead of the entry
    /// boundary) was self-consistent and invisible: at the instant of a
    /// Hall edge the rotor is by definition ON the boundary between the
    /// old and new sectors (≈ center − width/2 for forward rotation), and
    /// both quantities then advance at the same velocity, so a center
    /// anchor is a SYSTEMATIC half-sector (~30° electrical) lead across
    /// the whole sector — ≈13% torque loss + parasitic d-current. Both
    /// references anchor at the boundary: VESC uses the midpoint of the
    /// adjacent centroids (foc_math.c:636), MESC stores the boundary in
    /// its hall table and PLLs against it (MESCfoc.c:943).
    #[test]
    fn interpolation_tracks_continuous_rotor() {
        let ticks_per_sec = 1_000_000u64; // µs ticks
        let mut hall = HallSensor::new(ticks_per_sec);

        // Default calibration: centroid of sector k at k·60°, so the
        // forward entry boundary of sector k is at k·60° − 30°.
        let omega = 200.0f32; // rad/s elec — well above the interp threshold
        let width = TAU / 6.0;

        let mut theta = 0.0f32; // true rotor angle, starts mid-sector 0
        let mut sector = 0usize; // ground-truth sector index
        let dt_us = 50u64; // 20 kHz sampling
        let dt = dt_us as f32 / 1e6;

        hall.update(CW_SEQ[0], 0).unwrap();

        let mut max_err = 0.0f32;
        let mut settled_err = 0.0f32;
        for k in 1..4000u64 {
            let t = k * dt_us;
            theta = wrap_angle(theta + omega * dt);

            // Edge fires exactly when the rotor crosses into the next sector
            // (true boundary at sector_center + width/2 for forward).
            let next = (sector + 1) % 6;
            let next_entry = wrap_angle(next as f32 * width - width / 2.0);
            if angle_difference(theta, next_entry) >= 0.0
                && angle_difference(theta, next_entry) < 2.0 * omega * dt
            {
                sector = next;
                hall.update(CW_SEQ[sector], t).unwrap();
            }

            let est = hall.sample_at(t).unwrap().angle;
            let err = angle_difference(est, theta).abs();
            max_err = max_err.max(err);
            // Give the estimator one electrical revolution to settle
            // velocity, then record the worst tracking error.
            if k > 700 {
                settled_err = settled_err.max(err);
            }
        }

        // A boundary-anchored estimator tracks within a few degrees
        // (velocity quantization + edge timing). The center-anchored
        // convention fails this at ≈ width/2 ≈ 0.52 rad.
        assert!(
            settled_err < 0.15,
            "hall estimate leads/lags the true rotor by {settled_err} rad \
             (max {max_err}) — interpolation anchor is biased"
        );
    }

    #[test]
    fn velocity_estimate_decays_when_edges_stop() {
        // A motor that stops mid-sector stops producing edges; the estimate
        // must be bounded by the time already waited — if the motor were
        // still doing |vel|, an edge would have arrived within one sector
        // time. VESC bounds interpolation speed the same way through
        // max(hall_dt_now, hall_dt_last), which keeps growing after the last
        // edge (foc_math.c:645). Holding the last edge speed forever feeds
        // phantom velocity into blending logic and telemetry.
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        for s in CW_SEQ.iter().cycle().take(12) {
            hall.update(*s, t).unwrap();
            t += 1000; // 1 ms per sector ≈ 10k eRPM
        }
        let running = hall.sample_at(t).unwrap().omega.abs();
        assert!(running > 500.0, "sanity: motor was spinning, got {running}");

        // 50 ms after the last edge, with no new edges:
        let stale = hall.sample_at(t + 50_000).unwrap();
        assert!(
            stale.omega.abs() < 0.05 * running,
            "stale velocity {} did not decay (running was {})",
            stale.omega,
            running
        );
    }

    // ========================================================================
    // Per-bit wire-break detector (note_wire_activity)
    // ========================================================================

    /// Feed `revs` electrical revolutions of the CW sequence with hall bits
    /// forced by `mask_and`/`mask_or` (a stuck wire), deduplicating equal
    /// consecutive observed states — the capture hardware only fires on a
    /// pin CHANGE, so masked transitions produce no update() call.
    fn spin_with_stuck_bits(hall: &mut HallSensor, revs: usize, mask_and: u8, mask_or: u8) {
        const CW: [u8; 6] = [1, 3, 2, 6, 4, 5];
        let mut t = 0u64;
        let mut last_observed = 0xFFu8; // no dedup on the first state
        for _ in 0..revs {
            for raw in CW {
                let observed = (raw & mask_and) | mask_or;
                if observed != last_observed {
                    let _ = hall.update(observed, t);
                    last_observed = observed;
                    t += 1000;
                }
            }
        }
    }

    #[test]
    fn wire_dead_stuck_low_named() {
        // H1 (bit 0) stuck low: per revolution the observed sequence is
        // 0(invalid), 2, 6, 4 — bit 0 never toggles, one invalid per rev.
        let mut hall = HallSensor::new(1_000_000);
        spin_with_stuck_bits(&mut hall, 8, 0b110, 0);
        assert_eq!(
            hall.fault_kind(),
            Some(HallFaultKind::WireDead { mask: 0b001 }),
            "stuck-low H1 must be named"
        );
    }

    #[test]
    fn wire_dead_stuck_high_named() {
        // H3 (bit 2) stuck high (broken wire + pull-up): the 0b011 sector
        // reads 0b111 = invalid, bit 2 never toggles.
        let mut hall = HallSensor::new(1_000_000);
        spin_with_stuck_bits(&mut hall, 8, 0b111, 0b100);
        assert_eq!(
            hall.fault_kind(),
            Some(HallFaultKind::WireDead { mask: 0b100 }),
            "stuck-high H3 must be named"
        );
    }

    #[test]
    fn two_dead_wires_named() {
        // H1+H2 stuck low: only H3 toggles (observed 0/4 alternation),
        // invalid every revolution.
        let mut hall = HallSensor::new(1_000_000);
        spin_with_stuck_bits(&mut hall, 14, 0b100, 0);
        assert_eq!(
            hall.fault_kind(),
            Some(HallFaultKind::WireDead { mask: 0b011 }),
            "both dead wires must be named"
        );
    }

    #[test]
    fn rocking_across_one_boundary_is_not_a_dead_wire() {
        // Vibration / a board nudged on a slope: the rotor rocks across a
        // single sector boundary, toggling ONE bit back and forth. By
        // counters alone that reads as "the other two wires are dead" —
        // the invalid-state gate must keep the verdict away (legitimate
        // rocking never produces 0/7).
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        hall.update(1, t);
        for _ in 0..30 {
            t += 1000;
            hall.update(3, t); // 1↔3: bit 1 toggles, adjacent states
            t += 1000;
            hall.update(1, t);
        }
        assert_eq!(hall.fault_kind(), None, "rocking must not flag wires");
        assert_eq!(hall.error_count(), 0, "rocking is legitimate motion");
    }

    #[test]
    fn healthy_rotation_stays_verdict_free() {
        let mut hall = HallSensor::new(1_000_000);
        spin_with_stuck_bits(&mut hall, 10, 0b111, 0);
        assert_eq!(hall.fault_kind(), None);
    }

    #[test]
    fn bounce_flags_error_rate() {
        // Stochastic glitches: non-adjacent jumps (1↔2 is two sectors
        // apart) rack up error_count without invalid states — the rate
        // window flags it, the wire verdict stays away.
        let mut hall = HallSensor::new(1_000_000);
        let mut t = 0u64;
        for _ in 0..15 {
            hall.update(1, t);
            t += 1000;
            hall.update(2, t);
            t += 1000;
        }
        assert_eq!(hall.fault_kind(), Some(HallFaultKind::ErrorRate));
    }

    #[test]
    fn wire_details_name_the_pins() {
        let d = HallFaultKind::WireDead { mask: 0b101 }.details();
        assert_eq!(d.as_str(), "dead wire: H1 H3");
    }
}
