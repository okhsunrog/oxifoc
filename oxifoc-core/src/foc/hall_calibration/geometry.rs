//! Measured transition boundaries, distinct from Hall-sector centroids.

use super::CalibrationError;
use crate::foc::{angle_difference, wrap_angle};
use core::f32::consts::{PI, TAU};

// Three repeats with 5° RMS spread already have uncertain edge geometry.
const MIN_DIRECTION_CONCENTRATION: f32 = 0.995;
// A direct repeat bound catches outliers hidden by otherwise concentrated data.
const MAX_REPEAT_DIFFERENCE_RAD: f32 = 10.0 * TAU / 360.0;
// Includes friction/tracking lag; weak alignment must not become commutation data.
const MAX_DIRECTION_SEPARATION_RAD: f32 = TAU / 8.0;
const COMMAND_BIN_RAD: f32 = TAU / 360.0;

/// Validated geometry of one positive electrical revolution.
///
/// `entry_angles[raw]` is the boundary crossed when entering `raw` while the
/// commanded electrical angle increases. `next_states` records that order
/// explicitly, so Hall wiring does not prescribe the electrical direction.
/// Construct through `try_from_raw_parts`; deserialization must not bypass it.
#[derive(Clone, Copy, Debug)]
pub struct HallTransitionGeometry {
    entry_angles: [f32; 8],
    next_states: [u8; 8],
    sector_widths: [f32; 8],
}

impl HallTransitionGeometry {
    pub fn try_from_raw_parts(
        mut entry_angles: [f32; 8],
        mut next_states: [u8; 8],
        centroids: [f32; 8],
    ) -> Option<Self> {
        let mut seen = 0_u8;
        let mut raw = 1_u8;
        let mut total = 0.0;
        let mut sector_widths = [0.0; 8];
        for _ in 0..6 {
            if !(1..=6).contains(&raw) || seen & (1 << raw) != 0 {
                return None;
            }
            seen |= 1 << raw;
            let next = next_states[usize::from(raw)];
            if !(1..=6).contains(&next) || (raw ^ next).count_ones() != 1 {
                return None;
            }
            let entry = entry_angles[usize::from(raw)];
            let exit = entry_angles[usize::from(next)];
            let centroid = centroids[usize::from(raw)];
            if !entry.is_finite() || !exit.is_finite() || !centroid.is_finite() {
                return None;
            }
            let width = angle_difference(exit, entry);
            let center_offset = wrap_angle(centroid - entry);
            if !(0.0..PI).contains(&width) || width == 0.0 || center_offset > width + 1.0e-5 {
                return None;
            }
            entry_angles[usize::from(raw)] = wrap_angle(entry);
            sector_widths[usize::from(raw)] = width;
            total += width;
            raw = next;
        }
        if raw != 1 || seen != 0b0111_1110 || (total - TAU).abs() > 1.0e-4 {
            return None;
        }
        // Invalid raw states carry no geometry, regardless of DTO contents.
        entry_angles[0] = 0.0;
        entry_angles[7] = 0.0;
        next_states[0] = 0;
        next_states[7] = 0;
        Some(Self {
            entry_angles,
            next_states,
            sector_widths,
        })
    }

    pub fn entry_angles(&self) -> [f32; 8] {
        self.entry_angles
    }

    pub fn next_states(&self) -> [u8; 8] {
        self.next_states
    }

    pub fn boundary_for_positive_entry(&self, raw: u8) -> Option<f32> {
        (1..=6)
            .contains(&raw)
            .then(|| self.entry_angles[usize::from(raw)])
    }

    pub fn sector_width(&self, raw: u8) -> Option<f32> {
        (1..=6)
            .contains(&raw)
            .then(|| self.sector_widths[usize::from(raw)])
    }

    pub fn boundary_for_transition(&self, previous: u8, raw: u8) -> Option<f32> {
        if !(1..=6).contains(&previous) || !(1..=6).contains(&raw) {
            return None;
        }
        if self.next_states[usize::from(previous)] == raw {
            self.boundary_for_positive_entry(raw)
        } else if self.next_states[usize::from(raw)] == previous {
            self.boundary_for_positive_entry(previous)
        } else {
            None
        }
    }
}

/// Known commanded-angle direction of a calibration sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HallSweepDirection {
    Positive,
    Negative,
}

#[derive(Clone, Copy, Default)]
struct Moments {
    sin: [f32; 8],
    cos: [f32; 8],
    count: [u16; 8],
    // Track the observed range relative to the first edge, so the final mean
    // can be checked against every extreme without retaining all observations.
    first: [f32; 8],
    min_delta: [f32; 8],
    max_delta: [f32; 8],
}

impl Moments {
    fn covers_turns(&self, sweeps: u8) -> bool {
        let total: u32 = self.count.iter().map(|&n| u32::from(n)).sum();
        let expected = u32::from(sweeps) * 6;
        // Only the initial open endpoint can be absent; missing a whole turn
        // must not pass merely because every boundary individually has N-1.
        expected > 0 && (expected - 1..=expected).contains(&total)
    }

    fn record(&mut self, raw: u8, angle: f32) {
        use crate::foc::trig::{FastSinCos, SinCos};
        let i = usize::from(raw);
        if self.count[i] == 0 {
            self.first[i] = angle;
        }
        let delta = angle_difference(angle, self.first[i]);
        self.min_delta[i] = self.min_delta[i].min(delta);
        self.max_delta[i] = self.max_delta[i].max(delta);
        let (sin, cos) = FastSinCos::sin_cos(angle);
        self.sin[i] += sin;
        self.cos[i] += cos;
        self.count[i] = self.count[i].saturating_add(1);
    }

    fn mean(&self, raw: u8, sweeps: u8) -> Option<f32> {
        let i = usize::from(raw);
        let count = self.count[i];
        // An open-ended N-turn block can miss its initial boundary once.
        let minimum = u16::from(sweeps.saturating_sub(1).max(1));
        if count < minimum || count > u16::from(sweeps) {
            return None;
        }
        let length_squared = self.sin[i] * self.sin[i] + self.cos[i] * self.cos[i];
        let count = f32::from(count);
        let threshold = MIN_DIRECTION_CONCENTRATION * count;
        if length_squared < threshold * threshold {
            return None;
        }
        let mean = wrap_angle(libm::atan2f(self.sin[i], self.cos[i]));
        let mean_delta = angle_difference(mean, self.first[i]);
        if (self.min_delta[i] - mean_delta)
            .abs()
            .max((self.max_delta[i] - mean_delta).abs())
            > MAX_REPEAT_DIFFERENCE_RAD
        {
            return None;
        }
        Some(mean)
    }
}

#[derive(Clone, Copy, Default)]
pub(super) struct DirectedTransitions {
    positive: Moments,
    negative: Moments,
    next_states: [u8; 8],
    previous: Option<(f32, u8, HallSweepDirection)>,
    invalid: bool,
}

impl DirectedTransitions {
    pub(super) fn record(&mut self, angle: f32, raw: u8, direction: HallSweepDirection) {
        if !angle.is_finite() || !(1..=6).contains(&raw) {
            self.invalid = true;
            self.previous = None;
            return;
        }
        let previous = self.previous.replace((angle, raw, direction));
        let Some((previous_angle, previous_raw, previous_direction)) = previous else {
            return;
        };
        if previous_direction != direction {
            return;
        }
        let step = angle_difference(angle, previous_angle);
        let sign = if direction == HallSweepDirection::Positive {
            1.0
        } else {
            -1.0
        };
        // Bracket actual transitions by adjacent commands: the midpoint has
        // at most half a one-degree command bin of angular discretization.
        if step * sign < -1.0e-6 || step.abs() > COMMAND_BIN_RAD + 1.0e-5 {
            self.invalid = true;
            return;
        }
        if raw == previous_raw {
            return;
        }
        if (raw ^ previous_raw).count_ones() != 1 {
            self.invalid = true;
            return;
        }
        let boundary = wrap_angle(previous_angle + step / 2.0);
        let (entering_positive, leaving_positive, moments) = match direction {
            HallSweepDirection::Positive => (raw, previous_raw, &mut self.positive),
            HallSweepDirection::Negative => (previous_raw, raw, &mut self.negative),
        };
        let next = &mut self.next_states[usize::from(leaving_positive)];
        if *next != 0 && *next != entering_positive {
            self.invalid = true;
            return;
        }
        *next = entering_positive;
        moments.record(entering_positive, boundary);
    }

    pub(super) fn finish(
        self,
        centroids: [f32; 8],
        positive_sweeps: u8,
        negative_sweeps: u8,
    ) -> Result<HallTransitionGeometry, CalibrationError> {
        let invalid = CalibrationError::InvalidTransitionGeometry;
        if self.invalid
            || !self.positive.covers_turns(positive_sweeps)
            || !self.negative.covers_turns(negative_sweeps)
        {
            return Err(invalid);
        }
        let mut boundaries = [0.0; 8];
        for raw in 1..=6 {
            let positive = self.positive.mean(raw, positive_sweeps).ok_or(invalid)?;
            let negative = self.negative.mean(raw, negative_sweeps).ok_or(invalid)?;
            let separation = angle_difference(positive, negative);
            // This includes friction and field-tracking lag as well as Hall
            // hysteresis; equal directional weight cancels only its mean.
            if separation.abs() > MAX_DIRECTION_SEPARATION_RAD {
                return Err(invalid);
            }
            boundaries[usize::from(raw)] = wrap_angle(negative + separation / 2.0);
        }
        HallTransitionGeometry::try_from_raw_parts(boundaries, self.next_states, centroids)
            .ok_or(invalid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::hall_calibration::HallCalibrator;

    const RING: [u8; 6] = [3, 2, 6, 4, 5, 1];
    const ENTRIES_DEG: [f32; 8] = [0.0, 300.0, 70.0, 350.0, 200.0, 240.0, 110.0, 0.0];
    const CENTERS_DEG: [f32; 8] = [0.0, 325.0, 90.0, 30.0, 220.0, 270.0, 155.0, 0.0];

    fn radians(values: [f32; 8]) -> [f32; 8] {
        values.map(|v| v * TAU / 360.0)
    }

    fn successors() -> [u8; 8] {
        let mut next = [0; 8];
        for (i, raw) in RING.into_iter().enumerate() {
            next[usize::from(raw)] = RING[(i + 1) % 6];
        }
        next
    }

    fn physical_state(degrees: f32) -> u8 {
        for (i, raw) in RING.into_iter().enumerate() {
            let start = ENTRIES_DEG[usize::from(raw)];
            let end = ENTRIES_DEG[usize::from(RING[(i + 1) % 6])];
            if (degrees - start).rem_euclid(360.0) < (end - start).rem_euclid(360.0) {
                return raw;
            }
        }
        unreachable!()
    }

    fn directed_sweep(positive: u8, negative: u8, lag_deg: f32) -> HallCalibrator {
        let mut cal = HallCalibrator::new();
        for (count, direction) in [
            (positive, HallSweepDirection::Positive),
            (negative, HallSweepDirection::Negative),
        ] {
            for _ in 0..count {
                for step in 0..360 {
                    let (degrees, lag) = match direction {
                        HallSweepDirection::Positive => (step as f32, lag_deg),
                        HallSweepDirection::Negative => ((359 - step) as f32, -lag_deg),
                    };
                    cal.record_directed(
                        degrees * TAU / 360.0,
                        physical_state(degrees - lag),
                        direction,
                    );
                }
            }
        }
        cal
    }

    #[test]
    fn unequal_geometry_is_wiring_independent_and_maps_both_directions() {
        let g = HallTransitionGeometry::try_from_raw_parts(
            radians(ENTRIES_DEG),
            successors(),
            radians(CENTERS_DEG),
        )
        .unwrap();
        let expected = [80.0, 40.0, 90.0, 40.0, 60.0, 50.0];
        for (i, raw) in RING.into_iter().enumerate() {
            let previous = RING[(i + 5) % 6];
            let boundary = g.boundary_for_positive_entry(raw).unwrap();
            assert_eq!(g.boundary_for_transition(previous, raw), Some(boundary));
            assert_eq!(g.boundary_for_transition(raw, previous), Some(boundary));
            assert!((g.sector_width(raw).unwrap() * 360.0 / TAU - expected[i]).abs() < 0.001);
        }
        assert_eq!(g.boundary_for_transition(3, 6), None);
        assert_eq!(g.sector_width(0), None);
        assert_eq!(g.boundary_for_positive_entry(7), None);
        assert!(
            HallTransitionGeometry::try_from_raw_parts(
                g.entry_angles(),
                g.next_states(),
                radians(CENTERS_DEG),
            )
            .is_some()
        );
    }

    #[test]
    fn malformed_or_centroid_inconsistent_geometry_is_rejected() {
        let entries = radians(ENTRIES_DEG);
        let centers = radians(CENTERS_DEG);
        let mut bad = entries;
        bad[2] = f32::NAN;
        assert!(HallTransitionGeometry::try_from_raw_parts(bad, successors(), centers).is_none());
        let mut bad_ring = successors();
        bad_ring[3] = 1; // closes a two-state cycle, not a full electrical turn
        assert!(HallTransitionGeometry::try_from_raw_parts(entries, bad_ring, centers).is_none());
        let mut bad_centers = centers;
        bad_centers[2] = 180.0 * TAU / 360.0;
        assert!(
            HallTransitionGeometry::try_from_raw_parts(entries, successors(), bad_centers)
                .is_none()
        );
        let mut bad = entries;
        bad[2] = entries[3]; // zero width
        assert!(HallTransitionGeometry::try_from_raw_parts(bad, successors(), centers).is_none());
    }

    #[test]
    fn directed_sweeps_recover_boundaries_with_half_bin_error_and_equal_direction_weight() {
        // Equal weighting cancels mean tracking lag despite unequal4F/3R counts.
        let cal = directed_sweep(4, 3, 10.25);
        for raw in 1..=6 {
            assert!((3..=4).contains(&cal.transitions.positive.count[raw]));
            assert!((2..=3).contains(&cal.transitions.negative.count[raw]));
        }
        let result = cal.finish_bidirectional(4, 3).unwrap();
        let geometry = result.geometry.unwrap();
        for raw in 1..=6 {
            let error = angle_difference(geometry.entry_angles()[raw], radians(ENTRIES_DEG)[raw]);
            assert!(
                error.abs() <= COMMAND_BIN_RAD / 2.0 + 1.0e-5,
                "raw{raw}: {error}"
            );
        }
        // The legacy sin/cos sector centers are still supplied and lie inside
        // the measured sectors, rather than being replaced with edge angles.
        assert!(result.is_valid());
        assert!(angle_difference(result.angles[2], geometry.entry_angles()[2]).abs() > 0.2);
    }

    #[test]
    fn incomplete_or_excessive_directional_coverage_is_not_accepted() {
        assert!(
            directed_sweep(3, 0, 0.0)
                .finish_bidirectional(3, 0)
                .is_err()
        );
        assert!(
            directed_sweep(3, 3, 0.0)
                .finish_bidirectional(4, 3)
                .is_err()
        );
        assert!(
            directed_sweep(3, 3, 0.0)
                .finish_bidirectional(2, 3)
                .is_err()
        );
        let legacy = directed_sweep(3, 0, 0.0).finish().unwrap();
        assert!(legacy.geometry.is_none());
    }

    #[test]
    fn direction_separation_gate_rejects_weak_alignment() {
        assert!(
            directed_sweep(3, 3, 22.0)
                .finish_bidirectional(3, 3)
                .is_ok()
        );
        assert!(
            directed_sweep(3, 3, 23.0)
                .finish_bidirectional(3, 3)
                .is_err()
        );
    }

    #[test]
    fn concentration_and_repeat_limits_are_independent() {
        let mut spread = Moments::default();
        for degrees in [0.0_f32, -9.0, 9.0] {
            spread.record(1, degrees.to_radians());
        }
        assert!(spread.mean(1, 3).is_none()); // concentrated range, poor resultant
        let mut outlier = Moments::default();
        for i in 0..20 {
            outlier.record(1, if i == 19 { 11.0_f32.to_radians() } else { 0.0 });
        }
        assert!(outlier.mean(1, 20).is_none()); // resultant passes, outlier does not
        let mut good = Moments::default();
        for degrees in [359.0_f32, 0.0, 1.0] {
            good.record(1, degrees.to_radians());
        }
        assert!(angle_difference(good.mean(1, 3).unwrap(), 0.0).abs() < 0.001);
    }

    #[test]
    fn command_gaps_invalid_states_and_inconsistent_direction_ring_are_rejected() {
        let mut transitions = DirectedTransitions::default();
        transitions.record(0.0, 3, HallSweepDirection::Positive);
        transitions.record(2.0 * COMMAND_BIN_RAD, 2, HallSweepDirection::Positive);
        assert!(transitions.invalid);
        let mut transitions = DirectedTransitions::default();
        transitions.record(0.0, 0, HallSweepDirection::Positive);
        assert!(transitions.invalid);
        let mut transitions = DirectedTransitions::default();
        transitions.record(0.0, 3, HallSweepDirection::Positive);
        transitions.record(COMMAND_BIN_RAD, 6, HallSweepDirection::Positive);
        assert!(transitions.invalid); // two inputs changed between samples
    }

    #[test]
    fn reset_discards_directed_geometry_as_well_as_centroid_samples() {
        let mut cal = directed_sweep(3, 3, 0.0);
        cal.reset();
        assert_eq!(cal.sample_count(3), 0);
        assert_eq!(cal.transitions.positive.count, [0; 8]);
        assert_eq!(cal.transitions.negative.count, [0; 8]);
        assert!(cal.transitions.previous.is_none());
    }
}
