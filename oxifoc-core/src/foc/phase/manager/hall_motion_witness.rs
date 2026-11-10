//! Missing-edge evidence from independent angular travel, not a held speed.

use crate::foc::angle_difference;
use crate::foc::fast_math::u64_to_f32;
use crate::foc::hall_sensor::{STALE_MIN_WINDOW_S, STALE_SECTOR_MULTIPLE};
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::sensors::{HallEdgeObservation, HallEdgeTiming};

pub(super) struct HallMotionWitness {
    baseline: Option<Baseline>,
    seconds_per_tick: f32,
}

impl Default for HallMotionWitness {
    fn default() -> Self {
        Self {
            baseline: None,
            seconds_per_tick: 1e-6,
        }
    }
}

struct Baseline {
    edge_ticks: u64,
    reference_epoch: u32,
    phase: f32,
    travel: f32,
    stale: bool,
}

impl HallMotionWitness {
    pub(super) fn set_ticks_per_sec(&mut self, ticks_per_sec: u64) {
        self.seconds_per_tick = 1.0 / u64_to_f32(ticks_per_sec.max(1));
        self.reset();
    }
    pub(super) fn reset(&mut self) {
        self.baseline = None;
    }

    /// The observer must remain ready throughout the interval. Its constant
    /// angle offset cancels; a speed estimate left over after deceleration
    /// cannot turn a stopped rotor into a missing Hall edge. Signed travel
    /// also cancels rocking inside a sector rather than counting path length.
    pub(super) fn observe(
        &mut self,
        edge: HallEdgeObservation,
        observer: Option<PhaseOutput>,
        now_ticks: u64,
    ) -> bool {
        let Some(observer) = observer.filter(|s| s.angle.is_finite() && s.velocity.is_finite())
        else {
            self.reset();
            return false;
        };
        let phase = observer.angle;
        let distance = STALE_SECTOR_MULTIPLE * edge.sector_width_rad;
        let seconds_per_tick = self.seconds_per_tick;
        let overdue = || {
            let age = u64_to_f32(now_ticks.saturating_sub(edge.ticks)) * seconds_per_tick;
            let velocity = edge.velocity_rad_s.abs().min(observer.velocity.abs());
            velocity >= 1.0 && age > STALE_MIN_WINDOW_S && age * velocity > distance
        };
        // A recovered glitch reports a historical crossing. The current
        // observer phase cannot anchor travel at that earlier instant. Keep
        // the existing conservative timeout until a captured edge restores
        // a real baseline; otherwise recovery could mask another lost edge.
        if edge.timing == HallEdgeTiming::Reconstructed {
            self.reset();
            return overdue();
        }
        let Some(baseline) = self.baseline.as_mut().filter(|baseline| {
            baseline.edge_ticks == edge.ticks && baseline.reference_epoch == edge.reference_epoch
        }) else {
            self.baseline = Some(Baseline {
                edge_ticks: edge.ticks,
                reference_epoch: edge.reference_epoch,
                phase,
                travel: 0.0,
                stale: false,
            });
            return false;
        };
        baseline.travel += angle_difference(phase, baseline.phase);
        baseline.phase = phase;
        // Preserve the 300-us minimum observation window and 1.5-sector
        // allowance. At the FOC sample rate, successive observer angles must
        // be less than half a turn apart to unwrap travel unambiguously.
        if !baseline.stale && baseline.travel.abs() > distance && overdue() {
            baseline.stale = true;
        }
        // Once witnessed, a wrap or reversal cannot erase a missing edge.
        // A fresh physical edge (or a new observer/reference) rearms it.
        baseline.stale
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::wrap_angle;

    fn motion(angle: f32) -> Option<PhaseOutput> {
        Some(PhaseOutput {
            angle,
            velocity: 1000.0,
        })
    }

    const EDGE: HallEdgeObservation = HallEdgeObservation {
        reference_epoch: 0,
        ticks: 1000,
        timing: HallEdgeTiming::Captured,
        sector_width_rad: 0.6,
        velocity_rad_s: 1000.0,
    };

    #[test]
    fn deceleration_and_rocking_do_not_accumulate_fictitious_travel() {
        let mut witness = HallMotionWitness::default();
        for step in 0..2000 {
            let phase = [0.84, 1.04, 1.24, 1.04][step % 4];
            assert!(!witness.observe(EDGE, motion(phase), 1000 + step as u64 * 50));
        }
        assert!(!witness.observe(EDGE, motion(1.04), 10_000_000));
    }

    #[test]
    fn missing_edges_trip_in_both_directions_despite_angle_offset_and_wrap() {
        for direction in [-1.0, 1.0] {
            for offset in [0.0, 0.84, 6.2] {
                let mut witness = HallMotionWitness::default();
                for step in 0..30 {
                    let phase = wrap_angle(offset + direction * step as f32 * 0.05);
                    let stale = witness.observe(EDGE, motion(phase), 1000 + step * 50);
                    if step < 17 {
                        assert!(!stale);
                    } else if step > 19 {
                        assert!(stale);
                    }
                }
                // Returning to the original phase must not erase the verdict.
                assert!(witness.observe(EDGE, motion(offset), 3000));
            }
        }
    }

    #[test]
    fn new_edge_reference_or_observer_readiness_reanchors_the_witness() {
        let mut witness = HallMotionWitness::default();
        witness.observe(EDGE, motion(0.0), 1000);
        assert!(witness.observe(EDGE, motion(1.0), 2000));
        let edge = HallEdgeObservation {
            ticks: 2000,
            ..EDGE
        };
        assert!(!witness.observe(edge, motion(1.0), 2000));
        assert!(!witness.observe(
            HallEdgeObservation {
                reference_epoch: 1,
                ..edge
            },
            motion(3.0),
            3000
        ));
        assert!(!witness.observe(
            HallEdgeObservation {
                reference_epoch: 1,
                ..edge
            },
            None,
            4000
        ));
        assert!(!witness.observe(
            HallEdgeObservation {
                reference_epoch: 1,
                ..edge
            },
            motion(0.0),
            5000
        ));
        assert!(!witness.observe(
            HallEdgeObservation {
                reference_epoch: 1,
                ..edge
            },
            motion(f32::NAN),
            6000
        ));
        assert!(!witness.observe(
            HallEdgeObservation {
                reference_epoch: 1,
                ..edge
            },
            motion(3.0),
            7000
        ));
    }

    #[test]
    fn minimum_window_uses_sensor_clock_and_future_edges_do_not_wrap() {
        let mut witness = HallMotionWitness::default();
        witness.set_ticks_per_sec(2_000_000);
        let edge = HallEdgeObservation {
            sector_width_rad: 0.1,
            ..EDGE
        };
        witness.observe(edge, motion(0.0), 900);
        assert!(!witness.observe(edge, motion(1.0), 999));
        assert!(!witness.observe(edge, motion(1.0), 1599));
        assert!(witness.observe(edge, motion(1.0), 1601));
    }

    #[test]
    fn reconstructed_crossing_keeps_the_timeout_until_a_real_edge_arrives() {
        let mut witness = HallMotionWitness::default();
        let edge = HallEdgeObservation {
            timing: HallEdgeTiming::Reconstructed,
            ..EDGE
        };
        assert!(witness.observe(edge, motion(1.0), 2000));
        let captured = HallEdgeObservation {
            ticks: 2100,
            ..EDGE
        };
        assert!(!witness.observe(captured, motion(1.1), 2100));
        assert!(!witness.observe(captured, motion(1.2), 10_000));
    }
}
