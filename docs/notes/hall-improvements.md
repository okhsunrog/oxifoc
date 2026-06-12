# Hall Angle Estimation — Ideas to Borrow from VESC & MESC

> **STATUS: landed 2026-06-12, except §HallPll (open).** Details of what
> was implemented are in [decisions.md](../decisions.md) (the
> "table = centroids, anchor = boundary" convention) and commit `9f936bb`;
> the full original analysis is in this file's git history.

Working notes from a line-by-line comparison of the Hall pipeline against
**VESC** (`bldc`, `motor/foc_math.c`) and **MESC** (`MESC_Firmware`,
`MESC_Common/Src/MESCfoc.c` / `MESCmeasure.c`). The acquisition layer
(`oxifoc-g431/src/sensors.rs`, TIM4 XOR hardware capture, 1 MHz latched
edge timestamps) was already ahead of both references — everything here
concerned the platform-agnostic estimator/calibrator.

## Landed (details: decisions.md + 9f936bb)

- **§1 Half-sector interpolation lead — confirmed and fixed.**
  A regression test with an independent continuous rotor model
  (`interpolation_tracks_continuous_rotor`) measured 0.527 rad ≈ 30.2°
  of systematic lead (≈13% torque loss + parasitic d-current);
  `update()` now anchors the base to the boundary = midpoint of adjacent
  calibrated centroids (VESC-style); the centroid remains for the
  low-speed snap and fallbacks.
- **§2 Asymmetric sensor placement** — absorbed by the midpoint of the
  measured centroids; velocity uses the measured boundary-to-boundary
  sector width.
- **§3 Calibrator extension (width/boundaries)** — NOT NEEDED: the
  midpoint approach stores no widths (just like VESC).
- **§5 Regression test** — added; the lesson "paired sim/estimator
  conventions hide offsets" is recorded in decisions.md (the
  `VirtualMotor` hall convention was fixed along the way).
- Anchoring on a skipped edge (non-adjacent transition) — a known
  residual inaccuracy of ±30°; analysis and a sketch of the exact fix
  are in TODO.md; dissolved by the HallPll.

Bench leftover: confirm on hardware that the d-current at constant speed
is centered (TODO.md → Bench).

## [idea, OPEN] PLL-based Hall observer vs open-loop interpolation

Architectural, larger scope. Our Hall path is **open-loop interpolation**:
`base + velocity·dt` with a drift clamp and a rate limiter
(`sample_at_mut`, `hall_sensor.rs`). VESC is the same shape
(`foc_correct_hall`). MESC instead runs a **proper PLL** on the Hall edges
(`angleObserver`, `MESCfoc.c:961`):

```
FOCAngle += angle_step  −  one_on_period · hall_error
            (feed-forward)   (proportional pull toward the known boundary)
```

We already own a high-quality PLL — `BackEmfObserver` (`phase/observer.rs`)
uses exactly this structure. A "HallPll" variant would:

- track angle continuously (no per-cycle clamp/rate-limit hacks — the
  drift corrector, rate limiter and decayed-velocity bound all become
  loop dynamics with one bandwidth knob),
- naturally produce a smooth velocity estimate (no edge-to-edge
  quantization — direct cure for the hall-velocity lag limiting the
  velocity-loop bandwidth, see TODO.md),
- dissolve the skipped-edge anchor issue (innovation is taken against the
  known boundary of whatever sector we're in),
- share gain-tuning intuition with the back-EMF PLL.

Worth prototyping against the `VirtualMotor` — as an *additional*
`AngleSensor` next to the current estimator, judged by the
independent-rotor regression test; not a replacement until it wins on sim
and bench. Caveats: low speed needs gain scheduling or the same low-speed
snap (edges seconds apart); our 1 MHz hardware edge timestamps remove the
velocity-quantization pain MESC fights, so the win is the unified
boundary-anchored dynamics, not raw velocity precision. MESC's own code
carries several `// Does not work... Why??` dead ends here
(`MESCfoc.c:990,1004`); treat its gains as a starting hint, not gospel.

Triggers to actually do it: (a) the bench shows that velocity cruise on
halls can't be tuned with soft gains; (b) starting position control
(needs a clean continuous angle/velocity).

## Reference map

**ours**
- `oxifoc-core/src/foc/hall_sensor.rs` — `update()` (boundary anchor),
  `sample_at_mut()` (drift correction + rate limiter),
  `interpolation_tracks_continuous_rotor` (the referee test)
- `oxifoc-core/src/foc/phase/observer.rs` — `BackEmfObserver` (the PLL
  skeleton to reuse)

**VESC** (`~/motor_control/bldc`)
- `motor/foc_math.c:591` `foc_correct_hall()` — boundary midpoint `:636`,
  low-speed snap `:649`, drift `×0.01` `:658`, rate-limit `×1.5` `:666`

**MESC** (`~/motor_control/MESC_Firmware`)
- `MESC_Common/Inc/MESCfoc.h:295` — `hall_table[6][4]` `{start,end,center,width}`
- `MESC_Common/Src/MESCfoc.c:922` `hallAngleEstimator()` — boundary error `:943`
- `MESC_Common/Src/MESCfoc.c:961` `angleObserver()` — Hall PLL, `angle_step`
