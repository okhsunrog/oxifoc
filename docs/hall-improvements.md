# Hall Angle Estimation — Ideas to Borrow from VESC & MESC

Working notes from a line-by-line comparison of the Hall pipeline against the
two reference FOC firmwares: **VESC** (`bldc`, `motor/foc_math.c` /
`mcpwm_foc.c`) and **MESC** (`MESC_Firmware`, `MESC_Common/Src/MESCfoc.c` /
`MESCmeasure.c`). Status legend: **[bug?]** suspected defect to verify on the
bench · **[improvement]** correctness/quality upgrade · **[idea]** worth
evaluating.

The point of reference for "us" is:
- `oxifoc-core/src/foc/hall_sensor.rs` — platform-agnostic estimator
- `oxifoc-core/src/foc/hall_calibration.rs` — `HallCalibrator`
- `oxifoc-g431/src/sensors.rs` — TIM4 XOR hardware capture (the acquisition side
  is already *ahead* of both refs — hardware-latched 1 MHz edge timestamps +
  ICF debounce vs VESC's software N-sample majority vote in the ISR; nothing to
  borrow there)

---

## 1. [bug?] Interpolation anchors to the sector CENTER, not the entry BOUNDARY

This is the headline item — the one to check first on the bench, because an
unloaded motor still spins with it present.

### What we do

`HallCalibrator` stores the **centroid** of each Hall state. The sin/cos
average over every angle at which a state was active
(`hall_calibration.rs:118-164`) returns the *center* of that 60° sector — e.g.
θ = 30° for a sector spanning [0°, 60°].

On a Hall edge, `HallSensor::update` sets the interpolation base to that
centroid:

```rust
// hall_sensor.rs:433
self.angle = angle_raw;   // = calib.angle_for_state(raw_state) = sector CENTER
```

and `sample_at` then interpolates forward from it:

```rust
// hall_sensor.rs:569
let interpolated = wrap_angle(self.angle + velocity * dt);
```

### Why that is wrong at the edge

At the instant a Hall edge fires, the rotor is — by definition — sitting on the
**boundary between the old and new sectors**, i.e. at `center − width/2` for
forward rotation (≈ center − 30°). We seed the estimate at `center`, so:

| moment in sector | true rotor θ | our estimate | error |
|---|---|---|---|
| entry (dt=0)     | boundary (0°)        | center (30°)        | **+30°** |
| middle (dt=T/2)  | 30°                  | 60°                 | **+30°** |
| exit (dt=T)      | 60°                  | 90°                 | **+30°** |

The error does not wash out: both quantities advance at the same velocity, so a
**systematic ~half-sector (≈30° electrical) lead persists across the whole
sector** (a *lag* for reverse rotation). The soft drift-correction
(`drift × 0.01`, `hall_sensor.rs:580`) barely touches it because `drift =
interpolated − sector_center` is small through the first half of the sector.

Consequence: a constant commutation-angle offset → torque loss ≈ cos(30°) ≈ 13%
plus a spurious d-axis current. The motor still turns, which is exactly why this
survives casual bring-up.

### How BOTH references avoid it

**VESC — recenters to the boundary via the midpoint of two centroids:**

```c
// foc_math.c:636 — foc_correct_hall()
int ang_avg = motor->m_ang_hall_int_prev + diff / 2;   // midpoint of adjacent centers = the BOUNDARY
...
motor->m_ang_hall = ((float)ang_avg / 200.0) * 2.0 * M_PI;
// then interpolates forward: m_ang_hall += rad_per_sec_hall * dt   (foc_math.c:655)
```

Its comment is explicit: *"A transition was just made. The angle is in the
middle of the new and old angle."* The centroid (`ang_hall_now`) is used only
for the low-speed "just snap to nearest Hall" path (`foc_math.c:649`); the
*interpolation* starts from the boundary.

**MESC — stores the boundary explicitly and runs a PLL off it:**

```c
// MESCmeasure.c:501-504 — hall_table[state] = {start, end, center, width}
hall_table[i][2] = center;                      // centroid
hall_table[i][3] = width;                        // sector width
hall_table[i][0] = center - width/2;             // START boundary
hall_table[i][1] = center + width/2;             // END boundary
```

```c
// MESCfoc.c:943 — hallAngleEstimator(): error is taken against the START boundary (forward)
hall_error = FOCAngle - hall_table[state-1][0];  // [1]=end for reverse
```

MESC never jumps the running angle to the center at all — it feeds
`hall_error` (running angle vs the *exact* entry boundary) into a PLL/observer
(`angleObserver`, `MESCfoc.c:961`) with feed-forward `angle_step`.

### Why our unit tests don't catch it

`test_interpolation_and_velocity` / `test_low_speed_no_interpolation`
(`hall_sensor.rs` tests) define ground truth *as the stored sector angle
itself* — there is no independent continuous-rotor model. The center-based
convention is therefore self-consistent in the tests and the half-sector bias
is invisible. A regression test must drive a *separate* monotonic rotor angle
and assert the estimate tracks it within a few degrees through mid-sector.

### Proposed fix

1. In `update`, seed the interpolation base at the **entry boundary**, not the
   centroid:
   ```rust
   // forward: boundary = center - width/2 ; reverse: center + width/2
   self.angle = wrap_angle(sector_center - self.direction_sign() * (sector_width / 2.0));
   ```
2. Keep the **centroid** for the low-speed no-interpolation path — there the
   center is genuinely optimal (minimizes worst-case ±width/2 error).
3. Add a regression test with an independent continuous rotor angle (see §5).

This needs per-sector width/boundaries, which the calibrator does not yet
produce — see §3.

---

## 2. [improvement] Per-sector width — handle asymmetric Hall placement

Both refs treat each Hall state as having its **own** angular width; we hardcode
a uniform 60°:

```rust
// hall_sensor.rs:180
let angle_per_state = TAU / 6.0;   // assumes every sector is exactly 60°
```

Real Hall sensors are placed with ±5–10° mechanical scatter, so the six
electrical sectors are *not* equal. MESC carries `width` per state
(`hall_table[i][3]`) and uses it for the feed-forward step
(`MESCfoc.c:970`: `angle_step = (1/ticks) * hall_table[last_state][3]`). VESC's
boundary-midpoint approach absorbs asymmetry implicitly because it interpolates
between the two *measured* centroids.

Borrowing this means: velocity and interpolation use the **measured** width of
the sector actually being traversed, not a nominal 60°. Removes a second small
angle bias that stacks on top of §1 for badly-placed sensors.

---

## 3. [improvement] Extend `HallCalibrator` to extract width & boundaries

The calibrator is correct for what it does (sin/cos averaging with atan2 wrap
handling, 6-state validation, `min_samples=30` like VESC — `hall_calibration.rs`)
but it only produces **centroids**. To implement §1 and §2 it must also yield,
per state:

- **width** — span of observed angles. Can't come from sin/cos averaging;
  accumulate per-state min/max electrical angle (careful with wrap), or
  circular variance, during `record()`.
- **start / end boundaries** — `center ∓ width/2`, mirroring MESC's
  `hall_table[i][0..1]`.

MESC computes width directly from the per-state sample range during its sweep
(`MESCmeasure.c:495-504`). VESC derives the boundary on the fly from adjacent
centroids instead of storing width — either approach works; storing width is
more explicit and survives a non-uniform sweep.

Keep the centroid too — it stays the right value for the low-speed snap.

---

## 4. [idea] PLL-based Hall observer vs open-loop interpolation

Architectural, larger scope. Our Hall path is **open-loop interpolation**:
`base + velocity·dt` with a drift clamp and a rate limiter
(`sample_at_mut`, `hall_sensor.rs:616`). VESC is the same shape
(`foc_correct_hall`). MESC instead runs a **proper PLL** on the Hall edges
(`angleObserver`, `MESCfoc.c:961`):

```
FOCAngle += angle_step  −  one_on_period · hall_error
            (feed-forward)   (proportional pull toward the known boundary)
```

We already own a high-quality PLL — `BackEmfObserver` (`phase/observer.rs`)
uses exactly this structure. A "HallPll" variant would:

- track angle continuously (no per-cycle clamp/rate-limit hacks),
- naturally produce a smooth velocity estimate (no edge-to-edge quantization),
- share gain-tuning intuition with the back-EMF PLL,
- fold the drift correction and rate limiting into the loop dynamics instead of
  two ad-hoc post-filters.

Worth prototyping against the `VirtualMotor` once §1 is fixed; not urgent.
Caveat: our hardware capture already gives 1 MHz edge timestamps, so the
velocity-quantization MESC fights doesn't hurt us as much — the main win would
be the unified, boundary-anchored angle dynamics, not the velocity.

MESC's own code carries several `// Does not work... Why??` dead ends here
(`MESCfoc.c:990,1004`); treat its gains as a starting hint, not gospel.

---

## 5. Regression test that would have caught §1

Independent continuous rotor model, assert estimate tracks it through
mid-sector (not just at the stored angle):

```rust
// pseudocode — drive a monotonic electrical angle, feed Hall edges at the
// real sector boundaries, sample mid-sector, assert small error.
let mut theta = 0.0;
let width = TAU / 6.0;
for step in 0..N {
    theta = wrap_angle(theta + omega * dt);
    let sector = (theta / width) as u8;              // ground-truth sector
    if sector_changed { hall.update(raw_for(sector), t); }   // edge AT the boundary
    let est = hall.sample_at(t).unwrap().angle;
    // The current center-anchored code fails this by ~width/2:
    assert!(angle_difference(est, theta).abs() < 0.1, "lead/lag {} at θ={}", ..., theta);
}
```

Key difference from the existing tests: `theta` here is an external truth, not
`calib.angle_for_state(...)`.

---

## Priority / actionable order

1. **Verify §1 on the bench** (PSU-safe profile) — does the motor draw d-axis
   current / lose torque consistent with a ~30° lead? Cheapest evidence.
2. **Add the §5 regression test** — confirms the bias in software, guards the fix.
3. **Extend `HallCalibrator` (§3)** to emit width + boundaries.
4. **Fix `HallSensor` interpolation (§1)** to anchor at the entry boundary;
   keep centroid for the low-speed snap. Use measured width (§2).
5. **(Later) Prototype a HallPll (§4)** on `VirtualMotor`.

None of this touches the acquisition layer (`oxifoc-g431/src/sensors.rs`) — the
TIM4 XOR hardware capture is already better than both references. The fixes are
all in the platform-agnostic estimator + calibrator, so they're host-testable.

---

## Reference map (for when you come back to this)

**ours**
- `oxifoc-core/src/foc/hall_sensor.rs` — `update():433` (center anchor),
  `sample_at():544`, `sample_at_mut():616`, `decayed_velocity():602`,
  `angle_per_state` (uniform 60°) `:180`
- `oxifoc-core/src/foc/hall_calibration.rs` — sin/cos accumulate `:118`,
  `finish()` atan2 `:146`

**VESC** (`~/motor_control/bldc`)
- `motor/foc_math.c:591` `foc_correct_hall()` — boundary midpoint `:636`,
  low-speed snap `:649`, interpolate `:655`, drift `×0.01` `:658`,
  rate-limit `×1.5` `:666`, hall↔observer blend `:694`
- `motor/mcpwm_foc.c:2356` `mcpwm_foc_hall_detect()` — centroid table `:2443`
- `util/utils_sys.c:89` `utils_read_hall()` — software N-sample majority vote

**MESC** (`~/motor_control/MESC_Firmware`)
- `MESC_Common/Inc/MESCfoc.h:295` — `hall_table[6][4]`
- `MESC_Common/Src/MESCmeasure.c:495-504` — `{start,end,center,width}` extraction
- `MESC_Common/Src/MESCfoc.c:922` `hallAngleEstimator()` — boundary error `:943`
- `MESC_Common/Src/MESCfoc.c:961` `angleObserver()` — Hall PLL, `angle_step`
  from width `:970`, FOCAngle update `:982`
