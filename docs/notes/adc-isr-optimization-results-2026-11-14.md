# F405 ADC ISR optimization results, 2026-11-14

Baseline: `8fa1e5b`. This continues
[the earlier handoff](adc-isr-optimization-handoff-2026-11-14.md).
Artifacts, exact firmware images, patches, scripts, captures and logs:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-isr-opt-2026-11-14/`.

## Implemented changes

- Reuse the raw Hall snapshot for control at the same timestamp. The control
  angle still passes through its stateful limiter and physical-travel estimator;
  telemetry retains the raw angle and edge speed. Capture edges, configuration
  changes and mutable sensor access invalidate the snapshot. The two existing
  critical sections remain; this saves repeated arithmetic, not both locks.
- Give the command FIFO an atomic empty check. Queue publication and flag updates
  share a critical section, including resumed async sends; cancellation, FIFO
  order and full-queue behavior are retained. Native-rate watchdog and protection
  work remains outside the command drain and is unchanged.
- Write the three clamped TIM1 CCR values directly inside the existing UDIS
  transaction. Duty limiting, channel enables, dead time, stopped retiming and
  the post-write deadline witness are retained.
- Prepare source-dependent Hall/encoder/HFI usage and sensored-trust dispatch at
  source selection. Observer readiness, startup and fallback remain live checks.
- Replace selected divisions by reciprocal multiplication and avoid a current
  circle square root when the request is strictly inside the squared budget.
  The strict comparison retains the old boundary and underflow behavior.

No command or protection cadence was reduced. No observer work was decimated.
Command processing and PWM publication retain their order.

## Numerical contracts

Bit identity is required between direct and reused torque constraints, but the
frozen pre-optimization oracle allows at most two f32 ULPs for finite nonzero
results of reciprocal arithmetic. Zero, infinity and NaN behavior retains its
separate checks. A physical-unit test bounds bus-current error in both directions
by `4 * f32::EPSILON * max(bus_limit, 1 A)`; at 100 A this is under 48 microamps.
The measured-current trip is independent of this numerical tolerance.

Temperature multiplication has less than 0.00025 degrees C representation error
over the entire i16 input domain. Inputs already have a 0.1-degree quantum. At an
exact threshold, rounding can make the trip occur one input quantum earlier;
tested positive thresholds never trip later. Derating cadence remains unchanged.

Voltage/speed codec tests cover every signed/unsigned 16-bit boundary and adjacent
f32 values, saturation and nonfinite inputs. Encoding can differ from division
by one quantum. Angle tests use modular differences and cover every wire boundary
and adjacent f32 value over +/-4 turns, with at most one quantum error. This does
not establish a one-quantum bound for arbitrarily large floating-point angles.
Public `Scale` values whose reciprocal overflows or underflows retain division.

Hall tests include the frozen `8fa1e5b` control algorithm, separate from the new
snapshot/limiter helpers. That comparison shares the timebase conversion to
isolate the structural refactor from reciprocal rounding. Other tests cover
timestamp changes, intervening edges/configuration, reversals, glitches,
resynchronization and raw versus limited telemetry semantics.

## Matched 15 kHz measurements

MK5, H5055 unloaded, approximately 24 V, calibrated 167.836 MHz CPU,
HallToObserver 150/300 electrical rad/s, native Packed28 through SPI/C5.
Same compiler and release settings as the handoff. Two accepted 4-second runs
per speed and variant; baseline was repeated before and after the first variant.
All values below are DWT body times, not complete exception/WCET bounds.

| Variant | Mean at +120 erad/s, us | Mean at +450 erad/s, us |
| --- | ---: | ---: |
| Baseline before | 46.362 | 45.661 |
| Baseline repeat | 46.344 | 45.670 |
| Hall reuse + initial arithmetic | 45.191 | 44.595 |
| Plus FIFO empty check and direct CCR writes | 43.625 | 42.984 |
| Plus temperature, angle and bus-bound reciprocals | 43.751 | 43.103 |
| Plus prepared source dispatch (final) | **42.840** | **42.213** |

Final improvement is approximately 3.5 us / 7.6%. The last arithmetic group
alone did not show an elapsed-time improvement: its +0.12 us is within the
handoff's relinking/layout sensitivity. It is not credited with a measured win.
The Hall-only contribution was not isolated from the first arithmetic group.

Final p99 bin upper edges: 48.428 / 48.047 us. Observed maxima: 54.095 /
51.765 us. No 15-kHz period-budget violations occurred in accepted runs.

## 20 kHz experiment

A temporary, archived one-line boot override selected 20 kHz without changing
stored configuration. It has been removed from the working tree. Body timing
and PWM deadline witnesses use separate images; do not add their measurements
as exact accounting of one image. Actual timer rate is recorded in each
`*.registers.json` and capture metadata.

| Speed | Mean ISR, us | Maximum ISR, us | Bodies over one period | Active cycles |
| --- | ---: | ---: | ---: | ---: |
| +120 erad/s | 42.728 | 54.035 | 43 | 160126 |
| +450 erad/s | 42.125 | 51.711 | 8 | 160151 |

The separate deadline image retained 320294 CCR commits: zero late writes,
zero ambiguous entries and zero writes left masked. The smallest observed
margin to the preload transfer was 7.722 us (the first write of one run).
The other run minima were 13.203, 11.690 and 13.180 us.

All eight 20-kHz captures were lossless and had unchanged fault generations.
Nevertheless, the 51 body overruns in 320277 active cycles mean a strict 50-us
ISR bound is **not established**. These short unloaded runs also do not qualify
loaded operation or rare fault paths. The normal stored rate remains 15 kHz.

## Validation and reproduction

Core/Embassy: 741 tests passed, 2 ignored. Doctests: 11 passed. Host library/CLI:
48 tests passed in the Nix development environment. Release builds
passed for F405 MK5, F405 CF2 and G474. G474 hardware timing was not measured.
Formatting and patch-whitespace checks pass.

Unmodified detection code prevents the full strict core Clippy command passing:
two `needless_range_loop` and one `chunks_exact_to_as_chunks` finding in
`detection/sweep.rs`, and two `redundant_closure_for_method_calls` findings in
`detection/embassy_hw.rs`. The same check passes with only those three lint kinds
allowed on the command line. The no-detection feature slice passes strict Clippy.
No repository lint policy was relaxed.

`bench.py` requires an explicitly prepared motor bench. It uses exact-image
defmt decoding, stopped-only SWD setup/export, native-rate recording, terminal
Stop, capture integrity checks and a known recovery image. `production.py`
additionally verifies configuration readback and retains the normal image only
after its captures pass. Run through `nix develop` for the CLI's native libraries,
and `uv run --no-project python -B` for the scripts.

Three initial timing attempts were rejected because SWD disconnect disabled
DWT and its 1-Hz supervisor repaired it during the active measurement. The
harness now waits 1.2 seconds while Stopped after the last setup access. Failed
attempts and their captures remain in the artifact directory; they are not
included in accepted timing statistics.

Remaining performance work should target the rare expensive cycles, using the
existing outlier/section recorder to distinguish command handling, the scheduled
velocity update and fault logging. Their coincidence is a hypothesis here;
this session did not collect section attribution for the 20-kHz overruns.

## Final board state

The normal optimized image (`final-release.elf`, SHA-256
`50696f310f61e894eef666527125387a546019a4a80c443a2153037d746be281`)
is flashed and matches the normal MK5 build output. Four final production
captures retained 295832 native-rate samples without loss. Across the session,
37 accepted captures retained 2933939 samples, with every maneuver command
acknowledged and unchanged fault generations.

Final readback: Stopped, zero faults, 24.033 V, FET 32.3 degrees C. The normal
PWM setting remains 15 kHz. Configuration groups match the initial readback
except for fresh boot current offsets; no configuration persistence commands
were issued. The temporary 20-kHz override is absent from the working tree.

## Continuation: layout sensitivity, readiness cache, angle helpers, Hall reuse removed

Second session on the same day, on top of the changes above. Artifacts:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-isr-review.dW5aab/`
(`codex-final.patch` is the starting state, `claude-angles-readiness.patch`
the two added source changes, `tree-final.patch` the complete final diff
against `8fa1e5b`).

All images in this section were built from one worktree path, because the
source directory itself changes the linked layout: the unchanged final state
above measures 42.840 / 42.213 us when built in the repository and
43.09 / 42.46 us when built from the worktree. That +0.25 us is larger than
the +0.12 us seen after the temperature/angle/bus reciprocals, so that step
cannot be called a regression; treat differences below about 0.3 us between
two relinked images as unresolved.

| Variant (15 kHz, native Packed28) | Mean at +120, us | Mean at +450, us |
| --- | ---: | ---: |
| State above, worktree build (three blocks) | 43.09 / 43.07 / 43.09 | 42.48 / 42.46 / 42.45 |
| `-align-all-functions=4` | 43.03 | 42.33 |
| `-align-all-nofallthru-blocks=4` | 43.42 | 42.72 |
| Both alignment flags | 43.43 | 42.72 |
| State above, telemetry decimated to 1 kHz | 41.08 | 40.46 |
| Plus cached observer readiness | 42.29 / 42.32 | 41.48 / 41.48 |
| Plus integer fast paths in the angle helpers | 39.58 / 39.57 | 39.31 / 39.28 |
| Minus the Hall snapshot reuse (final) | **38.97** | **38.62** |

Final p99 bin upper edges 45.00 / 44.62 us, observed maxima 51.47 / 49.21 us,
no period-budget violations. Relative to the original `8fa1e5b` baseline
(46.35 / 45.67 us) this is about 7.2 us or 15.7 %.

Findings:

- Code alignment does not help. Aligning functions is within layout noise;
  aligning branch-target blocks grows `.text` by 8 % and costs 0.3 us.
- Native-rate recording costs 2.0 us of the ISR. With telemetry at 1 kHz the
  starting state of this section is 41.08 / 40.46 us; that, not the
  recording figure, is the normal operating cost.
- `BackEmfObserver::is_ready` was evaluated four to five times per cycle
  (about 0.95 us). It now returns a flag refreshed by every mutation of its
  inputs (`update`, `reset`, `force_phase`, `set_velocity` and the readiness
  refresh used by tests). Debug builds compare each read with the live
  evaluation, so the whole host test suite checks the cache.
- A fresh PC profile showed `angle_difference` at 40-70 cycles and
  `wrap_angle` at 30-40 cycles per call, six hot call sites each, 3.2 us
  together. The cost was not the arithmetic: every float bound is a flash
  literal load that misses the 8-line ART data cache (6-13 cycles each),
  plus one taken branch to an uncached target. Both helpers now decide the
  common case with one integer compare of the bit pattern (immediates, no
  literal) and call an out-of-line general path otherwise. A frozen copy of
  the previous bodies is compared bit for bit on every boundary value with
  neighbours and on two million random patterns. Measured gain 2.7 / 2.2 us
  against a prediction of 2.6 us from the per-site profile.
- The Hall snapshot reuse from the first section was removed at the owner's
  request for readability: `hall_embassy.rs` is back at `8fa1e5b` and
  `hall_sensor.rs` keeps only its two reciprocal edits. The first item of
  "Implemented changes" above therefore no longer applies. The profile had
  put its benefit at about 0.3 us (Hall work 4.5 -> 4.2 us), because the
  refactor split the sampler into two out-of-line functions. With the angle
  helpers fixed, removing it made the ISR 0.6-0.7 us faster.
- Before that removal 93 literal loads remained on the ISR path, 1.9 us in
  total and at most 13 cycles each (trig and arctangent coefficients,
  current-controller constants, thresholds). There is no further
  concentration comparable to the angle helpers.

Validation of the final tree: core 740 tests and 11 doctests; 56 host
library and CLI tests;
`cargo fmt --check` passes for core; strict Clippy passes for core with the
same three pre-existing lint kinds allowed as above. Release builds pass for
F405 MK5, F405 CF2 and G474. Timing was measured on the MK5 only, unloaded,
at +120 and +450 electrical rad/s; no 20 kHz run was repeated.

Board state: the normal image built by `just build-mk5-spi` from this tree
(SHA-256 `774bedced59aa55d59b27e6a3ac2f51a7f046ae20a0af585ce79cb7e479fd66e`)
is flashed. Its two production captures (+120 and +450) were accepted on the
first attempt. For the intermediate image with the Hall reuse, three attempts
had been rejected for host affirmation gaps of 150-310 ms while the host was
heavily loaded. Final readback: Stopped, zero faults, 24.0 V.

## Failsafe-path ISR logging disabled; what the long cycles are

Same artifact directory as the previous section. `tree-final.patch` is again
the complete diff against `8fa1e5b`; `cycle-classes-bench.patch` is the
bench-only recorder used below and is not part of the tree.

### Logging

The production image has 18 `warn!`/`error!` statements on the ADC ISR path
(found from the linked call graph, `isrlogs.py`). Each defmt message costs
about 15 us inside the ISR. Five are now commented out, at the owner's
request, with a dated note at each site:

- `state.rs`: "failsafe: host link lost while running", "CommTimeout
  raised", "failsafe: stopping-class fault active", and "KILL fault active"
  (which also ran before the gate cut);
- `failsafe.rs`: "stop sequence armed".

The first three plus the last could fire in one cycle. Check: the host CLI
was paused for 0.4 s during a +450 run (affirm gap 424 ms, fault generation
0 -> 2). The longest ISR body of that run, failsafe entry included, was
52.3 us; the same situation had produced 91.0 us before.

Consequences: these texts no longer appear in the device log.
`scripts/benchsuite.py` lists "CommTimeout raised" and "stop sequence armed"
in `MARK_FAULT` and will not see them; harnesses should compare the status
`fault_generation` before and after a run instead (it changes whenever a
fault is raised or cleared). Thirteen ISR log statements remain: command
rejections, over/under-voltage, over-temperature, software overcurrent,
"FOC step error", the park-brake release and three sensorless-startup
messages. Each is a single message per event.

### Long cycles

A bench-only recorder kept one body-time histogram per cycle class, flagged
by the code itself: scheduled velocity update, command drained, thermal
conversion, Hall edge since the previous cycle. 210 000 active cycles per
speed, 15 kHz, native Packed28, logs already disabled.

| Class at +120 | Share | Mean, us | p99, us | Max, us |
| --- | ---: | ---: | ---: | ---: |
| No event | 91.77 % | 39.01 | 40.42 | 43.78 |
| Velocity update | 6.56 % | 44.65 | 46.14 | 47.84 |
| Thermal conversion | 0.72 % | 39.01 | 40.42 | 41.77 |
| Hall edge | 0.72 % | 38.72 | 39.66 | 41.38 |
| Command | 0.12 % | 42.71 | 43.85 | 45.51 |
| Velocity update + command | 0.012 % | 48.65 | 51.86 | 51.80 |

At +450 the figures are the same within 0.4 us (velocity update 44.28,
command 42.42, both together 48.43, maximum 51.79 us).

- The scheduled velocity update adds 5.6 us to every fifteenth cycle and a
  drained command adds 3.7 us (about 18 per second, the host affirmation).
  Thermal conversion and Hall edges cost nothing.
- Every cycle above 48 us is a velocity update that coincided with a
  command; that happens for one command in fifteen. This population
  (0.012 %) matches the 0.016 % of bodies over 50 us seen at 20 kHz.
- Inside the velocity update the loop step is about 1.9 us and the torque
  constraints about 1.1 us. `DriverCommand` is 176 bytes and is copied
  several times when it is received, although the frequent affirmation
  needs a small fraction of that.
- Not separating the two events is the cheapest way to lower the maximum:
  without the coincidence it would be the larger of 47.8 and 45.5 us.
  Either the command drain or the velocity update can wait one cycle when
  the other is due. This changes scheduling by one cycle and was not done.

Board state: the normal image from this tree (SHA-256
`f66af7647f3b0a6b305f4655626d476b972677352b6f64f85c6458ce27d8418f`) is
flashed; two production captures accepted on the first attempt; Stopped,
zero faults. Core tests (740) and strict Clippy with the same three allowed
lint kinds pass after the logging change.
