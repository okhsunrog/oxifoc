# Performance optimization audit, 2026-11-09

Scope: the F405 MK5 control ISR and its 20-kHz telemetry pipeline, including
shared core algorithms, scheduling, device transports and host recording.
Source baseline: `0a4d3d6`. This is an investigation backlog, not a claim
that every candidate improves the shipping binary. The original audit was
read-only; implementation and device measurements are recorded below.
The inspected MK5 ELF SHA-256 is
`5308b883af4df65a05ada8ef9ff004973d499ab3e7c5ec617a1d3b5fe2c678f3`.

## Baseline and acceptance

Full persisted motor/Hall model, Hall commutation, 0.6-A Id hold:

- Minimal DWT timing: 6669 cycles / 39.7 us per 50-us cycle, about 79% CPU.
- Detailed timing: angle/estimators 1987 cycles (observer included: 782),
  controller 1092 (trigonometry included: 195), ADC reads 359, snapshot
  preparation 563, publication 502. These are overlapping/nested buckets;
  detailed instrumentation itself adds overhead.
- About 14.7k of 20k samples/s drain during the hold. The MCU raw queue
  overflows; broadcast admission failures are zero in those windows.
- Evidence and configuration: [current bench note](bench-flux-2026-11-09.md),
  `check-standard-channel.parquet.host.log` and
  `check-coherent-profile.parquet.host.log` under
  `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-current-2026-11-09/`.

Every candidate needs linked-assembly inspection where relevant, a focused
correctness check, and an isolated hardware A/B with the same persisted
model, workload and features. Measure total ISR mean/max and entry jitter,
not only a section whose work may have moved elsewhere. Confirm the gain
again without detailed profiling. Record sequence continuity, queue losses,
transport losses and command latency separately. Gapped captures cannot
qualify continuous waveform analysis. Keep 20-kHz native samples and all
independent protections. No target saving is assigned before measurement.

Boot/live MotorParams setup currently differs: live writes do not configure
all observers or the platform pole-pair mirror. Reboot after setting the
model for comparisons. Fix that through one coherent setup path before
using live writes in a benchmark harness. Derived-value caches should own
their source configuration or be rebuilt through one validated constructor,
so adding an optimization cannot leave stale coefficients behind.

Priorities: **first** = strong, local redundancy worth measuring first;
**next** = useful but smaller or more architectural; **experiment** = changes
numerical behavior, representation or scheduling enough to need extra proof.

## Hall and angle math

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| H1 | first | Share per-cycle Hall age/velocity/interpolation between `get_snapshot` (`oxifoc-f405/src/control/foc.rs:517`, `foc/hall_embassy.rs:112`) and control `sample_mut` (`foc/phase/manager/update.rs:110`). Keep separate unsmoothed telemetry and rate-limited, next-cycle control outputs. | Differential edge replay, invalid states, coast-window timing, reversals and threshold crossings; time both existing sampling sites. |
| H2 | next | Fuse staleness and stateful sample access (`foc/hall_embassy.rs:184,192`) under one estimator access, sharing edge age. Coordinate with H1; do not count the same removed work twice. | Previous observer readiness/speed must still gate staleness; stale samples must not advance the limiter. Test fallback and capture interleavings. |
| H3 | next | Move unused calibrated centroid lookup into the low-speed branch; consider caching normalized, advance-adjusted calibration angles (`foc/hall_sensor.rs:850,918,1051`). | Inspect whether LLVM already sinks the lookup. Test all raw states, calibration/advance changes and reversed wiring without shifting a live sector's interpolation base. |
| H4 | next | Avoid `angle_per_state / age` during healthy rotation; cache/test the expected sector duration and divide only when decaying late edges (`foc/hall_sensor.rs:886`). | Both directions, zero age/speed, long ages and one-ULP neighbors of the decay boundary; retain the one-sector advance bound. |
| H5 | next | Cache interpolation threshold in rad/s rather than converting to/from eRPM per sample (`foc/hall_sensor.rs:374,857,925,947`). | Preserve equality and nonfinite-setting behavior; test both signs at neighboring threshold floats. Measure the combined path, not source operation counts. |
| M1 | first | Extend `wrap_angle`'s fast interval to `[-TAU, 2*TAU)` (`oxifoc-core/src/lib.rs:228`); ordinary positive crossings currently enter `%`. | Exact `2*TAU` stays cold. Compare old/new bits over all 8388608 floats in `[TAU,2*TAU)`, boundaries, signed zero and large/nonfinite inputs. Preserve existing tiny-negative rounding behavior. |
| M2 | experiment | Share normalization/quadrant reduction for the FastSinCos pair (`foc/trig.rs:86,118`), retaining a cold fallback. | Shifted-angle rounding changes: enforce an explicit accuracy bound, quadrant-neighbor and Park round-trip tests. Measure the pair against its 195-cycle detailed baseline. |

Unqualified `foc/`, `motor/`, `runtime/` paths and filenames in the tables
are relative to `oxifoc-core/src/`.

## Observer and current controller

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| O1 | next | Cache timestep-derived filter coefficients across unchanged `dt` (`foc/phase/observer.rs:698,745,750,779,997`). Refresh on actual timestep changes; torque input may use its own timestep. | Compare complete trajectories with alternating 50/100-us timesteps, zero/invalid intervals and coefficient saturation. Reject if cache loads/branches exceed the saved work. |
| O2 | next | Cancel the differentiation division algebraically in the acceleration filter when `dt <= tau` (`foc/phase/observer.rs:745`), keeping the saturated branch. | Changed evaluation order must not disturb readiness acquisition/revocation. Test acceleration, braking, reversal, seed/reset and timesteps around 20 ms. Depends on the O1 coefficient design. |
| O3 | next | Cache a guarded PLL `ki` reciprocal for readiness compensation (`foc/phase/observer.rs:880,967`). | Preserve nonpositive-gain behavior and handle reciprocal overflow. Compare gain changes, compensation cap and readiness-boundary decisions. |
| C1 | first | Simplify final-PWM q-axis reconstruction (`foc/pwm.rs:97`, `foc/controller.rs:797`): algebra removes `/3`, and a guarded cached period reciprocal can remove `/period`. Both divisions survive in the current ELF. | Keep the typed constructor consuming final, clamped duties. Compare all sectors, current signs, common modes and supply-limit boundaries; retain the deadtime/sign-reversal regression. |
| C2 | first | Cache the effective modulation factor and integer PWM cap when their configuration/period changes (`foc/controller.rs:388,394`, `motor/foc_driver/driver.rs:47`). Current linked code still recalculates them. | Exercise every setter and invalid setting; preserve cap rounding, d-axis priority, anti-windup and final duties. Continue using fresh bus voltage each cycle. |

## ADC and ISR handoff

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| A1 | next | Transfer the three installed ADC handles to permanent ISR ownership (`oxifoc-f405/src/control/foc.rs:164,423,444,466`), avoiding per-cycle CS/RefCell/Option accesses. Inspect fixed-rank specialization separately. | Retain nine reads and flag clearing; never drop a HAL handle that stops triggering. Compare raw frames and 20-kHz JEOC cadence. The entire measured read segment is only 359 cycles. |
| A2 | first | Add a no-listener load fast path before `CycleNotification::wake`'s atomic exchange (`oxifoc-core/src/state.rs:174,1337`). | Retain the armed exchange and register-before-sequence-recheck protocol. Test publication/registration races and cancellation, both idle and detection-active. |
| A3 | next | Cache exact immutable PWM ARR for the pre-update snapshot (`oxifoc-f405/src/control/foc.rs:498`, `motor.rs:79`). | Compare snapshots through initialization and reconfiguration; preserve the actual register value rather than silently substituting a differently defined maximum duty. |
| A4 | experiment | Shadow previously written, clamped CCR values to avoid three APB reads, through every PWM writer (`oxifoc-f405/src/motor.rs:128,159,175,191`). | Temporary dual readback must agree through clamp/calibration/enable/disable transitions. Keep hardware MOE live; revise the register-value telemetry contract explicitly if needed. Separate from A3. |
| A5 | next | Pass/latch the current triplet directly into the driver instead of storing then reloading its atomics in the same ISR (`oxifoc-f405/src/control/foc.rs:432,453,475`, `foc/sensors.rs:475`). | Independent calibration consumers still need publication. Preserve identical raw/offset/calibrated triplets; require a measurable gain before widening the sensor API. |

## State, protections and diagnostic bookkeeping

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| S1 | first | Remove or feature-gate the unread `STALENESS_MAX_US.fetch_max` performed every active cycle (`oxifoc-core/src/state.rs:927`, `runtime/streaming.rs:128`). The current ELF retains the exclusive-update loop. | Keep staleness calculation and deadman checks unchanged. Run existing deadman/failsafe cases, then compare command-glue and whole ISR costs. |
| S2 | first | Skip idle excitation CS/borrows when no packet is installed (`oxifoc-core/src/state.rs:1041,1176`), using packet presence captured in the existing state snapshot at line 871. | Gate on packet presence, not just DETECTION_ACTIVE: cancelled/preparing packets still need cleanup. Test next-cycle arming, autonomous finish, cancellation and current-budget abort. Protections remain outside the gate. |
| S3 | first | Prepare protection voltage thresholds, budgets and decay from immutable board configuration and dt (`motor/foc_driver/protection.rs:322`). The existing ELF retains their divisions/conversions. | Preserve exact OV/UV trip cycles, decay/hysteresis and invalid-temperature behavior at several dt values. Rebuild prepared constants whenever their inputs change. |

`runtime/` paths above and below are relative to `oxifoc-core/src/`.

## Telemetry encoding and draining

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| T1 | first | Encode into reserved batch storage instead of a zeroed 35-byte scratch followed by append-copy (`oxifoc-core/src/fast_telemetry_codec.rs:86,105`). Both clearing and copying survive in the ELF. Keep the wire format. | Preserve full-frame reservation, dense fallback and transactional refused-push behavior. Compare exact encoded bytes for arbitrary frames and capacity edges; avoid replacing memcpy with costly per-byte checked pushes. |
| T2 | next | Fuse history update with the existing word-comparison pass (`fast_telemetry_codec.rs:18,123`), removing a second 34-byte read/copy. Coordinate with T1 and benchmark separately. | Dense fallback can stop the comparison early: all remaining history must still update. Failed pushes leave history unchanged. Test dense→sparse and refused→valid sequences. Scattered stores may be slower than memcpy. |
| T3 | experiment | Pack the delta/full discriminant into the 34-bit mask, saving one byte per delta (`fast_telemetry_codec.rs:148`, `types.rs:429`). | Explicit ICD/host migration and malformed-batch tests. Maximum saving is less than 20 kB/s at 20 kHz; CPU benefit is unknown. Replay recorded frames before considering a format change. |
| T4 | first | Aggregate consumer READ_OK/READ_BADLEN counters per drain slice instead of an atomic RMW per sample (`runtime/streaming.rs:455`). | Flush before every break/yield/await. On F405 the reporter shares the executor, preserving report-window attribution. Test malformed, partial, empty and retry paths; retain ISR-side atomic counters. |
| T5 | first | Count encoding time on partial-batch polls (`runtime/streaming.rs:447,481,496`); those currently exit before the timing update. This improves attribution, not throughput directly. | Partial→full, empty and failed-broadcast paths must account each encoding slice once. Define whether maximum duration describes a slice or a coalesced batch; wall time includes preemption. |

## USB, RTT and scheduling

Ergot paths below refer to the sibling `/home/okhsunrog/code/rust/ergot`
checkout, currently matching the pinned `3c502f4`.

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| U1 | next | Recycle completed nusb receive buffers with `RequestBuffer::reuse` instead of allocating per completion (`ergot/.../transports/nusb.rs:151,225`). This reduces host overhead. | Preserve depth 32, synchronous processing lifetime, empty completions, error drain and reconnect. Count allocations and CPU over 0/1/63/64/65/511/512-byte completions, then repeat an unchanged capture. |
| U2 | experiment | Recycle the host TX payload allocation (`ergot/.../transports/nusb.rs:344,350`), using the completed payload buffer rather than `frame.to_vec` each time. | Payload and ZLP buffers must stay distinct; the copy from bbqueue remains. Count allocations with aligned/nonaligned frames. Low priority: this is command downlink, not telemetry uplink. |
| R1 | first | Add a configurable real wait policy for zero-progress ergot RTT reads/writes (`ergot/.../transport/rtt.rs:94,142`), then choose worker priorities and bound successful work per activation. `yield_now` alone keeps a task ready. | Lower-priority sentinel, cancellation, empty/full rings, ordered resumption and handshake under streaming. Preserve default API compatibility; separate RX/TX policy state. No private duplicate app reader. |
| R2 | first | Make the host RTT pump fair/cancellable on zero/partial down writes and blocked RX delivery (`oxifoc-host-lib/src/transport/rtt.rs:499,521`). Retain pending bytes/offsets and budget work before servicing other directions/shutdown. | Fake stalled channels must still allow up reads, commands and bounded shutdown; preserve byte ordering. Check timeout/join ownership and close/reopen. |
| R3 | next | Split SWD-read time from async-delivery wait, and measure current-board drain gaps, read sizes and ring high-water marks (`oxifoc-host-lib/src/transport/rtt.rs:489`). | Qualify current 5/10/20-kHz captures and command latency after R1/R2. Historical G474 throughput is not current F405 acceptance; larger rings only absorb bursts. |
| E1 | first | Measure exclusive USB CPU using bench-only begin/end events for USART1 executor and OTG_FS (`oxifoc-f405/src/main.rs:34`, `transport/mod.rs:113`). Current endpoint-write timing is elapsed await time. | Reuse finite RAM/TIM5 trace, preserving Embassy bindings. Offline nested interval union must exclude ADC/other IRQs and count preempting USB only once. Test wrap, reordered records and incomplete windows; compare with recording off. |

The nusb paths abbreviated above are under
`ergot/crates/ergot/src/interface_manager/`; the RTT adapter is
`ergot/crates/ergot/src/transport/rtt.rs`.

## Host recording

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| P1 | next | Export Parquet in bounded row groups instead of constructing every column and rich sample for the whole capture (`oxifoc-host-cli/src/record.rs:415,435,490`). The current 32 column arrays cost 212 bytes/sample, in addition to raw/rich samples. | Compare every column/metadata across group-boundary gaps, sequence wrap, ADC/PWM sentinels and maneuver anchors. Measure peak RSS/export time. Raw capture storage remains O(N); export runs after Stop and cannot explain MCU loss. |
| P2 | next | Benchmark decoded-batch channel handoff versus per-sample `try_send`/`recv_timeout` (`oxifoc-host-lib/src/lib.rs:867,879`, `oxifoc-host-cli/src/record.rs:175`) before changing the API. | Replay sparse/dense batches at 20 kHz and faster, with consumer pauses. Preserve sample capacity/drop accounting and ACK/sequence-anchor latency; batch drops change loss granularity. |

## Memory and build

Current ELF: `.data` 8612 B, `.bss` 42452 B, nominal SRAM stack 80000 B
(not a measured high-water mark), no CCM section. Stack relocation is not
motivated by demonstrated pressure.

| ID | Priority | Small task and source | Required proof |
| --- | --- | --- | --- |
| B1 | experiment | Relocate only the CPU-owned 4116-byte FAST_TELEM_Q into CCM (`runtime/streaming.rs:39,330,455`). This queue was not included in the failed STATE/driver CCM experiment. | Proper startup initialization, CPU-only ownership and section-address checks; keep DMA buffers, STATE, driver and stack in SRAM. Compare publication/consumer costs and total throughput, with telemetry-off as a negative control. Check linked layout as a confounder. |
| B2 | experiment | Compare linked hot code with release incremental disabled (`oxifoc-f405/Cargo.toml:138`), changing no other setting. | Inspect instruction layout, call boundaries and frames first. Identical hot code needs no hardware test. If different, same-workload A/B is required; smaller size/frame is not proof of speed. Historical results used another source/toolchain. |

## Parallel execution

Thirteen independent read-only audits produced these 35 bounded tasks.
Each ID can become a separate implementation assignment and commit.
Use separate worktrees under the agreed temporary-work root and explicit
file ownership. A task changes only its assigned area; common interface
changes go through the integrator. Several IDs touch the same file and
must not be merged or benchmarked together merely because they look small.

Suggested first batches, at most three implementation workers plus the
integrator:

| Batch | Independent assignments | Integration dependency |
| --- | --- | --- |
| 0 | E1: exclusive USB CPU attribution; T5: complete encode timing | Diagnostic baseline, kept out of normal firmware. Existing minimal-versus-detailed ISR comparison is already done. |
| 1 | M1: wrap fast path; S1: unused margin atomic; T4: drain-slice counters | Separate files; integrate and measure each independently. T4 follows T5 in the shared streaming file. |
| 2 | H1: shared Hall evaluation; O1: observer coefficients; C1: final-PWM reconstruction | Hall outputs retain distinct telemetry/control timing. Other Hall tasks follow H1's interface; O2 follows O1. |
| 3 | A1: ADC ownership; T1: encoder storage; U1: host USB buffer reuse | ADC setup remains platform-owned. T2 follows T1. U1 is a separate ergot change/pin update. |
| 4 | R1: ergot RTT wait policy; R2: fair host RTT pump; P1: bounded Parquet export | R3 qualifies RTT only after the two fairness changes. |

The remaining IDs enter the same queue by priority. C2 follows C1 in the
controller area; A2 and S2/S3 share state/driver interfaces and should be
serialized; A3 precedes A4. B1 conflicts with telemetry storage changes.

## First implementation batch

Integrated: S1, S2, S3, M1, C1, C2, O1, A2, T1 and T4. T5 fixes profiling
attribution only. Focused differential/concurrency tests and linked F405
assembly checks accompany the changes. Two experiments were reverted:
H1/H2 increased whole-ISR cost by 33 cycles after the wrap fix; A1 showed
no whole-ISR gain (6149 versus 6144 cycles).

Same persisted model, Hall source and 0.6-A Id hold, minimal timing:

| Cumulative variant | Mean ISR cycles | Raw queue drops/s during hold |
| --- | ---: | ---: |
| Baseline | 6668 | 5299 / 5298 |
| S1 + T4 + S2 + C1 | 6529 | 4129 / 4143 |
| Cold wrap fallback, Hall cache removed | 6360 | 2525 / 2528 |
| T1 | 6382 | 707 / 685 |
| S3 | 6331 | 73 / 144 |
| A2 | 6264 | 0 / 0 |
| O1 | 6193 | 0 / 0 |
| C2 | 6144 | 0 / 0 |

These cumulative measurements include layout effects; their differences
are not independent per-function savings. The resulting mean is about
36.6 us versus 39.7 us. T1 mainly frees foreground drain capacity, rather
than reducing ISR time. A 20-second hold plus q-axis checks completed all
107 maneuver commands with zero capture gaps/losses. A separate normal
build without ISR timing also recorded 438148 rows without loss.

DWT can be disabled by debugger detach. The one-second reporter now
re-enables its clock/counter without resetting it, discards the affected
window and starts coherent timing again. A real SWD attach/detach during
20-kHz recording verified recovery; zero-cycle windows are invalid data.

E1 adds finite, bench-only USB IRQ/executor spans, validated in record
order with incomplete edges explicitly excluded. One qualified 27.35-ms
window measured USB spans minus ADC at 8.68%, ADC at about 75.3%. Other
IRQs and trace overhead remain included; untraced time is not necessarily
idle. This is an instrumented window, not an entire-run CPU average.
The full trace build itself lost telemetry samples and cannot qualify
continuous waveform analysis.

Calibration qualification exposed a PWM timing dependency: the original
binary accepted HFI Ld/Lq = 19.64/28.17 uH with the current persisted model;
the optimized binary rejected HFI and fell back to implausible 5–7 uH.
Those results were not applied. TIM1 readback confirmed RCR=0, so CCR
preloads transfer at both turns. A minimal before/after-CCR trace recorded
2548 down/down, 225 up/up and 43 down/up writes in 2816 events: duties can
land on either side of the valley, including a transfer between CCR stores.
Peak-only full-period updates (RCR=1, deliberately primed after UG) fixed
the HFI result: 18.65/28.33 uH, 80086 recorded rows, zero gaps/losses.
Three boot-only OPM freezes independently proved CNT=4195, DIR=down,
UIF=1, CEN=0 at each permitted update. The F405 also explicitly configures
the driver's nominal sample-to-actuation delay to one period, keeping
observer voltage pairing and output-vector advance consistent with the
timer. Near-peak ADC aperture offset is not a measured delay calibration.
One of 2816 diagnostic writes still crossed the peak itself. A second
boot-only test armed UDIS after the valley, suppressed only the next peak,
then released it mid-down. The following three OPM freezes all remained
exactly at CNT=4195/down. Thus UDIS can protect the three CCR writes
without moving this RCR=1 update phase. The ordinary build with that guard
still needs its final L/current repeat. This guarantees coherent duties;
CCER topology/enable changes remain immediate, and late batches can retain
the old vector for another period.

Next scheduling investigation: `NetStack<CriticalSectionRawMutex>::send_ty`
holds PRIMASK across router/local-socket traversal and frame serialization.
It therefore masks P0 ADC too. Its duration has not been measured. HFI
waveform generation is outside CS and its array transfers are already
split into 8/16-byte pieces; no whole-packet CS copy was found.

Artifacts, exact ELF hashes, logs and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-perf-2026-11-09/`.
Useful entry points: `results.json`, `performance-stages.png`,
`usb-cpu-timeline.png`, `pwm-latch.parquet.trace.bin.pwm.png`.
P2 and U2 are lower priority than the MCU drain deficit.

Only the integrator owns the physical board, flashing and captures. Pause
concurrent builds during timing-sensitive runs. Keep the exact matching
ELF for decoding; do not rebuild it mid-capture. Compare one change at a
time, then repeat the integrated image to detect interactions. Revert a
measured regression rather than accumulating speculative optimizations.

## Already used, rejected, or not yet justified

- ART prefetch/instruction/data caches and scalar float32 FPU are enabled;
  release already uses opt-level 3, fat LTO and one codegen unit.
- Hardware sqrt, approximate atan2, cached current/phase ADC scales,
  cached Hall tick rate, mixed-convention angle_difference fast path,
  native-word delta comparisons and bulk-byte serialization are present.
- Several controller/transform source-level repeats are already removed
  by LLVM: disabled-deadtime inverse Park, shared polarity branch,
  actuation square, constant halves and common subexpressions. Moving
  the vbus division into its setter only moves cost: vbus updates every ISR.
- The custom empty-command queue saved only 55 cycles and was removed.
  Whole STATE/driver CCM placement, forced wrapper inlining, target-cpu
  selection and frame-pointer experiments did not improve prior checks.
- Shadow-observer multirate work reduced loss but still failed continuous
  capture and was reverted. Revisit only with explicit estimator timing,
  fallback/readiness and transition validation; no rate change is assumed.
- Bigger queues cannot cure sustained producer/consumer imbalance. The
  current backlogged drain already yields between full batches without
  sleeping each time; shorter timers/per-sample wakeups lack evidence.
- Device USB RX/TX adapters already avoid a gratuitous whole-frame copy.
  Preserve FS packet boundaries/ZLPs; no source basis for larger packets.
- Host batch decoding uses heapless storage, not a heap allocation per
  batch. Recording enriches each sample once after Stop.
- No new priority/EDF policy, wholesale fixed-point rewrite, stack move,
  or hot-code relocation is justified by the current measurements. F405
  CCM is for CPU data, not DMA buffers or executable code; see ST
  [RM0090, system architecture / D-bus](https://www.st.com/resource/en/reference_manual/dm00031020.pdf).
