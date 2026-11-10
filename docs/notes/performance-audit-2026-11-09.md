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

At the audit baseline, live MotorParams writes did not configure all
observers or the platform pole-pair mirror. Boot and live writes now share
the prepared-model installer qualified below. Derived-value caches should
own their source configuration or be rebuilt through one validated
constructor, so an optimization cannot leave stale coefficients behind.

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

Audit-baseline ELF: `.data` 8612 B, `.bss` 42452 B, nominal SRAM stack 80000 B
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
| Peak-only PWM + coherent CCR writes | 6185 | 0 / 0 |

These cumulative measurements include layout effects; their differences
are not independent per-function savings. The final mean is about 36.9 us
versus 39.7 us (7.2% fewer cycles), including the coherent-write guard.
T1 mainly frees foreground drain capacity, rather than reducing ISR time.
The final build without ISR timing recorded 438171 rows at native
20004.27 Hz: a 20-second hold plus q-axis checks, all 107 commands successful,
zero gaps/losses, terminal Stop and fault count zero. The sampled dq peak
was 1.93 A; a middle hold window measured Id=0.59975 A, Iq=0.00024 A.
Hall errors/overcaptures were zero. This is a low-current bench workload.

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
without moving this RCR=1 update phase. The guard guarantees coherent duties;
CCER topology/enable changes remain immediate, and late batches can retain
the old vector for another period. The guard and configured actuation delay
then passed the normal build's final tests: HFI Ld/Lq=19.59/28.48 uH and
19.45/28.17 uH on repeat; R=30.72 mOhm. All three captures had zero loss,
and all detection results remained unapplied. Configuration readback
matched the initial model, Hall table, limits and voltage calibration;
only the expected runtime current offsets changed at boot.

E2 measured `NetStack<CriticalSectionRawMutex>::send_ty`: its recorded
PRIMASK-protected callback body reached 23.36 us across socket/router
traversal and serialization; acquisition/release tails are not timed.
In a matched finite trace, no ADC ran inside its 77 qualified lock bodies.
The raw ADC entry phase from the preceding PWM peak had p99=24.57 us,
maximum=27.43 us. These are phase snapshots after the ISR prologue, not
absolute JEOC latency or a whole-run worst-case bound.

F405 now selects an app-local BASEPRI P6 mutex through Ergot's existing
`NetStack<R>` API. Thread tasks and the USART1/P6 USB executor are its only
permitted callers; hardware USB/UART handlers only wake those tasks.
ADC P0 and Hall P1 can preempt. A per-instance atomic prevents reentry;
RAII releases it before restoring the inherited BASEPRI. Runtime checks
reject unsupported contexts or priority grouping. The USB executor and
mutex share one typed priority constant. Other global critical sections,
Ergot itself and G474 are unchanged.

With identical instrumentation, ADC preempted 45 of 79 qualified network
lock bodies. Raw phase p99 fell to 5.13 us, maximum to 6.17 us; a repeat
gave 5.08/6.20 us. Median remained 4.98 us. ADC append intervals changed
from p99/max=69.59/72.45 us to 50.15/51.17 us. Each trace covers about
44 ms. The longest new lock elapsed span was 144 us because ADC preempted
it repeatedly; subtracting recorded ADC 8/9 spans leaves at most 29.74 us.
That remainder still includes entry probes, untraced IRQs and trace work;
release work after the end marker is excluded. It is not exclusive
serialization time. The change removes long global masking, rather than
making serialization faster.

Both traced images passed all six boot mask/reentry checks on the board;
SWD readback confirmed exact mask restoration. Trace builds lost samples
and one baseline maneuver rejected a late command, then stopped safely.
They qualify the finite timing observations only, not continuous waveform
analysis. ADC trace probes now run only during the recording window.
`build-mk5-netstack-trace [basepri|primask]` and
`profile-netstack-mk5 CAPTURE [basepri|primask]` reproduce the comparison.
HFI waveform generation is outside CS and its array transfers are already
split into 8/16-byte pieces; no whole-packet CS copy was found.

Minimal whole-ISR timing with the new policy measured 6206/6207 cycles in
the steady hold, about 37.0 us versus the preceding 6185-cycle build.
That capture lost four samples and is not a continuous waveform reference.
Final qualification used the normal build without trace/ISR timing:
438550 rows, zero gaps/losses, all 107 commands successful, terminal Stop,
fault count zero, Hall errors/overcaptures zero, sampled dq peak 2.15 A.
A middle hold window measured Id=0.60022 A, Iq=0.00026 A. R=30.160 mOhm
and HFI Ld/Lq=19.547/28.130 uH were also recorded without any loss and
remained unapplied. Model, Hall table, limits and board calibration matched
the initial readback; only runtime DC offsets changed. The board is left
Stopped on the normal image, with a matching ELF and no trace symbols.

Artifacts, exact ELF hashes, logs and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-perf-2026-11-09/`.
Useful entry points: `results.json`, `performance-stages.png`,
`usb-cpu-timeline.png`, `pwm-latch.parquet.trace.bin.pwm.png`.
P2 and U2 are lower priority than the MCU drain deficit.

E2 artifacts, firmware hashes, boot witnesses and `netstack-comparison.png`:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-netstack-latency-2026-11-09/`.

## Coherent model application and second candidate batch

`PreparedMotorModel` constructs PI/limits, estimators and pole-pair state
outside the ISR. Boot and live writes consume it through one stopped-only
installer. A generation-tagged, single-flight mailbox handles cancellation
and stale commands without copying the prepared payload into the command
queue. Applied configuration/revision and telemetry pole pairs are committed
before the acknowledgement. Partial models clear unavailable estimators and
fall back to Manual; replacement restores them without rebooting.

The hardware check exercises full/partial/full models, rejected unavailable
sources and pole-pair replacement/restoration with PWM disabled throughout.
Its timing build recorded 130776 samples without loss; the largest observed
whole ISR was 6336 cycles, with zero overruns. This includes the apply work,
not an isolated apply duration. Software checks cover races, cancellation,
queued starts, reset and feature variants. The final host suite passed 537
tests (two existing ignores); strict core/virtual/host-lib Clippy and both
device release checks passed.

Same model, Hall source, 0.6-A Id-only hold, minimal DWT instrumentation:

| Variant | Mean ISR cycles | Recorded samples lost | Decision |
| --- | ---: | ---: | --- |
| Coherent-model baseline | 6309–6310 | 82 | Diagnostic timing reference |
| H3: sink unused centroid work | 6273 | 0 | Keep |
| H3 + H5: cache limiter minimum speed | 6251; repeat 6250 | 0 | Keep |
| H3 + H4: cache sector decay duration | 6282 | 0 | Reject H4 |
| H3 + H5 + O2: acceleration algebra | 6283 | 0 | Reject O2 |
| H3 + H5 + B1: queue alone in CCM | 6290 | 0 | Reject B1 |

H4 plus H5 measured 6247 cycles, only four below H5 alone; the extra cache
was removed. Selected changes save about 60 cycles / 0.35 us (0.9%);
these whole-image measurements include layout effects. Queue CCM startup
and placement were verified (4116 B at 0x10000000, STATE/driver in SRAM),
but the feature was removed after the regression. T2 was rejected before
MCU integration: pinned-host ABBA medians regressed 8.3% for mixed and 9.4%
for dense frames. Exact-wire/history tests across four alignments remain.

Final detailed profiling measured 6738–6739 whole-ISR cycles: estimator
update 1781 (observer included: 657), controller 1074, command glue 494,
ADC 350, snapshot 501 and publication 586. Buckets overlap. Instrumentation
lost 13341 samples; this capture qualifies timing only. The repeat with
minimal timing measured 6250 cycles / 37.24 us and no overruns.

Qualification then used the normal image without ISR timing/profiling:
418159 native 20004.27-Hz samples, all 101 commands successful, zero gaps
or lost samples, sampled dq peak 1.74 A. The middle hold averaged
Id=0.59995 A, Iq=-0.00012 A; Hall errors/overcaptures were zero. Separate
unapplied R=30.458 mOhm and HFI Ld/Lq=18.391/27.788 uH captures also had
zero loss. The live-model check repeated successfully with 126648 samples,
zero loss and exact configuration-value restoration. No flash writes were
performed. The matching normal ELF remains installed, Stopped/Hall, fault
count zero. Use `bench-current-mk5` for the already-built exact image and
`test-live-model-mk5` for the model replacement check.

Rotation is not qualified by the stationary benchmark. A velocity pilot
before the model fix and a later q-current test both hit the unchanged
6-A dq trip. Recorded bursts align with low-speed Hall centroid snaps and
speed/feedforward steps: in one contiguous interval lambda*delta_omega
predicts 0.585 V versus the observed 0.584-V q-voltage step. OxiFOC bypasses
the angle limiter below the interpolation threshold and uses unfiltered Hall
speed for feedforward; local VESC 7.00 limits angle at all speeds and filters
its corrected speed. This identifies a bounded follow-up investigation,
not proof of independently measured rotor speed or a reason to raise trips.
Hall calibration entries are state centroids, not measured transition edges.

This batch's captures, hashes, validation commands and plots are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-model-apply-perf-2026-11-09/`:
`performance-final.png`, `final-hold.parquet.png`, `software-final/` and
`hall-current-bursts-detail.png` are the useful entry points.

Only the integrator owns the physical board, flashing and captures. Pause
concurrent builds during timing-sensitive runs. Keep the exact matching
ELF for decoding; do not rebuild it mid-capture. Compare one change at a
time, then repeat the integrated image to detect interactions. Revert a
measured regression rather than accumulating speculative optimizations.

## Hall rotation follow-up

Rotation exposed separate problems from the stationary timing benchmark:
low-speed Hall angle steps, speed-dependent voltage feedforward enabled
implicitly by installing motor parameters, uneven calibration centroids,
and insufficient telemetry-drain time while the Hall estimate is moving.
Angle slew now applies at every speed; a reference-epoch-aware derivative
filters control velocity independently of raw edge diagnostics. Voltage
feedforward is an explicit, independently stored policy, disabled by default
(also disabled in the captured VESC 7.00 reference configuration).

The Hall calibration now groups forward sweeps before reverse sweeps.
With the configured 3-A ceiling it recorded 296358 native samples without
loss: forward/reverse Id was 2.99999/2.99998 A, with about 0.081-A standard
deviation. Adjacent centroids changed from 26.6–82.2 to 51.0–66.9 degrees.
Both sweep order and calibration current changed; their effects are not
isolated. The new table is RAM-only; `set-hall-usb` restores explicit JSON
arrays after flashing and requires `flash` to persist them.

The subsequent 1.2-A q-current staircase completed without a fault, but
lost 148 samples at the MCU producer queue. Its observed peak was 3.713 A,
compared with 5.886 A in an earlier run; motion differed and gaps prevent
claiming either a continuous peak bound or a controlled A/B improvement.
Native 20-kHz rotation capture remains an acceptance requirement.

The first combined control/raw-Hall handoff was rejected and reverted.
Two complete stationary 0.6-A Id windows in each exact timing ELF measured
6281 cycles with the handoff versus 6241 without it, both lossless. Moving
windows measured 6619 versus 6527 cycles, with different trajectories;
these are supporting diagnostics, not a controlled moving A/B. Avoid
assuming fewer calculations outweigh an owned snapshot's copying costs.

Calibration PI overrides are now driver-owned and temporary across the
OpenLoop/DirectVoltage sequence. Stop, ordinary control, faults and failsafe
restore the prior d/q tuning; persistent tuning supersedes the override.
Also distinguish a configuration readback from live Hall application:
older images applied direct Hall config writes only at boot. The
`scoped-pi-motion-timing` capture used the old live table despite a staged
new table in RAM config, hit the 6-A trip, and is excluded from table A/Bs.
The subsequent physical calibration and `actual-new-hall-motion-timing`
used the actual new estimator table and completed without a fault.

Artifacts, exact ELFs, provenance, plots and raw Hall-edge evidence:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-rotation-2026-11-09/`.

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

## ST-Link V3 read throughput (2026-11-10)

`just bench-probe-read --elf PATH` measures adjacent idle/read/idle phases
within a probe-rs Session. The measured loop reads existing SRAM with aligned
`read_32` and does not write test data. A later audit found that Session setup
and Drop both clear hardware breakpoints through a temporary halt/run. The
read-phase throughput/progress results below exclude that lifecycle cost;
they do not establish non-halting attachment or cleanup. Use this legacy
Session-based benchmark only with the bridge stopped.
The installed probe negotiates USB High Speed. With probe-rs 0.32, requested
4/12/24-MHz SWD selects actual 3.3/8/24 MHz. Sustained 4-KiB reads delivered
283/477/639 kB/s respectively; 24 MHz with 6-KiB blocks reached 645 kB/s.
These are raw SRAM results, before RTT cursor/protocol overhead. The current
34-byte record requires 680 kB/s at nominal 20 kHz, exceeding this measured
path. A different backend or protocol optimization would require remeasurement.

CurrentControl with zero Id/Iq, concurrent native-20-kHz USB recording and
24-MHz/6-KiB SWD reads produced 361889 samples without gaps or loss; all 86
commands and terminal Stop succeeded. Same-session idle/read ISR means were
6328/6328.5 cycles (about 37.70 us); the read-phase observed maximum was
7987 cycles (47.59 us), with no 50-us budget overruns. This is not a WCET proof.
TIM5 was not debug-frozen; bracketed TIM5/ADC-sequence and TIM5/DWT checks
resolved no aggregate deficit, but endpoint uncertainty still allows about
11.4 samples over the 3-s read phase. Gap-free telemetry alone cannot prove
that every hardware ADC trigger was serviced. Existing Ergot-over-RTT
connection resets/halts during setup; the benchmark bypasses that transport,
but did not bypass the underlying Session breakpoint-cleanup halt.

Exact ELFs, case JSON, timing logs, capture and summary plot are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-stlink-read-2026-11-10/`.
The normal image was restored, Stopped/Hall with zero faults, and the complete
measured Hall reference reapplied in RAM. No calibration was persisted.

A follow-up USBmon trace (64-byte headers only, no USB payload recorded)
measured 9.33 ms waiting for each 6144-byte data response out of 9.56 ms per
read. OUT commands and status responses each took about 20 us; host gaps
averaged 43 us. Reusing nusb endpoints in an isolated probe-rs copy did not
improve throughput: two uncached/cached pairs averaged 636.2/636.0 kB/s.
That experimental patch is not included in the project. The 6144-byte native
limit also matches [OpenOCD's ST-Link internal buffer limit](https://github.com/openocd-org/openocd/blob/master/src/jtag/drivers/stlink_usb.c#L74-L100);
larger caller buffers merely split into more commands. These observations
bound host-side overhead, without identifying the probe's internal bottleneck.

## Dedicated raw telemetry experiment (2026-11-10)

Optional `bench-raw-usb` / `bench-raw-rtt` images retain USB Ergot commands
and replace only the fast-data consumer. `just build-mk5-raw usb|rtt
normal|timing` selects the image. `just record-raw-mk5 SOURCE ENCODING OUT`
and `just test-raw-current-mk5 SOURCE ENCODING OUT MANEUVER` select the host
path; SOURCE is `usb`, `rtt`, or `ergot`, ENCODING is `raw34`, `packed26`,
or `packed26-delta`. Ergot raw blocks use a borrowed byte slice; the old
per-word telemetry topic remains available in standard images.
The RTT reader uses public debug-port/MEM-AP APIs without creating a Session,
then closes/detaches the owned probe normally. It validates the running M4/F405,
exact ELF's channel2 descriptor and SRAM bounds, and uses aligned SWD reads.
This avoids Session's hidden breakpoint-cleanup halt at both attach and Drop.
USB uses a separate interface1/EP2 IN, keeping
the Ergot interface0 unchanged. F405's existing FS peripheral still uses CPU
FIFO writes; this does not add USB DMA.

Both transports use the same lossless codec: 64 records and a 32-byte block
header. Packed26 retains all six 12-bit ADC readings, existing scalar
quantization and full CCR values; sequence stride and ARR move into the
header. Out-of-range input explicitly falls back to Raw34. At nominal 20 kHz
this requires 530/690 kB/s including full-block headers, respectively.
Arm/Stop/Status have fenced session tokens; END certifies record totals.
Stop disables the producer before I/O completes. Captures reject missing END,
block/count gaps, device/host drops and disconnects; recovery starts a new
explicit session. There is one queue consumer and no transport I/O in ADC ISR.

Experiment artifacts and exact binaries:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-raw-stream-2026-11-10/`.

Native-rate HIL results (actual 20004.272 Hz, unchanged motor model/limits):

| Path | CurrentControl, zero reference | Moving current staircase |
| --- | --- | --- |
| USB Packed26, normal | 362257 records / 86 commands | 80156 records / 20 commands |
| RTT Packed26, MEM-AP, normal | 362257 records / 86 commands | 80165 records / 20 commands |
| RTT Packed26, MEM-AP, timing | 362257 records / 86 commands | 80160 records / 20 commands |

These runs consumed matching END totals, had zero producer/host drops or seq
gaps, and acknowledged terminal Stop. Both transports recovered a fresh
lossless capture after forcibly terminating their reader, without MCU reset.
The direct RTT Current0 timing run observed a 6369-cycle interior ISR mean,
8028-cycle maximum (47.83 us) and zero overruns; its largest timeline ACK was
1.843 ms. Its captured DWT/TIM5 windows, including post-detach USB monitoring,
showed no halt deficit. These sampled maxima are not WCET guarantees.
Raw34 remained insufficient: USB CurrentControl lost 6538 samples in the
normal image (6909 with timing); RTT lost 6153 even while Stopped. The latter
is a sustained throughput deficit, not a buffer-size remedy. Packed26 kept
the existing 16-KiB RTT ring; measured reader statistics are in the host logs.

Earlier failures are retained: an old Session-based RTT run lost 546 samples
after about 14 seconds; its initial loss remains unattributed. Its subsequent
roughly 39-ms CPU deficit indicated by USB-SOF/DWT and ADC counts agrees with
the now-removed Session cleanup halt; no TIM5 window was retained for that event.
Separate USB runs missed the host maneuver's 5-ms scheduling tolerance while
retaining complete telemetry; their maneuver results remain failed. Successful
repeats do not erase these failures or establish a general desktop latency bound.

The accepted normal Ergot image was restored, with full Hall geometry reapplied
in RAM and reference model/limits/calibration read back equal. A final Stopped
20-kHz Ergot capture had 80040 records and zero loss; final status was
Stopped/Hall with zero faults. No calibration was written to flash.

## VelocityControl and carrier qualification (2026-11-10)

The preceding CurrentControl successes did **not** qualify VelocityControl.
The original moving-velocity failure was reproduced with both legacy Ergot
and direct USB. The outer velocity/position controller now runs on a 1-ms
schedule; the current loop, command timeout and protections remain native
20 kHz. Held output is invalidated on source/trust/mode transitions and
limited again each native cycle. This fixed stationary streaming in tested
normal images, but did not establish moving-velocity qualification.

Artifacts, exact ELF/source hashes, failed prefixes, full tables and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-velocity-stream-2026-11-10/`.
Start with `benchmark-report.md`, `benchmark-carriers.csv`,
`benchmark-final-summary.json` and the final-bytewise captures. All motor
runs use native nominal 20 kHz (derived 20004.272 Hz), Hall commutation,
the same measured model/geometry and 24-V supply. Limits remain Iq 3 A,
dq trip 6 A, bus input 1 A and zero bus regeneration; no NVM writes.

Stopped-only, ADC-owned DWT codec measurements use exact captured 64-record
fixtures, 2 warmups and 8 measured blocks per fixture/encoding. Empty-bracket
cost is subtracted; all 18 bytewise/word result rows were read from immutable
published RAM after DONE and checked against surviving logs. Input loading,
checksumming, reporting and transport I/O are outside the measured brackets.

| Moving fixture, bytewise codec | CPU per record | Bytes per record, including block header |
| --- | ---: | ---: |
| Raw34 | 0.508 us | 34.5 |
| Packed26 | 1.108 us | 26.5 |
| Packed26 + delta | 3.840 us | 17.891 |

Delta saves about 32.5% of this fixture's block bytes but adds 2.732 us per
record, about 5.46% of one CPU at 20 kHz. These are finite microbenchmarks,
not total pipeline cost or worst-case execution bounds. The optional word
comparison candidate was rejected: its whole image was slower, including
unchanged control cases, so code layout is a confounder. Wire checksums and
byte counts match across both implementations. Packed26 is the preferred
high-rate encoding; delta remains optional rather than being enabled by default.

The real Ergot `framed_stream::Sink::send_ty` was also timed against a plain
copy into the same kind of 2304-byte grant. Its observed Packed26 cost was
8047 cycles per 64 records, about 48 us/block or 0.75 us/record, including
the copy. The frame adds 18 bytes per block. This excludes routing, locks,
real queue contention, waking tasks and USB service. Copy alignment changes
the linked memcpy path substantially: arbitrary Ergot-minus-copy rows cannot
be used as a universal protocol overhead. Exact destination offsets and
disassembly are in `codec-framing-bench-v3.alignment.json`.

The 16-attempt carrier matrix retains every aborted command-timeout prefix.
Its word-codec candidate was subsequently rejected; the final bytewise
normal-image rechecks are recorded separately in the report. Direct USB
does not demonstrate a bypass advantage: it lost producer records with
both codecs; moving its encoder to the thread executor did not cure this
and was reverted. Ergot's raw encoder belongs on the thread executor:
putting a self-waking encoder on the USB interrupt executor starved command
handling. USB RX/TX remain on their interrupt executor.

RTT Packed26 and RTT delta preserved all samples and all 28 commands in
the normal and timing five-second-hold cases. An Ergot Packed26 attempt
preserved its prefix but timed out at 17/28 commands, so it is a failed
whole maneuver; earlier and final full normal-image passes remain separate
evidence. Direct-USB losses originate at the device producer queue, with
zero host drops. This is not evidence of USB wire corruption or proof that
Ergot serialization is the bottleneck. Executor and queue/write architecture
differ between carriers, so the matrix is not an isolated framework-cost test.

Final bytewise/Packed26 normal-image rechecks, same five-second velocity hold:

| Carrier | Saved records per attempt | Missing records | Commands | Largest ACK |
| --- | ---: | ---: | --- | ---: |
| Ergot USB, two attempts | 162220 / 162227 | 0 / 0 | 28/28 each | 10.494 ms |
| Raw RTT, two attempts | 162189 / 162190 | 0 / 0 | 28/28 each | 2.168 ms |
| Separate USB endpoint | 154333 | 7858 producer, 0 host | 28/28 | 2.276 ms |

Ergot/RTT rechecks have matching END totals and no Hall warnings/faults.
The RTT repeat uses the same image without reset. These are finite normal
data/command successes; their FOC deadlines were not measured.

All timing matrix images observed FOC overruns. RTT's active mean was about
42 us per 50-us cycle, with sampled maxima about 73-78 us. Contiguous
telemetry sequence numbers do not certify every hardware ADC deadline.
Normal images were not instrumented for deadline acceptance. Hall speed
also oscillates substantially, and some runs report the existing stale-edge
warning; lossless transport is not stable velocity-control acceptance.

For further control diagnosis, use RTT + Packed26 as the reference recorder
and keep commands on Ergot USB. Do not adopt the separate USB endpoint as
an assumed performance fix. Qualify FOC deadlines and speed tracking, then
address shared USB queue fairness and copying with measured evidence.

The standard normal image (no raw sink or CPU microbenchmark) was restored,
SHA-256 `4f5396dcf7d7e1b48f2acea608e9323b53a587de5cd1e07f41f92b20bb1d4ef2`.
Final Stopped verification captured 82032 native-rate records with zero loss.
Readback matches reference Hall geometry, motor model, current limits and
board calibration; fresh boot current offsets are intentionally excluded.
Final state: Stopped/Hall, zero faults, 24.033 V. No calibration NVM writes.

## Runtime CPU attribution and observer-log spikes (2026-11-10)

The sink-only microbenchmark above is not full USB cost. Current moving
VelocityControl traces now include Hall/UART/fault preemption as well as ADC,
OTG_FS and the USB executor. ADC end markers follow statistics bookkeeping.
The analyzer can retain validated closed prefix/tail frames and idle gaps
around a complete lowest-priority IRQ witness, avoiding sparse-USB busy-poll
bias. Original strict/trim behavior remains the default.

Native 20 kHz, +40 electrical rad/s, Packed26, unchanged model/geometry and
protections. Full report, exact images/patches/symbols, every failed attempt,
CSV/JSON and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-runtime-cpu-2026-11-10/runtime-report.md`.

| Instrumented finite windows | ADC/FOC | USB IRQ/executor, excluding traced preemption | Raw encoder/sink task |
| --- | ---: | ---: | ---: |
| No stream, Ergot image | 82.7-82.9% | 0.11-0.37% | 0.76% idle task |
| Packed26 over Ergot USB | 87.4-87.6% | 7.44-8.19% | 3.55-4.55% |
| No stream, RTT image | 85.1% | 0.21% | 0.73% idle task |
| Packed26 over RTT | 88.6-89.7% | 0.41-0.49% | 5.82-5.98% |

These seven windows span 20-33 ms each, with trace overhead. Thread task
times exclude traced IRQs; untraced TIM2/RNG and hooks may remain included.
Other/unattributed time is not a measured idle budget. Different images and
rotor trajectories prevent assigning cross-image ADC differences to SWD
contention. Minimal-timing RTT motion averaged 41.78 us with a 54.52-us
observed maximum; trace percentages must not be added to that separate build
as an exact production CPU accounting.

The longest original ADC records repeated modulo 8192. The observer's periodic
`info!` executed inside ADC and added roughly 5000 cycles outside its ordinary
observer/startup/output buckets. Same-ELF A/B, changing only the diagnostic
switch while Stopped, recorded three logs versus zero, with maximum ADC body
74.69 versus 55.56 us. Mean body time barely changed. The normal image now
compiles that text log and its counter out; opt-in `observer-logging` and
`bench-observer-logging` retain reproduction. No estimator equations changed.

Detailed complete active windows attributed about 12.14 us to phase estimation
(observer subset about 3.97 us), 6.82 us to the current controller, 2.49 us
to ADC/conversions, 2.51 us to snapshot/Hall preparation and 3.74 us to
publication. These nested diagnostic sections cannot all be added together.
Remaining maxima include more expensive command handling and thermal cycles.
Removing the logger does not establish the 50-us deadline or stable speed.

Ergot USB still exhibited ACK starvation and producer losses in profiling
builds. RTT's minimal-timing capture preserved 162178 records and all 28 ACKs;
other instrumented attempts retained Hall warnings or sample losses as failures.
No warning/protection/command deadline was weakened. A SPI bridge could offload
material USB work, but cannot repair independent ADC overruns; improve the ISR
and queue/command admission before adding hardware.

`just build-mk5-runtime-profile SINK timing|outliers|trace` owns feature flags.
An optional final `on` enables observer-log reproduction. The no-stream
control uses `just test-velocity-no-stream-mk5 ELF`, explicitly energizing the
secured motor while leaving the native control rate unchanged. Results in
the report separate capture status, ACK successes, warnings and whole-stage
acceptance. SWD setup/export occurs only while Stopped, outside active windows.

Normal and observer-logging core configurations passed 661 unit tests and
11 doctests; strict core/host-example Clippy, two outlier-harness tests and
16 CPU analyzer tests passed. The standard normal image was restored without
trace/outlier/logger-switch symbols. Final Stopped capture: 82032 records,
zero loss; Stopped/Hall, fault_count=0, 24.090 V. Motor model, Hall geometry,
current limits and board calibration read back equal; no calibration NVM writes.

## Further ADC optimization (2026-11-10)

Kept three measured changes: shared FastSinCos range reduction, a four-entry
actuation-voltage ring with prepared delay weights, and exact flash NTC tables
at the original sampling cadence. The tables add 16 KiB flash, no table RAM;
8194 MCU comparisons with the original conversion passed before PWM startup.
Normal builds retain warnings and compile periodic info/debug diagnostics out.
Full core validation: 663 tests and 11 doctests passed; strict Clippy passed.

Same +40 electrical rad/s velocity maneuver, native 20004.27 Hz, Packed26,
unchanged model/Hall geometry/limits, minimal timing:

| Image | Mean moving ISR | Observed moving maximum | Capture |
| --- | ---: | ---: | --- |
| Original, two runs | 41.77–42.05 us | 54.71 us | no loss; repeat retained a Hall warning |
| Optimized RTT, two runs | 39.36–39.37 us | 51.72–51.95 us | 162188 / 162194 records, no loss |
| Optimized Ergot USB | 39.42 us | 52.18 us | 162231 records, no loss |
| Optimized Ergot USB, no timing | unmeasured | unmeasured | 162229 records, no loss |

All four final runs acknowledged 28/28 commands and ended Stopped with zero
faults. Maximum ACK latency was 1.66 ms RTT and 4.84 ms USB. The approximately
6% mean improvement is repeatable; rare deadline overruns remain. These body
times exclude the final statistics tail and do not establish a WCET bound.
Detailed profiling itself lost 221 samples, so its section attribution is
diagnostic evidence, not capture acceptance. Remaining peaks include command
work; none of that diagnostic window's thermal cycles overran.

Executable SRAM and additional observer/phase/command outlining did not help
and were reverted. The actual speed-control assessment remains separate from
the derivative of the commutation angle carried in fast telemetry.
The final Stopped-only Hall export retained 127 real edges, all forward, with
no invalid transitions or hardware overcaptures. Rolling electrical-revolution
speed after 1 s ranged 26.99–66.06 rad/s (median 37.73) for the 40-rad/s target.
This confirms remaining regulation variation independently of the displayed
commutation derivative. The 128-entry diagnostic prefix omitted 54 later edges;
the full-rate USB capture remained lossless.

Report, exact images/patches, all failed attempts and visually checked plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-adc-opt-2026-11-10/optimization-report.md`.
Final board image uses `just build-mk5-raw ergot normal`, with the calibrated
clock and priority scheduler; it is Stopped, without detailed profiling or
periodic text diagnostics. No NVM configuration writes were made.

### Hall handoff diagnostic follow-up (2026-11-10)

Finite RAM events now capture Hall-health changes, commutation/observer angles,
edge age and observer validity. Opt in with `just build-mk5-raw ergot hall-phase`;
normal builds exclude its 3.5 KiB buffer and ADC work. Export only after Stop:
`just dump-hall-mk5 output.json exact.elf [prebuilt-cli]`. Probe attach can halt
briefly; this is not a live, nonintrusive export. Both feature modes build, and
the decoder was exercised against device captures and its doctests.

The proposed measured-sector/observer-angle stale guard was **not accepted**:
`guarded-soft` still reported a Hall warning and lost three producer samples.
Observer phase was already about 0.85 rad ahead of Hall re-entry, so it cannot
yet serve as an independent position witness in this regime. The candidate
was removed from the working tree; its patch and all failed evidence remain
under `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-speed-2026-11-10/`.

### Hall repeatability and sweep current (2026-11-10)

Nine runs captured 4,246,674 native-rate Packed26 records through Ergot with
zero producer/host loss, sequence gaps or Hall overcaptures. At 3 A and
1 degree/5 ms, all three native calibrations failed LowConfidence: one boundary
varied by 17 degrees across turns. At 10 A it varied by at most 4 degrees;
three native 17-A calibrations passed with worst ranges of 2–3 degrees and
averaged boundaries repeatable within 0.67 degrees between runs. These are
electrical-angle repeatability figures, not absolute accuracy. Slower 10-ms
diagnostic sweeps also passed offline checks; the 17-A case covered all seven
electrical turns per mechanical revolution in each direction.

The high-current runs used a temporary 30-A dq trip, with observed peak
17.477 A. The historical 6-A bench trip is not a board/motor rating. Local
VESC 7.00 wizard source selects Hall current as detected motor limit / 3,
about 17 A for the user's 50.93-A result; its PI gains are 0.0005/1 versus
0.01/10 in these sweeps. Sixfold Iq disturbances near phase-current zero
crossings repeat at 1.666/3.289 Hz in slow/fast sweeps. PWM-command deadtime
compensation and PI tuning are candidates for the reported pulsating sound;
no synchronized audio or causal A/B has established its source yet.

Original Hall table, model, limits and board calibration were restored and
read back; Stopped, zero faults, no NVM writes. Velocity stability remains
unqualified. Detailed results, source comparison and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-repeat-2026-11-10/report.md`.

### Command deadtime correction A/B (2026-11-10)

`just build-mk5-raw ergot normal on|off` selects command correction independently
of the unchanged 360-ns gate deadtime. Four matched 17-A Hall sweeps, repeated
in reverse order, confirmed the sharp sixfold current disturbance comes from
command correction in this regime. Forward Iq RMS was 0.230–0.231 A with
normal PI/correction, 0.099 A without correction, 1.024 A with soft VESC-wizard
PI/correction, and 0.165 A with soft PI/no correction. Both repeats closely
agree. Normal PI/no correction also passed native Hall calibration at 17 A.

This is not acceptance of globally disabling correction. At +40 electrical
rad/s, new Hall geometry barely improved velocity RMSE (11.70 to 11.49 rad/s);
disabling correction increased it to 21.77 rad/s and reported a Hall-staleness
warning. The batch stopped before its planned repeat. No guard was weakened.
All twelve captures (3,375,368 records) were lossless, including the failed
velocity run. Every plotted Hall edge was retained in the short velocity tests.
The centroid/interpolation transition around 500 eRPM and Hall/observer health
remain diagnostic targets; stable velocity control is not established.

Both firmware variants build. Restored compensation-on firmware and original
runtime configuration, Stopped; no NVM writes. Report and inspected plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-ab-2026-11-10/report.md`.

### Velocity PI with unavailable braking (2026-11-10)

Finite RAM snapshots exposed positive current requests during overspeed:
full back-calculation at a zero braking limit manufactured integral to cancel
the negative P term. With Kp=0.05/Ki=0.05 and a 40-erad/s target, the old code
reached 1942 erad/s feedback and a 25.4-A measured d-current peak. Its 128-edge
diagnostic prefix overflowed; full-run speed statistics from that prefix are invalid.
`ClampedPI` now uses conditional integration and bounds stored authority;
the current-loop PI remains unchanged. A regression fails before and passes
after; 665 core tests and 11 doctests pass, strict core Clippy passes.
The same gains after the fix gave 5.74-erad/s full-revolution speed RMSE,
4.05-A dq peak and no faults, but reverse edges and ripple remain. A new
200-erad/s safety ceiling starts derating at 180; sampled feedback stayed below
101 in that run. This is not qualification of smooth velocity control.
Captures, exact ELFs and Hall replay: `oxifoc-hall-motion-2026-11-10` under
`/home/okhsunrog/tmp_zfs/codex_tmp/`. All captures so far have complete USB records.

Voltage limits now install through the stopped ISR transaction, with the same
prepared thresholds used by fault integration, UV recovery and failsafe braking.
Boot uses that validation path; writes cannot exceed the board range. Hardware
checks with PWM stopped produced OV/UV faults at deliberately crossed thresholds,
rejected 58 V, and verified persistence/reboot of the 8–26 V bench range.
668 core tests and 11 doctests pass. Evidence and exact ELF:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-voltage-brake-2026-11-10/`.

Hall timeout now additionally requires independent observer angular travel
since the captured edge, cancelling a constant observer phase offset. Inferred
crossings after a glitch retain the conservative timeout until a real edge
arrives. 685 core/Embassy tests pass, including cable dropout and glitch recovery;
strict Clippy and the F405 build pass. Physical cable removal was not tested.
Final `witness-fast-off.elf` runs at +40/-40 electrical rad/s retained
82,230/82,173 native 20-kHz samples with no losses or Hall warnings. They still
had 7/5 reverse Hall transitions: smooth velocity control is not qualified.
An earlier reverse run lost 25 producer samples; a later clean run does not
establish that intermittent loss is fixed. Command deadtime correction was
disabled in these comparisons; hardware gate deadtime remained 360 ns.

The matched 3-W dissipative-braking experiment is rejected: 18.60-A dq peak,
7,817 producer losses and repeated Hall warnings, with 42–43-us mean FOC and
peaks around 60 us. Its finite Hall buffer overflowed, so its full-run speed
statistics are invalid. The algebraic solver fast path preserves power/current
bounds in tests but is not a demonstrated cure for the runtime problem.
Next isolate the large negative Id request and Hall-frame error during braking.

The follow-up isolated Id polarity with zero requested Iq: the +2 A staircase
produced no Hall transitions, while the otherwise identical negative staircase
produced 14, including reversals. Both 20 kHz captures were complete. The heat
allocator now defaults to aligning positive Id and preserves an explicitly
requested d-axis direction; the magnitude/power solve is separate. A physical
Hall-plant regression reproduces restoring versus destabilizing behaviour.
All 686 Embassy-enabled core tests and strict Clippy pass.

With the same 40 rad/s profile, 3 W heat budget and PI 0.05/0.05, the old sign
gave a 17.91 A dq peak and 6507 producer drops. The final implementation's
forward/reverse peaks were 4.09/4.63 A, with no Hall warning generations, but
43/36 producer drops and remaining speed pulsation. This fixes a contributor,
not smooth-speed or lossless-stream qualification. Evidence remains in the
same artifact directory (`id-sign-results.json`, `results.json`, exact ELFs,
20 kHz Parquet, stopped Hall exports and plots). NVM dissipation stays off.

The 0.01/0.2 PI and a 100-eRPM interpolation floor did not resolve the 40-rad/s
pulsation. At -120 rad/s, the full Hall trace had no reversals and a -125.62
rad/s median revolution speed, but 49 producer samples were lost. The 120→40
step reintroduced slow-speed pulsation (2.09 A peak, 323 lost samples, no Hall
warnings). The bench edge buffer is now 512 entries to retain these runs;
the exporter also accepts the older 128-entry and seven-word velocity layouts.
Offline replay (`hall-speed-audit.json`) shows that the current edge-speed
estimate does not integrate to observed net boundary travel during rocking;
do not equate a low-pass of that signal with independent rotor-speed evidence.
Raw captures, configurations, exact images, plots and failed runs remain in the
same artifact directory; `results.json` distinguishes incomplete recordings.
Final flash configuration, verified after reset: drive 10 A, dq trip 30 A,
bus draw 5 A, regen 0 A, brake ceiling 5 A, dissipation 0 W, VBUS 8–26 V.
Controller left Stopped with no faults; motor model and Hall NVM were unchanged.

### Hall travel-consistent speed (2026-11-10)

Control samples now differentiate bounded physical boundary travel with a
20-ms filter; raw edge speed and the commutation-angle limiter remain separate.
Captured displacement is accumulated before sampling, including zero net travel
at a same-boundary reversal. This removes the unbounded integral of the previous
`sector_width / edge_age` stop estimate. Six trajectory/regression tests cover
stops, asymmetric rocking, unequal sectors, both directions and missed sample
cadence; all 692 core/Embassy tests and strict Clippy pass.

Replaying four complete earlier captures changes the mean-speed disagreement
against net boundary travel from +6.51/-6.95/+1.98/+6.64 rad/s to
+0.14/+0.52/+0.10/-0.67. Finite-window filter lag remains. First hardware runs at
40 rad/s with PI 0.05/0.05 and 0.01/0.2 still oscillate (13/5 reverse edges,
4.18/2.44-A dq peaks), without faults; producer losses are 404/392 samples.
These runs establish neither smooth regulation nor streaming acceptance.
Artifacts: `motion-*` in the existing `oxifoc-voltage-brake-2026-11-10` directory.

`just build-mk5-raw ergot hall-outliers off` now retains Hall evidence alongside
section costs, including the due-only velocity update. In `motion-profile-1`,
the eight longest instrumented ADC bodies were 55.6–58.9 µs: velocity work
4.35–7.86 µs, estimator work 11.34–12.84 µs, current controller 4.67–4.78 µs.
Seven of these eight also processed expensive command/gate work or combined
scheduled work with braking. Detailed profiling adds overhead and lost 26,393
producer samples; it is not representative streaming acceptance.

Bench Hall images also retain 16 fault-generation/category/speed snapshots,
exported by `dump-hall-mk5`. A stopped-only temporary thermal-derating test
verified both Warning/Derating and its subsequent clear, with exact config
restoration. This closes the evidence gap where the host saw a changed registry
generation but no remaining active fault. The older `motion-forward120-2` run
had that gap (generation 2, startup feedback 190 rad/s versus a 180-rad/s
rolloff); speed derating is inferred there, not a captured fault-category fact.
