# F405 executor scheduling comparison, 2026-11-09

Follow-up: [USB IRQ together with task priority](bench-usb-priority-2026-11-09.md)
measures the combination and selects it for MK5. The results below concern
the earlier comparison of scheduling alternatives and remain unchanged.

This completes the earlier priority/USB experiment after clock correction and
adds an explicitly configured deadline scheduler. All measurements below use
one STM32F405 MK5 unit, UUID `2B002C001047343031363330`, calibrated clock profile,
protocol 9 and timer-derived FOC period 8390 / 167835842 s (49.98932 µs).

## Workload and capture

All physical phase outputs are disabled by `bench-no-phase-outputs`, also
checked in the handshake and recorded PWM-enable bits. The motor remains
connected but unexcited. Active FOC calculations use OpenLoop at zero current,
fixed angle, PI 0.01/10. Each profile has Stop at 0 s, active FOC at 0.5 s,
zero-reference commands every 20 ms for 10 s (502 commands total), then Stop.
Streaming requested at 5/20 kHz gives 5001.067998/20004.271990 samples/s.
Configuration is read, not changed; failed files and timeout errors are retained.

Four trace builds differ only in executor scheduling:

- `fifo`: USB tasks on the thread executor, default Embassy queue. The artifact
  label is historical; the implementation handles batches in reverse enqueue
  order and does not promise FIFO.
- `priority`: same executor, USB priority 3, protocol 2, fast telemetry 1,
  remaining tasks 0.
- `deadline`: same executor, fresh ready-relative deadline USB +125 µs,
  protocol +500 µs, fast telemetry +1000 µs, remaining tasks unset. A bench-only
  registry publishes static task Metadata and updates it in the ready hook
  before enqueue. Deadlines use Embassy's 32768 Hz tick resolution. This is a
  soft urgency policy, not a guarantee that these budgets are achievable.
- `irq`: USB device/RX/TX on the existing USART1 interrupt executor at NVIC P6;
  thread tasks use the default queue. OTG_FS stays P5, ADC P0.

Both schedulers remain cooperative within an executor. The pinned source sorts
ready tasks before polling; it cannot interrupt a long poll. With both features,
priority is the primary key, deadline only breaks ties. There is no blanket
claim here about every possible deadline/priority assignment.

Trace uses Embassy `trace` hooks plus ADC entry/exit, a finite 3072 × 12-byte RAM
buffer, no per-event logging or USB export during measurement. TIM5 PSC=0 clock
is declared 83917920 Hz, not the previous nominal 84 MHz; `--timer-hz` is now
explicit in export/analysis. SWD arms before activity and exports after Stop.
Attach/detach pauses are outside the trace window. Poll elapsed times include
preempting IRQs and are not exclusive CPU time. Finite maxima are observations,
not worst-case bounds; RX sample counts are especially small.

The first two variants (deadline/priority) used trace delays 0/200/500 ms across
three repeated profiles. At 20 kHz, a timeout can stop activity before a delayed
window. Only repeat 1 (immediate active trigger) is used for their active latency
comparison. Subsequent variants use immediate triggering in all repeats.

## The stress test exposed an application bug

At 50 commands/s, all four original variants failed at 20 kHz, including USB IRQ.
This is stronger than the earlier 5 commands/s qualification. In IRQ repeat 1,
USB TX ready→poll remained below 97.715 µs and RX below 34.415 µs, yet the fast
telemetry poll was still open after **at least 40.736 ms**. There was no completed
poll duration to put in a maximum; excluding that unfinished interval would
hide the main observation. The analyzer now plots censored polls explicitly.

The inner streaming loop keeps draining until its ISR queue is empty. With USB
on the same cooperative executor it eventually fills the TX queue and returns.
When USB is serviced preemptively, TX keeps succeeding, ADC keeps replenishing
the input queue, and this loop can monopolize thread mode. Protocol and detector
tasks cannot run even though USB RX itself runs promptly. Moving USB alone is
therefore insufficient at the stronger workload; scheduler choice must be
assessed again after bounding application work.

The core fix yields after every successfully broadcast full batch (at most 24
samples). It releases the stream configuration gate before yielding, reacquires
it afterwards and rechecks stream enable/session before broadcasting again.
There is no added timer sleep per batch. Stop/restart can invalidate retained
samples while the task is yielded. Existing stop-gate/session regressions and
the 403-test core suite pass; on-device comparisons follow below.

The fairness fix is commit `da88083`; the finite trace/deadline experiment is
`7132d03`. Requested deadline offsets 125/500/1000 µs round up to 5/17/33
Embassy ticks (nominal 152.588/518.799/1007.080 µs). The small remaining TIM2
scale error is documented in the clock study and cannot explain millisecond
poll stalls. Trace measurements themselves use TIM5.

`trace` is the instrumentation used for these measurements. `rtos-trace` is an
alternative event/backend integration, not a scheduler. In the pinned Embassy
source it adds task tracking/names and calls `rtos_trace::trace::*`; it still
needs a recorder/backend. No SystemView/rtos-trace backend was enabled or
measured here. The existing raw hooks already expose executor, task-ready,
poll and idle events; ADC entry/exit is added explicitly. This RAM recorder
resolves sub-microsecond scheduling events without sending them over the USB
path under test. Its 36 KiB buffer is a diagnostic feature, default off.

A plain-build 5 kHz run stopped at host step 299 because the CLI's 5 ms timeline
lateness guard fired (not a motor ACK timeout or a device fault). This failed
capture is retained separately, not silently replaced; another run is used to
check the same conditions. All status snapshots remain Stopped with zero
faults. Startup transport warnings are retained in the original logs.

## Results after the batch-yield fix

Each 20 kHz row below contains three runs with the same 50 commands/s profile.
Times are from the first immediate active trace window for each variant; windows
are 34.6–38.3 ms, not equal-duration worst-case searches. TX sample counts are
152/169/172/233 respectively. RX has only 1–3 samples, so its observed maxima
are plotted with sample counts but are not suitable for percentile comparisons.

| Executor configuration | Complete 20 kHz runs | Lost samples across three files | Max observed USB TX ready→poll | Max completed telemetry poll |
|---|---:|---:|---:|---:|
| Thread/default | 0/3, motor ACK timeout | 158 | 1251.5 µs | 839.9 µs |
| Thread/priority | 0/3, motor ACK timeout | 98 | 813.8 µs | 887.6 µs |
| Thread/deadline | 0/3, motor ACK timeout | 251 | 844.4 µs | 854.4 µs |
| USB IRQ, default thread queue | 3/3 | 0 | 97.9 µs | 1136.9 µs |

The IRQ case retained **654358 samples and 1506 successful command replies**,
including successful terminal Stop in every run. Per-run command ACK p99 did
not exceed 5.398 ms; the largest ACK was 6.495 ms. These are end-to-end host
round trips, distinct from device ready→poll. IRQ preemption contributes to the
elapsed telemetry poll; its larger elapsed value is not exclusive CPU cost.
The protocol task's observed ready→poll max in its first active window was
1.991 ms, versus a right-censored wait of at least 21.278 ms before the fix.

At 5 kHz all instrumented runs passed without gaps: three each for default/IRQ,
one each for priority/deadline after the fix (the earlier four-way comparison
also passed three 5 kHz runs per variant). The flags are unnecessary at this
lighter load.

Control builds remove the RAM trace overhead:

| Configuration after fix | 20 kHz control outcome | Lost samples |
|---|---|---:|
| Thread/default, trace omitted | 0/3, motor ACK timeout | 0 |
| Thread/priority, trace omitted | 0/3, motor ACK timeout | 172 |
| Thread/deadline, recorder never armed | 0/3, motor ACK timeout | 18 |
| USB IRQ, trace omitted | 3 complete + 1 host dispatch-guard stop | 0 |

Deadline refresh uses the trace ready hook, so its control build necessarily
retains the hook/registry code and RAM allocation but never records events.
Default/priority/IRQ controls omit `bench-executor-trace` entirely. An intact
sample stream alone does not certify control responsiveness: plain default
lost no samples yet failed motor command replies. IRQ's three complete plain
runs retained **654288 samples and 1506 command replies**. A fourth IRQ run
stopped at host step 285 on the 5 ms host timeline guard, with no sample gaps
or device fault; it is retained alongside the clean repeat. The analogous
plain-default 5 kHz host-guard failure at step 299 also has a clean repeat
(54504 rows, 502 replies, zero gaps). No timeouts or failed files were hidden.

## ADC timing and remaining work

ADC/FOC elapsed median remains about 30.5–30.6 µs of the 49.989 µs PWM period;
its instrumented-window occupancy is about 61%. Maximum completed ADC elapsed
is 36.8–37.3 µs across the four post-fix windows. Entry intervals still range
roughly 31.6–86.1 µs, with an entry-phase span about 36 µs. This is entry jitter,
not evidence that every ADC handler runs longer than one PWM period. The
scheduler flags did not remove it. Critical sections / interrupt masking are
candidates for the next measurement; the cause has not been isolated here.

Keep the separate USB interrupt executor and the batch-yield fix for the
current 20 kHz stream. Leave both scheduler features default-off: neither
replaced preemptive USB service with the policies tested. This does not prove
that all future uses of priority/EDF are pointless. Reconsider priority when
there are concrete task latency requirements; use EDF when jobs have meaningful
release times/deadlines. FOC remains interrupt-driven and independent of these
cooperative queue policies.

The native L detector remains a separate blocker: a 0.8–1.1 ms telemetry poll
is still far too long to assume async work advances once every 50 µs. Abort on
fault and missing sequence periods before another live L run, and move timed
excitation to a bounded mechanism synchronized with ADC/PWM. Do not interpret
the scheduler study or the fairness fix as validating the rejected L01 result.

## Artifacts, validation and final state

Raw captures, exact ELFs/symbols, source diffs, configurations, RTT, kernel log,
command errors and analysis results:

`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-scheduler-study-2026-11-09/`

`provenance.json` hashes 118 ELFs, hardware trace buffers and parquet captures.
`run-summaries.json` covers all 62 profiles; `final-statuses.json` confirms every
recorded endpoint state is Stopped with zero faults. Main plots and their
explicit active-window selection manifests are in `comparison-before/`,
`comparison-after/` and the root `*comparison-manifest.json` files. Versioned
copies are in vesc-bench's `docs/assets/executor-*-2026-11-09.png`.

Core tests: 403 + 11 doctests passed; 16 doctests intentionally ignored. Core
clippy passed with warnings denied; F405 deadline/trace build and clippy passed
with existing target-feature/PAC qualification/unused PLL diagnostic warnings.
The synthetic analysis fixture checks timer wrap, explicit clock scaling,
unfinished polls and ready waits, ADC intervals and invalid event rejection.
The plots were visually inspected. No rtos-trace/SystemView backend was tested.

Final running ELF is `bounded-irq-plain/bounded-irq-plain.elf`: calibrated clock,
USB IRQ, physical phase outputs inhibited, no trace feature. Final status:
Stopped, faults 0, Vbus 24.014 V, FET 29.2 °C, motor 27.5 °C. Configuration is
unchanged except fresh per-boot current-sensor zero offsets; stored board
voltage gain and all current limits compare equal to the initial snapshot.

Pinned primary sources:

- [Embassy run queue and scheduler ordering](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/raw/run_queue.rs)
- [Metadata deadline API](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/metadata.rs)
- [trace and rtos-trace integration](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/raw/trace.rs)
