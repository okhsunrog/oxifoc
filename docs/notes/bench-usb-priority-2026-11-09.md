# USB IRQ with task priorities, 2026-11-09

Select **USB IRQ + `task-priority`** for the current MK5 bring-up. The
[previous study](bench-schedulers-2026-11-09.md) compared priority as a
replacement for the USB interrupt executor. This follow-up retains USB IRQ
and measures whether priority also helps the remaining cooperative work.
Its recommendation supersedes the earlier default-off recommendation for MK5.

## Workload and measurement boundaries

Board UID `2B002C001047343031363330`, calibrated clock, PSU last confirmed
24 V / 5.5 A. Controller A/green connects to motor yellow, B/blue to blue,
C/white to black. Every build inhibits physical phase outputs; the handshake
and all recorded PWM-enable values (64, phase bits clear) confirm this.
FOC calculations run at zero current reference, fixed angle, PI 0.01/10.
Each full profile sends 502 commands, including Stop, with a 50 commands/s
active section lasting approximately 10.2 s. No motor measurement is performed.

Nominal 5/20 kHz streams use actual rates 5001.068/20004.272 samples/s.
TIM1 is 167835842 Hz with 8390 ticks/period; trace uses independent TIM5 at
83917920 Hz. SWD arms before activity and reads after Stop. No probe or
per-event USB trace export runs inside the measured window.

Fresh instrumented builds compare default and priority with the same recorder:
2816 twelve-byte events plus 32 task-progress slots of seven u32 fields.
The recorder samples short windows delayed 250/1250/4250 ms after activation;
whole-profile counters independently count readiness and polls until Stop.
The counter reset runs once at activation, outside the delayed trace windows.
Plain builds omit the trace hooks, buffer and counters entirely.

All variants include the bounded telemetry batch-yield fix `da88083`.
Task ordering is USB 3, protocol servers 2, fast telemetry 1, remaining tasks 0.
Priority changes ordering inside each executor; it cannot preempt a running
poll. USB remains on USART1's software-pended executor at NVIC P6, OTG_FS P5,
ADC P0. These NVIC levels and cooperative task priorities are separate scales.

## Command latency and capture integrity

Pooled successful ACK round trips at 20 kHz, in milliseconds:

| USB IRQ policy | ACK count | p50 | p95 | p99 | Max |
|---|---:|---:|---:|---:|---:|
| Default, traced | 1506 | 2.841 | 5.107 | 5.543 | 6.616 |
| Priority, traced | 705 | 1.851 | 3.248 | 3.731 | 4.555 |
| Default, plain, previous study | 1791 | 2.361 | 4.170 | 4.922 | 10.927 |
| Priority, plain | 1506 | 1.701 | 3.061 | 3.591 | 5.495 |

ACK distributions include valid prefixes of incomplete profiles; they do not
turn those profiles into passes. Plain default is a prior-session control,
not an interleaved randomized comparison. These observations support the
policy choice but do not establish a worst-case latency guarantee.

Fresh traced default passed all four profiles (one 5 kHz, three 20 kHz).
Traced priority completed only one of four: the host's 5 ms dispatch-lateness
guard stopped the other profiles at steps 461 (5 kHz), 24 and 179 (20 kHz).
There were no device ACK timeouts, sample gaps or faults in those prefixes.
The cause of host dispatch lateness has not been isolated; these files and
errors are retained. The prior plain-default set also contains a host-guard
stop at step 285, in addition to its three complete 20 kHz profiles.

Priority with trace omitted passed one 5 kHz and all three 20 kHz profiles:
708855 samples overall, including 654327 at 20 kHz, with zero gaps/losses and
all 2008 command replies. The final build using the new `task-priority` spelling
also passed a separate 20 kHz profile: 218102 samples, zero gaps, 502 replies,
ACK p99 3.758 ms, max 5.085 ms. It is the currently flashed image.

Finite device trace windows contained 11 default and 10 priority protocol
ready-to-poll samples across the three 20 kHz runs. Maximum observed wait
fell from 2178 to 1188 microseconds; windows were 33.7–35.1 ms long.
This small sample is corroborating evidence, not a tail-latency guarantee.
ADC entry jitter persists; task priority did not resolve it.

## Background progress and executor ownership

In the complete third 20 kHz traced profiles, both policies polled the clock
reference 10 times, heartbeat 20, log forwarder 20 and ISR statistics 10 times
over approximately 10.2 s. Fault LED polls were 100 versus 97. Its largest
inter-poll gap increased from 102.3 to 121.4 ms with priority. Such gaps include
intentional task sleeps; they are not queue-wait measurements. No task that
became ready went entirely unpolled in either full profile. A readiness near
the final Stop boundary may remain pending when counters freeze.

This verifies progress under the tested stream/command workload. It does not
qualify active detection, storage writes, fault injection, disconnect handling
or every possible starvation scenario. Idle listeners with no ready events
are not classified as starved. Counter decoding validates the exact ELF's task
symbols, SHA-256, timer wrap and capture duration.

The USB interrupt executor currently owns `usb_task`, `run_usb_rx` and
`run_usb_tx`; `usb_io_task` initializes and spawns them, then returns.
Protocol/routing services, telemetry encoding, configuration, detection and
logging stay in thread mode. Future short, bounded I/O workers may join if
measured latency requirements justify it. Work with a different urgency or
execution budget should use its own executor/IRQ level, rather than making
USB wait behind unrelated processing. UART/CAN are candidates to measure
when used, not automatic additions to this executor.

FOC remains in the ADC ISR. The L detector's excitation must synchronize with
PWM/ADC; neither the USB IRQ executor nor cooperative priorities guarantees
one update per 50 microseconds. Fault abort and stale/missing-sample rejection
remain required before repeating the rejected L01 measurement. No L fix or
live motor qualification is claimed here.

The [subsequent live-detection study](bench-inductance-packets-2026-11-09.md)
places bounded detection processing alongside telemetry at priority 1 and
qualifies finite ADC-synchronized packets separately. Its results supersede
the active-detection limitation above without changing the USB IRQ ownership.

## Source, artifacts and final state

`task-priority` is an explicit runtime feature; `bench-scheduler-priority`
remains its compatibility alias. MK5 build, flash, check and size recipes now
select it; CF2 defaults are unchanged. The qualified source is `dccafa2`,
instrumentation `f0bf753`, and the task-progress decoder in vesc-bench `3721497`.
The selected ELF was built with the archived source diff before that runtime
commit; recipe/documentation edits do not change its compiled code.

Raw captures, exact ELFs/symbols, configuration, source diffs, RTT, failed
profiles, analysis scripts and plots are under:

`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-usb-priority-2026-11-09/`

`summary.json` covers all 13 profiles; `decision-summary.json` includes the
prior plain-default control and background counts. `provenance.json` hashes
52 source/artifact files and verifies all 13 endpoint snapshots are Stopped
with zero faults. Synthetic decoder fixtures are excluded from hardware
provenance. The visually checked comparison is versioned in vesc-bench as
`docs/assets/usb-irq-priority-2026-11-09.png`.

Final ELF: `selected-irq-priority-plain/selected-irq-priority-plain.elf`, SHA-256
`30aee850f2608197f4e638aaf30d9a7bf6ecfe294f3819ee841588107477a023`.
Features: `transport-usb,transport-uart,board-vesc6-mk5,bench-calibrated-clock,
bench-no-phase-outputs,bench-clock-reference,task-priority`. Phase outputs remain
inhibited. Final readback: Stopped, faults 0, Vbus 24.071 V, FET 29.1 °C,
motor 27.4 °C. Configuration exactly matches the final boot's initial snapshot;
stored voltage gain and current limits are unchanged throughout the study.

F405 release build and clippy passed with the existing eight diagnostic/target
warnings. The legacy alias was built and exercised on-device before promoting
the policy; the new spelling was separately built, flashed and exercised.
The task-progress decoder passed synthetic wrap/checksum/boundary cases and
decoded all eight hardware progress recordings. Only Embassy `trace` hooks
were used; no `rtos-trace`/SystemView backend was tested.

Primary source: [pinned Embassy scheduler implementation](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/raw/run_queue.rs),
[interrupt executor](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/platform/cortex_m.rs).
