# F405 MK5 voltage acquisition and first current pilot

## Setup and provenance

User-confirmed PSU: 24 V / 5.5 A. Controller A (green) → motor yellow,
B (blue) → blue, C (white) → black. Communication: USB bulk / ergot;
logs: STLink V3 RTT. MCU STM32F405RGTx, UUID `2B002C001047343031363330`.

Flashed firmware sources: `1a1843e56b2aa71b5b4cb1ec608cbaff98e4d667`, release,
`transport-usb,transport-uart,board-vesc6-mk5`. ELF SHA-256:
`5a4a99db7506b7bda509361dcdbc1062478504034dfd0f512f213d648e68f6a3`.
This identifies the build passed to `probe-rs run --verify`, not a flash readback.
Later host-only changes required no reflash.

Captures, decoded logs, status/config snapshots, numerical reports and figures:

```text
/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-rich-bringup-2026-11-08/
```

`provenance.json` records per-run host commits, firmware artifact, current CLI
hash and artifact hashes. No per-build flash backups were made. VESC motor/app
XML was exported before switching firmware.

Calibration: ADC reference 3325 mV, divider 18.727272, shunt 0.5 mΩ, gain 20;
persisted bus-only gain 1.009725. Phase scaling remains nominal and does not
reuse the bus-only correction. Stored phase-current protection is 6 A,
commanded-current ceiling 3 A. Motor parameters/pole pairs are absent:
zero RPM columns cannot establish rotor motion or its absence.

## Idle qualification

The 34-byte ICD-v7 frame adds phase ADC and pre-update PWM registers; see
[the acquisition contract](resistance-bench-plan.md). Queue buffering,
pending-batch preservation and batch coalescing were needed: the original
full-output-queue path discarded entire batches.

| Capture | Stream / PWM | Saved rows | Gaps / lost |
| --- | --- | ---: | ---: |
| `idle-5khz-coalesced` | 5 / 20 kHz, M=4 | 24,836 | 0 / 0 |
| `idle-20khz-coalesced-30s` | 20 / 20 kHz, M=1 | 596,162 | 0 / 0 |

All saved rows have phase enables off (`pwm_enables=64`: MOE only), ARR=4199,
and complete phase ADC fields. The 20 kHz file spans 29.80805 native seconds;
requested wall time includes host/start/stop handling. Startup queue losses
occur before the discarded 150 ms warmup; saved-window acceptance does not
imply zero counters over the entire startup session.

Idle projected Id/Iq standard deviations: approximately 0.064/0.063 A.
The raw three-current sum is much noisier; it is a diagnostic residual,
not physical neutral current. Mean Vbus: 24.003 V. Floating phase terminals
are not zero references. Time plots, FFT, Welch PSD and spectrograms were
inspected. M=4 is sample dropping without anti-alias filtering, not 5 kHz PWM.

## Approved pilot: aborted, no R result

`stationary-voltage-pilot-0p6a.json`: manual angle 0, Id ramp 0→0.6 A over
200 ms, 1.8 s hold, ramp down and Stop; PI Kp=0.01/Ki=10; PWM and recording
20 kHz. Host `b88c6e2` preserved the failed recording.

The pilot did **not** reach 0.6 A. Acknowledged requests reached 0.24 A;
the 0.30 A request got no response after three attempts and may have applied.
Terminal Stop succeeded, an additional Stop was confirmed, and final state is
Stopped / fault_count=0. No test result or PI override was persisted.

- Device RTT: approximately 16.6k accepted frames/s and 3.3k ISR-queue drops/s
  under active FOC, with repeated full-output-queue broadcast failures.
  ISR average approximately 4924 cycles / 58% CPU, sampled maximum 6296 cycles,
  no reported budget overruns. The precise USB/CPU bottleneck is not isolated.
- The old runner blocked on replies without draining its 4096-sample channel.
  Two-to-six-second waits overflowed it; retries stretched the scenario past
  18 s. Saved 34,830 rows / 967 sequence discontinuities. Sequence-derived
  141,312 missing samples and 8.80705 s span are **lower bounds**: a 16-bit
  counter cannot reveal full wraps missed during long outages. No R fits,
  joined-fragment FFT, or assumptions that old event anchors are fresh.

The longest contiguous active segment has 987 samples / 49.3 ms and a projected
Id peak of 3.816 A, matching a CCR step and phase α voltage rise. FFT/PSD/
spectrograms use that segment only and are labelled startup-transient spectra.

Command-side deadtime correction is discontinuous at zero reference:
`apply_dead_time_comp` uses reference phase signs regardless of magnitude.
At 24 V / 20 kHz / 360 ns, a 0→positive alpha reference adds
`4/3 * 24 * 360e-9 * 20000 = 0.2304 V` before PI can respond. The observed
approximately ±30-count CCR step, phase-voltage rise and subsequent negative
PI command strongly support this as the startup impulse mechanism. This does
not calibrate effective deadtime or absolute phase-voltage accuracy. Hardware
deadtime and compensation were not changed by the acquisition patches.

## Host correction and stopped hardware check

Host `b9faea7` drains telemetry while awaiting motor replies, sends excitation
steps once and aborts late replies/overdue steps. It sends the terminal command,
preserves capture/error metadata, and attempts emergency Stop after telemetry
shutdown if the terminal reply fails. See [timing bounds](../../maneuvers/README.md).

Fifty Stop-only commands 20 ms apart with 20 kHz recording: all acknowledged,
maximum reply wait 2.294 ms, maximum send lateness 0.106 ms. Saved 33,474 rows,
zero gaps/loss, outputs disabled in every row. This validates the stopped host
path, not active throughput. Seven CLI tests and CLI clippy pass. For firmware:
453 core feature-rich tests passed (2 ignored), core clippy, MK5 release build/
clippy passed; CF2/G474 compilation was checked for the initial wire extension.

## Next gate

Do not expand the current/frequency matrix yet. Isolate active USB servicing/
backpressure and protect command replies from telemetry congestion. The motor
server ignores failed response sends; instrumentation can distinguish applied
commands with dropped replies from RX loss. No ergot follow-up changes were
made; a generic response/priority fix needs a separate maintainer discussion.

Separately remove the full compensation step near zero. Compare a bounded
stationary pilot with command-side compensation disabled or softened, keeping
hardware deadtime/protections fixed. New excitation requires permission.
Qualify recording and transients before signed plateaus, frequency sweeps and
winding/inverter R fits.
