# Synchronous inductance excitation, 2026-11-09

This follows the rejected [L01 run](bench-timing-bringup-2026-11-09.md#l01-rejected-after-a-protection-trip)
and the [USB priority study](bench-usb-priority-2026-11-09.md). The purpose is
to make native L excitation finite, synchronized with control samples and
cancellable, then qualify recording while the motor carries current. Agreement
of an average L with VESC alone is not calibration of Ld, Lq or saliency.

## Setup and acquisition

MK5 STM32F405 UID `2B002C001047343031363330`, PSU last confirmed 24 V / 5.5 A.
A/controller green connects to motor yellow, B/blue to blue, C/white to black.
The motor remains secured and unloaded. Stored limits are 3 A reference and
6 A dq trip, input 1 A, regeneration/brake 0. No detection result is applied or
persisted. The stored Vbus gain remains `1.009725`.

TIM1 is 167835842 Hz, 8390 ticks per control period: actual sample frequency
20004.271990 Hz, nominal 20 kHz. Independent TIM5 is 83917920 Hz. HFI carrier is
5000 Hz and the injection direction rotates at Fs/32 = 625.1335 Hz. Thus the
rotating excitation has sidebands around 4374.87 and 5625.13 Hz. These are
different from switching the control/PWM frequency to 5 kHz.

Each live measurement retains the full M=1 stream, raw ADC, engineering
currents, command and measured voltages, PWM compare registers and enable bits.
Sequence-derived sample time does not measure host arrival time. Exact ELFs,
symbols, build features, native outcomes, RTT and before/after configuration
are retained. No SWD attach/read occurs during an active capture; where used,
continuous RTT monitoring attaches before excitation.

## Excitation and shutdown changes

- Async/thread mode prepares finite voltage tables. The ADC control ISR applies
  one entry per call and stores paired currents; no waveform trig or FFT runs
  inside the ISR. A packet has at most 256 samples; ordinary HFI collection uses
  four 32-sample FFT windows plus pipeline priming, currently 130 samples.
- At packet end the ISR returns to the retained current PI hold before signaling
  completion. Thread scheduling cannot leave the last injection voltage applied
  indefinitely. The voltage-pulse fallback also uses finite scheduled periods.
- Each capture requires complete finite samples, consecutive wrapping software
  sequence numbers, bounded adjacent timer intervals and bounded cumulative
  duration. These checks reject stretched packets; they do not prove every
  physical ADC trigger was serviced or establish ISR WCET.
- Fault, host Stop, command override, timeout or dropped future abort excitation.
  Host cancellation stays latched across packets and rejects later internal
  active commands until a new explicit detection begins. Fault/timeout/missing
  samples cannot trigger another excitation method or yield success.
- A verified holding-current window is required before HFI or pulse excitation.
  The previous inhibited-output build could invent plausible L from noise;
  the new check returns MotorNotResponding before any packet is armed.
- Both PI voltage components are retained. Inverter correction keeps the same
  commanded current reference as the preceding PI hold during each packet.
  Ordinary DirectVoltage behavior remains unchanged. Current measurement,
  telemetry and independent dq protection still use the actual currents.
- Amplitude now comes from the separate phase-invariant lag/impedance probe.
  A noisy interim di/V estimate formerly inflated 1-A-hold excitation from an
  intended roughly 0.16 V to 1.252 V, producing a 4.981 A dq peak. That capture
  and its valid-looking result remain recorded as pre-fix evidence. The modeled
  ripple target is 25% of holding current, capped at 2 A; it is not an
  instantaneous current limit. The actual 6 A dq trip remains independent.

## Scheduling and capture integrity

The selected policy remains USB IRQ plus `task-priority`. Within cooperative
queues, USB workers are priority 3, protocol servers 2, detection and telemetry
both 1, background work 0. USB runs on the USART1 software-pended executor at
NVIC P6, OTG_FS at P5 and ADC at P0. Cooperative task priorities and NVIC
preemption levels are different scales.

Detection at priority 0 was starved by telemetry; at priority 2 its computation
starved telemetry. Preparation, copying and analysis now yield every eight
samples, sharing priority 1 with the bounded telemetry batches. ADC telemetry
notification is one-shot: an abandoned AtomicWaker subscription previously
continued waking detection on every ADC call. Packet completion has its own
notification instead of polling the current sample waiter for the whole packet.

The plain transport qualification retained 54528 samples at nominal 5 kHz and
218133 at nominal 20 kHz, zero gaps/losses, all 502 command replies per run at
50 commands/s. ACK p99/max were 2.195/3.632 ms and 4.005/9.282 ms. These
transport profiles inhibited physical outputs; the live tests below supply
the separate stationary-current acceptance.

Finite RAM trace windows show detection poll median reduced from 699 to
217 microseconds, with maxima still 2257 and 2038 microseconds. The latter
38.4-ms window measured mean ADC duration 32.33 microseconds, maximum 38.23,
about 64.6% occupancy. This is a finite instrumented sample, not a latency bound.
Trace builds themselves lost stream samples (final traced run: 4617 lost),
whereas ordinary builds passed. The CLI now exits nonzero on any recording
loss before applying a result, retaining raw data and native outcome; this was
verified with the traced failure. No rtos-trace/SystemView backend is claimed.

## Live outcomes and remaining calibration work

The six pre-amplitude-fix successful runs retained 550608 contiguous samples
with no recording loss or registered fault. Mean L ranged 24.203–24.395 uH.
They establish the packet/recording behavior, but include the 4.981 A case and
are not acceptance of the old amplitude policy. Earlier failed pilots and
inhibited/trace captures are retained separately, not counted as passes.

A first live packet pilot before correction-reference preservation tripped at
6.027 A. It stopped excitation and returned HardwareFault without fallback or
an applied result. Subsequent inspection found the PI-reference to
measured-current inverter-correction transition; holding Vq alone was too
small to explain the runaway. This distinction is preserved in the raw logs.

An on-device Stop during detection at nominal 20 kHz was acknowledged in
2.081 ms. The capture retained 8017 contiguous samples, and the test observed
750 ms with no restart and final Stopped/faults 0. The detection outcome uses
the existing HardwareFault wire code for cancellation; it does not mean a
registered hardware fault occurred. Later explicit detections succeeded on
the same boot. This exercises cancellation during the holding ramp, not every
possible instruction boundary inside a packet.

VESC 7.00 wizard reported average L 25.00 uH and Lq-Ld 11.33 uH. The earlier
native HFI results have average close to VESC but difference only 4.06–5.05 uH.
The LCR's 1-kHz between-lead extrema were about 35.3–37.1 / 52.5–52.9 uH,
equivalent-star 17.65–18.55 / 26.25–26.45 uH. Frequency, excitation level,
holding current, rotor position and estimator differ. The remaining saliency
disagreement needs an estimator/voltage-transfer study, not a forced match.

Use spectra for these known HFI packets to identify excitation/response
sidebands; DC holds and start/stop transitions use time plots and plateau
statistics. `vesc-bench/analysis/analyze_oxifoc_bursts.py` rejects decimated or
incomplete streams and selects a real packet from RTT. FFT plots use 128 real
samples with a Hann window; zero padding does not add frequency resolution.

Raw campaign root:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-sync-inductance-2026-11-09/`.
The final source/amplitude qualification, provenance hashes and endpoint
readback follow.

## Final amplitude qualification

All five runs below use `dfdb0a5`, ordinary live outputs, no trace hooks and
no configuration application. Same R input, 0.030444 ohm; the 0.05/0.2/0.5 W
requests derive the listed holding currents and clamp the last to 3 A.
The four repeats use the same firmware boot. All terminal states are
Stopped/faults 0 and all before/after configurations match exactly.

| Run | Hold (A) | Ld (uH) | Lq (uH) | Mean L (uH) | Peak dq (A) | Samples |
|---|---:|---:|---:|---:|---:|---:|
| Pilot | 1.046 | 22.469 | 28.599 | 25.534 | 1.527 | 90090 |
| Repeat 1 | 1.046 | 22.753 | 26.833 | 24.793 | 1.581 | 92090 |
| Repeat 2 | 2.093 | 21.903 | 26.622 | 24.263 | 3.259 | 86076 |
| Repeat 3 | 3.000 | 21.736 | 26.591 | 24.163 | 4.430 | 84090 |
| Control repeat | 1.046 | 22.497 | 26.990 | 24.744 | 1.573 | 92065 |

Total 444411 samples, zero sequence gaps/lost samples. The 1-A-hold pilot used
0.111875 V HFI amplitude, versus the previous pathological 1.252 V. This is
observed current bounding for this motor/setup, not proof of a universal peak
limit. The remaining spread and Lq-Ld 4.08–6.13 uH do not validate saliency.
The 3-A packet plot and the before/after current envelopes are versioned in
vesc-bench as `docs/assets/hfi-packet-2026-11-09.png` and
`docs/assets/hfi-amplitude-before-after-2026-11-09.png`.

Stop was re-tested on the final image: ACK 2.083 ms, 8020 contiguous samples,
observed no restart for 750 ms, final Stopped/faults 0. No fault threshold was
raised. The final readback is Stopped/faults 0, Vbus 24.166 V, FET 30.2 C,
motor 27.6 C. No motor result is stored.

Running ELF: `live-amplitude/live.elf`, SHA-256
`64f2436f969431a8ed4781a0c8aa89cd1b9c02ab8b42ceed96574c10d3577ae5`.
Source `dfdb0a5016d6550ae0a06fcfccf9f79f72359f69`; features
`transport-usb,transport-uart,board-vesc6-mk5,bench-calibrated-clock,
bench-clock-reference,task-priority`. No trace or inhibited-output feature.
Transport qualification and the finite trace comparison precede the amplitude
change (`4b30b8b`); the final live series independently qualifies ordinary
full-rate capture after that change.

The amplitude regression fails against the preceding implementation (1.400 V)
and passes with the fix. Final core suite: 466 passed, two existing exploratory
tests ignored; doctests 11 passed/18 ignored. Core clippy with runtime,
Embassy, delivery, storage, virtual motor, HFI detection and experimental
impedance-sweep passes `-D warnings`. Fixed-capacity packet arrays intentionally
live in the static detection future; their large-future lint exceptions state
that reason. F405 release build/clippy pass with eight existing target/PAC/
diagnostic warnings. G474 release builds with existing target warnings;
G474 hardware is untested. The host suite passed before the amplitude-only
change, and its ignored on-device Stop test passed on the final firmware.

`campaign-summary.json` independently rechecks every capture's sequence/loss
metadata and configuration equality. `provenance.json` hashes source snapshots,
ELFs, raw captures, logs, plots and scripts, including failed/intrusive trials.
Original failures remain separate from final acceptance; no missing samples
are interpolated or silently discarded.

Next: estimate the voltage-to-current transfer from these exact packet samples
with weighted fitting or synchronous demodulation, compare both axes and
separate inverter/ADC transfer from winding saliency. The current di/V estimator
near carrier zero crossings remains a mathematical follow-up. Do this offline
first; additional frequencies/angles are useful only after the fit has an
explicit voltage/sample-timing model.
