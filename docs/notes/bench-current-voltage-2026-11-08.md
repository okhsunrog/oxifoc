# Current startup, transport and signed voltage measurements

This continues the [earlier aborted pilot](bench-voltage-capture-2026-11-08.md).
The repeated 0.6 A pilot and subsequent signed 1.5/3 A series completed on the
MK5 with no transport sequence gaps or controller faults. The phase-ADC signed
slopes are about 30 mΩ, while ideal PWM-command slopes are about 43 mΩ. PI
voltage before compensation is a third, unsuitable standalone R signal.

## Setup and changes

PSU 24 V / 5.5 A; motor secured on a printed stand, shaft unloaded. Controller
A green → motor yellow, B blue → blue, C white → black. Independent phase
protection remains 6 A; stored command limit 3 A, bus input limit 1 A, regen 0 A.
All runs use fixed manual electrical angle 0, electrical velocity 0 and runtime
PI Kp=0.01 / Ki=10, without storing R or modifying persistent calibration.
The user subsequently authorized motor tests without separate per-test prompts.

Local commits:

- `7b9611f`: default-off `bench-no-phase-outputs` qualification build, with
  every phase-enable path inhibited and a distinct firmware identity.
- `734374f`: continuous command-side deadtime correction near zero. Hardware
  gate deadtime remains 360 ns. F405's effective correction amplitude is
  separately named; each phase reference blends linearly through ±1 A.
  **1 A is provisional, not an experimentally calibrated inverter parameter.**
- `1048708`: lossless, independent compressed batches (ICD v8) and all USB
  control/RX/TX tasks on one P6 interrupt executor. ADC P0, Hall P1 and USB
  hardware P5 remain higher priority. The USART1 vector is used as an executor
  wake IRQ; USART1 hardware/pins are not enabled. See the
  [wire and executor details](telemetry-batch-encoding.md).
- `a824e3d`, `3c7bfa6`: reproducible signed-current maneuvers. The 3 A profile
  pauses each up-ramp at signed 0.6 A for 1.8 s before increasing excitation.

The smooth near-zero form has precedent in
[Microchip's deadtime model](https://microchiptech.github.io/mcaf-doc/9.0.2/algorithms/dead-time-comp.html).
It does not establish this board's effective compensation. Other boards retain
their core legacy default unless they explicitly configure the linear region.

The first raw-frame IRQ experiment and compression on thread mode each failed
qualification. Splitting bulk USB into a preempting executor while USB control
remained on the thread exposed a self-repolling endpoint-waker interaction in
the pinned Embassy AtomicWaker. A debug backtrace is saved; keeping all USB
tasks at the same priority avoids this path. No ergot/Embassy patch was made.

## Recording acceptance and limits

| Capture | Stream / PWM | Rows | Gaps / lost | Timeline commands |
|---|---|---:|---:|---:|
| Combined, outputs inhibited | 20 / 20 kHz | 133,152 | 0 / 0 | 1 |
| Command stress, outputs inhibited | 20 / 20 kHz | 610,097 | 0 / 0 | 150 |
| Command stress, outputs inhibited | 5 / 20 kHz, M=4 | 53,160 | 0 / 0 | 50 |
| Normal firmware, stopped | 20 / 20 kHz | 99,360 | 0 / 0 | 0 |
| Normal firmware, 0.6 A pilot | 20 / 20 kHz | 67,645 | 0 / 0 | 21 |
| Normal firmware, signed 1.5 A | 20 / 20 kHz | 210,700 | 0 / 0 | 151 |
| Normal firmware, signed 3 A | 20 / 20 kHz | 377,592 | 0 / 0 | 301 |

All terminal Stop replies succeeded. The inhibited captures have phase-enable
bits 0–5 clear; production captures include enabled phases and disabled tails.
The 5 kHz test is unfiltered decimation of 20 kHz FOC, not a PWM sweep or a
motor-response test. Initial stress attempts while heavy builds were running
on the host missed runner timing bounds and aborted; their files are retained
as failed tests. Accepted stress/pilot/signed runs had no concurrent builds or
heavy analysis. Default defmt ELF paths can be overwritten by another profile;
the inhibited qualification used its exact ELF explicitly.

Production RTT during the pilot shows about 20k offered/read frames per MCU
second, queue drops 0, failed broadcasts 0 and roughly 365 kB/s encoded payload.
Active ISR average was about 4964 cycles / 59%, maximum 6432, measured execution
overruns 0. Neither ISR-duration counters nor stream sequence numbers prove
that every physical PWM/ADC trigger was serviced.

In fact, signed-run receipt anchors expose a repeatable timing discrepancy:
sequence time / host time is approximately **0.99345**, with residuals below
0.5 ms after an affine fit. The counter advances per serviced FOC interrupt.
This does not yet distinguish physical clock error, missed ADC service or
growing receipt latency. The analyzer selects holds by ordered sequence
anchors, not uncorrected host-time masks. Absolute cadence and missed-trigger
diagnostics need verification before precision dynamic/L/frequency work.
The table's rates are the nominal reported rates; zero gaps qualifies delivery
of the published samples, not an externally measured exact 20,000 Hz clock.

## Startup and settled current

The old 0→0.06 A request applied full sign compensation at once, adding about
0.2304 V along alpha at 24 V / 20 kHz / 360 ns. A roughly 30-count CCR jump
coincided with the 3.816 A Id impulse. That run also suffered transport loss.

With smooth compensation, the complete 0.6 A pilot has:

- maximum Id 0.981 A over the whole run; first 0.06 A window maximum 0.331 A;
- 0.9–2.4 nominal seconds: mean Id 0.60003 A, std 0.0915 A; mean Iq 0.000013 A;
- no comparable large startup impulse in the current/CCR traces;
- maximum command acknowledgement wait 7.91 ms and send lateness 0.114 ms.

The first signed 1.5 A run briefly reached Id −2.998 A with an Iq transient
after reversing the current vector. RTT reported Hall transitions, supporting
rotor reorientation; zero RPM columns cannot establish no motion because
MotorParams/pole-pair calibration is absent. The subsequent 3 A profile includes
low-current alignment holds; its full Id range was −3.377 to +3.389 A, with no
protection fault. These are measured responses, not guaranteed peak bounds.

## Signed voltage slopes

The analyzer excludes the first 0.5 nominal seconds after acknowledged hold
anchors and the last 0.1 s before the next command. All original samples in
each window are used. Positive repeats are averaged before differencing with
the negative point; use measured Id, not requested current, in the denominator.

| Current pair | Phase-ADC slope | Ideal PWM-command slope | Pre-compensation PI slope |
|---|---:|---:|---:|
| ±0.6 A alignment holds | 28.69 mΩ | 43.30 mΩ | −122.80 mΩ |
| ±1.5 A | 29.91 mΩ | 42.83 mΩ | −88.62 mΩ |
| ±3 A | 30.12 mΩ | 42.47 mΩ | −32.82 mΩ |

These are distinct voltage definitions, not three physical winding-resistance
estimates. The ideal PWM proxy is `(2/3)*Vbus*(CCRa−CCRb/2−CCRc/2)/ARR`, using
the modulator's max-duty convention and saved preload registers. It includes
command-side compensation but is not actual gate timing or terminal voltage.

The phase-ADC signed intercept is consistently about +30.4 mV; measured Vq is
about −38 mV at nearly zero Iq. Both merit per-channel scale/offset and sampling
checks. A constant offset cancels from a signed difference; gain error and
current-dependent offset do not. At 3 A, mean phase Vd is roughly +121/−60 mV,
so taking V/I at either sign separately would be misleading.

Positive phase-voltage repeat spans were 0.41 mV at 1.5 A and 0.30 mV at 3 A.
Settling-window checks at 0.15/0.3/0.5/1 s show why the negative 1.5 A alignment
transient should be excluded. On settled 1.5/3 A holds, mean current remains
at its request; voltage windows differ by tenths of a millivolt. These checks
measure repeatability/window sensitivity, not calibrated absolute uncertainty.

The roughly 30 mΩ phase-ADC results agree in scale with VESC's earlier phase
voltage slopes and the manual motor measurements. The approximately 13 mΩ
PWM-to-phase slope excess is consistent with an inverter/measurement-model
contribution, not proof that it is all MOSFET resistance. VESC used different
frequencies/current-vector direction in the earlier sweep; the comparison is
not yet a fully matched experiment. No R was saved or forced to VESC's value.

The PI signal changes with the chosen compensation law. Using its differential
slope as winding R without accounting for that law is now visibly invalid.
The next estimator change must define which voltage represents motor-terminal
voltage and use that definition consistently; changing only a stored R number
would hide the problem.

## Analysis choices and remaining checks

For DC resistance use time traces, settling checks, signed V/I means, offsets
and repeat drift. The voltage overview uses 100-sample block means only for
display; fitting uses the raw samples. The pilot's FFT/PSD/spectrogram helped
describe its noise background, but repeating those plots on every signed hold
adds little. Use spectra when investigating a specific oscillation or periodic
artifact. Preserve the raw data to revisit new hypotheses later.

The raw current sum is noisy even while stopped. In the pilot plateau, raw
phase correlations were 0.93–0.95; about 96% of raw-current variance lies in
the common three-channel component. Three-input Clarke rejects that component
for FOC, but independent raw-current protection remains. This observation does
not identify the physical noise source or justify relaxing protection.

Next: verify interrupt-start/PWM timing and ADC service independently of the
per-serviced-cycle stream counter; cross-check phase-sense scale/offset; then
compare compensation on/off at fixed hardware deadtime and matched axes.
Only then finalize the R estimator/observer voltage contract and expand PWM
frequency or inductance work. Motor tests are authorized; no additional
per-test permission gate is implied by this sequence.

Software validation: 458 core tests passed (2 ignored), core doctests 11 passed
(18 ignored), host library 18 and CLI 7 passed. Core/host clippy and MK5
normal/inhibited release build/clippy passed; CF2, G474 and MK5 UART-only
compilation passed (existing target-feature warnings remain). Signed profiles
passed offline maneuver validation; the analyzer processed both real captures.

## Artifacts

```text
/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-transport-comp-fix-2026-11-08/
/home/okhsunrog/motor_control/vesc-bench/analysis/analyze_oxifoc_signed_plateaus.py
```

The artifact directory retains successful/failed captures, exact ELF SHA-256
descriptors, RTT, statuses, numerical summaries, startup comparisons, signed
hold plots and `analysis-signed/voltage-current-comparison.png`. See its
`provenance.json` for identities and hashes. Normal ELF SHA-256:
`07811eee6429326fc5a3f572c7aff3510d31846343417bd6ff601f42566f35c0`.
Inhibited ELF SHA-256:
`1c0754b3d5a132c75c637d625bb62f9180cb1fca9cc4489697128469ab1b55c0`.
Production boot/captures report ADC reference 3325 mV; the inhibited boot
reported 3327 mV. Each capture stores its own actual handshake calibration.
Persistent Vbus-only gain remains 1.009725; phase divider is nominal 18.727272.
No new full-flash binary backup was created.
