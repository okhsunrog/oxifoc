# Timer-derived control period and return to motor measurements

Continues [the independent clock study](bench-executor-clock-2026-11-09.md).
Raw files, exact ELFs, build/test logs and provenance are retained under:

```text
/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-timing-bringup-2026-11-09/
```

The user removed the scope and reconnected the motor. Mapping stays A/controller
green → motor yellow, B/blue → blue, C/white → black. Last confirmed PSU settings
24 V / 5.5 A. Runtime limits read back before flashing: command 3 A, phase trip
6 A, bus input 1 A, regen 0 A. Vbus calibration gain remains 1.009725. The motor
is secured/unloaded and the user has authorized motor tests without repeated
permission requests. No full-flash binary backup is made for each build.

## Timing contract

The old F405 path derived FOC dt, compensation, detection's sample frequency and
host time from requested 20000 Hz. Actual TIM1 center-aligned period is
`2 * ARR * (PSC + 1)` source-clock ticks. At calibrated declared clock
167835842 Hz, ARR=4195/PSC=0, that is 8390 ticks, dt about 49.98932 µs and
20004.271990 Hz. This is a HAL/register-derived estimate, not an assertion of
absolute oscillator calibration or service of every ADC interrupt.

F405 freezes this ratio once after PWM setup, before starting ADC control IRQs.
FOC dt, deadtime compensation and L detection use it. ISR budget is derived from
the same period. Protocol tasks wait for its initialization rather than report
timing before PWM is configured. PWM frequency cannot change silently during a
capture; a future live frequency change must update this contract atomically.

ICD v9 adds the source clock, ticks per FOC cycle and applied decimation to the
telemetry configuration ACK. The frozen bootstrap shape and compressed sample
layout remain unchanged; exact version matching rejects v8/v9 skew. Whole-Hz
`actual_fast_hz` is the integer floor; raw timing consumers use the ratio.
Requested/nominal HardwareInfo frequency continues to select familiar stream
modes. The host validates the ACK, never guesses decimation from two rounded
frequencies, and computes unwrapped-sequence `t_s` from the rational period.
Parquet metadata includes control_timing, foc_hz_derived and fast_hz_derived.
Analysis tools prefer these rates and retain fallback support for old files.

G474's synthetic stream and the virtual backend retain their configured nominal
cadence via ControlTiming::from_hz; this change does not claim to have calibrated
another physical board. Shared signatures compile for both platforms. Embassy
TIM2's separate ~380 ppm alarm-clock rounding remains an explicit follow-up;
this change does not patch a private library timer or hide it in the FOC ratio.

## Validation before excitation

Host/core tests cover fractional rate, explicit decimation, disabled streams,
wide integer divisors, and Parquet time across a 16-bit sequence wrap with M=4
while the handshake still reports nominal 20000 Hz. Host CLI/lib/virtual Clippy
and F405/G474 compile checks pass. The bootstrap structure was not extended.

First hardware build inhibits every phase output and keeps USB/SOF diagnostics.
At requested 20 kHz, a 30.3 s zero-reference command timeline retained 614319
rows, zero gaps/loss; all 151 replies and terminal Stop succeeded. Derived
sample rate was 20004.271990 Hz. At requested 5 kHz, a 10.3 s / 51-command timeline
retained 53544 rows, zero gaps/loss, all replies and terminal Stop succeeded.
Applied M=4 gives 5001.067998 Hz. Both captures have phase-enable mask 0x40 only
(MOE alone, phase channels disabled). Parquet time increments match each wrapped
sequence delta times the source-clock ratio to below 2e-15 s arithmetic error;
this tests serialization/time reconstruction, not absolute clock accuracy.

## Stationary measurement shutdown

R02 showed a positive 3 A hold with PI command Vd about -0.098 V, while measured
phase Vd was about +0.122 V. The PI command precedes deadtime compensation and
must not be identified with terminal voltage. Before running L, inspection found
that HFI, voltage-pulse and impedance-sweep cleanup ramped this command toward
zero. For a negative holding command, that can increase current rather than
remove the hold.

All three stationary paths now return to the retained current PI loop (no gains
reset), slew the positive holding-current reference to zero, then Stop. The
initial reference equals the previous hold. Phase angle stays fixed. The
voltage-pulse path still shuts down before propagating an invalid measurement.
A regression runs the complete pulse procedure with subthreshold pulses and an
effective positive voltage correction: the holding command is negative and the
measurement fails, yet current remains bounded and Stop is reached. The old
voltage-to-zero cleanup would drive the fixture from 1.2 A toward 7.33 A.

## Motor R after clock correction

The normal, calibrated-clock build was flashed with USB SOF diagnostics and all
motor phases connected. No detection result was applied/persisted. Native R
uses max_power_w=1, the same 0.1→0.5 A probe then 0.6→3 A measurement, filtered
phase ADC, angle zero and PI 0.01/10 as the previous source-correction study.

| Run | Native R (mΩ) | Independent settled ΔV/ΔI (mΩ) | Rows | Gaps/lost |
|---|---:|---:|---:|---:|
| R02 | 30.485 | 30.593 | 316361 | 0/0 |
| R03 | 30.201 | 30.338 | 318331 | 0/0 |
| R04 | 30.647 | 30.563 | 318355 | 0/0 |

Native mean 30.444 mΩ, sample standard deviation 0.226 mΩ. VESC 7.00 wizard
reported 30.80 mΩ (-1.16% difference), previous OxiFOC mean was 30.374 mΩ.
Currents/temperature are not matched calibration conditions. LCR 100 Hz
minimum-position between-lead ESR 61–63 mΩ corresponds to equivalent-star
30.5–31.5 mΩ, not an exact DC resistance standard. Clock correction has not
materially changed DC ΔV/ΔI, as expected.

Each rich capture ended with outputs disabled, status Stopped and fault count
zero. Maximum reconstructed dq magnitude was about 3.385 A. The overview plots
retain transient envelopes; no FFT is used for DC resistance. The independent
fit selects long settled 0.6/3 A plateaus and records its windows explicitly;
they are not the exact on-board asynchronous 2000-snapshot windows.

An earlier R01 returned 30.472 mΩ with 312318 contiguous raw samples, but the
CLI's default full-rate selection consumed HardwareInfo before Capture::start
could use it. Engineering columns became NaN and scaling metadata was absent.
Its untouched raw file and native result are retained; exclude it from rich
engineering fits. Commit 2acbc49 passes the same handshake into capture, rejects
missing handshakes before recording, and corrects stale CIC wording: decimation
is plain sample dropping. R02–R04 use the default rate path and verify the fix.

R02 contains one unprojected A-shunt value 6.109 A at seq 35558. All three raw
channels jump together: unprojected sum 9.553 A, while projected Id/Iq remain
2.924/0.062 A. Adjacent duty registers barely change. This is a common/zero-
sequence component, not demonstrated 6.109 A winding current. The existing
software trip uses dq magnitude and the hardware protection is unchanged.
Sampling pickup/displacement current remains a possible acquisition follow-up;
this observation does not justify raising the protection threshold.

## L01: rejected after a protection trip

The first native L run retained 356371 contiguous full-rate samples, but its
reported Ld=8.115 mH / Lq=9.506 mH is invalid. Outputs were active only until
1.009934 s. The last 0.010/0.166 V dq command was held for 19 PWM frames
(about 0.95 ms); dq current reached 6.012 A and the configured protection
removed phase outputs. Status then showed Error/Stopped and one fault.
The asynchronous HFI task nevertheless continued its calculations and pulse
fallback, eventually reporting success. No result was applied or persisted.

Retain L01 as failure evidence. Before another live L run, detection must abort
on a fault and reject missing current-sample periods; the excitation must not
assume one async iteration equals one PWM period. The executor comparison is
being completed first with physical outputs inhibited. Current-loop shutdown
regression is tested, but L01 does not validate normal HFI shutdown on hardware.

Raw data, exact ELFs, RTT, configs and result JSONs for this section are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-timing-bringup-2026-11-09/`.
