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
