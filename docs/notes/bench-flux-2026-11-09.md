# MK5 flux-linkage bring-up, 2026-11-09

Same board/wiring and calibrated clock as the R/L campaign; PSU 24 V / 5.5 A.
Persisted reference/trip limits remain 3/6 A, input 1 A, regen 0 A.
The initial detection runs below did not apply or persist motor parameters.

The initial 20-kHz controls returned R = 30.860864 mOhm and
Ld/Lq = 18.300083/27.557062 uH (146151/68077 consecutive samples).
The 2-A Hall calibration observed all six physical states, no Hall errors
or timer overcaptures, and retained 294432 consecutive samples. Its result
now reaches the live Hall estimator in RAM without a reboot or flash write.
`bench-hall` emits 50-Hz observations tagged with ADC sequence numbers;
the control/capture interrupts do not log.

Raw captures, exact configuration and analysis are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-flux-2026-11-09/`.

## Native coast measurement

The Embassy adapter now supplies coherent terminal-voltage ADC and Hall
snapshots while the bridge is undriven. Previously the hardware backend did
not supply coast telemetry, so detection used the driven method. A fixed
500-sample coast window also outlasted this motor's useful coast interval.
Instantaneous Hall sector speed lags during rapid deceleration; the new
estimator integrates BEMF magnitude over three complete electrical turns,
using captured Hall edges for the boundaries. Missing sensors, skipped or
reversed sectors, poor turn consistency and the ADC noise floor reject the
measurement. Driven fallback remains available when coast is too short.

| Target ERPM | Native coast lambda, mWb | Difference from VESC |
| --- | --- | --- |
| 4000 | 3.093224 | -3.79% |
| 5000 | 3.160818 | -1.69% |
| 6000 | 3.190380 | -0.77% |
| 6000 repeat | 3.173070 | -1.31% |

Reference: VESC 7.00 full wizard readback, 3.215180 mWb, in
`/home/okhsunrog/tmp_zfs/codex_tmp/vesc-bench-bringup/full-detection-01/comparison.json`.
That wizard's waveform capture was incomplete; only its parameter readback
is used here. No scale correction was fitted to that result.

All four native runs completed without fallback, retaining 480641 consecutive
samples at 20 kHz in `qualified/`. No new Hall errors or overcaptures occurred;
the cumulative Hall error counter was already 1 before calibration. The
largest sampled dq-current magnitude was 4.198 A (6-A trip). Final status:
Stopped, fault count 0, bus 24.033 V, FET/motor temperatures 29.9/27.4 C.

The older driven method returned 3.476–3.501 mWb at 2000–6000 ERPM. Its
voltage-estimation bias is unresolved. Terminal ADC offsets and remaining
speed dependence also need investigation before claiming calibrated accuracy;
Hall closed-loop commutation is not qualified by these open-loop/coast runs.
No motor parameters or increased current limits were saved during those runs.

The initial Hall error was a startup-accounting bug: the first valid sector
was compared against the constructor's logical sector 0 as if an edge had
been skipped. A regression test reproduces it and covers all six starting
sectors plus real skipped transitions. After the fix and verified flash,
`hall-startup.parquet` retained 294301 consecutive samples with all six states,
zero errors/overcaptures, and a valid live RAM calibration (not persisted).
The subsequent 6000-ERPM coast run returned 3.207849 mWb with 120141
consecutive samples and zero Hall errors/overcaptures; final status is Stopped
with no faults. Core/Embassy tests: 503 passed, 2 ignored; strict Clippy passes.

## Persisted model and Hall current-control qualification

Hall calibration and R = 0.030860864 ohm, Ld = 18.300083 uH,
Lq = 27.557062 uH, lambda = 3.19 mWb, seven pole pairs were subsequently
saved and verified after reset. Current limits remain unchanged. Captures,
configuration readbacks and logs are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-current-2026-11-09/`.

The Hall current checks track 0.6-A Id and Iq steps up to 1.2 A without
faults, but full-model boot still overflows the MCU raw telemetry queue.
Earlier lossless RAM-only checks had no configured observer and do not
qualify the persisted setup. Gapped recordings are diagnostics only.
Live MotorParams application must also configure observers and pole pairs
through the same setup path as boot; that inconsistency remains open.

Coherent DWT profiling windows now snapshot all ISR counters together;
the diagnostic forwarder gets priority while profiling so active windows
are not silently omitted. Pure 0.6-A Id hold, full model at 20 kHz:

| Section | Cycles | Time at 167835842 Hz |
| --- | ---: | ---: |
| Entire ADC ISR, entry/exit timing only | 6614–6669 | 39.4–39.7 us |
| Entire ADC ISR, detailed profiling | 7020 | 41.8 us |
| Angle/estimator update, detailed | 1987 | 11.8 us |
| Back-EMF observer, included in the row above | 782 | 4.7 us |
| Current controller/transforms/modulation, detailed | 1092 | 6.5 us |
| ADC reads + snapshot preparation + publication, detailed | 1424 | 8.5 us |

Evidence: `check-coherent-profile.parquet.host.log`,
`check-timing.parquet.host.log` and `check-standard-channel.parquet.host.log`.
The first two runs used an experimental empty-command-queue fast path.
It saved only 55 cycles (0.33 us) and was removed to retain Embassy's
standard queue. Detailed profiling adds about 2.4 us with that path.
Minimal timing shows about 79% ISR occupancy, with about 15k of 20k
samples/s drained during the hold; transport admission failures are zero.
The saved VESC wizard setup used foc_f_zv = 30000 and V0 Only, which is
15-kHz actual control on this board. VESC ISR execution time has not yet
been measured under matched conditions. CCM placement and forced wrapper
inlining did not improve these checks and were not retained.
