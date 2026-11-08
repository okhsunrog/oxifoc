# Filtered phase-voltage resistance detection and timing checks

Continuation of [the signed-current study](bench-current-voltage-2026-11-08.md),
started on November 8 and finished after midnight November 9, Moscow time.
The native MK5 R detector now uses measured phase voltage and returns about
30.4 mΩ. The fix is flashed, tested and committed as `efbe783`.

## What changed in the detector

`DetectionHardware` has a separate resistance-sample interface with an explicit
voltage source. The Embassy backend selects `FilteredPhaseAdc` only on boards
declaring hardware phase filtering; other backends retain their declared
`ControllerCommand` path. This does not establish that a legacy command model
is accurate on an inverter without measured voltage.

On MK5, a genuinely new ADC snapshot and its FOC output are copied together
under the same critical section. `PhaseVoltageSense` converts the terminal
counts, and Park uses that output's angle to obtain measured Vd. Missing or
invalid phase ADC on a filtered board fails the measurement; it cannot silently
fall back to PI. A source change within a window also fails and sends Stop.

The low/high current levels, settling and sampling budget remain the existing
two-point procedure. `abs(ΔV/ΔI)` was removed: a negative slope now fails range
validation rather than becoming a plausible positive resistance. The generic
controller-voltage default remains supported when its slope is valid.

`wait_telemetry` and the meaning of `FocOutput.vd/vq` are unchanged. In particular,
the DirectVoltage holding-voltage calculation for inductance still needs a
command voltage, so it must not be replaced by terminal voltage. The observer
has not been switched to the biased phase measurement as part of this fix.
No ICD or persistent-record schema change was needed.

The board's switchable phase RC filters are enabled by PC13 high; current
filter PD2 is also high. Initialization and boot logs agree with this setting.
The board descriptor's filtered-sensing flag is therefore applicable here.

## Native hardware measurements

PSU remains 24 V / 5.5 A. Stored command limit 3 A, independent phase protection
6 A, bus input limit 1 A, regen 0 A. The native request uses `max_power_w=1` and
the F405 server's 3 A ceiling. It first probes 0.1→0.5 A, then measures at
0.6→3 A; fixed electrical angle zero, detection PI 0.01/10. No `--apply`.

| Run | Native R | Independent full-plateau cross-check | Raw rows | Gaps / lost |
|---|---:|---:|---:|---:|
| 01 | 30.166 mΩ | 30.524 mΩ | 268,444 | 0 / 0 |
| 02 | 30.213 mΩ | 30.455 mΩ | 268,438 | 0 / 0 |
| 03 | 30.742 mΩ | 30.318 mΩ | 270,436 | 0 / 0 |

Native mean 30.374 mΩ; sample standard deviation 0.320 mΩ; range 1.90% of the
mean. All three ended Stopped without a fault. Full-run Id maxima were
3.381/3.408/3.354 A. On-board logs explicitly report `filtered phase ADC` and
the sampled low/high V/I pairs. Results were not stored; Vbus gain remains
1.009725, and no phase correction has been persisted.

The independent calculation uses conservative windows selected from the full
recorded current-transition traces, excluding ramps and initial settling.
These are not the exact on-device sampling windows. The native detector samples
2000 fresh snapshots while its async task runs; they are not necessarily 2000
consecutive ISR frames. The small disagreement and run-to-run variation should
not be described as calibrated absolute uncertainty. Contiguous ISR averaging
would be a separate possible precision improvement if a tighter error budget
is needed. The wrong voltage source is corrected; measurement perfection is
not established.

## Phase-voltage offsets across directions

Additional signed runs used the same 20 kHz nominal acquisition, limits and PI.
The 90° run used ±3 A with low-current alignment holds; 0° and 120° used ±1.5 A.

| Fixed angle | Current | Phase-voltage signed R | Recovered stationary α/β intercept |
|---|---:|---:|---:|
| 0° | ±1.5 A | 29.714 mΩ | +30.665 / −38.418 mV |
| 120° | ±1.5 A | 29.770 mΩ | +31.138 / −37.918 mV |
| 90° | ±3 A | 30.202 mΩ | +31.676 / −37.472 mV |

One fixed stationary bias vector, approximately **(+31.16, −37.94) mV**, explains
the rotation of the d/q intercepts. Maximum vector deviation between those
runs is 0.69 mV. This supports a board/sensing contribution rather than a
rotor-frame offset. Unequal divider/channel gains at the approximately 12 V
PWM common-mode level are a plausible cause; the experiment does not uniquely
separate gain error, ADC offset, filter effects and real terminal imbalance.
It does not justify saving a constant d/q correction or calibrating per-phase
gains from this one operating voltage.

The 90° run had a transient Id minimum of −5.601 A and Hall transitions during
reorientation, with no fault. This was away from the zero-current crossing,
so it is distinct from the former compensation turn-on impulse. Remaining
directions were reduced to 1.5 A. The attempted 240° run aborted at requested
0.48 A because a command reply missed the runner deadline. Stop was confirmed,
fault count remained zero, and the retained incomplete capture was refused by
the plateau analyzer. It is not included in the table or the bias fit.

## Timing investigation

Commits `26145a0` and `7eee820` add entry-to-entry DWT statistics and a read-only
comparison against received USB SOF frame numbers. Execution-duration counters
alone cannot distinguish a late ISR from a lost conversion. The new diagnostics
do not replace the control clock or alter timer settings.

- A stopped 20 kHz capture with 50 Stop commands saved 413,281 rows, zero gaps.
  Sequence-time / host-time slope was 0.993460.
- Repeating with the probe session detached saved 413,280 rows, zero gaps,
  slope 0.993436. RTT polling is not the cause of the systematic drift.
- ADC entry averages stayed 8397–8398 CPU cycles, consistent with center-aligned
  TIM1 ARR=4199. Across 336 reported windows in the final firmware log, maximum
  entry interval was 16,203 cycles, below two nominal PWM periods. There is
  significant entry jitter, including long intervals under streaming load;
  do not equate this with an ideal uniformly timed command-update path.
- USB reference accumulated 337,228 frames / 56,269,538,660 DWT cycles, about
  166,859 CPU cycles per USB frame rather than the nominal 168,000. This agrees
  in scale with the host/sequence discrepancy and does not look like loss of
  0.65% of FOC publications. It is a relative clock observation, not a laboratory
  measurement proving which physical oscillator is wrong.
- RCC readback: CR `03037e83`, PLLCFGR `07402a04`, CFGR `0000940a`, SSCGR `0`.
  HSE/PLL selection and configured divisors are as intended; spread spectrum
  is disabled. Host read-only adjtimex showed normal −8.56 ppm correction,
  tick 10000 µs, not a 0.65% intentional time correction.

No clock calibration was guessed or persisted. Capture time/frequency remains
labelled nominal; exact dynamic/L/FFT frequency comparisons need this relative
clock discrepancy and interrupt-entry jitter accounted for. DC ΔV/ΔI does not
use elapsed time, so the R improvement does not depend on resolving this clock
calibration first.

## Validation and next work

468 core tests passed, 2 ignored; doctests 11 passed, 18 ignored. Core all-target
clippy, host library/CLI tests, MK5 release build/clippy, CF2/G474 compilation
passed. MK5 UART-only compilation passed with existing unused USB-code warnings.
New regressions cover offset cancellation despite negative PI slope, negative
slope rejection, source mixing and Stop, measured-voltage rotation, missing ADC,
unfiltered fallback and coherent fresh ADC/FOC copies. Angular plateau analysis
was validated on the accepted captures and rejected the failed 240° run.

Time traces, signed voltage/current fits, settling checks and the offset-rotation
plot answer the present questions. No new FFTs were produced for these DC tests.

The next distinct task is the inverter/observer voltage model: phase R is now
measured separately from the approximately 42–43 mΩ PWM-command slope, but
command-side compensation is still provisional. Cross-check phase sensing at
another common-mode voltage / with an independent reference before persisting
channel calibration or feeding absolute measured voltage to the observer.
Compare compensation on/off with hardware deadtime unchanged, then assess
dynamic control and L. Do not merge the inverter contribution into motor R
merely to make the current model fit.

Artifacts and reproducible analysis scripts:

```text
/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-cadence-voltage-2026-11-08/
/home/okhsunrog/motor_control/vesc-bench/analysis/analyze_oxifoc_signed_plateaus.py
```

The artifact root contains raw successful/failed captures, RTT, exact ELF hashes,
readbacks, test logs, `analysis-native/summary.json`, `analysis-axes/offset-comparison.json`
and figures. `provenance.json` records hashes and software revisions. No repeated
full-flash binary backup was made.
