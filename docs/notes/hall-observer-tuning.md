# Hall-to-observer tuning on the MK5

Keep the 150–300 electrical rad/s crossover for now. On the unloaded Flipsky
5055, the observer held the tested high speeds with less phase current, and
moving the crossover to the VESC range showed no clear benefit. Revisit the
thresholds and estimator under a repeatable load. This is an MK5/H5055 baseline.

## Current calculation

- `motor-feedback mode=hall` uses Hall at low speed and blends to back-EMF
  once the observer is ready. Weight is linear in `max(|ω_Hall|, |ω_observer|)`
  between `blend_low_erad_s` and `blend_high_erad_s`. The angle blend wraps
  around 2π and the outer loop receives blended rotor speed. Below the high
  threshold, an angle disagreement greater than 90° can reseed the observer
  from Hall. Hall loss takes the separate fallback path. An untrusted fallback
  inhibits torque. See [phase selection][selector] and [Hall fallback][fallback].
- The flux estimator integrates `(v − R·i)·dt − L·Δi` in αβ, with saliency and
  λ adaptation where configured. A PLL extracts angle and speed. Readiness
  checks flux confidence, acceleration-compensated PLL error, minimum speed
  and sustained signed back-EMF consistency against `λ·ω`. Keep those gates
  distinct from the speed thresholds for blending. See [observer][observer].
- On the MK5, the observer normally uses phase voltages measured through the
  board's RC filters. The driver can fall back to reconstructed applied
  voltage. When evaluating phase lag, verify which voltage path was used and
  align the voltage and current with the same actuation interval. See [voltage
  input][voltage] and [MK5 board config][board].
- VESC 7.00 `foc_correct_hall` uses the configured PLL speed estimate as the
  blend reference. Its default PLL input is corrected commutation phase. It
  also interpolates Hall from sector timing and rate-limits the resulting angle.
  OxiFOC uses measured sector boundaries where available. The comparison
  below changed only the thresholds in OxiFOC. The VESC references are
  `motor/foc_math.c`, `motor/mcpwm_foc.c` and `motor/mcconf_default.h` from
  [VESC 7.00][vesc] (bench build adds telemetry patches through `1ad6eab3`),
  and `widgets/detectallfocdialog.cpp` from [VESC Tool][tool].

## Bench baseline

Firmware change `5505305`, H5055 200 Kv, seven pole pairs, no mechanical load.
24 V supply, 15,001.4 Hz PWM and injected ADC, native Packed28 over the C5
bridge. The phase trace records weight and readiness separately from the
native capture. Speeds here are electrical unless specified.

| Test | Result |
|---|---|
| 450 → 320 → 40 rad/s, two runs each | At 320, weight was 1.0 with 150–300 rad/s and about 0.37 with 2500–4000 eRPM (262–419 rad/s). Both returned to Hall near the bottom of their respective bands. Hall without crossover served as the reference. |
| 900 and 1500 rad/s, two runs per mode | Mean `hypot(Id, Iq)` was 0.75 and 0.91 A with observer, versus 1.77 and 2.54 A with Hall without crossover. The 900-rad/s averaging window includes the end of acceleration. |
| 2300 rad/s (~3140 shaft rpm), one run per mode | Median estimated speed was 2297/2299 rad/s and mean `hypot(Id, Iq)` was 1.01/3.24 A (observer/Hall). |
| Invalid Hall input `111` injected on target | At ~454 rad/s and ~320 rad/s (inside the VESC threshold band), the observer took over and `HallError` was reported. Torque was withdrawn on deceleration after angle trust was lost. |

The completed recordings had no missing samples or command acknowledgements.
One host scheduling abort was repeated. The two commutation modes reported
speed from different sources, so the displayed speed ripple cannot establish
which had smoother mechanical motion. The injected fault does not exercise
the wiring or a single stuck Hall channel. The [bench journal][journal]
points to the captures, plots, exact ELFs and the injection patch.

## Revisit in this order

1. Reproduce `450 → 320 → 40` through both crossover bands in both
   directions with controlled load. Keep the motor model, current/regen
   limits, PWM and velocity PI fixed. Compare Hall without crossover,
   150–300 rad/s, and 262–419 rad/s before changing the estimator. Use an
   independent speed reference if comparing mechanical ripple. Include a
   load step near each boundary and repeat each condition.
2. For any repeatable angle or torque transient, log Hall and observer
   angle/speed before blending, selected angle/speed, actual blend weight,
   PLL raw angle and compensated error, signed q-axis back-EMF, valid travel,
   phase voltage source, `Id`/`Iq`, current grants and fault transitions.
   Separate bad phase tracking from a premature or oscillating weight.
   Compare peak current, speed/angle excursions and time without trusted
   feedback. Only then compare a PLL-based weight with the current `max`
   rule, or isolate voltage alignment, motor parameters and PLL tuning.
3. Repeat with a switchable single Hall line while above and inside the
   blend band, then decelerate. Verify fault diagnosis, angle/torque
   continuity and withdrawal of torque when trust is lost. VESC's open-loop
   recovery is a separate riding-policy decision.

[selector]: ../../oxifoc-core/src/foc/phase/manager/select.rs
[fallback]: ../../oxifoc-core/src/foc/phase/manager/hall.rs
[observer]: ../../oxifoc-core/src/foc/phase/observer.rs
[voltage]: ../../oxifoc-core/src/motor/foc_driver/driver.rs
[board]: ../../oxifoc-f405/src/config.rs
[vesc]: https://github.com/vedderb/bldc/tree/9483d8cf
[tool]: https://github.com/vedderb/vesc_tool/tree/f4ec39a5714908bdecdf5aaf607857d7dd467c39
[journal]: foc-loop-research.md#hall-policy-and-high-speed-comparison
