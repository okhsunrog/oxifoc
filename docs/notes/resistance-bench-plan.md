# Resistance and applied-voltage bench comparison

## Goal and confirmed setup

Identify winding resistance separately from inverter voltage errors, and make
the voltage model used by OxiFOC consistent with its estimator. VESC 7.00 and
the manual LC1020E measurements are independent comparisons, not values to
force the OxiFOC result to match.

Confirmed on 2026-11-08: PSU 24 V, current limit 5.5 A, motor connected and
controller powered. Controller output colors are different from motor colors:

| Controller phase | Controller lead | Motor lead |
|---|---|---|
| A | Green | Yellow |
| B | Blue | Blue |
| C | White | Black |

This supersedes the previously unverified controller-to-color mapping. The
LCR measurement aliases remain yellow=A, blue=B, black=C. Raw LCR results:
`/home/okhsunrog/motor_control/vesc-bench/docs/motor-lcr-2026-11-08.md`.
At 1 kHz the latest between-lead maxima are 52.9/52.5/52.9 µH; at 10 kHz,
51.2/50.7/51.1 µH. The 100 Hz increase remains unexplained. AC ESR is not DC R.

Flashing and tests without motor excitation are authorized. A new excitation
series must be presented with its actual current limits and motion implications
before starting. No new full-flash backup is needed for each build. Export
current VESC motor/application configurations before changing firmware; retain
the original firmware backup and preserve OxiFOC's storage sectors.

## Prepared changes

- `c523849` fixes detection's wait for a genuinely fresh ADC/FOC pair. It has
  host regression coverage; device acceptance is a separate step.
- F405 injected ADC rank 3 acquires PA0/PA1/PA2 terminal voltages. Current rank
  1 and auxiliary rank 2, sample times, and TIM1 CH4 trigger are unchanged.
  At the configured 21 MHz ADC clock and 15+12 cycles per conversion, phase
  sampling is nominally 2571 ns after current sampling. JEOC/control execution
  moves later by one conversion, about 1286 ns. Hardware timing still needs
  verification; nominal aperture delay is not an RC-filter group delay.
- Capture TIM1 CCR1-3, ARR and channel/main enables before `run_foc_cycle`.
  These include prior programmed compensation/clamping. CCR reads can expose
  preload registers; they do not independently measure gate waveforms.
- Acquisition is diagnostic only: it does not switch the observer to measured
  voltage, change deadtime correction, or replace the R estimator.

## Diagnostic wire and file contract

ICD v7 uses a 34-byte raw-Pod fast frame: the original 18 bytes plus three raw
phase ADC values, three PWM compares, ARR, and output-enable bits. Fourteen
frames occupy 476 bytes; adding the conservative 32-byte framing allowance
still fits the 512-byte F405 MTU. Raw data alone at 20 kHz is 680 kB/s;
lossless operation must be measured on USB, not inferred from nominal bandwidth.

Missing phase samples are `[65535; 3]`; ARR=0 means PWM snapshot unavailable.
Enable bits 0..2 are high-side A/B/C, 3..5 low-side A/B/C, bit 6 main enable.

The frozen HardwareInfo schema stays unchanged. Capability bit 1 declares
phase capture, bit 2 enabled hardware phase filtering. Under bit 1, reserved
bytes 0..4 contain the phase divider ratio as f32 LE, bytes 4..8 contain the
nominal current-to-phase acquisition offset as u32 LE. No phase scale is
inferred from the bus-voltage divider. The host uses the shared
PhaseVoltageSense conversion, with the reported ADC reference and resolution.
The phase divider is nominal; per-channel calibration is not established.
The existing bus-only correction must not silently calibrate phase voltages.

Parquet retains raw phase counts and PWM registers alongside measured terminal
and alpha/beta/d/q voltages. The original `vd_v/vq_v` columns remain PI commands
before PWM compensation. Full BoardCalib and the acquisition descriptor are
saved in metadata. Measured projections use the reported cycle angle without
filter/aperture-delay correction. PWM-on readings from an unfiltered board are
not automatically average applied voltages. Samples from different rotor
positions or missing stream segments must not be joined into a fit.

## Staged measurements

1. **Idle acceptance.** Verify identity, firmware/host protocol agreement,
   calibration, stopped mode, output enables, current offsets, Vbus and both
   temperatures. Capture the enriched frame at stream rates 5 and 20 kHz with
   the 20 kHz PWM unchanged. Check sample count, sequence continuity, queue/link
   losses and ISR cost. A 5 kHz stream here is decimation, not 5 kHz PWM.
2. **Pilot at fixed 20 kHz PWM.** Once idle recording passes, prepare a short
   stationary-current sequence, initially no higher than 3 A, with explicit
   permission. Start with the smallest plateau and inspect it before expanding.
   Preserve independent protection thresholds; read and document them first.
   Record ramps, settled plateaus and zero-current tails, not only averages.
3. **Voltage definitions and polarity.** At fixed manual angle, compare both
   current signs at 0.6/1.5/3 A. Use the same physical alpha/beta direction in
   VESC and OxiFOC: their original R routines excite different axes. Fit signed
   `V = R*I + offset` using PI command and phase-ADC voltages separately. Retain
   cross-axis residuals, measured-current sum, drift and opposite-polarity
   differences. Do not replace the estimator merely to obtain the reference R.
4. **Controlled frequency sweep.** Use 5/10/15/20 kHz PWM with the same current,
   angle, PI gains, ramp, settling and averaging windows. Record every native
   cycle. Recheck 20 kHz VESC recording under excitation before including it;
   historical active 20 kHz captures were incomplete. Store exact firmware
   artifacts/configurations for each PWM rate. Repeat a baseline point at the
   end to expose heating or time drift.
5. **Separate compensation and inverter effects.** Compare compensation on/off
   with hardware deadtime fixed. Change only one variable per pair, include
   both current signs and optionally the orthogonal current axis. Increase
   current beyond 3 A only after current/voltage acquisition and protections
   are validated and the expanded series is approved. A thermal R estimate from
   VESC's high-current wizard is a different experiment from low-current fits.
6. **Dynamic L cross-check.** Use measured voltage and current transients or
   known small-signal excitation to compare inductance, angle and current
   dependence with the LCR range. A 20 kHz stream cannot independently resolve
   a 10 kHz sinusoidal waveform or PWM-carrier ripple; that comparison needs a
   faster/different acquisition path. Do not equate PWM rate with LCR test rate.

At each stage keep numerical summaries and visual analysis: time traces,
sample zooms, V/I scatter and signed fits, current-sum/cross-axis residuals,
FFT, Welch PSD and time-frequency spectrograms. Split spectral windows at
gaps/rate changes and report actual native cadence. A full matrix of runs is
not a substitute for matched conditions and validated acquisition.
