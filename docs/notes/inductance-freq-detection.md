# Frequency-dependent inductance — bench findings & detection status

Motor: ZD2808 700 KV sensorless drone outrunner, **wye** (confirmed below).
Board: B-G431B-ESC1 (STM32G431, 20 kHz FOC). Bench PSU 12 V / 4 A.
Session: 2026-06-13/14, branch `bench-detection-2026-06-13`.

## Bench LCR (Kelvin 4-wire), line-to-line

| pair | 1 kHz | 10 kHz |
|------|-------|--------|
| red–yellow | 54.2 µH, 0.188 Ω | 53.5 µH, 0.444 Ω |
| red–black  | 44.2 µH, 0.218 Ω | 43.3 µH, 0.402 Ω |
| yellow–black | 46.6 µH, 0.218 Ω | 45.9 µH, 0.418 Ω |
| **avg** | **48.3 µH, 0.208 Ω** | **47.6 µH, 0.421 Ω** |

These are **line-to-line** (two phase windings in series).

## Per-phase reduction (wye)

`L_ll = 2·L_phase`, `R_ll = 2·R_phase` for wye →

- **L_phase ≈ 24 µH** @ 1 kHz (48.3/2), essentially flat to 10 kHz (−1.5 %).
- **R_phase ≈ 0.104 Ω** @ 1 kHz, **≈ 0.21 Ω** @ 10 kHz.

**Wye confirmed by the R check**: our on-device 2-point R = 0.127 Ω (≈0.10 DC +
dead-time), and `2·0.10 = 0.20 ≈ bench R_ll`. Delta would need `R_ll = ⅔·R_phase`
— off 3×. (If it were delta, expect L_phase ≈ 72 µH.)

The dq inductance the FOC measures is the per-phase synchronous L, so **expect
`Ld ≈ 24 µH`** (AC).

## Physics: why L and R are frequency-dependent

Eddy currents in the laminated iron **and the conductive sintered NdFeB magnets**
exclude AC flux from the conductive volume (Lenz):

- **L drops DC→AC**: di/dt voltage-pulse (~DC) reads ≈ 89 µH/phase; by ~1 kHz the
  flux exclusion saturates and L plateaus at ~24 µH, flat 1→10 kHz.
- **R rises with f**: the same eddy loss appears as added series resistance
  (LCR reports `Re(Z)`); plus copper skin/proximity. Power ∝ f² → R roughly
  doubles 1→10 kHz. The flux-exclusion (L) saturates by ~1 kHz but the *loss* (R)
  keeps climbing — that is why L plateaus while R keeps rising.

We measure **R at DC** (2-point ΔV/ΔI, locked d-axis, steady state) and **L at AC**
— the right pairing for the current loop (`kp = L·bw` wants the AC L ≈ 24 µH, not
the 89 µH pulse value; `ki = R·bw` wants DC R).

## Production FFT inductance path — latent bug (NOT yet fixed)

`detection/inductance.rs` `InductanceMeasurement` (rotating HFI + FFT, bins 0/2)
demodulates by per-sample `inverse_l = f·di / v_inductive` with a carrier-zero
clamp that copy-forwards the previous sample. At the default carrier
**5000 / 20000 = exactly 1/4**, `hfi_phase` starts at 0 and steps π/2, so
`hfi_sin = 0,1,0,−1,…` — **half the 32-sample FFT window lands on carrier zeros →
copy-forward**. Period-2 artifact (energy at bin 16) + leakage; bin 0 (L_avg)
survives, bin 2 (saliency, Ld−Lq) gets extra noise.

Invisible in sim: the unit test runs the carrier at **1000 Hz** (1/20, never hits
the zeros); real detection uses the **5000 Hz** default (1/4). Real-HW-only.
Fix later (detune the default off 1/N, e.g. ~4.7 kHz) — or just prefer the
FFT-free `|Z|` method. Not needed on this non-salient drone (no saliency).

## Experimental `impedance-sweep` feature (this session)

Cargo feature `impedance-sweep` (⇒ `hfi-detect`). Replaces the L step with a
one-rotor-lock R(f)/L(f) sweep (`measure_impedance_sweep` in `sweep.rs`),
logging `(f, |Z|, R, L)` over RTT. Method: **correlation lock-in** (not FFT) —
per carrier, correlate current against `sin φ` (→R) and `−cos φ` (→ωL).

- `|Z|` magnitude is **rotation/phase-invariant → robust**.
- The **R/L phase split is fragile** (needs exact carrier phase).
- Sweep freqs are **fractions of f_sw**, detuned off integer ratios, top f_sw/4.3
  (≥4 samples/carrier-period); cannot reach the bench 10 kHz point (= f_sw/2,
  carrier degenerates).
- Fixes applied: whole-period accumulation (`round(16·spp_exact)`, was
  `16·round(spp)` → partial-period leakage corrupted the phase); report
  `L = √(|Z|²−R²)/ω` as the robust primary.

### Latest run (R=0.127 Ω) — still noisy, NOT trustworthy

| f Hz | |Z| meas | |Z| expected (24µH) | L\|Z\| |
|------|---------|--------------------|--------|
| 500  | 0.144 | 0.148 | 21.6 µH |
| 900  | 0.133 | 0.186 | 7.1 µH |
| 1680 | 0.220 | 0.283 | 17.1 µH |
| 2700 | 0.166 | 0.427 | 6.3 µH |
| 3700 | 0.284 | 0.572 | 10.9 µH |
| 4640 | 0.563 | 0.711 | 18.8 µH |

`|Z|` itself is non-monotonic and systematically low → **the magnitude is noisy,
not just the phase**. Returned L = 18.8 µH (top point).

### Suspected causes (UNCONFIRMED — need spectrum)

1. **Rotating-injection beat**: the sweep reuses the production `HfiInjector`,
   which rotates the injection vector at `f_sw/32 = 625 Hz` (for saliency). At
   low carriers (500–900 Hz) this beats with the carrier and corrupts the lock-in.
   For a fixed-axis impedance sweep, inject on a **fixed d-axis** (no rotation).
2. Short windows (16 carrier periods) → high variance.
3. Residual fixed carrier-phase offset (~20° at 5 kHz, ADC/filter delay) on top
   of the half-step de-rotation → R/L split miscalibrated (|Z| survives).

## Resistance fix (DONE, reliable)

`measure_resistance` settle gate added an absolute floor `SETTLE_TOL_FLOOR_A =
0.15 A`: the bare 30 %-of-setpoint tolerance is `0.03 A` at the 0.1 A probe
setpoint — smaller than the dead-time current offset (~0.04 A) — and rejected a
good 2-point measurement (`UnexpectedMotion` → `HardwareFault`). 2-point ΔV/ΔI is
offset-robust, so a loose low-point landing does not bias R. Now reads
**0.127 Ω** stably.

## Next steps

1. **Spectrum diagnostic** — capture raw fast telemetry at full FOC rate (20 kHz)
   during the sweep and FFT it offline (Python) to *see* the current response:
   carrier cleanliness, noise floor, the 625 Hz rotation beat, harmonics, rotor
   wiggle. **Blocked on realtime 20 kHz streaming** (separate workstream — current
   telemetry tops out at ~2 kHz over UART@921600 because the frame is 44 B; needs
   a compact frame + RTT/higher baud).
2. Switch the sweep to **fixed d-axis injection** (drop the 625 Hz rotation).
3. Phase calibration for trustworthy R(f).
4. Re-run, overlay on the bench R(f)/L(f) curve.
