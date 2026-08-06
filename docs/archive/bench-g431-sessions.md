# g431 bench sessions (2026-06-13, 2026-07-05) — archived

Moved out of TODO.md when g431 (B-G431B-ESC1) support was dropped on
2026-08-06 (decisions.md). Board notes: [b-g431b-esc1.md](b-g431b-esc1.md);
session protocol: [bench-protocol-g431.md](bench-protocol-g431.md).

### Bench session 2026-07-05 (g431 + ZD2808) — detection re-measured, with recording

First detection run with full telemetry recording (the original goal of the
RTT pipeline work). Setup: B-G431B-ESC1 + ZD2808 (wye, 7 pp), 12 V / 4 A PSU,
firmware at 295a800 (transport-rtt + detection), `--record-hz 10000` (M=2,
loss-free; M=1 loses ~half the samples during detection — see the ISR-load
item under Size / performance).

| step | 2026-07-05 | 2026-06-13 | notes |
|------|------------|------------|-------|
| R    | **0.1271 Ω** (recorded, 71 k rows 0 gaps) / 0.1273 no-record | 0.127 | ΔV/ΔI recomputed from the capture = 0.127 ✓; LCR 0.104/ph + dead-time |
| Ld/Lq | **85.7 / 129.4 µH** (no-record — MUST run unrecorded, see Firmware/core item) | 86 / 122 | pulse ≈ DC L; AC L stays ~24 µH (eddy currents, known) |
| λ    | **1.2786 mWb** (recorded, 111 k rows 0 gaps, spin confirmed 700 eRPM) | 1.30 | expected ≈1.13 → bias REPRODUCED, then RESOLVED (below) |
| Kv   | **616 RPM/V** | 611 (after √3 fix) | nameplate 700 |

Everything reproduces June within ~2 % — June's numbers were solid (they were
taken without recording, so the ISR-trigger-loss bug never touched them).
Captures: `captures/detect-r-10k.parquet`, `detect-flux-10k.parquet`
(`detect-l-10k.parquet` exists but its device result is the distorted one).

**λ +13 % bias RESOLVED (same session, offline analysis of the capture +
validation run).** The back-EMF-vector λ at one speed carries an additive
`V_err/ω` bias: regressing the capture's per-window λ against 1/ω over the
ramp (iq held at 2 A) gives a clean fit (rms ~1 %) with **λ_true =
1.145 mWb** (intercept) and **V_err ≈ 9 mV** — the residual bridge/dead-time
error after compensation (0.38 V raw → 9 mV ≈ 97.6 % cancelled). At the
default `--erpm 700` the BEMF is only ~0.09 V (vs R·i ≈ 0.25 V), so 9 mV =
+12 %; the measurement regime is just too slow for a 12 V low-λ drone motor.
Validation: `detect flux --erpm 2800` → device reported **1.1673 mWb / Kv
675**, within 0.8 % of the model's prediction (1.176). True Kv ≈ **688** —
1.7 % off the nameplate 700. Motor and firmware math are fine.
- [ ] flux step default spin speed — kept as an open item in TODO.md → Bench.

### Bench session 2026-06-13 (g431 + ZD2808 700 KV sensorless) — findings

First real-hardware run of the g431 firmware on a sensorless drone motor
(ZD2808, 700 KV, 7 pp; 12 V / 4 A lab PSU). Detection ran end-to-end:

- [x] **HW comparator OCP — RESOLVED 2026-06-13: unusable on this board, break
  disabled.** Proven by on-device DAC sweep + stm32-data + host PWM test (full
  account in b-g431b-esc1.md). COMP1/2/4 tap the *raw shunt pad* (idle
  128 mV, slope only R_shunt×4/7 ≈ 1.71 mV/A — the ×16 PGA is downstream, so the
  comparator never sees the amplified signal), not the op-amp output. No current
  threshold clears the PWM switching-noise band; even ST's near-rail DAC=4083
  (≈3.29 V), with the break enabled, trips to Error on the *first* PWM-output
  enable every time (capacitive gate-drive transient spikes the hi-Z pad to the
  rail) → the motor can't start. ST parks the DAC at the rail (≈1850 A-equiv =
  effectively off) and relies on software OCP; so do we. `set_break_enable(false)`
  in `motor.rs`; COMP+DAC still configured near-rail for one-line re-arm *if*
  ST-style enable-sequencing (boot-cap-charge + non-fatal enable-window break) is
  added. Real protection = software measured-OC (40 A, ×9.14 ADC) + PSU CC.
- [x] **Detection biases — ALL three explained/fixed** (kept as a pointer;
  the original item's dead-time theory for L was WRONG). (1) L 3.6–5×
  "high": the pulse method measures ~DC L and L is genuinely
  frequency-dependent (eddy currents, NOT dead-time — disproven on HW
  2026-06-13c, see
  [notes/inductance-freq-detection.md](notes/inductance-freq-detection.md));
  the remaining gap is control-grade AC-L from detection — open item in
  «Algorithms». (2) λ +15 %: resolved 2026-07-05 — additive `V_err/ω`
  bias at the slow default spin, λ_true = 1.145 mWb (see the 2026-07-05
  bench section above). (3) Kv ×√3: fixed 2026-06-13 (`calculate_kv`
  carries the phase→line factor; verified 616–688 vs nameplate 700).
