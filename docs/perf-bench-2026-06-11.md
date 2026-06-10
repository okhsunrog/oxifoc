# On-target math/estimator benchmarks — G431, 2026-06-11

Measured on the B-G431B-ESC1 (STM32G431CB @ 170 MHz, FPv4-SP-D16) via
`tests/stm32g431` (`cargo test`, embedded-test + probe-rs, DWT CYCCNT).
ISR budget at 20 kHz: **8500 cycles**.

Caveats:
- Test profile is `opt-level = "s"` + fat LTO; the g431 firmware ships
  `opt-level = "z"` + `build-std`. Close, not identical — relative
  comparisons hold, absolute ISR utilization should be re-checked with
  live ISR instrumentation eventually.
- The test crate's dev profile is configured identically to its release
  profile (`opt-level = "s"`, fat LTO), and a `--release` re-run matched
  within noise (e.g. FOC step 714 vs 745, HFI 13153 vs 12751) — the
  numbers below are from optimized code.
- All loops include ~6 cycles of loop/load/DWT-read overhead (see the
  baseline row); identical across rows, so deltas are clean.

## Primitives (per call, avg / min / max cycles)

| op | avg | min | max |
|---|---|---|---|
| baseline (loop + load + black_box) | 6 | 6 | 11 |
| `libm::sinf` + `libm::cosf` (pair) | **6204** | 5373 | 6986 |
| `FastSinCos::sin_cos` (pair) | 120 | 115 | 208 |
| `CordicSinCos::sin_cos` (pair) | **103** | 103 | 168 |
| `libm::sqrtf` | 110 | 110 | 206 |
| `vsqrt.f32` (inline asm) | **25** | 25 | 29 |
| `libm::atan2f` | 169 | 51 | 272 |
| `fast_atan2` (VESC-style poly) | **46** | 46 | 64 |

Notes:
- **`libm::sinf/cosf` are catastrophic on this target**: libm does its
  argument reduction and polynomial in `f64`, and our hardware-validated
  `-fp64` rustflag makes every f64 op a softfloat call. ~6200 cycles per
  pair vs ~100–120 for FastSinCos/CORDIC (50–60×).
- `vsqrt.f32` matched `libm::sqrtf` bit-exactly over the test sweep
  (IEEE correctly-rounded), 4.4× faster.
- `fast_atan2` max error vs libm over a unit-circle sweep: **0.0101 rad**
  (0.58°) — fed into a PLL that filters it; fine for the observer.
- CORDIC slightly beats FastSinCos and stays (also the q31 conversion
  path is hardware-validated). FastSinCos is the right tool where the
  CORDIC peripheral isn't owned (estimators, F405).

## Composites (per ISR cycle, avg / min / max cycles)

| path | avg | min | max | % of 8500 |
|---|---|---|---|---|
| `FocController::step` (Clarke+CORDIC+Park+2×PI+invPark+SVPWM) | 745 | 738 | 755 | 8.8% |
| `BackEmfObserver::update` | 1092 | 1009 | 1158 | 12.8% |
| `HfiObserver` `get_injection`+`update` | **12751** | 6876 | 13597 | **150%** |

## Conclusions (action list)

1. **HFI cannot run at 20 kHz as-is** — 150% of the ISR budget on its
   own. The cost is almost entirely the three `libm::sinf/cosf` calls
   per cycle (carrier synth in `get_injection`, demod + phase estimate
   in `update`, observer.rs:490/512/641). Replacing them with
   `FastSinCos` should bring the HFI slot to roughly **700–900 cycles**
   (~15×). This is a correctness-grade fix, not an optimization: an ISR
   overrun at 20 kHz means continuous ISR re-entry, a starved main loop,
   and (since the IWDG landed) a watchdog reset.
2. `BackEmfObserver`: `atan2f` → `fast_atan2` and `sqrtf` → `vsqrt.f32`
   (with a libm fallback for host builds) takes ~1092 → ~850 cycles.
   Worth doing in the same pass, not urgent by itself.
3. Worst-case sensorless cycle after the fix (controller + both
   estimator slots in a crossover regime): ~745 + ~850 + ~850 ≈ **2450
   cycles ≈ 29%** of budget — comfortable, with room for the velocity
   loop and derating later.
4. CORDIC stays for the controller path (user decision + it measured
   fastest).

Raw run: `tests/stm32g431`, all 12 tests pass, defmt output in the
session log of 2026-06-11.
