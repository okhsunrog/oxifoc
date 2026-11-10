# F405 clock and memory placement bench, 10–11 November 2026

Board: MK5 UID `2B002C001047343031363330`, 24 V PSU. All experimental
images compile in `bench-no-phase-outputs`: the controller calculates current
control, but TIM1 CH1–3 and complementary outputs remain disabled. No motor
rotation or energized-loop stability is qualified here. NVM settings are unchanged.

Artifacts (exact ELFs, source diff, RCC/FLASH/ADC/SPI/TIM1 readbacks, RTT,
Packed28 Parquet, command acknowledgments and JSON summaries):
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-overclock-2026-11-10/`.
`analysis.json` additionally checks every recorded sequence modulo 16 bits,
hardware phase-enable bits, session END totals and both queue-drop counters.

## Clock qualification

RCC-derived frequencies use the previously measured HSE estimate of 7,944,893 Hz;
they are not new independent oscilloscope measurements. APB1/APB2 stay within
42/84 MHz and ADC within 36 MHz. PLLQ stays at or below 48 MHz; STM32 USB is absent.
TIM1 and the ADC trigger remain approximately 20 kHz despite the core-clock change.

Each full-stress step records 20 seconds of 20 kHz Packed28 over Ergot/SPI/C5,
while a cooperative task writes/verifies an 8 KiB SRAM buffer, reads a 2 KiB
flash pattern, checks FPU arithmetic and samples the internal die sensor.
This checks a bounded workload, not every memory cell or every instruction.

| Requested MHz | RCC-derived MHz | Maximum die °C | FLASH WS |
|---:|---:|---:|---:|
| 168 | 167.836 | 39.7 | 5 |
| 180 | 179.753 | 40.3 | 5 |
| 192 | 191.671 | 40.7 | 6 |
| 204 | 203.588 | 40.7 | 6 |
| 216 | 215.505 | 41.3 | 7 |
| 228 | 227.423 | 41.9 | 7 |
| 240 | 239.340 | 41.6 | 7 |
| 252 | 251.257 | 43.2 | 7 |
| 260 | 259.202 | 42.9 | 7 |

All nine completed these short integrity/transport tests without observed
MCU faults or telemetry loss. This does not establish a production frequency
limit: STM32F405 is rated for 168 MHz. Above 240 MHz even flash access exceeds
its nominal 30 MHz per wait-state interval (32.40 MHz at the last step).

The pinned Embassy `unchecked-overclocking` feature initially still rejected
PLLVCO >432 MHz before programming the PLL. Those failed attempts at 228 MHz
were a HAL assertion, not a hardware limit. `bench_prepare_embassy.py` makes a
private copy of the same pinned HAL, changes that assertion to `rcc_assert!`
and caps experimental F4 FLASH latency at the hardware's representable WS7.
It uses a separate lockfile; normal dependencies and Cargo's source cache are
untouched. Normal builds retain upstream clock checks and normal wait states.

## Current-control timing

Sparse background checks (`light`), two runs per setting using
`maneuvers/mk5-hall-hold-check.json`, 20 kHz recording, phase outputs inhibited.
Representative duration is the median of the two largest one-second mean
cycle counts from each run (four windows total), divided by RCC-derived HCLK.
It is not a worst-case execution-time bound. Maxima include transition windows.

| Requested MHz | Representative ISR µs | Observed maximum µs |
|---:|---:|---:|
| 168 | 36.76 | 45.48 |
| 204 | 33.82 | 41.87 |
| 216 | 35.16 | 43.42 |
| 240 | 31.65 | 38.98 |

These runs had no measured ISR-body overruns. Entry-latency/ADC-trigger losses
are not independently measured by `isr-timing`. The 168 MHz current baseline
uses normal APB divisors /4,/2; higher steps use /8,/4. The original full-stress
168 MHz stage used reduced buses, so it is not the timing baseline above.
The non-monotonic result shows why core frequency alone cannot predict the gain.

## Memory placement

F405 CCM is data-only and inaccessible to DMA. Executable code goes in ordinary
SRAM. `ram-foc` places ADC, `run_foc_cycle`,
`update_phase_with_applied_voltage` and `step_with_injection` in
`.data.ram_foc`; cortex-m-rt copies them before interrupts are enabled. Other
callees and constants can still reside in flash. ELF symbols verify placement.
`ccm` separately moves `STATE` and `FOC_DRIVER` (7,792 bytes) to CCM; stack and
DMA buffers stay in SRAM. ART prefetch and caches remain enabled in every case.

| Requested MHz | Code / state | Representative ISR µs | Observed max µs | Body overruns |
|---:|---|---:|---:|---:|
| 168 | Flash / SRAM | 36.76 | 45.62 | 0 |
| 168 | Flash / CCM | 36.81 | 45.72 | 0 |
| 168 | SRAM / SRAM | 40.62 | 50.69 | 2 |
| 168 | SRAM / CCM | 40.68 | 50.60 | 2 |
| 240 | Flash / SRAM | 31.65 | 38.98 | 0 |
| 240 | SRAM / CCM | 29.72 | 37.56 | 0 |

At 168 MHz, moving this hot set to SRAM is about 10.7% slower and crosses
the 50 µs body budget twice per variant across two runs. Moving state alone
to CCM is effectively neutral in this workload. At 240 MHz the combined
SRAM/CCM placement is about 6.1% faster than flash at that clock, or 19.2%
faster than the stock-clock flash baseline. Each current run's two selected
mean windows agrees closely with its repeat (see `analysis.json`).

The SRAM code set occupies about 38 KiB, reducing ordinary RAM/stack headroom.
Ordinary SRAM instruction fetches use the System bus, whereas Flash has ART;
that explains why SRAM placement need not win. Bus stalls/cache effects were
not separately measured. This experiment does not test SRAM remapping to the
code-region alias, a CCM stack, or an entirely flash-independent ISR.

All placement captures have complete Packed28 sessions, zero sequence gaps
and zero producer/host drops. Phase-enable readbacks remain zero. Both
experimental placement and overclock features remain off in the normal build;
real energized current/velocity control, temperature extremes and long-duration
qualification remain necessary before selecting an overclocked runtime profile.

## Reproduction

Run recipes from the repository's direnv development shell:

```
just bench-mk5-overclock OUTPUT_DIR
just bench-mk5-memory OUTPUT_DIR KNOWN_NORMAL_ELF 168
just bench-mk5-memory-step OUTPUT_DIR KNOWN_NORMAL_ELF 240 sram-ccm
just flash-mk5-spi
```

The harness restores the known normal ELF after every stage and on failure.
Experimental features remain default-off; the normal SPI build uses the stock
clock profile. See the artifact summaries for failures and restoration evidence.
