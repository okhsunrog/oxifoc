# B-G431B-ESC1 Hardware Reference

ST reference: MB1419, Rev B (26-March-2019), variant G431CBU6.

Schematic: [mb1419-g431cbu6-b01_schematic.pdf](mb1419-g431cbu6-b01_schematic.pdf)

## MCU

- **STM32G431CB** (LQFP48, Cortex-M4F, 170 MHz, hardware FPU, CORDIC)
- 128 KB Flash, 32 KB SRAM
- 8 MHz HSE crystal (Y2) on PF0/PF1

## Power Supply

- **V+**: Battery input (motor supply)
- **L7986TR** (U9): V+ → +10V buck converter (33 uH inductor L2)
- **LDFPVRs** (U12, U14): +10V → 5V_ESC (two separate 5V rails for analog/digital)
- **LD3905PU5R** (U7): 5V_ESC → +3.3V MCU supply
- **LD1117S50TR** (U3, daughterboard): +10V → +5V for ST-Link
- Bulk input capacitors: 14x 15 uF (C27-C40) on V+

## Power Stage

- **6x STL180N6F7** MOSFETs (Q2-Q7): 60 V, 120 A STripFET F7, PowerFLAT 5x6
  - Rds(on): 1.9 mohm typ / 2.4 mohm max (Vgs = 10 V, Id = 16 A)
  - Vgs(th): 2-4 V, Qg: 79.5 nC (Vdd = 30 V, Id = 32 A)
  - Switching: td(on) = 34 ns, tr = 36 ns, td(off) = 69 ns, tf = 42 ns
  - Id continuous: 120 A (Tc = 25 C), 32 A (Tpcb = 25 C), 20 A (Tpcb = 100 C)
  - Rth(j-pcb): 31.3 C/W
  - Body diode: Vsd = 1.2 V max, trr = 60 ns
- **3x L6387ED** gate drivers (U10, U11, U13): high/low side with internal bootstrap diode
  - Vcc: up to 17 V, supplied from +10V rail
  - Drive current: 400 mA source / 650 mA sink
  - Propagation delay: ton = 110 ns, toff = 105 ns
  - Output rise/fall: 50/30 ns (1 nF load)
  - Bootstrap diode Rdson: 125 ohm (internal DMOS)
  - UVLO: turn-on 6 V, turn-off 5.5 V (0.5 V hysteresis)
  - Hardware interlock: HIN=1 + LIN=1 → both outputs off
  - CMOS/TTL Schmitt trigger inputs with pull-down
- **Shunt resistors**: R54, R55, R56 = 3 mohm, 3 W each
- **Bootstrap caps**: 100 nF per phase (C47/C53/C60 on schematic)
- Output peak motor current: 40 A (per ST datasheet)
- No hardware overcurrent protection circuit on board

### PWM Channels (TIM1)

| Channel       | Pins       | Phase |
|---------------|------------|-------|
| TIM1_CH1/CH1N | PA8 / PC13 | A     |
| TIM1_CH2/CH2N | PA9 / PA12 | B     |
| TIM1_CH3/CH3N | PA10 / PB15| C     |

## Current Sensing

Three-phase inline shunt sensing via internal OPAMPs configured as PGA.

| Phase | OPAMP  | Non-inv (+) | Inv (-) | Output |
|-------|--------|-------------|---------|--------|
| A     | OPAMP1 | PA1         | PA3     | PA2    |
| B     | OPAMP2 | PA7         | PA5     | PA6    |
| C     | OPAMP3 | PB0         | PB2     | internal |

Shunt sensing resistor network: 22k pull-up to 3.3V, 1.5k series, 2.2k to ground (per phase).

## Analog Inputs

| Pin  | Signal                | Notes                                |
|------|-----------------------|--------------------------------------|
| PA0  | VBUS voltage          | Resistor divider from V+, BAT30KFILM protection |
| PB14 | NTC temperature       | 10k/NTC divider, 10 nF filter (C66) |
| PB12 | Potentiometer (speed) | 10k pot (R2) to 3.3V                |

## BEMF Detection (six-step)

| Pin  | Signal | Protection  |
|------|--------|-------------|
| PA4  | BEMF1  | BAT30KFILM  |
| PC4  | BEMF2  | BAT30KFILM  |
| PB11 | BEMF3  | BAT30KFILM  |
| PB5  | GPIO_BEMF enable | Controls BEMF divider power |

Voltage dividers: 2.2k series + 10k to ground per phase, with Schottky clamp diodes to 3.3V.

## Hall / Encoder Inputs

| Pin | Signal   | Notes                           |
|-----|----------|---------------------------------|
| PB6 | H1 / A+ | 10k pull-up, BAT30SWFILM protection |
| PB7 | H2 / B+ | 10k pull-up, BAT30SWFILM protection |
| PB8 | H3 / Z+ | 10k pull-up, BAT30SWFILM protection |

Powered from 5V_ESC. Filter caps: C67, C68, C69 = 10 uF each.

## CAN Bus

- **TCAN330DCNT** (U2): 3.3V CAN transceiver
- **CAN_TX**: PB9, **CAN_RX**: PA11
- **AS11P2TRQ** (U1): Analog switch for 120 ohm termination
- **CAN_TERM**: PC14 (controls termination relay)
- **CAN_SHDN**: PC11 (transceiver shutdown, active low)
- Protection: STPS1L20MF diode (D1), 250 mA fuse (F1)

## Debug / Daughterboard (ST-Link)

- Integrated ST-Link via **STM32F103CBT6** (U6)
- USB micro-B connector (U4) for programming and VCP
- **USART2 VCP**: PB3 (TX), PB4 (RX) — virtual COM port via ST-Link
- Communication with target: RTT (Real-Time Transfer) via probe-rs
- Bicolor LED: green/red (LED_STLINK)
- Power LED: green (D3, LNJ347W83RA)

## User Interface

| Pin  | Signal      | Notes                    |
|------|-------------|--------------------------|
| PC6  | STATUS LED  | Active low               |
| PC10 | User button | Active low, debounce cap |
| PA15 | PWM input   | External PWM command     |

## Test Points

| Pin  | Label |
|------|-------|
| PB1  | TP3   |
| PC11 | TP2 (shared with CAN_SHDN) |

## Full Pinout

| Pin           | Signal               |
|--------------:|----------------------|
| VBAT          | 3V3                  |
| PC13          | TIM1_CH1N            |
| PC14          | CAN_TERM             |
| PC15          | N.C.                 |
| PF0/OSC-IN    | HSE 8 MHz            |
| PF1/OSC-OUT   | HSE 8 MHz            |
| PG10/NRST     | RESET                |
| PA0           | VBUS                 |
| PA1           | Curr_fdbk1_OPAmp+   |
| PA2           | OP1_OUT              |
| PA3           | Curr_fdbk1_OPAmp-   |
| PA4           | BEMF1                |
| PA5           | Curr_fdbk2_OPAmp-   |
| PA6           | OP2_OUT              |
| PA7           | Curr_fdbk2_OPAmp+   |
| PC4           | BEMF2                |
| PB0           | Curr_fdbk3_OPAmp+   |
| PB1           | TP3                  |
| PB2           | Curr_fdbk3_OPAmp-   |
| PB10          | N.C.                 |
| PB11          | BEMF3                |
| PB12          | POTENTIOMETER        |
| PB13          | N.C.                 |
| PB14          | Temperature feedback |
| PB15          | TIM1_CH3N            |
| PC6           | STATUS LED           |
| PA8           | TIM1_CH1             |
| PA9           | TIM1_CH2             |
| PA10          | TIM1_CH3             |
| PA11          | CAN_RX               |
| PA12          | TIM1_CH2N            |
| PA13          | SWDIO                |
| PA14          | SWCLK                |
| PA15          | PWM input            |
| PC10          | BUTTON               |
| PC11          | CAN_SHDN / TP2       |
| PB3           | USART2_TX            |
| PB4           | USART2_RX            |
| PB5           | GPIO_BEMF            |
| PB6           | H1 / A+              |
| PB7           | H2 / B+              |
| PB8           | H3 / Z+              |
| PB9           | CAN_TX               |

---

## Firmware bringup log

### 2026-06-13 — first sensorless bench (ZD2808 700 KV), commit `e7d45a4`

First real-hardware run of the oxifoc-g431 firmware on a **sensorless** motor.
Motor: ZD2808 700 KV multirotor outrunner, 7 pole pairs (12N14P), rotor free.
Supply: **12 V / 4 A** lab PSU (CV 12 V, CC 4 A — the CC limit is the real
physical backstop). Baked config = lab-PSU-safe profile (`RampToZero` failsafe,
`bus_regen_max_a = 0`). No hall sensors → a Warning-level HallError is expected
and does not block (detection does not use the hall).

#### HW overcurrent (COMP1/2/4 + DAC3) false-trips at idle — DISABLED

Symptom: on power-up the firmware latches a **Kill OverCurrent at 0 A** (PWM
disabled). It re-asserts immediately after a host clear, so the device sits in
the Error latch and refuses motor commands.

Root cause (register dump via a temporary diagnostic in
`init_overcurrent_protection`):
- **DAC3 is correct**: `CR=0x0001_0001` (both channels EN), `MCR=0x0003_0003`
  (both MODE=0b011, on-chip no-buffer), `SR=0x0800_0800` (both DACRDY),
  `DOR1=DOR2=0x149`=329 → both channels output ≈265 mV.
- **COMP1/2/4 CSR = `0x4000_0041`**: bit30 VALUE=1 on all three (output high =
  break asserted), INMSEL=0b100 (DAC3), INPSEL=0 (INP0). So all three
  comparators see their **+ input above the 265 mV threshold at idle**.
- A DAC sweep confirms the comparators DO track DAC3 (VALUE=0 at DAC=4095,
  VALUE=1 at DAC=0) — routing is fine; the **threshold is simply too low**.

So `config.rs::overcurrent_dac_counts` mis-models the comparator input. It
assumes the COMP sees the shunt node attenuated ×4/7 + 127 mV bias (≈127 mV
idle, 264 mV at 80 A). The register evidence says the real idle level at the
COMP + input (PA1/PA7/PB0, shared with OPAMPx_VINP) is **above 265 mV**. The
current-sense network documented above (22k→3.3 V / 1.5k series / 2.2k→GND)
gives ~128 mV on its own, so the model is missing a term (OPAMP PGA
interaction at the shared pad?) or points at the wrong node.

**TODO next session — RESOLVE WITH THE SCHEMATIC**
([B-G431B-ESC1_schematic.pdf](B-G431B-ESC1_schematic.pdf)): trace the actual
COMP INP0 node and its true idle bias, recompute the DAC3 threshold, and check
polarity vs `invert_current_sign` (positive motor current drives the sense node
*down*, so the trip may need INVERTED polarity / a low-side threshold). Then
re-enable and bench-validate the trip at a known current.

Fix applied: `motor.rs` `set_break_enable(false)` — HW OCP off until corrected.
Protection meanwhile: the software measured-overcurrent trip
(`max_phase_current_a`) + the bench PSU current limit.

#### Detection results vs LCR / nameplate

LCR (Kelvin 4-wire, line-to-line): R_LL ≈ 0.21 Ω @1 kHz, L_LL ≈ 44–54 µH
(position spread). Per-phase (wye) ≈ R 0.105 Ω, L 24 µH.

| Param | Measured | Expected | Ratio | Note |
|---|---|---|---|---|
| R / phase | 0.127 Ω | ~0.105 Ω | 1.2× | residual 800 ns dead-time — OK |
| Ld | 86 µH | ~24 µH | **3.6×** | inflated |
| Lq | 122 µH | ~24 µH | **5.1×** | inflated |
| λ | 1.30 mWb | ~1.13 mWb (from 700 KV) | 1.15× | mild |
| Kv | 1051 RPM/V | 700 (nameplate) | **1.50× ≈ √3** | normalization |

Two systematic errors, both biasing HIGH:
1. **L badly inflated** — the g431 voltage-pulse L step: for a low-L motor the
   pulse voltage needed is small, so the ~0.38 V (800 ns) dead-time distortion
   dominates `V` in `L = (V − R·i)·dt/di` → L overestimated 3.6–5×.
2. **Kv off by √3** (≈1.5×) — `Kv = 60/(2π·λ·Pp)` omits the √3 phase/line
   factor (and λ itself reads +15 %). Decompose vs the SVPWM
   amplitude-invariance convention.

Implication: a sensorless spin on the **as-measured L** gives 3.6× hot
current-PI gains (`kp = L·bw`) and a biased observer `−L·Δi` term → **fix L (or
feed the LCR value ~24 µH) before trusting closed-loop sensorless.**

#### Runaway incident + lesson

`detect <step> --record` streams telemetry at the FOC rate (20 kHz × 44 B ≈
880 KB/s), flooding the 921600-baud VCP (~92 KB/s); the detect response gets
stuck behind the backlog, the host appears to hang, and — critically — the
**device-side detection task keeps driving the motor after the link drops** (it
runs to completion regardless). Killing the host left the motor oscillating; a
**probe reset** (boots to `emergency_stop`) was the reliable stop. Lessons:
(1) never FOC-rate `--record` over UART; (2) detection should abort the drive on
host link-loss; (3) keep the probe attached as the abort button.

#### Transport / telemetry capture

- **ergot over UART VCP** (`transport-uart`, default) — reliable for commands;
  telemetry tops out ~2 kHz (link bandwidth).
- **ergot over RTT** (`transport-rtt` feature) needed a host fix: the attach did
  a `ScanRegion::Ram` sweep that finds STALE `_SEGGER_RTT` control blocks left
  by previous firmware images in uninitialized CCMRAM/SRAM2 (reset doesn't clear
  them) → "multiple control blocks". Fixed by pinning to the live `_SEGGER_RTT`
  ELF symbol (`ScanRegion::Exact`) in host-lib (both attaches) AND flashprobe-mcp
  (the `monitor` path) — commit `e7d45a4` / flashprobe-mcp `41e14a2`.
- **RTT bandwidth on THIS ST-Link ≈ 1.8 kHz effective** (5 kHz request → 37 %
  arrived; `NoBlockSkip` overflow corrupts COBS frames → ergot decode errors).
  Not faster than UART here. Full-rate capture needs device-RAM burst-capture
  (see docs/TODO.md). The parquet pipeline itself is validated (clean schema,
  5 kHz timebase, M=4 decimation, python-analyzable).

#### Next session

1. Fix the HW OCP threshold + polarity from the schematic, re-enable, validate.
2. Fix the voltage-pulse L (dead-time) and the √3 in λ/Kv → trustworthy params.
3. Sensorless cold-start spin attempt with corrected params + gentle limits.
