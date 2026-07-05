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

### 2026-07-05 — detection re-measured with recording; two June mysteries closed

Full detection re-run on the ZD2808 with loss-free 10 kHz telemetry capture
(`detect --record --record-hz 10000`, commits e1f65b5…d15f671). Everything
below in the 2026-06-13 log reproduces within ~2 %, and two of its open
questions are now RESOLVED:

- **λ "+15 %" was a measurement-regime bias, not a motor/firmware property**:
  the back-EMF-vector λ carries an additive `V_err/ω` term (V_err ≈ 9 mV of
  residual bridge error after dead-time comp). At the default 700 eRPM spin
  the BEMF is only ~0.09 V → +12 %. Regression over the recorded ramp gives
  **λ_true = 1.145 mWb**; validated by `detect flux --erpm 2800` → 1.167.
  True Kv ≈ **688 RPM/V** (noname nameplate: 700). TODO carries the fix
  (scale spin speed / multi-speed extrapolation).
- **The RTT attach flakiness was host-side** (unjoined RTT I/O thread killed
  mid-USB-transaction wedged the ST-Link; the board always booted fine) —
  fixed in ca635b4, 15/15 back-to-back attaches after.

Params baked into `baked_config.rs` (R 0.127 Ω, AC L 24 µH, λ 1.145 mWb,
7 pp); the board now boots on the back-EMF observer. First two sensorless
spin attempts made — startup engages but bringup is unfinished, see
TODO/memory (`project_sensorless_bringup`).

### 2026-06-13 — first sensorless bench (ZD2808 700 KV), commit `e7d45a4`

First real-hardware run of the oxifoc-g431 firmware on a **sensorless** motor.
Motor: ZD2808 700 KV multirotor outrunner, 7 pole pairs (12N14P), rotor free.
Supply: **12 V / 4 A** lab PSU (CV 12 V, CC 4 A — the CC limit is the real
physical backstop). Baked config = lab-PSU-safe profile (`RampToZero` failsafe,
`bus_regen_max_a = 0`). No hall sensors → a Warning-level HallError is expected
and does not block (detection does not use the hall).

#### HW overcurrent (COMP1/2/4 + DAC3) — unusable on this board, DISABLED

> **2026-06-13 (later): the first diagnosis below was WRONG. Resolved with an
> on-device DAC sweep + silicon data + a host PWM test. Corrected account:**

**The comparators tap the RAW shunt pad, not the op-amp output.** COMP1/2/4 INP0
= PA1/PA7/PB0, which is the OPAMP *input* (`OPAMPx_VINP`), not its output.
Silicon-confirmed (stm32-data): COMP1 INP0=PA1 / INP1=PB1(=TP3), COMP2 INP0=PA7
/ INP1=PA3 — the op-amp outputs (PA2/PA6/internal) are not on any COMP input, so
there is **no path** to feed the amplified signal to the comparator. ST MCSDK
uses the same `LL_COMP_INPUT_PLUS_IO1` = PA1.

**On-device DAC sweep at true idle** (1-bit ADC: vary DAC3, read COMP `VALUE`):
the flip is at **C1=160 / C2=164 / C4=164 counts = 128–132 mV** — exactly the
`×4/7 + 127 mV` pad bias. The op-amp-output hypothesis (≈2.057 V) is ruled out
16×. So the comparator's current slope is only `R_shunt × 4/7` ≈ **1.71 mV/A**
(the ×16 PGA gain that gives the ADC its 27.4 mV/A is *downstream*, invisible to
the comparator). A useful current threshold (e.g. 60 A → ~231 mV, ~100 mV over
idle) therefore sits *inside* the PWM switching-noise band on the raw shunt.

**The earlier "trip at idle" was PWM switching noise, not the threshold.** At
*true* idle the old DAC=329 (265 mV) is comfortably **above** the 128 mV pad, so
the sweep reads VALUE=0 (no trip). The prior register dump that showed VALUE=1 at
DAC=329 was taken with the FOC loop running (PWM switching), not at rest — the
raw shunt pad picks up switching transients well past 265 mV. (This matches the
very first 2026-03 finding: "comparators trigger on PWM switching noise".)

**MCSDK effectively disables it.** `M1_DAC_CURRENT_THRESHOLD = 4083` (≈3.29 V) on
the 128 mV / 1.71 mV-per-A pad node ≈ a **1850 A** trip → the comparator never
fires on current. ST parks it at the rail and relies on the **software** OCP
(read from the ×9.14-amplified ADC, good SNR). The "45 A" in MC Workbench is the
op-amp-output-domain number; on this board's raw-pad comparator it lands at the
rail.

**We tried near-rail (4083) + break enabled — it's worse than disabled.** Host
test: `voltage --vd 0 --vq 0` (PWM on, zero current) latched **Error /
OverCurrent on the FIRST output-enable, every time, before any current flows** —
capacitive coupling from the gate-driver turn-on transient spikes the
high-impedance pad node to the rail. With the break armed the motor cannot even
start. ST avoids this because MCSDK sequences the enable through a controlled
boot-cap-charge phase and does not latch an enable-window break as fatal; we
don't replicate that.

**Resolution:** `motor.rs` `set_break_enable(false)` — the COMP→BKIN break stays
OFF. The COMP+DAC are still configured at the near-rail value
(`config::HW_OCP_DAC_COUNTS = 4083`) so re-arming is a one-liner *if* ST-style
enable-sequencing is ever added. Real protection: the **software**
measured-overcurrent trip (`BOARD.max_phase_current_a` = 40 A, from the
×9.14-amplified ADC signal) + the bench PSU current limit. Verified: with the
break off the device boots clean (no OverCurrent) and `voltage 0 0` enters
Running without tripping.

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

#### 2026-06-13 (item 2): L is frequency-dependent — the "dead-time" theory was WRONG

> The "L inflated 3.6× by dead-time" diagnosis above is **superseded**. Fixing
> the voltage-pulse to be dead-time-immune (measure the step above the settled
> hold) **barely moved L** (89→89 µH): the residual dead-time, measured on
> hardware, is only **0.028 V** — the firmware's `set_dead_time_comp` already
> cancels most of it. So dead-time is not the cause.

**The L gap is genuine frequency-dependence of the inductance.** Three methods at
three frequencies, on the same motor:

| Method | Effective freq | Ld |
|---|---|---|
| voltage-pulse (di/dt, slow ramp) | ~DC | **89 µH** |
| LCR (bench) | 1 kHz | **24 µH** |
| HFI \|Z\| probe (FFT-free) | 5 kHz | **10.8 µH** |

L drops monotonically with frequency — eddy currents in the stator iron + the
conductive NdFeB rotor shield the AC flux (≈8× DC→5 kHz). The voltage-pulse is
not buggy; it reads the **near-DC** inductance (the current ramps slowly because
L is high, self-selecting a low effective frequency). For the **current loop**
(bandwidth ~1–2 kHz) the relevant value is the AC inductance ≈ **20–24 µH**, NOT
the DC 89 µH. VESC measures L via HFI at ~f_sw/2 (high freq, current-limited)
for exactly this reason; MESC uses the di/dt pulse (DC) like our voltage-pulse.

**√3 Kv confirmed.** λ measured 1.29 mWb; Kv = 60/(√3·2π·λ·Pp) ≈ 611 (line-to-line)
vs 700 nameplate (residual from λ +15 %). The pre-fix 1051 was the per-phase Kv.

**Detection is now reliable** after two fixes (commits on `bench-detection-2026-06-13`):
- The link-loss failsafe was fighting the device-side measurement (host blocks on
  the result → ergot liveness times out in 1 s → failsafe cut the drive, latched,
  and the rotor oscillated ±20°). A `DETECTION_ACTIVE` flag now suspends the
  link-loss path during a bounded measurement (the command-staleness deadman and
  over-current checks are not gated). R→L→flux now run end-to-end.
- `config::SENSORLESS` keeps the boot angle source off Hall (Observer/Manual),
  killing the continuous HallError on the sensorless ZD2808.

**HFI on this motor:** the fixed 3 V injection drew ~tens of A and tripped the
bench PSU OCP (the garbage 8.5 mH it "measured" was the sagged-bus artifact, not
the algorithm). Current-budgeting the injection (probe at I·R, v_max from the
probed \|Z|) makes it safe; the full FFT/saliency result still falls back
(low-saliency SPM), but the FFT-free \|Z| probe gives a clean 10.8 µH.

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

1. ~~Fix the HW OCP threshold~~ **DONE** — proven unusable (raw-pad comparator),
   break disabled, software OCP is the protection. See the corrected account above.
2. ~~Fix the voltage-pulse L and the √3 in Kv~~ **DONE** — √3 Kv fixed; the L gap
   is real frequency-dependence (not dead-time). Detection now reliable. See the
   "item 2" section above.
3. **HFI inductance-vs-frequency sweep** on this motor — vary the carrier
   (e.g. 0.5/1/2/5/10 kHz) via the FFT-free \|Z| probe and map the L(f) curve to
   confirm the eddy-current roll-off and pick the loop-relevant value directly.
4. **Refactor to one inductance method** (separate task, fresh context): promote
   the FFT-free \|Z| probe to the primary L measurement (current-limited, gives
   the AC/loop-frequency value); gate the rotating-HFI + FFT saliency path behind
   a `saliency-detect` (or `hfi-fft-detect`) feature for IPM motors; drop the
   voltage-pulse (it reads the DC L, wrong for the current loop). Add a
   frequency-dependent-L term to `VirtualMotor` so the sim reproduces the
   method-divergence instead of agreeing with a constant L.
5. Sensorless cold-start spin attempt — feed the **AC L (~20–24 µH)**, not the
   pulse DC 89 µH, into the current-PI gains; gentle limits.
