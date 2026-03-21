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
