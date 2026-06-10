# NUCLEO-G474RE + X-NUCLEO-IHM08M1: pin mapping

Derived 2026-06-11 from three sources, cross-checked pin by pin:

- shield signal → morpho position + solder bridge: UM1996 Tables 3/4
  ([UM1996_IHM08M1_getting_started.pdf](UM1996_IHM08M1_getting_started.pdf))
  and the schematic MCU-pinout page
  ([X-NUCLEO-IHM08M1_schematic.pdf](X-NUCLEO-IHM08M1_schematic.pdf), Fig. 6);
- morpho position → G474RE pin: UM2505 Table 16
  ([UM2505_NUCLEO-G474RE_user_manual.pdf](UM2505_NUCLEO-G474RE_user_manual.pdf));
- G474RE pin → AF/peripheral: stm32-data (TIM1/TIM2 pin maps).

The shield was designed for NUCLEO-F302R8/F401RE; pin names printed in its
schematic are those MCUs'. Everything below is re-derived for the G474RE
through the morpho positions — do not trust the schematic's pin labels
directly (two of its resistor options land on G474 pins with *different*
timer functions, see Hi-Z warnings).

## PWM (TIM1, all native AF6)

| Signal | Morpho | G474RE | Function | Bridge |
|---|---|---|---|---|
| UH | CN10-23 | PA8 | TIM1_CH1 (AF6) | R56 |
| UL | CN10-15 | PA7 | TIM1_CH1N (AF6) | R58 |
| VH | CN10-21 | PA9 | TIM1_CH2 (AF6) | R64 |
| VL | CN7-34 | PB0 | TIM1_CH2N (AF6) | R67 |
| WH | CN10-33 | PA10 | TIM1_CH3 (AF6) | R70 |
| WL | CN10-24 | PB1 | TIM1_CH3N (AF6) | R72 |

⚠️ **PB15 (CN10-26) must stay Hi-Z.** R86 ties it to the same UL gate-driver
input as PA7 (an F302-era alternate). On the G474, PB15 is TIM1_**CH3N**
(AF4) — configuring it as a timer output would fight PA7 on the U-phase
low-side gate. Leave it floating input.

## Protection

| Signal | Morpho | G474RE | Function | Bridge |
|---|---|---|---|---|
| BKIN (OCP comparator out, active) | CN10-13 | PA6 | TIM1_BKIN (AF6) | R78 |
| BKIN same net, optional | CN10-14 | PA11 | TIM1_BKIN2 (AF12) | R73 |
| CPOUT (comparator out) | CN10-12 | PA12 | TIM1_ETR (AF11) | R52 |
| CURRENT REF (OCP threshold) | CN10-27 | PB4 | TIM3_CH1 PWM → RC (AF2) | R77 |

- The BKIN net also lands on **PB14 (CN10-28, R74)** — an F302 BKIN pin. On
  the G474 PB14 is TIM1_CH2N, *not* BKIN: **keep PB14 Hi-Z**.
- PA11 on the same net is usable as a second, independent break path
  (BKIN2). PA6 + PA11 both armed = belt and braces.
- The OCP threshold is set by filtered PWM on PB4 (DAC on PA4 is the
  unpopulated alternative, R76 N.M. — PA4 is the speed pot by default).

## Current / voltage sensing (FOC, 3-shunt)

| Signal | Morpho | G474RE | ADC channel | Bridge |
|---|---|---|---|---|
| Curr_fdbk_PhA | CN7-28 | PA0 | ADC12_IN1 | R47 |
| Curr_fdbk_PhB | CN7-36 | PC1 | ADC12_IN7 | R48 |
| Curr_fdbk_PhC | CN7-38 | PC0 | ADC12_IN6 | R50 |
| VBUS_sensing | CN7-30 | PA1 | ADC12_IN2 | R51 |
| Temperature (NTC 10k) | CN7-35 | PC2 | ADC12_IN8 | R54 |

- CN7-36/CN7-38 are solder-bridge-configurable on the Nucleo; the defaults
  (PC1/PC0 = ARD_A4/A5 per UM2505 Table 17) are what the shield expects.
- Shunt amplification is on the shield (board op-amps), signals arrive
  conditioned — G474 internal OPAMPs are not needed.
- VBUS divider on the shield: 169 kΩ / 9.31 kΩ → ratio ≈ 19.15:1.

## Hall / encoder (J3, pull-ups via JP3)

| Signal | Morpho | G474RE | Function | Bridge |
|---|---|---|---|---|
| H1 / Enc A | CN7-17 | PA15 | TIM2_CH1 (AF1) | R79 |
| H2 / Enc B | CN10-31 | PB3 | TIM2_CH2 (AF1) | R81 |
| H3 / Enc Z | CN10-25 | PB10 | TIM2_CH3 (AF1) | R84 |

**This is TIM2, not TIM4.** The shield's hall pins all land on TIM2
CH1/CH2/CH3 — the hall-sensor interface (TI1S XOR + capture) works the
same way as our TIM4 setup on G431, with two consequences for oxifoc-g474:

1. **embassy time driver conflict**: oxifoc-g474 currently uses
   `time-driver-tim2`. Move it to `time-driver-tim5` (G474 has 32-bit
   TIM5; G431 does not — g431 keeps tim2) and put the hall interface on
   TIM2.
2. **TIM2 is 32-bit** — `CaptureTimebase` in core is written for u16
   captures; the g474 hall module needs a u32 variant (overflow every
   ~71 min at 1 MHz instead of 65 ms, same race rules).
3. The current `oxifoc-g474/src/sensors/hall.rs` (TIM4, PB6-8, copied
   from G431) is **wrong for this shield** and must be redone before the
   motor stack is enabled. PB6/PB7 stay free (CN10-17/CN7-21).

## BEMF dividers (6-step only; unused in FOC)

| Signal | Morpho | G474RE | ADC | Bridge |
|---|---|---|---|---|
| BEMF1 | CN7-37 | PC3 | ADC12_IN9 | R59 |
| BEMF2 | CN10-18 | PB11 | ADC12_IN14 | R60 |
| BEMF2 (same net) | CN10-34 | PC4 | ADC2_IN5 | R61 |
| BEMF3 | CN10-6 | PC5 | ADC2_IN11 | R65 |
| GPIO_BEMF (divider enable) | CN10-1 | PC9 | GPIO | R55 |

BEMF2 is wired to *both* PB11 and PC4 (both bridges mounted) — use one,
keep the other analog/Hi-Z. For FOC keep GPIO_BEMF low/floating.

## Misc

| Signal | Morpho | G474RE | Note |
|---|---|---|---|
| Speed potentiometer | CN7-32 | PA4 (ADC2_IN17) | R181 mounted; remove for DAC use (R76 N.M.) |
| Red LED | CN10-22 | PB2 | via 510 Ω |
| Debug J7-2 / J7-3 | CN10-11 / CN10-29 | PA5 (R80 N.M.) / PB5 (R85) | DAC/PWM scope outputs |
| START/STOP button | CN7-23 | PC13 | Nucleo blue button |
| +5 V to Nucleo | CN7-6 (E5V) | — | shield powers the Nucleo via R170 — set Nucleo **JP5 to E5V** |
| VIN feed | CN7-24 | — | via shield J9 (open it for VBUS > 12 V!) |

## Jumper configuration for FOC (UM1996 §2.2.1)

- Shield: **JP1 + JP2 closed**, **J5 & J6 on the 3-Sh side**, **JP3
  closed** (hall pull-ups), **J9 open** before powering J1 with > 12 V,
  remove **C3, C5, C7** (6-step startup caps distort FOC current loops).
- Nucleo: JP1 open, **JP5 on E5V**, JP6 closed.

## Deltas vs oxifoc-g474 code

Applied 2026-06-11: `sensors/hall.rs` is on TIM2/PA15+PB3+PB10 with
32-bit captures (`CaptureTimebase<u32>`), the embassy time driver moved
to TIM5, `hardware/resources.rs` carries the corrected pins/CN numbers,
and `mod sensors` is compiled even while the motor stack is dormant.

Still open for bring-up (tracked in TODO.md):

- re-enable control/motor/calibration modules; their `foc.rs` must take
  `now_ticks` from `sensors::hall::now_ticks()` like g431 (already
  edited, not compile-checked until enabled);
- ADC assignment per this table: PA0/PC1/PC0 (IN1/IN7/IN6 on ADC1/2),
  VBUS PA1, NTC PC2; BKIN on PA6 (+ PA11 as BKIN2) with the same
  external-comparator model as F405's DRV nFAULT (the shield has its own
  OCP comparator → BKIN);
- **keep PB15 and PB14 Hi-Z** (see warnings above).
