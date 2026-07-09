# VESC 6 MK5 (STM32F405) bring-up notes

Covers the Trampa VESC 6 MK5 layout and its clones (the bench unit is a
Flipsky "Mini V6 MK5"). Facts extracted from the VESC firmware hwconf
headers (`hwconf/trampa/vesc6/hw_60_core.h`, `HW60_IS_MK5` branch) — pin
assignments and component values only, no code (see the clean-room note in
docs/decisions.md).

Verification model: the board ships running VESC firmware for a specific HW
target and works with VESC Tool out of the box — the stock firmware drives
exactly these pins, so **the HW name it reports validates the whole pin map
wholesale**. The Flipsky listing already states firmware `60_MK5` / HW
`VESC_6_MK5`, phase filter present, self-resetting power button (= the
shutdown-latch circuit); a quick VESC Tool connect before reflashing just
confirms it (a `60_MK3`/`60_MK4` board would have no PC13 phase-filter
switch → `phase_sense.has_filters` would be wrong; the rest of the MK3+
map is identical).

## Vendor specs (Flipsky listing)

- Firmware 6.02, HW `60_MK5`
- 70 A continuous / 200 A "instantaneous" (marketing abs-max, ignored)
- **Voltage 14–60 V** (4-13S; spikes must not exceed 60 V) — note the
  **14 V operating minimum**: power the bench from ≥14 V, not the usual 12 V
- BEC 5 V @ 1 A; USB/CAN/UART; ABI/HALL/AS5047/AS5048A sensor port
- Motor/power wires 12 AWG; 67×39×18.7 mm with heatsink

Firmware target: `oxifoc-f405` with `--no-default-features
--features transport-usb,transport-uart,board-vesc6-mk5`.

## Pin map

Identical to Cheap FOCer 2 (both follow the VESC reference layout):

- PWM (DRV8301 6-PWM): PA8/PA9/PA10 high (TIM1_CH1-3), PB13/PB14/PB15 low (CH1N-3N)
- EN_GATE: PB5 (active high); nFAULT: PB7 (active low, EXTI)
- Currents: CURR1 PC0 (IN10), CURR2 PC1 (IN11), CURR3 PC2 (IN12)
- VBUS (AN_IN): PC3 (IN13) via 39k/2.2k
- Temps: PA3 board NTC (10k/10k, β3380), PC4 motor NTC (10k pull-up, low-side)
- Phase voltage: SENS1 PA0, SENS2 PA1, SENS3 PA2 (IN0/1/2), divider = VBUS divider
- Halls: PC6/PC7/PC8
- UART (COMM port): USART3 TX PB10 / RX PB11
- USB FS: PA11/PA12
- LEDs: PB0 green, PB1 red
- Servo/PPM: PB6 (TIM4_CH1)
- CAN: PB8 RX / PB9 TX **[verify — hw.h default, not in hw_60_core.h]**

Different from CF2:

- **DRV8301 SPI is bit-banged**: SCK PC10, MOSI PB4, MISO PB3, CS PC9.
  PB3/PB4 don't form a valid hardware-SPI mapping (VESC bit-bangs it too);
  PC11/PC12 — SPI3 MISO/MOSI on CF2 — are the NRF51 UART on MK5.
  PB3/PB4 are JTAG pins (JTDO/NJTRST); GPIO reconfig doesn't affect SWD.
- **Shutdown latch: PC5** (MK3+). The power button only bridges power until
  firmware drives PC5 high — must be done as early as possible in boot or
  the board turns itself off when the button is released. Button state is
  sampled on the same net (ADC12_IN15). (If a clone omits the shutdown
  circuit, driving PC5 high is harmless.)
- **CURRENT_FILTER enable: PD2** (active high). Switchable RC filter on the
  current-sense path; VESC enables it at early init. Drive high.
- **PHASE_FILTER enable: PC13** (MK5/MK6 only, active high). Switchable RC
  filters on SENS1-3 — this is what makes phase-voltage sensing usable
  while PWMing (`phase_sense.has_filters = true`). Drive high.
- IMU: BMI160 on I2C, SDA PB2 / SCL PA15 — not used by oxifoc.
- NRF51 (permanent, MK3+): UART on PC11/PC12, SWD on PB12/PA4 — not used.

## Analog scaling

- Shunts: **in-line phase shunts** (`HW_HAS_PHASE_SHUNTS`), 0.5 mΩ,
  standard polarity (hw60 does NOT define `INVERTED_SHUNT_POLARITY`).
  Reading is valid over the whole PWM cycle, not just low-side conduction —
  bring-up keeps the CF2 sampling instant (mid low-side window, valid for
  both topologies); exploiting full-cycle validity is a later optimization.
- Amp gain: **20 V/V**, programmed into the DRV8301 internal amps
  (VESC: `drv8301_set_current_amp_gain(20)`; CF2 uses 10).
- VBUS divider: 39k/2.2k ⇒ ratio ≈ 18.73:1. Same for SENS1-3.
- NTC formulas identical to CF2 (board: 10k/10k high-side β3380;
  motor: 10k pull-up low-side).

## Limits (original Trampa hw60)

- HW_LIM_CURRENT ±120 A, absolute max 160 A, VIN 6–57 V, FET temp cutoff 110 °C.
- `BOARD` uses the Flipsky continuous rating: 70 A peak, temp cutoff kept at
  a conservative 100 °C. DRV8301 OC (VDS) threshold stays at CF2's 511 mV —
  recompute only if pushing past the vendor rating (needs the FET part).
- Dead time: 360 ns (VESC `HW_DEAD_TIME_NSEC` fallback; hw60 doesn't override).
- VESC defaults for this HW: f_zv 30 kHz, `MCCONF_FOC_SAMPLE_V0_V7 false`.

## Programming & recovery (SWD)

The board exposes no SWD header — just **four pads: VCC / GND / CLK / DIO**.
Those pads *are* the SWD port:

- **CLK = SWCLK**, **DIO = SWDIO**, **GND = ground**.
- **VCC** is the 3.3 V rail (target-voltage sense). Wire it to the probe's
  Vtref *sense* input only — **do not source power into it**. Power the board
  from its main input, not this pad.
- **NRST is not broken out.** probe-rs resets via SWD `SYSRESETREQ`
  (software reset), which is enough for flashing. `connect-under-reset` is
  unavailable — see the recovery note below for the substitute.

The Flipsky "smart switch" pad is **not** a reset line — it is the
power-latch / button net on **PC5** (see the shutdown-latch entry in the pin
map). Pressing the button bridges the regulator enable; firmware must drive
PC5 high early to keep power after release; sampling the button briefly
switches PC5 to analog-in (ADC12_IN15). oxifoc latches PC5 high in bootstrap
(`main.rs`/`hardware/mod.rs`) — effectively `ALWAYS_ON` while powered; it
does not implement button-off.

Flashing flow:

1. Bench PSU on the **main input ≥14 V**.
2. **Tap the power button** so the stock firmware boots and latches PC5 → the
   board stays on.
3. Connect SWD (GND + CLK + DIO; VCC to Vtref sense if the probe needs it) and
   flash the `board-vesc6-mk5` build via probe-rs.

### Recovery — don't let a bad flash brick it

There is **no boot-first bootloader**: on reset the MCU jumps straight to the
app at `0x08000000`. If a flashed image fails to raise PC5 early (crash before
the latch, bad build), the board **powers itself off** and the probe can't
catch it — it looks bricked.

The escape hatch: the button forces the regulator enable **in parallel with**
the PC5 latch, independent of firmware. So to recover:

- **Press and hold the power button** to force power on regardless of what the
  firmware does, then connect SWD → `halt` → `erase` → reflash a known-good
  image. Held button = guaranteed power = there is always a way back in.

Keep this in mind on every first flash of a new image.

## Bring-up checklist (when the board arrives)

1. Bench PSU at **≥14 V** (vendor operating minimum — the usual 12 V profile
   is below spec for this board).
2. Before reflashing: quick VESC Tool connect, confirm reported HW `60_MK5`
   (formality — the listing states it; see the verification-model note).
3. PSU-safe profile (RampToZero, bus_regen=0), current limit low.
4. Flash `board-vesc6-mk5` build via SWD; confirm boot log prints the MK5
   pin map and DRV8301 device ID reads back (bit-bang SPI works).
5. USB monitor 5 s: all six defmt categories at 1 Hz.
6. ADC sanity: VBUS matches PSU, temps plausible, phase currents ~0.
7. Motor detect (R/L) on the Flipsky 5065, then hall calibration,
   then `scripts/benchsuite.py`.
