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
--features transport-usb,board-vesc6-mk5,task-priority`.
The MK5 build/flash recipes select cooperative task priorities together with
the USB interrupt executor, following the [on-device comparison](../notes/bench-usb-priority-2026-11-09.md).
Other F405 board defaults are unchanged; omit `task-priority` for queue A/B tests.

## Pin map

Identical to Cheap FOCer 2 (both follow the VESC reference layout):

- PWM (DRV8301 6-PWM): PA8/PA9/PA10 high (TIM1_CH1-3), PB13/PB14/PB15 low (CH1N-3N)
- EN_GATE: PB5 (active high); nFAULT: PB7 (active low, EXTI)
- Currents: CURR1 PC0 (IN10), CURR2 PC1 (IN11), CURR3 PC2 (IN12)
- VBUS (AN_IN): PC3 (IN13) via 39k/2.2k
- Temps: PA3 board NTC (10k/10k, β3380), PC4 motor NTC (10k pull-up, low-side)
- Phase voltage: SENS1 PA0, SENS2 PA1, SENS3 PA2 (IN0/1/2), divider = VBUS divider
- Halls: PC6/PC7/PC8
- COMM TX/SCL: **PA7 and PB10 share `TX_SCL_MOSI`**. SPI1 MOSI uses PA7 AF5;
  PB10 must remain high impedance. F405 firmware no longer initializes USART3.
- COMM ADC1/ADC2: SPI1 SCK PA5 AF5 / MISO PA6 AF5.
- COMM RX/SDA: PB11 (`RX_SDA_NSS`), available as GPIO chip select for SPI master.
- USB FS: PA11/PA12
- LEDs: PB0 green, PB1 red, active high. OxiFOC uses TIM8_CH2N/CH3N
  at 4 kHz with complementary polarity inverted, capped at 10% illuminated
  duty. Green pulses for 50 ms each second; red lights only for a fault.
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

## C5 bridge wiring and continuity check

The [official schematic](https://vesc-project.com/sites/default/files/Benjamin%20Posts/VESC_6_mk5.pdf)
shows PA7 (pin 23) and PB10 (pin 29) on the same net on MCU sheet 3;
the main sheet connects it to COMM pin 6.

| COMM / controller | STM32 | ESP32-C5 |
|---|---|---|
| TX/SCL, pin 6 | PA7, PB10 Hi-Z | GPIO7 |
| ADC1, pin 4 | PA5 | GPIO6 |
| ADC2, pin 7 | PA6 | GPIO9 |
| RX/SDA, pin 5 | PB11 | GPIO10 |
| SERVO signal | PB6 | GPIO0 |

The actual TX/SCL wire is connected to **GPIO7**, as confirmed by the owner.
Initial tests used GPIO8 and therefore did not test that wire. Repeating the
STM32-to-C5 test with GPIO7 received all 32 five-bit connector codewords;
PA7 output also read correctly through PB10.

The finite probes keep the gate driver and all phase outputs disabled:
`just build-mk5-bridge-pins` drives connector patterns while C5 `pin_probe`
only reads. `just build-mk5-bridge-pins-rx` leaves STM32 connector pins as
inputs for C5 `pin_drive`. C5 sources and its build/run recipes live in
`/home/okhsunrog/code/rust/c5-hello-world`.

SPI DMA qualification uses `just bench-mk5-bridge-spi OUTPUT_DIR`. The standalone
STM32 master uses SPI1 (DMA2 streams 3 TX / 0 RX); C5 uses SPI2 with DMA_CH0,
mode 0, GPIO0 ready. PB10 remains Hi-Z, the gate driver and phase outputs stay
disabled, and persistent configuration is untouched. The master's HSI PLL
provides nominal SPI rates of 0.328125 / 1.3125 / 5.25 / 10.5 / 21 MHz,
independently of the board HSE calibration.

Three complete runs passed both directions: 31,200 full-duplex transactions,
15,594,480 bytes per direction, no byte, length or DMA errors. Each run tests
8/64/137/256/512-byte CS transactions at every rate, then 10,000 consecutive
512-byte frames at 21 MHz. The latter sustained 0.850–0.855 MB/s **per direction
simultaneously**, including the intentional 200-us ready-low delay, buffer
preparation and byte verification. No padding was required for the tested
137-byte transfers. This qualifies the quiet electrical/DMA path, not FOC
coexistence, Wi-Fi forwarding, arbitrary-length receive buffers, cancellation
or production transport recovery. Both ends know each test's length in advance.
Raw logs and per-run JSON are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-c5-spi-2026-11-10/qualified/`.

The separate `just bench-mk5-spi-cs-request OUTPUT_DIR` test alternates master
requests (CS low before C5 arms DMA, with 0/50/500-us preparation delays) and
slave requests (DMA/READY before CS). Three runs passed 31,200 full-duplex
transactions with no byte, length or DMA errors at the same rates and lengths.
Thus arming C5 while CS is already low works on this board, provided STM32
waits for READY before generating SCLK. This is an electrical handshake test,
not yet the production idle/wakeup or reset-recovery protocol. Results are in
the sibling `cs-request/` artifact directory. Restore both production images
after this finite, motor-inhibited test.

The production SPI transport uses CS as the master's request and GPIO0/PB6
as level READY: C5 arms DMA before asserting READY; STM32 waits before SCLK.
At completion STM32 releases CS, C5 lowers READY. Both sides arm their GPIO
wait before releasing their own signal, so a fast low/high boundary cannot be
lost. C5 only arms a new transaction for queued TX or asserted CS; STM32 waits
on its queue/EXTI rather than clocking empty probes. The double RX/TX buffer
pairs permit routing the previous slot while another transfer is armed, but
idle RX processing must run before sleeping because it can generate replies.
Peer epochs reset upstream discovery and downstream seed leases after reboot.

`just bench-bridge-handshake PORT SECONDS HZ` checks 20-Hz acknowledged Stop
commands simultaneously with 0/5000/20000-Hz legacy Ergot streaming. It never
energizes the motor. The 150-ms stream-enable interval is reported separately
from steady-state gaps, using the device's acknowledged decimation factor.

`just build-mk5-spi` / `just flash-mk5-spi` default to STM32 USB disabled;
pass `on` explicitly for USB diagnostics. High-level embassy-usb, its Ergot
adapter, USB task/executor/queues and RX storage are excluded in the SPI build.
The C5 bridge keeps GPIO26 low for the Waveshare onboard antenna.

With STM32 USB off, the stopped-motor qualification received 600,216 samples
at 20,004 Hz over 30 s with no sequence gaps and 600/600 concurrent Stop ACKs
(maximum 8.454 ms). The 5-kHz and disabled-stream checks each passed 200/200
commands; 5 kHz received 50,016 samples without gaps. These use legacy Ergot
delta telemetry, not Packed26. Ten STM32 and ten C5 resets all reconnected
within 4.022 s. The opt-in bridge `bench-delayed-upstream` feature delays route
setup by 3 s; the corrected host reconnect/discovery path passed in 4.737 s.
Normal bridge builds do not enable this injection. Idle traffic was 8 slots/s
for Ergot's upstream ping/reply, not continuous empty polling. Evidence is in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-c5-spi-2026-11-10/handshake-summary.json`.

## Optional CCM placement

F405 feature `ccm` moves only `STATE` and `FOC_DRIVER` (7488 bytes) to
CPU-only CCM; code, DMA buffers and stack stay in their usual regions.
An assembly `__pre_init` enables CCM and copies its initialized FLASH image.
The linker section precedes `.data` so cortex-m-rt's `__edata` remains in
SRAM; a linker assertion guards that startup boundary.

`just flash-mk5-profile sram` / `ccm` select otherwise matching finite-trace
builds. Three native 20-kHz L captures per placement were complete. In those
roughly 36-ms windows, ADC ISR median was 32.198 us in SRAM versus 31.960 us
in CCM (less than 1%); these are instrumented observations, not worst-case
timing guarantees. CCM remains off in the normal MK5 recipes. Exact ELFs,
traces and captures: `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-ccm-2026-11-09/`.

## Analog scaling

- MK5 uses `CurrentSenseTopology::ThreeShunt`: all three in-line shunts are
  valid at the configured sample point. Control, observers, detection and host
  enrichment use the full Clarke projection. The original phase measurements
  remain available to the independent per-phase protection. `i_sum_a` records
  their unprojected sum for diagnosing common-mode noise and channel faults.
  Two-channel sensors retain the two-input path through an explicit topology.
- ICD v6 reports this choice through `CAP_THREE_SHUNT_CURRENT` in the frozen
  `HardwareInfo.capabilities` field. Update host and firmware together; a v5
  host must fail the version check rather than reconstruct different dq currents.
  The fast-frame layout and persistent configuration record layouts do not change.
  Capture metadata includes protocol version and current topology, so earlier
  two-input captures can be distinguished from three-input recordings.

- Per-device corrections live in `BoardCalibrationConfig`, exposed as the
  `board-calibration` group. V1 contains only `vbus_gain` (default 1.0,
  finite, allowed range 0.95–1.05). Nominal divider values, current gain,
  NTC conversion and the boot-measured VREF remain independent.
  The gain is applied once at the ADC-to-VBUS boundary, so control,
  detection, protection and both telemetry streams consume the same voltage.
  The host does not multiply the received voltage again.
- Apply is RAM-only and atomic with respect to the ADC ISR. It requires
  `Stopped` mode, no boot/offset/detection maintenance and no active fast
  stream. Persist writes that exact live revision while stopped. `config set`
  performs Apply then Persist; inspect `persisted` in the result. The value
  belongs to this board, not every MK5 or its attached motor:

  ```sh
  just cli --transport usb --json config get board-calibration
  just cli --transport usb --json config set board-calibration vbus_gain=1.009725
  ```

  The example gain is a candidate from this bench board's 16/20/24 V
  measurements, not a model default. Reapply is absolute, not cumulative.
  Export the calibration/config together with `info` (including MCU UUID)
  and its measurement report before transferring or restoring it.
- Flash stores a versioned V1 record at key 12 in the existing storage
  sectors. A missing record means identity; an unreadable/invalid/unknown
  version sets `valid=false`, raises a calibration warning and independently
  blocks motor starts until a valid value is applied. Clearing the warning
  alone cannot bypass that gate. Apply a valid value, persist it and clear
  the old warning after repair. `ResetAll` erases this group as well; setting
  `vbus_gain=1.0` resets just this correction. Reset is also refused during
  a fast stream.
- Board calibration was introduced in application protocol version 5; host
  and firmware must be updated together. Older firmware does not recognize key 12, so rolling
  back firmware alone can make its storage scan fail. Retain a complete flash
  backup from before the new key was written. Existing key numbers/layouts
  are unchanged. Future calibration fields require a new stored-record
  version and explicit migration; do not append them to the frozen V1 layout.
- Capture records the active calibration and its revision/validity in
  `oxifoc.board_calibration` after the fast-stream ACK, when calibration
  changes are blocked; `oxifoc.config` contains that same active correction.

- At boot, ADC1 samples VREFINT 64 times (480-cycle sample time) before
  enabling the gate driver or timer-triggered conversions. The factory
  VREFINT calibration at `0x1FFF7A2A` (3300 mV) determines the measured
  VREF+ used by voltage/current conversion, detection and the host's
  `HardwareInfo.calib`. The boot log reports the factory code, sample sum
  and resulting millivolts. Invalid or out-of-range readings stop boot
  before the gate driver is enabled. Divider nominal values remain unchanged.
  NTC conversion remains ratiometric and does not use VREF+ in volts.

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
   flash with `just flash-f405-vesc6-mk5`. The recipe name includes the board
   deliberately; there is no generic/default F405 flash command.

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
3. Flash the exact board build via SWD: `just flash-f405-vesc6-mk5`.
   Confirm the boot log contains `board=VESC6_MK5`, `MK5 board ctrl`, the
   bit-bang SPI line and `DRV8301 ready` (including a valid device-ID read).
4. Release the power button. The board must remain powered from the PC5 latch.
5. Before connecting/energising the motor, run `just cli monitor --seconds 5
   --fast-hz 0` and `just cli faults`. VBUS must match the PSU, temperatures
   must be plausible and all three phase currents must be near zero.
6. Check the 1 Hz `isr/s` and `hall/s` log lines before enabling PWM. At a
   20 kHz PWM setting the ADC/control ISR cadence must be 20 kHz, not 40 kHz;
   `over=0` is required. Turn the motor by hand and require Hall edges, valid
   states (1, 3, 2, 6, 4, 5 in one direction or the reverse sequence) and
   `overcap=0`.
7. Persist the initial PSU-safe limits while stopped:

   ```sh
   just cli config set current-limits \
       max_iq_a=3 max_phase_current_a=5 bus_in_max_a=2 bus_regen_max_a=0
   just cli config set failsafe staleness_timeout_ms=800 policy=1
   just cli config get current-limits
   just cli config get failsafe
   ```

   `policy=1` is RampToZero. Do not use ControlledStop on a bench PSU until
   the board/motor pair has been validated for regenerative braking.
8. Only after those gates pass: motor R/L detection, Hall calibration and a
   very-low-current spin. Run `scripts/benchsuite.py` after the smoke test;
   CF2 thresholds are not automatically acceptance limits for this motor/MK5.
