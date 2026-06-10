# Project TODO / backlog

Working list of known gaps and planned work. A full external review
with verified bugs, gap analysis and a borrow-list from reference
projects (VESC/MESC/moteus/ODrive/MCSDK) lives in
[review-2026-06-10.md](review-2026-06-10.md); items below reference it. Safety-specific items live in
[safety.md](safety.md#open-questions--todo); hardware bring-up notes in the
board docs.

## Deferred until needed

- [ ] **`protocol_version` in `HardwareInfo`** + `env!("CARGO_PKG_VERSION")`
  instead of the hardcoded `"oxifoc-0.1.0"` strings. Not relevant while GUI
  and firmware are always built from the same checkout (`cargo run`), but
  required before any release/distribution: the wire schema (postcard) has
  no self-description, so a version mismatch shows up as silent garbage,
  not an error. The schema has already changed several times
  (`SlowTelemetry.phase_source`, `ConfigResponse::Busy`,
  `PhaseSourceEndpoint`).

## Firmware / core

- [ ] Virtual device only simulates `CurrentControl`/`Stopped`; OpenLoop,
  DirectVoltage and SixStep are accepted and ignored (limits/gains
  commands too).
- [ ] Remaining ISR dedup: ADC snapshot assembly + voltage/temp fault
  checks are still per-platform copies (small).
- [ ] g474 motor modules are commented out until the IHM08M1 shield is
  connected; `control/foc.rs` is kept in sync by hand but not
  compile-checked.

## Size / performance

- [ ] **g431 flash headroom is ~3.3 KB** (123 564 / 126 976 bytes) after
  enabling `build-std = ["core"]` (libcore rebuilt with opt-level="z";
  the shipped one is opt-level=3). When it runs out again: trim
  `.rodata` (~14 KB, mostly postcard schema tables), consider
  `panic_immediate_abort`.
- [ ] VSQRT (`vsqrt.f32`) instead of `libm::sqrtf` on Cortex-M4F hot paths.
- [ ] Revisit TIM6 hall-polling rate (5 µs currently).
- [ ] µs hall ticks on all platforms (consistent timebase).
- [ ] f405/g474 build with `opt-level = 3`, g431 with `"z"` (flash
  pressure). Intentional, but unmeasured: check what `"z"` would cost
  f405/g474 in ISR time, or whether it matters at all at 20 kHz.

## From external review (verified pending / host side)

- [ ] `oxifoc-virtual --pole-pairs`: reported as dead (sim and detect
  backends allegedly use `MotorParams::default()`); needs verification —
  the flag IS passed in main.rs, check where it stops propagating.
- [ ] CLI `--baud` has `default_value_t`, so it always overrides
  `serial_baud` from oxifoc-host.toml — config value never applies.
- [ ] Framed transports (UDP/USB/BLE): handshake result ignored and no
  reconnect loop, unlike the COBS path — a UDP host that loses link is
  dead until manual reconnect.
- [ ] GUI parses numeric fields with `unwrap_or(0.0)` — a typo in the
  resistance field silently writes 0 Ω to flash.
- [ ] **F405 ADC trigger suspicion (bench)**: ADC triggers from TIM1_CH4
  compare, which fires twice per period in center-aligned mode (G431
  correctly uses TRGO2/COMPARE_OC4, one edge). May work by accident
  (second trigger lands in a still-running injected sequence). Verify
  ISR rate on hardware or move F405 to TRGO.

## Host tools

- [ ] GUI (Slint): phase-source switcher + display of
  `SlowTelemetry.phase_source` (CLI `source` command already works).

## Hardware bench (waiting for the rig)

- [ ] Re-run motor detection — stored Flipsky params are 1.5× off after
  the SVPWM normalization fix.
- [ ] OCP with the BKF break filter under real load (g431).
- [ ] Dead-time compensation at low speed.
- [ ] Hall-dropout-at-speed and sensorless crossover behavior.
- [ ] HFI on the real B-G431B-ESC1: carrier defaults (1 kHz, 12.5% vbus)
  and polarity-probe pulse amplitude/length (`HFI_POLARITY_*` constants)
  may need tuning per motor.
- [ ] Source switching end-to-end via `oxifoc-host-cli source ...`.
