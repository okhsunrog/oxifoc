# Project TODO / backlog

Working list of known gaps and planned work. Safety-specific items live in
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

- [ ] `FLASH_DONE` is signaled by the storage workers but never awaited:
  either remove it or use it for a true write-through ack in
  `config_server` (respond Ok only after the flash write completed).
- [ ] Virtual device only simulates `CurrentControl`/`Stopped`; OpenLoop,
  DirectVoltage and SixStep are accepted and ignored (limits/gains
  commands too).
- [ ] Remaining ISR dedup: ADC snapshot assembly + voltage/temp fault
  checks are still per-platform copies (small).
- [ ] g474 motor modules are commented out until the IHM08M1 shield is
  connected; `control/foc.rs` is kept in sync by hand but not
  compile-checked.

## Size / performance

- [ ] **g431 flash headroom is ~2.4 KB** (124 556 / 126 976 bytes with
  `opt-level = "z"`, `codegen-units = 1`). Next sizable feature may not
  fit. Candidates: trim `.rodata` (13.7 KB, mostly postcard schema
  tables), feature-gate unused transports.
- [ ] VSQRT (`vsqrt.f32`) instead of `libm::sqrtf` on Cortex-M4F hot paths.
- [ ] Revisit TIM6 hall-polling rate (5 µs currently).
- [ ] µs hall ticks on all platforms (consistent timebase).
- [ ] f405/g474 build with `opt-level = 3`, g431 with `"z"` (flash
  pressure). Intentional, but unmeasured: check what `"z"` would cost
  f405/g474 in ISR time, or whether it matters at all at 20 kHz.

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
