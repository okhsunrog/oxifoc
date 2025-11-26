# Oxifoc

WIP/experimental motor control (FOC) firmware for STM32G431 (B‑G431B‑ESC1) with a lightweight host tool. Device↔host communication runs over RTT using [ergot](https://github.com/jamesmunns/ergot).

## Project Structure

```
oxifoc/
├── Cargo.toml          # Workspace root (host crates only)
├── justfile            # Build automation
├── oxifoc-device/      # STM32G431 firmware (excluded from workspace)
├── oxifoc-host-lib/    # Shared host backend (transport + config)
├── oxifoc-host-tauri/  # Tauri desktop/mobile GUI
├── oxifoc-host-egui/   # egui desktop frontend
├── oxifoc-host-cli/    # CLI frontend
├── protocol/           # Shared protocol definitions
├── ergot/              # Git submodule - networking stack
├── docs/               # Documentation
├── scripts/            # Helper scripts
└── oxifoc-host.toml    # Optional host config
```

This repo uses a Cargo workspace for host crates. Device firmware is excluded (different toolchain).

## Hardware

- **Board**: B-G431B-ESC1
- **MCU**: STM32G431CB (Cortex-M4F with hardware FPU)
- **Debug Interface**: ST-Link
- **Communication**: RTT (Real-Time Transfer) via probe-rs

### B-G431B-ESC1 Pinout (Oxifoc)

| Pin           | Signal               |
|--------------:|----------------------|
| VBAT          | 3V3                  |
| PC13/TAMP/RTC | TIM1_CH1N            |
| PC14          | CAN_TERM             |
| PC15          | N.C.                 |
| PF0/OSC-IN    | OSC 8MHz             |
| PF1/OSC-OUT   | OSC 8MHz             |
| PG10/NRST     | RESET                |
| PA0           | VBUS                 |
| PA1           | Curr_fdbk1_OPAmp+    |
| PA2           | OP1_OUT              |
| PA3           | Curr_fdbk1_OPAmp-    |
| PA4           | BEMF1                |
| PA5           | Curr_fdbk2_OPAmp-    |
| PA6           | OP2_OUT              |
| PA7           | Curr_fdbk2_OPAmp+    |
| PC4           | BEMF2                |
| PB0           | Curr_fdbk3_OPAmp+    |
| PB1           | TP3                  |
| PB2           | Curr_fdbk3_OPAmp-    |
| VREF+         | 3V3                  |
| VDDA          | 3V3                  |
| PB10          | N.C.                 |
| VDD4          | 3V3                  |
| PB11          | BEMF3                |
| PB12          | POTENTIOMETER        |
| PB13          | N.C.                 |
| PB14          | Temperature feedback |
| PB15          | TIM1_CH3N            |
| PC6           | STATUS               |
| PA8           | TIM1_CH1             |
| PA9           | TIM1_CH2             |
| PA10          | TIM1_CH3             |
| PA11          | CAN_RX               |
| PA12          | TIM1_CH2N            |
| VDD6          | 3V3                  |
| PA13          | SWDIO                |
| PA14          | SWCLK                |
| PA15          | PWM                  |
| PC10          | BUTTON               |
| PC11          | CAN_SHDN, TP2        |
| PB3           | USART2_TX            |
| PB4           | USART2_RX            |
| PB5           | GPIO_BEMF            |
| PB6           | A+/H1                |
| PB7           | B+/H2                |
| PB8           | Z+/H3                |
| PB9           | CAN_TX               |
| VDD8          | 3V3                  |

## Current Capabilities (short)

- Device: button input (single/double/hold), keepalive, and device info server over ergot/RTT; defmt logs; Embassy async runtime.
- Host: attaches via ST‑Link + RTT, streams defmt and ergot, queries DeviceInfo on connect, prints keepalives and button events.
- Handshake: host requests DeviceInfo on startup with retry/backoff; device delays keepalives until it sees an inbound request to avoid “NoRoute” noise.

## Building

### Device Firmware

```bash
cd oxifoc-device
cargo build --release
```

### Host Applications

Build the egui app:

```bash
cargo build --manifest-path oxifoc-host-egui/Cargo.toml --release
```

Build the CLI:

```bash
cargo build --manifest-path oxifoc-host-cli/Cargo.toml --release
```

## Running

### Flash and Run Device

Using probe-rs (recommended):

```bash
cd oxifoc-device
cargo run --release
```

This will flash the firmware and start the device. The device will:
1. Initialize RTT channels (defmt on up0, ergot on up1, ergot-down on down0)
2. Configure button input on PC10 (active-low)
3. Start ergot communication stack
4. Begin periodic heartbeat and keepalive messages

### Run Host Application (egui)

From the repo root with the board connected via ST-Link:

```bash
cargo run --manifest-path oxifoc-host-egui/Cargo.toml --release
```

Note: ensure no other `probe-rs` session is running (e.g., a prior `cargo run` in `device/` or a separate `probe-rs` tool) before starting the host; the ST‑Link/RTT connection can only be owned by one process at a time.

The host will:
1. Connect to the STM32G431 via ST‑Link and attach RTT.
2. Stream defmt logs and ergot messages.
3. Query DeviceInfo early (with retry/backoff) and then continue.
4. Display button events and keepalive messages.

#### Configuration (TOML)

The host reads an optional `oxifoc-host.toml` in the current working directory (or from `OXIFOC_HOST_CONFIG` env var):

```toml
# Optional: specify probe by VID:PID or VID:PID:SERIAL
probe = "0483:374b"

# Optional: override chip auto-detection
chip = "STM32G431CBTx"

# Optional: path to device ELF for defmt decoding
# Defaults to device/target/thumbv7em-none-eabihf/release/oxifoc-device
elf = "/path/to/device.elf"

# Optional: enable/disable channel streaming (both default to true)
stream_defmt = true
stream_ergot = true
```

Fields:
- `probe`: optional ST‑Link selector like `VID:PID` or `VID:PID:SERIAL`.
- `chip`: optional chip override (e.g. `STM32G431CBTx`).
- `elf`: path to device ELF with `.defmt` section used for decoding logs. Defaults to `oxifoc-device/target/thumbv7em-none-eabihf/release/oxifoc-device`.
- `stream_defmt` / `stream_ergot`: booleans to enable/disable streams (default true).

### RTT Channel Map

The device firmware configures RTT channels as follows:

- **up0 "defmt"**: Debug logging output (via defmt macros)
- **up1 "ergot"**: COBS-framed protocol messages (device→host)
- **down0 "ergot-down"**: Reserved for host→device protocol messages

Both channels operate simultaneously - defmt for debug logs, ergot for structured protocol communication. The host application reads from both channels in parallel.

## Network Topology

Ergot DirectEdge profile (point‑to‑point):
- Host: controller at `1.1.0`
- Device: target at `1.2.0`

## Development Notes (short)

- Device code: `oxifoc-device/src/main.rs`, `oxifoc-device/src/usart_io.rs`.
- Host backend: `oxifoc-host-lib/src/lib.rs` (+ `oxifoc-host-lib/src/config.rs`).
- Host frontends: `oxifoc-host-tauri/` (Tauri GUI), `oxifoc-host-egui/src/main.rs`, `oxifoc-host-cli/src/main.rs` (CLI).
- Protocol endpoints: `protocol/src/lib.rs` (Button, Motor, AdcSample, Info).

### Quick Commands (justfile)

```bash
just install    # Install Tauri frontend dependencies
just dev        # Run Tauri dev server
just build      # Build Tauri release
just egui       # Run egui app
just cli        # Run CLI
just flash      # Flash device firmware
just lint       # Lint all code
just format     # Format all code
```

## Debugging

You can view defmt logs either through the host application or directly via probe‑rs — use one at a time:

- Via host: run `cargo run --manifest-path oxifoc-host-egui/Cargo.toml --release` to stream defmt and ergot together.
- Via probe‑rs: attach with your preferred tool to view defmt output only.

For device-only debugging (flash + run):

```bash
cd oxifoc-device
../scripts/probe_run.sh target/thumbv7em-none-eabihf/release/oxifoc-device
```

If you switch to the host application afterwards, stop any running probe‑rs session first.

## Roadmap (draft)

- PWM generation/commutation setup for G4 TIMs with safe dead‑time.
- Current sense path bring‑up (ADC + PGA/OPAMP) and offset calibration.
- Rotor angle feedback: Hall and incremental encoder support; sensorless exploration.
- Control loops: Iq/Id PI, velocity/position layers; runtime tuning via host.
- Safety: over‑current/voltage/temperature limits; fault latching and reporting.
- Telemetry: structured streaming over ergot; capture buffers for tuning.
- Host tooling: simple UI/CLI for calibration, logging, and parameter edits.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
