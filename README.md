# Oxifoc

WIP/experimental motor control (FOC) firmware for STM32G431 (B‑G431B‑ESC1) with a lightweight host tool. Device↔host communication runs over RTT using [ergot](https://github.com/jamesmunns/ergot).

## Project Structure

```
oxifoc/
├── device/          # STM32G431 firmware (B-G431B-ESC1 board)
├── host/            # PC-side application for RTT communication
├── protocol/        # Shared protocol definitions
├── docs/            # Documentation
└── scripts/         # Helper scripts
```

This repo intentionally does not use a workspace at the root level (device and host use different targets/toolchains).

## Hardware

- **Board**: B-G431B-ESC1
- **MCU**: STM32G431CB (Cortex-M4F with hardware FPU)
- **Debug Interface**: ST-Link
- **Communication**: RTT (Real-Time Transfer) via probe-rs

## Current Capabilities (short)

- Device: button input (single/double/hold), keepalive, and device info server over ergot/RTT; defmt logs; Embassy async runtime.
- Host: attaches via ST‑Link + RTT, streams defmt and ergot, queries DeviceInfo on connect, prints keepalives and button events.
- Handshake: host requests DeviceInfo on startup with retry/backoff; device delays keepalives until it sees an inbound request to avoid “NoRoute” noise.

## Building

### Device Firmware

```bash
cd device
cargo build --release
```

### Host Application

```bash
cd host
cargo build --release
```

## Running

### Flash and Run Device

Using probe-rs (recommended):

```bash
cd device
cargo run --release
```

This will flash the firmware and start the device. The device will:
1. Initialize RTT channels (defmt on up0, ergot on up1, ergot-down on down0)
2. Configure button input on PC10 (active-low)
3. Start ergot communication stack
4. Begin periodic heartbeat and keepalive messages

### Run Host Application

With the board connected via ST-Link:

```bash
cd host
cargo run --release
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
# Defaults to ../device/target/thumbv7em-none-eabihf/release/oxifoc
elf = "/path/to/device.elf"

# Optional: enable/disable channel streaming (both default to true)
stream_defmt = true
stream_ergot = true
```

Fields:
- `probe`: optional ST‑Link selector like `VID:PID` or `VID:PID:SERIAL`.
- `chip`: optional chip override (e.g. `STM32G431CBTx`).
- `elf`: path to device ELF with `.defmt` section used for decoding logs. Defaults to `../device/target/thumbv7em-none-eabihf/release/oxifoc`.
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

- Device code: `device/src/main.rs`, `device/src/rtt_io.rs`.
- Host code: `host/src/main.rs`, `host/src/config.rs`.
- Protocol endpoints: `protocol/src/lib.rs` (Button, KeepAlive, Info).

## Debugging

You can view defmt logs either through the host application or directly via probe‑rs — use one at a time:

- Via host: run `cd host && cargo run --release` to stream defmt and ergot together.
- Via probe‑rs: attach with your preferred tool to view defmt output only.

For device-only debugging (flash + run):

```bash
cd device
../scripts/probe_run.sh target/thumbv7em-none-eabihf/release/oxifoc
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
