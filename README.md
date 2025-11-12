# Oxifoc

Open FOC (Field-Oriented Control) implementation using [ergot](https://github.com/jamesmunns/ergot) for communication between device and host.

## Project Structure

```
oxifoc/
├── device/          # STM32G431 firmware (B-G431B-ESC1 board)
├── host/            # PC-side application for RTT communication
├── protocol/        # Shared protocol definitions
└── docs/            # Documentation
```

This project does NOT use a Cargo workspace at the root level, as different targets (embedded MCU vs. host) require separate configurations.
We depend on the `ergot` crate from crates.io (no submodule required).

## Hardware

- **Board**: B-G431B-ESC1
- **MCU**: STM32G431CB
- **Debug Interface**: ST-Link v3
- **Communication**: RTT (Real-Time Transfer) via probe-rs

## Features

- Button input handling (single click, double click, hold)
- Ergot-based messaging protocol
- RTT communication for host-device interaction
- Embassy async runtime on device
- Tokio async runtime on host

## Building

### Device Firmware

```bash
cd device
cargo build --release

# Optional: build with defmt logging instead of Ergot fmt-topic
cargo build --release --no-default-features --features logs_via_defmt
```

### Host Application

```bash
cd host
cargo build --release
```

## Flashing

Flash the device firmware using probe-rs:

```bash
cd device
cargo run --release
```

Or using your preferred flashing tool with the generated binary.

## Running the Host Application

With the board connected via ST-Link:

```bash
cd host
cargo run

# Optional: select explicit log source and/or chip
cargo run -- --log-source ergot
cargo run -- --log-source defmt
cargo run -- --chip STM32G431CBTx
```

The host application will:
1. Connect to the STM32G431 via ST-Link
2. Attach to RTT and select a log source
3. Display logs and other debug output

Log source selection (runtime):
- `--log-source auto` (default): Prefer `ergot` channel, else `defmt` if present
- `--log-source ergot`: Read Ergot fmt-topic logs from RTT up channel named `ergot`
- `--log-source defmt`: Read the RTT up channel named `defmt` and print raw bytes

Note: defmt decoding with an ELF file will be added later (TODO); for now, defmt output is streamed as-is.

RTT channel map (device)
- up0: `defmt` (defmt logs when enabled)
- up1: `ergot` (COBS-framed Ergot frames, including fmt-topic logs)
- down0: `ergot-down` (reserved for future host→device messages)

Logging modes (device)
- Default (transport-agnostic): logs go over Ergot fmt-topic, carried by the selected interface (RTT in this project). Build normally.
- Defmt mode: logs go over defmt RTT (up0). Build with `--no-default-features --features logs_via_defmt`.

Host chip selection
- By default, the host auto-detects the target with `TargetSelector::Auto`.
- Override with `--chip <name>` (e.g., `STM32G431CBTx`) if needed.

## Protocol

The shared protocol (in `protocol/`) defines:
- `ButtonEvent`: Single click, double click, hold events
- `ButtonEndpoint`: Ergot endpoint for button communication

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
