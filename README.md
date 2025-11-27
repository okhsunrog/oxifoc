# Oxifoc

WIP/experimental motor control (FOC) firmware for STM32G431 (B‑G431B‑ESC1) with a lightweight host tool. Device↔host communication uses [ergot](https://github.com/jamesmunns/ergot) over either **Serial (UART)** or **RTT**.

## Project Structure

```
oxifoc/
├── Cargo.toml          # Workspace root (host + core)
├── justfile            # Build automation
├── oxifoc-core/        # Platform-agnostic FOC algorithms (testable)
├── oxifoc-g431/        # STM32G431 firmware (excluded from workspace)
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
- **Debug Interface**: ST-Link V2 (built-in)
- **Communication**: Serial (via ST-Link VCP) or RTT (via probe-rs)

See [docs/hardware.md](docs/hardware.md) for detailed pinout and functional groups.

## Current Capabilities (short)

- Device: button input (single/double/hold), keepalive, and device info server over ergot; defmt logs; Embassy async runtime. Supports Serial (UART) or RTT transport (compile-time feature).
- Host: connects via Serial (ST-Link VCP) or RTT (probe-rs), streams defmt and ergot, queries DeviceInfo on connect.
- Tauri GUI: desktop/mobile app with transport selection, real-time ADC charts, and log level controls.

## Building

### Device Firmware

```bash
cd oxifoc-g431

# Serial transport (default, uses ST-Link VCP at 921600 baud)
cargo build --release --features transport-uart

# RTT transport (uses probe-rs RTT channels)
cargo build --release --features transport-rtt
```

Note: Only one transport feature can be enabled at a time.

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

Using probe-rs:

```bash
cd oxifoc-g431

# Flash with Serial transport
cargo run --release --features transport-uart

# Flash with RTT transport
cargo run --release --features transport-rtt
```

The device will initialize the selected transport and start the ergot communication stack.

### Run Host Application (Tauri GUI)

```bash
cd oxifoc-host-tauri
bun install
bun tauri dev
```

The Tauri GUI allows selecting transport (Serial or RTT) and provides real-time ADC charts and log controls.

### Run Host Application (egui)

```bash
cargo run --manifest-path oxifoc-host-egui/Cargo.toml --release
```

Note: For RTT transport, ensure no other `probe-rs` session is running (the ST‑Link/RTT connection can only be owned by one process at a time).

#### Configuration (TOML)

The host reads an optional `oxifoc-host.toml` in the current working directory (or from `OXIFOC_HOST_CONFIG` env var):

```toml
# Transport: "serial" or "rtt" (default: serial)
transport = "serial"

# Serial transport options
serial_path = "/dev/ttyACM0"  # Auto-detected if not set
serial_baud = 921600          # Default: 921600

# RTT transport options
probe = "0483:374b"           # VID:PID or VID:PID:SERIAL
chip = "STM32G431CBUx"

# Path to device ELF for defmt decoding
elf = "/path/to/device.elf"

# Enable/disable channel streaming (both default to true)
stream_defmt = true
stream_ergot = true
```

### Transport Details

**Serial (UART) transport** (`transport-uart` feature):
- Uses ST-Link V2's built-in VCP (Virtual COM Port)
- UART pins: PB3 (TX), PB4 (RX)
- Default baud rate: 921600, 8N1
- Both defmt and ergot are multiplexed over a single UART (defmt forwarded via ergot network)

**RTT transport** (`transport-rtt` feature):
- Uses probe-rs RTT channels via ST-Link
- Channel map:
  - **up0 "defmt"**: Debug logging output (via defmt macros)
  - **up1 "ergot"**: COBS-framed protocol messages (device→host)
  - **down0 "ergot-down"**: Host→device protocol messages
- Separate channels for defmt and ergot (parallel streaming)

## Network Topology

Ergot DirectEdge profile (point‑to‑point):
- Host: controller at `1.1.0`
- Device: target at `1.2.0`

## Development Notes (short)

- Device code: `oxifoc-g431/src/main.rs`.
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

View defmt logs via the host application (Tauri GUI, egui, or CLI). For RTT transport, only one process can hold the ST-Link/RTT connection at a time.

For device-only debugging with RTT:

```bash
cd oxifoc-g431
cargo run --release --features transport-rtt
```

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
