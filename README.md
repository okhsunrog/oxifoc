# Oxifoc

Field-Oriented Control (FOC) firmware for STM32 motor controllers, written in Rust. Supports STM32G431 (B-G431B-ESC1) and STM32F405 (Cheap FOCer 2). Device↔host communication uses [ergot](https://github.com/jamesmunns/ergot) over either **Serial (UART)** or **RTT**.

## Project Structure

```
oxifoc/
├── Cargo.toml          # Workspace root (host + core)
├── justfile            # Build automation
├── oxifoc-core/        # Platform-agnostic FOC algorithms (testable)
├── oxifoc-g431/        # STM32G431 firmware (B-G431B-ESC1)
├── oxifoc-f405/        # STM32F405 firmware (Cheap FOCer 2)
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

This repo uses a Cargo workspace for host crates. Device firmware crates are excluded (different toolchain).

## Hardware

### Supported Boards

| Board | MCU | Features |
|-------|-----|----------|
| B-G431B-ESC1 | STM32G431CB | CORDIC accelerator, compact form factor |
| Cheap FOCer 2 | STM32F405RG | Higher current capacity, more GPIO |

- **Debug Interface**: ST-Link V2 (built-in or external)
- **Communication**: Serial (via ST-Link VCP) or RTT (via probe-rs)

See [docs/hardware.md](docs/hardware.md) for detailed pinout and functional groups.

## Current Capabilities

- **FOC Core Library** (`oxifoc-core`):
  - Clarke/Park transforms (ABC → αβ → dq and inverse)
  - Space Vector PWM (VESC geometric sector method)
  - PI controller with anti-windup
  - **Hall sensor with VESC-compatible features:**
    - 8-entry raw-state calibration table (works with any Hall wiring)
    - Soft drift correction (1% pull-back toward sector angle)
    - Rate limiting to prevent current spikes on transitions
    - Low-speed interpolation threshold (configurable in eRPM)
    - Direction reversal detection with clean velocity handling
    - Invalid state detection with recovery reset
    - Timeout detection for failure monitoring
  - **PhaseManager with automatic fallback:**
    - Hall → Observer → OpenLoop fallback chain
    - Hall health tracking (Ok, Stale, Invalid, NotPresent)
    - Velocity-based Hall-to-Observer blending
    - Fault system for error reporting
  - **Motor parameter detection:**
    - Resistance measurement
    - Inductance measurement (rotating HFI)
    - Flux linkage measurement
    - Hall sensor calibration
  - Back-EMF observer (sensorless, untested)
  - Fully tested on x86_64 (100+ unit tests)

- **Device firmware:**
  - FOC current control loop (20kHz PWM)
  - Hall sensor polling with 7-read majority voting (noise immunity)
  - ADC sampling (phase currents, Vbus, temperature)
  - Embassy async runtime with defmt logging
  - Supports Serial (UART) or RTT transport
  - Protocol endpoints: button events, device info, motor control, ADC samples, Hall sensor data

- **Host applications:**
  - Connects via Serial (ST-Link VCP) or RTT (probe-rs)
  - Streams defmt logs and ergot protocol messages
  - Tauri GUI: transport selection, real-time ADC charts, log level controls
  - CLI and egui frontends available

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

## Development Notes

- **FOC algorithms**: `oxifoc-core/src/foc/` (transforms, SVPWM, PI controller, Hall sensor)
- **Device firmware**: `oxifoc-g431/src/main.rs` (main task orchestration)
  - Motor control: `oxifoc-g431/src/motor/` (PWM, six-step, Hall sensor driver)
- **Protocol**: `protocol/src/lib.rs` (Button, Motor, AdcSample, HallSensor, Info endpoints)
- **Host backend**: `oxifoc-host-lib/src/lib.rs` (+ `config.rs` for TOML parsing)
- **Host frontends**:
  - Tauri GUI: `oxifoc-host-tauri/` (desktop/mobile)
  - egui: `oxifoc-host-egui/src/main.rs`
  - CLI: `oxifoc-host-cli/src/main.rs`

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

## Roadmap

### Completed

- ✅ **FOC Core Library** (`oxifoc-core`)
  - Clarke/Park transforms and inverses
  - Space Vector PWM (VESC geometric method)
  - PI controller with anti-windup

- ✅ **Hall Sensor (VESC-compatible)**
  - 8-entry raw-state calibration table
  - Soft drift correction and rate limiting
  - Low-speed interpolation threshold (eRPM-based)
  - Direction reversal detection
  - Invalid state detection with recovery reset
  - Timeout detection for failure monitoring
  - 7-read majority voting for noise immunity

- ✅ **PhaseManager with Fallback**
  - Hall → Observer → OpenLoop fallback chain
  - Hall health tracking (Ok, Stale, Invalid)
  - Velocity-based Hall-to-Observer blending
  - Fault system for error reporting

- ✅ **Motor Parameter Detection**
  - Resistance measurement
  - Inductance measurement (rotating HFI)
  - Flux linkage measurement
  - Hall sensor calibration

- ✅ **FOC Current Control Loop**
  - Id/Iq current control (dq frame)
  - Center-aligned PWM with dead-time
  - ADC synchronized to PWM

### In Progress

- 🔄 **Back-EMF Observer** - Implemented but needs testing
- 🔄 **Non-volatile storage** - Save calibration/config to flash

### Planned

#### Velocity & Position Control
- Velocity control outer loop
- Position control outer loop
- Torque/current limit enforcement
- Field weakening for high-speed operation

#### Encoder Support
- Incremental encoder (ABI) support
- Index pulse for absolute positioning
- Absolute encoder support (SPI-based: AS5047, MT6816)

#### Sensorless Control
- HFI angle tracking for zero/low speed
- Automatic sensored→sensorless transition

#### Safety & Protection
- Over-current protection (hardware + software)
- Over-voltage / under-voltage detection
- Over-temperature monitoring
- Fault latching and safe shutdown
- Watchdog integration

#### Host Tooling
- Calibration wizards (Hall, current offset, motor params)
- Real-time plotting and tuning UI
- Configuration save/load
- Firmware update via bootloader

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
