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

## Current Capabilities

- **Device firmware:**
  - Button input (single/double/hold), keepalive, device info server over ergot
  - Hall sensor angle estimation (6-state, async edge detection)
  - ADC sampling (phase currents, Vbus, FET temperature)
  - Embassy async runtime with defmt logging
  - Supports Serial (UART) or RTT transport (compile-time feature)
  - Protocol endpoints: button events, device info, motor control, ADC samples, Hall sensor data

- **FOC Core Library** (`oxifoc-core`):
  - Clarke/Park transforms (ABC → αβ → dq and inverse)
  - Space Vector PWM (VESC geometric sector method)
  - PI controller with anti-windup
  - Hall handling (software expectations inspired by VESC `mcpwm_foc`):
    - Calibrate per-motor Hall advance/offset and state sequence.
    - Reject invalid transitions; count errors and set a fault on repeated bad states.
    - Interpolate electrical angle between Hall edges using measured speed for smoother low-speed FOC.
    - Apply calibrated offset before Park transforms.
  - See `docs/cheap-focer2-notes.md` for the F405 pin/filter specifics (hardware).
  - Fully tested on x86_64 (33 unit tests)
  - Hall support: expect per-motor calibration (offset/sequence), apply offset before Park, and optionally blend an estimated angle from velocity for smoother operation at low speeds (similar to VESC). Calibration and interpolation are TODO in device firmware.

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
- ✅ Platform-agnostic FOC algorithm library (`oxifoc-core`)
  - Clarke/Park transforms and inverses
  - Space Vector PWM (VESC geometric method)
  - PI controller with anti-windup
- ✅ Basic Hall sensor support
  - Platform-agnostic Hall sensor logic (6-state, fixed 60° spacing)
  - STM32G431 async driver using ExtiInput
  - Hall sensor telemetry endpoint (`req/hall`)
  - Direction detection and error tracking

### In Progress / Next Steps

#### Phase 2: Current Sensing (FOC Foundation)
- ADC path for phase current sampling (synchronized with PWM)
  - Use existing injected ADC channels (ia, ib, ic)
  - Offset calibration routine (measure with motor stopped)
  - Current measurement telemetry
- Test current readings with known load

#### Phase 3: PWM Generation for FOC
- TIM1 3-phase complementary PWM configuration
  - Safe dead-time insertion (prevent shoot-through)
  - Center-aligned PWM for ADC synchronization
- Integrate SVPWM algorithm with TIM1
- Emergency shutdown on fault conditions
- Safe startup sequence

#### Phase 4: FOC Current Control Loop
- Implement Id/Iq current control (dq frame)
  - Use PI controllers from `oxifoc-core`
  - Runtime gain tuning via host
- Velocity control layer (outer loop)
- Torque/current limit enforcement
- Field weakening (optional, for high-speed operation)

### Advanced Hall Sensor Features (VESC-inspired)
**Priority: Medium** (enhance after basic FOC works)

- **Hall sensor calibration** (`mcpwm_foc_hall_detect` equivalent)
  - Automatic calibration routine: rotate motor via forced commutation
  - Learn actual electrical angle for each Hall state (store in calibration table)
  - Account for Hall sensor mounting misalignment
  - Store calibration in non-volatile memory

- **Hall angle interpolation** (`foc_correct_hall` equivalent)
  - Linear interpolation between Hall transitions using estimated speed
  - Speed calculation from Hall transition timing (`hall_dt_diff`)
  - Speed-dependent interpolation threshold (`foc_hall_interp_erpm`)
  - Midpoint angle on Hall state transitions

- **Hall angle rate limiting**
  - Limit maximum angle change per update (prevent current spikes)
  - Smooth angle transitions during acceleration/deceleration

- **Hall error detection and recovery**
  - Invalid state detection (all high/low, illegal transitions)
  - Glitch filtering (debounce, majority voting)
  - Fallback to sensorless observer on Hall failure
  - Error rate monitoring and reporting

- **Hall-sensorless hybrid mode** (VESC `FOC_SENSOR_MODE_HALL`)
  - Use Hall at low speed (< `foc_sl_erpm` threshold)
  - Transition to sensorless observer at higher speeds
  - Smooth blending between Hall and observer angles

### Future Enhancements

#### Encoder Support
- Incremental encoder (ABI) support
  - High-resolution angle feedback (> 6 states)
  - Index pulse for absolute positioning
- Absolute encoder support (SPI-based: AS5047, MT6816, etc.)

#### Sensorless Control
- Sensorless FOC (observer-based, VESC `m_observer`)
  - Back-EMF observer for angle estimation
  - High-frequency injection (HFI) for zero/low speed
  - Automatic transition from sensored to sensorless

#### Safety & Protection
- Over-current protection (hardware + software limits)
- Over-voltage / under-voltage detection
- Over-temperature monitoring (FET, motor)
- Fault latching and safe shutdown
- Watchdog integration

#### Telemetry & Diagnostics
- High-speed telemetry streaming (not just poll-based)
- Capture buffers for scope-like tuning (ia, ib, angle, duty)
- Motor parameter identification (resistance, inductance, flux linkage)
- Performance metrics (efficiency, power, RPM)

#### Host Tooling
- Calibration wizards (Hall, current offset, motor params)
- Real-time plotting and tuning UI
- Configuration save/load (non-volatile storage)
- Firmware update via bootloader

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
