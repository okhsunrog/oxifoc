# Oxifoc

Field-Oriented Control (FOC) firmware for STM32 motor controllers, written in Rust with [Embassy](https://embassy.dev/). Device-host communication uses [ergot](https://github.com/jamesmunns/ergot).

## Project Structure

```
oxifoc/
├── Cargo.toml             # Workspace root
├── justfile               # Build automation (just check, just fmt, etc.)
├── TODO.md                # Implementation backlog
│
├── oxifoc-core/           # Platform-agnostic FOC algorithms and protocol types
├── oxifoc-g4/             # Shared code for STM32G4 platforms (Hall, CORDIC)
├── oxifoc-g431/           # STM32G431 firmware (B-G431B-ESC1)
├── oxifoc-g474/           # STM32G474 firmware (NUCLEO-G474RE + IHM08M1)
├── oxifoc-f405/           # STM32F405 firmware (Simple FOCer 2)
│
├── oxifoc-host-lib/       # Shared host backend (transport, protocol, config)
├── oxifoc-host-cli/       # CLI host tool
├── oxifoc-host-slint/     # Slint GUI with real-time WGPU charts
├── slint-wgpu-plot/       # GPU-accelerated plot renderer
│
├── oxifoc-virtual/        # Virtual device (FocController + VirtualMotor over TCP)
│
├── tests/stm32g431/       # On-target integration tests (G431)
├── tests/stm32g474/       # On-target integration tests (G474)
│
└── .github/workflows/     # CI: fmt, clippy, tests, device builds
```

Workspace members: `oxifoc-core`, `oxifoc-host-lib`, `oxifoc-host-cli`, `oxifoc-host-slint`, `slint-wgpu-plot`, `oxifoc-virtual`.

Device firmware crates are excluded from the workspace (different Rust toolchain, `thumbv7em-none-eabihf`).

## Hardware

| Board | MCU | Flash/RAM | Communication | Notes |
|-------|-----|-----------|---------------|-------|
| B-G431B-ESC1 | STM32G431CB | 128K/32K | UART or RTT | Built-in opamps, CORDIC |
| NUCLEO-G474RE | STM32G474RE | 512K/128K | UART or RTT | Dual-bank flash, CORDIC |
| Simple FOCer 2 | STM32F405RG | 1M/128K | USB | DRV8301, high current |

## Core Library (`oxifoc-core`)

Platform-agnostic FOC algorithms, fully tested on host (140+ unit tests):

- **Transforms**: Clarke/Park and inverses
- **SVPWM**: Space Vector PWM (VESC geometric sector method)
- **PI Controller**: Split into `PIController` (raw output, external anti-windup for FOC circular clamping) and `ClampedPI` (self-contained with rectangular limits)
- **FocController**: Full current loop with circular voltage limiting, `apply_dq()` for direct voltage mode
- **FocDriver**: Integrates controller + PWM + current sensor + phase manager
- **Hall Sensor**: 8-entry calibration table, soft drift correction, rate limiting, direction detection, majority voting, timeout detection
- **PhaseManager**: Hall → Observer → OpenLoop fallback chain with health tracking
- **Motor Detection**: Resistance, inductance (rotating HFI), flux linkage, Hall calibration
- **Virtual Motor**: PMSM simulation with closed-loop tests (forward, reverse, load rejection)
- **Config Storage**: 8 config groups with `sequential-storage` + `PostcardValue` for flash persistence
- **Protocol**: Ergot endpoints for motor control, ADC, Hall, faults, config read/write

## Device Firmware

- 20kHz center-aligned PWM with dead-time insertion
- TIM1-triggered injected ADC sampling (phase currents, Vbus, temperature)
- Hall sensor polling via TIM6 at 5us with 7-read majority voting
- CORDIC hardware sin/cos on G4 platforms
- Embassy async runtime with defmt logging
- Persistent configuration in internal flash (sequential-storage)
- Boot flow: load stored config → apply PI gains/motor params → calibrate current sensor → run FOC
- Fault detection: overcurrent, overvoltage, undervoltage, overtemperature
- Protocol endpoints: device info, motor control, ADC samples, Hall data, faults, config

## Host Tools

### Transport Options

| Transport | Use Case | Config |
|-----------|----------|--------|
| Serial | UART over ST-Link VCP | `--transport serial --serial-path /dev/ttyACM0 --baud 921600` |
| RTT | Debug probe (probe-rs) | `--transport rtt --chip STM32G431CBUx` |
| TCP | Virtual device | `--transport tcp --tcp-host 127.0.0.1 --tcp-port 2025` |

### CLI (`oxifoc-host-cli`)

```bash
# List available devices
just cli list

# Monitor ADC telemetry for 10 seconds
just cli -- --transport serial monitor --seconds 10

# Start motor at 10% duty
just cli -- --transport tcp start --duty 10

# Stop motor
just cli -- --transport tcp stop
```

### GUI (`oxifoc-host-slint`)

Slint-based desktop GUI with GPU-accelerated real-time charts (WGPU) for phase currents, bus voltage, and temperature.

```bash
just gui
```

### Virtual Device (`oxifoc-virtual`)

Runs a simulated motor controller with the full ergot protocol over TCP. Host tools connect to it exactly as they would to real hardware.

```bash
# Start virtual device (default: port 2025, 20kHz FOC, 24V bus)
cargo run -p oxifoc-virtual

# Connect with CLI
just cli -- --transport tcp monitor --seconds 5
```

CLI options: `--port`, `--foc-freq`, `--batch`, `--vbus`.

## Building

### Quick Commands

```bash
just check         # Full check: fmt + clippy + tests (workspace + all device firmware)
just fmt           # Format all code
just test          # Run workspace tests
just build g431    # Build device firmware (release)
just flash g431    # Flash device firmware
just gui           # Run Slint GUI
just cli -- list   # Run CLI
```

### Device Firmware

Requires Rust nightly with `thumbv7em-none-eabihf` target:

```bash
cd oxifoc-g431
cargo build --release
cargo run --release  # flash via probe-rs
```

Transport selection via features: `--features transport-uart` (default) or `--features transport-rtt`.

### Host Applications

```bash
cargo build --workspace  # all host crates
```

System dependencies (for Slint GUI): `libwayland-dev libxkbcommon-dev libudev-dev libfontconfig-dev`

## Configuration

### Host Config (`oxifoc-host.toml`)

Optional TOML config, loaded from `./oxifoc-host.toml` or `OXIFOC_HOST_CONFIG` env var:

```toml
transport = "serial"       # "serial", "rtt", or "tcp"
serial_path = "/dev/ttyACM0"
serial_baud = 921600
probe = "0483:374b"        # VID:PID for RTT
chip = "STM32G431CBUx"     # required for RTT
tcp_host = "127.0.0.1"     # for TCP transport
tcp_port = 2025
elf = "path/to/device.elf" # for defmt decoding
stream_defmt = true
stream_ergot = true
```

### Device Config (Flash Storage)

Persistent configuration stored in internal flash using `sequential-storage`:

| Config Group | Contents |
|-------------|----------|
| MotorParams | R, Ld, Lq, flux linkage, pole pairs |
| HallCalibration | Sector angles, validity flags |
| DcOffsets | Current sensor zero-offset per phase |
| CurrentLimits | Max Iq, max phase current |
| VoltageLimits | Min/max bus voltage thresholds |
| PwmConfig | Frequency, max duty percent |
| PiGains | Kp, Ki, bandwidth |
| HallTuning | Interpolation, drift correction, timeout |

Configs are loaded at boot and applied to the FOC controller. Read/write via the `ConfigEndpoint` protocol command.

## Testing

### Unit Tests

```bash
cargo test --workspace                              # all workspace tests (140+)
cargo test -p oxifoc-core --features virtual-motor  # include virtual motor tests
cargo test -p oxifoc-core --features virtual-motor,microfft  # include HFI tests
```

### On-Target Integration Tests

Run on real hardware via `embedded-test`:

```bash
cd tests/stm32g431 && cargo test  # requires G431 board connected
cd tests/stm32g474 && cargo test  # requires G474 board connected
```

Tests: CORDIC accuracy, transform round-trips, FOC with synthetic currents, SVPWM all sectors.

## CI

GitHub Actions runs on every push/PR:
- **fmt**: rustfmt check (workspace + all device crates)
- **clippy**: `-D warnings` on workspace
- **test**: `cargo test --workspace`
- **device**: matrix build for G431/G474/F405 (fmt + clippy + release build) with `Swatinem/rust-cache`

Concurrency groups cancel stale runs.

## Network Topology

Ergot DirectEdge profile (point-to-point):
- Host: controller at `1.1:0` (node 1)
- Device: target at `1.2:0` (node 2)

Same topology for serial, RTT, and TCP transports.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
