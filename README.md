# Oxifoc

Open FOC (Field-Oriented Control) implementation using [ergot](https://github.com/jamesmunns/ergot) for communication between device and host.

## Project Structure

```
oxifoc/
├── device/          # STM32G431 firmware (B-G431B-ESC1 board)
├── host/            # PC-side application for RTT communication
├── protocol/        # Shared protocol definitions
├── ergot/           # ergot submodule for messaging
└── docs/            # Documentation
```

This project does NOT use a Cargo workspace at the root level, as different targets (embedded MCU vs. host) require separate configurations.

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
```

The host application will:
1. Connect to the STM32G431 via ST-Link
2. Attach to RTT channels
3. Display button events and other debug output

## Protocol

The shared protocol (in `protocol/`) defines:
- `ButtonEvent`: Single click, double click, hold events
- `ButtonEndpoint`: Ergot endpoint for button communication

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
