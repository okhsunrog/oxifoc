# Oxifoc

Open FOC (Field-Oriented Control) implementation using [ergot](https://github.com/jamesmunns/ergot) for communication between device and host.

## Project Structure

```
oxifoc/
├── device/          # STM32G431 firmware (B-G431B-ESC1 board)
├── host/            # PC-side application for RTT communication
├── protocol/        # Shared protocol definitions
├── docs/            # Documentation
└── scripts/         # Helper scripts
```

This project does NOT use a Cargo workspace at the root level, as different targets (embedded MCU vs. host) require separate configurations.

## Hardware

- **Board**: B-G431B-ESC1
- **MCU**: STM32G431CB (Cortex-M4F with hardware FPU)
- **Debug Interface**: ST-Link
- **Communication**: RTT (Real-Time Transfer) via probe-rs

## Features

### Device Firmware
- Button input handling (single click, double click, hold detection)
- Ergot DirectEdge protocol over RTT with COBS framing
- Embassy async runtime (version 0.9+)
- Multiple async tasks:
  - Button event detection and transmission
  - Network event handling (ergot server)
  - Periodic keepalive messages (every 3 seconds)
  - Device info request/response server
- Defmt logging for debugging
- Custom RTT I/O implementation using `embedded-io-async` traits
- Hard float support, optimized for size (`opt-level = "z"`)
- Rust Edition 2024

### Host Application
- Probe-rs based RTT communication
- Ergot DirectEdge stack in controller mode
- Tokio async runtime
- Simultaneous streaming of:
  - Defmt debug logs (decoded using device ELF)
  - Ergot protocol messages (COBS-framed)
- Button event and keepalive message handlers
- Device info query at startup
- Configuration via optional TOML file

### Protocol
The shared protocol (`protocol/`) defines three ergot endpoints:

- **ButtonEndpoint**: Device→Host button events (SingleClick, DoubleClick, Hold)
- **KeepAliveEndpoint**: Device→Host periodic keepalive with sequence number
- **InfoEndpoint**: Host→Device info query, returns hardware/software version

## Building

### Device Firmware

```bash
cd device
cargo build --release
```

The firmware uses:
- Rust toolchain: 1.89
- Target: `thumbv7em-none-eabihf` (Cortex-M4F with hard float)
- Edition: 2024

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

The host will:
1. Connect to the STM32G431 via ST-Link
2. Attach to RTT channels
3. Start ergot DirectEdge stack in controller mode (network 1, node 1)
4. Stream both defmt logs and ergot messages simultaneously
5. Query device info at startup
6. Display button events and keepalive messages

#### Configuration

The host can be configured via an optional `oxifoc-host.toml` file in the working directory:

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

Alternatively, set the `OXIFOC_HOST_CONFIG` environment variable to point to a config file.

### RTT Channel Map

The device firmware configures RTT channels as follows:

- **up0 "defmt"**: Debug logging output (via defmt macros)
- **up1 "ergot"**: COBS-framed protocol messages (device→host)
- **down0 "ergot-down"**: Reserved for host→device protocol messages

Both channels operate simultaneously - defmt for debug logs, ergot for structured protocol communication. The host application reads from both channels in parallel.

## Network Topology

The project uses ergot's DirectEdge profile for point-to-point communication:

- **Host**: Controller mode, address (network_id: 1, node_id: 1)
- **Device**: Target mode, address (network_id: 1, node_id: 2)

The device sends messages to the host's router address (1.1.0), and the host can send requests to the device address (1.2.0).

## Development

### Device Firmware

The device firmware is structured with:
- `main.rs`: Entry point, task spawning, RTT initialization
- `rtt_io.rs`: RTT channel wrappers implementing `embedded-io-async` traits

Key dependencies:
- `embassy-stm32` 0.4.0 (STM32G431CB support)
- `embassy-executor` 0.9.1 (async runtime)
- `ergot` 0.12.0 (messaging protocol)
- `defmt` 1.0.1 (logging)
- `rtt-target` 0.6.2 (RTT communication)

### Host Application

The host is structured with:
- `main.rs`: RTT connection, channel streaming, ergot stack setup
- `config.rs`: Optional TOML configuration loading

Key dependencies:
- `probe-rs` 0.30 (debug probe and RTT)
- `ergot` 0.12.0 (messaging protocol)
- `tokio` 1.45 (async runtime)
- `defmt-decoder` 1.0 (defmt log decoding)

### Protocol

The protocol is no_std compatible and uses:
- `ergot` 0.12.0 (endpoint definitions)
- `serde` 1.0 + `postcard-schema` 0.2.5 (serialization)
- `heapless` 0.9.2 (no_std collections)

## Debugging

View logs with the host application running, or use probe-rs directly:

```bash
probe-rs attach STM32G431CBTx
```

For device firmware development, you can use the helper script:

```bash
cd device
../scripts/probe_run.sh target/thumbv7em-none-eabihf/release/oxifoc
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
