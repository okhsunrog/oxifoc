# RTT Communication Setup

This document describes how ergot communication over RTT works in the oxifoc project.

## Overview

The project uses RTT (Real-Time Transfer) via probe-rs to communicate between the STM32G431 device and the PC host application. Currently, the setup uses defmt for logging over RTT, with plans to extend it for full ergot message passing.

## Device Side (STM32G431)

The device firmware uses:
- **defmt-rtt**: For debug logging over RTT
- **ergot**: For structured message passing (currently using `Null` profile)

### Current RTT Configuration

In `device/Cargo.toml`:
```toml
defmt = { version = "1.0.1", optional = true }
defmt-rtt = { version = "1.0.0", optional = true }
```

The firmware uses defmt logging macros (`info!`, `warn!`, `error!`) which are transmitted over RTT channel 0.

### Future: Ergot over RTT

To enable full ergot communication over RTT, you'll need to:

1. Add a custom RTT channel for ergot data (separate from defmt logs)
2. Implement an ergot interface using the `embedded-io-async` toolkit
3. Use RTT channels as the transport layer for ergot frames

Example approach:
```rust
// Create RTT channels for ergot
let channels = rtt_target::rtt_init! {
    up: {
        0: { // defmt logs
            size: 1024,
            mode: NoBlockSkip,
        }
        1: { // ergot data
            size: 2048,
            mode: BlockIfFull,
        }
    }
};

// Use channel 1 for ergot communication
// Implement Read/Write traits for the channel
// Use with ergot's embedded-io-async toolkit
```

## Host Side (PC)

The host application (`host/`) uses:
- **probe-rs**: To connect to the debug probe (ST-Link)
- **probe-rs-rtt**: To attach to RTT channels

### Current Implementation

The host currently:
1. Connects to the STM32G431 via ST-Link
2. Attaches to RTT channels
3. Reads and displays defmt log output from channel 0

### Future: Full Ergot Support

To implement full ergot communication:

1. Read from RTT channel 1 (ergot data channel)
2. Parse ergot frames from the RTT data
3. Use ergot's tokio-based router to handle messages
4. Implement bidirectional communication (read from channel 1, write to down channel)

Example:
```rust
// Read from ergot RTT channel
let ergot_channel = rtt.up_channels().take(1)?;

// Parse ergot frames and feed to ergot stack
// Create ergot router on host side
// Handle ButtonEvent messages
```

## Debugging

### View RTT logs

Use the host application:
```bash
cd host
RUST_LOG=info cargo run
```

Or use probe-rs directly:
```bash
probe-rs attach STM32G431CBUx
```

### Common Issues

1. **"No probes found"**: Ensure ST-Link is connected and drivers are installed
2. **"Failed to attach RTT"**: Make sure firmware is running with defmt-rtt enabled
3. **No output**: Check that the firmware is built with the `debug` feature enabled

## References

- [probe-rs Documentation](https://probe.rs/)
- [RTT (Real-Time Transfer)](https://wiki.segger.com/RTT)
- [ergot Documentation](https://github.com/jamesmunns/ergot)
- [defmt Book](https://defmt.ferrous-systems.com/)
