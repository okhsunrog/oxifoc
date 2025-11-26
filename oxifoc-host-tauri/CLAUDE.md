# CLAUDE.md

This file provides guidance to Claude Code when working with the Oxifoc Tauri GUI.

## Project Overview

Oxifoc Host Tauri is a Tauri + Vue 3 + TypeScript desktop application for controlling and monitoring the Oxifoc BLDC motor FOC system. It displays real-time ADC data (phase currents, voltage, temperature) and provides motor control.

## Location in Monorepo

This is part of the oxifoc monorepo:
```
oxifoc/
├── oxifoc-host-tauri/     # This directory - Tauri GUI
├── oxifoc-host-lib/       # Core backend library
├── oxifoc-host-cli/       # CLI tool
├── oxifoc-host-egui/      # egui GUI
├── protocol/              # Shared protocol types
└── oxifoc-device/         # Device firmware (excluded from workspace)
```

## Commands

**From monorepo root (using justfile):**
```bash
just install      # Install frontend dependencies
just dev          # Run Tauri dev server
just build        # Build Tauri release
just format       # Format all code
just lint         # Lint all code
```

**From this directory:**
```bash
bun install            # Install dependencies
bun tauri dev          # Development server (generates TypeScript bindings)
bun tauri build        # Production build
bun run type-check     # TypeScript type checking
bun run lint           # ESLint with auto-fix
bun run format         # Prettier format
```

## Architecture

### Rust Backend (`src-tauri/`)

The Tauri backend integrates with `oxifoc-host-lib` for device communication:

**Key dependencies:**
- `oxifoc-host-lib`: Core backend (starts serial connection, manages channels)
- `oxifoc-protocol`: Shared types (AdcSample, MotorCommand, etc.)

**Commands:**
- `init_device_connection()`: Starts the host backend
- `is_device_connected()`: Checks connection status
- `wait_for_device(timeout_secs)`: Waits for device handshake
- `motor_start(duty)`: Start motor at duty cycle (0-100%)
- `motor_stop()`: Stop motor
- `motor_set_speed(duty)`: Adjust duty while running
- `start_adc_stream(channel)`: Stream ADC samples to frontend
- `get_adc_sample()`: Get single sample (non-blocking)

**Data types (from protocol):**
```rust
struct AdcSample {
    ia: u16,           // Phase A current (raw ADC, 0-4095)
    ib: u16,           // Phase B current
    ic: u16,           // Phase C current
    vbus_mv: u32,      // DC bus voltage in millivolts
    fet_temp_c_x10: u16, // Temperature in 0.1°C units
    seq: u32,          // Sequence number
}

enum MotorCommand {
    Stop,
    Start { duty: u8 },
    SetSpeed { duty: u8 },
}
```

### Frontend (`src/`)

**Key stores:**
- `streamStore.ts`: Manages device connection and ADC data streaming

**Key components:**
- `TimeChartStream.vue`: WebGL-accelerated real-time chart for phase currents
- `ChartSwitcher.vue`: Chart container with window size controls

### Type-Safe Rust ↔ TypeScript Bridge

Uses **tauri-specta** for automatic bindings:
1. Rust commands annotated with `#[tauri::command]` and `#[specta::specta]`
2. Running `bun tauri dev` regenerates `src/bindings.ts`
3. Import via `import { commands, type AdcSample } from './bindings'`

**IMPORTANT**: The Rust crate name is `oxifoc_host_tauri_lib` (underscore, not hyphen).

### TimeChart Requirements

Use the npm beta series (`timechart@1.0.0-beta.10` or later). Older npm `0.5.x` lacks DataPointsBuffer mutation tracking and will break under sustained streaming.

## Configuration

Device connection is configured via `oxifoc-host.toml` in the monorepo root:
```toml
serial_path = "/dev/ttyACM0"
serial_baud = 921600
```

Or via environment variable: `OXIFOC_HOST_CONFIG=/path/to/config.toml`

## Development Notes

**Adding new Tauri commands:**
1. Define function in Rust with `#[tauri::command]` and `#[specta::specta]`
2. Add to `collect_commands![]` macro in `lib.rs`
3. Run `bun tauri dev` to regenerate bindings
4. Use from TypeScript via `commands.yourFunction()`

**Tracing Filters:**
- Debug builds: `oxifoc_host_tauri_lib=trace`
- Release builds: `oxifoc_host_tauri_lib=info`
- Respects `RUST_LOG` environment variable
