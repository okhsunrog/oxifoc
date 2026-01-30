# CLAUDE.md

This file provides guidance to Claude Code when working with the Oxifoc Tauri GUI.

## Project Overview

Oxifoc Host Tauri is a Tauri + Vue 3 + TypeScript desktop application for controlling and monitoring the Oxifoc BLDC motor FOC system. It displays real-time ADC data (phase currents, voltage, temperature) and provides motor control.

## Location in Monorepo

This is part of the oxifoc monorepo:
```
oxifoc/
├── oxifoc-host-tauri/     # This directory - Tauri GUI
├── oxifoc-host-flutter/   # Flutter GUI (cross-platform)
├── oxifoc-host-lib/       # Core backend library
├── oxifoc-host-cli/       # CLI tool
├── oxifoc-host-egui/      # egui GUI
└── oxifoc-core/           # Shared protocol types
```

## Commands

**From this directory:**
```bash
bun install            # Install dependencies
bun tauri dev          # Development server (generates TypeScript bindings)
bun tauri build        # Production build
bun run type-check     # TypeScript type checking
bun run lint           # ESLint with auto-fix
bun run format         # Prettier format
```

## Tech Stack

**Frontend:**
- Vue 3 + TypeScript
- TailwindCSS v4 + DaisyUI for styling
- ECharts (vue-echarts) for real-time plotting
- xterm.js for terminal/log display
- Pinia for state management

**Backend:**
- Tauri 2.x with tauri-specta for type-safe bindings
- oxifoc-host-lib for device communication

## Architecture

### Rust Backend (`src-tauri/`)

The Tauri backend integrates with `oxifoc-host-lib` for device communication:

**Key dependencies:**
- `oxifoc-host-lib`: Core backend (starts serial connection, manages channels)
- `oxifoc-core`: Shared types (AdcSample, ControlMode, etc.)

**Commands:**
- `init_device_connection()`: Starts the host backend
- `is_device_connected()`: Checks connection status
- `wait_for_device(timeout_secs)`: Waits for device handshake
- `start_adc_stream(channel)`: Stream ADC samples to frontend

**Data types (from oxifoc-core):**
```rust
struct AdcSample {
    ia: u16,           // Phase A current (raw ADC, 0-4095)
    ib: u16,           // Phase B current
    ic: u16,           // Phase C current
    vbus_mv: u32,      // DC bus voltage in millivolts
    fet_temp_c_x10: u16, // Temperature in 0.1°C units
    seq: u32,          // Sequence number
}
```

### Frontend (`src/`)

**Stores:**
- `streamStore.ts`: Manages ADC data streaming with client-side timestamping and normalization
- `connectionStore.ts`: Device connection state
- `terminalStore.ts`: Terminal/log output

**Key components:**
- `charts/EChartsPlot.vue`: High-performance real-time chart with pre-allocated tuple pools, binary search windowing, LTTB sampling
- `charts/VbusTempPlot.vue`: Voltage and temperature chart
- `terminal/TerminalDisplay.vue`: xterm-based log viewer
- `ConnectionCard.vue`: Device connection UI
- `ControlBar.vue`: Motor control interface

### ECharts Performance Optimizations

The plotting implementation uses several techniques to minimize GC pressure at 60Hz update rates:
- Pre-allocated tuple pools (`[number, number][]`) created at startup
- In-place tuple mutation instead of creating new arrays
- Binary search for time-based windowing
- LTTB (Largest Triangle Three Buckets) sampling for large datasets
- `shallowRef` for samples array to avoid deep Vue reactivity

### Type-Safe Rust ↔ TypeScript Bridge

Uses **tauri-specta** for automatic bindings:
1. Rust commands annotated with `#[tauri::command]` and `#[specta::specta]`
2. Running `bun tauri dev` regenerates `src/bindings.ts`
3. Import via `import { commands, type AdcSample } from './bindings'`

**IMPORTANT**: The Rust crate name is `oxifoc_host_tauri_lib` (underscore, not hyphen).

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
