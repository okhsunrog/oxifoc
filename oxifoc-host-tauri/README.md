# Oxifoc Host Tauri

Desktop GUI for controlling and monitoring the Oxifoc BLDC motor FOC system. Built with Tauri 2, Vue 3, TypeScript, Tailwind CSS v4, and DaisyUI v5.

## Quick Start

**Prerequisites**: Rust, Bun, platform-specific WebView libraries (see Tauri docs)

```bash
# From monorepo root (using justfile)
just install && just dev

# Or from this directory
bun install && bun tauri dev
```

## Features

- Multi-transport support (Serial UART / RTT debug probe)
- Automatic device discovery with USB-Serial filtering
- Real-time ADC visualization (phase currents, voltage, temperature)
- Motor control interface (start/stop, duty cycle adjustment)

## Configuration

Create `oxifoc-host.toml` in monorepo root:

```toml
serial_path = "/dev/ttyACM0"
serial_baud = 921600
```

Or set `OXIFOC_HOST_CONFIG=/path/to/config.toml`

## Development

```bash
bun tauri dev          # Dev server with hot-reload
bun tauri build        # Production build
bun type-check         # TypeScript check
bun lint:check         # ESLint
bun format             # Format with Prettier
```

**Adding Tauri commands**: See `CLAUDE.md` for architecture and development guide.

## Critical Notes

- **Type bindings**: Auto-generated in `src/bindings.ts` via tauri-specta
