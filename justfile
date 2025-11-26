# Oxifoc Monorepo Build Automation

# Default recipe shows available commands
default:
    @just --list

# Install frontend dependencies
install:
    cd oxifoc-host-tauri && bun install

# Run Tauri development server
dev:
    cd oxifoc-host-tauri && bun tauri dev

# Build Tauri application (release)
build:
    cd oxifoc-host-tauri && bun tauri build

# Run egui application
egui:
    cargo run -p oxifoc-host-egui

# Run CLI tool with arguments
cli *ARGS:
    cargo run -p oxifoc-host-cli -- {{ARGS}}

# Format all Rust code
fmt:
    cargo fmt --all

# Format frontend code
fmt-ts:
    cd oxifoc-host-tauri && bun format

# Format all code (Rust + TypeScript)
format: fmt fmt-ts

# Lint Rust code (clippy)
clippy:
    cargo clippy --workspace --all-targets -- -D warnings

# Lint frontend code (eslint)
lint-ts:
    cd oxifoc-host-tauri && bun lint:check

# Lint all code
lint: clippy lint-ts

# Type check frontend
type-check:
    cd oxifoc-host-tauri && bun type-check

# Check all workspace crates compile
check:
    cargo check --workspace

# Build all workspace crates (debug)
build-all:
    cargo build --workspace

# Build all workspace crates (release)
build-release:
    cargo build --workspace --release

# Run tests for all workspace crates
test:
    cargo test --workspace

# Flash device firmware (release build)
flash:
    cd oxifoc-device && cargo flash --release --chip STM32G431CBTx

# Flash device firmware (debug build)
flash-debug:
    cd oxifoc-device && cargo flash --chip STM32G431CBTx

# Build device firmware (release)
device:
    cd oxifoc-device && cargo build --release

# Build device firmware (debug)
device-debug:
    cd oxifoc-device && cargo build

# Clean all build artifacts
clean:
    cargo clean
    cd oxifoc-host-tauri && rm -rf dist node_modules

# Full rebuild from scratch
rebuild: clean install build-all
