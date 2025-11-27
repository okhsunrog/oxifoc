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

# Format all code (Rust + TypeScript, excludes ergot)
fmt:
    cargo fmt --all
    cd oxifoc-g431 && cargo fmt
    cd oxifoc-f405 && cargo fmt
    cd oxifoc-host-tauri && bun format

# Lint Rust code (clippy)
clippy:
    cargo clippy --workspace --all-targets -- -D warnings
    cd oxifoc-g431 && cargo clippy --all-targets -- -D warnings
    cd oxifoc-f405 && cargo clippy --all-targets -- -D warnings

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
    cd oxifoc-g431 && cargo run --release

# Flash device firmware (debug build)
flash-debug:
    cd oxifoc-g431 && cargo run

# Build device firmware (release)
device:
    cd oxifoc-g431 && cargo build --release

# Build device firmware (debug)
device-debug:
    cd oxifoc-g431 && cargo build

# Clean all build artifacts
clean:
    git clean -dfx

# Check all Rust code (format, clippy, build, test)
check-rust:
    @echo "Checking Rust formatting..."
    cargo fmt --all -- --check
    cd oxifoc-g431 && cargo fmt -- --check
    cd oxifoc-f405 && cargo fmt -- --check
    @echo "Running clippy..."
    just clippy
    @echo "Checking builds..."
    cargo check --workspace
    cd oxifoc-g431 && cargo check
    cd oxifoc-f405 && cargo check
    @echo "Running tests..."
    cargo test --workspace
    @echo "✓ Rust checks passed!"

# Check all TypeScript code (format, type-check, lint)
check-ts:
    @echo "Checking TypeScript formatting..."
    cd oxifoc-host-tauri && bun format --check
    @echo "Type checking TypeScript..."
    cd oxifoc-host-tauri && bun type-check
    @echo "Linting TypeScript..."
    cd oxifoc-host-tauri && bun lint:check
    @echo "✓ TypeScript checks passed!"

# Run all checks (Rust + TypeScript)
check-all: check-rust check-ts
    @echo "✓ All checks passed!"

# Full rebuild from scratch
rebuild: clean install build-all
