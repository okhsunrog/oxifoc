# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
device_crates := "oxifoc-g431 oxifoc-g474 oxifoc-f405"

# Run all checks (fmt, clippy, tests — workspace + device crates)
check:
    @just check-host
    @just check-device

# Host workspace: fmt + clippy + tests
check-host:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "rustfmt (workspace)..."
    cargo fmt --check --all
    echo "clippy (workspace)..."
    cargo clippy --workspace --quiet -- -D warnings
    echo "tests (workspace)..."
    output=$(cargo test --workspace --quiet 2>&1) || { echo "$output"; exit 1; }

# Device firmware: fmt + clippy + build (all targets)
check-device:
    #!/usr/bin/env bash
    set -euo pipefail
    filter() { grep -v 'unstable feature.*vfp2\|not stably supported\|unknown and unstable feature.*fp64\|still passed through to the codegen\|consider filing a feature request\|^  |\|^$' || true; }
    for crate in {{ device_crates }}; do
        echo "$crate: fmt + clippy + build..."
        (cd "$crate" && cargo fmt --check) || exit 1
        (cd "$crate" && cargo clippy --quiet -- -D warnings 2>&1 | filter) || exit 1
        (cd "$crate" && cargo build --release --quiet 2>&1 | filter) || exit 1
    done

# Format all code (workspace + device crates)
fmt:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "rustfmt..."
    cargo fmt --all
    for crate in {{ device_crates }}; do
        (cd "$crate" && cargo fmt)
    done

# Run workspace tests
test:
    cargo test --workspace

# Build device firmware (release)
build target="oxifoc-g431":
    cd {{ target }} && cargo build --release

# Flash device firmware (release)
flash target="oxifoc-g431":
    cd {{ target }} && cargo run --release

# Run host CLI with arguments
cli *ARGS:
    cargo run -p oxifoc-host-cli -- {{ ARGS }}

# Run host GUI
gui:
    cargo run -p oxifoc-host-slint

# Clean all build artifacts
clean:
    cargo clean
    for crate in {{ device_crates }}; do (cd "$crate" && cargo clean); done
