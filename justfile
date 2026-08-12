# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
device_crates := "oxifoc-g474 oxifoc-f405 oxifoc-bridge oxifoc-remote"

# On-target test crates (run on hardware; compiled here so they don't rot)
target_test_crates := "tests/stm32g474 tests/stm32f405"

# Run all checks (fmt, clippy, tests — workspace + device crates)
check:
    @just check-host
    @just check-device

# Host workspace: fmt + clippy + tests
check-host:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "git-rev sync across lock files..."
    python3 scripts/check-git-rev-sync.py
    echo "rustfmt (workspace)..."
    cargo fmt --check --all
    echo "clippy (workspace)..."
    cargo clippy --workspace --all-targets --quiet -- -D warnings
    echo "tests (workspace)..."
    output=$(cargo test --workspace --quiet 2>&1) || { echo "$output"; exit 1; }
    echo "oxifoc-core without detection (gate must not rot)..."
    # clippy, not check: the embassy-gated modules are compiled ONLY in this
    # slice (no workspace member enables the feature), so this is their one
    # lint gate.
    cargo clippy -p oxifoc-core --quiet --no-default-features \
        --features algorithms,runtime,storage,delivery,defmt,embassy,virtual-motor,std -- -D warnings

# Device firmware: fmt + clippy + build (all targets)
check-device:
    #!/usr/bin/env bash
    set -euo pipefail
    filter() { grep -v 'unstable feature.*vfp2\|not stably supported\|unknown and unstable feature.*fp64\|still passed through to the codegen\|consider filing a feature request\|^  |\|^$' || true; }
    for crate in {{ device_crates }}; do
        echo "$crate: fmt + clippy + build..."
        (cd "$crate" && cargo fmt --check) || exit 1
        (cd "$crate" && cargo clippy --quiet -- -D warnings -W clippy::disallowed-methods 2>&1 | filter) || exit 1
        (cd "$crate" && cargo build --release --quiet 2>&1 | filter) || exit 1
    done
    for crate in {{ target_test_crates }}; do
        echo "$crate: fmt + compile..."
        (cd "$crate" && cargo fmt --check) || exit 1
        (cd "$crate" && cargo build --tests --quiet 2>&1 | filter) || exit 1
    done
    # f405 second board must not rot (default = board-cf2).
    echo "oxifoc-f405 (vesc6-mk5): build..."
    (cd oxifoc-f405 && cargo build --release --quiet --no-default-features --features transport-usb,transport-uart,board-vesc6-mk5 2>&1 | filter) || exit 1
    # LAST: restore the canonical CF2 ELF — the mk5 variant overwrites
    # target/…/oxifoc-f405, and flashing/decoding with a mismatched board
    # ELF fails confusingly.
    echo "oxifoc-f405 (cf2): restore canonical ELF..."
    (cd oxifoc-f405 && cargo build --release --quiet 2>&1 | filter) || exit 1

# Format all code (workspace + device crates)
fmt:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "rustfmt..."
    cargo fmt --all
    for crate in {{ device_crates }} {{ target_test_crates }}; do
        (cd "$crate" && cargo fmt)
    done

# Run workspace tests
test:
    cargo test --workspace
    # HFI is behind features that are off by default; this pass runs the
    # `hfi`/`hfi-detect`-gated tests (g474/f405 config).
    cargo test -p oxifoc-core --features runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect

# Build device firmware (release)
build target="oxifoc-f405":
    cd {{ target }} && cargo build --release

# Flash device firmware (release)
flash target="oxifoc-f405":
    cd {{ target }} && cargo run --release

# Run host CLI with arguments
cli *ARGS:
    cargo run -p oxifoc-host-cli -- {{ ARGS }}

# Run host GUI
gui:
    cargo run -p oxifoc-host-slint

# Run the virtual device as a TCP Router on :2025 (extra args after `virtual`)
virtual *ARGS:
    cargo run -p oxifoc-virtual -- --transport tcp --port 2025 {{ ARGS }}

# End-to-end test: spawns the virtual Router and drives it via host-lib over
# both TCP and UDP (HardwareInfo handshake, at_least_once Motor,
# effectively_once Detect).
e2e:
    cargo test -p oxifoc-virtual --test e2e

# Build oxifoc-f405 for a board: `just f405` (cf2) or `just f405 vesc6-mk5`.
# Both boards share one ELF path (target/…/release/oxifoc-f405) — flash the
# board that matches the last build.
f405 board="cf2":
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f405
    if [ "{{ board }}" = "cf2" ]; then
        cargo build --release
    else
        cargo build --release --no-default-features \
            --features "transport-usb,transport-uart,board-{{ board }}"
    fi

# Build and flash the selected F405 board variant in one command. Keep this
# board-aware: the generic `just flash oxifoc-f405` uses the crate defaults
# and would silently rebuild a CF2 image before flashing an MK5.
f405-flash board="cf2":
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f405
    if [ "{{ board }}" = "cf2" ]; then
        cargo run --release
    else
        cargo run --release --no-default-features \
            --features "transport-usb,transport-uart,board-{{ board }}"
    fi

# Flash usage of the STM32 firmwares (see docs/flash-size.md)
size:
    #!/usr/bin/env bash
    set -euo pipefail
    measure() { # crate label limit_file extra_flags...
        local crate="$1" label="$2" memx="$3"; shift 3
        (cd "$crate" && cargo build --release --quiet "$@" 2>/dev/null) || { echo "$label: build failed"; exit 1; }
        local elf="$crate/target/thumbv7em-none-eabihf/release/$crate"
        local limit_k=$(grep -oP 'FLASH\s*:\s*ORIGIN[^,]*,\s*LENGTH\s*=\s*\K[0-9]+(?=K)' "$crate/$memx")
        local limit=$((limit_k * 1024))
        local used=$(arm-none-eabi-size "$elf" | tail -1 | awk '{print $1+$2}')
        printf "%-24s %7d / %7d bytes (%2d%%), headroom %d\n" \
            "$label" "$used" "$limit" "$((used * 100 / limit))" "$((limit - used))"
    }
    measure oxifoc-g474 oxifoc-g474 memory.x
    measure oxifoc-f405 oxifoc-f405 memory.x

# Clean all build artifacts
clean:
    cargo clean
    for crate in {{ device_crates }} {{ target_test_crates }}; do (cd "$crate" && cargo clean); done
