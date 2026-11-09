# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
device_crates := "oxifoc-g474 oxifoc-f405 oxifoc-bridge oxifoc-remote"

# On-target test crates (run on hardware; compiled here so they don't rot)
target_test_crates := "tests/stm32g474 tests/stm32f405"

core_test_features := "runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect,impedance-sweep"
core_embassy_features := core_test_features + ",embassy"
mk5_features := "transport-usb,transport-uart,board-vesc6-mk5,task-priority"
mk5_bench_features := mk5_features + ",bench-calibrated-clock,bench-clock-reference"
mk5_usb_metrics_features := mk5_bench_features + ",bench-usb-metrics"
mk5_elf := "oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405"
mk5_profile_features := mk5_usb_metrics_features + ",bench-trace-excitation,isr-profiling"

# Run all checks (fmt, clippy, tests — workspace + device crates)
check:
    @just check-host
    @just check-device

# Host workspace: fmt + clippy + tests
check-host:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "git-rev sync across lock files..."
    uv run --no-project python scripts/check-git-rev-sync.py
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
        if [ "$crate" = "oxifoc-f405" ]; then
            (cd "$crate" && cargo build --release --quiet --target-dir target/cf2 2>&1 | filter) || exit 1
        else
            (cd "$crate" && cargo build --release --quiet 2>&1 | filter) || exit 1
        fi
    done
    for crate in {{ target_test_crates }}; do
        echo "$crate: fmt + compile..."
        (cd "$crate" && cargo fmt --check) || exit 1
        (cd "$crate" && cargo build --tests --quiet 2>&1 | filter) || exit 1
    done
    # The second F405 board must not rot. Keep board artifacts in separate
    # directories so no ELF path silently changes electrical meaning.
    echo "oxifoc-f405 (vesc6-mk5): build..."
    (cd oxifoc-f405 && cargo build --release --quiet --target-dir target/vesc6-mk5 --no-default-features --features {{ mk5_features }} 2>&1 | filter) || exit 1

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
    @just test-core

# Core algorithms, runtime and HFI detection, without the GUI workspace.
[positional-arguments]
test-core *ARGS:
    cargo test -p oxifoc-core --locked --features {{ core_test_features }} "$@"

# Exercise the shared hardware adapters as well as the platform-independent math.
[positional-arguments]
test-core-embassy *ARGS:
    cargo test -p oxifoc-core --locked --lib --features {{ core_embassy_features }} "$@"

clippy-core-embassy:
    cargo clippy -p oxifoc-core --all-targets --locked --features {{ core_embassy_features }} -- -D warnings

# Strict lint checks for the same core feature set.
clippy-core:
    cargo clippy -p oxifoc-core --all-targets --locked --features {{ core_test_features }} -- -D warnings

# Build the CLI before a recording session.
build-cli:
    cargo build -p oxifoc-host-cli --locked

# Build STM32G474 firmware.
build-g474:
    cd oxifoc-g474 && cargo build --release

# Build and flash STM32G474 firmware.
flash-g474:
    cd oxifoc-g474 && cargo run --release

# Build Cheap FOCer 2 firmware (explicit board selection; no F405 default).
build-f405-cf2:
    cd oxifoc-f405 && cargo build --release --target-dir target/cf2

# Build and flash Cheap FOCer 2 firmware.
flash-f405-cf2:
    cd oxifoc-f405 && cargo run --release --target-dir target/cf2

# Build Flipsky Mini V6 MK5 firmware.
build-f405-vesc6-mk5:
    cd oxifoc-f405 && cargo build --release --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# Build and flash Flipsky Mini V6 MK5 firmware.
flash-f405-vesc6-mk5:
    cd oxifoc-f405 && cargo run --release --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# MK5 bench image with the UID-gated clock calibration and priority scheduler.
build-mk5-bench:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_bench_features }}

# Build and flash the MK5 bench image; leaves the target running.
flash-mk5-bench: build-mk5-bench
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

build-mk5-usb-metrics:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_usb_metrics_features }}

flash-mk5-usb-metrics: build-mk5-usb-metrics
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Hall bring-up logging at 50 Hz, correlated with the 20-kHz raw ADC stream.
build-mk5-hall profiling="off" memory="sram":
    @test {{ quote(profiling) }} = off -o {{ quote(profiling) }} = on
    @test {{ quote(memory) }} = sram -o {{ quote(memory) }} = ccm
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-hall" + (if profiling == "on" { ",isr-profiling" } else { "" }) + (if memory == "ccm" { ",ccm" } else { "" })) }}

flash-mk5-hall profiling="off" memory="sram": (build-mk5-hall profiling memory)
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Detect one parameter without applying motor parameters, recording every cycle.
detect-mk5 step capture *ARGS:
    RUST_LOG=warn,device=info,fast_rx=debug just cli-usb detect {{ quote(step) }} \
        --record-hz 20000 --record {{ quote(capture) }} {{ ARGS }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

detect-flux-mk5 capture resistance inductance pole_pairs erpm="6000":
    @just detect-mk5 flux {{ quote(capture) }} --resistance {{ quote(resistance) }} \
        --inductance {{ quote(inductance) }} --pole-pairs {{ quote(pole_pairs) }} \
        --erpm {{ quote(erpm) }}

# Energizes the secured MK5 motor: three speeds and a final repeat, all at 20 kHz.
test-flux-mk5 directory resistance inductance pole_pairs:
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p {{ quote(directory) }}
    trap 'just cli-usb stop >/dev/null' EXIT
    run=0
    for erpm in 4000 5000 6000 6000; do
        run=$((run + 1))
        capture={{ quote(directory) }}/flux-${run}-${erpm}.parquet
        just detect-flux-mk5 "$capture" {{ quote(resistance) }} {{ quote(inductance) }} {{ quote(pole_pairs) }} "$erpm"
        rg -q 'Spin-down flux linkage measurement complete' "$capture.host.log"
        # A fresh mechanical start for each ramp; telemetry stays with its run.
        sleep 2
    done

# Observe current bench firmware without reset or programming.
monitor-mk5-bench:
    probe-rs attach --chip STM32F405RGTx {{ mk5_elf }}

# Non-halting PC sampling. Run a separate current maneuver during this window.
profile-mk5 output seconds="8":
    probe-rs profile --chip STM32F405RGTx --duration {{ quote(seconds) }} \
        --rtt-channel-mode no-block-skip {{ mk5_elf }} flat --line-info --limit 20000 pcsr \
        > {{ quote(output) }} 2>&1

# Same trace instrumentation for both placements; independent of motor PWM.
build-mk5-profile memory="sram":
    @test {{ quote(memory) }} = sram -o {{ quote(memory) }} = ccm
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_profile_features + if memory == "ccm" { ",ccm" } else { "" }) }}

flash-mk5-profile memory="sram": (build-mk5-profile memory)
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Arm a finite TIM5 trace before native L detection; export after automatic Stop.
profile-l-mk5 capture symbols:
    uv run python scripts/bench_executor_trace.py arm --symbols {{ quote(symbols) }} --on-active --timer-hz 83917920
    @just detect-l-usb {{ quote(capture) }} 0.031
    uv run python scripts/bench_executor_trace.py read --symbols {{ quote(symbols) }} --out {{ quote(capture + ".trace.bin") }} --timer-hz 83917920

# Hardware: energizes the secured MK5 motor, records at 20 kHz, then tests Stop.
test-stop-mk5 capture:
    OXIFOC_HARDWARE_TEST=1 OXIFOC_HARDWARE_CAPTURE={{ quote(capture) }} \
        cargo test -p oxifoc-host-cli --locked hardware_stop_cancels_inductance_at_20khz -- --ignored --nocapture

# Build ESP32 bridge firmware.
build-bridge:
    cd oxifoc-bridge && cargo build --release

# Build and flash ESP32 bridge firmware.
flash-bridge:
    cd oxifoc-bridge && cargo run --release

# Build ESP32 remote firmware.
build-remote:
    cd oxifoc-remote && cargo build --release

# Build and flash ESP32 remote firmware.
flash-remote:
    cd oxifoc-remote && cargo run --release

# Run host CLI with arguments
[positional-arguments]
cli *ARGS:
    cargo run -p oxifoc-host-cli --locked -- "$@"

# USB CLI with the bench connection options and JSON output.
[positional-arguments]
cli-usb *ARGS:
    @just cli --transport usb --wait-secs 8 --json "$@"

# Short Hall-commutated current-control pilot; the motor must be secured.
test-current-mk5 capture maneuver="maneuvers/mk5-hall-current-pilot.json":
    #!/usr/bin/env bash
    set -euo pipefail
    trap 'just cli-usb stop >/dev/null' EXIT
    just cli-usb source hall > {{ quote(capture + ".source.json") }}
    RUST_LOG=warn,device=info,fast_rx=debug just cli-usb maneuver run \
        {{ quote(maneuver) }} --out {{ quote(capture) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

# Motor model in SI units; explicit flash selection, RAM by default.
set-motor-usb resistance ld lq flux pole_pairs storage="ram":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set motor-params \
        resistance_ohm={{ quote(resistance) }} inductance_d_h={{ quote(ld) }} \
        inductance_q_h={{ quote(lq) }} flux_linkage_wb={{ quote(flux) }} \
        pole_pairs={{ quote(pole_pairs) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# USB recording with matching JSON result and host timing log beside the parquet.
record-usb capture seconds="30" rate="20000" logs="warn,device=info,fast_rx=debug":
    RUST_LOG={{ quote(logs) }} just cli-usb record --out {{ quote(capture) }} \
        --seconds {{ quote(seconds) }} --fast-hz {{ quote(rate) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

# Also timestamp nusb frame handling and Linux URB submission/completion.
record-usb-trace capture seconds="10" rate="20000":
    @just record-usb {{ quote(capture) }} {{ quote(seconds) }} {{ quote(rate) }} \
        "warn,device=info,fast_rx=debug,ergot::interface_manager::transports::nusb=trace,nusb::platform::linux_usbfs=debug"

# Linux idle-only qualification. Propagates capture loss as a failing exit status.
test-usb-pause capture pauses_ms="5,15,30" rate="20000": build-cli
    uv run python scripts/bench_usb_pause.py {{ quote(capture) }} \
        --pauses-ms {{ quote(pauses_ms) }} --rate {{ quote(rate) }}

# Measure L without applying it; preserve native HFI samples at the default rate.
detect-l-usb capture resistance rate="20000" logs="warn,device=info,fast_rx=debug":
    RUST_LOG={{ quote(logs) }} just cli-usb detect inductance \
        --resistance {{ quote(resistance) }} --record-hz {{ quote(rate) }} --record {{ quote(capture) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

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

# Flash usage of the STM32 firmwares (see docs/flash-size.md)
size:
    #!/usr/bin/env bash
    set -euo pipefail
    measure() { # crate label limit_file target_dir extra_flags...
        local crate="$1" label="$2" memx="$3" target_dir="$4"; shift 4
        (cd "$crate" && cargo build --release --quiet --target-dir "$target_dir" "$@" 2>/dev/null) || { echo "$label: build failed"; exit 1; }
        local elf="$crate/$target_dir/thumbv7em-none-eabihf/release/$crate"
        local limit_k=$(grep -oP 'FLASH\s*:\s*ORIGIN[^,]*,\s*LENGTH\s*=\s*\K[0-9]+(?=K)' "$crate/$memx")
        local limit=$((limit_k * 1024))
        local used=$(arm-none-eabi-size "$elf" | tail -1 | awk '{print $1+$2}')
        printf "%-24s %7d / %7d bytes (%2d%%), headroom %d\n" \
            "$label" "$used" "$limit" "$((used * 100 / limit))" "$((limit - used))"
    }
    measure oxifoc-g474 oxifoc-g474 memory.x target
    measure oxifoc-f405 oxifoc-f405-cf2 memory.x target/cf2
    measure oxifoc-f405 oxifoc-f405-vesc6-mk5 memory.x target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# Clean all build artifacts
clean:
    cargo clean
    for crate in {{ device_crates }} {{ target_test_crates }}; do (cd "$crate" && cargo clean); done
