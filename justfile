set dotenv-load

# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
bridge_features := "ble,swd-probe"
device_crates := "oxifoc-g474 oxifoc-f405 oxifoc-bridge oxifoc-remote"

# On-target test crates (run on hardware; compiled here so they don't rot)
target_test_crates := "tests/stm32g474 tests/stm32f405"

core_test_features := "runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect"
core_embassy_features := core_test_features + ",embassy"

install-hooks:
    git config --local core.hooksPath .githooks

# Offline Hall estimation from a stopped capture; no motor or observer simulation.
[positional-arguments]
replay-hall *ARGS:
    cargo run --release --locked -p oxifoc-host-lib --example replay_hall -- "$@"

mk5_features := "transport-spi,board-vesc6-mk5"
mk5_bench_features := "transport-usb,board-vesc6-mk5"
mk5_elf := "oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405"
# Bench hardware selectors come from the environment or a local .env file
# (see .env.example). Unset, probe-rs uses the only attached probe and the
# host CLI the only attached C5 USB Serial/JTAG port.
mk5_probe := env("OXIFOC_MK5_PROBE", "")
mk5_probe_arg := if mk5_probe == "" { "" } else { "--probe " + mk5_probe }
c5_probe := env("OXIFOC_C5_PROBE", "")
c5_probe_arg := if c5_probe == "" { "" } else { "--probe " + c5_probe }
bridge_serial := env("OXIFOC_BRIDGE_SERIAL", "")
bridge_serial_arg := if bridge_serial == "" { "" } else { "--serial-path " + bridge_serial }
c5_project := env("OXIFOC_C5_PROJECT", "")

# Finite connector probe; flashing is deliberately a separate action.
build-mk5-bridge-pins:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-pins --bin bridge-pin-probe

build-mk5-bridge-pins-rx:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-pins-rx --bin bridge-pin-probe

build-mk5-bridge-spi:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-spi --bin bridge-spi-probe

bench-mk5-bridge-spi output c5_project=c5_project: build-mk5-bridge-spi
    @test -n {{quote(c5_project)}} || { echo "set OXIFOC_C5_PROJECT or pass c5_project" >&2; exit 1; }
    uv run --no-project python scripts/bench-bridge-spi.py --output {{quote(output)}} --c5-project {{quote(c5_project)}}

build-mk5-spi:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features {{quote(mk5_features)}} --bin oxifoc-f405

flash-mk5-spi: build-mk5-spi
    probe-rs download --verify --chip STM32F405RGTx {{mk5_probe_arg}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx {{mk5_probe_arg}}

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
    echo "CMSIS-DAP TCP framing..."
    mkdir -p target
    rustc --edition 2024 --test oxifoc-bridge/src/probe/framing.rs -o target/probe-framing-tests
    target/probe-framing-tests
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
        args=()
        if [ "$crate" = "oxifoc-bridge" ]; then
            args=(--features {{quote(bridge_features)}})
        fi
        (cd "$crate" && cargo clippy --quiet "${args[@]}" -- -D warnings -W clippy::disallowed-methods 2>&1 | filter) || exit 1
        if [ "$crate" = "oxifoc-f405" ]; then
            (cd "$crate" && cargo build --release --quiet --target-dir target/cf2 --no-default-features --features board-cf2,transport-usb 2>&1 | filter) || exit 1
        else
            (cd "$crate" && cargo build --release --quiet "${args[@]}" 2>&1 | filter) || exit 1
        fi
        if [ "$crate" = "oxifoc-bridge" ]; then
            # BLE is opt-in but is the deployed configuration: check it too.
            (cd "$crate" && cargo clippy --quiet --features ble -- -D warnings -W clippy::disallowed-methods 2>&1 | filter) || exit 1
            (cd "$crate" && cargo build --release --quiet --features ble 2>&1 | filter) || exit 1
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
    # Kept diagnostic images must not rot either: lint them and build the
    # standalone connector probes. Separate target directories keep the
    # production MK5 ELF untouched.
    echo "oxifoc-f405 (vesc6-mk5 diagnostics): clippy + build..."
    for features in {{ mk5_features }},isr-profiling {{ mk5_features }},hall-diagnostics {{ mk5_bench_features }},hall-diagnostics,isr-profiling; do
        (cd oxifoc-f405 && cargo clippy --release --quiet --target-dir target/diagnostics --no-default-features --features "$features" -- -D warnings -W clippy::disallowed-methods 2>&1 | filter) || exit 1
    done
    for probe in bridge-pin-probe:bench-bridge-pins bridge-pin-probe:bench-bridge-pins-rx bridge-spi-probe:bench-bridge-spi bridge-spi-probe:bench-spi-cs-request; do
        (cd oxifoc-f405 && cargo build --release --quiet --target-dir target/diagnostics --no-default-features --features "board-vesc6-mk5,${probe#*:}" --bin "${probe%%:*}" 2>&1 | filter) || exit 1
    done

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

# Flashes only the F405 wrap benchmark: MK5 power held, gate driver disabled.
# Replaces the controller firmware; restore the intended firmware afterwards.
bench-wrap-f405:
    cd tests/stm32f405 && cargo test --release --locked --test foc -- z_bench_wrap_angle --nocapture

# On-target microbenchmarks, exact ELF + JSON results, automatic image restore.
bench-embedded-f405 output recovery filter="z_bench_schedule" repeats="3":
    uv run --no-project python -B scripts/bench_embedded_f405.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --filter {{quote(filter)}} --repeats {{repeats}}

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
    cd oxifoc-f405 && cargo build --release --target-dir target/cf2 --no-default-features --features board-cf2,transport-usb

# Build and flash Cheap FOCer 2 firmware.
flash-f405-cf2:
    cd oxifoc-f405 && cargo run --release --target-dir target/cf2 --no-default-features --features board-cf2,transport-usb

# Build Flipsky Mini V6 MK5 firmware.
build-f405-vesc6-mk5:
    cd oxifoc-f405 && cargo build --release --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# Build and flash Flipsky Mini V6 MK5 firmware.
flash-f405-vesc6-mk5:
    cd oxifoc-f405 && cargo run --release --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# MK5 USB bench image with the UID-gated clock calibration.
build-mk5-bench:
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_bench_features }}

# Build and flash the MK5 bench image; leaves the target running.
flash-mk5-bench: build-mk5-bench
    probe-rs download --verify --chip STM32F405RGTx {{mk5_probe_arg}} {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx {{mk5_probe_arg}}

# USB bench image with hall-diagnostics: finite Hall/crossover records for
# dump-hall-mk5 and a 50-Hz Hall log; profiling=on adds isr-profiling.
build-mk5-hall profiling="off":
    @test {{ quote(profiling) }} = off -o {{ quote(profiling) }} = on
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",hall-diagnostics" + if profiling == "on" { ",isr-profiling" } else { "" }) }}

flash-mk5-hall profiling="off": (build-mk5-hall profiling)
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Detect one parameter without applying motor parameters, recording every cycle.
detect-mk5 step capture *ARGS:
    RUST_LOG=warn,device=info just cli-usb --elf {{ mk5_elf }} detect {{ quote(step) }} \
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

# Production SPI image with isr-profiling: the 1-Hz isr/s line plus per-section
# isrp/isrc/isrd/isre/isrh breakdowns over defmt.
build-mk5-profile:
    cd oxifoc-f405 && DEFMT_LOG=warn,oxifoc_f405::control::foc=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_features + ",isr-profiling") }}

flash-mk5-profile: build-mk5-profile
    probe-rs download --verify --chip STM32F405RGTx {{mk5_probe_arg}} {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx {{mk5_probe_arg}}

# ISR placement (docs/performance.md): build the MK5 timing image and capture
# its link in DIR (DIR/build.elf, DIR/link/) for scripts/isr_placement.py relink.
isr-capture dir:
    uv run --no-project python -B scripts/isr_placement.py build {{quote(dir)}}

# Pinned foc-hot.x residues and the ISR's statics vs SRAM2 of one or more images.
[positional-arguments]
isr-residues +elfs:
    uv run --no-project python -B scripts/isr_placement.py residues "$@"

# Code vs placement: hot-path code differences, then residues that differ.
isr-compare a b:
    uv run --no-project python -B scripts/isr_placement.py diff --hot {{quote(a)}} {{quote(b)}}
    uv run --no-project python -B scripts/isr_placement.py callgraph {{quote(a)}} {{quote(b)}}

# Hardware: energizes the secured MK5 motor, records at 20 kHz, then tests Stop.
test-stop-mk5 capture resistance:
    OXIFOC_HARDWARE_TEST=1 OXIFOC_HARDWARE_CAPTURE={{ quote(capture) }} OXIFOC_HARDWARE_RESISTANCE={{quote(resistance)}} \
        cargo test -p oxifoc-host-cli --locked hardware_stop_cancels_inductance_at_20khz -- --ignored --nocapture

# Addressed cancellation, duplicate/retired requests, native Packed28 through C5.
test-stop-bridge capture resistance:
    OXIFOC_HARDWARE_SERIAL={{quote(bridge_serial)}} just test-stop-mk5 {{quote(capture)}} {{quote(resistance)}}

# After a device reset, verify the saved operation cannot be revived.
test-replay-bridge capture resistance request:
    OXIFOC_HARDWARE_REPLAY={{quote(request)}} just test-stop-bridge {{quote(capture)}} {{quote(resistance)}}

# Build ESP32 bridge firmware.
build-bridge features=bridge_features:
    cd oxifoc-bridge && cargo build --release --features {{quote(features)}}

# Build and flash the C5 bridge without claiming the STM32 debug probe.
# --restore-unwritten keeps the NVS partition (BLE bonds); probe-rs otherwise
# erases it together with the bootloader block.
flash-bridge features=bridge_features: (build-bridge features)
    probe-rs download --verify --restore-unwritten --chip esp32c5 {{c5_probe_arg}} oxifoc-bridge/target/riscv32imac-unknown-none-elf/release/oxifoc-bridge
    probe-rs reset --chip esp32c5 {{c5_probe_arg}}

# Build nRF52840 remote firmware.
build-remote:
    cd oxifoc-remote && cargo build --release

# Build and flash nRF52840 remote firmware.
flash-remote:
    cd oxifoc-remote && cargo run --release

# Use the optimized host CLI for native-rate capture and device commands.
[positional-arguments]
cli *ARGS:
    cargo run --release -p oxifoc-host-cli --locked -- "$@"

# USB CLI with the bench connection options and JSON output.
[positional-arguments]
cli-usb *ARGS:
    @just cli --transport usb --wait-secs 8 --json "$@"

# Export hall-diagnostics evidence after Stop; probe attach may briefly halt, but does not reset.
dump-hall-mk5 output elf=mk5_elf cli="":
    uv run python scripts/bench_hall_edges.py {{ quote(output) }} --elf {{ quote(elf) }} {{ if cli == "" { "" } else { "--cli " + quote(cli) } }}

# Volatile model/source replacement while Stopped, continuously recording 20 kHz.
[positional-arguments]
test-live-model-mk5 capture elf=mk5_elf *ARGS:
    #!/usr/bin/env bash
    set -euo pipefail
    capture=$1
    elf=$2
    shift 2
    cargo run -p oxifoc-host-cli --locked --example live_model_apply -- --elf "$elf" --out "$capture" "$@"

# Short Hall-commutated current-control pilot; the motor must be secured.
test-current-mk5 capture maneuver="maneuvers/mk5-hall-current-pilot.json" elf="":
    #!/usr/bin/env bash
    set -euo pipefail
    cli=(just cli-usb)
    elf={{ quote(elf) }}
    if test -n "$elf"; then
        cli=(just cli --transport usb --elf "$elf" --wait-secs 8 --json)
    fi
    trap '"${cli[@]}" stop >/dev/null' EXIT
    "${cli[@]}" source hall > {{ quote(capture + ".source.json") }}
    RUST_LOG="${RUST_LOG:-warn,device=info}" "${cli[@]}" maneuver run \
        {{ quote(maneuver) }} --out {{ quote(capture) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

# Flash an already-built MK5 image and qualify it with its exact decoder ELF.
# The caller must exclude concurrent builds/hardware access for the whole run.
# Alternate ELFs require their source commit, rather than using this checkout's HEAD.
bench-current-mk5 capture maneuver="maneuvers/mk5-hall-hold-check.json" elf=mk5_elf source_commit="":
    #!/usr/bin/env bash
    set -euo pipefail
    elf={{ quote(elf) }}
    source_commit={{ quote(source_commit) }}
    if test -z "$source_commit"; then
        if test "$elf" != {{ quote(mk5_elf) }}; then
            echo 'Pass the source commit for an alternate ELF.' >&2
            exit 2
        fi
        source_commit=$(git rev-parse HEAD)
    fi
    printf '%s\n' "$source_commit" > {{ quote(capture + ".commit") }}
    sha256sum "$elf" > {{ quote(capture + ".elf.sha256") }}
    probe-rs download --verify --connect-under-reset --chip STM32F405RGTx "$elf" \
        > {{ quote(capture + ".flash.log") }} 2>&1
    probe-rs reset --chip STM32F405RGTx >> {{ quote(capture + ".flash.log") }} 2>&1
    capture_status=0
    just test-current-mk5 {{ quote(capture) }} {{ quote(maneuver) }} "$elf" || capture_status=$?
    just cli-usb status > {{ quote(capture + ".status.json") }}
    exit "$capture_status"

# Voltage protection in millivolts; applies while stopped, RAM by default.
set-voltage-limits-usb minimum_mv maximum_mv storage="ram":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set voltage-limits min_vbus_mv={{ quote(minimum_mv) }} \
        max_vbus_mv={{ quote(maximum_mv) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# All six current/power limits are explicit: A, A, A, A, A, W.
set-current-limits-usb drive trip bus regen brake heat storage="ram":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set current-limits max_iq={{ quote(drive) }} \
        max_phase_current={{ quote(trip) }} bus_in_max={{ quote(bus) }} \
        bus_regen_max={{ quote(regen) }} max_brake={{ quote(brake) }} \
        max_dissipation={{ quote(heat) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# Motor model in SI units; explicit flash selection, RAM by default.
set-motor-usb resistance ld lq flux pole_pairs storage="ram":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set motor-params \
        resistance={{ quote(resistance) }} inductance_d={{ quote(ld) }} \
        inductance_q={{ quote(lq) }} flux_linkage={{ quote(flux) }} \
        pole_pairs={{ quote(pole_pairs) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# Install one complete Hall reference; arrays/optional geometry use JSON syntax.
set-hall-usb angles valid storage="ram" geometry="null":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set hall-calibration angles={{ quote(angles) }} \
        valid={{ quote(valid) }} geometry={{ quote(geometry) }} \
        {{ if storage == "ram" { "--volatile" } else { "" } }}

# USB recording with matching JSON result and host timing log beside the parquet.
record-usb capture seconds="30" rate="20000" logs="warn,device=info":
    RUST_LOG={{ quote(logs) }} just cli-usb record --out {{ quote(capture) }} \
        --seconds {{ quote(seconds) }} --fast-hz {{ quote(rate) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

# Also timestamp nusb frame handling and Linux URB submission/completion.
record-usb-trace capture seconds="10" rate="20000":
    @just record-usb {{ quote(capture) }} {{ quote(seconds) }} {{ quote(rate) }} \
        "warn,device=info,ergot::interface_manager::transports::nusb=trace,nusb::platform::linux_usbfs=debug"

# Linux idle-only qualification. Propagates capture loss as a failing exit status.
test-usb-pause capture pauses_ms="5,15,30" rate="20000": build-cli
    uv run python scripts/bench_usb_pause.py {{ quote(capture) }} \
        --pauses-ms {{ quote(pauses_ms) }} --rate {{ quote(rate) }}

# Measure L without applying it; preserve native HFI samples at the default rate.
detect-l-usb capture resistance rate="20000" logs="warn,device=info":
    RUST_LOG={{ quote(logs) }} just cli-usb detect inductance \
        --resistance {{ quote(resistance) }} --record-hz {{ quote(rate) }} --record {{ quote(capture) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }}

# Run host GUI
gui:
    cargo run -p oxifoc-host-slint

# Run the experimental Dioxus Native frontend (TCP localhost by default)
gui-native:
    cargo run --manifest-path oxifoc-host-dioxus/Cargo.toml --release --locked

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
    measure oxifoc-f405 oxifoc-f405-cf2 memory.x target/cf2 --no-default-features --features board-cf2,transport-usb
    measure oxifoc-f405 oxifoc-f405-vesc6-mk5 memory.x target/vesc6-mk5 --no-default-features \
        --features {{ mk5_features }}

# Clean all build artifacts
clean:
    cargo clean
    for crate in {{ device_crates }} {{ target_test_crates }}; do (cd "$crate" && cargo clean); done

# Finite motor-inhibited test: alternate CS-before-DMA and slave-ready-first.
build-mk5-spi-cs-probe:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-spi-cs-request --bin bridge-spi-probe

bench-mk5-spi-cs-request output: build-mk5-spi-cs-probe
    @test -n {{quote(c5_project)}} || { echo "set OXIFOC_C5_PROJECT or pass c5_project" >&2; exit 1; }
    uv run --no-project python scripts/bench-bridge-spi.py --cs-request --output {{quote(output)}} --c5-project {{quote(c5_project)}}

# No motor excitation: telemetry plus acknowledged Stop commands on one connection.
bench-bridge-handshake port seconds="30" hz="20000":
    @cargo run --release --locked -p oxifoc-host-lib --example bridge_handshake -- {{quote(port)}} {{seconds}} {{hz}}

# PCSR sampling uses the probe; no concurrent RTT attach during this maneuver.
bench-mk5-foc-pc output maneuver="maneuvers/foc-profile-velocity40.json":
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p {{quote(output)}}
    trap 'just cli-bridge stop >/dev/null 2>&1' EXIT
    probe-rs profile --chip STM32F405RGTx {{mk5_probe_arg}} --duration 8 --rtt-channel-mode no-block-skip {{mk5_elf}} flat --line-info --limit 20000 pcsr > {{quote(output + "/pc.txt")}} 2>&1 &
    profiler=$!
    sleep 1
    status=0
    just test-bridge {{quote(output + "/capture.parquet")}} {{quote(maneuver)}} > {{quote(output + "/capture.driver.log")}} 2>&1 || status=$?
    wait "$profiler" || status=$?
    exit "$status"

# Production command route through the C5; never select the ST-Link VCP.
[positional-arguments]
cli-bridge *ARGS:
    @target/release/oxifoc-host-cli --transport usb-serial-jtag {{bridge_serial_arg}} --elf {{mk5_elf}} --wait-secs 12 --json "$@"

# Full motor commissioning, one native-rate Parquet capture per stage.
[positional-arguments]
detect-bridge output motor_class *ARGS:
    @shift 2 && just cli-bridge detect all --motor-class {{quote(motor_class)}} --record {{quote(output)}} "$@"

# Finite maneuver, full native-rate stream, terminal Stop on success or failure.
test-bridge capture maneuver velocity_debug="off" phase_source="configured":
    @just test-link bridge {{quote(capture)}} {{quote(maneuver)}} {{quote(velocity_debug)}} {{quote(phase_source)}}

# USB is a temporary A/B reference; bridge is the production route.
test-link link capture maneuver velocity_debug="off" phase_source="configured":
    #!/usr/bin/env bash
    set -euo pipefail
    test {{quote(link)}} = bridge -o {{quote(link)}} = usb
    cli=(target/release/oxifoc-host-cli --transport usb --elf {{mk5_elf}} --wait-secs 12 --json)
    if test {{quote(link)}} = bridge; then cli=(just cli-bridge); fi
    trap '"${cli[@]}" stop >/dev/null' EXIT
    export RUST_LOG="${RUST_LOG:-warn},device=info"
    if test {{quote(phase_source)}} = configured; then
        "${cli[@]}" status > {{quote(capture + ".source.json")}}
    else
        "${cli[@]}" source {{quote(phase_source)}} > {{quote(capture + ".source.json")}}
    fi
    "${cli[@]}" {{if velocity_debug == "on" { "--velocity-debug" } else { "" }}} maneuver run {{quote(maneuver)}} --out {{quote(capture)}} > {{quote(capture + ".json")}} 2> {{quote(capture + ".log")}}

# Production SPI image with hall-diagnostics, for captures through the C5.
build-mk5-motion:
    cd oxifoc-f405 && DEFMT_LOG=warn cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --bin oxifoc-f405 --features {{quote(mk5_features + ",hall-diagnostics")}}

flash-mk5-motion: build-mk5-motion
    probe-rs download --verify --chip STM32F405RGTx {{mk5_probe_arg}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx {{mk5_probe_arg}}

dump-hall-bridge output elf=mk5_elf:
    uv run --no-project python -B scripts/bench_hall_edges.py {{quote(output)}} --elf {{quote(elf)}} --bridge

bench-mk5-motion output recovery repeats="2" hall_current="0" interp="default" hall_config="stored":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-current {{quote(hall_current)}} {{if interp == "default" { "" } else { "--interp-min-erpm " + quote(interp) }}} {{if hall_config == "stored" { "" } else { "--hall-config " + quote(hall_config) }}}

# Isolate integral action with the exact measured Hall table and default P gain.
bench-mk5-motion-soft output recovery hall_config repeats="2":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --velocity-ki 0.02

# Matched long startup/settling runs; gains and Hall table remain RAM-only.
bench-mk5-motion-gains output recovery hall_config kp="0.01" ki="0.2" repeats="2":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --velocity-kp {{quote(kp)}} --velocity-ki {{quote(ki)}} --cases velocity40-long,velocity-negative40-long,velocity120-to40-long

# Durations shorten at higher speed so the complete Hall edge history fits.
bench-mk5-speed-sweep output recovery hall_config ki="0.2" repeats="2" cases="velocity20-sweep,velocity80-sweep,velocity160-sweep,velocity320-sweep,velocitynegative20-sweep,velocitynegative320-sweep":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --velocity-kp 0.01 --velocity-ki {{quote(ki)}} --cases {{quote(cases)}}

# Isolate low-speed commutation from the independent physical speed estimate.
bench-mk5-low-speed-interp output recovery hall_config interp="0" repeats="2":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --interp-min-erpm {{quote(interp)}} --cases velocity20-sweep,velocity40-long,velocity80-sweep

# Native velocity PI observations and Packed28 in one capture session.
bench-mk5-velocity-debug output recovery hall_config repeats="2" cases="velocity40-long,velocity120-to40-long,velocity40-to-iq045":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --velocity-debug --cases {{quote(cases)}}

bench-mk5-crossover output recovery hall_config repeats="1" phase_source="hall" cases="velocity320-hold-to40" hz="20000":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --phase-source {{quote(phase_source)}} --cases {{quote(cases)}} --capture-hz {{hz}}

test-velocity-debug:
    cargo test --locked -p oxifoc-core --lib --features {{quote(core_embassy_features + ",velocity-debug")}}
    cargo test --locked -p oxifoc-host-lib --lib raw_telemetry
    cargo test --locked -p oxifoc-host-cli --bin oxifoc-host-cli maneuver


# Hold torque after the same spin-up to separate speed PI from commutation ripple.
bench-mk5-torque-isolation output recovery hall_config repeats="2" cases="velocity40-to-iq03,velocity40-to-iq06":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --cases {{quote(cases)}} --speed-ceiling 200

# Compare current PI continuity across a real off interval and a small rearm.
bench-mk5-rearm output recovery hall_config repeats="2":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --cases rearm-current --speed-ceiling 200

# Small heat budget with no battery regeneration, fixed Hall table and PI defaults.
bench-mk5-motion-brake output recovery hall_config repeats="1" cases="velocity40":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --cases {{quote(cases)}} --brake-power-w 0.3 --speed-ceiling 200
