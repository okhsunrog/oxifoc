# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
device_crates := "oxifoc-g474 oxifoc-f405 oxifoc-bridge oxifoc-remote"

# On-target test crates (run on hardware; compiled here so they don't rot)
target_test_crates := "tests/stm32g474 tests/stm32f405"

core_test_features := "runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect,impedance-sweep"
core_embassy_features := core_test_features + ",embassy"

# SRAM read-phase benchmark. probe-rs Session setup/cleanup briefly halt/run.
[positional-arguments]
bench-probe-read *ARGS:
    cargo run --release --locked -p oxifoc-host-lib --example probe_read_bench -- "$@"

# Offline Hall estimation from a stopped capture; no motor or observer simulation.
[positional-arguments]
replay-hall *ARGS:
    cargo run --release --locked -p oxifoc-host-lib --example replay_hall -- "$@"

mk5_features := "transport-spi,board-vesc6-mk5,task-priority,fast-telemetry"
mk5_bench_features := "transport-usb,board-vesc6-mk5,task-priority,bench-calibrated-clock,bench-clock-reference"
mk5_usb_metrics_features := mk5_bench_features + ",bench-usb-metrics"
mk5_elf := "oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405"
mk5_probe := "0483:3754:002300203234511537333934"
mk5_profile_features := mk5_usb_metrics_features + ",bench-trace-excitation,isr-profiling"
mk5_usb_trace_features := mk5_bench_features + ",bench-hall,bench-executor-trace,isr-timing"
mk5_pwm_trace_features := mk5_bench_features + ",bench-hall,bench-pwm-commit"
mk5_netstack_trace_features := mk5_bench_features + ",bench-hall,bench-netstack-trace"

# Finite connector probe; flashing is deliberately a separate action.
build-mk5-bridge-pins:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-pins --bin bridge-pin-probe

build-mk5-bridge-pins-rx:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-pins-rx --bin bridge-pin-probe

build-mk5-bridge-spi:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features board-vesc6-mk5,bench-bridge-spi --bin bridge-spi-probe

bench-mk5-bridge-spi output c5_project="/home/okhsunrog/code/rust/c5-hello-world": build-mk5-bridge-spi
    uv run --no-project python scripts/bench-bridge-spi.py --output {{quote(output)}} --c5-project {{quote(c5_project)}}

bench-usb-serial-jtag port fps="625" bytes="832" seconds="30":
    cargo run --release --locked -p oxifoc-host-lib --features transport-bench --example usb_serial_jtag_bench -- {{quote(port)}} {{fps}} {{bytes}} {{seconds}}

build-mk5-spi:
    cd oxifoc-f405 && cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --features {{quote(mk5_features + ",bench-calibrated-clock")}} --bin oxifoc-f405

flash-mk5-spi: build-mk5-spi
    probe-rs download --verify --chip STM32F405RGTx --probe {{mk5_probe}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx --probe {{mk5_probe}}

# No motor outputs at any experimental clock; normal flash-mk5-spi restores stock.
build-mk5-overclock mhz="168" load="stress" memory="flash":
    #!/usr/bin/env bash
    set -euo pipefail
    test {{quote(load)}} = stress -o {{quote(load)}} = light
    case {{quote(memory)}} in flash) placement="";; ccm) placement=",ccm";; sram) placement=",ram-foc";; sram-ccm) placement=",ram-foc,ccm";; *) exit 2;; esac
    config=$(uv run --no-project python -B scripts/bench_prepare_embassy.py)
    cd oxifoc-f405
    OXIFOC_BENCH_CPU_MHZ={{quote(mhz)}} DEFMT_LOG=warn,oxifoc_f405::control::foc=info cargo -Z lockfile-path --config "$config" build --offline --release --target-dir target/vesc6-mk5 --no-default-features --features {{quote(mk5_features + if load == "light" { ",bench-overclock-light" } else { ",bench-overclock" })}}"$placement" --bin oxifoc-f405

flash-mk5-overclock mhz="168" load="stress" memory="flash": (build-mk5-overclock mhz load memory)
    probe-rs download --verify --connect-under-reset --chip STM32F405RGTx --probe {{mk5_probe}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx --probe {{mk5_probe}}

bench-mk5-overclock output seconds="20": build-mk5-spi
    uv run --no-project python -B scripts/bench_overclock.py {{quote(output)}} --seconds {{seconds}}

# Retry one step using the previously verified recovery image; no stock rebuild.
bench-mk5-overclock-step output mhz recovery seconds="20":
    uv run --no-project python -B scripts/bench_overclock.py {{quote(output)}} --steps {{mhz}} --recovery-elf {{quote(recovery)}} --seconds {{seconds}}

# Virtual current control only: phase outputs stay inhibited in every image.
bench-mk5-overclock-current output recovery seconds="10":
    uv run --no-project python -B scripts/bench_overclock.py {{quote(output)}} --steps 168 204 216 240 --load light --current-maneuver maneuvers/mk5-hall-hold-check.json --recovery-elf {{quote(recovery)}} --seconds {{seconds}}

# Four placements at an identical clock/load, two inhibited current runs each.
bench-mk5-memory output recovery mhz="168":
    #!/usr/bin/env bash
    set -euo pipefail
    for memory in flash ccm sram sram-ccm; do
        uv run --no-project python -B scripts/bench_overclock.py {{quote(output)}}/"$memory" --steps {{mhz}} --load light --memory "$memory" --current-maneuver maneuvers/mk5-hall-hold-check.json --recovery-elf {{quote(recovery)}} --seconds 10
    done

bench-mk5-memory-step output recovery mhz memory:
    uv run --no-project python -B scripts/bench_overclock.py {{quote(output)}} --steps {{mhz}} --load light --memory {{quote(memory)}} --current-maneuver maneuvers/mk5-hall-hold-check.json --recovery-elf {{quote(recovery)}} --seconds 10

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
            (cd "$crate" && cargo build --release --quiet --target-dir target/cf2 --no-default-features --features board-cf2,transport-usb 2>&1 | filter) || exit 1
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

# Flashes only the F405 wrap benchmark: MK5 power held, gate driver disabled.
# Replaces the controller firmware; restore the intended firmware afterwards.
bench-wrap-f405:
    cd tests/stm32f405 && cargo test --release --locked --test foc -- z_bench_wrap_angle --nocapture

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
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_bench_features }}

# Build and flash the MK5 bench image; leaves the target running.
flash-mk5-bench: build-mk5-bench
    probe-rs download --verify --chip STM32F405RGTx --probe {{ mk5_probe }} {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx --probe {{ mk5_probe }}

build-mk5-usb-metrics:
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ mk5_usb_metrics_features }}

flash-mk5-usb-metrics: build-mk5-usb-metrics
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Hall bring-up logging at 50 Hz, correlated with the 20-kHz raw ADC stream.
build-mk5-hall profiling="off" memory="sram" transport="usb":
    @test {{ quote(profiling) }} = off -o {{ quote(profiling) }} = on -o {{ quote(profiling) }} = timing
    @test {{ quote(memory) }} = sram -o {{ quote(memory) }} = ccm
    @test {{ quote(transport) }} = usb -o {{ quote(transport) }} = rtt
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-hall" + (if profiling == "on" { ",isr-profiling" } else if profiling == "timing" { ",isr-timing" } else { "" }) + (if memory == "ccm" { ",ccm" } else { "" }) + (if transport == "rtt" { ",transport-rtt" } else { "" })) }}

flash-mk5-hall profiling="off" memory="sram" transport="usb": (build-mk5-hall profiling memory transport)
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# Finite Stopped-only paired codec CPU experiment; this recipe never flashes.
build-mk5-codec-bench:
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-raw-codec") }}

# Raw sink: timing adds its 1-Hz report; hall-phase adds finite RAM events;
# hall-timing enables both; hall-outliers also records section costs of slow cycles.
# Ergot USB remains the command and defmt path; use build-mk5-hall for Hall text logs.
# compensation=off disables only command correction, retaining hardware gate deadtime.
build-mk5-raw sink="usb" profiling="normal" compensation="on":
    @test {{ quote(sink) }} = usb -o {{ quote(sink) }} = rtt -o {{ quote(sink) }} = ergot
    @test {{ quote(profiling) }} = normal -o {{ quote(profiling) }} = timing -o {{ quote(profiling) }} = hall-phase -o {{ quote(profiling) }} = hall-timing -o {{ quote(profiling) }} = hall-outliers
    @test {{ quote(compensation) }} = on -o {{ quote(compensation) }} = off
    cd oxifoc-f405 && DEFMT_LOG={{ quote(if profiling == "timing" { "warn,oxifoc_f405::control::foc=info" } else if profiling == "hall-timing" { "warn,oxifoc_f405::control::foc=info" } else if profiling == "hall-outliers" { "warn,oxifoc_f405::control::foc=info" } else { "warn" }) }} cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-hall,bench-raw-" + sink + (if profiling == "timing" { ",isr-timing" } else if profiling == "hall-phase" { ",bench-hall-phase" } else if profiling == "hall-timing" { ",bench-hall-phase,isr-timing" } else if profiling == "hall-outliers" { ",bench-hall-phase,isr-timing,bench-cycle-outliers" } else { "" }) + (if compensation == "off" { ",bench-no-deadtime-comp" } else { "" })) }}

# Detect one parameter without applying motor parameters, recording every cycle.
detect-mk5 step capture *ARGS:
    RUST_LOG=warn,device=info,fast_rx=debug just cli-usb --elf {{ mk5_elf }} detect {{ quote(step) }} \
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
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_profile_features + if memory == "ccm" { ",ccm" } else { "" }) }}

flash-mk5-profile memory="sram": (build-mk5-profile memory)
    probe-rs download --verify --chip STM32F405RGTx {{ mk5_elf }}
    probe-rs reset --chip STM32F405RGTx

# USB IRQ/executor spans and ADC boundaries; no nested DWT section profiling.
build-mk5-usb-cpu-trace:
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_usb_trace_features) }}

# Same velocity workload: minimal timing, section outliers, or finite CPU trace.
build-mk5-runtime-profile sink="ergot" detail="timing" observer_log="off" logging="timing":
    @test {{ quote(sink) }} = ergot -o {{ quote(sink) }} = usb -o {{ quote(sink) }} = rtt
    @test {{ quote(detail) }} = timing -o {{ quote(detail) }} = outliers -o {{ quote(detail) }} = trace
    @test {{ quote(observer_log) }} = off -o {{ quote(observer_log) }} = on
    @test {{ quote(logging) }} = timing -o {{ quote(logging) }} = verbose -o {{ quote(logging) }} = quiet
    cd oxifoc-f405 && DEFMT_LOG={{ quote((if logging == "verbose" { "debug" } else if logging == "quiet" { "warn" } else { "warn,oxifoc_f405::control::foc=info" }) + if observer_log == "on" { ",oxifoc_core::foc::phase::manager::update=info" } else { "" }) }} cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-hall,bench-raw-" + sink + ",isr-timing" + (if detail == "outliers" { ",bench-cycle-outliers" } else if detail == "trace" { ",bench-executor-trace" } else { "" }) + if observer_log == "on" { ",bench-observer-logging" } else { "" }) }}

# Exhaustive temperature conversion check before PWM initialization.
build-mk5-thermal-check:
    cd oxifoc-f405 && DEFMT_LOG=warn,oxifoc_f405::control::foc=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_bench_features + ",bench-hall,bench-raw-rtt,isr-timing,bench-ntc-check") }}

# Explicit motor excitation, no fast stream; same secured +40 erad/s CPU control.
test-velocity-no-stream-mk5 elf:
    cargo run --release --locked -p oxifoc-host-lib --example velocity_cpu_control -- --run {{ quote(elf) }}

# Minimal CCR timing probe; no trace work before the PWM writes.
build-mk5-pwm-trace:
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_pwm_trace_features) }}

# Network lock spans and ADC entry phase, without executor/PWM trace hooks.
build-mk5-netstack-trace mask="basepri":
    @test {{ quote(mask) }} = basepri -o {{ quote(mask) }} = primask
    cd oxifoc-f405 && DEFMT_LOG=info cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features \
        --features {{ quote(mk5_netstack_trace_features + if mask == "primask" { ",bench-netstack-primask" } else { "" }) }}

profile-netstack-mk5 capture mask="basepri" maneuver="maneuvers/mk5-hall-current-check.json":
    #!/usr/bin/env bash
    set -euo pipefail
    mask={{ quote(mask) }}
    test "$mask" = basepri -o "$mask" = primask
    symbols={{ quote(capture + ".symbols") }}
    rust-nm -n -S -C {{ mk5_elf }} > "$symbols"
    uv run python scripts/bench_executor_trace.py arm --symbols "$symbols" --on-active --active-delay-ms 1200 --timer-hz 83917920
    capture_status=0
    just test-current-mk5 {{ quote(capture) }} {{ quote(maneuver) }} usb {{ mk5_elf }} || capture_status=$?
    uv run python scripts/bench_executor_trace.py read --symbols "$symbols" --out {{ quote(capture + ".trace.bin") }} \
        --timer-hz 83917920 --schema netstack-v1 --elf {{ mk5_elf }} --capture {{ quote(capture) }}
    uv run python scripts/bench_netstack_analysis.py {{ quote(capture + ".trace.bin") }} --mask {{ quote(mask) }} \
        --trim-to-complete-window --export-spans --out {{ quote(capture + ".trace.json") }}
    exit "$capture_status"

profile-pwm-l-mk5 capture resistance:
    #!/usr/bin/env bash
    set -euo pipefail
    symbols={{ quote(capture + ".symbols") }}
    rust-nm -n -S -C {{ mk5_elf }} > "$symbols"
    uv run python scripts/bench_executor_trace.py arm --symbols "$symbols" --on-active --timer-hz 83917920
    capture_status=0
    RUST_LOG=warn,device=info,fast_rx=debug just cli-usb --elf {{ mk5_elf }} detect inductance \
        --resistance {{ quote(resistance) }} --max-power-w 10 --record-hz 20000 --record {{ quote(capture) }} \
        > {{ quote(capture + ".json") }} 2> {{ quote(capture + ".host.log") }} || capture_status=$?
    uv run python scripts/bench_executor_trace.py read --symbols "$symbols" --out {{ quote(capture + ".trace.bin") }} --timer-hz 83917920
    exit "$capture_status"

# Read the finite trace after Stop; preserve capture failure as an exit status.
profile-usb-cpu-mk5 capture maneuver="maneuvers/mk5-hall-current-check.json":
    #!/usr/bin/env bash
    set -euo pipefail
    symbols={{ quote(capture + ".symbols") }}
    rust-nm -n -S -C {{ mk5_elf }} > "$symbols"
    uv run python scripts/bench_executor_trace.py arm --symbols "$symbols" --on-active --active-delay-ms 1200 --timer-hz 83917920
    capture_status=0
    just test-current-mk5 {{ quote(capture) }} {{ quote(maneuver) }} || capture_status=$?
    uv run python scripts/bench_executor_trace.py read --symbols "$symbols" --out {{ quote(capture + ".trace.bin") }} --timer-hz 83917920
    uv run python scripts/bench_trace_analysis.py {{ quote(capture + ".trace.bin") }} --trim-to-complete-window --export-intervals --out {{ quote(capture + ".trace.json") }}
    exit "$capture_status"

# Arm a finite TIM5 trace before native L detection; export after automatic Stop.
profile-l-mk5 capture symbols resistance="0.031":
    uv run python scripts/bench_executor_trace.py arm --symbols {{ quote(symbols) }} --on-active --timer-hz 83917920
    @just detect-mk5 inductance {{ quote(capture) }} --resistance {{ quote(resistance) }}
    uv run python scripts/bench_executor_trace.py read --symbols {{ quote(symbols) }} --out {{ quote(capture + ".trace.bin") }} --timer-hz 83917920

# Hardware: energizes the secured MK5 motor, records at 20 kHz, then tests Stop.
test-stop-mk5 capture:
    OXIFOC_HARDWARE_TEST=1 OXIFOC_HARDWARE_CAPTURE={{ quote(capture) }} \
        cargo test -p oxifoc-host-cli --locked hardware_stop_cancels_inductance_at_20khz -- --ignored --nocapture

# Build ESP32 bridge firmware.
build-bridge features="":
    cd oxifoc-bridge && cargo build --release --features {{quote(features)}}

# Build and flash the C5 bridge without claiming the STM32 debug probe.
flash-bridge features="": (build-bridge features)
    probe-rs download --verify --chip esp32c5 --probe 303a:1001:3C:DC:75:8A:FF:10 oxifoc-bridge/target/riscv32imac-unknown-none-elf/release/oxifoc-bridge
    probe-rs reset --chip esp32c5 --probe 303a:1001:3C:DC:75:8A:FF:10

# Build ESP32 remote firmware.
build-remote:
    cd oxifoc-remote && cargo build --release

# Build and flash ESP32 remote firmware.
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

# Raw data over a dedicated channel; commands still use the USB Ergot interface.
record-raw-mk5 source encoding output seconds="4" hz="20000" elf=mk5_elf:
    @test {{ quote(source) }} = usb -o {{ quote(source) }} = rtt
    @test {{ quote(encoding) }} = raw34 -o {{ quote(encoding) }} = packed26
    just cli --transport usb --fast-source {{ quote(source + "-raw") }} \
        --fast-encoding {{ quote(encoding) }} --chip STM32F405RGTx --elf {{ quote(elf) }} \
        --wait-secs 8 --json record --out {{ quote(output) }} --seconds {{ quote(seconds) }} --fast-hz {{ quote(hz) }}

# Use the same native-rate maneuver/capture path for both raw transports.
test-raw-current-mk5 source encoding capture maneuver elf=mk5_elf:
    #!/usr/bin/env bash
    set -euo pipefail
    trap 'just cli-usb stop >/dev/null' EXIT
    RUST_LOG="${RUST_LOG:-warn,device=info}" just cli --transport usb \
        --fast-source {{ quote(source + "-raw") }} --fast-encoding {{ quote(encoding) }} \
        --chip STM32F405RGTx --elf {{ quote(elf) }} --wait-secs 8 --json \
        maneuver run {{ quote(maneuver) }} --out {{ quote(capture) }}

# MK5 RTT transport; opening it resets the controller before the handshake.
[positional-arguments]
cli-rtt *ARGS:
    @just cli --transport rtt --chip STM32F405RGTx --elf {{ mk5_elf }} --wait-secs 8 --json "$@"

# Export bench-hall evidence after Stop; probe attach may briefly halt, but does not reset.
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
test-current-mk5 capture maneuver="maneuvers/mk5-hall-current-pilot.json" transport="usb" elf="":
    #!/usr/bin/env bash
    set -euo pipefail
    transport={{ quote(transport) }}
    test "$transport" = usb -o "$transport" = rtt
    cli=(just "cli-$transport")
    elf={{ quote(elf) }}
    if test -n "$elf"; then
        cli=(just cli --transport "$transport" --chip STM32F405RGTx --elf "$elf" --wait-secs 8 --json)
    fi
    trap '"${cli[@]}" stop >/dev/null' EXIT
    "${cli[@]}" source hall > {{ quote(capture + ".source.json") }}
    RUST_LOG="${RUST_LOG:-warn,device=info,fast_rx=debug,oxifoc_host_lib::transport::rtt=info}" "${cli[@]}" maneuver run \
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
    just test-current-mk5 {{ quote(capture) }} {{ quote(maneuver) }} usb "$elf" || capture_status=$?
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
    @just cli-usb config set current-limits max_iq_a={{ quote(drive) }} \
        max_phase_current_a={{ quote(trip) }} bus_in_max_a={{ quote(bus) }} \
        bus_regen_max_a={{ quote(regen) }} max_brake_a={{ quote(brake) }} \
        max_dissipation_w={{ quote(heat) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# Motor model in SI units; explicit flash selection, RAM by default.
set-motor-usb resistance ld lq flux pole_pairs storage="ram":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set motor-params \
        resistance_ohm={{ quote(resistance) }} inductance_d_h={{ quote(ld) }} \
        inductance_q_h={{ quote(lq) }} flux_linkage_wb={{ quote(flux) }} \
        pole_pairs={{ quote(pole_pairs) }} {{ if storage == "ram" { "--volatile" } else { "" } }}

# Install one complete Hall reference; arrays/optional geometry use JSON syntax.
set-hall-usb angles valid storage="ram" geometry="null":
    @test {{ quote(storage) }} = ram -o {{ quote(storage) }} = flash
    @just cli-usb config set hall-calibration angles={{ quote(angles) }} \
        valid={{ quote(valid) }} geometry={{ quote(geometry) }} \
        {{ if storage == "ram" { "--volatile" } else { "" } }}

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
    uv run --no-project python scripts/bench-bridge-spi.py --cs-request --output {{quote(output)}} --c5-project /home/okhsunrog/code/rust/c5-hello-world

# No motor excitation: telemetry plus acknowledged Stop commands on one connection.
bench-bridge-handshake port seconds="30" hz="20000":
    @cargo run --release --locked -p oxifoc-host-lib --example bridge_handshake -- {{quote(port)}} {{seconds}} {{hz}}

# PCSR sampling uses the probe; no concurrent RTT attach during this maneuver.
bench-mk5-foc-pc output maneuver="maneuvers/foc-profile-velocity40.json":
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p {{quote(output)}}
    trap 'just cli-bridge stop >/dev/null 2>&1' EXIT
    probe-rs profile --chip STM32F405RGTx --probe {{mk5_probe}} --duration 8 --rtt-channel-mode no-block-skip {{mk5_elf}} flat --line-info --limit 20000 pcsr > {{quote(output + "/pc.txt")}} 2>&1 &
    profiler=$!
    sleep 1
    status=0
    just test-bridge {{quote(output + "/capture.parquet")}} {{quote(maneuver)}} > {{quote(output + "/capture.driver.log")}} 2>&1 || status=$?
    wait "$profiler" || status=$?
    exit "$status"

bench-mk5-foc output recovery detail="minimal" repeats="3" cases="hold,current,velocity40,velocity120" logging="timing" velocity_debug="off":
    uv run --no-project python -B scripts/bench_foc_profile.py {{quote(output)}} --detail {{quote(detail)}} --repeats {{repeats}} --cases {{quote(cases)}} --logging {{quote(logging)}} --recovery-elf {{quote(recovery)}} {{if velocity_debug == "on" { "--velocity-debug" } else { "" }}}

# Current production transport and model; diagnostics differ only by section marks.
build-mk5-foc-profile detail="minimal" logging="timing":
    @test {{quote(detail)}} = minimal -o {{quote(detail)}} = sections -o {{quote(detail)}} = sections-ram -o {{quote(detail)}} = phase-ram -o {{quote(detail)}} = deadline
    @test {{quote(logging)}} = timing -o {{quote(logging)}} = quiet
    cd oxifoc-f405 && DEFMT_LOG={{quote(if logging == "timing" { "warn,oxifoc_f405::control::foc=info" } else { "warn" })}} cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --bin oxifoc-f405 --features {{quote(mk5_features + ",bench-calibrated-clock,bench-foc-distribution" + if detail == "sections" { ",isr-profiling" } else if detail == "sections-ram" { ",bench-foc-sections-ram,isr-profiling" } else if detail == "phase-ram" { ",bench-foc-phase-ram" } else if detail == "deadline" { ",bench-foc-deadline" } else { "" })}}

flash-mk5-foc-profile detail="minimal" logging="timing": (build-mk5-foc-profile detail logging)
    probe-rs download --verify --chip STM32F405RGTx --probe {{mk5_probe}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx --probe {{mk5_probe}}

# Fixed-format telemetry through a chosen link; legacy remains an explicit baseline.
build-mk5-link-bench link="spi" source="ergot" detail="timing":
    @test {{quote(link)}} = spi -o {{quote(link)}} = usb
    @test {{quote(source)}} = ergot -o {{quote(source)}} = legacy-ergot -o {{quote(source)}} = raw-ergot -o {{quote(source)}} = raw-usb -o {{quote(source)}} = raw-rtt
    @test {{quote(detail)}} = timing -o {{quote(detail)}} = trace -o {{quote(detail)}} = codec
    cd oxifoc-f405 && DEFMT_LOG={{quote(if detail == "timing" { "warn,oxifoc_f405::control::foc=info" } else { "warn" })}} cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --bin oxifoc-f405 --features {{quote("board-vesc6-mk5,task-priority,bench-calibrated-clock,isr-timing,transport-" + link + (if source == "ergot" { ",fast-telemetry" } else if source == "raw-ergot" { ",bench-raw-ergot" } else if source == "raw-usb" { ",bench-raw-usb" } else if source == "raw-rtt" { ",bench-raw-rtt" } else { "" }) + (if detail == "trace" { ",bench-executor-trace" } else if detail == "codec" { ",bench-raw-codec" } else { "" }))}}

flash-mk5-link-bench link="spi" source="ergot" detail="timing": (build-mk5-link-bench link source detail)
    probe-rs download --verify --chip STM32F405RGTx --probe {{mk5_probe}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx --probe {{mk5_probe}}

# Production command route through the C5; never select the ST-Link VCP.
[positional-arguments]
cli-bridge *ARGS:
    @target/release/oxifoc-host-cli --transport usb-serial-jtag --serial-path /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_3C:DC:75:8A:FF:10-if00 --elf {{mk5_elf}} --wait-secs 12 --json "$@"

# Finite maneuver, full native-rate stream, terminal Stop on success or failure.
test-bridge capture maneuver source="ergot-raw" encoding="packed28" velocity_debug="off" phase_source="hall":
    @just test-link bridge {{quote(capture)}} {{quote(maneuver)}} {{quote(source)}} {{quote(encoding)}} {{quote(velocity_debug)}} {{quote(phase_source)}}

# USB is a temporary A/B reference; bridge is the production route.
test-link link capture maneuver source="ergot-raw" encoding="packed28" velocity_debug="off" phase_source="hall":
    #!/usr/bin/env bash
    set -euo pipefail
    test {{quote(link)}} = bridge -o {{quote(link)}} = usb
    cli=(target/release/oxifoc-host-cli --transport usb --elf {{mk5_elf}} --wait-secs 12 --json)
    if test {{quote(link)}} = bridge; then cli=(just cli-bridge); fi
    trap '"${cli[@]}" stop >/dev/null' EXIT
    export RUST_LOG="${RUST_LOG:-warn},device=info"
    "${cli[@]}" source {{quote(phase_source)}} > {{quote(capture + ".source.json")}}
    "${cli[@]}" --fast-source {{quote(source)}} --fast-encoding {{quote(encoding)}} {{if velocity_debug == "on" { "--velocity-debug" } else { "" }}} maneuver run {{quote(maneuver)}} --out {{quote(capture)}} > {{quote(capture + ".json")}} 2> {{quote(capture + ".log")}}

# Matching bench-executor-trace firmware must already be flashed.
profile-link-mk5 link output maneuver="maneuvers/mk5-hall-current-check.json" repeats="3":
    uv run --no-project python -B scripts/bench_link_profile.py {{quote(link)}} {{quote(maneuver)}} {{quote(output)}} --repeats {{repeats}}

# Native telemetry plus finite Hall/velocity evidence, without per-cycle timing marks.
build-mk5-motion crossover="off" spi_priority="off":
    cd oxifoc-f405 && DEFMT_LOG=warn cargo build --release --locked --target-dir target/vesc6-mk5 --no-default-features --bin oxifoc-f405 --features {{quote(mk5_features + ",bench-calibrated-clock,bench-hall-phase" + (if crossover == "on" { ",bench-hall-crossover" } else { "" }) + (if spi_priority == "on" { ",bench-spi-pump-priority" } else { "" }))}}

flash-mk5-motion crossover="off" spi_priority="off": (build-mk5-motion crossover spi_priority)
    probe-rs download --verify --chip STM32F405RGTx --probe {{mk5_probe}} {{mk5_elf}}
    probe-rs reset --chip STM32F405RGTx --probe {{mk5_probe}}

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

bench-mk5-crossover output recovery hall_config repeats="1" phase_source="hall-fallback" cases="velocity320-hold-to40" spi_priority="off" hz="20000":
    uv run --no-project python -B scripts/bench_motion_profile.py {{quote(output)}} --recovery-elf {{quote(recovery)}} --repeats {{repeats}} --hall-config {{quote(hall_config)}} --phase-source {{quote(phase_source)}} --cases {{quote(cases)}} {{if spi_priority == "on" { "--spi-priority" } else { "" }}} --capture-hz {{hz}}

test-velocity-debug:
    cargo test --locked -p oxifoc-core --lib --features {{quote(core_embassy_features + ",fast-telemetry")}}
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
