#!/usr/bin/env bash
# ISR timing run on the MK5 bench: flash ELF, read RTT with `probe-rs attach`
# while the host CLI runs MANEUVER, then save the logs under OUT.*
#
# usage: isr_bench.sh ELF MANEUVER OUT
#   ELF       image built with the `isr/s` lines enabled (scripts/isr_placement.py build)
#   MANEUVER  maneuvers/isr-timing-short.json (averages) or isr-timing.json (maxima)
#   OUT       output prefix: OUT.flash.log, OUT.rtt.log, OUT.json (maneuver
#             report), OUT.log (CLI stderr); the capture is OUT.parquet
#
# Environment (or a .env in the current directory):
#   OXIFOC_MK5_PROBE      probe-rs selector of the MK5 probe (recommended)
#   OXIFOC_HOST_CLI       host CLI binary (default: oxifoc-host-cli on PATH,
#                         else target/release/oxifoc-host-cli)
#   OXIFOC_BRIDGE_SERIAL  serial port of the C5 bridge, if several are attached
#   OXIFOC_CLI_TRANSPORT  CLI transport (default usb-serial-jtag, the C5 bridge)
#
# The script needs only probe-rs, the CLI and the two files, so it can run on
# a separate bench host. Summarise the RTT logs with scripts/isr_rtt.py.
set -euo pipefail
if [ $# -ne 3 ]; then
    sed -n '2,19p' "$0" >&2
    exit 2
fi
elf=$1 maneuver=$2 out=$3
# .env fills in only what the environment leaves unset.
if [ -f .env ]; then
    while IFS='=' read -r k v; do
        case $k in OXIFOC_*) if [ -z "${!k:-}" ]; then export "$k=$v"; fi ;; esac
    done < .env
fi
chip=STM32F405RGTx
probe=()
if [ -n "${OXIFOC_MK5_PROBE:-}" ]; then probe=(--probe "$OXIFOC_MK5_PROBE"); fi
cli=${OXIFOC_HOST_CLI:-}
if [ -z "$cli" ]; then
    cli=$(command -v oxifoc-host-cli || echo target/release/oxifoc-host-cli)
fi
cli_args=(--transport "${OXIFOC_CLI_TRANSPORT:-usb-serial-jtag}" --wait-secs 15 --json)
if [ -n "${OXIFOC_BRIDGE_SERIAL:-}" ]; then cli_args+=(--serial-path "$OXIFOC_BRIDGE_SERIAL"); fi
test -f "$elf" || { echo "no ELF $elf" >&2; exit 2; }
test -f "$maneuver" || { echo "no maneuver $maneuver" >&2; exit 2; }
test -x "$cli" || { echo "host CLI $cli not found; set OXIFOC_HOST_CLI" >&2; exit 2; }
mkdir -p "$(dirname "$out")"

# 4 MHz SWD: erase and programming dominate (about 33 s for the MK5 image,
# 37 s at the 1 MHz default; 24 MHz saves one more second). After a failed
# flash, retry at 1 MHz before anything else.
: > "$out.flash.log"
flashed=0
for speed in 4000 1000 1000; do
    if probe-rs download --verify --chip $chip "${probe[@]}" --speed $speed "$elf" \
        >> "$out.flash.log" 2>&1; then flashed=1; break; fi
    echo "flash failed at --speed $speed" >&2
    sleep 3
done
if [ $flashed != 1 ]; then echo "FLASH FAILED: $elf (see $out.flash.log)" >&2; exit 3; fi
probe-rs reset --chip $chip "${probe[@]}" >> "$out.flash.log" 2>&1
sha256sum "$elf" >> "$out.flash.log"
sleep 4

# Attach does not reset the core; it halts it for 50-80 ms, which the parser
# drops as short windows.
probe-rs attach --chip $chip "${probe[@]}" --speed 1000 "$elf" > "$out.rtt.log" 2>&1 &
attach=$!
trap 'kill -INT $attach 2>/dev/null || true; wait $attach 2>/dev/null || true' EXIT
sleep 3
status=0
timeout 180 "$cli" "${cli_args[@]}" maneuver run "$maneuver" --out "$out.parquet" \
    > "$out.json" 2> "$out.log" || status=$?
sleep 2
kill -INT $attach 2>/dev/null || true
wait $attach 2>/dev/null || true
trap - EXIT
echo "$out: $(grep -c 'isr/s' "$out.rtt.log" || true) isr/s lines, maneuver exit $status"
exit $status
