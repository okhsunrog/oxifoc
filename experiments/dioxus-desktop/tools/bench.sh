#!/usr/bin/env bash
set -euo pipefail
experiment=$(cd -- "$(dirname -- "$0")/.." && pwd)
task=/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-dioxus
export TMPDIR="$task"
measure="$experiment/tools/measure.py"
output="$experiment/docs/measurements.jsonl"
app_pid=
trap 'if [[ -n "$app_pid" ]]; then kill "$app_pid" 2>/dev/null || true; wait "$app_pid" 2>/dev/null || true; fi' EXIT
run_case() {
  local label=$1 mode=$2 visible=$3
  local logfile="$task/$label.log"
  if [[ "$mode" == desktop ]]; then
    PLOT_BENCH=1 PLOT_VISIBLE="$visible" nix develop "$experiment" -c "$experiment/target/release/oxifoc-desktop-plot-probe" >"$logfile" 2>&1 &
  else
    RELAY_PLOT_ONLY=1 RELAY_WIDTH=802 RELAY_HEIGHT=400 RELAY_VISIBLE="$visible" PLOT_STATS=1 nix develop /home/okhsunrog/code/rust/dioxus_native_test -c /home/okhsunrog/code/rust/dioxus_native_test/target/release/dioxus_native_test >"$logfile" 2>&1 &
  fi
  app_pid=$!
  for attempt in {1..200}; do
    kill -0 "$app_pid" || { cat "$logfile"; return 1; }
    case "$(readlink /proc/"$app_pid"/exe)" in
      *oxifoc-desktop-plot-probe|*dioxus_native_test) break;;
    esac
    sleep .2
  done
  uv run python "$measure" "$label" "$app_pid" "$logfile" --output "$output"
  kill "$app_pid"
  wait "$app_pid" || true
  app_pid=
}
if [[ "${1:-}" == desktop-peaks ]]; then
  run_case desktop-peaks-1 desktop 60000
  run_case desktop-peaks-2 desktop 60000
  exit
fi
run_case desktop-lines-1 desktop 1200
run_case native-lines-1 native 1200
run_case desktop-peaks-1 desktop 60000
run_case native-peaks-1 native 60000
run_case desktop-lines-2 desktop 1200
run_case native-lines-2 native 1200
run_case desktop-peaks-2 desktop 60000
run_case native-peaks-2 native 60000
PLOT_PAUSED=1 run_case desktop-paused desktop 1200
RELAY_PAUSED=1 run_case native-paused native 1200
