#!/usr/bin/env bash
set -euo pipefail
experiment=$(cd -- "$(dirname -- "$0")/.." && pwd)
workspace=$(cd -- "$experiment/../.." && pwd)
task=${OXIFOC_DIOXUS_TMP:-${TMPDIR:-/tmp}/oxifoc-dioxus}
mkdir -p "$task"
export TMPDIR="$task"
export OXIFOC_TCP_PORT=${OXIFOC_TCP_PORT:-25266}
nix develop "$workspace" -c "$workspace/target/debug/oxifoc-virtual" --transport tcp --port "$OXIFOC_TCP_PORT" >"$task/desktop-virtual.log" 2>&1 &
virtual_pid=$!
app_pid=
trap 'if [[ -n "$app_pid" ]]; then kill "$app_pid" 2>/dev/null || true; wait "$app_pid" 2>/dev/null || true; fi; kill "$virtual_pid" 2>/dev/null || true; wait "$virtual_pid" 2>/dev/null || true' EXIT
nix develop "$experiment" -c "$experiment/target/release/oxifoc-desktop-plot-probe" >"$task/desktop-live.log" 2>&1 &
app_pid=$!
wait "$app_pid"
app_pid=
