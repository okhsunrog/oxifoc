#!/usr/bin/env bash
set -euo pipefail

# Forward all args from Cargo (path to ELF and any filters)
# Run probe-rs and strip the leading "[defmt] " channel tag from each line.
# Preserve ANSI colors by running probe-rs in a pseudo-TTY using `script` if available.

CMD=(probe-rs run --chip STM32G431CBTx "$@")

if command -v script >/dev/null 2>&1; then
  # -q: quiet; -f: flush output as it is written
  script -q -f -c "${CMD[*]}" /dev/null \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
else
  # Fallback: force colors in some CLIs using CLICOLOR_FORCE
  CLICOLOR_FORCE=1 "${CMD[@]}" \
    | awk '{ sub(/^\[defmt\][[:space:]]*/, ""); print; fflush(); }'
fi
