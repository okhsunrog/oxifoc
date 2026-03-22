#!/usr/bin/env bash
# Check that release firmware binaries don't contain core::fmt float formatting.
# These routines add ~15 KB of flash and indicate that something is using
# Debug/Display on f32 values instead of defmt.
#
# Usage: ./scripts/check_bloat.sh
# Runs from a device crate directory (e.g. oxifoc-g431/).
# Requires: cargo-binutils (cargo install cargo-binutils)

set -euo pipefail

CRATE_NAME=$(grep '^name' Cargo.toml 2>/dev/null | head -1 | sed 's/.*"\(.*\)"/\1/')

SYMBOLS=$(CARGO_TERM_COLOR=never cargo nm --release -- -C 2>/dev/null)

EXIT_CODE=0

for pattern in "core::fmt::float" "core::num::flt2dec"; do
    matches=$(echo "$SYMBOLS" | grep -c "$pattern" || true)
    if [ "$matches" -gt 0 ]; then
        echo "FAIL: found $matches symbols matching '$pattern' in $CRATE_NAME"
        echo "$SYMBOLS" | grep "$pattern"
        EXIT_CODE=1
    fi
done

if [ "$EXIT_CODE" -eq 0 ]; then
    echo "OK: no core::fmt float formatting found in $CRATE_NAME"
fi

exit $EXIT_CODE
