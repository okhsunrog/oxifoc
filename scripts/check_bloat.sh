#!/usr/bin/env bash
# Size guards for a release firmware binary:
#
#  1. No core::fmt float formatting (~15 KB): something formats f32 with
#     Debug/Display instead of defmt.
#  2. No f64 software floating point, and no libm function that needs it
#     (rem_pio2f: the f64 argument reduction of sinf/cosf/tanf). The
#     Cortex-M4F FPU is single precision, so every f64 op is a library call;
#     sinf/cosf alone pull ~12 KB. Use the SinCos backends or fast_math.
#  3. Only allowlisted pure-f32 libm functions (LIBM_ALLOWED below, with
#     the reason for each).
#
# Usage: ./scripts/check_bloat.sh
# Runs from a device crate directory (e.g. oxifoc-f405/).
# Requires: cargo-binutils (cargo install cargo-binutils)

set -euo pipefail

CRATE_NAME=$(grep '^name' Cargo.toml 2>/dev/null | head -1 | sed 's/.*"\(.*\)"/\1/')

SYMBOLS=$(CARGO_TERM_COLOR=never cargo nm --release -- -C 2>/dev/null)

# Pure-f32 libm allowed outside the control ISR, kept for exactness:
#   atanf, atan2f  Hall calibration centroids (stored, reused for commutation)
#   log1pf         exact ZOH inductance fit, L = -R*dt / ln(1 - R*b)
LIBM_ALLOWED="atanf atan2f log1pf"

EXIT_CODE=0

fail_on() {
    local what=$1 pattern=$2 matches
    matches=$(echo "$SYMBOLS" | grep -E "$pattern" || true)
    if [ -n "$matches" ]; then
        echo "FAIL: $what in $CRATE_NAME"
        echo "$matches"
        EXIT_CODE=1
    fi
}

fail_on "core::fmt float formatting" 'core::fmt::float|core::num::flt2dec'
fail_on "f64 software floating point" \
    ' (__aeabi_d[a-z0-9]+|__aeabi_[fil]2d|__aeabi_ul2d|__(add|sub|mul|div)df3|__(extendsf|truncdf)[a-z0-9]+|__fix(uns)?df[a-z]+|__float(un)?[sd]idf|__(eq|ne|lt|le|gt|ge|un)df2)$'
fail_on "libm f64 argument reduction (sinf/cosf/tanf)" 'libm::math::rem_pio2f?::'

for fn in $(echo "$SYMBOLS" | grep -oE 'libm::math::[a-z0-9_]+::' | sed 's/libm::math:://; s/:://' | sort -u); do
    case " $LIBM_ALLOWED " in
        *" $fn "*) ;;
        *)
            echo "FAIL: libm::$fn in $CRATE_NAME is not in LIBM_ALLOWED (scripts/check_bloat.sh)"
            EXIT_CODE=1
            ;;
    esac
done

if [ "$EXIT_CODE" -eq 0 ]; then
    echo "OK: no float formatting, no f64 soft-float, libm within the allowlist in $CRATE_NAME"
fi

exit $EXIT_CODE
