#!/usr/bin/env bash
set -eu
compiler=$1
shift
args=("$@")
crate=''
while (( $# )); do
    if [[ $1 == --crate-name ]]; then
        crate=$2
        break
    fi
    shift
done
if [[ $crate == "$SPI_DIAGNOSTIC_CRATE" ]]; then
    exec "$compiler" "${args[@]}" "$SPI_DIAGNOSTIC_FLAG"
else
    exec "$compiler" "${args[@]}"
fi
