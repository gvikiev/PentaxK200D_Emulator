#!/bin/sh
# Builds and runs the host-based unit tests for the Pentax camera-model
# state machine (main/pentax_cam_model.c) with any C99 compiler -- no
# ESP-IDF or TinyUSB needed, since that file has zero hardware dependency.
#
# Usage: sh host_tests/run.sh   (from the repo root, or from host_tests/)
set -e

CC="${CC:-cc}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
MAIN="$ROOT/main"
OUT="$HERE/.out"
mkdir -p "$OUT"

CFLAGS="-std=c99 -Wall -Wextra -g -O1"

echo "== pentax_cam_model =="
"$CC" $CFLAGS -I"$MAIN" \
    "$HERE/test_pentax_cam_model.c" "$MAIN/pentax_cam_model.c" \
    -o "$OUT/test_pentax_cam_model"
"$OUT/test_pentax_cam_model"

echo
echo "All host test suites passed."
