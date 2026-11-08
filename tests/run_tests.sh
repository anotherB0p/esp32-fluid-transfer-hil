#!/usr/bin/env bash
# Builds and runs the host unit tests for common/transfer_core (Linux/macOS, any gcc/clang).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
core="$here/../common/transfer_core"
mkdir -p "$here/build"
"${CC:-gcc}" -std=c99 -O1 -g -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion \
    -I "$core/include" \
    "$core/src/proto.c" "$core/src/plant_model.c" "$core/src/controller_fsm.c" "$core/src/blackbox.c" \
    "$here/test_core.c" -lm -o "$here/build/test_core"
"$here/build/test_core"
