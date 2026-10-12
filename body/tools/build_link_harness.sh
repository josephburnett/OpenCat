#!/bin/bash
# Build body/test/link_harness.cpp: the real core/ link dispatch on the host,
# driven from stdin (used by mind/tests/test_crosscheck.py), or with a second
# argument another stdin harness over the same sources (core_harness: the
# real core as a virtual body, used by mind/kilo/vbody/core.py).
# Usage: body/tools/build_link_harness.sh OUTPUT_BINARY [link_harness|core_harness]
set -eu
BODY="$(cd "$(dirname "$0")/.." && pwd)"
. "$BODY/tools/common.sh"
OUT="${1:?usage: build_link_harness.sh OUTPUT_BINARY [link_harness|core_harness]}"
MAIN="${2:-link_harness}"
DIR="$(dirname "$OUT")"
mkdir -p "$DIR"
CC="${CC:-gcc}"
CXX="${CXX:-g++}"
objs=()
for f in "$BODY"/src/proto/*.c; do
  o="$DIR/${MAIN}_$(basename "$f" .c).o"
  "$CC" -std=c99 -Wall -Wextra -Werror -O1 -c "$f" -o "$o"
  objs+=("$o")
done
kilo_core_sources "$BODY"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -o "$OUT" "$BODY/test/$MAIN.cpp" \
  "${KILO_CORE_SRCS[@]}" "${objs[@]}"
