#!/bin/bash
# Host unit tests for body/src/core (doc §6): g++ with sanitizers, plus the
# bare-type lint. Usage: body/tools/host_tests.sh   (exit status = failures)
set -u
BODY="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$(cd "$BODY/.." && pwd)"
. "$BODY/tools/common.sh"
OUT="${TMPDIR:-/tmp}/kilo-host-tests"
mkdir -p "$OUT"
fail=0

# 1. Lint: core/ uses fixed-width types only (on AVR int is 16 bits, on the
#    host 32; a bare int/unsigned/long hides that). Comments are stripped
#    first so prose may say "int".
echo "== lint: bare int/unsigned/long in src/core"
bad=$(for f in "$BODY"/src/core/*.h "$BODY"/src/core/*.cpp; do
  sed -e 's://.*$::' "$f" | grep -nE '\b(int|unsigned|long)\b' | sed "s|^|$(basename "$f"):|"
done)
if [ -n "$bad" ]; then echo "$bad"; echo "lint FAILED"; fail=$((fail + 1)); else echo "lint ok"; fi

# 2. Lint: core/ includes no Arduino or AVR headers (pgm.h and crc16.cpp
#    may use avr-libc under #ifdef __AVR__) and uses no float/double.
echo "== lint: core is hardware-free and float-free"
bad=$(grep -nE '#include *[<"](Arduino|Wire|HardwareSerial|EEPROM|SPI)' "$BODY"/src/core/* ;
      for f in "$BODY"/src/core/* "$BODY"/src/hal/* "$BODY"/src/app/*; do
        sed -e 's://.*$::' "$f" | grep -nE '\b(float|double)\b' | sed "s|^|$(basename "$f"):|"; done)
if [ -n "$bad" ]; then echo "$bad"; echo "lint FAILED"; fail=$((fail + 1)); else echo "lint ok"; fi

# 3. Unit tests: every test links all of core/ (C++17 here; the firmware
#    itself is built as gnu++11 by arduino-cli). test/fake_avr/ stands in for
#    the avr-libc headers, so a test can compile a hal/ file against register
#    fakes (test_i2c.cpp).
CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined
       -ffp-contract=off -DREPO_ROOT="\"$REPO\"")
# test/fake_avr (the TWI register fake test_i2c.cpp uses) only for the core
# tests: test/hal ships its own avr/ shims and must not see this one.
CORE_FLAGS=("${FLAGS[@]}" -I"$BODY/test/fake_avr")
# The vendored protocol C files (src/proto, kept identical to protocol/ by
# protocol/vendor.py) are C99: compile them with the C compiler.
CC="${CC:-gcc}"
PROTO=()
for f in "$BODY"/src/proto/*.c; do
  o="$OUT/proto_$(basename "$f" .c).o"
  "$CC" -std=c99 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -c "$f" -o "$o" || fail=$((fail + 1))
  PROTO+=("$o")
done
LDLIBS=($(kilo_sanitizer_ldlibs "$CXX" "$OUT"))  # -latomic on armhf (common.sh)
kilo_core_sources "$BODY"
CORE=("${KILO_CORE_SRCS[@]}" "${PROTO[@]}")
for t in "$BODY"/test/test_*.cpp; do
  name=$(basename "$t" .cpp)
  echo "== $name"
  if ! "$CXX" "${CORE_FLAGS[@]}" -o "$OUT/$name" "$t" "${CORE[@]}" "${LDLIBS[@]}"; then
    echo "$name: BUILD FAILED"; fail=$((fail + 1)); continue
  fi
  if ! "$OUT/$name"; then fail=$((fail + 1)); fi
done

# 3b. HAL tests (test/hal): the real hal/ sources on fake AVR registers
#     (test/hal/fake_avr.h and its avr/, util/ shims). Each test #includes
#     the hal/ and core/ .cpp files it exercises, or lists sources to link
#     on a "// LINK:" line (globs relative to body/).
for t in "$BODY"/test/hal/test_*.cpp; do
  name=hal_$(basename "$t" .cpp)
  echo "== $name"
  srcs=()
  for f in $(sed -n 's|^// LINK: ||p' "$t"); do srcs+=("$BODY"/$f); done
  if ! "$CXX" "${FLAGS[@]}" -I"$BODY/test/hal" -o "$OUT/$name" "$t" "${srcs[@]}" "$BODY/test/hal/fake_avr.cpp" "${PROTO[@]}" "${LDLIBS[@]}"; then
    echo "$name: BUILD FAILED"; fail=$((fail + 1)); continue
  fi
  if ! "$OUT/$name"; then fail=$((fail + 1)); fi
done

# 4. The stdin-driven dispatch harness used by mind/tests/test_crosscheck.py
#    must keep building.
echo "== link_harness (build only; run by mind/tests/test_crosscheck.py)"
if ! "$BODY/tools/build_link_harness.sh" "$OUT/link_harness"; then
  echo "link_harness: BUILD FAILED"; fail=$((fail + 1))
fi
echo "== core_harness (build only; run by mind/tests/test_corebody.py)"
if ! "$BODY/tools/build_link_harness.sh" "$OUT/core_harness" core_harness; then
  echo "core_harness: BUILD FAILED"; fail=$((fail + 1))
fi

if [ "$fail" -eq 0 ]; then echo "host tests: ALL PASSED"; else echo "host tests: $fail FAILED"; fi
exit "$fail"
