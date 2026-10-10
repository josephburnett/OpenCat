#!/bin/bash
# Size and symbol harness for the body firmware (doc §6).
#
# Builds body/ with arduino-cli exactly as `kilo flash` does, prints flash
# and RAM, and FAILS if:
#   - a forbidden symbol is linked (the libraries and features this firmware
#     replaced: HardwareSerial/Print, Wire, I2Cdev, pulseIn, tone, the heap,
#     String, soft float, libm);
#   - flash > FLASH_MAX (default 16384, the P0 ceiling of doc §3h; P1 18432,
#     P2 20480);
#   - globals (.data + .bss + .noinit) > GLOBALS_MAX (default 1540).
#
# Usage: body/tools/size_check.sh [build-dir]       build, then check
#        body/tools/size_check.sh --elf FILE.elf       check an existing build
#                                                      (what `kilo flash` does)
# Env:   ARDUINO_CLI, FLASH_MAX, GLOBALS_MAX, EXTRA_FLAGS (e.g. -DKILO_BUZZER=0)
set -u
BODY="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FLASH_MAX="${FLASH_MAX:-16384}"
GLOBALS_MAX="${GLOBALS_MAX:-1540}"

if [ "${1:-}" = "--elf" ]; then
  ELF="${2:?--elf needs a file}"
  [ -f "$ELF" ] || { echo "size_check: no such ELF: $ELF"; exit 1; }
else
  BUILD="${1:-${TMPDIR:-/tmp}/kilo-body-build}"
  mkdir -p "$BUILD"
  args=(compile --fqbn arduino:avr:uno --build-path "$BUILD")
  if [ -n "${EXTRA_FLAGS:-}" ]; then args+=(--build-property "compiler.cpp.extra_flags=$EXTRA_FLAGS"); fi
  if ! out=$("$CLI" "${args[@]}" "$BODY" 2>&1); then
    echo "$out"
    echo "size_check: BUILD FAILED"
    exit 1
  fi
  ELF="$BUILD/body.ino.elf"
fi
TOOLS=$(ls -d "$HOME"/.arduino15/packages/arduino/tools/avr-gcc/*/bin 2>/dev/null | sort | tail -1)
NM="$TOOLS/avr-nm"
SIZE="$TOOLS/avr-size"
fail=0

# Flash = .text + .data (what avrdude writes); globals = .data + .bss +
# .noinit (the RAM that is not stack).
read -r text data bss < <("$SIZE" -A "$ELF" | awk '$1==".text"{t=$2} $1==".data"{d=$2} $1==".bss"{b=$2} $1==".noinit"{n=$2} END{print t, d, b+n}')
flash=$((text + data))
globals=$((data + bss))
echo "flash:   $flash B (limit $FLASH_MAX; .text $text + .data $data)"
echo "globals: $globals B (limit $GLOBALS_MAX; stack + margin get $((2048 - globals)) B)"
[ "$flash" -le "$FLASH_MAX" ] || { echo "FAIL: flash over budget by $((flash - FLASH_MAX)) B"; fail=1; }
[ "$globals" -le "$GLOBALS_MAX" ] || { echo "FAIL: globals over budget by $((globals - GLOBALS_MAX)) B"; fail=1; }

# Forbidden symbols. Matched against demangled names, whole-word where a
# substring would be ambiguous (e.g. "tone" vs "notes_tick").
FORBIDDEN='HardwareSerial|Serial[0-9]?\b|Print::|TwoWire|\bWire\b|\btwi_|I2Cdev|pulseIn|\btone\b|noTone|\bmalloc\b|\bfree\b|\brealloc\b|\bcalloc\b|\bString\b|String::|IRrecv'
SOFTFLOAT='__(add|sub|mul|div|neg)sf3|__fix(uns)?sf[sd]i|__float(un)?[sd]isf|__(cmp|eq|ne|lt|le|gt|ge|unord)sf2|__fp_|\b(sqrt|atan2?|asin|acos|sin|cos|tan|pow|exp|log|floor|ceil|round|lround|fabs)f?\b'
syms=$("$NM" -C "$ELF" | awk '{ $1=""; $2=""; sub(/^ +/, ""); print }')
hits=$(echo "$syms" | grep -E "$FORBIDDEN|$SOFTFLOAT" || true)
if [ -n "$hits" ]; then
  echo "FAIL: forbidden symbols linked:"
  echo "$hits" | sed 's/^/  /'
  fail=1
else
  echo "forbidden-symbol scan: clean (no Serial/Print/Wire/twi_/I2Cdev/pulseIn/tone/heap/String/soft-float/libm)"
fi

echo "largest symbols:"
"$NM" -S --size-sort -C -t d "$ELF" | tail -12 | awk '{printf "  %6d %s %s\n", $2, $3, substr($0, index($0,$4))}'

if [ "$fail" -eq 0 ]; then echo "size_check: PASS"; else echo "size_check: FAIL"; fi
exit "$fail"
