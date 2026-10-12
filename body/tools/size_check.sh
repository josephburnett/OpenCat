#!/bin/bash
# Size and symbol harness for the body firmware (doc §6).
#
# Builds body/ with arduino-cli exactly as `kilo flash` does (same size flags:
# -mcall-prologues for C and C++, -Wl,--relax for the link; doc §3h), prints
# flash and RAM, and FAILS if:
#   - a forbidden symbol is linked (the libraries and features this firmware
#     replaced: HardwareSerial/Print, Wire, I2Cdev, pulseIn, tone, the heap,
#     String, soft float, libm);
#   - flash > FLASH_MAX (default 30720, the P2 ceiling of doc §3h, set by the
#     owner 2026-10-11 after docs/firmware-size-report.md: the hardware allows
#     32256 (32 KB - 512 B optiboot); P0 was 20480, P1 24576);
#   - globals (.data + .bss + .noinit) > GLOBALS_MAX (default 1540);
#   - the ELF was not built with the size flags: no __prologue_saves__ (no
#     -mcall-prologues), or a call/jmp whose target an rcall/rjmp reaches
#     (no --relax);
#   - (build mode only, unless STACK_CHECK=0) body/tools/stack_check.sh, the
#     static worst-case stack estimate, fails: margin 2048 - globals - stack
#     below MIN_MARGIN (default 128 B). It builds once more with
#     -fstack-usage at link time (a few seconds with arduino-cli's cache).
#
# Usage: body/tools/size_check.sh [build-dir]       build, then check
#        body/tools/size_check.sh --elf FILE.elf       check an existing build
#                                                      (what `kilo flash` does)
# Env:   ARDUINO_CLI, FLASH_MAX, GLOBALS_MAX, EXTRA_FLAGS (e.g. -DKILO_BUZZER=0),
#        STACK_CHECK (1; 0 skips the stack estimate), MIN_MARGIN (128)
set -u
BODY="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FLASH_MAX="${FLASH_MAX:-30720}"
CFLAGS="-mcall-prologues"  # the same as mind/kilo/tools/flash.py
LDFLAGS="-Wl,--relax"
GLOBALS_MAX="${GLOBALS_MAX:-1540}"

if [ "${1:-}" = "--elf" ]; then
  ELF="${2:?--elf needs a file}"
  [ -f "$ELF" ] || { echo "size_check: no such ELF: $ELF"; exit 1; }
else
  BUILD="${1:-${TMPDIR:-/tmp}/kilo-body-build}"
  mkdir -p "$BUILD"
  args=(compile --fqbn arduino:avr:uno --build-path "$BUILD"
        --build-property "compiler.cpp.extra_flags=$CFLAGS${EXTRA_FLAGS:+ $EXTRA_FLAGS}"
        --build-property "compiler.c.extra_flags=$CFLAGS"
        --build-property "compiler.c.elf.extra_flags=$LDFLAGS")
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
OBJDUMP="$TOOLS/avr-objdump"
fail=0
echo "expected build flags: compiler.cpp/c.extra_flags=$CFLAGS compiler.c.elf.extra_flags=$LDFLAGS"

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

# Build flags. -mcall-prologues links libgcc's shared register save/restore
# code; --relax leaves no call/jmp within rcall/rjmp reach (+-4 KB).
if "$NM" "$ELF" | grep -q '__prologue_saves__'; then
  echo "flags: $CFLAGS ok (__prologue_saves__ linked)"
else
  echo "FAIL: built without $CFLAGS (no __prologue_saves__)"; fail=1
fi
near=$("$OBJDUMP" -d "$ELF" | awk '
  function hex(s,   i, v, c) { v = 0; s = tolower(s); sub(/^0x/, "", s)
    for (i = 1; i <= length(s); i++) { c = index("0123456789abcdef", substr(s, i, 1)); if (!c) break; v = v * 16 + c - 1 }
    return v }
  /^[0-9a-f]+ <.*>:$/ { fn = $2 }
  # The vector table is fixed 4-byte jmps; the linker never relaxes it.
  fn == "<__vectors>:" { next }
  /\t(call|jmp)\t0x/ { a = $1; sub(/:$/, "", a); n = split($0, f, "\t")
    for (i = 1; i < n; i++) if (f[i] == "call" || f[i] == "jmp") t = f[i + 1]
    d = hex(t) - (hex(a) + 2); if (d >= -4096 && d <= 4094) near++ }
  END { print near + 0 }')
if [ "$near" -eq 0 ]; then
  echo "flags: $LDFLAGS ok (no call/jmp within rcall/rjmp reach)"
else
  echo "FAIL: built without $LDFLAGS ($near call/jmp an rcall/rjmp would reach)"; fail=1
fi

echo "largest symbols:"
"$NM" -S --size-sort -C -t d "$ELF" | tail -12 | awk '{printf "  %6d %s %s\n", $2, $3, substr($0, index($0,$4))}'

if [ -n "${BUILD:-}" ] && [ "${STACK_CHECK:-1}" != 0 ]; then
  echo "stack estimate (body/tools/stack_check.sh):"
  sout=$("$BODY/tools/stack_check.sh" "$BUILD-stack" 2>&1); st=$?
  echo "$sout" | sed 's/^/  /'
  [ "$st" -eq 0 ] || fail=1
fi

if [ "$fail" -eq 0 ]; then echo "size_check: PASS"; else echo "size_check: FAIL"; fi
exit "$fail"
