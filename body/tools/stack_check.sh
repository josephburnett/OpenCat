#!/bin/bash
# Static worst-case stack estimate for the body firmware (doc §3h, §6).
#
# Builds body/ with arduino-cli with the same size flags as size_check.sh and
# `kilo flash` (-mcall-prologues, -Wl,--relax) plus -fstack-usage at link
# time: the sketch is built with -flto, so the frames are only final in the
# link-time (ltrans) compile, whose .su files land in our TMPDIR
# (-save-temps=obj keeps them). Neither flag changes the code. Then
# stack_check.py walks the call graph of `avr-objdump -d` from main() and
# from every ISR and prints the deepest path, the ISR add-on and the margin
# 2048 - globals - stack; FAILS if the margin is below MIN_MARGIN.
#
# The estimate is an upper bound on what the stack paint (`stack_free_min`
# in STATS) can see, give or take what the model does not cover (indirect
# calls and recursion are listed when present). Compare: margin here vs
# stack_free_min on the robot.
#
# Usage: body/tools/stack_check.sh [--min-margin N] [build-dir]
# Env:   ARDUINO_CLI, EXTRA_FLAGS (e.g. -DKILO_BUZZER=0), MIN_MARGIN (128)
set -u
BODY="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
MIN_MARGIN="${MIN_MARGIN:-128}"
if [ "${1:-}" = "--min-margin" ]; then MIN_MARGIN="${2:?--min-margin needs a value}"; shift 2; fi
BUILD="${1:-${TMPDIR:-/tmp}/kilo-body-stack}"
CFLAGS="-mcall-prologues"
LDFLAGS="-Wl,--relax"
PY="${PYTHON:-python3}"

mkdir -p "$BUILD"
LTRANS="$BUILD.ltrans"  # outside the build dir: arduino-cli may wipe that
rm -rf "$LTRANS"
mkdir -p "$LTRANS"
args=(compile --fqbn arduino:avr:uno --build-path "$BUILD"
      --build-property "compiler.cpp.extra_flags=$CFLAGS${EXTRA_FLAGS:+ $EXTRA_FLAGS}"
      --build-property "compiler.c.extra_flags=$CFLAGS"
      --build-property "compiler.c.elf.extra_flags=$LDFLAGS -fstack-usage -save-temps=obj")
if ! out=$(TMPDIR="$LTRANS" "$CLI" "${args[@]}" "$BODY" 2>&1); then
  echo "$out"
  echo "stack_check: BUILD FAILED"
  exit 1
fi
ELF="$BUILD/body.ino.elf"
TOOLS=$(ls -d "$HOME"/.arduino15/packages/arduino/tools/avr-gcc/*/bin 2>/dev/null | sort | tail -1)
read -r data bss < <("$TOOLS/avr-size" -A "$ELF" | awk '$1==".data"{d=$2} $1==".bss"{b=$2} $1==".noinit"{n=$2} END{print d, b+n}')
"$TOOLS/avr-objdump" -d -C "$ELF" > "$BUILD/body.dis"
shopt -s nullglob
su=("$LTRANS"/*.su)
if [ "${#su[@]}" -eq 0 ]; then echo "stack_check: no .su files from the link step in $LTRANS"; exit 1; fi
exec "$PY" "$BODY/tools/stack_check.py" --disasm "$BUILD/body.dis" --su "${su[@]}" \
  --globals "$((data + bss))" --min-margin "$MIN_MARGIN"
