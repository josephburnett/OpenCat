#!/usr/bin/env bash
# Run every protocol check: generator staleness, vendored copies, the docs'
# copies of the shared defaults, C tests (ASan/UBSan), Python tests, the
# C<->Python cross round trip, and (if installed) an avr-gcc build.
#
#   protocol/test.sh            full run (1e6 fuzz iterations)
#   FUZZ=10000 protocol/test.sh quicker
#   PYTHON=/path/to/python3     interpreter with pytest installed
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$HERE/build"
PYTHON="${PYTHON:-python3}"
FUZZ="${FUZZ:-1000000}"
CC="${CC:-gcc}"
CXX="${CXX:-g++}"
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all"
WARN="-Wall -Wextra -Werror"
mkdir -p "$BUILD"

# -latomic for the sanitizer link on armhf (the probe is shared with the body's host tests).
. "$HERE/../body/tools/common.sh"
LDLIBS="$(kilo_sanitizer_ldlibs "$CXX" "$BUILD")"

echo "== generator up to date"
"$PYTHON" "$HERE/gen.py" --check

echo "== vendored copies (body/src/proto, mind/kilo/protocol) up to date"
"$PYTHON" "$HERE/vendor.py" --check

echo "== docs' tables agree with the spec's shared defaults (doc_check.py)"
"$PYTHON" "$HERE/doc_check.py"

echo "== C build (host, sanitizers)"
for f in cobs crc16 kilo_frame; do
    "$CC" -std=c99 -pedantic $WARN $SAN -g -O1 -I"$HERE/gen" -c "$HERE/c/$f.c" -o "$BUILD/$f.o"
done
# The generated header must also compile as plain C99 and as C++ on its own.
printf '#include "kilo_protocol.h"\nint kilo_dummy;\n' > "$BUILD/hdr_c99.c"
"$CC" -std=c99 -pedantic $WARN -I"$HERE/gen" -c "$BUILD/hdr_c99.c" -o "$BUILD/hdr_c99.o"
"$CXX" -std=c++17 $WARN $SAN -g -O1 -I"$HERE/gen" -I"$HERE/c" \
    "$HERE/c/test_protocol.cpp" "$BUILD/cobs.o" "$BUILD/crc16.o" "$BUILD/kilo_frame.o" \
    -o "$BUILD/test_protocol" $LDLIBS

echo "== C tests"
"$BUILD/test_protocol" "$HERE/vectors" "$FUZZ"

echo "== Python tests (+ C<->Python round trip)"
KILO_C_TEST_BIN="$BUILD/test_protocol" KILO_REQUIRE_C=1 \
    "$PYTHON" -m pytest -q "$HERE/py"

AVR_BIN="$(ls -d "$HOME"/.arduino15/packages/arduino/tools/avr-gcc/*/bin 2>/dev/null | head -1 || true)"
if [ -n "$AVR_BIN" ] && [ -x "$AVR_BIN/avr-gcc" ]; then
    echo "== avr-gcc (atmega328p, -Os)"
    for f in cobs crc16 kilo_frame; do
        "$AVR_BIN/avr-gcc" -mmcu=atmega328p -Os -std=c99 $WARN -I"$HERE/gen" \
            -c "$HERE/c/$f.c" -o "$BUILD/avr_$f.o"
    done
    "$AVR_BIN/avr-gcc" -mmcu=atmega328p -Os -std=c99 $WARN -DKILO_CRC16_USE_AVR_LIBC \
        -c "$HERE/c/crc16.c" -o "$BUILD/avr_crc16_libc.o"
    "$AVR_BIN/avr-gcc" -mmcu=atmega328p -Os -std=c99 -pedantic $WARN -I"$HERE/gen" \
        -c "$BUILD/hdr_c99.c" -o "$BUILD/avr_hdr.o"
    # The body sketch is C++: the headers must also build with avr-g++ (gnu++11).
    printf '%s\n' '#include "kilo_frame.h"' '#include "cobs.h"' '#include "crc16.h"' \
        'int kilo_probe(uint8_t t, const uint8_t *p, uint8_t l) {' \
        '    kilo_set_gait_t g; (void)g;' \
        '    return kilo_payload_len_check(t, p, l) + kilo_msg_kind(t) + kilo_msg_has_epoch(t); }' \
        > "$BUILD/hdr_avr.cpp"
    "$AVR_BIN/avr-g++" -mmcu=atmega328p -Os -std=gnu++11 $WARN -I"$HERE/gen" -I"$HERE/c" \
        -c "$BUILD/hdr_avr.cpp" -o "$BUILD/avr_hdr_cpp.o"
    "$AVR_BIN/avr-size" "$BUILD"/avr_cobs.o "$BUILD"/avr_crc16.o "$BUILD"/avr_crc16_libc.o "$BUILD"/avr_kilo_frame.o
else
    echo "== avr-gcc not found; skipping AVR build"
fi

echo "== all protocol tests passed"
