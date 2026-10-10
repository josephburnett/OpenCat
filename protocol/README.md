# Kilo protocol

The body (NyBoard firmware) and the mind (Python on the Pi) talk over a UART
at 250000 baud using COBS-framed messages with a CRC-16/CCITT-FALSE. The
design is in `docs/architecture.md` §4 and Appendix E. **`spec.toml` is the
single source of truth.** Everything in `gen/` and `vectors/` is generated
from it and committed, so consumers never need to run the generator.

```
spec.toml              messages, enums, flags, link constants
gen.py                 generator (stdlib only, Python 3.11+)
vector_cases.py        the test-vector cases gen.py turns into vectors/*.json
gen/kilo_protocol.h    C99/C++: ids, enums, packed payload structs, constants, length rules
gen/kilo_protocol.py   Python: dataclasses with pack()/unpack(), enums, constants
c/cobs.[ch]            COBS encode/decode
c/crc16.[ch]           CRC-16/CCITT-FALSE (same semantics as avr-libc _crc_xmodem_update)
c/kilo_frame.[ch]      frame encode/decode and a byte-at-a-time receiver
c/test_protocol.cpp    C tests (vectors, fuzz, cross-language helpers)
py/kilo_link_codec.py  Python COBS, CRC, frame build/parse, stream decoder
py/test_codec.py       pytest suite
vectors/frames.json    frames (every type, both directions), corrupt frames, streams
vectors/cobs.json      COBS-only vectors
test.sh                runs everything
```

## Regenerating

```
python3 protocol/gen.py          # rewrite gen/ and vectors/ from spec.toml
python3 protocol/gen.py --check  # exit 1 if anything committed is stale
```

Output is deterministic; running it twice changes nothing. Commit the spec
change and the regenerated files together.

## Testing

```
protocol/test.sh                 # everything; must pass before merging
FUZZ=10000 protocol/test.sh      # shorter C fuzz run (default 10^6)
PYTHON=~/venv/bin/python protocol/test.sh   # interpreter that has pytest
```

`test.sh` runs these steps:

1. `gen.py --check`: the generated files are up to date.
2. Builds the C codec with `-std=c99 -pedantic -Wall -Wextra -Werror` and
   `test_protocol.cpp` with `g++ -std=c++17 -Wall -Wextra -Werror
   -fsanitize=address,undefined`. Runs it against the vectors, then fuzzes the
   receiver.
3. Runs pytest on `py/`. This includes the C↔Python cross round trip: the C
   binary writes 5000 random frames that Python decodes, and Python writes
   5000 that C decodes (`test_protocol --emit` / `--check`).
4. If the Arduino avr-gcc is installed, compiles the C files for the
   ATmega328P (`-Os`) and the headers with avr-g++, then prints `avr-size`.

The only dependency outside the standard library is pytest, which is needed
for the Python tests. The test suite also fails when the generated files are
stale.

## Adding or changing a message

1. Edit `spec.toml`. Pi→body ids are 0x01-0x7F (`goal` or `command`), and
   body→Pi ids are 0x80-0xFF (`stream` or `event`). Give the payload fields
   with types, units, scale and ranges. Epoch-bearing goals set
   `epoch = true` and start with `epoch u8`. A variable tail must be the last
   field: either `type = "bytes"` with `min`/`max`, or a struct array with
   `count_field`. The maximum payload is 60 B, and `gen.py` rejects anything
   larger.
2. Add at least one case for it in `vector_cases.py`. The generator asserts
   that every message type has a vector.
3. Run `python3 protocol/gen.py`, then `protocol/test.sh`.
4. Bump `meta.proto_minor` for additive changes. Bump `proto_major` if a
   layout changes, because the mind refuses to drive a body with a different
   major.
5. Update `docs/architecture.md` §4 and Appendix E to match.

## Using it

C (body). Add `protocol/gen` and `protocol/c` to the include path and compile
`cobs.c`, `crc16.c` and `kilo_frame.c`. Feed UART bytes to `kilo_rx_push()`.
On `KILO_FRAME_OK`, `memcpy` the payload into the matching `kilo_*_t` struct.
On `KILO_FRAME_BAD_PAYLOAD_LEN` or `KILO_FRAME_UNKNOWN_TYPE`, the header can
be trusted, so a command gets an `ACK` with `BAD_LEN` or `UNKNOWN_TYPE`. Send
with `kilo_frame_encode()`. On AVR you may define `KILO_CRC16_USE_AVR_LIBC`
to use `_crc_xmodem_update`.

Python (mind). Put `protocol/py` and `protocol/gen` on `sys.path`, or vendor
`kilo_link_codec.py` and `kilo_protocol.py` into one package, where the codec
imports its sibling.

```python
import kilo_protocol as P, kilo_link_codec as C
wire = C.encode_message(P.SetPosture(epoch=e, posture=P.Posture.STAND), seq)
for r in C.StreamDecoder().feed(serial_bytes):
    if isinstance(r, C.FrameError): ...          # r.kind, r.type/r.seq if trusted
    else: msg = P.decode_payload(r.type, r.payload)
```
