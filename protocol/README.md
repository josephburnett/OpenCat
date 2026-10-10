# Kilo protocol

The body (NyBoard firmware) and the mind (Python on the Pi) talk over a UART
at 250000 baud using COBS-framed messages with a CRC-16/CCITT-FALSE. The
design is in `docs/architecture.md` §4 and Appendix E. **`spec.toml` is the
single source of truth.** Everything in `gen/` and `vectors/` is generated
from it and committed, so consumers never need to run the generator.

```
spec.toml              messages, enums, flags, link constants, live params
gen.py                 generator (stdlib only, Python 3.11+)
vendor.py              copies gen/ + c/ into body/src/proto, gen/ + py/ codec into mind/kilo/protocol
vector_cases.py        the test-vector cases gen.py turns into vectors/*.json
gen/kilo_protocol.h    C99/C++: ids, enums, packed payload structs, constants, length rules
gen/kilo_protocol.py   Python: dataclasses with pack()/unpack(), enums, constants
c/cobs.[ch]            COBS encode/decode
c/crc16.[ch]           CRC-16/CCITT-FALSE (same semantics as avr-libc _crc_xmodem_update)
c/kilo_frame.[ch]      frame encode/decode (frame-sized COBS, small on AVR) and a byte-at-a-time receiver
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
python3 protocol/vendor.py       # copy the results into body/src/proto and mind/kilo/protocol
python3 protocol/gen.py --check  # exit 1 if anything committed is stale
python3 protocol/vendor.py --check
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

1. `gen.py --check`: the generated files are up to date; `vendor.py --check`:
   the body's and mind's copies are identical to them.
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
3. Run `python3 protocol/gen.py` and `python3 protocol/vendor.py`, then
   `protocol/test.sh`, `body/tools/host_tests.sh` and the mind's pytest (its
   `tests/test_crosscheck.py` plays every vector through the body's C dispatch
   and the virtual body).
4. Bump `meta.proto_minor` for additive changes. Bump `proto_major` if a
   layout changes, because the mind refuses to drive a body with a different
   major.
5. Update `docs/architecture.md` §4 and Appendix E to match.

## Using it

C (body). The body compiles the vendored copies in `body/src/proto/`
(`kilo_frame.c` and `crc16.c`; `cobs.c` is the general codec, unused by the
frame path). Elsewhere, add `protocol/gen` and `protocol/c` to the include
path and compile them. Feed UART bytes to `kilo_rx_push()`.
On `KILO_FRAME_OK`, `memcpy` the payload into the matching `kilo_*_t` struct.
On `KILO_FRAME_BAD_PAYLOAD_LEN` or `KILO_FRAME_UNKNOWN_TYPE`, the header can
be trusted, so a command gets an `ACK` with `BAD_LEN` or `UNKNOWN_TYPE`. Send
with `kilo_frame_encode()`. On AVR you may define `KILO_CRC16_USE_AVR_LIBC`
to use `_crc_xmodem_update`.

Python (mind). The mind imports the vendored package `kilo.protocol`
(`from kilo.protocol import kilo_protocol, kilo_link_codec`); the codec
imports its sibling. Elsewhere, put `protocol/py` and `protocol/gen` on
`sys.path`.

```python
import kilo_protocol as P, kilo_link_codec as C
wire = C.encode_message(P.SetPosture(epoch=e, posture=P.Posture.STAND), seq)
for r in C.StreamDecoder().feed(serial_bytes):
    if isinstance(r, C.FrameError): ...          # r.kind, r.type/r.seq if trusted
    else: msg = P.decode_payload(r.type, r.payload)
```

## Message classes

- **Goals** (`kind = "goal"`) are never acked. A rejection is
  `EVENT GOAL_REFUSED` with `arg = type << 8 | reason`, where the reason is an
  `ack_status` code, at most once per goal type per second.
- **Queries** (`kind = "command"`, `query = true`) are answered by the message
  whose `reply_to` names them, and get an `ACK` only on failure. They are never
  de-duplicated; the mind retries them with a fresh `seq`.
- **Other commands** get exactly one `ACK`. The body caches the last executed
  one's `(seq, type, status)` for `dedup_window_ms` and re-acks a repeat
  without executing it; the mind retries with the same `seq`.

Generated helpers: C `kilo_msg_is_goal()`, `kilo_msg_is_query()`,
`kilo_msg_has_epoch()`, `kilo_msg_kind()`, `kilo_payload_len_check()` (a
flash-resident table on AVR); Python `IS_QUERY`/`HAS_EPOCH`/`KIND` class
attributes and `QUERY_REPLY`.

## Params

`[[params]]` entries have a stable `id` (dense from 0, below
`link.param_slots`), `default`, `min`, `max`, `unit` and `doc`. C gets
`KILO_PARAM_<NAME>` ids, `_DEFAULT`/`_MIN`/`_MAX` macros and the
`KILO_PARAM_TABLE_INIT` / `KILO_PARAM_DEFAULTS_INIT` initialisers (the body puts
the table in flash); Python gets `ParamId`, `PARAMS` and `PARAMS_BY_NAME`. Add a
param by appending an entry with the next id and implementing it in
`body/src/core` (`PARAM(NAME)` reads the live value).
