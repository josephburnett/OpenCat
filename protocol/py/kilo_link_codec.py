"""Kilo link framing: COBS + CRC-16/CCITT-FALSE (docs/architecture.md §4).

    decoded: type u8 | seq u8 | payload (0..60) | crc16 u16 LE   (4..64 B)
    wire:    COBS(decoded) | 0x00                                (<= 66 B)
    crc16 = CRC-16/CCITT-FALSE over type, seq, payload.

Standard library only. Needs the generated `kilo_protocol` module (from
protocol/gen/) for per-type length rules; it is imported as a sibling when this
file is vendored into a package, or from sys.path otherwise.
"""
from __future__ import annotations

import binascii
from dataclasses import dataclass
from typing import Iterator

try:  # vendored next to kilo_protocol inside a package
    from . import kilo_protocol as _P  # type: ignore[attr-defined]
except ImportError:  # plain module on sys.path
    import kilo_protocol as _P  # type: ignore[no-redef]

MAX_PAYLOAD = _P.MAX_PAYLOAD          # 60
MIN_DECODED = _P.MIN_DECODED          # 4
MAX_DECODED = _P.MAX_DECODED          # 64
MAX_WIRE = _P.MAX_WIRE                # 66, including the 0x00 delimiter
MAX_ENCODED = MAX_WIRE - 1            # 65, COBS bytes before the delimiter

# Error classes (FrameError.kind); the same strings appear in the vectors.
BAD_COBS = "bad_cobs"          # COBS structure invalid (zero byte, code past end, empty)
BAD_CRC = "bad_crc"            # CRC mismatch (type/seq untrustworthy: never ACK)
BAD_LENGTH = "bad_length"      # decoded < 4 or > 64 B, or payload length wrong for type
UNKNOWN_TYPE = "unknown_type"  # CRC good but type not in the spec
OVERFLOW = "overflow"          # stream only: > 65 B before a delimiter (rx_overflow)
ERROR_KINDS = (BAD_COBS, BAD_CRC, BAD_LENGTH, UNKNOWN_TYPE, OVERFLOW)


class CobsError(ValueError):
    pass


class FrameError(ValueError):
    """A frame was rejected. `kind` is one of ERROR_KINDS. For BAD_LENGTH caused
    by a payload/type mismatch and for UNKNOWN_TYPE the CRC was good, so `type`
    and `seq` are trustworthy (the body ACKs commands with BAD_LEN /
    UNKNOWN_TYPE); otherwise they are None."""

    def __init__(self, kind: str, msg: str = "", type: int | None = None,
                 seq: int | None = None, payload: bytes | None = None):
        super().__init__(f"{kind}: {msg}" if msg else kind)
        self.kind = kind
        self.type = type
        self.seq = seq
        self.payload = payload


@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    payload: bytes


# ---------------------------------------------------------------------------
# CRC

def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
    crc16(b"123456789") == 0x29B1. Pass `crc` to continue a running CRC."""
    return binascii.crc_hqx(data, crc)


# ---------------------------------------------------------------------------
# COBS

def cobs_encode(data: bytes) -> bytes:
    """COBS-encode `data`. The result contains no 0x00 and does not include the
    trailing delimiter; it is at most len(data) + 1 + len(data) // 254 bytes.
    Data ending exactly on a 254-byte non-zero run gets no extra 0x01 code
    (the common encoder; the C implementation matches)."""
    out = bytearray([0])
    code_idx: int | None = 0
    code = 1
    n = len(data)
    for i, b in enumerate(data):
        if b:
            out.append(b)
            code += 1
        if not b or code == 0xFF:
            out[code_idx] = code  # type: ignore[index]
            code = 1
            if not b or i < n - 1:
                code_idx = len(out)
                out.append(0)
            else:
                code_idx = None
    if code_idx is not None:
        out[code_idx] = code
    return bytes(out)


def cobs_decode(enc: bytes) -> bytes:
    """Decode one COBS block (without the delimiter). Raises CobsError."""
    if not enc:
        raise CobsError("empty")
    out = bytearray()
    i, n = 0, len(enc)
    while i < n:
        code = enc[i]
        if code == 0:
            raise CobsError(f"zero byte at {i}")
        end = i + code
        if end > n:
            raise CobsError(f"code {code} at {i} runs past end ({n})")
        chunk = enc[i + 1:end]
        if 0 in chunk:
            raise CobsError("zero byte inside block")
        out += chunk
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# Frames

def build_decoded(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """type | seq | payload | crc16 LE."""
    if not 0 <= msg_type <= 0xFF or not 0 <= seq <= 0xFF:
        raise ValueError("type and seq must be u8")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} B > {MAX_PAYLOAD}")
    body = bytes([msg_type, seq]) + bytes(payload)
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def build_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """Wire bytes for one frame, including the trailing 0x00 delimiter.
    Does not check the payload against the type's length rule (see
    encode_message for that)."""
    return cobs_encode(build_decoded(msg_type, seq, payload)) + b"\x00"


def encode_message(msg, seq: int) -> bytes:
    """Wire bytes for a generated kilo_protocol dataclass instance."""
    return build_frame(msg.ID, seq, msg.pack())


def parse_decoded(dec: bytes, check_type: bool = True) -> Frame:
    """Validate a COBS-decoded frame: length, CRC, then (if check_type) the
    type and the payload length rule. Raises FrameError."""
    if len(dec) < MIN_DECODED or len(dec) > MAX_DECODED:
        raise FrameError(BAD_LENGTH, f"decoded length {len(dec)}")
    body, rx = dec[:-2], dec[-2] | (dec[-1] << 8)
    calc = crc16(body)
    if calc != rx:
        raise FrameError(BAD_CRC, f"crc {rx:#06x} != {calc:#06x}")
    t, s, payload = body[0], body[1], bytes(body[2:])
    if check_type:
        st = _P.payload_len_status(t, payload)
        if st == _P.LEN_UNKNOWN_TYPE:
            raise FrameError(UNKNOWN_TYPE, f"type {t:#04x}", t, s, payload)
        if st != _P.LEN_OK:
            raise FrameError(BAD_LENGTH, f"payload {len(payload)} B for type {t:#04x}", t, s, payload)
    return Frame(t, s, payload)


def parse_frame(wire: bytes, check_type: bool = True) -> Frame:
    """Parse one frame. `wire` may include the trailing 0x00 delimiter but no
    other zero byte. Raises FrameError with kind in ERROR_KINDS."""
    wire = bytes(wire)
    if wire.endswith(b"\x00"):
        wire = wire[:-1]
    if len(wire) > MAX_ENCODED:
        raise FrameError(BAD_LENGTH, f"encoded length {len(wire)}")
    try:
        dec = cobs_decode(wire)
    except CobsError as e:
        raise FrameError(BAD_COBS, str(e)) from None
    return parse_decoded(dec, check_type)


def decode_message(wire: bytes):
    """parse_frame + unpack into the generated dataclass. Returns (frame, msg)."""
    f = parse_frame(wire)
    return f, _P.decode_payload(f.type, f.payload)


class StreamDecoder:
    """Incremental receiver following §4's receiver rules.

    feed(bytes) yields Frame objects and FrameError objects (not raised) in
    arrival order. Empty segments (back-to-back delimiters) are ignored, so a
    sender may prefix 0x00 to force resync. More than MAX_ENCODED bytes before
    a delimiter yields one FrameError(OVERFLOW) and discards up to the next
    delimiter."""

    def __init__(self, check_type: bool = True):
        self._buf = bytearray()
        self._overflow = False
        self.check_type = check_type

    def feed(self, data: bytes) -> Iterator[Frame | FrameError]:
        for b in data:
            if b != 0:
                if self._overflow:
                    continue
                if len(self._buf) >= MAX_ENCODED:
                    self._overflow = True
                    self._buf.clear()
                    yield FrameError(OVERFLOW, "frame longer than max before delimiter")
                    continue
                self._buf.append(b)
                continue
            if self._overflow:
                self._overflow = False
                continue
            if not self._buf:
                continue
            seg = bytes(self._buf)
            self._buf.clear()
            try:
                yield parse_frame(seg, self.check_type)
            except FrameError as e:
                yield e
