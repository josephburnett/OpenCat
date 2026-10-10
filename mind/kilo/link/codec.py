"""Codec interface: framing of (type, seq, payload) records on the wire.

This is the seam between the link and the framing implementation. The real
framing (COBS + CRC-16/CCITT-FALSE, docs/architecture.md §4) is protocol/'s
``kilo_link_codec.py``, vendored byte for byte into ``kilo.protocol`` by
``protocol/vendor.py``; ``KiloCodec`` adapts it to this interface. Tests may
plug in other codecs (e.g. a trivial length-prefixed one) to exercise the
link's plumbing independently of the framing.

Interface (structural, any object with these methods works)::

    codec.encode(type: int, seq: int, payload: bytes) -> bytes
        One complete wire frame, including the trailing delimiter.
        Raises ValueError if type/seq are not u8 or payload is too long.

    codec.new_decoder() -> Decoder
        A fresh streaming decoder with empty state.

    decoder.feed(data: bytes) -> list[Frame | DecodeError]
        Consume any number of bytes (possibly partial frames) and return
        the frames and errors completed by them, in wire order.
"""

from __future__ import annotations

from typing import NamedTuple, Protocol, runtime_checkable

from kilo.protocol import kilo_link_codec as _C

MAX_PAYLOAD = _C.MAX_PAYLOAD  # §4: payload 0..60 B


class Frame(NamedTuple):
    """One decoded frame. ``type`` and ``seq`` are u8, ``payload`` is raw bytes."""

    type: int
    seq: int
    payload: bytes


class DecodeError(NamedTuple):
    """A frame that could not be decoded.

    ``kind`` is one of kilo_link_codec's error kinds: ``"bad_cobs"``,
    ``"bad_crc"``, ``"bad_length"``, ``"unknown_type"``, ``"overflow"``. For
    ``bad_length`` on a CRC-valid frame and for ``unknown_type`` the header was
    trustworthy and ``type``/``seq`` are set (the body ACKs such commands with
    BAD_LEN / UNKNOWN_TYPE); otherwise they are None. ``raw`` holds the payload
    when known, for logging.
    """

    kind: str
    raw: bytes = b""
    type: int | None = None
    seq: int | None = None


@runtime_checkable
class Decoder(Protocol):
    def feed(self, data: bytes) -> list[Frame | DecodeError]: ...


@runtime_checkable
class Codec(Protocol):
    def encode(self, type: int, seq: int, payload: bytes) -> bytes: ...

    def new_decoder(self) -> Decoder: ...


class KiloDecoder:
    """Streaming decoder over kilo_link_codec.StreamDecoder (checks type and length)."""

    def __init__(self) -> None:
        self._d = _C.StreamDecoder(check_type=True)

    def feed(self, data: bytes) -> list[Frame | DecodeError]:
        out: list[Frame | DecodeError] = []
        for r in self._d.feed(bytes(data)):
            if isinstance(r, _C.FrameError):
                out.append(DecodeError(r.kind, r.payload or b"", r.type, r.seq))
            else:
                out.append(Frame(r.type, r.seq, r.payload))
        return out


class KiloCodec:
    """The real §4 framing: COBS(type | seq | payload | crc16 LE) | 0x00."""

    def encode(self, type: int, seq: int, payload: bytes) -> bytes:
        return _C.build_frame(type, seq, bytes(payload))

    def new_decoder(self) -> KiloDecoder:
        return KiloDecoder()


def default_codec() -> Codec:
    """The codec the CLI, daemon and virtual body use."""
    return KiloCodec()
