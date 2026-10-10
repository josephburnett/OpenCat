"""Codec interface: framing of (type, seq, payload) records on the wire.

This is the seam between the link and the framing implementation. The real
framing (COBS + CRC-16/CCITT-FALSE, docs/architecture.md §4) lives in
protocol/'s ``kilo_link_codec.py``; until it is integrated, ``codec_stub``
provides a stand-in. Tests use a trivial length-prefixed codec to exercise the
plumbing independently of the framing.

A codec only frames bytes. It does not know message layouts (that is the
``Messages`` adapter in ``kilo.link.messages``) and it does not check that the
payload length matches the type; the message layer does that.

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

MAX_PAYLOAD = 60  # §4: payload 0..60 B


class Frame(NamedTuple):
    """One decoded frame. ``type`` and ``seq`` are u8, ``payload`` is raw bytes."""

    type: int
    seq: int
    payload: bytes


class DecodeError(NamedTuple):
    """A frame that could not be decoded.

    ``kind`` is one of: ``"overflow"`` (more than the maximum frame size before a
    delimiter; maps to the body's ``rx_overflow``), ``"crc"`` (``rx_crc_err``),
    ``"cobs"`` (malformed stuffing), ``"short"`` (decoded length < 4). Codecs may
    add other kinds; the link counts them all as decode errors. ``raw`` holds
    the offending bytes (possibly truncated) for logging.
    """

    kind: str
    raw: bytes = b""


@runtime_checkable
class Decoder(Protocol):
    def feed(self, data: bytes) -> list[Frame | DecodeError]: ...


@runtime_checkable
class Codec(Protocol):
    def encode(self, type: int, seq: int, payload: bytes) -> bytes: ...

    def new_decoder(self) -> Decoder: ...


def default_codec() -> Codec:
    """The codec the CLI and daemon use.

    TODO(protocol): return the protocol/ codec (kilo_link_codec) once it is
    integrated, adapting it to this interface if its names differ.
    """
    from kilo.link.codec_stub import StubCodec

    return StubCodec()
