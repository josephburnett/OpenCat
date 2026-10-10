"""STUB codec. TODO(protocol): replaced by protocol/'s kilo_link_codec at integration.

This is a minimal, unoptimised transcription of the §4 framing so that the
mind, the virtual body and the CLI can talk to each other before the real
codec lands::

    decoded: type u8 | seq u8 | payload (0..60) | crc16 u16 LE   (4..64 B)
    wire:    COBS(decoded) | 0x00                                (<= 66 B)

It is deliberately not the reference implementation: the protocol/ codec is,
and it is the one tested against protocol/vectors.json.
"""

from __future__ import annotations

import binascii

from kilo.link.codec import MAX_PAYLOAD, DecodeError, Frame

MAX_DECODED = 4 + MAX_PAYLOAD  # 64
MAX_WIRE = MAX_DECODED + 2  # 66 including the delimiter; 65 before it


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    block = bytearray()
    for b in data:
        if b == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
        else:
            block.append(b)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("zero byte inside COBS frame")
        i += 1
        end = i + code - 1
        if end > n:
            raise ValueError("COBS block overruns frame")
        out += data[i:end]
        i = end
        if code != 255 and i < n:
            out.append(0)
    return bytes(out)


class StubDecoder:
    def __init__(self) -> None:
        self._buf = bytearray()
        self._discarding = False

    def feed(self, data: bytes) -> list[Frame | DecodeError]:
        out: list[Frame | DecodeError] = []
        for b in data:
            if b != 0:
                if self._discarding:
                    continue
                self._buf.append(b)
                if len(self._buf) > MAX_WIRE - 1:
                    out.append(DecodeError("overflow", bytes(self._buf[:16])))
                    self._buf.clear()
                    self._discarding = True
                continue
            # delimiter
            if self._discarding:
                self._discarding = False
                continue
            raw = bytes(self._buf)
            self._buf.clear()
            if not raw:
                continue  # empty frame between delimiters: ignore
            try:
                dec = cobs_decode(raw)
            except ValueError:
                out.append(DecodeError("cobs", raw))
                continue
            if len(dec) < 4:
                out.append(DecodeError("short", raw))
                continue
            body, crc = dec[:-2], int.from_bytes(dec[-2:], "little")
            if crc16(body) != crc:
                out.append(DecodeError("crc", raw))
                continue
            out.append(Frame(body[0], body[1], bytes(body[2:])))
        return out


class StubCodec:
    """TODO(protocol): stand-in for the protocol/ codec."""

    def encode(self, type: int, seq: int, payload: bytes) -> bytes:
        if not (0 <= type <= 0xFF and 0 <= seq <= 0xFF):
            raise ValueError("type and seq must be u8")
        if len(payload) > MAX_PAYLOAD:
            raise ValueError(f"payload too long ({len(payload)} > {MAX_PAYLOAD})")
        body = bytes((type, seq)) + bytes(payload)
        dec = body + crc16(body).to_bytes(2, "little")
        return cobs_encode(dec) + b"\x00"

    def new_decoder(self) -> StubDecoder:
        return StubDecoder()
