import os
import random

import pytest

from kilo.link.codec import Codec, DecodeError, Frame
from kilo.link.codec_stub import StubCodec, cobs_decode, cobs_encode, crc16


def test_crc_check_value():
    assert crc16(b"123456789") == 0x29B1


def test_worked_frame_appendix_e():
    # HELLO_REQ seq=0: decoded 01 00 3e 2e, wire 02 01 03 3e 2e 00
    assert StubCodec().encode(0x01, 0, b"") == bytes.fromhex("0201033e2e00")


def test_interface():
    c = StubCodec()
    assert isinstance(c, Codec)


def test_round_trip_and_split_feed():
    c = StubCodec()
    rnd = random.Random(1)
    frames = [Frame(rnd.randrange(256), rnd.randrange(256), bytes(rnd.choice([0, 1, 255]) if rnd.random() < .5
                    else rnd.randrange(256) for _ in range(rnd.randrange(61)))) for _ in range(200)]
    wire = b"".join(c.encode(*f) for f in frames)
    d = c.new_decoder()
    out = []
    i = 0
    while i < len(wire):
        n = rnd.randrange(1, 20)
        out += d.feed(wire[i:i + n])
        i += n
    assert out == frames


def test_cobs_round_trip_edge_cases():
    for data in (b"", b"\x00", b"\x00\x00", bytes(range(1, 255)) * 2, bytes(300)):
        assert cobs_decode(cobs_encode(data)) == data


def test_crc_error_and_resync():
    c = StubCodec()
    good = c.encode(0x81, 7, b"\x01\x02")
    bad = bytearray(good)
    bad[2] ^= 0x40
    d = c.new_decoder()
    out = d.feed(b"\x13\x37garbage\x00" + bytes(bad) + good)
    kinds = [o.kind for o in out if isinstance(o, DecodeError)]
    assert "crc" in kinds
    assert out[-1] == Frame(0x81, 7, b"\x01\x02")


def test_overflow_discards_to_next_delimiter():
    c = StubCodec()
    d = c.new_decoder()
    out = d.feed(b"\x01" * 100 + b"\x00" + c.encode(2, 1, b""))
    assert isinstance(out[0], DecodeError) and out[0].kind == "overflow"
    assert out[-1] == Frame(2, 1, b"")
    assert len(out) == 2


def test_encode_limits():
    with pytest.raises(ValueError):
        StubCodec().encode(1, 0, bytes(61))
    with pytest.raises(ValueError):
        StubCodec().encode(256, 0, b"")
    assert len(StubCodec().encode(1, 0, os.urandom(60))) <= 66
