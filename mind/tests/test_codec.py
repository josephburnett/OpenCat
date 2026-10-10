"""The mind's codec and messages adapters against protocol/vectors (the
vendored kilo_link_codec / kilo_protocol must reproduce them byte for byte)."""

import json
import random
from pathlib import Path

import pytest

from kilo.link.codec import Codec, DecodeError, Frame, KiloCodec, default_codec
from kilo.link.messages import KiloMessages, default_messages, fields_of
from kilo.protocol import kilo_protocol as P

VECTORS = Path(__file__).resolve().parents[2] / "protocol" / "vectors" / "frames.json"
VENDORED = Path(__file__).resolve().parents[1] / "kilo" / "protocol"
PROTOCOL = Path(__file__).resolve().parents[2] / "protocol"


@pytest.fixture(scope="module")
def vectors():
    if not VECTORS.exists():
        pytest.skip("protocol/vectors not present")
    return json.loads(VECTORS.read_text())


def test_defaults_are_real():
    assert isinstance(default_codec(), KiloCodec) and isinstance(default_codec(), Codec)
    assert isinstance(default_messages(), KiloMessages)
    assert default_messages().PROTO_MAJOR == P.PROTO_MAJOR


def test_vendored_copies_match_protocol():
    if not PROTOCOL.exists():
        pytest.skip("protocol/ not present")
    assert (VENDORED / "kilo_protocol.py").read_bytes() == (PROTOCOL / "gen" / "kilo_protocol.py").read_bytes()
    assert (VENDORED / "kilo_link_codec.py").read_bytes() == (PROTOCOL / "py" / "kilo_link_codec.py").read_bytes()


def test_worked_frame_appendix_e():
    assert KiloCodec().encode(0x01, 0, b"") == bytes.fromhex("0201033e2e00")


def test_frame_vectors_both_ways(vectors):
    c, m = KiloCodec(), KiloMessages()
    for v in vectors["frames"]:
        payload = m.pack(v["msg"], **{k: x for k, x in v["fields"].items()
                                      if k not in ("n", "count")})
        assert payload.hex() == v["payload"], v["name"]
        assert c.encode(v["type"], v["seq"], payload).hex() == v["wire"], v["name"]
        out = c.new_decoder().feed(bytes.fromhex(v["wire"]))
        assert out == [Frame(v["type"], v["seq"], payload)], v["name"]
        assert fields_of(m.unpack(v["type"], payload)) == v["fields"], v["name"]


def test_error_vectors(vectors):
    for e in vectors["errors"]:
        if e["wire"] == "00":
            continue  # a lone delimiter is an empty segment: the stream decoder skips it
        out = KiloCodec().new_decoder().feed(bytes.fromhex(e["wire"]))
        errs = [o for o in out if isinstance(o, DecodeError)]
        # a frame longer than 66 B is an rx overflow to a stream decoder
        want = {e["error"]} | ({"overflow"} if len(e["wire"]) > 2 * 66 else set())
        assert errs and errs[0].kind in want, e["name"]
        if "type" in e:  # CRC-valid: header trusted (the body ACKs BAD_LEN / UNKNOWN_TYPE)
            assert (errs[0].type, errs[0].seq) == (e["type"], e["seq"]), e["name"]


def test_stream_vectors(vectors):
    for s in vectors["streams"]:
        got = []
        for r in KiloCodec().new_decoder().feed(bytes.fromhex(s["bytes"])):
            got.append({"error": r.kind} if isinstance(r, DecodeError) else {"ok": r.type, "seq": r.seq})
        assert got == s["expect"], s["name"]


def test_split_feed_round_trip():
    c = KiloCodec()
    rnd = random.Random(1)
    m = KiloMessages()
    frames = []
    for i in range(200):
        tok = rnd.randrange(65536)
        frames.append(Frame(P.MsgId.PING, i & 0xFF, m.pack("PING", token=tok)))
    wire = b"".join(c.encode(*f) for f in frames)
    d = c.new_decoder()
    out = []
    i = 0
    while i < len(wire):
        n = rnd.randrange(1, 20)
        out += d.feed(wire[i:i + n])
        i += n
    assert out == frames


def test_messages_pack_errors_and_implied_counts():
    m = KiloMessages()
    with pytest.raises(ValueError):
        m.pack("SET_POSTURE", epoch=0, posture=3, bogus=1)
    with pytest.raises(ValueError):
        m.pack("PING", token=70000)
    with pytest.raises(ValueError):
        m.pack("NOPE")
    beep = m.pack("BEEP", notes=[{"note": 12, "dur_10ms": 10}, P.Note(note=0, dur_10ms=5)])
    assert beep == bytes([2, 12, 10, 0, 5])
    assert m.name_of(0x7D) == "0x7D" and m.type_of("ACK") == 0x84
    with pytest.raises(ValueError):
        m.unpack(0x84, b"\x00")


def test_encode_limits():
    with pytest.raises(ValueError):
        KiloCodec().encode(1, 0, bytes(61))
    with pytest.raises(ValueError):
        KiloCodec().encode(256, 0, b"")
