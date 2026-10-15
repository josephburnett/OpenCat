"""Python codec tests against protocol/vectors/*.json, plus the C<->Python
cross round trip (needs the C test binary; protocol/test.sh provides it via
KILO_C_TEST_BIN and sets KILO_REQUIRE_C=1 so a missing binary fails)."""
from __future__ import annotations

import dataclasses
import json
import os
import random
import subprocess
from pathlib import Path

import pytest

import gen  # protocol/gen.py
import kilo_link_codec as C
import kilo_protocol as P

ROOT = Path(__file__).resolve().parent.parent
FRAMES = json.loads((ROOT / "vectors" / "frames.json").read_text())
COBS = json.loads((ROOT / "vectors" / "cobs.json").read_text())


# --- generator ---------------------------------------------------------------

def test_generated_files_are_fresh():
    for path, text in gen.render().items():
        assert path.read_text() == text, f"{path} is stale: run python3 protocol/gen.py"


def test_spec_constants():
    assert P.BAUD == 250000
    assert P.MAX_PAYLOAD == 60
    assert P.FRAME_OVERHEAD == 6
    assert P.MAX_WIRE == 66
    assert P.TELEMETRY_DEFAULT_HZ == 25
    assert P.WD_STOP_MS == 500 and P.WD_REST_MS == 1500
    assert P.Telemetry.MIN_LEN == 35 and P.Stats.MIN_LEN == 23 and P.Hello.MIN_LEN == 16 and P.Feet.MIN_LEN == 16


# --- CRC and COBS ------------------------------------------------------------

def test_crc_check_value():
    assert C.crc16(b"123456789") == 0x29B1 == FRAMES["crc_check"]["crc"] == P.CRC_CHECK


def test_crc_matches_bitwise_reference():
    def ref(data):
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc
    rng = random.Random(7)
    for n in range(0, 70):
        d = bytes(rng.randrange(256) for _ in range(n))
        assert C.crc16(d) == ref(d)


@pytest.mark.parametrize("v", COBS["vectors"], ids=lambda v: v["source"])
def test_cobs_vectors(v):
    data, enc = bytes.fromhex(v["data"]), bytes.fromhex(v["encoded"])
    assert C.cobs_encode(data) == enc
    assert 0 not in enc
    assert C.cobs_decode(enc) == data


def test_cobs_published_examples():
    # Independent of the generated vectors.
    assert C.cobs_encode(b"\x00") == b"\x01\x01"
    assert C.cobs_encode(b"\x11\x22\x00\x33") == b"\x03\x11\x22\x02\x33"
    assert C.cobs_encode(b"\x11\x00\x00\x00") == b"\x02\x11\x01\x01\x01"
    r = bytes(range(1, 255))
    assert C.cobs_encode(r) == b"\xff" + r


@pytest.mark.parametrize("enc", [b"", b"\x00", b"\x05\x01\x02", b"\x03\x01\x00\x02", b"\x02\x01\x03\x3e"])
def test_cobs_decode_rejects(enc):
    with pytest.raises(C.CobsError):
        C.cobs_decode(enc)


def test_cobs_random_roundtrip():
    rng = random.Random(99)
    for _ in range(2000):
        n = rng.randrange(0, 700)
        d = bytes(0 if rng.random() < 0.2 else rng.randrange(1, 256) for _ in range(n))
        e = C.cobs_encode(d)
        assert 0 not in e and len(e) <= n + 1 + n // 254
        assert C.cobs_decode(e) == d


# --- frames ------------------------------------------------------------------

def test_doc_worked_examples_literal():
    # Literals from docs/architecture.md Appendix E, independent of the vectors.
    assert C.build_frame(0x01, 0, b"") == bytes.fromhex("0201033e2e00")
    assert C.build_decoded(0x01, 0) == bytes.fromhex("01003e2e")
    msg = P.SetPosture(epoch=0, posture=P.Posture.STAND, transition_ticks=0, flags=P.PostureFlags.BALANCE)
    assert msg.pack() == bytes.fromhex("00030001")
    assert C.encode_message(msg, 1) == bytes.fromhex("03100102030401b4f700")
    names = {f["name"] for f in FRAMES["frames"]}
    assert {"doc_hello_req_seq0", "doc_set_posture_stand_seq1"} <= names


def test_vectors_cover_every_message():
    assert len(FRAMES["frames"]) >= 30
    types = {f["type"] for f in FRAMES["frames"]}
    assert types == set(P.MESSAGES)
    assert {f["dir"] for f in FRAMES["frames"]} == {"pi_to_body", "body_to_pi"}
    assert any(len(f["payload"]) == 2 * P.MAX_PAYLOAD for f in FRAMES["frames"])
    assert any(f["payload"].count("00") >= 20 for f in FRAMES["frames"])


@pytest.mark.parametrize("v", FRAMES["frames"], ids=lambda v: v["name"])
def test_frame_vectors(v):
    cls = P.MESSAGES[v["type"]]
    assert cls.NAME == v["msg"] and cls.KIND.value == v["kind"] and cls.DIR.value == v["dir"]
    obj = P.from_dict(cls, v["fields"])
    payload = bytes.fromhex(v["payload"])
    assert obj.pack() == payload
    assert cls.unpack(payload) == obj
    assert P.to_dict(cls.unpack(payload)) == v["fields"]
    wire = bytes.fromhex(v["wire"])
    assert C.build_frame(v["type"], v["seq"], payload) == wire
    assert C.encode_message(obj, v["seq"]) == wire
    assert C.build_decoded(v["type"], v["seq"], payload) == bytes.fromhex(v["decoded"])
    assert len(wire) == len(payload) + P.FRAME_OVERHEAD
    f, msg = C.decode_message(wire)
    assert (f.type, f.seq, f.payload) == (v["type"], v["seq"], payload)
    assert msg == obj
    if cls.HAS_EPOCH:
        assert payload[0] == v["fields"]["epoch"]


@pytest.mark.parametrize("v", FRAMES["errors"], ids=lambda v: v["name"])
def test_error_vectors(v):
    with pytest.raises(C.FrameError) as ei:
        C.parse_frame(bytes.fromhex(v["wire"]))
    assert ei.value.kind == v["error"]
    if "type" in v:
        assert (ei.value.type, ei.value.seq) == (v["type"], v["seq"])
    else:
        assert ei.value.type is None


@pytest.mark.parametrize("v", FRAMES["streams"], ids=lambda v: v["name"])
def test_stream_vectors(v):
    got = []
    for r in C.StreamDecoder().feed(bytes.fromhex(v["bytes"])):
        got.append({"error": r.kind} if isinstance(r, C.FrameError) else {"ok": r.type, "seq": r.seq})
    assert got == v["expect"]


def test_stream_byte_at_a_time_matches_bulk():
    data = b"".join(bytes.fromhex(f["wire"]) for f in FRAMES["frames"])
    dec = C.StreamDecoder()
    one = [r for b in data for r in dec.feed(bytes([b]))]
    assert [(f.type, f.seq, f.payload) for f in one] == [
        (f["type"], f["seq"], bytes.fromhex(f["payload"])) for f in FRAMES["frames"]]


def test_build_rejects_oversize():
    with pytest.raises(ValueError):
        C.build_frame(0x8F, 0, b"x" * 61)


def test_pack_rejects_bad_values():
    with pytest.raises(P.PayloadError):
        P.Ping(token=0x10000).pack()
    with pytest.raises(P.PayloadError):
        P.Beep(notes=[]).pack()
    with pytest.raises(P.PayloadError):
        P.TrickFrames(first=0, frames=[P.Keyframe()] * 5).pack()
    with pytest.raises(P.PayloadError):
        P.Joints(angle=[0] * 10).pack()
    with pytest.raises(P.PayloadError):
        P.Telemetry.unpack(b"\x00" * 25)


# --- property-style round trips of every message ------------------------------

def _random_value(rng, cls):
    """Random instance of a message/struct without a derived count field."""
    kw = {}
    fmt_types = {"B": (0, 255), "b": (-128, 127), "H": (0, 65535), "h": (-32768, 32767), "I": (0, 2**32 - 1)}
    # Walk the struct format alongside the dataclass fields.
    chars = list(cls._FMT.format.lstrip("<"))
    idx = 0
    for f in dataclasses.fields(cls):
        t = str(f.type)
        if t == "bytes":
            lo = cls.MIN_LEN - cls._FMT.size
            hi = cls.MAX_LEN - cls._FMT.size
            kw[f.name] = bytes(rng.randrange(256) for _ in range(rng.randint(lo, hi)))
        elif t.startswith("list[") and t != "list[int]":
            ec = getattr(P, t[5:-1])
            lo = (cls.MIN_LEN - cls._FMT.size) // ec.SIZE
            hi = (cls.MAX_LEN - cls._FMT.size) // ec.SIZE
            kw[f.name] = [_random_value(rng, ec) for _ in range(rng.randint(lo, hi))]
        elif t == "list[int]":
            n = len(f.default_factory())
            vals = []
            for _ in range(n):
                lo, hi = fmt_types[chars[idx]]
                vals.append(rng.randint(lo, hi))
                idx += 1
            kw[f.name] = vals
        else:
            lo, hi = fmt_types[chars[idx]]
            kw[f.name] = rng.randint(lo, hi)
            idx += 1
    return cls(**kw)


@pytest.mark.parametrize("msg_id", sorted(P.MESSAGES), ids=lambda i: P.MESSAGES[i].NAME)
def test_random_roundtrip_every_message(msg_id):
    cls = P.MESSAGES[msg_id]
    rng = random.Random(msg_id)
    for _ in range(200):
        if cls in (P.TrickFrames, P.Beep):
            # count field is derived; build the variable tail explicitly
            ec = P.Keyframe if cls is P.TrickFrames else P.Note
            items = [ec.unpack(bytes(rng.randrange(256) for _ in range(ec.SIZE)))
                     for _ in range(rng.randint(1, 4 if cls is P.TrickFrames else 8))]
            obj = cls(**({"first": rng.randrange(256), "frames": items} if cls is P.TrickFrames else {"notes": items}))
        else:
            obj = _random_value(rng, cls)
        payload = obj.pack()
        assert cls.MIN_LEN <= len(payload) <= cls.MAX_LEN
        assert P.payload_len_status(cls.ID, payload) == P.LEN_OK
        seq = rng.randrange(256)
        wire = C.encode_message(obj, seq)
        assert 0 not in wire[:-1] and wire[-1] == 0 and len(wire) <= P.MAX_WIRE
        f, back = C.decode_message(wire)
        assert f.seq == seq and back == obj


# --- C <-> Python cross round trip --------------------------------------------

def _c_bin():
    b = os.environ.get("KILO_C_TEST_BIN") or str(ROOT / "build" / "test_protocol")
    if not Path(b).exists():
        if os.environ.get("KILO_REQUIRE_C"):
            pytest.fail(f"C test binary not found: {b}")
        pytest.skip("C test binary not built (run protocol/test.sh)")
    return b


def test_c_encodes_python_decodes(tmp_path):
    out = tmp_path / "c_frames.txt"
    subprocess.run([_c_bin(), "--emit", str(out), "5000", "4242"], check=True, capture_output=True)
    n = 0
    for line in out.read_text().splitlines():
        t, s, ph, wh = line.split()
        payload = b"" if ph == "-" else bytes.fromhex(ph)
        wire = bytes.fromhex(wh)
        f, msg = C.decode_message(wire)
        assert (f.type, f.seq, f.payload) == (int(t), int(s), payload)
        assert msg.pack() == payload
        assert C.build_frame(int(t), int(s), payload) == wire
        n += 1
    assert n == 5000


def test_python_encodes_c_decodes(tmp_path):
    rng = random.Random(2024)
    lines = []
    ids = sorted(P.MESSAGES)
    for _ in range(5000):
        cls = P.MESSAGES[rng.choice(ids)]
        lo, hi = cls.MIN_LEN, cls.MAX_LEN
        payload = bytearray(0 if rng.random() < 0.3 else rng.randrange(256) for _ in range(hi))
        if cls in (P.TrickFrames, P.Beep):
            k = rng.randint(1, 4 if cls is P.TrickFrames else 8)
            n = (2 if cls is P.TrickFrames else 1) + k * (14 if cls is P.TrickFrames else 2)
            payload = payload[:n]
            payload[1 if cls is P.TrickFrames else 0] = k
        else:
            payload = payload[:rng.randint(lo, hi)]
        payload = bytes(payload)
        assert P.payload_len_status(cls.ID, payload) == P.LEN_OK
        seq = rng.randrange(256)
        lines.append(f"{cls.ID} {seq} {payload.hex() or '-'} {C.build_frame(cls.ID, seq, payload).hex()}")
    p = tmp_path / "py_frames.txt"
    p.write_text("\n".join(lines) + "\n")
    r = subprocess.run([_c_bin(), "--check", str(p)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "checked 5000" in r.stdout
