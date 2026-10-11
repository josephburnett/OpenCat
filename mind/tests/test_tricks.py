"""Trick model, legacy importer and the first-version library (§5, §3e)."""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P
from kilo.tricks import import_legacy, legacy, library
from kilo.tricks.model import (CANONICAL, JOINT_LIMITS, Easing, Keyframe, Posture, Trick,
                               TrickError, Trigger, pose)

REPO = Path(__file__).resolve().parents[2]
JOINTS_CPP = REPO / "body" / "src" / "core" / "joints.cpp"
FRAMES_JSON = REPO / "protocol" / "vectors" / "frames.json"

UP = (0, 0, 0, 0, 0, 0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30)


@pytest.fixture(scope="module")
def skills():
    return legacy.parse_file()


@pytest.fixture(scope="module")
def lib():
    return library.load_all()


def kf(angles=UP, **kw):
    return Keyframe(pose(angles), **kw)


# ---------------------------------------------------------------------------
# Legacy parser

SYNTHETIC = """
// a comment, and a fake array in a comment: const int8_t no[] PROGMEM = { 1, 0, 0, 1, };
const int8_t pst[] PROGMEM = {
1, -5, 12, 2,
    1,   2,   3,   0,   0,   0,   0,   0,  10,  11, -12, -13,  14,  15, -16, -17,};
const int8_t gt[] PROGMEM = {
2, 0, 3, 1,
  41,  46, -56, -24,  11,   1, -16, -13,
  39,  57, -57, -23,  11,   3, -18, -15,
};
const int8_t bh[] PROGMEM = { /* inline */
-3, 0, 0, 1,
 1, 2, 200,
    0, -20, -65,   0,   0,   0,   0,   0,  30,  30, -60, -60,  60,  60,  45,  45,	 8, 1, 0, 0,
   35, -15, -86,   0,  -3,  -3,   3,   3, -75,  43, -60, -72,  40,  53,  60,   6,	 0, 4, -2, 30,
    0,   0,   0,   0,   0,   0,   0,   0,  30,  30, -30, -30,  30,  30, -30, -30,	16, 0, 0, 0,
};
  const char* skillNameWithType[]={"pstI","gtN","bhI",};
"""


def test_parse_synthetic():
    s = legacy.parse_text(SYNTHETIC)
    assert list(s) == ["pst", "gt", "bh"]
    p, g, b = s["pst"], s["gt"], s["bh"]
    assert (p.kind, p.roll, p.pitch, p.angle_ratio, p.skill_type) == ("posture", -5, 12, 2, "I")
    assert p.angles(0)[8] == 20 and p.angles(0)[15] == -34        # ratio 2 doubles
    assert g.kind == "gait" and g.skill_type == "N" and len(g.frames) == 2
    assert g.angles(1) == (None,) * 8 + (39, 57, -57, -23, 11, 3, -18, -15)  # 8-DOF -> joints 8-15
    assert b.kind == "behaviour" and b.loop == (1, 2, 200)        # 200 stored as int8 -56
    assert b.extras == [(8, 1, 0, 0), (0, 4, -2, 30), (16, 0, 0, 0)]


def test_parser_round_trip(skills):
    for s in list(legacy.parse_text(SYNTHETIC).values()) + list(skills.values()):
        again = legacy.parse_text(s.to_c())[s.name]
        assert again.skill_type == ""                # to_c() writes the array only
        again.skill_type = s.skill_type
        assert again == s
        assert legacy.LegacySkill.from_array(s.name, s.to_array(), s.skill_type) == s


def test_parser_rejects_bad_length():
    with pytest.raises(ValueError):
        legacy.LegacySkill.from_array("x", [1, 0, 0, 1] + [0] * 15)
    with pytest.raises(ValueError):
        legacy.LegacySkill.from_array("x", [-1, 0, 0, 1, 0, 0, 0] + [0] * 16)


def test_synthetic_mapping():
    s = legacy.parse_text(SYNTHETIC)
    b = legacy.to_trick(s["bh"], skills={"up": legacy.LegacySkill("up", 1, 0, 0, 1, None, [list(UP)])})
    # frame 0 from `up`: maxDelta 75 (rb knee 45 vs -30)
    assert b.frames[0].dur == legacy.speed_to_ticks(75, 8)
    assert b.frames[0].hold == 3                       # 1 x 50 ms = 2.5 ticks -> 3
    assert b.frames[1].easing == Easing.STEP and b.frames[1].dur == 1   # speed 0 = jump
    assert b.frames[1].hold == 10
    assert b.frames[1].trigger == Trigger(-2, 30)
    assert (b.loop_start, b.loop_end, b.repeats) == (1, 2, 255)         # 200 cycles = int8 -> forever
    assert b.frames[1].angles[8] == -75 and b.frames[1].angles[4] is None
    with pytest.raises(TrickError, match="trigger"):
        b.validate()
    g = legacy.to_trick(s["gt"])
    assert (g.loop_start, g.loop_end, g.repeats) == (0, 1, 255)
    assert g.joint_mask == 0xFF00 and g.balance
    p = legacy.to_trick(s["pst"])
    assert p.expected_roll == -5 and p.expected_pitch == 12
    assert p.frames[0].angles[14] == -32


def test_speed_delay_mapping():
    assert legacy.speed_to_ticks(60, 0) == 1
    assert legacy.speed_to_ticks(0, 8) == 1
    # 60 deg at speed 8 = 2 deg/step -> 30 steps x 5.5 ms = 165 ms -> 9 ticks
    assert legacy.speed_to_ticks(60, 8) == 9
    assert legacy.delay_to_ticks(0) == 0
    assert legacy.delay_to_ticks(2) == 5
    assert legacy.delay_to_ticks(-4) == 10


def test_legacy_file_facts(skills):
    """What the legacy Nybble table holds (Appendix C5)."""
    assert len(skills) == 47
    assert all(s.angle_ratio == 1 for s in skills.values())
    assert not any(e[2] for s in skills.values() for e in s.extras)   # no IMU triggers
    counts = sorted((abs(s.period), n) for n, s in skills.items() if s.period < 0)
    assert counts[-3:] == [(10, "hds"), (11, "dg"), (22, "cmh")]


# ---------------------------------------------------------------------------
# First-version library

EXPECTED = {
    # name: (legacy, frames, loop_start, loop_end, repeats, roll, pitch, end_posture)
    "sit": ("sit", 1, 0, 0, 0, 0, -30, Posture.SIT),
    "rest": ("rest", 1, 0, 0, 0, 0, 0, Posture.REST),
    "stand": ("up", 1, 0, 0, 0, 0, 0, Posture.STAND),
    "stretch": ("str", 1, 0, 0, 0, 0, 15, Posture.STAND),
    "balance": ("balance", 1, 0, 0, 0, 0, 0, Posture.STAND),
    "hi": ("hi", 6, 2, 3, 3, 0, 0, Posture.STAND),
    "wash_face": ("wsf", 6, 2, 3, 3, 0, 0, Posture.SIT),
    "check_around": ("ck", 3, 0, 1, 2, 0, 0, Posture.STAND),
    "sniff": ("snf", 5, 2, 3, 3, 0, 0, Posture.STAND),
    "scratch": ("scrh", 6, 3, 4, 5, 0, 0, Posture.STAND),
    "pee": ("pee", 5, 2, 3, 2, 0, 0, Posture.STAND),
    "push_up": ("pu", 6, 1, 2, 3, 0, 0, Posture.STAND),
    "butt_up": ("buttUp", 1, 0, 0, 0, 0, 15, Posture.STAND),
}


def test_library_contents(lib):
    assert list(lib) == list(EXPECTED)
    assert sorted(p.stem for p in library.LIBRARY_DIR.glob("*.json")) == sorted(EXPECTED)
    for name, (leg, n, ls, le, rep, roll, pitch, end) in EXPECTED.items():
        t = lib[name]
        assert t.source["legacy_name"] == leg
        assert (len(t.frames), t.loop_start, t.loop_end, t.repeats) == (n, ls, le, rep), name
        assert (t.expected_roll, t.expected_pitch, t.end_posture) == (roll, pitch, end), name
        assert t.joint_mask == 0xFF07
        assert t.balance == (n == 1)
        assert t.notes
        t.validate()


def test_library_known_frames(lib, skills):
    hi = lib["hi"]
    # legacy rb shoulder -85 is clamped to its -80 limit
    assert hi.frames[2].angles == pose((35, -15, -86, 0, -3, -3, 3, 3, -75, 43, -80, -72, 40, 53, 60, 6))
    assert hi.frames[1].hold == legacy.delay_to_ticks(2)
    pu = lib["push_up"]
    assert pu.frames[3].angles == pose((5, 5, 0, 0, 0, 0, 0, 0, 75, -70, 55, 55, -50, 70, -75, -75))
    assert lib["stand"].frames[0].angles == pose(UP)
    assert lib["rest"].frames[0].angles == pose((-30, -80, -45, 0, -3, -3, 3, 3,
                                                  60, 60, -60, -60, -45, -45, 45, 45))
    assert library.load("wsf") == lib["wash_face"] and library.load("up") == lib["stand"]
    # every unclamped angle equals the legacy number
    for t in lib.values():
        s = skills[t.source["legacy_name"]]
        for k, f in enumerate(t.frames):
            for j in CANONICAL:
                lo, hi_ = JOINT_LIMITS[j]
                assert f.angles[j] == min(hi_, max(lo, s.angles(k)[j]))


def test_library_up_to_date():
    files = import_legacy.build()
    for path, text in files.items():
        assert path.read_text() == text, f"{path.name} is stale: run python -m kilo.tricks.import_legacy"
    assert import_legacy.main(["--check"]) == 0


def test_json_round_trip(lib):
    for t in lib.values():
        again = Trick.from_json(t.to_json())
        assert again == t and again.notes == t.notes and again.source == t.source
        assert json.loads(t.to_json())["name"] == t.name


# ---------------------------------------------------------------------------
# Model

def test_mirror_involution_and_swaps(lib):
    for t in lib.values():
        assert t.mirror().mirror() == t
    a = tuple(range(16))
    m = Keyframe(pose(a)).mirror().angles
    assert m[0] == 0 and m[1] == 1 and m[2] == -2
    assert (m[8], m[9], m[10], m[11]) == (9, 8, 11, 10)
    assert (m[12], m[13], m[14], m[15]) == (13, 12, 15, 14)
    assert m[3:8] == (None,) * 5
    t = Trick("x", (Keyframe(pose(a)),), expected_roll=7, expected_pitch=5)
    assert (t.mirror().expected_roll, t.mirror().expected_pitch) == (-7, 5)


def test_mirror_matches_legacy_rule(skills):
    """Mirror the raw legacy rows with Skill::mirror()'s exact loop and compare."""
    s = skills["hi"]
    rows = [list(r) for r in s.frames]
    for r in rows:                                   # period <= 1 branch
        r[0], r[2] = -r[0], -r[2]
        for col in range(2, 8):
            r[2 * col], r[2 * col + 1] = r[2 * col + 1], r[2 * col]
    t = legacy.to_trick(s).mirror()
    for k, r in enumerate(rows):
        for j in CANONICAL:
            lo, hi = JOINT_LIMITS[j]
            assert t.frames[k].angles[j] == min(hi, max(lo, r[j]))  # pairs share limits


def test_speed(lib):
    hi = lib["hi"]
    fast = hi.speed(2)
    assert [f.dur for f in fast.frames] == [max(1, (f.dur + 1) // 2) for f in hi.frames]
    assert hi.speed(1) == hi
    assert all(f.dur >= 1 for f in hi.speed(100).frames)
    with pytest.raises(TrickError):
        hi.speed(0.01)
    with pytest.raises(ValueError):
        hi.speed(0)


def test_validation():
    good = Trick("ok", [kf()] * 32)
    good.validate()
    with pytest.raises(TrickError, match="33 frames"):
        Trick("big", [kf()] * 33).validate()
    with pytest.raises(TrickError, match="lf_knee 121"):
        Trick("lim", [Keyframe.from_joints({"lf_knee": 121})]).validate()
    with pytest.raises(TrickError, match="head_tilt -81"):
        Trick("lim", [Keyframe.from_joints({"head_tilt": -81})]).validate()
    Trick("edge", [Keyframe.from_joints({"head_tilt": -80, "lf_knee": 120})]).validate()
    with pytest.raises(TrickError, match="not a driven joint"):
        Trick("j5", [Keyframe.from_joints({5: 0, 8: 0})]).validate()
    with pytest.raises(TrickError, match="dur 0"):
        Trick("d", [kf(dur=0)]).validate()
    with pytest.raises(TrickError, match="loop"):
        Trick("l", [kf()] * 2, loop_start=1, loop_end=2).validate()


def test_joint_limits_match_body():
    rows = re.findall(r"\{\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\}\s*,\s*//\s*(\d+)",
                      JOINTS_CPP.read_text())
    body = {int(r[5]): (int(r[3]), int(r[4])) for r in rows}
    assert len(body) == 16
    for j, lim in JOINT_LIMITS.items():
        assert body[j] == lim, j


def test_frame_encode_decode():
    f = Keyframe.from_joints({"head_pan": 10, "tail": -5.6, "lb_knee": -120},
                             dur=7, hold=3, easing=Easing.LINEAR, balance=True)
    b = f.encode()
    assert len(b) == 14
    assert b[:3] == bytes([7, 3, 0 | 4])
    assert b[3] == 10 and b[4] == 0x80 and b[5] == (256 - 6) and b[13] == 256 - 120
    g = Keyframe.decode(b)
    assert g.angles[2] == -6 and g.angles[1] is None and g.balance and g.easing == Easing.LINEAR
    assert Keyframe.decode(g.encode()) == g


def test_trick_encode_decode(lib):
    for t in lib.values():
        begin = t.begin(9)
        data = t.encode_frames()
        assert len(data) == 14 * len(t.frames)
        back = Trick.from_wire(P.TrickBegin.unpack(begin.pack()), data, name=t.name,
                               expected_roll=t.expected_roll, expected_pitch=t.expected_pitch)
        assert back == t
        msgs = t.upload_messages(9)
        assert isinstance(msgs[0], P.TrickBegin) and isinstance(msgs[-1], P.TrickCommit)
        chunks = msgs[1:-1]
        assert [m.first for m in chunks] == list(range(0, len(t.frames), 4))
        assert b"".join(k.pack() for m in chunks for k in m.frames) == data
        assert msgs[-1].crc16 == C.crc16(data)
        for m in msgs:
            type(m).unpack(m.pack())                 # the generated codec accepts them
        frames = [k for m in chunks for k in P.TrickFrames.unpack(m.pack()).frames]
        assert Trick.from_wire(begin, frames, name=t.name, expected_roll=t.expected_roll,
                               expected_pitch=t.expected_pitch) == t


def test_wire_matches_vectors():
    vec = {v["name"]: v for v in json.loads(FRAMES_JSON.read_text())["frames"]}
    v = vec["trick_frames_1"]
    w = v["fields"]["frames"][0]
    f = Keyframe.from_wire(P.Keyframe(**w))
    assert f.dur == 25 and f.easing == Easing.COSINE
    payload = bytes([v["fields"]["first"], 1]) + f.encode()
    assert payload.hex() == v["payload"]
    vb = vec["trick_begin"]
    fb = vb["fields"]
    t = Trick("v", [Keyframe.from_joints(dict.fromkeys(CANONICAL, 0), dur=1)] * fb["n_frames"],
              loop_start=fb["loop_start"], loop_end=fb["loop_end"], repeats=fb["repeats"],
              balance=bool(fb["flags"] & 1), end_posture=Posture(fb["end_posture"]))
    assert t.joint_mask == fb["joint_mask"]
    assert t.begin(fb["trick_id"]).pack().hex() == vb["payload"]
