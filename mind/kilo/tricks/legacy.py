"""Parser for the legacy OpenCat skill tables (src/InstinctNybble.h).

Array layout (src/skill.h dataLen()/formatSkill(), all int8):

    header  period, roll, pitch, angle_ratio            (4 B)
            + loop_start, loop_end, loop_cycles         (3 B more if period < 0)
    frames  |period| rows of
              period == 1  posture    16 angles (joints 0-15)
              period  > 1  gait        8 angles (joints 8-15 only)
              period  < 0  behaviour  16 angles + speed, delay, trigger axis, trigger angle

- roll/pitch are expectedRollPitch[0]/[1] (the header's roll comes first;
  Skill::info() mislabels them "pitch,roll").
- angle_ratio 2 means every stored angle is half the real one.
- Loop bytes are read as unsigned (`byte loopCycle[3]`).
- speed: transform() is called with speedRatio = speed / 4.0 and moves in
  round(maxDelta / speedRatio) steps of one PCA write per joint each, with
  cosine easing; speed <= 0 jumps straight to the frame.
- delay: delay(|delay| * 50) ms after the frame arrives.
- trigger axis != 0: block until ypr[|axis|] crosses `angle` in the direction
  of sign(axis).
- Behaviour loop: frames loop_start..loop_end are played loop_cycles times in
  total (0 and 1 = once; >= 129 wraps int8 negative = forever). A loop_end of
  0 never loops (`c != 0` in perform()).
- After a behaviour the last frame is held as a posture.

Mapping onto Kilo keyframes (50 Hz tick, 20 ms) is in `to_trick()` and in
mind/README.md "Tricks".
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from .model import (CANONICAL, JOINT_LIMITS, JOINT_NAMES, JOINT_SLOTS, REPEAT_FOREVER,
                    TICK_MS, UNUSED, Easing, Keyframe, Posture, Trick, Trigger, _round)

DOF = 16
WALKING_DOF = 8

# Duration of one legacy transform() step: 16 calibratedPWM() calls, each a
# few EEPROM reads, float maths and one PCA9685 write at 500 kHz I2C. ≈5-6 ms
# (est.); to be calibrated by filming both firmwares doing `hi` (§5).
LEGACY_STEP_MS = 5.5
# Legacy delay byte unit: delay(|delay| * 50) ms.
LEGACY_DELAY_MS = 50
# Legacy postures move from wherever the robot is at 2 deg/step; the start is
# unknown, so a posture trick uses the body's default posture transition.
POSTURE_DUR_TICKS = 25   # spec default_transition_ticks
# Gait frames: legacy writes one joint per loop() pass, 12 passes a frame
# (head + 8 legs), ≈ one 20 ms tick at Kilo's rate (est.).
GAIT_FRAME_TICKS = 1
# The pose a behaviour is assumed to start from when sizing frame 0's dur.
START_POSTURE = "up"

DEFAULT_PATH = Path(__file__).resolve().parents[3] / "src" / "InstinctNybble.h"

POSTURE_TABLES = (("up", Posture.STAND), ("sit", Posture.SIT), ("rest", Posture.REST))


@dataclass
class LegacySkill:
    """One raw skill array, values exactly as stored."""
    name: str
    period: int
    roll: int
    pitch: int
    angle_ratio: int = 1
    loop: tuple | None = None                       # (start, end, cycles), behaviours only
    frames: list = field(default_factory=list)      # stored angles, 16 or 8 per row
    extras: list = field(default_factory=list)      # (speed, delay, trig_axis, trig_angle) per row
    skill_type: str = ""                            # 'I' instinct / 'N' newbility, from skillNameWithType

    @property
    def kind(self) -> str:
        return "posture" if self.period == 1 else ("gait" if self.period > 1 else "behaviour")

    @property
    def frame_size(self) -> int:
        return DOF if self.period == 1 else (WALKING_DOF if self.period > 1 else DOF + 4)

    def angles(self, k: int) -> tuple:
        """Real angles of row k in the 16 joint slots (None = not in the row)."""
        row = [v * self.angle_ratio for v in self.frames[k]]
        if self.period > 1:
            return (None,) * (DOF - WALKING_DOF) + tuple(row)
        return tuple(row)

    @classmethod
    def from_array(cls, name: str, data: list[int], skill_type: str = "") -> "LegacySkill":
        data = [int(v) for v in data]
        if not data:
            raise ValueError(f"{name}: empty array")
        period = data[0]
        if period == 0:
            raise ValueError(f"{name}: period 0")
        hdr = 4 if period > 0 else 7
        size = DOF if period == 1 else (WALKING_DOF if period > 1 else DOF + 4)
        want = hdr + abs(period) * size
        if len(data) != want:
            raise ValueError(f"{name}: {len(data)} values, period {period} needs {want}")
        s = cls(name=name, period=period, roll=data[1], pitch=data[2], angle_ratio=data[3],
                skill_type=skill_type)
        if period < 0:
            s.loop = tuple(v & 0xFF for v in data[4:7])
        for k in range(abs(period)):
            row = data[hdr + k * size: hdr + (k + 1) * size]
            if period < 0:
                s.frames.append(row[:DOF])
                s.extras.append(tuple(row[DOF:]))
            else:
                s.frames.append(row)
        return s

    def to_array(self) -> list[int]:
        out = [self.period, self.roll, self.pitch, self.angle_ratio]
        if self.period < 0:
            out += [v if v < 128 else v - 256 for v in self.loop]
        for k, row in enumerate(self.frames):
            out += list(row)
            if self.period < 0:
                out += list(self.extras[k])
        return out

    def to_c(self) -> str:
        """The array as a C definition in the InstinctNybble.h style."""
        a = self.to_array()
        hdr = 4 if self.period > 0 else 7
        lines = [f"const int8_t {self.name}[] PROGMEM = {{ ", ", ".join(map(str, a[:4])) + ","]
        if self.period < 0:
            lines.append(" " + ", ".join(map(str, a[4:7])) + ", ")
        size = self.frame_size
        for k in range(abs(self.period)):
            row = a[hdr + k * size: hdr + (k + 1) * size]
            lines.append("  " + ", ".join(f"{v:3d}" for v in row) + ",")
        lines.append("};")
        return "\n".join(lines) + "\n"


_ARRAY_RE = re.compile(r"const\s+int8_t\s+(\w+)\s*\[\s*\]\s*PROGMEM\s*=\s*\{(.*?)\}\s*;", re.S)
_NAMES_RE = re.compile(r"skillNameWithType\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;", re.S)


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def parse_text(text: str) -> dict[str, LegacySkill]:
    """All skill arrays in a legacy Instinct*.h text, by array name, in file order."""
    text = _strip_comments(text)
    types: dict[str, str] = {}
    m = _NAMES_RE.search(text)
    if m:
        for tok in re.findall(r'"(\w+)"', m.group(1)):
            types[tok[:-1]] = tok[-1]
    out: dict[str, LegacySkill] = {}
    for m in _ARRAY_RE.finditer(text):
        name = m.group(1)
        vals = [int(v) for v in m.group(2).replace("\n", " ").split(",") if v.strip()]
        out[name] = LegacySkill.from_array(name, vals, types.get(name, ""))
    return out


def parse_file(path: str | Path = DEFAULT_PATH) -> dict[str, LegacySkill]:
    return parse_text(Path(path).read_text())


# ---------------------------------------------------------------------------
# Mapping to Kilo tricks

def speed_to_ticks(max_delta: float, speed: int) -> int:
    """Legacy transform() at speedRatio speed/4 → keyframe dur in ticks:
    steps = round(maxDelta / (speed/4)); dur = ceil(steps · LEGACY_STEP_MS / 20).
    speed <= 0 (jump) → 1."""
    if speed <= 0:
        return 1
    steps = _round(max_delta / (speed / 4.0))
    return max(1, math.ceil(steps * LEGACY_STEP_MS / TICK_MS))


def delay_to_ticks(delay: int) -> int:
    """Legacy delay(|delay|·50 ms) → hold ticks, rounded half up."""
    return _round(abs(delay) * LEGACY_DELAY_MS / TICK_MS)


def _max_delta(frm: tuple, to: tuple) -> float:
    d = 0.0
    for j in CANONICAL:
        if to[j] is not None and frm[j] is not None:
            d = max(d, abs(to[j] - frm[j]))
    return d


def _nearest_posture(angles: tuple, tables: dict) -> tuple[Posture, float]:
    best, best_d = Posture.STAND, math.inf
    for name, posture in POSTURE_TABLES:
        ref = tables.get(name)
        if ref is None:
            continue
        d = max(abs(angles[j] - ref[j]) for j in range(8, 16))
        if d < best_d:              # ties keep the earlier (STAND first)
            best, best_d = posture, d
    return best, best_d


def to_trick(skill: LegacySkill, name: str | None = None,
             skills: dict[str, LegacySkill] | None = None,
             source_file: str = "src/InstinctNybble.h") -> Trick:
    """Map a legacy skill onto a Kilo Trick. `skills` (the whole file) supplies
    the start pose (`up`) and the posture tables used for end_posture."""
    skills = skills or {}
    notes: list[str] = []
    tables = {n: skills[n].angles(0) for n, _ in POSTURE_TABLES if n in skills}
    start = tables.get(START_POSTURE) or (None,) * JOINT_SLOTS
    kind = skill.kind

    dropped = sorted({j for k in range(len(skill.frames)) for j, v in enumerate(skill.angles(k))
                      if j in UNUSED and v})
    if dropped:
        notes.append(f"dropped nonzero values on unused joints {dropped}")

    frames: list[Keyframe] = []
    clamped: set = set()
    prev = start
    for k in range(len(skill.frames)):
        raw = skill.angles(k)
        a: list = [None] * JOINT_SLOTS
        for j in CANONICAL:
            v = raw[j]
            if v is None:
                continue
            lo, hi = JOINT_LIMITS[j]
            if not lo <= v <= hi:
                clamped.add(JOINT_NAMES[j])
                v = min(hi, max(lo, v))
            a[j] = v
        a = tuple(a)
        if kind == "posture":
            kf = Keyframe(a, dur=POSTURE_DUR_TICKS, hold=0, easing=Easing.COSINE, balance=True)
        elif kind == "gait":
            kf = Keyframe(a, dur=GAIT_FRAME_TICKS, hold=0, easing=Easing.LINEAR, balance=True)
        else:
            speed, delay, axis, angle = skill.extras[k]
            dur = speed_to_ticks(_max_delta(prev, a), speed)
            hold = delay_to_ticks(delay)
            if dur > 255 or hold > 255:
                notes.append(f"frame {k}: dur {dur} / hold {hold} clipped to 255")
                dur, hold = min(dur, 255), min(hold, 255)
            kf = Keyframe(a, dur=dur, hold=hold,
                          easing=Easing.STEP if speed <= 0 else Easing.COSINE,
                          trigger=Trigger(axis, angle) if axis else None)
        frames.append(kf)
        prev = tuple(a[j] if a[j] is not None else prev[j] for j in range(JOINT_SLOTS))
    if clamped:
        notes.append(f"clamped to joint limits (legacy clamps too): {sorted(clamped)}")

    if kind == "posture":
        loop_start = loop_end = repeats = 0
        notes.append(f"posture: dur {POSTURE_DUR_TICKS} ticks (legacy moves from the current pose "
                     "at 2 deg/step), balance on (legacy gyro balance applies to postures)")
    elif kind == "gait":
        loop_start, loop_end, repeats = 0, len(frames) - 1, REPEAT_FOREVER
        notes.append(f"gait: {GAIT_FRAME_TICKS} tick/frame linear (est.), loops until stopped, "
                     "legs only (head bob/wag is the body's)")
    else:
        ls, le, cycles = skill.loop
        if le == 0 or cycles < 2:
            loop_start = loop_end = repeats = 0
        else:
            loop_start, loop_end = ls, le
            repeats = REPEAT_FOREVER if cycles >= 129 else cycles  # int8 wrap
        notes.append(f"behaviour: dur = ceil(round(maxDelta/(speed/4)) * {LEGACY_STEP_MS} ms / 20 ms), "
                     f"frame 0 from `{START_POSTURE}`; hold = round(|delay| * 50 ms / 20 ms); "
                     f"loop {list(skill.loop)} -> start {loop_start} end {loop_end} repeats {repeats}; "
                     "balance off (legacy perform() never balances behaviours)")
        if any(f.trigger for f in frames):
            notes.append("uses IMU-trigger columns: not supported by the body")

    # end_posture: what IDLE holds afterwards. Legacy holds the last frame;
    # Kilo returns to a built-in posture. Behaviours take the nearest one by
    # leg angles; posture tricks only their own (sit, rest, up), else STAND.
    end, d = Posture.STAND, None
    last = skill.angles(len(skill.frames) - 1)
    if kind != "gait":
        end, d = _nearest_posture(last, tables)
        if kind == "posture" and d != 0:
            end, d = Posture.STAND, None
    if d is None:
        notes.append(f"end_posture {end.name} (default)")
    else:
        notes.append(f"end_posture {end.name} (nearest built-in posture, max leg diff {d:g} deg)")

    fall_ok = abs(skill.roll) > 45 or abs(skill.pitch) > 45
    if fall_ok:
        notes.append("FALL_OK: expected tilt > 45 deg")

    return Trick(
        name=name or skill.name,
        frames=tuple(frames),
        loop_start=loop_start, loop_end=loop_end, repeats=repeats,
        expected_roll=skill.roll, expected_pitch=skill.pitch,
        balance=kind != "behaviour", fall_ok=fall_ok, end_posture=end,
        source={"file": source_file, "legacy_name": skill.name, "kind": kind,
                "period": skill.period, "angle_ratio": skill.angle_ratio,
                **({"loop": list(skill.loop)} if skill.loop else {})},
        notes=tuple(notes),
    )
