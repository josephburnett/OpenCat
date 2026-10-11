"""The body's keyframe player timing, in Python (body/src/core/keyframe.cpp).

Used by the virtual body (to send TRICK_DONE on the same tick as the real
body) and by the mind (to know how long a trick takes, for the play()
timeout). The C and Python versions are kept in step by
mind/tests/test_crosscheck.py and body/test/test_keyframe.cpp.

Timing (doc §3e): each frame is a MOVE of ``dur`` ticks then a HOLD of
``hold`` ticks; at speed ``s`` (Q6, 64 = 1x) a phase of n ticks takes
ceil(n * 64 / s) ticks. After ``loop_end`` the player goes back to
``loop_start`` until that section has been played ``repeats`` times in total
(0 and 1 = once, 255 = until stopped). TRICK_DONE comes on the tick after
the last phase.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

SPEED_UNITY = 64
FOREVER = 255
TICK_S = 0.02


def phase_ticks(n: int, speed_q6: int = SPEED_UNITY) -> int:
    """Ticks a phase of ``n`` ticks takes at ``speed_q6``."""
    return (n * 64 + speed_q6 - 1) // speed_q6


def play_order(n: int, loop_start: int, loop_end: int, repeats: int, max_plays: int | None = None) -> list[int]:
    """Frame indices in play order (``repeats`` 255 is cut at ``max_plays``)."""
    plays = max(repeats, 1) if repeats != FOREVER else (max_plays or 1)
    order = list(range(0, loop_end + 1))
    for _ in range(plays - 1):
        order += list(range(loop_start, loop_end + 1))
    return order + list(range(loop_end + 1, n))


def total_ticks(durs: Sequence[int], holds: Sequence[int], loop_start: int, loop_end: int, repeats: int,
                speed_q6: int = SPEED_UNITY) -> int | None:
    """Ticks from TRICK_PLAY to the last phase's end (None: until stopped).
    TRICK_DONE arrives one tick later."""
    if repeats == FOREVER:
        return None
    return sum(phase_ticks(durs[f], speed_q6) + phase_ticks(holds[f], speed_q6)
               for f in play_order(len(durs), loop_start, loop_end, repeats))


@dataclass
class Player:
    """Tick-exact mirror of kf_step(): call ``step()`` once per body tick."""
    durs: Sequence[int]
    holds: Sequence[int]
    loop_start: int = 0
    loop_end: int = 0
    repeats: int = 0
    speed: int = SPEED_UNITY
    running: bool = field(default=True, init=False)
    frame: int = field(default=0, init=False)
    hold: bool = field(default=False, init=False)
    plays: int = field(default=0, init=False)
    pos: int = field(default=0, init=False)

    def _len(self) -> int:
        return (self.holds[self.frame] if self.hold else self.durs[self.frame]) << 6

    def step(self) -> bool:
        """Advance one tick. True on the tick the trick finishes."""
        if not self.running:
            return False
        if self.pos >= self._len():
            self.pos = 0
            if not self.hold and self.holds[self.frame]:
                self.hold = True
            else:
                self.hold = False
                again = False
                if self.frame == self.loop_end:
                    self.plays = min(self.plays + 1, 254)
                    again = self.repeats == FOREVER or self.plays < self.repeats
                if again:
                    self.frame = self.loop_start
                else:
                    self.frame += 1
                    if self.frame >= len(self.durs):
                        self.running = False
                        return True
        self.pos += self.speed
        return False
