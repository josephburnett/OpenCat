"""The Petoi IR remote: key table and the daemon's ``--remote`` key map.

The body decodes the remote (NEC, address 0x00) and relays each key as EVENT
IR_KEY with ``arg = command | 0x100`` for a held-key repeat (docs/architecture.md
§0, §3a). The command bytes come from the legacy firmware's table
(``src/infrared.h``: 0x00FFA25D is address 0x00, ~0xFF, command 0xA2, ~0x5D,
written MSB first as the IRremote library did), and so do the legacy meanings
(its K00..K62 skill tokens), kept here for reference.

``ACTIONS`` is what ``kilo run --remote`` does with a key (the owner's map,
2026-10-11); ``action_for`` turns an IR_KEY arg into one.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable

REPEAT = 0x100


@dataclass(frozen=True)
class Key:
    command: int   # NEC command byte
    label: str     # what is printed on the key
    legacy: str    # what the legacy firmware did with it


# The remote's 21 keys, row by row as on the remote (and in src/infrared.h).
KEYS: tuple[Key, ...] = (
    Key(0xA2, "CH-", "rest"),
    Key(0x62, "CH", "forward"),
    Key(0xE2, "CH+", "gyro toggle"),
    Key(0x22, "|<<", "left"),
    Key(0x02, ">>|", "stand"),
    Key(0xC2, ">||", "right"),
    Key(0xE0, "-", "pause/limp"),
    Key(0xA8, "+", "backward"),
    Key(0x90, "EQ", "calibrate"),
    Key(0x68, "0", "stepping"),
    Key(0x98, "100+", "crawl"),
    Key(0xB0, "200+", "walk"),
    Key(0x30, "1", "trot"),
    Key(0x18, "2", "sit"),
    Key(0x7A, "3", "stretch"),
    Key(0x10, "4", "hi"),
    Key(0x38, "5", "push-up"),
    Key(0x5A, "6", "pee"),
    Key(0x42, "7", "wash face"),
    Key(0x4A, "8", "butt-up"),
    Key(0x52, "9", "last skill"),
)
BY_COMMAND: dict[int, Key] = {k.command: k for k in KEYS}
BY_LABEL: dict[str, Key] = {k.label: k for k in KEYS}


def split(arg: int) -> tuple[int, bool]:
    """An IR_KEY arg -> (command byte, held-key repeat)."""
    arg &= 0xFFFF
    return arg & 0xFF, bool(arg & REPEAT)


def describe(arg: int) -> str:
    """``2 (0x18, sit)``, ``2 (0x18, sit) repeat``; ``? (0x45)`` for another remote's key."""
    cmd, rep = split(arg)
    k = BY_COMMAND.get(cmd)
    s = f"{k.label} (0x{cmd:02X}, {k.legacy})" if k else f"? (0x{cmd:02X})"
    return s + (" repeat" if rep else "")


@dataclass(frozen=True)
class Action:
    kind: str        # "posture" | "trick" | "balance_toggle" | "walk" | "gait_pattern" | "ignore"
    name: str = ""   # the posture or trick name, the walk direction, the gait pattern


# kilo run --remote. The direction keys walk (P2) with the selected pattern
# until another key, like the legacy firmware; 100+ / 200+ / 1 select the
# pattern; EQ (calibrate) and 9 (last skill) do nothing.
WALK_DIRECTIONS = ("forward", "backward", "left", "right", "march")
_IGNORE = Action("ignore")
ACTIONS: dict[str, Action] = {
    "CH-": Action("posture", "rest"),
    ">>|": Action("posture", "stand"),
    "-": Action("posture", "limp"),
    "2": Action("posture", "sit"),
    "3": Action("trick", "stretch"),
    "4": Action("trick", "hi"),
    "5": Action("trick", "push_up"),
    "6": Action("trick", "pee"),
    "7": Action("trick", "wash_face"),
    "8": Action("trick", "butt_up"),
    "CH+": Action("balance_toggle"),
    "CH": Action("walk", "forward"),
    "+": Action("walk", "backward"),
    "|<<": Action("walk", "left"),
    ">||": Action("walk", "right"),
    "0": Action("walk", "march"),
    "100+": Action("gait_pattern", "crawl"),
    "200+": Action("gait_pattern", "walk"),
    "1": Action("gait_pattern", "trot"),
    "EQ": _IGNORE,
    "9": _IGNORE,
}


def action_for(arg: int) -> Action:
    """What ``--remote`` does with an IR_KEY arg. Repeats (a held key) and
    unknown keys are ignored."""
    cmd, rep = split(arg)
    k = BY_COMMAND.get(cmd)
    if rep or k is None:
        return _IGNORE
    return ACTIONS.get(k.label, _IGNORE)


class RemoteControl:
    """Carries out ``--remote`` actions through a BodyClient (blocking: the
    daemon runs ``perform`` in its thread pool, one at a time).

    A posture or trick key first stops a trick that is playing (the body
    would otherwise put the trick's end posture over a new posture, or refuse
    the new trick BUSY); a trick key from LIMP takes STAND first (a trick
    cannot start from LIMP), as ``kilo trick play --posture stand`` would.
    Tricks go through ``BodyClient.start_trick`` (uploads when the body does
    not hold the trick yet, cached otherwise) and are not waited for.

    Walking (P2): a direction key walks with the selected pattern (default
    trot) and its default stride (``kilo.body.client.walk_args``) until
    another key; 100+ / 200+ / 1 select crawl / walk / trot and, while
    walking, re-send the walk with the new pattern. A posture or trick key
    stops the gait first (``stop_walk``: the stop goal is latched, not waited
    for; the body's TRICK owner pre-empts the legs anyway, and a posture
    refreshed at 5 Hz is what the body takes after the stop's hand-over to
    STANCE).
    """

    def __init__(self, client: Any, settle_s: float = 1.0, sleep: Callable[[float], None] = time.sleep):
        self.client = client
        self.settle_s = settle_s  # the default posture move is 0.5 s
        self.sleep = sleep
        self.pattern = "trot"
        self.direction: str | None = None  # the walk the remote started, None when not walking

    def balance_enabled(self) -> bool:
        g = self.client.link.goals().get("SET_BALANCE")
        return bool(g is not None and g.fields.get("enable"))

    def walking(self) -> bool:
        g = self.client.link.goals().get("SET_GAIT")
        return bool(g is not None and g.fields.get("run")) or self.client.state.mode == "GAIT"

    def _stop_walking(self) -> None:
        if self.walking():
            self.client.stop_walk()
        self.direction = None

    def _walk(self, direction: str) -> None:
        from kilo.body.client import walk_args

        if self.client.state.mode == "TRICK":
            self.client.stop_trick()
        self.client.walk(**walk_args(direction, self.pattern))
        self.direction = direction

    def perform(self, action: Action) -> str:
        c = self.client
        if action.kind == "walk":
            self._walk(action.name)
            return f"walk {action.name} ({self.pattern})"
        if action.kind == "gait_pattern":
            self.pattern = action.name
            g = c.link.goals().get("SET_GAIT")
            if self.direction is not None and g is not None and g.fields.get("run"):
                self._walk(self.direction)
                return f"walk {self.direction} ({self.pattern})"
            return f"pattern {self.pattern}"
        if action.kind == "posture":
            self._stop_walking()
            if c.state.mode == "TRICK":
                c.stop_trick()
            c.posture(action.name)
            return f"posture {action.name}"
        if action.kind == "trick":
            from kilo.tricks import library

            t = library.load(action.name)
            self._stop_walking()
            mode = c.state.mode
            if mode == "TRICK":
                c.stop_trick()
            elif mode == "LIMP":
                c.posture("stand")
                self.sleep(self.settle_s)
            c.start_trick(t)
            return f"trick {action.name}"
        if action.kind == "balance_toggle":
            on = not self.balance_enabled()
            c.set_balance(on)
            return f"balance {'on' if on else 'off'}"
        return action.kind
