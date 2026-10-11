"""Passive timeline: what the body does on its own, one line per change.

Used by ``kilo monitor --passive`` and ``kilo posture ... --then-watch``. It
subscribes to a (silent) SerialLink and prints, with a host timestamp in
seconds relative to ``t0`` (0.1 s resolution):

- every EVENT the body emits, as it arrives;
- each change of TELEMETRY mode, epoch, owners and flags (the flags that
  flicker by design, DIST_FRESH and OVERRUN, are left out of the diff);
- a HELLO (boot), telemetry lost / back;
- once a second, a telemetry summary line (from ``poll()``), with the gait
  phase (0..1) while the body walks (mode GAIT or the GAIT flag); EVENT
  GAIT_STOPPED and GOAL_REFUSED (SET_GAIT: reason) print like every event.

It never sends anything; the link it watches must be silent for the timeline
to say anything about the body's link watchdog.
"""

from __future__ import annotations

import sys
import threading
from typing import Any, TextIO

from kilo import ir
from kilo.link import consts
from kilo.link.serial_link import SerialLink

VOLATILE_FLAGS = frozenset({"dist_fresh", "overrun"})


def _owners(v: int) -> str:
    return f"{consts.OWNERS.get(v & 0xF, v & 0xF)}/{consts.OWNERS.get(v >> 4, v >> 4)}"


def _flag_names(v: int) -> set[str]:
    return {name for bit, name in consts.FLAG_BITS.items()
            if v >> bit & 1 and name not in VOLATILE_FLAGS}


class Timeline:
    def __init__(self, link: SerialLink, t0: float, out: TextIO | None = None,
                 summary_s: float = 1.0):
        self.link = link
        self.t0 = t0
        self.out = out or sys.stdout
        self.summary_s = summary_s
        self._lock = threading.Lock()
        self._prev: dict[str, Any] | None = None
        self._last_summary: float | None = None
        self._frames = 0
        self._was_alive: bool | None = None
        self._ever_alive = False
        self.lines: list[tuple[float, str]] = []
        tel = link.state.telemetry
        if tel is not None:  # watching an open link: start from what it already knows
            self._on_telemetry(tel)
            self._frames = 0
        link.subscribe("telemetry", self._on_telemetry)
        link.subscribe("event", self._on_event)
        link.subscribe("hello", self._on_hello)

    # ---- output ------------------------------------------------------------------
    def say(self, text: str, now: float | None = None) -> None:
        t = (self.link.clock() if now is None else now) - self.t0
        with self._lock:
            self.lines.append((t, text))
            self.out.write(f"{t:+7.1f}s  {text}\n")
            self.out.flush()

    # ---- callbacks (reader thread) --------------------------------------------------
    def _on_event(self, msg: Any) -> None:
        name = consts.EVENTS.get(msg.code, str(msg.code))
        extra = ""
        if name == "GOAL_REFUSED":
            arg = msg.arg & 0xFFFF
            extra = (f" ({self.link.messages.name_of(arg >> 8)}: "
                     f"{consts.REFUSE_REASON.get(arg & 0xFF, arg & 0xFF)})")
        elif name == "IR_KEY":
            self.say(f"EVENT IR_KEY {ir.describe(msg.arg)} tick={msg.tick} epoch={msg.epoch}")
            return
        self.say(f"EVENT {name} arg={msg.arg}{extra} tick={msg.tick} epoch={msg.epoch}")

    def _on_hello(self, msg: Any, solicited: bool) -> None:
        self.say(f"HELLO reset {consts.RESET_CAUSES.get(msg.reset_cause, msg.reset_cause)} "
                 f"build {msg.fw_build:08x} epoch {msg.epoch}"
                 + ("" if solicited else " (unsolicited)"))

    def _on_telemetry(self, msg: Any) -> None:
        with self._lock:
            self._frames += 1
        cur = {"mode": consts.MODES.get(msg.mode, f"?{msg.mode}"), "epoch": msg.epoch,
               "owners": _owners(msg.owners), "flags": _flag_names(msg.flags)}
        prev, self._prev = self._prev, cur
        if prev is None:
            self.say(f"state: mode {cur['mode']}  epoch {cur['epoch']}  owners {cur['owners']}  "
                     f"flags {','.join(sorted(cur['flags'])) or '-'}")
            return
        for k in ("mode", "epoch", "owners"):
            if cur[k] != prev[k]:
                self.say(f"{k} {prev[k]} -> {cur[k]}{_note(k, prev, cur)}")
        if cur["flags"] != prev["flags"]:
            diff = ([f"+{f}" for f in sorted(cur["flags"] - prev["flags"])]
                    + [f"-{f}" for f in sorted(prev["flags"] - cur["flags"])])
            note = "  (servos off: LIMP)" if "servos_powered" in prev["flags"] - cur["flags"] else ""
            self.say("flags " + " ".join(diff) + note)

    # ---- main loop -----------------------------------------------------------------
    def poll(self) -> None:
        """Call often (≥ 10 Hz) from the main thread: alive changes and the summary line."""
        now = self.link.clock()
        st = self.link.state
        alive = st.alive
        if alive != self._was_alive:
            if alive and not self._ever_alive:
                if self._was_alive is not None:
                    self.say("telemetry flowing", now)
            elif self._was_alive is not None:
                self.say("telemetry " + ("back" if alive else "lost (none for 300 ms)"), now)
            self._was_alive = alive
            self._ever_alive |= alive
        last = self._last_summary
        if last is not None and now - last < self.summary_s:
            return
        self._last_summary = now
        with self._lock:
            n, self._frames = self._frames, 0
        snap = st.snapshot()
        if snap["tick"] is None:
            self.say("telemetry: none yet", now)
            return
        rate = "" if last is None else f"  {n / (now - last):.0f} Hz"
        gait = (f" phase {snap['gait_phase'] / 256:.2f}" if snap.get("gait_phase") is not None
                and (snap["mode"] == "GAIT" or "gait" in snap["flags"]) else "")
        self.say(f"tel tick {snap['tick']} mode {snap['mode']}{gait} epoch {snap['epoch']} "
                 f"pitch {_f(snap['pitch_deg'])} roll {_f(snap['roll_deg'])} "
                 f"batt {snap['batt_mv']} mV{rate}", now)


def _note(key: str, prev: dict[str, Any], cur: dict[str, Any]) -> str:
    if key == "mode" and cur["mode"] == "LINK_LOST":
        return "  (watchdog stage 1: stop)"
    if key == "owners" and cur["owners"].startswith("REFLEX") and not prev["owners"].startswith("REFLEX"):
        return "  (reflex took over" + (": watchdog REST)" if cur["mode"] == "LINK_LOST" else ")")
    return ""


def _f(v: Any, fmt: str = "{:.1f}") -> str:
    return "-" if v is None else fmt.format(v)
