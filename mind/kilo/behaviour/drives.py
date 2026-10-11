"""The four drives (docs/architecture.md §5): energy, curiosity, sociability,
arousal, each in [0, 1].

- energy drains while active (and slowly while awake and still), recovers
  while resting, and is capped by the battery: at ``battery_full_pct`` or more
  no cap, at ``battery_empty_pct`` or less a cap of 0, linear between.
- curiosity grows while not exploring and is satisfied by exploring.
- sociability grows while alone and drops while a face is seen or Kilo is
  petted.
- arousal spikes to a level on a startle or fast approach, then decays
  exponentially with ``half_life_s``.

``Drives.update(dt, inputs, rng)`` is deterministic given ``dt``, the inputs
and the RNG (which is only drawn from when ``drives.noise`` > 0). Rates are
per second, from the personality file.
"""

from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass
from typing import Any

# How the current behaviour spends energy and satisfies drives.
ACTIVITIES = ("rest", "idle", "active", "explore", "social")


@dataclass
class DriveInputs:
    activity: str = "idle"          # one of ACTIVITIES (from the running behaviour)
    moving: bool = False            # the body reports gait/trick/recovering
    battery_pct: float | None = None
    social: bool = False            # a face is seen or Kilo is petted
    startle: float = 0.0            # 0..1 startle strength this tick (a spike level)
    approaching: bool = False       # fast approach on the ultrasonic


@dataclass
class Drives:
    energy: float = 0.8
    curiosity: float = 0.2
    sociability: float = 0.3
    arousal: float = 0.0

    @classmethod
    def from_personality(cls, cfg: dict[str, Any]) -> "Drives":
        return cls(**cfg["drives"]["initial"])

    def as_dict(self, ndigits: int = 4) -> dict[str, float]:
        return {k: round(v, ndigits) for k, v in asdict(self).items()}

    def battery_cap(self, cfg: dict[str, Any], battery_pct: float | None) -> float:
        if battery_pct is None:
            return 1.0
        e = cfg["drives"]["energy"]
        lo, hi = e["battery_empty_pct"], e["battery_full_pct"]
        return _clamp((battery_pct - lo) / (hi - lo))

    def update(self, dt: float, inp: DriveInputs, cfg: dict[str, Any],
               rng: random.Random | None = None) -> None:
        if dt <= 0:
            return
        d = cfg["drives"]
        act = inp.activity
        # energy
        e = d["energy"]
        if act == "rest" and not inp.moving:
            self.energy += e["recover_rest"] * dt
        elif inp.moving or act in ("active", "explore", "social"):
            self.energy -= e["drain_active"] * dt
        else:
            self.energy -= e["drain_idle"] * dt
        # curiosity
        c = d["curiosity"]
        if act == "explore":
            self.curiosity -= c["satisfy"] * dt
        elif act != "rest":
            self.curiosity += c["growth"] * dt
        else:
            self.curiosity += 0.5 * c["growth"] * dt  # dreams of elsewhere, slowly
        # sociability
        s = d["sociability"]
        if inp.social:
            self.sociability -= s["satisfy"] * dt
        else:
            self.sociability += s["growth"] * dt
        # arousal: decay, then spikes
        a = d["arousal"]
        self.arousal *= math.exp(-math.log(2) * dt / a["half_life_s"])
        if inp.startle > 0:
            self.arousal = max(self.arousal, min(1.0, inp.startle) * a["startle"])
        if inp.approaching:
            self.arousal = max(self.arousal, a["approach"])
        # noise
        if d["noise"] > 0 and rng is not None:
            k = d["noise"] * math.sqrt(dt)
            self.energy += rng.gauss(0, k)
            self.curiosity += rng.gauss(0, k)
            self.sociability += rng.gauss(0, k)
        self.energy = _clamp(min(self.energy, self.battery_cap(cfg, inp.battery_pct)))
        self.curiosity = _clamp(self.curiosity)
        self.sociability = _clamp(self.sociability)
        self.arousal = _clamp(self.arousal)


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return lo if v < lo else hi if v > hi else v
