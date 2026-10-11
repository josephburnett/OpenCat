"""The utility root (docs/architecture.md §5): safety branches, then a scored
choice among the top-level behaviours, with hysteresis.

Each tick:

1. **Safety** branches are checked in order (fallen, low battery / wind-down,
   shutdown requested; see library.py). The first whose condition holds is
   ticked; if it does not FAIL it wins, and the running behaviour is halted.
2. **Scores.** ``score[b] = sum(weight[b][term] * term) + noise[b]``. Terms are
   the drives (energy, curiosity, sociability, arousal), their complements
   (fatigue = 1 - energy, calm = 1 - arousal), percepts as 0/1 (face, pet,
   near, approaching, alone), ``base`` = 1, and products written ``a*b``. ``noise[b]`` is uniform in
   ±``selector.noise``, redrawn every ``noise_period_s`` from the seeded RNG.
3. **Hysteresis.** The running behaviour keeps going unless a challenger
   scores at least ``margin`` more *and* the current one has run for
   ``min_dwell_s`` (or the behaviour's own longer ``min_dwell``, e.g. nap;
   ``urgent`` behaviours skip the dwell). When the current
   behaviour has just finished, the best one is taken freely.
4. The chosen behaviour is ticked; if it FAILS (a cooldown, a missing
   percept), the current one, then the rest by score, are tried in the same
   tick.

``last`` holds the scores, terms, choice and reason for the log.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .tree import FAILURE, RUNNING, SUCCESS, Context, Node, Status


@dataclass
class Behaviour:
    name: str
    tree: Node
    activity: str = "idle"  # drives.ACTIVITIES
    min_dwell: float = 0.0  # a longer dwell for this behaviour (nap)


@dataclass
class SafetyBranch:
    name: str
    condition: Callable[[Context], bool]
    tree: Node
    activity: str = "rest"


def terms(ctx: Context) -> dict[str, float]:
    d = ctx.drives
    p = ctx.percepts
    face = 1.0 if p.get("face") else 0.0
    pet = 1.0 if p.get("pet") else 0.0
    return {
        "base": 1.0,
        "energy": d.energy,
        "fatigue": 1.0 - d.energy,
        "curiosity": d.curiosity,
        "sociability": d.sociability,
        "arousal": d.arousal,
        "calm": 1.0 - d.arousal,
        "face": face,
        "pet": pet,
        "near": 1.0 if p.get("near") else 0.0,
        "approaching": 1.0 if p.get("approaching") else 0.0,
        "alone": 1.0 if not (face or pet) else 0.0,
    }


def _term(t: dict[str, float], term: str) -> float:
    if "*" not in term:
        return t[term]
    v = 1.0
    for f in term.split("*"):
        v *= t[f]
    return v


@dataclass
class Choice:
    chosen: str | None = None
    reason: str = ""
    status: Status | None = None
    scores: dict[str, float] = field(default_factory=dict)
    activity: str = "idle"


class UtilitySelector(Node):
    def __init__(self, behaviours: list[Behaviour], safety: list[SafetyBranch],
                 cfg: dict[str, Any], name: str = "root"):
        super().__init__(name)
        self.behaviours = {b.name: b for b in behaviours}
        self.safety = list(safety)
        self.weights = cfg["weights"]
        sel = cfg["selector"]
        self.margin = sel["margin"]
        self.min_dwell = sel["min_dwell_s"]
        self.noise_amp = sel["noise"]
        self.noise_period = sel["noise_period_s"]
        self.urgent = set(sel["urgent"])
        self.current: str | None = None  # behaviour or safety branch name
        self.since = 0.0
        self.finished = True
        self.noise: dict[str, float] = {b: 0.0 for b in self.behaviours}
        self._noise_at: float | None = None
        self.last = Choice()
        self.switches = 0

    # ---- scoring ---------------------------------------------------------------
    def _redraw_noise(self, ctx: Context) -> None:
        if self._noise_at is None or ctx.now - self._noise_at >= self.noise_period:
            self._noise_at = ctx.now
            self.noise = {b: ctx.rng.uniform(-self.noise_amp, self.noise_amp) if self.noise_amp else 0.0
                          for b in self.behaviours}

    def score(self, ctx: Context) -> dict[str, float]:
        self._redraw_noise(ctx)
        t = terms(ctx)
        return {b: sum(w * _term(t, term) for term, w in self.weights[b].items()) + self.noise[b]
                for b in self.behaviours}

    # ---- ticking ---------------------------------------------------------------
    def _switch(self, ctx: Context, name: str) -> None:
        if name != self.current:
            prev = self.current
            if prev is not None:
                self._node(prev).halt()
            self.current = name
            self.since = ctx.now
            self.switches += 1

    def _node(self, name: str) -> Node:
        if name in self.behaviours:
            return self.behaviours[name].tree
        for s in self.safety:
            if s.name == name:
                return s.tree
        raise KeyError(name)

    def _tick(self, ctx: Context) -> Status:
        for s in self.safety:
            try:
                hit = s.condition(ctx)
            except Exception as e:
                ctx.errors.append((s.name, repr(e)))
                hit = False
            if not hit:
                continue
            ctx._stack.append(s.name)
            try:
                st = s.tree.tick(ctx)
            finally:
                ctx._stack.pop()
            if st is FAILURE:
                if self.current == s.name:
                    s.tree.halt()
                    self.current, self.finished = None, True
                continue
            self._switch(ctx, s.name)
            self.finished = st is SUCCESS
            self.last = Choice(s.name, "safety", st, {}, s.activity)
            return st

        scores = self.score(ctx)
        order = sorted(scores, key=lambda b: (-scores[b], b))
        best = order[0]
        cur = self.current if self.current in self.behaviours else None
        if cur is None or self.finished:
            first, reason = best, "free"
        elif best == cur:
            first, reason = cur, "best"
        elif scores[best] >= scores[cur] + self.margin:
            dwell = max(self.min_dwell, self.behaviours[cur].min_dwell)
            if ctx.now - self.since >= dwell:
                first, reason = best, "switch"
            elif best in self.urgent:
                first, reason = best, "urgent"
            else:
                first, reason = cur, "dwell"
        else:
            first, reason = cur, "margin"
        candidates = [first]
        if cur is not None and cur != first and not self.finished:
            candidates.append(cur)
        candidates += [b for b in order if b not in candidates]

        for name in candidates:
            b = self.behaviours[name]
            if name != first:
                reason = f"fallback({first})" if not reason.startswith("fallback") else reason
            ctx._stack.append(name)
            try:
                st = b.tree.tick(ctx)
            finally:
                ctx._stack.pop()
            if st is FAILURE:
                b.tree.halt()
                if name == self.current:
                    self.finished = True
                continue
            self._switch(ctx, name)
            self.finished = st is SUCCESS
            self.last = Choice(name, reason, st, scores, b.activity)
            return st if st is RUNNING else RUNNING  # the root keeps living
        self.last = Choice(None, "none", FAILURE, scores, "idle")
        return FAILURE

    def halt(self) -> None:
        if self.current is not None:
            self._node(self.current).halt()
        self.current, self.finished = None, True
        super().halt()
