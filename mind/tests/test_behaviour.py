"""Behaviour layer: engine semantics, drives, the utility selector, personality
loading, the runner end to end with a fake body and a fast clock, and the
runner against the virtual body."""

from __future__ import annotations

import asyncio
import copy
import json
import random

import pytest

from kilo.behaviour import personality as pers
from kilo.behaviour.adapter import BodyAdapter
from kilo.behaviour.drives import DriveInputs, Drives
from kilo.behaviour.library import build_root
from kilo.behaviour.runner import Runner
from kilo.behaviour.selector import Behaviour, SafetyBranch, UtilitySelector
from kilo.behaviour.tree import (FAILURE, RUNNING, SUCCESS, Action, Chance, Condition, Context,
                                 Cooldown, ForceSuccess, Inverter, Node, Parallel, Repeat, Selector,
                                 Sequence, Timeout, Wait, tick_root)


class Clock:
    def __init__(self, t: float = 100.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt

    async def sleep(self, dt: float) -> None:
        self.t += dt
        await asyncio.sleep(0)


class Probe(Node):
    """A leaf returning a scripted status, counting ticks and halts."""

    def __init__(self, name: str, status=RUNNING):
        super().__init__(name)
        self.result = status
        self.ticks = 0
        self.halts = 0

    def _tick(self, ctx):
        self.ticks += 1
        ctx._leaf(self.result)
        return self.result

    def halt(self):
        self.halts += 1
        super().halt()


def make_ctx(clock=None, **kw) -> Context:
    return Context(clock=clock or Clock(), rng=random.Random(1), **kw)


# ---- engine ------------------------------------------------------------------------------

def test_selector_rechecks_and_preempts():
    flag = {"on": False}
    high = Probe("high")
    low = Probe("low")
    root = Selector("sel", [Sequence("guarded", [Condition("flag", lambda c: flag["on"]), high], memory=False),
                            low])
    ctx = make_ctx()
    assert tick_root(root, ctx) is RUNNING and ctx.path == ["sel", "low"]
    assert tick_root(root, ctx) is RUNNING and low.ticks == 2 and high.ticks == 0
    flag["on"] = True  # re-checked every tick: the higher branch preempts
    assert tick_root(root, ctx) is RUNNING
    assert ctx.path == ["sel", "guarded", "high"] and low.halts == 1 and high.ticks == 1
    flag["on"] = False  # the reactive guard fails: high is halted, low resumes
    assert tick_root(root, ctx) is RUNNING and ctx.path == ["sel", "low"]
    assert high.halts == 1 and root.children[0].idx == 0
    # a sequence with memory does not re-check its guard once past it
    flag["on"] = True
    mem = Sequence("mem", [Condition("flag", lambda c: flag["on"]), Probe("act")])
    assert tick_root(mem, ctx) is RUNNING
    flag["on"] = False
    assert tick_root(mem, ctx) is RUNNING and ctx.path == ["mem", "act"]


def test_selector_all_fail_and_first_success():
    ctx = make_ctx()
    assert tick_root(Selector("s", [Probe("a", FAILURE), Probe("b", FAILURE)]), ctx) is FAILURE
    b = Probe("b", SUCCESS)
    c = Probe("c")
    assert tick_root(Selector("s", [Probe("a", FAILURE), b, c]), ctx) is SUCCESS
    assert c.ticks == 0 and ctx.path == ["s", "b"]


def test_sequence_has_memory():
    calls = []
    seq = Sequence("seq", [Action("once", lambda c: calls.append(c.now)), Wait(1.0), Probe("end", SUCCESS)])
    clock = Clock()
    ctx = make_ctx(clock)
    assert tick_root(seq, ctx) is RUNNING
    clock.advance(0.5)
    assert tick_root(seq, ctx) is RUNNING
    assert len(calls) == 1 and ctx.path == ["seq", "Wait(1s)"]  # resumed at the running child
    clock.advance(0.5)
    assert tick_root(seq, ctx) is SUCCESS and len(calls) == 1
    assert tick_root(seq, ctx) is RUNNING and len(calls) == 2  # restarts after completing
    # a failure resets it
    fail = Sequence("f", [Probe("ok", SUCCESS), Probe("bad", FAILURE), Probe("never")])
    assert tick_root(fail, ctx) is FAILURE and fail.idx == 0 and fail.children[2].ticks == 0


def test_parallel_policies():
    ctx = make_ctx()
    a, b = Probe("a", SUCCESS), Probe("b")
    par = Parallel("all", [a, b], "all")
    assert tick_root(par, ctx) is RUNNING
    assert tick_root(par, ctx) is RUNNING and a.ticks == 1 and b.ticks == 2  # finished children rest
    b.result = SUCCESS
    assert tick_root(par, ctx) is SUCCESS
    # all: the first failure fails and halts the rest
    c, d = Probe("c", FAILURE), Probe("d")
    assert tick_root(Parallel("all", [d, c], "all"), ctx) is FAILURE and d.halts == 1
    # any: the first success wins and halts the rest
    e, f = Probe("e"), Probe("f", SUCCESS)
    assert tick_root(Parallel("any", [e, f], "any"), ctx) is SUCCESS and e.halts == 1
    g, h = Probe("g", FAILURE), Probe("h")
    par = Parallel("any", [g, h], "any")
    assert tick_root(par, ctx) is RUNNING
    h.result = FAILURE
    assert tick_root(par, ctx) is FAILURE
    with pytest.raises(ValueError):
        Parallel("x", [], "most")


def test_cooldown():
    clock = Clock()
    ctx = make_ctx(clock)
    p = Probe("p", SUCCESS)
    cd = Cooldown(5.0, p)
    assert tick_root(cd, ctx) is SUCCESS
    clock.advance(4.9)
    assert tick_root(cd, ctx) is FAILURE and p.ticks == 1
    clock.advance(0.1)
    assert tick_root(cd, ctx) is SUCCESS and p.ticks == 2
    # running is not cooled; finishing (even by failing) starts it
    p.result = RUNNING
    clock.advance(10)
    assert tick_root(cd, ctx) is RUNNING and tick_root(cd, ctx) is RUNNING
    p.result = FAILURE
    assert tick_root(cd, ctx) is FAILURE
    p.result = SUCCESS
    clock.advance(1)
    assert tick_root(cd, ctx) is FAILURE and p.ticks == 5
    # abandoning a running child also counts as a use
    clock.advance(10)
    p.result = RUNNING
    assert tick_root(cd, ctx) is RUNNING
    cd.halt()
    assert p.halts >= 1 and tick_root(cd, ctx) is FAILURE


def test_timeout():
    clock = Clock()
    ctx = make_ctx(clock)
    p = Probe("p")
    to = Timeout(2.0, p)
    assert tick_root(to, ctx) is RUNNING
    clock.advance(1.9)
    assert tick_root(to, ctx) is RUNNING
    clock.advance(0.1)
    assert tick_root(to, ctx) is FAILURE and p.halts == 1
    assert tick_root(to, ctx) is RUNNING  # a new run, a new deadline
    p.result = SUCCESS
    clock.advance(1.0)
    assert tick_root(to, ctx) is SUCCESS


def test_chance_is_seeded_and_sticky():
    def run(seed):
        ctx = Context(clock=Clock(), rng=random.Random(seed))
        ch = Chance(0.3, Probe("p", SUCCESS))
        return [tick_root(ch, ctx) is SUCCESS for _ in range(2000)]

    a = run(7)
    assert a == run(7) and a != run(8)
    assert 0.25 < sum(a) / len(a) < 0.35
    # once it passes, it stays with the running child without re-rolling
    ctx = Context(clock=Clock(), rng=random.Random(0))
    p = Probe("p")
    ch = Chance(0.5, p)
    while tick_root(ch, ctx) is FAILURE:
        pass
    state = ctx.rng.getstate()
    for _ in range(5):
        assert tick_root(ch, ctx) is RUNNING
    assert ctx.rng.getstate() == state
    with pytest.raises(ValueError):
        Chance(1.5, p)


def test_inverter_forcesuccess_repeat():
    ctx = make_ctx()
    assert tick_root(Inverter(Probe("a", SUCCESS)), ctx) is FAILURE
    assert tick_root(Inverter(Probe("a", FAILURE)), ctx) is SUCCESS
    assert tick_root(Inverter(Probe("a")), ctx) is RUNNING
    assert tick_root(ForceSuccess(Probe("a", FAILURE)), ctx) is SUCCESS
    p = Probe("p", SUCCESS)
    r = Repeat(p, times=3)
    assert [tick_root(r, ctx) for _ in range(4)] == [RUNNING, RUNNING, SUCCESS, RUNNING]
    assert tick_root(Repeat(Probe("x", FAILURE)), ctx) is FAILURE


def test_action_return_values_and_errors():
    ctx = make_ctx()
    assert tick_root(Action("none", lambda c: None), ctx) is SUCCESS
    assert tick_root(Action("false", lambda c: False), ctx) is FAILURE
    assert tick_root(Action("run", lambda c: RUNNING), ctx) is RUNNING
    assert tick_root(Action("boom", lambda c: 1 / 0), ctx) is FAILURE
    assert ctx.errors and ctx.errors[-1][0] == "boom"
    assert tick_root(Condition("bad", lambda c: c.nope), ctx) is FAILURE


def test_action_coroutine_runs_until_done_and_halt_cancels():
    async def main():
        ev = asyncio.Event()
        done = []

        async def act(ctx):
            await ev.wait()
            done.append(1)

        ctx = make_ctx()
        a = Action("async", act)
        assert tick_root(a, ctx) is RUNNING
        await asyncio.sleep(0)
        assert tick_root(a, ctx) is RUNNING and ctx.path == ["async"]
        ev.set()
        await asyncio.sleep(0)
        assert tick_root(a, ctx) is SUCCESS and done == [1]

        # halt cancels the task
        ev.clear()
        assert tick_root(a, ctx) is RUNNING
        task = a.task
        a.halt()
        await asyncio.sleep(0)
        assert task.cancelled() and a.task is None

        async def bad(ctx):
            raise RuntimeError("x")

        b = Action("bad", bad)
        assert tick_root(b, ctx) is RUNNING
        await asyncio.sleep(0)
        assert tick_root(b, ctx) is FAILURE and ctx.errors[-1][0] == "bad"

    asyncio.run(main())


def test_active_path_is_the_running_leaf():
    ctx = make_ctx()
    root = Parallel("par", [Sequence("s1", [Probe("a", SUCCESS), Probe("b")]), Probe("c", SUCCESS)], "all")
    tick_root(root, ctx)
    assert ctx.path == ["par", "s1", "b"]  # running wins over the later finished leaf


# ---- drives ---------------------------------------------------------------------------------

CFG = pers.from_dict({})


def run_drives(d: Drives, seconds: float, inp: DriveInputs, cfg=CFG, dt: float = 0.1) -> Drives:
    for _ in range(int(round(seconds / dt))):
        d.update(dt, inp, cfg)
    return d


def test_drive_directions():
    d = Drives(energy=0.5, curiosity=0.5, sociability=0.5, arousal=0.5)
    run_drives(d, 10, DriveInputs(activity="explore"))
    assert d.energy < 0.5 and d.curiosity < 0.5 and d.sociability > 0.5 and d.arousal < 0.5
    d2 = Drives(energy=0.5, curiosity=0.5, sociability=0.5, arousal=0.0)
    run_drives(d2, 10, DriveInputs(activity="rest", social=True))
    assert d2.energy > 0.5 and d2.curiosity > 0.5 and d2.sociability < 0.5
    d3 = Drives(energy=0.5, curiosity=0.5)
    run_drives(d3, 10, DriveInputs(activity="idle"))
    assert 0.49 < d3.energy < 0.5 and d3.curiosity > 0.5  # idle: slow drain, curiosity grows
    # motion reported by the body drains even if the behaviour is resting
    d4 = Drives(energy=0.5)
    run_drives(d4, 10, DriveInputs(activity="rest", moving=True))
    assert d4.energy < 0.5


def test_drive_bounds():
    rng = random.Random(3)
    cfg = copy.deepcopy(CFG)
    cfg["drives"]["noise"] = 0.2
    d = Drives()
    for i in range(20000):
        d.update(0.1, DriveInputs(activity=["rest", "explore", "social", "idle"][i // 3000 % 4],
                                  social=i % 7 == 0, startle=1.0 if i % 500 == 0 else 0.0), cfg, rng)
        for v in (d.energy, d.curiosity, d.sociability, d.arousal):
            assert 0.0 <= v <= 1.0


def test_drives_deterministic_given_rng():
    cfg = copy.deepcopy(CFG)
    cfg["drives"]["noise"] = 0.05

    def run(seed):
        d, rng = Drives(), random.Random(seed)
        for _ in range(500):
            d.update(0.1, DriveInputs(), cfg, rng)
        return d.as_dict(8)

    assert run(1) == run(1) != run(2)


def test_energy_tied_to_battery():
    d = Drives(energy=1.0)
    d.update(0.1, DriveInputs(activity="rest", battery_pct=35.0), CFG)  # (35-10)/(60-10) = 0.5
    assert d.energy == pytest.approx(0.5)
    run_drives(d, 100, DriveInputs(activity="rest", battery_pct=35.0))
    assert d.energy == pytest.approx(0.5)  # resting cannot recover past the battery
    d.update(0.1, DriveInputs(activity="rest", battery_pct=5.0), CFG)
    assert d.energy == 0.0
    d = Drives(energy=0.9)
    d.update(0.1, DriveInputs(activity="rest", battery_pct=90.0), CFG)
    assert d.energy > 0.9  # a full battery does not cap
    d.update(0.1, DriveInputs(activity="rest", battery_pct=None), CFG)
    assert d.energy > 0.9  # unknown (no battery / bench supply): no cap


def test_arousal_spike_and_half_life():
    d = Drives(arousal=0.0)
    d.update(0.1, DriveInputs(startle=1.0), CFG)
    assert d.arousal == pytest.approx(0.9)
    run_drives(d, 3.0, DriveInputs())
    assert d.arousal == pytest.approx(0.45, rel=0.01)
    d.update(0.1, DriveInputs(approaching=True), CFG)
    assert d.arousal == pytest.approx(0.6)


# ---- selector -------------------------------------------------------------------------------

def forever(name):
    return Repeat(Wait(1.0, name=f"{name}_hold"), name=f"{name}_tree")


def make_selector(cfg, safety=()):
    bs = [Behaviour(n, forever(n), a) for n, a in
          [("idle", "idle"), ("explore", "explore"), ("nap", "rest"), ("greet", "social"),
           ("startle", "active")]]
    return UtilitySelector(bs, list(safety), cfg)


def sel_cfg(**selector):
    cfg = pers.from_dict({
        "weights": {"idle": {"base": 0.4}, "explore": {"curiosity": 1.0}, "nap": {"fatigue": 1.0},
                    "greet": {"face": 1.0}, "startle": {"arousal": 1.0}},
        "selector": selector})
    return cfg


def test_selector_no_flicker_under_noise():
    cfg = sel_cfg(noise=0.04, noise_period_s=0.5, margin=0.1, min_dwell_s=1.0)
    clock = Clock()
    ctx = make_ctx(clock, drives=Drives(energy=1.0, curiosity=0.4, sociability=0.0, arousal=0.0))
    sel = make_selector(cfg)
    chosen = []
    for _ in range(6000):  # 10 minutes at 10 Hz; idle and explore tie, noise redrawn every 0.5 s
        tick_root(sel, ctx)
        chosen.append(sel.last.chosen)
        clock.advance(0.1)
    assert sel.switches == 1  # the first pick only
    assert set(chosen) <= {"idle", "explore"}

    # without hysteresis, the same noise flickers
    cfg0 = sel_cfg(noise=0.04, noise_period_s=0.5, margin=0.0, min_dwell_s=0.0)
    sel0 = make_selector(cfg0)
    ctx0 = make_ctx(Clock(), drives=Drives(energy=1.0, curiosity=0.4))
    for _ in range(6000):
        tick_root(sel0, ctx0)
        ctx0.clock.advance(0.1)
    assert sel0.switches > 50


def test_selector_switches_past_margin_after_dwell():
    cfg = sel_cfg(noise=0.0, margin=0.1, min_dwell_s=5.0, urgent=["startle"])
    clock = Clock()
    drives = Drives(energy=1.0, curiosity=0.0)
    ctx = make_ctx(clock, drives=drives)
    sel = make_selector(cfg)
    tick_root(sel, ctx)
    assert sel.last.chosen == "idle" and ctx.path[:3] == ["root", "idle", "idle_tree"]
    clock.advance(10)
    drives.curiosity = 0.49  # above idle (0.4) but within the margin
    tick_root(sel, ctx)
    assert sel.last.chosen == "idle" and sel.last.reason == "margin"
    drives.curiosity = 0.51
    tick_root(sel, ctx)
    assert sel.last.chosen == "explore" and sel.last.reason == "switch"
    assert sel.last.scores["explore"] == pytest.approx(0.51)
    # back: nap beats explore by the margin, but explore has not dwelt long enough
    clock.advance(1)
    drives.energy = 0.3  # fatigue 0.7
    tick_root(sel, ctx)
    assert sel.last.chosen == "explore" and sel.last.reason == "dwell"
    clock.advance(4)
    tick_root(sel, ctx)
    assert sel.last.chosen == "nap"
    # an urgent behaviour skips the dwell
    clock.advance(0.1)
    drives.arousal = 0.9
    tick_root(sel, ctx)
    assert sel.last.chosen == "startle" and sel.last.reason == "urgent"


def test_selector_product_terms():
    cfg = sel_cfg(noise=0.0)
    cfg["weights"]["greet"] = {"sociability*face": 1.0}
    sel = make_selector(cfg)
    ctx = make_ctx(drives=Drives(sociability=0.8))
    tick_root(sel, ctx)
    assert sel.last.scores["greet"] == 0.0
    ctx.percepts["face"] = True
    tick_root(sel, ctx)
    assert sel.last.scores["greet"] == pytest.approx(0.8)


def test_selector_falls_back_when_choice_fails():
    cfg = sel_cfg(noise=0.0, margin=0.1, min_dwell_s=0.0)
    ctx = make_ctx(drives=Drives(energy=1.0, curiosity=0.0))
    ctx.percepts["face"] = True
    bs = [Behaviour("idle", forever("idle")), Behaviour("explore", forever("explore")),
          Behaviour("nap", forever("nap")), Behaviour("greet", Probe("greet_fails", FAILURE)),
          Behaviour("startle", forever("startle"))]
    sel = UtilitySelector(bs, [], cfg)
    tick_root(sel, ctx)
    assert sel.last.chosen == "idle" and sel.last.reason.startswith("fallback")


def test_safety_preempts_and_releases():
    cfg = sel_cfg(noise=0.0, min_dwell_s=60.0)
    fallen = {"on": False}
    probe = Probe("wait_recover")
    sel = make_selector(cfg, [SafetyBranch("fallen", lambda c: fallen["on"], probe)])
    clock = Clock()
    ctx = make_ctx(clock, drives=Drives(energy=1.0, curiosity=0.0))
    tick_root(sel, ctx)
    idle_tree = sel.behaviours["idle"].tree
    assert sel.last.chosen == "idle"
    fallen["on"] = True
    tick_root(sel, ctx)
    assert sel.last.chosen == "fallen" and sel.last.reason == "safety"
    assert ctx.path == ["root", "fallen", "wait_recover"] and idle_tree.child.until is None  # halted
    fallen["on"] = False
    clock.advance(0.1)
    tick_root(sel, ctx)
    assert sel.last.chosen == "idle" and sel.last.reason == "free"  # no dwell after safety


# ---- personality ---------------------------------------------------------------------------

def test_personality_default_file_loads_and_matches_defaults():
    assert pers.load() == pers.from_dict({})


def test_mini_yaml_matches_subset():
    text = """
# comment
name: "kilo # not a comment"
a:
  b: 1
  c: -2.5e-1   # trailing
  d: {x: 1, y: [a, 'b c'], z: true}
  e: [1, 2.0, null]
list:
  - one
  - 2
  -
    k: v
empty:
"""
    assert pers.mini_yaml(text) == {
        "name": "kilo # not a comment", "a": {"b": 1, "c": -0.25, "d": {"x": 1, "y": ["a", "b c"], "z": True},
                                            "e": [1, 2.0, None]},
        "list": ["one", 2, {"k": "v"}], "empty": None}
    try:
        import yaml
    except ImportError:
        return
    assert yaml.safe_load(text) == pers.mini_yaml(text)
    assert yaml.safe_load(pers.DEFAULT_PATH.read_text()) == pers.mini_yaml(pers.DEFAULT_PATH.read_text())


def test_personality_overrides_and_validation(tmp_path):
    f = tmp_path / "p.yaml"
    f.write_text("name: shy\nweights:\n  greet: {face: 0.2}\nselector:\n  margin: 0.2\n")
    cfg = pers.load(f)
    assert cfg["name"] == "shy" and cfg["weights"]["greet"] == {"face": 0.2}
    assert cfg["weights"]["idle"] == pers.DEFAULTS["weights"]["idle"]
    assert cfg["selector"]["margin"] == 0.2 and cfg["selector"]["min_dwell_s"] == 6.0
    bad = [
        ({"selectr": {}}, "unknown key 'selectr'"),
        ({"selector": {"margin": "big"}}, "selector.margin must be a number"),
        ({"selector": {"margin": 2}}, "out of range"),
        ({"weights": {"dance": {"base": 1}}}, "unknown behaviour"),
        ({"weights": {"idle": {"boredom": 1}}}, "unknown term weights.idle.boredom"),
        ({"weights": {"idle": {"face*boredom": 1}}}, "unknown term weights.idle.face"),
        ({"drives": {"initial": {"energy": 1.5}}}, "drives.initial.energy"),
        ({"drives": {"energy": {"battery_empty_pct": 70}}}, "battery_empty_pct must be <"),
        ({"safety": {"critical_battery_pct": 50}}, "critical_battery_pct must be <="),
        ({"behaviours": {"idle": {"posture": "dance"}}}, "behaviours.idle.posture"),
        ({"behaviours": {"explore": {"looks": 0}}}, "looks"),
        ({"selector": {"urgent": ["nope"]}}, "selector.urgent"),
    ]
    for raw, msg in bad:
        with pytest.raises(pers.PersonalityError, match=msg.replace("(", r"\(")):
            pers.from_dict(raw)
    with pytest.raises(pers.PersonalityError):
        pers.load(tmp_path / "missing.yaml")


# ---- runner end to end with a fake body ------------------------------------------------------

class FakeBody:
    def __init__(self, clock):
        self.clock = clock
        self.calls: list[tuple[float, str, tuple]] = []
        self.snap = {"epoch": 1, "mode": "IDLE", "battery_pct": 90.0, "fallen": False, "moving": False,
                     "dist_mm": None, "approach_mm_s": None, "events": []}
        self.ticks = 0

    def snapshot(self):
        return dict(self.snap)

    def on_tick(self):
        self.ticks += 1

    def _rec(self, name, *a):
        self.calls.append((self.clock(), name, a))

    def posture(self, name, balance=None):
        self._rec("posture", name, balance)

    def set_balance(self, enable):
        self._rec("set_balance", enable)

    def look(self, pan, tilt):
        self._rec("look", pan, tilt)

    def beep(self, notes):
        self._rec("beep", notes)

    def play(self, trick, mirror=False):
        self._rec("play", trick)

        async def go():
            for _ in range(3):
                await asyncio.sleep(0)
        return go()

    def of(self, name):
        return [c for c in self.calls if c[1] == name]


def day_cfg():
    return pers.from_dict({
        "drives": {"initial": {"energy": 0.75, "curiosity": 0.2, "sociability": 0.3, "arousal": 0.0},
                   "energy": {"drain_active": 0.01, "drain_idle": 0.002, "recover_rest": 0.01}},
        "behaviours": {"explore": {"looks": 8}},
    })


def segments(chosen):
    out = []
    for c in chosen:
        if not out or out[-1] != c:
            out.append(c)
    return out


def test_runner_plausible_day(tmp_path):
    clock = Clock(1000.0)
    body = FakeBody(clock)
    face = {"on": False}
    log = tmp_path / "behaviour.jsonl"
    runner = Runner(body, day_cfg(), percepts=lambda: {"face": face["on"], "face_pan": 12.0},
                    clock=clock, sleep=clock.sleep, seed=42, log_path=log, history=100000)

    async def scenario():
        task = asyncio.ensure_future(runner.run(duration=420))
        # a face shows up after 6 minutes, for 20 s
        while clock() < 1360:
            await asyncio.sleep(0)
        face["on"] = True
        while clock() < 1380:
            await asyncio.sleep(0)
        face["on"] = False
        return await task

    why = asyncio.run(scenario())
    assert why == "duration" and runner.ticks == pytest.approx(4200, abs=2)
    hist = list(runner.history)
    segs = segments(r["chosen"] for r in hist)
    # idle -> curiosity-driven look-around -> nap when energy is low -> greet on the face
    assert segs[0] == "idle"
    i_explore = segs.index("explore")
    i_nap = segs.index("nap", i_explore)
    i_greet = segs.index("greet", i_nap)
    assert 0 < i_explore < i_nap < i_greet
    first = {name: next(r for r in hist if r["chosen"] == name) for name in ("explore", "nap", "greet")}
    assert first["explore"]["drives"]["curiosity"] > 0.4
    assert first["nap"]["drives"]["energy"] < 0.5
    assert 1360 <= first["greet"]["t"] < 1361
    # no flicker: few switches for 7 minutes
    assert len(segs) < 40, segs
    # the body saw it
    assert ("stand", None) in [c[2] for c in body.of("posture")]
    assert ("sit", None) in [c[2] for c in body.of("posture")]
    nap_t = first["nap"]["t"]
    assert any(c[2] == ("rest", None) and c[0] >= nap_t for c in body.of("posture"))
    assert any(c[2] == (False,) and c[0] >= nap_t for c in body.of("set_balance"))
    plays = body.of("play")
    assert [p[2] for p in plays] == [("hi",)]  # once: the cooldown holds it
    assert any(c[2] == (12.0, 0.0) for c in body.of("look"))  # looked at the face
    assert len(body.of("look")) > 20
    assert body.ticks == runner.ticks
    # postures are sent on change only
    postures = body.of("posture")
    assert all(a[2] != b[2] for a, b in zip(postures, postures[1:]))
    # the log
    lines = [json.loads(x) for x in log.read_text().splitlines()]
    assert lines[0]["kind"] == "start" and lines[0]["seed"] == 42
    assert lines[-1]["kind"] == "stop" and lines[-1]["reason"] == "duration"
    recs = [x for x in lines if x["kind"] == "behaviour"]
    assert 400 <= len(recs) < 4200  # changes + at least 1 Hz, not every tick
    r = recs[-1]
    assert set(r) >= {"t", "chosen", "reason", "path", "drives", "scores", "percepts"}
    assert set(r["drives"]) == {"energy", "curiosity", "sociability", "arousal"}
    assert r["path"][0] == "root" and r["path"][1] == r["chosen"]
    assert {x["chosen"] for x in recs} >= {"idle", "explore", "nap", "greet"}


def test_runner_is_deterministic_given_seed(tmp_path):
    def run():
        clock = Clock(0.0)
        r = Runner(FakeBody(clock), day_cfg(), clock=clock, sleep=clock.sleep, seed=5, log=False,
                   history=10000)
        asyncio.run(r.run(duration=120))
        return [(x["chosen"], tuple(x["path"]), x["drives"]["curiosity"]) for x in r.history]

    assert run() == run()


def test_runner_fall_startle_low_battery_wind_down():
    clock = Clock(0.0)
    body = FakeBody(clock)
    powered_off = []
    runner = Runner(body, day_cfg(), clock=clock, sleep=clock.sleep, seed=1, log=False, history=10000,
                    on_poweroff=lambda: powered_off.append(clock()))

    async def scenario():
        task = asyncio.ensure_future(runner.run(duration=600))
        while clock() < 10:
            await asyncio.sleep(0)
        body.snap.update(fallen=True, mode="FALLEN", epoch=2)
        while clock() < 12:
            await asyncio.sleep(0)
        body.snap.update(fallen=False, mode="IDLE")
        while clock() < 20:
            await asyncio.sleep(0)
        body.snap.update(dist_mm=150.0, approach_mm_s=600.0)  # something rushes in
        while clock() < 20.5:
            await asyncio.sleep(0)
        body.snap.update(dist_mm=None, approach_mm_s=None)
        while clock() < 30:
            await asyncio.sleep(0)
        body.snap.update(battery_pct=15.0)
        while clock() < 60:
            await asyncio.sleep(0)
        body.snap.update(battery_pct=6.0)
        return await task

    why = asyncio.run(scenario())
    hist = list(runner.history)
    segs = segments(r["chosen"] for r in hist)
    assert why == "poweroff" and powered_off and 60 <= powered_off[0] < 60 + 5.0 + 1
    assert segs[-2:] == ["low_battery", "wind_down"], segs
    i_fall = segs.index("fallen")
    fall_recs = [r for r in hist if r["chosen"] == "fallen"]
    assert fall_recs[0]["t"] == pytest.approx(10.0) and fall_recs[-1]["t"] >= 11.9 + 2.0 - 0.15  # settled
    assert max(r["drives"]["arousal"] for r in hist if r["t"] < 20) == pytest.approx(0.9, abs=0.01)
    i_startle = segs.index("startle", i_fall)
    startle = [r for r in hist if r["chosen"] == "startle"]
    assert 20 <= startle[0]["t"] < 20.15 and startle[0]["reason"] in ("urgent", "switch")  # within 0.5 s
    assert startle[0]["percepts"].get("approaching") and segs[i_startle + 1] != "startle"
    assert any(c[2] == ("stand", True) and c[0] >= 20 for c in body.of("posture"))
    # after the fall (epoch change) the posture is re-asserted
    assert any(c[0] >= 12 and c[2][0] == "stand" for c in body.of("posture"))
    low = [r for r in hist if r["chosen"] == "low_battery"]
    assert low and all(r["drives"]["energy"] <= 0.11 for r in low)  # (15-10)/50 cap
    assert body.of("beep") and body.of("posture")[-1][2] == ("rest", None)


def test_runner_shutdown_request_and_stop():
    clock = Clock(0.0)
    body = FakeBody(clock)
    r = Runner(body, day_cfg(), clock=clock, sleep=clock.sleep, seed=1, log=False)

    async def go():
        t = asyncio.ensure_future(r.run())
        while clock() < 3:
            await asyncio.sleep(0)
        r.request_shutdown()
        return await t

    assert asyncio.run(go()) == "poweroff"
    r2 = Runner(FakeBody(clock), day_cfg(), clock=clock, sleep=clock.sleep, seed=1, log=False)

    async def go2():
        t = asyncio.ensure_future(r2.run())
        while r2.ticks < 5:
            await asyncio.sleep(0)
        r2.stop()
        return await t

    assert asyncio.run(go2()) == "stop"


def test_build_root_names():
    root = build_root(pers.from_dict({}))
    assert set(root.behaviours) == {"idle", "explore", "nap", "greet", "startle"}
    assert [s.name for s in root.safety] == ["fallen", "wind_down", "low_battery"]


# ---- against the virtual body ------------------------------------------------------------------

def test_runner_against_vbody():
    from kilo.body.client import BodyClient
    from kilo.link.serial_link import LinkConfig, SerialLink
    from kilo.link.transport import loopback_pair
    from kilo.protocol import kilo_protocol as P
    from kilo.vbody.sim import VirtualBody

    from conftest import FAST

    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="loopback")
    link.mark_alive()
    link.start()
    try:
        client = BodyClient(link)
        client.wait_ready(2)
        adapter = BodyAdapter(client)
        cfg = pers.from_dict({"safety": {"fallen_settle_s": 0.3}})
        runner = Runner(adapter, cfg, seed=3, log=False, history=1000)

        async def scenario():
            loop = asyncio.get_running_loop()
            task = asyncio.ensure_future(runner.run(duration=4.0))
            await asyncio.sleep(1.0)
            sat = vb.posture
            loop.call_soon(vb.trigger_fall)
            await asyncio.sleep(1.5)
            loop.call_soon(vb.set_battery, P.BatteryLevel.LOW)
            await task
            return sat

        sat = asyncio.run(scenario())
        hist = list(runner.history)
        segs = segments(r["chosen"] for r in hist)
        assert sat == "SIT"  # idle sat down through the real link
        assert segs[0] == "idle" and "fallen" in segs and segs[-1] == "low_battery", segs
        assert vb.posture == "REST"
        assert 30 <= runner.ticks <= 42  # 10 Hz in real time
        assert hist[-1]["mode"] == "LOW_BATT"
        # head/trick/buzzer are not in BodyClient yet: the adapter stubs them
        assert set(adapter.unsupported) <= {"look", "play", "beep"}
        assert ("posture", ("sit", None)) in adapter.calls
        assert not any("errors" in r for r in hist), [r["errors"] for r in hist if "errors" in r]
    finally:
        link.stop()
        vb.stop()
