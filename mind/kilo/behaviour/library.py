"""The first behaviours (P3), written against a duck-typed body interface.

The body interface (``ctx.body``) the behaviours call:

- ``posture(name, balance=None)``: latch a posture goal (rest, sit, stand,
  balance); BodyClient has it.
- ``set_balance(enable)``: BodyClient has it.
- ``look(pan_deg, tilt_deg)``: head target. Not in the body yet (P2).
- ``play(trick, mirror=False)``: returns an awaitable that resolves on
  TRICK_DONE. Not in the body yet (P2).
- ``beep(notes)``: ``[(midi_note, ms), ...]``. Not in the body yet.

``adapter.BodyAdapter`` wraps a real BodyClient and stubs the missing ones.
Posture and balance goals are cached in ``ctx.data`` so they are sent on
change only; the runner clears the cache on an epoch change, when the body
has dropped all motion goals.

Top-level behaviours: idle, explore (stub: no gait yet, looks around
standing), nap, greet, startle. Safety branches, in priority order: fallen
(wait for the body's own recovery, then settle), wind-down (critical battery
or shutdown requested: rest, then ask for poweroff; latched), low battery
(rest with balance off until it recovers).
"""

from __future__ import annotations

from typing import Any

from .selector import Behaviour, SafetyBranch, UtilitySelector
from .tree import (RUNNING, SUCCESS, Action, Chance, Condition, Context, Cooldown,
                   ForceSuccess, Node, Repeat, Sequence, Wait)

LOW_BATTERY_HYSTERESIS_PCT = 5.0
CRITICAL_LEVEL = 3  # protocol BatteryLevel.CRITICAL (EVENT LOW_BATTERY arg)
SAD_BEEP = [(64, 120), (60, 120), (55, 300)]


# ---- action helpers ------------------------------------------------------------------

def posture(name: str, balance: bool | None = None) -> Action:
    def fn(ctx: Context) -> Any:
        want = (name, balance)
        if ctx.data.get("posture") != want:
            ctx.body.posture(name, balance=balance)
            ctx.data["posture"] = want
        return SUCCESS
    return Action(f"posture:{name}" + ("+bal" if balance else ""), fn)


def set_balance(enable: bool) -> Action:
    def fn(ctx: Context) -> Any:
        if ctx.data.get("balance") != enable:
            ctx.body.set_balance(enable)
            ctx.data["balance"] = enable
        return SUCCESS
    return Action("balance:" + ("on" if enable else "off"), fn)


def look(pan: float, tilt: float, name: str | None = None) -> Action:
    def fn(ctx: Context) -> Any:
        ctx.body.look(pan, tilt)
        ctx.data["head"] = (pan, tilt)
    return Action(name or f"look({pan:g},{tilt:g})", fn)


def look_random(pan: float, tilt: float) -> Action:
    def fn(ctx: Context) -> Any:
        p = round(ctx.rng.uniform(-pan, pan), 1)
        t = round(ctx.rng.uniform(-tilt / 2, tilt), 1)
        ctx.body.look(p, t)
        ctx.data["head"] = (p, t)
    return Action("look_around", fn)


def look_toward_stimulus() -> Action:
    def fn(ctx: Context) -> Any:
        p = ctx.percepts
        pan = float(p.get("stimulus_pan", p.get("face_pan", 0.0)) or 0.0)
        tilt = float(p.get("stimulus_tilt", p.get("face_tilt", 0.0)) or 0.0)
        ctx.body.look(pan, tilt)
        ctx.data["head"] = (pan, tilt)
    return Action("look_toward", fn)


def play(trick: str) -> Action:
    def fn(ctx: Context) -> Any:
        return ctx.body.play(trick)
    return Action(f"play:{trick}", fn)


def beep(notes: list[tuple[int, int]], name: str = "beep") -> Action:
    def fn(ctx: Context) -> Any:
        ctx.body.beep(list(notes))
    return Action(name, fn)


def forever() -> Node:
    return Repeat(Wait(1.0, name="hold"), name="Forever")


# ---- behaviours ------------------------------------------------------------------------

def idle(cfg: dict[str, Any]) -> Behaviour:
    b = cfg["behaviours"]["idle"]
    glance = Sequence("glance", [look_random(b["look_pan_deg"], b["look_tilt_deg"]),
                                 Wait(1.5), look(0, 0, "look_center")])
    loop = Repeat(Sequence("idle_beat", [Wait(b["look_every_s"]),
                                         ForceSuccess(Chance(b["look_chance"], glance))]))
    return Behaviour("idle", Sequence("idle_seq", [posture(b["posture"]), loop]), "idle")


def explore(cfg: dict[str, Any]) -> Behaviour:
    b = cfg["behaviours"]["explore"]
    # Stand and look around. BodyAdapter.walk() exists since P2, but no
    # behaviour walks until the owner allows autonomous walking.
    scan = Repeat(Sequence("scan", [look_random(b["look_pan_deg"], b["look_tilt_deg"]),
                                    Wait(b["look_every_s"])]), times=int(b["looks"]))
    return Behaviour("explore", Sequence("explore_seq", [posture(b["posture"]), scan,
                                                         look(0, 0, "look_center")]), "explore")


def nap(cfg: dict[str, Any]) -> Behaviour:
    b = cfg["behaviours"]["nap"]
    return Behaviour("nap", Sequence("nap_seq", [set_balance(False), posture(b["posture"]),
                                                 look(0, -10, "head_down"), forever()]), "rest",
                     min_dwell=b["min_s"])


def has_company(ctx: Context) -> bool:
    p = ctx.percepts
    return bool(p.get("face") or p.get("pet"))


def greet(cfg: dict[str, Any]) -> Behaviour:
    b = cfg["behaviours"]["greet"]
    act = Cooldown(b["cooldown_s"], Sequence("greet_seq", [
        posture("stand"), look_toward_stimulus(), play(b["trick"]), look(0, 0, "look_center")]))
    return Behaviour("greet", Sequence("greet_gate", [Condition("company", has_company), act]), "social")


def startle(cfg: dict[str, Any]) -> Behaviour:
    b = cfg["behaviours"]["startle"]
    return Behaviour("startle", Sequence("startle_seq", [
        posture("stand", balance=True), look_toward_stimulus(), Wait(b["hold_s"])]), "active")


# ---- safety ------------------------------------------------------------------------------

def _battery_critical(ctx: Context, cfg: dict[str, Any]) -> bool:
    pct = ctx.state.get("battery_pct")
    if pct is not None and pct <= cfg["safety"]["critical_battery_pct"]:
        return True
    if ctx.percepts.get("battery_critical"):
        return True
    from kilo.link.consts import EV  # read-only use of the generated codes
    for _, e in ctx.state.get("events") or ():
        if isinstance(e, dict) and e.get("code") == EV["LOW_BATTERY"] and (e.get("arg") or 0) >= CRITICAL_LEVEL:
            return True
    return False


def fallen_branch(cfg: dict[str, Any]) -> SafetyBranch:
    settle = cfg["safety"]["fallen_settle_s"]

    def cond(ctx: Context) -> bool:
        if ctx.state.get("fallen"):
            ctx.data["settle_until"] = ctx.now + settle
            ctx.data.pop("posture", None)  # the body dropped our goals
            ctx.data.pop("balance", None)
            return True
        return ctx.now < ctx.data.get("settle_until", float("-inf"))

    return SafetyBranch("fallen", cond, Action("wait_recover", lambda ctx: RUNNING))


def wind_down_branch(cfg: dict[str, Any]) -> SafetyBranch:
    def cond(ctx: Context) -> bool:
        if ctx.data.get("winding_down"):
            return True
        if ctx.data.get("shutdown_requested") or ctx.percepts.get("shutdown") or _battery_critical(ctx, cfg):
            ctx.data["winding_down"] = True
            return True
        return False

    def poweroff(ctx: Context) -> Any:
        ctx.data["poweroff"] = True

    tree = Sequence("wind_down_seq", [
        ForceSuccess(beep(SAD_BEEP, "sad_beep")), set_balance(False), posture("rest"),
        Wait(cfg["safety"]["wind_down_s"], name="settle"), Action("poweroff", poweroff),
        forever()])
    return SafetyBranch("wind_down", cond, tree)


def low_battery_branch(cfg: dict[str, Any]) -> SafetyBranch:
    low = cfg["safety"]["low_battery_pct"]

    def cond(ctx: Context) -> bool:
        pct = ctx.state.get("battery_pct")
        low_now = ctx.state.get("mode") == "LOW_BATT" or (pct is not None and pct < low)
        if low_now:
            ctx.data["low_battery"] = True
        elif ctx.data.get("low_battery") and (pct is None or pct >= low + LOW_BATTERY_HYSTERESIS_PCT):
            ctx.data["low_battery"] = False
        return bool(ctx.data.get("low_battery"))

    tree = Sequence("rest_low_battery", [set_balance(False), posture("rest"), forever()])
    return SafetyBranch("low_battery", cond, tree)


def build_root(cfg: dict[str, Any]) -> UtilitySelector:
    behaviours = [idle(cfg), explore(cfg), nap(cfg), greet(cfg), startle(cfg)]
    safety = [fallen_branch(cfg), wind_down_branch(cfg), low_battery_branch(cfg)]
    return UtilitySelector(behaviours, safety, cfg)
