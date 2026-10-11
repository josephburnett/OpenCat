"""Personality: drive rates, behaviour weights and selector tuning from YAML.

``load(path=None)`` reads ``personality.yaml`` (the packaged default when no
path is given), merges it over ``DEFAULTS`` and validates it: unknown keys,
wrong types and out-of-range values raise ``PersonalityError`` naming the key.

PyYAML is optional (``pip install 'kilo-mind[behaviour]'`` or apt
``python3-yaml``). Without it a small built-in parser reads the subset the
file uses: block mappings by indentation, ``- item`` lists, single-line
``{k: v}`` / ``[a, b]`` flow collections, comments, and int/float/bool/null/
string scalars.
"""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any

DEFAULT_PATH = Path(__file__).with_name("personality.yaml")

BEHAVIOURS = ("idle", "explore", "nap", "greet", "startle")
TERMS = ("base", "energy", "fatigue", "curiosity", "sociability", "arousal", "calm",
         "face", "pet", "near", "approaching", "alone")
DRIVES = ("energy", "curiosity", "sociability", "arousal")
POSTURES = ("rest", "sit", "stand", "balance")

DEFAULTS: dict[str, Any] = {
    "name": "kilo",
    "drives": {
        "initial": {"energy": 0.8, "curiosity": 0.2, "sociability": 0.3, "arousal": 0.0},
        "energy": {"drain_active": 0.004, "drain_idle": 0.0008, "recover_rest": 0.004,
                   "battery_full_pct": 60.0, "battery_empty_pct": 10.0},
        "curiosity": {"growth": 0.003, "satisfy": 0.02},
        "sociability": {"growth": 0.002, "satisfy": 0.05},
        "arousal": {"startle": 0.9, "approach": 0.6, "half_life_s": 3.0},
        "noise": 0.0,
    },
    "weights": {
        "idle": {"base": 0.35},
        "explore": {"curiosity": 1.0, "energy": 0.2, "fatigue": -0.4},
        "nap": {"fatigue": 1.0, "calm": 0.1, "base": -0.2},
        "greet": {"face": 0.5, "sociability*face": 0.6, "pet": 0.5},
        "startle": {"arousal": 1.2, "base": -0.3},
    },
    "selector": {"noise": 0.04, "noise_period_s": 8.0, "margin": 0.1, "min_dwell_s": 6.0,
                 "urgent": ["startle", "greet"]},
    "safety": {"low_battery_pct": 20.0, "critical_battery_pct": 8.0, "wind_down_s": 5.0,
               "fallen_settle_s": 2.0},
    "percepts": {"near_mm": 250.0, "approach_mm_s": 400.0},
    "behaviours": {
        "idle": {"posture": "sit", "look_every_s": 6.0, "look_chance": 0.5,
                 "look_pan_deg": 30.0, "look_tilt_deg": 15.0},
        "explore": {"posture": "stand", "look_every_s": 1.5, "looks": 12,
                    "look_pan_deg": 60.0, "look_tilt_deg": 25.0},
        "nap": {"posture": "rest", "min_s": 40.0},
        "greet": {"trick": "hi", "cooldown_s": 30.0},
        "startle": {"hold_s": 2.0},
    },
}


class PersonalityError(ValueError):
    pass


# ---- loading ----------------------------------------------------------------------

def parse_yaml(text: str) -> Any:
    """yaml.safe_load if PyYAML is installed, else the built-in subset parser."""
    try:
        import yaml  # type: ignore[import-not-found]
    except ImportError:
        return mini_yaml(text)
    return yaml.safe_load(text)


def load(path: str | Path | None = None) -> dict[str, Any]:
    p = Path(path) if path is not None else DEFAULT_PATH
    try:
        raw = parse_yaml(p.read_text(encoding="utf-8"))
    except OSError as e:
        raise PersonalityError(f"cannot read {p}: {e}") from None
    except ValueError as e:
        raise PersonalityError(f"{p}: {e}") from None
    return from_dict(raw or {})


def from_dict(raw: dict[str, Any]) -> dict[str, Any]:
    """Merge ``raw`` over DEFAULTS and validate. Weights replace per behaviour."""
    if not isinstance(raw, dict):
        raise PersonalityError("personality must be a mapping")
    cfg = copy.deepcopy(DEFAULTS)
    _merge(cfg, raw, "")
    validate(cfg)
    return cfg


def _merge(dst: dict, src: dict, prefix: str) -> None:
    for k, v in src.items():
        key = f"{prefix}{k}"
        if prefix == "weights.":
            if k not in BEHAVIOURS:
                raise PersonalityError(f"unknown behaviour {key!r} (known: {', '.join(BEHAVIOURS)})")
            if not isinstance(v, dict):
                raise PersonalityError(f"{key} must be a mapping of term: weight")
            dst[k] = dict(v)  # a behaviour's weights replace the default ones
            continue
        if k not in dst:
            raise PersonalityError(f"unknown key {key!r}")
        if isinstance(dst[k], dict):
            if not isinstance(v, dict):
                raise PersonalityError(f"{key} must be a mapping")
            _merge(dst[k], v, key + ".")
        else:
            dst[k] = v


def _num(cfg: dict, key: str, lo: float | None = 0.0, hi: float | None = None) -> None:
    node: Any = cfg
    parts = key.split(".")
    for p in parts[:-1]:
        node = node[p]
    v = node[parts[-1]]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise PersonalityError(f"{key} must be a number, got {v!r}")
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise PersonalityError(f"{key} = {v} out of range [{lo}, {hi}]")
    node[parts[-1]] = float(v)


def validate(cfg: dict[str, Any]) -> None:
    d = cfg["drives"]
    for k in DRIVES:
        if k not in d["initial"]:
            raise PersonalityError(f"drives.initial.{k} missing")
    for k in d["initial"]:
        if k not in DRIVES:
            raise PersonalityError(f"unknown drive drives.initial.{k}")
        _num(cfg, f"drives.initial.{k}", 0.0, 1.0)
    for k in ("drain_active", "drain_idle", "recover_rest"):
        _num(cfg, f"drives.energy.{k}", 0.0, 1.0)
    _num(cfg, "drives.energy.battery_full_pct", 0.0, 100.0)
    _num(cfg, "drives.energy.battery_empty_pct", 0.0, 100.0)
    if d["energy"]["battery_empty_pct"] >= d["energy"]["battery_full_pct"]:
        raise PersonalityError("drives.energy.battery_empty_pct must be < battery_full_pct")
    for drive in ("curiosity", "sociability"):
        for k in ("growth", "satisfy"):
            _num(cfg, f"drives.{drive}.{k}", 0.0, 1.0)
    _num(cfg, "drives.arousal.startle", 0.0, 1.0)
    _num(cfg, "drives.arousal.approach", 0.0, 1.0)
    _num(cfg, "drives.arousal.half_life_s", 0.01, None)
    _num(cfg, "drives.noise", 0.0, 0.5)
    for b in BEHAVIOURS:
        if b not in cfg["weights"]:
            raise PersonalityError(f"weights.{b} missing")
        for term in cfg["weights"][b]:
            if not term or any(f not in TERMS for f in term.split("*")):
                raise PersonalityError(f"unknown term weights.{b}.{term} (known: {', '.join(TERMS)})")
            _num(cfg, f"weights.{b}.{term}", None, None)
    s = cfg["selector"]
    _num(cfg, "selector.noise", 0.0, 1.0)
    _num(cfg, "selector.noise_period_s", 0.1, None)
    _num(cfg, "selector.margin", 0.0, 1.0)
    _num(cfg, "selector.min_dwell_s", 0.0, None)
    if not isinstance(s["urgent"], list) or any(u not in BEHAVIOURS for u in s["urgent"]):
        raise PersonalityError(f"selector.urgent must be a list of {', '.join(BEHAVIOURS)}")
    _num(cfg, "safety.low_battery_pct", 0.0, 100.0)
    _num(cfg, "safety.critical_battery_pct", 0.0, 100.0)
    if cfg["safety"]["critical_battery_pct"] > cfg["safety"]["low_battery_pct"]:
        raise PersonalityError("safety.critical_battery_pct must be <= low_battery_pct")
    _num(cfg, "safety.wind_down_s", 0.0, None)
    _num(cfg, "safety.fallen_settle_s", 0.0, None)
    _num(cfg, "percepts.near_mm", 0.0, None)
    _num(cfg, "percepts.approach_mm_s", 0.0, None)
    bh = cfg["behaviours"]
    for b in ("idle", "explore", "nap"):
        if bh[b]["posture"] not in POSTURES:
            raise PersonalityError(f"behaviours.{b}.posture must be one of {', '.join(POSTURES)}")
    for b in ("idle", "explore"):
        _num(cfg, f"behaviours.{b}.look_every_s", 0.1, None)
        _num(cfg, f"behaviours.{b}.look_pan_deg", 0.0, 90.0)
        _num(cfg, f"behaviours.{b}.look_tilt_deg", 0.0, 90.0)
    _num(cfg, "behaviours.idle.look_chance", 0.0, 1.0)
    looks = bh["explore"]["looks"]
    if isinstance(looks, bool) or not isinstance(looks, int) or looks < 1:
        raise PersonalityError("behaviours.explore.looks must be a positive integer")
    if not isinstance(bh["greet"]["trick"], str) or not bh["greet"]["trick"]:
        raise PersonalityError("behaviours.greet.trick must be a trick name")
    _num(cfg, "behaviours.greet.cooldown_s", 0.0, None)
    _num(cfg, "behaviours.nap.min_s", 0.0, None)
    _num(cfg, "behaviours.startle.hold_s", 0.0, None)


# ---- the built-in YAML subset -------------------------------------------------------

_NUM = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$")


def _strip_comment(line: str) -> str:
    out, q = [], None
    for i, c in enumerate(line):
        if q:
            if c == q:
                q = None
        elif c in "'\"":
            q = c
        elif c == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(c)
    return "".join(out).rstrip()


def _scalar(s: str) -> Any:
    s = s.strip()
    if s == "" or s in ("~", "null", "Null", "NULL"):
        return None
    if s in ("true", "True", "TRUE", "yes", "on"):
        return True
    if s in ("false", "False", "FALSE", "no", "off"):
        return False
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "'\"":
        return s[1:-1]
    if _NUM.match(s):
        return int(s) if re.match(r"^[-+]?\d+$", s) else float(s)
    if s.startswith("{") or s.startswith("["):
        return _flow(s)
    return s


def _split_top(s: str) -> list[str]:
    parts, depth, cur, q = [], 0, [], None
    for c in s:
        if q:
            q = None if c == q else q
        elif c in "'\"":
            q = c
        elif c in "[{":
            depth += 1
        elif c in "]}":
            depth -= 1
        elif c == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
            continue
        cur.append(c)
    if "".join(cur).strip():
        parts.append("".join(cur))
    return parts


def _flow(s: str) -> Any:
    s = s.strip()
    if s.startswith("{"):
        if not s.endswith("}"):
            raise ValueError(f"unterminated flow mapping: {s}")
        out = {}
        for item in _split_top(s[1:-1]):
            k, sep, v = item.partition(":")
            if not sep:
                raise ValueError(f"flow mapping item without ':': {item}")
            out[str(_scalar(k))] = _scalar(v)
        return out
    if not s.endswith("]"):
        raise ValueError(f"unterminated flow list: {s}")
    return [_scalar(x) for x in _split_top(s[1:-1])]


def mini_yaml(text: str) -> Any:
    lines = []
    for n, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise ValueError(f"line {n}: tabs in indentation")
        s = _strip_comment(raw)
        if s.strip() and s.strip() != "---":
            lines.append((len(s) - len(s.lstrip()), s.strip(), n))
    if not lines:
        return None
    val, i = _block(lines, 0, lines[0][0])
    if i != len(lines):
        raise ValueError(f"line {lines[i][2]}: unexpected indentation")
    return val


def _block(lines: list, i: int, indent: int) -> tuple[Any, int]:
    if lines[i][1].startswith("- ") or lines[i][1] == "-":
        out: list[Any] = []
        while i < len(lines) and lines[i][0] == indent and (lines[i][1].startswith("- ") or lines[i][1] == "-"):
            item = lines[i][1][1:].strip()
            i += 1
            if item:
                out.append(_scalar(item))
            elif i < len(lines) and lines[i][0] > indent:
                v, i = _block(lines, i, lines[i][0])
                out.append(v)
            else:
                out.append(None)
        return out, i
    mapping: dict[str, Any] = {}
    while i < len(lines) and lines[i][0] == indent:
        _, s, n = lines[i]
        m = re.match(r"^(\"[^\"]*\"|'[^']*'|[^:]+?)\s*:(\s+(.*))?$", s)
        if not m:
            raise ValueError(f"line {n}: expected 'key: value'")
        key = str(_scalar(m.group(1)))
        rest = (m.group(3) or "").strip()
        i += 1
        if rest:
            mapping[key] = _scalar(rest)
        elif i < len(lines) and lines[i][0] > indent:
            mapping[key], i = _block(lines, i, lines[i][0])
        else:
            mapping[key] = None
    return mapping, i
