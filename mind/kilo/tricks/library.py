"""The first-version trick library (§5), stored as JSON in tricks/library/.

The files are generated from src/InstinctNybble.h by
`python -m kilo.tricks.import_legacy` and committed; edit the importer (or
author new tricks by hand), not the generated files.
"""

from __future__ import annotations

from pathlib import Path

from .model import Trick

LIBRARY_DIR = Path(__file__).resolve().parent / "library"

# Kilo name -> legacy array name in src/InstinctNybble.h, in §5 order.
FIRST_VERSION = {
    # basics
    "sit": "sit",
    "rest": "rest",
    "stand": "up",
    "stretch": "str",
    "balance": "balance",
    # social
    "hi": "hi",
    "wash_face": "wsf",
    "check_around": "ck",
    "sniff": "snf",
    "scratch": "scrh",
    "pee": "pee",
    "push_up": "pu",
    "butt_up": "buttUp",
}
LEGACY_ALIASES = {legacy: name for name, legacy in FIRST_VERSION.items()}


def path_for(name: str) -> Path:
    return LIBRARY_DIR / f"{name}.json"


def resolve(name: str) -> str:
    """Kilo name for `name`, which may also be a legacy token (`up`, `wsf`)."""
    if name in FIRST_VERSION:
        return name
    if name in LEGACY_ALIASES:
        return LEGACY_ALIASES[name]
    raise KeyError(f"no trick {name!r} in the library")


def load(name: str) -> Trick:
    return Trick.from_json(path_for(resolve(name)).read_text())


def names() -> list[str]:
    return list(FIRST_VERSION)


def load_all() -> dict[str, Trick]:
    return {n: load(n) for n in FIRST_VERSION}
