"""Regenerate tricks/library/*.json from the legacy skill tables.

    python -m kilo.tricks.import_legacy [--check] [--source src/InstinctNybble.h]

Idempotent: the output depends only on the source file. With --check nothing
is written and the exit status is 1 if any file would change.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import legacy
from .library import FIRST_VERSION, LIBRARY_DIR, path_for


def build(source: Path = legacy.DEFAULT_PATH) -> dict[Path, str]:
    """{path: JSON text} for every first-version trick, validated."""
    skills = legacy.parse_file(source)
    out: dict[Path, str] = {}
    for name, legacy_name in FIRST_VERSION.items():
        trick = legacy.to_trick(skills[legacy_name], name=name, skills=skills).validate()
        out[path_for(name)] = trick.to_json()
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Regenerate the trick library from legacy skills.")
    ap.add_argument("--source", type=Path, default=legacy.DEFAULT_PATH)
    ap.add_argument("--check", action="store_true", help="only report stale files")
    args = ap.parse_args(argv)
    files = build(args.source)
    stale = [p for p, text in files.items() if not p.exists() or p.read_text() != text]
    if args.check:
        for p in stale:
            print(f"stale: {p.name}")
        return 1 if stale else 0
    LIBRARY_DIR.mkdir(exist_ok=True)
    for p in stale:
        p.write_text(files[p])
        print(f"wrote {p.name}")
    print(f"{len(files)} tricks, {len(stale)} changed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
