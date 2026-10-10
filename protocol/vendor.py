#!/usr/bin/env python3
"""Copy the protocol sources into the body and mind trees (byte for byte).

Neither consumer can reach protocol/ at build or run time on its own:
arduino-cli copies the sketch to a build directory before compiling (so a
relative #include of ../../protocol breaks), and the mind is installed as a
Python package. So both vendor exact copies, which this script keeps in step:

  protocol/gen/kilo_protocol.h          -> body/src/proto/
  protocol/c/{cobs,crc16,kilo_frame}.*  -> body/src/proto/
  protocol/gen/kilo_protocol.py         -> mind/kilo/protocol/
  protocol/py/kilo_link_codec.py        -> mind/kilo/protocol/

The copies are never edited by hand. Usage:

  python3 protocol/vendor.py           copy
  python3 protocol/vendor.py --check   exit 1 if any copy differs (protocol/test.sh runs this)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
BODY = REPO / "body" / "src" / "proto"
MIND = REPO / "mind" / "kilo" / "protocol"

COPIES = [
    (HERE / "gen" / "kilo_protocol.h", BODY),
    *((HERE / "c" / f"{n}.{ext}", BODY) for n in ("cobs", "crc16", "kilo_frame") for ext in ("c", "h")),
    (HERE / "gen" / "kilo_protocol.py", MIND),
    (HERE / "py" / "kilo_link_codec.py", MIND),
]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    stale = []
    for src, dst_dir in COPIES:
        dst = dst_dir / src.name
        data = src.read_bytes()
        if not dst.exists() or dst.read_bytes() != data:
            stale.append(dst)
            if not args.check:
                dst_dir.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(data)
    rel = [str(p.relative_to(REPO)) for p in stale]
    if args.check:
        if stale:
            print("vendored protocol copies are stale (run python3 protocol/vendor.py):", *rel, sep="\n  ")
            return 1
        print("vendored protocol copies are up to date")
        return 0
    print("copied:" if rel else "vendored copies already up to date", *rel, sep="\n  " if rel else "")
    return 0


if __name__ == "__main__":
    sys.exit(main())
