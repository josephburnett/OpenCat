#!/usr/bin/env python3
"""Check the docs' copies of the spec's shared defaults and limits.

The prose stays hand-written; this only finds the numbers in it and fails
when they drift from protocol/spec.toml ([gait], [gait_patterns.*],
[[joints]], [balance], [head]):

  docs/architecture.md  §3b joint table (limits), §3d gait parameter and
                        phase-offset tables, head bob / tail wag defaults,
                        §3f balance defaults and limit
  mind/README.md        "Walking" (the per-pattern defaults and body height),
                        the default Kp and look() speed

Usage: python3 protocol/doc_check.py   (exit 1 on drift; protocol/test.sh runs it)
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(HERE))
from gen import load_spec  # noqa: E402

ARCH = REPO / "docs" / "architecture.md"
README = REPO / "mind" / "README.md"


def section(text: str, heading: str) -> str:
    """The text from the line starting with `heading` to the next heading of the same level."""
    m = re.search(rf"^{re.escape(heading)}.*$", text, re.M)
    if not m:
        raise SystemExit(f"doc_check: heading {heading!r} not found")
    level = heading.split(" ")[0]
    nxt = re.compile(rf"^{'#' * len(level)} ", re.M).search(text, m.end())
    return text[m.start():nxt.start() if nxt else len(text)]


def flat(text: str) -> str:
    """Whitespace collapsed (prose wraps anywhere)."""
    return re.sub(r"\s+", " ", text)


class Checker:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.checks = 0

    def find(self, where: str, pattern: str, text: str) -> re.Match | None:
        m = re.search(pattern, text, re.M)
        if not m:
            self.errors.append(f"{where}: pattern not found (rewritten? update protocol/doc_check.py): {pattern}")
        return m

    def eq(self, where: str, doc, spec) -> None:
        self.checks += 1
        if doc != spec:
            self.errors.append(f"{where}: docs say {doc}, spec says {spec}")


def frac(v256: int, digits: int) -> str:
    return f"{v256 / 256:.{digits}f}"


def check_arch(c: Checker, spec: dict, text: str) -> None:
    pats = spec["gait_patterns"]
    names = list(pats)  # enum order: WALK, TROT, CRAWL

    # §3b joint table: | id | name | ch | dir | mid | lo..hi |
    s3b = section(text, "### 3b.")
    for j in spec["joints"]:
        if not j["driven"]:
            continue
        m = c.find(f"§3b joint {j['id']}", rf"^\| {j['id']} \|[^|]*\|[^|]*\|[^|]*\|[^|]*\| (-?\d+)\.\.(-?\d+) \|", s3b)
        if m:
            c.eq(f"§3b joint {j['id']} ({j['name']}) limits", (int(m[1]), int(m[2])), (j["lo"], j["hi"]))

    # §3d parameter table, columns walk / trot / crawl.
    s3d = section(text, "### 3d.")
    c.find("§3d table header", r"\| Walk / trot / crawl \|", s3d)
    for k in ("period_ms", "duty", "stride_mm", "step_h_mm"):
        m = c.find(f"§3d {k}", rf"^\| `{k}` \|[^|]*\|[^|]*\| (\d+) / (\d+) / (\d+)", s3d)
        if m:
            c.eq(f"§3d {k} (walk / trot / crawl)", tuple(int(x) for x in m.groups()),
                 tuple(pats[n][k] for n in names))
    m = c.find("§3d body_h_mm", r"^\| `body_h_mm` \|[^|]*\|[^|]*\| ([\d.]+) / ([\d.]+) / ([\d.]+) ×", s3d)
    if m:
        c.eq("§3d body_h_mm fraction (walk / trot / crawl)", m.groups(),
             tuple(frac(pats[n]["body_h_256"], 2) for n in names))
    # Phase offsets: | WALK | 0.50 | 0.00 | 0.25 | 0.75 | ..., or "as WALK".
    for n in names:
        m = c.find(f"§3d offsets {n}", rf"^\| {n} \| ([^|]*)\|([^|]*)\|([^|]*)\|([^|]*)\|", s3d)
        if not m:
            continue
        cells = [x.strip() for x in m.groups()]
        if cells[0].startswith("as "):
            other = cells[0][3:]
            c.eq(f"§3d offsets {n} ('as {other}')", tuple(pats[n]["offsets"]), tuple(pats.get(other, {}).get("offsets", ())))
        else:
            c.eq(f"§3d offsets {n} (LF RF RB LB)", tuple(cells), tuple(frac(o, 2) for o in pats[n]["offsets"]))
    f3d = flat(s3d)
    g = spec["gait"]
    m = c.find("§3d head bob", r"`A_bob·sin\(2·2πφ\)` \(default (\d+)°\)", f3d)
    if m:
        c.eq("§3d head bob default", int(m[1]), g["bob_deg"])
    m = c.find("§3d tail wag", r"`A_wag·sin 2πφ` \(default (\d+)°\)", f3d)
    if m:
        c.eq("§3d tail wag default", int(m[1]), g["wag_deg"])

    # §3f balance defaults.
    b = spec["balance"]
    f3f = flat(section(text, "### 3f."))
    m = c.find("§3f defaults", r"Kp = ([\d.]+) \(`kp_q8` (\d+)\), Kd = ([\d.]+) s \(`kd_ms` (\d+)\), DB (\d+)°, clamp (\d+)°", f3f)
    if m:
        c.eq("§3f Kp", float(m[1]), b["kp_q8"] / 256)
        c.eq("§3f kp_q8", int(m[2]), b["kp_q8"])
        c.eq("§3f Kd", float(m[3]), b["kd_ms"] / 1000)
        c.eq("§3f kd_ms", int(m[4]), b["kd_ms"])
        c.eq("§3f dead band", int(m[5]), b["deadband_deg"])
        c.eq("§3f clamp", int(m[6]), b["max_corr_deg"])
    m = c.find("§3f controller DB", r"DB default (\d+)°", f3f)
    if m:
        c.eq("§3f controller block DB", int(m[1]), b["deadband_deg"])
    m = c.find("§3f controller clamp", r"1°/tick\), ±(\d+)°\)", f3f)
    if m:
        c.eq("§3f controller block clamp", int(m[1]), b["max_corr_deg"])
    m = c.find("§3f limit", r"dead band and clamp ≤ (\d+)°", f3f)
    if m:
        c.eq("§3f dead band / clamp limit", int(m[1]), b["limit_deg"])


def check_readme(c: Checker, spec: dict, text: str) -> None:
    pats = spec["gait_patterns"]
    walk, trot, crawl = (pats[n] for n in ("WALK", "TROT", "CRAWL"))
    w = flat(section(text, "### Walking"))
    m = c.find("README Walking defaults",
               r"walk (\d+) ms / duty (\d+) / (\d+) mm / step (\d+) mm, trot (\d+) / (\d+) / (\d+) / (\d+), "
               r"crawl (\d+) / (\d+) / (\d+) / (\d+)", w)
    if m:
        keys = ("period_ms", "duty", "stride_mm", "step_h_mm")
        c.eq("README Walking defaults (walk, trot, crawl)", tuple(int(x) for x in m.groups()),
             tuple(p[k] for p in (walk, trot, crawl) for k in keys))
    m = c.find("README Walking body height", r"defaults to ([\d.]+) \(crawl ([\d.]+)\) ×", w)
    if m:
        c.eq("README Walking body height (walk and trot, crawl)", m.groups(),
             (frac(walk["body_h_256"], 1), frac(crawl["body_h_256"], 1)))
        c.eq("README Walking body height (trot = walk)", frac(trot["body_h_256"], 1), frac(walk["body_h_256"], 1))
    f = flat(text)
    m = c.find("README default Kp", r"with the given Kp \(default ([\d.]+), the firmware's", f)
    if m:
        c.eq("README default Kp", float(m[1]), spec["balance"]["kp_q8"] / 256)
    m = c.find("README look() speed", r"tail_deg=None, speed_dps=(\d+)\)", f)
    if m:
        c.eq("README look() speed_dps", int(m[1]), spec["head"]["speed_dps"])


def main() -> int:
    spec = load_spec()
    c = Checker()
    check_arch(c, spec, ARCH.read_text())
    check_readme(c, spec, README.read_text())
    if c.errors:
        print("docs disagree with protocol/spec.toml:", *c.errors, sep="\n  ")
        return 1
    print(f"docs agree with the spec's shared defaults ({c.checks} checks)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
