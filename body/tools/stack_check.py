#!/usr/bin/env python3
"""Static worst-case stack estimate for the body firmware (doc §3h).

Inputs: the ELF's disassembly (`avr-objdump -d -C`), the GCC stack-usage
files (`-fstack-usage`; with LTO they come from the link-time ltrans step),
and the globals size. Output: the deepest call path from main() with its
per-frame sizes, the worst interrupt on top of it, the total, and the margin
2048 - globals - total. Exit status 1 if the margin is below --min-margin.

Model (what is and is not counted):
  * A frame is the .su "static" size of the function, which on the AVR
    includes the 2 B return address, the saved registers (also the ones
    __prologue_saves__ pushes under -mcall-prologues) and the locals. A
    function without a .su entry (libgcc and other assembler) is estimated
    from its disassembly: 2 B + one byte per `push` before its first `ret`.
  * Edges: direct `call`/`rcall` to another function's entry adds a frame;
    a `jmp`/`rjmp` to another function is a tail call (it reuses the
    caller's return address after the caller's frame is gone), so it costs
    the callee's depth without the caller's frame. `rcall .+0` and branches
    inside a function are not edges. Jumps into __prologue_saves__ /
    __epilogue_restores__ are part of the frame (already in .su).
  * `icall`/`eicall`/`ijmp` (other than the ijmp in __prologue_saves__,
    which returns into its caller) are unknown edges: listed, not followed.
  * Interrupts: an ISR runs with interrupts off unless it executes `sei`;
    the add-on is the deepest single ISR, or the sum along the nesting
    chain if some ISR re-enables interrupts (reported).
  * Recursion: a cycle in the call graph is reported and cut.

Dependency-free (Python 3 standard library).
"""
import argparse
import glob
import re
import sys

RAM = 2048
LABEL = re.compile(r"^([0-9a-f]+) <(.*)>:$")
INSN = re.compile(r"^\s*([0-9a-f]+):\t(?:[0-9a-f]{2} )+\s*\t(\S+)\s*(.*)$")
TARGET = re.compile(r";\s*0x([0-9a-f]+) <(.*?)(\+0x[0-9a-f]+)?>\s*$")
HELPERS = ("__prologue_saves__", "__epilogue_restores__")
# ATmega328P vector numbers (datasheet table 11-6), for the report.
VECTORS = {1: "INT0", 2: "INT1", 3: "PCINT0", 4: "PCINT1", 5: "PCINT2", 6: "WDT", 7: "TIMER2_COMPA",
           8: "TIMER2_COMPB", 9: "TIMER2_OVF", 10: "TIMER1_CAPT", 11: "TIMER1_COMPA", 12: "TIMER1_COMPB",
           13: "TIMER1_OVF", 14: "TIMER0_COMPA", 15: "TIMER0_COMPB", 16: "TIMER0_OVF", 17: "SPI_STC",
           18: "USART_RX", 19: "USART_UDRE", 20: "USART_TX", 21: "ADC", 22: "EE_READY", 23: "ANALOG_COMP",
           24: "TWI", 25: "SPM_READY"}


def base_key(name):
    """Demangled 'ns::f(int) [clone .constprop.58]' or mangled-free 'f.constprop'
    -> ('f', '.constprop'). The .su files name clones without the number."""
    clone = ""
    m = re.search(r"\[clone (\.[a-z_]+)(?:\.\d+)?\]", name)
    if m:
        clone = m.group(1)
        name = name[: m.start()].strip()
    name = name.split("(", 1)[0]
    if "." in name and not name.startswith("."):
        name, rest = name.split(".", 1)
        clone = clone or "." + re.sub(r"\.\d+$", "", rest)
    return name.split("::")[-1], clone


def read_su(paths):
    su = {}
    for p in paths:
        with open(p) as f:
            for line in f:
                parts = line.rstrip("\n").split("\t")
                if len(parts) < 3:
                    continue
                name = parts[0].rsplit(":", 1)[-1]
                key = base_key(name)
                # Same name in two files (static helpers): keep the larger.
                su[key] = max(su.get(key, 0), int(parts[1]))
                if parts[2] != "static":
                    su.setdefault("__dynamic__", []).append(name)
    return su


def parse_disassembly(path):
    funcs = {}  # name -> dict(addr, insns=[(mnemonic, operands)])
    order = []
    cur = None
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            m = LABEL.match(line)
            if m:
                cur = m.group(2)
                funcs[cur] = {"addr": int(m.group(1), 16), "insns": []}
                order.append(cur)
                continue
            if cur is None:
                continue
            m = INSN.match(line)
            if m:
                funcs[cur]["insns"].append((m.group(2), m.group(3)))
    return funcs, order


def analyse(funcs, su):
    frames, estimated = {}, []
    calls, tails, indirect, sei = {}, {}, [], []
    for name, fn in funcs.items():
        c, t = set(), set()
        pushes, seen_ret = 0, False
        for mn, ops in fn["insns"]:
            if mn in ("ret", "reti"):
                seen_ret = True
            if mn == "push" and not seen_ret:
                pushes += 1
            if mn in ("icall", "eicall", "ijmp", "eijmp") and name not in HELPERS:
                indirect.append(name)
            if mn == "sei" and name.startswith("__vector_"):
                sei.append(name)
            if mn in ("call", "rcall", "jmp", "rjmp"):
                m = TARGET.search(ops)
                if not m:
                    continue
                tgt = m.group(2)
                if tgt in HELPERS or tgt == name:
                    continue  # the frame helpers, or a branch inside the function
                (c if mn.endswith("call") else t).add(tgt)
        calls[name], tails[name] = c, t
        key = base_key(name)
        if key in su:
            frames[name] = su[key]
        elif fn["insns"]:
            frames[name] = 2 + pushes
            estimated.append(name)
        else:
            frames[name] = 0  # data in .text (PROGMEM tables)
    return frames, calls, tails, sorted(set(indirect)), sorted(set(sei)), estimated


def deepest(root, frames, calls, tails, cycles):
    """worst(f) = frame(f) + max(worst(callee)) over calls, max(worst(g)) over tail jumps.
    Returns (depth, path) where path is [(name, frame, kind)]."""
    memo = {}

    def go(f, stack):
        if f in memo:
            return memo[f]
        if f in stack:
            cycles.add(" -> ".join(stack[stack.index(f):] + [f]))
            return 0, []
        stack.append(f)
        best, best_path = frames.get(f, 0), [(f, frames.get(f, 0), "call")]
        for g in sorted(calls.get(f, ())):
            d, p = go(g, stack)
            if frames.get(f, 0) + d > best:
                best, best_path = frames.get(f, 0) + d, [(f, frames.get(f, 0), "call")] + p
        for g in sorted(tails.get(f, ())):
            d, p = go(g, stack)
            if d > best:
                best, best_path = d, [(f, 0, "tail-jumps to")] + p
        stack.pop()
        memo[f] = (best, best_path)
        return memo[f]

    return go(root, [])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--disasm", required=True, help="avr-objdump -d -C output")
    ap.add_argument("--su", required=True, nargs="+", help=".su files (globs ok)")
    ap.add_argument("--globals", type=int, required=True, help=".data + .bss + .noinit bytes")
    ap.add_argument("--min-margin", type=int, default=128)
    ap.add_argument("--root", default="main")
    a = ap.parse_args()

    paths = [p for g in a.su for p in (glob.glob(g) or [g])]
    su = read_su(paths)
    dynamic = su.pop("__dynamic__", [])
    funcs, _ = parse_disassembly(a.disasm)
    if a.root not in funcs:
        print(f"stack_check: no function {a.root!r} in the disassembly")
        return 1
    frames, calls, tails, indirect, sei, estimated = analyse(funcs, su)
    if not su:
        print("stack_check: no stack-usage entries (was -fstack-usage given at link time?)")
        return 1

    cycles = set()
    depth, path = deepest(a.root, frames, calls, tails, cycles)
    print(f"deepest path from {a.root}() ({depth} B; frame = .su static size incl. 2 B return address):")
    for name, fr, kind in path:
        tag = " (estimated from pushes)" if name in estimated else ""
        if kind == "call":
            print(f"  {fr:4d} B  {name}{tag}")
        else:
            print(f"     -    {name} {kind}")

    isrs = sorted(f for f in funcs if re.fullmatch(r"__vector_\d+", f))
    print("interrupts (each runs with interrupts off unless it executes sei):")
    isr_depth = {}
    for v in isrs:
        d, p = deepest(v, frames, calls, tails, cycles)
        isr_depth[v] = d
        via = " -> ".join(n for n, _, _ in p[1:])
        vn = VECTORS.get(int(v.rsplit("_", 1)[1]), "?")
        print(f"  {d:4d} B  {v} ({vn}){' via ' + via if via else ''}{'  RE-ENABLES INTERRUPTS' if v in sei else ''}")
    if sei:
        # A re-enabling ISR can be interrupted by every other one (worst case:
        # one of each kind stacked); otherwise only the deepest single ISR.
        isr_add = sum(isr_depth.values())
        print(f"ISR add-on: {isr_add} B (nesting possible: {', '.join(sei)} re-enable interrupts; sum of all)")
    else:
        isr_add = max(isr_depth.values(), default=0)
        print(f"ISR add-on: {isr_add} B (no ISR re-enables interrupts: the deepest single one)")

    if indirect:
        print("indirect calls (not followed; their targets are unknown): " + ", ".join(indirect))
    if cycles:
        print("recursion (cut; depth is per pass): " + "; ".join(sorted(cycles)))
    if dynamic:
        print("dynamic or bounded frames (sizes are a lower bound): " + ", ".join(sorted(dynamic)))
    reached = {n for n, _, _ in path}
    est = [e for e in estimated if e in reached]
    if est:
        print("frames estimated from the disassembly on the deepest path: " + ", ".join(est))

    total = depth + isr_add
    margin = RAM - a.globals - total
    print(f"total: {total} B = {depth} B ({a.root}) + {isr_add} B (ISR)")
    print(f"margin: {margin} B = {RAM} - {a.globals} (globals) - {total} (stack) (minimum {a.min_margin})")
    if margin < a.min_margin:
        print("stack_check: FAIL")
        return 1
    print("stack_check: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
