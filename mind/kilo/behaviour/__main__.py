"""Run the behaviour runner against a body.

    python -m kilo.behaviour --vbody --seconds 120        # in-process virtual body
    python -m kilo.behaviour --port /dev/pts/5            # `kilo vbody` on a pty, or the real body
    python -m kilo.behaviour --port /dev/ttyAMA0 --poweroff-cmd 'sudo systemctl poweroff'

Ctrl-C stops; SIGTERM asks for a clean wind-down (rest, then poweroff).
The poweroff command only runs when given (it is the daemon's job later).
"""

from __future__ import annotations

import argparse
import asyncio
import shlex
import signal
import subprocess
import sys

from .adapter import BodyAdapter
from .personality import PersonalityError, load
from .runner import PerceptFeed, Runner


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m kilo.behaviour", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port or pty of the body")
    src.add_argument("--vbody", action="store_true", help="run an in-process virtual body")
    ap.add_argument("--personality", help="personality YAML (default: the packaged one)")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--seed", type=int, help="RNG seed (default: random, logged)")
    ap.add_argument("--log", help="behaviour log path (default ~/kilo-logs/behaviour-<time>.jsonl)")
    ap.add_argument("--poweroff-cmd", help="command run when the wind-down completes")
    ap.add_argument("-v", "--verbose", action="store_true", help="print each choice change")
    args = ap.parse_args(argv)

    try:
        cfg = load(args.personality)
    except PersonalityError as e:
        print(f"personality: {e}", file=sys.stderr)
        return 2

    from kilo.body.client import BodyClient
    from kilo.link.serial_link import SerialLink

    vb = None
    if args.vbody:
        from kilo.link.transport import loopback_pair
        from kilo.vbody.sim import VirtualBody

        a, b = loopback_pair()
        vb = VirtualBody(b).start()
        link = SerialLink(lambda: a, port_name="vbody")
    else:
        from kilo.link.transport import open_port

        link = SerialLink(lambda: open_port(args.port), port_name=args.port)
    link.mark_alive()
    link.start()
    client = BodyClient(link)
    try:
        client.wait_ready(3.0)
        client.set_stream(telem_div=2)
        feed = PerceptFeed(client.state)  # kilo.perception, as under the daemon
        runner = Runner(BodyAdapter(client, percepts=feed.percepts), cfg, percepts=feed, seed=args.seed,
                        log_path=args.log,
                        on_poweroff=(lambda: subprocess.run(shlex.split(args.poweroff_cmd), check=False))
                        if args.poweroff_cmd else None)
        print(f"behaviour log: {runner.log_path} (seed {runner.seed})", file=sys.stderr)
        return asyncio.run(_run(runner, args))
    finally:
        link.stop()
        if vb is not None:
            vb.stop()


async def _run(runner: Runner, args: argparse.Namespace) -> int:
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGINT, runner.stop)
    loop.add_signal_handler(signal.SIGTERM, runner.request_shutdown)
    if args.verbose:
        async def watch() -> None:
            last = None
            while True:
                r = runner.last_record
                if r and r["chosen"] != last:
                    last = r["chosen"]
                    print(f"{r['t']:.1f} {last} ({r['reason']}) drives={r['drives']}", file=sys.stderr)
                await asyncio.sleep(0.2)
        w = asyncio.ensure_future(watch())
    why = await runner.run(args.seconds)
    if args.verbose:
        w.cancel()
    print(f"stopped: {why} after {runner.ticks} ticks", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
