"""`kilo flash`: build and flash the body over the same UART (Appendix F).

Sequence:

1. Build (optional): ``arduino-cli compile --fqbn arduino:avr:uno --output-dir
   DIR --build-property "compiler.cpp.extra_flags=-mcall-prologues
   -DKILO_BUILD=0x<git hash>" --build-property
   compiler.c.extra_flags=-mcall-prologues --build-property
   compiler.c.elf.extra_flags=-Wl,--relax body/``, then ``body/tools/size_check.sh`` (the size/symbol harness) if present; stop on
   failure. Building happens *before* the link is stopped, so the body stays
   supervised during the 1-3 min compile on a Pi 2.
2. Stop the serial link: the in-process ``SerialLink``, or the daemon's (the
   ``link`` argument is then a port lease on the daemon socket, see
   kilo.cli). pyserial's ``exclusive=True`` (flock) makes any other kilo
   process fail to open the port rather than share it, and ``kilo flash``
   checks that lock itself (``port_busy``, the ``precheck``) right before
   avrdude: a daemon it could not reach (``--direct``, another user's, root's)
   still holds it, so the flash is refused unless ``--force``.
3. Reset into optiboot:
   - ``dtr``: USB adapter (CH340/CH343). ``avrdude -c arduino`` pulses DTR itself.
   - ``gpio``: Pi header. GPIO4 is pulsed low (emulated open-drain, see
     ``kilo.tools.gpio``) right before avrdude starts; optiboot waits ≈1 s.
   - ``auto``: ``gpio`` for the Pi's UART (ttyAMA*, ttyS*, serial0), else ``dtr``.
4. ``avrdude -c arduino -p atmega328p -P PORT -b 115200 -D -U flash:w:HEX:i``,
   retried once (with a fresh reset).
5. Settle the port (``kilo.link.transport.settle_port``): a few production
   opens (pyserial at the link baud, DTR/RTS asserted, HUPCL cleared). avrdude
   drops DTR on exit, re-enables HUPCL and leaves the line at 115200; the
   first settle open makes every transition at once (the one post-flash
   reset), the others absorb whatever else the driver does, so the verify
   open and every later ``kilo`` command find the line unchanged and do not
   reset the body.
6. Restart the link and, for new firmware, expect a HELLO after the flash
   (verified twice before giving up) within ``verify_timeout`` (8 s: reset, optiboot's wait, DMP init and the
   first HELLO take ≈2.2 s on Kilo, plus the link restart) with reset cause
   FLASHED (any cause over a USB adapter, where the settle reset makes it
   WARM_UNEXPECTED), and (when this run built the firmware) HELLO.fw_build
   equal to the build hash: the first 8 hex digits of the git commit, passed
   as ``-DKILO_BUILD=0x<hash>`` and reported by the body as a u32.

With a daemon running (``kilo run``), step 2 asks it to release the port
(``release_port`` with a lease) and step 6 has it reacquire the port and
verify on its link.

``--legacy`` flashes ``mind/firmware/opencat-legacy.hex`` (token protocol at
115200), skipping the build and the HELLO check.
"""

from __future__ import annotations

import glob
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

from kilo.link.transport import DEFAULT_BAUD, describe_settle, settle_port
from kilo.protocol.kilo_protocol import RESET_PULSE_MS
from kilo.tools.gpio import ResetBackend, pick_backend

MIND_DIR = Path(__file__).resolve().parents[2]
REPO_DIR = MIND_DIR.parent
FIRMWARE_DIR = MIND_DIR / "firmware"
LEGACY_HEX = FIRMWARE_DIR / "opencat-legacy.hex"
DEFAULT_SKETCH = REPO_DIR / "body"
DEFAULT_BUILD_DIR = Path.home() / ".cache" / "kilo-build"
FQBN = "arduino:avr:uno"
PI_UART_PREFIXES = ("/dev/ttyAMA", "/dev/ttyS", "/dev/serial")


class FlashError(Exception):
    pass


@dataclass
class FlashOptions:
    port: str
    build: bool = True
    legacy: bool = False
    hex_path: Path | None = None
    sketch: Path = DEFAULT_SKETCH
    build_dir: Path = DEFAULT_BUILD_DIR
    reset: str = "auto"  # auto | gpio | dtr | none
    reset_ms: float = float(RESET_PULSE_MS)
    size_check: bool = True
    verify: bool = True
    dry_run: bool = False
    avrdude: str | None = None
    avrdude_conf: str | None = None
    retries: int = 1
    verify_timeout: float = 8.0
    settle_cycles: int = 3
    baud: int = DEFAULT_BAUD
    verbose: bool = False


@dataclass
class FlashResult:
    hex_path: Path
    commands: list[list[str]] = field(default_factory=list)
    reset: str = ""
    attempts: int = 0
    verified: bool | None = None
    hello: dict[str, Any] | None = None  # the HELLO the verify saw (any cause), if any
    settle: list[Any] = field(default_factory=list)
    expected_build: int | None = None  # HELLO.fw_build to expect, if this run built it


def git_hash(repo: Path = REPO_DIR) -> str | None:
    try:
        out = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True, timeout=10)
        return out.stdout.strip()[:8] or None  # 8 hex digits = HELLO.fw_build u32
    except (OSError, subprocess.SubprocessError):
        return None


# Size flags for the body build (doc §3h): -mcall-prologues shares the
# register save/restore code between functions, --relax turns call/jmp into
# rcall/rjmp where they reach (≈420 B of flash, RAM unchanged). The same
# flags are in body/tools/size_check.sh.
CFLAGS = "-mcall-prologues"
LDFLAGS = "-Wl,--relax"


def compile_cmd(sketch: Path, out_dir: Path, build_hash: str | None) -> list[str]:
    cpp = CFLAGS + (f" -DKILO_BUILD=0x{build_hash}" if build_hash else "")
    return ["arduino-cli", "compile", "--fqbn", FQBN, "--output-dir", str(out_dir),
            "--build-property", f"compiler.cpp.extra_flags={cpp}",
            "--build-property", f"compiler.c.extra_flags={CFLAGS}",
            "--build-property", f"compiler.c.elf.extra_flags={LDFLAGS}",
            str(sketch)]


def find_avrdude() -> tuple[str, str | None]:
    """avrdude on PATH, else the copy arduino-cli installs (with its config)."""
    p = shutil.which("avrdude")
    if p:
        return p, None
    for exe in sorted(glob.glob(os.path.expanduser(
            "~/.arduino15/packages/arduino/tools/avrdude/*/bin/avrdude")), reverse=True):
        conf = Path(exe).parent.parent / "etc" / "avrdude.conf"
        return exe, str(conf) if conf.exists() else None
    raise FlashError("avrdude not found (apt install avrdude, or arduino-cli core install arduino:avr)")


def avrdude_cmd(port: str, hex_path: Path, avrdude: str = "avrdude", conf: str | None = None) -> list[str]:
    cmd = [avrdude]
    if conf:
        cmd += ["-C", conf]
    return cmd + ["-c", "arduino", "-p", "atmega328p", "-P", port, "-b", "115200", "-D",
                  "-U", f"flash:w:{hex_path}:i"]


def is_usb_port(port: str | None) -> bool:
    """A USB adapter (DTR wired to RESET), as opposed to the Pi header UART.
    No port (a daemon that has not found one yet) counts as USB."""
    return not (port or "").startswith(PI_UART_PREFIXES)


def port_busy(port: str) -> str | None:
    """Why ``port`` is in use, or None. Takes (and drops) the same flock that
    pyserial's ``exclusive=True`` takes, so a ``kilo run`` daemon or another
    kilo command holding the port shows up whoever runs it. A port that does
    not exist is not busy (avrdude reports it)."""
    import errno
    import fcntl

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except FileNotFoundError:
        return None
    except OSError as e:
        return f"cannot open it: {e}" if e.errno == errno.EBUSY else None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
        if e.errno in (errno.EWOULDBLOCK, errno.EAGAIN, errno.EACCES):
            return "another process holds its exclusive lock"
        return None
    else:
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)
    return None


def reset_method(port: str, requested: str = "auto") -> str:
    if requested != "auto":
        return requested
    return "gpio" if port.startswith(PI_UART_PREFIXES) else "dtr"


def flash(
    opts: FlashOptions,
    *,
    link: Any = None,
    run: Callable[..., Any] = subprocess.run,
    reset_backend: ResetBackend | None = None,
    verify: Callable[[float, int | None, float], Any] | None = None,
    say: Callable[[str], None] = print,
    settle: Callable[[str], Any] | None = None,
    precheck: Callable[[], None] | None = None,
) -> FlashResult:
    """Run the flash sequence. ``link`` (a SerialLink, or anything with
    ``running``/``stop()``/``start()``) is stopped and restarted.

    ``settle(port)`` runs after avrdude, before the link restarts (default
    ``settle_port`` with ``opts.settle_cycles`` and ``opts.baud``).

    ``precheck()`` runs after the link is stopped, right before the first
    avrdude (a FlashError from it aborts, the link is restarted).

    ``verify(since, expected_build, timeout)`` is called after the link
    restarts for new firmware; it returns a ``VerifyResult`` (or a bool),
    true once a HELLO with the right cause (and fw_build == expected_build,
    unless None) arrived after ``since`` (see ``verify_hello``). A failed
    verify is retried once; ``res.verified`` is then False (the caller exits
    non-zero) and ``res.hello`` the last HELLO seen.
    """
    if settle is None:
        def settle(port: str) -> Any:
            return settle_port(port, opts.baud, cycles=opts.settle_cycles)
    if opts.legacy:
        hex_path = opts.hex_path or LEGACY_HEX
        build = False
    else:
        build = opts.build and opts.hex_path is None
        hex_path = opts.hex_path or (opts.build_dir / f"{opts.sketch.name}.ino.hex")
    res = FlashResult(hex_path=hex_path)

    def sh(cmd: Sequence[str], what: str) -> None:
        res.commands.append(list(cmd))
        say("$ " + " ".join(cmd))
        if opts.dry_run:
            return
        try:
            run(list(cmd), check=True)
        except subprocess.CalledProcessError as e:
            raise FlashError(f"{what} failed (exit {e.returncode})") from None
        except OSError as e:
            raise FlashError(f"{what}: {e}") from None

    # 1. build + size harness (link still running)
    if build:
        if not (opts.sketch / f"{opts.sketch.name}.ino").exists() and not opts.dry_run:
            raise FlashError(f"no sketch at {opts.sketch} (use --sketch or --no-build --hex)")
        opts.build_dir.mkdir(parents=True, exist_ok=True)
        build_hash = git_hash()
        if build_hash:
            res.expected_build = int(build_hash, 16)
        sh(compile_cmd(opts.sketch, opts.build_dir, build_hash), "arduino-cli compile")
        size_sh = REPO_DIR / "body" / "tools" / "size_check.sh"
        if opts.size_check:
            if size_sh.exists():
                # Check the ELF that was just built (no second compile on the Pi).
                sh([str(size_sh), "--elf", str(opts.build_dir / f"{opts.sketch.name}.ino.elf")],
                   "size/symbol harness")
            else:
                say(f"warning: {size_sh} not found, size/symbol harness skipped")
    if not hex_path.exists() and not opts.dry_run:
        raise FlashError(f"hex file not found: {hex_path}")

    avr, conf = (opts.avrdude, opts.avrdude_conf) if opts.avrdude else (
        ("avrdude", None) if opts.dry_run else find_avrdude())
    res.reset = reset_method(opts.port, opts.reset)
    backend = None
    if res.reset == "gpio" and not opts.dry_run:
        backend = reset_backend or pick_backend()
        say(f"reset: GPIO4 via {backend.name}, {opts.reset_ms:.0f} ms low")
    elif res.reset == "dtr":
        say("reset: DTR (avrdude)")

    # 2. release the port
    was_running = bool(link is not None and getattr(link, "running", False))
    if was_running:
        say("stopping serial link")
        link.stop()
    if precheck is not None and not opts.dry_run:
        try:
            precheck()
        except FlashError:
            if was_running:  # nothing was flashed: no settle, just give the port back
                say("restarting serial link")
                link.start()
            raise
    t_flash = time.monotonic()
    try:
        last: FlashError | None = None
        for attempt in range(opts.retries + 1):
            res.attempts = attempt + 1
            # 3. reset, then 4. avrdude within ~200 ms
            if backend is not None:
                backend.pulse(opts.reset_ms)
            elif res.reset == "gpio":
                res.commands.append(["# GPIO4 reset pulse"])
                say(f"# GPIO4 low for {opts.reset_ms:.0f} ms, then input (dry run)")
            try:
                sh(avrdude_cmd(opts.port, hex_path, avr, conf), "avrdude")
                last = None
                break
            except FlashError as e:
                last = e
                say(f"{e}; " + ("retrying" if attempt < opts.retries else "giving up"))
        if last is not None:
            raise last
    finally:
        # 5. settle DTR/HUPCL/baud so that no later open resets the body
        if not opts.dry_run:
            res.settle = list(settle(opts.port) or [])
            _report_settle(res.settle, opts, say)
        # 6. restart the link whatever happened
        if was_running:
            say("restarting serial link")
            link.start()
    if opts.verify and not opts.legacy and not opts.dry_run and verify is not None:
        cause = "any reset cause" if is_usb_port(opts.port) else "reset_cause=FLASHED"
        want = cause + ("" if res.expected_build is None else f" fw_build={res.expected_build:08x}")
        r = verify(t_flash, res.expected_build, opts.verify_timeout)
        res.hello = getattr(r, "hello", None)
        if not r:
            seen = describe_hello(res.hello)
            say(f"no HELLO with {want} within {opts.verify_timeout:g} s"
                + (f" (last HELLO seen: {seen})" if seen else "") + "; verifying once more")
            r = verify(t_flash, res.expected_build, opts.verify_timeout)
            res.hello = getattr(r, "hello", None) or res.hello
        res.verified = bool(r)
        seen = describe_hello(res.hello)
        if res.verified:
            say(f"verified: HELLO {want}" + (f" ({seen})" if seen else ""))
        else:
            say(f"verify FAILED: no HELLO with {want} within 2 x {opts.verify_timeout:g} s; "
                + (f"last HELLO seen: {seen}" if seen else "no HELLO seen at all"))
    return res


def _report_settle(cycles: list[Any], opts: FlashOptions, say: Callable[[str], None]) -> None:
    if not cycles:
        return
    errs = [c for c in cycles if getattr(c, "error", None)]
    hup = [str(c.cycle) for c in cycles if getattr(c, "hupcl_was_set", None)]
    say(f"settled {opts.port}: {len(cycles)} open/close cycles at {opts.baud}"
        + (f", HUPCL was set on cycle {','.join(hup)}" if hup else "")
        + (f", {len(errs)} failed" if errs else ""))
    if opts.verbose or errs:
        for line in describe_settle(cycles).splitlines():
            say("  " + line)


def describe_hello(h: dict[str, Any] | None) -> str:
    if not h:
        return ""
    from kilo.link import consts

    cause = consts.RESET_CAUSES.get(h.get("reset_cause"), h.get("reset_cause"))
    build = h.get("fw_build")
    return (f"build {build:08x}" if isinstance(build, int) else f"build {build}") + f" reset {cause}"


@dataclass
class VerifyResult:
    ok: bool
    hello: dict[str, Any] | None = None  # the last HELLO seen after ``since`` (any cause)

    def __bool__(self) -> bool:
        return self.ok


def verify_hello(link: Any, since: float, expected_build: int | None = None,
                 timeout: float = 8.0, any_cause: bool = False, stamp: bool = False) -> VerifyResult:
    """Wait up to ``timeout`` for a HELLO received after ``since`` with reset
    cause FLASHED (and fw_build == ``expected_build`` unless that is None).

    ``any_cause``: accept any reset cause. Over a USB adapter the settle (or
    the verify open) pulses DTR and resets the body once more, so the HELLO
    says WARM_UNEXPECTED rather than FLASHED; the build hash still proves
    what is running. Whatever happens, the result carries the last HELLO
    seen after ``since`` so a failure can say what the body did report.
    ``stamp``: the caller is the mind's main loop (the CLI's own link), so
    waiting stamps the link's liveness; the daemon's pool thread does not."""
    from kilo.link.messages import fields_of
    from kilo.protocol.kilo_protocol import ResetCause

    def fresh() -> Any:
        st = link.state
        h = st.hello
        return h if h is not None and st.hello_at is not None and st.hello_at >= since else None

    def ok() -> bool:
        h = fresh()
        return (h is not None and (any_cause or h.reset_cause == ResetCause.FLASHED)
                and (expected_build is None or h.fw_build == expected_build))
    # Wait in slices; if no fresh HELLO came (the boot HELLO went out before
    # the port was open, the on-connect HELLO_REQ hit optiboot), ask for one.
    deadline = time.monotonic() + timeout
    good = False
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            break
        if link.wait_until(ok, min(1.0, left), alive=stamp):
            good = True
            break
        if fresh() is None and getattr(link.state, "alive", False) and hasattr(link, "request"):
            try:
                link.request("HELLO_REQ", timeout=0.3)
            except Exception:
                pass
    h = fresh()
    return VerifyResult(good, None if h is None else fields_of(h))
