"""`kilo flash`: build and flash the body over the same UART (Appendix F).

Sequence:

1. Build (optional): ``arduino-cli compile --fqbn arduino:avr:uno --output-dir
   DIR --build-property compiler.cpp.extra_flags=-DKILO_BUILD=0x<git hash>
   body/``, then ``tools/size.sh`` (the size/symbol harness) if present; stop on
   failure. Building happens *before* the link is stopped, so the body stays
   supervised during the 1-3 min compile on a Pi 2.
2. Stop the serial link (the in-process ``SerialLink``, if given). The daemon
   lock protocol (``/run/kilo/serial.lock``) arrives with the daemon; meanwhile
   pyserial's ``exclusive=True`` makes a second kilo process fail to open the
   port rather than share it.
3. Reset into optiboot:
   - ``dtr``: USB adapter (CH340/CH343). ``avrdude -c arduino`` pulses DTR itself.
   - ``gpio``: Pi header. GPIO4 is pulsed low (emulated open-drain, see
     ``kilo.tools.gpio``) right before avrdude starts; optiboot waits ≈1 s.
   - ``auto``: ``gpio`` for the Pi's UART (ttyAMA*, ttyS*, serial0), else ``dtr``.
4. ``avrdude -c arduino -p atmega328p -P PORT -b 115200 -D -U flash:w:HEX:i``,
   retried once (with a fresh reset).
5. Restart the link and, for new firmware, expect HELLO with reset cause
   FLASHED within 3 s (verification by build hash: TODO(protocol), once
   fw_build semantics are fixed in spec.toml).

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
    reset_ms: float = 100.0
    size_check: bool = True
    verify: bool = True
    dry_run: bool = False
    avrdude: str | None = None
    avrdude_conf: str | None = None
    retries: int = 1


@dataclass
class FlashResult:
    hex_path: Path
    commands: list[list[str]] = field(default_factory=list)
    reset: str = ""
    attempts: int = 0
    verified: bool | None = None


def git_hash(repo: Path = REPO_DIR) -> str | None:
    try:
        out = subprocess.run(["git", "-C", str(repo), "rev-parse", "--short=8", "HEAD"],
                             capture_output=True, text=True, check=True, timeout=10)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def compile_cmd(sketch: Path, out_dir: Path, build_hash: str | None) -> list[str]:
    cmd = ["arduino-cli", "compile", "--fqbn", FQBN, "--output-dir", str(out_dir)]
    if build_hash:
        cmd += ["--build-property", f"compiler.cpp.extra_flags=-DKILO_BUILD=0x{build_hash}"]
    return cmd + [str(sketch)]


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
    verify: Callable[[float], bool] | None = None,
    say: Callable[[str], None] = print,
) -> FlashResult:
    """Run the flash sequence. ``link`` (a SerialLink) is stopped and restarted.

    ``verify(since)`` is called after the link restarts for new firmware and
    should return True once a HELLO with reset cause FLASHED arrived after
    ``since`` (see ``verify_hello``).
    """
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
        sh(compile_cmd(opts.sketch, opts.build_dir, git_hash()), "arduino-cli compile")
        size_sh = REPO_DIR / "tools" / "size.sh"
        if opts.size_check:
            if size_sh.exists():
                sh([str(size_sh)], "size/symbol harness")
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
        # 5. restart the link whatever happened
        if was_running:
            say("restarting serial link")
            link.start()
    if opts.verify and not opts.legacy and not opts.dry_run and verify is not None:
        res.verified = verify(t_flash)
        say("verified: HELLO reset_cause=FLASHED" if res.verified
            else "warning: no HELLO with reset_cause=FLASHED within 3 s")
    return res


def verify_hello(link: Any, since: float, timeout: float = 3.0) -> bool:
    """True once ``link`` saw a HELLO with reset cause FLASHED after ``since``."""
    from kilo.link import consts

    flashed = [k for k, v in consts.RESET_CAUSES.items() if v == "FLASHED"][0]

    def ok() -> bool:
        st = link.state
        return (st.hello is not None and st.hello_at is not None and st.hello_at >= since
                and st.hello.reset_cause == flashed)
    # TODO(protocol): also compare HELLO.fw_build with the KILO_BUILD hash once
    # spec.toml fixes how the git hash maps into fw_build u32.
    return link.wait_until(ok, timeout)
