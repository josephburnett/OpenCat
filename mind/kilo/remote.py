"""The client end of the daemon socket (docs/architecture.md Appendix G).

``kilo run`` (kilo.daemon) owns the serial port and a BodyClient and serves a
Unix socket speaking JSON lines. Every other ``kilo`` command first tries the
socket (``connect()``); when no daemon listens it opens the port directly.

Wire format, one JSON object per line, UTF-8:

    request   {"id": 7, "op": "param_get", "args": {"id": "WD_LIMP_S"}}
    reply     {"id": 7, "ok": true, "result": {...}}
              {"id": 7, "ok": false, "error": {"type": "CommandError", "message": "...",
                                               "args": ["PARAM_SET", 3]}}
    stream    {"stream": "telemetry" | "snapshot" | "event" | "hello" | "reboot"
                         | "goals_cleared" | "connection" | "dropped", "t": <monotonic>, "data": ...}

Stream messages only follow a ``subscribe`` request and carry no id.

``socket_path()`` is the one resolver both sides use. ``RemoteBody`` has the
BodyClient methods the CLI calls (and proxies any other public BodyClient
method through the generic ``call`` op when its arguments are JSON-able), so
the command functions in kilo.cli run unchanged against either.
"""

from __future__ import annotations

import errno
import itertools
import json
import os
import socket
import threading
import time
from dataclasses import is_dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable

from kilo.body.client import (BALANCE_CLAMP, BALANCE_DEADBAND, BALANCE_KD_S, BALANCE_KP, BodyClient,
                              ProtocolMismatch, TrickError)
from kilo.link.serial_link import CommandError, LinkError, LinkTimeout, NotConnected

DEFAULT_CALL_TIMEOUT_S = 60.0
ALIVE_PERIOD_S = 0.5  # RemoteLink.mark_alive() pings the daemon at most this often


# ---- socket path ------------------------------------------------------------------
RUN_USER_ROOT = "/run/user"


def socket_candidates(explicit: str | os.PathLike | None = None, *, env: dict[str, str] | None = None,
                      euid: int | None = None, home: str | os.PathLike | None = None,
                      is_dir: Callable[[str], bool] = os.path.isdir) -> list[tuple[Path, str]]:
    """Every place a daemon may listen, most specific first, as (path, why).

    ``explicit`` (``--socket``) or ``$KILO_SOCKET`` alone when given;
    ``/run/kilo/kilo.sock`` for root, then (``sudo kilo ...``) the invoking
    user's ``/run/user/$SUDO_UID/kilo/kilo.sock``; else ``$XDG_RUNTIME_DIR/kilo/kilo.sock``,
    then ``/run/user/<uid>/kilo/kilo.sock`` when ``/run/user/<uid>`` exists
    (a non-login SSH shell has no ``$XDG_RUNTIME_DIR``, but the systemd user
    service listens there), then ``~/.kilo/kilo.sock``."""
    if explicit:
        return [(Path(explicit), "--socket")]
    env = os.environ if env is None else env
    if env.get("KILO_SOCKET"):
        return [(Path(env["KILO_SOCKET"]), "$KILO_SOCKET")]
    if euid is None:
        euid = os.geteuid() if hasattr(os, "geteuid") else -1
    if euid == 0:
        out0 = [(Path("/run/kilo/kilo.sock"), "root")]
        sudo_uid = env.get("SUDO_UID", "")
        if sudo_uid.isdigit() and int(sudo_uid) != 0:
            # `sudo kilo flash`: the invoking user's daemon (systemd user service)
            out0.append((Path(f"{RUN_USER_ROOT}/{sudo_uid}") / "kilo" / "kilo.sock",
                         f"{RUN_USER_ROOT}/{sudo_uid} ($SUDO_UID's runtime dir)"))
        return out0
    out: list[tuple[Path, str]] = []
    if env.get("XDG_RUNTIME_DIR"):
        out.append((Path(env["XDG_RUNTIME_DIR"]) / "kilo" / "kilo.sock", "$XDG_RUNTIME_DIR"))
    run_dir = f"{RUN_USER_ROOT}/{euid}"
    if euid >= 0 and is_dir(run_dir):
        out.append((Path(run_dir) / "kilo" / "kilo.sock", f"{run_dir} (systemd user runtime dir)"))
    out.append((Path(home if home is not None else Path.home()) / ".kilo" / "kilo.sock", "~/.kilo"))
    seen: set[Path] = set()
    return [(p, why) for p, why in out if not (p in seen or seen.add(p))]


def socket_path(explicit: str | os.PathLike | None = None, *, env: dict[str, str] | None = None,
                euid: int | None = None, home: str | os.PathLike | None = None,
                is_dir: Callable[[str], bool] = os.path.isdir) -> Path:
    """Where the daemon listens: the first of ``socket_candidates()``. In
    order: ``explicit`` (``--socket``), ``$KILO_SOCKET``, ``/run/kilo/kilo.sock``
    for root, ``$XDG_RUNTIME_DIR/kilo/kilo.sock``, ``/run/user/<uid>/kilo/kilo.sock``
    if ``/run/user/<uid>`` exists, else ``~/.kilo/kilo.sock``."""
    return socket_candidates(explicit, env=env, euid=euid, home=home, is_dir=is_dir)[0][0]


# ---- JSON helpers -------------------------------------------------------------------
def jsonable(obj: Any) -> Any:
    """Messages (generated dataclasses) as dicts, bytes as hex, tuples/sets as lists."""
    if is_dataclass(obj) and not isinstance(obj, type):
        from kilo.link.messages import fields_of

        return jsonable(fields_of(obj))
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, (set, frozenset)):
        return sorted(jsonable(v) for v in obj)
    if isinstance(obj, (bytes, bytearray)):
        return bytes(obj).hex()
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, SimpleNamespace):
        return jsonable(vars(obj))
    return str(obj)


def dumps(obj: Any) -> bytes:
    return (json.dumps(jsonable(obj), separators=(",", ":")) + "\n").encode()


ERROR_TYPES: dict[str, type[BaseException]] = {
    "LinkError": LinkError, "LinkTimeout": LinkTimeout, "NotConnected": NotConnected,
    "ProtocolMismatch": ProtocolMismatch, "ValueError": ValueError, "KeyError": KeyError,
    "TypeError": TypeError, "RuntimeError": RuntimeError, "TrickError": TrickError,
}


def encode_error(e: BaseException) -> dict[str, Any]:
    if isinstance(e, CommandError):
        return {"type": "CommandError", "message": str(e), "args": [e.name, e.status]}
    name = type(e).__name__
    if name not in ERROR_TYPES:
        name = "LinkError" if isinstance(e, LinkError) else "RuntimeError"
    args = [a if isinstance(a, (str, int, float, bool)) or a is None else str(a) for a in e.args]
    return {"type": name, "message": str(e), "args": args}


def decode_error(err: dict[str, Any]) -> BaseException:
    typ = err.get("type", "RuntimeError")
    args = err.get("args") or [err.get("message", "")]
    if typ == "CommandError":
        return CommandError(*args)
    cls = ERROR_TYPES.get(typ, DaemonError)
    try:
        return cls(*args)
    except TypeError:
        return cls(err.get("message", ""))


class DaemonError(Exception):
    """The daemon connection failed or the daemon reported an internal error."""


def restore_snapshot(snap: dict[str, Any]) -> dict[str, Any]:
    """Undo JSON's tuple -> list so a remote snapshot prints like a local one."""
    for k in ("owners", "rates_dps"):
        if isinstance(snap.get(k), list):
            snap[k] = tuple(snap[k])
    if isinstance(snap.get("events"), list):
        snap["events"] = [tuple(e) for e in snap["events"]]
    return snap


# ---- connection -----------------------------------------------------------------------
class DaemonConnection:
    """One socket to the daemon: synchronous ``call()`` from any thread, plus
    stream messages delivered to ``on_stream`` on the reader thread."""

    def __init__(self, sock: socket.socket, path: Path):
        self.sock = sock
        self.path = path
        self._ids = itertools.count(1)
        self._send_lock = threading.Lock()
        self._lock = threading.Lock()
        self._waiting: dict[int, list[Any]] = {}  # id -> [event, reply]
        self._closed = threading.Event()
        self.on_stream: Callable[[dict[str, Any]], None] | None = None
        self._reader = threading.Thread(target=self._read_loop, name="kilo-daemon-rx", daemon=True)
        self._reader.start()
        self.info: dict[str, Any] = {}

    def _read_loop(self) -> None:
        buf = b""
        try:
            while True:
                data = self.sock.recv(65536)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line)
                    except ValueError:
                        continue
                    if "stream" in msg:
                        cb = self.on_stream
                        if cb is not None:
                            try:
                                cb(msg)
                            except Exception:
                                pass
                        continue
                    with self._lock:
                        w = self._waiting.get(msg.get("id"))
                    if w is not None:
                        w[1] = msg
                        w[0].set()
        except OSError:
            pass
        finally:
            self._closed.set()
            with self._lock:
                for w in self._waiting.values():
                    w[0].set()

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def call(self, op: str, timeout: float | None = DEFAULT_CALL_TIMEOUT_S, **args: Any) -> Any:
        """Send ``op`` and wait for its reply. Raises the remote exception
        (LinkTimeout, CommandError, ValueError, ...) or DaemonError."""
        rid = next(self._ids)
        w: list[Any] = [threading.Event(), None]
        with self._lock:
            self._waiting[rid] = w
        try:
            if self.closed:
                raise DaemonError("daemon connection closed")
            try:
                with self._send_lock:
                    self.sock.sendall(dumps({"id": rid, "op": op, "args": args}))
            except OSError as e:
                raise DaemonError(f"daemon connection lost: {e}") from None
            if not w[0].wait(timeout):
                raise DaemonError(f"{op}: no reply from the daemon within {timeout:g} s")
            reply = w[1]
            if reply is None:
                raise DaemonError(f"{op}: daemon connection closed")
        finally:
            with self._lock:
                self._waiting.pop(rid, None)
        if reply.get("ok"):
            return reply.get("result")
        raise decode_error(reply.get("error") or {})

    def subscribe(self, callback: Callable[[dict[str, Any]], None], rate_hz: float = 10.0,
                  kinds: tuple[str, ...] = ("snapshot", "event")) -> Any:
        """Stream ``kinds`` to ``callback(msg)`` (called on the reader thread)."""
        self.on_stream = callback
        return self.call("subscribe", rate_hz=rate_hz, kinds=list(kinds))

    def close(self) -> None:
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        self._reader.join(1.0)

    def __enter__(self) -> "DaemonConnection":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


def connect(path: str | os.PathLike | None = None, timeout: float = 2.0,
            tried: list[tuple[Path, str]] | None = None) -> DaemonConnection | None:
    """Connect to a running daemon; None when nobody listens.

    With ``path`` only that socket is tried; without, every
    ``socket_candidates()`` path in turn (the first daemon that answers wins).
    ``tried`` collects (path, outcome) for each socket looked at."""
    if path is not None:
        return _connect_one(socket_path(path), timeout, tried)
    for p, _why in socket_candidates():
        conn = _connect_one(p, timeout, tried)
        if conn is not None:
            return conn
    return None


def _connect_one(p: Path, timeout: float, tried: list[tuple[Path, str]] | None) -> DaemonConnection | None:
    note = (lambda what: tried.append((p, what))) if tried is not None else (lambda what: None)
    if not p.exists():
        note("no socket file")
        return None
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(str(p))
    except OSError as e:
        s.close()
        if e.errno in (errno.ECONNREFUSED, errno.ENOENT, errno.ENOTSOCK):
            note("stale socket (nobody listens)")
            return None  # stale socket file: no daemon
        note(f"error: {e}")
        raise DaemonError(f"cannot connect to {p}: {e}") from None
    s.settimeout(None)
    conn = DaemonConnection(s, p)
    try:
        conn.info = conn.call("daemon", timeout=timeout) or {}
    except (DaemonError, OSError):
        conn.close()
        note("no answer")
        return None
    note("daemon answered")
    return conn


# ---- BodyClient-shaped proxy ------------------------------------------------------------
class RemoteState:
    def __init__(self, conn: DaemonConnection):
        self._conn = conn
        self.port = conn.info.get("port")

    def snapshot(self) -> dict[str, Any]:
        return restore_snapshot(self._conn.call("snapshot"))

    @property
    def mode(self) -> Any:
        return self.snapshot()["mode"]

    @property
    def epoch(self) -> Any:
        return self.snapshot()["epoch"]


class RemoteLink:
    """The bits of SerialLink the CLI touches: liveness and this session's goals."""

    def __init__(self, conn: DaemonConnection):
        self._conn = conn
        self._last_alive = -1e9
        self.silent = False

    def mark_alive(self) -> None:
        """Keep this client's hold lease fresh (throttled)."""
        now = time.monotonic()
        if now - self._last_alive >= ALIVE_PERIOD_S:
            self._last_alive = now
            self._conn.call("alive")

    def goals(self) -> dict[str, Any]:
        """Goals this client set that the daemon still latches."""
        return {n: None for n in self._conn.call("goals")}

    @staticmethod
    def clock() -> float:
        return time.monotonic()


def _ns(d: Any) -> Any:
    return SimpleNamespace(**d) if isinstance(d, dict) else d


class RemoteBody:
    """BodyClient over the daemon socket. Goals set here are held by the
    daemon while this connection lives (and keeps ``mark_alive``/``spin``
    going); closing it ends the hold."""

    param_info = staticmethod(BodyClient.param_info)
    param_id = staticmethod(BodyClient.param_id)

    def __init__(self, conn: DaemonConnection, override: bool = False):
        self.conn = conn
        self.state = RemoteState(conn)
        self.link = RemoteLink(conn)
        # `kilo --override`: goals and tricks pause the daemon's behaviour
        # runner for this session's hold instead of being refused.
        self.override = override

    def close(self) -> None:
        self.conn.close()

    def spin(self, seconds: float, period: float = 0.05) -> None:
        end = time.monotonic() + seconds
        while True:
            self.link.mark_alive()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(max(period, 0.05), left))

    def hello(self, timeout: float = 0.3) -> Any:
        return _ns(self.conn.call("request", name="HELLO_REQ", timeout_s=timeout))

    def wait_ready(self, timeout: float = 3.0, require_telemetry: bool = True) -> Any:
        return _ns(self.conn.call("wait_ready", timeout=timeout + 10.0, wait_s=timeout,
                                  require_telemetry=require_telemetry))

    # goals
    def posture(self, name: str, transition_ticks: int = 0, balance: bool | None = None) -> None:
        self.conn.call("posture", name=name, transition_ticks=transition_ticks, balance=balance,
                       override=self.override)

    def limp(self) -> None:
        self.posture("limp")

    def rest(self) -> None:
        self.posture("rest")

    def stand(self) -> None:
        self.posture("stand")

    def sit(self) -> None:
        self.posture("sit")

    def balance(self) -> None:
        self.posture("balance")

    def set_balance(self, enable: bool, kp: float = BALANCE_KP, kd_s: float = BALANCE_KD_S,
                    deadband_deg: float = BALANCE_DEADBAND, clamp_deg: float = BALANCE_CLAMP) -> dict[str, int]:
        return self.conn.call("set_balance", enable=enable, kp=kp, kd_s=kd_s, deadband_deg=deadband_deg,
                              clamp_deg=clamp_deg, override=self.override)

    def release(self) -> None:
        self.conn.call("release")

    # gait and body pose (P2; held by this session like a posture)
    def walk(self, stride_mm: float | None = None, turn_mm: float = 0, pattern: str | int = "trot",
             period_ms: int | None = None, duty: int | None = None, step_h_mm: float | None = None,
             body_h_mm: float | None = None, bob: bool = True, wag: bool = True,
             balance: bool = True) -> dict[str, int]:
        return self.conn.call("walk", stride_mm=stride_mm, turn_mm=turn_mm, pattern=pattern, period_ms=period_ms,
                              duty=duty, step_h_mm=step_h_mm, body_h_mm=body_h_mm, bob=bob, wag=wag,
                              balance=balance, override=self.override)

    def stop_walk(self, wait_s: float | None = None) -> Any:
        """``stop_walk``: without ``wait_s`` returns None; with it, whether the
        gait stopped (GAIT_STOPPED) within ``wait_s``."""
        r = self.conn.call("stop_walk", timeout=(wait_s or 0) + 10.0, wait_s=wait_s)
        return None if wait_s is None else bool(r["stopped"])

    def load_geometry(self) -> None:
        """The daemon reads the leg geometry itself (``walk`` op)."""
        return None

    def gait_state(self) -> dict[str, Any] | None:
        return self.conn.call("gait")

    def stop_timeout(self) -> float:
        from kilo.body.client import GAIT_DEFAULTS, GAIT_STOP_MARGIN_S

        g = self.gait_state() or {}
        period = g.get("period_ms") or max(d["period_ms"] for d in GAIT_DEFAULTS.values())
        return period / 1000.0 + GAIT_STOP_MARGIN_S

    def pose(self, pitch_deg: float = 0, roll_deg: float = 0, dheight_mm: float = 0,
             xshift_mm: float = 0) -> dict[str, int]:
        return self.conn.call("pose", pitch_deg=pitch_deg, roll_deg=roll_deg, dheight_mm=dheight_mm,
                              xshift_mm=xshift_mm, override=self.override)

    def release_pose(self) -> None:
        self.conn.call("release_pose")

    # tricks (daemon ops: uploaded, played and owned by the daemon's session)
    def trick_upload(self, name: str | None = None, trick: Any = None, force: bool = False) -> dict[str, Any]:
        """``trick_upload``: a library name or a Trick; returns {name, trick_id, uploaded, crc16, frames}."""
        return self.conn.call("trick_upload", name=name, trick=None if trick is None else trick.to_dict(),
                              force=force, override=self.override)

    def upload_trick(self, trick: Any, force: bool = False) -> tuple[int, bool]:
        """BodyClient.upload_trick() over the daemon (the Trick travels as its dict)."""
        r = self.trick_upload(trick=trick, force=force)
        return r["trick_id"], r["uploaded"]

    def trick_play(self, name: str | None = None, trick: Any = None, mirror: bool = False, speed: float = 1.0,
                   repeats: int | None = None, posture: str | None = None, hold_s: float | None = None,
                   timeout_s: float | None = None) -> dict[str, Any]:
        """``trick_play``: blocks until TRICK_DONE (the daemon plays it on its
        loop; closing this connection stops it). Returns {reason, trick_id,
        elapsed_s, expected_ticks, epoch, stopped_by_us, name}."""
        return self.conn.call("trick_play", timeout=None, name=name,
                              trick=None if trick is None else trick.to_dict(), mirror=mirror, speed=speed,
                              repeats=repeats, posture=posture, hold_s=hold_s, timeout_s=timeout_s,
                              override=self.override)

    async def play(self, trick: Any, mirror: bool = False, speed: float = 1.0, timeout: float | None = None) -> Any:
        """BodyClient.play() over the daemon: a TrickResult-shaped namespace."""
        import asyncio

        kw = {"name": trick} if isinstance(trick, str) else {"trick": trick}
        r = await asyncio.get_running_loop().run_in_executor(
            None, lambda: self.trick_play(mirror=mirror, speed=speed, timeout_s=timeout, **kw))
        return SimpleNamespace(**r)

    def stop_trick(self, transition_ticks: int = 0) -> None:
        self.conn.call("trick_stop", transition_ticks=transition_ticks)

    # commands
    def cal_get(self) -> list[int]:
        return self.conn.call("cal_get")

    def cal_set(self, joint: int, offset_a16: int) -> None:
        self.conn.call("cal_set", joint=joint, offset_a16=offset_a16)

    def cal_save(self, wait_saved: float | None = 2.0) -> bool:
        return self.conn.call("cal_save", wait_saved=wait_saved)

    def cal_revert(self) -> None:
        self.conn.call("cal_revert")

    def param_get(self, pid: str | int) -> Any:
        return _ns(self.conn.call("param_get", id=pid))

    def param_set(self, pid: str | int, value: int, persist: bool = False) -> None:
        self.conn.call("param_set", id=pid, value=value, persist=persist)

    def param_list(self) -> list[Any]:
        return [_ns(v) for v in self.conn.call("param_list")]

    def stats(self) -> dict[str, Any]:
        return self.conn.call("stats")

    def pca_counts(self) -> list[int]:
        return self.conn.call("pca_read")

    def eeprom_read(self, addr: int, length: int) -> bytes:
        return bytes.fromhex(self.conn.call("eeprom_read", addr=addr, len=length))

    def request(self, name: str, fields: dict[str, Any] | None = None, timeout: float | None = None) -> Any:
        """The raw link request (queries and commands); the reply as a namespace."""
        return _ns(self.conn.call("request", name=name, fields=fields or {}, timeout_s=timeout))

    def __getattr__(self, name: str) -> Any:
        """Any other public BodyClient method, through the daemon's generic
        ``call`` op (arguments and result must be JSON-able)."""
        if name.startswith("_") or not callable(getattr(BodyClient, name, None)):
            raise AttributeError(name)

        def proxy(*args: Any, **kwargs: Any) -> Any:
            return self.conn.call("call", method=name, args=list(args), kwargs=kwargs)
        return proxy
