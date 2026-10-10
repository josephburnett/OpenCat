"""A minimal simulated body for testing the mind without hardware (§6).

For P0 this is a Python simulation. §6 describes the eventual virtual body as
``body/test/vbody``: the real firmware ``core/`` linked with stub HALs and
speaking the real protocol on a pty; this module is the stand-in until then
and only models what the mind's link logic needs:

- an unsolicited HELLO + EVENT BOOT on start (and on REBOOT);
- TELEMETRY at 25 Hz (every 2nd 50 Hz tick) with a gently swaying fake
  pitch/roll, a slowly draining battery and the current mode/epoch;
- ACKs for commands with (seq, type) de-duplication, data replies where the
  Messages adapter can pack them (else ACK UNKNOWN_TYPE);
- epoch-bearing goals checked against the current epoch (first payload byte)
  with EVENT GOAL_REFUSED(type << 8 | STALE), rate-limited to 1/s per type;
- the two-stage watchdog: no frame for 0.5 s -> EVENT WATCHDOG, epoch bump,
  mode LINK_LOST; 1.5 s -> REST; 10 s later LIMP; a frame afterwards ->
  EVENT LINK_RESTORED. It arms on the first frame received;
- a fall (``trigger_fall()``; the CLI binds it to a flag and the ``f`` key):
  EVENT FELL, epoch bump, mode FALLEN, then RECOVERING and RECOVERED.

It talks through the same Codec and Messages interfaces as the mind, over any
Transport (a pty master fd in the CLI, a loopback pair in tests).
"""

from __future__ import annotations

import math
import os
import threading
import time
import tty
from typing import Any

from kilo.link import consts
from kilo.link.codec import Codec, DecodeError, default_codec
from kilo.link.messages import Messages, ProtocolNotImplemented, default_messages
from kilo.link.transport import FdTransport, Transport

TICK_S = 0.020
POSTURE_NAMES = {v: k.upper() for k, v in consts.POSTURES.items()}


def open_pty() -> tuple[FdTransport, str, int]:
    """Open a pty pair. Returns (transport on the master, slave path, slave fd).

    The slave fd is kept open by the caller so the master never sees EIO
    while no client is attached.
    """
    master, slave = os.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    return FdTransport(master), os.ttyname(slave), slave


class VirtualBody:
    def __init__(self, transport: Transport, codec: Codec | None = None,
                 messages: Messages | None = None, *, telem_div: int = 2,
                 fall_after: float | None = None, verbose: bool = False,
                 wd_stop_s: float = 0.5, wd_rest_s: float = 1.5, wd_limp_s: float = 10.0):
        self.t = transport
        self.codec = codec or default_codec()
        self.msgs = messages or default_messages()
        self.decoder = self.codec.new_decoder()
        self.telem_div = telem_div
        self.fall_after = fall_after
        self.verbose = verbose
        self.wd_stop_s, self.wd_rest_s, self.wd_limp_s = wd_stop_s, wd_rest_s, wd_limp_s
        self.lock = threading.RLock()
        self.seq = 0
        self.tick = 0
        self.epoch = 0
        self.mode = 1  # LIMP until told otherwise
        self.posture = "LIMP"
        self.batt_mv = 8100
        self.rx_err = 0
        self.last_rx_seq = 0
        self.last_cmd: tuple[int, int, int] | None = None  # (seq, type, status)
        self.last_rx_at: float | None = None
        self.wd_stage = 0  # 0 ok, 1 stopped, 2 rest, 3 limp
        self.fall_until: float | None = None
        self.recover_until: float | None = None
        self.fall_side = 1
        self.goals: dict[str, bytes] = {}
        self.refused_at: dict[int, float] = {}
        self.pending_events: list[tuple[float, int, int]] = []  # (due, code, arg)
        self.received: list[tuple[str, int, bytes]] = []
        self.ignore_types: set[str] = set()  # tests: pretend these frames were lost
        self.reset_cause = 0
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_time = time.monotonic()

    # ---- control ---------------------------------------------------------------
    def start(self) -> "VirtualBody":
        self._running.set()
        self._thread = threading.Thread(target=self.run, name="vbody", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(1.0)

    def trigger_fall(self, side: int = 1) -> None:
        with self.lock:
            if self.fall_until is not None:
                return
            self.fall_side = side
            self.epoch = (self.epoch + 1) & 0xFF
            self.mode = 5  # FALLEN
            self.goals.clear()
            self.fall_until = time.monotonic() + 1.0
            self._event("FELL", side)
        self._say("fell")

    def reboot(self, cause: int = 2) -> None:
        with self.lock:
            self.reset_cause = cause
            self.tick = 0
            self.mode = 1
            self.posture = "LIMP"
            self.goals.clear()
            self.wd_stage = 0
            self.last_rx_at = None
            self.last_cmd = None
            self._boot()

    # ---- loop ------------------------------------------------------------------
    def run(self) -> None:
        self._running.set()
        with self.lock:
            self._boot()
        next_tick = time.monotonic()
        while self._running.is_set():
            now = time.monotonic()
            if now < next_tick:
                # read while waiting for the tick deadline
                try:
                    data = self.t.read(256)
                except OSError:
                    data = b""
                    time.sleep(0.005)
                if data:
                    for item in self.decoder.feed(data):
                        self._rx(item)
                continue
            next_tick += TICK_S
            if now - next_tick > 0.2:
                next_tick = now  # we were suspended; do not burst
            with self.lock:
                self._tick(now)

    def _boot(self) -> None:
        self._send("HELLO", proto_major=getattr(self.msgs, "PROTO_MAJOR", None) or 0,
                   proto_minor=0, fw_build=0x5642_4F44, reset_cause=self.reset_cause,
                   last_alive_tick=0, caps=0b11101, max_payload=60, trick_capacity=32,
                   joint_count=16, config_state=0, epoch=self.epoch)
        self._event("BOOT", self.reset_cause)
        self._say(f"boot ({consts.RESET_CAUSES.get(self.reset_cause)}) epoch {self.epoch}")

    def _tick(self, now: float) -> None:
        self.tick = (self.tick + 1) & 0xFFFF
        if self.fall_after is not None and now - self._start_time >= self.fall_after:
            self.fall_after = None
            self.trigger_fall()
        # fall / recovery timeline
        if self.fall_until is not None and now >= self.fall_until:
            self.fall_until = None
            self.mode = 6  # RECOVERING
            self.recover_until = now + 1.0
        if self.recover_until is not None and now >= self.recover_until:
            self.recover_until = None
            self.mode = 2
            self.posture = "REST"
            self._event("RECOVERED", 0)
            self._say("recovered")
        # watchdog
        if self.last_rx_at is not None:
            silent = now - self.last_rx_at
            if self.wd_stage == 0 and silent >= self.wd_stop_s:
                self.wd_stage = 1
                self.epoch = (self.epoch + 1) & 0xFF
                self.mode = 9  # LINK_LOST
                self.goals.clear()
                self._event("WATCHDOG", 0)
                self._say("watchdog: stop")
            elif self.wd_stage == 1 and silent >= self.wd_rest_s:
                self.wd_stage = 2
                self.posture = "REST"
                self._say("watchdog: rest")
            elif self.wd_stage == 2 and silent >= self.wd_rest_s + self.wd_limp_s:
                self.wd_stage = 3
                self.posture = "LIMP"
                self.mode = 1
                self._say("watchdog: limp")
        for due, code, arg in list(self.pending_events):
            if now >= due:
                self.pending_events.remove((due, code, arg))
                self._event(consts.EVENTS[code], arg)
        if self.tick % 500 == 0:
            self.batt_mv = max(6600, self.batt_mv - 1)
        if self.telem_div and self.tick % self.telem_div == 0:
            self._send_telemetry(now)

    def _send_telemetry(self, now: float) -> None:
        t = now - self._start_time
        pitch = 2.0 * math.sin(t * 0.7)
        roll = 1.5 * math.sin(t * 1.1)
        if self.mode == 5:
            roll = 85.0 * self.fall_side
        flags = (1 << 2 if self.posture != "LIMP" else 0) | (1 << 7 if self.wd_stage == 0 else 0)
        owners = 0
        self._send("TELEMETRY", tick=self.tick, pitch=int(pitch * 16), roll=int(roll * 16),
                   rate_pitch=int(1.4 * math.cos(t * 0.7) * 16), rate_roll=int(1.6 * math.cos(t * 1.1) * 16),
                   rate_yaw=0, dist_mm=600 + int(100 * math.sin(t * 0.3)), batt_mv=self.batt_mv,
                   mode=self.mode, owners=owners, flags=flags, last_rx_seq=self.last_rx_seq,
                   trick_frame=0xFF, gait_phase=0, tick_load=31, rx_err=self.rx_err & 0xFF,
                   epoch=self.epoch)

    # ---- rx ----------------------------------------------------------------------
    def _rx(self, item: Any) -> None:
        if isinstance(item, DecodeError):
            with self.lock:
                self.rx_err += 1
            return
        name = self.msgs.name_of(item.type)
        if name in self.ignore_types:
            return
        now = time.monotonic()
        with self.lock:
            self.received.append((name, item.seq, item.payload))
            if len(self.received) > 2000:
                del self.received[:1000]
            self.last_rx_seq = item.seq
            self.last_rx_at = now
            if self.wd_stage:
                self.wd_stage = 0
                if self.mode == 9:
                    self.mode = 2 if self.posture != "LIMP" else 1
                self._event("LINK_RESTORED", 0)
                self._say("link restored")
            if name in consts.GOAL_TYPES:
                self._goal(name, item)
            elif item.type < 0x80:
                self._command(name, item)

    def _goal(self, name: str, item: Any) -> None:
        if name == "HEARTBEAT":
            return
        if name in consts.EPOCH_GOALS:
            if not item.payload:
                return self._refuse(item.type, 1)
            if item.payload[0] != self.epoch:
                return self._refuse(item.type, consts.STALE)
            if self.mode in (5, 6):
                return self._refuse(item.type, 8)
        self.goals[name] = item.payload
        if name == "SET_POSTURE":
            # posture code is the byte after the epoch (Appendix E); full
            # unpacking needs the protocol module
            code = item.payload[1] if len(item.payload) > 1 else None
            new = POSTURE_NAMES.get(code, f"?{code}")
            if new != self.posture:
                self._say(f"posture {new}")
            self.posture = new
            self.mode = 1 if new == "LIMP" else 2

    def _refuse(self, gtype: int, reason: int) -> None:
        now = time.monotonic()
        if now - self.refused_at.get(gtype, -1e9) >= 1.0:
            self.refused_at[gtype] = now
            self._event("GOAL_REFUSED", (gtype << 8) | reason)

    def _command(self, name: str, item: Any) -> None:
        if self.last_cmd and self.last_cmd[:2] == (item.seq, item.type) and name not in consts.REPLIES:
            self._ack(item.seq, item.type, self.last_cmd[2])  # de-dup: re-ack, do not execute
            return
        status = consts.OK
        if name == "HELLO_REQ":
            self._boot_reply()
            return
        if name in consts.REPLIES:
            if not self._reply(name, item):
                self._ack(item.seq, item.type, consts.UNKNOWN_TYPE)
            return
        if name == "CAL_SAVE" or name == "PARAM_SET":
            self.pending_events.append((time.monotonic() + 0.1, consts.EV["EEPROM_SAVED"], 0))
        elif name == "REBOOT":
            self._ack(item.seq, item.type, status)
            self.reboot(2)
            return
        elif name not in consts.TYPES:
            status = consts.UNKNOWN_TYPE
        self.last_cmd = (item.seq, item.type, status)
        self._ack(item.seq, item.type, status)

    def _boot_reply(self) -> None:
        self._send("HELLO", proto_major=getattr(self.msgs, "PROTO_MAJOR", None) or 0,
                   proto_minor=0, fw_build=0x5642_4F44, reset_cause=self.reset_cause,
                   last_alive_tick=0, caps=0b11101, max_payload=60, trick_capacity=32,
                   joint_count=16, config_state=0, epoch=self.epoch)

    def _reply(self, name: str, item: Any) -> bool:
        reply = consts.REPLIES[name]
        fields: dict[str, Any]
        if reply == "PONG":
            try:
                req = self.msgs.unpack(item.type, item.payload)
                fields = {"token": req.token, "tick": self.tick}
            except (ProtocolNotImplemented, ValueError):
                return False
        elif reply == "CAL_VALUES":
            fields = {"offsets": [0] * consts.CAL_JOINTS}
        else:
            return False  # STATS, PARAM_VALUE, ...: need the protocol module
        try:
            self._send(reply, **fields)
        except (ProtocolNotImplemented, ValueError):
            return False
        return True

    # ---- tx ------------------------------------------------------------------------
    def _ack(self, seq: int, type: int, status: int) -> None:
        self._send("ACK", seq=seq, type=type, status=status)

    def _event(self, name: str, arg: int) -> None:
        arg = arg - 0x10000 if arg >= 0x8000 else arg
        self._send("EVENT", code=consts.EV[name], arg=arg, tick=self.tick, epoch=self.epoch)

    def _send(self, name: str, **fields: Any) -> None:
        payload = self.msgs.pack(name, **fields)
        frame = self.codec.encode(self.msgs.type_of(name), self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.t.write(frame)
        except OSError:
            pass

    def _say(self, text: str) -> None:
        if self.verbose:
            print(f"[vbody {time.monotonic() - self._start_time:7.2f}s] {text}", flush=True)
