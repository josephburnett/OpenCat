"""Test plumbing: a trivial length-prefixed codec, a messages adapter with a
few extra (test-only) layouts, and a scripted peer on a loopback transport.

The length-prefixed codec is deliberately not the real framing: these tests
exercise the link's plumbing, not COBS/CRC (protocol/ owns those vectors).
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Any, Callable

import pytest

from kilo.link.codec import DecodeError, Frame
from kilo.link.messages import Message
from kilo.link.messages_stub import StubMessages
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import loopback_pair


class LenPrefixDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[Frame | DecodeError]:
        self.buf += data
        out: list[Frame | DecodeError] = []
        while self.buf:
            n = self.buf[0]
            if n < 2:
                out.append(DecodeError("short", bytes(self.buf[:1])))
                del self.buf[:1]
                continue
            if len(self.buf) < 1 + n:
                break
            body = bytes(self.buf[1:1 + n])
            del self.buf[:1 + n]
            out.append(Frame(body[0], body[1], body[2:]))
        return out


class LenPrefixCodec:
    """len u8 | type u8 | seq u8 | payload"""

    def encode(self, type: int, seq: int, payload: bytes) -> bytes:
        return bytes((2 + len(payload), type, seq)) + bytes(payload)

    def new_decoder(self) -> LenPrefixDecoder:
        return LenPrefixDecoder()


# test-only layouts: plumbing needs a few goals and replies to exist
_EXTRA = {
    "SET_POSTURE": ("<BBBB", ("epoch", "posture", "transition_ticks", "flags")),
    "SET_HEAD": ("<BBhhhH", ("epoch", "mode", "pan", "tilt", "tail", "max_speed_dps")),
    "SET_EYES": ("<BBBB", ("led_mask", "r", "g", "b")),
    "CAL_SET": ("<Bh", ("joint", "offset_a16")),
    "PING": ("<H", ("token",)),
    "PONG": ("<HH", ("token", "tick")),
}


class PlumbingMessages(StubMessages):
    PROTO_MAJOR = 0

    def pack(self, name: str, **fields: Any) -> bytes:
        if name == "CAL_VALUES":
            return struct.pack("<16h", *fields["offsets"])
        if name in _EXTRA:
            fmt, names = _EXTRA[name]
            return struct.pack(fmt, *(fields[n] for n in names))
        return super().pack(name, **fields)

    def unpack(self, type: int, payload: bytes) -> Message:
        name = self.name_of(type)
        if name == "CAL_VALUES":
            return Message(name, type, {"offsets": list(struct.unpack("<16h", payload))})
        if name in _EXTRA:
            fmt, names = _EXTRA[name]
            return Message(name, type, dict(zip(names, struct.unpack(fmt, payload))))
        return super().unpack(type, payload)


class Peer:
    """Scripted body end: records frames, optional handler answers them."""

    def __init__(self, end: Any, codec: Any = None, msgs: Any = None):
        self.end = end
        self.codec = codec or LenPrefixCodec()
        self.msgs = msgs or PlumbingMessages()
        self.dec = self.codec.new_decoder()
        self.frames: list[tuple[float, str, int, bytes]] = []
        self.handler: Callable[["Peer", str, Frame], None] | None = None
        self.seq = 0
        self._run = True
        self._lock = threading.Lock()
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self) -> None:
        while self._run:
            try:
                data = self.end.read(256)
            except OSError:
                time.sleep(0.01)
                continue
            for f in self.dec.feed(data):
                if isinstance(f, DecodeError):
                    continue
                name = self.msgs.name_of(f.type)
                with self._lock:
                    self.frames.append((time.monotonic(), name, f.seq, f.payload))
                if self.handler:
                    self.handler(self, name, f)

    def send(self, name: str, **fields: Any) -> None:
        payload = self.msgs.pack(name, **fields)
        self.end.write(self.codec.encode(self.msgs.type_of(name), self.seq, payload))
        self.seq = (self.seq + 1) & 0xFF

    def of(self, name: str, since: float = 0.0) -> list[tuple[float, str, int, bytes]]:
        with self._lock:
            return [f for f in self.frames if f[1] == name and f[0] >= since]

    def wait_for(self, pred: Callable[[], bool], timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.005)
        return pred()

    def stop(self) -> None:
        self._run = False
        self._t.join(1)

    # convenience senders
    def telemetry(self, tick: int = 1, epoch: int = 0, mode: int = 2, **kw: Any) -> None:
        f = dict(tick=tick, pitch=0, roll=0, rate_pitch=0, rate_roll=0, rate_yaw=0, dist_mm=0,
                 batt_mv=8000, mode=mode, owners=0, flags=1 << 7, last_rx_seq=0, trick_frame=255,
                 gait_phase=0, tick_load=0, rx_err=0, epoch=epoch)
        f.update(kw)
        self.send("TELEMETRY", **f)

    def hello(self, epoch: int = 0, reset_cause: int = 0, proto_major: int = 0) -> None:
        self.send("HELLO", proto_major=proto_major, proto_minor=0, fw_build=0xABCD, reset_cause=reset_cause,
                  last_alive_tick=0, caps=0, max_payload=60, trick_capacity=32, joint_count=16,
                  config_state=0, epoch=epoch)

    def event(self, name: str, arg: int = 0, epoch: int = 0, tick: int = 0) -> None:
        from kilo.link import consts

        if arg >= 0x8000:
            arg -= 0x10000
        self.send("EVENT", code=consts.EV[name], arg=arg, tick=tick, epoch=epoch)

    def ack(self, seq: int, type: int, status: int = 0) -> None:
        self.send("ACK", seq=seq, type=type, status=status)


FAST = dict(liveness_window_s=0.3, hello_solicit_window_s=0.3, reconnect_min_s=0.02,
            reconnect_max_s=0.1)


@pytest.fixture
def make_link():
    made: list[tuple[SerialLink, Peer]] = []

    def factory(**cfg: Any) -> tuple[SerialLink, Peer]:
        a, b = loopback_pair()
        conf = LinkConfig(**{**FAST, **cfg})
        link = SerialLink(lambda: a, codec=LenPrefixCodec(), messages=PlumbingMessages(), config=conf,
                          port_name="loopback")
        peer = Peer(b)
        link.mark_alive()
        link.start()
        made.append((link, peer))
        return link, peer

    yield factory
    for link, peer in made:
        link.stop()
        peer.stop()


def keep_alive(link: SerialLink, seconds: float, period: float = 0.02) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.mark_alive()
        time.sleep(period)
