"""Test plumbing: a scripted peer (body end) on a loopback transport, speaking
the real codec (COBS + CRC) and the generated message layouts.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import pytest

from kilo.link.codec import DecodeError, Frame, KiloCodec
from kilo.protocol import kilo_protocol as P
from kilo.link.messages import KiloMessages
from kilo.link.serial_link import LinkConfig, SerialLink
from kilo.link.transport import loopback_pair
from kilo.stepclock import StepClock


class Peer:
    """Scripted body end: records frames, optional handler answers them."""

    def __init__(self, end: Any, codec: Any = None, msgs: Any = None, clock: Callable[[], float] = time.monotonic):
        self.end = end
        self.clock = clock  # frames are stamped with it (a StepClock in stepped tests)
        self.consumed = 0  # bytes read and handled
        self.codec = codec or KiloCodec()
        self.msgs = msgs or KiloMessages()
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
                    self.frames.append((self.clock(), name, f.seq, f.payload))
                if self.handler:
                    self.handler(self, name, f)
            with self._lock:
                self.consumed += len(data)

    def sync(self, timeout: float = 5.0) -> None:
        """Return once every byte written to this end so far has been handled."""
        written = self.end.rx_written
        end = time.monotonic() + timeout
        while True:
            with self._lock:
                if self.consumed >= written:
                    return
            if time.monotonic() > end:
                raise TimeoutError("peer did not catch up")
            time.sleep(0.0005)

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

    def hello(self, epoch: int = 0, reset_cause: int = 0, proto_major: int = 1,
              proto_minor: int | None = None) -> None:
        if proto_minor is None:
            proto_minor = P.PROTO_MINOR  # the body of today; older minors lose the newer SET_GAIT flag bits
        self.send("HELLO", proto_major=proto_major, proto_minor=proto_minor, fw_build=0xABCD, reset_cause=reset_cause,
                  last_alive_tick=0, caps=0, max_payload=60, trick_capacity=32, joint_count=11,
                  config_state=0, epoch=epoch)

    def event(self, name: str, arg: int = 0, epoch: int = 0, tick: int = 0) -> None:
        from kilo.link import consts

        if arg >= 0x8000:
            arg -= 0x10000
        self.send("EVENT", code=consts.EV[name], arg=arg, tick=tick, epoch=epoch)

    def ack(self, seq: int, type: int, status: int = 0) -> None:
        self.send("ACK", seq=seq, type=type, status=status)


def count(peer: "Peer", name: str) -> int:
    return len(peer.of(name))


FAST = dict(liveness_window_s=0.3, hello_solicit_window_s=0.3, reconnect_min_s=0.02,
            reconnect_max_s=0.1)


@pytest.fixture
def make_link():
    made: list[tuple[SerialLink, Peer]] = []

    def factory(stepped: bool = False, **cfg: Any) -> tuple[SerialLink, Peer]:
        """``stepped=True``: link and peer share a StepClock (``link.clock``);
        time moves only on ``link.clock.step(dt)``, which returns once the
        link's writer and the peer have caught up."""
        a, b = loopback_pair()
        conf = LinkConfig(**{**FAST, **cfg})
        clock = StepClock() if stepped else time.monotonic
        link = SerialLink(lambda: a, codec=KiloCodec(), messages=KiloMessages(), config=conf,
                          port_name="loopback", clock=clock)
        peer = Peer(b, clock=clock)
        link.mark_alive()
        link.start()
        if stepped:
            clock.attach(link.sync)
            clock.attach(peer.sync)
            assert peer.wait_for(lambda: peer.of("HELLO_REQ"), 2)
            clock.settle()  # the connect's HELLO_REQ (and first HEARTBEAT) are out and read
        made.append((link, peer))
        return link, peer

    yield factory
    for link, peer in made:
        link.stop()
        peer.stop()


def stepped_alive(link: SerialLink, seconds: float, dt: float = 0.01,
                  each: Callable[[], Any] | None = None) -> None:
    """The stepped ``keep_alive``: the mind stamps liveness before every step."""
    def tick() -> None:
        link.mark_alive()
        if each is not None:
            each()
    link.clock.run(seconds, dt, tick)


class Background:
    """Run ``fn()`` on a thread (a ``request()`` blocked on a StepClock while
    the test steps time). ``result()`` joins and returns or re-raises;
    ``ended_at`` is the link clock when it returned or raised."""

    def __init__(self, fn: Callable[[], Any], clock: Callable[[], float]):
        self.value: Any = None
        self.error: BaseException | None = None
        self.ended_at: float | None = None

        def run() -> None:
            try:
                self.value = fn()
            except BaseException as e:  # re-raised by result()
                self.error = e
            finally:
                self.ended_at = clock()
        self._t = threading.Thread(target=run, daemon=True)
        self._t.start()

    def done(self) -> bool:
        return not self._t.is_alive()

    def result(self, timeout: float = 2.0) -> Any:
        self._t.join(timeout)
        assert not self._t.is_alive(), "background call did not return"
        if self.error is not None:
            raise self.error
        return self.value


def keep_alive(link: SerialLink, seconds: float, period: float = 0.02) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.mark_alive()
        time.sleep(period)


# Slow tests in files that other work owns right now (marked here rather
# than with @pytest.mark.slow in the file); see the "slow" marker in pyproject.
_SLOW_ELSEWHERE = {
    "test_daemon.py": {"test_cli_through_the_daemon_matches_direct", "test_cli_posture_hold_through_the_daemon",
                       "test_hold_ends_when_the_client_disconnects", "test_subscription_streams_at_the_requested_rate",
                       "test_a_slow_client_does_not_block_the_loop",
                       "test_release_lease_expires_and_disconnect_keeps_the_lease"},
    "test_hardening.py": {"test_link_stats_snapshot_under_concurrent_counter_growth", "test_cli_trick_through_the_daemon",
                          "test_reconnect_into_a_rebooting_body_does_not_stand_it_up",
                          "test_trick_ops_through_the_daemon"},
    "test_ir.py": {"test_daemon_remote_keys", "test_daemon_remote_walks"},
    "test_e2e_cli.py": {"test_kilo_balance_on", "test_kilo_posture_stand"},
}


def pytest_collection_modifyitems(config, items):
    for item in items:
        if item.originalname in _SLOW_ELSEWHERE.get(item.path.name, ()):
            item.add_marker(pytest.mark.slow)


@pytest.fixture(autouse=True)
def _no_real_daemon(monkeypatch, tmp_path_factory):
    """Never let a test (or a `kilo` subprocess) find a developer's running
    daemon: point the socket somewhere empty unless a test sets its own."""
    monkeypatch.setenv("KILO_SOCKET", str(tmp_path_factory.mktemp("nosock") / "none.sock"))
