"""The link's one request in flight (``SerialLink._req_lock``), measured.

Every request on a link waits for the one before it: with the daemon that
is every client's op, the IR remote and the behaviour runner. These tests
time what that costs (printed with ``pytest -s``) and pin the bounds the
daemon relies on: a request holds the slot for at most (retries + 1) x
timeout, and the ``request`` op's timeout is capped so one client cannot
hold it for long (kilo.daemon.ops "Requests").
"""

import json
import socket
import threading
import time

import pytest

from kilo.body.client import BodyClient
from kilo.daemon import REQUEST_TIMEOUT_MAX_S, Daemon
from kilo.link.serial_link import LinkConfig, LinkTimeout, SerialLink
from kilo.link.transport import loopback_pair
from kilo.vbody.sim import VirtualBody

from conftest import FAST
from test_daemon import sockdir  # noqa: F401 (fixture)


@pytest.fixture
def silent_link():
    """A link to a body that never answers (nobody reads the other end)."""
    a, b = loopback_pair()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="silent")
    link.mark_alive()
    link.start()
    assert link.wait_until(lambda: link._transport is not None, 2)
    yield link
    link.stop()


def _timed_requests(link, kws):
    """Start one thread per kwargs dict (10 ms apart, in order); return when each finished."""
    t0 = time.monotonic()
    done = {}

    def req(i, kw):
        try:
            link.request("STATS_REQ", **kw)
        except LinkTimeout:
            pass
        done[i] = time.monotonic() - t0

    ts = []
    for i, kw in enumerate(kws):
        ts.append(threading.Thread(target=req, args=(i, kw)))
        ts[-1].start()
        time.sleep(0.01)
    for t in ts:
        t.join(10)
    return [done[i] for i in range(len(kws))]


def test_two_concurrent_requests_to_a_silent_body_queue(silent_link):
    """Two requests that time out run one after the other: the second
    finishes after both have spent their 4 x 50 ms."""
    cfg = silent_link.cfg
    one = (cfg.retries + 1) * cfg.ack_timeout_s
    first, second = _timed_requests(silent_link, [{}, {}])
    print(f"\nsilent body, two concurrent requests: {first:.3f} s, {second:.3f} s (one alone {one:.2f} s)")
    assert first >= one * 0.9
    assert second >= 2 * one * 0.9  # queued behind the first
    assert second < 2 * one + 0.3


def test_a_slow_request_delays_everyone_by_its_whole_budget(silent_link):
    """A request with a long timeout holds every other request back for
    (retries + 1) x that timeout: why the daemon caps the `request` op's."""
    slow, other = _timed_requests(silent_link, [{"timeout": 0.25}, {}])
    print(f"\nsilent body, a 4 x 0.25 s request then a default one: {slow:.3f} s, {other:.3f} s")
    assert slow >= 1.0 * 0.9 and other >= slow


def test_request_latency_beside_a_busy_client():
    """An answering body: one client loops requests as fast as it can (a
    `param_list`, a trick upload); another's request waits at most for the
    one in flight, a few ms (measured, printed; bound generous for CI)."""
    a, b = loopback_pair()
    vb = VirtualBody(b).start()
    link = SerialLink(lambda: a, config=LinkConfig(**FAST), port_name="vb")
    link.mark_alive()
    link.start()
    try:
        c = BodyClient(link)
        c.wait_ready(3)
        t0, n = time.monotonic(), 0
        while time.monotonic() - t0 < 0.5:
            c.stats()
            n += 1
        alone = (time.monotonic() - t0) / n
        stop = threading.Event()
        hog_n = [0]

        def hog():
            while not stop.is_set():
                c.param_get(0)
                hog_n[0] += 1

        h = threading.Thread(target=hog)
        h.start()
        time.sleep(0.1)
        lat = []
        for _ in range(30):
            t = time.monotonic()
            c.stats()
            lat.append(time.monotonic() - t)
            time.sleep(0.01)
        stop.set()
        h.join(5)
        lat.sort()
        print(f"\nanswering body: a request alone {alone * 1000:.1f} ms; beside a busy client "
              f"median {lat[len(lat) // 2] * 1000:.1f} ms, max {lat[-1] * 1000:.1f} ms "
              f"(the busy client made {hog_n[0]} requests meanwhile)")
        assert lat[-1] < 0.25  # never starved behind the busy client
        assert hog_n[0] > 0
    finally:
        link.stop()
        vb.stop()


def test_daemon_request_op_caps_the_timeout(sockdir, monkeypatch):  # noqa: F811
    """Through the daemon, to a body that never answers: a client's
    `request` with a 5 s timeout gives up after 4 x REQUEST_TIMEOUT_MAX_S,
    and another client's `stats` behind it is answered (here: times out)
    right after, instead of 20 s later."""
    a, b = loopback_pair()
    sock = sockdir / "k.sock"
    monkeypatch.setenv("KILO_SOCKET", str(sock))
    d = Daemon("silent", socket=sock, transport_factory=lambda: a, link_config=LinkConfig(**FAST))
    t = threading.Thread(target=d.run, kwargs={"install_signals": False}, daemon=True)
    t.start()
    assert d.ready.wait(5)
    try:
        assert d.link.wait_until(lambda: d.link._transport is not None, 2)

        def call(op, args, out, key):
            s = socket.socket(socket.AF_UNIX)
            s.connect(str(sock))
            t0 = time.monotonic()
            s.sendall(json.dumps({"id": 1, "op": op, "args": args}).encode() + b"\n")
            f = s.makefile("rb")
            reply = json.loads(f.readline())
            out[key] = (time.monotonic() - t0, reply)
            s.close()

        out = {}
        ta = threading.Thread(target=call, args=("request", {"name": "STATS_REQ", "timeout_s": 5.0}, out, "a"))
        tb = threading.Thread(target=call, args=("stats", {}, out, "b"))
        ta.start()
        time.sleep(0.05)
        tb.start()
        ta.join(30)
        tb.join(30)
        (ta_s, ra), (tb_s, rb) = out["a"], out["b"]
        print(f"\ndaemon, silent body: request(timeout_s=5) gave up after {ta_s:.2f} s, "
              f"stats behind it after {tb_s:.2f} s")
        assert not ra["ok"] and ra["error"]["type"] == "LinkTimeout"
        assert not rb["ok"] and rb["error"]["type"] == "LinkTimeout"
        cap = (d.link.cfg.retries + 1) * REQUEST_TIMEOUT_MAX_S
        assert ta_s < cap + 0.5
        assert tb_s < cap + 0.2 + 0.5
    finally:
        d.request_stop()
        t.join(5)
