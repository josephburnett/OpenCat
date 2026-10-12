"""Streams: what a subscribed client receives without asking.

After ``subscribe {rate_hz, kinds}`` the daemon writes ``{"stream": kind,
"t": <monotonic>, "data": ...}`` lines without an id (``STREAM_KINDS``):
``snapshot`` (BodyState.snapshot() at ``rate_hz``, from a task of the
session), ``telemetry`` (decimated to ``rate_hz``), ``event``, ``hello``,
``reboot``, ``goals_cleared``, ``connection`` (``{connected, alive, port,
error}`` on every change) and ``percepts`` (``{events, flags}`` whenever
percept events happen).

Percepts. The daemon feeds a ``kilo.perception.SharedPercepts`` on every
TELEMETRY: the ``percepts`` op returns its flags and recent events, the
``percepts`` stream pushes each batch of percept events, and the behaviour
runner gets the same object.

Back-pressure. The loop never waits for a client: writes are buffered by
asyncio. A client whose buffer passes ``STREAM_BUFFER_MAX`` loses stream
messages (counted, then one ``dropped`` notice with the count); one past
``REPLY_BUFFER_MAX`` (kilo.daemon.clients) is disconnected.

The link's subscriber callbacks run on the loop (the link dispatches them
through ``loop.call_soon_threadsafe``); ``_percept_events`` may be called
from any thread and hops to the loop.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Callable

from kilo.remote import dumps

log = logging.getLogger("kilo.daemon")

STREAM_BUFFER_MAX = 256 << 10   # drop stream messages for a client beyond this backlog
STREAM_KINDS = ("snapshot", "telemetry", "event", "hello", "reboot", "goals_cleared", "connection", "percepts")


class StreamsMixin:
    def _subscribe_link(self) -> None:
        """Hook the fanout, the percept feed and the connection notes to the link."""
        for kind in ("telemetry", "event", "hello", "reboot", "goals_cleared", "connection"):
            self.link.subscribe(kind, self._make_fanout(kind))
        self.link.subscribe("telemetry", self._feed_percepts)
        self.link.subscribe("connection", self._log_connection)
        self.percepts.add_listener(self._percept_events)

    def _make_fanout(self, kind: str) -> Callable[..., None]:
        def fan(*args: Any) -> None:
            if not self.sessions:
                return
            if kind == "telemetry":
                data = args[0]
            elif kind == "hello":
                data = {"hello": args[0], "solicited": args[1]}
            elif kind == "reboot":
                data = {"hello": args[0], "why": args[1]}
            elif kind == "goals_cleared":
                data = {"reason": args[0], "names": args[1]}
            elif kind == "connection":
                data = {"connected": args[0], "alive": args[1], "port": self.port,
                        "error": self.link.last_open_error if self.link is not None else None}
            else:
                data = args[0]
            now = time.monotonic()
            for s in list(self.sessions.values()):
                if kind not in s.kinds:
                    continue
                if kind == "telemetry" and s.rate_hz > 0 and now - s.last_telemetry_sent < 1.0 / s.rate_hz:
                    continue
                if kind == "telemetry":
                    s.last_telemetry_sent = now
                self._stream(s, kind, data)
        return fan

    def _feed_percepts(self, msg: Any) -> None:
        self.percepts.update(self.client.state)

    def _percept_events(self, events: list[Any], d: dict[str, Any]) -> None:
        """SharedPercepts listener: any thread (the loop, or the behaviour runner)."""
        data = {"events": [e.as_dict() for e in events],
                "flags": {k: bool(d.get(k)) for k in ("near", "approaching", "clear", "tilted", "picked_up",
                                                      "tap", "pet", "startle")}}
        if self.loop is None or self._on_loop():
            self._fan_percepts(data)
        else:
            try:
                self.loop.call_soon_threadsafe(self._fan_percepts, data)
            except RuntimeError:
                pass

    def _fan_percepts(self, data: dict[str, Any]) -> None:
        for s in list(self.sessions.values()):
            if "percepts" in s.kinds:
                self._stream(s, "percepts", data)

    def _log_connection(self, connected: bool, alive: bool) -> None:
        was, self._was_connected = self._was_connected, connected
        if was and not connected and not self._shutting_down and not self.port_released:
            log.warning("port %s closed; reopening (every %.0f-%.0f s until it is back)", self.port,
                        self.link.cfg.reconnect_min_s, self.link.cfg.reconnect_max_s)

    def _stream(self, s: Any, kind: str, data: Any) -> None:
        if s.closed:
            return
        buf = s.writer.transport.get_write_buffer_size() if s.writer.transport else 0
        if buf > STREAM_BUFFER_MAX:
            s.dropped += 1
            return
        if s.dropped:
            s.writer.write(dumps({"stream": "dropped", "t": time.monotonic(), "data": {"count": s.dropped}}))
            s.dropped = 0
        s.writer.write(dumps({"stream": kind, "t": time.monotonic(), "data": data}))

    async def _snapshot_stream(self, s: Any) -> None:
        period = 1.0 / s.rate_hz
        while not s.closed:
            self._stream(s, "snapshot", self.client.state.snapshot())
            await asyncio.sleep(period)
