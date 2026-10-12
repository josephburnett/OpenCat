"""The socket protocol: accept a client, parse its lines, run its ops, reply.

JSON lines (wire format in kilo.remote): a request ``{"id", "op", "args"}``
runs ``op_<op>(session, **args)`` (kilo.daemon.ops) and gets one reply with
the same id, ``{"ok": true, "result": ...}`` or ``{"ok": false, "error":
{"type", "message", "args"}}`` (``kilo.remote.encode_error``; the client
re-raises the same exception type, so the CLI prints the same errors). A bad
line or an unknown op gets a ValueError reply; a TypeError from the op's
signature (a wrong argument) is reported as ValueError too.

Ordering. Ops run one at a time per client, in order, except
``CONCURRENT_OPS`` (``trick_play``), which runs as a task of its session so
the session still reads (pings, a ``trick_stop``, its own disconnect) while
the trick plays. Different clients' ops interleave on the loop; those that
reach the body also queue for the link's one in-flight request
(kilo.daemon.ops "Requests").

A client that does not read its replies past ``REPLY_BUFFER_MAX`` is
disconnected. Closing a session ends its hold (kilo.daemon.sessions),
cancels its streams and concurrent ops, and leaves a port release it made
to its lease (kilo.daemon.port).

All of this runs on the loop (kilo.daemon, thread-affinity rule 1).
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any, Callable

from kilo.daemon.sessions import Session
from kilo.link.serial_link import LinkError
from kilo.remote import dumps, encode_error, jsonable

log = logging.getLogger("kilo.daemon")

REPLY_BUFFER_MAX = 4 << 20      # disconnect a client that does not read its replies
LINE_LIMIT = 1 << 20
CONCURRENT_OPS = frozenset({"trick_play"})  # run as session tasks, not in line


class ClientsMixin:
    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        s = Session(self._next_sid, writer)
        self._next_sid += 1
        self.sessions[s.id] = s
        log.debug("client %d connected", s.id)
        try:
            while not s.closed:
                try:
                    line = await reader.readline()
                except (ValueError, asyncio.LimitOverrunError, ConnectionError):
                    break
                if not line:
                    break
                s.touch()
                await self._dispatch(s, line)
        finally:
            self._close_session(s)
            log.debug("client %d disconnected", s.id)

    def _close_session(self, s: Session) -> None:
        if s.closed:
            return
        assert self._on_loop()
        s.closed = True
        self.sessions.pop(s.id, None)
        if s.stream_task is not None:
            s.stream_task.cancel()
        self._end_hold(s)
        for t in list(s.tasks):
            t.cancel()
        if self.released_by is s and self.port_released:
            left = (self.release_until or time.monotonic()) - time.monotonic()
            log.warning("client %d released %s and went away: keeping it released for the rest of the "
                        "lease (%.0f s; avrdude may still be writing), or until `reacquire_port`",
                        s.id, self.port, max(0.0, left))
        try:
            s.writer.close()
        except Exception:
            pass

    def _reply(self, s: Session, msg: dict[str, Any]) -> None:
        if s.closed:
            return
        t = s.writer.transport
        if t is not None and t.get_write_buffer_size() > REPLY_BUFFER_MAX:
            log.warning("client %d does not read its replies: disconnecting", s.id)
            self._close_session(s)
            return
        s.writer.write(dumps(msg))

    async def _dispatch(self, s: Session, line: bytes) -> None:
        try:
            req = json.loads(line)
            rid, op, args = req.get("id"), req["op"], req.get("args") or {}
            if not isinstance(args, dict):
                raise ValueError("args must be an object")
        except (ValueError, KeyError, AttributeError, TypeError) as e:
            self._reply(s, {"id": None, "ok": False, "error": {"type": "ValueError", "message": f"bad request: {e}",
                                                                "args": [f"bad request: {e}"]}})
            return
        handler = getattr(self, "op_" + str(op), None)
        if handler is None:
            self._reply(s, {"id": rid, "ok": False, "error": {"type": "ValueError", "message": f"unknown op {op!r}",
                                                                "args": [f"unknown op {op!r}"]}})
            return
        if op in CONCURRENT_OPS:
            s.inflight += 1  # counted now: the lease must not lapse before the task starts
            t = asyncio.ensure_future(self._run_op(s, rid, op, handler, args, counted=True))
            s.tasks.add(t)
            t.add_done_callback(s.tasks.discard)
            return
        await self._run_op(s, rid, op, handler, args)

    async def _run_op(self, s: Session, rid: Any, op: str, handler: Callable[..., Any], args: dict[str, Any],
                      counted: bool = False) -> None:
        if not counted:
            s.inflight += 1
        try:
            result = await handler(s, **args)
            self._reply(s, {"id": rid, "ok": True, "result": jsonable(result)})
        except TypeError as e:
            self._reply(s, {"id": rid, "ok": False, "error": encode_error(ValueError(f"{op}: {e}"))})
        except Exception as e:
            if not isinstance(e, (LinkError, ValueError, KeyError)):
                log.exception("op %s failed", op)
            self._reply(s, {"id": rid, "ok": False, "error": encode_error(e)})
        finally:
            s.inflight -= 1
            s.touch()
