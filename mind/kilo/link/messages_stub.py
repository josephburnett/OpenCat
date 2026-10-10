"""STUB messages adapter. TODO(protocol): replaced by an adapter over kilo_protocol.

It knows only the layouts the link itself needs to function (HELLO, TELEMETRY,
ACK, EVENT, transcribed from Appendix E) plus the empty-payload messages.
Every other layout raises ProtocolNotImplemented, so the CLI reports
"TODO(protocol)" instead of sending bytes from a guessed layout.
"""

from __future__ import annotations

import struct
from typing import Any

from kilo.link import consts
from kilo.link.messages import Message, ProtocolNotImplemented

_EMPTY = ("HELLO_REQ", "HEARTBEAT", "STATS_REQ", "PCA_READ", "CAL_GET",
          "CAL_SAVE", "CAL_REVERT")

# name -> (struct format, field names). Appendix E.
_LAYOUTS: dict[str, tuple[str, tuple[str, ...]]] = {
    **{n: ("<", ()) for n in _EMPTY},
    "HELLO": ("<BBIBHHBBBBB", (
        "proto_major", "proto_minor", "fw_build", "reset_cause",
        "last_alive_tick", "caps", "max_payload", "trick_capacity",
        "joint_count", "config_state", "epoch")),
    "TELEMETRY": ("<HhhhhhHHBBHBBBBBB", (
        "tick", "pitch", "roll", "rate_pitch", "rate_roll", "rate_yaw",
        "dist_mm", "batt_mv", "mode", "owners", "flags", "last_rx_seq",
        "trick_frame", "gait_phase", "tick_load", "rx_err", "epoch")),
    "ACK": ("<BBB", ("seq", "type", "status")),
    "EVENT": ("<BhHB", ("code", "arg", "tick", "epoch")),
}
_STRUCTS = {n: (struct.Struct(f), names) for n, (f, names) in _LAYOUTS.items()}


class StubMessages:
    PROTO_MAJOR: int | None = None  # TODO(protocol): from spec.toml

    def type_of(self, name: str) -> int:
        return consts.TYPES[name]

    def name_of(self, type: int) -> str:
        return consts.NAMES.get(type, f"0x{type:02X}")

    def pack(self, name: str, **fields: Any) -> bytes:
        try:
            st, names = _STRUCTS[name]
        except KeyError:
            raise ProtocolNotImplemented(
                f"TODO(protocol): layout of {name} comes from protocol/spec.toml") from None
        missing = set(names) - set(fields)
        extra = set(fields) - set(names)
        if missing or extra:
            raise ValueError(f"{name}: missing {sorted(missing)} extra {sorted(extra)}")
        try:
            return st.pack(*(fields[n] for n in names))
        except struct.error as e:
            raise ValueError(f"{name}: {e}") from None

    def unpack(self, type: int, payload: bytes) -> Message:
        name = self.name_of(type)
        try:
            st, names = _STRUCTS[name]
        except KeyError:
            raise ProtocolNotImplemented(
                f"TODO(protocol): layout of {name} comes from protocol/spec.toml") from None
        if len(payload) != st.size:
            raise ValueError(f"{name}: payload {len(payload)} B, expected {st.size}")
        return Message(name, type, dict(zip(names, st.unpack(payload))))
