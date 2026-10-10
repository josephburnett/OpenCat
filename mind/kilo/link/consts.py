"""Protocol constants for the link, state model and CLI, derived from the
generated module (kilo.protocol.kilo_protocol, from protocol/spec.toml).

Nothing here is transcribed by hand: change spec.toml, regenerate, re-vendor.
"""

from __future__ import annotations

import enum

from kilo.protocol import kilo_protocol as P

# Event codes (EVENT.code). The generator names this enum EventCode because
# the EVENT message dataclass is called Event.
EventCode: type[enum.IntEnum] = P.EventCode

PROTO_MAJOR = P.PROTO_MAJOR
PROTO_MINOR = P.PROTO_MINOR

# --- message type codes -------------------------------------------------------
TYPES: dict[str, int] = {m.name: int(m) for m in P.MsgId}
NAMES: dict[int, str] = {v: k for k, v in TYPES.items()}
_CLS = {c.NAME: c for c in P.MESSAGES.values()}

# Goals are never acked; everything else Pi->body is a command.
GOAL_TYPES = frozenset(n for n, c in _CLS.items() if c.KIND is P.Kind.GOAL)
COMMAND_TYPES = frozenset(n for n, c in _CLS.items() if c.KIND is P.Kind.COMMAND)
# Goals and commands whose payload starts with the epoch byte.
EPOCH_GOALS = frozenset(n for n in GOAL_TYPES if _CLS[n].HAS_EPOCH)
# Desired motion goals cleared on an epoch change (§5).
MOTION_GOALS = EPOCH_GOALS
# Queries: answered by their data reply only (an ACK only on failure), never
# de-duplicated by the body, retried with a fresh seq.
QUERIES = frozenset(n for n, c in _CLS.items() if c.IS_QUERY)
REPLIES: dict[str, str] = {NAMES[q]: NAMES[r] for q, r in P.QUERY_REPLY.items()}

# --- ACK status ------------------------------------------------------------------
ACK_STATUS: dict[int, str] = {int(s): s.name for s in P.AckStatus}
OK = int(P.AckStatus.OK)
BAD_LEN = int(P.AckStatus.BAD_LEN)
BAD_ARG = int(P.AckStatus.BAD_ARG)
UNKNOWN_TYPE = int(P.AckStatus.UNKNOWN_TYPE)
EEPROM_BUSY = int(P.AckStatus.EEPROM_BUSY)
STALE = int(P.AckStatus.STALE)
# GOAL_REFUSED reasons reuse the ACK status codes.
REFUSE_REASON: dict[int, str] = {int(r): r.name for r in P.GoalRefusedReason}

# --- events ---------------------------------------------------------------------
EVENTS: dict[int, str] = {int(e): e.name for e in EventCode}
EV: dict[str, int] = {v: k for k, v in EVENTS.items()}
CLEARING_EVENTS = frozenset({EV["FELL"], EV["WATCHDOG"], EV["LOW_BATTERY"]})

# --- telemetry enums -------------------------------------------------------------
MODES: dict[int, str] = {int(m): m.name for m in P.Mode}
FLAG_BITS: dict[int, str] = {f.value.bit_length() - 1: f.name.lower() for f in P.TelemetryFlags}
OWNERS: dict[int, str] = {int(o): o.name for o in P.Owner}
RESET_CAUSES: dict[int, str] = {int(c): c.name for c in P.ResetCause}
CONFIG_STATES: dict[int, str] = {int(c): c.name for c in P.ConfigState}
POSTURES: dict[str, int] = {p.name.lower(): int(p) for p in P.Posture}
POSTURE_BALANCE_FLAG = int(P.PostureFlags.BALANCE)

REBOOT_MAGIC = P.REBOOT_MAGIC
CAL_JOINTS = P.CHANNEL_COUNT
PARAMS = P.PARAMS
PARAMS_BY_NAME = P.PARAMS_BY_NAME
