"""Messages adapter: names <-> type codes, fields <-> payload bytes.

The link and the BodyClient never touch payload layouts directly; they go
through this adapter over the generated dataclasses in
``kilo.protocol.kilo_protocol`` (protocol/spec.toml is the single source of
truth; the module is vendored by protocol/vendor.py).

Interface (structural; tests may substitute their own)::

    messages.PROTO_MAJOR: int | None        expected HELLO.proto_major, None = don't check
    messages.type_of(name: str) -> int      raises KeyError
    messages.name_of(type: int) -> str      "0x??" for unknown types
    messages.pack(name: str, **fields) -> bytes
        Payload bytes for message ``name``. Raises ValueError on a bad or
        missing field. Count fields of variable arrays (BEEP ``n``,
        TRICK_FRAMES ``count``) are implied by the list length; list elements
        may be dicts or the generated element dataclasses.
    messages.unpack(type: int, payload: bytes) -> dataclass instance
        Raises ValueError on an unknown type or a length mismatch.

Decoded messages are the generated dataclasses: fields are attributes, and
``fields_of(msg)`` gives a plain dict.
"""

from __future__ import annotations

from dataclasses import fields as dc_fields, is_dataclass
from typing import Any, Protocol

from kilo.protocol import kilo_protocol as P


class Messages(Protocol):
    PROTO_MAJOR: int | None

    def type_of(self, name: str) -> int: ...

    def name_of(self, type: int) -> str: ...

    def pack(self, name: str, **fields: Any) -> bytes: ...

    def unpack(self, type: int, payload: bytes) -> Any: ...


def fields_of(msg: Any) -> dict[str, Any]:
    """A decoded message as a plain dict (bytes as hex, element structs as dicts)."""
    return P.to_dict(msg)


def _build(cls: type, fields: dict[str, Any]) -> Any:
    known = {f.name: f for f in dc_fields(cls)}
    extra = set(fields) - set(known)
    if extra:
        raise ValueError(f"{cls.NAME if hasattr(cls, 'NAME') else cls.__name__}: unknown fields {sorted(extra)}")
    kw: dict[str, Any] = {}
    for k, v in fields.items():
        t = str(known[k].type)
        if t.startswith("list[") and t != "list[int]":
            ec = getattr(P, t[5:-1])
            v = [x if is_dataclass(x) else _build(ec, x) for x in v]
        elif t == "bytes" and isinstance(v, str):
            v = bytes.fromhex(v)
        kw[k] = v
    return cls(**kw)


class KiloMessages:
    PROTO_MAJOR: int | None = P.PROTO_MAJOR
    PROTO_MINOR: int = P.PROTO_MINOR

    def type_of(self, name: str) -> int:
        return int(P.MsgId[name])

    def name_of(self, type: int) -> str:
        try:
            return P.MsgId(type).name
        except ValueError:
            return f"0x{type:02X}"

    def cls_of(self, name: str) -> type:
        return P.MESSAGES[self.type_of(name)]

    def build(self, name: str, **fields: Any) -> Any:
        """The dataclass instance for ``name`` (missing scalar fields default to 0)."""
        try:
            cls = self.cls_of(name)
        except KeyError:
            raise ValueError(f"unknown message {name!r}") from None
        return _build(cls, fields)

    def pack(self, name: str, **fields: Any) -> bytes:
        try:
            return self.build(name, **fields).pack()
        except P.PayloadError as e:
            raise ValueError(str(e)) from None
        except TypeError as e:
            raise ValueError(f"{name}: {e}") from None

    def unpack(self, type: int, payload: bytes) -> Any:
        try:
            return P.decode_payload(type, bytes(payload))
        except KeyError:
            raise ValueError(f"unknown message type 0x{type:02X}") from None
        except P.PayloadError as e:
            raise ValueError(str(e)) from None


def default_messages() -> Messages:
    return KiloMessages()
