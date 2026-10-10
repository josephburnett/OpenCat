"""Messages adapter interface: names <-> type codes, fields <-> payload bytes.

The link and the BodyClient never touch payload layouts directly; they go
through an object with this interface. protocol/ generates dataclasses for
every message (kilo_protocol); at integration a thin adapter over them
replaces ``messages_stub``.

Interface (structural)::

    messages.PROTO_MAJOR: int | None        expected HELLO.proto_major, None = don't check
    messages.type_of(name: str) -> int      raises KeyError
    messages.name_of(type: int) -> str      "0x??" for unknown types
    messages.pack(name: str, **fields) -> bytes
        Payload bytes for message ``name``. Raises ProtocolNotImplemented if the
        layout is not available, ValueError on a bad field.
    messages.unpack(type: int, payload: bytes) -> Message
        Raises ProtocolNotImplemented for unknown layouts, ValueError on a
        length mismatch.

Field names follow Appendix E (``epoch``, ``posture``, ``transition_ticks`` ...).
"""

from __future__ import annotations

from typing import Any, Protocol


class ProtocolNotImplemented(NotImplementedError):
    """The message layout is not available yet. TODO(protocol)."""


class Message:
    """A decoded message: the fields as attributes, plus ``msg_name``/``msg_type``.

    (Not ``name``/``type``: ACK has a field called ``type``.) Generated
    dataclasses can be used instead as long as they expose the same field
    names; the link reads fields with ``getattr`` only.
    """

    __slots__ = ("msg_name", "msg_type", "fields")

    def __init__(self, name: str, type: int, fields: dict[str, Any] | None = None):
        self.msg_name = name
        self.msg_type = type
        self.fields = dict(fields or {})

    def __getattr__(self, item: str) -> Any:
        try:
            return self.fields[item]
        except KeyError:
            raise AttributeError(item) from None

    def as_dict(self) -> dict[str, Any]:
        return dict(self.fields)

    def __repr__(self) -> str:
        inner = ", ".join(f"{k}={v!r}" for k, v in self.fields.items())
        return f"{self.msg_name}({inner})"

    def __eq__(self, other: object) -> bool:
        return (isinstance(other, Message) and other.msg_name == self.msg_name
                and other.fields == self.fields)


class Messages(Protocol):
    PROTO_MAJOR: int | None

    def type_of(self, name: str) -> int: ...

    def name_of(self, type: int) -> str: ...

    def pack(self, name: str, **fields: Any) -> bytes: ...

    def unpack(self, type: int, payload: bytes) -> Message: ...


def default_messages() -> Messages:
    """TODO(protocol): return an adapter over the generated kilo_protocol module."""
    from kilo.link.messages_stub import StubMessages

    return StubMessages()
