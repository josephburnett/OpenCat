"""PerceptEvent: one timestamped event from a perception detector."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class PerceptEvent:
    t: float  # seconds, in the caller's time base (monotonic or log time)
    name: str  # near, approaching, clear, tilted, level, picked_up, put_down, tap, pet, startle
    data: dict[str, Any] = field(default_factory=dict, compare=False)

    def as_dict(self) -> dict[str, Any]:
        return {"t": self.t, "name": self.name, **self.data}

    def __str__(self) -> str:
        extra = " ".join(f"{k}={_fmt(v)}" for k, v in self.data.items())
        return f"{self.t:9.3f} {self.name}" + (f" {extra}" if extra else "")


def _fmt(v: Any) -> str:
    return f"{v:.1f}" if isinstance(v, float) else str(v)
