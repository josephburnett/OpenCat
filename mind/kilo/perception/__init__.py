"""Perception from body telemetry (docs/architecture.md §5 "Perception").

- ``ultrasonic.UltrasonicDetector``: near / approaching / clear;
- ``imu.ImuDetector``: tilted, picked_up / put_down, tap, pet, startle;
- ``percepts.Percepts``: feeds both from BodyState-like snapshots and keeps a
  10 s ``ring.TelemetryRing`` (numpy when available).

Pure Python; numpy is optional. No threads: call ``update()`` from the
behaviour loop or once per TELEMETRY.
"""

from kilo.perception.events import PerceptEvent
from kilo.perception.imu import ImuConfig, ImuDetector
from kilo.perception.percepts import Percepts, Sample, extract
from kilo.perception.ring import HAVE_NUMPY, TelemetryRing
from kilo.perception.ultrasonic import UltrasonicConfig, UltrasonicDetector

__all__ = [
    "HAVE_NUMPY", "ImuConfig", "ImuDetector", "PerceptEvent", "Percepts", "Sample",
    "TelemetryRing", "UltrasonicConfig", "UltrasonicDetector", "extract",
]
