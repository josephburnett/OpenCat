"""BodyClient: the mind's P0 API to the body (docs/architecture.md §5, §7 P0).

Thin and synchronous: goals go through ``SerialLink.set_goal`` (latched,
refreshed at 5 Hz, epoch-stamped, cleared on FELL/WATCHDOG/LOW_BATTERY);
commands go through ``SerialLink.request`` (seq, 50 ms ACK wait, 3 retries,
de-dup safe). Behaviours built later (walk/look/play/beep) follow the same two
patterns.

TODO(protocol): every method that builds a payload names the Appendix E
message and fields; packing goes through the Messages adapter, which in the
stub raises ProtocolNotImplemented for layouts it does not know. Swap in the
kilo_protocol adapter and confirm the field names against spec.toml.
"""

from __future__ import annotations

import time
from typing import Any

from kilo.link import consts
from kilo.link.serial_link import LinkError, LinkTimeout, SerialLink


class ProtocolMismatch(LinkError):
    pass


# Postures the CLI offers. "balance" is STAND with the balance flag (Appendix E
# SET_POSTURE flags bit 0); it is not a separate posture code.
POSTURE_CHOICES = ("limp", "rest", "stand", "sit", "balance", "stance", "calib", "zero")


class BodyClient:
    def __init__(self, link: SerialLink):
        self.link = link

    @property
    def state(self):
        return self.link.state

    # ---- main-loop helpers --------------------------------------------------
    def spin(self, seconds: float, period: float = 0.05) -> None:
        """Run the trivial main loop: stamp liveness for ``seconds``."""
        end = time.monotonic() + seconds
        while True:
            self.link.mark_alive()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(period, left))

    # ---- identity ------------------------------------------------------------
    def hello(self, timeout: float = 0.3) -> Any:
        """HELLO_REQ -> HELLO. Retries per the link config."""
        return self.link.request("HELLO_REQ", timeout=timeout)

    def wait_ready(self, timeout: float = 3.0, require_telemetry: bool = True) -> Any:
        """Wait for the port, a HELLO and (optionally) live telemetry.

        Raises LinkTimeout if the body does not show up, ProtocolMismatch if
        its protocol major differs (the mind then refuses to drive; §4).
        """
        link = self.link
        deadline = time.monotonic() + timeout
        if not link.wait_until(lambda: link.state.connected, timeout):
            raise LinkTimeout(f"port {link.state.port} did not open")
        if not link.wait_until(lambda: link.state.hello is not None, max(0.0, min(0.5, deadline - time.monotonic()))):
            while link.state.hello is None:
                if time.monotonic() >= deadline:
                    raise LinkTimeout("no HELLO from the body")
                try:
                    self.hello()
                except LinkTimeout:
                    pass
                except LinkError:
                    link.wait_until(lambda: link.state.connected, 0.2)
        if require_telemetry:
            left = max(0.0, deadline - time.monotonic())
            if not link.wait_until(lambda: link.state.alive, left):
                raise LinkTimeout("HELLO received but no telemetry")
        if link.state.proto_ok is False:
            raise ProtocolMismatch(
                f"body protocol {link.state.hello.proto_major}, mind expects "
                f"{link.messages.PROTO_MAJOR}: flash the body (kilo flash)")
        return link.state.hello

    def _check_drivable(self) -> None:
        if self.state.proto_ok is False:
            raise ProtocolMismatch("protocol mismatch: refusing to drive")

    # ---- postures (goal) -------------------------------------------------------
    def posture(self, name: str, transition_ticks: int = 0, balance: bool | None = None) -> None:
        """Latch SET_POSTURE. ``balance`` means STAND with the balance flag."""
        self._check_drivable()
        name = name.lower()
        if name == "balance":
            code, balance = consts.POSTURES["stand"], True if balance is None else balance
        else:
            code = consts.POSTURES[name]
        # TODO(protocol): SET_POSTURE fields per Appendix E: E, posture u8,
        # transition_ticks u8 (0 = 25), flags u8 (bit 0 balance).
        self.link.set_goal("SET_POSTURE", posture=code, transition_ticks=transition_ticks,
                           flags=1 if balance else 0)

    def limp(self) -> None:
        self.posture("limp")

    def rest(self) -> None:
        self.posture("rest")

    def stand(self) -> None:
        self.posture("stand")

    def sit(self) -> None:
        self.posture("sit")

    def balance(self) -> None:
        self.posture("balance")

    def release(self) -> None:
        """Stop asserting a posture (the body keeps what it is doing)."""
        self.link.clear_goal("SET_POSTURE")

    # ---- calibration (commands) ------------------------------------------------
    def cal_get(self) -> list[int]:
        """CAL_GET -> CAL_VALUES: 16 offsets in a16."""
        msg = self.link.request("CAL_GET")
        # TODO(protocol): CAL_VALUES field name for the 16 x i16 array.
        return list(_field(msg, "offsets"))

    def cal_set(self, joint: int, offset_a16: int) -> None:
        """CAL_SET: absolute offset for one joint, ±25° (±400 a16)."""
        if not 0 <= joint < consts.CAL_JOINTS:
            raise ValueError(f"joint {joint} out of range 0..{consts.CAL_JOINTS - 1}")
        if not -400 <= offset_a16 <= 400:
            raise ValueError("offset beyond ±25°")
        # TODO(protocol): CAL_SET fields per Appendix E: joint u8, offset_a16 i16.
        self.link.request("CAL_SET", {"joint": joint, "offset_a16": offset_a16})

    def cal_save(self, wait_saved: float | None = 2.0) -> bool:
        """CAL_SAVE is acked when queued; EVENT EEPROM_SAVED reports the write.

        Returns True when the save was confirmed (or ``wait_saved`` is None).
        """
        n_before = self._count_events("EEPROM_SAVED")
        self.link.request("CAL_SAVE")
        if wait_saved is None:
            return True
        return self.link.wait_until(lambda: self._count_events("EEPROM_SAVED") > n_before, wait_saved)

    def cal_revert(self) -> None:
        self.link.request("CAL_REVERT")

    # ---- params (commands) ------------------------------------------------------
    def param_id(self, name_or_id: str | int) -> int:
        if isinstance(name_or_id, int):
            return name_or_id
        s = str(name_or_id)
        if s.lstrip("-").isdigit() or s.lower().startswith("0x"):
            return int(s, 0)
        # TODO(protocol): stable param ids come from spec.toml (Appendix E lists names only).
        ids = getattr(self.link.messages, "PARAM_IDS", None) or {}
        try:
            return ids[s.upper()]
        except KeyError:
            raise KeyError(f"unknown param {s!r} (TODO(protocol): name->id table)") from None

    def param_get(self, pid: str | int) -> Any:
        """PARAM_GET -> PARAM_VALUE(id, value, min, max, default)."""
        # TODO(protocol): PARAM_GET fields per Appendix E: id u8.
        return self.link.request("PARAM_GET", {"id": self.param_id(pid)})

    def param_set(self, pid: str | int, value: int, persist: bool = False) -> None:
        # TODO(protocol): PARAM_SET fields per Appendix E: id u8, value i16, persist u8.
        self.link.request("PARAM_SET", {"id": self.param_id(pid), "value": int(value),
                                        "persist": 1 if persist else 0})

    # ---- misc -------------------------------------------------------------------
    def stats(self) -> Any:
        msg = self.link.request("STATS_REQ")
        with self.state.lock:
            self.state.last_stats = msg
        return msg

    def ping(self, token: int = 0x1234) -> float:
        """Round trip time in seconds. TODO(protocol): PING token u16 / PONG."""
        t0 = time.monotonic()
        self.link.request("PING", {"token": token})
        return time.monotonic() - t0

    def set_stream(self, telem_div: int = 2, joints_div: int = 0, imu_raw_div: int = 0) -> None:
        # TODO(protocol): SET_STREAM fields per Appendix E.
        self.link.request("SET_STREAM", {"telem_div": telem_div, "joints_div": joints_div,
                                         "imu_raw_div": imu_raw_div})

    def reboot(self) -> None:
        # TODO(protocol): REBOOT magic u16 = 0x4B49.
        self.link.request("REBOOT", {"magic": consts.REBOOT_MAGIC})

    def _count_events(self, name: str) -> int:
        code = consts.EV[name]
        with self.state.lock:
            return sum(1 for _, e in self.state.events if e.code == code)


def _field(msg: Any, name: str) -> Any:
    if msg is None:
        raise LinkError("no reply")
    return getattr(msg, name)
