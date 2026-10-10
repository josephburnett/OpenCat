"""BodyClient: the mind's P0 API to the body (docs/architecture.md §5, §7 P0).

Thin and synchronous: goals go through ``SerialLink.set_goal`` (latched,
refreshed at 5 Hz, epoch-stamped, cleared on an epoch change); commands go
through ``SerialLink.request`` (seq, 50 ms wait, 3 retries; queries answered
by their data reply, other commands by an ACK, de-dup safe). Message layouts
and field names are the generated ones (protocol/spec.toml, Appendix E).
"""

from __future__ import annotations

import time
from typing import Any

from kilo.link import consts
from kilo.link.serial_link import LinkError, LinkTimeout, SerialLink


class ProtocolMismatch(LinkError):
    pass


# Postures the CLI offers. "balance" is STAND with the balance flag (SET_POSTURE
# flags bit 0); it is not a separate posture code.
POSTURE_CHOICES = ("limp", "rest", "stand", "sit", "balance", "stance", "calib", "zero")

CAL_LIMIT_A16 = 400  # CAL_SET offset_a16 range: ±25°


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
        """Latch SET_POSTURE (epoch, posture, transition_ticks 0 = default, flags).

        ``balance`` sets flags bit 0; the name "balance" means STAND with it.
        """
        self._check_drivable()
        name = name.lower()
        if name == "balance":
            code, balance = consts.POSTURES["stand"], True if balance is None else balance
        else:
            try:
                code = consts.POSTURES[name]
            except KeyError:
                raise ValueError(f"unknown posture {name!r}") from None
        if not 0 <= transition_ticks <= 255:
            raise ValueError("transition_ticks must be 0..255")
        self.link.set_goal("SET_POSTURE", posture=code, transition_ticks=transition_ticks,
                           flags=consts.POSTURE_BALANCE_FLAG if balance else 0)

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
        """CAL_GET -> CAL_VALUES: 16 absolute offsets in a16 (1/16°)."""
        msg = self.link.request("CAL_GET")
        return list(_field(msg, "offset"))

    def cal_set(self, joint: int, offset_a16: int) -> None:
        """CAL_SET: absolute offset for one joint, ±25° (±400 a16)."""
        if not 0 <= joint < consts.CAL_JOINTS:
            raise ValueError(f"joint {joint} out of range 0..{consts.CAL_JOINTS - 1}")
        if not -CAL_LIMIT_A16 <= offset_a16 <= CAL_LIMIT_A16:
            raise ValueError("offset beyond ±25°")
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

    def imu_offsets_set(self, offsets: list[int], save: bool = False) -> None:
        """IMU_OFFSETS_SET: accel xyz, gyro xyz (MPU6050 offset registers)."""
        if len(offsets) != 6:
            raise ValueError("need 6 offsets: accel x y z, gyro x y z")
        names = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z")
        self.link.request("IMU_OFFSETS_SET", {**dict(zip(names, offsets)), "save": 1 if save else 0})

    # ---- params (commands) ------------------------------------------------------
    @staticmethod
    def param_info(name_or_id: str | int) -> Any:
        """The generated ParamInfo (id, name, default, min, max, unit, doc)."""
        pid = BodyClient.param_id(name_or_id)
        try:
            return consts.PARAMS[pid]
        except KeyError:
            raise KeyError(f"unknown param id {pid}") from None

    @staticmethod
    def param_id(name_or_id: str | int) -> int:
        """Param id from a number ("5", "0x05") or a name (case-insensitive)."""
        if isinstance(name_or_id, int):
            return name_or_id
        s = str(name_or_id).strip()
        if s.lstrip("-").isdigit() or s.lower().startswith("0x"):
            return int(s, 0)
        try:
            return consts.PARAMS_BY_NAME[s.upper()].id
        except KeyError:
            raise KeyError(f"unknown param {s!r} (known: {', '.join(consts.PARAMS_BY_NAME)})") from None

    def param_get(self, pid: str | int) -> Any:
        """PARAM_GET -> PARAM_VALUE(id, value, min, max, default)."""
        return self.link.request("PARAM_GET", {"id": self.param_id(pid)})

    def param_set(self, pid: str | int, value: int, persist: bool = False) -> None:
        """PARAM_SET (acked when applied; with persist, EVENT EEPROM_SAVED follows)."""
        i = self.param_id(pid)
        info = consts.PARAMS.get(i)
        if info is not None and not info.min <= int(value) <= info.max:
            raise ValueError(f"{info.name} must be {info.min}..{info.max} {info.unit}")
        self.link.request("PARAM_SET", {"id": i, "value": int(value), "persist": 1 if persist else 0})

    def param_list(self) -> list[Any]:
        """PARAM_VALUE for every param the mind knows (spec.toml table)."""
        return [self.param_get(i) for i in sorted(consts.PARAMS)]

    # ---- misc -------------------------------------------------------------------
    def stats(self) -> Any:
        msg = self.link.request("STATS_REQ")
        with self.state.lock:
            self.state.last_stats = msg
        return msg

    def pca_counts(self) -> list[int]:
        """PCA_READ -> PCA_COUNTS: 16 OFF counts (bit 12 = full-off)."""
        return list(_field(self.link.request("PCA_READ"), "counts"))

    def eeprom_read(self, addr: int, length: int) -> bytes:
        """EEPROM_READ -> EEPROM_DATA (≤ 56 bytes per request)."""
        return bytes(_field(self.link.request("EEPROM_READ", {"addr": addr, "len": length}), "data"))

    def ping(self, token: int = 0x1234) -> float:
        """PING -> PONG. Round-trip time in seconds."""
        t0 = time.monotonic()
        msg = self.link.request("PING", {"token": token})
        if _field(msg, "token") != token:
            raise LinkError(f"PONG token {msg.token:#06x} != {token:#06x}")
        return time.monotonic() - t0

    def set_stream(self, telem_div: int = 2, joints_div: int = 0, imu_raw_div: int = 0) -> None:
        """SET_STREAM: divisors of the 50 Hz tick (0 = off)."""
        self.link.request("SET_STREAM", {"telem_div": telem_div, "joints_div": joints_div,
                                         "imu_raw_div": imu_raw_div})

    def reboot(self) -> None:
        """REBOOT (watchdog restart of the application)."""
        self.link.request("REBOOT", {"magic": consts.REBOOT_MAGIC})

    def _count_events(self, name: str) -> int:
        code = consts.EV[name]
        with self.state.lock:
            return sum(1 for _, e in self.state.events if e.code == code)


def _field(msg: Any, name: str) -> Any:
    if msg is None:
        raise LinkError("no reply")
    return getattr(msg, name)
