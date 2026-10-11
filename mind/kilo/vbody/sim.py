"""A simulated body for testing the mind without hardware (§6).

§6's eventual virtual body is ``body/test/vbody``: the real firmware ``core/``
on stub HALs. This module is the Python stand-in. It speaks the real protocol
(the vendored COBS/CRC codec and the generated message dataclasses) and
implements the P0 body's message-level contract, the same one the C link layer
implements (docs/architecture.md §4 "P0 dispatch"); a cross-check test feeds
both the same frames and compares the replies:

- every CRC-valid frame feeds the link watchdog (armed by the first one);
  after a trip the next frame gives EVENT LINK_RESTORED;
- unknown type → ACK UNKNOWN_TYPE; bad payload length → ACK BAD_LEN for a
  command, EVENT GOAL_REFUSED(type, BAD_LEN) for a goal; body→Pi types ignored;
- goals are never acked. SET_POSTURE: BAD_ARG / STALE / (FALLEN, sim only) /
  LOW_BATT / UNCALIBRATED refusals, LIMP and REST never stale; BEEP accepted;
  SET_BALANCE (P1) accepted (BAD_ARG if enable > 1 or a band/clamp > 30 deg);
  the other goals (gait, head, body pose, eyes) are refused UNKNOWN_TYPE.
  At most one GOAL_REFUSED per goal type per second;
- balance (P1): the SET_POSTURE balance flag (cleared by an epoch bump) or
  SET_BALANCE enable turns it on in SIT and STAND outside any reflex; TELEMETRY
  flags bit 0 mirrors that, and the simulated pitch/roll wobble shrinks. The
  controller itself is not simulated;
- queries answer with their data reply only (ACK only on failure), never
  de-duplicated;
- other commands get exactly one ACK; a repeat of the last executed
  (seq, type) within 500 ms is re-ACKed with the cached status and not
  executed again. BAD_ARG is checked before EEPROM_BUSY;
- the two-stage watchdog (stop + epoch bump + EVENT WATCHDOG, REST, LIMP)
  driven by the WD_* params; a simulated fall (EVENT FELL, epoch bump);
- boot: unsolicited HELLO and EVENT BOOT; TELEMETRY at the TELEM_DIV divider.

Two ways to drive it:

- ``start()``: a thread runs the 50 Hz tick and reads the transport (the CLI's
  ``kilo vbody`` on a pty, integration tests on a loopback);
- ``process(data, now)``: synchronous and deterministic. Feeds bytes through
  the decoder and handles every frame inline; every reply and event is written
  at once to the transport (if any) and appended to ``sent`` as
  ``(type, seq, payload)``. ``take_sent()`` returns and clears that list.
"""

from __future__ import annotations

import math
import os
import threading
import time
import tty
from collections import deque
from typing import Any, Callable

from kilo.link import consts
from kilo.link.codec import Codec, DecodeError, default_codec
from kilo.link.messages import Messages, default_messages
from kilo.link.transport import FdTransport, Transport
from kilo.protocol import kilo_protocol as P

EV = consts.EventCode

TICK_S = P.TICK_MS / 1000.0
DEDUP_WINDOW_S = P.DEDUP_WINDOW_MS / 1000.0
REFUSE_INTERVAL_S = P.GOAL_REFUSED_MIN_INTERVAL_MS / 1000.0
FW_BUILD = 0x5642_4F44  # "VBOD"
POSTURE_NAMES = {int(p): p.name for p in P.Posture}
CAL_LIMIT_A16 = 400
EEPROM_SIZE = 1024
# Goals the body implements; every other goal is refused UNKNOWN_TYPE.
P0_GOALS = frozenset({P.MsgId.HEARTBEAT, P.MsgId.SET_POSTURE, P.MsgId.BEEP, P.MsgId.SET_BALANCE})
# SET_BALANCE defaults (body/src/core/balance.h) and the posture it runs in.
BALANCE_DEFAULTS = {"enable": 0, "kp_q8": 128, "kd_ms": 30, "deadband_deg": 2, "max_corr_deg": 15}
BALANCE_MAX_DEG = 30
BALANCE_POSTURES = ("SIT", "STAND")


def open_pty() -> tuple[FdTransport, str, int]:
    """Open a pty pair. Returns (transport on the master, slave path, slave fd).

    The slave fd is kept open by the caller so the master never sees EIO
    while no client is attached.
    """
    master, slave = os.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    return FdTransport(master), os.ttyname(slave), slave


class VirtualBody:
    def __init__(self, transport: Transport | None = None, codec: Codec | None = None,
                 messages: Messages | None = None, *, telem_div: int | None = None,
                 fall_after: float | None = None, verbose: bool = False,
                 config_state: int = P.ConfigState.OK, batt_mv: int = 8100,
                 batt_level: int = 0, reset_cause: int = P.ResetCause.COLD,
                 fw_build: int = FW_BUILD, save_busy_s: float = 0.1,
                 clock: Callable[[], float] = time.monotonic):
        self.t = transport
        self.codec = codec or default_codec()
        self.msgs = messages or default_messages()
        self.decoder = self.codec.new_decoder()
        self.clock = clock
        self.verbose = verbose
        self.fall_after = fall_after
        self.config_state = int(config_state)
        self.fw_build = fw_build
        self.save_busy_s = save_busy_s
        self.lock = threading.RLock()
        self.params: dict[int, int] = {i: p.default for i, p in P.PARAMS.items()}
        self.params_saved: dict[int, int] = dict(self.params)
        if telem_div is not None:
            self.params[P.ParamId.TELEM_DIV] = telem_div
        self.telem_div = self.params[P.ParamId.TELEM_DIV]
        self.joints_div = 0
        self.imu_raw_div = 0
        self.cal: list[int] = [0] * P.CHANNEL_COUNT
        self.cal_saved: list[int] = list(self.cal)
        self.imu_offsets: list[int] = [0] * 6
        self.eeprom = bytearray(b"\xff" * EEPROM_SIZE)
        self.batt_mv = batt_mv
        self.batt_level = batt_level
        self.sent: deque[tuple[int, int, bytes]] = deque(maxlen=5000)
        self.received: list[tuple[str, int, bytes]] = []
        self.ignore_types: set[str] = set()  # tests: pretend these frames were lost
        self.seq = 0
        self.rx_crc_err = 0
        self.rx_overflow = 0
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_time = self.clock()
        self._power_on(int(reset_cause), 0)

    def _power_on(self, cause: int, last_alive: int) -> None:
        self.reset_cause = cause
        self.last_alive_tick = last_alive
        self.tick = 0
        self.epoch = 0
        self.posture = "LIMP"  # cold boot stays LIMP
        self.last_rx_seq = 0
        self.last_cmd: tuple[int, int, int, float] | None = None  # (seq, type, status, at)
        self.wd_armed = False
        self.wd_stage = 0  # 0 ok, 1 stopped, 2 rest, 3 limp
        self.last_rx_at: float | None = None
        self.fall_until: float | None = None
        self.recover_until: float | None = None
        self.fall_side = 1
        self.goals: dict[str, bytes] = {}
        self.balance_req = False  # SET_POSTURE flags bit 0
        self.balance: dict[str, int] = dict(BALANCE_DEFAULTS)  # SET_BALANCE (live, not persisted)
        self.refused_at: dict[int, float] = {}
        self.save_until: float | None = None
        self.booted_at = self.clock()

    # ---- derived state --------------------------------------------------------
    @property
    def mode(self) -> int:
        if self.fall_until is not None:
            return P.Mode.FALLEN
        if self.recover_until is not None:
            return P.Mode.RECOVERING
        if self.batt_level >= P.BatteryLevel.LOW:
            return P.Mode.LOW_BATT
        if self.wd_stage:
            return P.Mode.LINK_LOST
        return P.Mode.LIMP if self.posture == "LIMP" else P.Mode.IDLE

    @property
    def reflex(self) -> bool:
        return (self.wd_stage >= 2 or self.fall_until is not None or self.recover_until is not None
                or self.batt_level >= P.BatteryLevel.LOW)

    @property
    def balance_active(self) -> bool:
        """TELEMETRY flags bit 0, as the body gates it (§3c): requested, SIT or STAND, no reflex."""
        return ((self.balance_req or bool(self.balance["enable"])) and self.posture in BALANCE_POSTURES
                and not self.reflex)

    def _epoch_bump(self) -> None:
        self.epoch = (self.epoch + 1) & 0xFF
        self.goals.clear()
        self.balance_req = False  # the posture flag is a motion goal; SET_BALANCE settings stay

    def save_busy(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else now
        return self.save_until is not None and now < self.save_until

    # ---- control ---------------------------------------------------------------
    def start(self) -> "VirtualBody":
        self._running.set()
        self._thread = threading.Thread(target=self.run, name="vbody", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(1.0)

    def trigger_fall(self, side: int = 1) -> None:
        with self.lock:
            if self.fall_until is not None:
                return
            self.fall_side = side
            self._epoch_bump()
            self.fall_until = self.clock() + 1.0
            self._event(EV.FELL, side)
        self._say("fell")

    def set_battery(self, level: int) -> None:
        """Simulate a battery level change (BatteryLevel); LOW or worse bumps the epoch."""
        with self.lock:
            prev, self.batt_level = self.batt_level, int(level)
            if level > prev:
                if level >= P.BatteryLevel.LOW and prev < P.BatteryLevel.LOW:
                    self._epoch_bump()
                    if self.posture != "LIMP":
                        self.posture = "REST"
                self._event(EV.LOW_BATTERY, level)
            elif level == 0 and prev:
                self._event(EV.BATTERY_OK, 0)

    def reboot(self, cause: int = P.ResetCause.REQUESTED) -> None:
        with self.lock:
            self._power_on(int(cause), self.tick)
            self._boot()

    def take_sent(self) -> list[tuple[int, int, bytes]]:
        with self.lock:
            out = list(self.sent)
            self.sent.clear()
            return out

    # ---- loop ------------------------------------------------------------------
    def run(self) -> None:
        self._running.set()
        with self.lock:
            self._boot()
        next_tick = self.clock()
        while self._running.is_set():
            now = self.clock()
            if now < next_tick:
                # read while waiting for the tick deadline
                data = b""
                if self.t is not None:
                    try:
                        data = self.t.read(256)
                    except OSError:
                        time.sleep(0.005)
                else:
                    time.sleep(min(0.005, next_tick - now))
                if data:
                    self.process(data)
                continue
            next_tick += TICK_S
            if now - next_tick > 0.2:
                next_tick = now  # we were suspended; do not burst
            with self.lock:
                self._tick(now)

    def _boot(self) -> None:
        self._send_hello()
        self._event(EV.BOOT, self.reset_cause)
        self._say(f"boot ({consts.RESET_CAUSES.get(self.reset_cause)}) epoch {self.epoch}")

    def _send_hello(self) -> None:
        self._send("HELLO", proto_major=P.PROTO_MAJOR, proto_minor=P.PROTO_MINOR,
                   fw_build=self.fw_build, reset_cause=self.reset_cause,
                   last_alive_tick=self.last_alive_tick,
                   caps=int(P.HelloCaps.BUZZER | P.HelloCaps.DMP | P.HelloCaps.ULTRASONIC),
                   max_payload=P.MAX_PAYLOAD, trick_capacity=0, joint_count=P.JOINT_COUNT,
                   config_state=self.config_state, epoch=self.epoch)

    def _housekeeping(self, now: float) -> None:
        if self.save_until is not None and now >= self.save_until:
            self.save_until = None
            # As on the body: a calibration written by the Pi replaces the
            # model defaults, so STAND is no longer refused UNCALIBRATED.
            if self.config_state == P.ConfigState.DEFAULTED:
                self.config_state = P.ConfigState.OK
            self._event(EV.EEPROM_SAVED, 0)

    def _tick(self, now: float) -> None:
        self.tick = (self.tick + 1) & 0xFFFF
        self._housekeeping(now)
        if self.fall_after is not None and now - self._start_time >= self.fall_after:
            self.fall_after = None
            self.trigger_fall()
        # fall / recovery timeline
        if self.fall_until is not None and now >= self.fall_until:
            self.fall_until = None
            self.recover_until = now + 1.0
        if self.recover_until is not None and now >= self.recover_until:
            self.recover_until = None
            self.posture = "REST"
            self._event(EV.RECOVERED, 0)
            self._say("recovered")
        # watchdog (params in ms / s, like the body)
        if self.wd_armed and self.last_rx_at is not None:
            silent = now - self.last_rx_at
            stop_s = self.params[P.ParamId.WD_STOP_MS] / 1000.0
            rest_s = max(stop_s, self.params[P.ParamId.WD_REST_MS] / 1000.0)
            limp_s = self.params[P.ParamId.WD_LIMP_S]
            if self.wd_stage == 0 and silent >= stop_s:
                self.wd_stage = 1
                self._epoch_bump()
                self._event(EV.WATCHDOG, 0)
                self._say("watchdog: stop")
            if self.wd_stage == 1 and silent >= rest_s:
                self.wd_stage = 2
                if self.posture != "LIMP":  # already limp stays limp
                    self.posture = "REST"
                self._say("watchdog: rest")
            if self.wd_stage == 2 and silent >= rest_s + limp_s:
                self.wd_stage = 3
                self.posture = "LIMP"
                self._say("watchdog: limp")
        if self.tick % 500 == 0:
            self.batt_mv = max(6600, self.batt_mv - 1)
        if self.telem_div and self.tick % self.telem_div == 0:
            self._send_telemetry(now)

    def _send_telemetry(self, now: float) -> None:
        t = now - self._start_time
        pitch = 2.0 * math.sin(t * 0.7)
        roll = 1.5 * math.sin(t * 1.1)
        F = P.TelemetryFlags
        flags = 0
        if self.balance_active:
            flags |= F.BALANCE_ACTIVE
            pitch, roll = pitch * 0.25, roll * 0.25  # the body holds itself level
        if self.fall_until is not None:
            roll = 85.0 * self.fall_side
        if self.posture != "LIMP":
            flags |= F.SERVOS_POWERED
        if self.wd_armed and self.wd_stage == 0:
            flags |= F.LINK_OK
        if self.batt_level >= P.BatteryLevel.WARN:
            flags |= F.BATT_WARN
        if self.save_busy(now):
            flags |= F.EEPROM_PENDING
        owner = int(P.Owner.REFLEX if self.reflex else P.Owner.IDLE)
        self._send("TELEMETRY", tick=self.tick, pitch=int(pitch * 16), roll=int(roll * 16),
                   rate_pitch=int(1.4 * math.cos(t * 0.7) * 16), rate_roll=int(1.6 * math.cos(t * 1.1) * 16),
                   rate_yaw=0, dist_mm=600 + int(100 * math.sin(t * 0.3)), batt_mv=self.batt_mv,
                   mode=int(self.mode), owners=owner | owner << 4, flags=int(flags),
                   last_rx_seq=self.last_rx_seq, trick_frame=0xFF, gait_phase=0, tick_load=31,
                   rx_err=(self.rx_crc_err + self.rx_overflow) & 0xFF, epoch=self.epoch)

    # ---- rx ----------------------------------------------------------------------
    def process(self, data: bytes, now: float | None = None) -> None:
        """Decode ``data`` and handle every complete frame now (deterministic)."""
        with self.lock:
            now = self.clock() if now is None else now
            self._housekeeping(now)
            for item in self.decoder.feed(data):
                self._rx(item, now)

    def flush_events(self) -> None:
        """Events are written as they happen; nothing is queued (cross-check API)."""

    def _link_frame(self, now: float) -> None:
        """Any CRC-valid frame from the Pi feeds (and arms) the link watchdog."""
        self.last_rx_at = now
        self.wd_armed = True
        if self.wd_stage:
            self.wd_stage = 0
            self._event(EV.LINK_RESTORED, 0)
            self._say("link restored")

    def _rx(self, item: Any, now: float) -> None:
        if isinstance(item, DecodeError):
            if item.type is None:  # header untrusted
                if item.kind == "overflow":
                    self.rx_overflow += 1
                else:
                    self.rx_crc_err += 1
                return
            self._link_frame(now)
            self.last_rx_seq = item.seq
            if item.kind == "unknown_type":
                self._ack(item.seq, item.type, P.AckStatus.UNKNOWN_TYPE)
            elif item.type < 0x80:  # bad payload length on a known Pi->body type
                if P.MESSAGES[item.type].KIND is P.Kind.GOAL:
                    self._refuse(item.type, P.GoalRefusedReason.BAD_LEN, now)
                else:
                    self._ack(item.seq, item.type, P.AckStatus.BAD_LEN)
            return
        name = self.msgs.name_of(item.type)
        if name in self.ignore_types:
            return
        self.received.append((name, item.seq, item.payload))
        if len(self.received) > 2000:
            del self.received[:1000]
        self._link_frame(now)
        self.last_rx_seq = item.seq
        if item.type >= 0x80:
            return  # a body->Pi type: not for us
        cls = P.MESSAGES.get(item.type)
        if cls is None:  # only with a codec that does not check types
            return self._ack(item.seq, item.type, P.AckStatus.UNKNOWN_TYPE)
        try:
            msg = cls.unpack(item.payload)
        except P.PayloadError:  # only with a codec that does not check lengths
            if cls.KIND is P.Kind.GOAL:
                return self._refuse(item.type, P.GoalRefusedReason.BAD_LEN, now)
            return self._ack(item.seq, item.type, P.AckStatus.BAD_LEN)
        if cls.KIND is P.Kind.GOAL:
            self._goal(name, item.type, msg, item.payload, now)
        elif cls.IS_QUERY:
            self._query(name, item, msg)
        else:
            self._command(name, item, msg, now)

    # ---- goals -------------------------------------------------------------------
    def _goal(self, name: str, type: int, msg: Any, payload: bytes, now: float) -> None:
        R = P.GoalRefusedReason
        if type not in P0_GOALS:
            return self._refuse(type, R.UNKNOWN_TYPE, now)
        if type == P.MsgId.HEARTBEAT:
            return
        if type == P.MsgId.BEEP:
            self.goals[name] = payload
            return
        if type == P.MsgId.SET_BALANCE:
            if msg.enable > 1 or msg.deadband_deg > BALANCE_MAX_DEG or msg.max_corr_deg > BALANCE_MAX_DEG:
                return self._refuse(type, R.BAD_ARG, now)
            was = self.balance_active
            self.balance = {k: int(getattr(msg, k)) for k in BALANCE_DEFAULTS}
            self.goals[name] = payload
            if self.balance_active != was:
                self._say(f"balance {'on' if self.balance_active else 'off'}")
            return
        # SET_POSTURE
        if msg.posture >= len(P.Posture) or msg.posture == P.Posture.STANCE:
            return self._refuse(type, R.BAD_ARG, now)
        safe = msg.posture in (P.Posture.LIMP, P.Posture.REST)
        if not safe:
            if msg.epoch != self.epoch:
                return self._refuse(type, R.STALE, now)
            if self.fall_until is not None or self.recover_until is not None:
                return self._refuse(type, R.FALLEN, now)  # sim only: no fall reflex in the P0 body
            if self.batt_level >= P.BatteryLevel.LOW:
                return self._refuse(type, R.LOW_BATT, now)
            if msg.posture == P.Posture.STAND and self.config_state == P.ConfigState.DEFAULTED:
                return self._refuse(type, R.UNCALIBRATED, now)
        self.goals[name] = payload
        self.balance_req = bool(msg.flags & P.PostureFlags.BALANCE)
        new = POSTURE_NAMES[msg.posture]
        if new != self.posture:
            self._say(f"posture {new}")
        self.posture = new

    def _refuse(self, gtype: int, reason: int, now: float) -> None:
        if now - self.refused_at.get(gtype, -1e9) >= REFUSE_INTERVAL_S:
            self.refused_at[gtype] = now
            self._event(EV.GOAL_REFUSED, (gtype << 8) | int(reason))

    # ---- queries -----------------------------------------------------------------
    def _query(self, name: str, item: Any, msg: Any) -> None:
        M = P.MsgId
        t = item.type
        if t == M.HELLO_REQ:
            self._send_hello()
        elif t == M.PING:
            self._send("PONG", token=msg.token, tick=self.tick)
        elif t == M.STATS_REQ:
            self._send("STATS", uptime_s=int(self.clock() - self.booted_at), overruns=0,
                       max_tick_us=3100, stack_free_min=420, rx_crc_err=self.rx_crc_err,
                       rx_overflow=self.rx_overflow, i2c_err=0, imu_stale=0, dmp_rate_hz=100)
        elif t == M.PCA_READ:
            off = P.PCA_FULL_OFF_BIT
            self._send("PCA_COUNTS", counts=[off if self.posture == "LIMP" else 1500] * P.CHANNEL_COUNT)
        elif t == M.CAL_GET:
            self._send("CAL_VALUES", offset=list(self.cal))
        elif t == M.PARAM_GET:
            p = P.PARAMS.get(msg.id)
            if p is None:
                return self._ack(item.seq, t, P.AckStatus.BAD_ARG)
            self._send("PARAM_VALUE", id=p.id, value=self.params[p.id], min=p.min, max=p.max,
                       default=p.default)
        elif t == M.EEPROM_READ:
            if not 1 <= msg.len <= P.EEPROM_READ_MAX or msg.addr + msg.len > EEPROM_SIZE:
                return self._ack(item.seq, t, P.AckStatus.BAD_ARG)
            self._send("EEPROM_DATA", addr=msg.addr, data=bytes(self.eeprom[msg.addr:msg.addr + msg.len]))

    # ---- commands ----------------------------------------------------------------
    def _command(self, name: str, item: Any, msg: Any, now: float) -> None:
        lc = self.last_cmd
        if lc and lc[:2] == (item.seq, item.type) and now - lc[3] < DEDUP_WINDOW_S:
            self._ack(item.seq, item.type, lc[2])  # de-dup: re-ack, do not execute
            return
        if item.type == P.MsgId.REBOOT:
            if msg.magic != P.REBOOT_MAGIC:
                status = P.AckStatus.BAD_ARG
            else:
                self._ack(item.seq, item.type, P.AckStatus.OK)
                self.reboot(P.ResetCause.REQUESTED)
                return
        else:
            status = self._execute(item.type, msg, now)
        self.last_cmd = (item.seq, item.type, int(status), now)
        self._ack(item.seq, item.type, status)

    def _start_save(self, now: float) -> None:
        self.save_until = now + self.save_busy_s

    def _execute(self, t: int, msg: Any, now: float) -> int:
        M, S = P.MsgId, P.AckStatus
        busy = self.save_busy(now)
        if t == M.SET_STREAM:
            self.telem_div, self.joints_div, self.imu_raw_div = msg.telem_div, msg.joints_div, msg.imu_raw_div
            return S.OK
        if t == M.CAL_SET:
            if msg.joint >= P.CHANNEL_COUNT or abs(msg.offset_a16) > CAL_LIMIT_A16:
                return S.BAD_ARG
            if busy:
                return S.EEPROM_BUSY
            self.cal[msg.joint] = msg.offset_a16
            return S.OK
        if t == M.CAL_SAVE:
            if busy:
                return S.EEPROM_BUSY
            self.cal_saved = list(self.cal)
            self._start_save(now)
            return S.OK
        if t == M.CAL_REVERT:
            if busy:
                return S.EEPROM_BUSY
            self.cal = list(self.cal_saved)
            return S.OK
        if t == M.IMU_OFFSETS_SET:
            if msg.save > 1:
                return S.BAD_ARG
            if busy:
                return S.EEPROM_BUSY
            self.imu_offsets = [msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z]
            if msg.save:
                self._start_save(now)
            return S.OK
        if t == M.PARAM_SET:
            p = P.PARAMS.get(msg.id)
            if p is None or not p.min <= msg.value <= p.max or msg.persist > 1:
                return S.BAD_ARG
            if msg.persist and busy:
                return S.EEPROM_BUSY
            self.params[p.id] = msg.value
            if msg.persist:
                self.params_saved[p.id] = msg.value
                self._start_save(now)
            return S.OK
        return S.UNKNOWN_TYPE  # TRICK_*: no trick store in P0

    # ---- tx ------------------------------------------------------------------------
    def _ack(self, seq: int, type: int, status: int) -> None:
        self._send("ACK", seq=seq, type=type, status=int(status))

    def _event(self, code: int, arg: int) -> None:
        arg = int(arg)
        arg = arg - 0x10000 if arg >= 0x8000 else arg
        self._send("EVENT", code=int(code), arg=arg, tick=self.tick, epoch=self.epoch)

    def _send(self, name: str, **fields: Any) -> None:
        typ = self.msgs.type_of(name)
        payload = self.msgs.pack(name, **fields)
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        self.sent.append((typ, seq, payload))
        if self.t is None:
            return
        try:
            self.t.write(self.codec.encode(typ, seq, payload))
        except OSError:
            pass

    def _say(self, text: str) -> None:
        if self.verbose:
            print(f"[vbody {self.clock() - self._start_time:7.2f}s] {text}", flush=True)
