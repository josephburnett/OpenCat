"""A simulated body for testing the mind without hardware (§6).

§6's virtual body on the real firmware ``core/`` is ``kilo.vbody.core``
(body/test/core_harness.cpp); this module is the Python stand-in, and
tests/test_corebody.py lists where the two differ (the core is right). It
speaks the real protocol (the vendored COBS/CRC codec and the generated message dataclasses) and
implements the P0 body's message-level contract, the same one the C link layer
implements (docs/architecture.md §4 "P0 dispatch"); a cross-check test feeds
both the same frames and compares the replies:

- every CRC-valid frame feeds the link watchdog (armed by the first one);
  after a trip the next frame gives EVENT LINK_RESTORED;
- unknown type → ACK UNKNOWN_TYPE; bad payload length → ACK BAD_LEN for a
  command, EVENT GOAL_REFUSED(type, BAD_LEN) for a goal; body→Pi types ignored;
- goals are never acked. SET_POSTURE: BAD_ARG / STALE / FALLEN / LOW_BATT /
  UNCALIBRATED refusals, LIMP and REST never stale, every posture but LIMP
  refused FALLEN while the fall reflex holds the joints (LIMP ends it; LIMP
  also ends a trick and a gait at once); STANCE (P2) accepted like SIT; BEEP
  accepted; SET_BALANCE (P1) accepted (BAD_ARG if enable > 1 or a band/clamp
  > 30 deg); SET_HEAD (P1, §3c layer 1): BAD_ARG for mode > 1, mode 0
  releases (never stale), mode 1 holds (STALE, FALLEN, LOW_BATT) until
  HEAD_HOLD_MS without a refresh or an epoch bump: TELEMETRY flag HEAD_HELD
  and head owner PI_HOLD (unless a trick owns the head or a reflex holds
  it); SET_EYES is refused UNKNOWN_TYPE. At most one GOAL_REFUSED per goal
  type per second;
- the gait (P2, §3d; the body's gait generator and IK are not simulated, only
  its state machine): SET_GAIT BAD_ARG for a field outside its spec range,
  an unknown pattern or flag bits; run=0 is never stale and stops; run=1:
  STALE, FALLEN, LOW_BATT, UNCALIBRATED (config DEFAULTED, as STAND). A start
  from another posture first moves to STANCE (POSTURE_TICKS ticks), then the
  phase starts at 0 and advances 65536·20/period_ms per tick (TELEMETRY
  gait_phase = phase >> 8); the stride ramps up over one period. From the
  accepted run=1 to the end: mode GAIT, flag GAIT, leg owner GAIT. A refresh
  updates the parameters; another pattern is a stop then a start (no
  GAIT_STOPPED in between). run=0: the stride ramps down over one period, then
  posture STANCE, EVENT GAIT_STOPPED (arg 0) once, mode IDLE. The watchdog's
  epoch bump (stage 1) ramps a running gait down like run=0 and stage 2's
  REST waits for it (§3g: Kilo never lies down mid-stride); a fall, LOW
  battery or SET_POSTURE LIMP ends it at once (a reflex took the legs). Either
  way GAIT_STOPPED carries the new epoch. A trick owns the legs it moves (TRICK
  owner, mode TRICK) while the gait keeps its phase underneath and resumes
  after. SET_POSTURE while walking only changes the posture under the gait
  (the stop hands over to STANCE). SET_BODY_POSE: STALE, FALLEN, LOW_BATT (the
  spec gives its fields no range: no BAD_ARG); the pose tilts the reported
  pitch/roll in STANCE and while walking; an epoch bump resets it;
- JOINTS at the SET_STREAM joints divider: fake angles (LIMP: -32768), legs
  oscillating with the gait phase, head tilt bob and tail wag when flagged;
- balance (P1): the SET_POSTURE balance flag or SET_BALANCE enable (both
  cleared by an epoch bump, as on the body; the link refreshes SET_BALANCE at
  5 Hz) turns it on in SIT and STAND outside any reflex; TELEMETRY
  flags bit 0 mirrors that, and the simulated pitch/roll wobble shrinks. The
  controller itself is not simulated;
- tricks (P1, docs §3e): the body's upload rules (TRICK_BEGIN / TRICK_FRAMES
  in order / TRICK_COMMIT with the CRC, idempotent as the cache check), the
  TRICK_PLAY gating (STALE, NO_TRICK, BAD_ARG, BUSY, LOW_BATT, UNCALIBRATED)
  and the player's tick-exact timing (kilo.tricks.player, the same stepping
  as core/keyframe.cpp), so EVENT TRICK_DONE(reason << 8 | id) arrives on the
  same tick as on the body: DONE, STOPPED (TRICK_STOP, SET_POSTURE LIMP),
  PREEMPTED (an epoch bump), FELL (the simulated fall). The posture becomes
  the trick's end posture; TELEMETRY shows mode TRICK, the TRICK flag, the
  frame and the TRICK owner. Joint angles are not simulated;
- queries answer with their data reply only (ACK only on failure), never
  de-duplicated;
- other commands get exactly one ACK; a repeat of the last executed
  (seq, type) within 500 ms is re-ACKed with the cached status and not
  executed again. BAD_ARG is checked before EEPROM_BUSY;
- the two-stage watchdog (stop + epoch bump + EVENT WATCHDOG, REST, LIMP)
  driven by the WD_* params; a simulated fall (``trigger_fall``: EVENT
  FELL(+-1) with the epoch bump, 1 s FALLEN, 1 s RECOVERING, then STAND (REST
  if uncalibrated) and EVENT RECOVERED; the body's rc timing is not
  simulated); TRICK_PLAY is refused REFUSED_FALLEN meanwhile;
- the IR remote (``ir_key``): EVENT IR_KEY as the body's decoder sends it;
- boot: unsolicited HELLO and EVENT BOOT; TELEMETRY at the TELEM_DIV divider.
  ``boot_delay`` (seconds, default 0) mimics the real body's 0.3-2 s init:
  after a reboot the body is silent for that long (no tick, no telemetry) and
  the bytes that arrive meanwhile are buffered, as in the UART's RX buffer,
  and handled right after the boot HELLO.
- boot epoch (doc §3c): the first power-on starts at epoch 0 (the link
  harness's fresh body); every ``reboot()`` starts at the previous run's
  last epoch + 1, skipping 0 (``boot_epoch``, the body's reset_boot_epoch),
  so goals stamped before the reboot are STALE after it.

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
from kilo.protocol import kilo_link_codec as LC
from kilo.protocol import kilo_protocol as P
from kilo.tricks.player import Player

EV = consts.EventCode

TICK_S = P.TICK_MS / 1000.0
DEDUP_WINDOW_S = P.DEDUP_WINDOW_MS / 1000.0
REFUSE_INTERVAL_S = P.GOAL_REFUSED_MIN_INTERVAL_MS / 1000.0
FW_BUILD = 0x5642_4F44  # "VBOD"
POSTURE_NAMES = {int(p): p.name for p in P.Posture}
CAL_LIMIT_A16 = 400
EEPROM_SIZE = 1024
# Goals the body implements; every other goal is refused UNKNOWN_TYPE.
P0_GOALS = frozenset({P.MsgId.HEARTBEAT, P.MsgId.SET_POSTURE, P.MsgId.BEEP, P.MsgId.SET_BALANCE,
                      P.MsgId.SET_HEAD})
P2_GOALS = frozenset({P.MsgId.SET_GAIT, P.MsgId.SET_BODY_POSE})
GAIT_RANGES = consts.field_ranges("SET_GAIT")
GAIT_FLAGS = int(P.GaitFlags.BOB | P.GaitFlags.WAG | P.GaitFlags.BALANCE)
# Leg phase offsets (fractions of a cycle) in the order LF, RF, RB, LB (§3d).
LEG_OFFSETS = {P.GaitPattern.WALK: (0.50, 0.00, 0.25, 0.75), P.GaitPattern.CRAWL: (0.50, 0.00, 0.25, 0.75),
               P.GaitPattern.TROT: (0.00, 0.50, 0.00, 0.50)}
# Fake joint angles (degrees, canonical order: pan, tilt, tail, shoulders
# LF RF RB LB, knees LF RF RB LB) for the JOINTS stream. Not the body's tables.
FAKE_ANGLES = {"REST": (0, 0, 0, -60, -60, 60, 60, 60, 60, -60, -60),
               "SIT": (0, 20, 0, 30, 30, 75, 75, -30, -30, 60, 60),
               "CALIB": (0,) * 11, "ZERO": (0,) * 11}
FAKE_STAND = (0, 0, 0, 30, 30, -30, -30, 30, 30, -30, -30)
BOB_DEG, WAG_DEG = 4.0, 15.0  # §3d head bob and tail wag amplitudes
HEAD_HOLD_S = P.HEAD_HOLD_MS / 1000.0
REST_MOVE_S = 1.0  # REST_MOVE_TICKS on the body: reflex moves to REST take 1 s


def boot_epoch(last_epoch: int) -> int:
    """The epoch a reboot starts at (body core/reset.cpp reset_boot_epoch):
    the previous run's last epoch + 1, skipping 0."""
    e = (last_epoch + 1) & 0xFF
    return e or 1

# SET_BALANCE defaults (body/src/core/balance.h) and the posture it runs in.
BALANCE_DEFAULTS = {"enable": 0, "kp_q8": 128, "kd_ms": 30, "deadband_deg": 2, "max_corr_deg": 15}
BALANCE_MAX_DEG = 30
BALANCE_POSTURES = ("SIT", "STAND")
# Tricks (body/src/core/keyframe.cpp).
TRICK_DRIVEN_MASK = 0xFF07          # joint ids 0-2 and 8-15
TRICK_END_POSTURES = frozenset({P.Posture.REST, P.Posture.SIT, P.Posture.STAND, P.Posture.CALIB, P.Posture.ZERO})
TRICK_FLAGS = int(P.TrickFlags.BALANCE | P.TrickFlags.FALL_OK)
TDR = P.TrickDoneReason


class TrickStore:
    """The body's one-trick RAM store: header, frames as sent, running CRC."""

    def __init__(self) -> None:
        self.state = "empty"  # empty, loading, ready
        self.hdr: Any = None
        self.frames: list[Any] = []
        self.crc = 0xFFFF
        self.player: Player | None = None
        self.mirror = False

    @property
    def running(self) -> bool:
        return self.player is not None and self.player.running

    def claims(self, joint: int) -> bool:
        """The TRICK owner holds joint id ``joint`` (L/R swapped when mirrored)."""
        if not self.running:
            return False
        if self.mirror and joint >= 8:
            joint ^= 1
        return bool(self.hdr.joint_mask >> joint & 1)


class Gait:
    """The gait generator's state machine (§3d "Start and stop"), no IK.

    ``state``: "stance" (moving to STANCE first), "run", "stop" (ramping the
    stride down). ``amp`` is the stride ramp 0..1; ``phase`` the u16 phi."""

    def __init__(self, msg: Any, to_stance_ticks: int = 0):
        self.phase = 0
        self.amp = 0.0
        self.t = 0
        self.state = "stance" if to_stance_ticks > 0 else "run"
        self.stance_ticks = to_stance_ticks
        self.restart: Any = None  # a pattern change: the SET_GAIT to start once stopped
        self.update(msg)

    def update(self, msg: Any) -> None:
        self.msg = msg
        self.pattern = int(msg.pattern)
        self.period_ticks = max(1, round(msg.period_ms / P.TICK_MS))
        self.step = 65536 * P.TICK_MS // msg.period_ms

    @property
    def flags(self) -> int:
        return int(self.msg.flags)

    def start_stop(self) -> None:
        if self.state != "stop":
            self.state, self.t = "stop", 0
            self._amp0 = self.amp

    def step_tick(self) -> bool:
        """One tick. Returns True when the gait has stopped (ramp-down done)."""
        self.t += 1
        if self.state == "stance":
            if self.t >= self.stance_ticks:
                self.state, self.t, self.phase = "run", 0, 0
            return False
        self.phase = (self.phase + self.step) & 0xFFFF
        if self.state == "run":
            self.amp = min(1.0, self.amp + 1.0 / self.period_ticks)
            return False
        self.amp = max(0.0, self._amp0 * (1.0 - self.t / self.period_ticks))
        return self.t >= self.period_ticks


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
                 clock: Callable[[], float] = time.monotonic, boot_delay: float = 0.0, p2: bool = True):
        self.t = transport
        # p2=False: the P1 body (SET_GAIT / SET_BODY_POSE refused UNKNOWN_TYPE,
        # SET_POSTURE STANCE BAD_ARG), for the cross-check against a firmware
        # without the gait generator.
        self.p2 = p2
        self.goal_types = P0_GOALS | P2_GOALS if p2 else P0_GOALS
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
        self.boot_delay = boot_delay
        self.booting_until: float | None = None  # silent init after a reboot (boot_delay)
        self._boot_rx = bytearray()
        self._power_on(int(reset_cause), 0)

    def _power_on(self, cause: int, last_alive: int, epoch: int = 0) -> None:
        self.reset_cause = cause
        self.last_alive_tick = last_alive
        self.tick = 0
        self.epoch = epoch
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
        self.head_until: float | None = None  # SET_HEAD hold: released at this time (None = not held)
        self.head: tuple[int, int, int] = (0, 0, 0)  # pan, tilt, tail a16 of the hold
        self.refused_at: dict[int, float] = {}
        self.save_until: float | None = None
        self.trick = TrickStore()
        self.gait: Gait | None = None
        self.pose: tuple[int, int, int, int] = (0, 0, 0, 0)  # SET_BODY_POSE pitch, roll (a16), dheight, xshift
        self.gait_stops = 0  # EVENT GAIT_STOPPED sent (tests)
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
        if self.trick.running:
            return P.Mode.TRICK
        if self.gait is not None:
            return P.Mode.GAIT
        return P.Mode.LIMP if self.posture == "LIMP" else P.Mode.IDLE

    @property
    def reflex(self) -> bool:
        return (self.wd_stage >= 2 or self.fall_until is not None or self.recover_until is not None
                or self.batt_level >= P.BatteryLevel.LOW)

    @property
    def fallen(self) -> bool:
        """The fall reflex holds the joints (FALLEN or RECOVERING)."""
        return self.fall_until is not None or self.recover_until is not None

    @property
    def balance_active(self) -> bool:
        """TELEMETRY flags bit 0, as the body gates it (§3c): requested, SIT or STAND, no reflex."""
        return ((self.balance_req or bool(self.balance["enable"])) and self.posture in BALANCE_POSTURES
                and not self.reflex)

    def _epoch_bump(self, trick_reason: int = TDR.PREEMPTED, gait_ramp: bool = False) -> None:
        self.epoch = (self.epoch + 1) & 0xFF
        self.goals.clear()
        self.balance_req = False  # the posture flag is a motion goal
        self.balance["enable"] = 0  # so is SET_BALANCE's enable (body_epoch_bump; the gains stay)
        self.head_until = None    # the head hold too
        self.pose = (0, 0, 0, 0)  # and the body pose
        self._trick_end(trick_reason)  # TRICK_DONE carries the new epoch
        # The body's bump stops the gait through its ramp (body_epoch_bump ->
        # gait_stop); a reflex taking the legs on the same tick (fall, low
        # battery) cuts it short. Only the watchdog leaves the legs to the gait.
        if gait_ramp:
            self._gait_stop("epoch bump")
        else:
            self._gait_end("stopped (epoch bump)")

    def _gait_stop(self, why: str = "run=0") -> None:
        """Stop through the ramp-down if the gait runs; at once if it has not
        started (still moving to STANCE), like the body's gait_stop()."""
        g = self.gait
        if g is None:
            return
        g.restart = None
        if g.state == "stance":
            self._gait_end(f"stopped before starting ({why})")
        else:
            g.start_stop()

    def _gait_end(self, why: str = "stopped") -> None:
        """The gait is over: EVENT GAIT_STOPPED once (with the current epoch)."""
        if self.gait is None:
            return
        self.gait = None
        self.gait_stops += 1
        self._event(EV.GAIT_STOPPED, 0)
        self._say(f"gait {why}")

    def _trick_end(self, reason: int) -> None:
        """End the playing trick: TRICK_DONE(reason << 8 | id), the end posture."""
        tr = self.trick
        if not tr.running:
            return
        tr.player = None
        self.posture = POSTURE_NAMES[tr.hdr.end_posture]
        self._event(EV.TRICK_DONE, (int(reason) << 8) | tr.hdr.trick_id)
        self._say(f"trick {tr.hdr.trick_id} {TDR(reason).name.lower()}")

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
            self._epoch_bump(TDR.FELL)
            self.fall_until = self.clock() + 1.0
            self._event(EV.FELL, side)
        self._say("fell")

    def ir_key(self, command: int, repeat: bool = False) -> None:
        """Simulate the IR remote (the body's NEC decoder on D4): EVENT
        IR_KEY(command | 0x100 for a held-key repeat). Test and `kilo vbody`
        key hook; the decoder itself is host-tested in body/test/test_ir.cpp."""
        with self.lock:
            self._event(EV.IR_KEY, (int(command) & 0xFF) | (0x100 if repeat else 0))
        self._say(f"IR key 0x{int(command) & 0xFF:02X}" + (" repeat" if repeat else ""))

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

    def reboot(self, cause: int = P.ResetCause.REQUESTED, boot_delay: float | None = None) -> None:
        """Restart: power-on state, then (after ``boot_delay``, default the
        constructor's) the boot HELLO and EVENT BOOT."""
        delay = self.boot_delay if boot_delay is None else boot_delay
        with self.lock:
            self._power_on(int(cause), self.tick, boot_epoch(self.epoch))
            if delay > 0:
                self.booting_until = self.clock() + delay
                self._boot_rx.clear()
                self._say(f"rebooting ({delay:g} s init)")
            else:
                self._boot()

    def _finish_boot(self) -> None:
        """End of the init: the boot HELLO, then the bytes buffered meanwhile."""
        self.booting_until = None
        self.booted_at = self.clock()
        self._boot()
        data, self._boot_rx = bytes(self._boot_rx), bytearray()
        if data:
            self.process(data)

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
                if self.booting_until is not None:
                    if now < self.booting_until:
                        continue  # still initialising: no tick, no telemetry
                    self._finish_boot()
                self._tick(now)

    def _boot(self) -> None:
        self._send_hello()
        self._event(EV.BOOT, self.reset_cause)
        self._say(f"boot ({consts.RESET_CAUSES.get(self.reset_cause)}) epoch {self.epoch}")

    def _send_hello(self) -> None:
        self._send("HELLO", proto_major=P.PROTO_MAJOR, proto_minor=P.PROTO_MINOR,
                   fw_build=self.fw_build, reset_cause=self.reset_cause,
                   last_alive_tick=self.last_alive_tick,
                   caps=int(P.HelloCaps.BUZZER | P.HelloCaps.IR | P.HelloCaps.DMP | P.HelloCaps.ULTRASONIC),
                   max_payload=P.MAX_PAYLOAD, trick_capacity=P.TRICK_CAPACITY, joint_count=P.JOINT_COUNT,
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
            self.posture = "REST" if self.config_state == P.ConfigState.DEFAULTED else "STAND"
            self._event(EV.RECOVERED, 0)
            self._say("recovered")
        if self.head_until is not None and now >= self.head_until:
            self.head_until = None  # HEAD_HOLD_MS without a refresh
            self._say("head released (no refresh)")
        # watchdog (params in ms / s, like the body)
        if self.wd_armed and self.last_rx_at is not None:
            silent = now - self.last_rx_at
            stop_s = self.params[P.ParamId.WD_STOP_MS] / 1000.0
            rest_s = max(stop_s, self.params[P.ParamId.WD_REST_MS] / 1000.0)
            # LIMP counts from the end of the 1 s REST move (REST_MOVE_TICKS
            # on the body), so 0.5 + 1.0 + 1.0 + 10 = 12.5 s at the defaults.
            limp_s = REST_MOVE_S + self.params[P.ParamId.WD_LIMP_S]
            if self.wd_stage == 0 and silent >= stop_s:
                self.wd_stage = 1
                self._epoch_bump(gait_ramp=True)
                self._event(EV.WATCHDOG, 0)
                self._say("watchdog: stop")
            # Stage 2 also waits for the gait's ramp-down and landing.
            if self.wd_stage == 1 and silent >= rest_s and self.gait is None:
                self.wd_stage = 2
                if self.posture != "LIMP":  # already limp stays limp
                    self.posture = "REST"
                self._say("watchdog: rest")
            if self.wd_stage == 2 and silent >= rest_s + limp_s:
                self.wd_stage = 3
                self.posture = "LIMP"
                self._say("watchdog: limp")
        # the keyframe player steps after the reflexes, as in body_tick
        if self.trick.running and self.trick.player.step():
            self.trick.player.running = True  # _trick_end ends a running trick
            self._trick_end(TDR.DONE)
        g = self.gait
        if g is not None and g.step_tick():
            if g.restart is not None:  # a pattern change: start the new one at phase 0
                self.gait = Gait(g.restart)
                self._say(f"gait {P.GaitPattern(self.gait.pattern).name.lower()}")
            else:
                self.posture = "STANCE"  # the hand-over to IDLE STANCE
                self._gait_end()
        if self.joints_div and self.tick % self.joints_div == 0:
            self._send_joints()
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
        g = self.gait
        if g is not None or self.posture == "STANCE":
            pitch += self.pose[0] / 16.0  # the body pose tilts the body (IK, §3d pose terms)
            roll += self.pose[1] / 16.0
        if g is not None and g.state != "stance":
            pitch += 0.8 * g.amp * math.sin(4 * math.pi * g.phase / 65536)  # the twice-per-cycle body bob
        if self.fall_until is not None:
            roll = 85.0 * self.fall_side
        if self.posture != "LIMP" or g is not None:
            flags |= F.SERVOS_POWERED
        if self.wd_armed and self.wd_stage == 0:
            flags |= F.LINK_OK
        if self.batt_level >= P.BatteryLevel.WARN:
            flags |= F.BATT_WARN
        if self.save_busy(now):
            flags |= F.EEPROM_PENDING
        owner = int(P.Owner.REFLEX if self.reflex else P.Owner.IDLE)
        leg = head = owner
        if g is not None and not self.reflex:
            flags |= F.GAIT
            leg = int(P.Owner.GAIT)
            if g.flags & (P.GaitFlags.BOB | P.GaitFlags.WAG):
                head = int(P.Owner.GAIT)  # bob and wag drive the head tilt and the tail
        if self.trick.running:
            flags |= F.TRICK
            leg = int(P.Owner.TRICK) if self.trick.claims(8) else leg
            head = int(P.Owner.TRICK) if self.trick.claims(0) else head
        if self.head_until is not None:
            flags |= F.HEAD_HELD
            if head in (P.Owner.IDLE, P.Owner.GAIT) and self.head[0] != -32768:
                head = int(P.Owner.PI_HOLD)
        self._send("TELEMETRY", tick=self.tick, pitch=int(pitch * 16), roll=int(roll * 16),
                   rate_pitch=int(1.4 * math.cos(t * 0.7) * 16), rate_roll=int(1.6 * math.cos(t * 1.1) * 16),
                   rate_yaw=0, dist_mm=600 + int(100 * math.sin(t * 0.3)), batt_mv=self.batt_mv,
                   mode=int(self.mode), owners=leg | head << 4, flags=int(flags),
                   last_rx_seq=self.last_rx_seq,
                   trick_frame=self.trick.player.frame if self.trick.running else 0xFF,
                   gait_phase=0 if g is None else g.phase >> 8, tick_load=31,
                   rx_err=(self.rx_crc_err + self.rx_overflow) & 0xFF, epoch=self.epoch)

    def joint_angles(self) -> list[float | None]:
        """The fake output angles in degrees (canonical order; None = limp)."""
        g = self.gait
        if self.posture == "LIMP" and g is None and not self.trick.running:
            return [None] * P.JOINT_COUNT
        a = [float(v) for v in FAKE_ANGLES.get(self.posture, FAKE_STAND)]
        if g is not None and g.state != "stance":
            p = g.phase / 65536.0
            stride = g.amp * g.msg.stride_mm
            for leg, off in enumerate(LEG_OFFSETS.get(P.GaitPattern(g.pattern), LEG_OFFSETS[P.GaitPattern.TROT])):
                s = math.sin(2 * math.pi * (p + off))
                turn = g.amp * g.msg.turn_mm * (-1 if leg in (0, 3) else 1)  # S_left = S - turn
                a[3 + leg] += 0.5 * (stride + turn) * s
                a[7 + leg] += g.amp * g.msg.step_h_mm * max(0.0, -s)
            if g.flags & P.GaitFlags.BOB:
                a[1] += BOB_DEG * g.amp * math.sin(4 * math.pi * p)
            if g.flags & P.GaitFlags.WAG:
                a[2] += WAG_DEG * g.amp * math.sin(2 * math.pi * p)
        if self.head_until is not None:
            for i, v in enumerate(self.head):
                if v != -32768:
                    a[i] = v / 16.0
        return a

    def _send_joints(self) -> None:
        self._send("JOINTS", tick=self.tick,
                   angle=[-32768 if v is None else int(round(v * 16)) for v in self.joint_angles()])

    # ---- rx ----------------------------------------------------------------------
    def process(self, data: bytes, now: float | None = None) -> None:
        """Decode ``data`` and handle every complete frame now (deterministic)."""
        with self.lock:
            if self.booting_until is not None:
                self._boot_rx += data  # the UART buffers it until the init is done
                return
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
        if type not in self.goal_types:
            return self._refuse(type, R.UNKNOWN_TYPE, now)
        if type == P.MsgId.HEARTBEAT:
            return
        if type == P.MsgId.BEEP:
            self.goals[name] = payload
            return
        if type == P.MsgId.SET_HEAD:
            return self._head_goal(msg, now)
        if type == P.MsgId.SET_GAIT:
            return self._gait_goal(msg, payload, now)
        if type == P.MsgId.SET_BODY_POSE:
            return self._pose_goal(msg, payload, now)
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
        if msg.posture >= len(P.Posture) or (msg.posture == P.Posture.STANCE and not self.p2):
            return self._refuse(type, R.BAD_ARG, now)
        safe = msg.posture in (P.Posture.LIMP, P.Posture.REST)
        if not safe and msg.epoch != self.epoch:
            return self._refuse(type, R.STALE, now)
        if self.fallen:
            # The fall reflex holds the joints; only LIMP (the emergency stop) ends it.
            if msg.posture != P.Posture.LIMP:
                return self._refuse(type, R.FALLEN, now)
            self.fall_until = self.recover_until = None
        if not safe:
            if self.batt_level >= P.BatteryLevel.LOW:
                return self._refuse(type, R.LOW_BATT, now)
            if msg.posture == P.Posture.STAND and self.config_state == P.ConfigState.DEFAULTED:
                return self._refuse(type, R.UNCALIBRATED, now)
        self.goals[name] = payload
        self.balance_req = bool(msg.flags & P.PostureFlags.BALANCE)
        if msg.posture == P.Posture.LIMP:
            self._trick_end(TDR.STOPPED)  # LIMP is the emergency stop
            self._gait_end("stopped (LIMP)")
        new = POSTURE_NAMES[msg.posture]
        if new != self.posture:
            self._say(f"posture {new}")
        self.posture = new

    def _head_goal(self, msg: Any, now: float) -> None:
        """SET_HEAD in the body's order (core/body.cpp body_goal_head)."""
        R = P.GoalRefusedReason
        if msg.mode > P.HeadMode.HOLD:
            return self._refuse(P.MsgId.SET_HEAD, R.BAD_ARG, now)
        if msg.mode == P.HeadMode.RELEASE:  # never stale
            if self.head_until is not None:
                self._say("head released")
            self.head_until = None
            return None
        if msg.epoch != self.epoch:
            return self._refuse(P.MsgId.SET_HEAD, R.STALE, now)
        if self.fallen:
            return self._refuse(P.MsgId.SET_HEAD, R.FALLEN, now)
        if self.batt_level >= P.BatteryLevel.LOW:
            return self._refuse(P.MsgId.SET_HEAD, R.LOW_BATT, now)
        if self.head_until is None or self.head != (msg.pan, msg.tilt, msg.tail):
            self._say(f"head hold pan {msg.pan / 16:g} tilt {msg.tilt / 16:g}"
                      + ("" if msg.tail == -32768 else f" tail {msg.tail / 16:g}"))
        self.head = (msg.pan, msg.tilt, msg.tail)
        self.head_until = now + HEAD_HOLD_S
        return None

    def _gait_goal(self, msg: Any, payload: bytes, now: float) -> None:
        """SET_GAIT (§3d): checks, then start / update / stop the gait."""
        R = P.GoalRefusedReason
        t = P.MsgId.SET_GAIT
        if (msg.pattern >= len(P.GaitPattern) or msg.flags & ~GAIT_FLAGS
                or any(not lo <= getattr(msg, k) <= hi for k, (lo, hi) in GAIT_RANGES.items())):
            return self._refuse(t, R.BAD_ARG, now)
        g = self.gait
        if not msg.run:  # a stop is never stale
            self.goals["SET_GAIT"] = payload
            if g is None:
                return None
            if g.state == "stance":
                return self._gait_end("cancelled before the first step")
            g.restart = None
            if g.state != "stop":
                g.start_stop()
                self._say("gait stopping")
            return None
        if msg.epoch != self.epoch:
            return self._refuse(t, R.STALE, now)
        if self.fallen:
            return self._refuse(t, R.FALLEN, now)
        if self.batt_level >= P.BatteryLevel.LOW:
            return self._refuse(t, R.LOW_BATT, now)
        if self.config_state == P.ConfigState.DEFAULTED:
            return self._refuse(t, R.UNCALIBRATED, now)
        self.goals["SET_GAIT"] = payload
        name = P.GaitPattern(msg.pattern).name.lower()
        if g is None:
            ticks = 0
            if self.posture != "STANCE":
                ticks = self.params[P.ParamId.POSTURE_TICKS]
                self.posture = "STANCE"
            self.gait = Gait(msg, ticks)
            self._say(f"gait {name} stride {msg.stride_mm} turn {msg.turn_mm}"
                      + (" (to STANCE first)" if ticks else ""))
        elif msg.pattern != g.pattern and g.state != "stance":
            g.restart = msg  # a pattern change: a stop followed by a start
            g.start_stop()
            self._say(f"gait -> {name}")
        else:
            g.restart = None
            if g.state == "stop":
                g.state = "run"
                self._say("gait resumed")
            g.update(msg)
        return None

    def _pose_goal(self, msg: Any, payload: bytes, now: float) -> None:
        R = P.GoalRefusedReason
        t = P.MsgId.SET_BODY_POSE
        if msg.epoch != self.epoch:
            return self._refuse(t, R.STALE, now)
        if self.fallen:
            return self._refuse(t, R.FALLEN, now)
        if self.batt_level >= P.BatteryLevel.LOW:
            return self._refuse(t, R.LOW_BATT, now)
        self.goals["SET_BODY_POSE"] = payload
        pose = (msg.pitch, msg.roll, msg.dheight_mm, msg.xshift_mm)
        if pose != self.pose:
            self._say(f"body pose pitch {msg.pitch / 16:g} roll {msg.roll / 16:g} dh {msg.dheight_mm} "
                      f"dx {msg.xshift_mm}")
        self.pose = pose
        return None

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
        if t in (M.TRICK_BEGIN, M.TRICK_FRAMES, M.TRICK_COMMIT, M.TRICK_PLAY, M.TRICK_STOP):
            return self._trick_command(t, msg)
        return S.UNKNOWN_TYPE

    def _trick_command(self, t: int, msg: Any) -> int:
        """TRICK_* with the body's checks, in the body's order (core/keyframe.cpp, body.cpp)."""
        M, S = P.MsgId, P.AckStatus
        tr = self.trick
        if t == M.TRICK_BEGIN:
            if tr.running:
                return S.BUSY
            if (not 1 <= msg.n_frames <= P.TRICK_CAPACITY or msg.loop_end >= msg.n_frames
                    or msg.loop_start > msg.loop_end or not msg.joint_mask or msg.joint_mask & ~TRICK_DRIVEN_MASK
                    or msg.flags & ~TRICK_FLAGS or msg.end_posture not in TRICK_END_POSTURES):
                return S.BAD_ARG
            tr.hdr, tr.frames, tr.crc, tr.state = msg, [], 0xFFFF, "loading"
            return S.OK
        if t == M.TRICK_FRAMES:
            if tr.running:
                return S.BUSY
            if tr.state != "loading":
                return S.NO_TRICK
            if msg.first != len(tr.frames) or msg.first + len(msg.frames) > tr.hdr.n_frames:
                return S.BAD_ARG
            if any(f.dur == 0 or (f.flags & 3) == 3 for f in msg.frames):
                return S.BAD_ARG
            for f in msg.frames:
                tr.crc = LC.crc16(f.pack(), tr.crc)
                tr.frames.append(f)
            return S.OK
        if t == M.TRICK_COMMIT:
            if tr.state == "empty" or msg.trick_id != tr.hdr.trick_id:
                return S.NO_TRICK
            if len(tr.frames) != tr.hdr.n_frames:
                return S.BAD_ARG
            if msg.crc16 != tr.crc:
                return S.CRC_MISMATCH
            tr.state = "ready"
            return S.OK
        if t == M.TRICK_STOP:
            self._trick_end(TDR.STOPPED)
            return S.OK
        # TRICK_PLAY
        if msg.epoch != self.epoch:
            return S.STALE
        if tr.state != "ready" or msg.trick_id != tr.hdr.trick_id:
            return S.NO_TRICK
        if msg.speed_q6 < 16 or msg.flags & ~int(P.TrickPlayFlags.MIRROR):
            return S.BAD_ARG
        if tr.running:
            return S.BUSY
        if self.fallen:
            return S.REFUSED_FALLEN
        if self.batt_level >= P.BatteryLevel.LOW:
            return S.REFUSED_LOW_BATT
        if self.reflex or self.posture in ("LIMP", "CALIB"):
            return S.BUSY
        if self.config_state == P.ConfigState.DEFAULTED:
            return S.UNCALIBRATED
        h = tr.hdr
        tr.mirror = bool(msg.flags & P.TrickPlayFlags.MIRROR)
        tr.player = Player([f.dur for f in tr.frames], [f.hold for f in tr.frames], h.loop_start, h.loop_end,
                           h.repeats, msg.speed_q6)
        self._say(f"trick {h.trick_id} play x{msg.speed_q6 / 64:g}" + (" mirrored" if tr.mirror else ""))
        return S.OK

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
