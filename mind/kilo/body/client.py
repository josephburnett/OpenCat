"""BodyClient: the mind's API to the body (docs/architecture.md §5, §7).

Thin and synchronous: goals go through ``SerialLink.set_goal`` (latched,
refreshed at 5 Hz, epoch-stamped, cleared on an epoch change); commands go
through ``SerialLink.request`` (seq, 50 ms wait, 3 retries; queries answered
by their data reply, other commands by an ACK, de-dup safe). Message layouts
and field names are the generated ones (protocol/spec.toml, Appendix E).

Tricks (P1, §3e): ``upload_trick`` (cached by (trick_id, crc)),
``await play(name_or_trick, mirror, speed)`` which resolves on EVENT
TRICK_DONE, and ``stop_trick``.

Gait and body pose (P2, §3d): ``walk(...)`` (SET_GAIT run=1 with the
per-pattern defaults), ``stop_walk()`` (run=0, cleared by EVENT
GAIT_STOPPED), ``wait_gait_stopped`` / ``await walk_until_stopped``,
``gait_state``, ``load_geometry`` (LEG_L1_MM/LEG_L2_MM for the default body
height), ``pose(...)`` / ``release_pose()`` (SET_BODY_POSE).
"""

from __future__ import annotations

import asyncio
import binascii
import threading
import time
from dataclasses import dataclass
from typing import Any

from kilo.link import consts
from kilo.link.serial_link import LinkError, LinkTimeout, SerialLink
from kilo.protocol import kilo_protocol as P


class ProtocolMismatch(LinkError):
    pass


# Postures the CLI offers. "balance" is STAND with the balance flag (SET_POSTURE
# flags bit 0); it is not a separate posture code.
POSTURE_CHOICES = ("limp", "rest", "stand", "sit", "balance", "stance", "calib", "zero")

CAL_LIMIT_A16 = 400  # CAL_SET offset_a16 range: ±25°

# SET_BALANCE defaults and limits (spec [balance], docs §3f; the body boots
# with the same raw values). Kp and Kd are derived here, in the mind's units.
BALANCE_KP = P.BALANCE_DEFAULTS["kp_q8"] / 256         # Kp 0.5, sent as kp_q8 (Q8, 0..255/256)
BALANCE_KD_S = P.BALANCE_DEFAULTS["kd_ms"] / 1000      # Kd 0.03 s, sent as kd_ms (0..255 ms)
BALANCE_DEADBAND = P.BALANCE_DEFAULTS["deadband_deg"]  # degrees
BALANCE_CLAMP = P.BALANCE_DEFAULTS["max_corr_deg"]     # degrees
BALANCE_MAX_DEG = P.BALANCE_LIMIT_DEG                  # dead band and clamp limit

# SET_HEAD (docs §3c layer 1; spec [head]): an i16 a16 of -32768 leaves that
# joint to its owner; targets beyond the joint limits are clamped by the body.
HEAD_NOT_HELD = P.HEAD_NOT_HELD
HEAD_MAX_DEG = P.HEAD_MAX_DEG
HEAD_SPEED_DPS = P.HEAD_SPEED_DPS  # default look() slew, °/s (0 = only the body's 500°/s safety slew)

# SET_GAIT defaults per pattern (spec [gait_patterns.*], docs/architecture.md
# §3d; estimates to be tuned on Kilo): period ms, duty /256, stride mm, step
# height mm, and the body height as a fraction of L1 + L2 (params LEG_L1_MM +
# LEG_L2_MM): the spec's body_h_256 / 256, the body's own default.
GAIT_DEFAULTS: dict[str, dict[str, float]] = {
    d.name.lower(): {"period_ms": d.period_ms, "duty": d.duty, "stride_mm": d.stride_mm,
                     "step_h_mm": d.step_h_mm, "body_h_frac": d.body_h_256 / 256}
    for d in P.GAIT_PATTERN_DEFAULTS.values()
}
GAIT_PATTERN_CHOICES = tuple(consts.GAIT_PATTERNS)  # walk, trot, crawl (spec enum gait_pattern)
DEFAULT_PATTERN = "trot"
DEFAULT_TURN_MM = 20     # `kilo walk left|right` and the remote's turn keys (turn in place)
GAIT_STOP_MARGIN_S = 1.5  # wait_gait_stopped(): one period of ramp-down, the hand-over, and this
WALK_DIRECTIONS = ("forward", "back", "backward", "left", "right", "march")


# Speed (docs/walking-and-balance.md §2, §3): the swing time is fixed and speed
# comes from the stride and the stance time. On a diagonal pair the body tips
# with a 90 ms time constant, so the trot's time on two feet must stay short
# whatever the speed; a longer period with the same duty would lengthen it.
#   swing_ms   per pattern, the spec default's (1 - duty) x period: trot 150 ms,
#              walk 500, crawl 297. Walk and crawl keep three feet down, and
#              their higher steps (15, 18 mm) need the longer swing to stay
#              under the 10 deg/tick safety slew; the trot's 25 mm stride in
#              150 ms is 9.6 deg/tick, so its swing must not get shorter.
#   stance_ms  = stride_mm / speed, period = swing + stance, duty = stance / period.
# Slower than the longest stance allows (duty 230/256, period 2000 ms), the
# stride shrinks (down to GAIT_STRIDE_MIN_MM) instead; faster than the default
# stride at the shortest stance allows, the speed is capped. The shortest
# stance is set by GAIT_DUTY_MIN: the trot keeps four-foot phases (duty >= 0.6),
# walk and crawl three feet down (duty >= 0.75). The spec defaults are each
# pattern's default speed (trot 25 mm / 450 ms = 56 mm/s, walk 20, crawl 15).
_GAIT_RANGES = consts.field_ranges("SET_GAIT")
GAIT_DUTY_MIN = {"walk": 192, "trot": 154, "crawl": 192}  # /256: 0.75, 0.6, 0.75
GAIT_STRIDE_MIN_MM = 8


def gait_swing_ms(pattern: str | int) -> float:
    """The pattern's fixed swing time: its spec default (1 - duty) x period."""
    d = GAIT_DEFAULTS[BodyClient.pattern_name(pattern)]
    return d["period_ms"] * (256 - d["duty"]) / 256


def gait_default_speed(pattern: str | int) -> float:
    """The speed (mm/s) that reproduces the pattern's spec defaults."""
    d = GAIT_DEFAULTS[BodyClient.pattern_name(pattern)]
    return gait_speed(d["period_ms"], d["duty"], d["stride_mm"])


def gait_speed(period_ms: float, duty: float, stride_mm: float) -> float:
    """Ground speed (mm/s) of a gait: |stride| over the stance time."""
    return abs(stride_mm) / (period_ms * duty / 256) * 1000


def _stance_limits(name: str, swing_ms: float) -> tuple[float, float]:
    """(shortest, longest) stance in ms for a swing: the duty and period ranges."""
    p_lo, p_hi = _GAIT_RANGES["period_ms"]
    d_lo, d_hi = max(GAIT_DUTY_MIN[name], _GAIT_RANGES["duty"][0]), _GAIT_RANGES["duty"][1]
    if not 0 < swing_ms < p_hi:
        raise ValueError(f"swing_ms must be between 0 and {p_hi} (got {swing_ms})")
    lo = max(swing_ms * d_lo / (256 - d_lo), p_lo - swing_ms)
    hi = min(swing_ms * d_hi / (256 - d_hi), p_hi - swing_ms)
    if lo > hi:
        raise ValueError(f"no stance time fits a {swing_ms:g} ms swing for {name}")
    return lo, hi


def gait_speed_range(pattern: str | int, swing_ms: float | None = None) -> tuple[float, float]:
    """(slowest, fastest) speed in mm/s that gait_timing() gives."""
    name = BodyClient.pattern_name(pattern)
    lo, hi = _stance_limits(name, gait_swing_ms(name) if swing_ms is None else swing_ms)
    return GAIT_STRIDE_MIN_MM / hi * 1000, GAIT_DEFAULTS[name]["stride_mm"] / lo * 1000


def gait_timing(pattern: str | int, speed_mm_s: float | None = None,
                swing_ms: float | None = None) -> dict[str, int]:
    """SET_GAIT period_ms, duty and stride_mm (a magnitude) for walking at
    ``speed_mm_s`` (default: the pattern's default speed) with a fixed swing
    time (default: the pattern's, gait_swing_ms()). Outside
    gait_speed_range() the speed is clamped; every output is within the
    SET_GAIT ranges."""
    name = BodyClient.pattern_name(pattern)
    swing = gait_swing_ms(name) if swing_ms is None else float(swing_ms)
    stance_lo, stance_hi = _stance_limits(name, swing)
    stride_def = GAIT_DEFAULTS[name]["stride_mm"]
    v = gait_default_speed(name) if speed_mm_s is None else abs(float(speed_mm_s))
    stance = stride_def / v * 1000 if v > 0 else float("inf")
    stride = float(stride_def)
    if stance > stance_hi:  # slow: the longest stance, and a shorter stride
        stance = stance_hi
        stride = max(GAIT_STRIDE_MIN_MM, v * stance_hi / 1000)
    stance = max(stance, stance_lo)  # fast: the shortest stance caps the speed
    period = swing + stance
    d_lo, d_hi = max(GAIT_DUTY_MIN[name], _GAIT_RANGES["duty"][0]), _GAIT_RANGES["duty"][1]
    p_lo, p_hi = _GAIT_RANGES["period_ms"]
    return {"period_ms": min(max(int(round(period)), p_lo), p_hi),
            "duty": min(max(int(round(256 * stance / period)), d_lo), d_hi),
            "stride_mm": int(round(stride))}


def walk_args(direction: str, pattern: str = DEFAULT_PATTERN, stride_mm: float | None = None,
              turn_mm: float | None = None, speed_mm_s: float | None = None) -> dict[str, Any]:
    """walk() arguments for a direction (`kilo walk`, the IR remote).

    forward / back(ward): the stride (default: the pattern's) forward or
    backward, ``turn_mm`` as given (signed, + = left: an arc), default 0.
    left / right: turn in place by ``turn_mm`` (magnitude, default
    DEFAULT_TURN_MM) to that side; a ``stride_mm`` makes it an arc.
    march: stride 0, turn 0 (stepping in place).

    With ``speed_mm_s`` the result also has ``period_ms`` and ``duty`` from
    gait_timing(), and forward / back take its stride unless ``stride_mm``
    is given (turning and marching keep their strides, at that cadence)."""
    d = direction.lower()
    if d not in WALK_DIRECTIONS:
        raise ValueError(f"unknown direction {direction!r} (known: {', '.join(WALK_DIRECTIONS)})")
    pattern = BodyClient.pattern_name(pattern)
    timing = None if speed_mm_s is None else gait_timing(pattern, speed_mm_s)
    extra = {} if timing is None else {"period_ms": timing["period_ms"], "duty": timing["duty"]}
    if d in ("forward", "back", "backward"):
        if stride_mm is None:
            stride_mm = GAIT_DEFAULTS[pattern]["stride_mm"] if timing is None else timing["stride_mm"]
        s = abs(stride_mm)
        return {"pattern": pattern, "stride_mm": s if d == "forward" else -s, "turn_mm": turn_mm or 0, **extra}
    if d in ("left", "right"):
        t = abs(DEFAULT_TURN_MM if turn_mm is None else turn_mm)
        return {"pattern": pattern, "stride_mm": stride_mm or 0, "turn_mm": t if d == "left" else -t, **extra}
    return {"pattern": pattern, "stride_mm": 0, "turn_mm": 0, **extra}


# Tricks: TRICK_PLAY speed_q6 range (0.25x .. 3.98x) and the play() timeout margin.
SPEED_MIN, SPEED_MAX = 16 / 64, 255 / 64
TRICK_DONE_MARGIN_S = 1.0


class TrickError(LinkError):
    pass


@dataclass
class TrickResult:
    """How a play() ended: EVENT TRICK_DONE's reason (DONE, STOPPED,
    PREEMPTED, FELL), the trick_id, seconds from TRICK_PLAY's ACK to the
    event, and the ticks the player was expected to take (None: until stopped)."""
    reason: str
    trick_id: int
    elapsed_s: float
    expected_ticks: int | None
    epoch: int


def trick_id_for(trick: Any) -> int:
    """A stable 1..254 tag for a trick's header and frames (the body caches one
    trick; TRICK_COMMIT's CRC is the real identity check)."""
    data = trick.begin(0).pack()[1:] + trick.encode_frames()
    return binascii.crc_hqx(data, 0xFFFF) % 254 + 1


class BodyClient:
    # True when the thread that waits in this client (wait_ready, cal_save,
    # play) is the mind's main loop, so waiting may stamp the link's liveness
    # (the CLI). The daemon sets False: its pool threads and the behaviour
    # runner's trick waits must not keep a stalled loop looking alive.
    stamp_liveness = True

    def __init__(self, link: SerialLink):
        self.link = link
        # The trick the body holds, as far as this client knows: (trick_id,
        # crc, reboots seen when it was committed). TRICK_COMMIT alone checks
        # it on the body (another process may have replaced it).
        self._trick_cached: tuple[int, int, int] | None = None
        self._trick_lock = threading.Lock()
        self._trick_done: list[tuple[float, Any]] = []
        self._upload_lock = threading.RLock()  # one upload sequence on the wire at a time
        # Gait (P2): EVENT GAIT_STOPPED seen, the last walk's wire fields (a
        # stop repeats them with run=0), and the leg lengths for the default
        # body height: the spec defaults until load_geometry() read the body's
        # params (None: not read on this boot of the body).
        self._gait_stopped: list[tuple[float, Any]] = []
        self._gait_fields: dict[str, int] | None = None
        self.leg_mm: tuple[int, int] = (consts.PARAMS_BY_NAME["LEG_L1_MM"].default,
                                        consts.PARAMS_BY_NAME["LEG_L2_MM"].default)
        self._geometry_reboots: int | None = None
        link.subscribe("event", self._on_event)

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
        a = self.stamp_liveness
        deadline = time.monotonic() + timeout
        if not link.wait_until(lambda: link.state.connected, timeout, alive=a):
            raise LinkTimeout(f"port {link.state.port} did not open")
        if not link.wait_until(lambda: link.state.hello is not None, max(0.0, min(0.5, deadline - time.monotonic())),
                               alive=a):
            while link.state.hello is None:
                if time.monotonic() >= deadline:
                    raise LinkTimeout("no HELLO from the body")
                try:
                    self.hello()
                except LinkTimeout:
                    pass
                except LinkError:
                    link.wait_until(lambda: link.state.connected, 0.2, alive=a)
        if require_telemetry:
            left = max(0.0, deadline - time.monotonic())
            if not link.wait_until(lambda: link.state.alive, left, alive=a):
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

    def set_balance(self, enable: bool, kp: float = BALANCE_KP, kd_s: float = BALANCE_KD_S,
                    deadband_deg: float = BALANCE_DEADBAND, clamp_deg: float = BALANCE_CLAMP) -> dict[str, int]:
        """Latch SET_BALANCE (goal, no epoch): enable and the gains, live, not persisted.

        enable=True balances SIT and STAND even without the posture flag;
        enable=False leaves only the flag (``posture("balance")``). Returns
        the wire fields. Raises ValueError outside the wire ranges.
        """
        kp_q8 = int(round(kp * 256))
        kd_ms = int(round(kd_s * 1000))
        db, cl = int(round(deadband_deg)), int(round(clamp_deg))
        if not 0 <= kp_q8 <= 255:
            raise ValueError("Kp must be 0..0.996 (Q8)")
        if not 0 <= kd_ms <= 255:
            raise ValueError("Kd must be 0..0.255 s")
        if not 0 <= db <= BALANCE_MAX_DEG or not 0 <= cl <= BALANCE_MAX_DEG:
            raise ValueError(f"dead band and clamp must be 0..{BALANCE_MAX_DEG}°")
        fields = {"enable": 1 if enable else 0, "kp_q8": kp_q8, "kd_ms": kd_ms, "deadband_deg": db,
                  "max_corr_deg": cl}
        self._check_drivable()
        self.link.set_goal("SET_BALANCE", **fields)
        return fields

    def release(self) -> None:
        """Stop asserting a posture (the body keeps what it is doing)."""
        self.link.clear_goal("SET_POSTURE")

    # ---- head (goal; docs §3c layer 1) ---------------------------------------------
    def look(self, pan_deg: float, tilt_deg: float, tail_deg: float | None = None,
             speed_dps: float = HEAD_SPEED_DPS) -> dict[str, int]:
        """Latch SET_HEAD mode=HOLD: point the head (pan, tilt, and the tail
        unless None; degrees in the joint convention of the postures) and keep
        it there while the head's owner is IDLE or GAIT (a trick that moves
        the head, or a reflex, wins).

        Refreshed at 5 Hz like every goal; the body drops the hold after
        HEAD_HOLD_MS (2 s) without a refresh, on SET_HEAD mode=0
        (``release_head``) and on any epoch bump (the link then clears the
        goal too). ``speed_dps`` limits the slew (0 = only the body's safety
        slew, 500°/s). Angles outside the joint limits are clamped by the
        body. Returns the wire fields; ValueError outside the wire ranges.
        """
        def a16(deg: float) -> int:
            v = int(round(deg * 16))
            if not -HEAD_MAX_DEG * 16 <= v <= HEAD_MAX_DEG * 16:
                raise ValueError(f"head angles must be within ±{HEAD_MAX_DEG}°")
            return v

        speed = int(round(speed_dps))
        if not 0 <= speed <= 0xFFFF:
            raise ValueError("speed must be 0..65535 °/s")
        fields = {"mode": int(P.HeadMode.HOLD), "pan": a16(pan_deg), "tilt": a16(tilt_deg),
                  "tail": HEAD_NOT_HELD if tail_deg is None else a16(tail_deg), "max_speed_dps": speed}
        self._check_drivable()
        self.link.set_goal("SET_HEAD", **fields)
        return fields

    def release_head(self) -> None:
        """Stop holding the head: clear the SET_HEAD goal and send SET_HEAD
        mode=RELEASE once (never stale), so the head crossfades back to its
        posture now rather than after HEAD_HOLD_MS."""
        self.link.clear_goal("SET_HEAD")
        self.link.send("SET_HEAD", epoch=self.state.epoch or 0, mode=int(P.HeadMode.RELEASE), pan=0, tilt=0,
                       tail=0, max_speed_dps=0)

    # ---- gait and body pose (goals; docs §3d, P2) -------------------------------------
    @staticmethod
    def _check_ranges(name: str, fields: dict[str, int]) -> None:
        """Client-side BAD_ARG: every field within the spec range (or its wire type's)."""
        rng = consts.field_ranges(name)
        for k, v in fields.items():
            lo, hi = rng[k]
            if not lo <= v <= hi:
                raise ValueError(f"{name} {k} must be {lo}..{hi} (got {v})")

    def load_geometry(self) -> tuple[int, int]:
        """Read LEG_L1_MM and LEG_L2_MM from the body (PARAM_GET), once per
        boot of the body; walk() derives the default body height from them.
        Keeps the spec defaults if the body cannot answer."""
        reboots = self.state.reboots
        if self._geometry_reboots != reboots:
            try:
                self.leg_mm = (int(self.param_get("LEG_L1_MM").value), int(self.param_get("LEG_L2_MM").value))
                self._geometry_reboots = reboots
            except LinkError:
                pass
        return self.leg_mm

    @staticmethod
    def pattern_name(pattern: str | int) -> str:
        if isinstance(pattern, str):
            name = pattern.lower()
            if name not in consts.GAIT_PATTERNS:
                raise ValueError(f"unknown gait pattern {pattern!r} (known: {', '.join(GAIT_PATTERN_CHOICES)})")
            return name
        try:
            return P.GaitPattern(int(pattern)).name.lower()
        except ValueError:
            raise ValueError(f"unknown gait pattern {pattern!r}") from None

    def gait_fields(self, stride_mm: float | None = None, turn_mm: float = 0, pattern: str | int = DEFAULT_PATTERN,
                    period_ms: int | None = None, duty: int | None = None, step_h_mm: float | None = None,
                    body_h_mm: float | None = None, bob: bool = True, wag: bool = True,
                    balance: bool = True, run: bool = True) -> dict[str, int]:
        """SET_GAIT's wire fields (no epoch) with the §3d defaults of ``pattern``
        for every field left None. ValueError outside the spec ranges."""
        name = self.pattern_name(pattern)
        d = GAIT_DEFAULTS[name]
        if body_h_mm is None:
            body_h_mm = d["body_h_frac"] * sum(self.leg_mm)
        F = P.GaitFlags
        fields = {"run": 1 if run else 0, "pattern": consts.GAIT_PATTERNS[name],
                  "period_ms": int(round(d["period_ms"] if period_ms is None else period_ms)),
                  "duty": int(round(d["duty"] if duty is None else duty)),
                  "stride_mm": int(round(d["stride_mm"] if stride_mm is None else stride_mm)),
                  "turn_mm": int(round(turn_mm)),
                  "step_h_mm": int(round(d["step_h_mm"] if step_h_mm is None else step_h_mm)),
                  "body_h_mm": int(round(body_h_mm)),
                  "flags": int((F.BOB if bob else 0) | (F.WAG if wag else 0) | (F.BALANCE if balance else 0))}
        self._check_ranges("SET_GAIT", fields)
        if fields["body_h_mm"] < 1:
            raise ValueError("SET_GAIT body_h_mm must be at least 1 mm")
        return fields

    def walk(self, stride_mm: float | None = None, turn_mm: float = 0, pattern: str | int = DEFAULT_PATTERN,
             period_ms: int | None = None, duty: int | None = None, step_h_mm: float | None = None,
             body_h_mm: float | None = None, bob: bool = True, wag: bool = True,
             balance: bool = True) -> dict[str, int]:
        """Latch SET_GAIT run=1: walk with ``pattern`` (walk, trot, crawl),
        ``stride_mm`` (signed, + = forward) and ``turn_mm`` (+ = left; with
        stride 0 Kilo turns in place). Fields left None take the §3d defaults
        of the pattern; the body height defaults to a fraction of L1 + L2
        (``load_geometry()``). Flags: head bob, tail wag, balance.

        A motion goal like a posture: stamped with the current epoch,
        refreshed at 5 Hz, cleared on any epoch change (watchdog, fall, low
        battery, reboot) and not sent before contact. The body moves to
        STANCE first when it stands in another posture, and ramps the stride
        up over one period. Changing the pattern while walking is a stop
        followed by a start on the body. Like play(), it releases the latched
        posture goal: a posture refreshed under the gait would be what the
        body takes after the stop's hand-over to STANCE. Returns the wire
        fields; ValueError outside the spec ranges (the body would refuse
        BAD_ARG).
        """
        fields = self.gait_fields(stride_mm, turn_mm, pattern, period_ms, duty, step_h_mm, body_h_mm, bob, wag,
                                  balance)
        self._check_drivable()
        self.link.clear_goal("SET_POSTURE")
        self.link.set_goal("SET_GAIT", **fields)
        self._gait_fields = dict(fields)
        return fields

    def stop_walk(self, wait_s: float | None = None) -> int | bool:
        """Latch SET_GAIT run=0 (the last walk's parameters): the body ramps
        the stride down over one period, hands over to STANCE and sends EVENT
        GAIT_STOPPED, which clears this goal. Never held back (a stop is safe
        to send at any time).

        Without ``wait_s`` returns the GAIT_STOPPED count before the stop, for
        ``wait_gait_stopped()``; with it, waits up to ``wait_s`` seconds and
        returns whether the gait stopped."""
        fields = dict(self._gait_fields or self.gait_fields(stride_mm=0))
        fields["run"] = 0
        with self._trick_lock:
            n = len(self._gait_stopped)
        self.link.set_goal("SET_GAIT", **fields)
        if wait_s is None:
            return n
        return self.wait_gait_stopped(n, wait_s)

    def gait_stopped_since(self, n: int) -> Any:
        """The first EVENT GAIT_STOPPED after the n-th one seen, or None."""
        with self._trick_lock:
            return self._gait_stopped[n][1] if len(self._gait_stopped) > n else None

    def walking(self) -> bool:
        """The body reports a gait (mode GAIT or the GAIT flag)."""
        st = self.state
        return st.mode == "GAIT" or "gait" in st.flags

    def stop_timeout(self) -> float:
        """How long a stop may take: one period of ramp-down plus a margin."""
        period = (self._gait_fields or {}).get("period_ms") or max(d["period_ms"] for d in GAIT_DEFAULTS.values())
        return period / 1000.0 + GAIT_STOP_MARGIN_S

    def wait_gait_stopped(self, n: int, timeout: float | None = None) -> bool:
        """Block until GAIT_STOPPED (after the n-th) or until the body has
        shown no gait for 0.3 s (it was not walking). False on timeout."""
        timeout = self.stop_timeout() if timeout is None else timeout
        link = self.link
        t0 = link.clock()
        quiet_since = None
        while True:
            if self.stamp_liveness:
                link.mark_alive()
            if self.gait_stopped_since(n) is not None:
                return True
            now = link.clock()
            if self.walking():
                quiet_since = None
            elif quiet_since is None:
                quiet_since = now
            elif now - quiet_since >= 0.3:
                return True
            if now - t0 >= timeout:
                return False
            time.sleep(0.02)

    async def walk_until_stopped(self, n: int | None = None, timeout: float | None = None) -> bool:
        """Await the end of the gait: EVENT GAIT_STOPPED after the n-th seen
        (default: from now; pass ``stop_walk()``'s return to include one that
        already came), or None-timeout forever. Returns False on timeout."""
        if n is None:
            with self._trick_lock:
                n = len(self._gait_stopped)
        t0 = self.link.clock()
        while self.gait_stopped_since(n) is None:
            if self.stamp_liveness:
                self.link.mark_alive()
            if timeout is not None and self.link.clock() - t0 > timeout:
                return False
            await asyncio.sleep(0.02)
        return True

    def gait_state(self) -> dict[str, Any] | None:
        """The latched SET_GAIT (pattern name, stride, turn, run...) with the
        body's mode and gait phase (0..1), or None when neither the mind nor
        the body has a gait."""
        g = self.link.goals().get("SET_GAIT")
        tel = self.state.telemetry
        if g is None and not self.walking():
            return None
        out: dict[str, Any] = {"mode": self.state.mode, "walking": self.walking()}
        if g is not None:
            f = g.fields
            out.update({"run": f["run"], "pattern": self.pattern_name(f["pattern"]), "stride_mm": f["stride_mm"],
                        "turn_mm": f["turn_mm"], "period_ms": f["period_ms"], "epoch": g.epoch})
        out["phase"] = None if tel is None else round(tel.gait_phase / 256.0, 3)
        return out

    def pose(self, pitch_deg: float = 0, roll_deg: float = 0, dheight_mm: float = 0,
             xshift_mm: float = 0) -> dict[str, int]:
        """Latch SET_BODY_POSE: body pitch (+ = nose down) and roll (+ = right
        side up) in degrees, height delta and forward shift in mm, applied on
        top of the gait and of the STANCE posture (other postures ignore it).
        An epoch goal like walk(). Returns the wire fields; ValueError outside
        the wire ranges."""
        fields = {"pitch": int(round(pitch_deg * 16)), "roll": int(round(roll_deg * 16)),
                  "dheight_mm": int(round(dheight_mm)), "xshift_mm": int(round(xshift_mm))}
        self._check_ranges("SET_BODY_POSE", fields)
        self._check_drivable()
        self.link.set_goal("SET_BODY_POSE", **fields)
        return fields

    def release_pose(self) -> None:
        """Back to the neutral pose: SET_BODY_POSE zeros, latched (an epoch
        change clears it like any epoch goal; the body forgets its pose then too)."""
        self.link.set_goal("SET_BODY_POSE", pitch=0, roll=0, dheight_mm=0, xshift_mm=0)

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
        return self.link.wait_until(lambda: self._count_events("EEPROM_SAVED") > n_before, wait_saved,
                                    alive=self.stamp_liveness)

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

    # ---- tricks (commands; docs §3e) -----------------------------------------
    def _on_event(self, msg: Any) -> None:
        if msg.code == consts.EV["TRICK_DONE"]:
            with self._trick_lock:
                self._trick_done.append((self.link.clock(), msg))
        elif msg.code == consts.EV["GAIT_STOPPED"]:
            with self._trick_lock:
                self._gait_stopped.append((self.link.clock(), msg))  # indexed by count: never trimmed
            # The stop goal (run=0) has done its job: stop refreshing it. A
            # walk latched meanwhile (run=1) stays.
            self.link.clear_goal("SET_GAIT", only_if=lambda g: not g.fields.get("run"))

    def upload_trick(self, trick: Any, force: bool = False) -> tuple[int, bool]:
        """Make the body hold ``trick``. Returns (trick_id, uploaded).

        Cached: when the body is known to hold it (same id and CRC, no reboot
        since) nothing is sent; otherwise TRICK_COMMIT alone asks the body
        (OK = it has exactly this trick), and only then TRICK_BEGIN,
        TRICK_FRAMES x ceil(n / 4) and TRICK_COMMIT go out. Raises
        TrickError/CommandError if the body refuses.
        """
        self._check_drivable()
        trick.validate()
        with self._upload_lock:
            return self._upload_locked(trick, force)

    def _upload_locked(self, trick: Any, force: bool) -> tuple[int, bool]:
        tid, crc = trick_id_for(trick), trick.crc16()
        reboots = self.state.reboots
        if not force and self._trick_cached == (tid, crc, reboots):
            return tid, False
        if not force:
            try:
                self.link.request("TRICK_COMMIT", {"trick_id": tid, "crc16": crc})
                self._trick_cached = (tid, crc, reboots)
                return tid, False
            except LinkError as e:
                if getattr(e, "status", None) is None:
                    raise
        self._trick_cached = None
        for m in trick.upload_messages(tid):
            self.link.request(m.NAME, P.to_dict(m))
        self._trick_cached = (tid, crc, self.state.reboots)
        return tid, True

    @staticmethod
    def speed_q6(speed: float) -> int:
        q = int(round(speed * 64))
        if not 16 <= q <= 255:
            raise ValueError(f"speed must be {SPEED_MIN:g}..{SPEED_MAX:.2f}")
        return q

    @staticmethod
    def expected_ticks(trick: Any, speed: float = 1.0) -> int | None:
        """Ticks from TRICK_PLAY to the last frame's end at ``speed`` (None: until stopped)."""
        from kilo.tricks.player import total_ticks

        return total_ticks([f.dur for f in trick.frames], [f.hold for f in trick.frames], trick.loop_start,
                           trick.loop_end, trick.repeats, BodyClient.speed_q6(speed))

    def start_trick(self, trick: Any, mirror: bool = False, speed: float = 1.0) -> tuple[int, int]:
        """Upload (cached) and send TRICK_PLAY; returns (trick_id, done_count
        before the play) for wait_trick(). The latched posture goal is
        released: the trick's end posture replaces it on the body."""
        if isinstance(trick, str):
            from kilo.tricks import library

            trick = library.load(trick)
        q = self.speed_q6(speed)
        tid, _ = self.upload_trick(trick)
        self.link.clear_goal("SET_POSTURE")
        epoch = self.state.epoch
        if epoch is None:
            raise TrickError("no epoch from the body yet (wait_ready first)")
        with self._trick_lock:
            n = len(self._trick_done)
        fields = {"epoch": epoch, "trick_id": tid, "speed_q6": q,
                  "flags": int(P.TrickPlayFlags.MIRROR) if mirror else 0}
        try:
            self.link.request("TRICK_PLAY", fields)
        except LinkError as e:
            if getattr(e, "status", None) != int(P.AckStatus.NO_TRICK):
                raise
            # Another process replaced the body's trick since we cached it.
            self.upload_trick(trick, force=True)
            self.link.request("TRICK_PLAY", fields)
        return tid, n

    def trick_done_since(self, n: int, trick_id: int) -> Any:
        """The first TRICK_DONE for ``trick_id`` after the n-th one seen, or None."""
        with self._trick_lock:
            for _, m in self._trick_done[n:]:
                if m.arg & 0xFF == trick_id:
                    return m
        return None

    async def play(self, trick: Any, mirror: bool = False, speed: float = 1.0,
                   timeout: float | None = None) -> TrickResult:
        """Play a trick (a library name or a Trick) and resolve on its TRICK_DONE.

        Timeout: the expected duration at ``speed`` plus TRICK_DONE_MARGIN_S
        (none for a trick that loops until stopped, unless ``timeout`` is
        given); on timeout the trick is stopped and TrickError raised. If the
        coroutine is cancelled (a behaviour preempted, Ctrl-C) before
        TRICK_DONE, the trick is stopped on the body too. It stamps the link's
        liveness while it waits only when ``stamp_liveness`` (the CLI, whose
        loop this is); under the daemon the runner's tick or the daemon's pump
        is the liveness source.
        """
        if isinstance(trick, str):
            from kilo.tricks import library

            trick = library.load(trick)
        ticks = self.expected_ticks(trick, speed)
        if timeout is None and ticks is not None:
            timeout = (ticks + 1) * P.TICK_MS / 1000.0 + TRICK_DONE_MARGIN_S
        tid, n = self.start_trick(trick, mirror, speed)
        t0 = self.link.clock()
        try:
            while True:
                if self.stamp_liveness:
                    self.link.mark_alive()
                m = self.trick_done_since(n, tid)
                if m is not None:
                    reason = P.TrickDoneReason((m.arg & 0xFFFF) >> 8).name
                    return TrickResult(reason, tid, self.link.clock() - t0, ticks, m.epoch)
                if timeout is not None and self.link.clock() - t0 > timeout:
                    self._stop_quietly()
                    raise TrickError(f"no TRICK_DONE within {timeout:.2f} s")
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            if self.trick_done_since(n, tid) is None:
                self._stop_quietly()  # preempted: do not leave the trick playing
            raise

    def _stop_quietly(self) -> None:
        try:
            self.stop_trick()
        except LinkError:
            pass

    def stop_trick(self, transition_ticks: int = 0) -> None:
        """TRICK_STOP: crossfade to the end posture (0 = XFADE_TICKS)."""
        self.link.request("TRICK_STOP", {"transition_ticks": transition_ticks})

    def _count_events(self, name: str) -> int:
        code = consts.EV[name]
        with self.state.lock:
            return sum(1 for _, e in self.state.events if e.code == code)


def _field(msg: Any, name: str) -> Any:
    if msg is None:
        raise LinkError("no reply")
    return getattr(msg, name)
