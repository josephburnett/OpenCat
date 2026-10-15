"""The body model: Kilo's geometry and masses, in one place.

Everything the planner (kilo.planner.plan) and the virtual body's physics
(kilo.vbody.physics) know about Kilo's body is a field of ``BodyModel``.
The lengths are measured (2026-10-11); the masses and where they sit, and
the servo lag, are ESTIMATES (marked below) until the owner weighs Kilo
per foot on a kitchen scale (with and without the battery) and runs a
servo step test with the IMU recording (docs/walking-and-balance.md §9,
§10). Change them here and nowhere else.

Frames and conventions:

- Body frame: origin midway between the four shoulder axles, x forward,
  y LEFT, z UP, millimetres. Legs in the protocol order LF RF RB LB.
- A foot is given as FEET gives it (protocol/spec.toml): ``x`` fore-aft
  from under its shoulder (+ forward) and ``z`` its depth below the
  shoulder (the leg's length, the body height at that leg), to the centre
  of the foot disc; the ground is ``foot_r_mm`` lower.
- Attitude as TELEMETRY reports it: pitch nose DOWN positive, roll RIGHT
  side up positive (docs/architecture.md §3f). The rotation from body to
  world is R = Ry(pitch) Rx(-roll): a nose-down pitch lowers the front, a
  positive roll lowers the left side and raises the right.
- The leg is planar (two servos, shoulder and knee): theta1 is the upper
  link's angle from straight down (+ = foot forward), theta2 the knee bend
  relative to it; the front knees point back (sigma +1), the back knees
  forward (sigma -1), as in body/src/core/ik.h.

Leg lengths and attitude, exactly: with every support foot on flat ground,
  z_i + r = H - a X_i - b Y_i,   a = tan(pitch) / cos(roll),  b = tan(roll)
where (X_i, Y_i) is foot i in the body frame and r the foot radius. The
planner sets z from a planned attitude with it, and recovers the attitude
the body will take from the z of the feet that are down (a plane through
three or four feet; on two feet the plane is underdetermined and the
previous attitude fills in).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

LF, RF, RB, LB = 0, 1, 2, 3
LEGS = (LF, RF, RB, LB)
LEG_NAMES = ("LF", "RF", "RB", "LB")
FRONT = (True, True, False, False)
LEFT = (True, False, False, True)
G_MM_S2 = 9810.0


@dataclass(frozen=True)
class Mass:
    """A lumped mass (grams) at a point of the body frame (mm)."""
    grams: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class BodyModel:
    # ---- geometry: measured (params LEG_L1_MM, LEG_L2_MM, LEG_LB_MM, LEG_W_MM) ----
    l1_mm: float = 45.0          # upper link, shoulder axle to knee axle
    l2_mm: float = 50.0          # lower link, knee axle to the foot disc's centre
    foot_r_mm: float = 6.0       # the floor is this far below the foot disc's centre
    length_mm: float = 123.0     # front to back shoulder axle
    track_mm: float = 65.0       # left to right foot centre
    # ---- masses: ESTIMATES (Kilo weighs roughly 1 kg with the battery) ----
    # The frame with the NyBoard, the four shoulder servos, the head (two
    # servos) and the tail servo; the head makes it a little nose-heavy.
    # Replace with the per-foot scale readings: front pair vs back pair gives
    # x, left pair vs right pair gives y.
    body: Mass = Mass(700.0, 5.0, 0.0, 5.0)
    # The 2S pack under the belly (est. 150 g).
    battery: Mass = Mass(150.0, 0.0, 0.0, -15.0)
    # The Pi 2 B is not carried yet (0 g); when it is, about 45 g on the back.
    pi: Mass = Mass(0.0, -10.0, 0.0, 25.0)
    upper_leg_g: float = 20.0    # upper link with the knee servo (est.)
    upper_leg_at: float = 0.8    # ...its centre this far from the shoulder (the servo sits at the knee)
    lower_leg_g: float = 8.0     # lower link and foot (est.)
    lower_leg_at: float = 0.5
    # ---- servos: ESTIMATES ----
    servo_lag_ms: float = 150.0  # command to motion under load; never measured (servo step test)
    slew_deg_per_tick: float = 10.0  # the body's safety slew (SLEW_MAX_A16)
    reach_frac: float = 0.95     # the IK keeps d <= 0.95 (L1 + L2) (core/ik.h)
    knee_max_deg: float = 150.0  # ...and the knee bend <= 150 deg

    # ---- derived -------------------------------------------------------------------
    @property
    def total_g(self) -> float:
        return (self.body.grams + self.battery.grams + self.pi.grams
                + 4 * (self.upper_leg_g + self.lower_leg_g))

    def hip(self, leg: int) -> tuple[float, float]:
        """Shoulder axle (x, y) in the body frame."""
        return (self.length_mm / 2 if FRONT[leg] else -self.length_mm / 2,
                self.track_mm / 2 if LEFT[leg] else -self.track_mm / 2)

    @property
    def reach_mm(self) -> float:
        return self.reach_frac * (self.l1_mm + self.l2_mm)

    @property
    def min_reach_mm(self) -> float:
        c = math.cos(math.radians(self.knee_max_deg))
        return math.sqrt(self.l1_mm ** 2 + self.l2_mm ** 2 + 2 * self.l1_mm * self.l2_mm * c)

    def stance_height(self, h_256: int) -> float:
        """A body height given as /256 of L1 + L2 (spec [gait] stance_h_256)."""
        return h_256 / 256.0 * (self.l1_mm + self.l2_mm)

    def drop_accel(self, lever_mm: float, com_height_mm: float) -> float:
        """rad/s^2 a side (or an end) of the body can drop at under gravity
        when its legs shorten: the body pivots on the far feet, ``lever_mm``
        away from the mass, like a pendulum (shortening a leg cannot pull the
        body down faster)."""
        return G_MM_S2 * lever_mm / (com_height_mm ** 2 + lever_mm ** 2)

    def lag_ticks(self, tick_ms: float = 20.0) -> float:
        return self.servo_lag_ms / tick_ms

    def tau_s(self, com_height_mm: float) -> float:
        """The inverted pendulum's time constant: a tilt grows as e^(t / tau)
        once the mass is past the edge of its support."""
        return math.sqrt(max(com_height_mm, 1.0) / G_MM_S2)

    # ---- the leg ---------------------------------------------------------------------
    def reachable(self, x: float, z: float) -> bool:
        d = math.hypot(x, z)
        return self.min_reach_mm - 1e-6 <= d <= self.reach_mm + 1e-6 and z > 0

    def ik(self, leg: int, x: float, z: float) -> tuple[float, float]:
        """Foot (x, z) -> (theta1, theta2) in radians, the distance clamped to
        the workspace like the body's IK."""
        l1, l2 = self.l1_mm, self.l2_mm
        d = math.hypot(x, z)
        d = min(max(d, self.min_reach_mm), self.reach_mm)
        c = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        c = min(1.0, max(-1.0, c))
        s = math.sqrt(1.0 - c * c) * (1.0 if FRONT[leg] else -1.0)
        t2 = math.atan2(s, c)
        t1 = math.atan2(x, z) - math.atan2(l2 * s, l1 + l2 * c)
        return t1, t2

    def fk(self, t1: float, t2: float) -> tuple[float, float]:
        """(theta1, theta2) -> foot (x, z)."""
        return (self.l1_mm * math.sin(t1) + self.l2_mm * math.sin(t1 + t2),
                self.l1_mm * math.cos(t1) + self.l2_mm * math.cos(t1 + t2))

    # ---- mass ------------------------------------------------------------------------
    def com(self, xs, zs) -> tuple[float, float, float]:
        """Centre of mass (x, y, z) in the body frame for feet (xs, zs)."""
        m = self.body.grams + self.battery.grams + self.pi.grams
        sx = self.body.grams * self.body.x + self.battery.grams * self.battery.x + self.pi.grams * self.pi.x
        sy = self.body.grams * self.body.y + self.battery.grams * self.battery.y + self.pi.grams * self.pi.y
        sz = self.body.grams * self.body.z + self.battery.grams * self.battery.z + self.pi.grams * self.pi.z
        mu, ml = self.upper_leg_g, self.lower_leg_g
        for leg in LEGS:
            hx, hy = self.hip(leg)
            t1, t2 = self.ik(leg, xs[leg], zs[leg])
            kx, kz = self.l1_mm * math.sin(t1), self.l1_mm * math.cos(t1)  # knee, z down
            fx, fz = xs[leg], zs[leg]
            ux, uz = kx * self.upper_leg_at, kz * self.upper_leg_at
            lx, lz = kx + (fx - kx) * self.lower_leg_at, kz + (fz - kz) * self.lower_leg_at
            sx += mu * (hx + ux) + ml * (hx + lx)
            sy += (mu + ml) * hy
            sz -= mu * uz + ml * lz
            m += mu + ml
        return sx / m, sy / m, sz / m


# ---- attitude and leg lengths -------------------------------------------------------------
def plane_coeffs(pitch_deg: float, roll_deg: float) -> tuple[float, float]:
    """(a, b) of the support plane for an attitude (module docstring)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return math.tan(p) / math.cos(r), math.tan(r)


def attitude_of(a: float, b: float) -> tuple[float, float]:
    """(pitch, roll) in degrees from the plane's (a, b)."""
    roll = math.atan(b)
    return math.degrees(math.atan(a * math.cos(roll))), math.degrees(roll)


def leg_lengths(model: BodyModel, xs, height: float, pitch_deg: float, roll_deg: float, out=None):
    """z for every leg so that all four feet stand on flat ground with the
    body at ``pitch``/``roll`` and the legs' mean length ``height``."""
    a, b = plane_coeffs(pitch_deg, roll_deg)
    zs = out if out is not None else [0.0] * 4
    tot = 0.0
    for leg in LEGS:
        hx, hy = model.hip(leg)
        zs[leg] = -a * (hx + xs[leg]) - b * hy
        tot += zs[leg]
    off = height - tot / 4
    for leg in LEGS:
        zs[leg] += off
    return zs


def fit_attitude(model: BodyModel, xs, zs, support: int, prev: tuple[float, float] = (0.0, 0.0),
                 reg: float = 1e-3) -> tuple[float, float]:
    """The attitude (pitch, roll) in degrees the body takes standing on the
    feet in ``support`` (bit i = leg i): least squares of the plane through
    them, pulled toward ``prev`` (pitch, roll) with weight ``reg`` (mm^2),
    which only matters on fewer than three feet."""
    a0, b0 = plane_coeffs(*prev)
    # unknowns (a, b, H): a X + b Y - H = -(z + r); normal equations + reg on a, b
    saa = sab = sah = sbb = sbh = shh = 0.0
    ra = rb = rh = 0.0
    r = model.foot_r_mm
    for leg in LEGS:
        if not support >> leg & 1:
            continue
        hx, hy = model.hip(leg)
        X, Y = hx + xs[leg], hy
        t = -(zs[leg] + r)
        saa += X * X
        sab += X * Y
        sah -= X
        sbb += Y * Y
        sbh -= Y
        shh += 1.0
        ra += X * t
        rb += Y * t
        rh -= t
    saa += reg
    sbb += reg
    ra += reg * a0
    rb += reg * b0
    if shh == 0:
        return prev
    # solve the symmetric 3x3 [[saa sab sah] [sab sbb sbh] [sah sbh shh]] (a b H) = (ra rb rh)
    m = ((saa, sab, sah), (sab, sbb, sbh), (sah, sbh, shh))
    sol = _solve3(m, (ra, rb, rh))
    if sol is None:
        return prev
    return attitude_of(sol[0], sol[1])


def _solve3(m, v):
    (a, b, c), (d, e, f), (g, h, i) = m
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return None
    x = (v[0] * (e * i - f * h) - b * (v[1] * i - f * v[2]) + c * (v[1] * h - e * v[2])) / det
    y = (a * (v[1] * i - f * v[2]) - v[0] * (d * i - f * g) + c * (d * v[2] - v[1] * g)) / det
    z = (a * (e * v[2] - v[1] * h) - b * (d * v[2] - v[1] * g) + v[0] * (d * h - e * g)) / det
    return x, y, z


def rotation(pitch_deg: float, roll_deg: float):
    """Rows of R = Ry(pitch) Rx(-roll) (module docstring)."""
    t, f = math.radians(pitch_deg), math.radians(roll_deg)
    ct, st, cf, sf = math.cos(t), math.sin(t), math.cos(f), math.sin(f)
    return ((ct, -st * sf, st * cf), (0.0, cf, sf), (-st, -ct * sf, ct * cf))


def world_points(model: BodyModel, xs, zs, pitch_deg: float, roll_deg: float):
    """World (x, y, height) of the COM and of the four feet's ground points,
    rotated by the attitude (origin: the body frame's)."""
    R = rotation(pitch_deg, roll_deg)
    cx, cy, cz = model.com(xs, zs)
    com = _rot(R, cx, cy, cz)
    feet = []
    for leg in LEGS:
        hx, hy = model.hip(leg)
        feet.append(_rot(R, hx + xs[leg], hy, -(zs[leg] + model.foot_r_mm)))
    return com, feet


def _rot(R, x, y, z):
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z)


# ---- support polygon ------------------------------------------------------------------------
def hull(points):
    """Convex hull (counter-clockwise) of a few 2D points."""
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-9:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-9:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _seg_dist(p, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def margin(p, poly) -> float:
    """Signed distance (mm) of point p from the convex polygon's boundary:
    positive inside. A segment (two feet) or a point (one foot) has no
    inside: minus the distance to it."""
    n = len(poly)
    if n == 0:
        return -math.inf
    if n == 1:
        return -math.hypot(p[0] - poly[0][0], p[1] - poly[0][1])
    if n == 2:
        return -_seg_dist(p, poly[0], poly[1])
    inside = True
    best = math.inf
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        d = (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / L  # left of a->b (inside for CCW) is +
        if d < 0:
            inside = False
        best = min(best, d)
    if inside:
        return best
    return -min(_seg_dist(p, poly[i], poly[(i + 1) % n]) for i in range(n))


def stability(model: BodyModel, xs, zs, support: int, pitch_deg: float, roll_deg: float) -> float:
    """Margin (mm) of the COM's projection inside the polygon of the support
    feet, with the body at that attitude."""
    com, feet = world_points(model, xs, zs, pitch_deg, roll_deg)
    poly = hull([(f[0], f[1]) for leg, f in enumerate(feet) if support >> leg & 1])
    return margin((com[0], com[1]), poly)
