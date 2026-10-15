import pytest

from kilo.body.state import BodyState, battery_percent, decode_flags
from kilo.protocol import kilo_protocol as P


def tel(**kw):
    f = dict(tick=1, pitch=0, roll=0, rate_pitch=0, rate_roll=0, rate_yaw=0, dist_mm=0,
             batt_mv=7600, mode=2, owners=0x40, flags=(1 << 2) | (1 << 7), last_rx_seq=0,
             trick_frame=255, tick_load=35, rx_err=0, epoch=0)
    f.update(kw)
    return P.Telemetry(**f)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_battery_curve():
    assert battery_percent(8400) == 100
    assert battery_percent(9000) == 100
    assert battery_percent(6000) == 0
    assert battery_percent(7600) == 50
    assert battery_percent(7700) == pytest.approx(57.5)
    assert battery_percent(0) is None
    assert battery_percent(None) is None


def test_flags_decode():
    assert decode_flags((1 << 0) | (1 << 9)) == {"balance_active", "no_battery"}


def test_tick_unwrap_and_backwards():
    s = BodyState()
    assert s.on_telemetry(tel(tick=65000), 0)[0] is False
    assert s.on_telemetry(tel(tick=10), 0)[0] is False
    assert s.tick == 65536 + 10
    assert s.on_telemetry(tel(tick=40000), 0)[0] is True


def test_epoch_tracking():
    s = BodyState()
    assert s.on_telemetry(tel(epoch=3), 0)[1] is None
    assert s.on_telemetry(tel(epoch=4), 0)[1] == 3
    assert s.epoch == 4


def test_derived_values_and_alive():
    clk = Clock()
    s = BodyState(clock=clk)
    s.connected = True
    s.on_telemetry(tel(pitch=16 * 10, roll=-16 * 5, dist_mm=500), clk())
    assert s.pitch_deg == 10 and s.roll_deg == -5
    assert s.upright is True
    assert s.mode == "IDLE"
    assert s.owners == ("IDLE", "PI_HOLD")
    assert "servos_powered" in s.flags and "link_ok" in s.flags
    assert not s.moving and not s.fallen
    assert s.battery_pct == 50
    assert s.alive
    clk.t += 0.31
    assert not s.alive
    s.on_telemetry(tel(mode=5, roll=16 * 80), clk())
    assert s.fallen and s.upright is False
    s.on_telemetry(tel(flags=1 << 3, mode=3), clk())
    assert s.moving
    s.on_telemetry(tel(flags=1 << 9), clk())
    assert s.battery_pct is None  # NO_BATTERY: on the bench supply / USB


def test_distance_median_and_approach():
    clk = Clock()
    s = BodyState(clock=clk)
    for d in (1000, 2000, 900, 800, 0, 700):  # 0 = no echo, skipped; 2000 a spike
        clk.t += 0.08
        s.on_telemetry(tel(dist_mm=d), clk())
    assert s.distance_mm == 800
    assert s.approach_mm_s > 0


def test_snapshot_shape():
    s = BodyState()
    snap = s.snapshot()
    for k in ("connected", "alive", "epoch", "mode", "pitch_deg", "battery_pct", "stats", "events",
              "reset_cause", "reboots"):
        assert k in snap
    s.on_telemetry(tel(), 0)
    s.on_hello(P.Hello(proto_major=1, proto_minor=1, fw_build=1, reset_cause=1, last_alive_tick=0,
                       caps=0, max_payload=60, trick_capacity=0, joint_count=11, config_state=0,
                       epoch=2), 0, expected_major=1)
    snap = s.snapshot()
    assert snap["reset_cause"] == "FLASHED" and snap["proto_ok"] is True and snap["epoch"] == 2
