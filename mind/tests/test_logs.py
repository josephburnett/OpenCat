"""Log reader, .kilolog format, summary/replay/plot (docs/architecture.md §5 "Logging")."""

from __future__ import annotations

import csv
import gzip
import json
import random
from pathlib import Path

import pytest

from kilo.link import consts
from kilo.logs import KilologWriter, analysis, load, read_kilolog
from kilo.logs.__main__ import main as logs_main
from kilo.protocol import kilo_link_codec as C
from kilo.protocol import kilo_protocol as P

FIXTURES = Path(__file__).parent / "fixtures"
SIGNCHECK = FIXTURES / "signcheck5_monitor.jsonl"

SERVOS = 1 << 2
FRESH = 1 << 11


def _session(t0: float = 1000.0):
    """A 14 s, 25 Hz session as (t, direction, type, seq, payload, fields) frames.

    quiet (pitch 0.9 roll -2.7) -> hand approaching at 400 mm/s from t=4 to
    100 mm -> no echo from t=8 -> tap at t=9 -> nose up to -60° from t=10,
    FELL at 10.9 (mode FALLEN). One telemetry is missing at t=3; ticks wrap.
    """
    rng = random.Random(42)
    frames = []
    tick = 65000
    d = 1200.0
    for i in range(14 * 25):
        t = i * 0.04
        tick = (tick + 2) & 0xFFFF
        if i == 75:
            continue  # lost frame
        fresh = i % 2 == 0
        if fresh and 4.0 <= t and d > 100:
            d = max(100.0, d - 400 * 0.08)
        dist = 0 if t >= 8.0 else int(d + rng.uniform(-5, 5))
        pitch = 0.9
        if 9.0 <= t < 9.04:
            pitch += 2.0
        elif 9.04 <= t < 9.08:
            pitch += 3.0
        elif 9.08 <= t < 9.12:
            pitch += 1.2
        if t >= 10.0:
            pitch = 0.9 - 61 * min(1.0, (t - 10.0) / 0.8)
        mode = int(P.Mode.FALLEN) if t >= 10.9 else int(P.Mode.IDLE)
        tel = P.Telemetry(tick=tick, pitch=round((pitch + rng.uniform(-0.3, 0.3)) * 16),
                          roll=round((-2.7 + rng.uniform(-0.3, 0.3)) * 16),
                          rate_pitch=round(rng.uniform(-2, 2) * 16), rate_roll=0, rate_yaw=0,
                          dist_mm=dist, batt_mv=7900 - i // 50, mode=mode, owners=0,
                          flags=SERVOS | (1 << 7) | (FRESH if fresh else 0), last_rx_seq=0,
                          trick_frame=255, gait_phase=0, tick_load=55, rx_err=0, epoch=1)
        frames.append((t0 + t, "rx", tel, i & 0xFF))
        if i % 3 == 0:
            frames.append((t0 + t + 0.001, "tx", P.Heartbeat() if hasattr(P, "Heartbeat") else None, i & 0xFF))
        if abs(t - 10.92) < 0.02:
            frames.append((t0 + t + 0.002, "rx", P.Event(code=consts.EV["FELL"], arg=-950, tick=tick, epoch=2),
                           (i + 1) & 0xFF))
    return [f for f in frames if f[2] is not None]


def write_jsonl(path: Path, frames) -> Path:
    with open(path, "w") as fh:
        fh.write(json.dumps({"t": frames[0][0] - 0.5, "wall": 1.7e9, "kind": "link", "connected": True,
                             "port": "/dev/ttyAMA0"}) + "\n")
        for t, direction, msg, seq in frames:
            payload = msg.pack().hex()
            if direction == "tx":
                fh.write(json.dumps({"t": t, "wall": 1.7e9 + t, "kind": "tx", "type": msg.NAME, "seq": seq,
                                     "payload": payload}) + "\n")
                continue
            fields = P.to_dict(msg)
            rec = {"t": t, "wall": 1.7e9 + t, "kind": "rx", "type": msg.NAME, "seq": seq,
                   "payload": payload, "fields": fields}
            # half the TELEMETRY records without fields: decoded from the payload hex
            if msg.NAME == "TELEMETRY" and seq % 2:
                rec["fields"] = None
            fh.write(json.dumps(rec) + "\n")
            if msg.NAME == "EVENT":
                fh.write(json.dumps({"t": t, "wall": 1.7e9 + t, "kind": "event", "name": "FELL",
                                     "code": msg.code, "arg": msg.arg, "tick": msg.tick,
                                     "epoch": msg.epoch}) + "\n")
        fh.write(json.dumps({"t": frames[-1][0], "wall": 0, "kind": "rx_error", "error": "bad_crc",
                             "raw": "", "type": None, "seq": None}) + "\n")
        fh.write("not json\n")
    return path


def write_kilolog(path: Path, frames) -> Path:
    t0 = frames[0][0]
    with KilologWriter(path, {"source": "test"}, clock=lambda: t0) as w:
        for t, direction, msg, seq in frames:
            w.write_message(msg, seq, direction, t)
        w.write_wire(b"\x05\x01\x02\x03\x00", "rx", frames[-1][0])  # garbage
    return path


@pytest.fixture
def session_logs(tmp_path):
    frames = _session()
    return (write_jsonl(tmp_path / "kilo-test.jsonl", frames),
            write_kilolog(tmp_path / "kilo-test.kilolog", frames))


# ---- readers ------------------------------------------------------------------------

def test_read_link_jsonl(session_logs):
    jl, _ = session_logs
    data = load(jl)
    assert data.format == "jsonl-link"
    assert len(data.samples) == 14 * 25 - 1
    assert data.samples[0].t == pytest.approx(0.5)  # rebased on the first record (the link record)
    s = data.samples[0]
    assert s.pitch_deg == pytest.approx(0.9, abs=0.35) and s.roll_deg == pytest.approx(-2.7, abs=0.35)
    assert s.mode == "IDLE" and "servos_powered" in s.flags and s.tick_load_ms == 5.5
    assert [e.name for e in data.events] == ["FELL"] and data.events[0].arg == -950
    assert data.rx_errors == {"bad_crc": 1} and data.bad_lines == 1
    assert data.tx_types["HEARTBEAT"] > 100


def test_kilolog_roundtrip_matches_jsonl(session_logs):
    jl, kl = session_logs
    a, b = load(jl), load(kl)
    assert b.format == "kilolog" and b.meta["source"] == "test" and b.meta["proto"] == [P.PROTO_MAJOR, P.PROTO_MINOR]
    assert len(a.samples) == len(b.samples)
    assert [s.pitch_deg for s in a.samples] == [s.pitch_deg for s in b.samples]
    assert [s.tick for s in a.samples] == [s.tick for s in b.samples]
    assert b.samples[5].t == pytest.approx(a.samples[5].t - 0.5, abs=0.002)  # ms resolution
    assert [e.name for e in b.events] == ["FELL"]
    assert b.rx_errors["bad_crc"] + b.rx_errors["bad_cobs"] + b.rx_errors["bad_length"] == 1


def test_kilolog_records_are_wire_frames(tmp_path):
    p = tmp_path / "x.kilolog.gz"
    with KilologWriter(p, clock=lambda: 0.0) as w:
        w.write_frame(int(P.MsgId.TELEMETRY), 7, P.Telemetry(tick=1).pack(), "rx", 0.25)
        w.write_frame(int(P.MsgId.PING), 8, P.Ping().pack(), "tx", 0.5)
    with gzip.open(p, "rb") as fh:
        assert fh.read(8) == b"KILOLOG\n"
    meta, recs = read_kilolog(p)
    recs = list(recs)
    assert meta["version"] == 1 and meta["tick_hz"] == 50
    assert [(r.t, r.direction, r.frame.type, r.frame.seq) for r in recs] == [
        (0.25, "rx", int(P.MsgId.TELEMETRY), 7), (0.5, "tx", int(P.MsgId.PING), 8)]
    # the stored bytes are exactly the wire frame, replayable through the stream decoder
    dec = C.StreamDecoder()
    out = [f for r in recs for f in dec.feed(r.wire)]
    assert [f.seq for f in out] == [7, 8]


def test_kilolog_truncated_tail_ignored(session_logs, tmp_path):
    _, kl = session_logs
    raw = kl.read_bytes()
    cut = tmp_path / "cut.kilolog"
    cut.write_bytes(raw[:-3])
    assert len(load(cut).samples) == len(load(kl).samples)


def test_read_monitor_fixture():
    data = load(SIGNCHECK)
    assert data.format == "jsonl-monitor"
    assert len(data.samples) == 40 and data.samples[-1].t == pytest.approx(39.0)
    assert min(s.pitch_deg for s in data.samples) < -73
    assert min(s.roll_deg for s in data.samples) < -87


# ---- summary / replay / plot ----------------------------------------------------------

def test_summary_link(session_logs):
    jl, _ = session_logs
    s = analysis.summary(load(jl))
    assert s["samples"] == 349
    assert s["telemetry_hz"] == pytest.approx(25, abs=0.2)
    assert s["tick_step"] == 2 and s["telemetry_gaps"] == 1 and s["telemetry_missed"] == 1
    assert s["pitch_deg"]["min"] < -59 and s["roll_deg"]["mean"] == pytest.approx(-2.7, abs=0.1)
    assert s["battery_mv"]["first"] == 7900 and s["battery_mv"]["min"] == 7894
    assert [m for _, m in s["modes"]] == ["IDLE", "FALLEN"]
    assert s["event_counts"] == {"FELL": 1}
    assert 0.3 < s["distance_mm"]["echo_fraction"] < 0.7
    text = analysis.format_summary(s)
    assert "FELL" in text and "IDLE" in text and "24.9 Hz" in text


def test_summary_monitor_fixture():
    s = analysis.summary(load(SIGNCHECK))
    assert s["duration_s"] == pytest.approx(39.0)
    assert s["telemetry_hz"] == pytest.approx(1.0)
    assert s["pitch_deg"]["max"] == pytest.approx(0.9, abs=0.5)
    assert "link" not in s


def test_replay_link_session(session_logs):
    jl, kl = session_logs
    for path in (jl, kl):
        evs = analysis.replay(load(path))
        names = [e.name for e in evs]
        # ultrasonic: approaching then near, clear on the no-echo; IMU: the tap, then the tilt
        order = [n for n in names if n in ("approaching", "near", "clear", "tap", "tilted")]
        assert order == ["approaching", "near", "clear", "tap", "tilted"], names
        tilted = next(e for e in evs if e.name == "tilted")
        assert tilted.data["direction"] == "nose_up" and 10.5 < tilted.t - (0.5 if path == jl else 0) < 11.5
        assert "pet" not in names


def test_replay_signcheck_fixture():
    evs = analysis.replay(load(SIGNCHECK))
    assert [(e.name, e.data.get("direction")) for e in evs] == [
        ("tilted", "nose_up"), ("level", None), ("tilted", "left_up"), ("level", None)]
    assert 14.0 <= evs[0].t <= 15.0 and 24.0 <= evs[2].t <= 25.0


def test_plot_falls_back_to_csv(session_logs, tmp_path):
    jl, _ = session_logs
    path, note = analysis.plot(load(jl), ["pitch", "roll", "dist", "mode"], tmp_path / "out.csv")
    assert path.suffix == ".csv" and "CSV" in note
    rows = list(csv.reader(open(path)))
    assert rows[0] == ["t", "pitch_deg", "roll_deg", "dist_mm", "mode"]
    assert len(rows) == 350
    assert rows[1][4] == str(int(P.Mode.IDLE))
    with pytest.raises(ValueError, match="balance corrections"):
        analysis.plot(load(jl), ["corr"])
    with pytest.raises(ValueError, match="unknown field"):
        analysis.plot(load(jl), ["nope"])


def test_plot_png_or_csv_default_path(tmp_path):
    src = tmp_path / "sc.jsonl"
    src.write_text(SIGNCHECK.read_text())
    path, note = analysis.plot(load(src), ["pitch", "roll"])
    assert path.parent == tmp_path and path.name.startswith("sc.jsonl.pitch_deg-roll_deg")
    assert path.exists() and path.suffix in (".png", ".csv")


def test_cli_module(session_logs, tmp_path, capsys):
    jl, kl = session_logs
    assert logs_main(["summary", str(kl)]) == 0
    assert "349 telemetry samples" in capsys.readouterr().out
    assert logs_main(["summary", str(SIGNCHECK), "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["samples"] == 40
    assert logs_main(["replay", str(jl), "--with-body-events"]) == 0
    out = capsys.readouterr().out
    assert "approaching" in out and "FELL" in out
    assert logs_main(["replay", str(SIGNCHECK), "--json"]) == 0
    lines = [json.loads(l) for l in capsys.readouterr().out.splitlines()]
    assert [l["name"] for l in lines] == ["tilted", "level", "tilted", "level"]
    assert logs_main(["plot", str(jl), "--fields", "pitch,batt", "--csv", "--out", str(tmp_path / "p")]) == 0
    assert (tmp_path / "p.csv").exists()
    assert logs_main(["plot", str(jl), "--fields", "corr"]) == 2
    assert logs_main(["summary", str(tmp_path / "missing.jsonl")]) == 1
