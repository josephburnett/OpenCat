"""Test-vector cases for protocol/vectors/*.json (used by gen.py).

Frame vectors are built from field values with the generated Python bindings
and the Python codec, then checked byte for byte against literals where an
independent source exists (the doc's worked examples, published COBS examples).
The C tests re-derive every frame independently and must agree.
"""
from __future__ import annotations

import random

# Worked examples. These literals are also in docs/architecture.md Appendix E.
DOC_WORKED = [
    {"name": "doc_hello_req_seq0", "msg": "HELLO_REQ", "seq": 0, "fields": {},
     "decoded": "01003e2e", "wire": "0201033e2e00"},
    {"name": "doc_set_posture_stand_seq1", "msg": "SET_POSTURE", "seq": 1,
     "fields": {"epoch": 0, "posture": 3, "transition_ticks": 0, "flags": 1},
     "decoded": "100100030001b4f7", "wire": "03100102030401b4f700"},
]

# Published COBS examples (Wikipedia "Consistent Overhead Byte Stuffing"),
# without the trailing delimiter.
_R = bytes(range(1, 256))
COBS_LITERALS = [
    ("00", "0101"),
    ("0000", "010101"),
    ("00110000", "0102110101"),  # leading zero, then 11 00 00
    ("11220033", "0311220233"),
    ("11223344", "0511223344"),
    ("11000000", "0211010101"),
    (_R[:254].hex(), "ff" + _R[:254].hex()),
    (("00" + _R[:254].hex()), "01ff" + _R[:254].hex()),
    (_R[:255].hex(), "ff" + _R[:254].hex() + "02ff"),
    ((_R[1:255] + b"\x00").hex(), "ff" + _R[1:255].hex() + "0101"),
    ((_R[2:255] + b"\x00\x01"), None),  # filled in below
]
COBS_LITERALS[-1] = ((_R[2:255] + b"\x00\x01").hex(), "fe" + _R[2:255].hex() + "0201")


def _kf(dur, hold, flags, ang):
    return {"dur": dur, "hold": hold, "flags": flags, "ang": ang}


def frame_cases():
    """(name, msg, seq, fields, note)"""
    U = -128
    kf_a = _kf(25, 0, 1, [0, -10, 30, 45, 45, -45, -45, 10, 10, -10, -10])
    kf_b = _kf(10, 5, 0x05, [U, U, U, 60, 60, -60, -60, U, U, U, U])
    kf_c = _kf(1, 0, 2, [0] * 11)  # zero-heavy
    kf_d = _kf(255, 255, 0, [127, -127, 0, 1, -1, 0, U, 0, 0, 0, 0])
    return [
        # Pi -> body
        ("hello_req_seq7", "HELLO_REQ", 7, {}, ""),
        ("heartbeat", "HEARTBEAT", 42, {}, ""),
        ("heartbeat_seq255", "HEARTBEAT", 255, {}, "seq wrap edge"),
        ("ping", "PING", 3, {"token": 0xBEEF}, ""),
        ("ping_zero_token", "PING", 0, {"token": 0}, "zero payload bytes"),
        ("set_stream_default", "SET_STREAM", 4, {"telem_div": 2, "joints_div": 0, "imu_raw_div": 0}, ""),
        ("stats_req", "STATS_REQ", 5, {}, ""),
        ("pca_read", "PCA_READ", 6, {}, ""),
        ("set_posture_rest_epoch255", "SET_POSTURE", 8,
         {"epoch": 255, "posture": 1, "transition_ticks": 50, "flags": 0}, "epoch byte first"),
        ("set_gait_trot", "SET_GAIT", 9,
         {"epoch": 3, "run": 1, "pattern": 1, "period_ms": 500, "duty": 140, "stride_mm": 35,
          "turn_mm": -10, "step_h_mm": 12, "body_h_mm": 80, "flags": 7}, "epoch byte first"),
        ("set_gait_stop", "SET_GAIT", 10,
         {"epoch": 0, "run": 0, "pattern": 0, "period_ms": 800, "duty": 192, "stride_mm": 0,
          "turn_mm": 0, "step_h_mm": 0, "body_h_mm": 0, "flags": 0}, "many zero bytes"),
        ("set_head_hold", "SET_HEAD", 11,
         {"epoch": 3, "mode": 1, "pan": -480, "tilt": 160, "tail": 0, "max_speed_dps": 120}, ""),
        ("set_body_pose", "SET_BODY_POSE", 12,
         {"epoch": 3, "pitch": 80, "roll": -48, "dheight_mm": -5, "xshift_mm": 4}, ""),
        ("set_balance", "SET_BALANCE", 13,
         {"enable": 1, "kp_q8": 128, "kd_ms": 20, "deadband_deg": 1, "max_corr_deg": 15}, ""),
        ("trick_begin", "TRICK_BEGIN", 14,
         {"trick_id": 7, "n_frames": 22, "loop_start": 2, "loop_end": 5, "repeats": 3,
          "joint_mask": 0xFF07, "flags": 1, "end_posture": 2}, ""),
        ("trick_frames_1", "TRICK_FRAMES", 15, {"first": 0, "frames": [kf_a]}, "min variable length"),
        ("trick_frames_4_max", "TRICK_FRAMES", 16,
         {"first": 4, "frames": [kf_a, kf_b, kf_c, kf_d]}, "58 B payload, -128 and zero bytes"),
        ("trick_commit", "TRICK_COMMIT", 17, {"trick_id": 7, "crc16": 0x1D0F}, ""),
        ("trick_play_mirror", "TRICK_PLAY", 18,
         {"epoch": 3, "trick_id": 7, "speed_q6": 96, "flags": 1}, "epoch byte first"),
        ("trick_stop", "TRICK_STOP", 19, {"transition_ticks": 10}, ""),
        ("set_eyes_left_green", "SET_EYES", 20, {"led_mask": 0x38, "r": 0, "g": 255, "b": 0}, ""),
        ("beep_1", "BEEP", 21, {"notes": [{"note": 12, "dur_10ms": 10}]}, ""),
        ("beep_8_max", "BEEP", 22,
         {"notes": [{"note": n, "dur_10ms": d} for n, d in
                    [(0, 5), (2, 5), (4, 5), (5, 5), (7, 5), (9, 5), (11, 5), (-12, 50)]]}, "max notes"),
        ("cal_get", "CAL_GET", 23, {}, ""),
        ("cal_set", "CAL_SET", 24, {"joint": 8, "offset_a16": -64}, ""),
        ("cal_save", "CAL_SAVE", 25, {}, ""),
        ("cal_revert", "CAL_REVERT", 26, {}, ""),
        ("imu_offsets_set", "IMU_OFFSETS_SET", 27,
         {"accel_x": -1234, "accel_y": 567, "accel_z": 1688, "gyro_x": 12, "gyro_y": -3,
          "gyro_z": 0, "save": 1}, ""),
        ("param_get", "PARAM_GET", 28, {"id": 1}, ""),
        ("param_set_persist", "PARAM_SET", 29, {"id": 5, "value": -300, "persist": 1}, ""),
        ("eeprom_read", "EEPROM_READ", 30, {"addr": 0, "len": 56}, ""),
        ("reboot", "REBOOT", 31, {"magic": 0x4B49}, ""),
        # Body -> Pi
        ("hello_boot", "HELLO", 0,
         {"proto_major": 1, "proto_minor": 0, "fw_build": 0xDEADBEEF, "reset_cause": 1,
          "last_alive_tick": 0, "caps": 0x1D, "max_payload": 60, "trick_capacity": 32,
          "joint_count": 11, "config_state": 0, "epoch": 0}, "unsolicited on boot"),
        ("telemetry", "TELEMETRY", 1,
         {"tick": 51234, "pitch": -35, "roll": 12, "rate_pitch": 160, "rate_roll": -16,
          "rate_yaw": 0, "dist_mm": 312, "batt_mv": 7820, "mode": 3, "owners": 0x11,
          "flags": 0x08AD, "last_rx_seq": 22, "trick_frame": 0xFF, "gait_phase": 128,
          "tick_load": 47, "rx_err": 0, "epoch": 3}, ""),
        ("joints", "JOINTS", 2,
         {"tick": 100, "angle": [0, -160, 480, 720, 720, -720, -720, 160, 160, -160, -1440]}, ""),
        ("imu_raw", "IMU_RAW", 3,
         {"tick": 101, "accel_x": 120, "accel_y": -340, "accel_z": 16384, "gyro_x": -5,
          "gyro_y": 3, "gyro_z": 0}, ""),
        ("ack_ok", "ACK", 4, {"seq": 30, "type": 0x3F, "status": 0}, ""),
        ("ack_stale_trick_play", "ACK", 5, {"seq": 18, "type": 0x23, "status": 10}, ""),
        ("event_fell", "EVENT", 6, {"code": 2, "arg": -1, "tick": 4000, "epoch": 4}, ""),
        ("event_goal_refused", "EVENT", 7,
         {"code": 13, "arg": (0x11 << 8) | 10, "tick": 4100, "epoch": 4}, "GAIT refused STALE"),
        ("cal_values_zero", "CAL_VALUES", 8, {"offset": [0] * 16}, "32 zero bytes: COBS stress"),
        ("cal_values", "CAL_VALUES", 9,
         {"offset": [16, -32, 0, 0, 0, 0, 0, 0, 48, -48, 64, -64, 80, -80, 96, -96]}, ""),
        ("param_value", "PARAM_VALUE", 10,
         {"id": 0, "value": 500, "min": 100, "max": 5000, "default": 500}, ""),
        ("eeprom_data_1", "EEPROM_DATA", 11, {"addr": 704, "data": "a5"}, "min variable length"),
        ("eeprom_data_56_zero", "EEPROM_DATA", 12, {"addr": 0, "data": "00" * 56},
         "58 B payload of mostly 0x00: COBS stress"),
        ("pong", "PONG", 13, {"token": 0xBEEF, "tick": 777}, ""),
        ("stats", "STATS", 14,
         {"uptime_s": 86400, "overruns": 0, "max_tick_us": 9876, "stack_free_min": 212,
          "rx_crc_err": 1, "rx_overflow": 0, "i2c_err": 2, "imu_stale": 0, "dmp_rate_hz": 100}, ""),
        ("pca_counts", "PCA_COUNTS", 15,
         {"counts": [0x1000, 0x1000, 0x1000, 0, 0, 0, 0, 0, 307, 300, 290, 310, 205, 410, 205, 410]},
         "full-off bit"),
        ("log_max", "LOG", 16,
         {"text": b"kilo: 60 byte log line ........................ end|".ljust(60, b"!").hex()},
         "max payload 60 B"),
    ]


def build(P, C):
    frames = []
    for name, msg, seq, fields, note in [(d["name"], d["msg"], d["seq"], d["fields"], "doc worked example")
                                         for d in DOC_WORKED] + frame_cases():
        cls = _cls(P, msg)
        obj = P.from_dict(cls, fields)
        payload = obj.pack()
        dec = C.build_decoded(cls.ID, seq, payload)
        wire = C.build_frame(cls.ID, seq, payload)
        frames.append({
            "name": name, "msg": msg, "type": cls.ID, "dir": cls.DIR.value,
            "kind": cls.KIND.value, "seq": seq, "fields": P.to_dict(obj),
            "payload": payload.hex(), "decoded": dec.hex(), "wire": wire.hex(),
            "note": note,
        })
    for d in DOC_WORKED:
        f = next(x for x in frames if x["name"] == d["name"])
        if (f["decoded"], f["wire"]) != (d["decoded"], d["wire"]):
            raise AssertionError(f"worked example {d['name']} mismatch: {f['decoded']} {f['wire']}")
    assert len(frames) >= 30
    for m in P.MESSAGES.values():
        assert any(f["type"] == m.ID for f in frames), f"no vector for {m.NAME}"

    def wire_of(t, s, payload):
        return C.build_frame(t, s, bytes.fromhex(payload) if isinstance(payload, str) else payload).hex()

    def raw_frame(body: bytes, crc_xor: int = 0) -> str:
        c = C.crc16(body) ^ crc_xor
        return (C.cobs_encode(body + bytes([c & 0xFF, c >> 8])) + b"\x00").hex()

    ping = bytes.fromhex(next(f for f in frames if f["name"] == "ping")["decoded"])
    tf = next(f for f in frames if f["name"] == "trick_frames_1")
    errors = [
        {"name": "crc_flipped", "wire": raw_frame(bytes([0x03, 0x03, 0xEF, 0xBE]), 0x0001),
         "error": "bad_crc"},
        {"name": "payload_bit_flipped", "wire": (C.cobs_encode(ping[:2] + bytes([ping[2] ^ 0x10]) + ping[3:]) + b"\x00").hex(),
         "error": "bad_crc"},
        {"name": "doc_example_crc_swapped", "wire": "0201032e3e00", "error": "bad_crc"},
        {"name": "cobs_code_past_end", "wire": "0501020300", "error": "bad_cobs"},
        {"name": "cobs_last_block_short", "wire": "0201033e00", "error": "bad_cobs"},
        {"name": "cobs_zero_inside", "wire": "0301000200", "error": "bad_cobs"},
        {"name": "cobs_empty", "wire": "00", "error": "bad_cobs"},
        {"name": "decoded_too_short", "wire": (C.cobs_encode(bytes([0x02, 0x00, 0x00])) + b"\x00").hex(),
         "error": "bad_length"},
        {"name": "encoded_too_long", "wire": (C.cobs_encode(bytes([0x8F, 0x00]) + b"x" * 61 + b"\x00\x00") + b"\x00").hex(),
         "error": "bad_length"},
        {"name": "ping_payload_too_long", "wire": wire_of(0x03, 9, "efbe00"), "error": "bad_length",
         "type": 0x03, "seq": 9},
        {"name": "heartbeat_with_payload", "wire": wire_of(0x02, 10, "00"), "error": "bad_length",
         "type": 0x02, "seq": 10},
        {"name": "trick_frames_count_mismatch",
         "wire": wire_of(0x21, 11, "0002" + tf["payload"][4:]), "error": "bad_length", "type": 0x21, "seq": 11},
        {"name": "beep_zero_notes", "wire": wire_of(0x31, 12, "00"), "error": "bad_length",
         "type": 0x31, "seq": 12},
        {"name": "eeprom_data_empty", "wire": wire_of(0x88, 13, "c002"), "error": "bad_length",
         "type": 0x88, "seq": 13},
        {"name": "telemetry_short", "wire": wire_of(0x81, 14, "00" * 25), "error": "bad_length",
         "type": 0x81, "seq": 14},
        {"name": "unknown_pi_type", "wire": wire_of(0x7D, 15, ""), "error": "unknown_type",
         "type": 0x7D, "seq": 15},
        {"name": "unknown_body_type", "wire": wire_of(0x90, 16, "0102"), "error": "unknown_type",
         "type": 0x90, "seq": 16},
        {"name": "type_zero", "wire": wire_of(0x00, 17, ""), "error": "unknown_type",
         "type": 0x00, "seq": 17},
    ]
    for e in errors:
        try:
            C.parse_frame(bytes.fromhex(e["wire"]))
        except C.FrameError as ex:
            if ex.kind != e["error"]:
                raise AssertionError(f"{e['name']}: got {ex.kind}, want {e['error']}")
        else:
            raise AssertionError(f"{e['name']}: parsed OK")

    # Streams: concatenated bytes and the expected sequence of results.
    w = {f["name"]: f["wire"] for f in frames}
    streams = [
        {"name": "back_to_back_with_resync_zeros",
         "bytes": "0000" + w["doc_hello_req_seq0"] + w["ping"] + "00" + w["telemetry"],
         "expect": [{"ok": 0x01, "seq": 0}, {"ok": 0x03, "seq": 3}, {"ok": 0x81, "seq": 1}]},
        {"name": "dropped_byte_damages_one_frame",
         "bytes": w["ping"][:4] + w["ping"][6:] + w["set_head_hold"],
         "expect": [{"error": "bad_crc"}, {"ok": 0x12, "seq": 11}]},
        {"name": "garbage_then_frame",
         "bytes": "ffffff00" + w["heartbeat"],
         "expect": [{"error": "bad_cobs"}, {"ok": 0x02, "seq": 42}]},
        {"name": "overflow_then_frame",
         "bytes": "01" * 70 + "00" + w["pong"],
         "expect": [{"error": "overflow"}, {"ok": 0x89, "seq": 13}]},
    ]
    for s in streams:
        got = []
        for r in C.StreamDecoder().feed(bytes.fromhex(s["bytes"])):
            got.append({"error": r.kind} if isinstance(r, C.FrameError) else {"ok": r.type, "seq": r.seq})
        if got != s["expect"]:
            raise AssertionError(f"stream {s['name']}: {got}")

    frames_doc = {
        "comment": "GENERATED by protocol/gen.py (cases in protocol/vector_cases.py). "
                   "wire includes the trailing 0x00. error kinds: bad_cobs, bad_crc, "
                   "bad_length, unknown_type, overflow (streams only). For bad_length on a "
                   "CRC-valid frame and unknown_type, type/seq are trustworthy.",
        "crc_check": {"input_ascii": "123456789", "crc": 0x29B1},
        "frames": frames,
        "errors": errors,
        "streams": streams,
    }

    cobs = []
    for data, enc in COBS_LITERALS:
        got = C.cobs_encode(bytes.fromhex(data)).hex()
        if got != enc:
            raise AssertionError(f"COBS literal mismatch for {data[:16]}...: {got[:32]}")
        cobs.append({"data": data, "encoded": enc, "source": "published example"})
    rng = random.Random(1234)
    for n in (0, 1, 2, 64, 253, 254, 255, 300, 508, 509, 600):
        data = bytes(rng.choice([0, 0, rng.randrange(1, 256)]) if rng.random() < 0.1 else rng.randrange(1, 256)
                     for _ in range(n))
        cobs.append({"data": data.hex(), "encoded": C.cobs_encode(data).hex(), "source": f"random n={n}"})
    for n in (508, 509, 762):
        data = bytes((i % 255) + 1 for i in range(n))
        cobs.append({"data": data.hex(), "encoded": C.cobs_encode(data).hex(), "source": f"nonzero run n={n}"})
    for c in cobs:
        assert C.cobs_decode(bytes.fromhex(c["encoded"])) == bytes.fromhex(c["data"]) or c["data"] == ""
    cobs_doc = {"comment": "GENERATED by protocol/gen.py. encoded excludes the 0x00 delimiter.",
                "vectors": cobs}
    return frames_doc, cobs_doc


def _cls(P, msg):
    return P.MESSAGES[P.MsgId[msg].value]
