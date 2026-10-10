"""Protocol constants transcribed from docs/architecture.md §4 and Appendix E.

TODO(protocol): import these from the generated module (kilo_protocol) once it
is integrated; spec.toml is authoritative and this file must then disappear.
Only constants the link, state model and CLI need are here.
"""

from __future__ import annotations

# --- message type codes (§4 message index) --------------------------------
TYPES: dict[str, int] = {
    "HELLO_REQ": 0x01,
    "HEARTBEAT": 0x02,
    "PING": 0x03,
    "SET_STREAM": 0x04,
    "STATS_REQ": 0x05,
    "PCA_READ": 0x06,
    "SET_POSTURE": 0x10,
    "SET_GAIT": 0x11,
    "SET_HEAD": 0x12,
    "SET_BODY_POSE": 0x13,
    "SET_BALANCE": 0x14,
    "TRICK_BEGIN": 0x20,
    "TRICK_FRAMES": 0x21,
    "TRICK_COMMIT": 0x22,
    "TRICK_PLAY": 0x23,
    "TRICK_STOP": 0x24,
    "SET_EYES": 0x30,
    "BEEP": 0x31,
    "CAL_GET": 0x38,
    "CAL_SET": 0x39,
    "CAL_SAVE": 0x3A,
    "CAL_REVERT": 0x3B,
    "IMU_OFFSETS_SET": 0x3C,
    "PARAM_GET": 0x3D,
    "PARAM_SET": 0x3E,
    "EEPROM_READ": 0x3F,
    "REBOOT": 0x7E,
    "HELLO": 0x80,
    "TELEMETRY": 0x81,
    "JOINTS": 0x82,
    "IMU_RAW": 0x83,
    "ACK": 0x84,
    "EVENT": 0x85,
    "CAL_VALUES": 0x86,
    "PARAM_VALUE": 0x87,
    "EEPROM_DATA": 0x88,
    "PONG": 0x89,
    "STATS": 0x8A,
    "PCA_COUNTS": 0x8B,
    "LOG": 0x8F,
}
NAMES: dict[int, str] = {v: k for k, v in TYPES.items()}

# Goals are never acked (§4); everything else Pi->body is a command.
GOAL_TYPES = frozenset(
    {"HEARTBEAT", "SET_POSTURE", "SET_GAIT", "SET_HEAD", "SET_BODY_POSE",
     "SET_BALANCE", "SET_EYES", "BEEP"}
)
# Goals whose payload starts with the epoch byte ("goal E").
EPOCH_GOALS = frozenset({"SET_POSTURE", "SET_GAIT", "SET_HEAD", "SET_BODY_POSE"})
# Desired motion goals cleared on FELL / WATCHDOG / LOW_BATTERY (§5).
MOTION_GOALS = EPOCH_GOALS
# Commands answered by a data reply instead of (or as well as) an ACK.
REPLIES: dict[str, str] = {
    "HELLO_REQ": "HELLO",
    "PING": "PONG",
    "STATS_REQ": "STATS",
    "PCA_READ": "PCA_COUNTS",
    "CAL_GET": "CAL_VALUES",
    "PARAM_GET": "PARAM_VALUE",
    "EEPROM_READ": "EEPROM_DATA",
}

# --- ACK status (§4) ---------------------------------------------------------
ACK_STATUS = {
    0: "OK", 1: "BAD_LEN", 2: "BAD_ARG", 3: "BUSY", 4: "UNKNOWN_TYPE",
    5: "CRC_MISMATCH", 6: "NO_TRICK", 7: "REFUSED_LOW_BATT", 8: "REFUSED_FALLEN",
    9: "EEPROM_BUSY", 10: "STALE",
}
OK = 0
UNKNOWN_TYPE = 4
STALE = 10
# GOAL_REFUSED reasons: assumed to share the ACK status codes (§4 does not say).
REFUSE_REASON = {1: "BAD_LEN", 2: "BAD_ARG", 7: "LOW_BATT", 8: "FALLEN", 10: "STALE"}

# --- events (Appendix E) -----------------------------------------------------
EVENTS = {
    1: "BOOT", 2: "FELL", 3: "RECOVERED", 4: "RECOVERY_FAILED", 5: "LOW_BATTERY",
    6: "BATTERY_OK", 7: "WATCHDOG", 8: "LINK_RESTORED", 9: "IMU_FAULT",
    10: "IMU_OK", 11: "TRICK_DONE", 12: "GAIT_STOPPED", 13: "GOAL_REFUSED",
    14: "CONFIG_MIGRATED", 15: "CONFIG_DEFAULTED", 16: "EEPROM_SAVED",
    17: "TX_DROPPED",
}
EV = {v: k for k, v in EVENTS.items()}
CLEARING_EVENTS = frozenset({EV["FELL"], EV["WATCHDOG"], EV["LOW_BATTERY"]})

# --- telemetry enums (§4) ------------------------------------------------------
MODES = {
    0: "BOOT", 1: "LIMP", 2: "IDLE", 3: "GAIT", 4: "TRICK", 5: "FALLEN",
    6: "RECOVERING", 7: "LOW_BATT", 8: "CALIB", 9: "LINK_LOST",
}
FLAG_BITS = {
    0: "balance", 1: "imu_fault", 2: "servos_powered", 3: "gait", 4: "trick",
    5: "head_held", 6: "batt_warn", 7: "link_ok", 8: "overrun", 9: "no_battery",
    10: "eeprom_pending", 11: "dist_fresh",
}
OWNERS = {0: "IDLE", 1: "GAIT", 2: "TRICK", 3: "REFLEX", 4: "PI"}  # TODO(protocol): numeric owner codes are not in the doc
RESET_CAUSES = {0: "COLD", 1: "FLASHED", 2: "REQUESTED", 3: "WARM_UNEXPECTED"}
POSTURES = {"limp": 0, "rest": 1, "sit": 2, "stand": 3, "stance": 4, "calib": 5, "zero": 6}

REBOOT_MAGIC = 0x4B49
CAL_JOINTS = 16
