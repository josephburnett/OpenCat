// The link layer (core/link.cpp): every Pi->body frame in
// protocol/vectors/frames.json is fed to the real dispatch and must produce
// the expected reply type / ACK status / event; plus de-dup, the query rules,
// GOAL_REFUSED mapping and rate limit, HELLO, the streams and the TX drop
// priority, calibration, params and EEPROM reads (doc §4, Appendix E).
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include "../src/core/body.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/posture.h"
#include "../src/core/reset.h"
#include "check.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"

static Config cfg;
static Body& b = g_body;

static void setup(uint8_t config_state = CFG_MIGRATED) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)config_state;
  cfg.writing = 0;
  cfg.newest = 0xFF;
  params_load(cfg.img);
  body_init(b, &cfg, nullptr, true);
  g_lh.reset();
  link_init(0xC0FFEE01u, KILO_HELLO_CAPS_DMP);
}

// Feed wire bytes, run the RX side, flush the event queue; return what was sent.
static std::vector<TxFrame> rx(const std::vector<uint8_t>& wire) {
  g_lh.feed(wire);
  link_poll();
  link_send_events();
  return g_lh.take();
}

static std::vector<TxFrame> rx(uint8_t type, uint8_t seq, const std::vector<uint8_t>& payload) {
  return rx(frame(type, seq, payload));
}

static bool is_ack(const std::vector<TxFrame>& out, uint8_t seq, uint8_t type, uint8_t status) {
  return out.size() == 1 && out[0].type == KILO_MSG_ACK && out[0].payload.size() == 3 &&
         out[0].payload[0] == seq && out[0].payload[1] == type && out[0].payload[2] == status;
}

static int16_t i16(const std::vector<uint8_t>& p, size_t off) { return (int16_t)(p[off] | (p[off + 1] << 8)); }

static bool is_refused(const std::vector<TxFrame>& out, uint8_t type, uint8_t reason) {
  return out.size() == 1 && out[0].type == KILO_MSG_EVENT && out[0].payload[0] == KILO_EVENT_GOAL_REFUSED &&
         (uint16_t)i16(out[0].payload, 1) == (uint16_t)((type << 8) | reason);
}

// ---- frames.json, read line by line (gen.py writes one key per line) ----
struct Vec {
  std::string name, dir, wire, error;
  int type = -1;
};

static std::string str_val(const std::string& line) {
  size_t a = line.find(':');
  a = line.find('"', a);
  size_t z = line.find('"', a + 1);
  return line.substr(a + 1, z - a - 1);
}

static void load_vectors(std::vector<Vec>& frames, std::vector<Vec>& errors) {
  std::ifstream f(std::string(REPO_ROOT) + "/protocol/vectors/frames.json");
  std::string line;
  std::vector<Vec>* cur = nullptr;
  while (std::getline(f, line)) {
    if (line.find("\"frames\": [") != std::string::npos) cur = &frames;
    if (line.find("\"errors\": [") != std::string::npos) cur = &errors;
    if (line.find("\"streams\": [") != std::string::npos) cur = nullptr;
    if (!cur) continue;
    // Only top-level keys of each entry (one space of indent deeper than the array).
    if (line.rfind("   \"", 0) != 0 || line.rfind("    ", 0) == 0) continue;
    if (line.find("\"name\":") != std::string::npos) {
      cur->push_back(Vec());
      cur->back().name = str_val(line);
    } else if (cur->empty()) {
      continue;
    } else if (line.find("\"dir\":") != std::string::npos) {
      cur->back().dir = str_val(line);
    } else if (line.find("\"wire\":") != std::string::npos) {
      cur->back().wire = str_val(line);
    } else if (line.find("\"error\":") != std::string::npos) {
      cur->back().error = str_val(line);
    } else if (line.find("\"type\":") != std::string::npos) {
      cur->back().type = std::stoi(line.substr(line.find(':') + 1));
    }
  }
}

// Expected result of each Pi->body vector against a fresh body (epoch 0,
// config MIGRATED, battery OK): reply type, or ACK + status, or EVENT
// GOAL_REFUSED + reason, or nothing.
enum Kind { NOTHING, REPLY, ACK, REFUSED };
struct Expect {
  Kind kind;
  uint8_t v;
};
static const std::map<std::string, Expect> EXPECT = {
    {"doc_hello_req_seq0", {REPLY, KILO_MSG_HELLO}},
    {"doc_set_posture_stand_seq1", {NOTHING, 0}},
    {"hello_req_seq7", {REPLY, KILO_MSG_HELLO}},
    {"heartbeat", {NOTHING, 0}},
    {"heartbeat_seq255", {NOTHING, 0}},
    {"ping", {REPLY, KILO_MSG_PONG}},
    {"ping_zero_token", {REPLY, KILO_MSG_PONG}},
    {"set_stream_default", {ACK, KILO_ACK_STATUS_OK}},
    {"stats_req", {REPLY, KILO_MSG_STATS}},
    {"pca_read", {REPLY, KILO_MSG_PCA_COUNTS}},
    {"set_posture_rest_epoch255", {NOTHING, 0}},  // REST is never stale
    {"set_gait_trot", {REFUSED, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE}},
    {"set_gait_stop", {REFUSED, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE}},
    {"set_head_hold", {REFUSED, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE}},
    {"set_body_pose", {REFUSED, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE}},
    {"set_balance", {NOTHING, 0}},
    {"trick_begin", {ACK, KILO_ACK_STATUS_OK}},
    {"trick_frames_1", {ACK, KILO_ACK_STATUS_NO_TRICK}},  // no TRICK_BEGIN yet
    {"trick_frames_4_max", {ACK, KILO_ACK_STATUS_NO_TRICK}},
    {"trick_commit", {ACK, KILO_ACK_STATUS_NO_TRICK}},
    {"trick_play_mirror", {ACK, KILO_ACK_STATUS_STALE}},  // epoch 3, body at 0
    {"trick_stop", {ACK, KILO_ACK_STATUS_OK}},  // nothing playing is not an error
    {"set_eyes_left_green", {REFUSED, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE}},
    {"beep_1", {NOTHING, 0}},
    {"beep_8_max", {NOTHING, 0}},
    {"cal_get", {REPLY, KILO_MSG_CAL_VALUES}},
    {"cal_set", {ACK, KILO_ACK_STATUS_OK}},
    {"cal_save", {ACK, KILO_ACK_STATUS_OK}},
    {"cal_revert", {ACK, KILO_ACK_STATUS_OK}},
    {"imu_offsets_set", {ACK, KILO_ACK_STATUS_OK}},
    {"param_get", {REPLY, KILO_MSG_PARAM_VALUE}},
    {"param_set_persist", {ACK, KILO_ACK_STATUS_BAD_ARG}},  // -300 is below SLEW_MAX_A16's min
    {"eeprom_read", {REPLY, KILO_MSG_EEPROM_DATA}},
    {"reboot", {ACK, KILO_ACK_STATUS_OK}},
};

static void test_vectors(void) {
  std::vector<Vec> frames, errors;
  load_vectors(frames, errors);
  CHECKF(frames.size() >= 50 && errors.size() >= 15, "frames.json: %zu frames, %zu errors", frames.size(),
         errors.size());
  bool seen[256] = {};
  for (const Vec& v : frames) {
    setup();
    std::vector<uint8_t> w = unhex(v.wire);
    std::vector<TxFrame> out = rx(w);
    if (v.dir != "pi_to_body") {
      CHECKF(out.empty(), "%s: a body->Pi frame must be ignored", v.name.c_str());
      continue;
    }
    auto it = EXPECT.find(v.name);
    if (it == EXPECT.end()) {
      CHECKF(false, "%s: no expectation (add it to EXPECT)", v.name.c_str());
      continue;
    }
    // The vector's own type and seq, recovered with the codec.
    kilo_rx_t r;
    kilo_rx_init(&r);
    kilo_frame_t f = {};
    for (uint8_t c : w) kilo_rx_push(&r, c, &f);
    uint8_t type = f.type;
    seen[type] = true;
    const Expect& e = it->second;
    switch (e.kind) {
      case NOTHING:
        CHECKF(out.empty(), "%s: expected no reply, got %zu frames", v.name.c_str(), out.size());
        break;
      case REPLY:
        CHECKF(out.size() == 1 && out[0].type == e.v, "%s: expected reply 0x%02X", v.name.c_str(), e.v);
        break;
      case ACK:
        CHECKF(is_ack(out, f.seq, type, e.v), "%s: expected ACK status %d", v.name.c_str(), e.v);
        break;
      case REFUSED:
        CHECKF(is_refused(out, type, e.v), "%s: expected GOAL_REFUSED reason %d", v.name.c_str(), e.v);
        break;
    }
  }
  // Every Pi->body message of the spec was exercised.
  for (int t = 1; t < 0x80; t++)
    if (kilo_msg_kind((uint8_t)t) != KILO_KIND_UNKNOWN) CHECKF(seen[t], "no vector exercised type 0x%02X", t);

  // Error vectors: untrusted headers get nothing (and count), trusted ones
  // get ACK BAD_LEN / UNKNOWN_TYPE (commands) or GOAL_REFUSED (goals).
  for (const Vec& v : errors) {
    setup();
    std::vector<TxFrame> out = rx(unhex(v.wire));
    const LinkCounters& c = link_counters();
    if (v.type < 0) {
      // (As a byte stream, "cobs_empty" is just a delimiter, "cobs_zero_inside"
      // two bad segments and "encoded_too_long" an overflow.)
      bool counted = v.name == "cobs_empty" || c.rx_crc_err + c.rx_overflow >= 1;
      CHECKF(out.empty() && counted, "%s: untrusted frame must be dropped and counted", v.name.c_str());
      CHECKF(!b.wd.armed, "%s: an untrusted frame must not feed the watchdog", v.name.c_str());
      continue;
    }
    CHECKF(b.wd.armed, "%s: a CRC-valid frame feeds the watchdog", v.name.c_str());
    uint8_t t = (uint8_t)v.type;
    if (v.error == "unknown_type") {
      CHECKF(out.size() == 1 && out[0].type == KILO_MSG_ACK && out[0].payload[2] == KILO_ACK_STATUS_UNKNOWN_TYPE,
             "%s: expected ACK UNKNOWN_TYPE", v.name.c_str());
    } else if (t & 0x80) {
      CHECKF(out.empty(), "%s: body->Pi type ignored", v.name.c_str());
    } else if (kilo_msg_is_goal(t)) {
      CHECKF(is_refused(out, t, KILO_GOAL_REFUSED_REASON_BAD_LEN), "%s: expected GOAL_REFUSED BAD_LEN",
             v.name.c_str());
    } else {
      CHECKF(out.size() == 1 && out[0].type == KILO_MSG_ACK && out[0].payload[2] == KILO_ACK_STATUS_BAD_LEN,
             "%s: expected ACK BAD_LEN", v.name.c_str());
    }
  }
}

static void test_hello_and_queries(void) {
  setup(CFG_MIGRATED);
  b.epoch = 7;
  link_boot(RESET_FLASHED, 1234);
  std::vector<TxFrame> out = g_lh.take();
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_HELLO && out[0].payload.size() == KILO_HELLO_LEN);
  kilo_hello_t h;
  std::memcpy(&h, out[0].payload.data(), sizeof h);
  CHECK(h.proto_major == KILO_PROTO_MAJOR && h.proto_minor == KILO_PROTO_MINOR && h.fw_build == 0xC0FFEE01u);
  CHECK(h.reset_cause == RESET_FLASHED && h.last_alive_tick == 1234 && h.caps == KILO_HELLO_CAPS_DMP);
  CHECK(h.max_payload == 60 && h.trick_capacity == 32 && h.joint_count == 11);
  CHECK(h.config_state == CFG_MIGRATED && h.epoch == 7);
  // PING: PONG echoes the token, never an ACK; a repeat (same seq) is answered again.
  for (int i = 0; i < 2; i++) {
    out = rx(KILO_MSG_PING, 9, {0xEF, 0xBE});
    CHECK(out.size() == 1 && out[0].type == KILO_MSG_PONG && (uint16_t)i16(out[0].payload, 0) == 0xBEEF);
  }
  // PARAM_GET: value and bounds from the spec; an unknown id is ACK BAD_ARG.
  out = rx(KILO_MSG_PARAM_GET, 10, {KILO_PARAM_WD_STOP_MS});
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_PARAM_VALUE && out[0].payload[0] == KILO_PARAM_WD_STOP_MS &&
        i16(out[0].payload, 1) == 500 && i16(out[0].payload, 3) == 100 && i16(out[0].payload, 5) == 5000 &&
        i16(out[0].payload, 7) == 500);
  CHECK(is_ack(rx(KILO_MSG_PARAM_GET, 11, {KILO_PARAM_COUNT}), 11, KILO_MSG_PARAM_GET, KILO_ACK_STATUS_BAD_ARG));
  // EEPROM_READ: data from the EEPROM; out-of-range is BAD_ARG.
  g_ee.mem[1000] = 0xA5;
  g_ee.mem[1023] = 0x5A;
  out = rx(KILO_MSG_EEPROM_READ, 12, {0xE8, 0x03, 24});  // 1000, 24 B: exactly to the end
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_EEPROM_DATA && out[0].payload.size() == 26 &&
        out[0].payload[2] == 0xA5 && out[0].payload[25] == 0x5A);
  CHECK(is_ack(rx(KILO_MSG_EEPROM_READ, 13, {0xE8, 0x03, 25}), 13, KILO_MSG_EEPROM_READ, KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_EEPROM_READ, 14, {0, 0, 57}), 14, KILO_MSG_EEPROM_READ, KILO_ACK_STATUS_BAD_ARG));
  // PCA_READ: counts as read; an I2C error is ACK BUSY.
  for (int c = 0; c < 16; c++) g_lh.pca[c] = (uint16_t)(c < 8 ? 0x1000 : 300 + c);
  out = rx(KILO_MSG_PCA_READ, 15, {});
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_PCA_COUNTS && i16(out[0].payload, 0) == 0x1000 &&
        i16(out[0].payload, 30) == 315);
  g_lh.pca_ok = false;
  CHECK(is_ack(rx(KILO_MSG_PCA_READ, 16, {}), 16, KILO_MSG_PCA_READ, KILO_ACK_STATUS_BUSY));
  // STATS: hardware fields from the HAL, the link's own counters added.
  g_lh.stats.max_tick_us = 4321;
  g_lh.stats.rx_overflow = 2;
  rx({0x05, 0x01, 0x00});  // garbage: one bad frame
  out = rx(KILO_MSG_STATS_REQ, 17, {});
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_STATS);
  kilo_stats_t s;
  std::memcpy(&s, out[0].payload.data(), sizeof s);
  CHECK(s.max_tick_us == 4321 && s.rx_overflow == 2 && s.rx_crc_err == 1);
}

static void test_dedup(void) {
  setup();
  // CAL_SAVE twice with the same seq: executed once, acked twice with the
  // cached status (a real second save would be EEPROM_BUSY).
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 40, {}), 40, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_OK));
  CHECK(config_busy(cfg));
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 40, {}), 40, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_OK));
  // A new seq executes: the save is still running.
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 41, {}), 41, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_EEPROM_BUSY));
  // ...and its BUSY is what a retry of 41 gets back.
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 41, {}), 41, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_EEPROM_BUSY));
  // Same seq, other type: not a duplicate.
  CHECK(is_ack(rx(KILO_MSG_SET_STREAM, 41, {2, 0, 0}), 41, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_OK));
  // The cache expires after 500 ms: a new session reusing the seq executes.
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 50, {8, 16, 0}), 50, KILO_MSG_CAL_SET, KILO_ACK_STATUS_EEPROM_BUSY));
  while (config_busy(cfg)) config_writer_step(cfg);
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 50, {8, 16, 0}), 50, KILO_MSG_CAL_SET, KILO_ACK_STATUS_EEPROM_BUSY));
  b.tick = (uint16_t)(b.tick + KILO_DEDUP_WINDOW_MS / 20);
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 50, {8, 16, 0}), 50, KILO_MSG_CAL_SET, KILO_ACK_STATUS_OK));
  CHECK(config_calib_a16(cfg.img, 8) == 16);
}

static void test_goals(void) {
  setup();
  // STAND with the current epoch: taken. A stale epoch: GOAL_REFUSED STALE,
  // once per second per type.
  CHECK(rx(KILO_MSG_SET_POSTURE, 1, {0, POSTURE_STAND, 0, 1}).empty());
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_STAND && b.balance_req == 1);
  // SET_BALANCE: no epoch, never acked; sets enable and the gains live.
  CHECK(rx(KILO_MSG_SET_BALANCE, 1, {1, 200, 40, 3, 12}).empty());
  CHECK(b.bal.enable == 1 && b.bal.kp_q8 == 200 && b.bal.kd_ms == 40 && b.bal.db_deg == 3 && b.bal.max_deg == 12);
  CHECK(is_refused(rx(KILO_MSG_SET_BALANCE, 1, {2, 128, 30, 2, 15}), KILO_MSG_SET_BALANCE,
                   KILO_GOAL_REFUSED_REASON_BAD_ARG));
  CHECK(rx(KILO_MSG_SET_BALANCE, 1, {0, 128, 30, 2, 31}).empty());  // BAD_ARG again, rate-limited
  CHECK(b.bal.enable == 1 && b.bal.max_deg == 12);                  // unchanged
  b.epoch = 3;
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 2, {2, POSTURE_SIT, 0, 0}), KILO_MSG_SET_POSTURE,
                   KILO_GOAL_REFUSED_REASON_STALE));
  CHECK(rx(KILO_MSG_SET_POSTURE, 3, {2, POSTURE_SIT, 0, 0}).empty());  // rate-limited
  for (int i = 0; i < 50; i++) link_tick();
  g_lh.take();
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 4, {2, POSTURE_SIT, 0, 0}), KILO_MSG_SET_POSTURE,
                   KILO_GOAL_REFUSED_REASON_STALE));
  // REST and LIMP are never stale. Bad posture codes are BAD_ARG.
  CHECK(rx(KILO_MSG_SET_POSTURE, 5, {99, POSTURE_REST, 0, 0}).empty());
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_REST);
  for (int i = 0; i < 50; i++) link_tick();  // next rate-limit window
  g_lh.take();
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 6, {3, 9, 0, 0}), KILO_MSG_SET_POSTURE, KILO_GOAL_REFUSED_REASON_BAD_ARG));
  // Low battery: motion postures refused, REST still fine.
  setup();
  b.batt.level = BATT_LOW;
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 7, {0, POSTURE_SIT, 0, 0}), KILO_MSG_SET_POSTURE,
                   KILO_GOAL_REFUSED_REASON_LOW_BATT));
  CHECK(rx(KILO_MSG_SET_POSTURE, 8, {0, POSTURE_REST, 0, 0}).empty());
  // No calibration (config DEFAULTED): STAND refused UNCALIBRATED, SIT fine;
  // after a CAL_SAVE completes, STAND is allowed.
  setup(CFG_DEFAULTED);
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 9, {0, POSTURE_STAND, 0, 1}), KILO_MSG_SET_POSTURE,
                   KILO_GOAL_REFUSED_REASON_UNCALIBRATED));
  CHECK(rx(KILO_MSG_SET_POSTURE, 10, {0, POSTURE_SIT, 0, 0}).empty());
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 11, {}), 11, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_OK));
  while (config_busy(cfg)) config_writer_step(cfg);
  CHECK(cfg.state == CFG_OK);
  CHECK(rx(KILO_MSG_SET_POSTURE, 12, {0, POSTURE_STAND, 0, 0}).empty());
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_STAND);
  // BEEP queues its notes; goals never ACK. Unimplemented goals: UNKNOWN_TYPE.
  setup();
  CHECK(rx(KILO_MSG_BEEP, 13, {2, 12, 10, 0, 5}).empty());
  CHECK(b.notes.count == 2);
  CHECK(is_refused(rx(KILO_MSG_SET_EYES, 14, {0x3F, 1, 2, 3}), KILO_MSG_SET_EYES,
                   KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE));
  // Wrong length on a goal: GOAL_REFUSED BAD_LEN (not an ACK).
  CHECK(is_refused(rx(KILO_MSG_SET_POSTURE, 15, {0, 3, 0}), KILO_MSG_SET_POSTURE, KILO_GOAL_REFUSED_REASON_BAD_LEN));
}

static void test_commands(void) {
  setup();
  // CAL_SET / CAL_GET round trip, range checks, CAL_REVERT.
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 1, {12, 0xC0, 0xFF}), 1, KILO_MSG_CAL_SET, KILO_ACK_STATUS_OK));  // -64
  std::vector<TxFrame> out = rx(KILO_MSG_CAL_GET, 2, {});
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_CAL_VALUES && i16(out[0].payload, 2 * 12) == -64);
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 3, {16, 0, 0}), 3, KILO_MSG_CAL_SET, KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 4, {1, 0x91, 0x01}), 4, KILO_MSG_CAL_SET, KILO_ACK_STATUS_BAD_ARG));  // 401
  CHECK(is_ack(rx(KILO_MSG_CAL_REVERT, 5, {}), 5, KILO_MSG_CAL_REVERT, KILO_ACK_STATUS_OK));
  CHECK(config_calib_a16(cfg.img, 12) == 0);
  // CAL_SAVE: ACK when queued, EVENT EEPROM_SAVED is the app's job (main.cpp).
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 6, {12, 0x20, 0}), 6, KILO_MSG_CAL_SET, KILO_ACK_STATUS_OK));
  CHECK(is_ack(rx(KILO_MSG_CAL_SAVE, 7, {}), 7, KILO_MSG_CAL_SAVE, KILO_ACK_STATUS_OK));
  CHECK(is_ack(rx(KILO_MSG_CAL_REVERT, 8, {}), 8, KILO_MSG_CAL_REVERT, KILO_ACK_STATUS_EEPROM_BUSY));
  while (config_busy(cfg)) config_writer_step(cfg);
  CHECK(is_ack(rx(KILO_MSG_CAL_SET, 9, {12, 0, 0}), 9, KILO_MSG_CAL_SET, KILO_ACK_STATUS_OK));
  CHECK(is_ack(rx(KILO_MSG_CAL_REVERT, 10, {}), 10, KILO_MSG_CAL_REVERT, KILO_ACK_STATUS_OK));
  CHECK(config_calib_a16(cfg.img, 12) == 32);  // back to the saved value
  // PARAM_SET: live, bounds, persist.
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 11, {KILO_PARAM_WD_STOP_MS, 0x58, 0x02, 0}), 11, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_OK));  // 600
  CHECK(PARAM(WD_STOP_MS) == 600 && WD_STOP_TICKS == 30 && cfg.img.params[KILO_PARAM_WD_STOP_MS] == CFG_PARAM_DEFAULT);
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 12, {KILO_PARAM_WD_STOP_MS, 99, 0, 0}), 12, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 13, {KILO_PARAM_COUNT, 0, 0, 0}), 13, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 14, {KILO_PARAM_TELEM_DIV, 5, 0, 2}), 14, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 15, {KILO_PARAM_TELEM_DIV, 5, 0, 1}), 15, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_OK));
  CHECK(PARAM(TELEM_DIV) == 5 && cfg.img.params[KILO_PARAM_TELEM_DIV] == 5 && config_busy(cfg));
  CHECK(is_ack(rx(KILO_MSG_PARAM_SET, 16, {KILO_PARAM_TELEM_DIV, 6, 0, 1}), 16, KILO_MSG_PARAM_SET,
               KILO_ACK_STATUS_EEPROM_BUSY));
  CHECK(PARAM(TELEM_DIV) == 5);
  while (config_busy(cfg)) config_writer_step(cfg);
  // The persisted value survives a reboot (config_boot + params_load).
  Config c2;
  config_boot(c2);
  params_load(c2.img);
  CHECK(PARAM(TELEM_DIV) == 5 && PARAM(WD_STOP_MS) == KILO_PARAM_WD_STOP_MS_DEFAULT);
  // IMU_OFFSETS_SET: written to the MPU and the image; save=1 persists.
  setup();
  CHECK(is_ack(rx(KILO_MSG_IMU_OFFSETS_SET, 17, {1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 2}), 17,
               KILO_MSG_IMU_OFFSETS_SET, KILO_ACK_STATUS_BAD_ARG));
  CHECK(is_ack(rx(KILO_MSG_IMU_OFFSETS_SET, 18, {1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 0xFA, 0xFF, 1}), 18,
               KILO_MSG_IMU_OFFSETS_SET, KILO_ACK_STATUS_OK));
  CHECK(g_lh.offset_writes == 1 && g_lh.offsets[5] == -6 && cfg.img.imu_offset[0] == 1 && config_busy(cfg));
  // SET_STREAM: JOINTS and IMU_RAW every tick, no TELEMETRY.
  CHECK(is_ack(rx(KILO_MSG_SET_STREAM, 19, {0, 1, 1}), 19, KILO_MSG_SET_STREAM, KILO_ACK_STATUS_OK));
  b.tick = 10;
  link_tick();
  std::vector<TxFrame> s = g_lh.take();
  CHECK(s.size() == 2 && s[0].type == KILO_MSG_JOINTS && s[1].type == KILO_MSG_IMU_RAW);
  CHECK(s.size() == 2 && i16(s[0].payload, 2) == JOINT_OFF);  // cold boot: limp
  // REBOOT: wrong magic BAD_ARG; right magic ACK OK, then the reboot.
  CHECK(is_ack(rx(KILO_MSG_REBOOT, 20, {0x49, 0x4C}), 20, KILO_MSG_REBOOT, KILO_ACK_STATUS_BAD_ARG));
  CHECK(g_lh.reboots == 0);
  CHECK(is_ack(rx(KILO_MSG_REBOOT, 21, {0x49, 0x4B}), 21, KILO_MSG_REBOOT, KILO_ACK_STATUS_OK));
  CHECK(g_lh.reboots == 1);
}

static void test_streams_and_priority(void) {
  setup();
  // Default: TELEMETRY on even ticks (TELEM_DIV 2), nothing else.
  b.tick = 41;
  link_tick();
  CHECK(g_lh.take().empty());
  b.tick = 42;
  b.epoch = 9;
  g_lh.sample.dist_mm = 345;
  g_lh.sample.dist_fresh = 1;
  g_lh.sample.tick_load = 47;
  link_tick();
  std::vector<TxFrame> out = g_lh.take();
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_TELEMETRY && out[0].payload.size() == KILO_TELEMETRY_LEN);
  kilo_telemetry_t t;
  std::memcpy(&t, out[0].payload.data(), sizeof t);
  CHECK(t.tick == 42 && t.epoch == 9 && t.dist_mm == 345 && t.tick_load == 47 && t.mode == KILO_MODE_LIMP);
  CHECK((t.flags & KILO_TELEMETRY_FLAGS_DIST_FRESH) && !(t.flags & KILO_TELEMETRY_FLAGS_SERVOS_POWERED));
  CHECK(!(t.flags & KILO_TELEMETRY_FLAGS_LINK_OK) && t.trick_frame == 0xFF);
  // The first valid frame arms the watchdog; LINK_OK follows.
  rx(KILO_MSG_HEARTBEAT, 0, {});
  b.tick = 44;
  link_tick();
  out = g_lh.take();
  std::memcpy(&t, out[0].payload.data(), sizeof t);
  CHECK(b.wd.armed && (t.flags & KILO_TELEMETRY_FLAGS_LINK_OK) && t.last_rx_seq == 0);
  CHECK(!(t.flags & KILO_TELEMETRY_FLAGS_BALANCE_ACTIVE));
  // Flags bit 0 reports the balance layer running this tick.
  b.bal.active = 1;
  b.tick = 46;
  link_tick();
  out = g_lh.take();
  std::memcpy(&t, out[0].payload.data(), sizeof t);
  CHECK(t.flags & KILO_TELEMETRY_FLAGS_BALANCE_ACTIVE);
  b.bal.active = 0;
  // TX priority: with no room for TELEMETRY, the events still go (they are
  // never dropped, only delayed), the stream frame is dropped and counted,
  // and a TX_DROPPED event follows within a second.
  g_lh.drain = 0;
  g_lh.ring_used = g_lh.cap - 2 * (KILO_EVENT_LEN + KILO_FRAME_OVERHEAD);  // room for exactly two events
  events_push(b.ev, EV_IMU_FAULT, 0, b.tick, b.epoch);
  events_push(b.ev, EV_IMU_OK, 0, b.tick, b.epoch);
  events_push(b.ev, EV_BATTERY_OK, 0, b.tick, b.epoch);
  b.tick = 46;
  link_tick();
  out = g_lh.take();
  CHECK(out.size() == 2 && out[0].type == KILO_MSG_EVENT && out[1].type == KILO_MSG_EVENT);
  CHECK(out.size() == 2 && out[0].payload[0] == KILO_EVENT_IMU_FAULT && out[1].payload[0] == KILO_EVENT_IMU_OK);
  CHECK(link_counters().tx_dropped == 1 && b.ev.count == 1);
  g_lh.drain = 1000;
  for (int i = 0; i < 50; i++) {
    b.tick++;
    link_tick();
  }
  out = g_lh.take();
  int ev_ok = 0, ev_drop = 0;
  for (const TxFrame& f : out)
    if (f.type == KILO_MSG_EVENT) {
      if (f.payload[0] == KILO_EVENT_BATTERY_OK) ev_ok++;
      if (f.payload[0] == KILO_EVENT_TX_DROPPED && i16(f.payload, 1) == 1) ev_drop++;
    }
  CHECK(ev_ok == 1 && ev_drop == 1);
  // Watchdog: silence for WD_STOP -> EVENT WATCHDOG with the bumped epoch;
  // the next frame -> LINK_RESTORED.
  setup();
  rx(KILO_MSG_HEARTBEAT, 1, {});
  uint16_t cnt[16];
  TickIn in = TickIn();
  in.imu = IMU_FRESH;
  in.att.gz = 16384;
  for (int i = 0; i < 26; i++) body_tick(b, in, cnt);
  link_send_events();
  out = g_lh.take();
  CHECK(out.size() == 1 && out[0].type == KILO_MSG_EVENT && out[0].payload[0] == KILO_EVENT_WATCHDOG &&
        out[0].payload[5] == 1);
  out = rx(KILO_MSG_HEARTBEAT, 2, {});
  CHECK(out.size() == 1 && out[0].payload[0] == KILO_EVENT_LINK_RESTORED);
}

int main() {
  test_vectors();
  test_hello_and_queries();
  test_dedup();
  test_goals();
  test_commands();
  test_streams_and_priority();
  return report("link");
}
