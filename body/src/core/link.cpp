#include "link.h"
#include <string.h>
#include "../proto/kilo_frame.h"
#include "body.h"
#include "config.h"
#include "params.h"
#include "pgm.h"
#include "posture.h"
#include "reset.h"

// Enum values of the body and of the spec are different C++ enum types.
#define SAME(a, b) ((int16_t)(a) == (int16_t)(b))

// The body's own lists must match the spec (protocol/spec.toml), not a copy.
static_assert(SAME(EV_BOOT, KILO_EVENT_BOOT) && SAME(EV_FELL, KILO_EVENT_FELL) &&
                  SAME(EV_RECOVERED, KILO_EVENT_RECOVERED) && SAME(EV_RECOVERY_FAILED, KILO_EVENT_RECOVERY_FAILED) &&
                  SAME(EV_LOW_BATTERY, KILO_EVENT_LOW_BATTERY) && SAME(EV_BATTERY_OK, KILO_EVENT_BATTERY_OK) &&
                  SAME(EV_WATCHDOG, KILO_EVENT_WATCHDOG) && SAME(EV_LINK_RESTORED, KILO_EVENT_LINK_RESTORED) &&
                  SAME(EV_IMU_FAULT, KILO_EVENT_IMU_FAULT) && SAME(EV_IMU_OK, KILO_EVENT_IMU_OK) &&
                  SAME(EV_TRICK_DONE, KILO_EVENT_TRICK_DONE) && SAME(EV_GAIT_STOPPED, KILO_EVENT_GAIT_STOPPED) &&
                  SAME(EV_GOAL_REFUSED, KILO_EVENT_GOAL_REFUSED) && SAME(EV_CONFIG_MIGRATED, KILO_EVENT_CONFIG_MIGRATED) &&
                  SAME(EV_CONFIG_DEFAULTED, KILO_EVENT_CONFIG_DEFAULTED) && SAME(EV_EEPROM_SAVED, KILO_EVENT_EEPROM_SAVED) &&
                  SAME(EV_TX_DROPPED, KILO_EVENT_TX_DROPPED),
              "core/events.h codes != spec");
static_assert(SAME(EVENT_QUEUE_LEN, KILO_EVENT_QUEUE_DEPTH), "event queue depth != spec");
static_assert(SAME(GOAL_BAD_ARG, KILO_GOAL_REFUSED_REASON_BAD_ARG) && SAME(GOAL_LOW_BATT, KILO_GOAL_REFUSED_REASON_LOW_BATT) &&
                  SAME(GOAL_STALE, KILO_GOAL_REFUSED_REASON_STALE) &&
                  SAME(GOAL_UNCALIBRATED, KILO_GOAL_REFUSED_REASON_UNCALIBRATED) &&
                  SAME(KILO_GOAL_REFUSED_REASON_UNCALIBRATED, KILO_ACK_STATUS_UNCALIBRATED),
              "GoalResult != GOAL_REFUSED reasons");
static_assert(SAME(POSTURE_LIMP, KILO_POSTURE_LIMP) && SAME(POSTURE_REST, KILO_POSTURE_REST) &&
                  SAME(POSTURE_SIT, KILO_POSTURE_SIT) && SAME(POSTURE_STAND, KILO_POSTURE_STAND) &&
                  SAME(POSTURE_STANCE, KILO_POSTURE_STANCE) && SAME(POSTURE_CALIB, KILO_POSTURE_CALIB) &&
                  SAME(POSTURE_ZERO, KILO_POSTURE_ZERO),
              "core/posture.h ids != spec");
static_assert(SAME(RESET_COLD, KILO_RESET_CAUSE_COLD) && SAME(RESET_FLASHED, KILO_RESET_CAUSE_FLASHED) &&
                  SAME(RESET_REQUESTED, KILO_RESET_CAUSE_REQUESTED) &&
                  SAME(RESET_WARM_UNEXPECTED, KILO_RESET_CAUSE_WARM_UNEXPECTED),
              "core/reset.h causes != spec");
static_assert(SAME(CFG_OK, KILO_CONFIG_STATE_OK) && SAME(CFG_MIGRATED, KILO_CONFIG_STATE_MIGRATED) &&
                  SAME(CFG_DEFAULTED, KILO_CONFIG_STATE_DEFAULTED),
              "config states != spec");
static_assert(SAME(OWN_IDLE, KILO_OWNER_IDLE) && SAME(OWN_GAIT, KILO_OWNER_GAIT) && SAME(OWN_TRICK, KILO_OWNER_TRICK) &&
                  SAME(OWN_REFLEX, KILO_OWNER_REFLEX),
              "owners != spec");
static_assert(SAME(DRIVEN_COUNT, KILO_JOINT_COUNT) && SAME(JOINT_COUNT, KILO_CHANNEL_COUNT), "joint counts != spec");
static_assert(SAME(PCA_FULL_OFF, KILO_PCA_FULL_OFF_BIT), "full-off bit != spec");
static_assert(SAME(NOTES_LEN, KILO_BEEP_MAX_NOTES), "BEEP queue != spec");

#define DEDUP_TICKS (KILO_DEDUP_WINDOW_MS / 20u)  // 25 ticks
#define TICKS_PER_S 50u

// The one body (app/main.cpp; the host tests define their own). A global
// rather than a pointer: on the AVR every access is then a direct load.
extern Body g_body;
#define B (&g_body)
static uint8_t wire[KILO_MAX_WIRE];
// Everything else in one struct, so link_init() can reset it in one go.
static struct {
  kilo_rx_t rx;
  uint8_t tx_seq, last_rx_seq;
  uint8_t div_telem, div_joints, div_imu;  // SET_STREAM divisors, 0 = off
  // De-dup cache: the last executed non-query command.
  uint8_t dd_seq, dd_type, dd_status;
  uint16_t dd_tick;
  uint8_t refused_mask;  // GOAL_REFUSED sent this second, one bit per goal type
  uint8_t sec_ticks;
  uint16_t drops_this_s;
  uint16_t pkt_prev;
  uint8_t dmp_rate;
  uint32_t fw_build;
  uint16_t caps;
  uint8_t reset_cause;
  uint16_t last_alive;
  LinkCounters cnt;
} L;

const LinkCounters& link_counters(void) { return L.cnt; }

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// Frame and queue one message. Every frame is exactly len + 6 wire bytes
// (len <= 60, so COBS adds exactly one code byte). `must`: replies and ACKs
// wait for room (the UART drains the 96 B ring in < 4 ms); streams are
// dropped and counted instead.
KILO_NOINLINE static bool emit(uint8_t type, const void* payload, uint8_t len, bool must) {
  uint8_t n = (uint8_t)(len + KILO_FRAME_OVERHEAD);
  if (link_hal_tx_free() < n) {
    if (!must) {
      L.cnt.tx_dropped++;
      L.drops_this_s++;
      return false;
    }
    while (link_hal_tx_free() < n) {
    }
  }
  kilo_frame_encode(type, L.tx_seq++, (const uint8_t*)payload, len, wire);
  link_hal_write(wire, n);
  return true;
}

KILO_NOINLINE static void ack(uint8_t seq, uint8_t type, uint8_t status) {
  kilo_ack_t a;
  a.seq = seq;
  a.type = type;
  a.status = status;
  emit(KILO_MSG_ACK, &a, sizeof a, true);
}

static void send_hello(void) {
  kilo_hello_t h;
  h.proto_major = KILO_PROTO_MAJOR;
  h.proto_minor = KILO_PROTO_MINOR;
  h.fw_build = L.fw_build;
  h.reset_cause = L.reset_cause;
  h.last_alive_tick = L.last_alive;
  h.caps = L.caps;
  h.max_payload = KILO_MAX_PAYLOAD;
  h.trick_capacity = 0;  // no keyframe store until P1
  h.joint_count = DRIVEN_COUNT;
  h.config_state = B->cfg->state;
  h.epoch = B->epoch;
  emit(KILO_MSG_HELLO, &h, sizeof h, true);
}

// One bit per goal type for the 1/s GOAL_REFUSED rate limit.
static uint8_t goal_bit(uint8_t t) {
  if (t >= KILO_MSG_SET_EYES) return (uint8_t)(0x20u << (t & 1u));  // SET_EYES, BEEP
  if (t == KILO_MSG_HEARTBEAT) return 0x80u;
  return (uint8_t)(1u << (t & 7u));  // SET_POSTURE .. SET_BALANCE
}

static void refuse(uint8_t type, uint8_t reason) {
  uint8_t bit = goal_bit(type);
  if (L.refused_mask & bit) return;
  L.refused_mask |= bit;
  events_push(B->ev, EV_GOAL_REFUSED, (int16_t)(((uint16_t)type << 8) | reason), B->tick, B->epoch);
}

static void goal(uint8_t t, const uint8_t* p) {
  switch (t) {
    case KILO_MSG_HEARTBEAT:
      return;
    case KILO_MSG_SET_POSTURE: {
      uint8_t r = body_goal_posture(*B, p[0], p[1], p[2], p[3]);
      if (r != GOAL_OK) refuse(t, r);
      return;
    }
    case KILO_MSG_BEEP:
      // A full queue drops the rest: BEEP is sent once, not refreshed.
      for (uint8_t i = 0; i < p[0]; i++) notes_push(B->notes, (int8_t)p[1 + 2 * i], p[2 + 2 * i]);
      return;
    default:
      // SET_GAIT, SET_HEAD, SET_BODY_POSE (P2/P1), SET_BALANCE (P1), SET_EYES.
      refuse(t, KILO_GOAL_REFUSED_REASON_UNKNOWN_TYPE);
  }
}

// Queries: send the data reply and return OK, or return the failure status.
static uint8_t query(uint8_t t, const uint8_t* p) {
  union {
    kilo_pong_t pong;
    kilo_stats_t st;
    kilo_pca_counts_t pc;
    kilo_cal_values_t cv;
    kilo_param_value_t pv;
    kilo_eeprom_data_t ed;
  } u;
  uint8_t rt, len;
  switch (t) {
    case KILO_MSG_HELLO_REQ:
      send_hello();
      return KILO_ACK_STATUS_OK;
    case KILO_MSG_PING:
      u.pong.token = rd16(p);
      u.pong.tick = B->tick;
      rt = KILO_MSG_PONG;
      len = sizeof u.pong;
      break;
    case KILO_MSG_STATS_REQ:
      link_hal_stats(&u.st);
      u.st.uptime_s = L.cnt.uptime_s;
      u.st.rx_crc_err = L.cnt.rx_crc_err;
      u.st.rx_overflow = (uint16_t)(u.st.rx_overflow + L.cnt.rx_overflow);
      u.st.imu_stale = B->imu_stale_total;
      u.st.dmp_rate_hz = L.dmp_rate;
      rt = KILO_MSG_STATS;
      len = sizeof u.st;
      break;
    case KILO_MSG_PCA_READ: {
      uint16_t c[KILO_CHANNEL_COUNT];
      if (!link_hal_pca_read(c)) return KILO_ACK_STATUS_BUSY;
      memcpy(u.pc.counts, c, sizeof c);
      rt = KILO_MSG_PCA_COUNTS;
      len = sizeof u.pc;
      break;
    }
    case KILO_MSG_CAL_GET:
      for (uint8_t j = 0; j < KILO_CHANNEL_COUNT; j++) u.cv.offset[j] = config_calib_a16(B->cfg->img, j);
      rt = KILO_MSG_CAL_VALUES;
      len = sizeof u.cv;
      break;
    case KILO_MSG_PARAM_GET: {
      uint8_t id = p[0];
      if (id >= KILO_PARAM_COUNT) return KILO_ACK_STATUS_BAD_ARG;
      int16_t info[3];
      param_info(id, info);
      u.pv.id = id;
      u.pv.value = g_param[id];
      u.pv.min = info[0];
      u.pv.max = info[1];
      u.pv.default_ = info[2];
      rt = KILO_MSG_PARAM_VALUE;
      len = sizeof u.pv;
      break;
    }
    default: {  // KILO_MSG_EEPROM_READ (the only other query)
      uint16_t addr = rd16(p);
      uint8_t n = p[2];
      if (n == 0 || n > KILO_EEPROM_READ_MAX || addr > 1024u - n) return KILO_ACK_STATUS_BAD_ARG;
      u.ed.addr = addr;
      // A config write in progress makes each read wait for it (<= 3.4 ms).
      for (uint8_t i = 0; i < n; i++) u.ed.data[i] = ee_read((uint16_t)(addr + i));
      rt = KILO_MSG_EEPROM_DATA;
      len = (uint8_t)(2 + n);
    }
  }
  emit(rt, &u, len, true);
  return KILO_ACK_STATUS_OK;
}

// Non-query commands: execute and return the ACK status. BAD_ARG is checked
// before EEPROM_BUSY.
static uint8_t command(uint8_t t, const uint8_t* p) {
  Config& c = *B->cfg;
  switch (t) {
    case KILO_MSG_SET_STREAM:
      L.div_telem = p[0];
      L.div_joints = p[1];
      L.div_imu = p[2];
      return KILO_ACK_STATUS_OK;
    case KILO_MSG_CAL_SET: {
      int16_t o = (int16_t)rd16(p + 1);
      if (p[0] >= KILO_CHANNEL_COUNT || o < -400 || o > 400) return KILO_ACK_STATUS_BAD_ARG;
      if (config_busy(c)) return KILO_ACK_STATUS_EEPROM_BUSY;
      config_set_calib_a16(c.img, p[0], o);
      return KILO_ACK_STATUS_OK;
    }
    case KILO_MSG_CAL_SAVE:
      return config_save(c) ? KILO_ACK_STATUS_OK : KILO_ACK_STATUS_EEPROM_BUSY;
    case KILO_MSG_CAL_REVERT:
      if (config_busy(c)) return KILO_ACK_STATUS_EEPROM_BUSY;
      config_revert(c);
      return KILO_ACK_STATUS_OK;
    case KILO_MSG_IMU_OFFSETS_SET:
      if (p[12] > 1) return KILO_ACK_STATUS_BAD_ARG;
      if (config_busy(c)) return KILO_ACK_STATUS_EEPROM_BUSY;
      memcpy(c.img.imu_offset, p, sizeof c.img.imu_offset);
      link_hal_imu_offsets(c.img.imu_offset);
      if (p[12]) config_save(c);
      return KILO_ACK_STATUS_OK;
    case KILO_MSG_PARAM_SET: {
      int16_t v = (int16_t)rd16(p + 1);
      if (!param_ok(p[0], v) || p[3] > 1) return KILO_ACK_STATUS_BAD_ARG;
      if (p[3]) {
        if (config_busy(c)) return KILO_ACK_STATUS_EEPROM_BUSY;
        c.img.params[p[0]] = v;
        config_save(c);
      }
      g_param[p[0]] = v;
      return KILO_ACK_STATUS_OK;
    }
    case KILO_MSG_REBOOT:
      return rd16(p) == KILO_REBOOT_MAGIC ? KILO_ACK_STATUS_OK : KILO_ACK_STATUS_BAD_ARG;
    default:  // TRICK_BEGIN .. TRICK_STOP: no keyframe store until P1
      return KILO_ACK_STATUS_UNKNOWN_TYPE;
  }
}

static void on_frame(uint8_t st, const kilo_frame_t& f) {
  body_link_frame(*B);  // any CRC-valid frame feeds the link watchdog
  L.last_rx_seq = f.seq;
  uint8_t t = f.type;
  if (st == KILO_FRAME_UNKNOWN_TYPE) {
    ack(f.seq, t, KILO_ACK_STATUS_UNKNOWN_TYPE);
    return;
  }
  if (t & 0x80u) return;  // a known body->Pi type: an echo, or a confused peer
  bool is_goal = kilo_msg_is_goal(t);
  if (st != KILO_FRAME_OK) {  // KILO_FRAME_BAD_PAYLOAD_LEN
    if (is_goal)
      refuse(t, KILO_GOAL_REFUSED_REASON_BAD_LEN);
    else
      ack(f.seq, t, KILO_ACK_STATUS_BAD_LEN);
    return;
  }
  if (is_goal) {
    goal(t, f.payload);
    return;
  }
  if (kilo_msg_is_query(t)) {
    uint8_t s = query(t, f.payload);
    if (s != KILO_ACK_STATUS_OK) ack(f.seq, t, s);
    return;
  }
  if (t == L.dd_type && f.seq == L.dd_seq && (uint16_t)(B->tick - L.dd_tick) < DEDUP_TICKS) {
    ack(f.seq, t, L.dd_status);  // a retry after a lost ACK: do not execute again
    return;
  }
  uint8_t s = command(t, f.payload);
  L.dd_seq = f.seq;
  L.dd_type = t;
  L.dd_status = s;
  L.dd_tick = B->tick;
  ack(f.seq, t, s);
  if (t == KILO_MSG_REBOOT && s == KILO_ACK_STATUS_OK) link_hal_reboot();
}

void link_init(uint32_t build, uint16_t capabilities) {
  memset(&L, 0, sizeof L);
  L.fw_build = build;
  L.caps = capabilities;
  kilo_rx_init(&L.rx);
  L.div_telem = (uint8_t)PARAM(TELEM_DIV);
  L.div_joints = L.div_imu = 0;
}

void link_boot(uint8_t cause, uint16_t alive) {
  L.reset_cause = cause;
  L.last_alive = alive;
  send_hello();
}

void link_poll(void) {
  uint8_t c;
  kilo_frame_t f;
  while (link_hal_read(&c)) {
    uint8_t st = kilo_rx_push(&L.rx, c, &f);
    if (st == KILO_FRAME_NONE) continue;
    if (st == KILO_FRAME_OK || st == KILO_FRAME_BAD_PAYLOAD_LEN || st == KILO_FRAME_UNKNOWN_TYPE)
      on_frame(st, f);
    else if (st == KILO_FRAME_OVERFLOW)
      L.cnt.rx_overflow++;
    else
      L.cnt.rx_crc_err++;  // bad CRC, COBS or length: nothing in it can be trusted
  }
}

void link_send_events(void) {
  const Event* e;
  while ((e = events_peek(B->ev)) != 0) {
    if (link_hal_tx_free() < KILO_EVENT_LEN + KILO_FRAME_OVERHEAD) return;  // wait for the next tick
    kilo_event_t m;
    m.code = e->code;
    m.arg = e->arg;
    m.tick = e->tick;
    m.epoch = e->epoch;
    emit(KILO_MSG_EVENT, &m, sizeof m, true);
    events_pop(B->ev);
  }
}

static void telemetry(const Body& b) {
  kilo_telemetry_t m;
  LinkSample s;
  link_hal_sample(&s);
  m.tick = b.tick;
  m.pitch = b.att.pitch;
  m.roll = b.att.roll;
  m.rate_pitch = b.att.rate_pitch;
  m.rate_roll = b.att.rate_roll;
  m.rate_yaw = b.att.rate_yaw;
  m.dist_mm = s.dist_mm;
  m.batt_mv = battery_mv(b.batt);
  m.mode = body_mode(b);
  m.owners = (uint8_t)(b.arb.owner[3] | (b.arb.owner[0] << 4));  // a leg, the head
  uint16_t fl = 0;
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++)
    if (b.arb.out[k] != JOINT_OFF) fl = KILO_TELEMETRY_FLAGS_SERVOS_POWERED;
  if (b.imu_fault) fl |= KILO_TELEMETRY_FLAGS_IMU_FAULT;
  if (b.batt.level >= BATT_WARN) fl |= KILO_TELEMETRY_FLAGS_BATT_WARN;
  if (b.wd.armed && !b.wd.stage) fl |= KILO_TELEMETRY_FLAGS_LINK_OK;
  if (s.overrun) fl |= KILO_TELEMETRY_FLAGS_OVERRUN;
  if (b.batt.no_battery) fl |= KILO_TELEMETRY_FLAGS_NO_BATTERY;
  if (config_busy(*b.cfg)) fl |= KILO_TELEMETRY_FLAGS_EEPROM_PENDING;
  if (s.dist_fresh) fl |= KILO_TELEMETRY_FLAGS_DIST_FRESH;
  m.flags = fl;
  m.last_rx_seq = L.last_rx_seq;
  m.trick_frame = 0xFF;
  m.gait_phase = 0;
  m.tick_load = s.tick_load;
  m.rx_err = (uint8_t)(L.cnt.rx_crc_err + L.cnt.rx_overflow);
  m.epoch = b.epoch;
  emit(KILO_MSG_TELEMETRY, &m, sizeof m, false);
}

// A stream with divisor div is due on ticks that are multiples of div;
// div 0 = off.
KILO_NOINLINE static bool due(uint16_t tick, uint8_t div) { return div && tick % div == 0; }

void link_tick(void) {
  const Body& b = *B;
  if (++L.sec_ticks >= TICKS_PER_S) {
    L.sec_ticks = 0;
    L.cnt.uptime_s++;
    L.refused_mask = 0;
    uint16_t p = link_hal_imu_packets();
    uint16_t d = (uint16_t)(p - L.pkt_prev);
    L.pkt_prev = p;
    L.dmp_rate = (uint8_t)(d > 255 ? 255 : d);
    if (L.drops_this_s) {
      events_push(B->ev, EV_TX_DROPPED, (int16_t)(L.drops_this_s > 0x7FFF ? 0x7FFF : L.drops_this_s), b.tick, b.epoch);
      L.drops_this_s = 0;
    }
  }
  link_send_events();
  if (due(b.tick, L.div_telem)) telemetry(b);
  if (due(b.tick, L.div_joints)) {
    kilo_joints_t m;
    m.tick = b.tick;
    for (uint8_t k = 0; k < DRIVEN_COUNT; k++) m.angle[k] = b.arb.out[k];
    emit(KILO_MSG_JOINTS, &m, sizeof m, false);
  }
  if (due(b.tick, L.div_imu)) {
    kilo_imu_raw_t m;
    int16_t r[6];
    m.tick = b.tick;
    link_hal_imu_raw(r);
    memcpy((uint8_t*)&m + 2, r, sizeof r);
    emit(KILO_MSG_IMU_RAW, &m, sizeof m, false);
  }
}
