// Small units: scheduler, reset cause, note queue, event queue, CRC, and two
// cross-checks against the legacy sources: the DMP image bytes and the
// built-in postures.
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "../src/proto/crc16.h"
#include "../src/core/events.h"
#include "../src/core/notes.h"
#include "../src/core/posture.h"
#include "../src/core/reset.h"
#include "../src/core/sched.h"
#include "../src/hal/dmp_image.h"
#include "check.h"

#ifndef REPO_ROOT
#define REPO_ROOT "../.."
#endif

static std::string slurp(const char* rel) {
  std::ifstream f(std::string(REPO_ROOT) + "/" + rel);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Every integer literal (decimal or 0x hex, optionally negative) in s,
// skipping /* comments */ (the DMP image has "/* bank # 0 */" markers).
static std::vector<int> numbers(const std::string& s) {
  std::vector<int> v;
  for (size_t i = 0; i < s.size();) {
    if (s.compare(i, 2, "/*") == 0) {
      size_t e = s.find("*/", i);
      i = e == std::string::npos ? s.size() : e + 2;
      continue;
    }
    bool neg = s[i] == '-' && i + 1 < s.size() && std::isdigit((unsigned char)s[i + 1]);
    if (std::isdigit((unsigned char)s[i]) || neg) {
      size_t used = 0;
      long x = std::stol(s.substr(i), &used, 0);
      v.push_back((int)x);
      i += used;
    } else {
      i++;
    }
  }
  return v;
}

int main() {
  // ---- sched: fixed grid, no drift, resync after a long stall, wrap-safe.
  Sched s;
  sched_start(s, 1000);
  CHECK(!sched_due(s, 20999) && sched_due(s, 21000));
  CHECK(sched_due(s, 41500) && s.next == 61000);  // late start, grid kept
  CHECK(!sched_due(s, 60999));
  CHECK(sched_due(s, 200000) && s.overruns == 1 && s.next == 220000);  // resync
  sched_start(s, 0xFFFFF000u);
  CHECK(!sched_due(s, 0xFFFFFFF0u) && sched_due(s, 0x00003E20u));  // across the micros() wrap
  // ---- reset cause.
  ResetRecord r = {0x1234, 0, 0, 0};
  uint16_t last;
  CHECK(reset_classify(r, 77, last) == RESET_COLD && last == 0);
  r.alive_tick = 500;
  CHECK(reset_classify(r, 78, last) == RESET_FLASHED && last == 500);
  r.alive_tick = 9;
  r.reason = RESET_REASON_REQUESTED;
  CHECK(reset_classify(r, 78, last) == RESET_REQUESTED && last == 9);
  r.alive_tick = 4242;
  CHECK(reset_classify(r, 78, last) == RESET_WARM_UNEXPECTED && last == 4242);
  // ---- CRC-16/CCITT-FALSE check value.
  CHECK(kilo_crc16((const uint8_t*)"123456789", 9) == 0x29B1);
  // ---- notes: legacy pitches, rests, clamp, timing.
  CHECK(note_hz(0) == 0 && note_hz(12) == 2092 && note_hz(23) == 3950 && note_hz(70) == 3950);
  CHECK(note_hz(1) == 1108 && note_hz(-60) == 32 && note_hz(-100) == 32);
  Notes n;
  notes_init(n);
  CHECK(notes_push(n, 12, 5) && notes_push(n, 0, 2) && notes_push(n, 13, 1));
  CHECK(notes_tick(n, 20) && n.hz == 2092);   // 50 ms: ticks 1-3
  CHECK(!notes_tick(n, 20) && n.hz == 2092);
  CHECK(!notes_tick(n, 20) && n.hz == 2092);
  CHECK(notes_tick(n, 20) && n.hz == 0);      // rest 20 ms
  CHECK(notes_tick(n, 20) && n.hz == 2217);   // 10 ms note still gets a tick
  CHECK(notes_tick(n, 20) && n.hz == 0 && n.count == 0);
  for (int i = 0; i < NOTES_LEN; i++) CHECK(notes_push(n, 1, 1));
  CHECK(!notes_push(n, 1, 1));
  // ---- events: FIFO, bounded, never overwrites.
  EventQueue e;
  events_init(e);
  for (int i = 0; i < 10; i++) events_push(e, (uint8_t)(i + 1), (int16_t)i, 0, 0);
  CHECK(e.count == 8 && e.lost == 2 && events_peek(e)->code == 1);
  events_pop(e);
  CHECK(events_peek(e)->code == 2);
  // ---- DMP image == legacy dmpMemory[], byte for byte.
  std::string h = slurp("src/mpu6050/MPU6050_6Axis_MotionApps_V6_12.h");
  size_t a = h.find("dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {");
  size_t b = h.find("};", a);
  CHECK(a != std::string::npos && b != std::string::npos);
  std::string body = h.substr(a, b - a);
  body = body.substr(body.find('{') + 1);
  std::vector<int> legacy = numbers(body);
  CHECKF(legacy.size() == DMP_IMAGE_SIZE, "legacy image has %zu bytes", legacy.size());
  int diff = 0;
  for (size_t i = 0; i < legacy.size() && i < DMP_IMAGE_SIZE; i++)
    if (legacy[i] != dmp_image[i]) diff++;
  CHECK(diff == 0);
  // ---- Built-in postures == InstinctNybble.h (joints 3-7 dropped).
  std::string inst = slurp("src/InstinctNybble.h");
  struct {
    const char* name;
    uint8_t id;
  } P[] = {{"rest", POSTURE_REST}, {"sit", POSTURE_SIT}, {"up", POSTURE_STAND},
           {"calib", POSTURE_CALIB}, {"zero", POSTURE_ZERO}, {"balance", POSTURE_STAND}};
  for (auto& p : P) {
    std::string key = std::string("const int8_t ") + p.name + "[] PROGMEM = {";
    size_t i = inst.find(key);
    CHECK(i != std::string::npos);
    std::vector<int> v = numbers(inst.substr(i + key.size(), inst.find("};", i) - i - key.size()));
    CHECK(v.size() == 20 && v[0] == 1);
    CHECKF(posture_roll(p.id) == v[1] * 16 && posture_pitch(p.id) == v[2] * 16, "%s expected roll/pitch", p.name);
    static const int J[11] = {0, 1, 2, 8, 9, 10, 11, 12, 13, 14, 15};
    for (int k = 0; k < 11; k++)
      CHECKF(posture_angle(p.id, (uint8_t)k) == v[4 + J[k]] * 16, "%s joint %d", p.name, J[k]);
  }
  return report("misc");
}
