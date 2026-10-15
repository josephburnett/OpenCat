// The body's real core (core/ + core/link.cpp) on the host as a virtual
// body: the doc §6 "body/test/vbody" (real core on stub HALs). Driven line by
// line from stdin by mind/kilo/vbody/core.py, which puts it on a pty or an
// in-process transport for the mind's SerialLink.
//
// It runs what app/main.cpp runs, minus the hardware: each tick is
// run_tick() (link_poll, body_tick, link_poll, PCA write, link_tick) with
// the IMU, battery and IMU-health inputs scripted, then a slice of
// background() (the EEPROM writer, EVENT EEPROM_SAVED). Between ticks the
// link is polled whenever bytes arrive (`rx`), as main()'s loop does.
// Nothing in core/ is changed for it: the link_hal_* seam is the host fake
// (test/fake_link_hal.cpp) and the EEPROM is test/fake_eeprom.cpp.
//
// Commands (one per line):
//   reset cold [CONFIG_STATE [IMU_OK]]   power-on: epoch 0 (the mind's and the
//                                  Python vbody's fresh body; a real cold
//                                  boot starts at a random epoch), limp,
//                                  HELLO(COLD) + EVENT BOOT (+ CONFIG_* and
//                                  IMU_FAULT events as main.cpp)
//   reset warm [CAUSE]             reset with the PCA9685 still driving: the
//                                  current outputs are held (warm-boot hold),
//                                  epoch = reset_boot_epoch(last), HELLO(CAUSE,
//                                  default WARM_UNEXPECTED) + EVENT BOOT
//   att PITCH ROLL [RP RR RY]      attitude for the following ticks (a16 and
//                                  a16/s; gravity z follows from pitch, roll)
//   imu fresh|none|err             the IMU read result for the following ticks
//   batt COUNTS|none               ADC counts sampled at 10 Hz as main.cpp
//                                  (default 803 = 8.1 V); none = no samples
//   rx HEX                         wire bytes from the Pi; link_poll
//   tick N                         N ticks (run_tick + a background slice)
//   dump                           one "state k=v ..." line (see dump())
//
// After every command (and once at start, for the power-on): "tx HEX" (every
// byte the body wrote, possibly none), then "end". Every boot (power-on,
// `reset`, an acked REBOOT) first prints "tx" (what was sent before it) and
// "boot", so the caller can hold the boot's HELLO for the AVR's init time.
// A REBOOT the body acked is emulated like the AVR: a warm reset (cause
// REQUESTED) right after the command.
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include "../src/core/body.h"
#include "../src/core/fx.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/reset.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"

static Config cfg;
static TickIn scripted;      // the IMU inputs of every tick
static int batt_counts = 803;  // < 0: no ADC samples
static uint16_t counts[16];
static size_t tx_sent = 0;   // bytes of g_lh.tx already printed

// HELLO.fw_build of the host core ("CORE").
#define CORE_BUILD 0x45524F43u

static void set_att(int pitch, int roll, int rp, int rr, int ry) {
  Attitude& a = scripted.att;
  a.pitch = (int16_t)pitch;
  a.roll = (int16_t)roll;
  a.rate_pitch = (int16_t)rp;
  a.rate_roll = (int16_t)rr;
  a.rate_yaw = (int16_t)ry;
  // gz = cos(pitch) cos(roll) in Q14 (negative upside down), as the DMP's
  // gravity vector would give for that tilt.
  a.gz = (int16_t)(((int32_t)fx_cos((int16_t)pitch) * fx_cos((int16_t)roll)) >> 14);
}

static const char* hexdigits = "0123456789abcdef";

// Print the bytes written since the last call: "tx HEX".
static void emit_tx(void) {
  std::string s = "tx ";
  for (size_t i = tx_sent; i < g_lh.tx.size(); i++) {
    s += hexdigits[g_lh.tx[i] >> 4];
    s += hexdigits[g_lh.tx[i] & 15];
  }
  tx_sent = g_lh.tx.size();
  if (tx_sent > 1u << 20) {  // keep the log bounded on long runs
    g_lh.tx.clear();
    g_lh.decoded_upto = 0;
    tx_sent = 0;
  }
  std::printf("%s\n", s.c_str());
}

// main.cpp's boot sequence from "config loaded" on.
static void boot(bool warm, uint8_t cause, uint8_t epoch, uint16_t last_alive, bool imu_ok) {
  // What the body sent before the reset, then the marker: the caller may
  // model the boot time (the AVR's DMP init, ~0.3 s, before HELLO).
  emit_tx();
  std::printf("boot\n");
  int16_t warm_angles[DRIVEN_COUNT];
  if (warm)
    for (uint8_t k = 0; k < DRIVEN_COUNT; k++) warm_angles[k] = g_body.arb.out[k];
  params_load(cfg.img);  // a reset reloads the live params from the stored image
  body_init(g_body, &cfg, warm ? warm_angles : nullptr, imu_ok, epoch);
  // The link fake keeps the PCA counts across a warm reset (the chip kept
  // driving); everything else starts over.
  uint16_t pca[16];
  for (int c = 0; c < 16; c++) pca[c] = warm ? g_lh.pca[c] : (uint16_t)PCA_FULL_OFF;
  std::vector<uint8_t> tx = g_lh.tx;
  g_lh.reset();
  g_lh.tx = tx;  // the TX stream continues (tx_sent indexes it)
  g_lh.decoded_upto = tx.size();
  for (int c = 0; c < 16; c++) g_lh.pca[c] = pca[c];
  uint16_t caps = KILO_HELLO_CAPS_BUZZER | KILO_HELLO_CAPS_ULTRASONIC | KILO_HELLO_CAPS_IR;
  if (imu_ok) caps |= KILO_HELLO_CAPS_DMP;
  link_init(CORE_BUILD, caps);
  link_boot(cause, last_alive);
  events_push(g_body.ev, EV_BOOT, cause, 0, g_body.epoch);
  if (cfg.state == CFG_MIGRATED) events_push(g_body.ev, EV_CONFIG_MIGRATED, 0, 0, g_body.epoch);
  if (cfg.state == CFG_DEFAULTED) events_push(g_body.ev, EV_CONFIG_DEFAULTED, cfg.reason, 0, g_body.epoch);
  if (!imu_ok) events_push(g_body.ev, EV_IMU_FAULT, 0, 0, g_body.epoch);
}

static void cold(uint8_t state, bool imu_ok) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)state;
  cfg.reason = state == CFG_DEFAULTED ? CFG_REASON_PWM_PIN : CFG_REASON_NONE;
  cfg.writing = 0;
  cfg.saved = 0;
  cfg.newest = 0xFF;
  scripted = TickIn();
  scripted.imu = IMU_FRESH;
  set_att(0, 0, 0, 0, 0);
  batt_counts = 803;
  boot(false, RESET_COLD, 0, 0, imu_ok);
}

static void warm(uint8_t cause) {
  boot(true, cause, reset_boot_epoch(g_body.epoch), g_body.tick, !g_body.imu_fault);
}

// main.cpp background(): the EEPROM writer, one byte per call. On the AVR a
// byte takes 3.4 ms and background() runs between ticks; the fake EEPROM is
// busy for 2 ready polls, so 18 calls is about 6 bytes per 20 ms tick.
static void background(void) {
  for (int i = 0; i < 18; i++) config_writer_step(cfg);
  if (cfg.saved) {
    cfg.saved = 0;
    events_push(g_body.ev, EV_EEPROM_SAVED, 0, g_body.tick, g_body.epoch);
  }
}

// main.cpp run_tick() with the scripted inputs.
static void run_tick(void) {
  Body& b = g_body;
  TickIn in = scripted;
  if (b.imu_fault) in.imu = IMU_NONE;  // main.cpp stops reading a faulted IMU
  if (in.imu == IMU_FRESH) {
    g_lh.packets = (uint16_t)(g_lh.packets + 2);  // the DMP runs at 100 Hz
    g_lh.raw[2] = 16384;                          // accel z: 1 g
  }
  // main.cpp starts a conversion after the tick where tick % 5 == 0 and
  // takes the result on the next one.
  in.batt_valid = 0;
  if (batt_counts >= 0 && b.tick && b.tick % BATT_SAMPLE_TICKS == 0) {
    in.batt_valid = 1;
    in.batt_counts = (uint16_t)batt_counts;
  }
  link_poll();
  body_tick(b, in, counts);
  link_poll();
  for (int c = 0; c < 16; c++) g_lh.pca[c] = counts[c];  // pca_write_all
  link_tick();
  background();
}

static void flush(void) {
  if (g_lh.reboots) warm(RESET_REQUESTED);  // REBOOT acked: the AVR resets through the watchdog
  emit_tx();
  std::printf("end\n");
  std::fflush(stdout);
}

static void list(const char* key, const int16_t* v, int n) {
  std::printf(" %s=", key);
  for (int i = 0; i < n; i++) std::printf(i ? ",%d" : "%d", v[i]);
}

// The state the mind cannot see directly, as key=value pairs. Per-joint
// lists are in canonical order (pan, tilt, tail, shoulders LF RF RB LB,
// knees LF RF RB LB); out is a16 (-32768 = limp).
static void dump(void) {
  const Body& b = g_body;
  const Arbiter& a = b.arb;
  std::printf("state tick=%u epoch=%u mode=%u posture=%u idle_moving=%d holder=%u reflex=%u reflex_posture=%u",
              b.tick, b.epoch, body_mode(b), arbiter_idle_posture(a), arbiter_idle_moving(a) ? 1 : 0, b.holder,
              a.reflex_active, a.reflex.posture);
  int16_t own[DRIVEN_COUNT];
  for (int k = 0; k < DRIVEN_COUNT; k++) own[k] = a.owner[k];
  list("owner", own, DRIVEN_COUNT);
  list("out", a.out, DRIVEN_COUNT);
  std::printf(" head_ttl=%u", a.head_ttl);
  std::printf(" wd_armed=%u wd_stage=%u wd_since=%u wd_limp_timer=%u", b.wd.armed, b.wd.stage, b.wd.since,
              b.wd.limp_timer);
  const Feet& g = b.feet;
  std::printf(" feet_state=%u feet_queued=%u feet_support=%u feet_late=%u feet_underruns=%u", g.state,
              feet_queued(g), feet_support(g), g.late, g.underruns);
  int16_t pose[4] = {g.pose[0], g.pose[1], g.dh, g.xs};
  list("pose", pose, 4);
  const Fall& f = b.fall;
  std::printf(" fall_state=%u fall_axis=%d fall_tries=%u fall_timer=%u", f.state, f.axis, f.tries, f.timer);
  const Keyframe& kf = b.kf;
  std::printf(" trick_run=%u trick_id=%u trick_store=%u trick_frame=%u", kf.run, kf.id, kf.store, kf.fi);
  std::printf(" balance_req=%u balance_enable=%u balance_active=%u", b.balance_req, b.bal.enable, b.bal.active);
  int16_t corr[2] = {b.bal.ax[BAL_ROLL].corr, b.bal.ax[BAL_PITCH].corr};
  list("balance_corr", corr, 2);
  std::printf(" batt_level=%u batt_mv=%u no_battery=%u imu_fault=%u cfg_state=%u cfg_busy=%d events_queued=%u\n",
              b.batt.level, battery_mv(b.batt), b.batt.no_battery, b.imu_fault, cfg.state, config_busy(cfg) ? 1 : 0,
              b.ev.count);
}

int main() {
  cold(CFG_OK, true);
  flush();
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "reset") {
      std::string kind;
      in >> kind;
      if (kind == "warm") {
        int cause = RESET_WARM_UNEXPECTED;
        in >> cause;
        warm((uint8_t)cause);
      } else {
        int state = CFG_OK, imu_ok = 1;
        in >> state >> imu_ok;
        cold((uint8_t)state, imu_ok != 0);
      }
    } else if (cmd == "att") {
      int p = 0, r = 0, rp = 0, rr = 0, ry = 0;
      in >> p >> r >> rp >> rr >> ry;
      set_att(p, r, rp, rr, ry);
    } else if (cmd == "imu") {
      std::string m;
      in >> m;
      scripted.imu = m == "fresh" ? IMU_FRESH : m == "err" ? IMU_ERR : IMU_NONE;
    } else if (cmd == "batt") {
      std::string v;
      in >> v;
      batt_counts = v == "none" ? -1 : std::atoi(v.c_str());
    } else if (cmd == "rx") {
      std::string h;
      in >> h;
      g_lh.feed(unhex(h));
      link_poll();
    } else if (cmd == "tick") {
      int n = 0;
      in >> n;
      for (int i = 0; i < n; i++) {
        run_tick();
        if (g_lh.reboots) break;  // the reset happens in flush()
      }
    } else if (cmd == "dump") {
      dump();
    } else if (!cmd.empty()) {
      std::printf("error unknown command %s\n", cmd.c_str());
    }
    flush();
  }
  return 0;
}
