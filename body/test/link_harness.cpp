// The body's real link dispatch (core/link.cpp + core/) on the host, driven
// line by line from stdin, for the cross-check against the mind's virtual
// body (mind/tests/test_crosscheck.py; doc §6 "the virtual body agrees with
// the real body logic").
//
//   reset CONFIG_STATE BATT_LEVEL   fresh boot state (epoch 0, cold, limp)
//   rx HEX                          feed wire bytes; run link_poll, the
//                                   app's EEPROM_SAVED step and the event flush
//   save                            finish a running config save (as main.cpp's
//                                   background loop would) and flush events
//   advance N                       move the tick on by N (de-dup windows)
//   tick N                          run N body ticks (IMU fresh and level, no
//                                   battery sample), then flush events: the
//                                   keyframe player and the reflexes run
//
// After every command: one line per frame the body sent, "tx TYPE PAYLOADHEX"
// (decimal type), then "end". A REBOOT that is acked OK is emulated like the
// AVR: the body restarts and sends HELLO (REQUESTED) and EVENT BOOT.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "../src/core/body.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/reset.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"

static Config cfg;

static void boot(uint8_t state, uint8_t batt) {
  g_ee.reset();
  config_defaults(cfg.img);
  cfg.state = (ConfigState)state;
  cfg.writing = 0;
  cfg.saved = 0;
  cfg.newest = 0xFF;
  params_load(cfg.img);
  body_init(g_body, &cfg, nullptr, true);
  g_body.batt.level = (BattLevel)batt;
  g_lh.reset();
  for (int c = 0; c < 16; c++) g_lh.pca[c] = PCA_FULL_OFF;  // cold boot: every channel off
  link_init(0x5642444Fu, KILO_HELLO_CAPS_DMP);
}

static void flush(void) {
  if (cfg.saved) {  // what main.cpp's background() does
    cfg.saved = 0;
    events_push(g_body.ev, EV_EEPROM_SAVED, 0, g_body.tick, g_body.epoch);
  }
  link_send_events();
  std::vector<TxFrame> out = g_lh.take();
  if (g_lh.reboots) {
    uint8_t state = cfg.state, batt = g_body.batt.level;
    uint16_t tick = g_body.tick;
    boot(state, batt);
    link_boot(RESET_REQUESTED, tick);
    events_push(g_body.ev, EV_BOOT, RESET_REQUESTED, 0, 0);
    link_send_events();
    for (const TxFrame& f : g_lh.take()) out.push_back(f);
  }
  for (const TxFrame& f : out) std::printf("tx %d %s\n", f.type, hex(f.payload).c_str());
  std::printf("end\n");
  std::fflush(stdout);
}

int main() {
  boot(CFG_OK, 0);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "reset") {
      int state = 0, batt = 0;
      in >> state >> batt;
      boot((uint8_t)state, (uint8_t)batt);
      g_lh.take();
    } else if (cmd == "rx") {
      std::string h;
      in >> h;
      g_lh.feed(unhex(h));
      link_poll();
    } else if (cmd == "save") {
      while (config_busy(cfg)) config_writer_step(cfg);
    } else if (cmd == "tick") {
      int n = 0;
      in >> n;
      TickIn ti = TickIn();
      ti.imu = IMU_FRESH;
      ti.att.gz = 16384;
      uint16_t counts[16];
      for (int i = 0; i < n; i++) {
        body_tick(g_body, ti, counts);
        link_send_events();
      }
    } else if (cmd == "advance") {
      int n = 0;
      in >> n;
      g_body.tick = (uint16_t)(g_body.tick + n);
    } else if (!cmd.empty()) {
      std::printf("error unknown command %s\n", cmd.c_str());
    }
    flush();
  }
  return 0;
}
