// LINK: src/core/*.cpp src/hal/uart.cpp src/hal/i2c.cpp src/hal/mpu6050.cpp src/hal/pca9685.cpp src/hal/sys.cpp src/hal/eeprom.cpp src/hal/adc.cpp src/hal/buzzer.cpp src/hal/ir.cpp src/hal/pcint2.cpp src/hal/ultrasonic.cpp src/app/link_hal.cpp
//
// app/main.cpp, the whole firmware on the fake AVR: every hal/ and core/
// source as built for Kilo except hal/clock.cpp, replaced by a virtual clock
// (clock_micros() advances 20 us per call, delivers the UART's UDRE
// interrupts, feeds the MPU model a DMP packet every 5 ms, and ends a run by
// throwing out of the main loop). The I2C chips are the register models of
// fake_i2c_devices.h on the fake TWI bus. Checks the boot sequence: cold vs
// warm (the PCA9685 probe, the read-back pose held), the reset cause in
// HELLO and EVENT BOOT, the DMP bring-up before the watchdog, IMU_FAULT
// when the IMU is missing, the runtime IMU re-init backoff (one attempt per
// 5 s while LIMP) and recovery (EVENT IMU_OK), and the 50 Hz tick.
// (The "LINK:" line above is read by tools/host_tests.sh.)
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../check.h"
#include "fake_avr.h"
#include "fake_i2c_devices.h"
#include <avr/wdt.h>
#include "../../src/proto/kilo_frame.h"
#define main kilo_main
#include "../../src/app/main.cpp"
#undef main

// ---- the virtual clock and the rest of the world ----

extern "C" void isr_USART_UDRE(void);  // hal/uart.cpp

struct StopRun {};
static uint32_t g_now, g_stop_after, g_stop_at, g_loop_at, g_next_pkt;
static bool g_in_loop;
static std::vector<uint8_t> g_tx;

// The MPU model, noting whether the watchdog was armed at each write.
struct Mpu : MpuModel {
  long writes_armed_at_boot = 0;
  void on_write(uint8_t reg, uint8_t v) override {
    if (!g_in_loop && fake_wdt_timeout != -1) writes_armed_at_boot++;
    MpuModel::on_write(reg, v);
  }
};
static Mpu mpu;
static PcaModel pca;
static bool mpu_attached;
static bool g_feed_dmp = true;  // the DMP delivers packets once enabled

static void dmp_packet(uint8_t* p) {
  std::memset(p, 0, DMP_PACKET_SIZE);
  p[0] = 0x40;  // quaternion w = 1.0 (q30, big-endian): level and still
}

void clock_init(void) {}
uint32_t clock_micros(void) {
  g_now += 20;
  // The UDRE interrupt: an infinitely fast UART line.
  while (UCSR0B & _BV(UDRIE0)) {
    isr_USART_UDRE();
    if (UCSR0B & _BV(UDRIE0)) g_tx.push_back((uint8_t)UDR0);
  }
  // The DMP produces a packet every 5 ms while it is enabled.
  if (g_feed_dmp && mpu_attached && (mpu.regs[0x6A] & 0x80) && (int32_t)(g_now - g_next_pkt) >= 0) {
    uint8_t p[DMP_PACKET_SIZE];
    dmp_packet(p);
    mpu.push_packet(p, DMP_PACKET_SIZE);
    g_next_pkt = g_now + 5000;
  }
  // The main loop starts right after the watchdog is armed.
  if (!g_in_loop && fake_wdt_timeout == WDTO_250MS) {
    g_in_loop = true;
    g_loop_at = g_now;
    g_stop_at = g_now + g_stop_after;
  }
  if (g_in_loop && (int32_t)(g_now - g_stop_at) >= 0) throw StopRun{};
  return g_now;
}

static void attach_mpu(bool on) {
  mpu_attached = on;
  g_twi.attach(MPU_ADDR, on ? &mpu : nullptr);
}

// Power the fake board up (RAM-preserving or not is up to the caller's
// sys_reset_record) and run main() until `run_us` after the loop starts.
static void boot(uint32_t run_us, bool with_mpu = true) {
  fake_avr_reset();
  mpu.reset();
  mpu.writes_armed_at_boot = 0;
  g_twi.attach(PCA_ADDR, &pca);
  attach_mpu(with_mpu);
  g_tx.clear();
  g_in_loop = false;
  g_stop_after = run_us;
  g_now = 1000;
  g_next_pkt = 0;
  imu_present = imu_reinit = 0;
  imu_retry_at = 0;
  try {
    kilo_main();
  } catch (const StopRun&) {
  }
}

// Continue the main loop for `us` more (as main()'s loop does).
static void run_more(uint32_t us) {
  g_stop_at = g_now + us;
  try {
    for (;;) {
      link_poll();
      if (sched_due(sched, clock_micros()))
        run_tick();
      else
        background();
    }
  } catch (const StopRun&) {
  }
}

struct Frame {
  uint8_t type;
  std::vector<uint8_t> p;
};
static std::vector<Frame> frames() {
  std::vector<Frame> out;
  kilo_rx_t rx;
  kilo_rx_init(&rx);
  for (uint8_t c : g_tx) {
    kilo_frame_t f;
    uint8_t st = kilo_rx_push(&rx, c, &f);
    if (st == KILO_FRAME_NONE) continue;
    out.push_back({st == KILO_FRAME_OK ? f.type : (uint8_t)0, std::vector<uint8_t>(f.payload, f.payload + f.len)});
  }
  return out;
}
static bool hello(kilo_hello_t* h) {
  for (const Frame& f : frames())
    if (f.type == KILO_MSG_HELLO && f.p.size() == sizeof *h) {
      std::memcpy(h, f.p.data(), sizeof *h);
      return true;
    }
  return false;
}
static int events(uint8_t code, int16_t* arg = nullptr) {
  int n = 0;
  for (const Frame& f : frames())
    if (f.type == KILO_MSG_EVENT && f.p.size() >= 3 && f.p[0] == code) {
      n++;
      if (arg) *arg = (int16_t)(f.p[1] | (f.p[2] << 8));
    }
  return n;
}
static long mpu_probes() {  // SLA+W to the MPU (each transfer starts with one)
  long n = 0;
  for (const TwiEvent& e : g_twi.log) n += e.kind == 'A' && e.v == (MPU_ADDR << 1);
  return n;
}
static long pca_bursts() {  // writes starting at LED0_ON_L with 64 data bytes
  long n = 0;
  for (size_t i = 0; i + 1 < g_twi.log.size(); i++)
    if (g_twi.log[i].kind == 'A' && g_twi.log[i].v == (PCA_ADDR << 1) && g_twi.log[i + 1].kind == 'W' &&
        g_twi.log[i + 1].v == 0x06 && i + 66 <= g_twi.log.size() && g_twi.log[i + 65].kind == 'W')
      n++;
  return n;
}

int main() {
  alarm(120);  // a hang (e.g. a busy-wait the fakes never satisfy) fails, not stalls, the suite

  // ---- cold boot: power-on RAM, a power-on PCA9685, the IMU present ----
  {
    sys_reset_record.magic = 0x1234;  // power-on garbage
    sys_reset_record.epoch = 41;
    pca.power_on();
    boot(1000000);
    CHECK(g_stats.reset_cause == RESET_COLD);
    kilo_hello_t h;
    CHECK(hello(&h));
    CHECK(h.reset_cause == RESET_COLD && h.fw_build == FW_BUILD);
    CHECK(h.caps == (KILO_HELLO_CAPS_BUZZER | KILO_HELLO_CAPS_DMP | KILO_HELLO_CAPS_ULTRASONIC | KILO_HELLO_CAPS_IR));
    int16_t arg = -1;
    CHECK(events(KILO_EVENT_BOOT, &arg) == 1 && arg == RESET_COLD);
    CHECK(events(KILO_EVENT_CONFIG_DEFAULTED) == 1);  // blank EEPROM, no legacy data
    CHECK(events(KILO_EVENT_IMU_FAULT) == 0);
    CHECK(h.epoch == 42 && g_body.epoch == 42);  // record epoch + 1
    // PCA9685 configured cold: prescaler, awake, every channel off (LIMP).
    CHECK(pca.regs[0xFE] == PCA_PRESCALE && pca.regs[0x00] == 0x20 && pca.prescale_ignored == 0);
    bool limp = true;
    for (int c = 0; c < 16; c++) limp = limp && pca.out_off[c] == PCA_FULL_OFF;
    CHECK(limp);
    // The DMP came up, before the watchdog was armed.
    CHECK(imu_present == 1 && mpu.regs[0x6A] == 0xC0 && mpu.device_resets == 1);
    CHECK(mpu.writes_armed_at_boot == 0);
    CHECK(fake_wdt_timeout == WDTO_250MS);
    CHECK(UCSR0B & _BV(RXCIE0));
    // One second of loop: 50 ticks, each one PCA burst and a watchdog kick.
    long bursts = pca_bursts();
    CHECKF(bursts >= 49 && bursts <= 51, "%ld bursts", bursts);
    CHECK(fake_wdt_resets >= 49);
    CHECK(g_body.tick >= 49 && g_body.tick <= 51 && !g_body.imu_fault);
    // The reset record now describes this run (for the next boot).
    CHECK(sys_reset_record.magic == RESET_MAGIC && sys_reset_record.build == RESET_BUILD_ID);
    CHECK(sys_reset_record.alive_tick == g_body.tick && sys_reset_record.epoch == 42);
    CHECK(g_stats.max_tick_us > 0);
  }

  // ---- warm boot: the AVR reset under a PCA9685 that kept the pose ----
  {
    uint16_t pose[16];
    for (uint8_t c = 0; c < 16; c++) pose[c] = PCA_FULL_OFF;
    for (uint8_t k = 0; k < DRIVEN_COUNT; k++)
      pose[joint_channel(joint_id(k))] = (uint16_t)(1300 + 20 * k);
    pca.power_on();
    pca.driving(pose);
    // sys_reset_record is as the cold run above left it: same build, alive.
    uint16_t alive = sys_reset_record.alive_tick;
    g_feed_dmp = false;  // no attitude yet: the hold lasts up to 1 s
    boot(600000);
    CHECK(g_stats.reset_cause == RESET_WARM_UNEXPECTED);
    kilo_hello_t h;
    CHECK(hello(&h) && h.reset_cause == RESET_WARM_UNEXPECTED && h.last_alive_tick == alive);
    CHECK(h.epoch == 43);
    // Not re-initialised (no SLEEP written) ...
    bool slept = false;
    for (const RegWrite& w : pca.writes) slept = slept || (w.reg == 0x00 && (w.val & 0x10));
    CHECK(!slept);
    // ... and the pose it was driving is what the ticks keep writing
    // (counts -> angles -> counts round trip, within a count).
    bool held = true;
    for (int c = 0; c < 16; c++) {
      int d = (int)pca.out_off[c] - (int)pose[c];
      if (d < -1 || d > 1) held = false;
    }
    CHECK(held);
    CHECK(pca_bursts() >= 29);
    // After 1 s the hold ends and, with no mind talking, the link-lost
    // policy takes the joints (doc §3a step 5, §3g): the pose moves.
    g_feed_dmp = true;
    run_more(1500000);
    CHECK(body_mode(g_body) == 9);
    CHECK(pca.out_off[joint_channel(joint_id(0))] != pose[joint_channel(joint_id(0))]);
  }

  // ---- REBOOT request and a reflash ----
  {
    sys_reset_record.reason = RESET_REASON_REQUESTED;
    pca.power_on();
    boot(100000);
    CHECK(g_stats.reset_cause == RESET_REQUESTED);
    sys_reset_record.build ^= 1;  // a different firmware ran before
    boot(100000);
    CHECK(g_stats.reset_cause == RESET_FLASHED);
    int16_t arg;
    CHECK(events(KILO_EVENT_BOOT, &arg) == 1 && arg == RESET_FLASHED);
  }

  // ---- no IMU: 3 attempts at boot, IMU_FAULT, then one retry per 5 s ----
  {
    pca.power_on();
    boot(12000000, false);
    CHECK(imu_present == 0);
    kilo_hello_t h;
    CHECK(hello(&h) && !(h.caps & KILO_HELLO_CAPS_DMP));
    CHECK(events(KILO_EVENT_IMU_FAULT) == 1);
    CHECK(g_body.imu_fault && body_mode(g_body) == 1);  // LIMP
    // Boot: 3 attempts, each failing at the WHO_AM_I probe. Loop (12 s):
    // the first retry at once, then at +5 s and +10 s.
    long probes = mpu_probes();
    CHECKF(probes == 3 + 3, "%ld MPU transfers", probes);
    // The IMU answers again (a loose cable reseated): the next retry, due
    // at +15 s, brings the DMP up and the body reports IMU_OK.
    size_t tx0 = g_tx.size();
    attach_mpu(true);
    run_more(2000000);
    CHECK(imu_present == 0 && mpu.device_resets == 0);  // still backing off at +14 s
    run_more(3000000);
    CHECK(imu_present == 1 && !g_body.imu_fault && imu_reinit == 0);
    CHECK(mpu.regs[0x6A] == 0xC0 && mpu.device_resets == 1);
    std::vector<uint8_t> all = g_tx;
    g_tx.assign(all.begin() + (long)tx0, all.end());
    CHECK(events(KILO_EVENT_IMU_OK) == 1);
    g_tx = all;
    // Fresh packets keep it healthy.
    run_more(1000000);
    CHECK(!g_body.imu_fault && mpu_packet_count() > 0);
  }

  return report("hal_main");
}
