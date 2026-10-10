#include "fake_link_hal.h"
#include <cstring>
#include "../src/core/body.h"

// core/link serves the global body; every host test links it.
Body g_body;
FakeLinkHal g_lh;

void FakeLinkHal::reset() {
  *this = FakeLinkHal();
  kilo_rx_init(&dec);
}

std::vector<TxFrame> FakeLinkHal::take() {
  std::vector<TxFrame> out;
  for (; decoded_upto < tx.size(); decoded_upto++) {
    kilo_frame_t f;
    uint8_t st = kilo_rx_push(&dec, tx[decoded_upto], &f);
    if (st == KILO_FRAME_NONE) continue;
    TxFrame t;
    t.type = st == KILO_FRAME_OK ? f.type : 0;  // 0 = the body sent a malformed frame
    t.seq = f.seq;
    t.payload.assign(f.payload, f.payload + f.len);
    out.push_back(t);
  }
  return out;
}

bool link_hal_read(uint8_t* c) {
  if (g_lh.rx.empty()) return false;
  *c = g_lh.rx.front();
  g_lh.rx.pop_front();
  return true;
}

uint8_t link_hal_tx_free(void) {
  g_lh.ring_used -= g_lh.drain;
  if (g_lh.ring_used < 0) g_lh.ring_used = 0;
  return (uint8_t)(g_lh.cap - g_lh.ring_used);
}

void link_hal_write(const uint8_t* data, uint8_t len) {
  g_lh.ring_used += len;
  g_lh.tx.insert(g_lh.tx.end(), data, data + len);
}

void link_hal_sample(LinkSample* s) {
  *s = g_lh.sample;
  g_lh.sample.dist_fresh = 0;
}

void link_hal_stats(kilo_stats_t* s) { *s = g_lh.stats; }
uint16_t link_hal_imu_packets(void) { return g_lh.packets; }
void link_hal_imu_raw(int16_t raw[6]) { std::memcpy(raw, g_lh.raw, sizeof g_lh.raw); }

bool link_hal_pca_read(uint16_t counts[16]) {
  std::memcpy(counts, g_lh.pca, sizeof g_lh.pca);
  return g_lh.pca_ok;
}

void link_hal_imu_offsets(const int16_t offsets[6]) {
  std::memcpy(g_lh.offsets, offsets, sizeof g_lh.offsets);
  g_lh.offset_writes++;
}

void link_hal_reboot(void) { g_lh.reboots++; }

std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> v;
  for (size_t i = 0; i + 1 < s.size(); i += 2) v.push_back((uint8_t)std::stoi(s.substr(i, 2), nullptr, 16));
  return v;
}

std::string hex(const std::vector<uint8_t>& v) {
  static const char* d = "0123456789abcdef";
  std::string s;
  for (uint8_t b : v) {
    s += d[b >> 4];
    s += d[b & 15];
  }
  return s;
}

std::vector<uint8_t> frame(uint8_t type, uint8_t seq, const std::vector<uint8_t>& payload) {
  uint8_t w[KILO_MAX_WIRE];
  uint8_t n = kilo_frame_encode(type, seq, payload.data(), (uint8_t)payload.size(), w);
  return std::vector<uint8_t>(w, w + n);
}
