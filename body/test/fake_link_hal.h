// The link_hal_* seam of core/link.h on the host: RX bytes come from a
// queue, TX bytes go to a vector, and the TX ring's free space is simulated
// so the drop priority can be tested. Linked into every host test.
#ifndef KILO_TEST_FAKE_LINK_HAL_H
#define KILO_TEST_FAKE_LINK_HAL_H
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "../src/core/link.h"
#include "../src/proto/kilo_frame.h"

struct TxFrame {
  uint8_t type, seq;
  std::vector<uint8_t> payload;
};

struct FakeLinkHal {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;   // every byte written, in order
  // Simulated TX ring: `ring_used` bytes are waiting; each tx_free() call
  // first drains `drain` of them (the UART ISR). drain >= cap = never full.
  int cap = 95, ring_used = 0, drain = 1000;
  uint16_t pca[16] = {};
  bool pca_ok = true;
  int reboots = 0, offset_writes = 0;
  int16_t offsets[6] = {};
  LinkSample sample = {};
  kilo_stats_t stats = {};
  uint16_t packets = 0;
  int16_t raw[6] = {};
  size_t decoded_upto = 0;
  kilo_rx_t dec;

  void reset();
  void feed(const std::vector<uint8_t>& bytes) { rx.insert(rx.end(), bytes.begin(), bytes.end()); }
  // Frames written since the last call (decoded with the protocol codec).
  std::vector<TxFrame> take();
};

extern FakeLinkHal g_lh;

std::vector<uint8_t> unhex(const std::string& s);
std::string hex(const std::vector<uint8_t>& v);
// Wire bytes for a Pi->body frame.
std::vector<uint8_t> frame(uint8_t type, uint8_t seq, const std::vector<uint8_t>& payload);

#endif
