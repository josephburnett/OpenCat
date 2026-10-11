// The NEC IR decoder (core/ir_nec), fed synthetic falling-edge times.
#include <vector>
#include "../src/core/ir_nec.h"
#include "check.h"

// Falling edges of one NEC frame for the 32-bit value v (MSB first, as the
// legacy IRremote table writes it), starting with the leader mark at t.
static std::vector<uint32_t> frame(uint32_t t, uint32_t v) {
  std::vector<uint32_t> e;
  e.push_back(t);
  t += 13500;  // leader 9 + 4.5 ms
  e.push_back(t);
  for (int8_t i = 31; i >= 0; i--) {
    t += (v >> i) & 1 ? 2250 : 1125;
    e.push_back(t);  // the last one is the stop mark
  }
  return e;
}

static std::vector<uint32_t> repeat(uint32_t t) { return {t, t + 11250}; }

// Feeds edges; returns the keys decoded.
static std::vector<uint16_t> feed(IrNec& s, const std::vector<uint32_t>& edges) {
  std::vector<uint16_t> keys;
  for (uint32_t t : edges)
    if (ir_nec_edge(s, t)) keys.push_back(s.key);
  return keys;
}

int main() {
  IrNec s;

  // A full frame: 0x00FFA25D (CH-) is command 0xA2.
  ir_nec_init(s);
  std::vector<uint16_t> k = feed(s, frame(1000000, 0x00FFA25DUL));
  CHECK(k.size() == 1 && k[0] == 0xA2);
  // Key "2" (0xFF18E7) is 0x18.
  k = feed(s, frame(2000000, 0x00FF18E7UL));
  CHECK(k.size() == 1 && k[0] == 0x18);

  // A held key: the repeat frame ~40 ms after the frame ends, then every
  // 108 ms, yields the command | 0x100.
  ir_nec_init(s);
  std::vector<uint32_t> f = frame(3000000, 0x00FF18E7UL);
  uint32_t end = f.back();
  k = feed(s, f);
  CHECK(k.size() == 1 && k[0] == 0x18);
  k = feed(s, repeat(end + 40000));
  CHECK(k.size() == 1 && k[0] == 0x118);
  k = feed(s, repeat(end + 40000 + 108000));
  CHECK(k.size() == 1 && k[0] == 0x118);
  k = feed(s, repeat(end + 40000 + 2 * 108000));
  CHECK(k.size() == 1 && k[0] == 0x118);
  // 0xA2 held: 0x1A2.
  f = frame(4000000, 0x00FFA25DUL);
  feed(s, f);
  k = feed(s, repeat(f.back() + 40000));
  CHECK(k.size() == 1 && k[0] == 0x1A2);
  // A repeat long after (key released, a stray repeat) is ignored.
  k = feed(s, repeat(f.back() + 40000 + 400000));
  CHECK(k.empty());

  // A lone repeat (no frame before it) yields nothing.
  ir_nec_init(s);
  k = feed(s, repeat(5000000));
  CHECK(k.empty());

  // A bad inverse byte (command or address) is rejected, and a repeat after
  // it yields nothing.
  ir_nec_init(s);
  f = frame(6000000, 0x00FFA25CUL);
  k = feed(s, f);
  CHECK(k.empty());
  k = feed(s, repeat(f.back() + 40000));
  CHECK(k.empty());
  k = feed(s, frame(7000000, 0x01FFA25DUL));
  CHECK(k.empty());
  // A good frame after a bad one still decodes.
  k = feed(s, frame(8000000, 0x00FFA25DUL));
  CHECK(k.size() == 1 && k[0] == 0xA2);

  // An out-of-range interval in the middle resets: the 5th data bit (a 0)
  // made 0.5 ms longer (1.625 ms, between the bit windows); the rest yields
  // nothing, and the next frame decodes.
  ir_nec_init(s);
  f = frame(9000000, 0x00FFA25DUL);
  for (size_t i = 6; i < f.size(); i++) f[i] += 500;
  k = feed(s, f);
  CHECK(k.empty());
  k = feed(s, frame(9200000, 0x00FFA25DUL));
  CHECK(k.size() == 1 && k[0] == 0xA2);

  // A timeout resets: a frame cut after 20 bits, 25 ms of silence, then the
  // remaining edges give nothing; the next frame decodes.
  ir_nec_init(s);
  f = frame(10000000, 0x00FFA25DUL);
  for (size_t i = 22; i < f.size(); i++) f[i] += 25000;
  k = feed(s, f);
  CHECK(k.empty());
  k = feed(s, frame(10300000, 0x00FF18E7UL));
  CHECK(k.size() == 1 && k[0] == 0x18);

  // Noise before the leader is ignored: short spikes and stray edges, then
  // a frame ("3").
  ir_nec_init(s);
  k = feed(s, {11000000, 11000300, 11000350, 11003000, 11009000});
  CHECK(k.empty());
  k = feed(s, frame(11030000, 0x00FF7A85UL));
  CHECK(k.size() == 1 && k[0] == 0x7A);
  // Noise just before the leader mark (3 ms): the frame still decodes ("4").
  ir_nec_init(s);
  feed(s, {12000000, 12000900});
  k = feed(s, frame(12003900, 0x00FF10EFUL));
  CHECK(k.size() == 1 && k[0] == 0x10);

  // Tolerance: every interval 10 % long still decodes ("6").
  ir_nec_init(s);
  f = frame(13000000, 0x00FF5AA5UL);
  std::vector<uint32_t> g = {f[0]};
  for (size_t i = 1; i < f.size(); i++) g.push_back(g.back() + (f[i] - f[i - 1]) * 11 / 10);
  k = feed(s, g);
  CHECK(k.size() == 1 && k[0] == 0x5A);

  // The microsecond clock wraps (every 71.6 min): a frame and a repeat
  // across the wrap.
  ir_nec_init(s);
  f = frame(0xFFFFFFFFUL - 30000, 0x00FFA25DUL);
  k = feed(s, f);
  CHECK(k.size() == 1 && k[0] == 0xA2);
  k = feed(s, repeat(f.back() + 40000));
  CHECK(k.size() == 1 && k[0] == 0x1A2);

  return report("ir");
}
