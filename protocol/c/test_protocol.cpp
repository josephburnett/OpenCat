// Host tests for the C protocol codec against protocol/vectors/*.json.
//
//   test_protocol VECTORS_DIR [FUZZ_ITERS]   run all vector tests + fuzz
//   test_protocol --emit FILE N SEED         write N random valid frames
//   test_protocol --check FILE               decode frames written by Python
//
// Line format for --emit/--check: "<type> <seq> <payload hex|-> <wire hex>".
// Build: g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined
#define KILO_PROTOCOL_FIELD_TABLE
#include "cobs.h"
#include "crc16.h"
#include "kilo_frame.h"
#include "kilo_protocol.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal JSON reader (objects keep key order; numbers are integers).

struct Json {
    enum Kind { NUL, BOOL, NUM, STR, ARR, OBJ } kind = NUL;
    long long num = 0;
    bool b = false;
    std::string str;
    std::vector<Json> arr;
    std::vector<std::pair<std::string, Json>> obj;

    const Json *get(const std::string &k) const {
        for (auto &p : obj)
            if (p.first == k) return &p.second;
        return nullptr;
    }
    const Json &operator[](const std::string &k) const {
        const Json *j = get(k);
        if (!j) {
            fprintf(stderr, "missing key %s\n", k.c_str());
            exit(2);
        }
        return *j;
    }
};

class JsonParser {
  public:
    explicit JsonParser(const std::string &s) : s_(s) {}
    Json parse() {
        Json j = value();
        ws();
        if (i_ != s_.size()) fail("trailing data");
        return j;
    }

  private:
    const std::string &s_;
    size_t i_ = 0;
    [[noreturn]] void fail(const char *m) {
        fprintf(stderr, "json: %s at %zu\n", m, i_);
        exit(2);
    }
    void ws() {
        while (i_ < s_.size() && strchr(" \t\r\n", s_[i_])) i_++;
    }
    char peek() {
        ws();
        if (i_ >= s_.size()) fail("eof");
        return s_[i_];
    }
    void expect(char c) {
        if (peek() != c) fail("unexpected char");
        i_++;
    }
    std::string string() {
        expect('"');
        std::string out;
        while (i_ < s_.size() && s_[i_] != '"') {
            if (s_[i_] == '\\') {
                i_++;
                char c = s_[i_++];
                switch (c) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'u': out += '?'; i_ += 4; break;
                default: out += c;
                }
            } else {
                out += s_[i_++];
            }
        }
        expect('"');
        return out;
    }
    Json value() {
        Json j;
        char c = peek();
        if (c == '{') {
            j.kind = Json::OBJ;
            i_++;
            if (peek() == '}') { i_++; return j; }
            for (;;) {
                std::string k = string();
                expect(':');
                j.obj.emplace_back(k, value());
                if (peek() == ',') { i_++; continue; }
                expect('}');
                return j;
            }
        }
        if (c == '[') {
            j.kind = Json::ARR;
            i_++;
            if (peek() == ']') { i_++; return j; }
            for (;;) {
                j.arr.push_back(value());
                if (peek() == ',') { i_++; continue; }
                expect(']');
                return j;
            }
        }
        if (c == '"') {
            j.kind = Json::STR;
            j.str = string();
            return j;
        }
        if (s_.compare(i_, 4, "true") == 0) { j.kind = Json::BOOL; j.b = true; i_ += 4; return j; }
        if (s_.compare(i_, 5, "false") == 0) { j.kind = Json::BOOL; i_ += 5; return j; }
        if (s_.compare(i_, 4, "null") == 0) { i_ += 4; return j; }
        size_t st = i_;
        if (s_[i_] == '-') i_++;
        while (i_ < s_.size() && isdigit((unsigned char)s_[i_])) i_++;
        if (st == i_) fail("bad value");
        j.kind = Json::NUM;
        j.num = std::stoll(s_.substr(st, i_ - st));
        return j;
    }
};

static Json load_json(const std::string &path) {
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "cannot open %s\n", path.c_str());
        exit(2);
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string s = ss.str();
    return JsonParser(s).parse();
}

// ---------------------------------------------------------------------------

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        g_checks++;                                                        \
        if (!(cond)) {                                                     \
            g_fail++;                                                      \
            fprintf(stderr, "FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
            fprintf(stderr, __VA_ARGS__);                                  \
            fprintf(stderr, "\n");                                         \
        }                                                                  \
    } while (0)

using Bytes = std::vector<uint8_t>;

static Bytes unhex(const std::string &h) {
    Bytes b;
    for (size_t i = 0; i + 1 < h.size(); i += 2) b.push_back((uint8_t)std::stoi(h.substr(i, 2), nullptr, 16));
    return b;
}

static std::string hex(const uint8_t *p, size_t n) {
    static const char *d = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; i++) {
        s += d[p[i] >> 4];
        s += d[p[i] & 15];
    }
    return s;
}
static std::string hex(const Bytes &b) { return hex(b.data(), b.size()); }

static const char *status_name(uint8_t st) {
    switch (st) {
    case KILO_FRAME_OK: return "ok";
    case KILO_FRAME_BAD_COBS: return "bad_cobs";
    case KILO_FRAME_BAD_CRC: return "bad_crc";
    case KILO_FRAME_BAD_LENGTH: return "bad_length";
    case KILO_FRAME_BAD_PAYLOAD_LEN: return "bad_length";
    case KILO_FRAME_UNKNOWN_TYPE: return "unknown_type";
    case KILO_FRAME_OVERFLOW: return "overflow";
    default: return "none";
    }
}

static const char *kind_name(uint8_t k) {
    switch (k) {
    case KILO_KIND_GOAL: return "goal";
    case KILO_KIND_COMMAND: return "command";
    case KILO_KIND_STREAM: return "stream";
    case KILO_KIND_EVENT: return "event";
    default: return "unknown";
    }
}

static long long read_field(const uint8_t *p, uint8_t size, bool is_signed) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < size; i++) v |= (uint32_t)p[i] << (8 * i);
    if (!is_signed) return v;
    if (size == 1) return (int8_t)v;
    if (size == 2) return (int16_t)v;
    return (int32_t)v;
}

// Decode a wire string (with trailing 0x00) through kilo_frame_decode.
static uint8_t decode_wire(const Bytes &wire, Bytes &scratch, kilo_frame_t &f) {
    scratch = wire;
    if (!scratch.empty() && scratch.back() == 0) scratch.pop_back();
    if (scratch.size() > 255) return KILO_FRAME_BAD_LENGTH;
    scratch.reserve(scratch.size() + 1);
    return kilo_frame_decode(scratch.data(), (uint8_t)scratch.size(), &f);
}

static void test_crc(const Json &frames) {
    const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(kilo_crc16(s, 9) == 0x29B1, "crc %04x", kilo_crc16(s, 9));
    CHECK(kilo_crc16(s, 9) == (uint16_t)frames["crc_check"]["crc"].num, "vector check value");
    CHECK(KILO_CRC_CHECK == 0x29B1u, "constant");
    CHECK(kilo_crc16(s, 0) == 0xFFFF, "empty");
}

static void test_cobs(const Json &cobs) {
    int n = 0;
    for (auto &v : cobs["vectors"].arr) {
        Bytes data = unhex(v["data"].str), enc = unhex(v["encoded"].str);
        Bytes out(KILO_COBS_MAX_ENCODED(data.size()) + 1);
        uint16_t en = kilo_cobs_encode(data.data(), (uint16_t)data.size(), out.data());
        out.resize(en);
        CHECK(out == enc, "cobs encode %s: %s != %s", v["source"].str.c_str(), hex(out).substr(0, 40).c_str(),
              hex(enc).substr(0, 40).c_str());
        CHECK(en <= KILO_COBS_MAX_ENCODED(data.size()), "bound");
        for (uint8_t b : out) CHECK(b != 0, "zero in output");
        Bytes dec(enc.size() + 1);
        int16_t dn = kilo_cobs_decode(enc.data(), (uint16_t)enc.size(), dec.data());
        CHECK(dn == (int16_t)data.size(), "cobs decode len %d", dn);
        if (dn >= 0) {
            dec.resize(dn);
            CHECK(dec == data, "cobs decode %s", v["source"].str.c_str());
        }
        Bytes inplace = enc;  // in-place decode
        dn = kilo_cobs_decode(inplace.data(), (uint16_t)inplace.size(), inplace.data());
        CHECK(dn == (int16_t)data.size() && std::equal(data.begin(), data.end(), inplace.begin()), "in place");
        n++;
    }
    printf("  cobs vectors: %d\n", n);
}

static void check_fields(const Json &v, const kilo_frame_t &f) {
    const Json &fields = v["fields"];
    for (size_t i = 0; i < KILO_FIELD_TABLE_LEN; i++) {
        const kilo_field_info_t &fi = kilo_field_table[i];
        if (fi.msg != f.type) continue;
        const Json *jv = fields.get(fi.name);
        if (!jv) continue;  // derived count fields are not in the Python dict
        if (fi.count == 1) {
            long long got = read_field(f.payload + fi.offset, fi.size, fi.is_signed);
            CHECK(got == jv->num, "%s.%s: %lld != %lld", v["msg"].str.c_str(), fi.name, got, jv->num);
        } else {
            CHECK(jv->arr.size() == fi.count, "array size");
            for (size_t k = 0; k < jv->arr.size() && k < fi.count; k++) {
                long long got = read_field(f.payload + fi.offset + k * fi.size, fi.size, fi.is_signed);
                CHECK(got == jv->arr[k].num, "%s.%s[%zu]", v["msg"].str.c_str(), fi.name, k);
            }
        }
    }
}

static void test_frames(const Json &frames) {
    int n = 0;
    std::map<int, int> seen;
    for (auto &v : frames["frames"].arr) {
        uint8_t type = (uint8_t)v["type"].num, seq = (uint8_t)v["seq"].num;
        Bytes payload = unhex(v["payload"].str), wire = unhex(v["wire"].str), decoded = unhex(v["decoded"].str);
        const char *name = v["name"].str.c_str();
        seen[type]++;
        // encode
        uint8_t w[KILO_MAX_WIRE];
        uint8_t wn = kilo_frame_encode(type, seq, payload.data(), (uint8_t)payload.size(), w);
        CHECK(Bytes(w, w + wn) == wire, "%s: encode %s != %s", name, hex(w, wn).c_str(), hex(wire).c_str());
        CHECK(wn == payload.size() + KILO_FRAME_OVERHEAD, "%s: wire = payload + 6", name);
        // decoded form
        Bytes dec(wire.size());
        int16_t dn = kilo_cobs_decode(wire.data(), (uint16_t)(wire.size() - 1), dec.data());
        CHECK(dn >= 0 && Bytes(dec.begin(), dec.begin() + dn) == decoded, "%s: decoded", name);
        // decode
        Bytes scratch;
        kilo_frame_t f;
        uint8_t st = decode_wire(wire, scratch, f);
        CHECK(st == KILO_FRAME_OK, "%s: status %s", name, status_name(st));
        if (st != KILO_FRAME_OK) continue;
        CHECK(f.type == type && f.seq == seq, "%s: header", name);
        CHECK(Bytes(f.payload, f.payload + f.len) == payload, "%s: payload", name);
        CHECK(std::string(kind_name(kilo_msg_kind(type))) == v["kind"].str, "%s: kind", name);
        bool first_is_epoch = !v["fields"].obj.empty() && v["fields"].obj[0].first == "epoch";
        if (v["dir"].str == "pi_to_body")
            CHECK((kilo_msg_has_epoch(type) != 0) == first_is_epoch, "%s: epoch flag", name);
        else
            CHECK(kilo_msg_has_epoch(type) == 0, "%s: epoch flag", name);
        CHECK(v["dir"].str == (type < 0x80 ? "pi_to_body" : "body_to_pi"), "%s: dir", name);
        check_fields(v, f);
        n++;
    }
    // every message type is covered
    for (size_t i = 0; i < KILO_MSG_TABLE_LEN; i++)
        CHECK(seen[kilo_msg_table[i].id] > 0, "no vector for %s", kilo_msg_table[i].name);
    printf("  frame vectors: %d (types covered: %zu)\n", n, seen.size());

    // Typed struct access on a few vectors (memcpy into the packed structs).
    for (auto &v : frames["frames"].arr) {
        Bytes p = unhex(v["payload"].str);
        if (v["msg"].str == "TELEMETRY") {
            kilo_telemetry_t t;
            CHECK(p.size() == sizeof t, "telemetry size");
            memcpy(&t, p.data(), sizeof t);
            CHECK(t.tick == v["fields"]["tick"].num && t.pitch == v["fields"]["pitch"].num &&
                      t.flags == v["fields"]["flags"].num && t.epoch == v["fields"]["epoch"].num,
                  "telemetry struct");
        } else if (v["msg"].str == "PARAM_VALUE") {
            kilo_param_value_t pv;
            memcpy(&pv, p.data(), sizeof pv);
            CHECK(pv.default_ == v["fields"]["default"].num && pv.min == v["fields"]["min"].num, "param struct");
        } else if (v["name"].str == "trick_frames_4_max") {
            kilo_trick_frames_t tf;
            CHECK(p.size() == KILO_TRICK_FRAMES_LEN_FOR(4) && p.size() == KILO_TRICK_FRAMES_MAX_LEN, "tf len");
            memcpy(&tf, p.data(), p.size());
            CHECK(tf.count == 4 && tf.frames[1].ang[0] == -128 &&
                      tf.frames[3].dur == v["fields"]["frames"].arr[3]["dur"].num,
                  "trick frames struct");
        } else if (v["msg"].str == "FEET") {
            kilo_feet_t ft;
            memcpy(&ft, p.data(), sizeof ft);
            CHECK(ft.epoch == v["fields"]["epoch"].num && ft.tick == v["fields"]["tick"].num &&
                      ft.x_mm[3] == v["fields"]["x_mm"].arr[3].num && ft.z_mm[2] == v["fields"]["z_mm"].arr[2].num &&
                      ft.roll == v["fields"]["roll"].num && ft.support == v["fields"]["support"].num,
                  "feet struct");
        }
    }
}

static void test_errors(const Json &frames) {
    int n = 0;
    for (auto &v : frames["errors"].arr) {
        Bytes wire = unhex(v["wire"].str), scratch;
        kilo_frame_t f;
        uint8_t st = decode_wire(wire, scratch, f);
        const char *name = v["name"].str.c_str();
        CHECK(v["error"].str == status_name(st), "%s: got %s want %s", name, status_name(st), v["error"].str.c_str());
        if (v.get("type")) {
            CHECK(st == KILO_FRAME_BAD_PAYLOAD_LEN || st == KILO_FRAME_UNKNOWN_TYPE, "%s: trusted header", name);
            CHECK(f.type == v["type"].num && f.seq == v["seq"].num, "%s: header", name);
        } else {
            CHECK(st != KILO_FRAME_BAD_PAYLOAD_LEN && st != KILO_FRAME_UNKNOWN_TYPE, "%s: untrusted", name);
            CHECK(f.len == 0 && f.payload == nullptr, "%s: cleared", name);
        }
        n++;
    }
    printf("  error vectors: %d\n", n);
}

static void test_streams(const Json &frames) {
    int n = 0;
    for (auto &v : frames["streams"].arr) {
        Bytes bytes = unhex(v["bytes"].str);
        kilo_rx_t rx;
        kilo_rx_init(&rx);
        std::vector<std::string> got, want;
        for (uint8_t b : bytes) {
            kilo_frame_t f;
            uint8_t st = kilo_rx_push(&rx, b, &f);
            if (st == KILO_FRAME_NONE) continue;
            char buf[64];
            if (st == KILO_FRAME_OK)
                snprintf(buf, sizeof buf, "ok %d %d", f.type, f.seq);
            else
                snprintf(buf, sizeof buf, "error %s", status_name(st));
            got.push_back(buf);
        }
        for (auto &e : v["expect"].arr) {
            char buf[64];
            if (e.get("ok"))
                snprintf(buf, sizeof buf, "ok %lld %lld", e["ok"].num, e["seq"].num);
            else
                snprintf(buf, sizeof buf, "error %s", e["error"].str.c_str());
            want.push_back(buf);
        }
        CHECK(got == want, "stream %s", v["name"].str.c_str());
        n++;
    }
    printf("  stream vectors: %d\n", n);
}

// Random valid frame from the message table.
static void random_frame(std::mt19937 &rng, uint8_t &type, uint8_t &seq, Bytes &payload) {
    const kilo_msg_info_t &m = kilo_msg_table[rng() % KILO_MSG_TABLE_LEN];
    type = m.id;
    seq = (uint8_t)rng();
    unsigned len = m.fixed_len;
    unsigned k = 0;
    if (m.elem_size) {
        k = m.tail_min + rng() % (m.tail_max - m.tail_min + 1u);
        len += m.elem_size * k;
    }
    payload.resize(len);
    bool zeros = rng() % 2;
    for (auto &b : payload) b = (uint8_t)((zeros && rng() % 3 == 0) ? 0 : rng());
    if (m.count_offset != 0xFF) payload[m.count_offset] = (uint8_t)k;
}

static void fuzz(unsigned iters) {
    std::mt19937 rng(12345);
    kilo_rx_t rx;
    kilo_rx_init(&rx);
    unsigned ok = 0, err = 0, rt = 0;
    for (unsigned it = 0; it < iters; it++) {
        uint8_t type, seq;
        Bytes payload;
        random_frame(rng, type, seq, payload);
        uint8_t w[KILO_MAX_WIRE];
        uint8_t wn = kilo_frame_encode(type, seq, payload.data(), (uint8_t)payload.size(), w);
        // clean round trip through the frame decoder
        {
            Bytes scratch(w, w + wn), dummy;
            kilo_frame_t f;
            uint8_t st = decode_wire(scratch, dummy, f);
            CHECK(st == KILO_FRAME_OK && f.type == type && f.seq == seq &&
                      Bytes(f.payload, f.payload + f.len) == payload,
                  "roundtrip");
            rt++;
        }
        // mutate and push through the stream receiver
        Bytes m(w, w + wn);
        switch (rng() % 6) {
        case 0: break;
        case 1: m[rng() % m.size()] ^= (uint8_t)(1u << (rng() % 8)); break;
        case 2: m.erase(m.begin() + rng() % m.size()); break;
        case 3: m.insert(m.begin() + rng() % m.size(), (uint8_t)rng()); break;
        case 4: for (unsigned i = 0, r = rng() % 80; i < r; i++) m.insert(m.begin(), (uint8_t)rng()); break;
        case 5: m.resize(rng() % m.size()); break;
        }
        for (uint8_t b : m) {
            kilo_frame_t f;
            uint8_t st = kilo_rx_push(&rx, b, &f);
            if (st == KILO_FRAME_OK) {
                ok++;
                CHECK(kilo_payload_len_check(f.type, f.payload, f.len) == KILO_LEN_OK, "accepted bad len");
            } else if (st != KILO_FRAME_NONE) {
                err++;
            }
        }
    }
    printf("  fuzz: %u iterations, %u round trips, %u frames accepted, %u rejected\n", iters, rt, ok, err);
}

static int emit(const char *path, unsigned n, unsigned seed) {
    std::mt19937 rng(seed);
    FILE *fp = fopen(path, "w");
    if (!fp) return 2;
    for (unsigned i = 0; i < n; i++) {
        uint8_t type, seq;
        Bytes payload;
        random_frame(rng, type, seq, payload);
        uint8_t w[KILO_MAX_WIRE];
        uint8_t wn = kilo_frame_encode(type, seq, payload.data(), (uint8_t)payload.size(), w);
        fprintf(fp, "%u %u %s %s\n", type, seq, payload.empty() ? "-" : hex(payload).c_str(), hex(w, wn).c_str());
    }
    fclose(fp);
    printf("emitted %u frames to %s\n", n, path);
    return 0;
}

static int check_file(const char *path) {
    std::ifstream f(path);
    if (!f) return 2;
    std::string line;
    unsigned n = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        unsigned type, seq;
        std::string ph, wh;
        ss >> type >> seq >> ph >> wh;
        Bytes payload = ph == "-" ? Bytes() : unhex(ph), wire = unhex(wh), scratch;
        kilo_frame_t fr;
        uint8_t st = decode_wire(wire, scratch, fr);
        CHECK(st == KILO_FRAME_OK && fr.type == type && fr.seq == seq &&
                  Bytes(fr.payload, fr.payload + fr.len) == payload,
              "python frame %u: %s", n, status_name(st));
        uint8_t w[KILO_MAX_WIRE];
        uint8_t wn = kilo_frame_encode((uint8_t)type, (uint8_t)seq, payload.data(), (uint8_t)payload.size(), w);
        CHECK(Bytes(w, w + wn) == wire, "python frame %u: re-encode", n);
        n++;
    }
    printf("checked %u Python-encoded frames: %s\n", n, g_fail ? "FAIL" : "ok");
    return g_fail ? 1 : (n ? 0 : 1);
}

int main(int argc, char **argv) {
    if (argc >= 5 && std::string(argv[1]) == "--emit")
        return emit(argv[2], (unsigned)atoi(argv[3]), (unsigned)atoi(argv[4]));
    if (argc >= 3 && std::string(argv[1]) == "--check") return check_file(argv[2]);
    if (argc < 2) {
        fprintf(stderr, "usage: %s VECTORS_DIR [FUZZ_ITERS] | --emit FILE N SEED | --check FILE\n", argv[0]);
        return 2;
    }
    std::string dir = argv[1];
    unsigned iters = argc >= 3 ? (unsigned)atoi(argv[2]) : 1000000u;
    Json frames = load_json(dir + "/frames.json");
    Json cobs = load_json(dir + "/cobs.json");
    printf("C protocol tests\n");
    test_crc(frames);
    test_cobs(cobs);
    test_frames(frames);
    test_errors(frames);
    test_streams(frames);
    fuzz(iters);
    printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
