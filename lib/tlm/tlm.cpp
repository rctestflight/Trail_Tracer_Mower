#include "tlm.h"

#include <math.h>
#include <string.h>

#include "telemlog.h"  // streams and body limits

float g_tlm_value[TLM_COUNT + 1];

#ifdef TLM_HOST_TRACE
void tlm_trace(uint8_t body, uint32_t t, int id, float v);  // test/telemetry_table_host: each value as packed
#endif

namespace tlm {

namespace {

struct Desc {
  uint8_t type, rate;
  float inv_scale;
  double scale;
};
const Desc DESC[TLM_COUNT + 1] = {
#define TLM(name, type, scale, unit, decimals, rate, group, label)   {TLM_T_##type, TLM_R_##rate, (float)(1.0 / (scale)), 1.0 * (scale)},
#include "telemetry_table.h"
#undef TLM
    {0, 0xFF, 1.0f, 1.0}};

// The table hash: the CRC-32 (zlib) of the table text, computed by the compiler (no flash, no start-up time). The text
// is "tlm 2" and a line feed, then one line per table line in table order (TLR, TLF and TLM lines): a tag (R, F, V),
// then the columns joined by '|', the tokens as the # operator writes them and the strings as their text, then a line
// feed. tools/gen_telemetry_def.py builds the same text and writes the hash as "version" in telemetry_def.json.
constexpr const char* const HASH_LINES[] = {
    "tlm 2\n",
#define TLR(body, name, stream, rate, header, block, count_offset, block_offset, stride, min_bytes, consts)           \
  "R|" #body "|" #name "|" #stream "|" #rate "|" header "|" block "|" #count_offset "|" #block_offset "|" #stride "|" \
      #min_bytes "|" consts "\n",
#define TLF(layout, name, type, offset, scale, unit, decimals, kind, none, group, label, names)                     \
  "F|" #layout "|" #name "|" #type "|" #offset "|" #scale "|" unit "|" #decimals "|" #kind "|" #none "|" group "|" \
      label "|" names "\n",
#define TLM(name, type, scale, unit, decimals, rate, group, label) \
  "V|" #name "|" #type "|" #scale "|" unit "|" #decimals "|" #rate "|" group "|" label "\n",
#include "telemetry_table.h"
#undef TLR
#undef TLF
#undef TLM
    "", "", "", "", "", "", "", ""};  // 8 ends: crcLines takes 8 lines a step

// C++11 constexpr: one return statement, recursion. Depth: the longest line plus the line count / 8 (GCC limit 512).
constexpr uint32_t crcBits(uint32_t c, int k) { return k ? crcBits((c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))), k - 1) : c; }
constexpr uint32_t crcStr(const char* s, uint32_t c) { return *s ? crcStr(s + 1, crcBits(c ^ (uint8_t)*s, 8)) : c; }
constexpr uint32_t crcLines(int i, uint32_t c) {
  return HASH_LINES[i][0]
             ? crcLines(i + 8, crcStr(HASH_LINES[i + 7], crcStr(HASH_LINES[i + 6], crcStr(HASH_LINES[i + 5],
                               crcStr(HASH_LINES[i + 4], crcStr(HASH_LINES[i + 3], crcStr(HASH_LINES[i + 2],
                               crcStr(HASH_LINES[i + 1], crcStr(HASH_LINES[i], c)))))))))
             : c;
}
constexpr uint32_t TABLE_HASH = ~crcLines(0, 0xFFFFFFFFu);

struct Cls {
  uint8_t body;
  int stream, bytes, room;
  uint8_t* buf;  // the open record (heap; null for a rate with no value)
  int n, rate;
  uint32_t ms0, next;
  bool started;
};
Cls cls[3] = {
    {BODY_1HZ, TELEMLOG_SLOW, TLM_BYTES_1HZ, TELEMLOG_SLOW_BODY_MAX},
    {BODY_10HZ, TELEMLOG_FAST, TLM_BYTES_10HZ, TELEMLOG_FAST_BODY_MAX},
    {BODY_LOG, TELEMLOG_FAST, TLM_BYTES_LOG, TELEMLOG_FAST_BODY_MAX},
};
add_fn s_add = nullptr;

void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void put32(uint8_t* p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

// Round and clamp to [lo, hi]; NaN gives none. lo and hi are floats that the type holds exactly.
int32_t qi(float v, float lo, float hi, int32_t none) {
  if (!(v == v)) return none;
  if (v <= lo) return (int32_t)lo;
  if (v >= hi) return (int32_t)hi;
  return (int32_t)lroundf(v);
}

uint8_t* pack(uint8_t* p, uint8_t type, float v) {
  switch (type) {
    case TLM_T_U8: *p++ = (uint8_t)qi(v, 0.0f, 254.0f, 255); break;
    case TLM_T_I8: *p++ = (uint8_t)(int8_t)qi(v, -127.0f, 127.0f, -128); break;
    case TLM_T_U16: put16(p, (uint16_t)qi(v, 0.0f, 65534.0f, 65535)); p += 2; break;
    case TLM_T_I16: put16(p, (uint16_t)(int16_t)qi(v, -32767.0f, 32767.0f, -32768)); p += 2; break;
    case TLM_T_U32: {
      uint32_t x;
      if (!(v == v)) x = 0xFFFFFFFFu;
      else if (v <= 0.0f) x = 0;
      else if (v >= 4294967040.0f) x = 4294967040u;  // the largest float under 2^32 - 1
      else x = (uint32_t)lroundf(v);
      put32(p, x);
      p += 4;
      break;
    }
    case TLM_T_I32: put32(p, (uint32_t)qi(v, -2147483520.0f, 2147483520.0f, (int32_t)0x80000000u)); p += 4; break;
    default: {  // F32: the scaled value as it is (NaN = no value)
      uint32_t x;
      memcpy(&x, &v, 4);
      put32(p, x);
      p += 4;
    }
  }
  return p;
}

int perRecord(const Cls& c) {  // samples a record: one second of samples, fewer when the body is full
  const int fit = (c.room - HDR) / c.bytes;
  const int n = c.rate < fit ? c.rate : fit;
  return n < 1 ? 1 : n;
}

void flush(Cls& c) {
  if (c.n == 0) return;
  if (s_add) s_add(c.stream, c.ms0, c.buf, (size_t)(HDR + c.n * c.bytes));
  c.n = 0;
}

void sample(Cls& c, int ci, uint32_t t) {
  if (c.n == 0) {
    c.ms0 = t;
    c.buf[0] = c.body;
    put32(c.buf + 1, TABLE_HASH);
    c.buf[6] = (uint8_t)c.rate;
  }
  uint8_t* p = c.buf + HDR + c.n * c.bytes;
  for (int i = 0; i < TLM_COUNT; i++) {
    if (DESC[i].rate != ci) continue;
#ifdef TLM_HOST_TRACE
    tlm_trace(c.body, t, i, g_tlm_value[i]);
#endif
    p = pack(p, DESC[i].type, g_tlm_value[i] * DESC[i].inv_scale);
  }
  c.n++;
  c.buf[5] = (uint8_t)c.n;
  if (c.n >= perRecord(c)) flush(c);
}

}  // namespace

void init(add_fn add) {
  s_add = add;
  for (int i = 0; i <= TLM_COUNT; i++) g_tlm_value[i] = NAN;
  for (int ci = 0; ci < 3; ci++) {
    Cls& c = cls[ci];
    if (c.bytes == 0 || c.buf) continue;
    const int fit = (c.room - HDR) / c.bytes;
    const int top = ci == TLM_R_1HZ ? 1 : ci == TLM_R_10HZ ? 10 : 100;  // the highest rate of the class
    c.buf = new uint8_t[HDR + (fit < top ? fit : top) * c.bytes];
  }
}

uint32_t hash() { return TABLE_HASH; }

double logged(int id, float v) {
  if (id < 0 || id >= TLM_COUNT) return NAN;
  const Desc& d = DESC[id];
  uint8_t b[4];
  pack(b, d.type, v * d.inv_scale);
  const uint32_t u = (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
  double x;
  switch (d.type) {
    case TLM_T_U8: if (b[0] == 255) return NAN; x = b[0]; break;
    case TLM_T_I8: if (b[0] == 0x80) return NAN; x = (int8_t)b[0]; break;
    case TLM_T_U16: if ((uint16_t)u == 65535) return NAN; x = (uint16_t)u; break;
    case TLM_T_I16: if ((uint16_t)u == 0x8000) return NAN; x = (int16_t)(uint16_t)u; break;
    case TLM_T_U32: if (u == 0xFFFFFFFFu) return NAN; x = u; break;
    case TLM_T_I32: if (u == 0x80000000u) return NAN; x = (int32_t)u; break;
    default: {  // F32
      float f;
      memcpy(&f, &u, 4);
      x = f;  // NaN stays NaN
    }
  }
  return x * d.scale;
}
int count() { return TLM_COUNT; }

void service(uint32_t now, int log_rate_hz) {
  for (int ci = 0; ci < 3; ci++) {
    Cls& c = cls[ci];
    if (c.bytes == 0 || !c.buf) continue;
    int rate = ci == TLM_R_1HZ ? 1 : ci == TLM_R_10HZ ? 10 : log_rate_hz;
    if (rate < 1) rate = 1;
    if (rate > 100) rate = 100;
    if (rate != c.rate) {  // a new log rate: the open record goes out as it is
      flush(c);
      c.rate = rate;
      c.started = false;
    }
    const uint32_t per = 1000u / (uint32_t)rate;
    if (!c.started || (int32_t)(now - c.next) >= (int32_t)(2 * per)) {
      // Start, or more than one sample late (a long loop pass): the open record goes out, the next one starts now.
      flush(c);
      c.next = now;
      c.started = true;
    }
    for (int k = 0; k < 2 && (int32_t)(now - c.next) >= 0; k++) {
      sample(c, ci, c.next);
      c.next += per;
    }
  }
}

}  // namespace tlm
