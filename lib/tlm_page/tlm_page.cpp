#include "tlm_page.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rover_records.h"
#include "telemlog.h"

static_assert(tlmpage::STATUS_MAX == rec::STATUS_BYTES, "tlm_page: STATUS_MAX is not the status record size");
static_assert(tlmpage::SAMPLE_MAX == rec::IMU_SAMPLE3_BYTES, "tlm_page: SAMPLE_MAX is not the largest sample");

namespace tlmpage {
namespace {

enum Src : uint8_t { S_STATUS, S_SAMPLE, S_IMU, S_TLM };
enum Kind : uint8_t { K_VALUE, K_BITS, K_ENUM, K_TEXT, K_TIME, K_HEX };

// One table line, as the X-macro gives it. Types, names and "no value" codes stay as their text (the # operator) and
// are read when a value is formatted.
struct Line {
  uint8_t src, kind, dec;
  uint16_t off;  // byte offset; S_TLM: the value id
  const char* name;
  const char* type;
  double scale;
  const char* unit;
  const char* none;
  const char* group;
  const char* label;
  const char* names;
};

// The layouts on the page. Every layout of the table needs a line here: a new layout stops the build until it is
// listed (kept or dropped).
#define TP_STATUS(...) __VA_ARGS__
#define TP_SAMPLE(...) __VA_ARGS__
#define TP_SAMPLE3(...) __VA_ARGS__
#define TP_IMU(...) __VA_ARGS__
#define TP_DEC(...)
#define TP_BINS(...)
#define TP_EVENT(...)
#define TP_NAME(...)
#define TP_SWEEP_RAW(...)
#define TP_SWEEP(...)
const uint8_t SRC_STATUS = S_STATUS, SRC_SAMPLE = S_SAMPLE, SRC_SAMPLE3 = S_SAMPLE, SRC_IMU = S_IMU;

const Line LINES[] = {
#define TLF(layout, name, type, offset, scale, unit, decimals, kind, none, group, label, names) \
  TP_##layout({SRC_##layout, K_##kind, decimals, offset, #name, #type, 1.0 * scale, unit, #none, group, label, names}, )
#define TLM(name, type, scale, unit, decimals, rate, group, label) \
  {S_TLM, K_VALUE, decimals, TLM_##name, #name, #type, 1.0 * (scale), unit, "-", group, label, ""},
#include "telemetry_table.h"
#undef TLF
#undef TLM
};
const int N_LINES = sizeof(LINES) / sizeof(LINES[0]);

// The short set: the values a user looks at first, in this order; every table value (TLM line) follows. The names
// are table names; the words come from the table.
const char* const SHORT[] = {"x1_mm",          "y1_mm",       "r1_mm",        "sample_flags",   "drive",
                             "steer",          "vbatt_mv",    "drive_mode",   "vehicle_status", "ap_state",
                             "temperature_c",  "dsp_period_us", "packets_last_s", "crc_errors", "cpu_busy_pct",
                             "free_heap"};
const int N_SHORT = sizeof(SHORT) / sizeof(SHORT[0]);

// Name parts: base length and array dimensions ("axis_state[6]" -> 10, {6}).
int dims(const char* name, int* base, int d[2]) {
  const char* b = strchr(name, '[');
  *base = b ? (int)(b - name) : (int)strlen(name);
  int n = 0;
  d[0] = d[1] = 1;
  while (b && n < 2) {
    d[n++] = atoi(b + 1);
    b = strchr(b + 1, '[');
  }
  return n;
}

bool same_base(const char* a, const char* b) {
  int la, lb, d[2];
  dims(a, &la, d);
  dims(b, &lb, d);
  return la == lb && !strncmp(a, b, la);
}

// A line on the page: not a TIME or HEX field, not a text to the body end, not a status field the sample also has.
bool shown_calc(int li) {
  const Line& L = LINES[li];
  if (L.kind == K_TIME || L.kind == K_HEX) return false;
  if (!strcmp(L.type, "TEXT") || !strcmp(L.type, "PTEXT")) return false;
  if (L.src == S_STATUS)
    for (int j = 0; j < N_LINES; j++)
      if ((LINES[j].src == S_SAMPLE || LINES[j].src == S_IMU) && same_base(L.name, LINES[j].name)) return false;
  return true;
}

// Worked out once (the first request), one bit a line.
uint8_t s_shown[(N_LINES + 7) / 8];
bool s_shown_set = false;
bool shown(int li) {
  if (!s_shown_set) {
    for (int l = 0; l < N_LINES; l++)
      if (shown_calc(l)) s_shown[l / 8] |= (uint8_t)(1 << (l % 8));
    s_shown_set = true;
  }
  return (s_shown[li / 8] >> (li % 8)) & 1;
}

int elements(int li) {
  int base, d[2];
  dims(LINES[li].name, &base, d);
  return d[0] * d[1];
}

// Field i -> line and element. false: past the end.
bool locate(int i, int* li, int* el) {
  for (int l = 0; l < N_LINES; l++) {
    if (!shown(l)) continue;
    const int n = elements(l);
    if (i < n) { *li = l; *el = i; return true; }
    i -= n;
  }
  return false;
}

// The field name: name, or name_i, name_i_j for an array element.
void field_name(int li, int el, char* out, size_t cap) {
  int base, d[2];
  const int nd = dims(LINES[li].name, &base, d);
  if (nd == 0) snprintf(out, cap, "%.*s", base, LINES[li].name);
  else if (nd == 1) snprintf(out, cap, "%.*s_%d", base, LINES[li].name, el);
  else snprintf(out, cap, "%.*s_%d_%d", base, LINES[li].name, el / d[1], el % d[1]);
}

int place(int li, int el) {
  if (LINES[li].src == S_TLM) {
    int k = 0;
    for (int j = 0; j < li; j++) k += LINES[j].src == S_TLM;
    return N_SHORT + 1 + k;
  }
  char n[48];
  field_name(li, el, n, sizeof(n));
  for (int k = 0; k < N_SHORT; k++)
    if (!strcmp(n, SHORT[k])) return k + 1;
  return 0;
}

// Output in pieces of up to 512 B to the sink.
struct Out {
  sink_fn sink;
  void* ctx;
  bool ok = true;
  size_t n = 0;
  char buf[512];
  Out(sink_fn s, void* c) : sink(s), ctx(c) {}
  void flush() {
    if (ok && n) ok = sink(ctx, buf, n);
    n = 0;
  }
  void put(const char* s, size_t len) {
    while (len) {
      if (n == sizeof(buf)) flush();
      size_t k = sizeof(buf) - n;
      if (k > len) k = len;
      memcpy(buf + n, s, k);
      n += k;
      s += k;
      len -= k;
    }
  }
  void str(const char* s) { put(s, strlen(s)); }
  void fmt(const char* f, ...) {
    char t[96];
    va_list a;
    va_start(a, f);
    const int k = vsnprintf(t, sizeof(t), f, a);
    va_end(a);
    if (k > 0) put(t, (size_t)k < sizeof(t) ? (size_t)k : sizeof(t) - 1);
  }
  void esc(const char* s, bool quotes = true) {  // a JSON string, with its quotes or (quotes false) without
    if (quotes) put("\"", 1);
    for (int i = 0; s[i] != 0; i++) {
      const char c = s[i];
      if (c == '"' || c == '\\') { put("\\", 1); put(&c, 1); }
      else if ((unsigned char)c < 0x20) fmt("\\u%04x", (unsigned)c);
      else put(&c, 1);
    }
    if (quotes) put("\"", 1);
  }
};

// The label part of element el: the names column, "|"-separated, one part per element of the last index.
void label_part(const Line& L, int el, Out& o) {
  int base, d[2];
  const int nd = dims(L.name, &base, d);
  const int k = nd == 2 ? el % d[1] : el;
  const char* p = L.names;
  for (int i = 0; i < k && p; i++) {
    p = strchr(p, '|');
    if (p) p++;
  }
  if (!p) return;
  const char* e = strchr(p, '|');
  char t[48];
  snprintf(t, sizeof(t), ", %.*s", (int)(e ? e - p : (long)strlen(p)), p);
  o.esc(t, false);
}

struct Type { int size; bool sign; int shift, mask; char t; };  // t: i integer, f F32, b BF16, v VERSION, u UNIXMS

Type parse_type(const char* s) {
  Type r = {0, false, 0, -1, 'i'};
  if (!strncmp(s, "BF16", 4)) { r.size = 2; r.t = 'b'; return r; }
  if (!strncmp(s, "F32", 3)) { r.size = 4; r.t = 'f'; return r; }
  if (!strncmp(s, "VERSION", 7)) { r.size = 3; r.t = 'v'; return r; }
  if (!strncmp(s, "UNIXMS", 6)) { r.size = 6; r.t = 'u'; return r; }
  r.sign = s[0] == 'I';
  r.size = atoi(s + 1) / 8;
  const char* p = s + 1;
  while (*p >= '0' && *p <= '9') p++;
  if (p[0] == '>' && p[1] == '>') r.shift = atoi(p + 2);
  else if (p[0] == '&') r.mask = atoi(p + 1);
  return r;
}

uint32_t le(const uint8_t* p, int n) {
  uint32_t v = 0;
  for (int i = n - 1; i >= 0; i--) v = (v << 8) | p[i];
  return v;
}

// x times the scale, as tools/rover_log.py does it: a scale of 1/k (k whole) divides by k.
double scaled(double x, double s) {
  if (s == 1.0) return x;
  if (s == floor(s)) return x * s;
  const double inv = 1.0 / s, k = floor(inv + 0.5);
  if (fabs(inv - k) < 1e-9 * k) return x / k;
  return x * s;
}

void number(Out& o, double v, int dec) {
  if (!(v == v) || isinf(v)) { o.str("null"); return; }
  char t[40];
  if (fabs(v) >= 1e15) snprintf(t, sizeof(t), "%.6g", v);
  else snprintf(t, sizeof(t), "%.*f", dec, v);
  const char* p = t;
  if (t[0] == '-' && strspn(t + 1, "0.") == strlen(t + 1)) p++;  // "-0.00" -> "0.00"
  o.str(p);
}

void u64(Out& o, uint64_t v) {
  char t[24];
  int i = sizeof(t) - 1;
  t[i] = 0;
  do { t[--i] = (char)('0' + v % 10); v /= 10; } while (v);
  o.str(t + i);
}

// The JSON value of field (li, el) from the snapshot.
void value(Out& o, const Snapshot& s, int li, int el) {
  const Line& L = LINES[li];
  if (L.src == S_TLM) {
    number(o, tlm::logged(L.off, s.tlm[L.off]), L.dec);
    return;
  }
  const uint8_t* b;
  int len, off = L.off;
  if (L.src == S_STATUS) { b = s.status; len = s.status_len; }
  else {
    b = s.sample;
    len = s.sample_len;
    if (L.src == S_IMU) {
      if (!s.sample_imu) len = 0;
      off += rec::SAMPLE3_BYTES;
    }
  }
  const Type t = parse_type(L.type);
  off += el * t.size;
  if (off + t.size > len) { o.str("null"); return; }
  const uint8_t* p = b + off;
  if (t.t == 'v') { o.fmt("\"%u.%u.%u\"", p[0], p[1], p[2]); return; }
  const bool has_none = strcmp(L.none, "-") != 0;
  const long none = has_none ? strtol(L.none, nullptr, 10) : 0;
  if (t.t == 'u') {
    const uint32_t sec = le(p, 4);
    if (has_none && (long)sec == none) { o.str("null"); return; }
    u64(o, (uint64_t)sec * 1000u + le(p + 4, 2));
    return;
  }
  if (t.t == 'f' || t.t == 'b') {
    const uint32_t w = t.t == 'f' ? le(p, 4) : le(p, 2) << 16;
    float f;
    memcpy(&f, &w, 4);
    number(o, f == f ? scaled(f, L.scale) : f, L.dec);
    return;
  }
  uint32_t u = le(p, t.size);
  int64_t x = u;
  if (t.sign && (u >> (8 * t.size - 1)) & 1) x = (int64_t)u - ((int64_t)1 << (8 * t.size));
  if (has_none && x == none) { o.str("null"); return; }
  if (t.shift) x >>= t.shift;
  if (t.mask >= 0) x &= t.mask;
  if (L.kind == K_BITS || L.kind == K_ENUM) { o.fmt("%ld", (long)x); return; }
  number(o, scaled((double)x, L.scale), L.dec);
}

const char* KIND[] = {"v", "bits", "enum", "text", "time", "hex"};
const char* REC[] = {"status", "samples", "samples", "tlm"};

}  // namespace

void take_status(Snapshot* s, const uint8_t* body, size_t len, uint32_t ms) {
  if (len < 1 || body[0] != rec::BODY_STATUS) return;
  if (--len > (size_t)STATUS_MAX) len = STATUS_MAX;
  memcpy(s->status, body + 1, len);
  s->status_len = (uint8_t)len;
  s->status_ms = ms;
}

void take_status_log(Snapshot* s) {
  telemlog_info_t li;
  telemlog_info(&li);
  const telemlog_range_t& r = li.r[TELEMLOG_SLOW][0];
  if (r.newest == TELEMLOG_NO_SEQ || r.oldest > r.newest) return;
  uint8_t b[TELEMLOG_SLOW_BODY_MAX];
  for (uint32_t q = r.newest, k = 0; k < TELEMLOG_SLOW_SLOTS; q--, k++) {
    uint32_t ms = 0;
    const size_t n = telemlog_get(TELEMLOG_SLOW, li.boot_id, q, b, sizeof(b), &ms);
    if (n && b[0] == rec::BODY_STATUS) { take_status(s, b, n, ms); return; }
    if (q == r.oldest) return;
  }
}

void take_sample(Snapshot* s, const uint8_t* b, uint32_t block_ms) {
  int hdr, stride;
  switch (b[0]) {
    case rec::BODY_SAMPLES3: hdr = 2; stride = rec::SAMPLE3_BYTES; break;
    case rec::BODY_SAMPLES3_IMU: hdr = 2; stride = rec::IMU_SAMPLE3_BYTES; break;
    case rec::BODY_SAMPLES3_DEC: hdr = 3; stride = rec::SAMPLE3_BYTES; break;
    case rec::BODY_SAMPLES3_IMU_DEC: hdr = 3; stride = rec::IMU_SAMPLE3_BYTES; break;
    default: s->sample_len = 0; return;
  }
  const int n = b[1];
  if (n < 1) { s->sample_len = 0; return; }
  memcpy(s->sample, b + hdr + (n - 1) * stride, stride);
  s->sample_len = (uint8_t)stride;
  s->sample_imu = stride == rec::IMU_SAMPLE3_BYTES;
  s->sample_ms = block_ms + le(s->sample, 2);  // sample bytes 0-1: ms after the record time
}

void take_values(Snapshot* s) {
  for (int i = 0; i < TLM_COUNT; i++) s->tlm[i] = g_tlm_value[i];
}

int count() {
  int n = 0;
  for (int l = 0; l < N_LINES; l++)
    if (shown(l)) n += elements(l);
  return n;
}

int short_count() {
  int n = 0, li, el;
  for (int i = 0; locate(i, &li, &el); i++) n += place(li, el) > 0;
  return n;
}

const char* name(int i) {
  static char n[48];
  int li, el;
  if (!locate(i, &li, &el)) return nullptr;
  field_name(li, el, n, sizeof(n));
  return n;
}

int missing_short() {
  int miss = 0;
  for (int k = 0; k < N_SHORT; k++) {
    bool found = false;
    for (int i = 0; !found && name(i); i++) found = !strcmp(name(i), SHORT[k]);
    miss += !found;
  }
  return miss;
}

bool def_answer(const char* fw, sink_fn sink, void* ctx) {
  Out o(sink, ctx);
  o.str("{\"fw\":");
  o.esc(fw);
  o.fmt(",\"hash\":\"%08lx\",\"count\":%d,\"fields\":[", (unsigned long)tlm::hash(), count());
  int i = 0;
  for (int li = 0; li < N_LINES; li++) {
    if (!shown(li)) continue;
    const Line& L = LINES[li];
    for (int el = 0, ne = elements(li); o.ok && el < ne; el++, i++) {
    char n[48];
    field_name(li, el, n, sizeof(n));
    o.str(i ? ",{\"n\":" : "{\"n\":");
    o.esc(n);
    o.fmt(",\"r\":\"%s\",\"g\":", REC[L.src]);
    o.esc(L.group);
    o.str(",\"l\":\"");
    o.esc(L.label, false);
    if (elements(li) > 1) label_part(L, el, o);
    o.str("\",\"u\":");
    o.esc(L.unit);
    o.fmt(",\"d\":%d,\"k\":\"%s\",\"p\":%d", L.dec, KIND[L.kind], place(li, el));
    if (L.kind == K_BITS) { o.str(",\"b\":"); o.esc(L.names); }
    if (L.kind == K_ENUM) { o.str(",\"e\":"); o.esc(L.names); }
    o.str("}");
    }
  }
  o.str("]}");
  o.flush();
  return o.ok;
}

bool values_answer(const Snapshot& s, bool all, const Heap& h, sink_fn sink, void* ctx) {
  Out o(sink, ctx);
  o.fmt("{\"ms\":%lu,\"st\":", (unsigned long)s.now_ms);
  if (s.status_len) o.fmt("%ld", (long)(int32_t)(s.now_ms - s.status_ms)); else o.str("null");
  o.str(",\"sa\":");
  if (s.sample_len) o.fmt("%ld", (long)(int32_t)(s.now_ms - s.sample_ms)); else o.str("null");
  o.fmt(",\"heap\":%lu,\"heap_min\":%lu,\"maxblk\":%lu,\"v\":[", (unsigned long)h.free, (unsigned long)h.min,
        (unsigned long)h.max_block);
  int k = 0;
  for (int li = 0; li < N_LINES; li++) {
    if (!shown(li)) continue;
    for (int el = 0, ne = elements(li); o.ok && el < ne; el++) {
      if (!all && place(li, el) == 0) continue;
      if (k++) o.put(",", 1);
      value(o, s, li, el);
    }
  }
  o.str("]}");
  o.flush();
  return o.ok;
}

}  // namespace tlmpage
