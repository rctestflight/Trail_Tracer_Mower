// rt_prof: see rt_prof.h.
#include "rt_prof.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_timer.h"

namespace rtprof {

namespace {

const int RING = 16;
const int HBINS = 200;      // latency histogram: 100 us steps to 20 ms, the last step holds the rest
const int MAX_PARTS = 24;  // diag_cpu: 24 (radio sub-parts)

struct Stat {
  uint32_t n = 0, max = 0;
  uint64_t sum = 0;
  void add(int64_t v) { const uint32_t u = v < 0 ? 0 : (uint32_t)v; n++; sum += u; if (u > max) max = u; }
};

volatile int64_t edges[RING];
volatile uint32_t e_head = 0;  // edges seen (interrupt)
uint32_t e_tail = 0;           // edges matched to frames
int64_t t_edge = 0, t_read = 0, t_dec = 0, cmd_edge = 0, t_can = 0, t_gyro = 0;
bool cmd_new = false, frame_has_edge = false;
#ifdef DIAG_LAT2
// Test build (2026-10-30): the raw time stamps of each frame whose edge -> CAN time is over 5 ms, to check the latency
// figure by a second method. One esp_timer clock. Bench "lat2" prints the last 16 and the count.
struct Lat2 { int64_t edge, prev_edge, read, dec, cmd, can, can_prev, stm_tx; uint32_t depth, depth_can, heads; uint8_t part; };
Lat2 l2[16]; uint32_t l2_n = 0, l2_cmds = 0;
int64_t l2_prev_edge = 0, l2_cmd = 0, l2_stm_tx = 0; uint32_t l2_depth = 0;
#endif
Stat s_wait, s_decode, s_control, s_lat, s_can, s_gyro, s_pass;
uint32_t hist[HBINS];
uint32_t no_edge = 0;
// loop parts
int64_t part_t = 0, pass_t = 0;
uint8_t part_cur = 0;
uint32_t part_max[MAX_PARTS];
uint64_t part_sum[MAX_PARTS];  // time per part in the window, for the mean per pass
uint8_t worst_pass_part = 0;      // the part that took the most time in the longest pass
uint32_t pass_part_us[MAX_PARTS];  // this pass, per part
uint32_t worst_pass_part_us = 0;
uint32_t long_passes = 0, period_cache = 10000;
uint32_t long_n[MAX_PARTS], long_part_max[MAX_PARTS], long_pass_max[MAX_PARTS];  // the others, by their biggest part

// pass kinds and wakes (report2)
uint8_t wait_part_ = 0xFF, pend_kind = 0;
uint32_t kind_n[3], kind_max[3], wake_n[3];
uint64_t kind_sum[3];
uint64_t part_tot[MAX_PARTS];
int64_t win2_t0 = 0;

}  // namespace

void setWaitPart(uint8_t p) { wait_part_ = p; }
void passKind(uint8_t k) { pend_kind = k < 3 ? k : 2; }
void wake(uint8_t how) { if (how < 3) wake_n[how]++; }

void IRAM_ATTR edgeIsr() {
  edges[e_head % RING] = esp_timer_get_time();
  e_head = e_head + 1;
}

void resync() { e_tail = e_head; }

void frameRead() {
  t_read = esp_timer_get_time();
  const uint32_t h = e_head;
  if (h - e_tail > RING) e_tail = h - RING;  // more edges than the ring holds: the oldest are gone
  if (e_tail != h) {
#ifdef DIAG_LAT2
    l2_depth = h - e_tail;
    l2_prev_edge = edges[(e_tail + RING - 1) % RING];
#endif
    t_edge = edges[e_tail % RING];
    e_tail++;
    frame_has_edge = true;
    s_wait.add(t_read - t_edge);
  } else {
    frame_has_edge = false;
    no_edge++;
  }
}

void frameDecoded() {
  t_dec = esp_timer_get_time();
  s_decode.add(t_dec - t_read);
}

void commandSet() {
  const int64_t now = esp_timer_get_time();
  s_control.add(now - t_dec);
  if (frame_has_edge) { cmd_edge = t_edge; cmd_new = true; }
#ifdef DIAG_LAT2
  l2_cmd = now; l2_cmds++;
#endif
}

void canStart() {
#ifdef DIAG_LAT2
  const int64_t can_prev = t_can;
#endif
  t_can = esp_timer_get_time();
  if (cmd_new) {
    cmd_new = false;
    const int64_t l = t_can - cmd_edge;
#ifdef DIAG_LAT2
    if (l > 5000) {
      Lat2& r = l2[l2_n++ % 16];
      r.edge = cmd_edge; r.prev_edge = l2_prev_edge; r.read = t_read; r.dec = t_dec; r.cmd = l2_cmd; r.can = t_can;
      r.can_prev = can_prev; r.stm_tx = l2_stm_tx; r.depth = l2_depth; r.depth_can = e_head - e_tail; r.heads = e_head;
      r.part = part_cur;
    }
#endif
    s_lat.add(l);
    int b = (int)(l / 100);
    hist[b < 0 ? 0 : b >= HBINS ? HBINS - 1 : b]++;
  }
  if (t_gyro) s_gyro.add(t_can - t_gyro);
}

void canEnd() { s_can.add(esp_timer_get_time() - t_can); }

void gyroSample() { t_gyro = esp_timer_get_time(); }
void gyroSampleAt(int64_t t_us) { t_gyro = t_us; }


void part(uint8_t p) {
  const int64_t now = esp_timer_get_time();
  if (part_t) {
    const uint32_t d = (uint32_t)(now - part_t);
    if (part_cur < MAX_PARTS) {
      if (d > part_max[part_cur]) part_max[part_cur] = d;
      part_sum[part_cur] += d;
      pass_part_us[part_cur] += d;
      part_tot[part_cur] += d;
    }
  }
  if (p == 0) {  // a new loop pass starts
    if (pass_t) {
      const uint32_t d = (uint32_t)(now - pass_t);
      s_pass.add(d);
      {
        const uint32_t w = wait_part_ < MAX_PARTS ? pass_part_us[wait_part_] : 0;
        const uint32_t act = d > w ? d - w : 0;  // the pass without its end wait
        kind_n[pend_kind]++;
        kind_sum[pend_kind] += act;
        if (act > kind_max[pend_kind]) kind_max[pend_kind] = act;
      }
      if (d > period_cache) {
        long_passes++;
        {
          uint8_t w = 0;
          for (uint8_t i = 1; i < MAX_PARTS; i++) if (pass_part_us[i] > pass_part_us[w]) w = i;
          long_n[w]++;
          if (pass_part_us[w] > long_part_max[w]) long_part_max[w] = pass_part_us[w];
          if (d > long_pass_max[w]) long_pass_max[w] = d;
        }
      }
      if (d >= s_pass.max) {  // the longest pass so far: remember its biggest part
        uint8_t w = 0;
        for (uint8_t i = 1; i < MAX_PARTS; i++) if (pass_part_us[i] > pass_part_us[w]) w = i;
        worst_pass_part = w;
        worst_pass_part_us = pass_part_us[w];
      }
    }
    pass_t = now;
    memset(pass_part_us, 0, sizeof(pass_part_us));
  }
  part_t = now;
  part_cur = p < MAX_PARTS ? p : MAX_PARTS - 1;
}

#ifdef DIAG_LAT2
void stmTxAt() { l2_stm_tx = esp_timer_get_time(); }
int lat2Line(int i, char* buf, size_t len) {  // line i of the report (0: the counts); 0 when no more (resets then)
  const uint32_t m = l2_n < 16 ? l2_n : 16;
  if (i == 0) return snprintf(buf, len, "LAT2 slow=%lu commands=%lu (times in us from the edge)", (unsigned long)l2_n,
                              (unsigned long)l2_cmds);
  if ((uint32_t)i > m) { l2_n = 0; l2_cmds = 0; return 0; }
  {
    const Lat2& r = l2[(l2_n - m + (uint32_t)i - 1) % 16];
    return snprintf(buf, len,
        "LAT2 edge=%lld prev_edge=%lld read=%lld dec=%lld cmd=%lld can=%lld can_prev=%lld stm_tx=%lld depth=%lu depth_can=%lu edges=%lu part=%u",
        (long long)r.edge, (long long)(r.prev_edge - r.edge), (long long)(r.read - r.edge), (long long)(r.dec - r.edge),
        (long long)(r.cmd - r.edge), (long long)(r.can - r.edge), (long long)(r.can_prev - r.edge),
        (long long)(r.stm_tx - r.edge), (unsigned long)r.depth, (unsigned long)r.depth_can, (unsigned long)r.heads,
        (unsigned)r.part);
  }
}
#endif

static uint32_t pct(double q) {
  const uint32_t want = (uint32_t)(q * s_lat.n + 0.999);
  uint32_t c = 0;
  for (int i = 0; i < HBINS; i++) { c += hist[i]; if (c >= want && c) return (uint32_t)(i + 1) * 100; }
  return HBINS * 100;
}

int report(char* buf, size_t len, const char* const* names, int n, uint32_t period_us) {
  period_cache = period_us ? period_us : 10000;
  auto mean = [](const Stat& s) { return s.n ? (uint32_t)(s.sum / s.n) : 0u; };
  size_t k = (size_t)snprintf(buf, len,
      "RT period_us=%lu frames=%lu no_edge=%lu | latency edge->CAN n=%lu p50<=%lu p90<=%lu p99<=%lu max=%lu us | "
      "wait mean=%lu max=%lu | decode mean=%lu max=%lu | control mean=%lu max=%lu | can mean=%lu max=%lu | "
      "gyro_age mean=%lu max=%lu | pass mean=%lu max=%lu long=%lu worst_part=%s %lu us | part max:",
      (unsigned long)period_cache, (unsigned long)s_decode.n, (unsigned long)no_edge, (unsigned long)s_lat.n,
      (unsigned long)pct(0.5), (unsigned long)pct(0.9), (unsigned long)pct(0.99), (unsigned long)s_lat.max,
      (unsigned long)mean(s_wait), (unsigned long)s_wait.max, (unsigned long)mean(s_decode), (unsigned long)s_decode.max,
      (unsigned long)mean(s_control), (unsigned long)s_control.max, (unsigned long)mean(s_can), (unsigned long)s_can.max,
      (unsigned long)mean(s_gyro), (unsigned long)s_gyro.max, (unsigned long)mean(s_pass), (unsigned long)s_pass.max,
      (unsigned long)long_passes, worst_pass_part < n && names ? names[worst_pass_part] : "?", (unsigned long)worst_pass_part_us);
  for (int i = 0; i < n && i < MAX_PARTS && k < len; i++)
    if (part_max[i]) k += (size_t)snprintf(buf + k, len - k, " %s=%lu", names[i], (unsigned long)part_max[i]);
  if (k < len) k += (size_t)snprintf(buf + k, len - k, " | part mean per pass:");
  for (int i = 0; i < n && i < MAX_PARTS && k < len; i++)
    if (part_sum[i] && s_pass.n) k += (size_t)snprintf(buf + k, len - k, " %s=%lu", names[i], (unsigned long)(part_sum[i] / s_pass.n));
  if (k < len) k += (size_t)snprintf(buf + k, len - k,
      " | over period, by biggest part (count/part us/pass us):");
  for (int i = 0; i < n && i < MAX_PARTS && k < len; i++)
    if (long_n[i]) k += (size_t)snprintf(buf + k, len - k, " %s=%lu/%lu/%lu", names[i], (unsigned long)long_n[i],
                                         (unsigned long)long_part_max[i], (unsigned long)long_pass_max[i]);
  if (k >= len) k = len - 1;
  memset(long_n, 0, sizeof(long_n)); memset(long_part_max, 0, sizeof(long_part_max)); memset(long_pass_max, 0, sizeof(long_pass_max));
  s_wait = s_decode = s_control = s_lat = s_can = s_gyro = s_pass = Stat();
  memset(hist, 0, sizeof(hist));
  memset(part_max, 0, sizeof(part_max));
  memset(part_sum, 0, sizeof(part_sum));
  no_edge = long_passes = worst_pass_part_us = 0;
  worst_pass_part = 0;
  return (int)k;
}

int report2(char* buf, size_t len, const char* const* names, int n) {
  const int64_t now = esp_timer_get_time();
  const double win = win2_t0 ? (double)(now - win2_t0) : 0;
  win2_t0 = now;
  static const char* const KN[3] = {"empty", "decode", "control"};
  size_t k = (size_t)snprintf(buf, len, "RT2 window_ms=%.0f | wake timeout=%lu edge=%lu nowait=%lu | pass n/active mean/max us:",
                              win / 1000, (unsigned long)wake_n[0], (unsigned long)wake_n[1], (unsigned long)wake_n[2]);
  for (int i = 0; i < 3 && k < len; i++)
    k += (size_t)snprintf(buf + k, len - k, " %s=%lu/%lu/%lu", KN[i], (unsigned long)kind_n[i],
                          (unsigned long)(kind_n[i] ? kind_sum[i] / kind_n[i] : 0), (unsigned long)kind_max[i]);
  if (k < len) k += (size_t)snprintf(buf + k, len - k, " | wall %% of window:");
  for (int i = 0; i < n && i < MAX_PARTS && k < len; i++)
    if (part_tot[i] && win > 0) k += (size_t)snprintf(buf + k, len - k, " %s=%.2f", names[i], 100.0 * part_tot[i] / win);
  if (k >= len) k = len - 1;
  memset(kind_n, 0, sizeof(kind_n)); memset(kind_max, 0, sizeof(kind_max)); memset(kind_sum, 0, sizeof(kind_sum));
  memset(wake_n, 0, sizeof(wake_n)); memset(part_tot, 0, sizeof(part_tot));
  return (int)k;
}

}  // namespace rtprof
