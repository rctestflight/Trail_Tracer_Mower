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
const int MAX_PARTS = 16;

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

}  // namespace

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
}

void canStart() {
  t_can = esp_timer_get_time();
  if (cmd_new) {
    cmd_new = false;
    const int64_t l = t_can - cmd_edge;
    s_lat.add(l);
    int b = (int)(l / 100);
    hist[b < 0 ? 0 : b >= HBINS ? HBINS - 1 : b]++;
  }
  if (t_gyro) s_gyro.add(t_can - t_gyro);
}

void canEnd() { s_can.add(esp_timer_get_time() - t_can); }

void gyroSample() { t_gyro = esp_timer_get_time(); }


void part(uint8_t p) {
  const int64_t now = esp_timer_get_time();
  if (part_t) {
    const uint32_t d = (uint32_t)(now - part_t);
    if (part_cur < MAX_PARTS) {
      if (d > part_max[part_cur]) part_max[part_cur] = d;
      part_sum[part_cur] += d;
      pass_part_us[part_cur] += d;
    }
  }
  if (p == 0) {  // a new loop pass starts
    if (pass_t) {
      const uint32_t d = (uint32_t)(now - pass_t);
      s_pass.add(d);
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

}  // namespace rtprof
