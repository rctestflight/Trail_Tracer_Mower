// Diagnostic build DIAG_STEPTIME (2026-10-29, control step timing, bench only, observation only): the
// interval between control steps, and for each late step what held the CPU in the gap.
// - onStep() runs at the start of each control step (rxFrameReady, after the frame read). Interval = CPU cycles since
//   the step before. Nominal interval: the control gate (CONTROL_PERIOD_US - 500 us) rounded up to whole frames.
//   Late: interval > nominal + half a frame. Skipped control frames: (interval - nominal) / frame, rounded. Lost
//   frames: frames in the interval (by time) minus the frame sequence step.
// - Gap: the CPU sampler's ring (lib/cpu_prof, 2 kHz) between the two steps. Each sample takes the time since the
//   sample before, up to 1.5 x the mean interval; the rest of a longer spacing is "unsampled" (flash cache off or
//   interrupts masked: a flash write or erase, or a long critical section).
// - v2: delay = the time from the frame's nDATA_READY edge to the step. Every step: its delay in a histogram, and the
//   sampler ring over (edge, step] by source: what the loop ran while the frame waited.
// - Phases: "start" = the first 60 s after a run start; "run" = later in the run. Steps with no run: not counted.
// Bench "steps": the report of each phase (cumulative), one line per loop pass when the USB queue has room (a whole
// report at once overflowed the queue); "steps reset": clear after the report.
#ifdef DIAG_STEPTIME
#include <Arduino.h>
#include <string.h>
#include "hal/cpu_hal.h"
#include "cpu_prof.h"
#include "usb_text.h"

extern UsbOut usbOut;
extern bool rover_active;
extern double stmFramePeriodS();
extern volatile uint32_t stm_edge_us;               // src/platform/rx_link.inc: the time of the newest frame edge
extern const char* const* cpuSectionNames(int* n);  // the loop part names (src/platform/board.inc)
extern volatile uint8_t g_st_heavy_now, g_st_heavy_seen;
extern uint32_t crc_error_count, uart_overrun_count;  // src/platform/rx_link.inc

namespace {
const int NSRC = 64, SRC_ISR = 24, SRC_OTHER = 25, SRC_LOOP = 32, SRC_UNSAMPLED = 63;
const int NB = 80;            // interval histogram: 0.5 ms bins, 0..40 ms (the last bin: 40 ms and more)
const int NL = 48;            // delay histogram: 0.25 ms bins, 0..12 ms (the last bin: 12 ms and more)
const int NS = 50;            // solve answer delay histogram: 2 ms bins, 0..100 ms (the last bin: 100 ms and more)
const int NGAP = 6;           // the worst gaps kept per phase
const int NPH = 2;
const uint32_t CONTROL_GATE_US = 9500;  // CONTROL_PERIOD_US - 500 (src/platform/rx_link.inc)
struct Gap { uint32_t dt_us, t_ms; uint8_t src[4]; uint32_t us[4]; };
// v3: the worst frame-to-step delays, with their 4 largest sources (IDLE in) and the loop's sleeps inside the delay
const int NLAT = 5;
struct LatW { uint32_t lat_us, t_ms; uint8_t src[4]; uint32_t us[4]; uint16_t waits, tmo; uint32_t wait_us, long_us; int32_t long_at_us; int16_t long_av;
              uint8_t bsec[2]; uint32_t bus[2]; };  // v3b: the loop's part while another task ran (the loop held up there)
struct Wait { uint32_t t0_us, t1_us; bool woken; int16_t av; };
const int NW = 16;
Wait wring[NW];  // the loop's last sleeps (setup_loop.inc)
uint32_t whead = 0;
// the steps and solve answers while a solve with the closed-form starts runs (about every 10th solve); the rest = all - heavy
struct Heavy { uint32_t n, max_us, late, lat_n, lat_max_us, sv_n, sv_max_us; uint64_t sv_sum_us; uint32_t hist[NB], lhist[NL], svhist[NS]; };
struct Phase {
  uint32_t n, late, skipped, lost, max_us, lat_max_us;
  uint32_t sec_ms, sec_n, sec_min, sec_min_t_ms, secs;  // control steps per whole second: the worst second
  uint32_t sv_n, sv_max_us, sv_wait_max_us;            // solve answers: start to pickup by the control step
  uint64_t sv_sum_us, sv_wait_sum_us;
  uint32_t svhist[NS];
  uint64_t sum_us, ms, lat_sum_us;
  uint32_t hist[NB];
  uint32_t lhist[NL];
  uint32_t tot_us[NSRC], worst_us[NSRC];   // late gaps
  uint32_t ltot_us[NSRC], lworst_us[NSRC]; // frame edge to step, every step
  Gap gap[NGAP];
  Heavy hv;
  uint32_t fa_n, fa_max_us, fahist[NB];  // frame arrival gap: newest frame edge to the one before (Rx STM32 side)
  LatW lw[NLAT];
  uint32_t seq_lost, crc0, ovr0;  // v3c: frames missing by sequence number; link counters at the phase start
};
Phase* ph[NPH] = {};  // on the heap, one block per phase (about 2 KB each)
int cur_pi = -1;      // the phase of the last control step
const char* const PH[NPH] = {"start", "run"};

int srcOf(const cpuprof::Ring& r) {
  if (r.task == cpuprof::RING_ISR) return SRC_ISR;
  if (r.task == cpuprof::RING_OTHER) return SRC_OTHER;
  if (r.sec != 0xFF) return SRC_LOOP + (r.sec < 30 ? r.sec : 30);
  return r.task < 24 ? r.task : SRC_OTHER;
}

const char* srcName(int s, char* t) {
  if (s == SRC_UNSAMPLED) return "unsampled";
  if (s >= SRC_LOOP) {
    int n = 0;
    const char* const* names = cpuSectionNames(&n);
    if (s - SRC_LOOP < n) { snprintf(t, 24, "loop:%s", names[s - SRC_LOOP]); return t; }
    return "loop:?";
  }
  if (s == SRC_ISR) return "isr";
  if (s == SRC_OTHER) return "other";
  return cpuprof::taskName((uint8_t)s);
}

// The sampler ring over (t0, now] (CPU cycles) into g[] by source, in us.
void walk(uint32_t t0, uint32_t now, uint32_t mhz, uint32_t* g, uint32_t* blk = nullptr) {
  uint32_t head = 0;
  const cpuprof::Ring* ring = cpuprof::ringBase(&head);
  const uint32_t nr = head < cpuprof::RING_N ? head : cpuprof::RING_N - 4;  // 4 slots margin: the ISR writes at the head
  const uint32_t cap = cpuprof::meanUs() * 3 / 2;
  // the first sample after t0 (newest first, stop at t0)
  uint32_t j0 = nr;
  for (uint32_t jj = nr; jj > 0; jj--) {
    if ((int32_t)(ring[(head - nr + jj - 1) % cpuprof::RING_N].cyc - t0) <= 0) break;
    j0 = jj - 1;
  }
  uint32_t last = t0;
  for (uint32_t jj = j0; jj < nr; jj++) {
    const cpuprof::Ring& rj = ring[(head - nr + jj) % cpuprof::RING_N];
    const uint32_t c = rj.cyc;
    if ((int32_t)(c - now) > 0) break;
    const uint32_t sp = (c - last) / mhz;
    last = c;
    const uint32_t a = sp < cap ? sp : cap;
    g[srcOf(rj)] += a;
    if (blk && rj.sec == 0xFF && rj.task != cpuprof::RING_ISR && rj.lsec < 32) blk[rj.lsec] += a;
    g[SRC_UNSAMPLED] += sp - a;
  }
  if ((int32_t)(now - last) > 0) {  // after the last sample: unsampled beyond 1.5 x the mean
    const uint32_t sp = (now - last) / mhz;
    if (sp > cap) g[SRC_UNSAMPLED] += sp - cap;
  }
}

// the report: line k of phase pi into b (false: no such line)
int rep_k = -1;
bool rep_reset = false;
bool line(int pi, int k, char* b, size_t bn) {
  Phase& p = *ph[pi];
  if (!p.n) return false;
  char t[24];
  int o = 0;
  auto add = [&](const char* f, ...) {
    if (o >= (int)bn - 1) return;
    va_list a;
    va_start(a, f);
    o += vsnprintf(b + o, bn - o, f, a);
    va_end(a);
  };
  auto ranked = [&](const uint32_t* tot, const uint32_t* worst) {
    bool done[NSRC] = {};
    for (int r = 0; r < 16; r++) {
      int m = -1;
      for (int s = 0; s < NSRC; s++) if (!done[s] && tot[s] && (m < 0 || tot[s] > tot[m])) m = s;
      if (m < 0) break;
      done[m] = true;
      add(" %s=%.1f/%.2f", srcName(m, t), tot[m] / 1000.0, worst[m] / 1000.0);
    }
  };
  const double min = p.ms / 60000.0;
  if (k == 0) {
    uint32_t p50 = 0, p99 = 0, acc = 0;  // from the histogram: the upper edge of the bin
    for (int i = 0; i < NB; i++) {
      acc += p.hist[i];
      if (!p50 && acc * 2 >= p.n) p50 = (i + 1) * 500;
      if (!p99 && acc * 100 >= p.n * 99ull) p99 = (i + 1) * 500;
    }
    add("STEPS ph=%s n=%lu s=%.0f frame_us=%.0f mean_us=%.0f p50_us<=%lu p99_us<=%lu max_us=%lu late=%lu late_per_min=%.1f "
        "skipped=%lu skipped_per_min=%.1f lost=%lu", PH[pi], (unsigned long)p.n, p.ms / 1000.0, stmFramePeriodS() * 1e6,
        (double)p.sum_us / p.n, (unsigned long)p50, (unsigned long)p99, (unsigned long)p.max_us, (unsigned long)p.late,
        min > 0 ? p.late / min : 0, (unsigned long)p.skipped, min > 0 ? p.skipped / min : 0, (unsigned long)p.lost);
    add(" seq_lost=%lu crc=%lu overrun=%lu", (unsigned long)p.seq_lost, (unsigned long)(crc_error_count - p.crc0),
        (unsigned long)(uart_overrun_count - p.ovr0));
  } else if (k == 1) {
    add("STEPHIST ph=%s", PH[pi]);
    for (int i = 0; i < NB; i++) if (p.hist[i]) add(" %d:%lu", i * 500, (unsigned long)p.hist[i]);
  } else if (k == 2) {
    add("STEPSRC ph=%s late=%lu (source=total_ms/worst_ms)", PH[pi], (unsigned long)p.late);
    ranked(p.tot_us, p.worst_us);
  } else if (k >= 3 && k < 3 + NGAP) {
    const Gap& e = p.gap[k - 3];
    if (!e.dt_us) return false;
    add("STEPGAP ph=%s dt_us=%lu t_s=%.1f", PH[pi], (unsigned long)e.dt_us, e.t_ms / 1000.0);
    for (int r = 0; r < 4; r++) if (e.us[r]) add(" %s=%lu", srcName(e.src[r], t), (unsigned long)e.us[r]);
  } else if (k == 3 + NGAP) {
    uint32_t p50 = 0, p90 = 0, p99 = 0, acc = 0;
    for (int i = 0; i < NL; i++) {
      acc += p.lhist[i];
      if (!p50 && acc * 2 >= p.n) p50 = (i + 1) * 250;
      if (!p90 && acc * 10 >= p.n * 9ull) p90 = (i + 1) * 250;
      if (!p99 && acc * 100 >= p.n * 99ull) p99 = (i + 1) * 250;
    }
    add("STEPLAT ph=%s n=%lu mean_us=%.0f p50_us<=%lu p90_us<=%lu p99_us<=%lu max_us=%lu", PH[pi], (unsigned long)p.n,
        (double)p.lat_sum_us / p.n, (unsigned long)p50, (unsigned long)p90, (unsigned long)p99, (unsigned long)p.lat_max_us);
  } else if (k == 4 + NGAP) {
    add("STEPLATHIST ph=%s", PH[pi]);
    for (int i = 0; i < NL; i++) if (p.lhist[i]) add(" %d:%lu", i * 250, (unsigned long)p.lhist[i]);
  } else if (k == 5 + NGAP) {
    add("STEPLATSRC ph=%s (source=total_ms/worst_ms)", PH[pi]);
    ranked(p.ltot_us, p.lworst_us);
  } else if (k == 6 + NGAP) {
    add("STEPSEC ph=%s secs=%lu worst_sec_steps=%lu at_s=%.0f", PH[pi], (unsigned long)p.secs, (unsigned long)p.sec_min,
        p.sec_min_t_ms / 1000.0);
  } else if (k == 7 + NGAP) {
    if (!p.sv_n) return false;
    uint32_t p50 = 0, p99 = 0, acc = 0;
    for (int i = 0; i < NS; i++) {
      acc += p.svhist[i];
      if (!p50 && acc * 2 >= p.sv_n) p50 = (i + 1) * 2000;
      if (!p99 && acc * 100 >= p.sv_n * 99ull) p99 = (i + 1) * 2000;
    }
    add("SOLVEDLY ph=%s n=%lu mean_us=%.0f p50_us<=%lu p99_us<=%lu max_us=%lu wait_mean_us=%.0f wait_max_us=%lu", PH[pi],
        (unsigned long)p.sv_n, (double)p.sv_sum_us / p.sv_n, (unsigned long)p50, (unsigned long)p99,
        (unsigned long)p.sv_max_us, (double)p.sv_wait_sum_us / p.sv_n, (unsigned long)p.sv_wait_max_us);
    add(" hist");
    for (int i = 0; i < NS; i++) if (p.svhist[i]) add(" %d:%lu", i * 2, (unsigned long)p.svhist[i]);
#ifndef DIAG_NO_SPLIT  // v3 build: left out to fit the update slot
  } else if (k == 8 + NGAP || k == 9 + NGAP) {
    // SPLIT: steps (interval), frame edge to step, solve answers; heavy = while a closed-form solve runs, rest = the others
    const bool hv = k == 8 + NGAP;
    const Heavy& h = p.hv;
    auto pct = [](const uint32_t* a, const uint32_t* sub, int nb, uint32_t n, uint32_t bin, uint32_t q) {
      uint64_t acc = 0;
      for (int i = 0; i < nb; i++) {
        acc += sub ? a[i] - sub[i] : a[i];
        if (n && acc * 100 >= (uint64_t)n * q) return (uint32_t)(i + 1) * bin;
      }
      return (uint32_t)0;
    };
    auto mx = [](const uint32_t* a, const uint32_t* sub, int nb, uint32_t bin) {
      for (int i = nb - 1; i >= 0; i--) if ((sub ? a[i] - sub[i] : a[i]) > 0) return (uint32_t)(i + 1) * bin;
      return (uint32_t)0;
    };
    uint32_t hn = 0, ln = 0;
    for (int i = 0; i < NB; i++) hn += p.hist[i];
    for (int i = 0; i < NL; i++) ln += p.lhist[i];
    const uint32_t sn = hv ? h.n : hn - h.n, lnn = hv ? h.lat_n : ln - h.lat_n, vn = hv ? h.sv_n : p.sv_n - h.sv_n;
    const uint32_t* sub = hv ? nullptr : h.hist;
    const uint32_t* lsub = hv ? nullptr : h.lhist;
    const uint32_t* vsub = hv ? nullptr : h.svhist;
    const uint32_t* sa = hv ? h.hist : p.hist;
    const uint32_t* la = hv ? h.lhist : p.lhist;
    const uint32_t* va = hv ? h.svhist : p.svhist;
    const uint64_t vsum = hv ? h.sv_sum_us : p.sv_sum_us - h.sv_sum_us;
    add("SPLIT ph=%s cls=%s steps=%lu gap_p99_us<=%lu gap_max_us%s%lu late=%lu lat_n=%lu lat_p99_us<=%lu lat_max_us%s%lu "
        "solves=%lu sv_mean_us=%.0f sv_p99_us<=%lu sv_max_us%s%lu", PH[pi], hv ? "heavy" : "rest",
        (unsigned long)sn, (unsigned long)pct(sa, sub, NB, sn, 500, 99), hv ? "=" : "<=",
        (unsigned long)(hv ? h.max_us : mx(sa, sub, NB, 500)), (unsigned long)(hv ? h.late : p.late - h.late), (unsigned long)lnn,
        (unsigned long)pct(la, lsub, NL, lnn, 250, 99), hv ? "=" : "<=", (unsigned long)(hv ? h.lat_max_us : mx(la, lsub, NL, 250)),
        (unsigned long)vn, vn ? (double)vsum / vn : 0.0, (unsigned long)pct(va, vsub, NS, vn, 2000, 99), hv ? "=" : "<=",
        (unsigned long)(hv ? h.sv_max_us : mx(va, vsub, NS, 2000)));
#endif
  } else if (k == 10 + NGAP) {
    if (!p.fa_n) return false;
    uint32_t p50 = 0, p99 = 0, acc = 0;
    for (int i = 0; i < NB; i++) {
      acc += p.fahist[i];
      if (!p50 && acc * 2 >= p.fa_n) p50 = (i + 1) * 500;
      if (!p99 && acc * 100 >= p.fa_n * 99ull) p99 = (i + 1) * 500;
    }
    add("FRAMEGAP ph=%s n=%lu p50_us<=%lu p99_us<=%lu max_us=%lu", PH[pi], (unsigned long)p.fa_n, (unsigned long)p50,
        (unsigned long)p99, (unsigned long)p.fa_max_us);
  } else if (k >= 11 + NGAP && k < 11 + NGAP + NLAT) {
    // the sleeps: count, ended by the 5 ms timeout, total and longest inside the delay, the longest's start after the
    // frame edge (negative: it began before the edge) and the received bytes at its start
    const LatW& e = p.lw[k - 11 - NGAP];
    if (!e.lat_us) return false;
    add("STEPLATW ph=%s lat_us=%lu t_s=%.1f waits=%u tmo=%u wait_us=%lu long_us=%lu long_at_us=%ld long_av=%d", PH[pi],
        (unsigned long)e.lat_us, e.t_ms / 1000.0, e.waits, e.tmo, (unsigned long)e.wait_us, (unsigned long)e.long_us,
        (long)e.long_at_us, e.long_av);
    for (int r = 0; r < 4; r++) if (e.us[r]) add(" %s=%lu", srcName(e.src[r], t), (unsigned long)e.us[r]);
    for (int r = 0; r < 2; r++) if (e.bus[r]) add(" held_in_%s=%lu", srcName(SRC_LOOP + e.bsec[r], t) + 5, (unsigned long)e.bus[r]);
  } else {
    return false;
  }
  return true;
}
const int NLINE = 11 + NGAP + NLAT;
}  // namespace

namespace steptime {

void onStep(uint8_t seq) {
  static uint32_t prev = 0, prev_ms = 0, run_ms = 0;
  static uint8_t prev_seq = 0;
  static bool have = false, was_active = false;
  const uint32_t now = cpu_hal_get_cycle_count();
  const uint32_t now_us = (uint32_t)esp_timer_get_time();
  const uint32_t edge_us = stm_edge_us;
  const uint32_t ms = millis();
  if (rover_active && !was_active) run_ms = ms;
  was_active = rover_active;
  if (!rover_active) { have = false; return; }
  const int pi = ms - run_ms < 60000 ? 0 : 1;
#ifdef DIAG_NO_SPLIT
  // v3 build: the run phase only, its block taken at the run start (at 60 s the largest free block was 2.8 KB on
  // an Rx board on Wi-Fi, under the 3.3 KB block: that window kept no run data)
  if (!ph[1]) ph[1] = (Phase*)calloc(1, sizeof(Phase));
  if (pi == 0) { have = false; return; }
#endif
  if (!ph[pi]) {
    ph[pi] = (Phase*)calloc(1, sizeof(Phase));
    if (!ph[pi]) return;
  }
  static uint32_t mhz = 0;
  if (!mhz) mhz = getCpuFrequencyMhz();
  Phase& p = *ph[pi];
  cur_pi = pi;
  // heavy: a closed-form solve ran at some time since the last step
  const uint8_t seen = g_st_heavy_seen;
  g_st_heavy_seen = 0;
  const bool heavy = seen || g_st_heavy_now;
  Heavy& h = p.hv;
  // control steps per whole second (the first second of each phase starts at its first step)
  if (!p.sec_ms) p.sec_ms = ms;
  while (ms - p.sec_ms >= 1000) {
    if (!p.secs || p.sec_n < p.sec_min) { p.sec_min = p.sec_n; p.sec_min_t_ms = p.sec_ms; }
    p.secs++;
    p.sec_ms += 1000;
    p.sec_n = 0;
  }
  p.sec_n++;
  // the frame arrival gap: this step's frame edge against the last step's (one step per frame at 10 ms)
  {
    static uint32_t prev_edge = 0;
    if (have && edge_us != prev_edge) {
      const uint32_t fa = edge_us - prev_edge;
      if (fa < 1000000) {
        p.fa_n++;
        if (fa > p.fa_max_us) p.fa_max_us = fa;
        p.fahist[fa / 500 < (uint32_t)NB ? fa / 500 : NB - 1]++;
      }
    }
    prev_edge = edge_us;
  }
  // the delay from the frame edge to this step
  const uint32_t lat = now_us - edge_us;
  if (lat < 100000) {
    p.lat_sum_us += lat;
    if (lat > p.lat_max_us) p.lat_max_us = lat;
    p.lhist[lat / 250 < (uint32_t)NL ? lat / 250 : NL - 1]++;
    if (heavy) {
      h.lat_n++;
      if (lat > h.lat_max_us) h.lat_max_us = lat;
      h.lhist[lat / 250 < (uint32_t)NL ? lat / 250 : NL - 1]++;
    }
    uint32_t g[NSRC] = {}, blk[32] = {};
    walk(now - lat * mhz, now, mhz, g, blk);
    for (int s = 0; s < NSRC; s++) {
      p.ltot_us[s] += g[s];
      if (g[s] > p.lworst_us[s]) p.lworst_us[s] = g[s];
    }
    int w = 0;
    for (int i = 1; i < NLAT; i++) if (p.lw[i].lat_us < p.lw[w].lat_us) w = i;
    if (lat > p.lw[w].lat_us) {
      LatW& e = p.lw[w];
      memset(&e, 0, sizeof(e));
      e.lat_us = lat;
      e.t_ms = ms;
      for (int r = 0; r < 4; r++) {
        int b = -1;
        for (int s = 0; s < NSRC; s++) {
          if (!g[s]) continue;
          bool used = false;
          for (int q = 0; q < r; q++) used |= (e.src[q] == s && e.us[q]);
          if (!used && (b < 0 || g[s] > g[b])) b = s;
        }
        if (b < 0) break;
        e.src[r] = (uint8_t)b;
        e.us[r] = g[b];
      }
      for (int i = 0; i < NW; i++) {
        const Wait& v = wring[i];
        if (!v.t1_us || (int32_t)(v.t1_us - edge_us) <= 0 || (int32_t)(v.t0_us - now_us) >= 0) continue;
        const uint32_t a = (int32_t)(v.t0_us - edge_us) > 0 ? v.t0_us : edge_us;
        const uint32_t z = (int32_t)(v.t1_us - now_us) < 0 ? v.t1_us : now_us;
        const uint32_t d = z - a;
        e.waits++;
        if (!v.woken) e.tmo++;
        e.wait_us += d;
        if (d > e.long_us) { e.long_us = d; e.long_at_us = (int32_t)(v.t0_us - edge_us); e.long_av = v.av; }
      }
      for (int r = 0; r < 2; r++) {
        int b = -1;
        for (int q = 0; q < 32; q++) if (blk[q] && (r == 0 || q != e.bsec[0]) && (b < 0 || blk[q] > blk[b])) b = q;
        if (b >= 0) { e.bsec[r] = (uint8_t)b; e.bus[r] = blk[b]; }
      }
    }
  }
  if (!p.n) { p.crc0 = crc_error_count; p.ovr0 = uart_overrun_count; }
  if (!have) { have = true; prev = now; prev_ms = ms; prev_seq = seq; p.n++; return; }
  const uint32_t dt = (now - prev) / mhz;
  const uint32_t t0 = prev;
  prev = now;
  p.n++;
  p.sum_us += dt;
  p.ms += ms - prev_ms;
  prev_ms = ms;
  if (dt > p.max_us) p.max_us = dt;
  p.hist[dt / 500 < (uint32_t)NB ? dt / 500 : NB - 1]++;
  if (heavy) {
    h.n++;
    if (dt > h.max_us) h.max_us = dt;
    h.hist[dt / 500 < (uint32_t)NB ? dt / 500 : NB - 1]++;
  }
  const double fp = stmFramePeriodS() * 1e6;
  const uint32_t frame = fp > 100 ? (uint32_t)fp : 10000;
  const uint32_t k = (CONTROL_GATE_US + frame - 1) / frame;
  const uint32_t nominal = (k ? k : 1) * frame;
  const uint8_t dseq = (uint8_t)(seq - prev_seq);
  prev_seq = seq;
  const uint32_t frames_by_time = (dt + frame / 2) / frame;
  if (frames_by_time > dseq) p.lost += frames_by_time - dseq;
  if (dseq > 1) p.seq_lost += dseq - 1;  // v3c: "lost" above counts by time (a late frame reads as lost); this one by number
  if (dt <= nominal + frame / 2) return;
  p.late++;
  if (heavy) h.late++;
  p.skipped += (dt - nominal + frame / 2) / frame;
  uint32_t g[NSRC] = {};
  walk(t0, now, mhz, g);
  for (int s = 0; s < NSRC; s++) {
    p.tot_us[s] += g[s];
    if (g[s] > p.worst_us[s]) p.worst_us[s] = g[s];
  }
  // keep the worst NGAP gaps, with their 4 largest sources (IDLE left out)
  int w = 0;
  for (int i = 1; i < NGAP; i++) if (p.gap[i].dt_us < p.gap[w].dt_us) w = i;
  if (dt > p.gap[w].dt_us) {
    Gap& e = p.gap[w];
    memset(&e, 0, sizeof(e));
    e.dt_us = dt;
    e.t_ms = ms;
    for (int r = 0; r < 4; r++) {
      int b = -1;
      for (int s = 0; s < NSRC; s++) {
        if (!g[s] || (s < 24 && !strcmp(cpuprof::taskName((uint8_t)s), "IDLE"))) continue;
        bool used = false;
        for (int q = 0; q < r; q++) used |= (e.src[q] == s && e.us[q]);
        if (!used && (b < 0 || g[s] > g[b])) b = s;
      }
      if (b < 0) break;
      e.src[r] = (uint8_t)b;
      e.us[r] = g[b];
    }
  }
}

void report(bool reset) {
  if (!ph[0] && !ph[1]) { usbOut.println("STEPS none yet"); return; }
  rep_k = 0;
  rep_reset = reset;
}

// From benchService, each pass: the next report line when the USB queue has room for it.
void service() {
  static char b[1100];
  while (rep_k >= 0 && rep_k < NPH * NLINE) {
    const int pi = rep_k / NLINE, k = rep_k % NLINE;
    if (!ph[pi] || !line(pi, k, b, sizeof(b) - 2)) { rep_k++; continue; }
    if (usbOut.room() < strlen(b) + 2) return;  // the next pass
    usbOut.println(b);
    rep_k++;
    return;  // one line a pass
  }
  if (rep_k >= 0) {
    if (rep_reset) for (int i = 0; i < NPH; i++) if (ph[i]) memset(ph[i], 0, sizeof(Phase));
    rep_k = -1;
  }
}

}  // namespace steptime
// A solve answer taken by the control step: start (task notified), answer ready, pickup (all esp_timer us).
volatile uint8_t g_st_heavy_now = 0, g_st_heavy_seen = 0;  // set by the solve (lib/wire_fit), cleared at the answer and by the step
void steptime_onSolve(uint32_t start_us, uint32_t done_us, uint32_t pick_us, bool heavy) {
  if (cur_pi < 0 || !ph[cur_pi] || !rover_active) return;
  Phase& p = *ph[cur_pi];
  const uint32_t d = pick_us - start_us, w = pick_us - done_us;
  if (d > 10000000 || w > d) return;
  p.sv_n++;
  p.sv_sum_us += d;
  p.sv_wait_sum_us += w;
  if (d > p.sv_max_us) p.sv_max_us = d;
  if (w > p.sv_wait_max_us) p.sv_wait_max_us = w;
  p.svhist[d / 2000 < (uint32_t)NS ? d / 2000 : NS - 1]++;
  if (heavy) {
    Heavy& h = p.hv;
    h.sv_n++;
    h.sv_sum_us += d;
    if (d > h.sv_max_us) h.sv_max_us = d;
    h.svhist[d / 2000 < (uint32_t)NS ? d / 2000 : NS - 1]++;
  }
}
void steptime_onWait(uint32_t t0_us, uint32_t t1_us, bool woken, int avail) {
  Wait& v = wring[whead++ % NW];
  v.t0_us = t0_us;
  v.t1_us = t1_us;
  v.woken = woken;
  v.av = (int16_t)avail;
}
void steptime_onStep(uint8_t seq) { steptime::onStep(seq); }
void steptimeReport(bool reset) { steptime::report(reset); }
void steptimeService() { steptime::service(); }
#endif
