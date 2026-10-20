// Desk test of the telemetry page answers (test/web_telemetry_host/run.py). The firmware code runs on the PC:
// lib/rover_telemetry packs random status records and samples (with the edge values), lib/telemlog holds the status
// records among other SLOW records as on the board, lib/tlm_page takes the snapshot (take_status_log, take_sample,
// take_values) and formats the answers. Writes, per case, the log records that hold the newest status and the newest
// sample, the table values set, and both answers (short set and all=1). run.py decodes the records with
// tools/rover_log.py and compares every page value with the log value.
//
// Usage: test.exe <out file> <cases> <seed>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "rover_records.h"
#include "telemlog.h"
#include "tlm.h"
#include "tlm_page.h"

static uint32_t rnd32() { return ((uint32_t)rand() << 17) ^ ((uint32_t)rand() << 3) ^ (uint32_t)rand(); }
static double unit() { return rand() / (double)RAND_MAX; }
static float pick(float lo, float hi) { return (float)(lo + (hi - lo) * unit()); }
static float edgy(float lo, float hi) {
  const int r = rand() % 40;
  if (r == 0) return NAN;
  if (r == 1) return INFINITY;
  if (r == 2) return -INFINITY;
  if (r == 3) return hi * 3.0f + 7.0f;
  if (r == 4) return lo * 3.0f - 7.0f;
  return pick(lo, hi);
}

static bool to_string(void* ctx, const char* t, size_t n) {
  ((std::string*)ctx)->append(t, n);
  return true;
}

static void hex(FILE* f, const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) fprintf(f, "%02x", b[i]);
}

static rec::Sample sample(uint32_t ms, bool imu) {
  rec::Sample s;
  s.ms = ms;
  s.x1_mm = edgy(-40000, 40000);
  s.y1_mm = edgy(-40000, 40000);
  s.r1_mm = edgy(-100, 70000);
  s.flags = (uint8_t)rand();
  s.ap = rand() % 10 == 0 ? rec::AP_OFF : (uint8_t)(rand() % 8 << 4 | rand() % 16);  // states 6, 7: unknown codes
  s.drive = edgy(-4, 4);
  s.steer = edgy(-4, 4);
  s.vel0_rps = edgy(-40, 40);
  s.vel1_rps = edgy(-40, 40);
  s.gyro = rand() % 5 != 0;
  s.gyro_dps = edgy(-700, 700);
  s.imu = imu;
  s.ax_g = edgy(-40, 40);
  s.ay_g = edgy(-40, 40);
  s.az_g = edgy(-40, 40);
  s.gx_dps = edgy(-700, 700);
  s.gy_dps = edgy(-700, 700);
  s.btx1_mm = rand() % 4 ? edgy(-10, 1100) : 1e9f;
  s.btx2_mm = rand() % 4 ? edgy(-10, 1100) : 1e9f;
  for (int i = 0; i < 4; i++) s.coil[i] = rand() % 30 ? (float)ldexp(pick(-1, 1), rand() % 60 - 30) : edgy(-1e30f, 1e30f);
  s.xf_valid = rand() % 4 != 0;
  s.xf_m = edgy(-40, 40);
  s.btx1_s2_mm = rand() % 4 ? edgy(-10, 1100) : 1e9f;
  s.btx2_s2_mm = rand() % 4 ? edgy(-10, 1100) : 1e9f;
  s.enc_pos[0] = (int32_t)rnd32();
  s.enc_pos[1] = rand() % 3 ? (int32_t)(rand() % 200000 - 100000) : (int32_t)rnd32();
  s.enc_age_ms[0] = rand() % 5 ? (uint8_t)(rand() % 255) : 255;
  s.enc_age_ms[1] = (uint8_t)rand();
  s.x2_mm = rand() % 6 ? edgy(-40000, 40000) : NAN;
  s.y2_mm = rand() % 6 ? edgy(-40000, 40000) : NAN;
  s.heading = (uint16_t)rand();
  return s;
}

static rec::Status status() {
  rec::Status st;
  st.drive_mode = (uint8_t)(rand() % 3);
  st.charge_step = (int8_t)(rand() % 12 - 2);
  st.vehicle_status = (uint8_t)(rand() % 3);
  st.vbatt = edgy(0, 70);
  st.vcharge = edgy(0, 70);
  st.icharge = edgy(-40, 40);
  st.v5v = edgy(0, 70);
  st.temperature = edgy(-130, 130);
  st.ap_state = rand() % 3 ? rec::AP_OFF : (uint8_t)(rand() % 6);
  st.ap_sub = (uint8_t)(rand() % 16);
  st.flags = (uint8_t)rand();
  st.status_bits = rnd32();
  st.charge_code = rnd32();
  st.crc_errors = rnd32();
  st.overruns = rnd32();
  st.packets_last_s = rand() % 500;
  st.loop_max_us = rnd32() % 100000;
  st.bounces = rnd32();
  st.turnarounds = rand();
  st.charges = rand();
  st.odrive_vbus = edgy(0, 70);
  st.odrive_ibus = edgy(-40, 40);
  for (int i = 0; i < 6; i++) st.axis_state[i] = (uint8_t)rand();
  st.axis_errors_or = rnd32();
  st.error_nodes = (uint8_t)rand();
  st.heartbeat_nodes = (uint8_t)rand();
  st.free_heap = rnd32() % 300000;
  st.send_fail = rnd32();
  st.backfill_last_s = rand();
  st.xf_valid = rand() % 4 != 0;
  st.xf_m = edgy(-40, 40);
  st.dist_m = edgy(0, 70);
  st.angle_deg = edgy(-10, 300);
  st.monitor_age_s = (uint8_t)rand();
  st.pc_unix_ms = rand() % 5 ? 1760000000000ull + rnd32() : 0;
  st.fw_ver[0] = 0; st.fw_ver[1] = 5; st.fw_ver[2] = (uint8_t)(rand() % 200);
  st.dsp_period_us = rand() % 5 ? 2000 + rand() % 20000 : 0;
  st.cpu_busy = (uint8_t)(rand() % 101);
  st.cpu_loop = (uint8_t)(rand() % 101);
  st.cpu_drive = (uint8_t)(rand() % 101);
  return st;
}

int main(int argc, char** argv) {
  if (argc < 4) return 2;
  FILE* f = fopen(argv[1], "w");
  const int cases = atoi(argv[2]);
  srand((unsigned)atoi(argv[3]));
  tlm::init(nullptr);
  telemlog_init(0x12345678u, false);

  std::string def;
  tlmpage::def_answer("0.5.117-desk", to_string, &def);
  fprintf(f, "INFO count=%d short=%d missing_short=%d tlm=%d\n", tlmpage::count(), tlmpage::short_count(),
          tlmpage::missing_short(), TLM_COUNT);
  fprintf(f, "DEF %s\n", def.c_str());

  static tlmpage::Snapshot snap;  // as the firmware: one static copy, kept from request to request
  rec::Block blk(3);
  uint32_t t = 1000;
  uint8_t last_full[rec::BLOCK_BYTES];
  int last_full_n = 0;
  uint32_t last_full_ms = 0;
  size_t max_short = 0, max_all = 0, sum_short = 0, sum_all = 0;
  int kept_status = 0;
  for (int c = 0; c < cases; c++) {
    // SLOW records: a status, then 0 to 30 other records (events; some cases push the status out of the ring)
    uint8_t st_body[rec::STATUS_BYTES];
    const int st_n = rec::packStatus(st_body, status());
    const uint32_t st_ms = t;
    telemlog_add(TELEMLOG_SLOW, st_ms, st_body, (size_t)st_n);
    const int others = rand() % 8 ? rand() % 6 : rand() % 31;
    for (int k = 0; k < others; k++) {
      uint8_t ev[rec::EVENT_MAX];
      const int n = rec::packEvent(ev, (uint16_t)(1 + rand() % 20), (int32_t)rnd32(), (int32_t)rnd32(), "desk");
      telemlog_add(TELEMLOG_SLOW, t, ev, (size_t)n);
    }
    // Samples at one log rate, 1 to 6 of them, as telemetrySample() adds them
    const int rates[] = {100, 20, 5};
    const uint8_t rate = (uint8_t)rates[rand() % 3];
    const bool imu = rand() % 2;
    const int m = 1 + rand() % 6;
    for (int k = 0; k < m; k++) {
      t += 1000 / rate;
      const rec::Sample s = sample(t, imu);
      if (!blk.fits(s, rate)) blk.clear();
      if (blk.add(s, rate)) {
        last_full_n = blk.size();
        memcpy(last_full, blk.body(), last_full_n);
        last_full_ms = blk.ms();
        blk.clear();
      }
    }
    // Table values
    for (int i = 0; i < TLM_COUNT; i++) {
      const int r = rand() % 12;
      g_tlm_value[i] = r == 0 ? NAN : r == 1 ? edgy(-1e9f, 1e9f) : pick(-70000, 70000) * (float)pow(10.0, -(rand() % 5));
    }
    t += 7;

    snap.now_ms = t;
    const uint8_t had = snap.status_len;
    const uint32_t old_ms = snap.status_ms;
    tlmpage::take_status_log(&snap);
    const bool found = snap.status_ms == st_ms && snap.status_len;
    if (!found && had) kept_status++;
    tlmpage::take_sample(&snap, blk.body(), blk.ms());
    tlmpage::take_values(&snap);

    std::string a_short, a_all;
    const tlmpage::Heap h = {123456, 98765, 43210};
    tlmpage::values_answer(snap, false, h, to_string, &a_short);
    tlmpage::values_answer(snap, true, h, to_string, &a_all);
    max_short = a_short.size() > max_short ? a_short.size() : max_short;
    max_all = a_all.size() > max_all ? a_all.size() : max_all;
    sum_short += a_short.size();
    sum_all += a_all.size();

    fprintf(f, "CASE %d now=%u status_found=%d others=%d status_ms=%u old_status_ms=%u\n", c, t, found ? 1 : 0, others,
            snap.status_ms,
            had ? old_ms : 0);
    if (found) { fprintf(f, "STATUS %u ", st_ms); hex(f, st_body, st_n); fprintf(f, "\n"); }
    // The record that holds the newest sample: the open one, or the last full one after it went out
    if (blk.count() > 0) { fprintf(f, "SAMPLES %u ", blk.ms()); hex(f, blk.body(), blk.size()); }
    else { fprintf(f, "SAMPLES %u ", last_full_ms); hex(f, last_full, last_full_n); }
    fprintf(f, "\n");
    for (int i = 0; i < TLM_COUNT; i++) {
      uint32_t w;
      memcpy(&w, &g_tlm_value[i], 4);
      fprintf(f, "TLMV %d %08x\n", i, w);
    }
    fprintf(f, "SHORT %s\nALL %s\n", a_short.c_str(), a_all.c_str());
  }
  fprintf(f, "SIZES def=%u short_max=%u short_mean=%u all_max=%u all_mean=%u kept_status=%d\n", (unsigned)def.size(),
          (unsigned)max_short, (unsigned)(sum_short / cases), (unsigned)max_all, (unsigned)(sum_all / cases), kept_status);
  fclose(f);
  return 0;
}
