// Desk test of the fixed records (test/telemetry_table_host/run.py): lib/rover_telemetry packs every fixed record on the
// PC as the firmware does, with random values and the edge values (NaN, infinity, over and under the range, the "no value"
// cases). Writes the records as "LOG <hex>" lines (the USB "log dump" form). The sweep records (0x33, 0x32) are packed
// here in the layout of src/main.cpp sweepAdd(); the firmware has no library function for them.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rover_records.h"

static FILE* f_rec = nullptr;
static uint32_t seq[2] = {0, 0};
static uint32_t boot = 0x0BADC0DEu, t_ms = 1000;
static unsigned long count_by_body[256];

static uint32_t rnd32() { return ((uint32_t)rand() << 17) ^ ((uint32_t)rand() << 3) ^ (uint32_t)rand(); }
static double unit() { return rand() / (double)RAND_MAX; }
static float pick(float lo, float hi) { return (float)(lo + (hi - lo) * unit()); }
// A float with the edge cases: NaN, infinities, far outside the range, and the plain range.
static float edgy(float lo, float hi) {
  const int r = rand() % 40;
  if (r == 0) return NAN;
  if (r == 1) return INFINITY;
  if (r == 2) return -INFINITY;
  if (r == 3) return hi * 3.0f + 7.0f;
  if (r == 4) return lo * 3.0f - 7.0f;
  return pick(lo, hi);
}

static void hex_le(FILE* f, uint32_t v) {
  for (int i = 0; i < 4; i++) fprintf(f, "%02x", (unsigned)((v >> (8 * i)) & 0xFF));
}

static void put(int stream, const uint8_t* body, size_t len) {
  fprintf(f_rec, "LOG 40%02x", stream);
  hex_le(f_rec, boot);
  hex_le(f_rec, seq[stream]++);
  hex_le(f_rec, t_ms);
  for (size_t i = 0; i < len; i++) fprintf(f_rec, "%02x", body[i]);
  fprintf(f_rec, "\n");
  count_by_body[body[0]]++;
  t_ms += 1 + rand() % 50;
}

static rec::Sample sample(bool imu) {
  rec::Sample s;
  s.ms = t_ms;
  s.x1_mm = edgy(-40000, 40000);
  s.y1_mm = edgy(-40000, 40000);
  s.r1_mm = edgy(-100, 70000);
  s.flags = (uint8_t)rand();
  const int a = rand() % 10;
  s.ap = a == 0 ? rec::AP_OFF : (uint8_t)(rand() % 8 << 4 | rand() % 16);  // states 6, 7: unknown codes
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

static void blocks(int layout, int count) {
  static const uint8_t rates[] = {100, 50, 25, 20, 10, 5, 4, 2, 1};
  rec::Block blk(layout);
  for (int k = 0; k < count; k++) {
    const bool imu = rand() % 2;
    const uint8_t rate = rates[rand() % 9];
    const int n = 1 + rand() % 7;
    blk.clear();
    for (int i = 0; i < n; i++) {
      rec::Sample s = sample(imu);
      s.ms = t_ms + (uint32_t)(i * (rand() % 300));
      if (blk.add(s, rate)) break;  // full
    }
    put(0, blk.body(), (size_t)blk.size());
  }
}

static void statuses(int count, bool coil_mag_boot) {
  for (int k = 0; k < count; k++) {
    rec::Status s;
    s.drive_mode = (uint8_t)(rand() % 3);
    s.charge_step = (int8_t)(rand() % 20 - 5);
    s.vehicle_status = (uint8_t)(rand() % 3);
    s.vbatt = edgy(-1, 70);
    s.vcharge = edgy(-1, 70);
    s.icharge = edgy(-40, 40);
    s.v5v = edgy(0, 6);
    s.temperature = edgy(-200, 200);
    s.ap_state = rand() % 4 ? (uint8_t)(rand() % 6) : rec::AP_OFF;
    s.ap_sub = (uint8_t)(rand() % 16);
    s.flags = (uint8_t)rand();
    s.status_bits = rnd32();
    if (coil_mag_boot) s.status_bits |= 1u << 24;
    s.charge_code = rnd32();
    s.crc_errors = rnd32();
    s.overruns = rnd32();
    s.packets_last_s = rand() % 5 ? (uint32_t)(rand() % 400) : rnd32();
    s.loop_max_us = rand() % 5 ? (uint32_t)(rand() % 20000) : rnd32();
    s.bounces = rnd32();
    s.turnarounds = rnd32();
    s.charges = rnd32();
    s.odrive_vbus = edgy(0, 70);
    s.odrive_ibus = edgy(-40, 40);
    for (int i = 0; i < 6; i++) s.axis_state[i] = (uint8_t)rand();
    s.axis_errors_or = rnd32();
    s.error_nodes = (uint8_t)rand();
    s.heartbeat_nodes = (uint8_t)rand();
    s.free_heap = rnd32();
    s.send_fail = rnd32();
    s.backfill_last_s = rand() % 5 ? (uint32_t)(rand() % 100) : rnd32();
    s.xf_valid = rand() % 3 != 0;
    s.xf_m = edgy(-40, 40);
    s.dist_m = edgy(-1, 70);
    s.angle_deg = edgy(-10, 300);
    s.monitor_age_s = (uint8_t)rand();
    s.pc_unix_ms = rand() % 4 ? 1760000000000ull + (uint64_t)rnd32() * 1000ull + (uint64_t)(rand() % 1000) : 0;
    s.fw_ver[0] = 0;
    s.fw_ver[1] = (uint8_t)(rand() % 6);
    s.fw_ver[2] = (uint8_t)rand();
    if (coil_mag_boot) { s.fw_ver[1] = 5; s.fw_ver[2] = 115; }
    s.dsp_period_us = rand() % 4 ? (uint32_t)(2354 + rand() % 10000) : 0;
    s.cpu_busy = (uint8_t)(rand() % 101);
    s.cpu_loop = (uint8_t)(rand() % 101);
    s.cpu_drive = (uint8_t)(rand() % 101);
    uint8_t b[rec::STATUS_BYTES];
    rec::packStatus(b, s);
    // Old firmware lengths too (0.5.20: 67, 0.5.21: 68, 0.5.22: 74, 0.5.24: 77, 0.5.97: 80): the first bytes as sent then.
    static const int lens[] = {67, 68, 74, 77, 80};
    const size_t len = rand() % 6 == 0 ? (size_t)lens[rand() % 5] : (size_t)rec::STATUS_BYTES;
    put(1, b, len);
  }
}

static void events(int count) {
  static const char* texts[] = {"", "fw=0.5.116-telem cfg=1a2b3c4d", "set telemetry_rate_hz 20 -> telemetry_rate_hz=20",
                                "read-back cmd 6 got 0.000 want 11.900: send all again"};
  for (int k = 0; k < count; k++) {
    char text[200];
    if (rand() % 4) snprintf(text, sizeof(text), "%s", texts[rand() % 4]);
    else {  // long text, printable ASCII and a few bytes over 127
      const int n = rand() % 120;
      for (int i = 0; i < n; i++) text[i] = rand() % 30 ? (char)(32 + rand() % 95) : (char)(128 + rand() % 120);
      text[n] = 0;
    }
    const uint16_t code = (uint16_t)(rand() % 6 == 0 ? 18 : rand() % 23);
    const int32_t a = code == 18 ? rand() % 12 - 1 : (int32_t)rnd32();
    uint8_t b[rec::EVENT_MAX];
    const int n = rec::packEvent(b, code, a, (int32_t)rnd32(), text);
    put(1, b, (size_t)n);
  }
}

static void names(int count) {
  for (int k = 0; k < count; k++) {
    char name[40];
    const int n = rand() % 22;
    for (int i = 0; i < n; i++) name[i] = "AZaz09 _-"[rand() % 9];
    name[n] = 0;
    uint8_t b[3 + rec::VEHICLE_NAME_MAX];
    const int len = rec::packName(b, (uint8_t)rand(), name);
    put(1, b, (size_t)len);
  }
}

static void bins(int count) {
  for (int k = 0; k < count; k++) {
    float hz[rec::BINS];
    int32_t v[rec::BINS][4];
    for (int b = 0; b < rec::BINS; b++) {
      hz[b] = rand() % 5 ? pick(1000, 130000) : (rand() % 2 ? 0.0f : 200000.0f);
      for (int c = 0; c < 4; c++) {
        const int r = rand() % 10;
        v[b][c] = r == 0 ? rec::BINS_NONE : r == 1 ? (int32_t)rnd32() : (int32_t)(rand() % 2000000 - 1000000);
      }
    }
    uint8_t body[rec::BINS_BYTES];
    rec::packBins(body, (uint8_t)rand(), (uint16_t)rand(), hz, v);
    put(0, body, rec::BINS_BYTES);
  }
}

static void put24(uint8_t* p, int32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
}

static void sweeps(int count) {
  for (int k = 0; k < count; k++) {
    uint8_t o[228];
    const bool raw = rand() % 3 != 0;
    const uint32_t f0 = rnd32() & 0xFFFFFF;
    const uint16_t step = (uint16_t)rand(), pts = (uint16_t)(1 + rand() % 400);
    const int n = 1 + rand() % 6;
    o[0] = raw ? rec::BODY_SWEEP_RAW : rec::BODY_SWEEP;
    o[1] = (uint8_t)rand();
    put24(o + 2, (int32_t)f0);
    o[5] = (uint8_t)step;
    o[6] = (uint8_t)(step >> 8);
    o[7] = (uint8_t)pts;
    o[8] = (uint8_t)(pts >> 8);
    o[9] = (uint8_t)n;
    int len;
    if (raw) {
      o[10] = (uint8_t)rand();
      const int u = 1 + rand() % ((228 - 12) / (6 + 12 * n));
      o[11] = (uint8_t)u;
      len = 12;
      uint16_t tag = (uint16_t)(rand() % pts);
      for (int f = 0; f < u; f++) {
        const bool page = rand() % 3 == 0;
        o[len] = (uint8_t)(k + f);
        o[len + 1] = (uint8_t)((page ? 1 : 0) | (rand() % 10 == 0 ? 2 : 0));
        const uint16_t t = page ? tag : 0xFFFF, tp = page ? (uint16_t)((tag + pts - n) % pts) : 0xFFFF;
        o[len + 2] = (uint8_t)t; o[len + 3] = (uint8_t)(t >> 8);
        o[len + 4] = (uint8_t)tp; o[len + 5] = (uint8_t)(tp >> 8);
        len += 6;
        for (int i = 0; i < 4 * n; i++) { put24(o + len, rand() % 8 ? rand() % 2000000 - 1000000 : rec::BINS_NONE); len += 3; }
        tag = (uint16_t)((tag + n) % pts);
      }
    } else {
      const int u = 1 + rand() % ((228 - 11) / (14 * n));
      o[10] = (uint8_t)u;
      len = 11;
      for (int i = 0; i < u * n; i++) {
        const uint16_t g = (uint16_t)(rand() % pts);
        o[len] = (uint8_t)g; o[len + 1] = (uint8_t)(g >> 8);
        len += 2;
        for (int c = 0; c < 4; c++) { put24(o + len, rand() % 8 ? rand() % 2000000 - 1000000 : rec::BINS_NONE); len += 3; }
      }
    }
    put(0, o, (size_t)len);
  }
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  f_rec = fopen(argv[1], "w");
  srand(7);
  // Boot 1: a status record with Rx STM32 status bit 24 and firmware 0.5.115 first, so its samples get "coil_mag".
  statuses(1, true);
  blocks(3, 2000);
  blocks(1, 600);
  boot = 0x12345679u;
  seq[0] = seq[1] = 0;
  statuses(3000, false);
  blocks(3, 2000);
  blocks(1, 600);
  events(3000);
  names(500);
  bins(1500);
  sweeps(600);
  fclose(f_rec);
  for (int b = 0; b < 256; b++)
    if (count_by_body[b]) printf("BODY 0x%02X %lu\n", b, count_by_body[b]);
  return 0;
}
