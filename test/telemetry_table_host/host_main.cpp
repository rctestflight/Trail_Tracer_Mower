// Desk test of the table values (test/telemetry_table_host/run.py): lib/tlm as the firmware builds it, on the PC.
// Writes the records as "LOG <hex>" lines (the USB "log dump" form) and each packed value with its sample time
// (tlm_trace, the TRUTH file). Time is simulated: tlm::service() every 7 to 13 ms, a 70 ms gap every 7 s, log rate
// 20 Hz, then 100 Hz, then 5 Hz.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tlm.h"

static FILE* f_rec = nullptr;
static FILE* f_truth = nullptr;
static uint32_t seq[2] = {0, 0};
static const uint32_t BOOT = 0x12345678u;
static unsigned long too_long = 0;
static double bytes_by_rate[3][2];  // [log rate phase][stream]: LOG frame bytes (14 B header + body)
static int phase = 0;

static void hex_le(FILE* f, uint32_t v) {
  for (int i = 0; i < 4; i++) fprintf(f, "%02x", (unsigned)((v >> (8 * i)) & 0xFF));
}

static uint32_t sink(int stream, uint32_t ms, const uint8_t* body, size_t len) {
  if (len > (size_t)(stream ? 84 : 228)) too_long++;
  fprintf(f_rec, "LOG 40%02x", stream);
  hex_le(f_rec, BOOT);
  hex_le(f_rec, seq[stream]);
  hex_le(f_rec, ms);
  for (size_t i = 0; i < len; i++) fprintf(f_rec, "%02x", body[i]);
  fprintf(f_rec, "\n");
  bytes_by_rate[phase][stream] += 14.0 + (double)len;
  return seq[stream]++;
}

static const char* const NAMES[TLM_COUNT + 1] = {
#define TLM(name, type, scale, unit, decimals, rate, group, label) #name,
#include "telemetry_table.h"
#undef TLM
    ""};

void tlm_trace(uint8_t body, uint32_t t, int id, float v) {
  fprintf(f_truth, "%u %lu %s %.9g\n", (unsigned)body, (unsigned long)t, NAMES[id], (double)v);
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  f_rec = fopen(argv[1], "w");
  f_truth = fopen(argv[2], "w");
  tlm::init(sink);
  printf("HASH %08lx COUNT %d BYTES 1hz=%d 10hz=%d log=%d\n", (unsigned long)tlm::hash(), tlm::count(), TLM_BYTES_1HZ,
         TLM_BYTES_10HZ, TLM_BYTES_LOG);
#ifdef TELEMETRY_NEWVALUE
  srand(1);
  const uint32_t T0 = 1000, PHASE_MS = 200000;
  uint32_t t = T0;
  int calls = 0;
  while (t < T0 + 3 * PHASE_MS) {
    // "Frame path": the values change every update period (2960 us); here they are set before each service call.
    const double s = (t - T0) / 1000.0;
    const uint32_t frames = (uint32_t)((t - T0) * 1000.0 / 2960.0);
    if (t >= T0 + 2000) tlm_set(TLM_test_frames, (float)(frames & 0xFFFFu));  // the first 2 s: never set ("no value")
    float r1 = (float)(300.0 + 200.0 * sin(s));
    if (fmod(s, 30.0) >= 5.0 && fmod(s, 30.0) < 6.0) r1 = NAN;   // no reading
    if (fmod(s, 30.0) >= 8.0 && fmod(s, 30.0) < 9.0) r1 = 70000.0f;  // over the U16 range: clamped
    tlm_set(TLM_test_r1_mm, r1);
    float drive = (float)(1.2 * sin(s * 3.0));
    if (fmod(s, 30.0) >= 10.0 && fmod(s, 30.0) < 11.0) drive = -4.0f;  // under the I16 range at scale 0.0001
    tlm_set(TLM_test_drive, drive);
    tlm_set(TLM_test_heap_kb, (float)(150.0 + fmod(s, 17.0)));
    phase = (int)((t - T0) / PHASE_MS);
    const int rate = phase == 0 ? 20 : phase == 1 ? 100 : 5;
    tlm::service(t, rate);
    calls++;
    uint32_t step = 7 + (uint32_t)(rand() % 7);
    if ((t - T0) % 7000 < step) step += 70;
    t += step;
  }
  printf("SERVICE calls=%d end_ms=%lu\n", calls, (unsigned long)t);
  for (int p = 0; p < 3; p++)
    printf("RATE phase=%d log_hz=%d stream0_Bps=%.1f stream1_Bps=%.1f\n", p, p == 0 ? 20 : p == 1 ? 100 : 5,
           bytes_by_rate[p][0] / (PHASE_MS / 1000.0), bytes_by_rate[p][1] / (PHASE_MS / 1000.0));
  // Two bad records for the decoder: a wrong table hash, and a right hash with one byte short.
  {
    uint8_t b[7 + 4] = {0x36, 0, 0, 0, 0, 1, 20, 1, 2, 3, 4};
    const uint32_t h = tlm::hash() ^ 1u;
    memcpy(b + 1, &h, 4);
    sink(0, t, b, sizeof(b));
    const uint32_t g = tlm::hash();
    memcpy(b + 1, &g, 4);
    sink(0, t + 1, b, sizeof(b) - 1);
  }
#endif
  printf("TOO_LONG %lu\n", too_long);
  fclose(f_rec);
  fclose(f_truth);
  return 0;
}
