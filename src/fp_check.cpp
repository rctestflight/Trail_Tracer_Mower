// Diagnostic builds diag_cpu / diag_iramfp (2026-10-28, float cost study, observation only): bench "fpcheck"
// prints checksums of the float library results and of the wire solve results on recorded lawn frames. Two builds that
// print the same checksums give bit-identical results. Run it with the rover stopped: the solve part takes about 10 s
// of loop passes (one solve per pass) and 5 kB of heap while it runs.
#ifdef DIAG_SOLVE_FEED
#include <Arduino.h>
#include <math.h>
#include <new>
#include <string.h>
#include "usb_text.h"
#include "wire_fit_bounce.h"
#include "solve_timing_frames.h"

extern UsbOut usbOut;

namespace {
uint32_t crcAdd(uint32_t c, const void* p, size_t n) {
  const uint8_t* b = (const uint8_t*)p;
  c = ~c;
  for (size_t i = 0; i < n; i++) {
    c ^= b[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}
uint32_t crcF(uint32_t c, float v) { return crcAdd(c, &v, sizeof(v)); }

wirefit::WireFitBounce* w = nullptr;
int k = 0, solves = 0, valid = 0;
uint32_t crc = 0, t0 = 0;
bool verbose = false;   // "fpcheck v": one line per solve, the fields of the PC check (solvecheck.cpp)
}  // namespace

#ifndef DIAG_FP_NOFUNC
// The function sweeps: each function over its operand range, 4001 points (volatile: no compile-time folding).
static void fpCheckFunctions() {
  const int N = 4001;
  uint32_t c_log = 0, c_exp = 0, c_atan2 = 0, c_sin = 0, c_cos = 0, c_asin = 0, c_acos = 0, c_sqrt = 0, c_atan = 0;
  for (int i = 0; i < N; i++) {
    volatile float u = (float)i / (N - 1);                // 0..1
    volatile float lx = powf(10.0f, -8.0f + 16.0f * u);   // 1e-8..1e8
    c_log = crcF(c_log, logf(lx));
    c_sqrt = crcF(c_sqrt, sqrtf(lx));
    c_exp = crcF(c_exp, expf(-80.0f + 160.0f * u));
    c_sin = crcF(c_sin, sinf(-200.0f + 400.0f * u));
    c_cos = crcF(c_cos, cosf(-200.0f + 400.0f * u));
    c_asin = crcF(c_asin, asinf(-1.0f + 2.0f * u));
    c_acos = crcF(c_acos, acosf(-1.0f + 2.0f * u));
    c_atan = crcF(c_atan, atanf(-50.0f + 100.0f * u));
    const float a = 6.2831853f * u;                     // all four quadrants, radius 1e-3..1e3
    c_atan2 = crcF(c_atan2, atan2f(sinf(a) * lx, cosf(a) * lx));
  }
  usbOut.printf("FPCHECK functions logf=%08lx expf=%08lx atan2f=%08lx atanf=%08lx sinf=%08lx cosf=%08lx asinf=%08lx acosf=%08lx sqrtf=%08lx\n",
                (unsigned long)c_log, (unsigned long)c_exp, (unsigned long)c_atan2, (unsigned long)c_atan,
                (unsigned long)c_sin, (unsigned long)c_cos, (unsigned long)c_asin, (unsigned long)c_acos, (unsigned long)c_sqrt);
}

#endif

void fpCheckStart(bool v) {
  if (w) { usbOut.printf("FPCHECK busy\n"); return; }
  verbose = v;
#ifndef DIAG_FP_NOFUNC  // size-limited builds: the function sweep left out for the update slot (the solve part stays)
  fpCheckFunctions();
#endif
  w = new (std::nothrow) wirefit::WireFitBounce();
  if (!w) { usbOut.printf("FPCHECK no memory\n"); return; }
  w->reset();
  k = solves = valid = 0; crc = 0; t0 = millis();
}

// One buffer sample (10 frames, the same scaling as the solve feed) and one solve per loop pass.
void fpCheckService() {
  if (!w) return;
  for (int i = 0; i < 10 && k < SOLVE_TIMING_FRAMES; i++, k++) {
    const int16_t* f = solveTimingFrames[k];
    w->frame(f[0] * 1e-5f, f[1] * 1e-5f, f[4] > 0, f[2] * 1e-3f, f[3] * 1e-3f, f[4] * 1e-3f);
  }
  wirefit::BounceResult r;
  if (w->prepare(r)) r = w->iterate();
  if (verbose) {
    if (usbOut.room() < 300) usbOut.drain(100);
    usbOut.printf("FPS1 %d %d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d %d %d\n", solves, (int)r.valid, r.bestStart, r.d,
                  r.psiDeg, r.H, r.bg, r.psiSd, r.Hsd, r.Hc, r.cost, r.n, r.starts, r.evals);
  }
  solves++;
  if (r.valid) valid++;
  const float v[] = {r.valid ? 1.0f : 0.0f, r.d, r.psiDeg, r.betaDeg, r.H, r.cost, r.bg, r.Hsd, r.Hc, r.Hcsd, r.psiSd,
                     (float)r.n, (float)r.starts, (float)r.evals, r.refX, r.refY, r.refTh};
  crc = crcAdd(crc, v, sizeof(v));
  if (k >= SOLVE_TIMING_FRAMES) {
    usbOut.printf("FPCHECK solve frames=%d solves=%d valid=%d crc=%08lx ms=%lu\n", k, solves, valid, (unsigned long)crc,
                  (unsigned long)(millis() - t0));
    delete w;
    w = nullptr;
  }
}
#endif
