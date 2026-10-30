// frame_period: constants that count packets, kept in time at any update period (2026-10-30, for the 20 ms update
// period). The firmware was tuned at 10 ms packets; each count, step and filter weight below
// was a value per packet. The period factor r = update period / 10 ms converts them:
//   a count of N packets       max(1, round(N / r))
//   a step per packet          step x r
//   a filter weight a          1 - (1 - a)^r          (new = a x input + (1 - a) x old, once per packet)
// r snaps to the nearest whole number when within 5 % of it (the Rx rounds 10000 us to 9930 us: r = 1; 20000 us to
// 19860 us: r = 2). Under the control gate (update period under 9 ms, src/platform/rx_link.inc) the control step runs
// about each 10 ms whatever the period: r = 1. At r = 1 each function returns its input unchanged, so every value
// equals the value before this change bit for bit. No Arduino dependency: builds on the PC (test/rover_emu).
#pragma once
#include <math.h>

namespace fperiod {

const double BASE_S = 0.010;        // the update period the packet constants were tuned at
const double GATE_S = 0.009;        // the control gate: CONTROL_PERIOD_US x 0.9 (src/platform/rx_link.inc)
const double SNAP = 0.05;           // snap to a whole number within 5 %

// The period factor of an update period in s.
inline double ratio(double period_s) {
  if (!(period_s >= GATE_S)) return 1.0;
  const double r = period_s / BASE_S, n = floor(r + 0.5);
  return (n >= 1.0 && fabs(r - n) <= SNAP * n) ? n : r;
}

// A count of n packets at period factor r.
inline int count(int n, double r) {
  if (r == 1.0) return n;
  const long k = lround(n / r);
  return k < 1 ? 1 : (int)k;
}

// A step per packet at period factor r.
inline double step(double s, double r) { return r == 1.0 ? s : s * r; }

// A filter weight per packet at period factor r.
// (single precision: expf and logf; double pow costs about 5 kB of flash on the ESP32-S2)
inline float weight(float a, double r) { return r == 1.0 ? a : 1.0f - expf((float)r * logf(1.0f - a)); }

// Whole steps of an integer ramp of n per packet: n x r, the fraction carried to the next packet in carry.
inline int wholeSteps(int n, double r, double& carry) {
  if (r == 1.0) return n;
  carry += n * r;
  const int k = (int)carry;
  carry -= k;
  return k;
}

}  // namespace fperiod
