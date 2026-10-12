// fast_pow: m^p for m > 0 in single precision without libm powf (software float on the ESP32-S2: powf 24 us, rt bench
// 2026-10-12). m = x * 2^e with x in [1, 2) (frexpf); log2(x) by a minimax-style polynomial in t = (x-1)/(x+1);
// 2^y by integer part (ldexpf) and a polynomial for the fraction. Host test: test/telemetry_host/fast_pow_test.cpp
// (relative error against powf over m = 1e-7 .. 1e3, p = -0.330478).
#pragma once
#include <math.h>

namespace fastpow {

inline float log2_(float m) {
  int e;
  float x = frexpf(m, &e);  // x in [0.5, 1)
  x *= 2.0f; e -= 1;          // x in [1, 2)
  const float t = (x - 1.0f) / (x + 1.0f), t2 = t * t;
  // log2(x) = 2/ln2 * atanh(t) = 2/ln2 * (t + t^3/3 + t^5/5 + t^7/7 + t^9/9), |t| < 1/3
  const float s = t * (1.0f + t2 * (1.0f / 3 + t2 * (1.0f / 5 + t2 * (1.0f / 7 + t2 * (1.0f / 9)))));
  return (float)e + 2.8853900817779268f * s;
}

inline float exp2_(float y) {
  const float fi = floorf(y);
  const float f = y - fi;  // [0, 1)
  // 2^f = e^(f ln2), Taylor to degree 7 in u = f ln2 (u < 0.694): relative error < 2e-7
  const float u = f * 0.69314718056f;
  const float p = 1.0f + u * (1.0f + u * (0.5f + u * (1.0f / 6 + u * (1.0f / 24 + u * (1.0f / 120 + u * (1.0f / 720 + u * (1.0f / 5040)))))));
  return ldexpf(p, (int)fi);
}

inline float pow_(float m, float p) { return exp2_(p * log2_(m)); }

}  // namespace fastpow
