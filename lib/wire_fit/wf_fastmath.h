// Fast natural log and atan2 for the wire solve (float cost study, build flag WF_FASTMATH, 2026-10-28).
// Reason: the ESP32-S2 has no FPU; the library logf and atan2f cost 2,000-3,500 CPU cycles a call (5,800-6,500 in
// operation) and the solve calls them about 4,000 times a second. Here the polynomial runs in 32-bit fixed point
// (integer multiplies), with one float conversion at the end: a few hundred cycles a call.
// Accuracy (PC check against double precision): log error under 3e-9 (relative to the result for x in [0.71, 1.41)),
// atan2 error under 1e-8 rad (relative in the first octant), plus the float rounding of the result and of the operand
// ratio. Arguments outside the fast range (log: x <= 0, subnormal, infinite or NaN; atan2: both 0, infinite or NaN) go
// to the library functions, which give the IEEE special values.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace wirefit {

// ln(x): x = 2^e * m, m in [sqrt(1/2), sqrt(2)); ln(m) = z * q(z), z = m - 1 in Q31, q in Q30 (degree 9, least
// squares on Chebyshev nodes)
inline float fastLogf(float x) {
  uint32_t b;
  memcpy(&b, &x, sizeof(b));
  const uint32_t ex = b >> 23;                       // sign and exponent: 1..254 for positive normal x
  if (ex == 0 || ex >= 255) return logf(x);
  int32_t e = (int32_t)ex - 127;
  const uint32_t m = b & 0x7FFFFFu;
  int32_t z;
  if (m > 0x3504F3u) { e++; z = (int32_t)((m << 7) - (0x800000u << 7)); }   // m / 2 - 1, Q31
  else z = (int32_t)(m << 8);                                                 // m - 1, Q31
  static const int32_t C[10] = {1073741824, -536870786, 357913841, -268452476, 214770609,
                                -178339138, 152257436, -142172466, 138664848, -82018762};
  int32_t acc = C[9];
  for (int k = 8; k >= 0; k--) acc = C[k] + (int32_t)(((int64_t)acc * z) >> 31);
  const int64_t p = (int64_t)acc * z;                                          // ln(m), Q61
  if (e == 0) return (float)p * 4.33680869e-19f;                               // x near 1: 2^-61, full relative precision
  const int64_t r = (p >> 31) + (int64_t)e * 744261118;                         // Q30: ln(m) + e ln 2
  return (float)r * 9.31322575e-10f;                                          // 2^-30, exact scale
}

// atan2(y, x): a = min(|y|, |x|) / max(|y|, |x|) in [0, 1]; atan(a) = a * r(a^2), r in Q30 (degree 9), the product and
// the octant in float
inline float fastAtan2f(float y, float x) {
  const float ax = fabsf(x), ay = fabsf(y);
  if (!(ax < INFINITY) || !(ay < INFINITY) || (ax == 0 && ay == 0)) return atan2f(y, x);
  const bool swap = ay > ax;
  const float a = swap ? ax / ay : ay / ax;
  uint32_t bx, by;
  memcpy(&bx, &x, sizeof(bx));
  memcpy(&by, &y, sizeof(by));
  float f;
  if (a < 9.765625e-4f) {                                                     // a < 2^-10: atan(a) = a - a^3/3, error a^5/5
    f = a - a * a * a * 0.333333333f;
  } else {                                                                    // atan(a) = a * r(a^2)
    const int32_t aq = (int32_t)(a * 1073741824.0f);                          // Q30, only for a^2
    const int32_t t = (int32_t)(((int64_t)aq * aq) >> 29);                    // a^2, Q31
    static const int32_t C[10] = {1073741821, -357913393, 214729366, -153132481, 117469492,
                                  -89894352, 61701025, -32982674, 11468335, -1872285};
    int32_t acc = C[9];
    for (int k = 8; k >= 0; k--) acc = C[k] + (int32_t)(((int64_t)acc * t) >> 31);
    f = a * ((float)acc * 9.31322575e-10f);                                   // r in Q30, 2^-30 exact scale
  }
  if (swap) f = 1.57079633f - f;                                              // pi/2 - atan(a)
  if (bx >> 31) f = 3.14159265f - f;                                          // x < 0 (or -0): pi - angle
  return (by >> 31) ? -f : f;                                                 // y < 0 (or -0): the negative angle
}

}  // namespace wirefit
