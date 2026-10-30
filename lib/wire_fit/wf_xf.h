// Inline software floating point on whole-number instructions for the wire solve cost loop (solve speed study,
// 2026-10-29, test only). Reason: the ESP32-S2 has no FPU; a ROM float multiply costs 66 cycles and an add 62
// (measured), plus the call. Here a value is a 32-bit signed mantissa m and an exponent e: value = m * 2^e, with
// |m| in [2^30, 2^31) (zero: m = 0, e = XF_ZERO_E). A multiply is one MULSH and a normalise (NSAU); an add is an
// align, an add and a normalise. No rounding (truncation), no infinities, NaN or subnormals: the caller keeps the
// values finite (the solve clamps its denominators). Precision: 29-30 bits against 24 bits for float.
// Bit-identical on the PC and the board (whole-number operations only, plus the float conversions).
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace wirefit {

struct xf { int32_t m; int32_t e; };
// XF_NOINLINE: add, reciprocal, log and atan2 as small called functions; the multiply stays inline (about 10
// instructions). Reason: the ESP32-S2 instruction cache is 8 kB (Arduino sdkconfig); fully inlined, the cost loop
// grows to about 13 kB of flash code. XF_COUNT: count each operation (PC study only).
#if defined(XF_NOINLINE) && defined(__GNUC__)
#define XF_FN __attribute__((noinline)) static
#else
#define XF_FN inline
#endif
#if defined(XF_MUL_NOINLINE) && defined(__GNUC__)
#define XF_FNM __attribute__((noinline)) static
#else
#define XF_FNM inline
#endif
#if defined(XF_NOINLINE) && defined(__GNUC__) && !defined(__clang__)
#define XF_NOUNROLL _Pragma("GCC unroll 1")
#else
#define XF_NOUNROLL
#endif
#ifdef XF_COUNT
extern long xfCount[6];   // mul, add, recip, log, atan2, from float
#define XF_CNT(i) (xfCount[i]++)
#else
#define XF_CNT(i) ((void)0)
#endif
const int32_t XF_ZERO_E = -100000;

inline int xclz(uint32_t a) {   // a != 0
#if defined(_MSC_VER)
  unsigned long i; _BitScanReverse(&i, a); return 31 - (int)i;
#else
  return __builtin_clz(a);
#endif
}
inline int32_t xmulsh(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 32); }

inline xf xnorm(int32_t m, int32_t e) {
  if (m == 0) return {0, XF_ZERO_E};
  const uint32_t a = m < 0 ? 0u - (uint32_t)m : (uint32_t)m;
  const int s = xclz(a) - 1;
  if (s < 0) return {m >> 1, e + 1};
  return {(int32_t)((uint32_t)m << s), e - s};
}
inline xf xfromf(float f) {
  XF_CNT(5);
  uint32_t b; memcpy(&b, &f, 4);
  const int32_t e8 = (int32_t)((b >> 23) & 255u);
  if (e8 == 0) return {0, XF_ZERO_E};                      // zero and subnormals: zero
  const int32_t m = (int32_t)(((b & 0x7FFFFFu) | 0x800000u) << 7);
  return {(b >> 31) ? -m : m, e8 - 157};
}
inline float xtof(xf x) {
  if (x.m == 0) return 0.0f;
  uint32_t a = x.m < 0 ? 0u - (uint32_t)x.m : (uint32_t)x.m;
  uint32_t mant = (a + 64u) >> 7;                          // round to 24 bits
  int32_t e8 = x.e + 157;
  if (mant >= 0x1000000u) { mant >>= 1; e8++; }
  if (e8 <= 0) return 0.0f;
  if (e8 >= 255) e8 = 254, mant = 0xFFFFFFu;
  const uint32_t b = ((x.m < 0) ? 0x80000000u : 0u) | ((uint32_t)e8 << 23) | (mant & 0x7FFFFFu);
  float f; memcpy(&f, &b, 4); return f;
}
XF_FNM xf xmul(xf a, xf b) {   // a zero (m = 0) gives p = 0 and xnorm returns zero
  XF_CNT(0);
  const int32_t p = xmulsh(a.m, b.m);                      // |p| in [2^28, 2^30]
  return xnorm(p, a.e + b.e + 32);
}
inline xf xmuli(xf a, int32_t k) { return {a.m, a.e + k}; }   // a * 2^k
inline xf xneg(xf a) { return {-a.m, a.e}; }
XF_FN xf xadd(xf a, xf b) {
  XF_CNT(1);
  if (a.e < b.e) { const xf t = a; a = b; b = t; }
  const int32_t d = a.e - b.e;
  if (d > 30) return a;                                    // b below the last bit of a
  return xnorm((a.m >> 1) + (b.m >> (d + 1)), a.e + 1);
}
inline xf xsub(xf a, xf b) { return xadd(a, xneg(b)); }
// 1 / b (b != 0): Newton from a linear start; 3 steps, error under 2^-31
XF_FN xf xrecip(xf b) {
  XF_CNT(2);
  const bool neg = b.m < 0;
  const uint32_t U = (neg ? 0u - (uint32_t)b.m : (uint32_t)b.m) << 1;   // u * 2^32, u in [0.5, 1)
  uint32_t X = 3031741620u - (uint32_t)(((uint64_t)U * 2021161080u) >> 32);   // (48 - 32 u) / 17, Q30
  for (int i = 0; i < 3; i++) {
    const int32_t err = (int32_t)(1073741824u - (uint32_t)(((uint64_t)U * X) >> 32));   // 1 - u x, Q30
    X += (uint32_t)(int32_t)(((int64_t)(int32_t)(X >> 1) * err) >> 29);             // x += x (1 - u x)
  }
  // 1 / b = x * 2^-30 * 2^(-31 - e): X in (2^30, 2^31]
  const xf r = xnorm((int32_t)(X >> 1), -60 - b.e);
  return neg ? xneg(r) : r;
}
inline xf xdiv(xf a, xf b) { return xmul(a, xrecip(b)); }
inline bool xgt(xf a, xf b) { return xsub(a, b).m > 0; }
// natural log of a positive value: the fastLogf polynomial (wf_fastmath.h) on the mantissa
XF_FN xf xlog(xf x) {
  XF_CNT(3);
  int32_t E = x.e + 30;                                    // value = M * 2^E, M = m / 2^30 in [1, 2)
  const uint32_t m = (uint32_t)x.m;
  int32_t z;
  if (m > 1518500249u) { E++; z = (int32_t)(m - 2147483648u); }   // M / 2 - 1, Q31
  else z = (int32_t)((m - 1073741824u) << 1);                       // M - 1, Q31
  static const int32_t C[10] = {1073741824, -536870786, 357913841, -268452476, 214770609,
                                -178339138, 152257436, -142172466, 138664848, -82018762};
  int32_t acc = C[9];
  for (int k = 8; k >= 0; k--) acc = C[k] + (int32_t)(((int64_t)acc * z) >> 31);
  const int32_t lm = (int32_t)(((int64_t)acc * z) >> 30);           // ln(M'), Q31 (|ln| < 0.35)
  if (E == 0) return xnorm(lm, -31);
  const int64_t r = ((int64_t)lm >> 1) + (int64_t)E * 744261118;    // Q30
  // |r| < 2^31 * 2^? : E up to about +-200, r up to 1.5e11: normalise from 64 bits
  int64_t ar = r < 0 ? -r : r;
  int sh = 0;
  while (ar >= ((int64_t)1 << 31)) { ar >>= 1; sh++; }
  return xnorm((int32_t)(r >> sh), -30 + sh);
}

// sin and cos of a float angle (|x| < 1e5 rad; else the library): the quadrant from one float multiply, the remainder
// exact in 64-bit whole numbers (pi/2 in Q40), Taylor polynomials on [-pi/4, pi/4] in Q30 (error about 1e-9).
// Reason: the library sinf and cosf run from flash, about 6,400 cycles a call in operation (measured)
inline void xsincos(float x, float& so, float& co) {
  if (!(fabsf(x) < 1e5f)) { so = sinf(x); co = cosf(x); return; }
  uint32_t b; memcpy(&b, &x, 4);
  const int32_t e8 = (int32_t)((b >> 23) & 255u);
  int64_t X = 0;                                            // x in Q40
  if (e8 != 0) {
    const int64_t mant = (int64_t)((b & 0x7FFFFFu) | 0x800000u);
    const int32_t sh = e8 - 150 + 40;                       // value = mant * 2^(e8 - 150)
    X = sh >= 0 ? (mant << sh) : (sh > -63 ? (mant >> -sh) : 0);
    if (b >> 31) X = -X;
  }
  const float kf = x * 0.636619772f;
  const int32_t k = (int32_t)(kf + (kf >= 0 ? 0.5f : -0.5f));
  const int64_t r40 = X - (int64_t)k * 1727108826179LL;     // pi/2 in Q40
  const int32_t r = (int32_t)(r40 >> 9);                    // Q31, |r| <= about pi/4
  const int32_t t = (int32_t)(((int64_t)r * r) >> 31);      // r^2, Q31
  static const int32_t SP[7] = {1073741824, -178956971, 8947849, -213044, 2959, -27, 0};
  static const int32_t CP[8] = {1073741824, -536870912, 44739243, -1491308, 26631, -296, 2, 0};
  int32_t a = SP[6];
  for (int i = 5; i >= 0; i--) a = SP[i] + (int32_t)(((int64_t)a * t) >> 31);
  const int32_t sv = (int32_t)(((int64_t)r * a) >> 30);     // sin(r), Q31
  int32_t c = CP[7];
  for (int i = 6; i >= 0; i--) c = CP[i] + (int32_t)(((int64_t)c * t) >> 31);
  const xf S = xnorm(sv, -31), C = xnorm(c, -30);           // cos(r) in Q30 (up to 1)
  switch (k & 3) {
    case 0: so = xtof(S); co = xtof(C); break;
    case 1: so = xtof(C); co = -xtof(S); break;
    case 2: so = -xtof(S); co = -xtof(C); break;
    default: so = -xtof(C); co = xtof(S); break;
  }
}

}  // namespace wirefit
