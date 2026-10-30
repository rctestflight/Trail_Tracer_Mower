// Wire position solve on a filtered, decimated buffer: see wire_fit_bounce.h.
#if defined(__GNUC__) && defined(WF_OS)
#pragma GCC optimize("Os")  // test: the solve at -Os (instruction cache)
#elif defined(__GNUC__)
#pragma GCC optimize("O2")
#endif
#include "wire_fit_bounce.h"
#include <math.h>
#if defined(WF_STOP_DBG) || defined(WF_START_DBG)
#include <stdio.h>
#endif
#ifdef WF_XF
#include "wf_xf.h"
#endif
#ifdef WF_FASTMATH
#include "wf_fastmath.h"
#define WF_LOGF(x) fastLogf(x)          // test build: fixed-point log and atan2 (wf_fastmath.h)
#define WF_ATAN2F(y, x) fastAtan2f(y, x)
#else
#define WF_LOGF(x) logf(x)
#define WF_ATAN2F(y, x) atan2f(y, x)
#endif
#if defined(ESP_PLATFORM) && defined(WF_IRAM)
#include "esp_attr.h"
#define WF_FAST IRAM_ATTR             // the cost loop in internal RAM, away from flash cache misses
#else
#define WF_FAST
#endif
#ifndef WF_LAM_MAX
#define WF_LAM_MAX 1e6f              // test: the damping at which a start gives up
#endif

#ifdef DIAG_STEPTIME
// step timing diagnostic: a solve with the closed-form starts runs (src/diag_steptime.cpp)
extern volatile uint8_t g_st_heavy_now, g_st_heavy_seen;
#endif
namespace wirefit {

namespace {
const float PI_F = 3.14159265f;
const float D2R = PI_F / 180.0f;
const float NU = 4.0f;                 // Student-t degrees of freedom, as the other versions
const float H_LO = 0.01f;              // m: H > 0 (the solve divides by it)
inline float wrapPi(float a) { while (a > PI_F) a -= 2 * PI_F; while (a <= -PI_F) a += 2 * PI_F; return a; }

// Predicted reading (ln r, doubled angle) for the signed distance d, direction psi, vertical distance H; Jacobian
inline void predict(float d, float c, float s, float H, float& L, float& Phi, float J[2][3]) {
  const float q0 = H * H * c * c + d * d, q = q0 > 1e-9f ? q0 : 1e-9f, R2 = d * d + H * H, iq = 1 / q, iR = 1 / R2;
  L = 0.5f * WF_LOGF(R2 * R2 * iq);
  Phi = 2 * WF_ATAN2F(d, H * c);
  J[0][0] = 2 * d * iR - d * iq;
  J[0][1] = H * H * c * s * iq;
  J[0][2] = 2 * H * iR - H * c * c * iq;
  J[1][0] = 2 * H * c * iq;
  J[1][1] = 2 * d * H * s * iq;
  J[1][2] = -2 * d * c * iq;
}

// Predicted reading with a uniform vertical field b added to the wire's field (the front-plane field of the wire:
// lateral -H c u, vertical d u, u = 2 / R^2); Jacobian over (d, psi, H, b). With b = 0 it equals predict().
inline void predictB(float d, float c, float s, float H, float b, float& L, float& Phi, float J[2][4]) {
  const float R2 = d * d + H * H, u = 2 / R2;
  const float dud = -u * 2 * d / R2, duH = -u * 2 * H / R2;
  const float X = H * c * u, Y = d * u + b, M0 = X * X + Y * Y, M = M0 > 1e-12f ? M0 : 1e-12f, iM = 1 / M;
  L = 0.69314718f - 0.5f * WF_LOGF(M);
  Phi = 2 * WF_ATAN2F(Y, X);
  const float dX[4] = {H * c * dud, -H * s * u, c * u + H * c * duH, 0}, dY[4] = {u + d * dud, 0, d * duH, 1};
#ifdef WF_SOLVE_1B  // 2026-10-29: 6 % slower on the board (two Rx boards), faster on the PC; off by default
  // dY[1] = 0 and dX[3] = 0 left out of the rows (the same values; a zero product adds nothing)
  J[0][0] = -(X * dX[0] + Y * dY[0]) * iM; J[1][0] = 2 * (X * dY[0] - Y * dX[0]) * iM;
  J[0][1] = -(X * dX[1]) * iM;             J[1][1] = 2 * (-(Y * dX[1])) * iM;
  J[0][2] = -(X * dX[2] + Y * dY[2]) * iM; J[1][2] = 2 * (X * dY[2] - Y * dX[2]) * iM;
  J[0][3] = -Y * iM;                      J[1][3] = 2 * X * iM;   // dY[3] = 1
#else
  for (int i = 0; i < 4; i++) { J[0][i] = -(X * dX[i] + Y * dY[i]) * iM; J[1][i] = 2 * (X * dY[i] - Y * dX[i]) * iM; }
#endif
}

bool solveN(float A[4][4], const float* b, float* x, int n) {   // A x = b, Gaussian elimination with pivots
  float M[4][5];
  for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) M[i][j] = A[i][j]; M[i][n] = b[i]; }
  for (int c = 0; c < n; c++) {
    int pv = c;
    for (int r = c + 1; r < n; r++) if (fabsf(M[r][c]) > fabsf(M[pv][c])) pv = r;
    if (!(fabsf(M[pv][c]) > 1e-20f)) return false;
    if (pv != c) for (int j = 0; j <= n; j++) { const float t = M[c][j]; M[c][j] = M[pv][j]; M[pv][j] = t; }
    const float ip = 1 / M[c][c];
    for (int r = c + 1; r < n; r++) { const float f = M[r][c] * ip; for (int j = c; j <= n; j++) M[r][j] -= f * M[c][j]; }
  }
  for (int i = n - 1; i >= 0; i--) { float v = M[i][n]; for (int j = i + 1; j < n; j++) v -= M[i][j] * x[j]; x[i] = v / M[i][i]; }
  return true;
}
}  // namespace

void WireFitBounce::reset() {
  head_ = n_ = since_ = 0;
  total_ = 0;
  X_ = Y_ = T_ = S_ = 0;
  nf_ = nr_ = 0;
  sx_ = sy_ = sr_ = spx_ = spy_ = sth_ = 0;
  lx_ = ly_ = 0;
  hi_ = hcount_ = 0;
  sS_ = 0;
  warm_ = false;
  Hc_ = bc.hStart; Vc_ = bc.hStartSd * bc.hStartSd; lastS_ = 0;
}


void WireFitBounce::frame(float ds, float dTheta, bool have, float x, float y, float r) {
  const float a = T_ + 0.5f * dTheta;
  X_ += ds * cosf(a);
  Y_ += ds * sinf(a);
  T_ += dTheta;
  S_ += fabsf(ds);
  // the pose lagFrames back pairs with this reading
  hx_[hi_] = X_; hy_[hi_] = Y_; ht_[hi_] = T_; hs_[hi_] = S_;
  const int lag = bc.lagFrames < LAG ? bc.lagFrames : LAG - 1, j = (hi_ - lag + LAG) % LAG;
  hi_ = (hi_ + 1) % LAG;
  if (hcount_ < LAG) hcount_++;
  const int jj = lag < hcount_ ? j : (hi_ - hcount_ + LAG) % LAG;   // at the start: the oldest pose there is
  const float Xl = hx_[jj], Yl = hy_[jj], Tl = ht_[jj];
  const float c = cosf(Tl), s = sinf(Tl);
  spx_ += Xl + c * cfg.sensorFwd - s * cfg.sensorLeft;
  spy_ += Yl + s * cfg.sensorFwd + c * cfg.sensorLeft;
  sth_ += Tl;
  sS_ = hs_[jj];
  if (have && r > 0) {
    // sign aligned to the last reading: the field has no sign, so a reading and its negative are one reading
    if (lx_ * x + ly_ * y < 0) { x = -x; y = -y; }
    lx_ = x; ly_ = y;
    sx_ += x; sy_ += y; sr_ += r; nr_++;
  }
  if (++nf_ < bc.decim) return;
  BounceSample& b = buf_[head_];
  const float inf = 1.0f / nf_;
  b.px = spx_ * inf; b.py = spy_ * inf; b.th = sth_ * inf; b.s = sS_;
  if (2 * nr_ >= nf_) { const float inr = 1.0f / nr_; b.x = sx_ * inr; b.y = sy_ * inr; b.r = sr_ * inr; }   // half the frames read
  else b.x = b.y = b.r = 0;
  head_ = (head_ + 1) % N;
  if (n_ < N) n_++;
  if (since_ < N) since_++;
  total_++;
  nf_ = nr_ = 0;
  sx_ = sy_ = sr_ = spx_ = spy_ = sth_ = 0;
}

// Robust cost of p = (d, psi, H[, b]) over the m samples of w_; with g and A: the gradient and Gauss-Newton matrix
#ifdef WF_XF
// atan2(y, x) on whole numbers: angle in Q29 rad (the fastAtan2f polynomial, wf_fastmath.h)
XF_FN int32_t xatan2q29(xf y, xf x) {
  XF_CNT(4);
  if (y.m == 0 && x.m == 0) return 0;
  const xf ay = {y.m < 0 ? -y.m : y.m, y.e}, ax = {x.m < 0 ? -x.m : x.m, x.e};
  const bool swap = ay.m != 0 && (ax.m == 0 || ay.e > ax.e || (ay.e == ax.e && ay.m > ax.m));
  const xf a = swap ? xmul(ax, xrecip(ay)) : xmul(ay, xrecip(ax));   // [0, 1]
  int32_t aq = 0;                                                    // Q30
  if (a.m != 0) { const int32_t sh = -30 - a.e; aq = sh <= 0 ? (a.m >> 0) : (sh >= 31 ? 0 : a.m >> sh); }
  if (aq > 1073741824) aq = 1073741824;
  const int32_t t = (int32_t)(((int64_t)aq * aq) >> 29);            // a^2, Q31
  static const int32_t C[10] = {1073741821, -357913393, 214729366, -153132481, 117469492,
                                -89894352, 61701025, -32982674, 11468335, -1872285};
  int32_t acc = C[9];
  for (int k = 8; k >= 0; k--) acc = C[k] + (int32_t)(((int64_t)acc * t) >> 31);
  int32_t f = (int32_t)(((int64_t)aq * acc) >> 31);                 // atan(a), Q29
  if (swap) f = 843314857 - f;                                       // pi/2 - atan(a)
  if (x.m < 0) f = 1686629713 - f;                                   // pi - angle
  return y.m < 0 ? -f : f;
}
static inline int32_t xq(xf v, int q) {   // v * 2^q as a whole number (|v| * 2^q < 2^31)
  if (v.m == 0) return 0;
  const int32_t sh = -q - v.e;
  if (sh <= 0) return v.m << (-sh);
  return sh >= 31 ? 0 : v.m >> sh;
}
#endif

WF_FAST float WireFitBounce::cost(const float* p, int m, float* g, float A[4][4]) const {
  float f = 0;
  const int np = bc.bgField ? 4 : 3;
#ifdef WF_XF
  if (bc.bgField) {
    // the same model and cost as below, in inline whole-number floating point (wf_xf.h)
#ifdef WF_XSC
    float spf, cpf; xsincos(p[1], spf, cpf);
#else
    const float spf = sinf(p[1]), cpf = cosf(p[1]);
#endif
    const xf sp = xfromf(spf), cp = xfromf(cpf), p0 = xfromf(p[0]), H = xfromf(p[2]), b = xfromf(p[3]);
    const xf H2 = xmul(H, H), Mmin = xfromf(1e-12f), LN2 = xfromf(0.69314718f);
    const xf Z = {0, XF_ZERO_E};
    xf fx = Z, gx[4] = {Z, Z, Z, Z}, Ax[4][4];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) Ax[i][j] = Z;
    const int32_t PI27 = 421657428, TWOPI27 = 843314857;
    for (int k = 0; k < m; k++) {
      const W& w = w_[k];
      const xf wdx = xfromf(w.dx), wdy = xfromf(w.dy), sdt = xfromf(w.sdt), cdt = xfromf(w.cdt);
      const xf dk = xsub(xadd(p0, xmul(wdx, sp)), xmul(wdy, cp)), gk = xadd(xmul(wdx, cp), xmul(wdy, sp));
      const xf c = xadd(xmul(cp, cdt), xmul(sp, sdt)), s = xsub(xmul(sp, cdt), xmul(cp, sdt));
      // predictB
      const xf R2 = xadd(xmul(dk, dk), H2), iR2 = xrecip(R2), u = xmuli(iR2, 1);
      const xf dud = xneg(xmul(xmul(u, xmuli(dk, 1)), iR2)), duH = xneg(xmul(xmul(u, xmuli(H, 1)), iR2));
      const xf Hc = xmul(H, c);
      const xf X = xmul(Hc, u), Y = xadd(xmul(dk, u), b);
      xf M = xadd(xmul(X, X), xmul(Y, Y));
      if (!xgt(M, Mmin)) M = Mmin;
      const xf iM = xrecip(M);
      const xf L = xsub(LN2, xmuli(xlog(M), -1));
      const int32_t Phi27 = xatan2q29(Y, X) >> 1;                     // 2 atan2, Q27... (Q29 angle = Q28 doubled)
      const xf dX[4] = {xmul(Hc, dud), xneg(xmul(xmul(H, s), u)), xadd(xmul(c, u), xmul(Hc, duH)), Z};
      const xf dY[4] = {xadd(u, xmul(dk, dud)), Z, xmul(dk, duH), {1073741824, -30}};
      xf J0[4], J1[4];
      XF_NOUNROLL
      for (int i = 0; i < 4; i++) {
        J0[i] = xneg(xmul(xadd(xmul(X, dX[i]), xmul(Y, dY[i])), iM));
        J1[i] = xmuli(xmul(xsub(xmul(X, dY[i]), xmul(Y, dX[i])), iM), 1);
      }
      // residuals
      const xf e0 = xsub(xfromf(w.L), L);
      int32_t d27 = xq(xfromf(w.Phi), 27) - Phi27;
      while (d27 > PI27) d27 -= TWOPI27;
      while (d27 <= -PI27) d27 += TWOPI27;
      const xf e1 = xnorm(d27, -27);
      const xf iL2 = xfromf(w.iL2), iA2 = xfromf(w.iA2);
      const xf nis = xadd(xmul(xmul(e0, e0), iL2), xmul(xmul(e1, e1), iA2));
      const xf ONE = {1073741824, -30}, FOUR = {1073741824, -28};
      fx = xadd(fx, xmul(xlog(xadd(ONE, xmuli(nis, -2))), {1610612736, -29}));   // 0.5 (NU + 2) = 3
      evals_++;
      if (!g) continue;
      const xf wt = xmul({1610612736, -28}, xrecip(xadd(FOUR, nis)));           // (NU + 2) / (NU + nis)
      const xf r0[4] = {J0[0], xadd(xmul(J0[0], gk), J0[1]), J0[2], J0[3]};
      const xf r1[4] = {J1[0], xadd(xmul(J1[0], gk), J1[1]), J1[2], J1[3]};
      const xf a0 = xmul(wt, iL2), a1 = xmul(wt, iA2);
      XF_NOUNROLL
      for (int i = 0; i < 4; i++) {
        const xf u0 = xmul(a0, r0[i]), u1 = xmul(a1, r1[i]);
        gx[i] = xadd(gx[i], xadd(xmul(u0, e0), xmul(u1, e1)));
        XF_NOUNROLL
        for (int j = i; j < 4; j++) Ax[i][j] = xadd(Ax[i][j], xadd(xmul(u0, r0[j]), xmul(u1, r1[j])));
      }
    }
    f = xtof(fx);
    if (g) for (int i = 0; i < 4; i++) { g[i] = xtof(gx[i]); for (int j = i; j < 4; j++) A[i][j] = xtof(Ax[i][j]); }
  } else
#endif
#ifdef WF_XF_ONLY   // size measurement only: the float path left out (bgField must be true)
  {}
#else
  {
  if (g) for (int i = 0; i < np; i++) { g[i] = 0; for (int j = 0; j < np; j++) A[i][j] = 0; }
  const float sp = sinf(p[1]), cp = cosf(p[1]);
  for (int k = 0; k < m; k++) {
    const W& w = w_[k];
    // the sample's distance and direction: the wire line seen from the sample's pose
    const float dk = p[0] + w.dx * sp - w.dy * cp, gk = w.dx * cp + w.dy * sp;
    const float ck = cp * w.cdt + sp * w.sdt, sk = sp * w.cdt - cp * w.sdt;   // cos and sin of psi - dth
    float L, Phi, J[2][4];
    if (bc.bgField) predictB(dk, ck, sk, p[2], p[3], L, Phi, J);
    else {
      float J3[2][3];
      predict(dk, ck, sk, p[2], L, Phi, J3);
      for (int i = 0; i < 2; i++) { for (int j = 0; j < 3; j++) J[i][j] = J3[i][j]; J[i][3] = 0; }
    }
    const float e0 = w.L - L, e1 = wrapPi(w.Phi - Phi);
    const float nis = e0 * e0 * w.iL2 + e1 * e1 * w.iA2;
    f += 0.5f * (NU + 2) * WF_LOGF(1 + nis / NU);
    evals_++;
    if (!g) continue;
    const float wt = (NU + 2) / (NU + nis);                       // Student-t weight (iteratively reweighted)
    // rows of d(prediction)/dp: d dk/d psi = gk
    const float r0[4] = {J[0][0], J[0][0] * gk + J[0][1], J[0][2], J[0][3]};
    const float r1[4] = {J[1][0], J[1][0] * gk + J[1][1], J[1][2], J[1][3]};
    const float a0 = wt * w.iL2, a1 = wt * w.iA2;
    for (int i = 0; i < np; i++) {
#ifdef WF_SOLVE_1B
      const float u0 = a0 * r0[i], u1 = a1 * r1[i];   // once per row (the same products, the same order)
      g[i] += u0 * e0 + u1 * e1;
      for (int j = i; j < np; j++) A[i][j] += u0 * r0[j] + u1 * r1[j];
#else
      g[i] += a0 * r0[i] * e0 + a1 * r1[i] * e1;
      for (int j = i; j < np; j++) A[i][j] += a0 * r0[i] * r0[j] + a1 * r1[i] * r1[j];
#endif
    }
  }
  }   // float path
#endif
  if (g) a22Data_ = A[2][2];
  if (prior_ && Vc_ > 0) {   // the carried height: a Gaussian prior on H
    const float e = Hc_ - p[2], iv = 1 / Vc_;
    f += 0.5f * e * e * iv;
    if (g) { g[2] += e * iv; A[2][2] += iv; }
  }
  if (g) for (int i = 0; i < np; i++) for (int j = 0; j < i; j++) A[i][j] = A[j][i];
  return f;
}

BounceResult WireFitBounce::solve(bool exhaustive) {
  BounceResult res;
  if (!prepare(res)) return res;
  return iterate(exhaustive);
}

bool WireFitBounce::prepare(BounceResult& res) {
  res = BounceResult();
  evals_ = 0;
  pm_ = 0;
  if (n_ < 3) return false;
  // samples of the last windowM of travel, relative to the newest sample (its rover frame, sensor 1 at the origin)
  nwc_ = sample(n_ - 1);
  const BounceSample& nw = nwc_;
  const float cn = cosf(nw.th), sn = sinf(nw.th);
  int m = 0;
  for (int k = n_ - 1; k >= 0 && m < NW; k--) {
    const BounceSample& b = sample(k);
    if (nw.s - b.s > bc.windowM) break;
    if (!(b.r > 0) || b.r > cfg.rMax) continue;
#ifdef WF_DECIM
    // test: every WF_DECIM-th sample, each with WF_DECIM times the weight (the same information if neighbours repeat)
    { static int kk = 0; if (k == n_ - 1) kk = 0; if (kk++ % WF_DECIM) continue; }
#endif
    W& w = w_[m++];
    const float ex = b.px - nw.px, ey = b.py - nw.py;
    w.dx = cn * ex + sn * ey; w.dy = -sn * ex + cn * ey;
    const float dth = b.th - nw.th;
#if defined(WF_XF) && defined(WF_XSC)
    xsincos(dth, w.sdt, w.cdt);         // test: fast sin and cos (wf_xf.h)
#else
    w.sdt = sinf(dth); w.cdt = cosf(dth);
#endif
    w.L = WF_LOGF(b.r);
    w.Phi = 2 * WF_ATAN2F(b.x, b.y);
    const float sL = cfg.sigmaLnR + cfg.sigmaLnRPerM * b.r, sA = 2 * (cfg.sigmaAngDeg + cfg.sigmaAngDegPerM * b.r) * D2R;
    w.iL2 = 1 / (sL * sL); w.iA2 = 1 / (sA * sA);
#ifdef WF_DECIM
    w.iL2 *= WF_DECIM; w.iA2 *= WF_DECIM;
#endif
  }
  res.n = m;
  pm_ = m;
  return m >= 3;
}

BounceResult WireFitBounce::iterate(bool exhaustive) {
  BounceResult res;
  const int m = pm_;
  res.n = m;
  const BounceSample& nw = nwc_;
  const float cn = cosf(nw.th), sn = sinf(nw.th);
  // the carried height: its uncertainty grows with the travel since the last solve
  if (bc.carryH) { Vc_ += bc.hWalkPerM * fabsf(nw.s - lastS_); lastS_ = nw.s; }
  prior_ = bc.carryH;
  // starts: the warm answer moved to the newest sample, and the closed-form candidates of the newest reading
  float starts[24][4] = {};
  int ns = 0;
  if (warm_) {
    const float ex = warmPx_ - nw.px, ey = warmPy_ - nw.py;   // the warm answer's sensor position, newest frame
    const float dx = cn * ex + sn * ey, dy = -sn * ex + cn * ey, psi = wrapPi(warmP_[1] - wrapPi(nw.th - warmTh_));
    starts[ns][0] = warmP_[0] + dx * sinf(psi) - dy * cosf(psi); starts[ns][1] = psi; starts[ns][2] = warmP_[2]; starts[ns][3] = warmP_[3]; ns++;
  }
  {
    const W& w = w_[0];
    const float r = expf(w.L), ang = 0.5f * w.Phi;              // reading angle, -90..90 deg after the sign fold
    float wl = sinf(ang), wd = cosf(ang);
    if (wd < 0) { wl = -wl; wd = -wd; }
    const float t = wd > 1e-4f ? wl / wd : (wl >= 0 ? 1e4f : -1e4f), t2 = t * t;
    static const float HS[4] = {0.1f, 0.2f, 0.35f, 0.5f};      // start values only: H is free in the solve
    const bool search = !warm_ || exhaustive || ++sinceRestart_ >= bc.restartEvery;
    if (search) sinceRestart_ = 0;
#ifdef DIAG_STEPTIME
    if (search) { g_st_heavy_now = 1; g_st_heavy_seen = 1; }
#endif
    const int nh = !search ? 0 : (warm_ ? 1 : 4);
    for (int h = 0; h < nh; h++) {
      float H = warm_ ? warmP_[2] : HS[h];
      if (H > r) H = r;
      const float K = r / (H * wd), disc = K * K - 4 * t2;
      float roots[2]; int nr = 0;
      if (t2 < 1e-8f) roots[nr++] = 1 / K;
      else if (disc < 0) roots[nr++] = K / (2 * t2);
      else { const float sq = sqrtf(disc); roots[nr++] = 2 / (K + sq); roots[nr++] = (K + sq) / (2 * t2); }
      for (int i = 0; i < nr; i++) {
        float a = roots[i];
        if (!(a > 0)) continue;
        if (a > 1) a = 1;
        const float d = fabsf(t) * H * a, psi0 = acosf(t >= 0 ? a : -a);
        for (int sg = -1; sg <= 1 && ns < 24; sg += 2) { starts[ns][0] = d; starts[ns][1] = sg * psi0; starts[ns][2] = H; ns++; }
      }
    }
  }
  // offline reference: a grid over (d, psi, H); the best 8 points become starts
  if (exhaustive) {
    float bestF[8]; float bestP[8][3]; int nb = 0;
    for (float H = 0.04f; H <= 0.8f; H += 0.04f)
      for (float d = 0; d <= 1.5f; d += 0.05f)
        for (float pd = -180; pd < 180; pd += 10) {
          const float p[4] = {d, pd * D2R, H, 0};
          const float f = cost(p, m, nullptr, nullptr);
          int slot = nb < 8 ? nb++ : -1;
          if (slot < 0) { int wk = 0; for (int i = 1; i < 8; i++) if (bestF[i] > bestF[wk]) wk = i; if (f < bestF[wk]) slot = wk; }
          if (slot >= 0) { bestF[slot] = f; bestP[slot][0] = p[0]; bestP[slot][1] = p[1]; bestP[slot][2] = p[2]; }
        }
    for (int i = 0; i < nb && ns < 24; i++) { starts[ns][0] = bestP[i][0]; starts[ns][1] = bestP[i][1]; starts[ns][2] = bestP[i][2]; ns++; }
  }
  // damped Gauss-Newton from each start
  const int np = bc.bgField ? 4 : 3;
  float best = 1e30f, bp[4] = {0, 0, 0, 0};
  float Ab[4][4] = {}, a22b = 0;       // the Gauss-Newton matrix at the answer (with the prior) and its data-only H entry
  int bs = -1;
  for (int s = 0; s < ns; s++) {
    float p[4] = {starts[s][0], starts[s][1], starts[s][2], starts[s][3]}, g[4], A[4][4];
    float f = cost(p, m, g, A), lam = 1e-3f, a22 = a22Data_;
#ifdef WF_START_DBG
    const int ev0 = evals_; float ftrace[16]; int nft = 0; ftrace[nft++] = f;
#endif
    for (int it = 0; it < bc.iters; it++) {
      float Ad[4][4], dp[4] = {0, 0, 0, 0};
      for (int i = 0; i < np; i++) for (int j = 0; j < np; j++) Ad[i][j] = A[i][j] + (i == j ? lam * (A[i][i] + 1e-6f) : 0);
      if (!solveN(Ad, g, dp, np)) break;
#ifdef WF_PRED_STOP
      // test: the quadratic model's gain for this step; below the stop rule's gain the step cannot pass it: stop
      // without the evaluation (a rejected step costs one full evaluation)
      { float gd = 0, q2 = 0; for (int i = 0; i < np; i++) { gd += g[i] * dp[i]; for (int j = 0; j < np; j++) q2 += dp[i] * A[i][j] * dp[j]; }
        if (gd - 0.5f * q2 < WF_PRED_STOP * 1e-4f * (1 + f)) break; }
#endif
      float q[4] = {p[0] + dp[0], wrapPi(p[1] + dp[1]), p[2] + dp[2], p[3] + dp[3]};
      if (q[2] < bc.hMin) q[2] = bc.hMin;
      if (q[2] > bc.hMax) q[2] = bc.hMax;
      float g2[4], A2[4][4];
      const float f2 = cost(q, m, g2, A2);
#ifdef WF_STOP_DBG
      fprintf(stderr, "%s\n", f2 < f ? "A" : "R");
#endif
      if (f2 < f) {
        const float gain = f - f2;
        for (int i = 0; i < np; i++) { p[i] = q[i]; g[i] = g2[i]; for (int j = 0; j < np; j++) A[i][j] = A2[i][j]; }
        f = f2; lam *= 0.3f; a22 = a22Data_;
#ifdef WF_START_DBG
        if (nft < 16) ftrace[nft++] = f;
#endif
        if (gain < 1e-4f * (1 + f)) break;
#ifdef WF_STOP_SD
        // test: stop when the step was under WF_STOP_SD of the answer's own uncertainty (step' A step, A at the answer)
        { float s2 = 0; for (int i = 0; i < np; i++) for (int j = 0; j < np; j++) s2 += dp[i] * A[i][j] * dp[j];
#ifdef WF_STOP_DBG
          fprintf(stderr, "it %d s2 %g gain %g f %g lam %g\n", it, s2, gain, f, lam);
#endif
          if (s2 < WF_STOP_SD * WF_STOP_SD) break; }
#endif
      } else { lam *= 10; if (lam > WF_LAM_MAX) break; }
    }
#ifdef WF_START_DBG
    { fprintf(stderr, "ST %d %d %d %d %.6g %.6g", ns, s, (evals_ - ev0) / (m > 0 ? m : 1) + 1, f < best ? 1 : 0, f, best);
      for (int i = 0; i < nft; i++) fprintf(stderr, " %.6g", ftrace[i]); fputc('\n', stderr); }
#endif
    if (f < best) {
      best = f; bs = s; bp[0] = p[0]; bp[1] = p[1]; bp[2] = p[2]; bp[3] = p[3];
      for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) Ab[i][j] = A[i][j];
      a22b = a22;
    }
  }
  // the uncertainties below use the matrix of the answer that the last step already holds (no new evaluation);
  // with no start, the matrix at the zero answer as before
  if (bs < 0) { float g[4]; cost(bp, m, g, Ab); evals_ -= m; a22b = a22Data_; }
  // the wire direction uncertainty: the psi entry of the inverse Gauss-Newton matrix at the answer, with the height
  // prior as in the solve. Driving square-on, the distance falls at nearly the full speed for any angle from 60 to 90 deg
  // (sin 60 = 0.87), so the matrix holds little psi information there and the entry grows
  {
    float e1[4] = {0, 1, 0, 0}, col[4] = {0, 0, 0, 0};
    res.psiSd = solveN(Ab, e1, col, np) && col[1] > 0 ? sqrtf(col[1]) / D2R : 180.0f;
    if (res.psiSd > 180.0f) res.psiSd = 180.0f;
  }
  // the fit's own height information (its data only): the H entry of the inverse Gauss-Newton matrix, no prior
  {
    prior_ = false;
    float e2[4] = {0, 0, 1, 0}, col[4] = {0, 0, 0, 0};
    Ab[2][2] = a22b;                   // the data only: no prior
    res.Hsd = solveN(Ab, e2, col, np) && col[2] > 0 ? sqrtf(col[2]) : 1e3f;
    // refine the carried estimate: each sample sits in about m solves, so one solve adds 1 / m of its information
    if (bc.carryH && col[2] > 0) {
      const float iFit = 1 / (col[2] * m), iC = 1 / Vc_;
      Hc_ = (Hc_ * iC + bp[2] * iFit) / (iC + iFit);
      Vc_ = 1 / (iC + iFit);
    }
    res.Hc = Hc_; res.Hcsd = sqrtf(Vc_);
  }
  lastM_ = m; lastF_ = best; for (int i = 0; i < 4; i++) lastP_[i] = bp[i];
  // fold to d >= 0 (the same line seen with the opposite direction)
  if (bp[0] < 0) { bp[0] = -bp[0]; bp[1] = wrapPi(bp[1] + PI_F); bp[3] = -bp[3]; }
  warm_ = true; warmP_[0] = bp[0]; warmP_[1] = bp[1]; warmP_[2] = bp[2]; warmP_[3] = bp[3];
  warmPx_ = nw.px; warmPy_ = nw.py; warmTh_ = nw.th;
  res.valid = true;
  res.d = bp[0]; res.psiDeg = bp[1] / D2R; res.betaDeg = asinf(sinf(bp[1])) / D2R; res.H = bp[2]; res.cost = best; res.bg = bp[3];
  res.starts = ns; res.evals = evals_; res.bestStart = bs;
  res.refX = nw.px; res.refY = nw.py; res.refTh = nw.th;
  return res;
}

// The newest answer moved to the current pose (the motion since the newest sample)
BounceResult WireFitBounce::now(const BounceResult& r0) const {
  BounceResult r = r0;
  if (!r.valid || n_ == 0) return r;
  // from the sample the answer belongs to (with the solve in a background task, an older one than the newest)
  const float c = cosf(T_), s = sinf(T_);
  const float px = X_ + c * cfg.sensorFwd - s * cfg.sensorLeft, py = Y_ + s * cfg.sensorFwd + c * cfg.sensorLeft;
  const float cn = cosf(r0.refTh), sn = sinf(r0.refTh), ex = px - r0.refX, ey = py - r0.refY;
  const float dx = cn * ex + sn * ey, dy = -sn * ex + cn * ey, psi0 = r0.psiDeg * D2R;
  float d = r0.d + dx * sinf(psi0) - dy * cosf(psi0), psi = wrapPi(psi0 - wrapPi(T_ - r0.refTh));
  if (d < 0) { d = -d; psi = wrapPi(psi + PI_F); }
  r.d = d; r.psiDeg = psi / D2R; r.betaDeg = asinf(sinf(psi)) / D2R;
  return r;
}

// The cost profile over psi: from the last answer, step psi by stepDeg to each side; at each step fit d, H and the field
// term again (damped Gauss-Newton with psi held, from the previous step). Driving square-on, the distance falls at
// nearly the full speed for any angle from 60 to 90 deg (sin 60 = 0.87), so the profile stays flat there and the
// uncertainty grows
float WireFitBounce::psiProfile(float delta, float stepDeg) const {
  if (lastM_ < 3) return 90.0f;
  const int m = lastM_, np = bc.bgField ? 4 : 3;
  const bool pr = prior_;
  prior_ = bc.carryH;
  float worst = 0;
  for (int side = -1; side <= 1; side += 2) {
    float p[4] = {lastP_[0], lastP_[1], lastP_[2], lastP_[3]};
    float reach = 90.0f;
    for (float off = stepDeg; off <= 90.0f + 1e-3f; off += stepDeg) {
      p[1] = wrapPi(lastP_[1] + side * off * D2R);
      float g[4], A[4][4];
      float f = cost(p, m, g, A), lam = 1e-3f;
      for (int it = 0; it < bc.iters; it++) {
        float Ad[4][4], dp[4] = {0, 0, 0, 0};
        for (int i = 0; i < np; i++) for (int j = 0; j < np; j++) Ad[i][j] = (i == 1 || j == 1) ? (i == j ? 1.0f : 0.0f) : A[i][j] + (i == j ? lam * (A[i][i] + 1e-6f) : 0);
        float gg[4] = {g[0], 0, g[2], g[3]};
        if (!solveN(Ad, gg, dp, np)) break;
        float q[4] = {p[0] + dp[0], p[1], p[2] + dp[2], p[3] + dp[3]};
        if (q[2] < bc.hMin) q[2] = bc.hMin;
        if (q[2] > bc.hMax) q[2] = bc.hMax;
        float g2[4], A2[4][4];
        const float f2 = cost(q, m, g2, A2);
        if (f2 < f) {
          const float gain = f - f2;
          for (int i = 0; i < np; i++) { p[i] = q[i]; g[i] = g2[i]; for (int j = 0; j < np; j++) A[i][j] = A2[i][j]; }
          f = f2; lam *= 0.3f;
          if (gain < 1e-4f * (1 + f)) break;
        } else { lam *= 10; if (lam > 1e6f) break; }
      }
      if (f - lastF_ > delta) { reach = off - stepDeg; break; }
    }
    if (reach > worst) worst = reach;
  }
  prior_ = pr;
  return worst;
}

}  // namespace wirefit
