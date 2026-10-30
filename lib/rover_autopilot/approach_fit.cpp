// Approach fit: see approach_fit.h.
#include "approach_fit.h"
#include <math.h>

namespace rover {

namespace {
typedef ApproachFit::real real;
const real R2D = (real)57.29577951308232;
const real F_SIGNIFICANT = 9;   // one more term: 3 standard deviations
real clamp1(real v) { return v > 1 ? 1 : (v < -1 ? -1 : v); }
}  // namespace

void ApproachFit::Line::add(real w, real s, real z) {
  if (!(w > 0)) return;
  const real v[3] = {s, s * s, z};
  real d0[3];
  W += w;
  for (int i = 0; i < 3; i++) { d0[i] = v[i] - m[i]; m[i] += w / W * d0[i]; }
  for (int i = 0; i < 3; i++)
    for (int j = i; j < 3; j++) C[i][j] += w * d0[i] * (v[j] - m[j]);
  n++;
}

// Slope of z against s at the last frame: the straight line, or the curvature term when it is significant (the wire
// or the field bends over the span; the heading that counts is the one at the decision).
bool ApproachFit::Line::slope(real sLast, uint16_t minN, real& b, real& sb, real& zNow, real& bLin, real& F) const {
  const real Css = C[0][0], Csz = C[0][2], Czz = C[2][2];
  if (n < minN || !(Css > 0)) return false;
  bLin = Csz / Css;
  real rss = Czz - bLin * Csz;
  if (rss < 0) rss = 0;
  b = bLin;
  sb = sqrt(rss / ((n - 2) * Css));
  zNow = m[2] + bLin * (sLast - m[0]);
  F = 0;
  const real Csq = C[0][1], Cqq = C[1][1], Cqz = C[1][2];
  const real det = Css * Cqq - Csq * Csq;
  if (n > 3 && det > 0) {
    const real b1 = (Cqq * Csz - Csq * Cqz) / det, b2 = (Css * Cqz - Csq * Csz) / det;
    real rq = Czz - b1 * Csz - b2 * Cqz;
    if (rq < 0) rq = 0;
    F = rq > 0 ? (rss - rq) / (rq / (n - 3)) : 1e9f;
    if (F > F_SIGNIFICANT) {
      b = b1 + 2 * b2 * sLast;
      const real v = rq / (n - 3) / det;   // slope variance at sLast from the 2 x 2 covariance
      sb = sqrt(fabs(v * (Cqq - 4 * sLast * Csq + 4 * sLast * sLast * Css)));
      zNow = m[2] + b1 * (sLast - m[0]) + b2 * (sLast * sLast - m[1]);
    }
  }
  return true;
}

void ApproachFit::reset() {
  d_ = Line();
  x_ = Line();
  side_ = 0;
  s_ = sMin_ = sMax_ = sLast_ = 0;
  n_ = 0;
}

void ApproachFit::add(float x, float y, float Rf, float Hf, int8_t inside) {
  if (!(Rf > 0) || n_ == 0xFFFF) return;
  const real R = Rf, H = Hf, s = s_;
  const real d = sqrt(R * R > H * H ? R * R - H * H : 0);
  const real h = H / R, w = d * d * h * h * h * h / (R * R);   // d^2 H^4 / R^6
  if (!(w > 0)) return;
  const real sg = inside ? inside : 1;
  d_.add(w, s, sg * d);
  side_ += ((x < 0) != (y < 0)) ? -w : w;
  // Offset: |X| = |H x / y|, signed as d, weight (y / R)^4 (H / R)^2 (zero square-on: X is not formed)
  const real c = (real)y / R, c2 = c * c, wx = c2 * c2 * h * h;
  if (wx > 0) x_.add(wx, s, sg * (real)fabs(H * x / y));
  if (n_ == 0) sMin_ = sMax_ = s;
  if (s < sMin_) sMin_ = s;
  if (s > sMax_) sMax_ = s;
  sLast_ = s;
  n_++;
}

bool ApproachFit::solve(FitResult& r, int8_t polSide) const {
  r = FitResult();
  r.n = n_;
  r.span = span();
  real b, sb, dNow, bLin, F;
  if (!d_.slope(sLast_, minN_, b, sb, dNow, bLin, F)) return false;
  // Distance fit: |sin(psi)| = |slope|; toward the wire when |d| falls at the last frame
  const real sn = clamp1(fabs(b));
  const real lr = polSide ? polSide : (side_ < 0 ? -1 : 1), toward = (dNow * b < 0) ? 1 : -1;
  real psi = lr * toward * asin(sn) * R2D;
  r.psiRDeg = (float)psi;
  r.curveF = (float)F;
  r.curveDeg = (float)((asin(sn) - asin(clamp1(fabs(bLin)))) * R2D);
  // half the heading range that +-1 standard deviation of the slope gives (finite at 90 deg)
  real sig = (real)0.5 * (asin(clamp1(sn + sb)) - asin(clamp1(sn - sb))) * R2D;
  // Offset fit: |tan(psi)| = |slope of |X||. The larger of the two headings counts: the radius fit reads low in a
  // loop (the rest of the loop adds to the strength), the offset fit reads low with a coil roll (the vertical field
  // leaks into the sideways part); a low heading turns too little.
  real bx, sbx, xNow, bxLin, Fx;
  if (x_.slope(sLast_, minN_, bx, sbx, xNow, bxLin, Fx)) {
    const real psiX = lr * toward * atan(fabs(bx)) * R2D;
    r.psiXDeg = (float)psiX;
    if (fabs(psiX) > fabs(psi)) { psi = psiX; sig = sbx / (1 + bx * bx) * R2D; }
  }
  r.psiDeg = (float)psi;
  r.sigmaDeg = (float)sig;
  r.uncDeg = (float)sig;
  r.dNow = (float)(lr * fabs(dNow));
  const real rss = d_.C[2][2] - bLin * d_.C[0][2];
  r.residual = (float)sqrt((rss > 0 ? rss : 0) / d_.W);
  if (polSide) r.polCheck = (int8_t)((side_ < 0 ? -1 : 1) == polSide ? 1 : -1);
  r.valid = true;
  return true;
}

}  // namespace rover
