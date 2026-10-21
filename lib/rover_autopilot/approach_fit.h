// Approach fit: the wire heading from a straight drive in range (decision 2026-10-21, "approach fit").
//
// psi = heading: the angle of the centreline from the wire, left positive. s = distance driven (dead reckoned).
// The Rx radius R comes from the field strength in the coil plane: a coil roll turns the field in that plane and
// leaves the strength as it is, so the fit does not depend on roll. Tx current regulated, Rx gain known: the scale is
// known. Distance to the wire d = sqrt(R^2 - H^2) (long straight wire, first order). On a straight drive d is a
// straight line in s: |d|(s) changes by sin(psi) per metre.
//   sign of d   + when polarity says inside (a crossing stays one straight line); unsigned when polarity is not known
//   weight      1 / variance for a fixed noise on the coil strengths: d^2 H^4 / R^6
//   left/right  the sign of psi: polarity side when known, else the offset sign (sign of x y), weighted as d
//   offset fit  |X| = |H x / y| against s gives |tan(psi)|; weight (y / R)^4 (H / R)^2. The heading is the larger of
//               the two: the radius fit reads low in a loop (the rest of the loop adds to the strength), the offset
//               fit reads low with a coil roll. A low heading turns too little at a bounce.
//
// Quality, from the fit's own data:
//   residual    rms scatter of d about the line, m
//   sigma       heading uncertainty from the residual scatter and the distance span (1 standard deviation)
//   curvature   a quadratic term in each fit; when significant (F over 9: 3 standard deviations) the heading is the
//               one at the last frame (a straight drive past a curved wire, a corner, a field that is not a long
//               straight wire): the heading at the decision counts, not the average over the drive
//   unc         reported heading uncertainty
//   polCheck    polarity side against the offset sign
// Per frame: one weighted running mean and co-moment update (stable in float). The solve: closed form, once.
//
// No Arduino dependencies: the same file builds on the PC for test/rover_emu.
#pragma once
#include <stdint.h>

#ifndef APPROACH_FIT_REAL
#define APPROACH_FIT_REAL float
#endif

namespace rover {

struct FitResult {
  bool valid = false;
  uint16_t n = 0;           // frames in the fit
  float span = 0;           // m of distance driven between the first and the last frame in the fit
  float psiDeg = 0;         // heading, -90..90 deg, left positive: the larger of the radius and offset headings
  float psiRDeg = 0, psiXDeg = 0;   // radius fit, offset fit
  float sigmaDeg = 0;
  float dNow = 0;           // distance to the wire at the last frame from the fit, m, wire on the left positive
  float residual = 0;       // m
  float curveDeg = 0, curveF = 0;
  float uncDeg = 0;
  int8_t polCheck = 0;      // polarity side against the offset sign: +1 agree, -1 disagree, 0 not known
};

class ApproachFit {
 public:
  typedef APPROACH_FIT_REAL real;
  void reset();
  void advance(float ds) { s_ += ds; }      // distance driven since the last frame, m (signed: reverse negative)
  void add(float x, float y, float R, float H, int8_t inside);   // inside: polarity, +1, -1, 0 not known
  bool solve(FitResult& r, int8_t polSide = 0) const;              // polSide: polarity, +1 wire left, 0 not known
  uint16_t count() const { return n_; }
  float span() const { return n_ ? (float)(sMax_ - sMin_) : 0.0f; }

 private:
  // Weighted running means and co-moments of (s, s^2, z) (stable in float)
  struct Line {
    real W = 0, m[3] = {}, C[3][3] = {};
    uint16_t n = 0;
    void add(real w, real s, real z);
    bool slope(real sLast, real& b, real& sb, real& zNow, real& bLin, real& F) const;
  };
  Line d_;                  // distance from the radius
  Line x_;                  // |lateral wire offset|
  real side_ = 0;           // weighted sum of the offset sign (left positive)
  real s_ = 0, sMin_ = 0, sMax_ = 0, sLast_ = 0;
  uint16_t n_ = 0;
};

}  // namespace rover
