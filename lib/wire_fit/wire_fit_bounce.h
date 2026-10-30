// Wire fit at the bounce point: one straight-wire solve at the bounce trigger,
// from a buffer of filtered, decimated readings and motion. No continuous estimator.
//
// Per frame (the Rx frame rate, 100 Hz): the wheel and gyro motion integrates into the pose, and the sensor 1 reading
// (sign aligned: the field has no sign) and the pose add into a block average. Each block of decim frames gives one
// buffer sample. The block average is a boxcar filter with nulls at multiples of the sample rate; with the Rx's own
// 200 ms filter it takes content above the half sample rate down by 20 dB or more before decimation.
//
// The buffer holds the last N samples. The re-arm history is the samples since the end of the last turn (turnEnd).
// The solve uses the last windowM of travel, through turns (the motion carries the pose through them).
//
// Solve (continuous, at every buffer sample: its distance decides the bounce, its direction the departure): the
// straight wire (distance d from sensor 1, direction psi in the rover frame, vertical distance H; all
// free) that best explains the samples of the last windowM of travel. Robust least squares (Student-t) on the log
// radius and the doubled reading angle, Gauss-Newton with damping, from the closed-form candidates of the newest
// sample (at a few start values of H on the first solve), and from the last answer moved by the motion. exhaustive = true adds a grid over (d, psi, H): the offline reference.
// No Arduino dependencies.
#pragma once
#include <stdint.h>
#include "wire_fit.h"

namespace wirefit {

struct BounceSample {
  float x, y, r;                       // filtered sensor 1 reading, m (r = 0: no reading in the block)
  float px, py, th;                    // sensor 1 position (m) and rover heading (rad), odometry frame
  float s;                             // travel, m, since reset
};

struct BounceConfig {
#ifndef WF_BOUNCE_DECIM
#define WF_BOUNCE_DECIM 10
#endif
  int decim = WF_BOUNCE_DECIM;         // frames per buffer sample: 100 Hz frames to 10 Hz samples (4 cm at 0.4 m/s;
                                       // the field changes over the vertical distance, about 0.2 m)
                                       // (the autopilot sets it to the same 10 Hz at its update period: periodApply)
  float windowM = 1.0f;                // m of travel before the trigger that the solve uses
  int iters = 15;                      // damped Gauss-Newton steps per start
  int restartEvery = 10;               // solves between searches from the closed-form candidates; the others start
                                       // from the last answer only. Reason: CPU (one start costs about a fifth)
  // Height (vertical distance sensor to wire): a fitted unknown every solve, within physical bounds, carried from
  // solve to solve as an estimate (the solve finds the wire height too)
  float hMin = 0.01f, hMax = 10.0f;    // m: test bounds only; the model needs height > 0
  bool carryH = true;                  // the carried estimate enters each solve as a prior; each fit refines it.
                                       // Reason: at the bounce trigger (about 3 heights out) height is not observable;
                                       // readings near the wire (follow, join, crossing) pin it
  float hStart = 0.18f, hStartSd = 1.0f;   // m: no information at the start
  float hWalkPerM = 0.0005f;           // m^2 per m driven: measured; well-determined fits differ by 0.018 m sd at
                                       // 0.5-1 m apart, 0.028 m at 1-2 m (5 runs, 2026-10-25)
  bool bgField = true;                // a 4th unknown, a uniform vertical field over the window (the rest of
                                       // the loop: inside a loop every part adds vertical field in one direction)
  int lagFrames = 10;                  // the reading pairs with the pose this many frames back: the Rx filter (a
                                       // boxcar of filter_n = 20 frames) delays the reading by (20 - 1) / 2 frames
                                       // (build flag WF_LAG_TIME: the autopilot sets it in time, periodApply)
};

struct BounceResult {
  bool valid = false;
  float d = 0, psiDeg = 0, betaDeg = 0, H = 0, cost = 0, bg = 0;   // bg: uniform vertical field, units of 2 / m
  float Hsd = 0;                       // m: the fit's own height uncertainty (its data only, no carried prior)
  float Hc = 0, Hcsd = 0;              // m: the carried height estimate after this solve
  float psiSd = 0;                     // deg: the fit's wire direction uncertainty (1 standard deviation, with the
                                       // carried height prior). Near square-on the drive gives little direction information
  int n = 0, starts = 0, evals = 0;    // samples used, starts tried, model evaluations
  int bestStart = -1;                  // the start the answer came from (index into the starts tried)
  float refX = 0, refY = 0, refTh = 0; // the pose of the sample the answer belongs to (now() moves it from there)
};

class WireFitBounce {
 public:
#ifndef WIRE_FIT_BN
#define WIRE_FIT_BN 64                 // samples: also the most one solve uses (NW). 1 m of travel at the 10 Hz sample
                                       // rate down to 0.16 m/s (Bounce mode approach: 0.3 m/s, 34 samples; emulator
                                       // decisions: at most 45). The rover uses no re-arm history. The ESP32-S2 heap
                                       // holds it only in a Bounce mode run (0.7.0: about 4.6 kB)
#endif
  static const int N = WIRE_FIT_BN;
  Config cfg;                          // shared settings: sensor position, reading noise, rMax
  BounceConfig bc;
  void reset();
  void frame(float ds, float dTheta, bool have, float x, float y, float r);   // one Rx frame
  void turnEnd() { since_ = 0; }       // the end of a turn: the re-arm history starts here
  int sinceTurn() const { return since_ < n_ ? since_ : n_; }   // samples of the re-arm history
  BounceResult solve(bool exhaustive = false);  // on the newest sample; warm start from the last solve
  // solve() in two parts, for a solve in a background task: prepare() copies the window of the newest sample (the
  // caller's thread, with frame()); iterate() is the rest and reads only that copy. false from prepare: no answer
  // (res says why). No prepare() while an iterate() or psiProfile() runs
  bool prepare(BounceResult& res);
  BounceResult iterate(bool exhaustive = false);
  BounceResult now(const BounceResult& r) const;  // the answer moved to the current pose
  // The honest wire direction uncertainty of the last solve: the profile of the cost over psi (d, H and the field term
  // fitted again at each psi). Returns the largest psi offset (deg) on either side where the cost rises less than
  // delta over the best, at most 90 (no direction information)
  float psiProfile(float delta, float stepDeg = 5.0f) const;
  int size() const { return n_; }
  long total() const { return total_; }   // samples since reset
  const BounceSample& sample(int k) const { return buf_[(head_ - n_ + k + N) % N]; }   // k = 0: oldest

 private:
  float cost(const float* p, int m, float* g, float A[4][4]) const;
  BounceSample buf_[N];
  int head_ = 0, n_ = 0, since_ = 0;
  long total_ = 0;
  // pose of the rover reference point
  float X_ = 0, Y_ = 0, T_ = 0, S_ = 0;
  static const int LAG = 32;           // pose history, frames (lagFrames < LAG)
  float hx_[LAG], hy_[LAG], ht_[LAG], hs_[LAG];
  int hi_ = 0, hcount_ = 0;
  float sS_ = 0;
  // block sums
  int nf_ = 0, nr_ = 0;
  float sx_ = 0, sy_ = 0, sr_ = 0, spx_ = 0, spy_ = 0, sth_ = 0, lx_ = 0, ly_ = 0;
  // the samples of one solve, relative to the newest: dx, dy (m, newest rover frame), dth (rad), ln r, doubled angle, sd
  struct W { float dx, dy, sdt, cdt, L, Phi, iL2, iA2; };   // sdt, cdt: sin and cos of the heading change
  static const int NW = N;             // samples in one solve at most
  W w_[NW];
  bool warm_ = false;                  // the last answer, for the next solve's start
  float warmP_[4] = {0, 0, 0, 0}, warmPx_ = 0, warmPy_ = 0, warmTh_ = 0;
  mutable int evals_ = 0;
  int sinceRestart_ = 0;
  int pm_ = 0;                         // prepare(): the samples in w_, and the newest sample
  BounceSample nwc_ = {};
  int lastM_ = 0;                      // the last solve: its samples in w_, its answer before the fold, its cost
  float lastP_[4] = {0, 0, 0, 0}, lastF_ = 0;
  float Hc_ = 0, Vc_ = 0, lastS_ = 0;  // carried height estimate, its variance, travel at the last solve
  mutable bool prior_ = false;         // cost() adds the carried height as a prior
  mutable float a22Data_ = 0;          // cost(): the H entry of the Gauss-Newton matrix before the prior (data only)
};

}  // namespace wirefit
