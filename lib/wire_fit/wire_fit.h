// Wire fit estimator: the local straight wire from sensor 1 (both coils) and the rover's own motion
//
// Model (the rover assumes nothing besides a cylindrical straight-line field): one infinite
// straight wire at the vertical distance H below sensor 1. The coils give the field projected onto the rover's front
// plane; the Rx reports it as the reading (x left, y down, radius r) of an equivalent wire across the rover. For a
// wire at horizontal offset d and direction psi in the rover frame:
//   x = d R^2 / q,  y = H cos(psi) R^2 / q,  r = R^2 / sqrt(q),  R^2 = d^2 + H^2,  q = H^2 cos^2(psi) + d^2
// The field has no sign (AC): (x, y) and (-x, -y) are the same reading, so the estimator compares the doubled angle
// 2 atan2(x, y) and ln r. The pair (d, psi) and (-d, psi + 180 deg) then read the same; polarity, when given,
// separates them.
//
// State: the probability of each (d, psi) on a grid; d from -1.6 m to 1.6 m, finer near the wire; psi in 5 deg steps.
// The predicted reading of a cell comes from a row term and a column term (one division per cell, no full table: the
// firmware has about 40 kB of heap free and no static RAM to spare). Memory 15 kB.
//   predict   the rover's motion (wheels, gyro) moves and turns the wire line in the rover frame, exactly for each cell
//   correct   each new reading weights every cell by a robust (Student-t) likelihood of its predicted reading
//   fade      a small blur and a uniform floor per metre driven and per turn: a short memory, so a curved wire looks
//             straight inside the window, and a wrong lock recovers
// Ambiguities (mirror psi, near/far elevation) stay as separate peaks until motion separates them.
//
// Frames: rover forward, left; psi = direction of the wire in the rover frame, counter-clockwise positive (the wire
// direction is the one with the sensor on its right, so d > 0). With a clockwise loop the follow direction has the
// inside on its right: then d > 0 means inside and yaw (rover heading minus wire direction) = -psi.
//
// No Arduino dependencies: the same file builds on the PC for the log replay (test/wire_fit).
#pragma once
#include <stdint.h>

namespace wirefit {

#ifndef WIRE_FIT_ND_HALF
#define WIRE_FIT_ND_HALF 25
#endif
#ifndef WIRE_FIT_NP
#define WIRE_FIT_NP 72
#endif
const int ND_HALF = WIRE_FIT_ND_HALF; // grid levels on each side of the wire (d = 0 shared)
const int ND = 2 * ND_HALF + 1;
const int NP = WIRE_FIT_NP;           // psi steps over 360 deg (72: 5 deg)

struct Config {
  float H = 0.226f;           // m, sensor 1 height plus wire depth (simulation fit; the reference setting 0.25)
  float sensorFwd = 0.128f;   // m, sensor 1 ahead of the turn centre (simulation fit)
  float sensorLeft = -0.023f; // m, sensor 1 left of the centreline (simulation fit)
  float alphaDeg = 0.0f;      // coil angle error; the model check found 0 to -2 deg, not separable from noise
  // Reading scatter about the straight-wire model. The model check gives median |ln r| 0.05 and angle 5 deg with long
  // tails (the rest of the loop bends the field, more so far out). Values chosen on the logs of 2 runs (departure
  // angle error 12 -> 5 deg median), checked on the other 2.
  float sigmaLnR = 0.20f;
  float sigmaLnRPerM = 0.40f;  // added per metre of reading
  float sigmaAngDeg = 10.0f;
  float sigmaAngDegPerM = 20.0f;  // added per metre of reading
  float rMax = 1.2f;          // m: readings farther than this are not a straight-wire field (simulation: the loop
                              // centre reads 1.07 m with the local wire 2.37 m away); no correction beyond it
  float fadePerM = 0.15f;     // share of the probability spread evenly per metre driven: memory about 1 m
  float fadePerRad = 0.03f;   // the same per radian turned (turns on the spot keep their data)
  float blurDegPerM = 8.0f;   // psi blur, 1 sd per metre driven: the wire may curve inside the window
  float blurDPerM = 0.03f;    // d blur, 1 sd per metre driven
  float polTrust = 0.0f;      // polarity side weight, 0 = not used, 0.6 = right 60 % of the time
};

struct Estimate {
  bool valid = false;
  float d = 0;               // m, horizontal distance sensor 1 to the wire (>= 0 side-free)
  float radial = 0;          // m, sqrt(d^2 + H^2)
  float elevDeg = 0;         // elevation of the wire below the horizontal at sensor 1, atan2(H, d)
  float psiDeg = 0;          // wire direction in the rover frame, with the sensor on the wire's right (side-free)
  float betaDeg = 0;         // departure angle: the angle of the heading to the wire line, + away, - toward
  float betaSdDeg = 0;       // spread of beta about the estimate (1 sd, the peak only)
  float peakShare = 0;       // probability within 15 deg of beta: 1 = one clear answer
  float dCentre = 0;         // m, the rover centre's horizontal distance (side-free frame)
  int8_t side = 0;           // with polarity: +1 sensor right of the follow direction (inside), -1, 0 not known
  float yawDeg = 0;          // with polarity: rover heading minus the follow direction, counter-clockwise positive
};

class WireFit {
 public:
  Config cfg;
  void reset();                                    // uniform probability; builds the tables
  void move(float ds, float dTheta);               // rover centre: m forward, rad counter-clockwise since the last call
  void reading(float x, float y, float r, int8_t side = 0);  // sensor 1 reading, m; side: polarity +1 inside, -1, 0
  Estimate estimate() const;
  float dGrid(int i) const { return d_[i]; }

 private:
  void apply();                                    // the motion since the last reading, then fade
  float p_[NP][ND];
  float tmp_[ND > NP ? ND : NP];
  float d_[ND], w_[ND];                            // grid d (m) and cell width (m); p_ is a density in d
  float hc_[NP / 2 + 1], d2_[ND], iR4_[ND];      // per row H cos(psi) (psi and -psi share it); per column d^2, 1/R^4
  float X_ = 0, Y_ = 0, T_ = 0, travel_ = 0, turned_ = 0;   // centre motion since the last apply (old frame)
  float H_ = 0;
};

}  // namespace wirefit
