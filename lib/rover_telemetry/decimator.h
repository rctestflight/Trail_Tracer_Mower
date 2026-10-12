// Anti-alias filter and decimation of the per-packet samples (about 100 Hz) to a lower log rate
// (docs/espnow_protocol.md, "Sample rate"). No Arduino dependency: builds on a PC.
//
// Rates: 100 Hz divided by N = 1, 2, 4, 5, 10, 20, 25, 50, 100 (100, 50, 25, 20, 10, 5, 4, 2, 1 Hz).
// Filter: two moving averages of N inputs in series = a triangle of weights 1..N..1 over 2N-1
// inputs, its peak on the output sample's own time. All weights are positive: no overshoot and no
// ringing, and every channel has the same delay (N inputs). Gain at the output's Nyquist frequency:
// (2/pi)^2 = 0.41; sinc^2 nulls at each multiple of the output rate, where the aliases come from.
//
// One output per block of N inputs, from a sum S and a first moment M per channel and block
// (inputs i = 0..N-1 of the block):
//   y = ((M_prev + S_prev) + ((N-1) S_cur - M_cur)) / N^2
// = weights 1..N over the previous block and N-1..0 over the current one. The output carries the
// time of the previous block's last input. Cost per input: 2 additions and 1 multiplication per
// channel; state: 4 floats per channel, no store of past inputs.
//
// Channels that are not averaged:
//   flags      OR over the inputs of the output's own block: a flag that was set in any input shows
//   ap         the last value of that block
//   imu, gyro  true only when every input in the triangle had them
//   r1         averaged only over inputs with a wire (r1 < 65535); none: the "no wire" value
//   gyro z, xf averaged only over inputs where they are valid; none: invalid
//   btx        clamped to the packed range (1020 mm) before the average
#pragma once
#include "rover_records.h"

namespace rec {

const int DECIM_IN_HZ = 100;
bool rateAllowed(int hz);      // 100, 50, 25, 20, 10, 5, 4, 2 or 1
int rateNearest(int hz);       // the allowed rate nearest to hz (ties to the higher rate)

class Decimator {
 public:
  void setRate(int out_hz);    // nearest allowed rate; clears the filter
  int rate() const { return rate_; }
  int factor() const { return n_; }
  // One input sample. Returns true when *out holds a new output sample. At 100 Hz every input
  // goes straight out, with no delay.
  bool push(const Sample& in, Sample* out);

 private:
  enum { X1, Y1, DRIVE, STEER, VEL0, VEL1, AX, AY, AZ, GX, GY, BTX1, BTX2, C0, C1, C2, C3,
         R1, R1_W, GZ, GZ_W, XF, XF_W, NCH };
  void endBlock();
  int rate_ = DECIM_IN_HZ;
  int n_ = 1;
  int i_ = 0;                  // input index in the current block
  int blocks_ = 0;             // complete blocks since setRate (an output needs 2)
  float s_[NCH] = {}, m_[NCH] = {}, ps_[NCH] = {}, pm_[NCH] = {};
  uint8_t flags_ = 0, pflags_ = 0;
  uint8_t ap_ = AP_OFF, pap_ = AP_OFF;
  bool imu_ = true, pimu_ = true, gyro_ = true, pgyro_ = true;
  uint32_t ms_ = 0, pms_ = 0;  // time of the block's last input
};

}  // namespace rec
