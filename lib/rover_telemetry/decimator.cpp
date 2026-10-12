#include "decimator.h"

namespace rec {

namespace {
const int RATES[] = {100, 50, 25, 20, 10, 5, 4, 2, 1};
const float R1_NONE = 65535.0f;   // packSample clamps r1 here: "no wire"
const float BTX_MAX = 1020.0f;    // packSample: 255 x 4 mm
inline float clampBtx(float v) { return v > BTX_MAX ? BTX_MAX : (v < 0 ? 0 : v); }
}  // namespace

bool rateAllowed(int hz) {
  for (int r : RATES) if (r == hz) return true;
  return false;
}

int rateNearest(int hz) {
  int best = RATES[0];
  for (int r : RATES) {
    const int d = r > hz ? r - hz : hz - r, db = best > hz ? best - hz : hz - best;
    if (d < db) best = r;
  }
  return best;
}

void Decimator::setRate(int out_hz) {
  rate_ = rateNearest(out_hz);
  n_ = DECIM_IN_HZ / rate_;
  i_ = 0;
  blocks_ = 0;
  for (int c = 0; c < NCH; c++) s_[c] = m_[c] = ps_[c] = pm_[c] = 0;
  flags_ = pflags_ = 0;
  ap_ = pap_ = AP_OFF;
  imu_ = pimu_ = gyro_ = pgyro_ = true;
}

void Decimator::endBlock() {
  for (int c = 0; c < NCH; c++) {
    ps_[c] = s_[c];
    pm_[c] = m_[c];
    s_[c] = m_[c] = 0;
  }
  pflags_ = flags_;
  pap_ = ap_;
  pimu_ = imu_;
  pgyro_ = gyro_;
  pms_ = ms_;
  flags_ = 0;
  imu_ = gyro_ = true;
  i_ = 0;
  blocks_++;
}

bool Decimator::push(const Sample& in, Sample* out) {
  if (n_ == 1) {
    *out = in;
    return true;
  }
  float v[NCH];
  v[X1] = in.x1_mm; v[Y1] = in.y1_mm; v[DRIVE] = in.drive; v[STEER] = in.steer;
  v[VEL0] = in.vel0_rps; v[VEL1] = in.vel1_rps;
  v[AX] = in.ax_g; v[AY] = in.ay_g; v[AZ] = in.az_g; v[GX] = in.gx_dps; v[GY] = in.gy_dps;
  v[BTX1] = clampBtx(in.btx1_mm); v[BTX2] = clampBtx(in.btx2_mm);
  for (int k = 0; k < 4; k++) v[C0 + k] = in.coil[k];
  const bool wire = in.r1_mm < R1_NONE;
  v[R1] = wire ? in.r1_mm : 0; v[R1_W] = wire ? 1.0f : 0;
  v[GZ] = in.gyro ? in.gyro_dps : 0; v[GZ_W] = in.gyro ? 1.0f : 0;
  v[XF] = in.xf_valid ? in.xf_m : 0; v[XF_W] = in.xf_valid ? 1.0f : 0;
  const float fi = (float)i_;
  for (int c = 0; c < NCH; c++) {
    s_[c] += v[c];
    m_[c] += fi * v[c];
  }
  flags_ |= in.flags;
  ap_ = in.ap;
  imu_ = imu_ && in.imu;
  gyro_ = gyro_ && in.gyro;
  ms_ = in.ms;
  if (++i_ < n_) return false;

  // Block complete: the output centred on the previous block's last input.
  bool ready = blocks_ >= 1;
  float y[NCH];
  if (ready) {
    const float n1 = (float)(n_ - 1), inv = 1.0f / ((float)n_ * (float)n_);
    for (int c = 0; c < NCH; c++) y[c] = ((pm_[c] + ps_[c]) + (n1 * s_[c] - m_[c])) * inv;
    Sample o;
    o.ms = pms_;
    o.x1_mm = y[X1]; o.y1_mm = y[Y1]; o.drive = y[DRIVE]; o.steer = y[STEER];
    o.vel0_rps = y[VEL0]; o.vel1_rps = y[VEL1];
    o.ax_g = y[AX]; o.ay_g = y[AY]; o.az_g = y[AZ]; o.gx_dps = y[GX]; o.gy_dps = y[GY];
    o.btx1_mm = y[BTX1]; o.btx2_mm = y[BTX2];
    for (int k = 0; k < 4; k++) o.coil[k] = y[C0 + k];
    o.r1_mm = y[R1_W] > 0 ? y[R1] / y[R1_W] : 1e9f;
    o.gyro = pgyro_ && gyro_ && y[GZ_W] > 0;
    o.gyro_dps = o.gyro ? y[GZ] / y[GZ_W] : 0;
    o.xf_valid = y[XF_W] > 0;
    o.xf_m = o.xf_valid ? y[XF] / y[XF_W] : 0;
    o.imu = pimu_ && imu_;
    o.flags = pflags_;
    o.ap = pap_;
    *out = o;
  }
  endBlock();
  return ready;
}

}  // namespace rec
