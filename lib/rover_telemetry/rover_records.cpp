#include "rover_records.h"

#include <math.h>
#include <string.h>

namespace rec {

namespace {

void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void put32(uint8_t* p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

int16_t i16(float v) {  // round and clamp; NaN gives I16_NONE
  if (!(v == v)) return I16_NONE;
  if (v >= 32767.0f) return 32767;
  if (v <= -32767.0f) return -32767;
  return (int16_t)lroundf(v);
}
uint16_t u16(float v) {
  if (!(v == v) || v <= 0.0f) return 0;
  if (v >= 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}
uint8_t u8(float v) {
  if (!(v == v) || v <= 0.0f) return 0;
  if (v >= 255.0f) return 255;
  return (uint8_t)lroundf(v);
}

}  // namespace

uint16_t halfFromFloat(float f) {
  uint32_t x;
  memcpy(&x, &f, 4);
  const uint16_t sign = (uint16_t)((x >> 16) & 0x8000u);
  const uint32_t absx = x & 0x7FFFFFFFu;
  if (absx >= 0x7F800000u) return (uint16_t)(sign | (absx > 0x7F800000u ? 0x7E00u : 0x7C00u));  // NaN, inf
  if (absx >= 0x477FF000u) return (uint16_t)(sign | 0x7C00u);  // rounds to over 65504: inf
  if (absx < 0x33000001u) return sign;                         // under half the smallest subnormal: 0
  int e = (int)(absx >> 23) - 127;
  uint32_t m = (absx & 0x7FFFFFu) | 0x800000u;                 // 24-bit significand
  int shift;
  uint32_t he;
  if (e < -14) { shift = 13 + (-14 - e); he = 0; }             // subnormal half
  else { shift = 13; he = (uint32_t)(e + 15); }
  uint32_t mant = m >> shift;
  const uint32_t rem = m & ((1u << shift) - 1), halfway = 1u << (shift - 1);
  if (rem > halfway || (rem == halfway && (mant & 1u))) mant++;  // round to nearest even
  uint32_t h;
  if (he == 0) h = mant;                                       // subnormal (mant may carry to normal)
  else h = (he << 10) + (mant - 0x400u);                       // carry into the exponent is correct
  return (uint16_t)(sign | h);
}

float floatFromHalf(uint16_t h) {
  const uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
  const int e = (h >> 10) & 0x1F;
  const uint32_t m = h & 0x3FFu;
  float v;
  if (e == 0) v = ldexpf((float)m, -24);
  else if (e == 31) v = m ? NAN : INFINITY;
  else v = ldexpf((float)(m | 0x400u), e - 25);
  uint32_t x;
  memcpy(&x, &v, 4);
  x |= sign;
  memcpy(&v, &x, 4);
  return v;
}

uint16_t bf16FromFloat(float f) {
  uint32_t x;
  memcpy(&x, &f, 4);
  if ((x & 0x7FFFFFFFu) > 0x7F800000u) return (uint16_t)((x >> 16) | 0x0040u);  // NaN stays NaN
  x += 0x7FFFu + ((x >> 16) & 1u);  // round to nearest even (overflow rounds to inf)
  return (uint16_t)(x >> 16);
}

float floatFromBf16(uint16_t b) {
  const uint32_t x = (uint32_t)b << 16;
  float f;
  memcpy(&f, &x, 4);
  return f;
}

void packSample(uint8_t* o, const Sample& s, uint32_t block_ms) {
  put16(o + 0, (uint16_t)(s.ms - block_ms));
  put16(o + 2, (uint16_t)i16(s.x1_mm));
  put16(o + 4, (uint16_t)i16(s.y1_mm));
  put16(o + 6, u16(s.r1_mm));
  o[8] = s.flags;
  o[9] = s.ap;
  put16(o + 10, (uint16_t)i16(s.drive * 10000.0f));
  put16(o + 12, (uint16_t)i16(s.steer * 10000.0f));
  put16(o + 14, (uint16_t)i16(s.vel0_rps * 1000.0f));
  put16(o + 16, (uint16_t)i16(s.vel1_rps * 1000.0f));
  put16(o + 18, (uint16_t)(s.gyro ? i16(s.gyro_dps * 50.0f) : I16_NONE));
  o[20] = u8(s.btx1_mm * 0.25f);
  o[21] = u8(s.btx2_mm * 0.25f);
  for (int i = 0; i < 4; i++) put16(o + 22 + 2 * i, bf16FromFloat(s.coil[i]));
  put16(o + 30, (uint16_t)(s.xf_valid ? i16(s.xf_m * 1000.0f) : I16_NONE));
}

void packSampleImu(uint8_t* o, const Sample& s, uint32_t block_ms) {
  packSample(o, s, block_ms);
  put16(o + 32, (uint16_t)i16(s.ax_g * 1000.0f));  // 1 mg
  put16(o + 34, (uint16_t)i16(s.ay_g * 1000.0f));
  put16(o + 36, (uint16_t)i16(s.az_g * 1000.0f));
  put16(o + 38, (uint16_t)i16(s.gx_dps * 50.0f));  // 0.02 deg/s, as gyro z
  put16(o + 40, (uint16_t)i16(s.gy_dps * 50.0f));
}

int packStatus(uint8_t* o, const Status& s) {
  memset(o, 0, STATUS_BYTES);
  o[0] = BODY_STATUS;
  o[1] = s.drive_mode;
  o[2] = (uint8_t)s.charge_step;
  o[3] = s.vehicle_status;
  put16(o + 4, u16(s.vbatt * 1000.0f));
  put16(o + 6, u16(s.vcharge * 1000.0f));
  put16(o + 8, (uint16_t)i16(s.icharge * 1000.0f));
  put16(o + 10, u16(s.v5v * 1000.0f));
  const float t = s.temperature;
  o[12] = (uint8_t)(int8_t)(t > 127.0f ? 127 : t < -128.0f ? -128 : (int)lroundf(t));
  o[13] = s.ap_state;
  o[14] = s.ap_sub;
  o[15] = s.flags;
  put32(o + 16, s.status_bits);
  put32(o + 20, s.charge_code);
  put16(o + 24, (uint16_t)s.crc_errors);
  put16(o + 26, (uint16_t)s.overruns);
  put16(o + 28, (uint16_t)(s.packets_last_s > 65535u ? 65535u : s.packets_last_s));
  put16(o + 30, (uint16_t)(s.loop_max_us > 65535u ? 65535u : s.loop_max_us));
  put16(o + 32, (uint16_t)s.bounces);
  put16(o + 34, (uint16_t)s.turnarounds);
  put16(o + 36, (uint16_t)s.charges);
  put16(o + 38, u16(s.odrive_vbus * 1000.0f));
  put16(o + 40, (uint16_t)i16(s.odrive_ibus * 1000.0f));
  memcpy(o + 42, s.axis_state, 6);
  put32(o + 48, s.axis_errors_or);
  o[52] = s.error_nodes;
  o[53] = s.heartbeat_nodes;
  put32(o + 54, s.free_heap);
  put16(o + 58, (uint16_t)s.send_fail);
  put16(o + 60, (uint16_t)(s.backfill_last_s > 65535u ? 65535u : s.backfill_last_s));
  put16(o + 62, (uint16_t)(s.xf_valid ? i16(s.xf_m * 1000.0f) : I16_NONE));
  put16(o + 64, u16(s.dist_m * 1000.0f));
  o[66] = u8(s.angle_deg);
  o[67] = s.monitor_age_s;
  put32(o + 68, (uint32_t)(s.pc_unix_ms / 1000));
  put16(o + 72, (uint16_t)(s.pc_unix_ms % 1000));
  memcpy(o + 74, s.fw_ver, 3);
  return STATUS_BYTES;
}

int packEvent(uint8_t* o, uint16_t code, int32_t a, int32_t b, const char* text) {
  o[0] = BODY_EVENT;
  put16(o + 1, code);
  put32(o + 3, (uint32_t)a);
  put32(o + 7, (uint32_t)b);
  int n = 11;
  if (text) {
    const size_t room = (size_t)(EVENT_MAX - n);
    size_t len = strlen(text);
    if (len > room) len = room;
    memcpy(o + n, text, len);
    n += (int)len;
  }
  return n;
}

bool Block::add(const Sample& s, uint8_t rate_hz) {
  if (n_ == 0) {
    ms0_ = s.ms;
    imu_ = s.imu;
    rate_ = rate_hz;
    if (rate_ < 100) {
      buf_[0] = imu_ ? BODY_SAMPLES_IMU_DEC : BODY_SAMPLES_DEC;
      buf_[2] = rate_;
    } else {
      buf_[0] = imu_ ? BODY_SAMPLES_IMU : BODY_SAMPLES;
    }
  }
  if (imu_) packSampleImu(buf_ + hdr() + n_ * IMU_SAMPLE_BYTES, s, ms0_);
  else packSample(buf_ + hdr() + n_ * SAMPLE_BYTES, s, ms0_);
  n_++;
  buf_[1] = (uint8_t)n_;
  return n_ == (imu_ ? IMU_SAMPLES_PER_BLOCK : SAMPLES_PER_BLOCK);
}

uint32_t crc32(const uint8_t* p, size_t n, uint32_t crc) {
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

}  // namespace rec
