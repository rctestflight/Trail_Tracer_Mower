// stm_v2: see stm_v2.h.
#include "stm_v2.h"
#include "fast_pow.h"

#include <math.h>
#include <string.h>
#include <esp_attr.h>  // IRAM_ATTR

namespace stmv2 {

// CRC-16, polynomial 0x8005, init 0xFFFF, no reflection: one table lookup a byte (the bitwise loop took 55 us for a
// 126-byte frame on the ESP32-S2, rt bench 2026-10-12). The table is built once, on the first call.
// 0.5.102: crc16, mag and decode run from IRAM. From flash they took 375 us a frame in the loop (8 KB instruction
// cache, evicted by the rest of the pass) against 104 us in a tight loop (rt bench 2026-10-13).
IRAM_ATTR uint16_t crc16(const uint8_t* p, size_t n) {
  static uint16_t t[256];
  static bool built = false;
  if (!built) {
    for (int b = 0; b < 256; b++) {
      uint16_t c = (uint16_t)(b << 8);
      for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x8005) : (uint16_t)(c << 1);
      t[b] = c;
    }
    built = true;
  }
  uint16_t c = 0xFFFF;
  while (n--) c = (uint16_t)((c << 8) ^ t[((c >> 8) ^ *p++) & 0xFF]);
  return c;
}

Cal::Cal() {
  for (unsigned i = 0; i < WFRX_BINS; i++) {  // the v1 values (STM32 defaults of CID 21..24); bin gain 1
    for (int sn = 0; sn < 2; sn++) { gain[sn][i] = 1.0f; have_gain[sn][i] = true; }
    weak[i] = 1.0f / 600.0f;
    lost[i] = 1.0f / 1200.0f;
    regained[i] = 1.0f / 1000.0f;
  }
}

void Decoder::reset() {
  have_seq_ = false;
  cal = Cal();
  sweep_on = false;  // a new Rx STM32 starts with the sweep off
  sweep = PageSweep{};
  lost[0] = lost[1] = true;
  regain_frames_[0] = regain_frames_[1] = 0;
}

static float asFloat(uint32_t raw) { float f; memcpy(&f, &raw, 4); return f; }

void Decoder::learn(const Answer& ans) {  // values in effect, from reads and writes answered OK
  if (!ans.present || (ans.status != RSP_OK && ans.status != RSP_OK_CLAMPED)) return;
  const uint8_t id = ans.cid & 0x7F;
  const float v = asFloat(ans.value);
  const unsigned i = ans.index;
  switch (id) {
    case 26: if (i < 2 * WFRX_BINS) { cal.gain[i / WFRX_BINS][i % WFRX_BINS] = v; cal.have_gain[i / WFRX_BINS][i % WFRX_BINS] = true; } break;
    case 27: if (i < 2) { cal.radius[i] = v; cal.have_radius[i] = true; } break;
    case 28: if (i < 2) { cal.angle[i] = v; cal.have_angle[i] = true; } break;
    case 21: if (i < WFRX_BINS) cal.weak[i] = v; break;
    case 22: if (i < WFRX_BINS) cal.lost[i] = v; break;
    case 23: if (i < WFRX_BINS) cal.regained[i] = v; break;
    case 24: cal.regain_ms = ans.value; break;
    default: break;
  }
}

IRAM_ATTR static float mag(const WfrxCoilPair& p, float* s1, float* s2) {
  *s1 = (float)wfrx_int24_get(p.v1) / (float)(1L << WFRX_MAG_LSB_SHIFT);
  *s2 = (float)wfrx_int24_get(p.v2) / (float)(1L << WFRX_MAG_LSB_SHIFT);
  return sqrtf(*s1 * *s1 + *s2 * *s2);
}

bool Decoder::centreAngle(int s, float* phi_new) const {
  if (s < 0 || s > 1 || lost[s] || !cal.have_angle[s]) return false;
  *phi_new = cal.angle[s] + atan2f(a[s], b[s]);
  return true;
}

IRAM_ATTR bool Decoder::decode(const uint8_t* f, __StmTxPacketStruct& out, Answer& ans, int len, uint32_t now_ms) {
  StmFrameV2 fr;
  memcpy(&fr, f, sizeof(fr));  // the first 126 B are the same in versions 2 and 3; fr.crc is not used
  const uint16_t crc_rx = (uint16_t)(f[len - 2] | (f[len - 1] << 8));
  if (crc16(f, (size_t)len - 2) != crc_rx) { st.crc_err++; return false; }
  const bool v3 = len == (int)STM_FRAME_LEN_V3;
  if (fr.proto_ver != (v3 ? WFRX_PROTO_VER3 : WFRX_PROTO_VER)) { st.ver_err++; return false; }
  proto = fr.proto_ver;
  if (v3) {  // the extension slot: 4 items
    // 10 ms units; 0 means never came, so 0 becomes 1. (Before 0.6.21 this was "| 1", which put an even tick one unit
    // ahead of the clock: "now - t" then wrapped to 65535 and a fresh item read as stale about half the time.)
    uint16_t t10 = (uint16_t)(now_ms / 10u);
    if (!t10) t10 = 1;
    for (unsigned k = 0; k < WFRX_EXT_ITEMS; k++) {
      const uint8_t* it = f + WFRX_EXT_OFFSET + 4 * k;
      const uint8_t id = it[0], ix = it[1];
      if (id == 0) continue;
      if (id == WFRX_EXT_POLARITY) {  // 24 bits of codes in the index byte and the value
        const uint32_t bits = (uint32_t)ix | ((uint32_t)it[2] << 8) | ((uint32_t)it[3] << 16);
        for (unsigned k = 0; k < 10; k++) {
          const unsigned code = (bits >> (2 * k)) & 3u;
          pol[k / 5][k % 5] = code == 1 ? 1 : code == 3 ? -1 : 0;
        }
        pol_t10 = t10;
        ext_items++;
        continue;
      }
      if (id > WFRX_EXT_IDS || (ix & 0xE8u)) { ext_bad++; continue; }  // index: bits 0-2 (bin or Tx channel 0-4) and bit 4 only
      const unsigned slot = (ix & 7u) | ((ix >> 1) & 8u);
      int16_t v;
      memcpy(&v, it + 2, 2);
      ext_val[id - 1][slot] = v;
      ext_t10[id - 1][slot] = t10;
      ext_items++;
    }
  }
  st.frames++;
  if (have_seq_) st.seq_lost += (uint8_t)(fr.seq - last_seq_ - 1);
  have_seq_ = true;
  last_seq_ = fr.seq;
  status_bits = fr.status_bits;
  polar_on = (fr.status_bits & WFRX_STATUS_POLAR_STREAM) != 0;
  if (polar_on) {
    static_assert(sizeof(fr.coil[1]) == POLAR_BLOCK, "result 2 is the 48 B detector block");
    memcpy(polar, fr.coil[1], POLAR_BLOCK);
    memset(fr.coil[1], 0, sizeof(fr.coil[1]));
  }
  memcpy(coil, fr.coil, sizeof(coil));

  ans.present = fr.resp_cid != 0;
  ans.seq = fr.resp_seq;
  ans.cid = fr.resp_cid;
  ans.index = fr.resp_index;
  ans.status = fr.resp_status;
  memcpy(&ans.value, &fr.resp_value, 4);
  learn(ans);

  // v1 status bits: keep the ones with the v1 meaning; weak, lost and disabled come from below.
  const uint32_t retired = (1u << 0) | (1u << 4) | (1u << 5) | (1u << 9) | (1u << 10) | (1u << 17) | (1u << 18);
  uint32_t sb = fr.status_bits & ~retired;
  lanes = (uint8_t)(((fr.status_bits >> 22) & 1u) | (((fr.status_bits >> 23) & 1u) << 1));
  bool done[2] = {false, false};
  for (int k = 0; k < 2; k++) {
    const int s = (lanes >> k) & 1;  // the sensor of lane k
    if (done[s]) continue;           // both lanes on one sensor: lane 1 gives the sensor values
    done[s] = true;
    float s1, s2;
    // beacons: bins 5 and 6
    for (int j = 0; j < 2; j++) {
      const int bin = 5 + j;
      if (sweep_on && ((sweep.mask >> bin) & 1u)) {  // a sweep bin: no beacon
        if (s == 0) out.s1_btx_radius[j] = 32767.0f; else out.s2_btx_radius[j] = 32767.0f;
        continue;
      }
      const float m = mag(fr.coil[k][bin], &s1, &s2) * (cal.have_gain[s][bin] ? cal.gain[s][bin] : 0.0f);
      const float r = m > 0 ? 12.22f * fastpow::pow_(m, -0.330478f) : 32767.0f;  // powf to 1e-6 (fast_pow.h)
      if (s == 0) out.s1_btx_radius[j] = r; else out.s2_btx_radius[j] = r;
    }
    // wire bin
    const int wb = wire_bin_[s];
    float x = 0, y = 0, r = 32767.0f;
    bool weak = true;
    if (wb < 0 || wb >= (int)WFRX_BINS) {
      lost[s] = true;
      sb |= s == 0 ? (1u << 17) : (1u << 18);
      a[s] = b[s] = 0;
    } else {
      mag(fr.coil[k][wb], &s1, &s2);
      // coil_data[sensor] = {coil 1, 0, coil 2, 0} in int24 LSB (magnitude x 2^24): the v1 {re, im, re, im} places,
      // so the main-branch coilStrength() (src/main.cpp) gives each coil's magnitude on layout 2 too.
      float* cd = out.coil_data[s];
      cd[0] = s1 * 16777216.0f; cd[1] = 0; cd[2] = s2 * 16777216.0f; cd[3] = 0;
      const bool known = cal.have_gain[s][wb] && cal.have_radius[s] && cal.have_angle[s] && cal.radius[s] != 0;
      float m = 0;
      if (known) {
        if (!ang_ok_[s] || cal.angle[s] != ang_[s]) {
          ang_[s] = cal.angle[s]; cos_[s] = cosf(ang_[s]); sin_[s] = sinf(ang_[s]); ang_ok_[s] = true;
        }
        const float g = cal.gain[s][wb], c = cos_[s], n = sin_[s];
        const float u = g * s1, v = g * s2;
        // (s1, s2) = m (sin t, cos t), t = atan2(s1, s2) as the STM32 angle; (a, b) = (sin, cos) of t - phi
        a[s] = (u * c - v * n) / cal.radius[s];
        b[s] = (v * c + u * n) / cal.radius[s];
        m = sqrtf(a[s] * a[s] + b[s] * b[s]);
      } else {
        a[s] = b[s] = 0;
      }

      if (m > 0) { r = 1.0f / m; x = a[s] / (m * m); y = b[s] / (m * m); }
      // v1 rules: lost below the lost threshold; back after the regained threshold holds for the wait
      if (!known || m < cal.lost[wb]) { lost[s] = true; regain_frames_[s] = 0; }
      else if (lost[s]) {
        if (m > cal.regained[wb]) {
          if (++regain_frames_[s] * 10u >= cal.regain_ms) { lost[s] = false; regain_frames_[s] = 0; }
        } else {
          regain_frames_[s] = 0;
        }
      }
      weak = m < cal.weak[wb];
    }
    if (lost[s]) { x = 0; y = 0; sb |= s == 0 ? (1u << 5) : (1u << 10); }
    if (weak) sb |= s == 0 ? (1u << 4) : (1u << 9);
    if (s == 0) { out.s1_x = x; out.s1_y = y; out.s1_radius = r; }
    else        { out.s2_x = x; out.s2_y = y; out.s2_radius = r; }
  }
  out.status_bits = sb;

  page_sweep = false;
  if (sweep_on && (fr.seq & 3u) == 1) {
    sweep = fr.page.sweep;
    page_sweep = true;
  } else if ((fr.seq & 1u) == 0) {
    meas = fr.page.meas;
    have_meas = true;
    out.vbatt = meas.vbatt_mv * 0.001f;
    out.vcharge = meas.vcharge_mv * 0.001f;
    out.icharge = meas.icharge_ma * 0.001f;
    out.v5v = meas.v5v_mv * 0.001f;
    out.vesp_3v3 = meas.vesp_3v3_mv * 0.001f;
    out.temperature = meas.temperature_dc * 0.1f;
    out.charge_code = meas.charge_code;
  } else {
    health = fr.page.health;
    have_health = true;
  }
  out.rid = 0;
  out.response.int32 = 0;
  return true;
}

void encodeCmd(uint8_t seq, uint8_t cid8, uint8_t index, uint32_t value, uint8_t out[ESP_FRAME_LEN]) {
  out[0] = seq;
  out[1] = cid8;
  out[2] = index;
  out[3] = 0;
  memcpy(out + 4, &value, 4);
  const uint16_t c = crc16(out, 8);
  out[8] = (uint8_t)(c & 0xFF);
  out[9] = (uint8_t)(c >> 8);
}

void encodeUartFrame(uint8_t version, int16_t angle, uint32_t flags, uint8_t seq, uint8_t cid8, uint8_t index, uint32_t value,
                     uint8_t out[ESP_UART_FRAME_LEN], uint16_t crc_xor) {
  out[0] = version;
  memcpy(out + 1, &angle, 2);
  memcpy(out + 3, &flags, 4);
  out[7] = seq;
  out[8] = cid8;
  out[9] = index;
  memcpy(out + 10, &value, 4);
  const uint16_t c = crc16(out, 14) ^ crc_xor;
  out[14] = (uint8_t)(c & 0xFF);
  out[15] = (uint8_t)(c >> 8);
}

}  // namespace stmv2
