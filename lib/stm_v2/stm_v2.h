// stm_v2: the Trail Tracer v2.0 STM32 -> ESP32 frame, layout 2 (128 B), and the 10 B command frame
// (include/rctf_wfrx_comms_v2.h, shared with TrailTracer-Rx-STM32). No Arduino dependency: builds on a
// PC (test/telemetry_host/stmv2_test.cpp).
//
// The STM32 sends 2 results (lane 1, lane 2) x 8 bins of signed coil magnitudes (s1, s2), in every
// frame and every signal state. All bins stay in magnitude mode (the default), so the raw values stay
// available. The decoder turns them into the v1 packet struct (__StmTxPacketStruct) that the rest of
// the firmware reads, so the drive, charge, autopilot and log code stay as they are:
//   - wire position of each sensor from its wire bin (setWireBin; v1 rx_channel - 1, or -1 = off):
//     a = g(s1 cos phi - s2 sin phi)/R, b = g(s2 cos phi + s1 sin phi)/R (the STM32 angle t = atan2(s1, s2),
//     less phi); x = a/(a^2+b^2), y = b/(a^2+b^2), r = 1/sqrt(a^2+b^2)
//     (STM32 app design, 2026-10-11). g = bin gain: 1 (the magnitudes are PGA-calibrated; an answer to a
//     CID 26 read would replace it). R = radius calibration (CID 27), phi = angle calibration (CID 28):
//     read once at start (setup), learned from the answers.
//   - weak and lost per sensor with the v1 rules, from the magnitude m = sqrt(a^2+b^2): lost below 1/1200,
//     back after m stays above 1/1000 for 500 ms, weak below 1/600 (Cal: the ESP32's own values; answers
//     to CID 21..24 would replace them). Written into the v1 status bits 4, 5
//     (sensor 1) and 9, 10 (sensor 2); x = y = 0 while lost, as v1. A sensor with its wire bin off sets
//     the v1 disabled bit (17, 18). Values unknown yet (no answer so far): lost.
//   - beacon radius: 12.22 * m^-0.330478 with m = sqrt(s1^2+s2^2) * bin gain (1 unless read, bins 5, 6).
//     Bin 5 (62.5 kHz) -> btx_radius[0], bin 6 (46.875 kHz) -> btx_radius[1], as v1.
//   - MEAS page: vbatt, vcharge, icharge, v5v, vesp_3v3, temperature, charge_code. HEALTH page: kept.
//   - coil_data[sensor] = coil 1, 0, coil 2, 0 of the wire bin, int24 LSB (the v1 re/im places: coilStrength()).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "rctf_wfrx_comms_v2.h"
#include "wfrx_stm32_comms.h"

namespace stmv2 {

// CRC-16, poly 0x8005, init 0xFFFF, no reflection, no final XOR (check 0xAEE7).
uint16_t crc16(const uint8_t* p, size_t n);

struct Answer {
  bool present = false;  // resp_cid != 0
  uint8_t seq = 0;
  uint8_t cid = 0;       // with the write bit (0x80)
  uint8_t index = 0;
  uint8_t status = 0;    // RspStatus
  uint32_t value = 0;    // raw 32 bits
};

struct Stats {
  uint32_t frames = 0;    // good frames
  uint32_t crc_err = 0;
  uint32_t ver_err = 0;   // good CRC, proto_ver not 2
  uint32_t seq_lost = 0;  // frames lost, from seq gaps
};

// STM32 values that the ESP32 needs for the conversion, learned from command answers.
struct Cal {
  float gain[2][WFRX_BINS] = {};
  bool have_gain[2][WFRX_BINS] = {};
  float radius[2] = {0, 0};
  float angle[2] = {0, 0};
  bool have_radius[2] = {false, false};
  bool have_angle[2] = {false, false};
  float weak[WFRX_BINS], lost[WFRX_BINS], regained[WFRX_BINS];
  uint32_t regain_ms = 500;
  Cal();
};

class Decoder {
 public:
  // false: CRC or version bad, out not changed. true: out updated, ans holds this frame's answer.
  bool decode(const uint8_t* f, __StmTxPacketStruct& out, Answer& ans);
  void setWireBin(int sensor, int bin) { if (sensor == 0 || sensor == 1) wire_bin_[sensor] = bin; }
  int wireBin(int sensor) const { return wire_bin_[sensor]; }
  void reset();  // the STM32 may be new: forget the seq and the learned values
  uint8_t lastSeq() const { return last_seq_; }  // seq of the last good frame
  // Centre calibration (v1 CID 4/5): the new angle calibration of a sensor, phi + atan2(a, b) of its
  // present wire bin (STM32 app design, 2026-10-11). false: no values yet.
  bool centreAngle(int sensor, float* phi_new) const;

  Stats st;
  Cal cal;
  uint32_t status_bits = 0;  // as sent
  uint8_t lanes = 0x02;      // status bits 22, 23
  PageMeas meas{};
  PageHealth health{};
  bool have_meas = false, have_health = false;
  float a[2] = {0, 0}, b[2] = {0, 0};  // per sensor, the wire bin after calibration
  bool lost[2] = {true, true};
  WfrxCoilPair coil[WFRX_RESULTS][WFRX_BINS];  // the last frame's coil data (bench print)

 private:
  void learn(const Answer& ans);
  int wire_bin_[2] = {0, 0};
  uint32_t regain_frames_[2] = {0, 0};
  bool have_seq_ = false;
  uint8_t last_seq_ = 0;
};

// One command frame (10 B): seq, cid (bit 7 = write), index, reserved 0, value, CRC.
void encodeCmd(uint8_t seq, uint8_t cid8, uint8_t index, uint32_t value, uint8_t out[ESP_FRAME_LEN]);

}  // namespace stmv2
