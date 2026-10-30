// Record bodies of the rover telemetry log (docs/espnow_protocol.md, "Telemetry log"):
// 0x20 sample block and 0x22 sample block with the six IMU axes (stream 0), 0x21 status and
// 0x30 event (stream 1). Packing only; the
// numbering, storage and radio are in lib/telemlog. No Arduino dependency: builds on a PC.
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace rec {

const uint8_t BODY_SAMPLES = 0x20, BODY_STATUS = 0x21, BODY_SAMPLES_IMU = 0x22, BODY_EVENT = 0x30;
// 0x24 and 0x25: 0x20 and 0x22 at a log rate below 100 Hz (lib/rover_telemetry/decimator.h). One
// more header byte, the rate in Hz: type, count, rate, then the samples (same layout).
const uint8_t BODY_SAMPLES_DEC = 0x24, BODY_SAMPLES_IMU_DEC = 0x25;
const int SAMPLE_BYTES = 32, SAMPLES_PER_BLOCK = 7;
// 0x22: the 32 bytes of 0x20, then accelerometer x, y, z and gyro x, y (gyro z is at byte 18).
// A block holds 5 samples: 2 + 5 x 42 = 212 bytes, 20 blocks a second at 100 Hz.
const int IMU_SAMPLE_BYTES = 42, IMU_SAMPLES_PER_BLOCK = 5;
const int BLOCK_BYTES = 3 + SAMPLES_PER_BLOCK * SAMPLE_BYTES;  // 227, the largest (0x24)
// Layout 3 (0.5.58), for offline track reconstruction: the 32 bytes of layout 1, then sensor 2 beacons,
// cumulative wheel positions with their ages, sensor 2 x/y and the gyro heading (50 bytes; 60 with the
// IMU axes). 0x2A 100 Hz, 0x2B with IMU, 0x2C decimated, 0x2D decimated with IMU. Layout 1 stays readable.
const uint8_t BODY_SAMPLES3 = 0x2A, BODY_SAMPLES3_IMU = 0x2B, BODY_SAMPLES3_DEC = 0x2C, BODY_SAMPLES3_IMU_DEC = 0x2D;
const int SAMPLE3_BYTES = 50, SAMPLES3_PER_BLOCK = 4, IMU_SAMPLE3_BYTES = 60, IMU_SAMPLES3_PER_BLOCK = 3;
// 0x2E (0.5.66, stream 0): the coil magnitudes of all 8 frequency bins of both sensors, for the per-bin plots
// of the telemetry app. From 0.5.68 at the rate telemetry_bins_hz, through the log's anti-alias filter
// (decimator.h). type, filter factor N (u8; 0.5.66: frames averaged), output period ms (u16; 0.5.66: window),
// 8 x bin frequency in Hz/2 (u16, 0 = bin off), then per bin: sensor 1 coil 1, sensor 1 coil 2, sensor 2 coil 1,
// sensor 2 coil 2 as int24 in Rx STM32 magnitude LSB (2^-24): the filtered |v| with the sign of the filtered v.
// BINS_NONE: no data (the sensor is on no lane, or the bin is off).
const uint8_t BODY_BINS = 0x2E;
const int BINS = 8, BINS_BYTES = 4 + BINS * 2 + BINS * 4 * 3;  // 116
const int32_t BINS_NONE = -8388608;  // 0x800000
// 0x31 (0.5.69, stream 1, each 10 s and after a change): the vehicle name. type, vehicleID (u8), name length n
// (u8, 16 at most), n name bytes (A-Z a-z 0-9 space _ -).
const uint8_t BODY_NAME = 0x31;
// 0x32 (0.5.87, stream 0): bin sweep readings with their grid points (Rx STM32 2.0.51+). type, table id (u8; a change
// starts a new trace), f0 (u24, 0.1 Hz), step (u16, 0.1 Hz), grid points P (u16), sweep bins n (u8), updates u (u8),
// then u x n x {grid point g (u16), sensor 1 coil 1, coil 2, sensor 2 coil 1, coil 2 (int24, as 0x2E)}. Frequency of g:
// (f0 + g x step) / 10 Hz. BINS_NONE: the sensor is on no lane.
const uint8_t BODY_SWEEP = 0x32;
const int SWEEP_HDR = 11;
// 0x33 (0.5.103, stream 0; replaces 0x32): bin sweep readings as the Rx STM32 gives them; the PC places them on the grid.
// type, table id (u8), f0 (u24, 0.1 Hz), step (u16, 0.1 Hz), grid points P (u16), sweep bins n (u8), bin mask (u8),
// frames u (u8), then u x {frame seq (u8), flags (u8: bit 0 the frame holds the sweep page, bit 1 frame overrun), tag
// (u16), previous-frame tag (u16; both 0xFFFF on a frame without the page), n x 4 int24 (sensor 1 coil 1, coil 2,
// sensor 2 coil 1, coil 2, as 0x2E)}. BINS_NONE: the sensor is on no lane.
const uint8_t BODY_SWEEP_RAW = 0x33;
const int SWEEP_RAW_HDR = 12;
const int SWEEP_ENTRY_HDR = 6;
// 0x37 (0.5.120, stream 0): polarity detector blocks as the Rx STM32 gives them (2.0.67+, status bit 26, CID 52 on).
// type, frames u (u8), then u x {frame seq (u8), n samples (u8, 0-10), detector (bits 0-2) and samples lost (bit 7)
// (u8), gap count (u16), output index of the first sample (u16), input index after the last housekeeping gap (u16),
// n x (re, im) int16}. 978 samples a second.
const uint8_t BODY_POLAR = 0x37;
const int POLAR_HDR = 2;
const int VEHICLE_NAME_MAX = 16;
const int STATUS_BYTES = 84;  // 67 up to 0.5.20; 68 in 0.5.21 (monitor age); 74 in 0.5.22 and 0.5.23 (PC time); 77 from 0.5.24 (firmware version); 80 from 0.5.97 (CPU load); 84 from 0.5.104 (update period in effect)
const int EVENT_MAX = 80;  // telemlog SLOW body limit
const int16_t I16_NONE = -32768;

// Sample flags (byte 8)
const uint8_t F_LOST = 1, F_OVERLOAD = 2, F_DISABLED = 4, F_WEAK = 8, F_ENTRY_BTX = 16, F_EXIT_BTX = 32,
              F_CHARGING = 64, F_WHEELS_FRESH = 128;
const uint8_t AP_OFF = 0xFF;

// Event codes
enum EventCode : uint16_t {
  EV_BOOT = 1, EV_MODE = 2, EV_AP_STATE = 3, EV_STUCK = 4, EV_CHARGE_STEP = 5, EV_ODRIVE_ERROR = 6,
  EV_HALTED = 7, EV_SETTING = 8, EV_STALL = 9, EV_BAD_KEY = 10,
  // EV_SETTING: a result (0 ok, 1 unknown key, 2 bad value, 4 refused), b source (0 USB, 1 ESP-NOW, 2 config.json or NVS:
  // a value that took its default, 0.5.112-settings; with a 0 from 0.6.30: a key that config.json changed), text the
  // command and its answer
  EV_RUN = 11,  // run start / run stop / STOP frame: a 1 start 0 stop, b result, text reason
  EV_IMU = 12,  // boot: a accelerometer range in g (0 = no IMU), b gyro range in deg/s, text offsets
  EV_RESTART_REFUSED = 13,  // restart or ROM-mode request while the vehicle moves: a 1 restart 2 ROM, b source (0 USB 1 ESP-NOW)
  EV_PREV_BOOT = 14,  // boot: the state before the reset. a reset reason (esp_reset_reason), b uptime ms, text stage, mode, heap
  EV_STM32 = 15,      // power-on STM32 handshake: a 1 HELLO answered (text: its payload in hex) 0 no answer, b payload bytes
                      // 0.6.34: a 2: the wire bin, radius, angle and wire-bin gain of each sensor in use (text), at each
                      // Rx STM32 start and change
  EV_TIME = 16,       // PC time from a monitor (MON 0x44): a unix seconds, b millis() at that time, text unix ms
  // 17 to 20: see the event names in include/telemetry_table.h
  EV_SETTINGS = 21,        // all settings ("list" lines), at boot and each 10 min: a part (0..), b part count, text
                           // "key=value key=value ..." (values as "list" prints them: text in quotes)
  EV_SETTING_CHANGE = 22,  // a setting changed (any source: app, USB, config.json, Rx STM32 effective value): a the
                           // "list" index, text "key old -> new"
  EV_MONITOR = 17,    // a monitor heard (a 1) or no MON frame for 5 s (a 0); text its MAC
  EV_WIFI = 19,       // radio_mode "wifi" (lib/wifi_telem): a 1 associated (b RSSI dBm), 2 association lost, 3 server address
                      // learned (b port), 4 no server in 30 s (b 30); text SSID, IP, server. Never the password
  EV_STM32_CMD = 18,  // STM32 command link (lib/stm_settings, 0.5.25): a kind (stmset::Kind), b command ID, gap ms or HELLO/BOOT result; text
  EV_USB = 20,        // 0.5.104: USB link state change: a 1 output stop seen, 2 guard reconnect, 3 host port opened, 4 host port
                      // closed, 5 no USB input for 60 s with the port open, 6 port closed 60 s with the bus up (text: stack counters);
                      // b the count of that kind; text the counters. 0.6.34: 7 USB connected (the host enumerated the
                      // board), 8 USB disconnected; b 0 at the first check after the start (the state then), else the count
  EV_BUTTON = 23,     // 0.6.34: a button press: a action (ButtonAction, include/rx_platform.h: 0 none, 1 auto drive mode on,
                      // 2 cruise toggle, 3 centre calibration, 4 steering direction, 5 erase hold start, 6 erase cancelled,
                      // 7 erase done), b press length ms, text the action and its result
  EV_SONAR = 24,      // 0.7.5: the sonar (src/main.cpp). a 1 to 4: a sonar trigger, the sonar number; b its distance (the Nano
                      // unit, mm); text the action. a 0: the Nano stream; b 1 records arrive (the first of the start, or
                      // again after a stop), 0 no record for SONAR_STALE_MS (the rover ignores the sonar)
};

// One sample, in engineering units; packSample() quantises it.
struct Sample {
  uint32_t ms = 0;              // millis() of the packet
  float x1_mm = 0, y1_mm = 0, r1_mm = 0;
  uint8_t flags = 0;
  uint8_t ap = AP_OFF;          // state << 4 | sub, or AP_OFF
  float drive = 0, steer = 0;   // ODrive commands, -1..1
  float vel0_rps = 0, vel1_rps = 0;
  bool gyro = false;
  float gyro_dps = 0;           // gyro z (yaw rate)
  bool imu = false;             // true: the six axes go out in a 0x22 block
  float ax_g = 0, ay_g = 0, az_g = 0, gx_dps = 0, gy_dps = 0;
  float btx1_mm = 1e9f, btx2_mm = 1e9f;
  float coil[4] = {0, 0, 0, 0};  // coil_data[0][0..3]
  bool xf_valid = false;
  float xf_m = 0;
  // layout 3
  float btx1_s2_mm = 1e9f, btx2_s2_mm = 1e9f;  // sensor 2 beacon distances
  int32_t enc_pos[2] = {0, 0};   // wheel position per side, 1e-4 turn, cumulative since boot
  uint8_t enc_age_ms[2] = {255, 255};  // ms since that position arrived; 255 = 255 ms or more, or none
  float x2_mm = NAN, y2_mm = NAN; // sensor 2 position; NaN = sensor 2 off or no wire
  uint16_t heading = 0;          // integral of gyro z, 360/65536 deg, wraps; zero at boot
};

struct Status {
  uint8_t drive_mode = 0;
  int8_t charge_step = 0;
  uint8_t vehicle_status = 0;
  float vbatt = 0, vcharge = 0, icharge = 0, v5v = 0, temperature = 0;
  uint8_t ap_state = AP_OFF, ap_sub = 0, flags = 0;
  uint32_t status_bits = 0, charge_code = 0;
  uint32_t crc_errors = 0, overruns = 0;
  uint32_t packets_last_s = 0, loop_max_us = 0;
  uint32_t bounces = 0, turnarounds = 0, charges = 0;
  float odrive_vbus = 0, odrive_ibus = 0;
  uint8_t axis_state[6] = {0, 0, 0, 0, 0, 0};
  uint32_t axis_errors_or = 0;
  uint8_t error_nodes = 0, heartbeat_nodes = 0;
  uint32_t free_heap = 0;
  uint32_t send_fail = 0, backfill_last_s = 0;
  bool xf_valid = false;
  float xf_m = 0, dist_m = 0, angle_deg = 0;
  uint8_t monitor_age_s = 255;  // seconds since the last MON frame from a monitor; 255 = none or 255 s and more
  uint64_t pc_unix_ms = 0;      // PC time at this record (from MON frames); 0 = unknown
  uint8_t fw_ver[3] = {0, 0, 0}; // firmware version major, minor, patch
  uint32_t dsp_period_us = 0;  // 0.5.104: the update period in effect (the Rx STM32 answer to CID 18), us; 0 = no answer yet
  uint8_t cpu_busy = 0, cpu_loop = 0, cpu_drive = 0;  // 0.5.97: CPU load of the last second, % (all tasks but idle; loop(); its drive parts)
};

// Status flags (byte 15)
const uint8_t S_GO_HOME = 1, S_GYRO = 2, S_CAN = 4, S_ESPNOW = 8, S_ROVER_ENABLED = 16, S_LOCKED = 32,
              S_IMU_FAKE = 64,  // the IMU values are bench fill (BENCH_FAKE_IMU), not measurements
              S_MOVING = 128;   // not standing still by vehicleStationary() (stationary-only settings refused)

uint16_t halfFromFloat(float f);   // IEEE 754 binary16, round to nearest even
float floatFromHalf(uint16_t h);
// bfloat16: the top 16 bits of a float32, rounded to nearest even. The float32 exponent range,
// 8 significant bits (relative error 0.4 % at most). For the coil values, whose scale is unknown.
uint16_t bf16FromFloat(float f);
float floatFromBf16(uint16_t b);

void packSample(uint8_t* out, const Sample& s, uint32_t block_ms);     // SAMPLE_BYTES
void packSampleImu(uint8_t* out, const Sample& s, uint32_t block_ms);  // IMU_SAMPLE_BYTES
void packSample3(uint8_t* out, const Sample& s, uint32_t block_ms);    // SAMPLE3_BYTES
void packSample3Imu(uint8_t* out, const Sample& s, uint32_t block_ms); // IMU_SAMPLE3_BYTES
int packStatus(uint8_t* out, const Status& s);                      // returns STATUS_BYTES
// Event body: code, a, b, then text cut to fit EVENT_MAX. Returns its length.
int packEvent(uint8_t* out, uint16_t code, int32_t a, int32_t b, const char* text);
// Name body 0x31. Returns its length (3 + name length).
int packName(uint8_t* out, uint8_t vehicle_id, const char* name);
// Bin body 0x2E. v[bin][k]: k = sensor * 2 + coil; values outside int24 are clamped. Returns BINS_BYTES.
int packBins(uint8_t* out, uint8_t frames, uint16_t window_ms, const float hz[BINS], const int32_t v[BINS][4]);

// Collects samples into one sample body: layout 3 (0x2A-0x2D, the firmware) or layout 1 (0x20-0x25,
// the host tests). The IMU body when the first sample has imu set.
// add() returns true when the block is full; the caller then stores body()/size() and calls
// clear(). fits() is false for a sample more than 65 s after the block's first one, or with the
// other imu setting: the caller then stores the block as it is first.
class Block {
 public:
  explicit Block(int layout = 3) : layout_(layout) {}
  bool add(const Sample& s, uint8_t rate_hz = 100);  // rate below 100: body 0x24 or 0x25
  int count() const { return n_; }
  uint32_t ms() const { return ms0_; }
  const uint8_t* body() const { return buf_; }
  int size() const { return hdr() + n_ * sampleBytes(); }
  void clear() { n_ = 0; }
  bool fits(const Sample& s, uint8_t rate_hz = 100) const {
    return n_ == 0 || (s.ms - ms0_ <= 65000u && s.imu == imu_ && rate_hz == rate_);
  }
 private:
  int hdr() const { return rate_ < 100 ? 3 : 2; }
  int sampleBytes() const {
    return layout_ == 3 ? (imu_ ? IMU_SAMPLE3_BYTES : SAMPLE3_BYTES) : (imu_ ? IMU_SAMPLE_BYTES : SAMPLE_BYTES);
  }
  int perBlock() const {
    return layout_ == 3 ? (imu_ ? IMU_SAMPLES3_PER_BLOCK : SAMPLES3_PER_BLOCK) : (imu_ ? IMU_SAMPLES_PER_BLOCK : SAMPLES_PER_BLOCK);
  }
  int layout_ = 3;
  uint8_t buf_[BLOCK_BYTES];
  int n_ = 0;
  bool imu_ = false;
  uint8_t rate_ = 100;
  uint32_t ms0_ = 0;
};

uint32_t crc32(const uint8_t* p, size_t n, uint32_t crc = 0);  // CRC-32 (zlib), for the config hash

}  // namespace rec
