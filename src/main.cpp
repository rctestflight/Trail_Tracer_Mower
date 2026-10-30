// Trail Tracer Rx: the vehicle program.
//
// This file holds what the vehicle does: the drive modes, the steering PID and its filter, the throttle, wire
// follow, BTx follow, the rover autopilot step, charging, and the choice of servo or CAN motor output. Edit this file
// to change how the vehicle drives.
//
// The board itself (USB, radio, telemetry, settings store, updates, the Rx STM32 link, bench commands, watchdog) is
// in src/platform/. include/rx_platform.h lists the board functions this file calls, each with a short description.
//
// The order of this file: the settings values and the vehicle state, the motor output, setup(), loop(), then the
// drive modes, the button, charging, BTx follow, the settings and the rover autopilot.

#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>
#include "frame_pid.h"     // the steering PIDs stepped once per STM32 frame (lib/frame_pid)
#include "frame_period.h"  // packet counts, steps and filter weights kept in time at any update period (lib/frame_pid)
#include "rover_autopilot.h"
#include "rctf_wfrx_status_bits.h"
#include "rctf_wfrx_exp32.h"
#include "tlm.h"           // table values: one line in include/telemetry_table.h defines a logged value (lib/tlm)
#include "rx_platform.h"   // the board: pins, Rx STM32 frames, motor controllers, servo outputs, gyro, settings, USB text

static const char* TAG = "wfrx";

// Latest complete Nano record: four unsigned ranges, comma separated, ending in LF or CRLF.
// Units are passed through exactly as sent by the Nano.
static uint32_t sonar_ranges[4] = {};
static bool sonar_pending = false;
static void sonarRecordDone();  // 0.7.5: each complete record: the near counts and the log values (below)

static void readSonar() {
  static uint32_t fields[4] = {};
  static uint8_t field = 0;
  static bool digits = false;
  static bool bad = false;
  // Bound each pass so a busy or noisy UART cannot hold up vehicle control.
  for (unsigned budget = 256; budget && Serial0.available(); --budget) {
    const int c = Serial0.read();
    if (c < 0) break;
    if (c == '\n') {
      if (!bad && field == 3 && digits) {
        for (unsigned i = 0; i < 4; ++i) sonar_ranges[i] = fields[i];
        sonar_pending = true;
        sonarRecordDone();
      }
      for (unsigned i = 0; i < 4; ++i) fields[i] = 0;
      field = 0;
      digits = false;
      bad = false;
    } else if (c == '\r') {
      // Ignore the CR in the Nano's Serial.println() CRLF terminator.
    } else if (!bad) {
      if (c >= '0' && c <= '9') {
        const uint32_t digit = c - '0';
        if (fields[field] > (UINT32_MAX - digit) / 10) bad = true;
        else { fields[field] = fields[field] * 10 + digit; digits = true; }
      } else if (c == ',' && digits && field < 3) {
        ++field;
        digits = false;
      } else {
        bad = true;  // Discard malformed records through their next newline.
      }
    }
  }
}

// ===== Sonar behaviour (0.7.5; settings rover_sonar_mm, rover_sonar_count; lib/rover_autopilot sonarCheck) =====
// Per sonar: the number of complete records in a row with a reading under the sonar trigger distance. A reading of 0
// (no echo) or at or over the distance sets the count to 0. The autopilot gets bit i of rover_sonar_near when sonar i+1
// has the set count, and only while the stream is fresh. No Nano (every bench board): no record, the mask stays 0, no
// event, and the autopilot runs as before.
// SONAR_STALE_MS: no complete record for this long: the rover ignores the sonar (the counts start again from 0) and
// logs one event; the next record logs one event again. 500 ms: the Nano record rate is not documented (commit
// 89b5e40 gives none; the planning estimate is 10 to 20 records a second), so 500 ms is 5 to 10 missed records, longer
// than any gap of a running Nano, and at the roam speed (0.5 m/s) 0.25 m of drive, under the 0.4 m trigger distance.
static const uint32_t SONAR_STALE_MS = 500;
static uint8_t sonar_count[4] = {};
static uint32_t sonar_record_ms = 0;
static bool sonar_have = false;    // a record came since the start
static bool sonar_stale = true;    // no fresh record: the sonar is ignored
uint8_t rover_sonar_near = 0;      // bit i: sonar i+1 near for the set count (roverStep: rover::Inputs::sonarNear)

static void sonarRecordDone() {
  const uint32_t trig = g_deviceConfig.rover_sonar_mm > 0 ? (uint32_t)g_deviceConfig.rover_sonar_mm : 0;
  for (unsigned i = 0; i < 4; ++i) {
    const uint32_t v = sonar_ranges[i];
    sonar_count[i] = (v != 0 && v < trig) ? (uint8_t)(sonar_count[i] < 255 ? sonar_count[i] + 1 : 255) : 0;
  }
  sonar_record_ms = millis();
  sonar_have = true;
  tlm_set(TLM_sonar1_mm, (float)(sonar_ranges[0] < 65534u ? sonar_ranges[0] : 65534u));
  tlm_set(TLM_sonar2_mm, (float)(sonar_ranges[1] < 65534u ? sonar_ranges[1] : 65534u));
  tlm_set(TLM_sonar3_mm, (float)(sonar_ranges[2] < 65534u ? sonar_ranges[2] : 65534u));
  tlm_set(TLM_sonar4_mm, (float)(sonar_ranges[3] < 65534u ? sonar_ranges[3] : 65534u));
}

// Each loop pass, after readSonar(): the stale rule, its events, and the near mask.
static void sonarService() {
  if (!sonar_have) return;
  const bool stale = (uint32_t)(millis() - sonar_record_ms) > SONAR_STALE_MS;
  if (stale != sonar_stale) {
    sonar_stale = stale;
    if (stale) for (unsigned i = 0; i < 4; ++i) sonar_count[i] = 0;
    char t[64];
    if (stale) snprintf(t, sizeof t, "Nano stream: no record for %lu ms, sonar ignored", (unsigned long)SONAR_STALE_MS);
    else snprintf(t, sizeof t, "Nano stream: records arrive");
    sonarEvent(0, stale ? 0 : 1, t);
  }
  uint8_t near = 0;
  const uint8_t need = g_deviceConfig.rover_sonar_count ? g_deviceConfig.rover_sonar_count : 1;
  if (!sonar_stale && g_deviceConfig.rover_sonar_mm > 0)
    for (unsigned i = 0; i < 4; ++i) if (sonar_count[i] >= need) near |= (uint8_t)(1u << i);
  rover_sonar_near = near;
}

// ===== Rover autopilot values =====

// Rover autopilot (lib/rover_autopilot): roam inside the perimeter, return to the wire after the return distance
// or below the return voltage, join it in the goal direction (rover_goal_cw), follow it to the dock with the wire
// follower below, charge, depart. Runs in drive mode 2 (rover_mode rover or follow only); rover_mode position follower: mode 2 is the plain wire
// follower.
rover::Autopilot roverAp;
bool rover_enabled = false;
double rover_seek_vbatt = 0.0;      // return battery voltage (0 = off)
uint8_t rover_entry_btx = 0;
uint8_t rover_exit_btx = 0;
bool rover_active = false;          // mode 2 with the autopilot driving
bool rover_need_charge = false;     // latched until the next charge ends
float rover_driven_m = 0.0f;        // distance driven since the last charge
uint32_t rover_last_packet_us = 0;
bool rover_was_docked = false;
bool rover_charge_ran = false;       // charge step 3 seen in this docking
float rover_vbatt_filt = 0.0f;
float rover_btx_mm[2] = {9999.0f, 9999.0f};  // beacon radius per channel, filtered as BTx1Signal, not clamped
rover::Output rover_out;
void roverStep();  // one autopilot step per STM32 packet; sets throttle and steering_output
float roverFollowSign();  // rover mode: the sign that the follower applies to x (Rx quadrant state)

// ===== Settings values (config.json fills them at start and after each change) =====

// Runtime values are populated from USB config/NVS during setup.
uint8_t vehicleID; // Used for ESP-NOW Telemetry. Set to the same value on receiving ESP32
uint8_t rx_channel; // Set this to match the channel on the Trail Tracer Tx. Allows for multiple tracks to operate in close proximity
int CRUISE_THROTTLE; // The vehicle will drive at this throttle level when in reveiverless mode (press button on Rx to enter receiverless mode)
int CREEP_THROTTLE; // Throttle level used when triggered by BTx Channel 1
int PWM_OUT_1_DEFAULT; // Center steering PWM value (typically 1500)
int PWM_OUT_1_MIN; // Minimum steering PWM value (typically 1000)
int PWM_OUT_1_MAX; // Maximum steering PWM value (typically 2000)
int PWM_OUT_2_DEFAULT; // Default throttle PWM value (typically 1500)
int PWM_OUT_2_MIN; // Minimum throttle PWM value (typically 1000)
int PWM_OUT_2_MAX; // Maximum throttle PWM value (typically 2000)
int PWM_OUT_3_DEFAULT; // Default PWM value for output 3 (typically 1500)
int PWM_OUT_3_MIN; // Minimum PWM value for output 3  (typically 1000)
int PWM_OUT_3_MAX; // Maximum PWM value for output 3 (typically 2000)
int PWM_OUT_4_DEFAULT;
int PWM_OUT_4_MIN;
int PWM_OUT_4_MAX;
// Drive output (0.5.69, config.json drive_output): 0 auto, 1 servos, 2 motor controllers over CAN; the servo
// output channel of each function (steer_output, throttle_output). Outputs with no function hold 1500 us.
uint8_t drive_output = 0, steer_output = 1, throttle_output = 2;
void servoIdle();  // every servo output to its function's centre (1500 us with no function)
double kp; // Steering controller proportional gain (higher number = more aggressive steering)
double ki; // Steering controller integral gain (usually set to 0 or a small value, helps eliminate steady-state error but can cause instability if too high)
double kd; // Steering controller derivative gain (usually never needed)
int pid_direction; // PID direction (0 = normal, 1 = reversed)
bool enable_sensor_2; // Enable second sensor. Save and Reboot after changing
StuckVehicle stuckDetector(&stmPacket.items.s1_y, 15, 18000, 2000);  // Distance threshold (mm) and time threshold (milliseconds). For 1/10 scale vehicles start with 15mm and 15000ms. For 1/18 vehicles start with 10mm and 15000ms.
uint8_t BTxFollowChannel; // If set to 0, BTx signals control creep throttle and sine swerve disable. For BTx follow functionality, set to the value of the BTx channel on the vehicle.
double BTxAccelRate; // How fast/slow the vehicle accelerates for BTx follow
int BTxCreepDuration; // How many seconds BTx creep lasts for
uint8_t SineSwerveAmplitude; // Setting this to 0 disables sine swerve. Non zero values specify the size of the sine swerve in millimeters.
uint8_t SineSwerveFrequency; // frequency of the sine swerve in seconds
double CHARGE_VBATT_STOP; // Voltage to stop charging and resume driving. usually set this to 4V per cell
double CHARGE_MAX_CURRENT; // Maximum current to draw when charging, used to prevent drawing too much current from weak chargers or when battery is very low. Usually keep this set at 6A (maximum) for most setups
int    CHARGE_MAX_SECONDS; // Maximum charge time in seconds. Vehicle will resume driving after this time even if voltage/current thresholds have not been met. Usually set this to 2-4 minutes (120-240 seconds)

// ===== Motor output: tuning and state =====
#define ODRIVE_BUZZER_PERIOD_MS 2000
#define ODRIVE_BUZZER_PULSE_MS 100

// Fresh calculated drive outputs and valid RC inputs enable ODrive control.
bool odrive_rc_active = false;
#define ODRIVE_RC_UPDATE_MS 12  // Example control period.
#define ODRIVE_RC_SIGNAL_TIMEOUT_MS 250
uint32_t odrive_rc_update_millis = 0;
// The gyro rate PID: its last step (micros), and a step of the 12 ms resend since the last packet step. The packet step
// after a resend step takes the time since that step, not the update period (at 20 ms the resend runs between packets).
uint32_t rate_pid_us = 0;
bool rate_resend_stepped = false;

// The period factor (lib/frame_pid/frame_period.h) of the update period in effect: computed again when
// stmFramePeriodS() changes (the Rx answer to CID 18; 10 ms until it answers).
double periodRatio() {
  static double period_s = -1.0, r = 1.0;
  const double p = stmFramePeriodS();
  if (p != period_s) { period_s = p; r = fperiod::ratio(p); }
  return r;
}
#define ODRIVE_OUTPUT_TIMEOUT_MS 250
#define ODRIVE_DRIVE_SCALE_RPS 1.5f // Full throttle [rev/s].
#define ODRIVE_TURN_SCALE_RPS  1.5f // Full steering [rev/s]; combined maximum is 2.3 rev/s.
float odrive_drive_command = 0.0f;
float odrive_steer_command = 0.0f;
bool odrive_output_enabled = false;
bool odrive_arm_switch_was_high = false;
uint32_t odrive_output_millis = 0;

// ===== Button =====
// Button configuration
#define BUTTON_DEBOUNCE_PERIODS 3  // 0.03s
#define BUTTON_CLICK_MAX_LENGTH 40  // 0.4s
#define BUTTON_CAL_MIN_PRESS 200  // 2s
#define BUTTON_CAL_MAX_PRESS 400  // 4s
#define BUTTON_6S_PRESS 600  // 6s

// ===== Charging =====
#define CHARGE_AUTO_ENABLE true  // bool to enable/disable automatic charging
int8_t charge_status = 0;  // charge status flag for state machine
uint32_t charge_start_millis = 0;
bool lapFlag = false;

// ===== Drive mode and vehicle status =====
int vehicleStatus = 0; //0=stuck, 1=driving, 2=charging - used for telemetry
int lastVehicleStatus = 1; // Track previous status
uint32_t stuckStartTime = 0; // Track when vehicle became stuck

uint16_t throttle_micros = 1500;
uint32_t drive_mode_timer = 0;
int8_t drive_mode_status = 0;
uint32_t led_blink_last_ms = 0;
bool led_blink_state = false;
uint16_t longpress_flash_cycles = 0;
uint32_t config_saved_led_until_ms = 0;  // When non-zero, hold LED solid-on until this timestamp
bool case2_use_cruise = true;  // Toggle output in case 2
bool case2_aux_entry_is_manual = false;  // Aux position when entering case 2
bool case2_aux_changed = false;  // Aux changed since entering case 2
int case2_throttle_current = PWM_OUT_2_DEFAULT;  // Ramped throttle in case 2
bool btx_throttle_override_active = false;  // Freeze case 1/2 throttle accumulation while BTx logic overrides output
uint32_t btx_user_override_until_ms = 0;  // While active, user input takes priority over BTx creep/follow

int throttle = PWM_OUT_2_DEFAULT;

// ===== BTx and sine swerve =====
int BTxThreshold = 200; // Threshold for BTx follow proximity. Initial Placeholder
float BTx1Signal = BTxThreshold;
float BTx2Signal = BTxThreshold;
float BTx1LastDistance = BTxThreshold;
float BTx2LastDistance = BTxThreshold;
float BTxFilterAlpha = 0.4; //0.456 = 5Hz low pass filter. Smaller value = more filtering
uint32_t sine_swerve_disabled_until_ms = 0;
double sine_wave_blend = 0.0;
bool SineSwerveEnabled = true;
const double SineSwerveSnapRate = 0.01; // per-loop gain when sine swerve resumes
bool sine_wave_force_disabled = false;

// ===== Steering PID =====
double target_x_pos = 0;  // target x distance from wire
double live_position = 0;

// 0.6.36: sensor 2 acts on behaviour only with both "Sensor 2" (enable_sensor_2: the data) and "Use sensor 2 for control"
// (use_sensor_2) on. Else every decision uses sensor 1, and sensor 2 is data for the log and the app only.
bool sensor2Control(){ return enable_sensor_2 && g_deviceConfig.use_sensor_2; }

// 0.6.21: world facts from the polarity (Rx STM32 2.2.0+), at the sensor nearest the wire (0.6.36: sensor 1 only, unless
// sensor2Control()), for the followed Tx channel
// (rx_channel). world_inside: +1 inside the loop, -1 outside. world_heading: +1 clockwise round the loop, -1
// counterclockwise (seen from above). 0: not known. With D = the current direction setting of the channel (+1
// clockwise, -1 counterclockwise): inside = polarity x sign(x) x D; heading = polarity x sign(y) x D. Each fact
// counts only away from its blind line (|x|, or |y|, at least 25 % of the radius: above the wire x is near 0, square
// to the wire y is near 0); else, and while the polarity is 0, the last value holds. No sensor in range for 2 s,
// no fresh polarity item for 1 s, or no current direction set: both 0.
// 0.6.36: world_inside comes from the inside filter below, not from the held polarity.
int8_t world_inside = 0, world_heading = 0;
float world_inside_p = 0.5f;  // the inside filter's belief that the sensor is inside the loop, 0..1

// 0.6.36: the inside filter. The sensor can change sides only by a crossing, and a crossing needs the sensor near the
// wire. A rover log (2026-10-24) had "outside" polarity bursts of 0.1-0.3 s at 1 m from the wire: the join then started
// the follow 1 m from the wire, and starts faulted. The filter mixes every raw sign read (Rx STM32 extension items 9
// sign and 10 sign spread, about 5 per s) into a belief, by Bayes' rule, in 2 steps per new read:
// 1. Crossing chance since the last read: INSIDE_FLIP_MAX / (1 + (rNear / rCross)^12) + INSIDE_FLIP_MIN. rNear is the
//    least radius seen, grown since then at the roam speed (the top speed): the nearest the sensor can have been to
//    the wire. rCross is the farthest radius at the first usable read after a crossing: worst case the deepest wire
//    (height setting + wire_max_depth_mm) and a square approach. The sensor is then past the wire by the blind line
//    (|x| < 25 % of the radius: 0.26 x the depth), plus one read at the top speed, plus 0.1 m for field distortion.
//    A soft edge, not a gate: at 1.25 x rCross the chance is 1/8 of its value at rCross, so a radius error moves the
//    edge but never stops a crossing.
// 2. The read: a sign at its spread, weighted by the wrong-read rate at that spread (Rx STM32 sign reads against the
//    true side in 2 rover logs, 2026-10-24: 0 in 1345 under 25 deg, 0.8 % at 25-35, 2.3 % at 35-45, 13 % at 45-55).
// The value changes at a belief of 95 % (one read under 25 deg near the wire, or 2-3 far from it). No sensor in
// range for 2 s, or no read for 1 s: the value is unknown (0), but the belief stays, because the side cannot change
// far from the wire. While no sensor is in range, the nearest radius falls at the roam speed from the last radius
// seen: the sensor can close on the wire unseen. A change of the current direction setting starts the belief again
// at 50 %.
const float INSIDE_READ_S = 0.2f;       // s, time between two Rx sign reads (rover logs: about 5 per s)
const float INSIDE_DISTORT = 0.1f;      // m, margin on rCross for field distortion near the wire
const float INSIDE_FLIP_MAX = 0.5f;     // crossing chance per read right over the wire: either side is as likely, so
                                        // the reads alone decide (one read under 25 deg). Log replay: 0 false side
                                        // changes far from the wire, median delay 0 s (0.05: 0.1 s, 1 crossing lost)
const float INSIDE_FLIP_MIN = 1e-6f;    // crossing chance per read far from the wire: 3 reads under 25 deg still
                                        // change the value (a crossing during a signal loss)
const float INSIDE_SURE = 0.95f;        // belief at which the value changes
float insideReadErr(float spread_deg){
  if (spread_deg < 25) return 0.002f;   // 0 wrong in 1345: the rate is at most this
  if (spread_deg < 35) return 0.008f;
  if (spread_deg < 45) return 0.023f;
  if (spread_deg < 55) return 0.126f;
  if (spread_deg < 70) return 0.3f;     // estimate between the measured 13 % and no information
  return 0.5f;                          // no information
}
void insideFilter(int s, int c, int D, float rs, bool seen, bool gone, uint32_t now){
  static float r_near = 99, r_last = 99;
  static int d_last = 0;
  static uint32_t last_ms = 0;
  static uint16_t last_t10[2] = {};
  const float v = g_deviceConfig.rover_roam_speed_mps;
  const float dt = last_ms ? (now - last_ms) * 0.001f : 0;
  last_ms = now;
  r_last = seen ? rs : fmaxf(0, r_last - v * dt);
  r_near = fminf(r_near + v * dt, r_last);
  if (D != d_last) { d_last = D; world_inside_p = 0.5f; world_inside = 0; }
  const uint16_t now10 = (uint16_t)(now / 10u);
  const unsigned sl = (unsigned)s * 8u + (unsigned)c;
  const uint16_t t = stmDec.ext_t10[8][sl], tp = stmDec.ext_t10[9][sl];
  const bool fresh = t && tp && (uint16_t)(now10 - t) <= 100u;
  if (D == 0 || !fresh || gone) { world_inside = 0; return; }
  if (t == last_t10[s]) return;  // no new read
  last_t10[s] = t;
  const int sign = stmDec.ext_val[8][sl];
  const float x = s == 0 ? stmPacket.items.s1_x : stmPacket.items.s2_x;
  float p = world_inside_p;
  if (seen && sign != 0 && fabsf(x) >= 0.25f * rs) {  // over the wire x says nothing about the side
    const float h = (float)(g_deviceConfig.rover_sensor_height_mm + g_deviceConfig.wire_max_depth_mm) * 0.001f;
    const float r_cross = hypotf(h, 0.26f * h + v * INSIDE_READ_S + INSIDE_DISTORT);
    const float q = r_near / r_cross, q2 = q * q, q4 = q2 * q2;
    const float flip = INSIDE_FLIP_MAX / (1 + q4 * q4 * q4) + INSIDE_FLIP_MIN;
    p = p * (1 - flip) + (1 - p) * flip;
    const float e = insideReadErr(stmDec.ext_val[9][sl] * 0.1f);
    const bool says_in = sign * (x > 0 ? 1 : -1) * D > 0;
    const float pin = p * (says_in ? 1 - e : e), pout = (1 - p) * (says_in ? e : 1 - e);
    p = fminf(fmaxf(pin / (pin + pout), 1e-6f), 1 - 1e-6f);
    world_inside_p = p;
  }
  if (p >= INSIDE_SURE) world_inside = 1;
  else if (p <= 1 - INSIDE_SURE) world_inside = -1;
}

void worldFacts(){
  const int c = (int)rx_channel - 1;
  const uint8_t dirs[5] = {g_deviceConfig.tx_dir_ch1, g_deviceConfig.tx_dir_ch2, g_deviceConfig.tx_dir_ch3,
                           g_deviceConfig.tx_dir_ch4, g_deviceConfig.tx_dir_ch5};
  const int D = (c >= 0 && c < 5) ? (dirs[c] == 1 ? 1 : dirs[c] == 2 ? -1 : 0) : 0;
  const uint32_t now = millis();
  const bool fresh = stmDec.pol_t10 && (uint16_t)((uint16_t)(now / 10u) - stmDec.pol_t10) <= 100u;
  static uint32_t seen_ms = 0;
  static int s_last = 0;
  int s = -1;
  float rs = 0;
  for (int k = 0; k < (sensor2Control() ? 2 : 1); k++) {
    if (stmDec.lost[k]) continue;
    const float r = k == 0 ? stmPacket.items.s1_radius : stmPacket.items.s2_radius;
    if (s < 0 || r < rs) { s = k; rs = r; }
  }
  if (s >= 0) { seen_ms = now; s_last = s; }
  const bool gone = now - seen_ms > 2000u;
  insideFilter(s_last, c < 0 ? 0 : c, D, rs, s >= 0, gone, now);
  if (D == 0 || !fresh || gone) { world_heading = 0; return; }
  if (s < 0) return;
  const int p = stmDec.pol[s][c];
  if (p == 0) return;
  const float y = s == 0 ? stmPacket.items.s1_y : stmPacket.items.s2_y;
  if (fabsf(y) >= 0.25f * rs) world_heading = (int8_t)(p * (y > 0 ? 1 : -1) * D);
}
// PID Initialization
double pid_output = 0.0;
double steering_output = PWM_OUT_1_DEFAULT;
PID steeringPID(&live_position, &pid_output, &target_x_pos, kp, ki, kd, pid_direction);

// Inner loop accepts physical yaw-rate commands. A future angle controller can
// convert heading error (degrees) to a bounded deg/s request through this API.
double steering_rate_input_dps = 0.0;
double steering_rate_target_dps = 0.0;
double steering_rate_output = 0.0;
double steering_rate_max_dps = 90.0;
PID steeringRatePID(&steering_rate_input_dps, &steering_rate_output,
                    &steering_rate_target_dps, 0.02, 0.01, 0.0, DIRECT);

void steeringRateSetTargetDps(double rate_dps) {
  steering_rate_target_dps = isfinite(rate_dps)
    ? constrain(rate_dps, -steering_rate_max_dps, steering_rate_max_dps) : 0.0;
}

// Both PIDs run once per STM32 frame with dt = the frame period (lib/frame_pid: PID_v1's algorithm; the
// PID_v1 Compute() above ran on its own millis() timer and scaled ki and kd by a fixed sample time). The
// PID objects keep the gains, direction and mode as set (settings, config.json); each step copies them.
framepid::Pid steeringFramePid, steeringRateFramePid;

bool framePidCompute(PID& pid, framepid::Pid& fp, double setpoint, double input, double* output,
                     double out_min, double out_max, double dt_s) {
  if (pid.GetMode() != AUTOMATIC) { fp.stop(); return false; }
  fp.setTunings(pid.GetKp(), pid.GetKi(), pid.GetKd());
  fp.setReverse(pid.GetDirection() == REVERSE);
  fp.setOutputLimits(out_min, out_max);
  if (!fp.started()) fp.start(input, *output);  // as PID_v1 SetMode(AUTOMATIC): no bump
  *output = fp.step(setpoint, input, dt_s);
  return true;
}

void steeringRateReset() {
  steeringRatePID.SetMode(MANUAL);
  steeringRateFramePid.stop();
  steering_rate_output = 0.0; // Clear integral on the next MANUAL -> AUTOMATIC transition.
}

bool steeringRateGyroValid() {
  return gyro_ready && millis() - gyro_sample_ms <= 50 && isfinite(mpu.getGyroZ());
}

// ===== Steering PID low-pass filter =====
bool low_pass_enable = true;  // true = low pass active, false disables low-pass
float low_pass_cutoff = 5;  // Frequency of a 2nd order low-pass filter.  Smaller number = more filtering
float low_pass_alpha = 0.0;  // filter coefficient that's calculated from the cutoff value
float low_pass_beta = 0.0;  // filter coefficient that's calculated from the cutoff value
bool low_pass_compute = false;  // bool to only compute when PID loop updates
bool low_pass_first_cycle = true;
float low_pass_last_val_1 = 0.0;
float low_pass_last_val_2 = 0.0;

// ===== Debug print and button counters =====
int32_t bttn_counter = 0;  // button counter for debounce logic
int32_t bttn_state = 0;  // debounced state of the user button
int32_t bttn_click_counter = 0;  // button counter for "click" logic
int32_t bttn_long_counter = 0;  // button counter for "long" press logic
int32_t bttn_3s_armed = 0;  // calibration press window armed
int32_t bttn_6s_state = 0;  // state of a 6s press

// ===== Functions further down in this file (C++ needs each name before its first use) =====
void applyDeviceConfig(const DeviceConfig& cfg); // Apply cfg values to firmware globals

void buttonLogic();  // handle button events
void buttonClickEvent();  // Called on button release if the press time is considered a "click"
void button3sPressEvent();  // Called on button release after a 2s to 4s press
void button6sPressEvent();  // Called on a 6-second button press

void chargeControl();
uint16_t driveMode();
int BTxFollow(int base_throttle, double &btx_throttle, bool &btx_throttle_init, bool &btx_follow_active, float btx_signal);

// ===== Coil strength (the debug print in loop()) =====
// Signal strength of one coil: DFT magnitude in counts (before the STM32 gain calibration).
// sensor: 0 or 1. coil: 0 or 1.
float coilStrength(const StmTxPacket& p, uint8_t sensor, uint8_t coil) {
  const float* c = &p.items.coil_data[sensor][coil * 2]; // re, im
  return hypotf(c[0], c[1]);
}


// ===== Motor output: servos or motor controllers on the CAN bus =====
bool odrive_rcInputsValid() {
  // Case 2 follows the wire autonomously, without an RC transmitter.
  // Fresh calculated outputs are still required by odrive_outputUpdate().
  if (drive_mode_status == 2) return true;
  return pwm_read_rmt_valid(0, ODRIVE_RC_SIGNAL_TIMEOUT_MS) &&
         pwm_read_rmt_valid(1, ODRIVE_RC_SIGNAL_TIMEOUT_MS);
}

// RC block bits: 1=axis error, 4=wire overload,
// 8=STM32 supply fault, 16=overtemperature.
uint32_t odrive_rc_fault_bits = 0;
bool odrive_rcFault() {
  uint32_t faults = 0;
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    if (odrive_axis_errors[node]) faults |= 1;
  }
  if (WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_OVERLOAD)) faults |= 4;
  if (WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_VDD_FAULT)) faults |= 8;
  if (stmPacket.items.temperature > 100) faults |= 16;
  odrive_rc_fault_bits = faults;
  return faults != 0;
}

// Map the final servo-equivalent output to -1..1 using its configured center
// and endpoints. Do not apply RC deadband again to calculated PID outputs.
float odrive_normalizeOutput(double pulse_us, int center, int minimum, int maximum) {
  const double offset = pulse_us - center;
  const int span = offset >= 0.0 ? maximum - center : center - minimum;
  if (span <= 0) return 0.0f;
  return constrain((float)(offset / span), -1.0f, 1.0f);
}

void odrive_stopRc() {
  odrive_rc_active = false;
  steeringRateReset();
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    odrive_setInputVel(node, 0.0f);
    odrive_setAxisState(node, ODRIVE_AXIS_STATE_IDLE);
  }
  /* usbOut.println("ODrive RC: STOP REQUESTED (RC signal or safety fault)"); */;
}

void odrive_startRc() {
  if (digitalRead(MOTOR_ARM_SWITCH_PIN) == LOW ||
      !can_init_ok || !odrive_rcInputsValid() || odrive_rcFault()) return;
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    if (digitalRead(MOTOR_ARM_SWITCH_PIN) == LOW) {
      odrive_stopRc();
      return;
    }
    odrive_clearErrors(node);
    delay(2);
    odrive_setControllerMode(node, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    delay(2);
    // Preserve the example's per-node current limits.
    odrive_setLimits(node, ODRIVE_VEL_LIMIT_RPS, (node == 2 || node == 3) ? 6.0f : 3.0f);
    delay(2);
    odrive_setInputVel(node, 0.0f);
    if (digitalRead(MOTOR_ARM_SWITCH_PIN) == LOW) {
      odrive_stopRc();
      return;
    }
    odrive_setAxisState(node, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    delay(5);
  }
  odrive_rc_update_millis = millis();
  odrive_rc_active = true;
  /* usbOut.println("ODrive RC: ARM REQUESTED (nodes 0-5); PWM1 steer / PWM2 throttle"); */;
}

// Called where throttle/steering servos used to be commanded.
void odrive_setDriveOutput(double throttle_us, double steering_us, bool enabled) {
  odrive_output_millis = millis();
  odrive_output_enabled = enabled;
  odrive_drive_command = enabled ? odrive_normalizeOutput(throttle_us,
    PWM_OUT_2_DEFAULT, PWM_OUT_2_MIN, PWM_OUT_2_MAX) : 0.0f;
  if (!enabled) odrive_steer_command = 0.0f;
  if (enabled) {
    // All modes share this conversion after wire PID / manual arbitration.
    steeringRateSetTargetDps(odrive_normalizeOutput(steering_us,
      PWM_OUT_1_DEFAULT, PWM_OUT_1_MIN, PWM_OUT_1_MAX) * steering_rate_max_dps);
  } else {
    steeringRateSetTargetDps(0.0);
    steeringRateReset();
  }
  if (!enabled && odrive_rc_active) odrive_stopRc();
}

// frame_now: called with a new STM32 frame (the CAN commands go out with every frame). loop() also calls it
// every pass: with no frame for ODRIVE_RC_UPDATE_MS it sends the last commands again (ODrive watchdog floor).
void odrive_outputUpdate(bool frame_now) {
  imuTake(1);  // the newest IMU sample (test build IMU_I2C_NB; empty otherwise)
  const uint32_t now = millis();
  const bool motor_idle_requested = digitalRead(MOTOR_ARM_SWITCH_PIN) == LOW;
  const bool arm_switch_high = !motor_idle_requested;
  if (arm_switch_high && !odrive_arm_switch_was_high && can_init_ok) {
    // Clear faults before rc_ready checks the reported axis errors. Otherwise
    // a latched fault prevents odrive_startRc() from ever reaching Clear_Errors.
    // Keep the reported faults until CAN confirms the clear, including for the
    // buzzer; an unsuccessful clear must not silence a real fault.
    for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
      odrive_clearErrors(node);
      delay(2);
    }
  }
  odrive_arm_switch_was_high = arm_switch_high;
  if (motor_idle_requested && odrive_rc_active) odrive_stopRc();
  if (!frame_now && now - odrive_rc_update_millis < ODRIVE_RC_UPDATE_MS) return;
  // dt of the gyro rate PID step: the true time since its last step. A packet step with no resend step since the last
  // packet step, or any packet step at period factor 1: the update period (as before; at 10 ms a late packet can let
  // the resend run, and the packet step after it keeps the old dt). The resend at period factor 1 (it runs only when
  // packets stop): the time since the last pass, in ms (as before); else in us.
  const uint32_t now_us = micros();
  const double dt_since = constrain((now_us - rate_pid_us) * 1e-6, 0.001, 0.1);
  const double dt_s = frame_now ? ((rate_resend_stepped && periodRatio() != 1.0) ? dt_since : stmFramePeriodS())
                                : (periodRatio() == 1.0 ? constrain((now - odrive_rc_update_millis) / 1000.0, 0.001, 0.1) : dt_since);
  if (frame_now) rate_resend_stepped = false;
  odrive_rc_update_millis = now;
  const bool rc_ready = !motor_idle_requested && can_init_ok && odrive_output_enabled &&
    now - odrive_output_millis <= ODRIVE_OUTPUT_TIMEOUT_MS &&
    odrive_rcInputsValid() && !odrive_rcFault() && steeringRateGyroValid();
  if (odrive_rc_active && !rc_ready) {
    odrive_stopRc();
  } else if (!odrive_rc_active && rc_ready) {
    odrive_startRc();
    serviceMpu(); // Arming takes time; refresh feedback before the first rate calculation.
    if (odrive_rc_active && !steeringRateGyroValid()) odrive_stopRc();
  }
  float even_rps = 0.0f, odd_rps = 0.0f;
  if (odrive_rc_active) {
    steering_rate_input_dps = mpu.getGyroZ();
    steeringRatePID.SetMode(AUTOMATIC);
    framePidCompute(steeringRatePID, steeringRateFramePid, steering_rate_target_dps, steering_rate_input_dps,
                    &steering_rate_output, -1.0, 1.0, dt_s);  // was steeringRatePID.Compute()
    rate_pid_us = now_us;
    if (!frame_now) rate_resend_stepped = true;
    // Keep the existing motor-mixer polarity; PID direction is configured separately.
    odrive_steer_command = -(float)steering_rate_output;
    // Skid steering: opposite motor mounting directions on the two sides.
    even_rps = constrain(-(ODRIVE_DRIVE_SCALE_RPS * odrive_drive_command + ODRIVE_TURN_SCALE_RPS * odrive_steer_command),
      -ODRIVE_VEL_LIMIT_RPS, ODRIVE_VEL_LIMIT_RPS);
    odd_rps = constrain(ODRIVE_DRIVE_SCALE_RPS * odrive_drive_command - ODRIVE_TURN_SCALE_RPS * odrive_steer_command,
      -ODRIVE_VEL_LIMIT_RPS, ODRIVE_VEL_LIMIT_RPS);
  }
  if (!can_init_ok) return;
  // Repeat zero and idle while disarmed, so single-shot arbitration losses
  // cannot leave a previously commanded motor running after signal loss.
  rtprof::canStart();  // real-time path timing (lib/rt_prof)
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    odrive_setInputVel(node, (node & 1) ? odd_rps : even_rps);
    if (!odrive_rc_active) odrive_setAxisState(node, ODRIVE_AXIS_STATE_IDLE);
  }
  rtprof::canEnd();
}

// Nonblocking alarm; start immediately on a fault and repeat every two seconds.
// Drive output (0.5.69). Servos: the steering and throttle pulses on their outputs, as V1.0 does; stopped:
// both centres. Motor controllers over CAN: as before (odrive_setDriveOutput). drive_output auto: CAN once a
// motor controller heartbeat arrived in this boot, servos until then.
bool driveUsesCan(){
  if (drive_output == 1) return false;
  if (drive_output == 2) return true;
  static bool seen = false;
  for (int n = 0; n < ODRIVE_NODE_COUNT && !seen; n++) seen = odrive_heartbeat_received[n];
  return seen;
}
void servoIdle(){
  for (uint8_t ch = 1; ch <= 4; ch++)
    writePwmMicros(ch, ch == steer_output ? PWM_OUT_1_DEFAULT : ch == throttle_output ? PWM_OUT_2_DEFAULT : 1500);
}
void driveOutput(double throttle_us, double steering_us, bool enabled){
  if (driveUsesCan()) {
    odrive_setDriveOutput(throttle_us, steering_us, enabled);
    servoIdle();
    return;
  }
  odrive_setDriveOutput(PWM_OUT_2_DEFAULT, PWM_OUT_1_DEFAULT, false);  // motor controllers idle
  servoIdle();
  if (enabled) {
    writePwmMicros(steer_output, (uint32_t)lround(steering_us));
    writePwmMicros(throttle_output, (uint32_t)lround(throttle_us));
  }
}

void odrive_updateBuzzer() {
  static bool fault_was_present = false;
  static uint32_t pulse_started_ms = 0;
  bool fault_present = false;
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    if (odrive_axis_errors[node] || odrive_active_errors[node] || odrive_disarm_reasons[node]) {
      fault_present = true;
      break;
    }
  }
  const uint32_t now = millis();
  if (fault_present && (!fault_was_present || now - pulse_started_ms >= ODRIVE_BUZZER_PERIOD_MS)) {
    pulse_started_ms = now;
  }
  digitalWrite(BUZZER_PIN,
    fault_present && now - pulse_started_ms < ODRIVE_BUZZER_PULSE_MS ? HIGH : LOW);
  fault_was_present = fault_present;
}


// ===== setup(): runs once at power-on =====
void setup() {
#ifdef DIAG_QIO_BOOT
  extern void qioBoot();
  qioBoot();  // diag_s2q: the flash cache in QIO mode from here (src/diag_qio.cpp)
#endif
  // Board start: the Rx STM32 serial line, the crash counter, the settings from the USB drive.
  // A board started as the PC monitor does only that job: no vehicle code starts.
  if (platformBegin()) return;

  // The settings to the vehicle values (applyDeviceConfig, below)
  applyDeviceConfig(g_deviceConfig);

  // Board: radio mode, web pages, telemetry log, USB serial, updates, Rx STM32 serial port
  platformStart();
#ifdef DIAG_QIO_BOOT
  extern void qioReport();
  qioReport();  // the QIO boot result on USB (release: a log event)
#endif

  // Configure pins
  pinMode(nLED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  eraseWindowOpen();  // the settings erase gesture: a press in the first 3 s after power-on
  pinMode(MOTOR_ARM_SWITCH_PIN, INPUT_PULLUP);
  odrive_arm_switch_was_high = digitalRead(MOTOR_ARM_SWITCH_PIN) == HIGH;
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(PWM_IN_1, INPUT);
  pinMode(PWM_IN_2, INPUT);
  pinMode(PWM_IN_3, INPUT);

  pinMode(PWM_OUT_1, OUTPUT);
  pinMode(PWM_OUT_2, OUTPUT);
  pinMode(PWM_OUT_3, OUTPUT);
  pinMode(PWM_OUT_4, OUTPUT);

  // GPIO39/40 are reserved for MPU6050 I2C, not auxiliary inputs.

  digitalWrite(nLED, 1);  // turn the LED off

  // Servo PWM setup
  ledcSetup(1, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(2, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(3, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(4, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcAttachPin(PWM_OUT_1, 1);
  ledcAttachPin(PWM_OUT_2, 2);
  ledcAttachPin(PWM_OUT_3, 3);
  ledcAttachPin(PWM_OUT_4, 4);
  servoIdle();

  // MPU6050: the main-branch serviceMpu() (src/platform/imu.inc) owns it: init, calibration, one sample per
  // loop. The main-branch gyro yaw-rate steering needs it; with no MPU its lock keeps the motors off. The log
  // reads the same object (mpu_ok follows gyro_ready). Main branch f10cd31.

  if (!imu_halt) {
  // Install TWAI before allocating RMT and STM32 GPIO interrupts.
  canBegin();

  // Set up PWM inputs (in progress)
  rcInputsBegin();

  // PID setup
  steeringPID.SetOutputLimits(PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT, PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);  // Set the output contstraints
  steeringPID.SetSampleTime(10);  // Set the sample interval in milliseconds
  steeringPID.SetControllerDirection(pid_direction);  // Apply direction loaded from config
  steeringPID.SetMode(AUTOMATIC);  // Turn on the control loop

  // PID low pass setup
  // PID low pass: alpha from the frame period, in loop() (0.5.69; was a fixed 100 Hz here)

  // The Rx STM32 link: frame interrupt, start handshake
  rxLinkBegin();

  // Set up charge port
  espPacketFifo_addCmdFloat(6 | 0x80000000, CHARGE_VBATT_STOP);  // Set voltage stop
  espPacketFifo_addCmdFloat(7 | 0x80000000, CHARGE_MAX_CURRENT);  // Set overcurrent
  espPacketFifo_addCmdInt32(8 | 0x80000000, CHARGE_MAX_SECONDS);  // Set charge time limit, s (0.5.69: as it is)

  //RX channel setup
  espPacketFifo_addCmdInt32(1 | 0x80000000, (rx_channel - 1));  // write Rx channel
  espPacketFifo_addCmdInt32(2 | 0x80000000, enable_sensor_2 ? 0x00 : 0x80);  // Enable/disable sensor 2
  // No CID 3 save here (0.5.82): the channel is not in the Rx STM32 calibration block, and a save at each boot
  // was a repeating Rx STM32 flash write. Saves: the centre calibration (button) and the restore after an update.
  }  // !imu_halt

  // The radio of the settings: ESP-NOW or Wi-Fi
  radioStart();
  // Watchdogs on; the "SETUP ms=" line
  setupDone();
}

// ===== loop(): runs again and again =====
#ifdef LOOP_NO_RETURN
static void loopPass() {
#else
void loop() {
#endif
  // Board first: watchdog, USB, updates. During an update, the PC monitor job or a bench pass-through, the board
  // uses the whole pass and the vehicle code waits.
  if (platformLoopStart()) return;

  readSonar();
  sonarService();

  // Button, every 10 ms
  static uint32_t last_button_update = 0;
  if (millis() - last_button_update >= 10) {
    last_button_update = millis();
    buttonLogic();
  }

  // Motor controllers on the CAN bus
  odrive_recoverIfNeeded();
  odrive_pollAndPrintCan();
  odrive_updateBuzzer();
  odrive_outputUpdate(false);  // Send calculated outputs; stop if outputs or RC go stale.
  odrive_requestEncoderEstimates();

  // Settings: config.json changed on the USB drive
  if (settingsFileChanged()) {
    applyDeviceConfig(g_deviceConfig);
    config_saved_led_until_ms = millis() + 1500;  // Signal config saved to user via LED
  }
  settingsBadValues();

  // A new frame from the Rx STM32: the control step
  if (rxFrameReady()) {
#ifdef DIAG_CTRL_PROF
    mark(SEC_C_PRE);  // diag: the control step parts
#endif
    // update the double X position from the float in the latest STM32 packet
    live_position = stmPacket.items.s1_x;
    if (rover_active) live_position *= roverFollowSign();  // the Rx quadrant state, learned from steady follow
    // 0.6.21: steering frame "world": x signed by the heading round the loop against the goal direction
    // (rover_goal_cw). The goal way: x as it is. The wrong way: -x, so the steering turns the vehicle round. Heading not
    // known: x as it is (vehicle frame).
    worldFacts();
    if (g_deviceConfig.steer_frame == 1 && world_heading != 0)
      live_position *= (double)(world_heading * (g_deviceConfig.rover_goal_cw ? 1 : -1));
    uint32_t loop_now_ms = millis();

    // Filter and condition BTx signals
    BTx1Signal = constrain(stmPacket.items.s1_btx_radius[0], 10, (BTxThreshold + 2));
    BTx2Signal = constrain(stmPacket.items.s1_btx_radius[1], 10, (BTxThreshold + 2));
    // alpha per packet at 10 ms; the same time constant at any update period (frame_period.h)
    const float btx_a = fperiod::weight(BTxFilterAlpha, periodRatio());
    BTx1Signal = btx_a * BTx1Signal + (1 - btx_a) * BTx1LastDistance;
    BTx2Signal = btx_a * BTx2Signal + (1 - btx_a) * BTx2LastDistance;
    BTx1LastDistance = BTx1Signal;
    BTx2LastDistance = BTx2Signal;

    if(BTx2Signal < BTxThreshold && BTxFollowChannel == 0){
      sine_swerve_disabled_until_ms = loop_now_ms + 10000U;
    }

    // Reset target each loop so optional modifiers can add on cleanly
    target_x_pos = 0.0;

    // Sine wave steering component
    bool sine_wave_enabled = SineSwerveEnabled && (SineSwerveAmplitude != 0);
    if(sine_wave_enabled){
      bool sine_wave_allowed = (loop_now_ms >= sine_swerve_disabled_until_ms) && !sine_wave_force_disabled;
      double blend_target = sine_wave_allowed ? 1.0 : 0.0;
      const double snap = fperiod::step(SineSwerveSnapRate, periodRatio());  // per packet at 10 ms
      if(sine_wave_blend < blend_target){
        sine_wave_blend = fmin(blend_target, sine_wave_blend + snap);
      }else if(sine_wave_blend > blend_target){
        sine_wave_blend = fmax(blend_target, sine_wave_blend - snap);
      }

      double time_sec = loop_now_ms / 1000.0;
      double sine_offset = (SineSwerveAmplitude) * sin((2.0 * 3.14159 / SineSwerveFrequency) * time_sec);
      target_x_pos = sine_wave_blend * sine_offset;
    }else{
      sine_wave_blend = 0.0;
    }

#ifdef DIAG_CTRL_PROF
    mark(SEC_C_PID);
#endif
    // Steering control
    low_pass_compute = framePidCompute(steeringPID, steeringFramePid, target_x_pos, live_position, &pid_output,
        PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT, PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT, stmFramePeriodS());  // was steeringPID.Compute()

    // apply low pass to PID output (mostly to reduce jitter due to kd term)
    if(low_pass_compute && low_pass_enable){  // 0.5.69: steer_lpf_hz at the real frame period
      low_pass_alpha = expf(-6.2831853f * low_pass_cutoff * (float)stmFramePeriodS());
      low_pass_beta = 1 - low_pass_alpha;
    }
    if(low_pass_compute && low_pass_enable && (!low_pass_first_cycle)){
      pid_output = pid_output * low_pass_beta + low_pass_last_val_1 * low_pass_alpha;  // Compute first low-pass
      low_pass_last_val_1 = pid_output;  // save intermediate value
      pid_output = pid_output * low_pass_beta + low_pass_last_val_2 * low_pass_alpha;  // Compute second low-pass
      low_pass_last_val_2 = pid_output;  // save last value
    }
    if(low_pass_compute && low_pass_first_cycle && low_pass_enable){
      low_pass_last_val_1 = pid_output;
      low_pass_last_val_2 = pid_output;
      low_pass_first_cycle = false;
    }
    throttle = driveMode(); // get throttleMicros from drive mode logic

    if(charge_status > 0){
      stuckDetector.ResetStatus();  // ignore stuck detection when charging
    }


    //Btx Logic -----------------------------------------------------------------------
    // Creep Speed -------------------------------
    // Manual mode always gives RC throttle priority and clears BTx override state.
    bool btx_user_override_active = (drive_mode_status == 0 || millis() < btx_user_override_until_ms);

    if(BTxFollowChannel == 0){ //if BTx follow is turned off, use BTx 0 for creep speed
      int base_throttle = throttle;
      static double creep_throttle = PWM_OUT_2_DEFAULT;
      static bool creep_throttle_init = false;
      static uint32_t creep_until_ms = 0;
      static bool creep_returning = false;
      static bool creep_was_active = false;
      static int creep_return_base = PWM_OUT_2_DEFAULT;
      if(!creep_throttle_init){
        creep_throttle = base_throttle;
        creep_throttle_init = true;
      }

      if(btx_user_override_active){
        creep_until_ms = 0;
        creep_returning = false;
        creep_was_active = false;
        creep_throttle = base_throttle;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{

      bool creep_triggered = (BTx1Signal < BTxThreshold
        && ((drive_mode_status == 2 && case2_use_cruise)
        || (drive_mode_status == 1 && (throttle > CREEP_THROTTLE))));

      if(creep_triggered){
        creep_until_ms = millis() + (uint32_t)BTxCreepDuration * 1000U;
        creep_returning = false;
      }
      bool creep_active = (creep_triggered || (millis() < creep_until_ms)); // Enter creep if triggered by BTx

      if(!creep_active && creep_was_active){
        creep_return_base = base_throttle;
        creep_returning = true;
      }
      creep_was_active = creep_active;

      if(creep_active){
        double target_throttle = CREEP_THROTTLE;
        double step = fperiod::step(fabs(BTxAccelRate), periodRatio());  // us per packet at 10 ms
        creep_throttle = (step <= 0.0) ? target_throttle
          : (creep_throttle < target_throttle) ? fmin(creep_throttle + step, target_throttle)
          : (creep_throttle > target_throttle) ? fmax(creep_throttle - step, target_throttle)
          : creep_throttle;
        throttle = (int)lround(creep_throttle);
      }else if(creep_returning){
        double target_throttle = creep_return_base;
        double step = fperiod::step(fabs(BTxAccelRate), periodRatio());  // us per packet at 10 ms
        creep_throttle = (step <= 0.0) ? target_throttle
          : (creep_throttle < target_throttle) ? fmin(creep_throttle + step, target_throttle)
          : (creep_throttle > target_throttle) ? fmax(creep_throttle - step, target_throttle)
          : creep_throttle;
        throttle = (int)lround(creep_throttle);
        if((int)lround(creep_throttle) == creep_return_base){
          creep_returning = false;
        }
      }else{
        creep_throttle = base_throttle;
        throttle = base_throttle;
      }

      btx_throttle_override_active = (creep_active || creep_returning);
      }

    }
    //BTx follow feature for BTx channel 1 --------------------------
    if(BTxFollowChannel == 1){ //if BTx follow ch 1 is selected, do the follow behavior.
      int base_throttle = throttle;
      static double btx1_throttle = PWM_OUT_2_DEFAULT;
      static bool btx1_throttle_init = false;
      static bool btx1_follow_active = false;
      if(btx_user_override_active){
        btx1_throttle = base_throttle;
        btx1_follow_active = false;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{
        throttle = BTxFollow(base_throttle, btx1_throttle, btx1_throttle_init, btx1_follow_active, BTx1Signal);
        btx_throttle_override_active = btx1_follow_active;
      }
    }
    //BTx follow feature for BTx channel 2 ---------------------------
    if(BTxFollowChannel == 2){ //if BTx follow ch 2 is selected, do the follow behavior.
      int base_throttle = throttle;
      static double btx2_throttle = PWM_OUT_2_DEFAULT;
      static bool btx2_throttle_init = false;
      static bool btx2_follow_active = false;
      if(btx_user_override_active){
        btx2_throttle = base_throttle;
        btx2_follow_active = false;
        throttle = base_throttle;
        btx_throttle_override_active = false;
      }else{
        throttle = BTxFollow(base_throttle, btx2_throttle, btx2_throttle_init, btx2_follow_active, BTx2Signal);
        btx_throttle_override_active = btx2_follow_active;
      }
    }

    if(throttle >= 1470 && throttle <= 1530){ //Reset stuck detector when throttle is low
      stuckDetector.ResetStatus();
    }

    throttle = constrain(throttle, PWM_OUT_2_MIN, PWM_OUT_2_MAX);

    // Manual steering sets the yaw-rate request directly; wire following applies only in modes 1/2.
    int manual_steer_offset = 0;
    double pid_steer_contribution = pid_output;

    if(drive_mode_status == 0 ){  // Manual mode
      pid_steer_contribution = 0.0;
      uint32_t steering_input = pwm_read_rmt_dur(0);
      bool steering_valid = (steering_input >= 500 && steering_input <= 2500);
      if(steering_valid){
        manual_steer_offset = (int)steering_input - 1500;
      }
    }

    steering_output = PWM_OUT_1_DEFAULT + pid_steer_contribution + manual_steer_offset; //calculate steering output
    steering_output = constrain(steering_output, PWM_OUT_1_MIN, PWM_OUT_1_MAX); //Keep steering output within limits

    mark(SEC_ROVER);  // CPU profiler: the autopilot part
    roverStep();  // Mode 2 with rover_enabled: the autopilot replaces throttle and steering_output
#ifdef DIAG_CTRL_PROF
    mark(SEC_C_OUT);
#else
    mark(SEC_PACKET);  // CPU profiler: back to the frame part
#endif


     // Check for faults before commanding skid steering and remaining servo outputs.
     if( WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_OVERLOAD)  //  signal overload fault bit
      || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_VDD_FAULT)  // STM32 VDD fault bit
      || (stmPacket.items.temperature > 100)  // STM32 internal temp in °C
      ){
        driveOutput(PWM_OUT_2_DEFAULT, PWM_OUT_1_DEFAULT, false);

     }else if(((drive_mode_status == 1 || drive_mode_status == 2)
          && !rover_active && stuckDetector.GetStatus())  // Stop wire-follow modes when stuck (the rover is off the wire most of the time).
          || (charge_status > 0)  // vehicle charge in progress
          || (rover_active && roverAp.state == rover::FAULT)  // rover fault: stuck, lost too long, no polarity
          ){
        driveOutput(PWM_OUT_2_DEFAULT, PWM_OUT_1_DEFAULT, false);
        sine_wave_force_disabled = true; sine_wave_blend = 0.0; target_x_pos = 0.0;
        vehicleStatus = 2; //set status to 'charging' for telemetry purposes
        if (charge_status == 0) { // If there is a fault other than charging
         vehicleStatus = 0; //the vehicle should be considered stuck
         }

     }else{
       driveOutput(throttle, steering_output, true);
       sine_wave_force_disabled = false;
       vehicleStatus = 1; //set status to driving for telemetry purposes
     }
    rtprof::commandSet();  // real-time path timing (lib/rt_prof) (the drive command now comes from this frame)
    odrive_outputUpdate(true);  // the CAN commands of this frame, now (loop() keeps the 12 ms floor)
#ifdef DIAG_CTRL_PROF
    mark(SEC_PACKET);
#endif

    // LED blink for stuckDetector
    static uint32_t stuck_led_last_ms = 0;
    if (stuckDetector.GetStatus()) {
      uint32_t now = millis();
      if ((now - stuck_led_last_ms) < 7) {
        digitalWrite(nLED, 0); // LED ON (Low = ON)
      } else {
        digitalWrite(nLED, 1); // LED OFF
      }
      if ((now - stuck_led_last_ms) >= 250) {
        stuck_led_last_ms = now;
      }
    }

    // Telemetry: one log sample of this frame and the outputs it gave
    telemetryFrame();
#ifdef TELEMETRY_NEWVALUE
    // The example lines of include/user_telemetry.h: one tlm_set per value, here once per control pass
    tlm_set(TLM_test_frames, (float)(stmDec.st.frames & 0xFFFFu));
    tlm_set(TLM_test_r1_mm, stmPacket.items.s1_radius);
    tlm_set(TLM_test_drive, odrive_drive_command);
#endif

    // Charging
    mark(SEC_STMCMD);  // CPU profiler: the time from here counts to the "stmcmd" part
    chargeControl();  // monitor and control charge port
  }
  rxFrameDone();  // the time of the frame part (bench "timing")

  // Board services
  rxLinkService();  // Rx STM32 answers and commands, link watch
  radioService();   // ESP-NOW or Wi-Fi: telemetry, commands from the PC, web pages
  logService();     // telemetry log, crash counter, update confirm
  benchService();   // USB serial commands, USB link events, heartbeat line

  // LED: long-press flash, config saved
  if(longpress_flash_cycles > 0){ // Flash LED on long-press - sensor calibration
    led_blink_state = !led_blink_state;
    digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
    longpress_flash_cycles--;
  } else if (config_saved_led_until_ms && millis() < config_saved_led_until_ms) {
    digitalWrite(nLED, 0);  // Solid ON — config saved
  } else {
    config_saved_led_until_ms = 0;  // Clear once expired
  }

  // Serial Print debug data to the USB virtual COM port
  mark(SEC_DEBUG);  // CPU profiler: the debug print part
  static uint32_t last_debug_ms = 0;
  const uint32_t debug_now_ms = millis();
  if(debug_now_ms - last_debug_ms >= 100){
    last_debug_ms = debug_now_ms;

    // Individual prints kept for debugging — uncomment as needed:
    //Serial.print(" Ch1: "); Serial.print(pwm_read_rmt_dur(0)); // R/C PWM input 1 (steering input from receiver)
    //Serial.print(" Ch2: "); Serial.print(pwm_read_rmt_dur(1)); // R/C PWM input 2 (throttle input from receiver)
    //Serial.print(" Ch3: "); Serial.print(pwm_read_rmt_dur(2)); // R/C PWM input 3 (auxiliary input from receiver)
    // Former R/C PWM input 4 is now the Nano sonar UART0 RX (GPIO33).
    //Serial.print(" Thr Out: "); Serial.print(throttle);
    //Serial.print(" Str Out: "); Serial.print(steering_output);
    //Serial.print(" PID Output: "); Serial.print(pid_output);
    //Serial.printf(" Drive Mode:  %12d\n\r", drive_mode_status);
    //Serial.print(" BTx1: "); Serial.print(BTx1Signal);
    //Serial.print(" BTx2: "); Serial.print(BTx2Signal);
    //Serial.print(" X1: "); Serial.print(stmPacket.items.s1_x);
    //Serial.print(" Y1: "); Serial.print(stmPacket.items.s1_y);
    //Serial.print(" R1: "); Serial.print(stmPacket.items.s1_radius);
    //Serial.print("Channel: "); Serial.print(rx_channel);  // Rx board channel selected
    //Serial.print(" X2: "); Serial.print(stmPacket.items.s2_x);
    //Serial.print(" Y2: "); Serial.print(stmPacket.items.s2_y);
    //Serial.print(" R2: "); Serial.print(stmPacket.items.s2_radius);
    //Serial.print(" BTx1: "); Serial.print(stmPacket.items.s1_btx_radius[0]); // Sensor 1 BTx raw
    //Serial.print(" BTx2: "); Serial.print(stmPacket.items.s1_btx_radius[1]); // Sensor 1 BTx raw
    //Serial.print("S2 BTx1: "); Serial.print(stmPacket.items.s2_btx_radius[0]); // Sensor 2 BTx raw
    //Serial.print("S2 BTx2: "); Serial.print(stmPacket.items.s2_btx_radius[1]); // Sensor 2 BTx raw
    //Serial.print("  target_x_pos : "); Serial.print(target_x_pos);
    //Serial.printf("5V:       %12.4f\n\r", stmPacket.items.v5v);  // 5V input voltage to Rx board
    //Serial.printf("Vbatt:    %12.4f V\n\r", stmPacket.items.vbatt);  // vbatt
    //Serial.printf("Vpsu:     %12.4f V\n\r", stmPacket.items.vcharge);  // PSU input voltage
    //Serial.printf("Icharge:  %12.4f A\n\r", stmPacket.items.icharge);  // Charge current
    //Serial.printf("Chrg stat:%12d\n\r", (charge_status));  // charge status
    //Serial.printf("Chrg cool:%12d\n\r", (WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)));  // charge cooldown
    //Serial.printf("Chrg Code:%12lu\n\r", (stmPacket.items.charge_code));  // charge code
    //Serial.printf("Re(S1C1): %12.4f\n\r", (stmPacket.items.coil_data[0][0]));
    //Serial.printf("Im(S1C1): %12.4f\n\r", (stmPacket.items.coil_data[0][1]));
    // Serial.printf("Re(S1C2): %12.4f\n\r", (stmPacket.items.coil_data[0][2]));
    // Serial.printf("Im(S1C2): %12.4f\n\r", (stmPacket.items.coil_data[0][3]));
    // Serial.printf("Re(S2C1): %12.4f\n\r", (stmPacket.items.coil_data[1][0]));
    // Serial.printf("Im(S2C1): %12.4f\n\r", (stmPacket.items.coil_data[1][1]));
    // Serial.printf("Re(S2C2): %12.4f\n\r", (stmPacket.items.coil_data[1][2]));
    // Serial.printf("Im(S2C2): %12.4f\n\r", (stmPacket.items.coil_data[1][3]));
    // Serial.println(mpu.getGyroZ(), 3);  // Yaw rate in degrees/second
    // Serial.printf(" GPIO35: %d GPIO37: %d\n", digitalRead(MOTOR_ARM_SWITCH_PIN), digitalRead(BUZZER_PIN));

    if (sonar_pending) {
      usbOut.printf("Sonar: %lu,%lu,%lu,%lu\n",
                    (unsigned long)sonar_ranges[0], (unsigned long)sonar_ranges[1],
                    (unsigned long)sonar_ranges[2], (unsigned long)sonar_ranges[3]);
      sonar_pending = false;
    }


    //Serial.print(stmPacket.items.s1_x);
    //Serial.print(" ");
    //Serial.print(stmPacket.items.s1_y);
    //Serial.print(" ");
    //Serial.print(stmPacket.items.s1_radius);
    //Serial.print(" ");

    // The main-branch coil strength print (cc9bedb), 10 per second. It goes through usbOut (the USB text queue) in place of
    // Serial, so it cannot cut into the other USB lines. Layout 2 STM32: coilStrength() reads the coil magnitudes in
    // int24 LSB (lib/stm_v2 fills coil_data for it); v1 STM32: counts, as before.
    //const float strength_s1c1 = coilStrength(stmPacket, 0, 0);
    //const float strength_s1c2 = coilStrength(stmPacket, 0, 1);
    //usbOut.print(strength_s1c1);
    //usbOut.print(" ");
    //usbOut.print(strength_s1c2);
    //usbOut.println();

  }

  /*
  // Status log at ~1 s intervals (compatible with ESP-IDF log format)
  static uint32_t dbg_last_ms = 0;
  uint32_t dbg_now = millis();
  if(dbg_now - dbg_last_ms >= 1000){
    dbg_last_ms = dbg_now;
    ESP_LOGI(TAG, "lp:%lu mode:%d ovr:%lu crc_err:%lu x1:%.1f r1:%.1f thr:%d chrg:%d sb:0x%04lX",
      main_loop_counter,
      drive_mode_status,
      uart_overrun_count,
      crc_error_count,
      (float)stmPacket.items.s1_x,
      (float)stmPacket.items.s1_radius,
      throttle,
      charge_status,
      (unsigned long)stmPacket.items.status_bits);
    if(loop_exec_count > 0){
      ESP_LOGI(TAG, "loop_us: cnt:%lu min:%lu max:%lu",
        loop_exec_count, loop_min_us, loop_max_us);
    }
    loop_min_us = 0xFFFFFFFF;
    loop_max_us = 0;
    loop_exec_count = 0;
    // ESP_LOGI(TAG, "crc: calc=0x%04X stored=0x%04X %s",
    //   last_computed_crc,
    //   (uint16_t)stmPacket.items.crc,
    //   (last_computed_crc == stmPacket.items.crc) ? "OK" : "FAIL");
  }
  */

  // Board: monitor LED blink, one gyro sample, then wait for the next Rx STM32 frame
  loopEnd();
} // End of main loop -------------------------------------------------------------------------

#ifdef LOOP_NO_RETURN
// Test build (2026-10-29, control step timing): loop() never returns to the Arduino core. On a single-core chip the
// core sleeps 5 ms every 2 s between two loop() calls (yieldIfNecessary), so that the idle task runs; a frame then
// waits up to 5 ms for its control step (bench: the worst frame-to-step delays on a 2 s grid). The pass already
// waits for the next frame edge (loopEnd), and the idle task runs there. What the core does after each loop() call:
// the task watchdog reset (here, below) and serial events (none defined in this firmware). A pass with no wait for
// 1 s (bench pass-through, monitor and update paths poll) gets a 1 tick sleep, as the core gave it.
#include "esp_task_wdt.h"
extern bool loopTaskWDTEnabled;
extern uint32_t loop_waits;  // src/platform/board.inc: the end-of-pass waits
void loop() {
  uint32_t last_wait_ms = millis();
  for (;;) {
    const uint32_t w0 = loop_waits;
    loopPass();
    if (loopTaskWDTEnabled) esp_task_wdt_reset();
    const uint32_t now = millis();
    if (loop_waits != w0) last_wait_ms = now;
    else if (now - last_wait_ms >= 1000) { vTaskDelay(1); last_wait_ms = millis(); }
  }
}
#endif

// Cruise Control logic ---------------------------------------------------------
uint16_t driveMode(){
  // Passthrough input throttle command until PWM_IN_3 goes above 1500µs for 500ms, then latch on to the last throttle command.
  // Incriment and decriment the latched throttle level based on the input from throttle input.
  // Return to passthrough mode as soon as PWM_IN_3 goes to 500 - 1500µs.  This should allow for the RC Tx to be turned off

  uint32_t steering_input = pwm_read_rmt_dur(0);  // read steering input
  uint32_t throttle_input_raw_pwm = pwm_read_rmt_dur(1);  // raw throttle PWM input
  uint32_t aux_input = pwm_read_rmt_dur(2);  // read aux channel input
  bool steering_valid = (steering_input >= 500 && steering_input <= 2500);  // check if steering is in valid range
  bool throttle_valid = (throttle_input_raw_pwm >= 500 && throttle_input_raw_pwm <= 2500);  // check if throttle is in valid range
  bool aux_valid = (aux_input >= 500 && aux_input <= 2500);  // check if aux channel is in valid range
  uint32_t throttle_input_pwm = throttle_valid ? (uint32_t)constrain((int)throttle_input_raw_pwm, 1000, 2000) : 1500;

  // Start in autonomous mode when no throttle PWM is present at startup
  static bool startup_checked = false;
  static uint32_t startup_start_ms = 0;
  if(!startup_checked){
    if(startup_start_ms == 0){
      startup_start_ms = millis();
    }
    startup_checked = true;
    if(throttle_valid){ // if throttle input is valid at startup
      drive_mode_status = 0; // start in manual mode
    }else if(millis() - startup_start_ms > 500){ // wait briefly for valid throttle
      drive_mode_status = 2;
      case2_use_cruise = false; // ramp to creep on boot into case 2
      case2_throttle_current = PWM_OUT_2_DEFAULT;
    }else{
      startup_checked = false; // keep waiting for a valid throttle signal
    }
  }

  // Ensure throttle is neutral whenever entering case 2
  static int8_t last_drive_mode_status = 0;
  if(drive_mode_status == 2 && last_drive_mode_status != 2){
    case2_throttle_current = PWM_OUT_2_DEFAULT;
  }
  last_drive_mode_status = drive_mode_status;

  switch (drive_mode_status){
    case 0:  // Throttle passthrough (Manual mode), waiting for aux channel trigger
      if(aux_input > 1450 && steering_valid){ //using 1450 instead of 1500 for compatablity with 3 pos switches
        drive_mode_timer ++;
      }else{
        drive_mode_timer = 0;
      }

      if(drive_mode_timer > (uint32_t)fperiod::count(50, periodRatio())){ // switch to cruise control mode after 50 cycles (500ms) at 10 ms packets
        drive_mode_status = 1;
      }

      if(!throttle_valid){  // verify throttle is in range
        throttle_input_pwm = 1500;
      }

      throttle_micros = throttle_input_pwm;

      if(millis() - led_blink_last_ms >= 50){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return throttle_micros;  // passthrough throttle
      break;

    case 1:  // cruise control enabled
      if(aux_valid && steering_valid && aux_input < 1450){ //looking for aux channel to go to 500 - 1500µs
        drive_mode_status = 0;
      }

      // Manual throttle stick input takes priority over BTx creep/follow.
      if(throttle_valid && ((throttle_input_pwm > 1550) || (throttle_input_pwm < 1450 && throttle_input_pwm > 800))){
        btx_user_override_until_ms = millis() + 300;
      }

      if(!btx_throttle_override_active){
        // 1 us per packet (5 us for the fast slow-down) at 10 ms packets; the same rate in us/s at any update period
        static double dm1_carry[3] = {0, 0, 0};
        const double pr = periodRatio();
        if(throttle_input_pwm > 1550 && throttle_micros < PWM_OUT_2_MAX)  {  // Accelerate
          throttle_micros = (uint16_t)min<int>(PWM_OUT_2_MAX, throttle_micros + fperiod::wholeSteps(1, pr, dm1_carry[0]));
        }else if(throttle_input_pwm < 1450 && throttle_input_pwm > 1150 && throttle_micros > PWM_OUT_2_DEFAULT){  // decelerate
          throttle_micros = (uint16_t)max<int>(PWM_OUT_2_DEFAULT, throttle_micros - fperiod::wholeSteps(1, pr, dm1_carry[1]));
        }else if(throttle_input_pwm > 800 && throttle_input_pwm < 1100){  // slow down quickly
          throttle_micros = throttle_micros - fperiod::wholeSteps(5, pr, dm1_carry[2]);
        }
      }


      if(!throttle_valid){  // verify throttle is in range
        throttle_input_pwm = 1500; //if not, set to neutral for safety
      }

      if(millis() - led_blink_last_ms >= 400){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return throttle_micros;
      break;

    case 2:  // Wire-following autonomous mode with no Remote control input
      bool aux_in_manual = (pwm_read_rmt_dur(2) > 500 && pwm_read_rmt_dur(2) < 1450);
      if(aux_in_manual != case2_aux_entry_is_manual){
        case2_aux_changed = true;
      }
      if(case2_aux_changed && steering_valid){ // exit case 2 on any aux change
        drive_mode_status = aux_in_manual ? 0 : 1;
      }

      // Ramped throttle in case 2: cruise when selected, otherwise creep.
      if(!btx_throttle_override_active){
        // 1 us per packet at 10 ms packets; the same rate in us/s at any update period
        static double dm2_carry = 0;
        const int target = case2_use_cruise ? CRUISE_THROTTLE : CREEP_THROTTLE;
        if(case2_throttle_current < target){
          case2_throttle_current = min<int>(target, case2_throttle_current + fperiod::wholeSteps(1, periodRatio(), dm2_carry));
        }else if(case2_throttle_current > target){
          case2_throttle_current = max<int>(target, case2_throttle_current - fperiod::wholeSteps(1, periodRatio(), dm2_carry));
        }
      }

      if(millis() - led_blink_last_ms >= 1000){
        led_blink_last_ms = millis();
        led_blink_state = !led_blink_state;
        digitalWrite(nLED, led_blink_state ? 0 : 1);  // Low = ON, High = OFF
      }
      return case2_throttle_current;
      break;
  }
  return 1500;
}


// Read and process the button state
void buttonLogic(){

  uint8_t gpio_state = buttonRead();
  // The settings erase gesture (a press in the first 3 s after power-on) holds the button while it runs
  const int erase = buttonEraseGesture(gpio_state);
  if (erase == ERASE_RELEASED) {  // released before 5 s: nothing happens, and the press makes no click or calibration
    bttn_state = 0; bttn_counter = 0; bttn_click_counter = 0; bttn_long_counter = 0; bttn_3s_armed = 0; bttn_6s_state = 0;
    return;
  }
  if (erase == ERASE_HOLD) return;  // the gesture owns the button: no click, no calibration, no 6 s action

  // regular press logic
	if(!bttn_state && gpio_state)  // button was not pressed & is currently pressed
		bttn_counter++;
	else if(bttn_state && !gpio_state)  // button was pressed & is not currently pressed
		bttn_counter--;
  else if(bttn_state && gpio_state)
    bttn_click_counter++;

	if(bttn_counter < 0){
    // Falling edge
    ESP_LOGI(TAG, "btn: release (held %ld cycles)", bttn_long_counter);
    bool acted = bttn_6s_state;  // the 6 s action logged its event when it ran
    if(bttn_3s_armed && !bttn_6s_state && bttn_long_counter < BUTTON_CAL_MAX_PRESS){
      button3sPressEvent();
      acted = true;
    }
		bttn_state = 0;
		bttn_counter = 0;
    if(bttn_click_counter <= BUTTON_CLICK_MAX_LENGTH){
      buttonClickEvent();
      acted = true;
    }
    if(!acted) buttonEvent(BTN_NONE, (uint32_t)bttn_long_counter * 10u, "no action");  // 0.4 s to 2 s, or 4 s to 6 s
	}

	if(bttn_counter > BUTTON_DEBOUNCE_PERIODS){
    // Rising edge
    ESP_LOGI(TAG, "btn: press");
		bttn_state = 1;
		bttn_counter = BUTTON_DEBOUNCE_PERIODS;
    bttn_click_counter = 0;
	}

  // Timed press logic
  if(bttn_state){
    bttn_long_counter++;
  }else{
    bttn_long_counter = 0;
    bttn_3s_armed = 0;
    bttn_6s_state = 0;
  }

  if(!bttn_3s_armed && bttn_long_counter > BUTTON_CAL_MIN_PRESS){
    bttn_3s_armed = 1;
    ESP_LOGI(TAG, "btn: 3s window armed (release now to calibrate)");
  }

  if(!bttn_6s_state && bttn_long_counter > BUTTON_6S_PRESS){
    bttn_6s_state = 1;
    ESP_LOGI(TAG, "btn: 6s threshold reached");
    button6sPressEvent();
  }
}

// Called on button release if the press duration was less than the click threshold
void buttonClickEvent(){
  if(drive_mode_status == 2){
    case2_use_cruise = !case2_use_cruise; // Toggle case 2 output
    btx_user_override_until_ms = millis() + 2000;
    ESP_LOGI(TAG, "btn: click (mode2 cruise->%d)", case2_use_cruise);
    buttonEvent(BTN_CRUISE, (uint32_t)bttn_long_counter * 10u, case2_use_cruise ? "cruise on" : "cruise off");
  }else{
    drive_mode_status = 2; // Enter autonomous wire-following mode
    case2_use_cruise = true;
    case2_throttle_current = PWM_OUT_2_DEFAULT;
    uint32_t aux_pwm = pwm_read_rmt_dur(2);
    case2_aux_entry_is_manual = (aux_pwm > 500 && aux_pwm < 1500);
    case2_aux_changed = false;
    ESP_LOGI(TAG, "btn: click -> mode2 (auto)");
    buttonEvent(BTN_AUTO_ON, (uint32_t)bttn_long_counter * 10u,
                rover_enabled ? "drive mode auto (rover)" : "drive mode auto (wire follow)");
  }
}

// called when the long press threshold has been reached
void button3sPressEvent(){
  ESP_LOGI(TAG, "btn: 3s press -> calibrate center");

  // Calibrate cS1 center
  espPacketFifo_addCmdInt32(4 | 0x80000000, 0);  // Write S1 calibrate center
  espPacketFifo_addCmdInt32(5 | 0x80000000, 0);  // Write S2 calibrate center
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new calibration

  buttonEvent(BTN_CENTRE_CAL, (uint32_t)bttn_long_counter * 10u, "centre calibration, saved");
  longpress_flash_cycles = 0;
  config_saved_led_until_ms = millis() + 2000;  // Solid LED feedback for 2 seconds
  digitalWrite(nLED, 0);  // Low = ON
}

// No setting save in run mode (2026-10-29): a save is refused, not held until the stop. A flash write halts
// the processor and breaks the even control loop. The "save" command and the 6 s button save ask here; a refusal keeps
// the setting live in memory, and its text says why. Returns nullptr when a save may run.
bool vehicleStationary();  // src/platform/telemetry.inc: the stop rule
const char* roverSaveRefusal() {
  if (drive_mode_status == 2) return "refused in run mode: a save stops the control loop; the setting is live, save after the run";
  if (!vehicleStationary()) return "refused: the vehicle moves; stop it first";  // a flash write halts the processor
  return nullptr;
}

void button6sPressEvent(){
  pid_direction = (pid_direction == DIRECT) ? REVERSE : DIRECT;
  steeringPID.SetControllerDirection(pid_direction);
  g_deviceConfig.pid_direction = pid_direction;
  ESP_LOGI(TAG, "btn: 6s press -> pid_direction=%d", pid_direction);
  if (const char* why = roverSaveRefusal()) {  // no save in run mode: the direction is live, not saved
    usbOut.printf("CFG 4 save=%s\n", why);
    buttonEvent(BTN_STEER_DIR, (uint32_t)bttn_long_counter * 10u,
                pid_direction == DIRECT ? "steering direction normal, not saved: run mode, save after the run"
                                        : "steering direction reversed, not saved: run mode, save after the run");
    return;
  }
  const bool saved = msc_fat_save_all(&g_deviceConfig);
  if (!saved) {
    usbOut.println("Warning: failed to persist pid_direction to config file");
  }
  buttonEvent(BTN_STEER_DIR, (uint32_t)bttn_long_counter * 10u,
              pid_direction == DIRECT ? (saved ? "steering direction normal, saved" : "steering direction normal, save failed")
                                      : (saved ? "steering direction reversed, saved" : "steering direction reversed, save failed"));
}

void chargeControl(){
  // 0.5.110: the charge enable is a lease (Rx STM32 2.0.63+ drops it 1 s after the last CID 9 enable write): while
  // charging is wanted (enable sent, or charging), the enable goes again every 250 ms. A hung or restarting ESP32 so
  // leaves the charge port off within 1 s.
  // 0.5.111: every 250 ms or every second frame, whichever is longer (one command goes per frame; Rx STM32 2.0.65 holds
  // the lease 1 s or 4 update periods, whichever is longer), so a long update period does not fill the command queue.
  static uint32_t lease_ms = 0;
  const uint32_t lease_every_ms = max<uint32_t>(250, (uint32_t)(2000.0 * stmFramePeriodS()));
  if (CHARGE_AUTO_ENABLE && (charge_status == 2 || charge_status == 3) && millis() - lease_ms >= lease_every_ms) {
    lease_ms = millis();
    espPacketFifo_addCmdInt32(9 | 0x80000000, 1);
  }
  if(CHARGE_AUTO_ENABLE){
    switch (charge_status){
      case 0:  // Idle, waiting for input power
        if((stmPacket.items.vcharge > (stmPacket.items.vbatt + 0.4))){ //charge voltage must be 0.4v above battery voltage
          charge_status = 1;  // stops car
          lapFlag = true; // indicate vehicle has hit charger for telemetry lap counting
          charge_start_millis = millis();  // start 1 second timer
        }
        break;

      case 1:  // Input power present, waiting to request charge
        if((millis() - charge_start_millis) > 1000 && !WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
          espPacketFifo_addCmdInt32(9 | 0x80000000, 1);
          lease_ms = millis();
          charge_start_millis = millis();
          charge_status = 2;
        }
        break;

      case 2:  // CP enable request sent, waiting to see charging has enabled
      if(WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
          charge_status = 3;
        }

        if((millis() - charge_start_millis) > 4000 || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
          charge_start_millis = millis();
          charge_status = -1;  // abandon if charging does not start in 4 seconds, or a charging fault occurred
        }
        break;

      case 3: // Charging, waiting for charging to stop (for any reason)
        if(!WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
          charge_start_millis = millis();
          charge_status = -1;
        }
        break;

      case -1:  // charging has ended, resume throttle.  Do not allow re-attempt until after STM32 cooldown and 4-second ESP32 cooldown
        if(!WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN) && (millis() - charge_start_millis > 4000)){
          charge_status = 0;
        }
    }

  }else{
    if(WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
      espPacketFifo_addCmdInt32(9 | 0x80000000, 0);
    }
  }
}



int BTxFollow(int base_throttle, double &btx_throttle, bool &btx_throttle_init, bool &btx_follow_active, float btx_signal){
  bool following_now = (btx_signal < BTxThreshold);

  if(!btx_throttle_init){
    btx_throttle = base_throttle;
    btx_throttle_init = true;
  }

  if(!following_now && !btx_follow_active){
    btx_throttle = base_throttle;
    return base_throttle;
  }

  if(following_now){
    btx_follow_active = true;
  }

  double target_throttle = base_throttle;
  if(following_now && base_throttle > PWM_OUT_2_DEFAULT){
    target_throttle = PWM_OUT_2_DEFAULT;
  }

  double step = fperiod::step(fabs(BTxAccelRate), periodRatio());  // us per packet at 10 ms
  if(step <= 0.0){
    btx_throttle = target_throttle;
  }else if(btx_throttle < target_throttle){
    btx_throttle = fmin(btx_throttle + step, target_throttle);
  }else if(btx_throttle > target_throttle){
    btx_throttle = fmax(btx_throttle - step, target_throttle);
  }

  int output = (int)lround(btx_throttle);
  if(!following_now && btx_follow_active && output == base_throttle){
    btx_follow_active = false;
  }

  return output;
}

// -----------------------------------------------------------------------------
// applyDeviceConfig
//
// Copies all fields from cfg into the global firmware variables and reconfigures
// the steering PID, stuck detector, and charge parameters to match.
// Call after msc_fat_init() and again whenever msc_fat_loop() returns true.
// -----------------------------------------------------------------------------
void applyDeviceConfig(const DeviceConfig& cfg)
{
  // DSP settings: a change goes to the STM32 at once (a set, or a config.json edit on the USB drive)
  dspSettingsCheck(cfg);

  // Identification
  vehicleID           = cfg.vehicleID;
  rx_channel          = cfg.rx_channel;

  // Drive output and the servo map (0.5.69)
  drive_output        = cfg.drive_output;
  steer_output        = cfg.steer_output;
  throttle_output     = cfg.throttle_output;

  // Steering PID low-pass (0.5.69 steer_lpf_hz; 0 = off)
  if ((cfg.steer_lpf_hz > 0) != low_pass_enable) low_pass_first_cycle = true;
  low_pass_enable     = cfg.steer_lpf_hz > 0;
  low_pass_cutoff     = (float)cfg.steer_lpf_hz;

  // Drive parameters
  CRUISE_THROTTLE     = cfg.CRUISE_THROTTLE;
  CREEP_THROTTLE      = cfg.CREEP_THROTTLE;

  // PWM output limits and centres
  PWM_OUT_1_DEFAULT   = cfg.PWM_OUT_1_DEFAULT;
  PWM_OUT_1_MIN       = cfg.PWM_OUT_1_MIN;
  PWM_OUT_1_MAX       = cfg.PWM_OUT_1_MAX;
  PWM_OUT_2_DEFAULT   = cfg.PWM_OUT_2_DEFAULT;
  PWM_OUT_2_MIN       = cfg.PWM_OUT_2_MIN;
  PWM_OUT_2_MAX       = cfg.PWM_OUT_2_MAX;
  PWM_OUT_3_DEFAULT   = cfg.PWM_OUT_3_DEFAULT;
  PWM_OUT_3_MIN       = cfg.PWM_OUT_3_MIN;
  PWM_OUT_3_MAX       = cfg.PWM_OUT_3_MAX;
  PWM_OUT_4_DEFAULT   = cfg.PWM_OUT_4_DEFAULT;
  PWM_OUT_4_MIN       = cfg.PWM_OUT_4_MIN;
  PWM_OUT_4_MAX       = cfg.PWM_OUT_4_MAX;

  // Front PID gains and direction
  kp                  = cfg.kp;
  ki                  = cfg.ki;
  kd                  = cfg.kd;
  pid_direction       = cfg.pid_direction;
  steeringPID.SetControllerDirection(pid_direction);
  steeringPID.SetTunings(kp, ki, kd);
  steeringPID.SetOutputLimits(PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT,
                               PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);

  // Rate PID is reset when live settings change to discard old integral state.
  steeringRateReset();
  steering_rate_max_dps = cfg.steering_rate_max_dps;
  steeringRateSetTargetDps(steering_rate_target_dps);
  steeringRatePID.SetControllerDirection(cfg.steering_rate_pid_direction);
  steeringRatePID.SetTunings(cfg.steering_rate_kp, cfg.steering_rate_ki, cfg.steering_rate_kd);
  steeringRatePID.SetOutputLimits(-1.0, 1.0);
  steeringRatePID.SetSampleTime(ODRIVE_RC_UPDATE_MS);

  // Stuck detector
  stuckDetector.SetThreshold(cfg.stuck_threshold_mm);
  stuckDetector.SetTimeout(cfg.stuck_timeout_ms);

  // BTx proximity
  BTxFollowChannel    = cfg.BTxFollowChannel;
  BTxThreshold        = cfg.BTxThreshold;
  BTxAccelRate        = cfg.BTxAccelRate;
  BTxCreepDuration    = cfg.BTxCreepDuration;

  // Sensor 2
  enable_sensor_2     = cfg.enable_sensor_2;

  // Sine swerve
  SineSwerveAmplitude = cfg.SineSwerveAmplitude;
  SineSwerveFrequency = cfg.SineSwerveFrequency;

  // Charging -- queue updated parameters to STM32
  CHARGE_VBATT_STOP   = cfg.CHARGE_VBATT_STOP;
  CHARGE_MAX_CURRENT  = cfg.CHARGE_MAX_CURRENT;
  CHARGE_MAX_SECONDS  = cfg.CHARGE_MAX_SECONDS;
  // Only on a change: a "set" of any other key must not refill the STM32 command queue
  // (50 slots, 1 command at a time) or write the STM32's flash again.
  static bool stm_first = true;
  static double last_vstop = 0, last_imax = 0;
  static int last_tmax = 0, last_ch = -1;
  static bool last_s2 = false;
  if (stm_first || CHARGE_VBATT_STOP != last_vstop || CHARGE_MAX_CURRENT != last_imax || CHARGE_MAX_SECONDS != last_tmax) {
    espPacketFifo_addCmdFloat(6 | 0x80000000, (float)CHARGE_VBATT_STOP);
    espPacketFifo_addCmdFloat(7 | 0x80000000, (float)CHARGE_MAX_CURRENT);
    espPacketFifo_addCmdInt32(8 | 0x80000000, CHARGE_MAX_SECONDS);  // s; up to 0.5.68 2 x the setting
  }

  // RX channel
  if (stm_first || rx_channel != last_ch) {
    espPacketFifo_addCmdInt32(1 | 0x80000000, (int32_t)(rx_channel - 1));
  }
  // Sensor 2: setup() sends it at start-up (after this first call); from 0.5.25 also on a change,
  // so the STM32 holds the value of config.json with no restart.
  if (!stm_first && enable_sensor_2 != last_s2) {
    espPacketFifo_addCmdInt32(2 | 0x80000000, enable_sensor_2 ? 0x00 : 0x80);
  }
  stm_first = false;
  last_vstop = CHARGE_VBATT_STOP; last_imax = CHARGE_MAX_CURRENT; last_tmax = CHARGE_MAX_SECONDS; last_ch = rx_channel;
  last_s2 = enable_sensor_2;

  // Rover autopilot
  rover_enabled               = cfg.rover_mode != 2 && !safe_mode;  // autonomous runs the rover unless "position follower"; safe mode (crash resets): no rover mode
  rover_seek_vbatt            = cfg.rover_seek_vbatt;
  rover_entry_btx             = cfg.rover_entry_btx;
  rover_exit_btx              = cfg.rover_exit_btx;
  rover::Config& rc           = roverAp.cfg;
  rc.goalCW                   = cfg.rover_goal_cw;
  rc.followOnly               = cfg.rover_mode == 1;
  rc.joinNoPolarityStop       = cfg.rover_join_no_pol == 1;
  rc.sensorHeight             = (float)(cfg.rover_sensor_height_mm * 0.001);
  rc.roamSpeed                = cfg.rover_roam_speed_mps;
  rc.approachFraction         = cfg.rover_approach_pct * 0.01f;
  rc.bounceDistance           = cfg.rover_bounce_m;
  rc.bounceSpreadDeg          = cfg.rover_bounce_spread_deg;
  rc.bounceMinDeg             = cfg.rover_bounce_min_deg;
  rc.rearmDrop                = cfg.rover_rearm_drop_m;
  rc.rearmFall                = cfg.rover_rearm_fall_mm * 0.001f;
  rc.beaconBounce             = cfg.rover_beacon_bounce;
  rc.beaconDistance           = cfg.rover_beacon_m;
  rc.beaconRearm              = cfg.rover_beacon_rearm_m;
  rc.beaconSpreadDeg          = cfg.rover_beacon_spread_deg;
  rc.approachDistance         = cfg.rover_approach_m;
  rc.approachAngleDeg         = cfg.rover_join_angle_deg;
  rc.wrongDirCheck            = cfg.rover_wrong_dir_check;
  rc.departMin                = cfg.rover_depart_min_m;
  rc.departMax                = cfg.rover_depart_max_m;
  rc.probeLength              = cfg.rover_probe_m;
  rc.roamTimeS                = cfg.rover_roam_time_s;
  rc.followTimeS              = cfg.rover_follow_time_s;
  rc.followExitDeg            = cfg.rover_follow_exit_deg;
  rc.noTxRoamS                = cfg.rover_no_tx_roam_s;
  rc.noTxS                    = cfg.rover_no_tx_s;
  rc.lostTimeout              = (float)cfg.rover_lost_timeout_s;
  rc.bounceStyle              = cfg.rover_bounce_style;
  rc.followInset              = cfg.rover_follow_inset_m;
  rc.turnRadius               = cfg.rover_turn_radius_m;
  rc.bounceFollowS            = cfg.rover_bounce_follow_s;
  rc.wireMaxDepth             = cfg.wire_max_depth_mm * 0.001f;
  // The autopilot steer command goes through the gyro rate loop: full steer requests steering_rate_max_dps
  // (odrive_setDriveOutput), so the autopilot's full yaw rate is that value.
  rc.maxYawRate               = (float)(steering_rate_max_dps * M_PI / 180.0);
  applyTxPower();  // live: the next frame goes out at the new power
}

// The Tx signal is present on the followed channel: the wire bin SNR (Rx STM32 extension item 1) of either sensor at
// or above ROVER_TX_SNR_DB within the last 1 s. Not known (no item 1 yet): present.
#define ROVER_TX_SNR_DB 10.0f
bool roverTxSeen(){
  const int b = stmDec.wireBin(0);
  if (b < 0 || b > 7) return true;
  const uint16_t now10 = (uint16_t)(millis() / 10u);
  bool known = false;
  for (unsigned r = 0; r < 2; r++) {
    const unsigned sl = (unsigned)b | (r << 3);
    const uint16_t t = stmDec.ext_t10[0][sl];
    if (!t || (uint16_t)(now10 - t) > 100u) continue;
    known = true;
    if (stmDec.ext_val[0][sl] * 0.01f >= ROVER_TX_SNR_DB) return true;
  }
  return !known;
}

// Rover autopilot step, once per valid STM32 packet after the wire-follow outputs are computed. In mode 2 with
// rover_enabled it replaces throttle and steering_output, except in FOLLOW and DEPART: there the wire follower's own
// throttle and steering stay (the vehicle's follower and its tuning).
// Rover mode, follower input: the Rx quadrant state can flip (x, y) -> (-x, -y) after a pivot through square-on, and
// the follower steers on x. Near the wire (|y| > |x|) the wire is below the sensor, so the sign of y tells the state.
// The sign of y in the follower's working state is learned in FOLLOW after the probe length of steady holding
// (|x| within the join capture offset, the wire below) in the goal direction (polarity, when known): the follower holds
// the wire steadily only in its working state, or mirrored in the other direction. Near
// the wire a different sign of y gives -x; elsewhere the last decision holds; before the first steady hold, raw x.
int8_t rover_y_below = 0;   // learned sign of y with the wire below (0 = not yet)
static int8_t rover_x_sign = 1;
int roverFollowSignNow() { return rover_x_sign; }  // for the log and the emulator
float roverFollowSign(){
  static float steady_from_m = -1.0f;
  const float x = stmPacket.items.s1_x, y = stmPacket.items.s1_y;  // mm
  const bool lost = WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_LOST);
  const bool below = !lost && fabsf(y) > fabsf(x);
  const int8_t ys = y < 0 ? -1 : 1;
  // A follower with x mirrored also holds the wire, in the other direction: learn only in the goal direction
  const int8_t goal = g_deviceConfig.rover_goal_cw ? 1 : -1;
  const bool goalWay = world_heading == goal || world_heading == 0;
  if (roverAp.state == rover::FOLLOW && rover_out.external && below && goalWay && fabsf(x) <= 100.0f) {
    if (steady_from_m < 0) steady_from_m = rover_driven_m;
    if (rover_driven_m - steady_from_m >= roverAp.cfg.probeLength && rover_y_below != ys * rover_x_sign) {
      rover_y_below = (int8_t)(ys * rover_x_sign);  // the sign of y in the working state
    }
  } else {
    steady_from_m = -1.0f;
  }
  if (roverAp.yBelow != 0) { rover_y_below = roverAp.yBelow; roverAp.yBelow = 0; }  // set by the join from geometry
  if (rover_y_below != 0 && below) rover_x_sign = (ys == rover_y_below) ? 1 : -1;
  return (float)rover_x_sign;
}

void roverStep(){
  imuTake(0);  // the newest IMU sample (test build IMU_I2C_NB; empty otherwise)
  const bool want = rover_enabled && drive_mode_status == 2;
  const uint32_t now_us = micros();
  float dt = rover_last_packet_us ? (now_us - rover_last_packet_us) * 1e-6f : 0.01f;
  rover_last_packet_us = now_us;
  if(dt > 0.1f) dt = 0.1f;  // a stalled loop must not become one long step
  if(want && !rover_active){  // entering mode 2: start from a fresh state
    roverAp.reset();
    rover_out = rover::Output();
    rover_need_charge = false;
    rover_driven_m = 0.0f;
    rover_was_docked = charge_status > 0;
    rover_charge_ran = false;
  }
  if(!want && rover_active) roverAp.releaseSolve();  // the end of a run: the Bounce mode solve's memory back to the heap
  rover_active = want;
  if(!want) return;

  const uint32_t sb = stmPacket.items.status_bits;
  const uint32_t now_ms = millis();
  rover::Inputs in;
  in.dt = dt;
  in.periodR = (float)periodRatio();
  in.periodS = (float)stmFramePeriodS();
  in.inRange = !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_LOST)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_OVERLOAD)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_DISABLED);
  in.x = stmPacket.items.s1_x * 0.001f;  // mm -> m, as the Rx sends them
  in.y = stmPacket.items.s1_y * 0.001f;
  in.R = stmPacket.items.s1_radius * 0.001f;
  // IMU roll correction (optional): a roll turns the field in the coil plane; turn x and y back by the accelerometer
  // roll. The radius does not change.
  if (g_deviceConfig.rover_imu_roll && gyro_ready) {
    const float a = (g_deviceConfig.rover_imu_roll == 2 ? 1.0f : -1.0f) * IMU_ROLL16() * (float)(M_PI / 32768.0);
    const float x = in.x, y = in.y, c = cosf(a), s = sinf(a);
    in.x = x * c - y * s;
    in.y = x * s + y * c;
  }
  // Sensor 2 for control (sensor2Control): the leading sensor of a reverse roam in the sweeping modes (rover_autopilot.cpp
  // leadInputs). Sensor 2 for data only: sensor 1 leads both ways
  in.inRange2 = sensor2Control()
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_LOST)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_OVERLOAD)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_DISABLED);
  in.x2 = stmPacket.items.s2_x * 0.001f;
  in.y2 = stmPacket.items.s2_y * 0.001f;
  in.R2 = stmPacket.items.s2_radius * 0.001f;
  if (g_deviceConfig.rover_imu_roll && gyro_ready) {   // the same roll correction as sensor 1
    const float a = (g_deviceConfig.rover_imu_roll == 2 ? 1.0f : -1.0f) * IMU_ROLL16() * (float)(M_PI / 32768.0);
    const float x = in.x2, y = in.y2, c = cosf(a), s = sinf(a);
    in.x2 = x * c - y * s;
    in.y2 = x * s + y * c;
  }
  in.inside = world_inside;    // polarity facts (worldFacts); 0 when not known
  in.heading = world_heading;

  // Speed and distance driven: the wheel speeds when fresh (full drive command = roam speed), else the command. Turn
  // measure: the gyro when fitted, else the command.
  const bool wheels_fresh = odrive_vel_received[0] && odrive_vel_received[1]
      && now_ms - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS && now_ms - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS;
  const float k_mps = roverAp.cfg.roamSpeed / ODRIVE_DRIVE_SCALE_RPS;  // m/s per rev/s
  float travel_mps;
  if (wheels_fresh) {
    const float left = odrive_vel_rps[1] * k_mps, right = -odrive_vel_rps[0] * k_mps;  // skid mix: even nodes reversed
    in.v = 0.5f * (left + right);
    travel_mps = 0.5f * (fabsf(left) + fabsf(right));
  } else {
    in.v = rover_out.external ? (throttle - PWM_OUT_2_DEFAULT) / (float)(PWM_OUT_2_MAX - PWM_OUT_2_DEFAULT) * roverAp.cfg.roamSpeed
                              : rover_out.v;
    travel_mps = fabsf(in.v);
  }
  in.yawRate = steeringRateGyroValid() ? (float)(mpu.getGyroZ() * M_PI / 180.0) : rover_out.w;
  static uint32_t tx_seen_ms = 0;
  if (roverTxSeen()) tx_seen_ms = now_ms;
  in.txOffS = (now_ms - tx_seen_ms) * 0.001f;  // the autopilot stops after its limit for the state
  // Stuck detector: only while the signal is not lost; its window starts again while lost or while the autopilot
  // holds still.
  if (!in.inRange || (!rover_out.external && rover_out.v == 0.0f)) stuckDetector.ResetStatus();
  in.stuck = in.inRange && stuckDetector.GetStatus();

  // Charging: DOCK while the charge state machine runs; a charge clears the return.
  const bool docked = charge_status > 0;
  in.docked = docked;
  in.onContacts = stmPacket.items.vcharge > stmPacket.items.vbatt + 0.4f;  // the chargeControl() test
  if (charge_status == 3) rover_charge_ran = true;  // the STM32 charged
  if (rover_was_docked && !docked && rover_charge_ran) {
    rover_need_charge = false;
    rover_driven_m = 0.0f;
  }
  if (!docked) rover_charge_ran = false;
  rover_was_docked = docked;

  // Return: distance driven since the last charge, or battery voltage (filtered: it sags under load).
  rover_driven_m += travel_mps * dt;
  const float vb = stmPacket.items.vbatt;
  if (periodRatio() == 1.0) rover_vbatt_filt = (rover_vbatt_filt <= 0.0f) ? vb : 0.99f * rover_vbatt_filt + 0.01f * vb;
  else {  // 0.01 per packet at 10 ms: the same time constant (1 s) at any update period
    const float a = fperiod::weight(0.01f, periodRatio());
    rover_vbatt_filt = (rover_vbatt_filt <= 0.0f) ? vb : (1.0f - a) * rover_vbatt_filt + a * vb;
  }
  if(rover_seek_vbatt > 0 && rover_vbatt_filt > 1.0f && rover_vbatt_filt < rover_seek_vbatt) rover_need_charge = true;
  in.needCharge = rover_need_charge;

  // Beacons: the radius of each channel filtered as BTx1Signal but not clamped (the re-arm distance is past the BTx
  // threshold); entry and exit by mower channel number with the follower's signals (BTx1Signal, BTx2Signal).
  for (int c = 0; c < 2; c++) {
    const float r = constrain(stmPacket.items.s1_btx_radius[c], 10.0f, 2000.0f);
    const float a = fperiod::weight(BTxFilterAlpha, periodRatio());  // alpha per packet at 10 ms
    rover_btx_mm[c] = a * r + (1 - a) * rover_btx_mm[c];
  }
  in.beaconRange = fminf(rover_btx_mm[0], rover_btx_mm[1]) * 0.001f;
  const float btx_signal[3] = {(float)BTxThreshold + 1.0f, BTx1Signal, BTx2Signal};
  in.entryBeacon = rover_entry_btx != 0 && btx_signal[rover_entry_btx] < BTxThreshold;
  in.exitBeacon = rover_exit_btx != 0 && btx_signal[rover_exit_btx] < BTxThreshold;
  // Sonar (sonarService): the near mask and the last record; 0 with no Nano
  in.sonarNear = rover_sonar_near;
  for (int i = 0; i < 4; i++) in.sonarMm[i] = (uint16_t)(sonar_ranges[i] < 65535u ? sonar_ranges[i] : 65535u);

  const float prev_w = rover_out.w;
#ifdef DIAG_CTRL_PROF
  mark(SEC_C_AP);
#endif
  rover_out = roverAp.step(in);
#ifdef DIAG_CTRL_PROF
  mark(SEC_ROVER);
#endif
  // A pivot ended: clear the rate loop integral, or it keeps the vehicle turning for seconds after the pivot.
  if (prev_w != 0.0f && rover_out.w == 0.0f && !rover_out.external) steeringRateReset();

  // Decision log: each bounce, check, join, reversal, beacon turn, dock, depart, Tx change and fault.
  static uint32_t dec_seen = 0;
  if (roverAp.dec.seq != dec_seen) {
    dec_seen = roverAp.dec.seq;
    const rover::Decision& d = roverAp.dec;
    tlm_set(TLM_rover_dec_seq, (float)(d.seq & 0xFFFFu));
    tlm_set(TLM_rover_dec_code, (float)d.code);
    tlm_set(TLM_rover_dec_side, (float)d.side);
    tlm_set(TLM_rover_dec_psi_deg, d.psiDeg);
    tlm_set(TLM_rover_dec_turn_deg, d.turnDeg);
    tlm_set(TLM_rover_fault, (float)roverAp.fault);
    tlm_set(TLM_rover_dec_d_mm, constrain(d.d * 1000.0f, 0.0f, 65000.0f));
    if (d.solveOk) {
      tlm_set(TLM_rover_dec_solve_deg, d.solvePsiDeg);
      tlm_set(TLM_rover_dec_solve_mm, constrain(d.solveD * 1000.0f, 0.0f, 65000.0f));
      tlm_set(TLM_rover_dec_solve_sensor, d.solveSensor);
    } else {
      tlm_set(TLM_rover_dec_solve_deg, NAN);   // no solve answer: no value
      tlm_set(TLM_rover_dec_solve_mm, NAN);
      tlm_set(TLM_rover_dec_solve_sensor, NAN);
    }
    if (d.code == rover::DEC_SONAR) {   // one log event per sonar trigger: sonar, distance, action
      char t[72];
      if (roverAp.cfg.bounceStyle == 2)
        snprintf(t, sizeof t, "sonar %u at %u mm: stop, drive %s", (unsigned)d.sonar, (unsigned)d.sonarMm,
                 roverAp.driveDir() > 0 ? "backward" : "forward");
      else
        snprintf(t, sizeof t, "sonar %u at %u mm: stop, turn 180 deg %s", (unsigned)d.sonar, (unsigned)d.sonarMm,
                 d.turnDeg > 0 ? "left" : "right");
      sonarEvent(d.sonar, d.sonarMm, t);
    }
    if (usb_verbose)
      usbOut.printf("ROVER %s %s d=%.2f X=%.2f psi=%.0f turn=%.0f side=%d fault=%d\n", rover::stateName(roverAp.state),
                    rover::decisionName(d.code), d.d, d.X, d.psiDeg, d.turnDeg, d.side, d.reason);
  }

  if (rover_out.external) return;  // FOLLOW or DEPART: the wire follower's outputs stay

  // Command to the servo-equivalent outputs that odrive_setDriveOutput() normalises back.
  const float drive = constrain(rover_out.v / roverAp.cfg.roamSpeed, -1.0f, 1.0f);
  const float steer = constrain(rover_out.w / roverAp.cfg.maxYawRate, -1.0f, 1.0f);  // left (CCW) positive
  throttle = (int)lround(drive >= 0 ? PWM_OUT_2_DEFAULT + drive * (PWM_OUT_2_MAX - PWM_OUT_2_DEFAULT)
                                    : PWM_OUT_2_DEFAULT + drive * (PWM_OUT_2_DEFAULT - PWM_OUT_2_MIN));
  // Wire following with pid_direction DIRECT turns toward a wire on the left with a positive
  // ODrive steer command, so DIRECT maps left to positive; REVERSE flips it.
  const float left = (pid_direction == DIRECT) ? steer : -steer;
  steering_output = left >= 0 ? PWM_OUT_1_DEFAULT - left * (PWM_OUT_1_DEFAULT - PWM_OUT_1_MIN)
                              : PWM_OUT_1_DEFAULT - left * (PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);

  static uint32_t print_ms = 0;
  if(usb_verbose && millis() - print_ms >= 1000){
    print_ms = millis();
    usbOut.printf("rover %s sub=%d in=%d X=%.2f d=%.2f v=%.2f w=%.2f home=%d driven=%.0f inside=%d head=%d tx=%d\n",
      rover::stateName(roverAp.state), (int)roverAp.sub, (int)in.inRange, roverAp.rd.Xf, roverAp.rd.df,
      rover_out.v, rover_out.w, (int)rover_need_charge, rover_driven_m,
      in.inside, in.heading, (int)(in.txOffS < 2.0f));
  }
}
