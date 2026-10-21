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
#include "rover_autopilot.h"
#include "rctf_wfrx_status_bits.h"
#include "rctf_wfrx_exp32.h"
#include "tlm.h"           // table values: one line in include/telemetry_table.h defines a logged value (lib/tlm)
#include "rx_platform.h"   // the board: pins, Rx STM32 frames, motor controllers, servo outputs, gyro, settings, USB text

static const char* TAG = "wfrx";

// ===== Rover autopilot values =====

// Rover autopilot (lib/rover_autopilot): roam inside the perimeter, return to the wire after the return distance
// or below the return voltage, join it in the goal direction (rover_goal_cw), follow it to the dock with the wire
// follower below, charge, depart. Runs in drive mode 2 (rover_mode rover or follow only); rover_mode position follower: mode 2 is the plain wire
// follower.
rover::Autopilot roverAp;
bool rover_enabled = false;
double rover_seek_vbatt = 0.0;      // return battery voltage (0 = off)
float rover_return_m = 0.0f;        // return distance (0 = off)
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

// 0.6.21: world facts from the polarity (Rx STM32 2.2.0+), at the sensor nearest the wire, for the followed Tx channel
// (rx_channel). world_inside: +1 inside the loop, -1 outside. world_heading: +1 clockwise round the loop, -1
// counterclockwise (seen from above). 0: not known. With D = the current direction setting of the channel (+1
// clockwise, -1 counterclockwise): inside = polarity x sign(x) x D; heading = polarity x sign(y) x D. Each fact
// counts only away from its blind line (|x|, or |y|, at least 25 % of the radius: above the wire x is near 0, square
// to the wire y is near 0); else, and while the polarity is 0, the last value holds. No sensor in range for 2 s,
// no fresh polarity item for 1 s, or no current direction set: both 0.
int8_t world_inside = 0, world_heading = 0;
void worldFacts(){
  const int c = (int)rx_channel - 1;
  const uint8_t dirs[5] = {g_deviceConfig.tx_dir_ch1, g_deviceConfig.tx_dir_ch2, g_deviceConfig.tx_dir_ch3,
                           g_deviceConfig.tx_dir_ch4, g_deviceConfig.tx_dir_ch5};
  const int D = (c >= 0 && c < 5) ? (dirs[c] == 1 ? 1 : dirs[c] == 2 ? -1 : 0) : 0;
  const uint32_t now = millis();
  const bool fresh = stmDec.pol_t10 && (uint16_t)((uint16_t)(now / 10u) - stmDec.pol_t10) <= 100u;
  static uint32_t seen_ms = 0;
  int s = -1;
  float rs = 0;
  for (int k = 0; k < 2; k++) {
    if (stmDec.lost[k]) continue;
    const float r = k == 0 ? stmPacket.items.s1_radius : stmPacket.items.s2_radius;
    if (s < 0 || r < rs) { s = k; rs = r; }
  }
  if (s >= 0) seen_ms = now;
  if (D == 0 || !fresh || now - seen_ms > 2000u) { world_inside = world_heading = 0; return; }
  if (s < 0) return;
  const int p = stmDec.pol[s][c];
  if (p == 0) return;
  const float x = s == 0 ? stmPacket.items.s1_x : stmPacket.items.s2_x;
  const float y = s == 0 ? stmPacket.items.s1_y : stmPacket.items.s2_y;
  if (fabsf(x) >= 0.25f * rs) world_inside = (int8_t)(p * (x > 0 ? 1 : -1) * D);
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
uint32_t data_sent = 0;  // flag that debug UART statements have been sent
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
  const double dt_s = frame_now ? stmFramePeriodS() : constrain((now - odrive_rc_update_millis) / 1000.0, 0.001, 0.1);
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
  // Board start: the Rx STM32 serial line, the crash counter, the settings from the USB drive.
  // A board started as the PC monitor does only that job: no vehicle code starts.
  if (platformBegin()) return;

  // The settings to the vehicle values (applyDeviceConfig, below)
  applyDeviceConfig(g_deviceConfig);

  // Board: radio mode, web pages, telemetry log, USB serial, updates, Rx STM32 serial port
  platformStart();

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
  pinMode(PWM_IN_4, INPUT);

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
void loop() {
  // Board first: watchdog, USB, updates. During an update, the PC monitor job or a bench pass-through, the board
  // uses the whole pass and the vehicle code waits.
  if (platformLoopStart()) return;

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
    BTx1Signal = BTxFilterAlpha * BTx1Signal + (1 - BTxFilterAlpha) * BTx1LastDistance;
    BTx2Signal = BTxFilterAlpha * BTx2Signal + (1 - BTxFilterAlpha) * BTx2LastDistance;
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
      if(sine_wave_blend < blend_target){
        sine_wave_blend = fmin(blend_target, sine_wave_blend + SineSwerveSnapRate);
      }else if(sine_wave_blend > blend_target){
        sine_wave_blend = fmax(blend_target, sine_wave_blend - SineSwerveSnapRate);
      }

      double time_sec = loop_now_ms / 1000.0;
      double sine_offset = (SineSwerveAmplitude) * sin((2.0 * 3.14159 / SineSwerveFrequency) * time_sec);
      target_x_pos = sine_wave_blend * sine_offset;
    }else{
      sine_wave_blend = 0.0;
    }

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
        double step = fabs(BTxAccelRate);
        creep_throttle = (step <= 0.0) ? target_throttle
          : (creep_throttle < target_throttle) ? fmin(creep_throttle + step, target_throttle)
          : (creep_throttle > target_throttle) ? fmax(creep_throttle - step, target_throttle)
          : creep_throttle;
        throttle = (int)lround(creep_throttle);
      }else if(creep_returning){
        double target_throttle = creep_return_base;
        double step = fabs(BTxAccelRate);
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
    mark(SEC_PACKET);  // CPU profiler: back to the frame part


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
  uint32_t interval = 100; // interval between prints in milliseconds
  if(millis() % interval == 0 && !data_sent){

    // Individual prints kept for debugging — uncomment as needed:
    //Serial.print(" Ch1: "); Serial.print(pwm_read_rmt_dur(0)); // R/C PWM input 1 (steering input from receiver)
    //Serial.print(" Ch2: "); Serial.print(pwm_read_rmt_dur(1)); // R/C PWM input 2 (throttle input from receiver)
    //Serial.print(" Ch3: "); Serial.print(pwm_read_rmt_dur(2)); // R/C PWM input 3 (auxiliary input from receiver)
    //Serial.print(" Ch4: "); Serial.print(pwm_read_rmt_dur(3)); // R/C PWM input 4 (unused)
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


    //Serial.print(stmPacket.items.s1_x);
    //Serial.print(" ");
    //Serial.print(stmPacket.items.s1_y);
    //Serial.print(" ");
    //Serial.print(stmPacket.items.s1_radius);
    //Serial.print(" ");

    // The main-branch coil strength print (cc9bedb), 10 per second. It goes through usbOut (the USB text queue) in place of
    // Serial, so it cannot cut into the other USB lines. Layout 2 STM32: coilStrength() reads the coil magnitudes in
    // int24 LSB (lib/stm_v2 fills coil_data for it); v1 STM32: counts, as before.
    const float strength_s1c1 = coilStrength(stmPacket, 0, 0);
    const float strength_s1c2 = coilStrength(stmPacket, 0, 1);
    usbOut.print(strength_s1c1);
    usbOut.print(" ");
    usbOut.print(strength_s1c2);
    usbOut.println();

    data_sent = 1;
  }
  if(millis() % interval >= 1){
    data_sent = 0;  // reset flag to allow new data to send
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

      if(drive_mode_timer > 50){ // switch to cruise control mode after 50 cycles (500ms)
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
        if(throttle_input_pwm > 1550 && throttle_micros < PWM_OUT_2_MAX)  {  // Accelerate
          throttle_micros++;
        }else if(throttle_input_pwm < 1450 && throttle_input_pwm > 1150 && throttle_micros > PWM_OUT_2_DEFAULT){  // decelerate
          throttle_micros--;
        }else if(throttle_input_pwm > 800 && throttle_input_pwm < 1100){  // slow down quickly
          throttle_micros = throttle_micros - 5;
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
        if(case2_use_cruise){
          if(case2_throttle_current < CRUISE_THROTTLE){
            case2_throttle_current++;
          }else if(case2_throttle_current > CRUISE_THROTTLE){
            case2_throttle_current--;
          }
        }else{
          if(case2_throttle_current < CREEP_THROTTLE){
            case2_throttle_current++;
          }else if(case2_throttle_current > CREEP_THROTTLE){
            case2_throttle_current--;
          }
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
    if(bttn_3s_armed && !bttn_6s_state && bttn_long_counter < BUTTON_CAL_MAX_PRESS){
      button3sPressEvent();
    }
		bttn_state = 0;
		bttn_counter = 0;
    if(bttn_click_counter <= BUTTON_CLICK_MAX_LENGTH){
      buttonClickEvent();
    }
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
  }else{
    drive_mode_status = 2; // Enter autonomous wire-following mode
    case2_use_cruise = true;
    case2_throttle_current = PWM_OUT_2_DEFAULT;
    uint32_t aux_pwm = pwm_read_rmt_dur(2);
    case2_aux_entry_is_manual = (aux_pwm > 500 && aux_pwm < 1500);
    case2_aux_changed = false;
    ESP_LOGI(TAG, "btn: click -> mode2 (auto)");
  }
}

// called when the long press threshold has been reached
void button3sPressEvent(){
  ESP_LOGI(TAG, "btn: 3s press -> calibrate center");

  // Calibrate cS1 center
  espPacketFifo_addCmdInt32(4 | 0x80000000, 0);  // Write S1 calibrate center
  espPacketFifo_addCmdInt32(5 | 0x80000000, 0);  // Write S2 calibrate center
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new calibration

  longpress_flash_cycles = 0;
  config_saved_led_until_ms = millis() + 2000;  // Solid LED feedback for 2 seconds
  digitalWrite(nLED, 0);  // Low = ON
}

void button6sPressEvent(){
  pid_direction = (pid_direction == DIRECT) ? REVERSE : DIRECT;
  steeringPID.SetControllerDirection(pid_direction);
  g_deviceConfig.pid_direction = pid_direction;
  ESP_LOGI(TAG, "btn: 6s press -> pid_direction=%d", pid_direction);
  if (!msc_fat_save_all(&g_deviceConfig)) {
    usbOut.println("Warning: failed to persist pid_direction to config file");
  }
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

  double step = fabs(BTxAccelRate);
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
  rover_return_m              = cfg.rover_return_m;
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
  rc.rearmDistance            = cfg.rover_rearm_m;
  rc.rearmDrop                = cfg.rover_rearm_drop_m;
  rc.beaconBounce             = cfg.rover_beacon_bounce;
  rc.beaconDistance           = cfg.rover_beacon_m;
  rc.beaconRearm              = cfg.rover_beacon_rearm_m;
  rc.beaconSpreadDeg          = cfg.rover_beacon_spread_deg;
  rc.approachDistance         = cfg.rover_approach_m;
  rc.joinPivotDistance        = cfg.rover_join_pivot_m;
  rc.approachAngleDeg         = cfg.rover_join_angle_deg;
  rc.wrongDirCheck            = cfg.rover_wrong_dir_check;
  rc.departMin                = cfg.rover_depart_min_m;
  rc.departMax                = cfg.rover_depart_max_m;
  rc.turnMinDeg               = cfg.rover_turn_min_deg;
  rc.turnMaxDeg               = cfg.rover_turn_max_deg;
  rc.probeLength              = cfg.rover_probe_m;
  rc.noTxRoamS                = cfg.rover_no_tx_roam_s;
  rc.noTxS                    = cfg.rover_no_tx_s;
  rc.lostTimeout              = (float)cfg.rover_lost_timeout_s;
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
  rover_active = want;
  if(!want) return;

  const uint32_t sb = stmPacket.items.status_bits;
  const uint32_t now_ms = millis();
  rover::Inputs in;
  in.dt = dt;
  in.inRange = !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_LOST)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_OVERLOAD)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_DISABLED);
  in.x = stmPacket.items.s1_x * 0.001f;  // mm -> m, as the Rx sends them
  in.y = stmPacket.items.s1_y * 0.001f;
  in.R = stmPacket.items.s1_radius * 0.001f;
  // IMU roll correction (optional): a roll turns the field in the coil plane; turn x and y back by the accelerometer
  // roll. The radius does not change.
  if (g_deviceConfig.rover_imu_roll && gyro_ready) {
    const float a = (g_deviceConfig.rover_imu_roll == 2 ? 1.0f : -1.0f) * imu_roll16 * (float)(M_PI / 32768.0);
    const float x = in.x, y = in.y, c = cosf(a), s = sinf(a);
    in.x = x * c - y * s;
    in.y = x * s + y * c;
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
  rover_vbatt_filt = (rover_vbatt_filt <= 0.0f) ? vb : 0.99f * rover_vbatt_filt + 0.01f * vb;
  if(rover_seek_vbatt > 0 && rover_vbatt_filt > 1.0f && rover_vbatt_filt < rover_seek_vbatt) rover_need_charge = true;
  if(rover_return_m > 0 && rover_driven_m >= rover_return_m) rover_need_charge = true;
  in.needCharge = rover_need_charge;

  // Beacons: the radius of each channel filtered as BTx1Signal but not clamped (the re-arm distance is past the BTx
  // threshold); entry and exit by mower channel number with the follower's signals (BTx1Signal, BTx2Signal).
  for (int c = 0; c < 2; c++) {
    const float r = constrain(stmPacket.items.s1_btx_radius[c], 10.0f, 2000.0f);
    rover_btx_mm[c] = BTxFilterAlpha * r + (1 - BTxFilterAlpha) * rover_btx_mm[c];
  }
  in.beaconRange = fminf(rover_btx_mm[0], rover_btx_mm[1]) * 0.001f;
  const float btx_signal[3] = {(float)BTxThreshold + 1.0f, BTx1Signal, BTx2Signal};
  in.entryBeacon = rover_entry_btx != 0 && btx_signal[rover_entry_btx] < BTxThreshold;
  in.exitBeacon = rover_exit_btx != 0 && btx_signal[rover_exit_btx] < BTxThreshold;

  const float prev_w = rover_out.w;
  rover_out = roverAp.step(in);
  // A pivot ended: clear the rate loop integral, or it keeps the vehicle turning for seconds after the pivot.
  if (prev_w != 0.0f && rover_out.w == 0.0f && !rover_out.external) steeringRateReset();

  // Decision log: each bounce, check, join, reversal, beacon turn, dock, depart, Tx change and fault.
  static uint32_t dec_seen = 0;
  if (roverAp.dec.seq != dec_seen) {
    dec_seen = roverAp.dec.seq;
    const rover::Decision& d = roverAp.dec;
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
