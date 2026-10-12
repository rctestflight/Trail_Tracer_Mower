// SN65HVD230 CAN transceiver connected to the native ESP32-S2 TWAI controller
// P9 (GPIO36) - TXD
// P10 (GPIO38) - RXD

// MPU6050 I2C
// SDA - GPIO39
// SCL - GPIO40

// Motor Arming Switch - P11 (GPIO35)
// Buzzer - P8 (GPIO37)

#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "rt_prof.h"       // real-time path timing: STM32 frame -> control -> CAN (lib/rt_prof; bench "rt")
#include "frame_pid.h"     // the steering PIDs stepped once per STM32 frame (lib/frame_pid)
#include <driver/twai.h>

MPU6050 mpu(Wire);
bool mpu_ok = false;  // false on a board with no gyro; the gyro is logged only (the autopilot does not use it)
double gyro_heading_deg = 0.0;  // log layout 3: the integral of gyro z over every MPU sample, deg; zero at boot
void gyroHeadingUpdate() {
  static uint32_t last_us = 0;
  const uint32_t now_us = micros();
  if (last_us && mpu_ok) gyro_heading_deg += mpu.getGyroZ() * (double)(now_us - last_us) * 1e-6;
  last_us = now_us;
}
bool imu_halt = false;  // retired at the merge of f10cd31: the main-branch gyro lock keeps the motors off with no MPU; always false
#define SDA_PIN 39
#define SCL_PIN 40

bool gyro_ready = false;
void mpuReport(bool ok, unsigned status);  // main-branch MPU messages through the USB text queue and the log
uint32_t gyro_sample_ms = 0;

// Take one calibration sample per loop, leaving STM32 packet handling active.
// Keep the sensor stationary until the calibration-complete message appears.
static void serviceMpu() {
  enum class State { Init, Settling, Sampling, Ready, Failed };
  static State state = State::Init;
  static uint32_t started_ms = 0;
  static uint32_t last_sample_ms = 0;
  static uint16_t samples = 0;
  static float sums[6] = {};
  const uint32_t now = millis();

  if (state == State::Init) {
    Wire.begin(SDA_PIN, SCL_PIN);
    byte status = mpu.begin();
    if (status != 0) {
      mpuReport(false, status);  // USB text and event 7 (defined after the USB queue and the log)
      state = State::Failed;
      return;
    }
    mpu.setGyroOffsets(0, 0, 0);
    mpu.setAccOffsets(0, 0, 0);
    started_ms = millis();
    state = State::Settling;
    return;
  }
  if (state == State::Failed) return;
  if (state == State::Settling) {
    if (now - started_ms < 500) return;
    state = State::Sampling;
  }
  if (state == State::Sampling) {
    if (now - last_sample_ms < 1) return;
    last_sample_ms = now;
    mpu.fetchData();
    sums[0] += mpu.getAccX();
    sums[1] += mpu.getAccY();
    sums[2] += mpu.getAccZ() - 1.0f;
    sums[3] += mpu.getGyroX();
    sums[4] += mpu.getGyroY();
    sums[5] += mpu.getGyroZ();
    if (++samples == CALIB_OFFSET_NB_MES) {
      mpu.setAccOffsets(sums[0] / samples, sums[1] / samples, sums[2] / samples);
      mpu.setGyroOffsets(sums[3] / samples, sums[4] / samples, sums[5] / samples);
      state = State::Ready;
      // Wait for the first offset-corrected update before enabling motors.
      mpuReport(true, 0);
    }
    return;
  }
  if (now - last_sample_ms >= 1) {
    last_sample_ms = now;
    mpu.update();
    gyro_sample_ms = millis();
    rtprof::gyroSample();  // real-time path timing (lib/rt_prof)
    gyroHeadingUpdate();   // log layout 3: heading = integral of gyro z
    gyro_ready = isfinite(mpu.getGyroZ());
    mpu_ok = gyro_ready;  // the log reads the gyro while it is valid
  }
}
#include "wfrx_stm32_comms.h"
#include "rctf_wfrx_status_bits.h"
#include "rctf_wfrx_exp32.h"
#include "pwm_read_rmt.h"
#include "usb_msc_fat.h"
#include <WiFi.h>
#include <esp_wifi.h>  // bench RSSI experiment (promiscuous receive) and transmit power
#include <esp_now.h>
#include "esp_log.h"
#include "rover_autopilot.h"
#include "esp32-hal-tinyusb.h"  // usb_persist_restart(): restart into ROM download mode
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "usb_ota.h"    // OTA self-update over USB, with rollback (lib/usb_ota)
#include "usb_guard.h"  // no bootloader from DTR/RTS, USB transmit-stall re-enumeration (lib/usb_guard)
#include "monitor_role.h"  // USB command "monitor": this boot only, the same image runs as the PC monitor
#include "esp_partition.h"
#include "telemlog.h"       // numbered telemetry log: rings, live broadcast, backfill (monitor role)
#include "rover_records.h"  // record bodies: 100 Hz samples, status, events
#include "decimator.h"     // anti-alias filter and decimation of the samples
#include "stm_settings.h"  // STM32 commands: answer check, retry, read-back of the settings (lib/stm_settings)
#include "stm_v2.h"        // STM32 link version 2: 98 B frames, 8 B command frames with seq and CRC (lib/stm_v2)
#include <stdarg.h>
#include "cpu_prof.h"      // CPU share per FreeRTOS task and per loop part, by sampling (lib/cpu_prof; bench "cpu")

// USB text output that never blocks loop(). Serial.write() on the ESP32-S2 repeats until all
// bytes are in the 64-byte CDC transmit buffer while a host holds the port (DTR set), even with
// setTxTimeoutMs(0); a host that stops reading would then stop loop(). Every print goes into this
// queue instead (a print that does not fit is dropped and counted), and service() hands at most
// the free buffer space to Serial each loop().
// 0.5.2: back to Serial (the Arduino USB lock). 0.5.1 called tud_cdc_write() and
// tud_cdc_write_flush() from loop() without that lock, beside the Arduino USB task; after each
// warm restart (OTA end) the host disabled the port 1.3 s into enumeration and the board never
// came back on USB (bench test). A cold start worked.
class UsbOut : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* p, size_t n) override {
    if (n > sizeof(q_) - len_) { dropped++; return 0; }
    for (size_t i = 0; i < n; i++) q_[(head_ + len_ + i) % sizeof(q_)] = p[i];
    len_ += n;
    return n;
  }
  size_t room() const { return sizeof(q_) - len_; }
  void service() {
    int free = Serial.availableForWrite();  // 0 with no host (DTR off): nothing is handed over
    while (free > 0 && len_ > 0) {
      size_t n = min((size_t)free, min(len_, sizeof(q_) - head_));
      Serial.write(q_ + head_, n);
      usb_guard::progress();  // the buffer took bytes: no stall
      head_ = (head_ + n) % sizeof(q_);
      len_ -= n;
      free -= (int)n;
    }
  }
  void drain(uint32_t ms) {  // before a restart only: wait up to ms for the queue to go out
    const uint32_t t0 = millis();
    while (len_ > 0 && millis() - t0 < ms) { service(); delay(1); }
  }
  uint32_t dropped = 0;
 private:
  uint8_t q_[1792];  // 0.3.8: 256 B less (dram0_0_seg); update mode keeps the output small
  size_t head_ = 0, len_ = 0;
};
UsbOut usbOut;

// Bench "usbload <bytes_per_s> <seconds>": paced 100-byte lines on USB, to measure USB output stops
// (lib/usb_guard) against the output rate. The start and end lines carry the guard counters.
uint32_t usbload_bps = 0, usbload_t0 = 0, usbload_end = 0, usbload_seq = 0, usbload_skip = 0;
uint64_t usbload_sent = 0;
void usbLoadService(){
  if (!usbload_bps) return;
  const uint32_t now = millis();
  if ((int32_t)(now - usbload_end) >= 0) {
    usbload_bps = 0;
    usbOut.printf("BENCH usbload end lines=%lu skipped=%lu dropped=%lu stalls=%lu reconnects=%lu tx_events=%lu\n",
                  (unsigned long)usbload_seq, (unsigned long)usbload_skip, (unsigned long)usbOut.dropped,
                  (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts(), (unsigned long)usb_guard::tx_events());
    return;
  }
  const uint64_t due = (uint64_t)usbload_bps * (now - usbload_t0) / 1000;
  for (int i = 0; i < 20 && usbload_sent + 100 <= due; i++) {
    usbload_sent += 100;
    if (usbOut.room() < 200) { usbload_skip++; continue; }  // the queue is full: the line is counted, not sent
    char b[101];
    int n = snprintf(b, sizeof(b), "U %08lu %010lu ", (unsigned long)++usbload_seq, (unsigned long)now);
    while (n < 99) b[n++] = '.';
    b[99] = '\n'; b[100] = 0;
    usbOut.write((const uint8_t*)b, 100);
  }
}

static const char* TAG = "wfrx";
#ifdef BENCH_FAKE_IMU
#define ROVER_FW_VERSION "0.5.61+fakeimu"  // bench build: static IMU values (status flag bit 6)
#else
#define ROVER_FW_VERSION "0.5.61"  // 0.5.61: the autopilot full yaw rate is steering_rate_max_dps (pivot angles as planned through the gyro rate loop); 0.5.60: 0.5.59 + bench tools (radio slot, usbload, rt bench, stm stream, monitor promisc/quiet, usbstall diag); table CRC, fast beacon pow; monitor PC-gone 10 s; USB flush kick at 250 ms
#endif
#define RX_BOARD "RX_REVA"  // the hardware this build targets: Trail Tracer Rx board, revision A

// Rover autopilot (lib/rover_autopilot): bounce inside the perimeter, go home on low battery,
// follow the wire to the dock in rover_goal_cw direction, depart. Runs in drive mode 2 when
// rover_enabled is set in config.json; otherwise mode 2 is the plain wire follower.
rover::Autopilot roverAp;
bool rover_enabled = false;
double rover_seek_vbatt = 0.0;
int rover_seek_after_s = 0;
uint8_t rover_entry_btx = 0;
uint8_t rover_exit_btx = 0;
bool rover_active = false;          // mode 2 with the autopilot driving
bool rover_need_charge = false;     // latched until the next charge ends
uint32_t rover_last_charge_ms = 0;
uint32_t rover_last_packet_us = 0;
bool rover_was_docked = false;
bool rover_charge_ran = false;       // charge step 3 seen in this docking
// Stall stop: drive command over 15 % of full and the wheels under a quarter of it for 3 s (ODrive
// wheel speeds), or, without fresh wheel speeds, the stuck detector while the wire is in range.
#define ROVER_STALL_MS 3000
uint32_t rover_stall_since_ms = 0;
bool rover_stall_halted = false;
float rover_vbatt_filt = 0.0f;
rover::Output rover_out;
void roverStep();  // one autopilot step per STM32 packet; sets throttle and steering_output
void telemetryBegin();
void telemetrySample();
uint8_t telemetryApByte();
void telemetryService();
void telemetryEvent(uint16_t code, int32_t a, int32_t b, const char* text);
void telemetryUsbCommand(const char* line);
void flashTestCommand(const char* arg);
void flashTestService();
bool telemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len);
uint32_t telem_loop_max_us = 0;  // longest packet handling since the last status record

StmTxPacket stmPacket;  // Data container for UART buffer and individual values

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

// Green LED pin (Low = ON, high = OFF)
#define nLED GPIO_NUM_1

// GPIO from STM32 for new data interrupt
#define STM_nDATA_READY GPIO_NUM_8

//User button pin / BOOT pin
#define BUTTON_PIN GPIO_NUM_0

// PWM Inputs
#define PWM_IN_NUM 4
#define PWM_IN_1 GPIO_NUM_14
#define PWM_IN_2 GPIO_NUM_21
#define PWM_IN_3 GPIO_NUM_15
#define PWM_IN_4 GPIO_NUM_33

// PWM Outputs
#define PWM_OUT_1 GPIO_NUM_26
#define PWM_OUT_2 GPIO_NUM_34
#define PWM_OUT_3 GPIO_NUM_16
#define PWM_OUT_4 GPIO_NUM_13

// Spare Inputs
#define SPARE_IN_1 GPIO_NUM_9
#define SPARE_IN_2 GPIO_NUM_10

// Spare Outputs
#define SPARE_OUT_1 GPIO_NUM_11
#define SPARE_OUT_2 GPIO_NUM_12


// Spare I2C/SPI, but can be as GPIO too
#define SPARE_NSS       GPIO_NUM_38
#define SPARE_SCL_SCK   GPIO_NUM_36
#define SPARE_SDA_DOUT  GPIO_NUM_37
#define SPARE_DIN       GPIO_NUM_35

#define MOTOR_ARM_SWITCH_PIN GPIO_NUM_35  // Low forces every ODrive idle.
#define BUZZER_PIN GPIO_NUM_37  // Active-high buzzer.
#define ODRIVE_BUZZER_PERIOD_MS 2000
#define ODRIVE_BUZZER_PULSE_MS 100

// Native ESP32-S2 TWAI controller connected to a 3.3V SN65HVD230 transceiver.
#define TWAI_TX_PIN GPIO_NUM_36
#define TWAI_RX_PIN GPIO_NUM_38
#define CAN_BUS_BITRATE 1000000  // Must match every connected ODrive CAN baud rate.
#define ODRIVE_NODE_COUNT 6  // Node IDs 0-5

// ODrive CAN protocol command IDs (see ODrive CAN protocol docs)
enum OdriveCanCmd : uint8_t {
  ODRIVE_CMD_HEARTBEAT              = 0x001,
  ODRIVE_CMD_ESTOP                  = 0x002,
  ODRIVE_CMD_GET_MOTOR_ERROR        = 0x003,
  ODRIVE_CMD_GET_ERROR              = 0x003, // Micro firmware: active errors + disarm reason.
  ODRIVE_CMD_GET_ENCODER_ERROR      = 0x004,
  ODRIVE_CMD_SET_AXIS_STATE         = 0x007,
  ODRIVE_CMD_GET_ENCODER_ESTIMATES  = 0x009,
  ODRIVE_CMD_SET_CONTROLLER_MODE    = 0x00B,
  ODRIVE_CMD_SET_INPUT_POS          = 0x00C,
  ODRIVE_CMD_SET_INPUT_VEL          = 0x00D,
  ODRIVE_CMD_SET_INPUT_TORQUE       = 0x00E,
  ODRIVE_CMD_SET_LIMITS             = 0x00F,
  ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT = 0x017,
  ODRIVE_CMD_CLEAR_ERRORS           = 0x018,
};

// ODrive controller config values (from verified working CAN_Example_src)
#define ODRIVE_CONTROL_MODE_VELOCITY  2  // CONTROL_MODE_VELOCITY_CONTROL
#define ODRIVE_INPUT_MODE_PASSTHROUGH 1  // INPUT_MODE_PASSTHROUGH
#define ODRIVE_AXIS_STATE_IDLE        1
#define ODRIVE_AXIS_STATE_CLOSED_LOOP 8
#define ODRIVE_VEL_LIMIT_RPS          3.0f   // velocity limit [rev/s]
#define ODRIVE_CURRENT_LIMIT_A        6.0f   // current limit [A]

bool can_driver_installed = false;
bool can_init_ok = false;  // Peripheral running, not proof of an ODrive connection.

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
uint32_t odrive_output_millis = 0;
uint32_t odrive_axis_errors[ODRIVE_NODE_COUNT] = {};
bool odrive_heartbeat_received[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_heartbeat_millis[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_axis_states[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_status_byte5[ODRIVE_NODE_COUNT] = {};
uint8_t odrive_status_byte6[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_seen_errors[ODRIVE_NODE_COUNT] = {}; // Retain faults even after they clear.
uint32_t odrive_active_errors[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_disarm_reasons[ODRIVE_NODE_COUNT] = {};
bool odrive_error_received[ODRIVE_NODE_COUNT] = {};
uint32_t odrive_error_millis[ODRIVE_NODE_COUNT] = {};
float odrive_node0_vbus = 0.0f;
float odrive_node0_ibus = 0.0f;
bool odrive_node0_ibus_received = false;
bool odrive_node0_vbus_received = false;
uint32_t odrive_node0_vbus_millis = 0;

uint32_t odrive_encoder_poll_millis = 0;  // last time encoder estimates were requested
// Wheel speeds for the telemetry log: Get_Encoder_Estimates (pos, vel) from nodes 0 and 1, one
// per side. ODrive firmware 0.6 sends them by itself every 10 ms by default; a node that has
// been silent for ODRIVE_VEL_STALE_MS is asked by RTR, at most one request each 10 ms.
float odrive_vel_rps[ODRIVE_NODE_COUNT] = {};
float odrive_pos_rev[ODRIVE_NODE_COUNT] = {};  // encoder position, rev (Get_Encoder_Estimates; log layout 3)
uint32_t odrive_vel_millis[ODRIVE_NODE_COUNT] = {};
bool odrive_vel_received[ODRIVE_NODE_COUNT] = {};
#define ODRIVE_VEL_STALE_MS 60
#define ODRIVE_ENCODER_POLL_MS 500  // encoder estimate poll interval

// Main loop configuration
#define MAIN_LOOP_PERIOD 360000  // 1hr rollover
#define SERIAL1_FLUSH_MAX_BYTES 64  // Limit post-read drain to avoid long blocking stalls
#define SERIAL1_FLUSH_MAX_US 150  // Microsecond budget for post-read drain

// Button configuration
#define BUTTON_DEBOUNCE_PERIODS 3  // 0.03s
#define BUTTON_CLICK_MAX_LENGTH 40  // 0.4s
#define BUTTON_CAL_MIN_PRESS 200  // 2s
#define BUTTON_CAL_MAX_PRESS 400  // 4s
#define BUTTON_6S_PRESS 600  // 6s

// PWM OUT configuration
#define PWM_OUT_FREQUENCY 50  // Frequency (Hz) 50 is default
#define PWM_OUT_RESOLUTION_BITS 14  // 14-bits is maximum

// Charging Control
#define CHARGE_AUTO_ENABLE true  // bool to enable/disable automatic charging
int8_t charge_status = 0;  // charge status flag for state machine
uint32_t charge_start_millis = 0;
uint32_t last_charge_code = 0;
uint32_t last_status_bits = 0;
bool lapFlag = false;



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


// Steering ------------------------------------------------------
double target_x_pos = 0;  // target x distance from wire
double live_position = 0;
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
double stmFramePeriodS();  // the STM32 frame period in effect, s (defined with the DSP settings)

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

// PID Low pass filterint setup -----------------------------------------------------------------------------------
bool low_pass_enable = true;  // true = low pass active, false disables low-pass
float low_pass_cutoff = 5;  // Frequency of a 2nd order low-pass filter.  Smaller number = more filtering
float low_pass_alpha = 0.0;  // filter coefficient that's calculated from the cutoff value
float low_pass_beta = 0.0;  // filter coefficient that's calculated from the cutoff value
bool low_pass_compute = false;  // bool to only compute when PID loop updates
bool low_pass_first_cycle = true;
float low_pass_last_val_1 = 0.0;
float low_pass_last_val_2 = 0.0;

volatile uint32_t data_ready = 0;  // incremented by ISR each time STM32 asserts nDATA_READY
// Bench "radio slot": the radio part of loop() (all sends) runs only from radio_slot_us to radio_slot_us +
// RADIO_SLOT_LEN_US after the last nDATA_READY edge; -1 = no slot. radio_tx_hist: send-done times after the
// edge, in tenths of the frame period (10 = later than one period).
volatile uint32_t stm_edge_us = 0;
int32_t radio_slot_us = -1;
const uint32_t RADIO_SLOT_LEN_US = 1000;
uint32_t radio_tx_hist[11] = {};
uint32_t uart_overrun_count = 0;   // diagnostic counter, incremented on each overrun recovery
// Layout 2 (0.5.50): a late loop pass keeps the frames. The UART buffer (2048 B) holds 16 frames of 128 B; each pass
// reads one, in order, while 128 B or more wait. Frames stay aligned: each one has its CRC. A bad CRC drains the buffer
// and sets stm_v2_resync: the next frame counts only when exactly 128 B wait at an nDATA_READY edge. No command goes
// out while frames wait (stm_v2_backlog): one command per STM32 command window.
bool stm_v2_resync = true;
bool stm_v2_backlog = false;
uint32_t stm_backlog_frames = 0;   // frames read from the backlog (a late pass), counted
uint32_t crc_error_count = 0;      // diagnostic counter, incremented on each CRC mismatch
uint16_t last_computed_crc = 0;    // last CRC computed by the ESP32, for diagnostics
uint32_t data_sent = 0;  // flag that debug UART statements have been sent
uint32_t main_loop_counter;  // counter incremented on each new STM32 data packet
int32_t bttn_counter = 0;  // button counter for debounce logic
int32_t bttn_state = 0;  // debounced state of the user button
int32_t bttn_click_counter = 0;  // button counter for "click" logic
int32_t bttn_long_counter = 0;  // button counter for "long" press logic
int32_t bttn_3s_armed = 0;  // calibration press window armed
int32_t bttn_6s_state = 0;  // state of a 6s press


void receiveStmUart();  // ISR routine to set data ready flag
void flushSerial1RxBounded();  // Drain limited stale UART bytes without blocking the loop
static uint16_t crc16_ibm(const uint8_t *data, size_t len);  // CRC-16/IBM: poly 0x8005, init 0xFFFF, no reflection
int stmFrameLen();     // bytes of one STM32 frame for the link version (unknown: the v2 length, the shorter)
bool stmReadFrame();   // one STM32 frame (v1 or v2) into stmPacket; false: bad CRC or version (counted)
void stmV2Config();    // v2: lane inputs and detail formats to the link
void dspApply();       // v2: the DSP settings (config.json dsp_*) to the STM32, then run
void dspOnAnswer(const stmv2::Answer& a);  // v2: a DSP write answer -> g_dsp_eff
void stmCaptureService();  // bench: paced print of captured v2 frames
void stm32_boot_handshake();  // IAP hello/boot handshake — call once after Serial1.begin()
void stmFastFrameGuard();     // boot: a v2 STM32 with frames faster than 100 a second gets the 10 ms period
size_t serial1Drain();        // the STM32 UART input bytes there now, read in blocks; returns the count
int stm32_iap_boot(int hello_tries, char* hello_text, size_t text_len, uint8_t* hello_len);  // HELLO, BOOT, first nDATA_READY
void writePwmMicros(uint8_t ch, uint32_t micros); // Write PWM output pulse width in microseconds
void mainLoopCounter();  // increment main loop counter
void buttonLogic();  // handle button events

// USB MSC / config management
DeviceConfig g_deviceConfig;                     // Runtime config struct (MSC + NVS backed)
void applyDeviceConfig(const DeviceConfig& cfg); // Apply cfg values to firmware globals

void buttonClickEvent();  // Called on button release if the press time is considered a "click"
void button3sPressEvent();  // Called on button release after a 2s to 4s press
void button6sPressEvent();  // Called on a 6-second button press

void chargeControl();
uint16_t driveMode();
int BTxFollow(int base_throttle, double &btx_throttle, bool &btx_throttle_init, bool &btx_follow_active, float btx_signal);

/*
Commands to the STM32 (0.5.25: lib/stm_settings). Use the espPacketFifo_addCmdxxx functions to queue a
command, as before. espPacketFifo_sendCmd runs once per valid STM32 packet: it reads the STM32 answer
in the packet (CMD_ACK, rid, response), then sends at most one command. The link sends the next
command only after the answer to the one before, or after a timeout; it sends a command with no
answer again (not 3, 4, 5), counts each failure, reads back the five settings (1, 2, 6, 7, 8) one each
2 s, and sends the five again after a wrong read-back or a gap in the packets. Events: 18.
*/
stmset::Link stmLink;
// STM32 link version: learned in-band from the first good frame. At the nDATA_READY edge the whole
// frame is buffered, so its length tells the version: 122 B = v1 (all shipped STM32 apps), 128 B with
// proto_ver 2 and a good CRC = Trail Tracer v2.0 link, layout 2 (include/rctf_wfrx_comms_v2.h, app
// 2.0.2 and later). Layout 1 (98 B, app 2.0.0/2.0.1 prototypes) is no longer read. 50 bad frames in a
// row make the version unknown again (for example after an STM32 app update through the bridge).
stmv2::Decoder stmDec;
// Bench "rt bench" (rt task draft, step 1): CPU cycles of the live v2 frame read and decode (mean, max since the
// last report), then 1000 decodes of the last good frame in a copy of the decoder, then single float operations.
uint8_t rt_last_frame[STM_FRAME_LEN];
bool rt_last_ok = false;
uint64_t rt_read_cyc = 0, rt_dec_cyc = 0;
uint32_t rt_read_max = 0, rt_dec_max = 0, rt_bench_n = 0;
stmv2::Answer stmAns;
uint8_t stm_link_ver = 0;          // 0 not known, 1 = v1, 2 = v2.0 layout 2
uint32_t stm_bad_frames_run = 0;
bool stm_v2_cfg_sent = false;      // lanes and detail formats sent after v2 was learned
uint8_t stm_v2_lanes = 0x02;       // CID 13: lane 1 = sensor 1, lane 2 = sensor 2 (v1 behaviour)
bool stm_v2_save_ok = false;       // layout 2: a CID 3 goes out only after a centre calibration
// Layout 2 (app 2.0.6+): the STM32 boots idle; CID 1 = 1 starts acquisition, after the configuration.
// In the queue the run command has its own ID (STM_RUN_CMD): CID 1 there is the v1 channel select.
#define STM_RUN_CMD 126
bool stm_v2_autorun = true;        // send run after the configuration and each STM32 reset ('stm run 0' stops it)
uint32_t stm_v2_run_ms = 0;        // millis() of the last run command queued
uint32_t stm_ans_print_until = 0;  // bench: print each v2 answer until this millis()
// Bench (performance plan): raw v2 frames into RAM ('stm cap <n>', n <= 300, 128 B each, heap), printed
// later at the pace the USB queue takes ('stm dump'). Command round trip in frames: the frame seq when a
// command goes out against the frame seq of its answer (stm v2 'rtt').
uint8_t* stm_cap = nullptr;
uint16_t stm_cap_n = 0, stm_cap_want = 0, stm_cap_out = 0;
// Bench "stm stream <s>" (coded-Tx test): per good v2 frame one line with frame bytes 1 (seq), 14-19 and
// 38-43 (bin 0 coil 1 and coil 2 of result 1, CID 40 raw mode), 80-83 (t0) as hex, and the PGA bits; about 17 KB/s
// at 2354 us frames. A line that does not fit the USB queue is dropped (counted in the end line).
uint32_t stm_stream_end = 0, stm_stream_n = 0, stm_stream_drop = 0;
void stmStreamFrame(const uint8_t* f){
  static const char hx[] = "0123456789abcdef";
  static const uint8_t pos[] = {1, 14, 15, 16, 17, 18, 19, 38, 39, 40, 41, 42, 43, 80, 81, 82, 83};
  char b[2 + 2 * sizeof(pos) + 4];
  int m = 0;
  b[m++] = 'F';
  for (uint8_t p : pos) { b[m++] = hx[f[p] >> 4]; b[m++] = hx[f[p] & 15]; }
  b[m++] = ' ';
  b[m++] = (char)('0' + ((stmDec.status_bits >> 2) & 3));
  b[m++] = '\n';
  if (usbOut.room() < (size_t)m + 64) { stm_stream_drop++; return; }
  usbOut.write((const uint8_t*)b, m);
  stm_stream_n++;
}
// Bench (noise floor, the Rx as a spectrum analyser): 'stm acc <n>' sums the coil values of every bin, both
// lanes and both coils over n frames on the ESP32 (no frame lost to the USB pace), then prints per channel
// the rms, the mean of |v| and the largest |v| in int24 LSB (magnitude 2^-24). mean/rms: 0.886 for noise
// (Rayleigh), near 1 for a steady tone; max/rms shows impulses.
uint32_t stm_acc_want = 0, stm_acc_n = 0, stm_acc_ovl = 0, stm_acc_lost = 0;
uint8_t stm_acc_seq = 0;
double stm_acc_sq[WFRX_RESULTS][WFRX_BINS][2], stm_acc_abs[WFRX_RESULTS][WFRX_BINS][2];
uint32_t stm_acc_max[WFRX_RESULTS][WFRX_BINS][2];
// Bench (coil check, live): 'stm coils <ms>' prints the mean |v| of each coil of both lanes, bin 0, over each
// <ms> period (0 = off), in int24 LSB (magnitude 2^-24): STMCOIL ms=<millis> n=<frames> s1c1 s1c2 s2c1 s2c2.
// 0.5.48 (telemetry program request): also all 8 bins, STMCOIL8 ms= n= hz=<f1>,..,<f8> <32 values> (bin 0
// s1c1 s1c2 s2c1 s2c2, then bin 1 ... bin 7); a sensor that no lane reads gives 0. hz: the default bin frequencies,
// updated from each CID 20 answer (0 = bin off). 0.5.49: each STMCOIL8 value carries a sign: sign(mean v) x mean |v|,
// with v the signed int24 coil value (the STM32 sign: its quadrant state per coil). |value| stays mean |v| (for dBFS).
uint32_t stm_coil_ms = 0, stm_coil_t0 = 0, stm_coil_n = 0;
double stm_coil_sum[2][WFRX_BINS][2];   // [sensor][bin][coil] sum of |v|
double stm_coil_ssum[2][WFRX_BINS][2];  // sum of signed v
float stm_bin_hz[WFRX_BINS] = {53333.3f, 55315.5f, 51323.2f, 57347.7f, 49344.6f, 62500.0f, 46875.0f, 0.0f};
void stmCoilFrame(){
  bool done[2] = {false, false};
  for (unsigned k = 0; k < WFRX_RESULTS; k++) {
    const unsigned sn = (stmDec.lanes >> k) & 1u;  // the sensor this lane reads
    if (done[sn]) continue;  // both lanes on one sensor: lane 1 counts
    done[sn] = true;
    for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
      const int32_t v1 = wfrx_int24_get(stmDec.coil[k][bn].v1), v2 = wfrx_int24_get(stmDec.coil[k][bn].v2);
      stm_coil_sum[sn][bn][0] += v1 < 0 ? -v1 : v1;
      stm_coil_sum[sn][bn][1] += v2 < 0 ? -v2 : v2;
      stm_coil_ssum[sn][bn][0] += v1;
      stm_coil_ssum[sn][bn][1] += v2;
    }
  }
  stm_coil_n++;
  const uint32_t now = millis();
  if (now - stm_coil_t0 < stm_coil_ms) return;
  const double n = (double)stm_coil_n;
  usbOut.printf("STMCOIL ms=%lu n=%lu %.0f %.0f %.0f %.0f\n", (unsigned long)now, (unsigned long)stm_coil_n,
                stm_coil_sum[0][0][0] / n, stm_coil_sum[0][0][1] / n, stm_coil_sum[1][0][0] / n, stm_coil_sum[1][0][1] / n);
  char line[480];
  int m = snprintf(line, sizeof(line), "STMCOIL8 ms=%lu n=%lu hz=", (unsigned long)now, (unsigned long)stm_coil_n);
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) m += snprintf(line + m, sizeof(line) - m, bn ? ",%.1f" : "%.1f", stm_bin_hz[bn]);
  for (unsigned bn = 0; bn < WFRX_BINS; bn++)
    for (unsigned sn = 0; sn < 2; sn++)
      for (unsigned c = 0; c < 2; c++)
        m += snprintf(line + m, sizeof(line) - m, " %.0f", (stm_coil_ssum[sn][bn][c] < 0 ? -1.0 : 1.0) * stm_coil_sum[sn][bn][c] / n);
  usbOut.printf("%s\n", line);
  memset(stm_coil_sum, 0, sizeof(stm_coil_sum));
  memset(stm_coil_ssum, 0, sizeof(stm_coil_ssum));
  stm_coil_n = 0;
  stm_coil_t0 = now;
}
// Channel scan: with the Tx off, the user drives in
// manual mode over the full throttle range and the ESP32 ranks the bins by interference. Per frame and bin: level
// I = sqrt(S1^2 + S2^2) of each lane (int24 S units), the larger lane counts; sign and bin status are ignored. One
// histogram per bin in 1.5 dB steps (heap, 3 kB, no samples). 'scan start' / 'scan stop' / 'scan' (report) /
// 'scan wire <S>' (wire level at the farthest working distance; default the lost threshold 1/1200 x 2^24 = 13981).
// Rank by p99, then p99.9, then the median; margin = 20 log10(S_wire / p99): >= 20 dB clean, 10-20 usable, < 10 reject.
// Candidates are the Tx channels (bins 0-4); bins 5-7 are information only. Valid for the frame period of the scan.
#define SCAN_STEPS 96   // 1.5 dB steps from I = 1 (0 dB) to 144 dB
uint32_t (*scan_hist)[SCAN_STEPS] = nullptr;  // [WFRX_BINS][SCAN_STEPS]
bool scan_on = false;
uint32_t scan_n = 0, scan_t0 = 0;
float scan_wire = 13981.0f;
float scan_max[WFRX_BINS], scan_max_drive[WFRX_BINS];
void scanFrame(){
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
    float lv = 0;
    for (unsigned k = 0; k < WFRX_RESULTS; k++) {
      const float a = (float)wfrx_int24_get(stmDec.coil[k][bn].v1), b = (float)wfrx_int24_get(stmDec.coil[k][bn].v2);
      const float l = sqrtf(a * a + b * b);
      if (l > lv) lv = l;
    }
    int s = lv < 1.0f ? 0 : (int)(20.0f * log10f(lv) / 1.5f);
    scan_hist[bn][s < 0 ? 0 : s >= SCAN_STEPS ? SCAN_STEPS - 1 : s]++;
    if (lv > scan_max[bn]) { scan_max[bn] = lv; scan_max_drive[bn] = odrive_drive_command; }
  }
  scan_n++;
}
float scanPct(unsigned bn, float q){  // level (upper edge of the step) that a fraction q of the frames does not exceed
  const uint32_t want = (uint32_t)ceilf(q * scan_n);
  uint32_t c = 0;
  for (int s = 0; s < SCAN_STEPS; s++) { c += scan_hist[bn][s]; if (c >= want && c) return powf(10.0f, (s + 1) * 1.5f / 20.0f); }
  return powf(10.0f, SCAN_STEPS * 1.5f / 20.0f);
}
void scanReport(){
  if (!scan_hist || !scan_n) { usbOut.println("SCAN no data: 'scan start' with the Tx off, then drive"); return; }
  static const float f_def[WFRX_BINS] = {53333.3f, 55315.5f, 51323.2f, 57347.7f, 49344.6f, 62500.0f, 46875.0f, 0.0f};
  usbOut.printf("SCAN n=%lu s=%lu wire=%.0f period: as set; levels in S units (int24); margin = 20 log10(wire / level)\n",
                (unsigned long)scan_n, (unsigned long)((millis() - scan_t0) / 1000), scan_wire);
  int best = -1;
  float best_p99 = 0, best_p999 = 0, best_med = 0;
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
    const float med = scanPct(bn, 0.5f), p99 = scanPct(bn, 0.99f), p999 = scanPct(bn, 0.999f);
    const float m99 = 20.0f * log10f(scan_wire / p99), mmed = 20.0f * log10f(scan_wire / med);
    usbOut.printf("SCAN bin %u %s%.1f Hz: median %.0f p99 %.0f p99.9 %.0f max %.0f (drive %.2f) margin p99 %.1f dB median %.1f dB %s\n",
                  bn, bn < 5 ? "CH" : "info ", f_def[bn], med, p99, p999, scan_max[bn], scan_max_drive[bn], m99, mmed,
                  m99 >= 20 ? "clean" : m99 >= 10 ? "usable" : "reject");
    if (bn < 5 && (best < 0 || p99 < best_p99 || (p99 == best_p99 && (p999 < best_p999 || (p999 == best_p999 && med < best_med))))) {
      best = (int)bn; best_p99 = p99; best_p999 = p999; best_med = med;
    }
  }
  usbOut.printf("SCAN recommend Tx CH %d (bin %d): p99 margin %.1f dB%s\n", best + 1, best, 20.0f * log10f(scan_wire / best_p99),
                scan_n < 3000 ? "; fewer than 3000 frames: drive longer over idle, part and full throttle and braking" : "");
  // BTx (beacon) transmitters: the candidates are bins 5 and 6 (62.5 and 46.875 kHz)
  const unsigned bb = scanPct(5, 0.99f) <= scanPct(6, 0.99f) ? 5 : 6;
  usbOut.printf("SCAN recommend BTx bin %u (%.1f Hz): p99 margin %.1f dB\n", bb, f_def[bb], 20.0f * log10f(scan_wire / scanPct(bb, 0.99f)));
}
void stmAccFrame(){
  if (stm_acc_n && (uint8_t)(stmDec.lastSeq() - stm_acc_seq) != 1) stm_acc_lost += (uint8_t)(stmDec.lastSeq() - stm_acc_seq - 1);
  stm_acc_seq = stmDec.lastSeq();
  if (stmDec.status_bits & ((1u << 6) | (1u << 11))) stm_acc_ovl++;
  for (unsigned k = 0; k < WFRX_RESULTS; k++)
    for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
      const int32_t v[2] = {wfrx_int24_get(stmDec.coil[k][bn].v1), wfrx_int24_get(stmDec.coil[k][bn].v2)};
      for (int c = 0; c < 2; c++) {
        const uint32_t a = (uint32_t)(v[c] < 0 ? -v[c] : v[c]);
        stm_acc_sq[k][bn][c] += (double)a * a;
        stm_acc_abs[k][bn][c] += a;
        if (a > stm_acc_max[k][bn][c]) stm_acc_max[k][bn][c] = a;
      }
    }
  if (++stm_acc_n < stm_acc_want) return;
  stm_acc_want = 0;
  usbOut.printf("STMACC n=%lu lost=%lu ovl=%lu\n", (unsigned long)stm_acc_n, (unsigned long)stm_acc_lost, (unsigned long)stm_acc_ovl);
  for (unsigned k = 0; k < WFRX_RESULTS; k++)
    for (int c = 0; c < 2; c++) {
      char line[400];
      int m = snprintf(line, sizeof(line), "STMACC lane%u coil%d", k + 1, c + 1);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++)
        m += snprintf(line + m, sizeof(line) - m, " %.2f,%.2f,%lu", sqrt(stm_acc_sq[k][bn][c] / stm_acc_n), stm_acc_abs[k][bn][c] / stm_acc_n,
                      (unsigned long)stm_acc_max[k][bn][c]);
      usbOut.printf("%s\n", line);
    }
}
bool stm_cap_dump = false;
bool stm_rtt_wait = false;
uint8_t stm_rtt_cmd_seq = 0, stm_rtt_frame_seq = 0;
uint32_t stm_rtt_hist[16] = {};  // [d] = answers d frames after the send (15 = 15 or more)
uint8_t stm_dup_frame[ESP_FRAME_LEN];  // bench 'stm dup': one v2 command frame sent twice, 2 frames apart
int stm_dup_step = 0;               // 0 off; 1 send; 2, 3 wait; 4 send again
uint32_t stm_last_packet_ms = 0;   // millis() of the last valid STM32 packet
bool stm_packet_seen = false;      // a valid STM32 packet this boot
bool stm_rom_used = false;         // "stm rom" this boot: no HELLO/BOOT recovery (a ROM loader may listen)
uint8_t stm_recover_tries = 0;     // HELLO/BOOT recoveries in this packet gap
uint32_t stm_recover_ms = 0;
uint32_t stm_iap_boots = 0;        // HELLO/BOOT recoveries that started the STM32 application
uint32_t stm_iap_fails = 0;

// For cid, don't forget to do (cid | 0x80000000) for write commands
void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd);  // Add a command with a int32 payload to the queue
void espPacketFifo_addCmdFloat(uint32_t cid, float cmd);  //  add a command with a float payload to the queue
void espPacketFifo_sendCmd();  // Read the answer in this packet, then send up to one command; call after each valid STM32 packet
void stmLinkEvents();  // link events to the vehicle log (event 18) and USB
void stmRecoveryService();  // STM32 silent after packets: HELLO/BOOT again (an STM32 that restarted waits in its boot checker)

// Signal strength of one coil: DFT magnitude in counts (before the STM32 gain calibration).
// sensor: 0 or 1. coil: 0 or 1.
float coilStrength(const StmTxPacket& p, uint8_t sensor, uint8_t coil) {
  const float* c = &p.items.coil_data[sensor][coil * 2]; // re, im
  return hypotf(c[0], c[1]);
}

// ESP-NOW Stuff ---------------------------------------------------------------------
void broadcastTelemetry();
void telemetryFifo_addMessage(const String &message);
void telemetryFifo_sendMessage();

// Telemetry FIFO queue for ESP-NOW messages (prevents flooding)
#define TELEMETRY_FIFO_LENGTH 10
typedef struct {
  String message;
} TelemetryMessage;
TelemetryMessage telemetryFifo[TELEMETRY_FIFO_LENGTH];
uint32_t telemetry_fifo_head = 0;
uint32_t telemetry_fifo_tail = 0;
uint32_t lastTelemetrySendTime = 0;  // Track last send time for 100ms throttling

uint32_t brodcastMillis = 0;
uint32_t telemetryBrodcastMillis = 0;

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) // Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

// ESP-NOW bench link (protocol agreed with the monitor role, little-endian, byte 0 = type):
//   0x01 THRU  u8 type, u32 seq, u32 sender_us, zero padding to the test size (the rover broadcasts)
//   0x02 PING  u8 type, u32 seq, u32 rxb_us (the monitor sends it to this board)
//   0x03 PONG  the PING bytes with byte 0 set to 0x03, sent back to the PING's source
// The callbacks run in the Wi-Fi task at priority 23 and interrupt loop(): they only copy
// and count. loop() sends the PONG and the THRU frames (espnowService()).
#define ESPNOW_TYPE_THRU 0x01
#define ESPNOW_TYPE_PING 0x02
#define ESPNOW_TYPE_PONG 0x03
bool espnow_started = false;
volatile uint32_t espnow_bad_key = 0;  // CMD or RESTART frames refused for a wrong or missing fleet key
bool espnow_failed = false;  // esp_now_init() failed once: not tried again
// Startup times for the USB "boot" command (the boot lines go out before a PC opens the port).
uint32_t boot_setup_ms = 0, boot_first_packet_ms = 0, boot_espnow_ms = 0;
volatile bool espnow_ping_pending = false;
uint8_t espnow_ping_mac[6];
uint8_t espnow_ping_buf[16];
volatile uint8_t espnow_ping_len = 0;
volatile uint32_t espnow_rx_count = 0, espnow_ping_count = 0, espnow_ping_missed = 0;
volatile uint32_t espnow_tx_ok = 0, espnow_tx_fail = 0;
uint32_t espnow_send_err = 0, espnow_pong_count = 0;
bool espnow_thru_active = false;
uint32_t espnow_thru_period_us = 0, espnow_thru_size = 0, espnow_thru_left = 0, espnow_thru_seq = 0, espnow_thru_last_us = 0;
bool bench_timing_print = false;
bool bench_erase_block = false;  // bench "erase off": no flash ring erase (throughput tests without the erase losses)
bool bench_sim_armed = false;    // bench "armed sim on": the erase rule treats the motors as armed (simulated run, no ODrive)
bool update_mode = false;        // from "ota begin": loop() serves only USB and the update (see loop())
bool monitor_role_on = false;    // this boot runs the monitor role: monitor_role::loop(), no vehicle code
// USB command "monitor": the next boot only runs the monitor role (PC-tethered monitor role). The flag
// is in RTC memory and is cleared at once at boot, so every board starts as a vehicle, and any later
// restart, crash or power cycle returns it to the vehicle role. The PC side sends "monitor" again.
static RTC_NOINIT_ATTR uint32_t monitor_once;
constexpr uint32_t MONITOR_ONCE_MAGIC = 0x4D4F4E31u;  // "MON1"
uint32_t update_last_ms = 0;     // millis() of the last "ota" line
uint8_t espnow_thru_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Bench loss simulation (USB "loss <tx_pct> <rx_pct>", 0 at boot): the rover drops this share of
// its telemetry sends (live, INFO and backfill) and of the frames it receives (REQ, CMD, PING).
volatile uint8_t sim_loss_tx = 0, sim_loss_rx = 0;
volatile uint32_t sim_dropped_tx = 0, sim_dropped_rx = 0;

// Config commands (CMD 0x10 / ACK 0x11) and restarts (0x7F "RST!" / "DLD!").
#define ESPNOW_TYPE_CMD     0x10
#define ESPNOW_TYPE_ACK     0x11
#define ESPNOW_TYPE_STOP    0x12  // u8 type, u16 seq, u32 fleet key: stop at once (docs/espnow_protocol.md)
#define SETTINGS_DEF_VERSION 6  // docs/settings_def.json version (6: DSP settings dsp_*, groups for users)
#define ESPNOW_TYPE_RESTART 0x7F
#define ESPNOW_TYPE_MON     0x44  // monitor broadcast, once a second: u8 type, u8 version, u32 monitor ms, u64 PC unix ms (0 = unknown), u8 n
// The last MON frame (set in the Wi-Fi task, read in loop()).
volatile uint32_t mon_rx_ms = 0;       // millis() at the last MON frame; 0 = none since boot
volatile uint64_t mon_rx_unix_ms = 0;  // its PC time; 0 = the monitor has no PC time
uint8_t mon_mac[6] = {0};
volatile bool mon_new = false;
bool pc_time_known = false;    // a MON frame brought PC time
int64_t pc_offset_ms = 0;      // PC unix ms - millis()
uint32_t mon_blip_until = 0;   // LED inverted until this millis() (100 ms per MON frame)
bool mon_blip_on = false;      // the LED pin is inverted now
uint8_t mon_blip_level = 0;    // the level that the inversion wrote
#define ESPNOW_CMD_TEXT_MAX 200
volatile bool espnow_cmd_pending = false;
uint8_t espnow_cmd_mac[6];
uint16_t espnow_cmd_seq = 0;
char espnow_cmd_text[ESPNOW_CMD_TEXT_MAX + 1];
volatile bool espnow_busy_pending = false;   // a CMD arrived while one was in hand: answer "busy"
uint8_t espnow_busy_mac[6];
uint16_t espnow_busy_seq = 0;
volatile uint8_t espnow_restart_request = 0;  // 1 normal restart, 2 restart into ROM download mode
volatile int espnow_list_index = -1;          // >= 0 while a "list" answer is in progress
volatile bool espnow_stop_request = false;    // STOP frame received; loop() stops at its next pass
uint8_t espnow_stop_mac[6];
uint16_t espnow_stop_seq = 0;
uint32_t espnow_list_ms = 0;

// ESP-NOW runs for the text status (espnow_enabled) or for the telemetry log (telemetry_enabled,
// on by default). Live frames are broadcast and nothing waits for a peer, so a monitor that is
// off costs the mower nothing.
// Crash-loop guard: after 3 crash resets in a row (panic or watchdog), the firmware starts in a
// safe mode with no telemetry log and no ESP-NOW. USB, settings and "dld" stay, so a board whose
// new code crashes at boot can still be reflashed without the BOOT button. The counter is in RTC
// memory: a power-on clears it, and 60 s of running clears it.
RTC_NOINIT_ATTR uint32_t crash_magic;
RTC_NOINIT_ATTR uint32_t crash_count;
// The last state before a reset, kept through any reset except power loss: the next boot writes it
// to the vehicle log (event 14) and to USB. stage: the part of loop() that ran last.
enum : uint8_t { ST_SETUP = 0, ST_USB = 1, ST_UPDATE = 2, ST_PACKET = 3, ST_RADIO = 4, ST_LOG = 5, ST_BENCH = 6, ST_IDLE = 7 };
const char* const STAGE_NAMES[] = {"setup", "usb", "update", "packet", "radio", "log", "bench", "idle"};
struct LastState { uint32_t magic, uptime_ms, heap_min; uint8_t stage, drive_mode, ap_state, section; };  // section: the last mark()
RTC_NOINIT_ATTR LastState last_state;
const uint32_t LAST_STATE_MAGIC = 0x4C535431;  // "LST1"
inline void stage(uint8_t st) { last_state.stage = st; last_state.uptime_ms = millis(); }
// Loop parts for the CPU profiler (cpuprof::section; bench "cpu"). A part runs until the next mark.
enum : uint8_t { SEC_USB = 0, SEC_UPDATE, SEC_BRIDGE, SEC_MONITOR, SEC_DRIVE, SEC_MSC, SEC_PACKET, SEC_ROVER, SEC_SAMPLE,
                 SEC_STMCMD, SEC_RADIO, SEC_LOG, SEC_BENCH, SEC_DEBUG, SEC_MPU, SEC_N };
// One mark per loop part: the CPU profiler (shares) and the real-time path timing (wall time per part and pass).
inline void mark(uint8_t s) { cpuprof::section(s); rtprof::part(s); last_state.section = s; }
const char* const SEC_NAMES[SEC_N] = {"usb", "update", "bridge", "monitor", "drive", "msc", "packet", "rover", "sample",
                                      "stmcmd", "radio", "log", "bench", "debug", "mpu"};
bool safe_mode = false;
bool espnowWanted(){ return !safe_mode && (g_deviceConfig.espnow_enabled || g_deviceConfig.telemetry_enabled); }

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
// Called when data is received (Wi-Fi task): copy a PING, CMD or RESTART for loop(), nothing else
{
  if (sim_loss_rx && esp_random() % 100 < sim_loss_rx) { sim_dropped_rx++; return; }  // bench loss simulation
  espnow_rx_count++;
  telemlog_on_frame(macAddr, data, dataLen);  // REQ 0x41 only; it ignores every other frame
  // Commands and restarts carry the fleet key (cmd_key). The frame key must equal cmd_key; cmd_key 0
  // is a blank password and accepts key 0 (design choice 2026-10-10). Any MAC may send them: every Rx
  // board has its own MAC, so a dongle or PC can change without a change on the vehicles.
  const uint32_t key = (uint32_t)g_deviceConfig.cmd_key;
  if (dataLen >= 1 && (data[0] == ESPNOW_TYPE_RESTART || data[0] == ESPNOW_TYPE_CMD || data[0] == ESPNOW_TYPE_STOP)) {
    const int at = data[0] == ESPNOW_TYPE_RESTART ? 5 : 3;
    uint32_t got = 0;
    if (dataLen >= at + 4) memcpy(&got, &data[at], 4);
    if (dataLen < at + 4 || got != key) { espnow_bad_key++; return; }  // no answer, no effect
  }
  if (dataLen >= 9 && data[0] == ESPNOW_TYPE_RESTART) {
    if (!memcmp(&data[1], "RST!", 4)) espnow_restart_request = 1;
    else if (!memcmp(&data[1], "DLD!", 4)) espnow_restart_request = 2;
    return;
  }
  if (dataLen >= 7 && data[0] == ESPNOW_TYPE_STOP) {  // never busy, never cached: a flag for loop()
    memcpy(espnow_stop_mac, macAddr, 6);
    espnow_stop_seq = (uint16_t)(data[1] | (data[2] << 8));
    espnow_stop_request = true;
    return;
  }
  if (dataLen >= 7 && data[0] == ESPNOW_TYPE_CMD) {
    const uint16_t seq = (uint16_t)(data[1] | (data[2] << 8));
    if (espnow_cmd_pending || espnow_list_index >= 0) {  // one CMD at a time
      memcpy(espnow_busy_mac, macAddr, 6);
      espnow_busy_seq = seq;
      espnow_busy_pending = true;
      return;
    }
    const int n = min(dataLen - 7, ESPNOW_CMD_TEXT_MAX);
    memcpy(espnow_cmd_mac, macAddr, 6);
    espnow_cmd_seq = seq;
    memcpy(espnow_cmd_text, &data[7], n);
    espnow_cmd_text[n] = 0;
    espnow_cmd_pending = true;
    return;
  }
  if (dataLen >= 15 && data[0] == ESPNOW_TYPE_MON) {  // a monitor hears this radio channel; PC time
    memcpy(mon_mac, macAddr, 6);
    uint64_t u = 0;
    memcpy(&u, &data[6], 8);
    mon_rx_unix_ms = u;
    mon_rx_ms = millis() ? millis() : 1;
    mon_new = true;
    return;
  }
  if (dataLen >= 9 && dataLen <= (int)sizeof(espnow_ping_buf) && data[0] == ESPNOW_TYPE_PING) {
    if (espnow_ping_pending) {  // loop() has not answered the previous PING yet
      espnow_ping_missed++;
      return;
    }
    memcpy(espnow_ping_mac, macAddr, 6);
    memcpy(espnow_ping_buf, data, dataLen);
    espnow_ping_len = dataLen;
    espnow_ping_count++;
    espnow_ping_pending = true;
  }
}

void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) // Called when data is sent
{
  if (status == ESP_NOW_SEND_SUCCESS) espnow_tx_ok++;
  else espnow_tx_fail++;
  const uint32_t per_us = (uint32_t)(stmFramePeriodS() * 1e6);
  const uint32_t dt = (uint32_t)esp_timer_get_time() - stm_edge_us;
  radio_tx_hist[per_us ? min<uint32_t>(10, dt * 10 / per_us) : 10]++;
}

void broadcast(const String &message) {
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);

  // Only add the peer if it's not already added
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // Print results to serial monitor
  if (result != ESP_OK) {
    usbOut.println("ESP-NOW send failed");
  }
}

// Bench radio slot (see radio_slot_us): true when the radio part of loop() may run now.
bool radioSlotOpen(){
  if (radio_slot_us < 0) return true;
  const uint32_t dt = micros() - stm_edge_us;
  return dt >= (uint32_t)radio_slot_us && dt < (uint32_t)radio_slot_us + RADIO_SLOT_LEN_US;
}

// Start ESP-NOW once; also called from loop() so a config.json change takes effect without a reset.
// Radio transmit power (setting espnow_tx_power_dbm), for every frame of this radio. Wi-Fi must run.
void applyTxPower(){
  wifi_mode_t m;
  if (esp_wifi_get_mode(&m) != ESP_OK || m == WIFI_MODE_NULL) return;  // no radio yet (both roles)
  esp_wifi_set_max_tx_power((int8_t)lround(g_deviceConfig.espnow_tx_power_dbm * 4.0));
}

// MON frames (docs/espnow_protocol.md): event 17 when a monitor is heard or not heard for 5 s;
// event 16 when the PC time first comes, or when it differs from the last one by more than 1 s.
uint8_t monitorAgeS(){
  const uint32_t r = mon_rx_ms;
  if (!r) return 255;
  const uint32_t s = (millis() - r) / 1000;
  return s > 254 ? 255 : (uint8_t)s;
}
void monitorLinkService(){
  static bool heard = false;
  if (mon_new) {
    mon_new = false;
    const uint32_t r = mon_rx_ms;
    mon_blip_until = millis() + 100;  // green LED: a 100 ms inversion per MON frame (a monitor hears this rover)
    const uint64_t u = mon_rx_unix_ms;
    char t[48];
    if (!heard) {
      heard = true;
      snprintf(t, sizeof(t), "monitor %02x:%02x:%02x:%02x:%02x:%02x", mon_mac[0], mon_mac[1], mon_mac[2], mon_mac[3], mon_mac[4], mon_mac[5]);
      telemetryEvent(rec::EV_MONITOR, 1, 0, t);
      usbOut.printf("MONITOR heard: %s\n", t);
    }
    if (u) {
      const int64_t off = (int64_t)u - (int64_t)r;
      if (!pc_time_known || llabs(off - pc_offset_ms) > 1000) {
        pc_time_known = true;
        pc_offset_ms = off;
        snprintf(t, sizeof(t), "unix_ms=%llu", (unsigned long long)u);
        telemetryEvent(rec::EV_TIME, (int32_t)(u / 1000), (int32_t)r, t);
        usbOut.printf("TIME %s at ms=%lu\n", t, (unsigned long)r);
      }
    }
  }
  if (heard && monitorAgeS() >= 5) {
    heard = false;
    telemetryEvent(rec::EV_MONITOR, 0, 5, "no MON frame for 5 s");
    usbOut.println("MONITOR lost: no MON frame for 5 s");
  }
}

// Green LED (nLED, GPIO1): invert the level that the drive-mode code set. The output register holds
// it (the pin is output only, so digitalRead() cannot read it).
uint8_t ledLevel(){ return (REG_READ(GPIO_OUT_REG) >> nLED) & 1; }
// Once per loop pass: inverted while a MON window runs (100 ms). When the drive-mode code writes the
// LED in the window, the next pass inverts its new level. At the window end, the pass restores it.
void monLedService(){
  const uint8_t cur = ledLevel();
  if ((int32_t)(millis() - mon_blip_until) < 0) {
    if (!mon_blip_on || cur != mon_blip_level) {
      mon_blip_level = cur ? 0 : 1;
      digitalWrite(nLED, mon_blip_level);
      mon_blip_on = true;
    }
  } else if (mon_blip_on) {
    if (cur == mon_blip_level) digitalWrite(nLED, cur ? 0 : 1);
    mon_blip_on = false;
  }
}

void espnowStart(){
  if (espnow_started || espnow_failed) return;
  const uint32_t t0 = millis();
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station (no AP, default channel)
  if (esp_now_init() != ESP_OK) {
    usbOut.println("ESP-NOW Init Failed (ESP-NOW stays off until a restart)");
    espnow_failed = true;  // no retry, no restart: the vehicle keeps driving without the radio
    return;
  }
  esp_now_register_recv_cb(receiveCallback);  // Register receive callback
  esp_now_register_send_cb(sentCallback);     // Register send callback
  espnow_started = true;
  applyTxPower();
  boot_espnow_ms = millis() - t0;
  usbOut.printf("ESP-NOW Init Success, MAC %s, %lu ms\n", WiFi.macAddress().c_str(), (unsigned long)boot_espnow_ms);
}

// A unicast send needs the destination in the peer list.
void espnowEnsurePeer(const uint8_t* mac){
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  esp_now_add_peer(&peer);
}

// ACK 0x11: u8 type, u16 seq, u8 result, ASCII text.
void espnowSendAck(const uint8_t* mac, uint16_t seq, uint8_t result, const char* text){
  uint8_t frame[ESP_NOW_MAX_DATA_LEN];
  frame[0] = ESPNOW_TYPE_ACK;
  frame[1] = seq & 0xFF;
  frame[2] = seq >> 8;
  frame[3] = result;
  const size_t n = min(strlen(text), (size_t)(ESP_NOW_MAX_DATA_LEN - 4));
  memcpy(&frame[4], text, n);
  espnowEnsurePeer(mac);
  if (esp_now_send(mac, frame, 4 + n) != ESP_OK) espnow_send_err++;
}

uint8_t configCommand(const char* t, bool over_espnow, String& reply);
uint8_t configCommandRun(const char* t, bool over_espnow, String& reply);

// Restarts that the PC sees (bench tests). With USB persist off
// (arduino-esp32 2.0.17), usb_persist_restart() resets the USB core but leaves the D+ pull-up on:
// the PC never sees the board leave the bus, and never addresses the new USB device after the
// restart, so the board looks silent. Detach first, and give the PC time to see it.
// Not yet verified on a board.
void usbDetachForRestart(){
  msc_fat_eject();  // the PC stops its drive reads first (a PC that read the drive at the detach lost USB ports)
  usbOut.drain(200);
  delay(500);
  tud_disconnect();  // D+ pull-up off: the PC sees a removal
  delay(300);
}
void restartIntoRom(){  // ROM download mode, for esptool --before no_reset --after hard_reset
  usbDetachForRestart();
  usb_persist_restart(RESTART_BOOTLOADER);
  ESP.restart();  // only if the shutdown handler could not be registered
}
void restartApp(){
  usbDetachForRestart();
  ESP.restart();
}

// usb_ota "may start": only a vehicle that stands still takes an update, and it then enters update
// mode: outputs to their centre pulses, then loop() serves only USB and the update.
bool vehicleStationary();
bool otaMayStart(){
  if (!vehicleStationary()) return false;
  writePwmMicros(1, PWM_OUT_1_DEFAULT);
  writePwmMicros(2, PWM_OUT_2_DEFAULT);
  writePwmMicros(3, PWM_OUT_3_DEFAULT);
  writePwmMicros(4, PWM_OUT_4_DEFAULT);
  if (!update_mode) {
    usbOut.println("UPDATE mode on: USB and the update only; loop watchdog paused");
    disableLoopWDT();  // the image check at "ota end" can run past 5 s (bench test)
  }
  update_mode = true;
  update_last_ms = millis();
  return true;
}

// Config commands and restarts, from loop() (docs/espnow_protocol.md).
uint8_t runCommand(bool start, const char* source, String& reply);
void espnowCommands(){
  if (espnow_stop_request) {
    espnow_stop_request = false;
    uint8_t mac[6];
    memcpy(mac, espnow_stop_mac, 6);
    String reply;
    runCommand(false, "STOP frame", reply);
    espnowSendAck(mac, espnow_stop_seq, 0, "stop");
  }
  if (espnow_restart_request) {
    const int what = espnow_restart_request;
    espnow_restart_request = 0;
    if (!vehicleStationary()) {  // never a restart while the vehicle moves (the main-branch baseline has no remote restart)
      usbOut.printf("ESP-NOW restart request %d refused: the vehicle moves\n", what);
      telemetryEvent(rec::EV_RESTART_REFUSED, what, 1, "ESP-NOW restart refused: the vehicle moves");
    } else {
      usbOut.printf("ESP-NOW restart request %d\n", what);
      if (what == 2) restartIntoRom();
      restartApp();
    }
  }
  if (espnow_busy_pending) {
    uint8_t mac[6];
    memcpy(mac, espnow_busy_mac, 6);  // copy before the flag is cleared: the callback may write again
    const uint16_t seq = espnow_busy_seq;
    espnow_busy_pending = false;
    espnowSendAck(mac, seq, 3, "busy");
  }
  if (espnow_list_index >= 0 && millis() - espnow_list_ms >= 5) {  // one key every 5 ms
    espnow_list_ms = millis();
    String line;
    if (msc_fat_key_at(&g_deviceConfig, espnow_list_index, &line)) {
      espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, 0, line.c_str());
      espnow_list_index = espnow_list_index + 1;
    } else {
      espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, 0,  // key count: a lost item shows; def: settings_def.json version
                    (String("end ") + espnow_list_index + " def " + SETTINGS_DEF_VERSION).c_str());
      espnow_list_index = -1;
    }
  }
  static uint32_t bad_key_logged = 0;
  if (espnow_bad_key != bad_key_logged) {
    bad_key_logged = espnow_bad_key;
    telemetryEvent(rec::EV_BAD_KEY, (int32_t)bad_key_logged, 0, "CMD/RESTART with a wrong fleet key");
  }
  if (!espnow_cmd_pending) return;
  // A retry (same sender, same seq: the ACK was lost) gets the cached ACK and does not run again.
  static uint8_t last_mac[6];
  static uint16_t last_seq = 0;
  static bool have_last = false;
  static uint8_t last_result = 0;
  static String last_reply;
  if (!strcmp(espnow_cmd_text, "list")) {
    espnow_list_index = 0;  // the CMD's mac and seq stay in hand until "end"
    espnow_list_ms = 0;
  } else if (have_last && espnow_cmd_seq == last_seq && !memcmp(espnow_cmd_mac, last_mac, 6)
             && strncmp(espnow_cmd_text, "get ", 4)) {
    // Only a command with an effect (set, save, run) is answered from the cache. A read ("get ...")
    // is answered fresh: a cached "get run" said running after a STOP had stopped the vehicle
    // (PC telemetry simulation, 1 in 18 runs at 30 % loss).
    espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, last_result, last_reply.c_str());
  } else {
    String reply;
    const uint8_t r = configCommand(espnow_cmd_text, true, reply);
    espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, r, reply.c_str());
    memcpy(last_mac, espnow_cmd_mac, 6);
    last_seq = espnow_cmd_seq;
    last_result = r;
    last_reply = reply;
    have_last = true;
  }
  espnow_cmd_pending = false;
}

// One config command, "get <key>", "set <key> <value>" or "save", from ESP-NOW (CMD 0x10) or
// from the USB serial port. Result: 0 ok, 1 unknown key, 2 bad value, 4 refused.
// Keys that change only while the vehicle stands still (vehicleStationary()); every other key is
// live. Reasons: a lost wire or a changed charge limit in the STM32 (rx_channel, enable_sensor_2, CHARGE_*), a jump of
// the steering sign, a servo centre or a limit (pid_direction, PWM_OUT_*), a step of the drive
// command or a change of mission (rover_* calibration, enable, direction, beacons; BTxFollowChannel),
// a Wi-Fi start that stops loop() for about 220 ms (espnow_enabled, telemetry_enabled).
// docs/rover_guide.md, "Settings", has the same list.
static const char* const STATIONARY_KEYS[] = {
  "rx_channel", "enable_sensor_2", "pid_direction",
  "PWM_OUT_1_DEFAULT", "PWM_OUT_1_MIN", "PWM_OUT_1_MAX", "PWM_OUT_2_DEFAULT", "PWM_OUT_2_MIN", "PWM_OUT_2_MAX",
  "PWM_OUT_3_DEFAULT", "PWM_OUT_3_MIN", "PWM_OUT_3_MAX", "PWM_OUT_4_DEFAULT", "PWM_OUT_4_MIN", "PWM_OUT_4_MAX",
  "CHARGE_VBATT_STOP", "CHARGE_MAX_CURRENT", "CHARGE_MAX_SECONDS", "BTxFollowChannel",
  "espnow_enabled", "telemetry_enabled",
  "rover_enabled", "rover_goal_cw", "rover_sensor_height_mm", "rover_full_speed_mps", "rover_full_yaw_rate_dps",
  "rover_entry_btx", "rover_exit_btx", "imu_required",
};
bool stationaryKey(const char* key){
  if (!strncmp(key, "dsp_", 4)) return true;  // a DSP change stops the STM32 run for a moment
  for (const char* k : STATIONARY_KEYS) if (!strcmp(k, key)) return true;
  return false;
}
bool vehicleStationary();

// Run commands (docs/control_api.md in Trail-Tracer-PC-Telemetry): "run start" enters mode 2
// (autonomous: the autopilot when rover_enabled, else the wire follower) unless an RC transmitter is
// present with its aux switch in manual (RC has priority); "run stop" (and the STOP frame) leaves
// mode 2 for mode 0: with no RC that is neutral output, and the autopilot does not run. The stop is
// held in RAM: a restart with no RC enters mode 2 again (boot behaviour unchanged).
uint8_t runCommand(bool start, const char* source, String& reply){
  const uint32_t aux = pwm_read_rmt_dur(2);
  const bool rc_manual = aux > 500 && aux < 1450;
  uint8_t r = 0;
  if (start && rc_manual) {
    reply = "run=stopped rc_manual";
    r = 4;
  } else if (start) {
    if (drive_mode_status != 2) {
      drive_mode_status = 2;
      case2_use_cruise = false;  // ramp to creep, as at boot
      case2_throttle_current = PWM_OUT_2_DEFAULT;
      case2_aux_entry_is_manual = rc_manual;
      case2_aux_changed = false;
    }
    reply = "run=running";
  } else {
    drive_mode_status = 0;
    reply = "run=stopped";
  }
  telemetryEvent(rec::EV_RUN, start ? 1 : 0, r, (String(source) + " -> " + reply).c_str());
  usbOut.printf("RUN %s (%s)\n", reply.c_str(), source);
  return r;
}

uint8_t configCommand(const char* t, bool over_espnow, String& reply){
  if (!strcmp(t, "run start") || !strcmp(t, "run stop"))
    return runCommand(!strcmp(t, "run start"), over_espnow ? "CMD" : "USB", reply);
  if (!strcmp(t, "get run")) {
    reply = drive_mode_status == 2 ? "run=running" : "run=stopped";
    return 0;
  }
  const uint8_t r = configCommandRun(t, over_espnow, reply);
  if (strncmp(t, "get ", 4)) {  // one event per set or save, applied or refused: the log shows what was tried
    telemetryEvent(rec::EV_SETTING, r, over_espnow ? 1 : 0, (String(t) + " -> " + reply).c_str());
  }
  return r;
}

uint8_t configCommandRun(const char* t, bool over_espnow, String& reply){
  char key[48] = {0}, value[64] = {0};
  String out;
  if (!strcmp(t, "save")) {
    if (!vehicleStationary()) { reply = "save=not while moving"; return 4; }  // a flash write stops the CPU
    // NVS and config.json (design choice 2026-10-10: a save is a save of the file). config.json overrides
    // NVS at boot, so an NVS-only save of a key that the file holds was lost at the next boot.
    if (msc_fat_save_all(&g_deviceConfig)) { reply = "save=json"; return 0; }
    reply = "save=nvs only: config.json write failed";
    return 4;
  }
  if (sscanf(t, "get %47s", key) == 1) {
    if (!msc_fat_get_key(&g_deviceConfig, key, &out)) { reply = key; return 1; }
    reply = String(key) + "=" + out;
    return 0;
  }
  if (sscanf(t, "set %47s %63s", key, value) == 2) {
    if (strpbrk(value, ",{}[]:\"")) { reply = key; return 2; }  // one literal only: no JSON injection of other keys
    if (over_espnow && !strcmp(key, "cmd_key")) { reply = "cmd_key not over ESP-NOW"; return 4; }
    if (stationaryKey(key) && !vehicleStationary()) {
      msc_fat_get_key(&g_deviceConfig, key, &out);
      reply = String(key) + "=" + out + " not while moving";
      return 4;
    }
    const bool radio_key = !strcmp(key, "espnow_enabled") || !strcmp(key, "telemetry_enabled");
    if (over_espnow && radio_key && !strcmp(value, "false")) {
      reply = String(key) + "=true";  // would cut the link that carries the command
      return 4;
    }
    const int r = msc_fat_set_key(&g_deviceConfig, key, value);
    if (r != 0) { reply = key; return (uint8_t)r; }
    applyDeviceConfig(g_deviceConfig);
    msc_fat_get_key(&g_deviceConfig, key, &out);
    reply = String(key) + "=" + out;
    return 0;
  }
  reply = "unknown command";
  return 2;
}

// Answer a PING and send THRU frames. Runs in loop(), never in a callback.
void espnowService(){
  if (espnow_ping_pending) {
    uint8_t mac[6], buf[sizeof(espnow_ping_buf)];
    const uint8_t len = espnow_ping_len;
    memcpy(mac, espnow_ping_mac, 6);
    memcpy(buf, espnow_ping_buf, len);
    espnow_ping_pending = false;
    espnowEnsurePeer(mac);
    buf[0] = ESPNOW_TYPE_PONG;
    if (esp_now_send(mac, buf, len) == ESP_OK) espnow_pong_count++;
    else espnow_send_err++;
  }
  if (espnow_thru_active) {
    const uint32_t now = micros();
    if (now - espnow_thru_last_us >= espnow_thru_period_us) {
      // Keep the rate; after a long stall restart the schedule instead of sending a burst.
      espnow_thru_last_us = (now - espnow_thru_last_us > 10 * espnow_thru_period_us) ? now : espnow_thru_last_us + espnow_thru_period_us;
      espnowEnsurePeer(espnow_thru_mac);
      uint8_t frame[ESP_NOW_MAX_DATA_LEN] = {0};
      frame[0] = ESPNOW_TYPE_THRU;
      memcpy(&frame[1], &espnow_thru_seq, 4);
      memcpy(&frame[5], &now, 4);
      if (esp_now_send(espnow_thru_mac, frame, espnow_thru_size) != ESP_OK) espnow_send_err++;
      espnow_thru_seq++;  // one number per attempt: a refused send shows as a gap and in send_err
      if (--espnow_thru_left == 0) {
        espnow_thru_active = false;
        usbOut.printf("BENCH thru done sent=%lu send_err=%lu\n", (unsigned long)espnow_thru_seq, (unsigned long)espnow_send_err);
      }
    }
  }
}

// USB serial commands for the bench tests (one line each):
//   thru <rate_hz> <size_bytes> <count>   broadcast THRU frames      thru stop
//   timing on | timing off                1 s loop and ESP-NOW counters
//   stat                                  print the counters once
void benchPrintCounters(const char* tag, uint32_t loops, uint32_t min_us, uint32_t max_us){
  usbOut.printf("%s loops=%lu min_us=%lu max_us=%lu crc_err=%lu overrun=%lu espnow=%d rx=%lu pings=%lu missed=%lu pongs=%lu tx_ok=%lu tx_fail=%lu send_err=%lu\n",
    tag, (unsigned long)loops, (unsigned long)(loops ? min_us : 0), (unsigned long)max_us,
    (unsigned long)crc_error_count, (unsigned long)uart_overrun_count, (int)espnow_started,
    (unsigned long)espnow_rx_count, (unsigned long)espnow_ping_count, (unsigned long)espnow_ping_missed,
    (unsigned long)espnow_pong_count, (unsigned long)espnow_tx_ok, (unsigned long)espnow_tx_fail, (unsigned long)espnow_send_err);
}

// "send <mac> <hex>": one ESP-NOW frame, e.g. "send 01:23:45:67:89:ab 7f52535421" (RESTART "RST!").
void benchSend(const char* arg){
  unsigned m[6];
  char hex[2 * ESP_NOW_MAX_DATA_LEN + 1];
  if (sscanf(arg, "%x:%x:%x:%x:%x:%x %500s", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], hex) != 7) {
    usbOut.println("BENCH send: usage send <mac> <hex>");
    return;
  }
  if (!espnow_started) { usbOut.println("BENCH send: espnow off"); return; }
  uint8_t mac[6], frame[ESP_NOW_MAX_DATA_LEN];
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
  const size_t n = strlen(hex) / 2;
  if (n == 0 || n > sizeof(frame) || strlen(hex) % 2) { usbOut.println("BENCH send: bad hex"); return; }
  for (size_t i = 0; i < n; i++) { unsigned b; sscanf(hex + 2 * i, "%2x", &b); frame[i] = (uint8_t)b; }
  espnowEnsurePeer(mac);
  const esp_err_t r = esp_now_send(mac, frame, n);
  usbOut.printf("BENCH send len=%u err=%d\n", (unsigned)n, (int)r);
}

// Bench: a raw link to the STM32's ROM loader (AN3155: 115200 baud, 8 data bits, even parity) for
// the first load of a blank STM32. A blank STM32G0 starts its ROM loader by itself (empty check);
// the Rx board has no BOOT0 or reset line. "stm rom" opens the link (only while the vehicle stands
// still), "stm tx <hex>" sends bytes, the board prints "STM rx <hex>" for each read, and "stm off"
// returns to the 2 Mbaud packet link. A PC tool runs the protocol.
bool stm_raw = false;
// STM32 app update through the boot checker (IAP), bench and field: "stm iaphold" sets a one-shot
// flag in NVS. At the next power-on the handshake sends HELLO only (no BOOT), so the STM32 stays in
// its boot checker, and the board becomes a transparent bridge between USB and the STM32 UART
// (2 Mbaud): the PC runs the IAP update (protocol in TrailTracer-Rx-STM32
// iap_proto.h). No USB text in bridge mode. The bridge ends STM_BRIDGE_IDLE_MS after the last USB
// byte (STM_BRIDGE_WAIT_MS if none came); then HELLO/BOOT if the STM32 is still silent. The board
// then continues as usual, with no ESP32 restart.
#define STM_BRIDGE_WAIT_MS 300000
#define STM_BRIDGE_IDLE_MS 10000
#define STM_BRIDGE_HELLO_TRIES 3
bool stm_bridge = false;
uint32_t stm_bridge_start_ms = 0, stm_bridge_rx_ms = 0, stm_bridge_up = 0, stm_bridge_down = 0;
bool stmIapHoldTake(){  // read and clear the one-shot flag
  Preferences p;
  if (!p.begin("wfrx_bench", false)) return false;
  const bool hold = p.getBool("iaphold", false);
  if (hold) p.remove("iaphold");
  p.end();
  return hold;
}
void stmIapHoldSet(){
  msc_fat_eject();  // a power-off follows: the PC stops its drive reads now
  Preferences p;
  const bool ok = p.begin("wfrx_bench", false) && p.putBool("iaphold", true);
  p.end();
  usbOut.println(ok ? "STM iaphold set: at the next power-on the STM32 stays in its boot checker and USB bridges to it"
                    : "STM iaphold: NVS write failed");
}

// STM32 calibration copy (design choice, 2026-10-11): an IAP update erases the STM32 NVM (it lies in
// the app region), and the new app starts with its firmware defaults. The ESP32 keeps the per-unit
// calibration in NVS ("wfrx_cal"): bin gain (CID 26, 16), radius (27, 2), angle (28, 2), PGA (37, 12).
// "stm iaphold" reads all 32 first and stores them; if a read fails, the stored copy stays. Then the
// restore flag is set. At the next v2 start the board writes the copy, saves it (CID 3), reads all 32
// back and clears the flag when they are equal. One command at a time, when the link is not busy.
#define STM_CAL_N 32
#define STM_CAL_PHASE_MS 8000
enum { CAL_IDLE, CAL_READ, CAL_WRITE, CAL_SAVE, CAL_VERIFY };
uint8_t stm_cal_phase = CAL_IDLE;
bool stm_cal_then_hold = false;     // the read was for "stm iaphold": set the flag at its end
uint32_t stm_cal_val[STM_CAL_N];    // the stored copy (raw float bits)
bool stm_cal_valid = false;
uint32_t stm_cal_new[STM_CAL_N];    // this read, or the verify read
uint8_t stm_cal_i = 0, stm_cal_ok = 0;
bool stm_cal_sent = false, stm_cal_got = false;
uint32_t stm_cal_phase_ms = 0;
void stmCalItem(int k, uint8_t* cid, uint8_t* idx){
  if (k < 16) { *cid = 26; *idx = (uint8_t)k; }
  else if (k < 18) { *cid = 27; *idx = (uint8_t)(k - 16); }
  else if (k < 20) { *cid = 28; *idx = (uint8_t)(k - 18); }
  else { *cid = 37; *idx = (uint8_t)(k - 20); }
}
void stmCalLoad(){
  Preferences p;
  stm_cal_valid = p.begin("wfrx_cal", true) && p.getBytes("v", stm_cal_val, sizeof(stm_cal_val)) == sizeof(stm_cal_val);
  p.end();
}
bool stmCalPending(){
  Preferences p;
  const bool r = p.begin("wfrx_cal", true) && p.getBool("restore", false);
  p.end();
  return r;
}
void stmCalSetPending(bool on){
  Preferences p;
  if (p.begin("wfrx_cal", false)) p.putBool("restore", on);
  p.end();
}
void stmCalStart(uint8_t phase){
  stm_cal_phase = phase;
  stm_cal_i = 0;
  stm_cal_ok = 0;
  stm_cal_sent = false;
  stm_cal_phase_ms = millis();
}
// One v2 answer (before stmLink.onAnswerV2).
void stmCalOnAnswer(const stmv2::Answer& a){
  if (stm_cal_phase == CAL_IDLE || !stm_cal_sent || !a.present || (a.status != RSP_OK && a.status != RSP_OK_CLAMPED)) return;
  if (stm_cal_phase == CAL_SAVE) { if (a.cid == (3 | ESP_CID_WRITE)) stm_cal_got = true; return; }
  uint8_t cid, idx;
  stmCalItem(stm_cal_i, &cid, &idx);
  const uint8_t want = stm_cal_phase == CAL_WRITE ? (uint8_t)(cid | ESP_CID_WRITE) : cid;
  if (a.cid != want || a.index != idx) return;
  stm_cal_new[stm_cal_i] = a.value;
  stm_cal_got = true;
}
void stmCalEnd(){
  const uint8_t phase = stm_cal_phase;
  stm_cal_phase = CAL_IDLE;
  if (phase == CAL_READ) {
    if (stm_cal_ok == STM_CAL_N) {
      memcpy(stm_cal_val, stm_cal_new, sizeof(stm_cal_val));
      Preferences p;
      stm_cal_valid = p.begin("wfrx_cal", false) && p.putBytes("v", stm_cal_val, sizeof(stm_cal_val)) == sizeof(stm_cal_val);
      p.end();
      usbOut.printf("STM cal read: %u of %u, stored %s\n", (unsigned)stm_cal_ok, STM_CAL_N, stm_cal_valid ? "ok" : "FAILED");
    } else {
      usbOut.printf("STM cal read failed: %u of %u answered; the stored copy stays (%s)\n", (unsigned)stm_cal_ok, STM_CAL_N, stm_cal_valid ? "valid" : "none");
    }
    if (stm_cal_then_hold) {
      stm_cal_then_hold = false;
      if (stm_cal_valid) stmCalSetPending(true);
      stmIapHoldSet();
    }
  } else if (phase == CAL_VERIFY) {
    int eq = 0;
    for (int k = 0; k < STM_CAL_N; k++) eq += stm_cal_new[k] == stm_cal_val[k];
    if (eq == STM_CAL_N) stmCalSetPending(false);
    usbOut.printf("STM cal restore: written %u, saved, %d of %u read back equal%s\n", (unsigned)stm_cal_ok, eq, STM_CAL_N,
                  eq == STM_CAL_N ? "" : "; FAILED, the restore runs again at the next start");
    telemetryEvent(rec::EV_STM32_CMD, stmset::K_CAL_RESTORE, eq, eq == STM_CAL_N ? "STM32 calibration restored after update" : "STM32 calibration restore failed");
  } else if (phase == CAL_SAVE) {
    usbOut.printf("STM cal restore: CID 3 save %s\n", stm_cal_got ? "ok" : "no answer");
    stmCalStart(CAL_VERIFY);
  }
}
// Once a v2 frame, after stmLink.onAnswerV2: the next command when the last one is done.
void stmCalService(uint32_t now){
  if (stm_cal_phase == CAL_IDLE) return;
  if (stm_cal_sent) {
    if (stmLink.busy()) {
      if (now - stm_cal_phase_ms < STM_CAL_PHASE_MS) return;
    }
    stm_cal_sent = false;
    if (stm_cal_got) stm_cal_ok++;
    if (stm_cal_phase == CAL_SAVE) { stmCalEnd(); return; }
    if (++stm_cal_i >= STM_CAL_N || now - stm_cal_phase_ms >= STM_CAL_PHASE_MS) {
      if (stm_cal_phase == CAL_WRITE) { stmCalStart(CAL_SAVE); }
      else { stmCalEnd(); return; }
    }
  }
  if (stmLink.busy()) return;
  stm_cal_got = false;
  if (stm_cal_phase == CAL_SAVE) {
    stm_v2_save_ok = true;
    stmLink.pushInt(3 | stmset::WRITE, 1);
  } else {
    uint8_t cid, idx;
    stmCalItem(stm_cal_i, &cid, &idx);
    if (stm_cal_phase == CAL_WRITE) stmLink.push(cid | stmset::WRITE, stm_cal_val[stm_cal_i], idx);
    else stmLink.push(cid, 0, idx);
  }
  stm_cal_sent = true;
}
void stmBridgeService(){
  uint8_t b[256];
  int n = 0;
  while (n < (int)sizeof(b) && Serial.available()) b[n++] = (uint8_t)Serial.read();
  if (n) { Serial1.write(b, n); stm_bridge_up += n; stm_bridge_rx_ms = millis(); }
  int room = Serial.availableForWrite();  // never block: what the host cannot take waits in Serial1's buffer
  n = 0;
  while (n < room && n < (int)sizeof(b) && Serial1.available()) b[n++] = (uint8_t)Serial1.read();
  if (n) { Serial.write(b, n); stm_bridge_down += n; }
  const uint32_t now = millis();
  const bool idle = stm_bridge_rx_ms ? now - stm_bridge_rx_ms >= STM_BRIDGE_IDLE_MS
                                     : now - stm_bridge_start_ms >= STM_BRIDGE_WAIT_MS;
  // nDATA_READY edges do not show a running application: the boot checker makes them too (bench
  // test). So the bridge ends only by time; send nothing to the board until "STM bridge off".
  if (!idle) return;
  stm_bridge = false;
  stm_link_ver = 0;  // the STM32 app may be new: learn the link version again
  stm_v2_cfg_sent = false;
  stmDec.reset();
  data_ready = 0;
  delay(50);
  usbOut.printf("STM bridge off: up=%lu down=%lu B\n", (unsigned long)stm_bridge_up, (unsigned long)stm_bridge_down);
  if (data_ready == 0 && Serial1.available() == 0) {  // STM32 still in its boot checker
    char t[72];
    uint8_t plen = 0;
    const int r = stm32_iap_boot(STM_BRIDGE_HELLO_TRIES, t, sizeof(t), &plen);
    usbOut.printf("STM bridge: HELLO/BOOT result %d, %s\n", r, t);
    telemetryEvent(rec::EV_STM32, 1, plen, t);
  }
  serial1Drain();
  data_ready = 0;
}
void stmRawService(){
  uint8_t b[64];
  int n = 0;
  while (n < (int)sizeof(b) && Serial1.available()) b[n++] = (uint8_t)Serial1.read();
  if (!n) return;
  char hex[2 * sizeof(b) + 1];
  for (int i = 0; i < n; i++) sprintf(hex + 2 * i, "%02x", b[i]);
  usbOut.printf("STM rx %s\n", hex);
}
void stmRawCommand(const char* arg){
  if (!strcmp(arg, "rom")) {
    if (!vehicleStationary()) { usbOut.println("STM refused: the vehicle moves"); return; }
    detachInterrupt(digitalPinToInterrupt(STM_nDATA_READY));
    Serial1.end();
    Serial1.begin(115200, SERIAL_8E1);
    serial1Drain();
    stm_raw = true;
    stm_rom_used = true;
    usbOut.println("STM rom link on: 115200 8E1");
  } else if (!strcmp(arg, "peek")) {
    stm_raw = true;  // the 2 Mbaud link as it is: print the bytes already in the buffer, send nothing
    usbOut.printf("STM peek: %d bytes buffered at 2 Mbaud\n", Serial1.available());
  } else if (!strcmp(arg, "off")) {
    Serial1.end();
    Serial1.begin(2000000);
    Serial1.setRxFIFOFull(64);  // a 128 B frame: the default 120 B leaves 8 B of margin
    stm_raw = false;
    data_ready = 0;
    attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
    usbOut.println("STM rom link off: 2 Mbaud packet link");
  } else if (!strncmp(arg, "tx ", 3)) {
    if (!stm_raw) { usbOut.println("STM tx refused: link off"); return; }
    const char* h = arg + 3;
    uint8_t b[300];
    size_t n = 0;
    while (h[0] && h[1] && n < sizeof(b)) { unsigned v; if (sscanf(h, "%2x", &v) != 1) break; b[n++] = (uint8_t)v; h += 2; }
    Serial1.write(b, n);
    Serial1.flush();
    usbOut.printf("STM tx %u\n", (unsigned)n);
  } else if (!strncmp(arg, "cmd ", 4) || !strncmp(arg, "read ", 5) || !strncmp(arg, "dup ", 4)) {
    // bench: "stm cmd <cid> <value> [index]" (a write; a value with '.' is a float), "stm read <cid> [index]",
    // "stm dup <cid> <value> [index]" (one write frame with its own seq, sent twice 3 frames apart, for the
    // STM32 repeat rule). The v2 answers print for 2 s (STM ans ...).
    const bool dup = arg[0] == 'd', w = arg[0] == 'c' || dup;
    const char* q = arg + (arg[0] == 'r' ? 5 : 4);
    char* e = nullptr;
    const long cid = strtol(q, &e, 10);
    if (cid < 1 || cid > 127) { usbOut.println("STM usage: stm cmd <cid> <value> [index] | stm read <cid> [index] | stm dup <cid> <value> [index]"); return; }
    uint32_t raw = 0;
    while (*e == ' ') e++;
    if (w) {
      char* e2 = nullptr;
      if (strpbrk(e, ".") && strpbrk(e, ".") < (strchr(e, ' ') ? strchr(e, ' ') : e + strlen(e))) { const float f = strtof(e, &e2); memcpy(&raw, &f, 4); }
      else raw = (uint32_t)strtol(e, &e2, 0);
      e = e2;
      while (*e == ' ') e++;
    }
    const uint8_t idx = (uint8_t)strtol(e, nullptr, 0);
    if (cid == 13 && w && raw <= 3) stm_v2_lanes = (uint8_t)raw;
    stm_ans_print_until = millis() + 2000;
    if (dup) {
      if (stm_link_ver != 2) { usbOut.println("STM dup: v2 link only"); return; }
      stmv2::encodeCmd((uint8_t)(stmLink.seq() + 128), (uint8_t)(cid | ESP_CID_WRITE), idx, raw, stm_dup_frame);
      stm_dup_step = 1;
      usbOut.printf("STM dup cid %ld index %u seq %u\n", cid, (unsigned)idx, (unsigned)stm_dup_frame[0]);
      return;
    }
    const bool ok = stmLink.push((uint32_t)cid | (w ? stmset::WRITE : 0), raw, idx);
    usbOut.printf("STM %s %ld index %u queued=%d\n", w ? "cmd" : "read", cid, (unsigned)idx, (int)ok);
  } else if (!strcmp(arg, "pos")) {
    // bench: the decoded STM32 values as the firmware uses them (v1 and v2)
    const __StmTxPacketStruct& k = stmPacket.items;
    usbOut.printf("STMPOS ver=%u ms=%lu sb=0x%08lx s1 %.1f %.1f %.1f s2 %.1f %.1f %.1f btx1 %.0f %.0f btx2 %.0f %.0f vb=%.2f t=%.1f chan=0x%02x\n",
      (unsigned)stm_link_ver, (unsigned long)millis(), (unsigned long)k.status_bits, k.s1_x, k.s1_y, k.s1_radius,
      k.s2_x, k.s2_y, k.s2_radius, k.s1_btx_radius[0], k.s1_btx_radius[1], k.s2_btx_radius[0], k.s2_btx_radius[1],
      k.vbatt, k.temperature, (unsigned)stmDec.lanes);
  } else if (!strncmp(arg, "coils ", 6)) {
    const long ms = atol(arg + 6);
    memset(stm_coil_sum, 0, sizeof(stm_coil_sum));
    memset(stm_coil_ssum, 0, sizeof(stm_coil_ssum));
    stm_coil_n = 0;
    stm_coil_t0 = millis();
    stm_coil_ms = ms <= 0 ? 0 : (uint32_t)(ms < 50 ? 50 : ms);
    usbOut.printf("STMCOIL period %lu ms (0 = off)\n", (unsigned long)stm_coil_ms);
  } else if (!strncmp(arg, "acc ", 4)) {
    memset(stm_acc_sq, 0, sizeof(stm_acc_sq));
    memset(stm_acc_abs, 0, sizeof(stm_acc_abs));
    memset(stm_acc_max, 0, sizeof(stm_acc_max));
    stm_acc_n = stm_acc_ovl = stm_acc_lost = 0;
    const long n = atol(arg + 4);
    stm_acc_want = (uint32_t)(n < 1 ? 1 : n);
    usbOut.printf("STMACC started n=%lu\n", (unsigned long)stm_acc_want);
  } else if (!strncmp(arg, "stream ", 7)) {
    const long s = atol(arg + 7);
    stm_stream_end = s > 0 ? millis() + (uint32_t)(s > 600 ? 600 : s) * 1000u : 0;
    stm_stream_n = 0;
    usbOut.printf("STM stream %s %ld s: one line a frame, F<hex of bytes 1, 14-19, 38-43, 80-83> <PGA bits>\n",
                  stm_stream_end ? "on" : "off", s);
  } else if (!strncmp(arg, "cap ", 4)) {
    const int n = atoi(arg + 4);
    if (stm_cap) { free(stm_cap); stm_cap = nullptr; }
    stm_cap_dump = false;
    stm_cap_n = stm_cap_out = 0;
    stm_cap_want = (uint16_t)(n < 1 ? 1 : n > 300 ? 300 : n);
    stm_cap = (uint8_t*)malloc((size_t)STM_FRAME_LEN * stm_cap_want);
    usbOut.printf("STM cap %u frames %s\n", (unsigned)stm_cap_want, stm_cap ? "started" : "failed: no memory");
  } else if (!strcmp(arg, "dump")) {
    usbOut.printf("STM dump %u frames%s\n", (unsigned)stm_cap_n, stm_cap ? "" : " (none)");
    stm_cap_dump = stm_cap != nullptr;
    stm_cap_out = 0;
  } else if (!strcmp(arg, "rtt")) {
    usbOut.printf("STMRTT");
    for (int i = 0; i < 16; i++) usbOut.printf(" %lu", (unsigned long)stm_rtt_hist[i]);
    usbOut.printf("\n");
  } else if (!strcmp(arg, "rtt0")) {
    memset(stm_rtt_hist, 0, sizeof(stm_rtt_hist));
    usbOut.println("STMRTT cleared");
  } else if (!strncmp(arg, "run ", 4)) {
    // bench: "stm run 1" = acquisition on (and kept on), "stm run 0" = idle mode (layout 2, app 2.0.6+)
    const int v = atoi(arg + 4) ? 1 : 0;
    stm_v2_autorun = v == 1;
    stmLink.pushInt(STM_RUN_CMD | 0x80000000, v);
    stm_ans_print_until = millis() + 2000;
    usbOut.printf("STM run %d%c", v, 10);
  } else if (!strcmp(arg, "bins")) {
    // bench: the coil magnitudes (or x/y in um for a bin in xy mode) of all 8 bins, both lanes, last frame
    for (unsigned k = 0; k < WFRX_RESULTS; k++) {
      usbOut.printf("STMBINS lane%u", k + 1);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++)
        usbOut.printf(" %ld,%ld", (long)wfrx_int24_get(stmDec.coil[k][bn].v1), (long)wfrx_int24_get(stmDec.coil[k][bn].v2));
      usbOut.printf("%c", 10);
    }
  } else if (!strcmp(arg, "v2")) {
    // bench: the decoder state, the learned STM32 values and the last pages
    usbOut.printf("STMV2 ver=%u frames=%lu crc_err=%lu ver_err=%lu seq_lost=%lu lanes=%u status=0x%08lx wire_bin=%d,%d lost=%d,%d a=%.6f,%.6f b=%.6f,%.6f\n",
      (unsigned)stm_link_ver, (unsigned long)stmDec.st.frames, (unsigned long)stmDec.st.crc_err, (unsigned long)stmDec.st.ver_err,
      (unsigned long)stmDec.st.seq_lost, (unsigned)stmDec.lanes, (unsigned long)stmDec.status_bits, stmDec.wireBin(0), stmDec.wireBin(1),
      (int)stmDec.lost[0], (int)stmDec.lost[1], stmDec.a[0], stmDec.a[1], stmDec.b[0], stmDec.b[1]);
    for (int sn = 0; sn < 2; sn++) {
      usbOut.printf("STMV2 cal s%d R=%.6f phi=%.6f gain", sn + 1, stmDec.cal.radius[sn], stmDec.cal.angle[sn]);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++) usbOut.printf(" %.6f", stmDec.cal.gain[sn][bn]);
      usbOut.printf("\n");
    }
    usbOut.printf("STMV2 thr weak=%.6f lost=%.6f regained=%.6f wait=%lu ms\n", stmDec.cal.weak[0], stmDec.cal.lost[0], stmDec.cal.regained[0], (unsigned long)stmDec.cal.regain_ms);
    const PageMeas& m = stmDec.meas;
    usbOut.printf("STMV2 meas vbatt=%u vcharge=%u icharge=%d v5v=%u v3v3=%u temp=%d timer=%d code=0x%02x\n", m.vbatt_mv, m.vcharge_mv,
      m.icharge_ma, m.v5v_mv, m.vesp_3v3_mv, m.temperature_dc, m.charge_timer_s, m.charge_code);
    const PageHealth& h = stmDec.health;
    usbOut.printf("STMV2 health work_end=%u/%u x10us l2_wait_min=%u isr_worst=%u overruns=%u hk_skip=%u cmd_ok=%u cmd_crc_err=%u uart_err=%u rx_restart=%u bad_length=%u\n",
      h.work_end_last, h.work_end_worst, h.l2_wait_min, h.isr_worst, h.overrun_count, h.hk_skip_count, h.cmd_ok, h.cmd_crc_err,
      h.uart_err, h.rx_restart, h.bad_length);
  } else if (!strcmp(arg, "iaphold")) {
    if (stm_link_ver == 2 && stm_cal_phase == CAL_IDLE) { stm_cal_then_hold = true; stmCalStart(CAL_READ); }  // copy first
    else { if (stm_cal_valid) stmCalSetPending(true); stmIapHoldSet(); }
  } else if (!strcmp(arg, "cal")) {
    usbOut.printf("STM cal copy %s, restore %s:", stm_cal_valid ? "valid" : "none", stmCalPending() ? "pending" : "off");
    for (int k = 0; k < STM_CAL_N; k++) { float f; memcpy(&f, &stm_cal_val[k], 4); usbOut.printf(" %.7g", f); }
    usbOut.printf("\n");
  } else if (!strcmp(arg, "cal read") || !strcmp(arg, "cal restore")) {
    // bench: the copy without an update ("cal read"), or the restore at once ("cal restore")
    if (stm_link_ver != 2 || stm_cal_phase != CAL_IDLE) usbOut.println("STM cal: busy, or no v2 link");
    else if (arg[4] == 'r' && arg[5] == 'e' && arg[6] == 'a') stmCalStart(CAL_READ);
    else if (!stm_cal_valid) usbOut.println("STM cal: no stored copy");
    else stmCalStart(CAL_WRITE);
  } else {
    usbOut.println("STM usage: stm rom | stm peek | stm tx <hex> | stm off | stm iaphold | stm cmd <cid> <value> [index] | stm read <cid> [index] | stm dup <cid> <value> [index] | stm pos | stm v2");
  }
}

void benchSerial(){
  static char line[1200];  // "ota w <offset> <hex of up to 512 bytes>" fits
  static uint16_t n = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      if (n < sizeof(line) - 1) line[n++] = c;
      continue;
    }
    if (n == 0) continue;
    line[n] = 0;
    n = 0;
    unsigned long hz = 0, size = 0, count = 0;
    unsigned tm[6];
    const int thru_args = sscanf(line, "thru %lu %lu %lu %x:%x:%x:%x:%x:%x", &hz, &size, &count,
                                 &tm[0], &tm[1], &tm[2], &tm[3], &tm[4], &tm[5]);
    unsigned long lt = 0, lr = 0;
    if ((thru_args == 3 || thru_args == 9) && hz >= 1 && hz <= 1000 && count > 0) {
      if (!espnow_started) { usbOut.println("BENCH espnow off (set espnow_enabled in config.json)"); continue; }
      for (int i = 0; i < 6; i++) espnow_thru_mac[i] = thru_args == 9 ? (uint8_t)tm[i] : 0xFF;
      espnow_thru_size = constrain(size, 9UL, (unsigned long)ESP_NOW_MAX_DATA_LEN);
      espnow_thru_period_us = 1000000UL / hz;
      espnow_thru_left = count;
      espnow_thru_seq = 0;
      espnow_send_err = 0;
      espnow_thru_last_us = micros();
      espnow_thru_active = true;
      usbOut.printf("BENCH thru start rate_hz=%lu size=%lu count=%lu to=%02x:%02x:%02x:%02x:%02x:%02x\n", hz,
        (unsigned long)espnow_thru_size, count, espnow_thru_mac[0], espnow_thru_mac[1], espnow_thru_mac[2],
        espnow_thru_mac[3], espnow_thru_mac[4], espnow_thru_mac[5]);
    } else if (!strcmp(line, "loss") || (sscanf(line, "loss %lu %lu", &lt, &lr) == 2 && lt <= 100 && lr <= 100)) {
      if (strcmp(line, "loss")) { sim_loss_tx = (uint8_t)lt; sim_loss_rx = (uint8_t)lr; }
      usbOut.printf("BENCH loss tx=%u rx=%u dropped_tx=%lu dropped_rx=%lu\n", (unsigned)sim_loss_tx, (unsigned)sim_loss_rx,
        (unsigned long)sim_dropped_tx, (unsigned long)sim_dropped_rx);
    } else if (!strcmp(line, "thru stop")) {
      espnow_thru_active = false;
      usbOut.printf("BENCH thru stopped sent=%lu\n", (unsigned long)espnow_thru_seq);
    } else if (!strcmp(line, "timing on") || !strcmp(line, "timing off")) {
      bench_timing_print = !strcmp(line, "timing on");
      usbOut.printf("BENCH timing %s\n", bench_timing_print ? "on" : "off");
    } else if (!strcmp(line, "armed sim on") || !strcmp(line, "armed sim off")) {
      // bench: the flash erase rule sees armed motors (a simulated run on a board with no ODrive)
      bench_sim_armed = !strcmp(line, "armed sim on");
      usbOut.printf("BENCH armed sim %s: flash erase %s\n", bench_sim_armed ? "on" : "off", bench_sim_armed ? "held" : "per the rule");
    } else if (!strcmp(line, "erase off") || !strcmp(line, "erase on")) {
      bench_erase_block = !strcmp(line, "erase off");
      usbOut.printf("BENCH flash ring erase %s\n", bench_erase_block ? "blocked" : "allowed");
    } else if (!strncmp(line, "ota ", 4)) {
      update_last_ms = millis();
      usb_ota::command(line + 4);
    } else if (!strncmp(line, "flashtest ", 10)) {
      flashTestCommand(line + 10);
    } else if (!strncmp(line, "log", 3)) {
      telemetryUsbCommand(line);
    } else if (!strncmp(line, "send ", 5)) {
      benchSend(line + 5);
    } else if ((!strcmp(line, "restart") || !strcmp(line, "dld")) && !vehicleStationary()) {
      usbOut.printf("%s refused: the vehicle moves\n", line);  // never a restart while the vehicle moves
      telemetryEvent(rec::EV_RESTART_REFUSED, strcmp(line, "dld") ? 1 : 2, 0, "USB restart refused: the vehicle moves");
    } else if (!strcmp(line, "restart")) {
      usbOut.println("BENCH restart");  // software reset: the telemetry rings survive it
      restartApp();
    } else if (!strcmp(line, "monitor")) {
      // A restart before "ota confirm" makes the bootloader return to the previous image.
      esp_ota_img_states_t st;
      const bool unconfirmed = esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK &&
                               st == ESP_OTA_IMG_PENDING_VERIFY;
      if (!vehicleStationary()) usbOut.println("MONITOR refused: the vehicle moves");
      else if (unconfirmed) usbOut.println("MONITOR refused: the new image waits for ota confirm");
      else if (update_mode) usbOut.println("MONITOR refused: update mode");
      else {
        usbOut.println("MONITOR restart into the monitor role (this boot only)");
        monitor_once = MONITOR_ONCE_MAGIC;
        restartApp();
      }
    } else if (!strcmp(line, "dld")) {
      usbOut.println("BENCH restart into ROM download mode");  // for esptool, with no 1200 bps touch
      restartIntoRom();
    } else if (!strcmp(line, "crash")) {  // bench: a forced panic, to check the record after a crash
      if (!vehicleStationary()) usbOut.println("crash refused: the vehicle moves");
      else { usbOut.println("BENCH crash: abort() now"); usbOut.drain(200); abort(); }
    } else if (!strncmp(line, "stm ", 4)) {
      stmRawCommand(line + 4);
    } else if (!strcmp(line, "boot")) {
      usbOut.printf("BOOT rover-firmware %s reset=%d setup_ms=%lu first_packet_ms=%lu espnow_start_ms=%lu espnow=%d mac=%s board=%s\n",
        ROVER_FW_VERSION, (int)esp_reset_reason(), (unsigned long)boot_setup_ms, (unsigned long)boot_first_packet_ms,
        (unsigned long)boot_espnow_ms, (int)espnow_started, WiFi.macAddress().c_str(), RX_BOARD);
    } else if (!strcmp(line, "drv")) {
      // bench: the drive chain for the PC Start/Stop trace: mode, rover state, the ODrive commands (CAN)
      // The commanded values come before the motor locks (gyro, arm switch, CAN): a bench board with no
      // gyro shows what rover mode and Start/Stop command, with rc_active 0 (motors locked).
      usbOut.printf("DRV ms=%lu mode=%d rover_enabled=%d rover_active=%d ap=%d out_enabled=%d drive=%.3f rate_req=%.1f dps steer=%.3f rc_active=%d gyro_valid=%d arm=%d can=%d%c",
        (unsigned long)millis(), (int)drive_mode_status, (int)rover_enabled, (int)rover_active, rover_active ? (int)roverAp.state : -1,
        (int)odrive_output_enabled, odrive_drive_command, steering_rate_target_dps, odrive_steer_command, (int)odrive_rc_active,
        (int)steeringRateGyroValid(), digitalRead(MOTOR_ARM_SWITCH_PIN), (int)can_init_ok, 10);
    } else if (!strcmp(line, "scan start")) {
      // channel scan: Tx off, magnitude mode on all bins, the vehicle's frame period; drive in manual mode
      if (stm_link_ver != 2) { usbOut.println("SCAN needs the v2 STM32 link"); }
      else {
        if (!scan_hist) scan_hist = (uint32_t (*)[SCAN_STEPS])malloc(sizeof(uint32_t) * WFRX_BINS * SCAN_STEPS);
        if (!scan_hist) { usbOut.println("SCAN failed: no memory"); }
        else {
          memset(scan_hist, 0, sizeof(uint32_t) * WFRX_BINS * SCAN_STEPS);
          for (unsigned bn = 0; bn < WFRX_BINS; bn++) scan_max[bn] = scan_max_drive[bn] = 0;
          scan_n = 0; scan_t0 = millis(); scan_on = true;
          usbOut.println("SCAN started: Tx off; drive idle, part throttle, full throttle and braking for 3000 frames or more; 'scan' reports, 'scan stop' ends");
        }
      }
    } else if (!strcmp(line, "scan stop")) {
      scan_on = false;
      scanReport();
    } else if (!strcmp(line, "scan")) {
      scanReport();
    } else if (!strncmp(line, "scan wire ", 10)) {
      const float w = strtof(line + 10, nullptr);
      if (w > 0) scan_wire = w;
      usbOut.printf("SCAN wire level %.0f\n", scan_wire);
    } else if (!strcmp(line, "rt")) {
      // bench: real-time path timing since the last "rt" (lib/rt_prof): frame edge -> control -> CAN, loop passes
      static int64_t rt_t0 = 0;
      static uint32_t rt_f0 = 0;
      const int64_t now = esp_timer_get_time();
      const uint32_t frames = stmDec.st.frames;
      const uint32_t per = (rt_t0 && frames > rt_f0) ? (uint32_t)((now - rt_t0) / (frames - rt_f0)) : 10000;
      rt_t0 = now; rt_f0 = frames;
      static char b[1100];
      rtprof::report(b, sizeof(b), SEC_NAMES, SEC_N, per);
      usbOut.printf("%s%c", b, 10);
    } else if (!strncmp(line, "usbload ", 8)) {
      unsigned long bps = 0, sec = 0;
      if (sscanf(line + 8, "%lu %lu", &bps, &sec) == 2 && bps <= 200000 && sec <= 7200) {
        usbload_bps = bps; usbload_t0 = millis(); usbload_end = usbload_t0 + sec * 1000; usbload_sent = 0;
        usbload_seq = 0; usbload_skip = 0;
        usbOut.printf("BENCH usbload start bps=%lu s=%lu dropped=%lu stalls=%lu reconnects=%lu tx_events=%lu\n", bps, sec,
                      (unsigned long)usbOut.dropped, (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts(),
                      (unsigned long)usb_guard::tx_events());
      } else usbOut.println("BENCH usbload: usage usbload <bytes_per_s 0..200000> <seconds 0..7200>");
    } else if (!strncmp(line, "radio", 5)) {
      // bench: "radio slot <us>" | "radio slot off" | "radio": slot and send-done histogram (then cleared)
      if (!strncmp(line, "radio slot ", 11)) radio_slot_us = strcmp(line + 11, "off") ? atoi(line + 11) : -1;
      usbOut.printf("RADIO slot=%ld len=%lu period_us=%lu hist=", (long)radio_slot_us, (unsigned long)RADIO_SLOT_LEN_US,
                    (unsigned long)(stmFramePeriodS() * 1e6));
      for (int i = 0; i < 11; i++) { usbOut.printf("%lu%c", (unsigned long)radio_tx_hist[i], i < 10 ? ',' : 10); radio_tx_hist[i] = 0; }
    } else if (!strcmp(line, "rt bench")) {
      const double mhz = ESP.getCpuFreqMHz();
      usbOut.printf("RTBENCH live n=%lu read mean=%.1f max=%.1f us decode mean=%.1f max=%.1f us\n", (unsigned long)rt_bench_n,
                    rt_bench_n ? rt_read_cyc / mhz / rt_bench_n : 0.0, rt_read_max / mhz,
                    rt_bench_n ? rt_dec_cyc / mhz / rt_bench_n : 0.0, rt_dec_max / mhz);
      rt_read_cyc = rt_dec_cyc = 0; rt_read_max = rt_dec_max = 0; rt_bench_n = 0;
      if (rt_last_ok) {
        static stmv2::Decoder d2;
        d2 = stmDec;  // same calibration and lanes; its counters do not touch the live decoder
        static __StmTxPacketStruct it;
        static stmv2::Answer an;
        uint32_t mx = 0; uint64_t sum = 0;
        for (int i = 0; i < 1000; i++) {
          const uint32_t a = ESP.getCycleCount();
          d2.decode(rt_last_frame, it, an);
          const uint32_t b = ESP.getCycleCount();
          sum += b - a; if (b - a > mx) mx = b - a;
        }
        uint32_t a = ESP.getCycleCount();
        volatile uint16_t cr = 0;
        for (int i = 0; i < 1000; i++) cr ^= stmv2::crc16(rt_last_frame, STM_FRAME_LEN - 2);
        const double crc_us = (ESP.getCycleCount() - a) / mhz / 1000.0;
        usbOut.printf("RTBENCH stored decode x1000 mean=%.1f max=%.1f us crc16(126 B)=%.1f us\n", sum / mhz / 1000.0, mx / mhz, crc_us);
      }
      volatile float x = 1.2345f, y = 0.789f, z = 0;
      volatile double dx = 1.2345, dy = 0.789, dz = 0;
      struct { const char* name; int op; } ops[] = {{"fmul", 0}, {"fdiv", 1}, {"sqrtf", 2}, {"atan2f", 3}, {"powf", 4}, {"sinf", 5},
                                                    {"dmul", 6}, {"ddiv", 7}, {"sqrt", 8}};
      char lb[300]; int m = snprintf(lb, sizeof(lb), "RTBENCH op ns:");
      for (auto& o : ops) {
        const uint32_t a = ESP.getCycleCount();
        for (int i = 0; i < 1000; i++) {
          switch (o.op) {
            case 0: z = x * y; break;
            case 1: z = x / y; break;
            case 2: z = sqrtf(x); break;
            case 3: z = atan2f(x, y); break;
            case 4: z = powf(x, -0.330478f); break;
            case 5: z = sinf(x); break;
            case 6: dz = dx * dy; break;
            case 7: dz = dx / dy; break;
            default: dz = sqrt(dx); break;
          }
        }
        m += snprintf(lb + m, sizeof(lb) - m, " %s=%.0f", o.name, (ESP.getCycleCount() - a) * 1000.0 / mhz / 1000.0);
      }
      (void)z; (void)dz;
      usbOut.printf("%s (loop overhead included)\n", lb);
    } else if (!strcmp(line, "cpu")) {
      // bench: CPU share per FreeRTOS task and per loop part since the last "cpu" (lib/cpu_prof)
      char b[900];
      cpuprof::report(b, sizeof(b), SEC_NAMES, SEC_N);
      usbOut.printf("%s%c", b, 10);
    } else if (!strcmp(line, "stat")) {
      benchPrintCounters("BENCH stat", 0, 0, 0);
      const stmset::Counters& c = stmLink.counters();
      usbOut.printf("STMCMD stat sent=%lu answers=%lu retries=%lu no_answer=%lu wrong_answer=%lu checks=%lu read_back_bad=%lu resends=%lu gaps=%lu queue_full=%lu events_lost=%lu iap_boots=%lu iap_fails=%lu\n",
        (unsigned long)c.sent, (unsigned long)c.replies, (unsigned long)c.retries, (unsigned long)c.no_reply,
        (unsigned long)c.bad_reply, (unsigned long)c.checks, (unsigned long)c.mismatches, (unsigned long)c.resyncs,
        (unsigned long)c.gaps, (unsigned long)c.queue_full, (unsigned long)c.events_lost,
        (unsigned long)stm_iap_boots, (unsigned long)stm_iap_fails);
      usbOut.printf("STMLINK ver=%u frames=%lu crc_err=%lu ver_err=%lu seq_lost=%lu refused=%lu stm_resets=%lu\n",
        (unsigned)stm_link_ver, (unsigned long)stmDec.st.frames, (unsigned long)stmDec.st.crc_err,
        (unsigned long)stmDec.st.ver_err, (unsigned long)stmDec.st.seq_lost, (unsigned long)c.refused, (unsigned long)c.stm_resets);
    } else if (!strcmp(line, "list")) {
      String kv;
      int n_keys = 0;
      for (; msc_fat_key_at(&g_deviceConfig, n_keys, &kv); n_keys++) {
        if (usbOut.room() < 200) usbOut.drain(100);  // 84 lines overrun the USB text queue: let it empty
        usbOut.printf("CFG 0 %s\n", kv.c_str());
      }
      usbOut.printf("CFG 0 end %d def %d\n", n_keys, SETTINGS_DEF_VERSION);  // key count; settings_def.json version
    } else if (!strncmp(line, "get ", 4) || !strncmp(line, "set ", 4) || !strcmp(line, "save")
               || !strcmp(line, "run start") || !strcmp(line, "run stop")) {
      String reply;  // the same config commands as ESP-NOW CMD 0x10, for a board with ESP-NOW off
      const uint8_t r = configCommand(line, false, reply);
      usbOut.printf("CFG %u %s\n", (unsigned)r, reply.c_str());
    } else {
      usbOut.printf("BENCH unknown command: %s\n", line);
    }
  }
}


// IAP boot handshake ---------------------------------------------------------------------------------------------------------------
#define IAP_CID_HELLO             0x0001
#define IAP_CID_BOOT              0x0005
#define IAP_STATUS_OK             0x00
#define IAP_MAX_FRAME             32    // largest expected frame is 13 bytes; 32 gives headroom
#define IAP_HELLO_RETRIES         10
#define IAP_HELLO_TIMEOUT_MS      100
#define IAP_BOOT_TIMEOUT_MS       100
#define IAP_DATA_READY_TIMEOUT_MS 200

static void iap_encode_frame(uint16_t cid, uint8_t *buf) {
  // Zero-payload frame: length(2LE) + cid(2LE) + crc16_ibm(2LE)
  buf[0] = 0x06; buf[1] = 0x00;
  buf[2] = cid & 0xFF; buf[3] = (cid >> 8) & 0xFF;
  uint16_t crc = crc16_ibm(buf, 4);
  buf[4] = crc & 0xFF; buf[5] = (crc >> 8) & 0xFF;
}

static bool iap_read_frame(uint32_t timeout_ms, uint16_t *cid_out, uint8_t *payload_out, uint8_t *payload_len_out) {
  uint32_t deadline = millis() + timeout_ms;

  while (Serial1.available() < 4) { if (millis() > deadline) return false; }
  uint8_t hdr[4];
  Serial1.readBytes(hdr, 4);
  uint16_t length = hdr[0] | ((uint16_t)hdr[1] << 8);
  uint16_t cid    = hdr[2] | ((uint16_t)hdr[3] << 8);
  if (length < 6 || length > IAP_MAX_FRAME) return false;

  uint8_t rest_len = length - 4;  // payload + 2 CRC bytes
  uint8_t rest[IAP_MAX_FRAME];
  while (Serial1.available() < rest_len) { if (millis() > deadline) return false; }
  Serial1.readBytes(rest, rest_len);

  uint8_t plen = length - 6;
  uint16_t crc_recv = rest[plen] | ((uint16_t)rest[plen + 1] << 8);
  uint8_t crc_buf[4 + IAP_MAX_FRAME];
  memcpy(crc_buf, hdr, 4);
  memcpy(crc_buf + 4, rest, plen);
  if (crc16_ibm(crc_buf, 4 + plen) != crc_recv) return false;

  *cid_out = cid;
  *payload_len_out = plen;
  memcpy(payload_out, rest, plen);
  return true;
}

void stm32_boot_handshake() {
  stmCalLoad();
  if (stmIapHoldTake()) {  // one-shot: HELLO only, then the USB bridge (stmBridgeService)
    uint8_t frame[6], payload[IAP_MAX_FRAME], plen = 0;
    uint16_t cid = 0;
    bool ok = false;
    for (int i = 0; i < IAP_HELLO_RETRIES && !ok; i++) {
      serial1Drain();
      iap_encode_frame(IAP_CID_HELLO, frame);
      Serial1.write(frame, 6);
      ok = iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen) && cid == IAP_CID_HELLO;
    }
    usbOut.printf("STM bridge on: STM32 held in its boot checker (HELLO %s); USB is now the STM32 UART, no text until it ends\n",
                  ok ? "answered" : "not answered");
    usbOut.drain(500);
    serial1Drain();
    data_ready = 0;  // edges from the STM32 power-up are not an application
    stm_bridge = true;
    stm_bridge_start_ms = millis();
    return;
  }
  // After a firmware upload only the ESP32 resets; the STM32 is already running
  // its application and will not respond to IAP HELLO.  Skip the handshake for
  // any reset reason other than a cold power-on so we don't restart-loop.
  // If the STM32 is silent after an ESP32-only reset, it waits in its bootloader
  // (for example after an STM32 load), so send HELLO/BOOT anyway. The running app
  // signals a packet on nDATA_READY every 10 ms.
  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason != ESP_RST_POWERON) {
    data_ready = 0;
    delay(50);
    if (data_ready > 0 || Serial1.available() > 0) {
      usbOut.print("STM32 handshake skipped (reset reason: ");
      usbOut.print((int)reset_reason);
      usbOut.println(", STM32 app running)");
      return;
    }
    usbOut.println("STM32 silent after reset: sending HELLO/BOOT");
  }

  char t[72];
  uint8_t plen = 0;
  const int r = stm32_iap_boot(IAP_HELLO_RETRIES, t, sizeof(t), &plen);
  if (r == 1) {
    usbOut.println("IAP HELLO failed");
    telemetryEvent(rec::EV_STM32, 0, 0, "IAP HELLO failed: STM32 gives no answer");
    return;  // STM32 may already be running; continue without handshake
  }
  telemetryEvent(rec::EV_STM32, 1, plen, t);  // the HELLO answer as it came (IAP and app versions, image state)
  if (r == 2) usbOut.println("IAP BOOT failed");
  else if (r == 3) usbOut.println("IAP nDATA_READY timeout");
  else usbOut.println("STM32 boot OK");
}

// Drain the STM32 UART input: only the bytes there now, read in blocks. A byte-by-byte drain
// ("while available: read") never ends when frames arrive faster than one read() per byte (bench:
// 2.3 ms frames, a byte each 18 us): loop() hung and the task watchdog reset the board.
size_t serial1Drain(){
  uint8_t tmp[128];
  size_t total = 0;
  int n = Serial1.available();
  while (n > 0) {
    const size_t k = Serial1.read(tmp, n < (int)sizeof(tmp) ? (size_t)n : sizeof(tmp));
    if (k == 0) break;
    total += k;
    n -= (int)k;
  }
  return total;
}

// Boot guard. The STM32 keeps its frame period over an ESP32 reset. At frames below about 2.7 ms the loop
// did not keep up (task watchdog reset, set-up S2N), so the next boot would reset again. Before loop()
// starts: more than 100 frames a second (more than about 1500 B in 50 ms) can only come from a v2 STM32;
// it gets a write of the frame period 10000 us. A v1 STM32 sends 100 packets a second and gets nothing.
void stmFastFrameGuard(){
  for (int attempt = 0; attempt < 3; attempt++) {
    const uint32_t t0 = millis();
    uint32_t n = 0;
    while (millis() - t0 < 50) {
      n += serial1Drain();
      delay(1);
    }
    if (n < 1500) return;
    uint8_t f[ESP_FRAME_LEN];
    stmv2::encodeCmd((uint8_t)esp_random(), 18 | ESP_CID_WRITE, 0, 10000, f);
    Serial1.write(f, ESP_FRAME_LEN);
    usbOut.printf("STM32 fast frames at boot (%lu B in 50 ms): frame period 10000 us sent\n", (unsigned long)n);
    delay(30);
  }
}

// HELLO (up to hello_tries sends), then BOOT, then the first nDATA_READY of the STM32 application.
// Result: 0 ok, 1 no HELLO answer, 2 BOOT failed, 3 nDATA_READY timeout. hello_text: the HELLO
// answer as it came, "hello <n> B: xx ..." (IAP and application versions, image state).
int stm32_iap_boot(int hello_tries, char* hello_text, size_t text_len, uint8_t* hello_len){
  uint8_t frame[6];
  uint8_t payload[IAP_MAX_FRAME];
  uint8_t plen;
  uint16_t cid;
  hello_text[0] = 0;
  *hello_len = 0;

  // Flush, send HELLO, wait for HELLO response; retry on flush-race or garbled frame
  bool hello_ok = false;
  for (int i = 0; i < hello_tries && !hello_ok; i++) {
    serial1Drain();
    iap_encode_frame(IAP_CID_HELLO, frame);
    Serial1.write(frame, 6);
    if (!iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen)) continue;
    if (cid != IAP_CID_HELLO) continue;
    hello_ok = true;
  }
  if (!hello_ok) return 1;
  {
    int k = snprintf(hello_text, text_len, "hello %u B:", (unsigned)plen);
    for (int j = 0; j < plen && k < (int)text_len - 3; j++) k += snprintf(hello_text + k, text_len - k, " %02x", payload[j]);
    *hello_len = plen;
  }

  // BOOT — STM32 state confirmed by HELLO, single attempt
  iap_encode_frame(IAP_CID_BOOT, frame);
  Serial1.write(frame, 6);
  if (!iap_read_frame(IAP_BOOT_TIMEOUT_MS, &cid, payload, &plen) ||
      cid != IAP_CID_BOOT || plen < 1 || payload[0] != IAP_STATUS_OK) {
    return 2;
  }

  // Wait for STM32 app to assert nDATA_READY for the first time
  uint32_t deadline = millis() + IAP_DATA_READY_TIMEOUT_MS;
  while (digitalRead(STM_nDATA_READY) != LOW) {
    if (millis() > deadline) return 3;
  }
  return 0;
}

// An STM32 that restarts alone stays in its boot checker: IAP 0.1.0 (TrailTracer-Rx-STM32
// iap_fsm.c) sends one HELLO frame at its start and then waits for BOOT, so no packet comes and
// the STM32 settings are lost. After STM_RECOVER_GAP_MS with no valid packet (packets seen earlier
// in this boot), send HELLO/BOOT again: 3 HELLO tries, at most 3 times per gap, 5 s apart. Only
// when the line is silent for 50 ms: a running STM32 application reads 8-byte commands, and a 6-byte
// HELLO would shift every later command. When packets come back, the link sends the five settings
// again (gap event). Not after "stm rom" (a ROM loader may listen), not in update mode, not with
// the drive off for a failed IMU (no packet interrupt). Blocks loop() for 50 ms to 650 ms; the
// drive outputs stopped already, 250 ms after the last packet (ODRIVE_OUTPUT_TIMEOUT_MS).
#define STM_RECOVER_GAP_MS 1000
void stmRecoveryService(){
  if (stm_raw || stm_rom_used || imu_halt || update_mode || !stm_packet_seen) return;
  const uint32_t now = millis();
  if (now - stm_last_packet_ms < STM_RECOVER_GAP_MS) { stm_recover_tries = 0; return; }
  if (stm_recover_tries >= 3 || (stm_recover_tries > 0 && now - stm_recover_ms < 5000)) return;
  stm_recover_tries++;
  stm_recover_ms = now;
  const int pending = Serial1.available();  // the boot checker's own HELLO frame at its start: 13 bytes
  serial1Drain();
  data_ready = 0;
  delay(50);
  char t[72];
  if (data_ready > 0 || Serial1.available() > 0) {
    snprintf(t, sizeof(t), "silent %lu ms, line not silent (%lu edges, %d B): no HELLO",
             (unsigned long)(now - stm_last_packet_ms), (unsigned long)data_ready, Serial1.available());
    usbOut.printf("STM32 %s\n", t);
    telemetryEvent(rec::EV_STM32_CMD, stmset::K_IAP_BOOT, -1, t);
    return;
  }
  char hello[72];
  uint8_t plen = 0;
  const int r = stm32_iap_boot(3, hello, sizeof(hello), &plen);
  serial1Drain();
  data_ready = 0;
  if (r == 0) stm_iap_boots++; else stm_iap_fails++;
  static const char* const RESULT[] = {"boot ok", "no HELLO answer", "BOOT failed", "no nDATA_READY"};
  snprintf(t, sizeof(t), "silent %lu ms, %d B: %s; %s", (unsigned long)(now - stm_last_packet_ms), pending,
           RESULT[r], hello[0] ? hello : "-");
  usbOut.printf("STM32 HELLO/BOOT try %u: %s\n", (unsigned)stm_recover_tries, t);
  telemetryEvent(rec::EV_STM32_CMD, stmset::K_IAP_BOOT, r, t);
}

// Install and start native TWAI in normal mode; no external CAN controller.
bool odriveCan_init() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  // UART/RMT/GPIO consume level-1 interrupts on the single-core ESP32-S2.
  // Use a dedicated level-2 interrupt; IRAM requires a different SDK build.
  general.intr_flags = ESP_INTR_FLAG_LEVEL2;
  general.tx_queue_len = 16;
  general.rx_queue_len = 128;  // Buffer bursts while the application handles STM32 data.
  general.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_RX_QUEUE_FULL;
#if CAN_BUS_BITRATE == 1000000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BUS_BITRATE == 250000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
#else
#error "Select a TWAI timing configuration matching CAN_BUS_BITRATE"
#endif
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  /* usbOut.println("Boot: TWAI driver install (level-2 interrupt)"); */;
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  /* usbOut.printf("Boot: TWAI install returned %s\n", esp_err_to_name(err)); */;
  if (err != ESP_OK) {
    /* usbOut.printf("TWAI install FAILED: %s\n", esp_err_to_name(err)); */;
    return false;
  }
  can_driver_installed = true;
  /* usbOut.println("Boot: TWAI driver start"); */;
  err = twai_start();
  if (err != ESP_OK) {
    /* usbOut.printf("TWAI start FAILED: %s\n", esp_err_to_name(err)); */;
    twai_driver_uninstall();
    can_driver_installed = false;
    return false;
  }
  /* usbOut.printf("TWAI init OK: TX=%d RX=%d bitrate=%u normal mode\n",
                (int)TWAI_TX_PIN, (int)TWAI_RX_PIN, (unsigned)CAN_BUS_BITRATE); */;
  return true;
}

// Success means queued for transmission, not acknowledged by the ODrive.
// Single-shot prevents unacknowledged frames from retrying indefinitely.
bool sendODriveCAN(uint8_t nodeID, uint8_t commandID,
                   const uint8_t *data, uint8_t length, bool remote = false) {
  if (nodeID > 0x3F || commandID > 0x1F || length > 8 ||
      (!remote && length && !data) || !can_init_ok) return false;
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK ||
      status.state != TWAI_STATE_RUNNING) return false;
  twai_message_t frame = {};
  frame.identifier = (static_cast<uint32_t>(nodeID) << 5) | commandID;
  frame.data_length_code = length;
  frame.rtr = remote;
  frame.ss = 1;
  if (!remote && length) memcpy(frame.data, data, length);
  // Zero timeout: a full queue fails immediately, preserving loop responsiveness.
  esp_err_t err = twai_transmit(&frame, 0);
  if (err != ESP_OK) {
    static uint32_t last_report = 0;
    if (millis() - last_report >= 1000) {
      last_report = millis();
      twai_get_status_info(&status);
      /* usbOut.printf("TWAI TX queue failed: id=0x%03lX %s pending=%lu RXpin=%d bus_errors=%lu arb_lost=%lu\n",
                    (unsigned long)frame.identifier, esp_err_to_name(err),
                    (unsigned long)status.msgs_to_tx, gpio_get_level(TWAI_RX_PIN),
                    (unsigned long)status.bus_error_count,
                    (unsigned long)status.arb_lost_count); */;
    }
  }
  return err == ESP_OK;
}

// Exposes arbitration ID, DLC and payload in the native TWAI message.
bool receiveODriveCAN(twai_message_t &frame) {
  return can_init_ok && twai_receive(&frame, 0) == ESP_OK;
}

// Commands an ODrive axis into the given AXIS_STATE (see ODrive Enums.AxisState)
// Match nano_temp.ino: the axis-state payload is one uint32_t (four bytes).
void odrive_setAxisState(uint8_t node_id, uint32_t axis_state) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_AXIS_STATE;
  frame.data_length_code = 4;
  memset(frame.data, 0, sizeof(frame.data));
  memcpy(frame.data, &axis_state, sizeof(axis_state));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sends a velocity setpoint (turns/s) with optional torque feed-forward (Nm) to an ODrive axis
void odrive_setInputVel(uint8_t node_id, float vel, float torque_ff = 0.0f) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_INPUT_VEL;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &vel, sizeof(vel));
  memcpy(&frame.data[4], &torque_ff, sizeof(torque_ff));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sets an ODrive axis velocity limit [rev/s] and current limit [A]
void odrive_setLimits(uint8_t node_id, float vel_limit, float current_limit) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_LIMITS;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &vel_limit, sizeof(vel_limit));
  memcpy(&frame.data[4], &current_limit, sizeof(current_limit));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Sets an ODrive axis control mode and input mode (see ODrive ControlMode/InputMode enums)
void odrive_setControllerMode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_SET_CONTROLLER_MODE;
  frame.data_length_code = 8;
  memcpy(&frame.data[0], &control_mode, sizeof(control_mode));
  memcpy(&frame.data[4], &input_mode, sizeof(input_mode));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

// Clears all errors on an ODrive axis (and re-arms the brake resistor etc.)
// Match nano_temp.ino's zero-length Clear_Errors command.
void odrive_clearErrors(uint8_t node_id) {
  twai_message_t frame = {};
  frame.identifier = (node_id << 5) | ODRIVE_CMD_CLEAR_ERRORS;
  frame.data_length_code = 0;
  memset(frame.data, 0, sizeof(frame.data));
  sendODriveCAN(node_id, frame.identifier & 0x1F, frame.data, frame.data_length_code);
}

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

// Recovery runs asynchronously: initiate once, poll state, restart after recovery.
void odrive_recoverIfNeeded() {
  if (!can_driver_installed) return;
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);
  static uint32_t last_report = 0;
  if (alerts && millis() - last_report >= 1000) {
    last_report = millis();
    /* usbOut.printf("TWAI alerts: 0x%08lX\n", (unsigned long)alerts); */;
  }
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;
  can_init_ok = status.state == TWAI_STATE_RUNNING;
  if (status.state == TWAI_STATE_BUS_OFF) {
    esp_err_t err = twai_initiate_recovery();  // Clears pending TX; never waits for recovery.
    if (err == ESP_OK) /* usbOut.println("TWAI bus-off: recovery started"); */;
  } else if (status.state == TWAI_STATE_STOPPED) {
    esp_err_t err = twai_start();  // Recovery completes in the stopped state.
    can_init_ok = err == ESP_OK;
    if (can_init_ok) /* usbOut.println("TWAI recovered: normal mode restarted"); */;
  }
}

// Requests encoder estimates from one ODrive node (cycles through nodes each call).
// The ODrive responds with a Get_Encoder_Estimates message.
void odrive_requestEncoderEstimates() {
  if (!can_init_ok) return;
  // Wheel speeds of nodes 0 and 1 for the telemetry log: ask only a node whose own messages stopped.
  static uint32_t vel_poll_ms = 0;
  static uint8_t vel_node = 0;
  if (millis() - vel_poll_ms >= 10) {
    vel_poll_ms = millis();
    vel_node ^= 1;
    if (millis() - odrive_vel_millis[vel_node] > ODRIVE_VEL_STALE_MS)
      sendODriveCAN(vel_node, ODRIVE_CMD_GET_ENCODER_ESTIMATES, nullptr, 8, true);
  }
  if (millis() - odrive_encoder_poll_millis < ODRIVE_ENCODER_POLL_MS) return;

  static uint8_t poll_node = 0;
  twai_message_t frame = {};
  frame.identifier = (poll_node << 5) | ODRIVE_CMD_GET_ENCODER_ESTIMATES;
  frame.data_length_code = 8;  // RTR requests the eight-byte response; no data is transmitted.
  memset(frame.data, 0, sizeof(frame.data));
  bool queued = sendODriveCAN(poll_node, ODRIVE_CMD_GET_ENCODER_ESTIMATES, nullptr,
                              frame.data_length_code, true);
  // Request node 0 bus voltage alongside the periodic encoder poll.
  sendODriveCAN(0, ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT, nullptr, 8, true);
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    sendODriveCAN(node, ODRIVE_CMD_GET_ERROR, nullptr, 8, true);
  }
  if (poll_node == 5) {
    /* usbOut.printf("ODrive node 5 encoder request: id=0x0A9 RTR=1 DLC=8 %s\n",
                  queued ? "queued (delivery not confirmed)" : "queue failed"); */;
  }

  poll_node++;
  if (poll_node >= ODRIVE_NODE_COUNT) poll_node = 0;
  odrive_encoder_poll_millis = millis();
}

// Drains CAN RX and reports new faults and closed-loop-to-idle transitions.
void odrive_printErrorNames(uint32_t errors);
void odrive_pollAndPrintCan() {
  if (!can_init_ok) return;
  twai_message_t frame = {};
  const uint32_t started_us = micros();
  // Drain bursts with bounded work; inspect every received frame for events.
  for (uint8_t received = 0; received < 64 &&
       micros() - started_us < 2000 && receiveODriveCAN(frame); received++) {
    if (frame.extd || frame.rtr || frame.data_length_code > 8) continue;
    uint8_t node_id = frame.identifier >> 5;
    uint8_t cmd_id = frame.identifier & 0x1F;
    if (node_id >= ODRIVE_NODE_COUNT) continue;
    if (cmd_id == ODRIVE_CMD_HEARTBEAT && frame.data_length_code >= 7) {
      const uint32_t previous_errors = odrive_axis_errors[node_id];
      const bool went_idle = odrive_heartbeat_received[node_id] &&
        odrive_axis_states[node_id] == ODRIVE_AXIS_STATE_CLOSED_LOOP &&
        frame.data[4] == ODRIVE_AXIS_STATE_IDLE;
      memcpy(&odrive_axis_errors[node_id], frame.data, sizeof(uint32_t));
      const uint32_t new_errors = odrive_axis_errors[node_id] & ~previous_errors;
      if (new_errors || went_idle) {
        usbOut.printf("ODrive N%u: %s error=0x%08lX procedure=%u status6=%u", node_id,
          went_idle ? "CLOSED_LOOP -> IDLE" : "axis fault",
          (unsigned long)odrive_axis_errors[node_id], frame.data[5], frame.data[6]);
        odrive_printErrorNames(new_errors);
        usbOut.println();
      }
      odrive_seen_errors[node_id] |= odrive_axis_errors[node_id];
      odrive_axis_states[node_id] = frame.data[4];
      // Preserve raw bytes: newer firmware uses procedure result/trajectory done;
      // older firmware can use motor/encoder/controller flags here.
      odrive_status_byte5[node_id] = frame.data[5];
      odrive_status_byte6[node_id] = frame.data[6];
      odrive_heartbeat_received[node_id] = true;
      odrive_heartbeat_millis[node_id] = millis();
    }
    if (cmd_id == ODRIVE_CMD_GET_ERROR && frame.data_length_code == 8) {
      const uint32_t previous_active = odrive_active_errors[node_id];
      const uint32_t previous_disarm = odrive_disarm_reasons[node_id];
      memcpy(&odrive_active_errors[node_id], frame.data, sizeof(uint32_t));
      memcpy(&odrive_disarm_reasons[node_id], &frame.data[4], sizeof(uint32_t));
      odrive_seen_errors[node_id] |= odrive_active_errors[node_id] | odrive_disarm_reasons[node_id];
      const uint32_t new_active = odrive_active_errors[node_id] & ~previous_active;
      const uint32_t new_disarm = odrive_disarm_reasons[node_id] & ~previous_disarm;
      if (new_active || new_disarm) {
        usbOut.printf("ODrive N%u: active=0x%08lX disarm=0x%08lX", node_id,
          (unsigned long)odrive_active_errors[node_id],
          (unsigned long)odrive_disarm_reasons[node_id]);
        odrive_printErrorNames(new_active | new_disarm);
        usbOut.println();
      }
      odrive_error_received[node_id] = true;
      odrive_error_millis[node_id] = millis();
    }
    if (cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES && frame.data_length_code == 8) {
      memcpy(&odrive_vel_rps[node_id], &frame.data[4], sizeof(float));  // pos [rev] at 0, vel [rev/s] at 4
      memcpy(&odrive_pos_rev[node_id], &frame.data[0], sizeof(float));
      odrive_vel_millis[node_id] = millis();
      odrive_vel_received[node_id] = true;
    }
    // Keep bus telemetry cached without printing periodic updates.
    if (node_id == 0 && cmd_id == ODRIVE_CMD_GET_BUS_VOLTAGE_CURRENT &&
        frame.data_length_code >= 4) {
      memcpy(&odrive_node0_vbus, frame.data, sizeof(odrive_node0_vbus));
      odrive_node0_vbus_received = true;
      odrive_node0_vbus_millis = millis();
      if (frame.data_length_code == 8) {
        memcpy(&odrive_node0_ibus, &frame.data[4], sizeof(odrive_node0_ibus));
        odrive_node0_ibus_received = true;
      } else {
        odrive_node0_ibus_received = false;
      }
    }
  }
}

void odrive_printErrorNames(uint32_t errors) {
  // Common Micro errors; all bits are also printed in hex for other faults.
  static const struct { uint32_t bit; const char* name; } names[] = {
    {0x20, "DRV_FAULT"}, {0x100, "DC_BUS_OVER_VOLTAGE"},
    {0x200, "DC_BUS_UNDER_VOLTAGE"}, {0x400, "DC_BUS_OVER_CURRENT"},
    {0x800, "DC_BUS_OVER_REGEN_CURRENT"}, {0x1000, "CURRENT_LIMIT_VIOLATION"},
    {0x2000, "MOTOR_OVER_TEMP"}, {0x4000, "INVERTER_OVER_TEMP"},
    {0x8000, "VELOCITY_LIMIT_VIOLATION"}, {0x1000000, "WATCHDOG_TIMER_EXPIRED"},
    {0x2000000, "ESTOP_REQUESTED"}, {0x4000000, "SPINOUT_DETECTED"}
  };
  for (const auto& entry : names) {
    if (errors & entry.bit) usbOut.printf(" %s", entry.name);
  }
}

// Void Setup --------------------------------------------------------------------------------------------------------------
#ifdef BOOT_TRACE
// Diagnostic build (env boot_trace): USB serial first, the reset reason, then one line per start-up
// step, each flushed, so the last line before an early restart shows the step.
// The step in progress is also kept in RTC memory (kept over a crash restart, not over a power loss), so the
// next start prints "TRACE previous start died during step <n> <name>".
static const char* const TRACE_NAMES[] = {"start", "USB drive and config.json", "apply config, telemetry log", "USB serial",
  "debug UART", "MPU6050 set-up", "CAN (TWAI) start", "RC PWM inputs (RMT)", "STM32 UART and data-ready interrupt",
  "STM32 boot handshake", "ESP-NOW and Wi-Fi start", "loop watchdog", "loop pass 1", "loop pass 2", "loop pass 3", "running"};
static RTC_NOINIT_ATTR uint32_t trace_magic;
static RTC_NOINIT_ATTR uint32_t trace_step;
#define TRACE(n) do { trace_magic = 0x7EACE001u; trace_step = (n); \
  Serial.printf("TRACE %d %s ms=%lu\n", (n), TRACE_NAMES[(n) < 16 ? (n) : 0], (unsigned long)millis()); Serial.flush(); delay(20); } while (0)
#else
#define TRACE(n) do {} while (0)
#endif
void setup() {
  LastState prev_state{};  // last_state of the boot before (valid when magic == LAST_STATE_MAGIC); stack, not static RAM
  // The ESP32 TX line to the STM32 (GPIO17) high at once: a low glitch while the ESP32 starts can
  // lock a blank STM32's ROM loader on a wrong baud rate (bench test).
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
#ifdef BOOT_TRACE
  Serial.begin(115200);
  for (uint32_t t0 = millis(); !tud_cdc_connected() && millis() - t0 < 8000;) delay(10);  // up to 8 s for a terminal
  Serial.printf("TRACE start firmware %s reset=%d (1 power-on, 3 software, 4 crash, 5/6/7 watchdog, 9 brownout)\n",
                ROVER_FW_VERSION, (int)esp_reset_reason());
  if (trace_magic == 0x7EACE001u && trace_step < 15)
    Serial.printf("TRACE previous start died during step %lu %s\n", (unsigned long)trace_step, TRACE_NAMES[trace_step]);
  else if (trace_magic == 0x7EACE001u)
    Serial.printf("TRACE previous start reached the running loop\n");
  trace_magic = 0;
  Serial.flush();
#endif
  cpuprof::begin();  // CPU profiler: loopTask is the task that runs setup()
  const bool monitor_boot = monitor_once == MONITOR_ONCE_MAGIC && esp_reset_reason() == ESP_RST_SW;
  monitor_once = 0;  // never sticky
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const bool crash = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
    if (crash_magic != 0x5AFE0001u || !crash) { crash_magic = 0x5AFE0001u; crash_count = 0; }
    if (crash) crash_count++;
    safe_mode = crash_count >= 2;  // two crash resets, each within 60 s of its start: no telemetry, no rover mode, STM32 at 10 ms
    prev_state = last_state;  // the state before this reset (no power loss: RTC memory kept it)
    if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT) prev_state.magic = 0;
    last_state = LastState{LAST_STATE_MAGIC, 0, 0xFFFFFFFFu, ST_SETUP, 0, 0, 0};
  }

  // Initialise USB MSC + FAT filesystem and load config.
  TRACE(1);
  msc_fat_init(&g_deviceConfig);
  if (monitor_boot && !safe_mode) {
    // Monitor role, this boot only: the board is the PC monitor, not a vehicle. No vehicle hardware
    // starts (no CAN, no outputs, no STM32 link).
    monitor_role_on = true;
    last_state.magic = 0;  // the monitor role keeps no vehicle state
    monitor_role::setup(ROVER_FW_VERSION);
    return;
  }
  TRACE(2);
  applyDeviceConfig(g_deviceConfig);
  if (!safe_mode) telemetryBegin();  // boot_id, the log rings (kept over a warm reset), the boot event

  // Start Serial
  TRACE(3);
  usb_guard::begin();  // before the host opens the port: the DTR/RTS reboot sequence is off
  Serial.setRxBufferSize(2048);  // STM32 bridge: one IAP chunk frame is 1048 B
  Serial.begin(115200);  // Start USB VCP
  Serial.setTxTimeoutMs(0);  // Never block if no CDC host is connected
  usbOut.printf("BOOT rover-firmware %s reset=%d board=%s\n", ROVER_FW_VERSION, (int)esp_reset_reason(), RX_BOARD);  // banner: shows each reboot
  if (prev_state.magic == LAST_STATE_MAGIC) {  // what the board did before this reset
    char t[80];
    snprintf(t, sizeof(t), "stage=%s sec=%s mode=%u ap=%u heap_min=%lu crashes=%lu",
             STAGE_NAMES[prev_state.stage < 8 ? prev_state.stage : 0],
             prev_state.section < SEC_N ? SEC_NAMES[prev_state.section] : "?", prev_state.drive_mode, prev_state.ap_state,
             (unsigned long)prev_state.heap_min, (unsigned long)crash_count);
    usbOut.printf("PREV reset=%d uptime_ms=%lu %s\n", (int)esp_reset_reason(), (unsigned long)prev_state.uptime_ms, t);
    if (!safe_mode) telemetryEvent(rec::EV_PREV_BOOT, (int32_t)esp_reset_reason(), (int32_t)prev_state.uptime_ms, t);
  }
  usb_ota::begin(usb_ota::Hooks{&usbOut, otaMayStart, restartApp});
#ifdef BENCH_DRILL_PANIC
  // Bench drill 1: a new OTA image that crashes at boot. Expect the bootloader to start the
  // previous image. The drill panics once only: if this image starts again (no rollback), it runs
  // normally, so OTA still works and there is no lockout.
  {
    static RTC_NOINIT_ATTR uint32_t drill_magic;
    if (drill_magic != 0xD1A11001u) {
      drill_magic = 0xD1A11001u;
      usbOut.println("DRILL panic in setup (once)");
      usbDetachForRestart();
      abort();
    }
    drill_magic = 0;
    usbOut.println("DRILL FAIL: drill image started again, no rollback");
  }
#endif
  TRACE(4);
  Serial0.setTxBufferSize(1024);  // Prevents log writes from blocking at 1 Mbaud
  Serial0.begin(1000000);  // Start debug UART (visible on tag-connect)
  Serial1.setRxBufferSize(2048);  // ~176 ms headroom at 100 Hz × 116 bytes/packet
  Serial1.begin(2000000);  // Start ESP32 <-> STM32 UART
  Serial1.setRxFIFOFull(64);  // v2.0 layout 2 frame is 128 B = the hardware FIFO: interrupt at 64 B, not 120 B
  pinMode(STM_nDATA_READY, INPUT_PULLUP);


  // Configure pins
  pinMode(nLED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ARM_SWITCH_PIN, INPUT_PULLUP);
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
  writePwmMicros(1, PWM_OUT_1_DEFAULT);
  writePwmMicros(2, PWM_OUT_2_DEFAULT);
  writePwmMicros(3, PWM_OUT_3_DEFAULT);
  writePwmMicros(4, PWM_OUT_4_DEFAULT);

  TRACE(5);
  // MPU6050: the main-branch serviceMpu() (top of this file) owns it: init, calibration, one sample per
  // loop. The main-branch gyro yaw-rate steering needs it; with no MPU its lock keeps the motors off. The log
  // reads the same object (mpu_ok follows gyro_ready). Main branch f10cd31.

  if (!imu_halt) {
  // Install TWAI before allocating RMT and STM32 GPIO interrupts.
  TRACE(6);
  can_init_ok = odriveCan_init();

  // Set up PWM inputs (in progress)
  uint8_t pwm_input_pins[] = {PWM_IN_1, PWM_IN_2, PWM_IN_3, PWM_IN_4};
  TRACE(7);
  pwm_read_rmt_init(pwm_input_pins, PWM_IN_NUM);

  // PID setup
  steeringPID.SetOutputLimits(PWM_OUT_1_MIN - PWM_OUT_1_DEFAULT, PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);  // Set the output contstraints
  steeringPID.SetSampleTime(10);  // Set the sample interval in milliseconds
  steeringPID.SetControllerDirection(pid_direction);  // Apply direction loaded from config
  steeringPID.SetMode(AUTOMATIC);  // Turn on the control loop

  // PID low pass setup
  low_pass_alpha = pow(2.71828183, (-1.0 / (100 * 1/(low_pass_cutoff * 6.2831))));
  low_pass_beta = 1 - low_pass_alpha;

  // Attach falling edge interrupt to STM_nDATA_READY pin
  TRACE(8);
  attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
  pwm_reset_readings();

  TRACE(9);
  stm32_boot_handshake();
  stmFastFrameGuard();  // a fast STM32 frame period left from before this reset: back to 10 ms first

  data_ready = 0;  // Clear any pending data ready flags that may have been set during boot
  stmLink.setSeq((uint8_t)esp_random());  // v2: an ESP32 restart cannot repeat the last seq

  // Set up charge port
  espPacketFifo_addCmdFloat(6 | 0x80000000, CHARGE_VBATT_STOP);  // Set voltage stop
  espPacketFifo_addCmdFloat(7 | 0x80000000, CHARGE_MAX_CURRENT);  // Set overcurrent
  espPacketFifo_addCmdInt32(8 | 0x80000000, (CHARGE_MAX_SECONDS * 2));  // Set charge time limit

  //RX channel setup
  espPacketFifo_addCmdInt32(1 | 0x80000000, (rx_channel - 1));  // write Rx channel
  espPacketFifo_addCmdInt32(2 | 0x80000000, enable_sensor_2 ? 0x00 : 0x80);  // Enable/disable sensor 2
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new channel value
  }  // !imu_halt

  if (espnowWanted()) {
    TRACE(10);
    espnowStart();  // ESP-NOW Setup
  }
  TRACE(11);
  enableLoopWDT();  // loop() must run at least each 5 s (task watchdog, panic restart), as in the monitor role
  boot_setup_ms = millis();
  usbOut.printf("SETUP ms=%lu\n", (unsigned long)boot_setup_ms);  // startup time: nothing above waits for a peer
}

// Main Loop ---------------------------------------------------------------------------------------------------------------
void loop() {
#ifdef BOOT_TRACE
  {
    static uint8_t pass = 0;
    if (pass < 3) { pass++; TRACE(11 + pass); }   // the first 3 loop passes: steps 12, 13, 14
    else if (pass == 3) { pass++; TRACE(15); }    // "running": the start-up finished
  }
#endif
  if (monitor_role_on) { mark(SEC_MONITOR); monitor_role::loop(); return; }
  stage(ST_USB);
  mark(SEC_USB);
  usbOut.service();  // queued USB text, never blocking
  stmCaptureService();  // bench: captured STM32 frames, printed at the USB pace
  usb_guard::service(millis(), vehicleStationary());  // USB output stop: a USB reconnect, no restart (lib/usb_guard)
  usbLoadService();  // bench: paced USB output
  if (stm_bridge) { mark(SEC_BRIDGE); stmBridgeService(); return; }  // STM32 app update through the boot checker: nothing else runs
  if (update_mode) {
    stage(ST_UPDATE);
    mark(SEC_UPDATE);
    // Update mode: from "ota begin" the board serves only USB and the update. No STM32 handling,
    // no telemetry, no radio, no flash log, no autopilot: a busy loop and a full output path made
    // the upload fail and the host lose the board (bench test). Ends with the restart into
    // the new image, or 120 s after the last "ota" line.
    if (millis() - update_last_ms < 120000) {
      usb_ota::service();
      benchSerial();
      static uint32_t update_hb_ms = 0;
      if (millis() - update_hb_ms >= 1000) {
        update_hb_ms = millis();
        usbOut.printf("HB ms=%lu update=1\n", (unsigned long)millis());
      }
      return;
    }
    update_mode = false;
    enableLoopWDT();
    usbOut.println("UPDATE mode off: 120 s without an ota line; loop watchdog on");
  }

  mark(SEC_DRIVE);
  static uint32_t last_button_update = 0;
  if (millis() - last_button_update >= 10) {
    last_button_update = millis();
    buttonLogic();
  }
  odrive_recoverIfNeeded();
  odrive_pollAndPrintCan();
  odrive_updateBuzzer();
  odrive_outputUpdate(false);  // Send calculated outputs; stop if outputs or RC go stale.
  odrive_requestEncoderEstimates();


  // Check if USB MSC host has finished editing config.json (3 s idle window).
  mark(SEC_MSC);
  if (msc_fat_loop(&g_deviceConfig)) {
    applyDeviceConfig(g_deviceConfig);
    config_saved_led_until_ms = millis() + 1500;  // Signal config saved to user via LED
  }

  static uint32_t loop_min_us = 0xFFFFFFFF;
  static uint32_t loop_max_us = 0;
  static uint32_t loop_exec_count = 0;

  stage(ST_PACKET);
  mark(SEC_PACKET);
  last_state.drive_mode = (uint8_t)drive_mode_status;
  last_state.ap_state = telemetryApByte();
  if (stm_raw) stmRawService();  // bench: STM32 ROM loader link, no packets
  else if (stm_link_ver == 2 && stm_v2_resync && data_ready && Serial1.available() != (int)STM_FRAME_LEN) {
    serial1Drain();  // resync: wait for an edge with exactly one frame waiting
    data_ready = 0;
    rtprof::resync();
    uart_overrun_count++;
  }
  else if((stm_link_ver == 2 && Serial1.available() >= (int)STM_FRAME_LEN && (!stm_v2_resync || data_ready)) ||
          (stm_link_ver != 2 && data_ready && (data_ready > 1 || Serial1.available() >= stmFrameLen()))){
    uint32_t loop_start_us = micros();
    // Overrun detection (v1 and unknown link): ISR fired more than once, or extra bytes are buffered from a
    // previous cycle.  Drain and skip this cycle; the next interrupt will bring a
    // fresh, complete packet that is guaranteed to be aligned. Layout 2 keeps late frames (see stm_v2_backlog).
    if(stm_link_ver != 2 && (data_ready > 1 || (stm_link_ver && Serial1.available() > stmFrameLen()))){
      serial1Drain();  // drain stale bytes first
      data_ready = 0;                             // then reset (ISR during drain → clean next cycle)
      rtprof::resync();
      uart_overrun_count++;
      if (++stm_bad_frames_run >= 50 && stm_link_ver) {  // the other version sends: learn it again
        stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
        usbOut.println("STM link version unknown again: 50 bad frames in a row");
      }
      // Fall through to the rest of loop() without touching stmPacket.
    } else {
    // Normal path: exactly one interrupt fired and one full packet is ready.
    if(!stmReadFrame()){
      // a bad frame: counted in stmReadFrame (crc_error_count)
    } else {

    // update the double X position from the float in the latest STM32 packet
    static bool first_packet = true;
    if (first_packet) {  // startup time to control: the first valid STM32 packet
      first_packet = false;
      boot_first_packet_ms = millis();
      usbOut.printf("FIRST_PACKET ms=%lu\n", (unsigned long)boot_first_packet_ms);
    }
    live_position = stmPacket.items.s1_x;
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

    mark(SEC_ROVER);
    roverStep();  // Mode 2 with rover_enabled: the autopilot replaces throttle and steering_output
    mark(SEC_PACKET);


     // Check for faults before commanding skid steering and remaining servo outputs.
     if( WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_OVERLOAD)  //  signal overload fault bit
      || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_VDD_FAULT)  // STM32 VDD fault bit
      || (stmPacket.items.temperature > 100)  // STM32 internal temp in °C
      ){
        odrive_setDriveOutput(PWM_OUT_2_DEFAULT, PWM_OUT_1_DEFAULT, false);
        writePwmMicros(3, PWM_OUT_3_DEFAULT);  // write default value to output 3
        writePwmMicros(4, PWM_OUT_4_DEFAULT);

     }else if(((drive_mode_status == 1 || drive_mode_status == 2)
          && !rover_active && stuckDetector.GetStatus())  // Stop wire-follow modes when stuck (the rover is off the wire most of the time).
          || (charge_status > 0)  // vehicle charge in progress
          || (rover_active && roverAp.state == rover::HALTED)  // rover lost the wire too long
          ){
        odrive_setDriveOutput(PWM_OUT_2_DEFAULT, PWM_OUT_1_DEFAULT, false);
        sine_wave_force_disabled = true; sine_wave_blend = 0.0; target_x_pos = 0.0;
        vehicleStatus = 2; //set status to 'charging' for telemetry purposes
        if (charge_status == 0) { // If there is a fault other than charging
         vehicleStatus = 0; //the vehicle should be considered stuck
         }

     }else{
       odrive_setDriveOutput(throttle, steering_output, true);
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

  mainLoopCounter();  // increment data loop counter
  mark(SEC_SAMPLE);
  telemetrySample();  // one 100 Hz sample of this packet and the outputs it gave
  mark(SEC_STMCMD);
  chargeControl();  // monitor and control charge port
  espPacketFifo_sendCmd();  // Send one STM32 UART command from the command queue

  }  // end else (CRC pass)
  }  // end else (normal packet path)
    uint32_t loop_elapsed_us = micros() - loop_start_us;
    if(loop_elapsed_us < loop_min_us) loop_min_us = loop_elapsed_us;
    if(loop_elapsed_us > loop_max_us) loop_max_us = loop_elapsed_us;
    if(loop_elapsed_us > telem_loop_max_us) telem_loop_max_us = loop_elapsed_us;
    loop_exec_count++;
  }  // end if(data_ready)
  mark(SEC_STMCMD);
  stmRecoveryService();  // STM32 silent after packets: HELLO/BOOT again
  stmLinkEvents();       // STM32 command failures, read-back differences, packet gaps: event 18 and USB
  mark(SEC_RADIO);
  if (espnowWanted() && radioSlotOpen()) {
    espnowStart();  // no-op once started; starts it after a config.json change
    broadcastTelemetry();  // Queue telemetry message if needed
    telemetryFifo_sendMessage();  // Send one telemetry message from queue (throttled to 100ms)
    stage(ST_RADIO);
    espnowService();  // PONG answers and THRU test frames
    espnowCommands();  // config commands (CMD/ACK) and restarts
    monitorLinkService();  // MON frames: "monitor heard" and PC time events
    if (espnow_started) telemlog_poll(millis(), telemetrySend);  // live log frames, INFO, backfill
  }
  stage(ST_LOG);
  mark(SEC_LOG);
  if (!safe_mode) {
    telemetryService();  // 1 s status record, events, USB log dump
    flashTestService();  // bench: flash erase/write stall measurement
  }
  if (crash_count && millis() > 60000) crash_count = 0;  // 60 s without a crash: not a crash loop
  usb_ota::service();  // a new OTA image: valid after 30 s, or after "ota confirm" (rollback at 120 s)
  stage(ST_BENCH);
  mark(SEC_BENCH);
  benchSerial();  // USB serial commands for the bench tests
  static uint32_t bench_print_ms = 0;
  if (millis() - bench_print_ms >= 1000) {
    bench_print_ms = millis();
    if (bench_timing_print) {  // the 1 s ESP-IDF log further down is commented out
      benchPrintCounters("TIMING", loop_exec_count, loop_min_us, loop_max_us);
      loop_min_us = 0xFFFFFFFF;
      loop_max_us = 0;
      loop_exec_count = 0;
    } else {
      // Heartbeat: keeps the USB link busy (an idle ESP32-S2 USB link can stop for good, see
      // bench tests) and shows that loop() runs and STM32 packets arrive.
      static uint32_t hb_last_packets = 0;
      if (safe_mode) usbOut.printf("SAFE MODE after %lu crash resets: no telemetry, no ESP-NOW\n", (unsigned long)crash_count);
      if (imu_halt) usbOut.println("IMU HALT: no drive (MPU6050 failed at start-up, imu_required=true)");
      usbOut.printf("HB ms=%lu packets=%lu mode=%d espnow=%d gyro=%d heap=%lu maxblk=%lu psram=%lu dtr=%d\n", (unsigned long)millis(),
        (unsigned long)((main_loop_counter - hb_last_packets + MAIN_LOOP_PERIOD) % MAIN_LOOP_PERIOD), (int)drive_mode_status, (int)espnow_started, (int)mpu_ok,
        (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)ESP.getPsramSize(),
        (int)tud_cdc_connected());
      if (usbOut.dropped || usb_guard::restarts() || usb_guard::full_resets())
        usbOut.printf("USB dropped=%lu guard_reconnects=%lu full_resets=%lu stalls=%lu\n", (unsigned long)usbOut.dropped,
                      (unsigned long)usb_guard::restarts(), (unsigned long)usb_guard::full_resets(),
                      (unsigned long)usb_guard::stalls());
      hb_last_packets = main_loop_counter;
      last_state.heap_min = ESP.getMinFreeHeap();
    }
  }

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
  mark(SEC_DEBUG);
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

  mark(SEC_MPU);
  monLedService();  // green LED: 100 ms inversion per MON frame
  // Process STM32 packets first, then do at most one sensor sample.
  serviceMpu();
} // End of main loop -------------------------------------------------------------------------

// STM32 UART ISR, triggered by falling edge of nDATA_READY after each full packet
void IRAM_ATTR receiveStmUart(){
  data_ready++;
  stm_edge_us = (uint32_t)esp_timer_get_time();
  rtprof::edgeIsr();
}

void flushSerial1RxBounded(){
  uint32_t start_us = micros();
  uint16_t flushed = 0;
  while(Serial1.available() && (flushed < SERIAL1_FLUSH_MAX_BYTES)){
    Serial1.read();
    flushed++;
    if((micros() - start_us) >= SERIAL1_FLUSH_MAX_US){
      break;
    }
  }
}

// CRC-16/IBM: poly 0x8005, init 0xFFFF, no input/output reflection, no final XOR.
static uint16_t crc16_ibm(const uint8_t *data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i = 0; i < len; i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b = 0; b < 8; b++){
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
  }
  return crc;
}

// write a PWM output channel's on-time in microseconds
void writePwmMicros(uint8_t ch, uint32_t micros){
  uint16_t duty_cycle = (int32_t)((float)micros * 1e-6 * (float)(PWM_OUT_FREQUENCY * ((1 << PWM_OUT_RESOLUTION_BITS) - 1)));
  ledcWrite(ch, duty_cycle);
}

// Increment the main loop counter
void mainLoopCounter(){
  main_loop_counter++;
  if(main_loop_counter == MAIN_LOOP_PERIOD){
    main_loop_counter = 0;
  }
}

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

  uint8_t gpio_state = !digitalRead(BUTTON_PIN);

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

void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd){
  stmLink.pushInt(cid, cmd);  // a full queue: dropped, counted, event 18
}

void espPacketFifo_addCmdFloat(uint32_t cid, float cmd){
  stmLink.pushFloat(cid, cmd);
}

void espPacketFifo_sendCmd(){
  const uint32_t now = millis();
  stm_last_packet_ms = now;
  stm_packet_seen = true;
  const bool v2 = stm_link_ver == 2;
  stmLink.setV2(v2);
  if (v2) {
    if (!stm_v2_cfg_sent) {  // v2 only: lanes and the values the decoder needs (a v1 STM32 knows none of them)
      stm_v2_cfg_sent = true;
      stmV2Config();
    }
    if (stmAns.present && (int32_t)(stm_ans_print_until - now) > 0)
      usbOut.printf("STM ans seq=%u cid=%u%s index=%u status=%u value=%ld (0x%08lx)\n", (unsigned)stmAns.seq, (unsigned)(stmAns.cid & 0x7F),
                    (stmAns.cid & 0x80) ? "w" : "r", (unsigned)stmAns.index, (unsigned)stmAns.status, (long)(int32_t)stmAns.value,
                    (unsigned long)stmAns.value);
    if (stm_rtt_wait && stmAns.present && stmAns.seq == stm_rtt_cmd_seq) {
      const uint8_t d = (uint8_t)(stmDec.lastSeq() - stm_rtt_frame_seq);
      stm_rtt_hist[d < 15 ? d : 15]++;
      stm_rtt_wait = false;
    }
    stmCalOnAnswer(stmAns);
    dspOnAnswer(stmAns);
    if (stmAns.present && (stmAns.cid & 0x7F) == 20 && stmAns.index < WFRX_BINS && (stmAns.status == RSP_OK || stmAns.status == RSP_OK_CLAMPED))
      memcpy(&stm_bin_hz[stmAns.index], &stmAns.value, 4);  // the bin frequency in effect, for STMCOIL8
    stmLink.onAnswerV2(now, stmDec.status_bits, stmAns.present, stmAns.seq, stmAns.cid, stmAns.index, stmAns.status, stmAns.value);
    // run after the configuration: at start (stmV2Config) and after each STM32 reset (bit 21 rises: the link
    // queued the settings again just now, so run goes after them). Not on status bit 24: app 2.0.6 does not
    // set it (bench test 2026-10-11), and each run command restarts the STM32 tracking state and PGA steps.
    static bool reset_prev = false;
    const bool reset_now = (stmDec.status_bits & WFRX_STATUS_STM_RESET) != 0;
    if (reset_now && !reset_prev && stm_v2_autorun && now - stm_v2_run_ms >= 1000) dspApply();  // DSP settings, then run
    reset_prev = reset_now;
    stmCalService(now);
  } else {
    stmLink.onPacket(now, stmPacket.items.status_bits, stmPacket.items.rid, (uint32_t)stmPacket.items.response.int32);
  }
  if (v2 && stm_dup_step) {  // bench: the repeat rule (the STM32 answers a repeat within 10 frames, runs it once)
    if (stm_dup_step == 1 || stm_dup_step == 4) Serial1.write(stm_dup_frame, ESP_FRAME_LEN);
    stm_dup_step = stm_dup_step == 4 ? 0 : stm_dup_step + 1;
    return;  // no link command in these frames: one frame per window
  }
  if (v2 && stm_v2_backlog) return;  // frames wait: this one is old; the command goes with the newest frame
  uint32_t cid, raw;
  if (!stmLink.next(now, &cid, &raw)) return;
  if (!v2) {
    EspTxPacket p;
    p.items.cid = cid;
    p.items.cmd.int32 = (int32_t)raw;
    Serial1.write(p.uart_data, ESP_TX_PACKET_SIZE);
    return;
  }
  uint32_t id = cid & 0x7FFFFF;
  const bool w = (cid & stmset::WRITE) != 0;
  if (id == STM_RUN_CMD) {  // the run command goes as CID 1 (layout 2 meaning)
    stmLink.rewrite(1 | stmset::WRITE, raw, 0);
    uint8_t f[ESP_FRAME_LEN];
    stmv2::encodeCmd(stmLink.seq(), 1 | ESP_CID_WRITE, 0, raw, f);
    Serial1.write(f, ESP_FRAME_LEN);
    return;
  }
  // Layout 2: the wire channel is a bin choice on the ESP32 (bins = Tx CH 1..5); the v1 detail formats
  // are gone; a save goes only after a centre calibration; centre calibration (4, 5) becomes a write of
  // the angle calibration (28) of that sensor: phi + atan2(a, b) of its present wire bin.
  if (id == 1 || id == 2) {
    if (w) {
      stmDec.setWireBin((int)id - 1, (raw & 0x80u) ? -1 : (int)(raw & 0x7Fu));
    }
    stmLink.doneLocal();
    return;
  }
  if (id == 11 || id == 12 || (id == 3 && !stm_v2_save_ok)) { stmLink.doneLocal(); return; }
  if (id == 3) stm_v2_save_ok = false;
  if ((id == 4 || id == 5) && w) {
    float phi;
    if (!stmDec.centreAngle((int)id - 4, &phi)) { stmLink.doneLocal(); return; }
    uint32_t pr;
    memcpy(&pr, &phi, 4);
    stmLink.rewrite(28 | stmset::WRITE, pr, (uint8_t)(id - 4));
    stm_v2_save_ok = true;
    cid = 28 | stmset::WRITE;
    raw = pr;
  }
  uint8_t f[ESP_FRAME_LEN];
  stmv2::encodeCmd(stmLink.seq(), (uint8_t)((cid & 0x7F) | (w ? ESP_CID_WRITE : 0)), stmLink.idx(), raw, f);
  Serial1.write(f, ESP_FRAME_LEN);
  if (!stm_rtt_wait || stm_rtt_cmd_seq != f[0]) { stm_rtt_wait = true; stm_rtt_cmd_seq = f[0]; stm_rtt_frame_seq = stmDec.lastSeq(); }
}

// Bench: print the captured frames while the USB queue has room (one frame = about 270 characters).
void stmCaptureService(){
  if (!stm_cap_dump || !stm_cap) return;
  static const char hx[] = "0123456789abcdef";
  while (stm_cap_out < stm_cap_n && usbOut.room() > 400) {
    char line[300];
    int k = snprintf(line, sizeof(line), "STMRAW %u ", (unsigned)stm_cap_out);
    const uint8_t* fr = stm_cap + (size_t)STM_FRAME_LEN * stm_cap_out;
    for (unsigned i = 0; i < STM_FRAME_LEN; i++) { line[k++] = hx[fr[i] >> 4]; line[k++] = hx[fr[i] & 15]; }
    line[k++] = '\n';
    usbOut.write((const uint8_t*)line, k);
    stm_cap_out++;
  }
  if (stm_cap_out >= stm_cap_n && usbOut.room() > 100) {
    usbOut.printf("STMRAW end %u\n", (unsigned)stm_cap_n);
    free(stm_cap);
    stm_cap = nullptr;
    stm_cap_dump = false;
  }
}

// v2 layout 2: the STM32 streams magnitudes only; the ESP32 derives positions, weak and lost itself.
// Set and get are for setup and checks only: here the lanes (tracked) and 4 setup reads, the radius and
// angle calibration of each sensor (27, 28). Bin gains stay 1 (the magnitudes are PGA-calibrated). The
// wire bin of each sensor follows rx_channel and enable_sensor_2, as v1 CID 1 and 2.
// DSP settings (config.json dsp_*) to the STM32 v2 app, at each STM32 start and after a change. The STM32
// takes 38 and 39 only while idle, so: run 0, sensors (38), slow ADC (39), period (18; 39 changes its step),
// bins (20), lanes (13), PGA (29), xy mask (25), run 1 (TrailTracer-Rx-STM32 rctf_wfrx_comms_v2.h). The
// answers fill g_dsp_eff (dspOnAnswer): the values in effect, shown as the read-only <key>_eff settings.
void dspApply(){
  if (stm_link_ver != 2) return;
  const DeviceConfig& c = g_deviceConfig;
  for (double& v : g_dsp_eff) v = NAN;
  stmLink.pushInt(STM_RUN_CMD | 0x80000000, 0);
  stmLink.pushInt(38 | 0x80000000, c.dsp_sensors);
  stmLink.pushInt(39 | 0x80000000, c.dsp_slow_adc);
  stmLink.pushInt(18 | 0x80000000, safe_mode ? 10000 : c.dsp_frame_period_us);  // safe mode: the default period
  for (int b = 0; b < 8; b++) stmLink.pushFloat(20 | 0x80000000, (float)c.dsp_bin_hz[b], (uint8_t)b);
  stm_v2_lanes = (uint8_t)c.dsp_lanes;
  stmLink.pushInt(13 | 0x80000000, c.dsp_lanes);
  stmLink.pushInt(29 | 0x80000000, c.dsp_pga);
  stmLink.pushInt(25 | 0x80000000, c.dsp_xy_mask);
  if (stm_v2_autorun) { stmLink.pushInt(STM_RUN_CMD | 0x80000000, 1); stm_v2_run_ms = millis(); }  // app 2.0.6: boots idle
}

// The STM32 frame period in effect, s: the v2 app's answer to CID 18 (dsp_frame_period_us_eff); 10 ms for a
// v1 app (100 packets a second) and before the first answer.
double stmFramePeriodS(){
  const double us = g_dsp_eff[DSP_EFF_PERIOD];
  return (stm_link_ver == 2 && isfinite(us) && us > 0) ? us * 1e-6 : 0.010;
}

// One v2 answer to a DSP write: the value in effect (the period in effect for 18, the bin frequency for 20).
void dspOnAnswer(const stmv2::Answer& a){
  if (!a.present || !(a.cid & 0x80) || (a.status != RSP_OK && a.status != RSP_OK_CLAMPED)) return;
  const int32_t v = (int32_t)a.value;
  switch (a.cid & 0x7F) {
    case 18: g_dsp_eff[DSP_EFF_PERIOD] = v; break;
    case 38: g_dsp_eff[DSP_EFF_SENSORS] = v; break;
    case 39: g_dsp_eff[DSP_EFF_SLOW] = v; break;
    case 13: g_dsp_eff[DSP_EFF_LANES] = v; break;
    case 29: g_dsp_eff[DSP_EFF_PGA] = v; break;
    case 25: g_dsp_eff[DSP_EFF_XY] = v; break;
    case 20: if (a.index < 8) { float f; memcpy(&f, &a.value, 4); g_dsp_eff[DSP_EFF_BIN0 + a.index] = f; } break;
  }
}

void stmV2Config(){
  stmDec.setWireBin(0, (int)rx_channel - 1);
  stmDec.setWireBin(1, enable_sensor_2 ? 0 : -1);
  for (int sn = 0; sn < 2; sn++) { stmLink.push(27, 0, (uint8_t)sn); stmLink.push(28, 0, (uint8_t)sn); }
  dspApply();  // lanes, the DSP settings, then run
  if (stm_cal_valid && stm_cal_phase == CAL_IDLE && stmCalPending()) {  // an IAP update ran: the calibration copy goes back
    usbOut.println("STM cal restore: started");
    stmCalStart(CAL_WRITE);
  }
}

int stmFrameLen(){ return stm_link_ver == 2 ? (int)STM_FRAME_LEN : (int)sizeof(stmPacket.uart_data); }

bool stmReadFrame(){
  rtprof::frameRead();
  int n = Serial1.available();
  if (stm_link_ver == 0) { delayMicroseconds(150); n = Serial1.available(); }  // the last bytes of the frame
  bool ok = false;
  if (stm_link_ver == 1 || (stm_link_ver == 0 && n == (int)sizeof(stmPacket.uart_data))) {
    Serial1.readBytes(stmPacket.uart_data, sizeof(stmPacket.uart_data));
    // CRC covers every byte except the trailing crc field itself.
    const uint16_t computed_crc = crc16_ibm(stmPacket.uart_data, sizeof(stmPacket.uart_data) - sizeof(stmPacket.items.crc));
    last_computed_crc = computed_crc;
    ok = computed_crc == stmPacket.items.crc;
    rtprof::frameDecoded();
    if (ok && stm_link_ver == 0) { stm_link_ver = 1; usbOut.println("STM link v1: 122 B packets"); }
  } else if (stm_link_ver == 2 || (stm_link_ver == 0 && n == (int)STM_FRAME_LEN)) {
    uint8_t f[STM_FRAME_LEN];
    const bool known = stm_link_ver == 2;
    const uint32_t c0 = ESP.getCycleCount();
    Serial1.readBytes(f, STM_FRAME_LEN);
    const uint32_t c1 = ESP.getCycleCount();
    ok = stmDec.decode(f, stmPacket.items, stmAns);
    const uint32_t c2 = ESP.getCycleCount();
    rtprof::frameDecoded();
    rt_read_cyc += c1 - c0; rt_dec_cyc += c2 - c1; rt_bench_n++;
    if (c1 - c0 > rt_read_max) rt_read_max = c1 - c0;
    if (c2 - c1 > rt_dec_max) rt_dec_max = c2 - c1;
    if (ok) { memcpy(rt_last_frame, f, STM_FRAME_LEN); rt_last_ok = true; }
    if (stm_stream_end) {
      if ((int32_t)(millis() - stm_stream_end) >= 0) {
        stm_stream_end = 0;
        usbOut.printf("STM stream end lines=%lu dropped=%lu\n", (unsigned long)stm_stream_n, (unsigned long)stm_stream_drop);
        stm_stream_drop = 0;
      } else if (ok) stmStreamFrame(f);
    }
    if (known) {  // layout 2: frames that wait stay in the buffer for the next passes
      stm_v2_backlog = Serial1.available() >= (int)STM_FRAME_LEN;
      if (stm_v2_backlog) stm_backlog_frames++;
      if (data_ready) data_ready--;
      stm_v2_resync = !ok;
      if (!ok) { serial1Drain(); data_ready = 0; rtprof::resync(); }
    }
    if (ok && stm_cap && stm_cap_n < stm_cap_want) memcpy(stm_cap + (size_t)STM_FRAME_LEN * stm_cap_n++, f, STM_FRAME_LEN);
    if (ok && stm_acc_want) stmAccFrame();
    if (ok && stm_coil_ms) stmCoilFrame();
    if (ok && scan_on) scanFrame();
    if (ok && stm_link_ver == 0) { stm_link_ver = 2; stm_v2_resync = false; usbOut.println("STM link v2.0 layout 2: 128 B frames"); }
    if (known) {
      if (ok) { stm_bad_frames_run = 0; return true; }
      crc_error_count++;
      if (++stm_bad_frames_run >= 50) {
        stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
        usbOut.println("STM link version unknown again: 50 bad frames in a row");
      }
      return false;
    }
  } else {
    serial1Drain();  // neither length: wait for the next frame
  }
  data_ready = 0;
  rtprof::resync();
  flushSerial1RxBounded();  // stray bytes: the next frame starts clean
  if (ok) { stm_bad_frames_run = 0; return true; }
  crc_error_count++;
  if (++stm_bad_frames_run >= 50 && stm_link_ver) {
    stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
    usbOut.println("STM link version unknown again: 50 bad frames in a row");
  }
  return false;
}

// A value as the STM32 gives it: float for 6 and 7, else an integer.
static void stmValueText(char* out, size_t n, uint32_t cid, uint32_t raw){
  const uint32_t id = cid & 0x7FFFFF;
  if (id == 6 || id == 7) { float f; memcpy(&f, &raw, 4); snprintf(out, n, "%.3f", f); }
  else snprintf(out, n, "%ld", (long)(int32_t)raw);
}

void stmLinkEvents(){
  stmset::Event e;
  while (stmLink.popEvent(&e)) {
    const stmset::Counters& c = stmLink.counters();
    char t[72], got[16], want[16];
    stmValueText(got, sizeof(got), e.cid, e.got);
    stmValueText(want, sizeof(want), e.cid, e.want);
    switch (e.kind) {
      case stmset::K_NO_REPLY:
        snprintf(t, sizeof(t), "no answer cmd %lu%s tries %u fails %lu", (unsigned long)(e.cid & 0x7FFFFF),
                 (e.cid & stmset::WRITE) ? "w" : "r", (unsigned)e.tries, (unsigned long)c.failures());
        break;
      case stmset::K_BAD_REPLY:
        snprintf(t, sizeof(t), "wrong answer cmd %luw got %s want %s try %u", (unsigned long)(e.cid & 0x7FFFFF), got, want, (unsigned)e.tries);
        break;
      case stmset::K_MISMATCH:
        snprintf(t, sizeof(t), "read-back cmd %lu got %s want %s: send all again", (unsigned long)(e.cid & 0x7FFFFF), got, want);
        break;
      case stmset::K_GAP:
        snprintf(t, sizeof(t), "packets back after %lu ms: send all again", (unsigned long)e.cid);
        break;
      case stmset::K_QUEUE_FULL:
        snprintf(t, sizeof(t), "queue full: cmd %lu dropped", (unsigned long)(e.cid & 0x7FFFFF));
        break;
      case stmset::K_STM_RESET:
        snprintf(t, sizeof(t), "STM32 reset (status bit 21): send all again");
        break;
      case stmset::K_REFUSED:
        snprintf(t, sizeof(t), "refused cmd %lu%s status %lu", (unsigned long)(e.cid & 0x7FFFFF),
                 (e.cid & stmset::WRITE) ? "w" : "r", (unsigned long)e.got);
        break;
      default:
        snprintf(t, sizeof(t), "kind %ld", (long)e.kind);
        break;
    }
    usbOut.printf("STMCMD %s\n", t);
    telemetryEvent(rec::EV_STM32_CMD, e.kind, (int32_t)e.cid, t);
  }
}

void chargeControl(){
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



void broadcastTelemetry(){
  if (   // Check if we should queue a telemetry message
      (millis() - telemetryBrodcastMillis >= 1000) // if at least 1 second since last broadcast
      | (stmPacket.items.charge_code != last_charge_code) //or charge code has changed
      | (stmPacket.items.status_bits != last_status_bits) //or status bits have changed
   ){
    String message = "- " +
      String(vehicleID) + " " +
      String(vehicleStatus) + " " +
      String(millis()) + " " +
      String(stmPacket.items.vbatt, 2) + " " +
      String(stmPacket.items.vcharge, 2) + " " +
      String(stmPacket.items.icharge, 2) + " " +
      String(charge_status) + " " +
      String(stmPacket.items.status_bits) + " " +
      String(stmPacket.items.charge_code) + " " +
      String(stmPacket.items.temperature) + " " +
      String(stmPacket.items.s1_radius) + " " +
      String(stmPacket.items.v5v, 2) + " " +
      String(steering_output) + " " +
      String(lapFlag)
      //+ " " + String(uart_overrun_count) // remove this after debugging stm <-> esp communication
      //+ " " + String(crc_error_count) // remove this after debugging stm <-> esp communication
      ;
    telemetryFifo_addMessage(message);  // Queue message instead of sending immediately
    telemetryBrodcastMillis = millis();
    lapFlag = false;  //reset lap flag after telemetry broadcast
  }
  last_charge_code = stmPacket.items.charge_code;
  last_status_bits = stmPacket.items.status_bits;
}

void telemetryFifo_addMessage(const String &message){ //Queue a telemetry message
  telemetryFifo[telemetry_fifo_head].message = message;
  telemetry_fifo_head++;
  if(telemetry_fifo_head == TELEMETRY_FIFO_LENGTH){
    telemetry_fifo_head = 0;
  }
}

void telemetryFifo_sendMessage(){ // Send one telemetry message from the queue (throttled to 100ms)
  // Only send if at least 100ms has passed since last send
  if(millis() - lastTelemetrySendTime >= 100){
    if(telemetry_fifo_head != telemetry_fifo_tail){
      // Queue has messages, send one
      broadcast(telemetryFifo[telemetry_fifo_tail].message);
      telemetry_fifo_tail++;
      if(telemetry_fifo_tail == TELEMETRY_FIFO_LENGTH){
        telemetry_fifo_tail = 0;
      }
      lastTelemetrySendTime = millis();
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
  static bool dsp_known = false;
  static DeviceConfig dsp_last;
  const bool dsp_changed = !dsp_known || dsp_last.dsp_frame_period_us != cfg.dsp_frame_period_us
      || dsp_last.dsp_sensors != cfg.dsp_sensors || memcmp(dsp_last.dsp_bin_hz, cfg.dsp_bin_hz, sizeof(cfg.dsp_bin_hz))
      || dsp_last.dsp_xy_mask != cfg.dsp_xy_mask || dsp_last.dsp_slow_adc != cfg.dsp_slow_adc
      || dsp_last.dsp_pga != cfg.dsp_pga || dsp_last.dsp_lanes != cfg.dsp_lanes;
  if (dsp_known && dsp_changed) dspApply();  // the first call (boot) leaves it to stmV2Config
  dsp_last = cfg;
  dsp_known = true;

  // Identification
  vehicleID           = cfg.vehicleID;
  rx_channel          = cfg.rx_channel;

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
    espPacketFifo_addCmdInt32(8 | 0x80000000, CHARGE_MAX_SECONDS * 2);  // the STM32 limit is 2 x the setting, as on shipped units
  }

  // RX channel
  if (stm_first || rx_channel != last_ch) {
    espPacketFifo_addCmdInt32(1 | 0x80000000, (int32_t)(rx_channel - 1));
    espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // save calibration, as before (the channel is not in the STM32 calibration block)
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
  rover_enabled               = cfg.rover_enabled && !safe_mode;  // safe mode (crash resets): no rover mode
  rover_seek_vbatt            = cfg.rover_seek_vbatt;
  rover_seek_after_s          = cfg.rover_seek_after_s;
  rover_entry_btx             = cfg.rover_entry_btx;
  rover_exit_btx              = cfg.rover_exit_btx;
  roverAp.cfg.goalCW          = cfg.rover_goal_cw;
  roverAp.cfg.sensorHeight    = (float)(cfg.rover_sensor_height_mm * 0.001);
  roverAp.cfg.maxSpeed        = (float)cfg.rover_full_speed_mps;
  // The autopilot steer command goes through the gyro rate loop: full steer requests steering_rate_max_dps
  // (odrive_setDriveOutput). The autopilot times its pivots from its own yaw rate, so its full yaw rate must
  // be the same value, or a planned pivot turns by steering_rate_max_dps / full yaw rate of the plan (90/115
  // with the defaults: 25 deg planned, 19.6 deg turned). rover_full_yaw_rate_dps is no longer used.
  roverAp.cfg.maxYawRate      = (float)(steering_rate_max_dps * M_PI / 180.0);
  roverAp.cfg.lostTimeout     = (float)cfg.rover_lost_timeout_s;
  roverAp.cfg.speedLimit      = (float)cfg.rover_speed_limit_mps;
  applyTxPower();  // live: the next frame goes out at the new power
  roverAp.cfg.exitGuard       = cfg.rover_exit_btx != 0;
}

// Rover autopilot step, once per valid STM32 packet after the wire-follow outputs are
// computed. In mode 2 with rover_enabled it replaces throttle and steering_output.
void roverStep(){
  const bool active = rover_enabled && drive_mode_status == 2;
  const uint32_t now_us = micros();
  float dt = rover_last_packet_us ? (now_us - rover_last_packet_us) * 1e-6f : 0.01f;
  rover_last_packet_us = now_us;
  if(dt > 0.1f) dt = 0.1f;  // a stalled loop must not become one long step
  if(active && !rover_active){  // entering mode 2: start roaming from a fresh state
    roverAp.reset(rover::ROAM);
    rover_out = rover::Output();
    rover_need_charge = false;
    rover_last_charge_ms = millis();
    rover_was_docked = charge_status > 0;
    rover_charge_ran = false;
    rover_stall_since_ms = 0;
    rover_stall_halted = false;
  }
  rover_active = active;
  if(!active) return;

  const uint32_t sb = stmPacket.items.status_bits;
  rover::Inputs in;
  in.dt = dt;
  in.packet = true;
  in.inRange = !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_LOST)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_OVERLOAD)
            && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_DISABLED);
  in.fwX = stmPacket.items.s1_x * 0.001f;  // mm -> m
  in.fwY = stmPacket.items.s1_y * 0.001f;
  in.fwR = stmPacket.items.s1_radius * 0.001f;
  in.v = rover_out.v;  // no odometry: the commanded speed after the slew
  // No gyro (design choice, 2026-10-08): the pivot angle comes from the autopilot's own slewed yaw-rate
  // command, as the speed does. Only the magnitude is used. Track slip in a pivot makes the real
  // turn smaller than this (lib/rover_autopilot/README.md, slip runs).
  in.yawRate = rover_out.w;

  // Charging: hold while the charge state machine runs; charged when it ends.
  const bool docked = charge_status > 0;
  in.docked = docked;
  if (charge_status == 3) rover_charge_ran = true;  // the STM32 charged; a refused or failed charge never gets here
  in.charged = rover_was_docked && !docked && rover_charge_ran;
  if (!docked) rover_charge_ran = false;
  rover_was_docked = docked;
  if(in.charged){
    rover_need_charge = false;
    rover_last_charge_ms = millis();
  }

  // Go home on battery voltage (filtered: it sags under load) or time since the last charge.
  const float vb = stmPacket.items.vbatt;
  rover_vbatt_filt = (rover_vbatt_filt <= 0.0f) ? vb : 0.99f * rover_vbatt_filt + 0.01f * vb;
  if(rover_seek_vbatt > 0 && rover_vbatt_filt > 1.0f && rover_vbatt_filt < rover_seek_vbatt) rover_need_charge = true;
  if(rover_seek_after_s > 0 && millis() - rover_last_charge_ms > (uint32_t)rover_seek_after_s * 1000U) rover_need_charge = true;
  in.needCharge = rover_need_charge;

  // Beacons, by mower channel number (BTx1Signal, BTx2Signal: filtered radius in mm).
  const float btx_signal[3] = {(float)BTxThreshold + 1.0f, BTx1Signal, BTx2Signal};
  in.entryBtx = rover_entry_btx != 0 && btx_signal[rover_entry_btx] < BTxThreshold;
  in.exitBtx = rover_exit_btx != 0 && btx_signal[rover_exit_btx] < BTxThreshold;

  // Stall stop (the stuck detector does not act in rover mode: off the wire, s1_y does not move).
  const uint32_t now_ms = millis();
  const bool wheels_fresh = odrive_vel_received[0] && odrive_vel_received[1]
      && now_ms - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS && now_ms - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS;
  const float drive_cmd = fabsf(rover_out.v) / roverAp.cfg.maxSpeed;
  bool stalled;
  if (wheels_fresh) {
    const float turning = 0.5f * (fabsf(odrive_vel_rps[0]) + fabsf(odrive_vel_rps[1]));
    stalled = drive_cmd > 0.15f && turning < 0.25f * drive_cmd * ODRIVE_DRIVE_SCALE_RPS;
  } else {
    stalled = in.inRange && stuckDetector.GetStatus();
  }
  if (!stalled) rover_stall_since_ms = 0;
  else if (!rover_stall_since_ms) rover_stall_since_ms = now_ms;
  else if (now_ms - rover_stall_since_ms > ROVER_STALL_MS && roverAp.state != rover::HALTED) {
    roverAp.halt();
    rover_stall_halted = true;
  }

  rover_out = roverAp.step(in);

  // Slewed command to the servo-equivalent outputs that odrive_setDriveOutput() normalises back.
  const float drive = rover_out.v / roverAp.cfg.maxSpeed;
  const float steer = rover_out.w / roverAp.cfg.maxYawRate;  // left (CCW) positive
  throttle = (int)lround(drive >= 0 ? PWM_OUT_2_DEFAULT + drive * (PWM_OUT_2_MAX - PWM_OUT_2_DEFAULT)
                                    : PWM_OUT_2_DEFAULT + drive * (PWM_OUT_2_DEFAULT - PWM_OUT_2_MIN));
  // Wire following with pid_direction DIRECT turns toward a wire on the left with a positive
  // ODrive steer command, so DIRECT maps left to positive; REVERSE flips it.
  const float left = (pid_direction == DIRECT) ? steer : -steer;
  steering_output = left >= 0 ? PWM_OUT_1_DEFAULT - left * (PWM_OUT_1_DEFAULT - PWM_OUT_1_MIN)
                              : PWM_OUT_1_DEFAULT - left * (PWM_OUT_1_MAX - PWM_OUT_1_DEFAULT);

  static uint32_t print_ms = 0;
  if(millis() - print_ms >= 1000){
    print_ms = millis();
    usbOut.printf("rover %s sub=%d in=%d X=%.2f d=%.2f psi=%.0f v=%.2f w=%.2f home=%d turns=%lu\n",
      rover::stateName(roverAp.state), (int)roverAp.sub, (int)in.inRange, roverAp.rd.Xf, roverAp.rd.df,
      roverAp.rd.psif * 57.2958f, rover_out.v, rover_out.w, (int)rover_need_charge, (unsigned long)roverAp.turnarounds);
  }
}

// Telemetry log ---------------------------------------------------------------------------------
// docs/espnow_protocol.md, "Telemetry log". lib/telemlog numbers and stores the records and does
// the radio work; lib/rover_telemetry packs them. Each STM32 packet gives one 100 Hz sample (7 a
// record, stream 0); each second gives one status record (stream 1), plus event records on
// changes. Nothing here waits: the log runs whether a collector listens or not.
#define FLASHTEST_SECTORS 16  // the last 16 sectors of tlog: flashtest only, never the telemetry ring
uint32_t telem_boot_id = 0;
rec::Block telem_block;
rec::Decimator* telem_decim = nullptr;  // from the heap in telemetryBegin()

// SLOW flash ring (telemlog): the "tlog" partition, 512 KB. Reads go through a memory map, because
// esp_partition_read() turns the cache off as an erase or a write does. Erases only while the
// rover stands still (telemetryEraseAllowed()); page programs (256 B, about every 3 s) at any time.
const esp_partition_t* tlog_part = nullptr;
const uint8_t* tlog_map = nullptr;
spi_flash_mmap_handle_t tlog_map_handle;
bool tlogErase(uint32_t sector){ return esp_partition_erase_range(tlog_part, sector * 4096u, 4096) == ESP_OK; }
bool tlogWrite(uint32_t off, const void* p, size_t n){ return esp_partition_write(tlog_part, off, p, n) == ESP_OK; }
bool tlogRead(uint32_t off, void* p, size_t n){
  if (off + n > tlog_part->size) return false;
  memcpy(p, tlog_map + off, n);
  return true;
}
telemlog_flash_t tlog_flash = {tlogErase, tlogWrite, tlogRead, 0};

// An erase stops the CPU for about 45 ms (the STM32 packets of that time are lost). Allowed only
// when nothing drives: docked, the wheels known to stand still, or no ODrive on the bus (bench).
bool telemetryEraseAllowed();
bool vehicleStationary(){ return telemetryEraseAllowed(); }
bool telemetryEraseAllowed(){
  if (charge_status > 0) return true;
  const uint32_t now = millis();
  bool any_odrive = false;
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++)
    if (odrive_heartbeat_received[n] && now - odrive_heartbeat_millis[n] < 1000) any_odrive = true;
  if (!any_odrive) return true;
  const bool fresh = odrive_vel_received[0] && odrive_vel_received[1]
      && now - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS && now - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS;
  return fresh && fabsf(odrive_vel_rps[0]) < 0.02f && fabsf(odrive_vel_rps[1]) < 0.02f
      && fabsf(odrive_drive_command) < 0.01f && fabsf(odrive_steer_command) < 0.01f;
}
uint32_t telem_packets = 0;           // packets since the last status record
uint32_t telem_status_ms = 0;
int telem_last_mode = -1, telem_last_ap = -2, telem_last_charge = 99;
bool telem_last_stuck = false, telem_last_halted = false;
uint32_t telem_last_axis_errors[ODRIVE_NODE_COUNT] = {};
// USB "log dump": records go out paced by the USB buffer, so that no line is cut.
int telem_dump_stream = -1;
uint32_t telem_dump_boot = 0, telem_dump_seq = 0, telem_dump_left = 0;

uint8_t telemetryApByte(){
  if (!rover_active) return rec::AP_OFF;
  return (uint8_t)(((uint8_t)roverAp.state << 4) | ((uint8_t)roverAp.sub & 0x0F));
}

bool telemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len){
  if (sim_loss_tx && esp_random() % 100 < sim_loss_tx) { sim_dropped_tx++; return true; }  // bench loss simulation
  espnowEnsurePeer(mac);  // a send needs its destination (broadcast, or the requester) in the peer list
  return esp_now_send(mac, buf, len) == ESP_OK;
}

void telemetryEvent(uint16_t code, int32_t a, int32_t b, const char* text){
  if (safe_mode) return;  // the log is not started in safe mode
  uint8_t body[rec::EVENT_MAX];
  const int n = rec::packEvent(body, code, a, b, text);
  telemlog_add(TELEMLOG_SLOW, millis(), body, (size_t)n);
}

void telemetryBegin(){
  Preferences p;
  p.begin("telem", false);
  telem_boot_id = p.getUInt("boot", 0) + 1;
  p.putUInt("boot", telem_boot_id);
  p.end();
  const esp_reset_reason_t rr = esp_reset_reason();
  const bool warm = rr != ESP_RST_POWERON && rr != ESP_RST_BROWNOUT && rr != ESP_RST_UNKNOWN;
  telemlog_init(telem_boot_id, warm);
  telem_decim = new rec::Decimator();  // about 400 B; heap, because static RAM is full
  telem_decim->setRate(g_deviceConfig.telemetry_rate_hz);
  tlog_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "tlog");
  if (tlog_part && esp_partition_mmap(tlog_part, 0, tlog_part->size, SPI_FLASH_MMAP_DATA,
                                      (const void**)&tlog_map, &tlog_map_handle) == ESP_OK) {
    tlog_flash.sectors = tlog_part->size / 4096u - FLASHTEST_SECTORS;  // the last 16 sectors belong to flashtest
    telemlog_flash_attach(&tlog_flash);  // the index scan runs later, one sector per service call
  } else {
    usbOut.println("tlog partition missing or not mapped: SLOW records in RAM only");
    tlog_part = nullptr;
  }
  // Settings hash: CRC-32 over the "key=value" lines, as the "list" command prints them.
  uint32_t crc = 0;
  String kv;
  for (int i = 0; msc_fat_key_at(&g_deviceConfig, i, &kv); i++) crc = rec::crc32((const uint8_t*)kv.c_str(), kv.length(), crc);
  char text[64];
  snprintf(text, sizeof(text), "fw=%s cfg=%08lx", ROVER_FW_VERSION, (unsigned long)crc);
  telemetryEvent(rec::EV_BOOT, (int32_t)rr, (int32_t)(telem_boot_id - 1), text);  // b: previous boot_id (NVS)
}

void telemetrySample(){
  if (safe_mode) return;
  const uint32_t sb = stmPacket.items.status_bits;
  rec::Sample s;
  s.ms = millis();
  s.x1_mm = stmPacket.items.s1_x;
  s.y1_mm = stmPacket.items.s1_y;
  s.r1_mm = stmPacket.items.s1_radius;
  s.flags = (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_LOST) ? rec::F_LOST : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_OVERLOAD) ? rec::F_OVERLOAD : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_DISABLED) ? rec::F_DISABLED : 0)
          | (WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S1_SIGNAL_WEAK) ? rec::F_WEAK : 0)
          | (charge_status > 0 ? rec::F_CHARGING : 0);
  const float btx[3] = {(float)BTxThreshold + 1.0f, BTx1Signal, BTx2Signal};
  if (rover_entry_btx && btx[rover_entry_btx] < BTxThreshold) s.flags |= rec::F_ENTRY_BTX;
  if (rover_exit_btx && btx[rover_exit_btx] < BTxThreshold) s.flags |= rec::F_EXIT_BTX;
  const uint32_t now = millis();
  if (odrive_vel_received[0] && odrive_vel_received[1] && now - odrive_vel_millis[0] < ODRIVE_VEL_STALE_MS
      && now - odrive_vel_millis[1] < ODRIVE_VEL_STALE_MS) s.flags |= rec::F_WHEELS_FRESH;
  s.ap = telemetryApByte();
  s.drive = odrive_drive_command;
  s.steer = odrive_steer_command;
  s.vel0_rps = odrive_vel_rps[0];
  s.vel1_rps = odrive_vel_rps[1];
#ifdef BENCH_FAKE_IMU
  // Bench fill: static values, different on each axis so that a swapped field shows. The status
  // record flags them (S_IMU_FAKE), so they never pass as measurements.
  s.imu = s.gyro = true;
  s.ax_g = 0.125f; s.ay_g = -0.250f; s.az_g = 1.000f;
  s.gx_dps = 1.50f; s.gy_dps = -2.50f; s.gyro_dps = 3.50f;
#else
  s.gyro = s.imu = mpu_ok;  // with an MPU6050: all six axes (0x22 block); without: 0x20, gyro z "none"
  if (mpu_ok) {
    s.ax_g = mpu.getAccX(); s.ay_g = mpu.getAccY(); s.az_g = mpu.getAccZ();
    s.gx_dps = mpu.getGyroX(); s.gy_dps = mpu.getGyroY(); s.gyro_dps = mpu.getGyroZ();
  }
#endif
  s.btx1_mm = stmPacket.items.s1_btx_radius[0];
  s.btx2_mm = stmPacket.items.s1_btx_radius[1];
  if (stm_link_ver == 2) {
    // Layout 2 (0.5.45+): the coil fields carry magnitudes, sensor 1 coil 1, sensor 1 coil 2, sensor 2 coil 1,
    // sensor 2 coil 2, at the wire bin of each sensor (bin 0 if the sensor has none), magnitude = |v| x 2^-24.
    // rover_log.py gives them as "coil_mag" when the status record shows firmware 0.5.45+ and status bit 24.
    for (int sn = 0; sn < 2; sn++) {
      const int lane = ((stmDec.lanes >> 1) & 1) == sn ? 1 : 0;  // lane 2 if it reads this sensor, else lane 1
      const int bn = stmDec.wireBin(sn) < 0 ? 0 : stmDec.wireBin(sn);
      const int32_t v1 = wfrx_int24_get(stmDec.coil[lane][bn].v1), v2 = wfrx_int24_get(stmDec.coil[lane][bn].v2);
      s.coil[2 * sn] = (float)(v1 < 0 ? -v1 : v1) * (1.0f / 16777216.0f);
      s.coil[2 * sn + 1] = (float)(v2 < 0 ? -v2 : v2) * (1.0f / 16777216.0f);
    }
  } else {
    for (int i = 0; i < 4; i++) s.coil[i] = stmPacket.items.coil_data[0][i];
  }
  s.xf_valid = rover_active && roverAp.rd.valid;
  s.xf_m = roverAp.rd.Xf;
  // Layout 3: sensor 2 beacons and position, the wheel positions with their ages, the gyro heading
  s.btx1_s2_mm = stmPacket.items.s2_btx_radius[0];
  s.btx2_s2_mm = stmPacket.items.s2_btx_radius[1];
  const bool s2_on = enable_sensor_2 && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_LOST)
      && !WFRX_STATUS_GET_BIT(sb, WFRX_STATUS_S2_SIGNAL_DISABLED);
  s.x2_mm = s2_on ? stmPacket.items.s2_x : NAN;
  s.y2_mm = s2_on ? stmPacket.items.s2_y : NAN;
  for (int k = 0; k < 2; k++) {
    s.enc_pos[k] = (int32_t)lroundf(odrive_pos_rev[k] * 10000.0f);
    const uint32_t age = now - odrive_vel_millis[k];
    s.enc_age_ms[k] = (!odrive_vel_received[k] || age > 254) ? 255 : (uint8_t)age;
  }
  s.heading = (uint16_t)lround(fmod(fmod(gyro_heading_deg, 360.0) + 360.0, 360.0) * (65536.0 / 360.0));
  telem_packets++;
  // Log rate: the anti-alias filter and decimation (lib/rover_telemetry/decimator.h). Its state is
  // on the heap (static RAM is full). A rate change clears the filter.
  if (!telem_decim) return;
  if (telem_decim->rate() != g_deviceConfig.telemetry_rate_hz) telem_decim->setRate(g_deviceConfig.telemetry_rate_hz);
  rec::Sample o;
  if (!telem_decim->push(s, &o)) return;
  const uint8_t rate = (uint8_t)telem_decim->rate();
  if (!telem_block.fits(o, rate)) {  // 65 s since the block began, or another block type or rate: store it as it is
    telemlog_add(TELEMLOG_FAST, telem_block.ms(), telem_block.body(), (size_t)telem_block.size());
    telem_block.clear();
  }
  if (telem_block.add(o, rate)) {
    telemlog_add(TELEMLOG_FAST, telem_block.ms(), telem_block.body(), (size_t)telem_block.size());
    telem_block.clear();
  }
}

void telemetryStatus(){
  telemlog_counters_t c;
  telemlog_counters(&c);
  rec::Status st;
  st.drive_mode = (uint8_t)drive_mode_status;
  st.charge_step = charge_status;
  st.vehicle_status = (uint8_t)vehicleStatus;
  st.vbatt = stmPacket.items.vbatt;
  st.vcharge = stmPacket.items.vcharge;
  st.icharge = stmPacket.items.icharge;
  st.v5v = stmPacket.items.v5v;
  st.temperature = stmPacket.items.temperature;
  st.ap_state = rover_active ? (uint8_t)roverAp.state : rec::AP_OFF;
  st.ap_sub = (uint8_t)roverAp.sub;
#ifdef BENCH_FAKE_IMU
  const uint8_t imu_flags = rec::S_GYRO | rec::S_IMU_FAKE;
#else
  const uint8_t imu_flags = mpu_ok ? rec::S_GYRO : 0;
#endif
  st.monitor_age_s = monitorAgeS();
  st.pc_unix_ms = pc_time_known ? (uint64_t)((int64_t)millis() + pc_offset_ms) : 0;
  {
    unsigned a = 0, b = 0, c = 0;
    sscanf(ROVER_FW_VERSION, "%u.%u.%u", &a, &b, &c);
    st.fw_ver[0] = (uint8_t)a; st.fw_ver[1] = (uint8_t)b; st.fw_ver[2] = (uint8_t)c;
  }
  st.flags = (rover_need_charge ? rec::S_GO_HOME : 0) | imu_flags | (can_init_ok ? rec::S_CAN : 0)
           | (vehicleStationary() ? 0 : rec::S_MOVING)
           | (espnow_started ? rec::S_ESPNOW : 0) | (rover_enabled ? rec::S_ROVER_ENABLED : 0)
           | (roverAp.isLocked() ? rec::S_LOCKED : 0);
  st.status_bits = stmPacket.items.status_bits;
  st.charge_code = stmPacket.items.charge_code;
  st.crc_errors = crc_error_count;
  st.overruns = uart_overrun_count;
  st.packets_last_s = telem_packets;
  st.loop_max_us = telem_loop_max_us;
  st.bounces = roverAp.bounces;
  st.turnarounds = roverAp.turnarounds;
  st.charges = roverAp.charges;
  st.odrive_vbus = odrive_node0_vbus;
  st.odrive_ibus = odrive_node0_ibus;
  const uint32_t now = millis();
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++) {
    st.axis_state[n] = odrive_axis_states[n];
    st.axis_errors_or |= odrive_axis_errors[n];
    if (odrive_axis_errors[n]) st.error_nodes |= (uint8_t)(1 << n);
    if (odrive_heartbeat_received[n] && now - odrive_heartbeat_millis[n] < 1000) st.heartbeat_nodes |= (uint8_t)(1 << n);
  }
  st.free_heap = ESP.getFreeHeap();
  st.send_fail = c.send_fail;
  st.backfill_last_s = c.backfill_sent_last_s;
  st.xf_valid = rover_active && roverAp.rd.valid;
  st.xf_m = roverAp.rd.Xf;
  st.dist_m = roverAp.rd.df;
  st.angle_deg = roverAp.rd.psif * 57.29578f;
  uint8_t body[rec::STATUS_BYTES];
  rec::packStatus(body, st);
  telemlog_add(TELEMLOG_SLOW, now, body, sizeof(body));
  telem_packets = 0;
  telem_loop_max_us = 0;
}

void telemetryEvents(){
  if (drive_mode_status != telem_last_mode) {
    if (telem_last_mode >= 0) telemetryEvent(rec::EV_MODE, telem_last_mode, drive_mode_status, nullptr);
    telem_last_mode = drive_mode_status;
  }
  const int ap = rover_active ? (int)roverAp.state : -1;
  if (ap != telem_last_ap) {
    if (telem_last_ap != -2) {
      char text[40];
      snprintf(text, sizeof(text), "%s->%s", telem_last_ap < 0 ? "off" : rover::stateName((rover::State)telem_last_ap),
               ap < 0 ? "off" : rover::stateName((rover::State)ap));
      telemetryEvent(rec::EV_AP_STATE, telem_last_ap, ap, text);
    }
    telem_last_ap = ap;
  }
  if (charge_status != telem_last_charge) {
    if (telem_last_charge != 99) telemetryEvent(rec::EV_CHARGE_STEP, telem_last_charge, charge_status, nullptr);
    telem_last_charge = charge_status;
  }
  const bool stuck = stuckDetector.GetStatus();
  if (stuck && !telem_last_stuck) telemetryEvent(rec::EV_STUCK, 0, 0, nullptr);
  telem_last_stuck = stuck;
  const bool halted = rover_active && roverAp.state == rover::HALTED;
  if (halted && !telem_last_halted) {
    if (rover_stall_halted) telemetryEvent(rec::EV_STALL, 0, 0, "stalled: halted");
    else telemetryEvent(rec::EV_HALTED, 0, 0, "wire lost too long");
  }
  telem_last_halted = halted;
  for (int n = 0; n < ODRIVE_NODE_COUNT; n++) {
    const uint32_t fresh = odrive_axis_errors[n] & ~telem_last_axis_errors[n];
    if (fresh) telemetryEvent(rec::EV_ODRIVE_ERROR, n, (int32_t)odrive_axis_errors[n], nullptr);
    telem_last_axis_errors[n] = odrive_axis_errors[n];
  }
}

void telemetryUsbCommand(const char* line){
  if (safe_mode) { usbOut.println("LOG off (safe mode)"); return; }
  unsigned long st = 0, from = 0, count = 0;
  telemlog_info_t in;
  telemlog_info(&in);
  if (!strcmp(line, "log info")) {
    usbOut.printf("LOG info boot=%lu prev=%lu fast=%ld..%ld slow=%ld..%ld fast_prev=%ld..%ld slow_prev=%ld..%ld\n",
      (unsigned long)in.boot_id, (unsigned long)in.prev_boot_id,
      (long)in.r[0][0].oldest, (long)in.r[0][0].newest, (long)in.r[1][0].oldest, (long)in.r[1][0].newest,
      (long)in.r[0][1].oldest, (long)in.r[0][1].newest, (long)in.r[1][1].oldest, (long)in.r[1][1].newest);
    telemlog_counters_t c;
    telemlog_counters(&c);
    usbOut.printf("LOG counters live_sent=%lu live_skipped=%lu backfill_sent=%lu send_fail=%lu req=%lu req_dropped=%lu add_refused=%lu prev_kept=%u\n",
      (unsigned long)c.live_sent, (unsigned long)c.live_skipped, (unsigned long)c.backfill_sent, (unsigned long)c.send_fail,
      (unsigned long)c.req_received, (unsigned long)c.req_dropped, (unsigned long)c.add_refused, (unsigned)c.prev_kept);
    if (tlog_part) {
      telemlog_flash_counters_t f;
      telemlog_flash_counters(&f);
      usbOut.printf("LOG flash pages_written=%lu sectors_erased=%lu entries_skipped=%lu write_fail=%lu erase_fail=%lu used_sectors=%lu erased_sectors=%lu erase_allowed=%d\n",
        (unsigned long)f.pages_written, (unsigned long)f.sectors_erased, (unsigned long)f.entries_skipped,
        (unsigned long)f.write_fail, (unsigned long)f.erase_fail, (unsigned long)f.used_sectors,
        (unsigned long)f.erased_sectors, (int)telemetryEraseAllowed());
    }
    return;
  }
  // "log dump <stream> <from_seq> <count>" (this boot) or "log dumpprev <stream> <from> <count>"
  const bool prev = !strncmp(line, "log dumpprev ", 13);
  if (sscanf(line + (prev ? 13 : 9), "%lu %lu %lu", &st, &from, &count) == 3 && st <= 1
      && (prev || !strncmp(line, "log dump ", 9))) {
    telem_dump_stream = (int)st;
    telem_dump_boot = prev ? in.prev_boot_id : in.boot_id;
    const telemlog_range_t& r = in.r[st][prev ? 1 : 0];
    telem_dump_seq = (r.oldest != TELEMLOG_NO_SEQ && from < r.oldest) ? r.oldest : from;
    telem_dump_left = (r.oldest == TELEMLOG_NO_SEQ || telem_dump_seq > r.newest) ? 0
                    : (uint32_t)min((unsigned long)(r.newest - telem_dump_seq + 1), count);
    usbOut.printf("LOG dump boot=%lu stream=%lu from=%lu count=%lu\n", (unsigned long)telem_dump_boot, st,
      (unsigned long)telem_dump_seq, (unsigned long)telem_dump_left);
    return;
  }
  usbOut.println("LOG usage: log info | log dump <stream> <from> <count> | log dumpprev <stream> <from> <count>");
}

void telemetryDumpStep(){
  if (telem_dump_stream < 0) return;
  // One line is at most 7 + 2 x 242 + 1 bytes; queue it only when the USB queue has room for it.
  while (telem_dump_left > 0 && usbOut.room() >= 500) {
    uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_FAST_BODY_MAX];
    uint32_t ms = 0;
    const size_t n = telemlog_get(telem_dump_stream, telem_dump_boot, telem_dump_seq, frame + TELEMLOG_LOG_HDR,
                                  TELEMLOG_FAST_BODY_MAX, &ms);
    if (n) {
      frame[0] = TELEMLOG_T_LOG;
      frame[1] = (uint8_t)telem_dump_stream;
      memcpy(frame + 2, &telem_dump_boot, 4);
      memcpy(frame + 6, &telem_dump_seq, 4);
      memcpy(frame + 10, &ms, 4);
      char hex[2 * sizeof(frame) + 1];
      for (size_t i = 0; i < TELEMLOG_LOG_HDR + n; i++) snprintf(hex + 2 * i, 3, "%02x", frame[i]);
      usbOut.printf("LOG %s\n", hex);
    }
    telem_dump_seq++;
    telem_dump_left--;
  }
  if (telem_dump_left == 0) {
    usbOut.println("LOG end");
    telem_dump_stream = -1;
  }
}

void telemetryService(){
  // A sector erase turns the flash cache off for tens of ms; the UART interrupt (not in IRAM in
  // arduino-esp32 2.0.17) cannot empty its 128-byte FIFO, and 3 or 4 STM32 packets are lost
  // (bench test 2026-10-09: 9 losses in 300 s, every 32 s = one sector of SLOW records). Erases run
  // only while the vehicle stands still; TELEMLOG_FLASH_ERASED_AHEAD sectors are kept ready for
  // driving. "erase off" (bench) blocks them, so that a test measures without the erase losses.
  // At most one erase each 2 s: 48 erases in a row after a boot kept loop() slow for a minute,
  // and a USB host lost the busy board (bench test).
  static uint32_t last_erase_ok = 0;
  const uint32_t now_ms = millis();
  // 0.5.54 (design choice, 2026-10-12): a flash erase stalls the CPU about 50 ms. It runs only while the motors are
  // disarmed (ODrive axes not in closed loop) or while charging, never on "stationary" alone: a rover that stops
  // mid-run stays armed, and the next frames must not meet an erase. The pre-erased sectors cover the run.
  const bool armed = odrive_rc_active || bench_sim_armed;
  const bool erase_ok = !bench_erase_block && (charge_status > 0 || !armed) && now_ms - last_erase_ok >= 2000;
  if (erase_ok) last_erase_ok = now_ms;
  if (tlog_part) telemlog_flash_service(now_ms, erase_ok);
  telemetryEvents();
  if (millis() - telem_status_ms >= 1000) {
    telem_status_ms = millis();
    telemetryStatus();
  }
  telemetryDumpStep();
}

// Bench: flash stall measurement ----------------------------------------------------------------
// "flashtest erase <n>", "flashtest write <n>" or "flashtest idle <n>": n operations on the "tlog"
// partition (never another one), one each 500 ms from loop(). Each operation's duration is
// measured. At the end: the STM32 packet, CRC error and UART overrun counts over the test, and
// the packets expected at 100 Hz. "idle" does no flash operation (the baseline).
// A flash erase or write turns the cache off; the UART driver's interrupt is not in IRAM, so
// packets can be lost during it. This measures how many.
int ft_mode = 0;  // 0 off, 1 erase, 2 write, 3 idle
uint32_t ft_left = 0, ft_done = 0, ft_next_ms = 0, ft_start_ms = 0, ft_offset = 0;
uint32_t ft_pk0 = 0, ft_crc0 = 0, ft_ovr0 = 0, ft_us_max = 0, ft_us_sum = 0;
const esp_partition_t* ft_part = nullptr;

void flashTestCommand(const char* arg){
  char op[8] = {0};
  unsigned long n = 0;
  if (sscanf(arg, "%7s %lu", op, &n) != 2 || n == 0 || n > FLASHTEST_SECTORS) { usbOut.println("FLASHTEST usage: flashtest erase|write|idle <1..16>"); return; }
  ft_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "tlog");
  if (!ft_part) { usbOut.println("FLASHTEST no tlog partition (flash the new partition table)"); return; }
  ft_mode = !strcmp(op, "erase") ? 1 : !strcmp(op, "write") ? 2 : !strcmp(op, "idle") ? 3 : 0;
  if (!ft_mode) { usbOut.println("FLASHTEST unknown op"); return; }
  ft_left = n; ft_done = 0; ft_us_max = 0; ft_us_sum = 0;
  ft_offset = ft_part->size - FLASHTEST_SECTORS * 4096u;  // the start of the test area: n sectors erased, or n pages written
  ft_start_ms = millis(); ft_next_ms = ft_start_ms + 500;
  ft_pk0 = main_loop_counter; ft_crc0 = crc_error_count; ft_ovr0 = uart_overrun_count;
  usbOut.printf("FLASHTEST start %s n=%lu partition=0x%lx size=0x%lx\n", op, n, (unsigned long)ft_part->address, (unsigned long)ft_part->size);
}

void flashTestService(){
  if (!ft_mode || (int32_t)(millis() - ft_next_ms) < 0) return;
  ft_next_ms += 500;
  if (ft_left > 0) {
    static uint8_t page[256];
    for (int i = 0; i < 256; i++) page[i] = (uint8_t)(i ^ ft_done);
    const uint32_t t0 = micros();
    esp_err_t err = ESP_OK;
    if (ft_mode == 1) err = esp_partition_erase_range(ft_part, ft_offset & ~0xFFFu, 4096);
    else if (ft_mode == 2) err = esp_partition_write(ft_part, ft_offset, page, sizeof(page));
    const uint32_t us = micros() - t0;
    ft_offset += (ft_mode == 1) ? 4096 : (ft_mode == 2 ? 256 : 0);
    if (err != ESP_OK) usbOut.printf("FLASHTEST op error %d\n", (int)err);
    if (us > ft_us_max) ft_us_max = us;
    ft_us_sum += us;
    ft_done++;
    ft_left--;
    return;
  }
  // One more 500 ms after the last operation, then the result.
  const uint32_t ms = millis() - ft_start_ms;
  const uint32_t packets = (main_loop_counter - ft_pk0 + MAIN_LOOP_PERIOD) % MAIN_LOOP_PERIOD;
  usbOut.printf("FLASHTEST done mode=%d ops=%lu op_us_max=%lu op_us_mean=%lu ms=%lu packets=%lu expected=%lu crc_err=%lu overrun=%lu\n",
    ft_mode, (unsigned long)ft_done, (unsigned long)ft_us_max, (unsigned long)(ft_done ? ft_us_sum / ft_done : 0),
    (unsigned long)ms, (unsigned long)packets, (unsigned long)(ms / 10), (unsigned long)(crc_error_count - ft_crc0),
    (unsigned long)(uart_overrun_count - ft_ovr0));
  ft_mode = 0;
}

// the main-branch serviceMpu() result (merge of f10cd31): USB text through the queue, and event 7 in the log.
void mpuReport(bool ok, unsigned status){
  if (ok) {
    usbOut.println("MPU6050 calibration complete");
    telemetryEvent(rec::EV_IMU, 2, 250, "serviceMpu: library ranges, gyro and acc offsets");
  } else {
    usbOut.printf("MPU6050 init failed: %u; sensor disabled, motors stay locked%c", status, 10);
    telemetryEvent(rec::EV_IMU, 0, 0, "absent: no gyro, motors stay locked");
  }
}
