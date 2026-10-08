#include <Arduino.h>
#include <math.h>
#include <PID_v1.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <driver/twai.h>
#include "wfrx_stm32_comms.h"
#include "rctf_wfrx_status_bits.h"
#include "pwm_read_rmt.h"

MPU6050 mpu(Wire);
unsigned long timer = 0;
#define SDA_PIN 39
#define SCL_PIN 40

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
// Temporary hardware test: set to false to disable GPIO26 steering output again.
constexpr bool PWM_OUT_1_AVAILABLE = true;
#define PWM_OUT_2 GPIO_NUM_34
#define PWM_OUT_3 GPIO_NUM_16
#define PWM_OUT_4 GPIO_NUM_13

// Spare Inputs
#define SPARE_IN_1 GPIO_NUM_9
#define SPARE_IN_2 GPIO_NUM_10

// Spare Outputs
#define SPARE_OUT_1 GPIO_NUM_11
#define SPARE_OUT_2 GPIO_NUM_12

// Spare GPIO
#define SPARE_GPIO_IO39 GPIO_NUM_39
#define SPARE_GPIO_IO40 GPIO_NUM_40

// Spare I2C/SPI, but can be as GPIO too
#define SPARE_NSS       GPIO_NUM_38
#define SPARE_SCL_SCK   GPIO_NUM_36
#define SPARE_SDA_DOUT  GPIO_NUM_37
#define SPARE_DIN       GPIO_NUM_35

// Native ESP32-S2 TWAI controller connected to a 3.3V SN65HVD230 transceiver.
// TEMPORARY: set to 0 to restore the normal application and TWAI operation.
#define ENABLE_CAN_TX_TOGGLE_TEST 0
// Temporary RC-only troubleshooting; set to 0 to restore the application.
#define ENABLE_RC_INPUT_TEST 0
#define TWAI_TX_PIN GPIO_NUM_36
#define TWAI_RX_PIN GPIO_NUM_38
#define CAN_BUS_BITRATE 1000000  // Must match every connected ODrive CAN baud rate.
#define ODRIVE_NODE_COUNT 6  // Node IDs 0-5

// ODrive CAN protocol command IDs (see ODrive CAN protocol docs)
enum OdriveCanCmd : uint8_t {
  ODRIVE_CMD_HEARTBEAT              = 0x001,
  ODRIVE_CMD_ESTOP                  = 0x002,
  ODRIVE_CMD_GET_MOTOR_ERROR        = 0x003,
  ODRIVE_CMD_GET_ENCODER_ERROR      = 0x004,
  ODRIVE_CMD_SET_AXIS_STATE         = 0x007,
  ODRIVE_CMD_GET_ENCODER_ESTIMATES  = 0x009,
  ODRIVE_CMD_SET_CONTROLLER_MODE    = 0x00B,
  ODRIVE_CMD_SET_INPUT_POS          = 0x00C,
  ODRIVE_CMD_SET_INPUT_VEL          = 0x00D,
  ODRIVE_CMD_SET_INPUT_TORQUE       = 0x00E,
  ODRIVE_CMD_SET_LIMITS             = 0x00F,
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

// Valid RC inputs automatically enable control. PWM_IN_1 = steering, PWM_IN_2 = throttle.
bool odrive_rc_active = false;
#define ODRIVE_RC_UPDATE_MS 12  // Example control period.
#define ODRIVE_RC_SIGNAL_TIMEOUT_MS 250
uint32_t odrive_rc_update_millis = 0;
float odrive_drive_filtered = 0.0f;
float odrive_steer_filtered = 0.0f;
uint32_t odrive_axis_errors[ODRIVE_NODE_COUNT] = {};

uint32_t odrive_encoder_poll_millis = 0;  // last time encoder estimates were requested
#define ODRIVE_ENCODER_POLL_MS 500  // encoder estimate poll interval

// Main loop configuration
#define MAIN_LOOP_PERIOD 360000  // 1hr rollover

// Button configuration
#define BUTTON_DEBOUNCE_PERIODS 3  // 0.03s
#define BUTTON_CLICK_MAX_LENGTH 40  // 0.4s
#define BUTTON_LONG_PRESS_PERIODS 200  // 2s

// PWM OUT configuration
#define PWM_OUT_FREQUENCY 50  // Frequency (Hz)
#define PWM_OUT_RESOLUTION_BITS 14  // 14-bits is maximum

// PWM OUT 1 (Steering)
#define PWM_OUT_1_DEFAULT 1500
#define PWM_OUT_1_MIN     950
#define PWM_OUT_1_MAX     2050

// PWM OUT 2 (Throttle)
#define PWM_OUT_2_DEFAULT 1499
#define PWM_OUT_2_MIN     1499
#define PWM_OUT_2_MAX     2000

// PWM OUT 3 
#define PWM_OUT_3_DEFAULT 1500
#define PWM_OUT_3_MIN     1000
#define PWM_OUT_3_MAX     2000

// PWM OUT 3
#define PWM_OUT_4_DEFAULT 0
#define PWM_OUT_4_MIN     0
#define PWM_OUT_4_MAX     0

// Charging Control
#define CHARGE_AUTO_ENABLE true  // bool to enable/disable automatic charging
#define CHARGE_START_DELTA 1.0
#define CHARGE_VBATT_STOP 12  // Voltage to stop charging at.  Should be ≤ 4.0V per LiPo cell
#define CHARGE_MAX_CURRENT 6  // Overcurrent protection threshold.  Set about 0.5A higher than PSU limit
#define CHARGE_MAX_SECONDS (2 * 60)  // Maximum charge time in seconds. 1hr maximum.  <- 4 minutes
int8_t charge_status = 0;  // charge status flag for state machine
uint32_t charge_start_millis = 0;

// Extra GPIO control
#define GPIO_39_RESTART_CYCLES 1000  // number of 100Hz cycles to wait to restart (1000 = 10s)
int8_t gpio_39_status;
uint32_t gpio_39_timer;

StmTxPacket stmPacket;  // Data container for UART buffer and individual values
float last_mag = 0;

uint16_t throttle_micros = 1500;
uint32_t cruise_control_timer = 0;
int8_t cruise_control_status = 0;

int throttle = 1500;

// PID TUNABLE VALUES --------------------------------------------------------------------------------------


int steering_servo_angle = 90;
int steering_max_angle = 170; //adjust these to trim center
int steering_min_angle = 10; //adjust these to trim center
double target_x_pos = 0;  // target x distance from wire
double live_position = 0;
double kp = 2.5;  // PID Proportional term
double ki = 0.0;  // PID integral term
double kd = 0.025;  // PID derivative term
int pid_direction = DIRECT;  // PID direction.  DIRECT or REVERSE.  Change if car is steering away from wire

// PID Initialization
double pid_output = 90;
PID steeringPID(&live_position, &pid_output, &target_x_pos, kp, ki, kd, pid_direction);


// PID Low pass filterint setup -----------------------------------------------------------------------------------
bool low_pass_enable = true;  // true = low pass active, false disables low-pass
float low_pass_cutoff = 5;  // Frequency of a 2nd order low-pass filter.  Smaller number = more filtering
// ---------------------------------------------------------------------------------------------------------
float low_pass_alpha = 0.0;  // filter coefficient that's calculated from the cutoff value
float low_pass_beta = 0.0;  // filter coefficient that's calculated from the cutoff value
bool low_pass_compute = false;  // bool to only compute when PID loop updates
bool low_pass_first_cycle = true;
float low_pass_last_val_1 = 0.0;
float low_pass_last_val_2 = 0.0;


// DEAD ZONE TUNABLE VALUES --------------------------------------------------------------------------------------
// The "dead zone" essentially makes the wire look more like a wide tape instead of a thin wire.
// This should allow the car to ignore wiggles in the wire, at least while the car is over the dead zone

bool dead_zone_enable = false;  // true = dead-zone active, false disables dead zone
float dead_zone_width = 0; // Dead zone width in mm.  0 to 250mm range.  Width is total, so 50mm would be 25mm left and 25mm right of wire center
float dead_zone_gain = 1; // Value from 0.0 to 1.0.  A value of zero means center of zone is completely flat or "dead"
// deadGain value of 0.5 means slope at center is 1/2 the normal slope, and a value of 1 removes the dead zone

/*
Circular FIFO and control variables for handling commands sent to the STM32
Only one command can be sent per data update (100Hz)
Use the espPacketFifo_AddCmdxxx functions to add a command to the FIFO depending on payload datatype
The espPacketFifo_sendCmd function sends one command from the queue
There is no protection from overloading the FIFO, but commands are sparse so unlikely to be an issue
*/ 
#define ESP_PACKET_FIFO_LENGTH 50 // Number of commands that can be buffered
EspTxPacket espPacketFifo[ESP_PACKET_FIFO_LENGTH];  // buffer
uint32_t esp_packet_fifo_head = 0;  // index for adding data to buffer
uint32_t esp_packet_fifo_tail = 0;  // index for reading data out of buffer

// For cid, don't forget to do (cid | 0x80000000) for write commands
void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd);  // Add a command with a int32 payload to the queue
void espPacketFifo_addCmdFloat(uint32_t cid, float cmd);  //  add a command with a float payload to the queue
void espPacketFifo_sendCmd();  // Send up to one command from the queue, call after each new STM32 packet is received

volatile uint32_t data_ready = 0;  // flag for new data from STM32
uint32_t main_loop_counter;  // counter incremented on each new STM32 data packet
int32_t bttn_counter = 0;  // button counter for debounce logic
int32_t bttn_state = 0;  // debounced state of the user button
int32_t bttn_click_counter = 0;  // button counter for "click" logic
int32_t bttn_long_counter = 0;  // button counter for "long" press logic
int32_t bttn_long_state = 0;  // state of a long press

uint8_t sensor2_enabled = 0;  // Set to 1 is sensor 2 is used
uint8_t rx_channel = 0;  // starting Rx channel.  Changed and saved with user button click

void receiveStmUart();  // ISR routine to set data ready flag
void writePwmMicros(uint8_t ch, uint32_t micros); // Write PWM output pulse width in microseconds
void writePwmAngle(uint8_t ch, int32_t degrees); // Write PWM output pulse width according to servo rotation in degrees
void mainLoopCounter();  // increment main loop counter
void buttonLogic();  // handle button events

void buttonClickEvent();  // Called on button release if the press time is considered a "click"
void buttonLongPressEvent();  //  Called on a button long-press

void chargeControl();
uint16_t cruiseControl2ch();
uint16_t cruiseControl();
int8_t gpio39Status();

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
  /* Serial.println("Boot: TWAI driver install (level-2 interrupt)"); */;
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  /* Serial.printf("Boot: TWAI install returned %s\n", esp_err_to_name(err)); */;
  if (err != ESP_OK) {
    /* Serial.printf("TWAI install FAILED: %s\n", esp_err_to_name(err)); */;
    return false;
  }
  can_driver_installed = true;
  /* Serial.println("Boot: TWAI driver start"); */;
  err = twai_start();
  if (err != ESP_OK) {
    /* Serial.printf("TWAI start FAILED: %s\n", esp_err_to_name(err)); */;
    twai_driver_uninstall();
    can_driver_installed = false;
    return false;
  }
  /* Serial.printf("TWAI init OK: TX=%d RX=%d bitrate=%u normal mode\n",
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
      /* Serial.printf("TWAI TX queue failed: id=0x%03lX %s pending=%lu RXpin=%d bus_errors=%lu arb_lost=%lu\n",
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

// Apply the example's intended 10% deadband and 50% quadratic expo.
float odrive_conditionRc(int32_t pulse_us) {
  float cmd = constrain((pulse_us - 1500) / 500.0f, -1.0f, 1.0f);
  float magnitude = fabsf(cmd);
  magnitude = magnitude < 0.1f ? 0.0f : (magnitude - 0.1f) / 0.9f;
  return copysignf(0.5f * magnitude + 0.5f * magnitude * magnitude, cmd);
}

void odrive_stopRc() {
  odrive_rc_active = false;
  odrive_drive_filtered = odrive_steer_filtered = 0.0f;
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    odrive_setInputVel(node, 0.0f);
    odrive_setAxisState(node, ODRIVE_AXIS_STATE_IDLE);
  }
  /* Serial.println("ODrive RC: STOP REQUESTED (RC signal or safety fault)"); */;
}

void odrive_startRc() {
  if (!can_init_ok || !odrive_rcInputsValid() || odrive_rcFault()) return;
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    odrive_clearErrors(node);
    delay(2);
    odrive_setControllerMode(node, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    delay(2);
    // Preserve the example's per-node current limits.
    odrive_setLimits(node, ODRIVE_VEL_LIMIT_RPS, (node == 2 || node == 3) ? 6.0f : 3.0f);
    delay(2);
    odrive_setInputVel(node, 0.0f);
    odrive_setAxisState(node, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    delay(5);
  }
  odrive_drive_filtered = odrive_steer_filtered = 0.0f;
  odrive_rc_update_millis = millis();
  odrive_rc_active = true;
  /* Serial.println("ODrive RC: ARM REQUESTED (nodes 0-5); PWM1 steer / PWM2 throttle"); */;
}

void odrive_rcUpdate() {
  const uint32_t now = millis();
  if (now - odrive_rc_update_millis < ODRIVE_RC_UPDATE_MS) return;
  const float dt = (now - odrive_rc_update_millis) * 0.001f;
  odrive_rc_update_millis = now;
  const bool rc_ready = can_init_ok && odrive_rcInputsValid() && !odrive_rcFault();
  if (odrive_rc_active && !rc_ready) {
    odrive_stopRc();
  } else if (!odrive_rc_active && rc_ready) {
    odrive_startRc();
  }
  float even_rps = 0.0f, odd_rps = 0.0f;
  if (odrive_rc_active) {
    float drive = odrive_conditionRc(pwm_read_rmt_dur(1));
    float steer = -odrive_conditionRc(pwm_read_rmt_dur(0));
    const float alpha = dt / (dt + 0.1f);
    odrive_drive_filtered += alpha * (drive - odrive_drive_filtered);
    odrive_steer_filtered += alpha * (steer - odrive_steer_filtered);
    even_rps = -1.5f * (odrive_drive_filtered + odrive_steer_filtered);
    odd_rps = 1.5f * (odrive_drive_filtered - odrive_steer_filtered);
  }
  if (!can_init_ok) return;
  // Repeat zero and idle while disarmed, so single-shot arbitration losses
  // cannot leave a previously commanded motor running after signal loss.
  for (uint8_t node = 0; node < ODRIVE_NODE_COUNT; ++node) {
    odrive_setInputVel(node, (node & 1) ? odd_rps : even_rps);
    if (!odrive_rc_active) odrive_setAxisState(node, ODRIVE_AXIS_STATE_IDLE);
  }
}

// Recovery runs asynchronously: initiate once, poll state, restart after recovery.
void odrive_recoverIfNeeded() {
  if (!can_driver_installed) return;
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);
  static uint32_t last_report = 0;
  if (alerts && millis() - last_report >= 1000) {
    last_report = millis();
    /* Serial.printf("TWAI alerts: 0x%08lX\n", (unsigned long)alerts); */;
  }
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;
  can_init_ok = status.state == TWAI_STATE_RUNNING;
  if (status.state == TWAI_STATE_BUS_OFF) {
    esp_err_t err = twai_initiate_recovery();  // Clears pending TX; never waits for recovery.
    if (err == ESP_OK) /* Serial.println("TWAI bus-off: recovery started"); */;
  } else if (status.state == TWAI_STATE_STOPPED) {
    esp_err_t err = twai_start();  // Recovery completes in the stopped state.
    can_init_ok = err == ESP_OK;
    if (can_init_ok) /* Serial.println("TWAI recovered: normal mode restarted"); */;
  }
}

// Requests encoder estimates from one ODrive node (cycles through nodes each call).
// The ODrive responds with a Get_Encoder_Estimates message we print in odrive_pollAndPrintCan.
void odrive_requestEncoderEstimates() {
  if (!can_init_ok) return;
  if (millis() - odrive_encoder_poll_millis < ODRIVE_ENCODER_POLL_MS) return;

  static uint8_t poll_node = 0;
  twai_message_t frame = {};
  frame.identifier = (poll_node << 5) | ODRIVE_CMD_GET_ENCODER_ESTIMATES;
  frame.data_length_code = 8;  // RTR requests the eight-byte response; no data is transmitted.
  memset(frame.data, 0, sizeof(frame.data));
  bool queued = sendODriveCAN(poll_node, ODRIVE_CMD_GET_ENCODER_ESTIMATES, nullptr,
                              frame.data_length_code, true);
  if (poll_node == 5) {
    /* Serial.printf("ODrive node 5 encoder request: id=0x0A9 RTR=1 DLC=8 %s\n",
                  queued ? "queued (delivery not confirmed)" : "queue failed"); */;
  }

  poll_node++;
  if (poll_node >= ODRIVE_NODE_COUNT) poll_node = 0;
  odrive_encoder_poll_millis = millis();
}

// Drains pending CAN RX buffers and prints ODrive messages for diagnostics.
// Prints heartbeats (state/error), encoder estimates (pos/vel), and vbus voltage.
void odrive_pollAndPrintCan() {
  if (!can_init_ok) return;
  twai_message_t frame = {};
  static uint32_t last_print[ODRIVE_NODE_COUNT][3] = {};
  const uint32_t started_us = micros();
  // Drain bursts with bounded work; repeated diagnostics must not flood USB Serial.
  for (uint8_t received = 0; received < 64 &&
       micros() - started_us < 2000 && receiveODriveCAN(frame); received++) {
    if (frame.extd || frame.rtr || frame.data_length_code > 8) continue;
    uint8_t node_id = frame.identifier >> 5;
    uint8_t cmd_id = frame.identifier & 0x1F;
    if (node_id >= ODRIVE_NODE_COUNT) continue;
    if (cmd_id == ODRIVE_CMD_HEARTBEAT && frame.data_length_code >= 7) {
      memcpy(&odrive_axis_errors[node_id], frame.data, sizeof(uint32_t));
    }
    uint8_t print_slot = cmd_id == ODRIVE_CMD_HEARTBEAT ? 0 :
                         cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES ? 1 : 2;
    if (millis() - last_print[node_id][print_slot] < 500) continue;
    if (cmd_id == ODRIVE_CMD_HEARTBEAT && frame.data_length_code >= 7) {
      last_print[node_id][print_slot] = millis();
      uint32_t axis_error;
      memcpy(&axis_error, &frame.data[0], sizeof(axis_error));
      uint8_t axis_state = frame.data[4];
      uint8_t procedure_result = frame.data[5];
      /* Serial.printf("ODrive node %u heartbeat: state=%u error=0x%08lX proc_result=%u\n",
                    node_id, axis_state, (unsigned long)axis_error, procedure_result); */;
    }
    else if (cmd_id == ODRIVE_CMD_GET_ENCODER_ESTIMATES) {
      last_print[node_id][print_slot] = millis();
      if (frame.data_length_code != 8) {
        /* Serial.printf("ODrive node %u encoder frame: unexpected DLC=%u\n",
                      node_id, frame.data_length_code); */;
        continue;
      }
      float pos, vel;
      memcpy(&pos, &frame.data[0], 4);
      memcpy(&vel, &frame.data[4], 4);
      /* Serial.printf("ODrive node %u encoder: pos=%.3f rev  vel=%.3f rps\n",
                    node_id, (double)pos, (double)vel); */;
    }
    else if (cmd_id == 0x017 && frame.data_length_code >= 4) {  // Get_Bus_Voltage_Current
      last_print[node_id][print_slot] = millis();
      float vbus;
      memcpy(&vbus, &frame.data[0], 4);
      /* Serial.printf("ODrive node %u vbus=%.2f V\n", node_id, (double)vbus); */;
    }
  }
}

// Void Setup --------------------------------------------------------------------------------------------------------------
void setup() {
#if ENABLE_RC_INPUT_TEST
  Serial.begin(1000000);
  Serial.setTxTimeoutMs(0);
  uint8_t rc_pins[] = {PWM_IN_1, PWM_IN_2};
  pwm_read_rmt_init(rc_pins, 2);
  return;
#endif
#if ENABLE_CAN_TX_TOGGLE_TEST
  Serial.begin(1000000);
  pinMode(TWAI_TX_PIN, OUTPUT);
  digitalWrite(TWAI_TX_PIN, HIGH);
  /* Serial.println("GPIO36 test: HIGH (3.3 V); toggles every 3 seconds"); */;
  return;  // Run only the pin test, without installing TWAI or other peripherals.
#endif


  // Visual heartbeat, independent of Serial, to confirm the board is actually booting/running
  pinMode(nLED, OUTPUT);
  for (uint8_t i = 0; i < 6; i++) {
    digitalWrite(nLED, !digitalRead(nLED));
    delay(150);
  }

  // Start Serial
  Serial.begin(1000000);  // Start USB VCP
  Serial.setTxTimeoutMs(0);  // Diagnostics must not stall motor updates on USB backpressure.
  Serial0.begin(1000000);  // Start debug UART (visible on tag-connect)
  Serial1.begin(2000000);  // Start ESP32 <-> STM32 UART
  delay(1000);  // Give the native USB CDC time to enumerate so early prints aren't lost
  /* Serial.println("setup"); */;
  
  // Initialize MPU6050 on GPIO39 (SDA) and GPIO40 (SCL).
  Wire.begin(SDA_PIN, SCL_PIN);
  byte status = mpu.begin();
  while (status != 0) {
    delay(500);  // Halt silently if the MPU6050 cannot be reached.
  }
  delay(500);  // Keep the sensor stationary during calibration.
  mpu.calcOffsets(true, true);  // Calibrate gyroscope and accelerometer.
  delay(100);

  /* Serial.println("Boot: configuring GPIO"); */;
  // Configure pins
  pinMode(nLED, OUTPUT);
  pinMode(STM_nDATA_READY, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(PWM_IN_1, INPUT);
  pinMode(PWM_IN_2, INPUT);
  pinMode(PWM_IN_3, INPUT);
  pinMode(PWM_IN_4, INPUT);

  if (PWM_OUT_1_AVAILABLE) {
    pinMode(PWM_OUT_1, OUTPUT);
  } else {
    /* Serial.println("Boot: PWM_OUT_1 disabled by configuration"); */;
  }
  pinMode(PWM_OUT_2, OUTPUT);
  pinMode(PWM_OUT_3, OUTPUT);
  pinMode(PWM_OUT_4, OUTPUT);

  // GPIO39 is reserved for MPU6050 SDA; do not reconfigure it as an input.

  digitalWrite(nLED, 1);  // turn the LED off

  /* Serial.println("Boot: GPIO configured; initializing PWM outputs"); */;
  // Servo PWM setup
  ledcSetup(1, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(2, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(3, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  ledcSetup(4, PWM_OUT_FREQUENCY, PWM_OUT_RESOLUTION_BITS);
  if (PWM_OUT_1_AVAILABLE) ledcAttachPin(PWM_OUT_1, 1);
  ledcAttachPin(PWM_OUT_2, 2);
  ledcAttachPin(PWM_OUT_3, 3);
  ledcAttachPin(PWM_OUT_4, 4);
  writePwmMicros(1, PWM_OUT_1_DEFAULT);
  writePwmMicros(2, PWM_OUT_2_DEFAULT);
  writePwmMicros(3, PWM_OUT_3_DEFAULT);
  writePwmMicros(4, PWM_OUT_4_DEFAULT);

  /* Serial.println("Boot: PWM outputs initialized; queuing charge settings"); */;
  // Set up charge port
  espPacketFifo_addCmdFloat(6 | 0x80000000, CHARGE_VBATT_STOP);  // Set voltage stop
  espPacketFifo_addCmdFloat(7 | 0x80000000, CHARGE_MAX_CURRENT);  // Set overcurrent
  espPacketFifo_addCmdInt32(8 | 0x80000000, CHARGE_MAX_SECONDS);  // Set overcurrent

  // Install TWAI before allocating RMT and STM32 GPIO interrupts.
  /* Serial.println("Boot: initializing TWAI"); */;
  can_init_ok = odriveCan_init();

  // Set up PWM inputs (in progress)
  uint8_t pwm_input_pins[] = {PWM_IN_1, PWM_IN_2, PWM_IN_3, PWM_IN_4};
  /* Serial.println("Boot: initializing PWM reader"); */;
  pwm_read_rmt_init(pwm_input_pins, PWM_IN_NUM);
  /* Serial.println("Boot: PWM reader initialized"); */;

  // PID setup
  steeringPID.SetOutputLimits(steering_min_angle, steering_max_angle);  // Set the output contstraints
  steeringPID.SetSampleTime(10);  // Set the sample interval in milliseconds
  steeringPID.SetMode(AUTOMATIC);  // Turn on the control loop

  // Dead Zone setup
  if(dead_zone_width < 0.0){
    dead_zone_width = 0.0;
  }
  if(dead_zone_width > 250){
    dead_zone_width = 250;
  }

  if(dead_zone_gain < 0.0){
    dead_zone_gain = 0.0;
  }
  if(dead_zone_gain > 1.0){
    dead_zone_gain = 1.0;
  }

  dead_zone_width = dead_zone_width / 1.352;  // Adjust Dead Width for units in mm

  // PID low pass setup
  low_pass_alpha = pow(2.71828183, (-1.0 / (100 * 1/(low_pass_cutoff * 6.2831))));
  low_pass_beta = 1 - low_pass_alpha;

  // Attach falling edge interrupt to STM_nDATA_READY pin
  attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
  pwm_reset_readings();

  espPacketFifo_addCmdInt32(0x02 & 0x80000000, 0x80);

  /* Serial.println("Boot: setup complete"); */;
}
// Main Loop ---------------------------------------------------------------------------------------------------------------
void loop() {
  mpu.update();
  static uint32_t last_yaw_print = 0;
  if (millis() - last_yaw_print >= 20) {
    last_yaw_print = millis();
    Serial.println(mpu.getGyroZ(), 3);  // Z-axis yaw rate in degrees/second.
  }

#if ENABLE_RC_INPUT_TEST
  static uint32_t last_rc_print = 0;
  if (millis() - last_rc_print >= 250) {
    last_rc_print = millis();
    /* Serial.printf("RC: PWM_IN_1(GPIO14)=%ld us valid=%d | PWM_IN_2(GPIO21)=%ld us valid=%d\n",
                  (long)pwm_read_rmt_dur(0), pwm_read_rmt_valid(0, 250),
                  (long)pwm_read_rmt_dur(1), pwm_read_rmt_valid(1, 250)); */;
  }
  delay(1);
  return;
#endif
#if ENABLE_CAN_TX_TOGGLE_TEST
  static uint32_t last_toggle = 0;
  static bool pin_high = true;
  if (millis() - last_toggle >= 3000) {
    last_toggle = millis();
    pin_high = !pin_high;
    digitalWrite(TWAI_TX_PIN, pin_high ? HIGH : LOW);
    /* Serial.printf("GPIO36 test: %s\n", pin_high ? "HIGH (3.3 V)" : "LOW (0 V)"); */;
  }
  delay(1);  // Yield to USB and the scheduler during this standalone diagnostic.
  return;
#endif

  static uint32_t last_button_update = 0;
  if (millis() - last_button_update >= 10) {
    last_button_update = millis();
    buttonLogic();  // Maintain the existing 100 Hz debounce without relying on STM32 packets.
  }
  odrive_recoverIfNeeded();
  odrive_pollAndPrintCan();
  odrive_rcUpdate();  // RC mixing and signal-loss stop, independent of STM32 traffic
  odrive_requestEncoderEstimates();  // periodically query one node's pos/vel

  static bool stm_waiting = false;
  static uint32_t stm_wait_started = 0;
  if (data_ready && !stm_waiting) {
    stm_waiting = true;
    stm_wait_started = millis();
  }
  bool stm_packet_received = false;
  if (stm_waiting && Serial1.available() >= STM_TX_PACKET_SIZE) {
    data_ready = 0;
    stm_waiting = false;
    // HardwareSerial::read(buffer, size) reads currently available bytes without waiting.
    StmTxPacket incoming = {};
    if (Serial1.read(incoming.uart_data, STM_TX_PACKET_SIZE) == STM_TX_PACKET_SIZE) {
      stmPacket = incoming;  // Publish only a complete packet to control logic.
      stm_packet_received = true;
    }
    int extra_bytes = Serial1.available();
    for (int i = 0; i < extra_bytes; ++i) Serial1.read();
  } else if (stm_waiting && millis() - stm_wait_started >= 5) {
    data_ready = 0;
    stm_waiting = false;
    int partial_bytes = Serial1.available();
    for (int i = 0; i < partial_bytes; ++i) Serial1.read();
  }

  if(stm_packet_received){

    // update the double X position from the float in the latest STM32 packet
    live_position = stmPacket.items.s1_x;

    // Dead zone computation
    if(dead_zone_enable){
      live_position = live_position - (1 - dead_zone_gain) * dead_zone_width * atan((live_position - target_x_pos) / dead_zone_width);  // compute dead zone
    }

    // Steering control
    low_pass_compute = steeringPID.Compute();  // compute new steering command according to straight PID

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

   
    
    // Check for faults and write servo values
    if(  // Will cut throttle and stop steering if one or more faults are present
      WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_S1_SIGNAL_OVERLOAD)  //  signal overload fault bit
      || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_VDD_FAULT)  // STM32 VDD fault bit
      || (stmPacket.items.temperature > 100)  // STM32 internal temp in °C
      ){
        writePwmMicros(2, 1500);  // Cut throttle on any of the above faults.

      }else{
       //writePwmMicros(2, cruiseControl());  // throttle according to aux ch cruise control logic
       // writePwmMicros(2, cruiseControl2ch());  // throttle according to 2ch cruise control logic
       writePwmMicros(2, pwm_read_rmt_dur(1));  // throttle
       writePwmAngle(1, pid_output);  // write steering angle
       writePwmAngle(3, (180 -pid_output));  // write rear steering angle
    }

    mainLoopCounter();  // increment data loop counter
    chargeControl();  // monitor and control charge port
    espPacketFifo_sendCmd();  // Send one STM32 UART command from the command queue

    // Passthrough test
    //writePwmMicros(1, pwm_read_rmt_dur(0));  // steering
    //writePwmMicros(3, (3000 - pwm_read_rmt_dur(0)));  // Rear steering
    //writePwmMicros(2, cruiseControl2ch());  // throttle according to 2ch cruise control logic
    //writePwmMicros(2, pwm_read_rmt_dur(1));  // throttle
    
  }

  
  

  




//END MAIN LOOP ------------------------------------------------------------
}

// STM32 UART ISR, triggered by pin interrupt
void receiveStmUart(){
  data_ready = 1;  
}

// write a PWM output channel's on-time in microseconds
void writePwmMicros(uint8_t ch, uint32_t micros){
  if (!PWM_OUT_1_AVAILABLE && ch == 1) return;  // Steering output disabled.
  uint16_t duty_cycle = (int32_t)((float)micros * 1e-6 * (float)(PWM_OUT_FREQUENCY * ((1 << PWM_OUT_RESOLUTION_BITS) - 1)));
  ledcWrite(ch, duty_cycle);
}

void writePwmAngle(uint8_t ch, int32_t degrees){
  uint16_t micros = (uint16_t)(1000 + (1000.0 * degrees / 180));
  writePwmMicros(ch, micros);
}

// Increment the main loop counter
void mainLoopCounter(){
  main_loop_counter++;
  if(main_loop_counter == MAIN_LOOP_PERIOD){
    main_loop_counter = 0;
  }
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
		bttn_state = 0;
		bttn_counter = 0;
    if(bttn_click_counter <= BUTTON_CLICK_MAX_LENGTH){
      buttonClickEvent();
    }
	}
  
	if(bttn_counter > BUTTON_DEBOUNCE_PERIODS){
    // Rising edge
		bttn_state = 1;
		bttn_counter = BUTTON_DEBOUNCE_PERIODS;
    bttn_click_counter = 0;
	}

  // long press logic
  if(!bttn_long_state && bttn_state)
		bttn_long_counter++;

	if(bttn_long_state && !bttn_state){
    // Falling edge
		bttn_long_state = 0;
		bttn_long_counter = 0;
	}

	if(bttn_long_counter > BUTTON_LONG_PRESS_PERIODS){
    // Rising edge
		bttn_long_state = 1;
		bttn_long_counter = BUTTON_LONG_PRESS_PERIODS;
    buttonLongPressEvent();
	}
}



// Called on button release if the press duration was less than the click threshold
void buttonClickEvent(){
  digitalWrite(nLED, 0);

  // increment channel
  rx_channel++;  
  if(rx_channel > 4){
    rx_channel = 0;
  }
  espPacketFifo_addCmdInt32(1 | 0x80000000, rx_channel);  // write Rx channel
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new channel value


  digitalWrite(nLED, 1);
}

// called when the long press threshold has been reached
void buttonLongPressEvent(){

  // Calibrate cS1 center
  espPacketFifo_addCmdInt32(4 | 0x80000000, 0);  // Write S1 calibrate center
  espPacketFifo_addCmdInt32(3 | 0x80000000, 0);  // Save new calibration
}

void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd){
  espPacketFifo[esp_packet_fifo_head].items.cid = cid;
  espPacketFifo[esp_packet_fifo_head].items.cmd.int32 = cmd;
  esp_packet_fifo_head ++;
  if(esp_packet_fifo_head == ESP_PACKET_FIFO_LENGTH){
    esp_packet_fifo_head = 0;
  }
}

void espPacketFifo_addCmdFloat(uint32_t cid, float cmd){
  espPacketFifo[esp_packet_fifo_head].items.cid = cid;
  espPacketFifo[esp_packet_fifo_head].items.cmd.flt32 = cmd;
  esp_packet_fifo_head ++;
  if(esp_packet_fifo_head == ESP_PACKET_FIFO_LENGTH){
    esp_packet_fifo_head = 0;
  }
}


void espPacketFifo_sendCmd(){
  if(esp_packet_fifo_head - esp_packet_fifo_tail &&
     Serial1.availableForWrite() >= ESP_TX_PACKET_SIZE){
    for(uint32_t i=0; i<(ESP_TX_PACKET_SIZE); i++){
      Serial1.write(espPacketFifo[esp_packet_fifo_tail].uart_data[i]);
    }
    esp_packet_fifo_tail ++;
    if(esp_packet_fifo_tail == ESP_PACKET_FIFO_LENGTH){
      esp_packet_fifo_tail = 0;
    }
  }
}

void chargeControl(){
  if(CHARGE_AUTO_ENABLE){
    switch (charge_status){
      case 0:  // Idle, waiting for input power
        if(stmPacket.items.vcharge > (stmPacket.items.vbatt + 0.4)){
          charge_status = 1;  // stops car
          charge_start_millis = millis();  // start 1 second timer
        }
        break;

      case 1:  // Input power present, waiting to request charge
        if((millis() - charge_start_millis) > 1000 && !WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
          espPacketFifo_addCmdInt32(9 | 0x80000000, 1);
          charge_status = 2;
        }
        break;

      case 2:  // CP enable request sent, waiting to see charging has enabled
      if(WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
        charge_status = 3;
      }

      if((millis() - charge_start_millis) > 4000 || WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_COOLDOWN)){
        charge_status = -1;  // abandon if charging does not start in 4 seconds, or a charging fault occurred
      }
      break;

    case 3: // Charging, waiting for charging to stop (for any reason)
        if(!WFRX_STATUS_GET_BIT(stmPacket.items.status_bits, WFRX_STATUS_CP_ENABLE)){
          charge_status = -1;
          charge_start_millis = millis();
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

uint16_t cruiseControl(){
  // Passthrough input throttle command until PWM_IN_3 goes above 1500µs for 500ms, then latch on to the last throttle command.
  // Return to passthrough mode as soon as PWM_IN_3 goes to 500 - 1500µs.  This should allow for the RC Tx to be turned off

  uint32_t throttle_input_pwm = pwm_read_rmt_dur(1);  // read throttle PWM input
  switch (cruise_control_status){
    case 0:  // idle, throttle passthrough, looking for aux channel trigger
      if(pwm_read_rmt_dur(2) > 1500){
        cruise_control_timer ++;
      }else{
        cruise_control_timer = 0;
      }

      if(cruise_control_timer > 50){
        cruise_control_status = 1;
      }

      if(throttle_input_pwm > 2100 || throttle_input_pwm < 900){  // verify throttle is in range
        throttle_input_pwm = 1500;
      }
      throttle_micros = throttle_input_pwm;
      return throttle_micros;  // passthrough throttle
      break;

    case 1:  // cruise throttle set, wait for aux channel to go to 500 - 1500µs
      if(pwm_read_rmt_dur(2) > 500 && pwm_read_rmt_dur(2) < 1500){
        cruise_control_status = 0;
        if(throttle_input_pwm > 2100 || throttle_input_pwm < 900){  // verify throttle is in range
          throttle_input_pwm = 1500;
        }
      throttle_micros = throttle_input_pwm;  // passthrough throttle command
      }
      return throttle_micros;
  }

  return 1500;
}

uint16_t cruiseControl2ch(){
  uint32_t throttle_input_pwm = pwm_read_rmt_dur(1);  // read throttle PWM input

  if(throttle_input_pwm > 1550 && throttle_micros < 2000)  {  // Accelerate
    throttle_micros++;
  }else if(throttle_input_pwm < 1400 && throttle_input_pwm > 1200 && throttle_micros > 1500){  // decelerate
    throttle_micros--;
  }else if(throttle_input_pwm > 900 && throttle_input_pwm < 1200){  // stop
    throttle_micros = 1500;
  }

  return throttle_micros;

}

int8_t gpio39Status(){
  if(digitalRead(SPARE_GPIO_IO39)){
    gpio_39_timer ++;  // increment restart timer when pin is high
    if(gpio_39_timer >= GPIO_39_RESTART_CYCLES){
      gpio_39_timer = GPIO_39_RESTART_CYCLES;
      gpio_39_status = 0;
    }
  }else{
    gpio_39_timer = 0;  // hold in reset when pin is low
    gpio_39_status = 1;
  }
  return gpio_39_status;
}

