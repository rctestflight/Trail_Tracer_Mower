// The board side of the Rx firmware, as src/main.cpp uses it: pins, start-up and the platform part of each loop()
// pass, the Rx STM32 link, the motor controllers, the servo outputs, the gyro, the settings and the USB text.
// The code is in src/platform/ (one area per file). main.cpp holds the vehicle behaviour.
#pragma once
#include <Arduino.h>
#include <MPU6050_light.h>
#include "cpu_prof.h"
#include "rt_prof.h"
#include "wfrx_stm32_comms.h"
#include "stm_v2.h"
#include "pwm_read_rmt.h"
#include "usb_msc_fat.h"
#include "usb_text.h"

// ===== Board pins =====
// SN65HVD230 CAN transceiver connected to the native ESP32-S2 TWAI controller
// P9 (GPIO36) - TXD
// P10 (GPIO38) - RXD

// MPU6050 I2C
// SDA - GPIO39
// SCL - GPIO40

// Motor Arming Switch - P11 (GPIO35)
// Buzzer - P8 (GPIO37)

// Green LED pin (Low = ON, high = OFF)
#define nLED GPIO_NUM_1

// GPIO from STM32 for new data interrupt
#define STM_nDATA_READY GPIO_NUM_8

//User button pin / BOOT pin
#define BUTTON_PIN GPIO_NUM_0

// Nano sonar stream on UART0 (115200, 8N1); TX stays on the ESP32-S2 default.
#define SONAR_UART_RX_PIN GPIO_NUM_33  // Former PWM input 4.
#define SONAR_UART_TX_PIN GPIO_NUM_43

// PWM Inputs
#define PWM_IN_NUM 3
#define PWM_IN_1 GPIO_NUM_14
#define PWM_IN_2 GPIO_NUM_21
#define PWM_IN_3 GPIO_NUM_15
// PWM input 4 (GPIO33) is now SONAR_UART_RX_PIN; no RC capture on that pin.

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

// ===== Start-up (called from setup(), in this order) =====

/**
 * @brief First part of the start-up: the Rx STM32 serial line, the crash counter, the settings from the USB drive
 *        and NVS into g_deviceConfig.
 * @return true when this boot runs as the PC monitor: the monitor started, and setup() must return at once.
 */
bool platformBegin();

/**
 * @brief Second part of the start-up, after the settings went to the vehicle values: the radio mode, the web pages,
 *        the telemetry log, USB serial with the boot lines, the update functions and the Rx STM32 serial port.
 */
void platformStart();

/**
 * @brief Opens the settings erase window on a power-on: a button press in the first 3 s, held 5 s, sets every
 *        setting and the Wi-Fi details to their defaults.
 */
void eraseWindowOpen();

/**
 * @brief Starts the CAN bus to the motor controllers. can_init_ok tells if it runs.
 */
void canBegin();

/**
 * @brief Starts the RC receiver inputs (PWM_IN_1 to PWM_IN_3). Read them with pwm_read_rmt_dur().
 */
void rcInputsBegin();

/**
 * @brief Starts the link to the Rx STM32: the frame interrupt, the start handshake, the first frames.
 *        Also clears the RC input readings.
 */
void rxLinkBegin();

/**
 * @brief Starts the radio that the settings ask for: ESP-NOW or Wi-Fi.
 */
void radioStart();

/**
 * @brief Last part of the start-up: the watchdogs on, and the "SETUP ms=" USB line.
 */
void setupDone();

// ===== Each loop() pass (called from loop(), in this order) =====

/**
 * @brief The platform part at the start of each pass: watchdog, USB output, update progress.
 * @return true when the platform uses this whole pass (an update, the monitor role, a bench pass-through):
 *         loop() must return at once.
 */
bool platformLoopStart();

/**
 * @brief Checks config.json on the USB drive (3 s after the PC stops writing it).
 * @return true when the file changed and g_deviceConfig holds the new settings: call applyDeviceConfig().
 */
bool settingsFileChanged();

/**
 * @brief A setting from config.json or NVS that was not valid took its default: one USB line and one log event each.
 */
void settingsBadValues();

/**
 * @brief Reads the frames that the Rx STM32 sent.
 * @return true when a new frame is in stmPacket and this pass runs the control step. At short frame periods the
 *         frames between control steps are only decoded and logged (the control runs at 100 Hz at most).
 */
bool rxFrameReady();

/**
 * @brief The telemetry log sample of this frame and the outputs it gave. Call after the control step.
 */
void telemetryFrame();

/**
 * @brief Ends the frame part of the pass: the loop timing counters.
 */
void rxFrameDone();

/**
 * @brief Rx STM32 commands and answers, the link watch (an Rx STM32 that restarted gets its start again).
 */
void rxLinkService();

/**
 * @brief The radio: ESP-NOW or Wi-Fi telemetry, commands from the PC, the web pages.
 */
void radioService();

/**
 * @brief The telemetry log (status record each second, events), the crash counter, the update confirm.
 */
void logService();

/**
 * @brief 0.6.34 (operator 2026-10-23): one log event for each button press, with the action it started (event 23:
 * a = action, b = press length in ms, text = the action in words). Setting and mode changes keep their own events.
 */
enum ButtonAction { BTN_NONE = 0, BTN_AUTO_ON = 1, BTN_CRUISE = 2, BTN_CENTRE_CAL = 3, BTN_STEER_DIR = 4,
                    BTN_ERASE_START = 5, BTN_ERASE_CANCEL = 6, BTN_ERASE_DONE = 7 };
void buttonEvent(ButtonAction action, uint32_t held_ms, const char* text);

/**
 * @brief 0.7.5: one log event for the sonar (event 24, rec::EV_SONAR): a = the sonar number 1 to 4 of a trigger (b = its
 * distance, text = the action), or a = 0 for the Nano stream (b = 1 records arrive, 0 no record: the sonar is ignored).
 */
void sonarEvent(int32_t a, int32_t b, const char* text);

/**
 * @brief USB serial commands, USB link events and the heartbeat line each second.
 */
void benchService();

/**
 * @brief The end of each pass: the monitor LED blink, one gyro sample, then a wait for the next Rx STM32 frame
 *        (5 ms at most).
 */
void loopEnd();

// ===== Settings =====

/**
 * @brief All settings now in effect (config.json on the USB drive, NVS, "set" commands). The names are in
 *        include/settings_table.h.
 */
extern DeviceConfig g_deviceConfig;

/**
 * @brief true after two crash resets in a row: no telemetry, no rover mode, Rx STM32 at 10 ms.
 */
extern bool safe_mode;

/**
 * @brief Sends the DSP settings (dsp_*, dt_channels) to the Rx STM32 when one of them changed.
 * @param cfg the new settings
 */
void dspSettingsCheck(const DeviceConfig& cfg);

/**
 * @brief Sets the radio transmit power from the setting espnow_tx_power_dbm.
 */
void applyTxPower();

// ===== Button =====

/**
 * @brief The button (BUTTON_PIN) as pressed or not; the bench command "btn0 down" also counts as pressed.
 * @return 1 when pressed, 0 when released
 */
uint8_t buttonRead();

/** @brief Results of buttonEraseGesture(). */
enum { ERASE_NONE = 0, ERASE_HOLD = 1, ERASE_RELEASED = 2 };

/**
 * @brief The settings erase gesture: a press that starts in the first 3 s after power-on, held 5 s, sets every
 *        setting and the Wi-Fi details to their defaults and restarts the board.
 * @param pressed the button now (buttonRead())
 * @return ERASE_NONE: the button is free for the vehicle. ERASE_HOLD: the gesture holds the button.
 *         ERASE_RELEASED: released before 5 s, nothing erased; clear the press counters.
 */
int buttonEraseGesture(uint8_t pressed);

// ===== Rx STM32 =====

/**
 * @brief The newest frame from the Rx STM32: wire position (s1_x, s1_y, s1_radius), BTx radius, status bits,
 *        battery and charge values, temperature.
 */
extern StmTxPacket stmPacket;

/**
 * @brief The frame decoder of the Rx STM32 link (frame count in stmDec.st.frames).
 */
extern stmv2::Decoder stmDec;

/**
 * @brief Queues a command with a whole-number value to the Rx STM32. One command goes with each frame.
 * @param cid the command number; a write command is (cid | 0x80000000)
 * @param cmd the value
 */
void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd);

/**
 * @brief Queues a command with a decimal value to the Rx STM32. One command goes with each frame.
 * @param cid the command number; a write command is (cid | 0x80000000)
 * @param cmd the value
 */
void espPacketFifo_addCmdFloat(uint32_t cid, float cmd);

/**
 * @brief The time between two Rx STM32 frames now in effect.
 * @return the frame period in seconds
 */
double stmFramePeriodS();
/**
 * @brief The period factor: the update period in effect / 10 ms (lib/frame_pid/frame_period.h), for the constants
 * that count packets. 1 at 10 ms, 2 at 20 ms; computed again when stmFramePeriodS() changes.
 */
double periodRatio();

// ===== Servo outputs =====
// PWM OUT configuration
#define PWM_OUT_FREQUENCY 50  // Frequency (Hz) 50 is default
#define PWM_OUT_RESOLUTION_BITS 14  // 14-bits is maximum

/**
 * @brief Sets the pulse width of one servo output.
 * @param ch the output, 1 to 4 (PWM_OUT_1 to PWM_OUT_4)
 * @param micros the pulse width in microseconds (1500 = centre)
 */
void writePwmMicros(uint8_t ch, uint32_t micros);

// ===== Motor controllers (ODrive) on the CAN bus =====
#define ODRIVE_NODE_COUNT 6  // Node IDs 0-5

// ODrive controller config values (from verified working CAN_Example_src)
#define ODRIVE_CONTROL_MODE_VELOCITY  2  // CONTROL_MODE_VELOCITY_CONTROL
#define ODRIVE_INPUT_MODE_PASSTHROUGH 1  // INPUT_MODE_PASSTHROUGH
#define ODRIVE_AXIS_STATE_IDLE        1
#define ODRIVE_AXIS_STATE_CLOSED_LOOP 8
#define ODRIVE_VEL_LIMIT_RPS          3.0f   // velocity limit [rev/s]
#define ODRIVE_CURRENT_LIMIT_A        6.0f   // current limit [A]
#define ODRIVE_VEL_STALE_MS 60  // a wheel speed older than this is not used

/** @brief true while the CAN bus runs (not proof that a motor controller answers). */
extern bool can_init_ok;
/** @brief Axis error bits of each motor controller (heartbeat message). */
extern uint32_t odrive_axis_errors[ODRIVE_NODE_COUNT];
/** @brief true for each motor controller that sent a heartbeat since start. */
extern bool odrive_heartbeat_received[ODRIVE_NODE_COUNT];
/** @brief Active error bits of each motor controller. */
extern uint32_t odrive_active_errors[ODRIVE_NODE_COUNT];
/** @brief Disarm reason bits of each motor controller. */
extern uint32_t odrive_disarm_reasons[ODRIVE_NODE_COUNT];
/** @brief Wheel speed of each motor controller, rev/s. */
extern float odrive_vel_rps[ODRIVE_NODE_COUNT];
/** @brief millis() of the last wheel speed of each motor controller. */
extern uint32_t odrive_vel_millis[ODRIVE_NODE_COUNT];
/** @brief true for each motor controller that sent a wheel speed since start. */
extern bool odrive_vel_received[ODRIVE_NODE_COUNT];

/**
 * @brief Sets the axis state of one motor controller.
 * @param node_id the motor controller, 0 to ODRIVE_NODE_COUNT - 1
 * @param axis_state ODRIVE_AXIS_STATE_IDLE or ODRIVE_AXIS_STATE_CLOSED_LOOP
 */
void odrive_setAxisState(uint8_t node_id, uint32_t axis_state);

/**
 * @brief Sends a wheel speed command to one motor controller.
 * @param node_id the motor controller
 * @param vel the speed, rev/s
 * @param torque_ff the torque feed-forward, Nm (0 if left out)
 */
void odrive_setInputVel(uint8_t node_id, float vel, float torque_ff = 0.0f);

/**
 * @brief Sets the speed and current limits of one motor controller.
 * @param node_id the motor controller
 * @param vel_limit the speed limit, rev/s
 * @param current_limit the current limit, A
 */
void odrive_setLimits(uint8_t node_id, float vel_limit, float current_limit);

/**
 * @brief Sets the control mode and input mode of one motor controller.
 * @param node_id the motor controller
 * @param control_mode for example ODRIVE_CONTROL_MODE_VELOCITY
 * @param input_mode for example ODRIVE_INPUT_MODE_PASSTHROUGH
 */
void odrive_setControllerMode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);

/**
 * @brief Clears the errors of one motor controller.
 * @param node_id the motor controller
 */
void odrive_clearErrors(uint8_t node_id);

/**
 * @brief Restarts the CAN bus after a bus-off error. Call each loop() pass.
 */
void odrive_recoverIfNeeded();

/**
 * @brief Reads the CAN messages from the motor controllers (heartbeat, errors, wheel speeds, bus voltage) and
 *        prints new faults on USB. Call each loop() pass.
 */
void odrive_pollAndPrintCan();

/**
 * @brief Asks the motor controllers for wheel speeds, bus voltage and errors when they do not send them alone.
 *        Call each loop() pass.
 */
void odrive_requestEncoderEstimates();

// ===== Gyro (MPU6050) =====

/** @brief The gyro and accelerometer: mpu.getGyroZ() is the yaw rate in deg/s. */
#ifdef IMU_I2C_NB
#include "imu_nb.h"
extern ImuNb mpu;  // test build: I2C transfers that never hold the loop (lib/i2c_nb)
#else
extern MPU6050 mpu;
#endif
/** @brief true once the gyro calibration is done and the yaw rate is a number. */
extern bool gyro_ready;
extern int16_t imu_roll16;   // the accelerometer roll, 65 536 = one turn; 0 with no IMU
/** @brief millis() of the last gyro sample. */
extern uint32_t gyro_sample_ms;
/** @brief Always false (retired: the gyro check in the motor output keeps the motors off with no gyro). */
extern bool imu_halt;

/**
 * @brief Starts and calibrates the gyro, then takes one sample per call. Keep the board still until the USB line
 *        "MPU6050 calibration complete".
 */
void serviceMpu();
/** @brief Test build IMU_I2C_NB: takes the newest finished IMU read, before a use of the values. Empty otherwise. */
void imuTake(uint8_t site = 3);  // site: 0 autopilot step, 1 motor output step, 2 IMU service (test build counts)
#ifdef IMU_SAVE
int16_t imuRoll16();       // the roll from the newest sample, computed once per sample when asked
double gyroHeadingDeg();   // the heading from the whole-number integral
#define IMU_ROLL16() imuRoll16()
#else
#define IMU_ROLL16() imu_roll16
#endif

// ===== USB text =====

/** @brief true after the USB command "verbose on": extra lines each second. */
extern bool usb_verbose;

// ===== CPU profiler: loop parts =====
// The last state before a reset, kept through any reset except power loss: the next boot writes it
// to the vehicle log (event 14) and to USB. stage: the part of loop() that ran last.
enum : uint8_t { ST_SETUP = 0, ST_USB = 1, ST_UPDATE = 2, ST_PACKET = 3, ST_RADIO = 4, ST_LOG = 5, ST_BENCH = 6, ST_IDLE = 7 };
struct LastState { uint32_t magic, uptime_ms, heap_min; uint8_t stage, drive_mode, ap_state, section; };  // section: the last mark()
extern LastState last_state;
/** @brief Notes the part of loop() that runs now, for the record after a reset. */
inline void stage(uint8_t st) { last_state.stage = st; last_state.uptime_ms = millis(); }
// Loop parts for the CPU profiler (cpuprof::section; bench "cpu"). A part runs until the next mark.
enum : uint8_t { SEC_USB = 0, SEC_UPDATE, SEC_BRIDGE, SEC_MONITOR, SEC_DRIVE, SEC_MSC, SEC_PACKET, SEC_ROVER, SEC_SAMPLE,
                 SEC_STMCMD, SEC_RADIO, SEC_LOG, SEC_BENCH, SEC_DEBUG, SEC_MPU, SEC_WAIT,
                 SEC_R_TLM, SEC_R_CMD, SEC_R_LOG, SEC_R_WIFI, SEC_IMU,  // imu: the IMU sample take and read start (test build IMU_I2C_NB)
#ifdef DIAG_CTRL_PROF
                 SEC_C_PRE, SEC_C_PID, SEC_C_AP, SEC_C_OUT,  // diag: parts of the control step
#endif
                 SEC_N };  // diag_cpu: the end-of-pass wait; radio sub-parts
// Diagnostic build (diag_cpu): the kind of the loop pass that runs now (cpuprof::passEnd, rtprof::passKind); the CPU cycles
// inside mark() (the cost of the part marks, bench "cpu").
extern uint8_t pass_kind;
extern uint64_t mark_cycles;
// One mark per loop part: the CPU profiler (shares) and the real-time path timing (wall time per part and pass).
/** @brief Starts a loop part for the CPU profiler (bench command "cpu"): the time until the next mark counts to it. */
inline void mark(uint8_t s) {
  const uint32_t c0 = ESP.getCycleCount();
  cpuprof::section(s); rtprof::part(s); last_state.section = s;
  mark_cycles += ESP.getCycleCount() - c0;
}
