// The vehicle values and functions of src/main.cpp that the platform (src/platform/) reads or calls: telemetry,
// the run commands, the settings, the update checks. A rename in main.cpp needs the same rename here.
#pragma once
#include <Arduino.h>
#include "rover_autopilot.h"
#include "rctf_wfrx_exp32.h"
#include "usb_msc_fat.h"

extern rover::Autopilot roverAp;
extern bool rover_enabled;
extern uint8_t rover_entry_btx;
extern uint8_t rover_exit_btx;
extern bool rover_active;
extern bool rover_need_charge;
extern uint8_t vehicleID;
extern uint8_t rx_channel;
extern int PWM_OUT_2_DEFAULT;
extern bool enable_sensor_2;
bool sensor2Control();  // 0.6.36: sensor 2 acts on behaviour ("Sensor 2" and "Use sensor 2 for control" both on)
extern StuckVehicle stuckDetector;
extern bool odrive_rc_active;
extern float odrive_drive_command;
extern float odrive_steer_command;
extern bool odrive_output_enabled;
extern int8_t charge_status;
extern bool lapFlag;
extern int vehicleStatus;
extern int8_t drive_mode_status;
extern int8_t world_inside;
extern float world_inside_p;
extern int8_t world_heading;
extern bool case2_use_cruise;
extern bool case2_aux_entry_is_manual;
extern bool case2_aux_changed;
extern int case2_throttle_current;
extern int BTxThreshold;
extern float BTx1Signal;
extern float BTx2Signal;
extern double steering_output;
extern double steering_rate_target_dps;

bool steeringRateGyroValid();
void servoIdle();
void applyDeviceConfig(const DeviceConfig& cfg);
