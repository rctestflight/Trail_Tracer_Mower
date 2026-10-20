// The platform part of the Rx firmware: everything the board does besides driving the vehicle. src/main.cpp calls it
// through include/rx_platform.h. One translation unit: the area files below share their static values and functions
// as one source file did before.
#include "platform/platform_internal.h"

#include "platform/board.inc"       // watchdog, crash counter, last state, start times, loop timing
#include "platform/usb.inc"         // USB text queue, USB link events
#include "platform/settings.inc"    // settings store in RAM, config.json check, settings erase gesture
#include "platform/imu.inc"         // gyro and accelerometer (MPU6050)
#include "platform/motors.inc"      // motor controllers on the CAN bus, servo pulse outputs
#include "platform/rx_link.inc"     // Rx STM32 link and frame decode
#include "platform/radio.inc"       // ESP-NOW, Wi-Fi, text telemetry, monitor link
#include "platform/telemetry.inc"   // telemetry log
#include "platform/commands.inc"    // settings and run commands from USB, the radio and the settings page
#include "platform/updates.inc"     // restarts, ESP32 and Rx STM32 app updates
#include "platform/bench.inc"       // bench commands on USB serial
#include "platform/setup_loop.inc"  // the platform parts of setup() and loop()
