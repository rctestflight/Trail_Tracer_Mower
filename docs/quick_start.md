# Quick start: Rx firmware 0.5.60

This branch is `main` plus the Rx v2 firmware: rover mode, the monitor role and the STM32 v2 link. Your drive code is unchanged: ODrive CAN drive, gyro rate steering, manual mode and coil telemetry. Details are in [rover_guide.md](rover_guide.md).

## 1. Install from the web programmer page

1. Open https://rctestflight.github.io/Trail_Tracer_Mower/ in Chrome or Edge.
2. In the menu, select `Trail_Tracer_Mower`, branch `feature/rx-v2-0.5.60`.
3. Connect the Rx by USB. Press "Connect an Rx" and select the board.
4. Press the full install. It writes the ESP32 firmware, then the STM32 app 2.0.24.
5. At the end, unplug USB and all vehicle power, wait 3 s, and plug USB in again.

**CAUTION:** The full install erases the settings. Copy `config.json` from the rover's USB drive before you start.

## 2. Monitor set-up

1. Install the same firmware on a second Rx. This Rx needs no sensors and no vehicle.
2. Each rover needs its own `vehicleID`. The monitor receives all rovers.
3. Connect the monitor Rx to the PC by USB. The telemetry program puts it in the monitor role.

## 3. Rover mode

1. Leave `rover_mode` at rover, the default (USB drive `config.json`, USB `set`, or the telemetry program).
2. Keep the RC transmitter on for the first tests. Click the board button to start rover mode. Move the aux switch to stop it.
3. Read sections 1 and 2 of [rover_guide.md](rover_guide.md) before the first drive: steering sign, speed settings and dock direction.

**WARNING:** With no RC throttle signal at power-on, the rover enters rover mode 0.5 s after power-on and drives.

## 4. Telemetry

1. Install Python 3.12 or later from python.org. Tick "Add python.exe to PATH".
2. Get the telemetry program from its repository (invitation by e-mail). Follow its `INSTALL.md`.
3. Start `start.bat`. The screen opens at `http://127.0.0.1:8795/` and shows each rover that the monitor receives.
