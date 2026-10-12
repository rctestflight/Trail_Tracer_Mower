# Trail Tracer rover firmware: user guide

Branch `feature/rover-autopilot`, firmware 0.5.25 (`ROVER_FW_VERSION` in `src/main.cpp`). The autopilot ran in simulation (rover-sim). Firmware 0.5.24 runs on two bench Rx boards (a rover and a monitor) with no vehicle. One image serves both Rx roles. Every board starts as a rover; a PC can put a board on USB into the monitor role (section 5). No part of this guide drove a real rover yet: see "Not verified" at the end.

## 1. What the rover does on its own

The rover is standalone. It needs no Monitor Rx, no PC and no radio to behave correctly. A Monitor Rx and the telemetry program on a PC only add telemetry and remote commands.

| Mode | How it starts | What it does |
|---|---|---|
| 0, manual | RC transmitter on at power-on, aux switch in manual | RC throttle and steering pass through |
| 1, cruise | aux switch to auto (RC on) | cruise control on the RC throttle |
| 2, autonomous | **no RC throttle signal 0.5 s after power-on**, or a click of the board button, or `run start` from the telemetry program | the autopilot (`rover_enabled` true) or the wire follower (`rover_enabled` false) |

The autopilot bounces inside the perimeter wire, goes home on low battery or after a set time, follows the wire to the dock in one direction, charges, and departs again (`lib/rover_autopilot/README.md`).

**WARNING:** With no RC throttle signal at power-on, the rover enters mode 2 0.5 s after power-on and starts to drive. Keep the RC transmitter on for the first tests: the rover then starts in manual mode, a click of the board button enters mode 2, and a change of the aux switch leaves mode 2.

## 2. Before the first drive

**WARNING:** Start the rover at least 1 m inside the wire, facing away from it. The STM32 reports the wire only after 0.5 s in range. A rover that starts on the wire, facing out, can leave the perimeter before it sees the wire; the lost timeout (`rover_lost_timeout_s`, 30 s) then stops it.

1. Flash the firmware (section 8).
2. Set the settings (section 3).
3. Check the ODrive CAN watchdog on all six nodes (an ODrive setting). With it off, an ODrive keeps its last speed when the ESP32 stops sending, for example after a restart.
4. Check the steering sign with the wheels off the ground: set `rover_enabled` true and enter mode 2; hold the sensor near the wire. The rover must stop and pivot away from the wire. If it pivots toward the wire, invert `pid_direction`. If the wire follower already follows correctly, the sign is correct.
5. Measure the two speed settings on the ground: full drive in mode 0 gives the ground speed (`rover_full_speed_mps`); full steer gives the pivot rate (`rover_full_yaw_rate_dps`). A wrong pivot rate gives wrong pivot angles.
6. On the first runs, check the dock direction. The autopilot takes `s1_x` positive as the wire on the left. If the rover follows the wire counter-clockwise with `rover_goal_cw` true, invert `rover_goal_cw`.
7. Check the IMU at boot: USB shows `MPU6050 +-8 g, +-500 deg/s`. With "IMU HALT", the rover does not drive (section 6).
8. Watch the first runs in rover mode. Rover mode does not detect a stall or an obstacle, so the rover pushes on. Stop it with the RC, `run stop`, or the STOP button of the telemetry program. In the simulator, a rover that lost the wire signal in roam drove about 20 m before the lost timeout stopped it.

## 3. Settings

Three ways, all with the same keys:
- `config.json` on the rover's USB drive;
- over USB: `python tools/rover_settings.py COM7 set <key> <value>`, then `... COM7 save` (or a serial terminal: `set`, `save`);
- the telemetry program's Vehicle control, over a Monitor Rx (needs the fleet password, `cmd_key`).

`docs/settings_def.json` defines every key: type, unit, limits, default, live or stationary-only, and a description. The telemetry program uses an unchanged copy of it.

**`save`.** `save` writes all settings to `config.json` and to the ESP32's flash. The answer is `save=json`. A PC that has the USB drive open sees no medium for 2 s, and then reads the new file.

| Key | Default | Meaning |
|---|---|---|
| rover_enabled | false | Mode 2 runs the autopilot instead of the wire follower |
| rover_goal_cw | true | Dock direction along the wire, clockwise seen from above |
| rover_sensor_height_mm | 150 | Height of the sensor centre above the wire |
| rover_full_speed_mps | 1.0 | Ground speed at full drive command (measure it) |
| rover_full_yaw_rate_dps | 115 | Pivot rate at full steer command (measure it) |
| rover_seek_vbatt | 0 | Go home below this filtered battery voltage, V (0 = off) |
| rover_seek_after_s | 600 | Go home this long after the last charge, s (0 = off) |
| rover_entry_btx | 1 | BTx channel before the dock: slow approach (0 = none) |
| rover_exit_btx | 2 | BTx channel just past the dock exit: wrong-way guard (0 = none) |
| rover_lost_timeout_s | 30 | Stop for good after this long without the wire (0 = never) |
| rover_speed_limit_mps | 0.5 | Cap on every autopilot speed, m/s. Pivots are not affected |
| battery_cells | 2 | Series cells (2 = 2S). The telemetry program scales its power graph from it |
| telemetry_enabled | true | Vehicle log over ESP-NOW |
| espnow_tx_power_dbm | 19.5 | The radio transmit power, 2 to 19.5 dBm, in 0.25 dB steps. A change applies at once. A low power can cut the radio link to the vehicle |
| telemetry_rate_hz | 20 | The sample rate of the vehicle log, in Hz: 100, 50, 25, 20, 10, 5, 4, 2 or 1. Below 100 Hz, an anti-alias filter runs first. A change applies at once |
| imu_required | true | When the IMU (the MPU6050 motion sensor) fails at start-up, the rover does not drive, as on `main`. Set false only for a board with no IMU |
| cmd_key | 0 | Fleet password for remote commands. A command must carry the same number. 0 is a blank password. Put the same number in every vehicle and in the telemetry program. It goes in clear: a guard against mistakes, not an attack. USB or `config.json` only |

**Stationary-only keys.** These change only while the rover stands still (docked, both wheels still with zero commands, or no ODrive on the bus); otherwise the answer is result 4, `not while moving`. `save` also needs it.

| Keys | Why |
|---|---|
| `rx_channel`, `enable_sensor_2`, `CHARGE_VBATT_STOP`, `CHARGE_MAX_CURRENT`, `CHARGE_MAX_SECONDS` | The STM32 changes at once: the wire is lost until the channel matches, and a charge limit changes. `rx_channel` also causes one STM32 flash write |
| `pid_direction`, `PWM_OUT_1..4_DEFAULT/MIN/MAX` | A jump of the steering sign, a servo centre or a limit |
| `rover_enabled`, `rover_goal_cw`, `rover_sensor_height_mm`, `rover_full_speed_mps`, `rover_full_yaw_rate_dps`, `rover_entry_btx`, `rover_exit_btx`, `BTxFollowChannel` | A step of the drive command, or a change of mission |
| `espnow_enabled`, `telemetry_enabled` | Starting Wi-Fi stops the control loop for about 220 ms |

All other keys are live. Every `set` and `save`, applied or refused, is an event in the vehicle log. Beacon placement for the wrong-way guard: `lib/rover_autopilot/README.md`, "Exit beacon guard".

**STM32 settings.** The ESP32 sends these five settings to the STM32 at start-up and after a change. The STM32 holds them in RAM only; after an STM32 start they are 0, and a charge limit of 0 blocks every charge.

| Key | Default | Range | What the STM32 does |
|---|---|---|---|
| `rx_channel` | 1 | 1 to 5 | Sensor 1 listens to this transmitter channel |
| `enable_sensor_2` | false | | Sensor 2 on or off. Sensor 2 always listens to transmitter channel 1. From 0.5.25 a change applies with no restart |
| `CHARGE_VBATT_STOP` | 11.9 | 6 to 16 V | Ends the charge when the battery voltage is above this value. STM32 limit 16.0 V |
| `CHARGE_MAX_CURRENT` | 6.0 | 0.1 to 6 A | Stops the charge above this current, then waits 10 s. STM32 limit 6.0 A |
| `CHARGE_MAX_SECONDS` | 180 | 5 to 1800 s | Stops a charge after **2 x** this value: 360 s with the default. STM32 limit 3600 s. Shipped units (V1.0) send 2 x the value too |

A value above a limit in an older `config.json` becomes the limit at start-up; the STM32 used the limit already. From 0.5.25 the rover keeps the STM32 equal to `config.json`:

- It sends one command at a time and checks the STM32 answer (`rid`, `response`, status bit `CMD_ACK`). A command with no answer goes again, 3 sends at most; the save and centre-calibration commands go once only.
- It reads one of the five values back every 2 s, and sends all five again when a value differs, or when the STM32 packets come back after 0.5 s or more without them.
- An STM32 that restarts alone waits in its boot checker and sends no packets. After 1 s without packets, the rover sends HELLO and BOOT again (3 times at most per gap, 5 s apart), then the five settings.
- Each failure is event 18 (`stm32_cmd`) in the vehicle log and a `STMCMD` line on USB. `stat` prints the counters.

## 4. The rover without a Monitor Rx

Checked in the source (0.5.0), every ESP-NOW path:

| Path | Without a Monitor Rx |
|---|---|
| ESP-NOW start (`setup()`, once) | Starts in about 43 ms and waits for nobody. If it fails, the radio stays off and the rover goes on. With `espnow_enabled` and `telemetry_enabled` both false, Wi-Fi never starts |
| Vehicle log, status text | Broadcast: no receiver needed. A failed send is counted, nothing waits |
| Backfill, ACK, PONG | Sent only to a board that asked |
| CMD, STOP, RESTART | Act only when a board sends them with the fleet password |

So the rover drives the same with or without a Monitor Rx, and when a Monitor Rx appears or goes.

**The vehicle log without a Monitor Rx.** The rover keeps the last 70 samples (3.5 s at 20 Hz) and the last 24 status records in RAM, and the status and event records in a 448 KB flash ring (about 1.5 h, survives a power cycle). After a run, connect USB and use a serial terminal (115200 baud):

- `log info`: which records the rover holds, for this boot and the previous one;
- `log dump 1 0 1000000`: all status and event records; `log dump 0 0 1000000`: all held samples (one `LOG <hex>` line each, then `LOG end`);
- `log dumpprev <stream> <from> <count>`: the previous boot's records.

Or in one step, with the rover on USB (needs Python with pyserial; COM7 is an example):

**bash**
```bash
python tools/rover_log.py usbdump COM7 --stream 1 --out status.jsonl
```

Decode a saved terminal log on the PC:

**bash**
```bash
python tools/rover_log.py decode terminal.log --out records.jsonl
```

**After a crash or a reset.** At each start, the rover writes what it did before the reset. This record survives every reset except a power loss.

- USB shows `PREV reset=<reason> uptime_ms=<time> stage=<part of the loop> mode=<drive mode> ap=<autopilot state> heap_min=<bytes> crashes=<count>`.
- The vehicle log holds the same data as event 14 (`prev_boot`). Event 1 (`boot`) holds the reset reason and the firmware version.
- Reset reason 4 is a crash, 6 is the loop watchdog, and 1 is a power-on.
- At power-on, event 15 (`stm32`) holds the STM32 answer, or "no answer".
- After 3 crashes in a row, the rover starts in safe mode: no log and no radio, but it still drives.

## 5. With a Monitor Rx and the telemetry program

A Monitor Rx is an Rx board on the PC's USB port with this same firmware in the monitor role. It receives the vehicle log and asks the rover again for records it missed, while the rover still holds them. The telemetry program (Trail-Tracer-PC-Telemetry) shows the data, uploads it, and sends commands. Layouts: `docs/espnow_protocol.md`.

**Monitor role.** Every board starts as a rover. The USB command `monitor` restarts the board into the monitor role, for that boot only. The PC software sends `monitor` again after each restart of the board. The board goes back to the rover role after any restart, and 2 s after it loses USB. So a board with no USB is always a rover. The rover refuses `monitor` while it moves, during an update, and while a new image waits for its confirm. The line protocol of the monitor role is in `src/monitor_role.cpp`.

Remote commands (the telemetry program sends them; all need the fleet password):

| Command | Effect |
|---|---|
| `run start` | Mode 2. Refused (`run=stopped rc_manual`) when an RC transmitter is on with its aux switch in manual: the RC has priority |
| STOP (frame 0x12) | Mode 0 at once; with no RC, neutral output. Never busy. The telemetry program sends it 5 times |
| `get run` | `run=running` or `run=stopped` |
| `get`, `set`, `save`, `list` | Settings, as in section 3 |
| RESTART "RST!" | Restart |

A remote stop holds until the next restart. After a restart with no RC signal, the rover enters mode 2 again (section 1).

**Monitor heartbeat.**

- Each second, a Monitor Rx sends a MON message. The rover then logs event 17 "monitor" and sets byte 67 of its status record to the age of the last MON message. When the PC sends its clock to the Monitor Rx, the rover logs event 16 "time". Each status record then also holds the PC time (bytes 68 to 73).
- **Green LED:** each MON message inverts the green LED for 100 ms. A blink once a second on top of the drive-mode pattern shows that a Monitor Rx hears the rover.

## 6. IMU (MPU6050)

The autopilot does not use the IMU (it takes its pivot angles from its own commands). The vehicle log records it for analysis.

- **Failed IMU:** when the MPU6050 does not start, the rover does not drive, as on `main`. USB, updates, the log and the radio still work. The rover sends "IMU HALT" on USB each second. The boot event 12 shows "DRIVE OFF". The setting `imu_required` false removes this stop, for a board with no MPU6050.
- **Data:** all six axes in the sample blocks 0x22 and 0x25, at the log rate: accelerometer x, y, z in mg, gyro x, y, z in 0.02 deg/s.
- **Ranges:** accelerometer +-8 g, gyro +-500 deg/s. Low-pass filter 44 Hz.
- **Axes:** the sensor's own axes, as mounted. The PC rotates them to the vehicle frame.
- **Calibration:** at each boot the rover measures the gyro offsets for 0.5 s; keep it still while it boots. The accelerometer is not offset-corrected (the library's correction assumes a flat mounting); the PC levels it.
- **Boot event 12 ("imu")** records the ranges and the gyro offsets, or "absent".

## 7. Stops in rover mode

| Stop | Condition | Ends |
|---|---|---|
| As the wire follower | Signal overload, STM32 supply fault, over-temperature, charging | When the condition ends |
| Lost wire | No wire for `rover_lost_timeout_s` | Leave mode 2 and enter it again |
| Stall | Drive command over 15 % of full while the wheel speeds (ODrive nodes 0 and 1) stay under a quarter of it, for 3 s. Without wheel speeds: the stuck detector while the wire is in range | Leave mode 2 and enter it again |
| RC aux switch | A change of the aux channel (RC on) | Mode 0 or 1 |
| STOP | From the telemetry program | `run start`, the button, or a restart |

## 8. Flashing and updates

**First flash, once per board (BOOT button):**
1. Hold BOOT while you power the board on (USB 303a:0002).
2. Run `pio run -t upload`. esptool then reports that it cannot leave download mode: this is expected.
3. Power-cycle the board without BOOT.

**Every later update, over USB with no button:** `python tools/ota_upload.py COM7 .pio/build/lolin_s2_mini/firmware.bin --expect rover-firmware` (Python with pyserial). The tool refuses an image that is not an ESP32-S2 app with the text `rover-firmware`, before it opens the port. With `--quiet-check`, it also waits for a regular heartbeat for 10 s before it starts. The rover accepts it only while it stands still. It stops all its work for the update, writes the image into the slot that is not running (each line with a CRC-32, the whole image with a SHA-256), and restarts into it. The tool then confirms the new image over USB. With no confirm in 120 s, or after a crash, the bootloader starts the previous image again. About 80 s for an 880 KB image.

The loop watchdog is a timer that restarts the board when `loop()` stops for 5 s. The loop watchdog pauses during an update. A USB fault on the PC side does not restart the board.

## 9. USB serial commands (115200 baud)

| Command | Effect |
|---|---|
| `get <key>`, `set <key> <value>`, `save`, `list` | Settings |
| `run start`, `run stop` | As in section 5 |
| `log info`, `log dump ...`, `log dumpprev ...` | Vehicle log, section 4 |
| `boot` | Version, reset reason, start-up times |
| `stat` | CRC errors, UART overruns, ESP-NOW counters |
| `restart` | Software restart (the RAM log survives it). The rover refuses it while it moves. |
| `monitor` | Restart into the monitor role, for one boot (section 5) |
| `ota ...` | Used by `ota_upload.py` |

Bench only: `timing on|off`, `thru`, `loss`, `send`, `erase off|on`, `flashtest`, `dld`, `crash` (a forced crash, only while the rover stands still), `stm rom|peek|tx|off` (first load of a blank STM32). The transmit power is the setting `espnow_tx_power_dbm`. The Monitor Rx measures the signal strength. The rover prints `BOOT rover-firmware <version> reset=<reason>` at each boot and an `HB` line each second. Its `dtr=` field shows whether the PC holds the port with DTR on; the rover sends its output either way.

## 10. Known limits

- A flash erase while the rover stands still loses 3 or 4 STM32 packets (about every 32 s on a bench board, which always counts as standing still). While it drives, the rover does not erase; 48 sectors kept erased cover about 25 min of driving.
- The samples stay in RAM for 70 sample times (50 with the IMU): 3.5 s at 20 Hz. A Monitor Rx out of range for longer loses them. The status records stay in flash.
- A Windows PC can stop enumerating a USB port after failed requests; only a replug clears it. Do not open the port while the rover restarts.
- After a PC opens the USB port, the board sometimes sends no more USB output. The USB guard finds this stop after 3 s. The guard then disconnects USB for 0.3 s and connects it again, with no restart. The PC sees a short USB loss, and the output continues. We do not know the cause of the stop yet.

## 11. Verified on the bench, and not verified

Bench Rx boards with STM32s at 100 packets a second, no vehicle, no ODrive, no IMU.

| Measurement | Value |
|---|---|
| 60 min run of rover, monitor and telemetry program, log at 20 Hz | 0 restarts; 71904 of 71911 samples; 0 records lost on the radio and USB |
| Packet time, `main` against 0.5.17 on the same STM32 | Shortest 360 against 522 us; median of the per-second maxima 431 against 886 us; missed STM32 packets 12 against 5 in 155 s |
| Updates over USB, three boards at the same time | About 80 s each; back on USB 2.5 to 2.8 s after the restart |
| USB guard: PC port closed and opened 20 times | 6 output stops, 6 recovered with no restart |
| Start on vehicle power with no USB, USB later | Enumerated and talked, 2 of 2 boards |
| Rover role to monitor role and back | STM32 packets back at 100 a second, with no power cycle |

Not verified: anything on a real rover (steering sign, speed and yaw calibration, track slip, ODrive wheel speed messages, docking), a real MPU6050, the real wire noise, ESP-NOW range in a garden, the 0.5.25 STM32 answer checks and HELLO/BOOT recovery on a board.
