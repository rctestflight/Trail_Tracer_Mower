# Rover release notes

Newest first. Each release gives what changed for the user, the settings that the user must set or check, and what
is not tested. The full change history for developers is in `include/firmware_version.h`.

## 0.6.33

### Changes

- **Settings in the log.**
  - At each start and every 10 minutes, the log records every setting. The settings go as a series of setting dump
    events, about 3 s long.
  - Each setting change gives one log event with the old and the new value. This applies to all sources: the app,
    the USB port, config.json, the button, and the DSP values that the Rx STM32 reports.
  - The fleet key (`cmd_key`) and the Wi-Fi password show only "set" or "not set". Their values never go into the log.
  - To read the settings from a log file: `python tools/rover_log.py settings <log file>`.
- **Timed roam and follow** (Rover mode).
  - The rover roams for the roam time. Then it finds the wire, joins it and follows it for the follow time. Then it
    stops, turns inward by a random angle (inward turn minimum to maximum) and roams again.
  - A charge return comes first. After a charge, both times start again.
  - The roam time does not count while the rover waits for the Tx signal. The follow time does not count while the
    wire is lost.
  - New settings: roam time `rover_roam_time_s`, default 60 s; follow time `rover_follow_time_s`, default 30 s.
  - A new rover decision: 16, follow time end.
- **Sensor 2 wire radius in the log** (`s2_radius_mm`). The log already had the sensor 2 position and the sensor 2
  beacon radii. All sensor 2 values are in the log only when sensor 2 is on.
- **More free RAM.** Buffers for bench and diagnostic reports now use memory only while in use. This frees 11 KB of
  static RAM for later features. There is no change for the user.

### Settings to set or check

- **The timed cycle is on by default.** A rover that gets this update starts the cycle with no setting change. To keep
  the previous behaviour (roam until a charge return), set the roam time or the follow time to 0.
- **Sensor 2 data.** Do these steps:
  1. Fit sensor 2. Sensor 2 always listens to Tx channel 1.
  2. Stop the rover. Set `enable_sensor_2` to true (app, USB `set enable_sensor_2 1`, or config.json). The rover
     refuses this change while it moves. The change applies at once, with no restart.
  3. Save the settings, so that the value stays after a restart.
  4. Make sure that "Sensors on the board" (`dsp_sensors`) is 2 and "Sensor lanes" (`dsp_lanes`) is 2. These are
     the defaults. With `dsp_sensors` 1, the sensor board reads only one sensor.
  5. Make sure that sensor 2 has the same orientation and sign as sensor 1. With sensor 2 on, the inside/outside
     polarity uses the sensor that is nearer to the wire. A sensor 2 with the opposite sign gives the wrong side.

### Not tested

- The setting dump and the change events were not tested on a rover. The main build is good. A bench or field log
  from this release is the first test: `python tools/rover_log.py settings <log file>` must show one complete dump
  at the start.
- The timed cycle was tested in the PC emulator only, on its 12 m circle: roam 60 s, follow 30 s,
  turn inward, roam again; 4 charges in 40 min with no escape; roam time 0 gives roam only.
