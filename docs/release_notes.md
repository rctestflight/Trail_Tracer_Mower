# Rover release notes

Newest first. Each release gives what changed for the user, the settings that the user must set or check, and what
is not tested. The full change history for developers is in `include/firmware_version.h`.

## 0.6.35

### Changes

- **Every change from follow to roam uses the wire leave angle.** This applies at the end of a timed follow and after
  a charge. The rover turns to the wire leave angle (default 90°), plus or minus the bounce spread, and never less than
  the bounce minimum leave angle from the wire.
- The setting `rover_follow_exit_deg` now has the label "Wire leave angle". Its value stays.
- The inward turn minimum and maximum (`rover_turn_min_deg`, `rover_turn_max_deg`) are no longer used.
- **Fix: repeated bounces near the wire.** After a turn, the rover bounced again after 0.5 m of drive, often while it
  still drove away from the wire. Now, after any turn, the bounce arms again only when the rover closes on the wire:
  the wire radius passes its highest value since the turn and then falls by the re-arm radius fall.
  - New setting: re-arm radius fall `rover_rearm_fall_mm`, default 30 mm.
  - The re-arm distance `rover_rearm_m` is no longer used.
  - The re-arm drop `rover_rearm_drop_m` now sets only how often the rover turns round outside the loop.
- **The bounce distance comes from the wire radius.** The lateral offset reads high near the wire, so the distance
  from it could read 0.25 m at any point inside about 0.5 m. A wire depth or height setting error of 120 mm now moves
  the bounce point by less than 6 cm.

### Settings to set or check

- None. The new setting works at its default.

### Not tested

- PC emulator only. Bounce re-arm: 648 missions of 10 min, 6 track shapes, wire depth up to 120 mm off the height
  setting and a depth that changes round the loop. False bounces 2254 -> 246; the longest chain of false bounces
  7 -> 3. Crossings of the wire in roam rose from 83 to 189, most of them with a sensor at the rover centre and
  shallow leave angles, and in a 2.5 m wide loop.

## 0.6.34

### Changes

- **Fix: the rover does nothing after the button press.** At the start, the rover could lock on the wrong Tx
  frequency until the next restart. It then saw no wire signal and waited. It now keeps the right frequency.
- **The turn at the end of a timed follow uses the bounce settings.** The rover leaves the wire at the follow end leave
  angle, plus or minus the bounce spread. The angle to the wire is never less than the bounce minimum leave angle.
- New setting: follow end leave angle `rover_follow_exit_deg`, default 90° (straight into the loop).
- The inward turn minimum and maximum now apply only when the rover leaves the charger.
- **Button presses in the log.** Each press gives one event with its action: drive mode auto, cruise on or off, centre
  calibration, steering direction, settings erase (start, cancelled, done), or no action.
- **USB in the log.** USB connect and disconnect, and the USB state at each start.
- **Sensor frequency and calibration in the log** at each start of the Rx STM32.

### Settings to set or check

- None. All changes work with the defaults.

### Not tested

- The frequency fix was tested on a bench board: the old firmware locked on the wrong frequency when the start command
  had to be sent again; the new firmware kept the right frequency at 3 of 3 starts. Not tested on a rover.
- The follow end turn was tested in the PC emulator only: 35 turns in 80 min, all from 65° to 115°, no escape.
- Button events: each action needs one real press. The USB and calibration events were read on a bench board.

## 0.6.33

### Changes

- **Settings in the log.**
  - At each start and every 10 minutes, the log records every setting. The settings go as a series of setting dump
    events, about 3 s long.
  - Each setting change gives one log event with the old and the new value. This applies to all sources: the app,
    the USB port, config.json, the button, and the DSP values that the Rx STM32 reports.
  - The fleet key (`cmd_key`) and the Wi-Fi password show only "set" or "not set". Their values never go into the log.
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

- The setting dump and the change events were tested on a bench board only, through the PC telemetry station: one
  complete dump (149 settings) 5.4 s after the start, and one change event for each USB setting change. The repeat
  every 10 minutes was not tested.
- The timed cycle was tested in the PC emulator only, on its 12 m circle: roam 60 s, follow 30 s,
  turn inward, roam again; 4 charges in 40 min with no escape; roam time 0 gives roam only.
