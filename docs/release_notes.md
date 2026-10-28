# Rover release notes

Newest first. Each release gives what changed for the user, the settings that the user must set or check, and what
is not tested. The full change history for developers is in `include/firmware_version.h`.

## 0.7.3

This release follows 0.7.2. It adds one change only.

### Changes

- **Square-on guard in every roam mode.** When the rover drives straight at the wire close to square-on, the wire
  signal fades and the rover can misjudge its distance to the wire. A Bounce return could then miss its join and
  cross the wire. Now, near the wire on a square-on approach, the rover drives one gentle turn toward the goal
  direction, without a stop, and meets the wire at the Approach angle. The turn is 90 deg minus the Approach angle:
  45 deg with the default. The guard works in Bounce, Reverse bounce and Join then leave, in roam and in return. The
  log shows each guard turn as a square-on decision.

### Settings to set or check

- None. No new setting. The guard turn uses the Approach angle setting (group Rover: Join then leave) in every
  roam mode; its help text says so.

### Not tested

- On a real rover. The guard is tested in the PC simulation of a skid steer only: the 0.7.2 runs (8 m round loop,
  12 x 10 m ellipse, rough lawn), 60 min each, and the square-on approach of the 0.7.1 field run in every mode, roam
  and return. All these runs used Approach angle 30 deg (guard turn 60 deg), not the default 45.

### Update

- Stop the rover before an update. An update during a run can fail with the page message "ESP32 firmware
  refused: begin No Error". Stop the run and install again.

## 0.7.2

This release follows 0.7.1.

### Changes

- **New setting: Turn radius.** Default 0.5 m. In Join then leave and Reverse bounce, the rover never turns
  tighter than this radius. A larger radius slips the wheels less and spares the lawn. 0: the rover turns on the
  spot, as in 0.7.1.
- **New Roam mode: Reverse bounce.** The rover makes the same bounce decision as in Bounce. At the bounce distance
  it stops, drives the other way and sweeps round to the depart angle. It does not turn on the spot.
- **Join then leave: sweeping turns.** The join, the end of the follow, the depart, the turn-back and the return
  are driven turns at the turn radius or wider. The rover plans the join from the measured wire distance and
  direction. The 0.7.1 turn on the spot at the join is gone.
- Outside the loop, Join then leave and Reverse bounce turn back by a change of drive direction.
- A follow that loses the wire in these two modes backs up 0.25 m after 1 s.
- **Bounce and Reverse bounce: probe on entry.** When the rover approaches the wire steeper than the depart angle
  limit, it drives a gentle 15 deg arc on the way in, without a stop. The known turn helps the wire measurement, and
  the rover keeps out of the square-on spot where the wire signal fades. The stop, turn and back-off after the
  bounce point is gone.
- **Depart angle spread: default 90 deg (was 25).** A wider spread covers the lawn more evenly. With 25 the rover can
  repeat the same paths on an oval loop and leave one end unmown.
- New log value: rover_dec_solve_sensor, the sensor that the wire solve used at a decision.
- The rover settings show in three groups: every roam mode, Bounce and Reverse bounce, Join then leave.

### Settings to set or check

- Roam mode: Bounce stays the default. Choose Reverse bounce or Join then leave for sweeping turns.
- Turn radius: check it against the space at the wire.
- Depart angle spread: a saved value stays as it is. Set 90, or remove the saved value, for the new default.
- Reverse bounce uses sensor 2 only with "Use sensor 2 for control" on.
- Sensor 2 at the back: with Sensor 2 and Use sensor 2 for control both on, sensor 2 leads when the rover drives
  in reverse. Sensor 2 must face the same way as sensor 1. With either setting off, sensor 1 leads both ways.

### Not tested

- On a real rover: neither sweeping mode. The sweeping turns are tested in the PC simulation of a skid steer only:
  an 8 m round loop and a 12 x 10 m ellipse, clockwise goal, 3 starts, polarity on and off, 60 min each.

### Update

- Stop the rover before an update. An update during a run can fail with the page message "ESP32 firmware
  refused: begin No Error". Stop the run and install again.

## 0.7.1

This release follows 0.7.0.

### Changes

- **Join then leave: every vehicle turns on the spot at the wire.** 0.7.0 marked a vehicle as a car when it turned
  less than 5 deg in 1 s of the turn command, and kept the mark until power-off. A slow turn on grass can take
  almost 1 s, so a skid steer could get the car mark by mistake. 0.7.1 removes this check.
- A car is not supported in this build.

### Settings to set or check

- None. The settings are the same as in 0.7.0.

### Update

- Stop the rover before an update. An update during a run can fail with the page message "ESP32 firmware
  refused: begin No Error". Stop the run and install again.

## 0.7.0

This release follows 0.6.36. It also holds the 0.6.37 changes below.

### Changes

- **Two roam modes in one build.** The new setting "Roam mode" (`rover_bounce_style`, group "Rover: both roam
  modes") chooses how the rover turns back at the wire. Change it in the setup page, the telemetry app or
  config.json.
  - **Bounce** (the default): As it drives, the rover measures the distance and the direction of the wire. At the
    bounce distance it stops, turns on the spot and drives back into the loop at the mirror of the wire angle.
  - **Join then leave**: At the wire the rover turns onto it and follows it for the short follow time. A skid
    steer stops and turns on the spot by the measured wire angle; a car turns on an arc. The rover then drives an
    arc back into the loop at the depart angle.
- **Settings in three groups**: "Rover: both roam modes", "Rover: Bounce mode only" and "Rover: Join then leave
  only". The help text of each setting starts with the mode or modes that use it.
- **Older names load.** In config.json, `estimator` and `pivot` load as Bounce, and `basics` loads as Join then
  leave. The rover writes the new names.
- **Return distance is gone.** A charge falls due by battery voltage only.
- **Update page: both boards in one step.** One upload sends the sensor board file and the rover image together. If
  one part fails, neither board changes.
- **Removed settings** (used by neither mode): `rover_rearm_m`, `rover_join_pivot_m`, `rover_turn_min_deg`,
  `rover_turn_max_deg`. An older config.json with them loads with no error.

### Settings to set or check

Saved values stay in charge: the update changes no saved value. Two settings need a check:

- **Depart angle limit** (`rover_bounce_min_deg`): the least angle between the wire and the depart heading; the most
  is 180 deg minus it. The rover takes 90 at most, and at 90 every depart leaves square-on. We recommend 65.
- **Depart angle spread** (`rover_bounce_spread_deg`): a random angle from minus to plus this value is added to each
  planned depart angle. With the limit at 65 and the target at 90, the departs can only fall from 65 to 115 deg. A
  spread of 90 puts 36 % of the departs at each limit; 180 puts 43 % at each limit. 0.7.2 changes the advice: a
  wider spread covers the lawn more evenly (see 0.7.2).

### Other fixes

- **Outside the loop, the rover crosses back in.** Before, a rover outside the wire could turn back every few
  seconds short of the wire and stay outside for minutes. Now it drives on to the wire and crosses it.
- **Join then leave on a skid steer** no longer joins on an arc. An arc join of a rover that met the wire against
  the goal direction could end over the wire.

### Not tested

- On a real rover: neither mode. The roam behaviour is tested in the PC simulation of a skid steer only: 6 loops
  from 8 m to 20 m across, 2 goal directions, 3 starts, with and without the inside/outside reading, 10 min each.
  - Bounce: 144 runs with the defaults and with a field set of saved values (spread 90): no escape, no fault.
  - Join then leave: 144 runs with the defaults and 72 with the same field set: no escape, no fault.
- The update: bench board, update page, 0.6.41 to 0.7.0 and 0.6.36 to 0.7.0, 2 of 2 each; settings kept, sensor
  board running after each. One of 10 updates from 0.7.0 gave no page answer for over 6 min, but it
  installed. With no answer, wait 10 min, then reload the page; never cut the power.

## 0.6.37

### Changes

- **No event during a turn on the spot**. While the rover turns on the spot, and until it stops
  turning, it takes no decision: no bounce, join, turn-back, wrong-direction turn, follow end, return, stuck or lost
  fault. This covers every turn on the spot: bounce, beacon, turn-back, join, follow end and the turn after a charge.
  It replaces the 0.6.36 stop after a turn. A turn on the spot is a commanded turn, or a measured turn about a point
  within 0.15 m of the rover centre: in such a turn the inner wheel stands still or turns backward.
- **Wi-Fi test over USB** for the setup page: `wifi test [seconds]` joins the stored network and looks for the
  telemetry server, with no save and no restart. It works in either radio mode and gives two results: network joined
  (signal, address), and telemetry server found (address).

### Settings to set or check

- None.

### Not tested

- The turn rule: PC emulator only. Sensor 1 ahead of the wheels (the emulator default), 4 missions of 15 min with
  60 s roam and follow: no outside crossing and no turn-back, as 0.6.36. The rover log case of 2026-10-24 (the bounce
  after each follow end) is not reproduced in the emulator: it needs the sensor position on that rover.
- Wi-Fi test: bench board, test access point with the telemetry program: join and server found 3 of 3 in ESP-NOW
  mode (about 2 s), and in Wi-Fi mode. Network absent: "not found" after 15 s.

## 0.6.36

### Changes

- **Fix: the rover starts the follow far from the wire.** A short false "outside" polarity reading 1 m from the wire made
  the join start the follow there: the rover turned in a circle for the follow time. The same reading stopped some starts
  with the fault "polarity says outside". Now the inside/outside value changes side only when the readings show a
  crossing, and a crossing needs the sensor near the wire. Near the wire one clear reading changes the side; far from
  the wire it takes 3 clear readings in a row.
  - New setting: wire extra depth, maximum `wire_max_depth_mm`, default 120 mm: the most the wire lies deeper than the
    sensor height setting anywhere round the loop.
  - New log value `wire_inside_pct`: how sure the rover is that it is inside the loop, 0 to 100 %.
- **Fix: a second bounce right after a turn.** In a pivot the sensor swings round the rover centre, so its distance to
  the wire changes with no drive. After turns over about 90 deg, the swing back read as closing on the wire, and the
  rover bounced again 0.3 s after the turn, often out over the wire. Now the rover stops turning at the end of each
  pivot before anything can act, and the bounce re-arm starts from that stopped reading.
- **Fix: new settings missing from config.json.** After an update, config.json kept its old list of settings: new
  settings, such as the roam time and the follow time, showed in the app and the USB list but not in the file. Now, at
  the first start after an update, the rover writes config.json again with every setting. The values in the file stay;
  the new settings show at their defaults.
- **Sensor 2: two settings.**
  - "Sensor 2" (`enable_sensor_2`): on, the sensor board reads both sensors, and the rover shows and logs sensor 2 data.
    Off (default), the board reads sensor 1 only, for the best sensor 1 signal.
  - New: "Use sensor 2 for control" (`use_sensor_2`, default off). Only with both on do the inside/outside value, the
    heading round the loop and the follow direction use the sensor nearest the wire.
  - The sensor board settings `dsp_sensors` and `dsp_lanes` are no longer settings: "Sensor 2" sets them. An older
    config.json with them loads with no error.

### Settings to set or check

- `rover_sensor_height_mm` must be the sensor height above the wire. If parts of the wire lie more than 120 mm deeper,
  set `wire_max_depth_mm` to that depth.
- **Sensor 2 data now needs "Sensor 2" on.** Before, the sensor board read sensor 2 with `enable_sensor_2` off, and
  the log had its spectrum and signal level. To keep sensor 2 in the log, set `enable_sensor_2` to true.
- Set "Use sensor 2 for control" on only when sensor 2 faces the same way as sensor 1 and has the same sign.

### Not tested

- Replay of 2 rover logs (2026-10-24): side changes far from the wire 62 -> 0; 22 of 26 real crossings found within
  0.4 s, the other 4 were bad readings or over the wire. PC emulator: see the next update of these notes.

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
