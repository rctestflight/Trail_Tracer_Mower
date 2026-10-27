// Firmware version of the Rx ESP32 (USB banner, "boot", telemetry, update checks) and the change history,
// newest first. A new version: change ROVER_FW_VERSION and the bench build names below, add one history line.
#pragma once

// Change history
//   0.7.2: sweeping turns. New setting Turn radius (default 0.5 m; 0 = turn on the spot). New Roam mode Reverse
//     bounce: the Bounce solve and depart decision; at the bounce distance the rover stops, changes its drive
//     direction and sweeps to the depart heading. Join then leave: the join, the follow end, the depart, the
//     turn-back and the return are driven arcs at the turn radius or wider; the join is planned from the wire solve
//     (one arc on, or an arc away and an arc on). Outside the loop both modes turn back by a change of drive
//     direction. Bounce keeps its turn on the spot. With Use sensor 2 for control on, sensor 2 (at the back) leads when the rover
//     drives in reverse; it feeds every roam reading and the solve, with no distance between the sensors assumed.
//     The arc's turn rate follows the measured speed, with a learned ratio for the shortfall of the steering rate
//     loop at part commands. A follow lost in a sweeping mode backs up 0.25 m after 1 s. New log value
//     rover_dec_solve_sensor. Settings groups: every roam mode, Bounce and Reverse bounce, Join then leave;
//     settings definition version 17.
//     Bounce and Reverse bounce: the probe on entry. On an approach steeper than the depart angle limit, the rover
//     drives a 15 deg arc inside the approach distance, without a stop: the solve takes the known heading change, and
//     the rover leaves the square-on null. The stop, 15 deg turn and back-off after the bounce point is removed, and
//     with it the direction profile wait at the bounce (emulator, 60 min runs: bounces steeper than 75 deg 15 of 262).
//     Depart angle spread default 90 deg (was 25): 25 repeated paths on an oval loop (86-91 % of the area in 60 min,
//     emulator); 90 visited 97-99 %.
//   0.7.1: Join then leave: every vehicle joins by a stop and a turn on the spot. The car check is removed (1 s of
//     the turn command with under 5 deg of turn marked the vehicle a car until power-off; one slow pivot on grass
//     took 0.84 s in field logs). A car is not supported in this build.
//   0.7.0: two roam modes for the field test. The Roam mode setting (stored key and values of the
//     bounce style) chooses Bounce (the default; the wire position solve measures the wire's distance and direction,
//     the rover stops and turns on the spot) or Join then leave (join at the approach angle, short follow, depart on
//     an arc). The rover settings show in three groups: both roam modes, Bounce mode only, Join then leave only.
//     Depart angle spread default 25 deg (90 put 72 % of departs at the limits). The solve runs in a background task
//     with its memory from the heap only during a Bounce mode run (no RAM change outside it); the bounce decision
//     comes at the stop, with the solve's direction profile (perturb when it is over the depart angle limit).
//     The bounce log carries the solve (rover_dec_solve_deg, rover_dec_solve_mm). Return distance is gone: a charge
//     falls due by battery voltage only. Older choice names in a config.json (estimator, basics, pivot) still load.
//     Join then leave fixes (emulator, the reference skid steer, follow inset 0): the join distance is at least sensor
//     height + wire extra depth (nearer, a square-on arrival reads the wire lost: 14 escapes in 132 runs before); a
//     skid steer's join radius is the join speed over the turn rate, not the radius measured at the faster first
//     join; the join ends once the rover no longer closes, with no wire side check. The three-point turn stays (a car).
//     Outside the loop (both modes): a depart that ends outside counts as the turn toward the wire, and no distance
//     rise counts in the square-on null band (a false rise there turned the rover back 6 cm short of the wire every
//     4.6 s; emulator: 178 turn-backs in 144 runs before, 0 after).
//     Join then leave on a skid steer: the join stops and turns on the spot by the wire solve's angle (the Bounce mode
//     join); a vehicle that does not turn at zero speed in 1 s is a car and joins on the arc. An arc join of an
//     arrival against the goal direction turned through square-on near the wire and could end over it (emulator: 3
//     escapes in 72 runs with spread 90 before, 0 after; no reverse legs).
//   0.6.41: settings cleanup (two bounce modes). The bounce
//     style setting is the bounce mode: basics or estimator (the pivot bounce, same stored values). Removed, used by
//     neither mode: rover_rearm_m, rover_join_pivot_m, rover_turn_min_deg, rover_turn_max_deg. Each rover setting's
//     help says which mode uses it; the depart settings read as the depart angle target, spread and limit.
//   0.6.40: update page pair order. In one form post the Rx STM32 file goes to flash and waits; the ESP32 image goes
//     to its slot and waits; the Rx STM32 update runs only after both arrive, and the restart only after it passes.
//     A failed part changes neither: the Rx STM32 file is not used, or the boot slot goes back to the running image.
//     Before, a failed ESP32 part after the Rx STM32 update left a new Rx STM32 with the old ESP32. The parts can
//     come in either order.
//   0.6.39: bounce style basics fixes (emulator, the reference rover and 8 m loop). The square-on guard acts only within
//     15 deg of square-on, where the field null loses the wire; at the 45 deg approach angle it fired near parallel
//     and made each short follow 25-30 s. Near the follow line the follow steers fully to parallel. The roam join
//     needs the side reading to fall as well as the radius (in a small loop the radius falls on the way to the
//     middle). The bounce follow time starts only at the follow line.
//   0.6.38: bounce style basics (the basics loop of the PC simulation), new setting
//     rover_bounce_style, default basics. At the wire the rover no longer stops and turns on the spot. It joins
//     toward the inside at the approach angle, and starts the turn early enough that its turning circle (approach
//     speed and full steering rate) fits before the follow line. It follows the wire for the new bounce follow time
//     (3 s) at the new follow inset (0.30 m inside), with its closing speed limited so it never meets the wire
//     square-on. Once parallel it drives an arc back into the loop by the wire leave angle, the bounce spread and
//     minimum, planned at the join. The return to the wire uses the same join. Lost signal in the join or follow:
//     stop and wait. Approach angle default 45 deg (was 30); a stored value is kept. pivot keeps the 0.6.37 bounce.
//   0.6.37: the pivot rule (no event of any kind on pivot turns). While the rover turns on the
//     spot (a commanded pivot, or a measured one: yaw rate over 0.25 rad/s about a point within 0.15 m of the rover
//     centre), and until a pivot that has ended stops turning, no decision or fault triggers. It covers every pivot
//     (bounce, beacon, turn-back, join, wrong-direction reverse, follow-end and depart turn, lost search) and replaces
//     the 0.6.36 stop after a pivot. The side filter still reads in a turn. USB "wifi test [seconds]": join the stored
//     network and find the telemetry server, two results with progress lines, in either radio mode (setup page).
//   0.6.36: inside/outside from the new inside filter, not the held polarity. Every raw sign
//     read (Rx STM32 items 9 and 10) goes into a belief, weighted by its spread; a side change needs a crossing, and
//     a crossing needs the sensor near the wire (height setting + the new setting wire_max_depth_mm, 120 mm, and the
//     roam speed). Field case 2026-10-24: "outside" bursts at 1 m from the wire started the follow away from the
//     wire and faulted starts. New log value wire_inside_pct (the belief).
//     config.json: a setting that the file does not hold (new in an update) makes the file be written again at the
//     start, with its values kept: every setting shows in config.json (field case 2026-10-24: no roam and follow time
//     in the file after the 0.6.33 update). Before, only a "save" added new keys.
//     Pivot settle: after each pivot that goes on to roam (bounce, turn-back, beacon, follow end)
//     the rover stops turning before any rule acts; the bounce re-arm starts from the stopped reading. Field case
//     2026-10-24: the sensor swing-back after turns of 65-115 deg armed a bounce 0.3 s after the turn.
//     Sensor 2 as two settings: enable_sensor_2 ("Sensor 2") sets the Rx STM32 sensor count
//     (CID 38: 2 or 1) and lanes (CID 13: 0x02 or 0x00) and the sensor 2 data; new use_sensor_2 ("Use sensor 2 for
//     control") lets sensor 2 act on the nearest-sensor choice (worldFacts, follow_dir). dsp_sensors and dsp_lanes
//     are F_LEGACY.
//   0.6.35: each change from follow to roam (the end of a timed follow, and after a charge) turns to the wire leave
//     angle (rover_follow_exit_deg, label "Wire leave angle"), plus the bounce spread, at least the bounce minimum.
//     The inward turn minimum and maximum are not used.
//     Bounce re-arm: after any turn the bounce arms when the 0.3 s radius falls by the new
//     re-arm radius fall (rover_rearm_fall_mm, 30 mm) below its peak since the turn; the 0.5 m drive rule, the
//     re-arm distance and the approach-fit closing test go. The bounce distance comes from the radius and the height
//     setting, not the offset x. rover_rearm_drop_m now sets only the outside turn-back repeat.
//   0.6.34: the turn at a follow end (timed roam and follow) leaves the wire at the new follow end leave angle
//     (rover_follow_exit_deg, 90 deg = straight into the loop), plus the bounce spread, kept at least the bounce
//     minimum from the wire, as at a bounce. Before, it used the inward turn (60 to 140 deg), which ignored the bounce
//     settings. The inward turn now applies only after a charge.
//     Fix: a retry of the Rx STM32 run command (no answer in 35 ms) went out as the wire channel command and set
//     the sensor 1 wire bin to 1 (55.3 kHz): signal lost and "Tx off" with a strong wire signal until a restart
//     (field case 2026-10-23). The retry now sends the run again.
//     Log: each button press and its action (event 23 button); USB connect and disconnect (event 20 usb, a 7 and 8,
//     and the state after the start); the wire bin and calibration of each sensor at each Rx STM32 start (event 15
//     stm32, a 2).
//   0.6.33: settings in the log. At the start and each 10 min, every "list" line goes into the log as setting dump
//     events (21 settings, about 3 s of parts); each change from any source (app, USB, config.json, the button, the
//     DSP values the Rx STM32 reports) gives one event (22 setting_change) "key old -> new". The fleet key and the
//     Wi-Fi password give only "set". Sensor 2 wire radius in the log (s2_radius_mm, with enable_sensor_2 on).
//     Rover mode: timed roam and follow (rover_roam_time_s 60 s, rover_follow_time_s 30 s; 0 = off): roam for the
//     roam time, then find the wire, join it and follow it for the follow time, then stop, turn inward by a random
//     angle and roam again. A charge return comes first. New decision 16 follow time end.
//     Static RAM: 11 KB more free (bench, "list saved" and Monitor Rx report buffers now come from the heap only
//     while in use; the static region had about 100 B free).
//   0.6.32: new defaults for the true gyro rate (7bd3d9e): steering rate kp 0.01 (was 0.005), bounce spread 90
//     deg (was 45), bounce minimum 65 deg (was 35), roam stop after no Tx signal 20 s (was 60). A value saved on the
//     rover stays. include/settings_table.h is the one source of defaults: a build no longer stops when
//     docs/settings_def.json has another default; it writes the table's default into that file.
//     The start-check fault (polarity says outside, the Tx direction setting the wrong way) has its own halted text;
//     before, it read "fault: wire lost too long".
//   0.6.31: the gyro and accelerometer scales follow the ranges that the IMU reports after the start. A range write
//     that the chip dropped no longer changes the turn angle. The imu event b gives the gyro range in use (deg/s).
//   0.6.30: each setting that config.json changes (an edit on the USB drive, or at the start a file that differs from
//     NVS) gives one USB line "SETTING changed" and one log event (setting, a 0, b 2), "config.json: <key> <old> ->
//     <new>". Before, a value inside its range changed with no trace in the log. IMU: the gyro and accelerometer ranges
//     are written again after the wake and read back into the imu event ("gyro reg 0x08 acc reg 0x00 id 0x68" wanted;
//     b 500, the gyro range the library uses). A rover nose-up test read the yaw gyro 2.3 x the gravity angle.
//   0.6.29: "stm raw" works again: since 0.5.128 the send paths held back the idle mode and CID 42 commands of the
//     request, so no stream started and every later Rx STM32 command waited until a restart. A request with no
//     answer in 2 s now ends with a refusal. After the CID 42 frame no byte goes to the Rx STM32 (one byte at the
//     answer frame edge stopped the stream: 0 packets). The coils of one sensor only: mask 1, 2, 3, 4, 8, 12 (four
//     coils need 95 % of the stream baud; the Rx STM32 dropped half the packets). The PGA hold ("stm cmd 29") and a
//     "bin" frequency stay through the DSP set after each stream (it returned them to the settings). tools/rx_usb.py: samples, polarity and bins over USB for the PC.
//   0.6.28: rover self-audit fixes. No side check in the bounce pivot (a 10 mm radius change flipped the side on noise).
//     Pivots count the net gyro angle (a sum of |rate| counted noise as turning: short turns). Stopped under 0.25 rad/s
//     (0.05 rad/s never read stopped on a driving rover). The next bounce arms after 0.5 m of straight drive since a
//     turn. The start probe drives forward only, and only beyond the probe length. Telemetry: the last decision
//     (rover_dec_*: number, kind, wire side, fitted heading, planned turn, distance).
//     Tx direction default clockwise (tx_dir_ch* 1, was 0 = off: polarity never ran). Wrong way: fault at the start
//     (polarity says outside before the first turn; rover_fault 4), not a silent inversion.
//   0.6.27: settings: a number out of range takes the nearest limit (was: the default), with a note "limit". Rover ranges
//     widened to what the code and the geometry allow (rover_bounce_min_deg up to 90, spreads up to 180, distances and
//     speeds for large loops and vehicles). The settings definition takes every range from the firmware table: the
//     app, the settings page and the programmer keep no range of their own. Settings definition version 14.
//   0.6.26: the approach fit restarts only on a commanded turn, not on gyro readings over 0.05 rad/s: a real rover in
//     straight drive reads several deg/s, so the fit never formed and every bounce took the wire as square-on.
//   0.6.25: the Rx describes itself: the build puts its telemetry_def.json and settings_def.json, gzipped, into the
//     image (tools/gen_def_files_build.py); "defget <tlm|set> <offset>" sends them in chunks over USB, Wi-Fi and
//     ESP-NOW (docs/espnow_protocol.md, "Definition files"), so the telemetry app needs no update for a new build.
//   0.6.24: rover wire heading from the approach fit (lib/rover_autopilot/approach_fit.h): every straight drive in
//     range fits the distance from the Rx radius and the lateral wire offset against the distance driven; the larger
//     heading counts, the local one when the fit bends. No wire angle from one reading. Probe (rover_probe_m): a
//     straight drive at the start in range, and before a bounce or join when the fit is weak. Join by the signed
//     heading (closing or moving away). The bounce re-arms while the rover closes on the wire. Polarity says outside
//     in roam: turn round. Wrong direction in follow: pivot 90 deg to the inside, leave the wire, turn round, join
//     again. Follow on x as the Rx sends it, with the sign of y. Optional IMU roll correction (rover_imu_roll). Least
//     leave angle 35 deg; stuck detector 15 mm / 18 s. Join: drive to the wire, turn along it in the goal direction, follow; the join sets the follower x sign
//     from the known wire side. Rx regain wait 0 (rx_regain_wait_ms default 0). Autonomous mode is rover mode: rover_mode rover (default),
//     follow only, or position follower (expert). Removed: rover_enabled, rover_follow_only, rover_full_speed_mps,
//     rover_full_yaw_rate_dps, rover_seek_after_s, rover_speed_limit_mps.
//   0.6.23: rover state machine of the agreed design (lib/rover_autopilot): roam, return, join, follow, dock, depart,
//     fault. Readings from x, y and radius as the Rx sends them: lateral wire offset, distance from the vertical field
//     part, wire angle. Re-acquisition check at each bounce and join (the first 10 deg of the pivot; polarity decides
//     when known). Join by the approach angle in the goal direction; polarity gives inside or outside, else the setting
//     rover_join_no_pol. Follow and depart with the wire follower (in rover mode it steers on the lateral wire offset).
//     Wrong-direction check: polarity direction or the exit beacon before the entry beacon, then a 180 deg pivot. Lost
//     in join or follow, or Tx off in any state: stop and wait. Stuck detector (signal not lost) or lost longer than
//     rover_lost_timeout_s in join, follow or depart: fault. The rate loop integral clears at the end of each pivot.
//     Settings: rover_follow_only, rover_join_no_pol, rover_roam_speed_mps, rover_bounce_m, rover_bounce_spread_deg,
//     rover_bounce_min_deg, rover_rearm_m, rover_rearm_drop_m, rover_beacon_bounce, rover_beacon_m,
//     rover_beacon_rearm_m, rover_beacon_spread_deg, rover_approach_m, rover_approach_pct, rover_join_pivot_m,
//     rover_join_angle_deg, rover_return_m, rover_wrong_dir_check, rover_depart_min_m, rover_depart_max_m,
//     rover_turn_min_deg, rover_turn_max_deg. Retired (older config.json only): rover_full_speed_mps,
//     rover_full_yaw_rate_dps, rover_seek_after_s, rover_speed_limit_mps. Autopilot states in telemetry: 0 roam,
//     1 return, 2 join, 3 follow, 4 depart, 5 fault, 6 dock.
//   0.6.21: polarity per sensor and Tx channel from Rx STM32 2.2.x (ext id 12, every frame): table values
//     dt_polarity_s1_ch1..dt_polarity_s2_ch5, wire_inside and wire_heading (inside or outside, clockwise or
//     counterclockwise); follow_dir from the polarity when the channel's current direction is set. Settings tx_dir_ch1..5
//     (current direction of each Tx channel), steer_frame (vehicle or world: world signs x by the heading round the
//     loop, so a vehicle on the wire the wrong way turns round), dt_spread_limit_deg (CID 54 index 14),
//     rx_regain_wait_ms (CID 24), rx_weak_mm_0..7, rx_lost_mm_0..7, rx_regain_mm_0..7 (CID 21 to 23, sent at each DSP
//     set). Extension items are no longer read as stale at random (their time stamp could run one tick ahead of the
//     clock: field sign, spread and follow_dir values dropped out). Bench commands "stm pol" and "stm ext <id>". The
//     Upload button stops when another program holds the port. The ODrive arming switch clears faults.
//   0.6.19: the update page works again in update mode: the web server stops only for a download the board fetches
//     ("ota url"), not during a page upload (it waited for the upload handler: an RTC watchdog reset or "no answer
//     from the main loop")
//   0.6.18: src/main.cpp holds the vehicle code only; the board code is in src/platform/ behind include/rx_platform.h
//     (no change of behaviour)
//   0.6.17: during an app update download the web server and mDNS stop until the restart (more free memory for Wi-Fi)
//   0.6.16: Wi-Fi sweep record: every frame's sweep values get in at 10 ms with up to 7 sweep bins; "log info" prints
//     the sweep record counters
//   0.6.15: "list saved": each setting with the value of the next start beside the active value; "dld" turns the RTC
//     watchdog off first; update mode waits 1 ms each pass (the update page upload needs CPU time); no telemetry
//     datagrams during an update page upload (more free memory for Wi-Fi)
//   0.6.14: no USB reconnect in the vehicle role: a host that held the port open and read nothing could lose the board
//     (both chips reset); undrained output is dropped and counted
//   0.6.13: settings dt_side_a_ch1..5 (Side A of each Tx channel inside or outside the perimeter); table value
//     follow_dir (1 Hz): the direction round the perimeter in follow mode, from the followed channel's sign and its
//     Side A setting
//   0.6.12: table value dt_decoded (1 Hz): the Tx channels the Rx STM32 decodes now, read each 1 s (CID 50 index 0)
//   0.6.11: setting dt_channels (255 auto, else a CH1..CH5 bit mask, 0 off; live): the Tx channels the Rx STM32 decodes
//     for polarity, sent as CID 50 at each DSP set; dual tone polarity detection items of Rx STM32 2.1.134+ (extension
//     items 9 sign and 10 sign angle spread, per sensor and Tx channel) in "stm ext" and the telemetry table (10 Hz,
//     group Polarity detection)
//   0.6.10: the USB serial number comes from the chip MAC (the board files gave every board "0": Windows saw one disk
//     serial on every drive, disk event 158 at each restart)
//   0.6.9: the USB drive gets one disk signature and volume serial per board, from the chip MAC (every board formatted
//     after an erase showed the PC the same disk: Windows disk event 158, then USB losses at a detach); written once
//     per board
//   0.6.8: the Monitor Rx reads the Rx STM32 input in bounded blocks (its byte loop never ended on the first Monitor Rx
//     start after a USB load: no line after the banner, port lost 5 s later)
//   0.6.7: the Monitor Rx starts the RTC watchdog too (8 s; a stopped Monitor Rx stayed off USB until a power cycle)
//   0.6.6: the Monitor Rx radio starts 300 ms after enumeration again (the 0.6.4 change did not stop the USB loss); the
//     Monitor Rx keeps the loop() step running and its start time over a restart and prints them in the next "prev"
//     line (step=, step_ms=, in_write=)
//   0.6.5: GET /update/status counts failed heap allocations since boot, with the size, caps and task of the last
//     (alloc_fail*)
//   0.6.4: the Monitor Rx radio starts 1.5 s after the PC opens the port (or 6 s after the start with no PC), not 300
//     ms after enumeration, which fell on the PC's port open (USB stopped in about 1 start in 10)
//   0.6.3: the upload heap trace adds the least free heap since boot at the request, at the end and after the answer
//     (up_bootmin_*)
//   0.6.2: GET /update/status gives the free heap over the last upload (up_heap_start, up_heap_min, up_maxblk_min,
//     up_heap_end)
//   0.6.1: "set dsp_frame_period_us" sends the period again when the setting is unchanged but the period in effect
//     differs by more than 1 % (after a direct CID 18 write the 10 ms set did not take); the first "rt" after boot
//     shows the period in effect (it showed 10000); 0.6.0 (Rx STM32 2.1.0 pair; a comms change starts a minor version):
//     the 144 B Rx STM32 frame (version 3) beside 128 B, the Rx UART frame; an unknown Rx STM32 link learns layout 2
//     from 128 B or 144 B frames (0.5.128 took 128 B only: after a restart into 2.0.81 no frame was read); "stm restart
//     force" and "stm url" send command 41 with no answer awaited when frames come but are not read, so a wrong-frame
//     app never blocks its own replacement
//   0.5.128: Rx STM32 frame version 3 (144 B, 2.0.81+) beside version 2 (128 B), told apart by the version byte; the 4
//     extension slot items are kept per item id and index with their age ("stm ext"); no Rx UART task: loop() builds
//     the frames after each good Rx STM32 frame and the edge interrupt sends them (the task stack cost the page update
//     3.6 KB of heap)
//   0.5.127: bench "uart jitter <max_us>": each Rx UART frame goes a random 0..max_us after its edge (proof that the Rx
//     STM32 takes frames at any timing); "uart" shows edges, timing counts and the Rx STM32 frame errors
//   0.5.126: the Rx UART frame goes from the frame edge interrupt into the transmit FIFO (the task now only builds the
//     next frame): a task woken by the edge started more than 1 ms late about 1 edge in 300; "uart" shows edge
//     none/fifo counts
//   0.5.125: the Rx UART frame goes from its own task (priority 22) at each Rx STM32 frame edge, not from loop(): a
//     late loop() pass sent it late (CRC fails) or skipped a frame (block in 86.6 % of frames at 2.96 ms); loop()
//     leaves the next command in a pending slot; "uart" shows task late/wait counts
//   0.5.124: "uart" counts the good frames that get no control block, by reason (catch-up, backlog)
//   0.5.123: the frame to the Rx STM32 goes right after the frame read, before the control step (sent after it, a 16 B
//     frame crossed the Rx STM32 frame tick: bad CRC and length counts)
//   0.5.122: Rx UART frame (Rx STM32 with status bit 27): every frame carries the control block (version 1, angle =
//     vehicle roll from the accelerometer, flags: bit 0 charge enable) and one optional command, one CRC (16 B); the
//     charge enable goes as flag bit 0 (no CID 9); older Rx STM32 apps get the 10 B command frame as before; bench
//     "uart" (roll imu|sweep|<deg>, bad crc|ver, pause, off|on)
//   0.5.121: raw stream: the UART receive buffer fits the largest free heap block (16 KB down to 4 KB; in Wi-Fi mode 16
//     KB failed with no message and the stream gave 0 bytes), the page server stops during the stream, a failed start
//     says so; "RAW start" gives rxbuf
//   0.5.120: Rx STM32 2.0.67 polarity detector stream (status bit 26, CID 52 on): frame result 2 is the detector block,
//     not coil data (lane 2 reads zero); the block goes to the log as it comes (body 0x37: frame seq, block header,
//     samples; Wi-Fi every sample); "stm polar" counters; tools/rover_log.py polar writes the samples as CSV
//   0.5.119: after a page upload stops part way, a USB "ota begin" works with no restart (the Update writer is closed
//     first)
//   0.5.117: settings and telemetry pages answer a short busy during an update; telemetry page /telemetry on the web
//     server (src/web_telemetry.cpp, lib/tlm_page): the status record, the newest log sample and every table value, by
//     name, with label, unit, decimals and group from the telemetry table at run time; GET /api/telemetry (short set or
//     ?all=1) is a copy that loop() takes on request, formatted by the server task; ?def=1 gives the field list; the
//     page polls at 1 Hz (2 Hz as an option) while its tab is visible
//   0.5.116: telemetry table: the telemetry table describes every log record (TLR record lines, TLF field lines: name,
//     type, offset, scale, unit, decimals, kind, "no value" code, group, label; the fixed record bytes do not change);
//     telemetry_def.json format 2 lists every record, "version" = the table hash (compile time, over every table line);
//     "get telemetry_def" answers it; tools/rover_log.py decodes every record from the file; one telemetry table
//     (include/telemetry_table.h, user lines in include/user_telemetry.h): one line defines a logged value (name, wire
//     type, scale, unit, decimals, rate 1 Hz, 10 Hz or the log rate, group, label); tlm_set(TLM_<name>, value) sets it;
//     one record per rate (bodies 0x34 1 Hz on stream 1, 0x35 10 Hz and 0x36 log rate on stream 0) with the table hash,
//     the sample count and the rate; the build writes telemetry_def.json (tools/gen_telemetry_def.py); "tlm" answers
//     the table hash; tools/rover_log.py csv decodes the records; the fixed records do not change; web server in Wi-Fi
//     mode (src/web_server.cpp): GET / lists the pages; update page /update (src/web_update.cpp): a browser form post
//     with the Rx STM32 file and the ESP32 image, the same writers, checks and refusals as "stm url" and "ota url";
//     "stm url" also checks the Rx STM32 file header (magic, product, header CRC-32); bench USB "moving on|off";
//     settings page /settings on the web server: every setting from the settings table, built from the table at run
//     time; set, save, restart and "config erase" through the radio command rules (src/web_settings.cpp)
//   0.5.115: one settings table (include/settings_table.h) drives defaults, range check, NVS, config.json, set, get and
//     list; a bad value takes its default with a "SETTING bad value" line; "config erase [wifi]"; erase gesture: the 0
//     button pressed in the first 3 s after power-on and held 5 s erases every setting and the Wi-Fi details; USB "btn0
//     down|up" and "btn0 window" fire the button handler on the bench
//   0.5.114: update commands over the radio (Wi-Fi or ESP-NOW CMD frame) and USB, ACK text = the USB line: "ota url",
//     "ota status" (state=idle|download|done|failed got size, error=), "ota abort", "stm url" (Rx STM32 update from an
//     HTTP server: download into the tlog partition, SHA-256, command 41, the ESP32 feeds the boot checker from flash,
//     3 tries), "stm status", "get rx_stm32_version" (CID 30); radio commands also in update mode; "ota url": ESP32
//     update over Wi-Fi from an HTTP server (lib/usb_ota)
//   0.5.113: the Monitor Rx holds its output for 100 ms after the PC opens the port (a Linux PC echoed board output
//     back in front of the first command)
//   0.5.112: USB driver: no bus reset on IN endpoint status bit 15 (it cleared the USB device address; the Monitor Rx,
//     and rarely the vehicle role, then gave no USB answer until a power cycle)
//   0.5.111: charge lease renewal every 250 ms or every second frame, whichever is longer
//   0.5.110: charge enable as a 250 ms lease (CID 9 again while charging is wanted); Rx STM32 BOOT retried every 30 s
//     after the first 3
//   0.5.109: the Monitor Rx radio start moves from setup() to loop() (0.5.108 waited in setup(): task watchdog reset at
//     each Monitor Rx start)
//   0.5.108: the Monitor Rx role starts its radio after the PC has configured USB (a radio start during the USB start
//     left USB dead about 1 in 100 boots)
//   0.5.106: the USB class driver copy is back to 0.5.103 (no stack counters, no USB event 6; Monitor Rx USB stop
//     2026-10-15)
//   0.5.105: shortest update period 2938 us in every link mode ("Update period too short: 2.94 ms or longer")
//   0.5.104: USB events (output stop, guard reconnect, host port open and closed, no input for 60 s); update period
//     floor = the shortest the Rx STM32 runs (2354 us, 2336 us with the slow ADC off); "set dsp_frame_period_us"
//     answers the period in effect; status record 84 B with the period in effect; "bin" keeps the frequency in effect
//     (Rx STM32 2.0.58 reads); bench "pwm" pulse check
//   0.5.103: "bin <index> <hz>" (CID 20 pass-through, no save) on USB and radio, "sweep" on USB; sweep record body
//     0x33: the frame seq, page tag and bin values as they come, the PC places them on the grid (was 0x32 with grid
//     points); shortest update period per link mode (2000 us in each mode: no link, ESP-NOW, Wi-Fi): a shorter "set" is
//     refused
//   0.5.102: bin record input at 100 Hz (it took every frame); frame decode in IRAM
//   0.5.101: services at set rates (USB 250 Hz; link watch, USB drive, telemetry service, bench input 100 Hz)
//   0.5.100: the radio part runs at most at 250 Hz (it ran on every pass)
//   0.5.99: end-of-pass wait up to 5 ms (was 1 ms)
//   0.5.98: loop() waits for the next frame edge (at most 1 ms) when nothing waits, so the CPU load is the real work
//   0.5.97: CPU load (busy, loop, drive %) in the HB line and the status record (bytes 77-79); HB packets = good frames
//     per second (control = control passes); every decoded frame has its Rx STM32 answer read (catch-up and decode-only
//     frames lost their answers since 0.5.85)
//   0.5.96: control at most 100 Hz (frames between decoded only); sweep records capped (Wi-Fi 40/s, ESP-NOW 4/s);
//     refusals read "Stop the vehicle first"
//   0.5.95: save, wifi set and wifi clear refused while driving (run model: saves only while stopped); USB output
//     emptied before a save
//   0.5.94: RTC watchdog: a board that stops for 8 s resets itself and keeps its PREV and PANIC records
//   0.5.93: bench "tasks": FreeRTOS tasks by name, priority, least free stack
//   0.5.92: a panic leaves its reason, PC and backtrace in RTC memory, printed at the next boot and on "boot" (no flash
//     write); the resync search of 0.5.88-0.5.90 is out (it found no frame on real data)
//   0.5.91: v2 Rx STM32: no periodic read-back, no resend after a frame gap or a read-back difference (a setting goes
//     only on a change, a user command or an Rx STM32 start); bench "rt": commands, writes, retries per window
//   0.5.90: resync search back with its window in static RAM
//   0.5.89: the 0.5.88 resync search is off (it panicked at 2.35 ms with cut frames)
//   0.5.88: layout 2 resync: after a bad CRC the next good frame is searched in the bytes in hand (a cut frame no
//     longer costs the frames after it); USB output stop in the vehicle role: counted, and a USB reconnect once a host
//     sent a command this boot
//   0.5.87: bin sweep record (body 0x32: each sweep reading with its grid point; Wi-Fi every update, ESP-NOW every
//     10th), setting dsp_sweep_bins (default auto), USB and radio "sweep <index> <value>" (CID 43 pass-through), frame
//     header of Rx STM32 2.0.54, a USB line per full DSP set
//   0.5.86: commands over Wi-Fi: CMD, STOP, RESTART, REQ from the server as datagrams, ACK back to it (same frames as
//     ESP-NOW)
//   0.5.85: layout 2: older waiting frames are decoded without the control step (no frame lost at 2.35 ms frames;
//     control on the newest)
//   0.5.84: bench "rt": mean time per pass of each loop part
//   0.5.83: CID 3 read goes to the Rx STM32 (2.0.52+: flash state; it was held), bench "stm cmd 3" save goes out
//   0.5.82: no repeating flash writes: the log is RAM only (flash ring removed), boot_id random (no NVS boot counter),
//     settings to NVS at boot only after a config.json change, no Rx STM32 CID 3 save at each boot or Rx STM32 start,
//     Wi-Fi driver settings in RAM; bench "nvs": NVS entry counts
//   0.5.81: bench "rt": passes over the frame period, with a log write apart, the others by their biggest part
//   0.5.80: "wifi scan" while associated no longer fails at 2.4 s (the library timeout; the scan takes longer on a
//     connected station); USB input stops a raw stream early (RAW end stop=usb)
//   0.5.79: USB "wifi blast": Wi-Fi throughput test (datagrams of a set size and rate to the server address)
//   0.5.78: the Wi-Fi key is calculated from the passphrase before the drive loop (the Wi-Fi driver stopped loop()
//     190-417 ms at each Wi-Fi start)
//   0.5.77: USB "save" no longer overflows the loop() stack (panic restart, 4 of 8 saves); Wi-Fi mode sends log frames
//     only while the server is known; full "wifi" status line; bench "stack"
//   0.5.76: USB CDC class copy with a 4096 B transmit FIFO and 1024 B transfers (was 64 B: 132 kB/s); nothing to the Rx
//     STM32 during a raw stream (TX held high at the baud switch, silence recovery held)
//   0.5.75: raw usb writes each block in pieces (the USB buffer is under 512 B)
//   0.5.74: stm raw: Rx STM32 raw stream (command 42) end to end to USB
//   0.5.73b: raw tests up to 5 Mbaud
//   0.5.73: raw stream pass-through and rate tests (raw usb, raw pass, raw loop)
//   0.5.72: log sample coil values signed (were |v|)
//   0.5.71: radio_mode wifi: telemetry as UDP over a Wi-Fi network (lib/wifi_telem, the 2026-10-12 work, ported)
//   0.5.70: "set" takes text settings (vehicle_name, drive_output) as the rest of the line
//   0.5.69: settings from a review: drive_output (servos as V1.0, or CAN; auto), servo map by function, vehicle_name,
//     4S limit, steer_lpf_hz, charge time as it is
//   0.5.68: the 8-bin coil record (0x2E) goes through the log anti-alias filter at telemetry_bins_hz (default 5)
//   0.5.67: USB driver copy: TX FIFO register of the FIFO in use (DIEPTXF[TxFNum-1]; the CDC output FIFO pointed
//     outside the FIFO RAM: output stops at Monitor Rx start and mid-run), IN endpoints disabled through the core at a
//     bus reset and a close; a guard reconnect ejects the drive first; usbregs with FIFO registers and driver counters,
//     the stop record printed 3 times
//   0.5.66: telemetry body 0x2E, the coil magnitudes of all 8 bins of both sensors, 5 a second
//   0.5.65: "stm restart" (Rx STM32 command 41, app 2.0.39+): Rx STM32 update with no power cycle
//   0.5.64: USB output fix from the bench line (0.5.60-dev10 to dev13): project copy of the TinyUSB ESP32-S2 device
//     driver (lib/tinyusb_dcd_fix) with no zero-length packet on data IN endpoints; USB register record at each output
//     stop and over a restart (monitor). 0.5.63: after a reset that is not a power-on, only STM32 UART bytes count as a
//     running app (the boot checker also makes nDATA_READY edges), so an STM32 waiting in its boot checker gets
//     HELLO/BOOT
//   0.5.62: USB optional in the vehicle role (the USB guard never reconnects; undrained output dropped), monitor
//     reconnect only after the PC program connected
//   0.5.61: the autopilot full yaw rate is steering_rate_max_dps (pivot angles as planned through the gyro rate loop)
//   0.5.60: 0.5.59 + bench tools (radio slot, usbload, rt bench, stm stream, monitor promisc/quiet, usbstall diag);
//     table CRC, fast beacon pow; monitor PC-gone 10 s; USB flush kick at 250 ms

#if defined(OTA_TEST_VERSION)
#define ROVER_FW_VERSION OTA_TEST_VERSION  // OTA test images (envs ota_a, ota_b, ota_test_crash)
#elif defined(BENCH_FAKE_IMU)
#define ROVER_FW_VERSION "0.7.2+fakeimu"  // bench build: static IMU values (status flag bit 6)
#elif defined(SETTINGS_NEWKEY)
#define ROVER_FW_VERSION "0.7.2-newkey"  // bench build: one more table line (test_new_setting)
#elif defined(TELEMETRY_NEWVALUE)
#define ROVER_FW_VERSION "0.7.2-newvalue"  // bench build: the example lines of include/user_telemetry.h
#else
#define ROVER_FW_VERSION "0.7.2"
#endif
#define RX_BOARD "RX_REVA"  // the hardware this build targets: Trail Tracer Rx board, revision A
