# rover_autopilot

Draft autopilot for the mower: it bounces inside the perimeter wire, goes home on low battery, follows the wire to the dock in one direction, charges, and departs. It has not run on hardware. Everything below was verified in closed loop against rover-sim (Documentation repo, `rover-sim/src/core`, commit b6b1ae4) with the harness in `test/sim_harness`.

## Behaviour

| State | What the rover does |
|---|---|
| ROAM | Drives straight. At 0.5 m from the wire it stops and pivots away (mirror bounce plus a random spread). |
| SEEK | Battery low: like ROAM, but within 1.1 m of the wire it changes to ACQUIRE. |
| ACQUIRE | Approaches to 0.6 m, stops, pivots onto the wire at 30 deg in the dock direction, drives straight until the wire is under the sensor. |
| FOLLOW | PD on the wire's crossing point X (P) and its rate dX/dt (D). After 0.5 m locked on the wire it goes fast and stays fast until the wire is lost. Holds still while docked. |
| DEPART | After charging: follows 1 m, pivots 60 to 140 deg inward, ROAM. |
| HALTED | Wire lost for `rover_lost_timeout_s`: outputs off until mode 2 is entered again. |

The direction along the wire comes from two facts: the rover is always inside the perimeter, and the wire is always below the sensor. When ACQUIRE stops near the wire, the side of the wire gives the dock direction. FOLLOW keeps that direction by continuity. The transmitter's phase is never used.

## Exit beacon guard

A beacon transmitter (BTx) lies on the wire just past the dock exit. Travelling the dock direction, the rover meets the dock first. Travelling the wrong way, it meets the beacon first. In FOLLOW, the exit beacon before the dock means the wrong way. The rover then stops, pivots 180 deg toward the inside, and follows on.

- Place the exit beacon on the wire, at least the distance from the charge contacts to the sensor plus 0.3 m past the point where the contacts touch. In rover-sim, 1.0 m past the pad worked.
- The guard arms only after 1.5 m of FOLLOW (`Config::guardArm`), so a rover that joins the wire between dock and beacon going the right way does not turn around. Keep `guardArm` larger than the dock-to-beacon distance.
- With an entry beacon as well (`rover_entry_btx`), an exit beacon seen within 3 m after the entry beacon is ignored. This covers a dock that did not register a charge (charger off). Without an entry beacon, the rover then turns around and enters the dead dock from the wrong side.
- Blind stretch: a wrong-way join less than `guardArm` before the beacon, or between beacon and dock, is not caught (about 2.5 m of wire).

## Wiring in main.cpp

`roverStep()` runs once per valid STM32 packet. In drive mode 2 with `rover_enabled` it replaces `throttle` and `steering_output`; the existing ODrive skid mixing, fault cut-offs and charge hold stay in place. The stuck detector is skipped in rover mode, because the rover is off the wire most of the time.

- Speed input: the slewed command (no odometry).
- Yaw rate input: the autopilot's own slewed yaw-rate command (no gyro, design choice 2026-10-08). Only its magnitude is used (pivot angles). The MPU6050, if fitted, is logged only.
- Steering sign: taken from `pid_direction`. DIRECT means a positive ODrive steer command turns left, as the wire follower already does.
- Units: STM32 `s1_x`, `s1_y` and `s1_radius` in mm are converted to m.

## config.json keys

| Key | Default | Meaning |
|---|---|---|
| rover_enabled | false | Mode 2 runs the autopilot instead of the wire follower |
| rover_goal_cw | true | Dock direction along the wire, seen from above |
| rover_sensor_height_mm | 150 | Sensor centre above the wire |
| rover_full_speed_mps | 1.0 | Ground speed at full throttle command (measure it) |
| rover_full_yaw_rate_dps | 115 | Not used from firmware 0.5.61: `maxYawRate` is `steering_rate_max_dps`, the full scale of the gyro rate loop |
| rover_seek_vbatt | 0 | Go home below this battery voltage, filtered (0 = off) |
| rover_seek_after_s | 600 | Go home this long after the last charge (0 = off) |
| rover_entry_btx | 1 | BTx channel before the dock: creep at 0.2 m/s (0 = none) |
| rover_exit_btx | 2 | BTx channel after the dock: wrong-way guard (0 = none) |
| rover_lost_timeout_s | 30 | Stop for good after this long without the wire (0 = never) |

All other tuning (speeds, distances, gains) is in `Config` in `rover_autopilot.h`, with rover-sim's values.

## Differences from rover-sim

| Change | Why |
|---|---|
| Exit beacon guard | Wrong-way protection (new). |
| A charge in any state leads to DEPART | The rover then sits in the dock on the wire, facing the dock direction. rover-sim acts on a charge only in FOLLOW. |
| `filterReadings` (on) | Bounce and transition decisions use a low-passed distance and an axis-averaged bearing (cos 2θ, sin 2θ), which keeps the wire's side through the noise flips near the sensor's horizon. Off reproduces rover-sim. |
| Distance from the firmware's x and R | The vertical field is √2·H·\|x\|/R² in rover-sim's units, so `coil_data` is not needed. |
| HALTED on lost timeout | Safety on an escape. |
| No beacon bounce in ROAM | The beacons mark the dock (entry creep, exit guard), not a keep-out zone. |

## Verification (rover-sim, no hardware)

rover-sim b6b1ae4 (2026-10-08 23:4x): follow D term on dX/dt, the speed lock held until the wire is lost, pivots the shorter way, battery range 300 m. Each mission run is 450 s of simulated time, 100 Hz packets, float build, one autopilot call per packet, and the commanded speed used as the measured speed. "Battery" deaths come from rover-sim's seek margin of 1.2 wire lengths.

| Check | Sim time | Result |
|---|---|---|
| `filterReadings` off, double build, compared step by step with rover-sim's own autopilot | 8 runs x 400 s | 6 runs identical throughout. 1 run: the deliberate charge-in-ACQUIRE change at 302 s. 1 run: rounding (1e-9 m at 302 s), then a battery death on a different packet at 344 s. |
| Defaults, exit beacon, no noise, circle and garden tracks | 3.0 h | 23 charges, 0 turnarounds, 0 wrong-way, 1 battery |
| Same, half of rover-sim's noise | 3.0 h | 22 charges, 1 turnaround, 0 wrong-way, 2 battery |
| Same, full noise, roam speed 0.7 m/s | 3.0 h | 16 charges, 1 turnaround, 1 wrong-way, 0 battery |
| Every transition forced the wrong way, no noise, seek margin 2.5 | 3.0 h | 23 turnarounds, 23 charges, 1 wrong-way |

Yaw rate without a gyro (design choice, rover-sim c3ef53d): the firmware gives the autopilot its own slewed yaw-rate command as the measured yaw rate (harness `wModel`). Track slip makes the real pivot smaller or larger than this belief; the harness scales the belief. 12 runs x 450 s each (1.5 h):

| Belief / real yaw rate | Defaults, exit beacon, no noise | Every transition forced the wrong way |
|---|---|---|
| 1.0 | 12 charges, 0 wrong-way | 12 turnarounds, 12 charges, 0 wrong-way |
| 1.43 (real pivot 70 % of the command) | 12 charges, 0 wrong-way | 11 turnarounds, 11 charges, 1 wrong-way |
| 0.8 (real pivot 125 % of the command) | 12 charges, 1 wrong-way | 11 turnarounds, 12 charges, 2 wrong-way |

With the belief equal to the real rate, the half-noise and full-noise batches (3.0 h each) give the same counts as the runs with the true yaw rate: 22 charges, 1 turnaround, 2 battery; and 16 charges, 1 turnaround, 1 wrong-way.

Speed cap `rover_speed_limit_mps` 0.5 (the firmware default) against no cap, every transition forced the wrong way, command-based yaw rate, 24 runs x 1500 s (10 h) each: no cap 120 joins, 110 turnarounds, 97 charges, 5 wrong-way (4.2 %); cap 0.5 m/s 53 joins, 46 turnarounds, 45 charges, 4 wrong-way (7.5 %). The difference is within chance at these counts. The cap halves the joins because rover-sim drains the battery per metre.

Earlier results (rover-sim c1c6401, D term on asin(dX/ds), battery range 100 m, so about 4 times the charges): 98 charges with no noise; 96 turnarounds and 80 charges when forced the wrong way; 13 wrong-way entries and 0 charges with the guard off; noise A/B 55 failures against 4 for `filterReadings`.

## Not verified

- Hardware: steering sign, speed and yaw-rate calibration (`rover_full_speed_mps`, `rover_full_yaw_rate_dps`), ODrive response, real track slip in a pivot.
- The real noise level. rover-sim's noise model (noise equals the wire's signal at 1.5 m) is an estimate; at that level escapes stay at 2 to 8 per hour and the lost timeout is the safety net.
- Tracks with sharp corners or narrow necks beyond rover-sim's library.
