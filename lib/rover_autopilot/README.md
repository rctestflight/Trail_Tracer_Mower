# rover_autopilot

The rover state machine of the agreed design (2026-10-20), with the wire heading from the approach fit (2026-10-21). It roams inside the perimeter wire and bounces off it, returns to the wire, joins it in the goal direction, follows it to the dock with the vehicle's own wire follower, charges and departs. It has not run on a real rover. It ran in closed loop in the PC emulator of what the ESP32 receives: the PC build of `src/main.cpp` and this library, unchanged, in a simple world (Biot-Savart wire field, coil strengths, the Rx position solve with its quadrant rule, polarity, beacon, skid-steer motion).

## States

| State | What the rover does |
|---|---|
| ROAM | Drives at roam speed; inside the approach distance, at approach speed. At the bounce distance: the fitted heading (probe first when its uncertainty is over the least leave angle), stop, re-acquisition check, pivot (mirror bounce away from the wire, plus a random spread, at least the least leave angle from the wire, the shorter way round). The next bounce arms at the re-arm distance, when the distance falls by the re-arm drop below its highest value since the bounce, or when the fit shows the rover closing on the wire over at least the probe length. A beacon at the beacon bounce distance: stop, pivot 180 deg +- the beacon spread. Polarity says outside: stop, turn round once, no bounce until inside again. A lost signal is normal: drive on. |
| RETURN | Return distance driven since the last charge, or the battery under the return voltage: as ROAM; on entry into the approach distance, JOIN. |
| JOIN | Drive straight to the wire (a lost signal drives on). On the wire (close by the radius, lost right after a close reading, or polarity says outside): stop, turn by the signed fitted heading to point along the wire in the goal direction, FOLLOW. The join also sets the follower's x sign: the wire side on the way in against the Rx x sign, and the sign of y on the wire. Polarity gives inside or outside; without it the join assumes inside or stops (`rover_join_no_pol`). |
| FOLLOW | The wire follower drives on x with the follower sign (set at the join, refined in steady follow in the goal direction). Wrong direction (polarity direction against the goal, or the exit beacon before the entry beacon): stop, turn 180 deg on the wire, follow on. Lost: stop; after `rover_no_tx_s` one 30 deg turn in place out of the field null. Dock contacts: DOCK. |
| DOCK | Holds still while it charges. Charge end: DEPART (follow-only: FOLLOW). A dock reached from ROAM or RETURN goes back to that state. |
| DEPART | Follows a random distance from depart minimum to maximum after the dock contacts open, stops, pivots inward by a random angle, ROAM with the bounce disarmed. |
| FAULT | Stopped until mode 2 is entered again: the stuck detector (only while the signal is not lost), lost longer than `rover_lost_timeout_s` in JOIN, FOLLOW or DEPART, or no polarity at a join with `rover_join_no_pol` stop. |

No Tx signal (the wire bin SNR of both sensors under 10 dB): stop and wait; ROAM and RETURN after `rover_no_tx_roam_s` (default 60 s), the other states after `rover_no_tx_s` (default 2 s). 
## Heading: the approach fit (`approach_fit.h`)

Every straight drive in range feeds two weighted line fits against the distance driven (wheel speeds, else the command):
- distance d = sqrt(R^2 - H^2) from the Rx radius: |sin(heading)| per metre. The radius is the field strength in the coil plane; a coil roll does not change it. It reads low in a small loop (the rest of the loop adds to the strength).
- the lateral wire offset |H x / y|: |tan(heading)| per metre. It reads low under a coil roll (the vertical field leaks into the sideways part); the optional IMU roll correction (`rover_imu_roll`) turns x and y back by the accelerometer roll first.

The heading is the larger of the two (a low heading turns too little at a bounce). Each fit carries a curvature term; when it is significant, the heading is the one at the last frame (a straight drive past a curved wire). Distances are signed by the polarity inside fact when known. Left or right: polarity, else the offset sign. Quality: the heading uncertainty from the residual scatter and the distance span.

Probe: a straight drive of `rover_probe_m` on the same line (forward when the distance to the wire is over the probe length, else reverse); the fit with the probe frames then decides.

No wire angle from one reading (decision 2026-10-21).

## Re-acquisition check

At each bounce and join, the first 10 deg of the planned pivot is the perturbation; turning toward the wire must bring it closer, so a distance change of more than 10 mm the other way reverses the wire side; polarity (inside x direction), when known, decides the side.

## Wiring in main.cpp

`roverStep()` runs once per Rx packet after the wire follower. In mode 2 with `rover_enabled` it replaces `throttle` and `steering_output`, except in FOLLOW and DEPART, where the wire follower's outputs stay. Speed: the wheel speeds (full drive command = roam speed), else the command. Turn measure: the gyro, else the command. The rate loop integral clears at the end of each pivot. Settings: `docs/settings_def.json`, group "Rover mode".
