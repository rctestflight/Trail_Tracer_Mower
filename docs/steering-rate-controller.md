# Steering rate controller

The command path is wire-position PID -> existing low-pass filter and manual
steering arbitration -> yaw-rate request (degrees/second) -> gyro rate PID ->
ODrive differential motor mixer. Manual (0), cruise (1), and autonomous (2)
modes all use the rate loop, including reverse driving and zero-throttle pivots.
Manual mode uses only RC steering for the rate request, with no wire-position
PID contribution even when the stick is centered. RC throttle also takes
priority over BTx creep/follow in manual mode.
The rate loop runs at the existing 12 ms motor update interval, independently
of the wire-position PID updates. Fresh STM32 commands are still required.

## USB config.json

The firmware creates the user-editable JSON file on its USB drive. The following
fields are written for new configurations and added to older files on boot
(using the current configuration values). Like existing settings, edits are
validated, persisted in NVS, and applied after the USB write idle window.
The boot upgrade reserializes the configuration and removes comments.

```json
{
  "steering_rate_kp": 0.02,
  "steering_rate_ki": 0.01,
  "steering_rate_kd": 0.0,
  "steering_rate_pid_direction": false,
  "steering_rate_max_dps": 90.0
}
```

These are initial tuning values, not measured mower gains. `kp`, `ki`, `kd`,
and `pid_direction` continue to configure the outer wire-position PID.

- `steering_rate_kp`: normalized motor steering output per deg/s of error.
- `steering_rate_ki`: normalized output per degree of accumulated rate error.
- `steering_rate_kd`: normalized output per deg/s squared; derivative acts on
  measured rate through PID_v1.
- `steering_rate_pid_direction`: `false` = DIRECT, `true` = REVERSE. Select the
  direction for which a positive output change increases measured MPU yaw rate.
- `steering_rate_max_dps`: positive full-scale request, at most 500 deg/s.
  Steering center maps to zero; the configured positive/negative PWM endpoints
  map to +/- this rate, respecting asymmetric PWM spans. Default: 90 deg/s.

Feedback is calibrated `mpu.getGyroZ()` in deg/s. The MPU Z axis must align
with the vehicle yaw axis. Positive requests mean positive gyro Z rotation;
verify the mounting, motor polarity, and PID direction on the vehicle. A
zero request regulates zero yaw rate; it does not hold an absolute heading.

Begin with integral and derivative gains at zero and confirm negative feedback
at a low requested rate. Tune proportional gain first, then add integral gain
to reduce steady rate error. Revisit the wire PID tuning because its former PWM
output now specifies a yaw rate. Reverse driving uses the same gyro rate sign.
The inner loop output and integral are limited to [-1, 1].

Driving is inhibited until stationary MPU calibration completes and a corrected
sample is available. Missing initialization, nonfinite feedback, or feedback older
than 50 ms inhibits motors. This freshness check monitors updates; it cannot
detect every I2C fault that returns plausible data. Existing arm, RC-loss,
command-timeout, charging, stuck, and drive-fault stops remain active. Stops and
live configuration changes reset the rate PID integral before restarting.

## Future angle commands

`steeringRateSetTargetDps(rate_dps)` is the bounded input point in `src/main.cpp`.
For a future relative 45-degree pivot, an outer angle controller can integrate
calibrated gyro rate into relative heading, turn angle error into a decelerating
rate request, and submit it here while commanding neutral throttle. That future
command source must arbitrate with the wire/manual source currently submitted
by `odrive_setDriveOutput()` and retain fresh-command and safety checks.

This change provides the rate loop and its physical-unit input interface; it
does not yet implement an angle-command parser, heading controller, or completed
45-degree pivot maneuver. Gyro-only relative angles will also accumulate drift.
