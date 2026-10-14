# MQ Sentinel: Project Controller

`Project.ino` is the integrated V8 momentum-funnel controller for an Arduino
Nano. It combines the tested V8 IR receiver, reaction-wheel, encoder,
MPU-6050 body-gyro, state machine, fault, telemetry, and laser-safety paths in
the main project sketch.

The sketch requires the `IRremote` Arduino library. It uses three independent
38 kHz receiver instances and must be uploaded to a Nano-compatible
ATmega328P board with the wiring below. The MPU-6050 is accessed directly over
I2C using the Arduino `Wire` library; no additional IMU library is required.

## Hardware Connections

| Component | Arduino pin |
| --- | --- |
| Left IR sensor | D2 |
| Centre IR sensor | D3 |
| Right IR sensor | D4 |
| Motor driver IN1 | D6 |
| Motor driver IN2 | D7 |
| Laser diode | D8 |
| Encoder channel A | D9 |
| Encoder channel B | D10 |
| Status LED | D13 |
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |

The IR receivers are active LOW. The motor uses PWM on D6 and a direction
level on D7. The V8 controller keeps the reaction wheel spinning in one
direction and changes its speed to produce both chassis torque directions; it
does not use the electrical brake as a normal steering command.

Mount the MPU-6050 rigidly on the sentinel body, not on the reaction wheel.
The default yaw axis is Z (`GYRO_YAW_AXIS = 2`) and the default sign is
positive (`GYRO_YAW_SIGN = 1.0`). Change these constants after the axis and
direction are verified with the diagnostic sketch. Use a suitable
bidirectional level shifter for Nano 5 V I2C signals unless the exact
breakout board is confirmed to provide level shifting.

## Controller States

1. `STARTUP` gently starts the wheel, learns encoder direction, and enters the
   configured search-speed window.
2. `SEARCH` sweeps continuously until any receiver reports a valid IR frame.
3. `TRACK` applies the current sensor zone as a smooth steering target and
   handles brief receiver dropouts.
4. `CONFIRM` settles an `ALL_THREE` alignment before firing.
5. `FIRE` enables the laser only while all three sensors remain active.
6. `WAIT_CLEAR` holds the wheel state until the target has cleared.
7. `FAULT` turns the laser off and ramps the wheel down if a safety trip fires.

If the target is lost during `FIRE`, the laser is disabled immediately and the
firing dwell must start again. The encoder is used for wheel speed and safety
checks; it is not a chassis-angle or yaw sensor.

The gyro is used in three bounded ways:

- `gyroYawRateDps` adds rate damping to the steering torque, reducing
  overshoot while the body coasts.
- `gyroYawRateDps` must remain below the confirmation threshold for the full
  alignment dwell before firing.
- The laser is immediately disabled if the gyro is unavailable, stale, or
  reports motion above `GYRO_FIRE_RATE_LIMIT_DPS`.

`gyroYawDegrees` is an integrated, bias-corrected, boot-relative angle. It
will drift over time and must not be treated as a compass heading. The
stationary startup calibration must be performed with the sentinel completely
still.

## Important Tuning Constants

The main tuning values are grouped near the top of
[Project.ino](Project.ino), including:

- `IR_HOLD_MS`, `ALIGN_CONFIRM_MS`, and `FIRE_TIME_MS` for receiver hold,
  alignment confirmation, and laser dwell.
- `DRIFT_OUTER`, `DRIFT_INNER`, `DRIFT_CREEP`, and `DRIFT_SEARCH` for steering
  targets.
- `WHEEL_TRACK_MIN_RPM`, `WHEEL_TRACK_MAX_RPM`, and
  `WHEEL_SEARCH_MIN_RPM`/`WHEEL_SEARCH_MAX_RPM` for the speed windows.
- `WHEEL_TRIP_RPM`, `WHEEL_BACKWARDS_TRIP_RPM`, and `ENCODER_STALL_MS` for
  wheel safety faults.
- `GYRO_YAW_AXIS`, `GYRO_YAW_SIGN`, `GYRO_RATE_DAMPING`,
  `GYRO_CONFIRM_RATE_LIMIT_DPS`, and `GYRO_FIRE_RATE_LIMIT_DPS` for body
  motion feedback.
- `TELEMETRY_MS` for periodic serial diagnostics; set it to `0` to disable
  telemetry.

The default serial speed is 115200 baud. Telemetry reports state, sensor zone,
wheel speed, setpoint, drift estimate, PWM, laser state, and fault status.

## Validation

There is no host-side automated test harness for the Arduino sketch. Before
connecting or enabling the laser, use the focused sketches in
[Testing/Documentation.txt](../Testing/Documentation.txt) to verify receiver
polarity, motor direction, encoder direction, and electrical connections.
Run the [MPU-6050 diagnostic procedures](../WedGyroTests/GyroTests/README_TEST_PROCEDURES.md)
before integrating the sensor. Confirm the correct yaw axis, sign, stationary
bias stability, and I2C wiring. Then test startup, target loss during firing,
full target completion, gyro motion aborts, and the fault indicators with the
laser physically disabled or shielded.

## Further gyro improvements

The current retrofit intentionally avoids using integrated yaw as an absolute
target heading. Useful next improvements, after collecting test data, are:

1. Calibrate the angular offset between each IR zone and the laser axis.
2. Use gyro rate and measured yaw change to adapt the carry distance and
   `DRIFT_INNER` instead of relying on fixed values.
3. Add a stationary-period bias monitor that updates gyro bias only when the
   body is demonstrably still; never adapt bias during a turn.
4. Record a gyro-relative heading when a physically verified target lock is
   achieved, then use that landmark to reacquire a known target after dropout.
5. Add a complementary yaw estimate that combines gyro integration with
   confirmed IR landmarks; this can reduce drift but cannot identify
   anonymous identical beacons.
6. Use yaw acceleration and rate to detect mechanical oscillation, string
   release, or a loose sensor mount and enter a conservative recovery mode.
