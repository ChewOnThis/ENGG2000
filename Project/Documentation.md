# MQ Sentinel: Project Controller

## Scope

This guide describes [Project.ino](Project.ino), the simple reference
implementation in this repository. It is separate from the more advanced
controllers in `Goated/` and the historical experiments in `Testing/`.

The Arduino Nano reads three active-LOW infrared receivers and commands a
two-direction motor. A target detected on the left or right selects a motor
direction; a centre-only detection stops the motor and, after a stable dwell,
fires the laser. An encoder counts motor movement for an optional angle
calculation.

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

The IR sensors are active LOW. The project sketch does not contain an IR
remote enable/disable input, does not use PWM speed control, and does not use
EEPROM logging. `motorStop()` writes HIGH to both motor inputs, which is the
electrical-braking command used by this sketch's motor driver.

## Tunable Constants

| Constant | Value | Purpose |
| --- | ---: | --- |
| `COUNTS_PER_REV` | 700 | Approximate encoder counts per output revolution |
| `SENSOR_HOLD_MS` | 80 ms | Holds a recent active sensor reading |
| `CENTRE_STABLE_MS` | 150 ms | Centre-only dwell before firing |
| `LASER_TIME_MS` | 2000 ms | Laser-on duration |
| `SENSOR_IDLE_RESET_MS` | 1500 ms | Inactivity before the target-complete latch resets |

## Control Flow

1. `setup()` configures the sensor, motor, laser, and encoder pins, brakes the
   motor, turns the laser off, stores the initial encoder-A level, and starts
   Serial at 9600 baud.
2. Each `loop()` samples the encoder and reads the three sensors.
3. Active sensor readings are held for 80 ms to reduce flicker and brief gaps.
4. Centre-only detection brakes the motor and starts the centre timer. After
   150 ms, the laser turns on for up to 2 seconds.
5. Losing centre cancels an active firing cycle and turns the laser off.
6. Outside the centre-only condition, left detection commands `motorLeft()`;
   otherwise right detection commands `motorRight()`; with no side detection,
   the sketch searches left.
7. After 1.5 seconds without sensor activity, the target-complete latch and
   firing state are reset for the next target.

## Encoder

`updateEncoder()` counts rising edges on encoder channel A and samples channel
B to infer direction. `getAngleDegrees()` converts the count using the
approximate 700-count-per-revolution value. The helper is available for future
telemetry, but the project loop does not currently print the angle.

## Limitations and Validation

- The default search command is left, so no-target behaviour is not a neutral
  idle state.
- The centre criterion is centre-only. Overlapping side and centre readings
  do not fire.
- The encoder is sampled in the main loop rather than using an interrupt.
- There is no automated unit-test harness. Use the focused sketches in
  [Testing/Documentation.txt](../Testing/Documentation.txt) for hardware
  checks before enabling the laser.

The advanced EEPROM/logging controllers in `Goated/` use different state
machines, timings, serial commands, and safety assumptions. Do not use this
document as their operating procedure.
