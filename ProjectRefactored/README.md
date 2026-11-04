# MQ Sentinel V8 (refactored sketch)

This folder is a standalone Arduino sketch derived from `Project/Project.ino`.
The original sketch is intentionally untouched.

## Files

- `main.ino` — the only Arduino `setup()`/`loop()` pair; delegates to the controller.
- `SentinelController.h/.cpp` — V8 state machine, telemetry, and laser interlock. It consumes the component APIs below and contains no sensor or motor implementation.
- `MotorController.h/.cpp` — DRV8874 motor pinout and the tested left/right/neutral PWM behavior.
- `Encoder.h/.cpp` — encoder setup, Nano PCINT ISR (with polling fallback), count access, direction sign, and speed-window measurement.
- `Mpu6050Gyro.h/.cpp` — MPU-6050 register transactions, WHO_AM_I/initialization status, stationary bias calibration, filtered X/Y/Z rates, and integrated X/Y/Z angles.
- `IrSensors.h/.cpp` — IRremote receiver instances, frame hold/zone state, decoding, and the shared Timer2 callback required for multi-receiver operation.

## Build requirements

Open `main/main.ino` in the Arduino IDE (or build the `main` folder with Arduino CLI) for an
Arduino Nano/ATmega328P. Install **IRremote** (the `IRremote.hpp` API, with
multiple receiver instances enabled) and use the built-in **Wire** library.
The sketch uses pins D2/D3/D4 for IR, D6/D7 for the motor driver, D8 for the
laser, D9/D10 for the encoder, D13 for the status LED, and the Nano I2C pins
A4/A5 for the MPU-6050. Do not add another `setup()` or `loop()` to this folder.

The original V8 constants, state transitions, safety checks, telemetry, and
laser rule are kept in `SentinelController.cpp`; changes should be made there
when tuning behavior.

## Success criteria

The project is successful when the assembled Sentinel can autonomously detect,
track, and respond to an IR target while remaining safe and diagnosable:

1. **Build and startup** — `main/main.ino` compiles for an Arduino
   Nano/ATmega328P with the required IRremote and Wire libraries, starts
   without linker errors, and reports its initialization status at 115200 baud.
2. **Sensor detection** — the three IR receivers reliably distinguish a
   sustained target signal from short noise or flicker and identify the
   target's left, centre, and right position.
3. **Reference calibration** — after the first qualified detection, the
   MPU-6050 completes stationary bias calibration and provides valid filtered
   angular-rate and integrated-angle data.
4. **Arc acquisition** — the reaction wheel scans when required and records a
   valid, current IR arc only after the target has been detected continuously
   for the configured verification period and has subsequently cleared.
5. **Closed-loop alignment** — the motor controller uses gyro feedback and
   encoder feedback to move toward the recorded target direction, limits wheel
   speed, settles within the configured alignment tolerance, and avoids
   excessive overshoot.
6. **Response sequence** — the controller fires only when a fresh target arc,
   centre IR confirmation, acceptable body-rate, settled alignment, and safe
   wheel-speed conditions are all satisfied; it then exits the fired region and
   can repeat the cycle without recalibrating unnecessarily.
7. **Fault handling** — MPU, encoder, motor, IR, timeout, or invalid-state
   conditions leave the motor and laser outputs in a safe state and report a
   clear fault over Serial.
8. **Laser interlock** — the laser output remains disabled by default and cannot
   activate unless the explicit compile-time enable is set and every runtime
   alignment and safety interlock passes.
9. **Diagnostics and maintainability** — serial status, `GYRO`, `RECENTER`, and
   motor-test commands provide enough information to verify sensor readings,
   state transitions, alignment, and faults without changing the control logic.
10. **Repeatable hardware test** — during controlled bench testing, the system
    completes repeated detection, alignment, and safe shutdown cycles without
    resets, uncontrolled motor motion, stale-target firing, or loss of encoder
    counts.

## Gyro and IR diagnostics

The normal autonomous setup/loop remains active by default. At 115200 baud,
send a complete line with one of these commands:

- `GYRO` (or `G`) prints WHO_AM_I, initialization/ready state, calibrated X/Y/Z
  rates, integrated X/Y/Z angles, and I2C read failures.
- `RECENTER` (or `R`) starts a passive IR capture. The first and last live IR
  detections define a midpoint in the existing integrated gyro yaw; after the
  IR gap, D13 indicates that the chassis is within the recenter tolerance.
  This is a diagnostic indication only and does not replace autonomous
  steering. Send `R` again to reset the capture, or `NORMAL` to leave the
  diagnostic indication mode.

The gyro angles (including yaw) are relative to startup calibration and will
drift over time; they are not an absolute compass heading. The diagnostic LED
uses D13, not D6: D6 is reserved for motor IN1.
