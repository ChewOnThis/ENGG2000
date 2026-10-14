# MQ Sentinel V8 (refactored sketch)

This folder is a standalone Arduino sketch derived from `Project/Project.ino`.
The original sketch is intentionally untouched.

## Files

- `main.ino` — the only Arduino `setup()`/`loop()` pair; delegates to the controller.
- `SentinelController.h/.cpp` — V8 state machine, telemetry, and laser interlock. It consumes the component APIs below and contains no sensor or motor implementation.
- `MotorController.h/.cpp` — DRV8874 motor pinout and the tested left/right/neutral PWM behavior.
- `Encoder.h/.cpp` — encoder setup, Nano PCINT ISR (with polling fallback), count access, direction sign, and speed-window measurement.
- `Mpu6050Gyro.h/.cpp` — MPU-6050 register transactions, initialization, stationary bias calibration, filtered rate, and integrated yaw.
- `IrSensors.h/.cpp` — IRremote receiver instances, frame hold/zone state, decoding, and the shared Timer2 callback required for multi-receiver operation.

## Build requirements

Open `main.ino` in the Arduino IDE (or build the folder with Arduino CLI) for an
Arduino Nano/ATmega328P. Install **IRremote** (the `IRremote.hpp` API, with
multiple receiver instances enabled) and use the built-in **Wire** library.
The sketch uses pins D2/D3/D4 for IR, D6/D7 for the motor driver, D8 for the
laser, D9/D10 for the encoder, D13 for the status LED, and the Nano I2C pins
A4/A5 for the MPU-6050. Do not add another `setup()` or `loop()` to this folder.

The original V8 constants, state transitions, safety checks, telemetry, and
laser rule are kept in `SentinelController.cpp`; changes should be made there
when tuning behavior.
