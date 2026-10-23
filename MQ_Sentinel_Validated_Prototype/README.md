# MQ Sentinel — Reference-Safe IR/Gyro Survey and Diagnostics

**Status:** Sensor-only configuration is ready for Arduino IDE compilation and bench validation. Motor and encoder experiments are included but **disabled by default**. This is **not** an autonomous reaction-wheel aiming controller: without a motor, the satellite must be rotated by hand; without an absolute heading reference, no gyro-only program can promise target positions persist across restart or severe gyro disturbances.

## What is in the project

- `MQ_Sentinel_Validated_Prototype.ino` — open this file in Arduino IDE (the sketch needs the other files in the same folder).
- `SentinelFirmware.cpp` — all implementation and user settings at the top.
- `SentinelFirmware.h` — interfaces to the Arduino entry point.
- `TEST_PLAN.md` — the exact experiments and logs requested.
- `RESULTS_TEMPLATE.csv` — an optional tracking worksheet.

## Required hardware / wiring

| Component | Nano pin | Notes |
| --- | --- | --- |
| Left IR receiver OUT | D2 | Active LOW 38 kHz TSSP-style detector |
| Centre IR receiver OUT | D3 | The default *primary* receiver whose arc centre becomes a target bearing |
| Right IR receiver OUT | D4 | Active LOW; individually measured for comparison |
| MPU-6050 SDA | A4 | I²C, normally 0x68 (AD0 LOW) |
| MPU-6050 SCL | A5 | I²C |
| MPU-6050 power/GND | correct rated rail, common GND | Many breakouts have regulators, bare chips do not. Check your board. |
| Status LED | D13 | Built-in LED; solid when mapping complete, blinking on reference loss |
| Laser control (optional) | D8 | **OFF by default**. Use a suitable driver, never feed a bare laser diode directly from a Nano output. |
| DRV8874 IN1 / IN2 (later) | D6 PWM / D7 | Only when `ENABLE_MOTOR_TESTS=1`. Motor must have separately safe power supply and shutdown. |
| Wheel encoder A/B (later) | D9 / D10 | Only when `ENABLE_MOTOR_TESTS=1`, 700 counts per revolution is provisional. |

**One IR receiver on D2 instead?** Change `#define IR_SENSOR_COUNT 1` and `#define PRIMARY_SENSOR 0` in `SentinelFirmware.cpp`. If all three are connected, keep defaults (3, 1). A regular TV-remote receiver with short demodulated packets may not pass the continuous-carrier occupancy filter; this sketch assumes the active-LOW TSSP detector described in the original code.

## Build and Serial Monitor

1. Open `MQ_Sentinel_Validated_Prototype.ino` in Arduino IDE. Keep `.h` and `.cpp` in the same sketch folder.
2. Board: Arduino Nano; Processor: choose your Nano's bootloader variant; Port: its COM port.
3. Upload. Open Serial Monitor at **115200 baud**, **Newline**.
4. Enter `HELP` to see supported commands, `STATUS` to see state, `MAP` to see target data.

## First trigger and startup-inside-cone behavior

1. On power-up, outputs remain OFF; MPU-6050 initialises; no target map is trusted.
2. The sketch waits for IR to pass its 16-sample occupancy filter and **65 ms additional confirmation**. A single glitch or pulse does not trigger calibration.
3. On the **first accepted IR detection**, it prints `CAL_START`; hold the suspended satellite stationary for approximately **2.2 seconds** to estimate gyro Z bias. If noise or acceleration exceeds limits, it prints `CAL_REJECT` and retries.
4. When `CAL_OK` appears, gyro yaw is defined as **0° at that instant**. This does **not** mean it is pointing at the geometric centre of the beacon; it is merely the start reference.
5. The original, possibly incomplete IR cone is deliberately **ignored**. Rotate the satellite until all IR receivers have cleared for at least 200 ms. The sketch prints `SURVEY_START` and only then begins accepting full arcs.
6. With the beacon fixed at ~1 metre, rotate **slowly in one direction** through a full physical revolution. The primary receiver records its first complete qualified ON→OFF arc, centre = mean(entry and exit angles), width = absolute difference. The scan is not complete until the gyro reports >365° of net travel and the last arc has closed.
7. `SURVEY_END,status=READY` appears once the requested number of distinct arcs has been found. If insufficient beacons were seen, it prints `INCOMPLETE`. The map exists **in RAM only**; rebooting, `RESET`, or major jolt invalidates it.

**Target count:** `TARGETS 1` for your current single beacon (default); `TARGETS 3` only once you have **three separate fixed beacons at distinguishable bearings**. Targets are indexed by detection order, not by unique ID: identical 38 kHz beacons cannot be identified individually without a separate modulation/protocol.

**Centre sensor slit:** Filter confirmation requires that an IR cone remain visible for roughly 80–100 ms. Rotate slowly enough that the centre slit receives enough samples. Start around 3–6 degrees per second. Adjust verified time thresholds based on measured flicker logs, not guesses. For directional accuracy, do not physically move the beacon during an arc sweep: the gyroscope measures only **satellite** rotation.

## Software states

```
WAIT_TRIGGER -> CALIBRATING -> WAIT_CLEAR -> SURVEYING -> MAP_READY
        \             \              \               \          \
         \------------- CAL_REJECT     \------> MAP_INCOMPLETE

Any trustworthy-reference failure -> REFERENCE_LOST -> explicit RESET
MAP_READY -> VERIFY -> repeated complete arcs -> VERIFY STOP -> MAP_READY
```

## Failure and limits

- Gyro heading is obtained by integrating angular velocity; drift accumulates. A large acceleration (>1.65g or <0.45g) or near-saturation rate (>=235 deg/s) flags reference invalid and discards all target angles. **Not all errors or jolts are detectable**. A stall, cable twist, unexpected rotation during calibration, or constant slow movement may bias the result without raising a fault.
- No EEPROM writes are performed: stale angles after power loss are not considered valid. Storing a gyro bias or a map **does not preserve an absolute world angle** because the body might have moved while unpowered.
- A 360° scan is reported when **integrated gyro yaw** passes 360°, not independently verified physical angle. Use a fixed external mark with `MARK` and `CHECK 360` to check gyro scale and drift.
- The IR sensor is a binary detector, **not** a precise measurement of signal direction or intensity. The midpoint assumption is a repeatable empirical alignment method, not a guarantee of laser-centre accuracy.
- Laser output is **disabled in the default build**. If enabled and a safely driven eye-safe module is connected, it allows only a short manual `ARM LASER` then `LASER 100` pulse when stationary, within 2° of Target 1, and IR is verified; it never automatically fires.
- This firmware does not control reaction-wheel yaw autonomously. Future velocity control requires validated wheel encoder, motor direction, gyro-axis orientation, inertia, stop/coast behaviour and safe bench data. The optional motor-test build gathers those values; it does not use unvalidated pulse durations for pointing.

## Commands

```
HELP                 List commands
STATUS               Current gyro, reference, sensor mask and state
MAP                  Print target bearings and repeatability
TARGETS 1            Configure one beacon, force new calibration
TARGETS 3            Configure three beacons, force new calibration
RESET                Discard gyro reference and wait for next verified IR
SURVEY               Discard map and begin a new survey using current gyro reference
VERIFY               Preserve map; begin repeat-arc recording after exiting cone
VERIFY STOP          Finish repeat recording; print updated map
FLICKER 20           Measure raw active-LOW occupancy and qualified edges for 20 sec
DRIFT 60             Record gyro yaw, velocity and temperature for 60 sec, held STILL
MARK                 Store a temporary gyro angle for physical-mark experiment
CHECK 360            Compare angular change to known 360° manual turn (or CHECK -360)
CHECK 0              Check error returning to same physical mark
TRACE ON / TRACE OFF Read 250ms status lines while moving slowly
SPOT 20 1000         Given measured 20 mm laser offset at 1000 mm, compute error deg
STOP                 Turn laser and any motor drive off (not a physical wheel brake)
```

**Optional later**, after the actual motor/encoder are wired and `ENABLE_MOTOR_TESTS=1` in `SentinelFirmware.cpp`: `ARM MOTOR`, then `PULSE 65 150`. Single pulse drive <=1200 ms at PWM 1..180, followed by 1800 ms coast-time logging. The program prints `MOTOR_DATA` every ~60 ms and a `RESULT_MOTOR` summary including gyro yaw change, wheel counts, provisional observed K, peak encoder RPM, late-drive average RPM and ending motion. Test must be re-armed each time. No automatic pulse batches. The encoder count convention and motor-driver truth table **must** be confirmed on your actual hardware. Motor tests do not infer a true unloaded maximum speed from a brief pulse.

## Validation performed on the source

Host-side C++ syntax checks passed with mock Arduino headers for both `ENABLE_MOTOR_TESTS=0` and `1`. A synthetic-sensor simulation passed the **starting-inside-IR, calibration, clearing, complete single-beacon survey** scenario and an additional scenario with **three beacons**. Neither checks substitute for a real Arduino AVR build, physical MPU orientation test or motor hardware testing.
