# MQ Sentinel — Gyroscope and IR Test Suite (use a markdown previewer to read this)

**Version:** 1.0 — 14 October 2026  
**Target hardware:** Arduino Nano V3 (ATmega328P) + Core Electronics 018-MPU-6050 + three active-LOW 38 kHz demodulating IR receivers + optional passive FIT0186 encoder + optional eye-safe laser.  
**This is diagnostic firmware:** no motor commands, no EEPROM, no automatic spacecraft motion. Laser is OFF unless explicitly compiled on and armed.

## The approach

**Do not use a gyro alone as an absolute compass.** The MPU-6050 measures angular velocity. For the project, place it rigidly on the **body** with one gyro axis parallel to the suspension axis; integrate the bias-corrected yaw rate to track a continuous relative heading:

```
yaw += (measured_gyro_yaw_rate - calibrated_bias) * dt
error_to_saved_target = wrap_to_minus180_plus180(saved_heading - yaw)
```

Save a heading as a *landmark* only when the physical laser and target are **actually aligned**, not merely when a broad IR field is seen. Landmarks A/B/C live in **RAM** and share the same arbitrary zero established on boot. Once the correct, previously identified IR target is observed again, `SYNC A` (etc.) can shift the gyro's angular reference to cancel accumulated drift. This does **not** physically rotate the satellite, and it cannot identify which anonymous transmitter has turned on. Only the active 38 kHz beacon provides independent directional evidence; three targets with identical signals cannot be distinguished by identity using this IMU.

**Main new architecture for the eventual controller (not present in this firmware):**

1. IR interrupt detector proposes search/centering direction. Gyro gives live body angular velocity and relative heading. Flywheel encoder gives wheel RPM only.
2. Identify target and establish heading when IR is qualified and laser is physically calibrated on the real target. Track estimated target heading as the body rotates; aim using error angle and yaw rate.
3. During approach, *brake the satellite's measured yaw rate*, not just the reaction wheel. Cutting or electrically braking a wheel does not necessarily stop body rotation immediately.
4. Only begin a 2-second firing dwell after **both** optical qualification and low *body* angular velocity are sustained; abort if either is lost. Gyro makes coasting/overshoot measurable.
5. When a known beacon is independently reacquired, correct integration drift. If beacon identity is not known, search or check the saved candidate headings rather than blindly choosing one.
6. Use empirical IR-to-laser angular offset calibration. `010` or `111` means an IR sensor sees carrier; neither alone proves the laser is on the scoring bullseye. `111` may be reflection/crosstalk.

With an 8 cm-diameter bullseye approximately 1 metre from the laser, the nominal angular half-width is `atan(0.04 / 1.0) ≈ 2.29°` (before mechanical misalignment). To score the full 2 s, you need margin for offset, gyro drift and motion. A trial threshold of **|yaw rate| < 0.65°/s** is used, corresponding to up to ~1.3° yaw change over 2 s even at that threshold; this is only a test starting point, **not** a guaranteed bullseye.

## Critical wiring and safety

| Component | Nano | Notes |
|---|---|---|
| Left IR output | D2 | Active LOW; `1` in printed mask means detecting |
| Centre IR output | D3 | Active LOW; `010` represents centre-only |
| Right IR output | D4 | Active LOW |
| DRV8874 IN1 / IN2 | D6 / D7 | **Both held LOW (coast)**; disconnect motor supply for bench testing if feasible |
| Laser control | D8 | Defaults LOW/OFF. Test physically that LOW = OFF with your transistor/module |
| FIT0186 encoder A / B | D9 / D10 | Optional, passive. Approx. 700 counts/output rev is unverified for this interrupt decoding method |
| MPU-6050 SDA / SCL | A4 / A5 | **Use a bidirectional 5V↔3.3V I²C level converter** for Nano, as recommended by Core Electronics for this module |
| MPU-6050 VCC / GND | Module-rated supply / GND | This particular breakout advertises 3–5 V *power* and an onboard regulator. Power **bare chips** at 3.3 V only. Verify the specific board |
| Built-in LED | D13 | Reserved, no dependence on it |

Important distinction: **3–5 V module supply is not the same as 5 V-safe SDA/SCL.** The seller specifies 3.3 V logic and explicitly recommends a converter for 5 V controllers. Never connect the module SDA/SCL to pins with 5 V pull-ups unless level shifting is verified. Do not solder/adhere permanently to any supplied returnable parts in violation of the course constraints. Ensure the laser is eye-safe, fixed to the test apparatus, and never pointed at people or reflective surfaces.

**Mounting:** The code uses `YAW_GYRO_AXIS = 2` and `YAW_SIGN = +1` initially. If your board is sideways, change the axis (X=0, Y=1, Z=2). Use Test 1 while manually turning the suspended satellite to discover which axis changes. If the heading sign is reversed from your intended left/right convention, set `YAW_SIGN = -1`.

## Install and start

1. Put `MQ_Sentinel_IMU_TestSuite.ino` in a folder with the same name. Open in Arduino IDE with **Arduino Nano**, correct processor/bootloader and COM port.
2. At the top of the code, set `USE_MPU6050`, `USE_IR_SENSORS`, `USE_ENCODER` to `1` or `0` based on connected hardware. You can build with **all three 0** and run the pure serial maths test. By default, `ENABLE_LASER 0` disables the laser **at compile time**.
3. Open Serial Monitor at **115200 baud** and select **Newline** (or Both NL & CR). Put the satellite down and KEEP IT STILL for the 300-sample (~3 s) gyro calibration at each power-up.
4. Type `HELP` for the menu. Enter either `7` or `MODE 7` to switch tests **without reflashing**. Enter `RATE 200` to set the status line interval to 200 ms (range 50–2000 ms).
5. Be aware that a Nano typically **resets when Serial Monitor opens**. Reopening the monitor will usually erase RAM landmarks. A dangling USB cable itself may apply torque: bench-test with sufficient slack, then retest mechanically with minimal cable interference.

### Runtime tests

| Mode | Connected parts | Purpose / exact procedure | What to measure |
|---|---|---|---|
| **0** | Nano only | Start/default. Type `STATUS` and `HELP` | No unintended laser or motor output; gyro available/fault |
| **1** | MPU | Keep flat, then yaw/tilt each axis separately | Which `gx,gy,gz` follows body yaw; accelerometer gravity near 1 g in vector norm; temperature changes slowly |
| **2** | MPU | Rotate body clockwise/counterclockwise by ~90° by hand, then stop | Angle sign and gyro rate; when stopped, `still=1` after 300 ms |
| **3** | MPU | Start test with body stationary for **60 s** | Net `delta_deg` and mean drift deg/s. Repeat after 5 minutes of warming |
| **4** | MPU | Mark physical index on suspended body; rotate +90°, -90°, return home by hand | Heading-vs-protractor error; overshoot and return repeatability |
| **5** | IR | Test TX OFF, one active TX, several angles, then turn TX OFF | `raw` fast LOW mask vs 75 ms `held` mask; ISR falling counts; presence/absence of spurious detection |
| **6** | IR | At target centre and just outside centre, run 20–30 s each; vary light source distance | Mask occupancy histogram (`010`, `111`, `101`, etc.). 50 Hz approximate sampling, not continuous logic analyser |
| **7** | MPU + IR | Manually sweep left-to-centre-to-right slowly through an active beacon | IR direction-vs-gyro yaw, `CENTRE LOCK EVENT`, width of detected angle, whether `111` appears |
| **8** | MPU | At actual aimed point type `MARK A`; move away then type `ERROR A`; repeat B/C | Heading error to stored reference; wrap correctly through ±180° |
| **9** | MPU + IR | Manually hold target/IR centre steady for 2 s | **Dry-run only**: `DRY SUCCESS`, reset count, qualification duty-cycle; compare 010/111 stability |
| **10** | MPU + laser, **no IR necessary** | Enable compile-time laser, select mode, type `ARM`, stay still then type `FIRE` | One 2-s laser pulse, cuts off if gyro body motion too high; check true pointing stability |
| **11** | MPU + IR + laser | Enable compile-time laser, select mode, `ARM`, and manually align | One supervised 2-s shot starts on qualified IR + rate gate; cuts off on loss; no motor |
| **12** | Any connected | Let CSV stream; capture from Serial Monitor or terminal to file | Timestamp, gyro yaw/rate, 3 axes accel/gyro, temp, raw/held IR, qualified, encoder |
| **13** | Encoder (+ MPU useful) | Hand-spin/backdrive wheel if mechanically safe; don't energise motor | Wheel relative counts and approximate RPM compared with BODY gyro yaw. A stationary wheel count is normal if not rotated |
| **14** | **Nano only** | Type `SIM 350 10`, `SIM 10 350`, `SIM -179 179` | Signed shortest angular errors: +20°, -20°, -2°. No sensors read to calculate these values |
| **15** | MPU + IR | Save A with `MARK A`, move, reacquire the **same** target; `ERROR A`, then `SYNC A` only when qualified | Before/after landmark residual; can correct accumulated gyro angle bias without motors |

**Test 5 key insight:** Demodulating 38 kHz receivers can **suppress a perfectly continuous, unbroken carrier** because of automatic gain control. Use a *38 kHz carrier sent in bursts*, separated by gaps (your earlier 600 µs ON / 600 µs OFF test is a starting point). This program does **not** calculate modulation frequency from demodulated outputs; it only timestamps/counts the active-LOW transitions. Keep the same target/transmitter behaviour the university will actually use.

**Test 6 key insight:** Your earlier `010`↔`111` observations explain why insisting that the *entire* 3-bit mask remain unchanged can stop shots from ever qualifying. The code maintains one uninterrupted **accepted-mask streak** across `010` and `111` transitions. It resets when the mask becomes invalid (e.g. `101`). For future precision, favour centre-only `010` as a high-confidence optical reference **if the experiments show it correlates with actual bullseye alignment**.

## Exact commands for the heading-memory experiment

```
CAL                 // stationary gyro-bias calibration
ZERO                // sets heading to 0, clears ALL saved landmarks
8                   // heading-memory mode
MARK A              // while mechanically positioned at target A
ERROR A             // shortest signed heading error from current heading
MARK B              // after manually aligning on B
MARK C              // after manually aligning on C
LIST                // shows saved headings and live return errors
15                  // relock experiment (does not move motor)
ERROR A             // rotate manually to target A, inspect residual
SYNC A              // ONLY if 010/111 stable AND this really is A
LIST                // all stored headings remain, reference is corrected
```

Example: if a saved landmark is at 350° and current heading is 10°, the shortest way back is **-20°**, not +340°. When you `SYNC A`, the firmware changes its *software angle offset*, not the actual satellite rotation. If you accidentally `SYNC A` at beacon B, **all landmark estimates become wrong**. There is **no magnetometer** on an MPU-6050, so yaw cannot be independently restored if target identity is lost and the gyro has drifted.

## Controlled laser tests

For tests 10–11 only, deliberately change `#define ENABLE_LASER 0` to `1`, reupload, and verify that D8 LOW really switches the particular laser interface OFF. The test begins DISARMED; `ARM` must be entered in the current mode. `DISARM`, changing mode, sensor failure, excessive measured gyro yaw rate or 2,000 ms expiry turns D8 LOW. **Test 10** additionally needs a manual `FIRE` command; **Test 11** starts only after a 250 ms accepted IR streak and a 300 ms steady gyro streak. Each `ARM` authorises **one** shot only.

**No firmware can guarantee alignment from digital IR masks alone.** Record overhead video with a protractor/grid and the physical target to establish the true laser offset and real 2-second hit rate before trusting autonomous fire.

## Suggested pass criteria — engineering hypotheses, not supplier specs

| Measurement | Initial trial acceptance condition | Why |
|---|---|---|
| Gyro yaw rotation | Manual +90° and back-to-zero error < 3° across 10 repetitions | Detect mounting errors and integrated drift |
| Stationary drift | Prefer < 2° per minute after calibration | A 4 cm radial scoring zone at 1 m subtends only ~2.29° |
| Stable target gate | `qualified=1` stays high > 2 s at real laser-centred aim | The prior firmware never reliably completed the full dwell |
| IR false positives | 0 stable `qualified=1` events when transmitter is OFF | Avoid false firing from lighting/reflections |
| Optical qualification | Record `%010`, `%111`, `%101` separately at true bullseye and off-axis | Establish whether 111 is genuinely useful or misleading |
| Two-second aim | Video shows laser remains in same scoring zone for full 2 s | IR and IMU alone cannot certify scoring |
| Landmark return | Aim error within ±2° after manual return from each direction | Relevant to repeated A/B/C targeting |

Write down failed runs too; only results from actual hardware can validate these proposed thresholds.

## Known limitations

- Gyro integration **drifts** because of bias, temperature and timing; its accelerometer cannot correct yaw while the satellite rotates about the vertical gravity axis.
- 100 Hz gyro samples (DLPF about 44 Hz) are sufficient for initial manual tests, not proof of closed-loop performance with the motor running. Retest vibration and I²C reliability under realistic mechanical operation.
- `010` and `111` both count as accepted per your last firmware decision. The diagnostic distinguishes them, but **neither mathematically proves an 8 cm bullseye**.
- The 75 ms receiver activity hold is intended to survive gaps between bursts. Rapidly sweeping past multiple receivers can temporarily create composite masks. Change this parameter only after observing Test 5/6 logs.
- The passive quadrature decoder reports wheel ticks, not the satellite's inertial angle. `ENCODER_COUNTS_PER_REV_APPROX = 700` is a previous estimate and must be measured for the exact encoder and 4x decoding.
- The laser interlock is a **software safety mechanism**, not a hardware safety-rated circuit; keep supervision and a physical way to disconnect laser power.
- With `USE_MPU6050=0`, `USE_IR_SENSORS=0`, `USE_ENCODER=0`, mode 14 still tests the angle-maths commands using just a Nano and Serial Monitor.
- Data logs are streamed only; there is **no EEPROM writing**. If you run off battery without a serial connection, nothing is retrospectively retrievable after reset. Record live output with a terminal or use a later approved logger; avoid a taut USB lead that imparts yaw torque.

## Sources and project constraints

- *MQ Sentinel — Space Debris Cleanup Challenge* brief, ENGG2000 2026 S2, pp. 4–8, 12–13: 38 kHz target sources, Nano primary controller, 5-minute test, 2 s consecutive laser hold, reaction-wheel gearing, 8/16/24 cm scoring rings, specified supplier restrictions.
- Core Electronics **018-MPU-6050**: [product listing](https://core-electronics.com.au/mpu-6050-module-3-axis-gyroscope-acce-lerometer.html), confirms 3–5 V module supply, **3.3 V logic level**, 0x68/0x69 addresses, onboard power regulator and optional matching level shifter.
- InvenSense **MPU-6000/MPU-6050 Product Specification** (Rev. 3.4), available from the same product listing; yaw-axis rotation rates and I²C interface. Detailed register map values used in the sketch: `0x6B` power mode, `0x19` sample divider, `0x1A` DLPF, `0x1B` gyro range, `0x1C` accel range, data burst beginning at `0x3B`.

**Validation:** Source was checked for C++ syntax using a mocked Arduino/Nano API (`g++ -std=gnu++11 -Wall -Wextra -Werror -fsyntax-only`). An actual Nano toolchain and the physical electronics were **not available** here, so final memory size, I²C behaviour, and real hardware operation have to be validated in Arduino IDE and on the rig.
