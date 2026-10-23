# MQ Sentinel: measurements to send back

Set up ONE beacon at **1.00 m from the receiver** (measure centre-to-centre) and keep the beacon FIXED during all angular survey passes. Arduino Serial Monitor: **115200 baud**, **Newline**. Copy serial output as plain text, preserving all lines, including `CAL_REJECT`, `REJECT_ARC` and `FAULT`.

## 0. Wiring and first reference

- D2 left, D3 centre/primary, D4 right, gyro MPU-6050 I2C A4/A5; no motor needed. If only D2 is present, set `IR_SENSOR_COUNT 1`, `PRIMARY_SENSOR 0` in `SentinelFirmware.cpp` before uploading.
- On power-up, put beacon within the field of at least one sensor. Wait for `CAL_START`, then **hold completely still ~2.2s** until `CAL_OK` prints. If `CAL_REJECT` repeats, send those lines as well.
- Save the full `CAL_OK` message including `bias_dps`, `noise_sd_dps`, `temp_c`.

## 1. IR detection/noise at 1 metre (requires no movement)

- Point primary sensor directly at the beacon. Enter `FLICKER 20`. Save all `RESULT_FLICKER` lines.
- Keep the body fixed and point the beacon outside the sensor's field (or switch transmitter off). Enter `FLICKER 20` again. Save results.
- Finally position receiver near an IR cone edge, where it sometimes loses reception. Enter `FLICKER 20` a third time.
- We use raw-low % and filtered edge transitions to choose the ON confirmation, OFF confirmation and occupancy thresholds; intermittent noisy transmitters may need a different signal-handling strategy.

## 2. One complete IR arc and bias of the midpoint

- Enter `SURVEY` if calibrated. If IR is currently active, rotate OUT of all cones first. Wait for `SURVEY_START`.
- Keep beacon fixed at 1 m. Slowly turn chassis in ONE direction through **a full physical revolution**, passing completely out of the IR cone. Keep a visible physical mark of the starting heading. Ideally use ~3-6°/s.
- Capture all `SENSOR_ARC`, `REJECT_ARC`, `ARC`, `TARGET_NEW`, `SWEEP_360_REACHED`, `SURVEY_END` and `MAP` lines.
- `SENSOR_ARC` on D2/D3/D4 reveals how their FOVs overlap. `ARC` and `TARGET` reflect the selected PRIMARY detector. You can move the beacon to 2 m and repeat later to test sensitivity to range.

## 3. Repeatability and sweep-direction bias

- Once `MAP_READY`, enter `VERIFY`.
- Exit the primary sensor IR cone if currently in it. Wait for `VERIFY_READY`.
- Slowly pass through its **entire** field of view 5 times in the clockwise direction and 5 times anticlockwise, stopping only outside the cone. Pause briefly between passes. Afterward, enter `VERIFY STOP` and `MAP`.
- Send every `ARC`, `TARGET_UPDATE`, `ARC_EXTRA`, `REJECT_ARC`, and final `TARGET` line. We will evaluate centre dispersion, edge delay and whether slit geometry produces directional bias.

## 4. Stationary gyro drift

- Support or hold the suspended satellite so it is **really still** and run `DRIFT 60`.
- Save every `DRIFT_SAMPLE` and `RESULT_DRIFT` line. Run twice: near room temperature, and later after it has been powered for a while (temperature/bias sensitivity).
- A drifting real chassis invalidates the interpretation: the gyro measures physical motion as well as bias error. Use a fixed physical indicator to verify the body stayed still.

## 5. Test physical 360-degree gyro accuracy

- Place a physical alignment mark on satellite and fixed frame. With the calibration intact, type `MARK`.
- Rotate carefully through exactly **one physically verified** 360° clockwise turn back to the mark (avoid cable twisting). Type `CHECK 360` or `CHECK -360`, matching actual sign in `STATUS`. Capture `RESULT_CHECK`.
- For a simple return-to-same-heading test, type `MARK`, turn away ~30-90° then return to the exact marked orientation, and send `CHECK 0`.
- These independent physical angles expose gyro scale/axis error and offset drift that cannot be detected by relying only on gyro-based `SURVEY_END`.

## 6. Jolt / reference-loss diagnosis

- Never hit the satellite hard or intentionally overspeed the gyro. Rotate and stop it normally, and see whether reference accuracy changes. After any accidental strong jolt, save `JOLT` / `FAULT` lines and compare the gyro to the physical mark.
- If the reference becomes suspect even with no `FAULT`, type `RESET` and recalibrate from a new verified IR trigger. No old EEPROM angles are loaded.

## 7. Optional laser alignment

- Use only a correctly driven, eye-safe laser, with a suitable optical target and no people in beam path. With laser output disabled in firmware, you may temporarily power your existing laser using its designed driver and manually measure lateral error between its spot and beacon optical centre.
- Enter `SPOT 20 1000` for an *observed*, not estimated, 20 mm error at 1 m. Save `RESULT_SPOT`. Repeat after physically adjusting mounts. This is a mechanical boresight calibration, distinct from gyro angle.

## 8. Motor tests AFTER you obtain the motor + encoder

- You cannot determine `K_BODY_PER_WHEEL` or `WHEEL_NO_LOAD_RPM` from gyro, IR and laser alone. The gyro reads body angle only. To measure body/wheel ratio you must record **wheel angular displacement** with the encoder.
- Enable `ENABLE_MOTOR_TESTS 1`, verify D6/D7 IN/IN driver and D9/D10 encoder direction and counts per rev, and arrange emergency motor-power isolation.
- `ARM MOTOR`, `PULSE 65 150` first. Copy all `MOTOR_DATA` and `RESULT_MOTOR` lines; start with low PWM and observe mechanical safety before other powers. Never start with a high-power batch.
- Data supports approximate `K = -delta_body_angle / delta_wheel_relative_angle` only when a consistent yaw axis / sign convention is used and the response is sufficiently settled. `peak_rpm` and `tail_mean_rpm` are **not automatically no-load RPM**. Genuine max-speed estimation needs adequately long safe tests showing a plateau.

## What to submit to ChatGPT

Paste complete Serial Monitor text grouped by test, plus: exact sensor model, MPU-6050 mounting orientation, beacon distance, whether all three IR receivers are connected, direction of physical rotation, approximate temperature, and whether chassis moved during gyro calibration. The most useful first batch is `CAL_OK`, three `RESULT_FLICKER` sets, 5+ `ARC` lines from `VERIFY`, `RESULT_DRIFT`, and `RESULT_CHECK`.
