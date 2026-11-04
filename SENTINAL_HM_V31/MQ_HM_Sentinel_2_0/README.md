# MQ HM Sentinel 2.0

Based on `main/MQ_Sentinel_V35/MQ_Sentinel_V35.ino` (blob `c89f56c380051ff01768095778f6af440603c575`). This starts a new HM version series; it is not based on the older MQ_HM_Sentinel_V35/V36 experiments.

## Change and rationale

The operator reported that the second target acquisition stuck jittering without firing. No usable EEPROM dump was available; the operator requested proceeding by source review. The cause is therefore a code-based hypothesis, not confirmed by telemetry.

V35 suppressed fine pulses only when angle, recent IR, fast filtered rate and wheel RPM all passed `centred()`. Rate ripple above 3.2 but below 3.5 deg/s could permit a fresh pulse even while within the firing angle window, potentially exciting motion and delaying the independent 85 ms firing confirmation.

HM 2.0 uses a positional window for pulse suppression: calibrated gyro, valid arc, recent IR, and the existing firing angle tolerance. It also stops an ongoing pulse when it reaches that window. Active rate damping and bounded hold bias continue; outside the window the existing fine pulses remain available. Firing angle/rate/IR/RPM checks, confirmation duration, two-second laser dwell, coarse AIM and search recovery are unchanged. The startup wait is two seconds, retaining Harrison's previously requested preference.

This specifically removes an unnecessary-pulse pathway. It does not prove that all jitter, mechanical vibration, gyro error or firing blockers are resolved. Physical testing is still required.

## Validation

- Arduino CLI 1.2.0, `arduino:avr:nano:cpu=atmega328old`: 30,628/30,720 bytes flash; 880/2,048 bytes static RAM. Only 92 flash bytes remain.
- Extracted production fine-control functions were exercised in a native C++ harness: no new pulse during in-window rate ripple, active pulse termination in-window, retained pulses outside, 5-20 ms pulse bounds, and independent firing rejection for missing IR, excess wheel/body speed and angle error.
- Source comparisons verify that V35 firing tolerance, entry/hold checks, arming, coarse return and recovery checks remain unchanged.
- Run `python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_0/validate_fine_window.py MQ_Sentinel_V35/MQ_Sentinel_V35.ino` from the repository root. Requires Python 3 and clang++.

## Test and EEPROM

Open `MQ_HM_Sentinel_2_0.ino`, compile/upload to the Nano using the established board configuration. No upload was performed as part of this change. Retain the previously tested suspension and stationary targets for comparison. After sending A and receiving ARMED_NEXT_BOOT, power off, unplug USB and test from battery. Motor startup follows a two-second wait. Verify that the second acquisition settles and completes a continuous two-second laser command; record whether the laser stays on through any remaining small jitter.

EEPROM schema remains 35 because the record layout is unchanged. The dump header remains EVENT_LOG_BEGIN,V35 for compatibility; the startup banner identifies HM 2.0. Uploading this code does not deliberately clear a same-format log, but arming and starting a new run replaces it. Save any needed dump before the next run.

## Versioning

Current version: HM 2.0. Minor changes create 2.1, 2.2, etc.; a major change starts the next major series, 3.0. Keep each released sketch in its own matching folder/file name, record changes and validation, and preserve earlier versions. Publish updates under SENTINAL_HM_V31 on main without replacing other teammates' files.
