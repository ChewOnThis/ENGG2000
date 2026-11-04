# HM 2.1 — damped target hold

Minor update from HM 2.0, based on the original main-branch MQ Sentinel V35. Requested goal: resist drift smoothly and approach zero angular velocity during firing. No new usable EEPROM evidence was supplied. These settings are provisional and require physical testing on the same suspension.

## Changes

- The near-target requested body rate now grows as 0.45 times angle error, capped at +/-1.2 deg/s, versus 0.6 previously. The rate-error gain increases from 4 to 6 PWM/(deg/s). This slightly increases position restoring stiffness (2.7 vs 2.4 PWM/degree before bias) while increasing rate damping by 50%. It does not establish the damping ratio or guarantee stability of the physical suspension.
- The bounded +/-20 PWM integral hold bias remains, including its saturation checks. Its anti-windup calculation uses the same new gains as the applied controller.
- HM 2.0's positional pulse suppression remains: no new correction pulses inside the firing angle window, and active pulses end upon entering it. Active hold damping continues there. Outside the window the original fine pulses and return controller remain.
- Entry requires the 150 ms filtered pointing-rate estimate <=1.0 deg/s AND the faster filtered body rate <=2.0 deg/s for an uninterrupted 140 ms. Previous values were <=3.2 deg/s, <8 deg/s and 85 ms respectively. IR, angle, wheel RPM and calibrated/valid-reference checks remain.
- During FIRE, faster rate <=2.0 deg/s and pointing rate <=1.5 deg/s sustain hold. Violations get the existing 40 ms confirmation window; faster rate >=4.0 deg/s cuts the laser immediately. Previously only faster rate <=4.3 and an 8.0 hard cutoff applied. Existing IR/angle/RPM constraints still cut hold immediately when their conditions fail. A completed laser command still requires a continuous two seconds.
- EEPROM gate summaries use the new entry-rate criteria. The ordinary CSV flag 16 remains a fast-rate-only indicator and is not the complete entry-rate gate; use event 21 summaries and direct rates for diagnosis.

Exact zero is not a practical sensor threshold. The controller corrects motion during FIRE; the limits above define what motion is accepted, rather than guaranteeing physical stillness. Stricter thresholds can delay or prevent firing if mechanical vibration, gyro noise, drift or insufficient actuator authority persist. Do not infer that a failure to fire necessarily needs looser gates.

## Validation and limits

Nano ATmega328P old-bootloader compile: **30,678/30,720 bytes flash**, 880/2,048 bytes static RAM. Only 42 flash bytes remain. Not uploaded or physically tested during this update.

`validate_hold.py` extracts the production functions and constants into a native C++ harness. It checks restoring-force growth and sign, damping direction, bounded integral learning, pulse suppression and resumption, both rate gates, uninterrupted 140 ms confirmation/reset, IR/angle/wheel rejection, and persistent/high-speed hold interruption. It also checks that coarse return, recovery and the pulse algorithm match HM 2.0. This does not simulate the motor, string torque or gyro noise.

From the repository root, run:

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_1/validate_hold.py SENTINAL_HM_V31/MQ_HM_Sentinel_2_0/MQ_HM_Sentinel_2_0.ino
```

Requires Python 3 and clang++. Standard build:

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_2_1
```

## Comparison test

Keep the tested suspension and stationary target arrangement. Check both first and second target acquisition. Observe whether drift is corrected smoothly, whether it settles enough to fire, and whether each laser command lasts the full two seconds. Save EEPROM before starting another armed run. Compare against HM 2.0 if settling worsens or firing stalls; the controller gains have not been identified from physical response data.

Two-second startup and A/next-boot/D-dump workflow remain. EEPROM layout/version remains 35, preserving compatibility and the existing V35 dump label; the startup banner identifies HM 2.1. Record software version alongside each dump.

Versioning: minor changes advance 2.1 -> 2.2; major changes start 3.0. Preserve prior folders and teammates' files when publishing to main.
