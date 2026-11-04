# HM 2.2 — reduce near-centre chatter

Minor update from HM 2.1. The operator reported persistent centre jitter without firing and switched off the test. HM 2.1 remains preserved for comparison.

## Evidence

`observed_HM_2_1_partial_dump.csv` contains the rows actually visible in Arduino Serial Monitor. This is a partial capture, not a complete EVENT_LOG dump: no start/end marker or final record counts were captured. It is not evidence that all events in the run were absent.

For event 21, `state_or_block` has state in the low nibble and the most frequent blocker in the high nibble. The rate column packs peak fast rate in the low byte (divide by 2 for deg/s) and peak pointing rate in the high byte (divide by 10). The PWM/confirmation column is the best entry-confirmation duration in milliseconds, not motor PWM. Decode it differently from direct AIM event 22.

At 35 s state 58 decodes to FINE with angle the largest blocker. At 40-60 s, state 74 decodes to FINE with motion the largest blocker. Sampled angle errors at 45/50/55/60 s were -0.19/+0.17/+0.25/+0.07 degrees. The corresponding five-second peak fast rates were 9.5/10.5/10/10.5 deg/s and peak pointing rates 2.4/2.3/2.3/2.3 deg/s. Best uninterrupted entry windows were only 10/8/8/12 ms, below the required 140 ms. Flags 80 indicate rate blocking and at least one all-gates-pass observation in those windows; they do not show continuous valid lock. Flag 81 at 60 s also records IR blocking somewhere in that window.

This supports a small-position/high-rate settling problem. It does not distinguish real body oscillation from vibration, gyro noise, motor feedback amplification or other disturbances. The records do not identify pulse timing or prove a particular physical cause.

## Changes

- A continuous 0.2-degree position quiet band removes tiny reversing position commands. Outside it, restoring error is `sign(error)*max(0,abs(error)-0.2)`, so correction grows smoothly with drift rather than jumping at the boundary.
- Integral hold bias uses the same adjusted error. It stops accumulating for tiny errors inside the quiet band, while retaining the learned balancing torque against the string. Its +/-20 PWM limit and saturation checks remain; the learning exclusion still checks raw position error.
- Restore the V35/HM 2.0 rate gain of 4 instead of HM 2.1's 6. Outside the quiet band, position gain 0.6 gives restoring stiffness 2.4 PWM/degree, matching the earlier controller. The higher HM 2.1 gain may amplify rate ripple; that mechanism is a hypothesis, not established by these summaries.
- Within the existing firing angle window, hold output changes by at most 2 PWM counts per motor tick (nominal 5 ms, or 400 counts/s). This bounds changes to actual applied voltage, including feedforward. It reduces abrupt reversals, but adds actuator response delay; physical validation is required. Outside that window the slew restriction does not apply, preserving pulse authority. Sustained drift still receives correction.
- HM 2.1's independent firing entry and hold limits remain unchanged: 140 ms entry, pointing rate <=1 deg/s, faster rate <=2 deg/s; hold uses 1.5/2 deg/s, with 40 ms rate confirmation and an immediate 4 deg/s cutoff. Angle, IR and wheel checks remain. The two-second dwell and two-second startup remain.
- Removed verbose human-readable ARC_OK detail, ARC_BAD span/efficiency, and LASER_ARM angle/rate plus Serial.flush to make flash space. ARC_OK, rejection reason, laser output confirmation, CSV telemetry and EEPROM records remain. The laser command no longer waits for Serial.flush.

This is an attempt to reduce the motion that blocks firing, not a relaxation of motion limits. Near-zero angular motion during firing remains the objective. Exact zero cannot be established with finite, noisy sensors.

## Validation

- Arduino CLI 1.2.0, Nano ATmega328P old bootloader: **30,652/30,720 flash bytes**, 876/2,048 static RAM bytes. Only 68 flash bytes remain.
- Native harness extracts production functions/constants and checks the quiet band, continuous restoring response, opposing velocity torque, per-tick output slew, bias retention/learning/bounds, pulse behavior, entry confirmation/reset, and hold rejection.
- Comparisons verify that coarse AIM, recovery, pulse algorithm, fire tolerance and firing/hold gate functions match HM 2.1.
- Not uploaded or physically tested during this update. No simulation claims are made for the real string/motor/sensor dynamics. A slew limiter can worsen settling if actuator delay dominates; compare against the preserved baseline.

From the repository root:

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_2/validate_hold.py SENTINAL_HM_V31/MQ_HM_Sentinel_2_1/MQ_HM_Sentinel_2_1.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_2_2
```

Python 3 and clang++ are required for the harness. Use the established A/next battery boot/D-dump workflow and the same physical setup for comparison. Check whether centre jitter decays and each laser command completes two continuous seconds. Save each dump before another armed run; retain the software version with it. EEPROM schema and dump label remain V35 for compatibility; startup banner identifies HM 2.2.

Versioning: minor updates advance 2.2 -> 2.3; major updates start 3.0. Preserve earlier sketches and other team files when publishing under SENTINAL_HM_V31 on main.
