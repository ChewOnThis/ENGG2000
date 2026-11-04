# HM 3.2 — body braking and lost-headroom recovery

Fix candidate based on the HM 3.1 EEPROM run. Fine centering and firing are preserved; physical improvement is not yet established.

## Evidence

Selected observed rows are saved in `reference_excerpt/observed_HM_3_1.csv`; this is an excerpt, not a full dump. The visible log identified HM3.1, format 23, with samples=40, events=14, bad_crc=0, dropped=7.

At 70.95 s, wheel RPM was 147.7, applied PWM -255 and body rate +28.60 deg/s. By 75.76 s, wheel RPM was still 148.7 at PWM -255 but body rate was -0.36 deg/s; at 76.17 s it was -5.57 deg/s. This strongly supports lost torque headroom: more PWM was unavailable while wheel speed plateaued and requested body motion failed. A fixed 250 RPM safety limit did not detect that operating limit.

The first accepted scan had negative return error (-21.04 degrees) and positive body motion; the later scan had positive return error (+21.22 degrees), negative body motion and a still-positive wheel speed. The old post-scan phase applied the same electrical wheel-brake command regardless of body direction. That is a plausible contributor to the direction-dependent jolt. The old dump did not sample the braking interval, so it cannot prove the full transient cause.

## Changes

- **Active body braking after a scan:** ARC_BRAKE now requests zero body rate using the existing signed rate controller and wheel-speed voltage feedforward, with correction capped at 90 PWM. Applied command changes by at most two PWM counts per motor tick, nominally 5 ms. It no longer forces motorHardware(0), the explicit electrical wheel-brake state. The wheel can keep spinning while the body slows. See the preserved [braking-physics explanation](../MQ_HM_Sentinel_3_0/BRAKING_PHYSICS.md).
- **Measured handoff:** require the ready slower pointing-rate estimate <=3 deg/s and faster rate <8 deg/s for 100 ms before AIM. If this does not happen within 1.5 s, use controlled recovery instead of proceeding blindly. Normal fine/firing rate checks are unchanged.
- **Plateau detection:** during travel or post-scan braking, require applied PWM magnitude >=245, wheel magnitude >=40 RPM, drive polarity attempting to accelerate that wheel, and a persistent body-rate miss. Travel rate miss is greater than max(1.5 deg/s, 25% of requested rate); ARC_BRAKE uses slower pointing rate >3 deg/s. If wheel magnitude does not increase by more than 2 RPM for 500 ms, initiate recovery. Continued wheel acceleration restarts the window; flat RPM alone at the desired body rate does not trigger it. Departure and speed-guard overrides are excluded.
- **Momentum-aware recovery direction:** choose the search drive opposite the wheel's maintaining-voltage polarity, rather than blindly flipping the previous search direction. At low wheel speed, default to reversing the old search direction. Use the existing controlled recovery, seeded from actual PWM and body rate, retaining its ramped rate reference and applied-voltage handoff. This is not an instantaneous motor-polarity flip or a promise that the body stops immediately. Recovery can deliberately leave the current target to regain authority and then reacquire it; it is not a new closed-loop wheel-RPM desaturator.
- The new detector excludes FINE and FIRE. Existing close tracking protections remain. Fine pulses, fine hold/bias learning, IR-entry continuity, laser angle/rate/IR/RPM gates, confirmation and two-second firing branch are checked against HM 3.1.

Thresholds are trial values, not identified motor/string parameters. A stalled encoder, load change, supply sag or other problem can resemble lost headroom. The detector can miss cases outside its conditions. Real deceleration and accuracy must be checked on the Sentinel.

## EEPROM and flash

Format 23 and allocation remain **1,019/1,024 EEPROM bytes**. Add direct 200 ms samples in ARC_BRAKE and UNWIND, and event 7 marks detected headroom recovery. Existing event 15 marks the following recovery. The decoder labels event 7 wheel_headroom_recovery. CRC, committed-record handling, queue/drop reporting and the short rolling history remain. Earlier logs are preserved on upload; save/label them before another armed run.

Continuous USB CSV streaming is disabled (`LIVE_CSV=0`) to fit the added controller logic on the Nano. Startup, arming messages, D dumps and EEPROM recording remain. Re-enabling live CSV requires checking flash size again.

Final Nano ATmega328P old-bootloader build: **29,302/30,720 flash bytes**, 781/2,048 static RAM bytes. No firmware upload or arming was performed during development.

## Validation

The native recovery harness uses production functions. It checks the plateau window, continued-acceleration reset, both wheel signs and unload-direction choices, spinning-wheel body-braking correction, the two-count command bound, settled handoff, timeout recovery and the saturation marker. Replaying the saved plateau samples flags recovery at 73.15 s, before the recorded body reversal around 75.76 s. This is an offline input replay, not a physical prediction for the modified system.

Source comparisons protect fine/hold/firing functions and constants, the FIRE completion branch, continuous first-IR control and the existing ramped-recovery implementation. Logger tests cover signed measurements, full PWM range, record integrity, interrupted writes, ring ordering, bounds/quotas, braking/recovery samples and reserved event capacity. No physical plant simulation or successful hardware test is claimed.

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_3_2/validate_recovery.py SENTINAL_HM_V31/MQ_HM_Sentinel_3_1/MQ_HM_Sentinel_3_1.ino
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_3_2/validate_logger.py MQ_Sentinel_V35/MQ_Sentinel_V35.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_3_2
```

Next test: confirm startup **MQ HM 3.2 | BODY BRAKE / HEADROOM**. Compare both approach directions and the second detection; note whether the post-scan jolt is reduced and whether recovery occurs before a long maximum-PWM stall. Save the full D dump at 115200 baud before another armed run. Minor updates advance 3.2 -> 3.3; major control revisions start 4.0. Preserve earlier releases and teammates' files.
