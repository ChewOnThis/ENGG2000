# HM 2.3 — V35 rollback and single-target accuracy test

HM 2.2 was reported to spin without lining up; no EEPROM was captured for that run. Its cause is unproven. This release discards the HM 2.1/2.2 controller changes and starts from main/MQ_Sentinel_V35, blob c89f56c380051ff01768095778f6af440603c575. The operator accepts small jitter provided aim is accurate and the laser activates.

## Targeting

- V35 search/scan, coarse return, fine pulse strengths/timing, hold damping and bias learning are restored. No HM 2.2 voltage slew or position deadband; no HM 2.1 rate-gain increase or stricter firing-rate thresholds.
- Firing angle tolerance is fixed at +/-0.60 degree about the measured IR arc midpoint, versus the earlier adaptive 0.85-2.20 degrees. Hold angle margin is 0.20 degree, so a firing hold must remain within +/-0.80 degree. This improves the software acceptance criterion; it does not establish physical bullseye accuracy or correct mechanical laser alignment.
- V35 motion criteria remain: entry pointing-rate estimate <=3.2 deg/s, faster filtered rate <8 deg/s, 85 ms confirmation. Hold faster rate <=4.3 deg/s, 40 ms excursion confirmation, immediate cutoff at >=8 deg/s. IR validity and wheel-speed checks remain. Small corrective jitter is allowed; zero physical velocity is not guaranteed.
- After a continuous two-second laser command, laser turns off and the controller continues holding the same reference in FIRE instead of deliberately departing to search for another target. If lock is lost, normal fine reacquisition resumes; another confirmed lock can fire again. Safety and failed-tracking recovery still apply. This is an intentional single-target accuracy test, not the repeated-target production mode.
- Two-second startup and explicit A/next-battery-boot workflow remain. No speed increase is included. Search and return motor control match the known V35 source; a startup/recovery problem still requires physical diagnosis.

## EEPROM: explicit direct measurements

New format 23, labelled EVENT_LOG_BEGIN,HM2.3. It cannot be decoded as V35 packed windows. Existing earlier-format bytes are preserved at an unarmed boot; D reports that the previous sketch is needed to dump them. Sending A authorizes initializing the new format; the next armed boot starts a new log. Save any needed earlier dump before doing that.

Nano EEPROM usage is statically checked: 16-byte header + (40 rolling samples + 19 retained events) x 17 bytes = **1,019/1,024 bytes**. No storage expansion is assumed. A four-record RAM queue and nonblocking byte writes keep control servicing active. Writes pause during fine pulses.

Every 200 ms during AIM/FINE/FIRE, a direct sample is queued. A selected pulse start/end pair is also sampled at most once per second. The ring preserves the most recent roughly 5-8 seconds of targeting diagnostics, rather than all samples for the whole test. Earlier sample history is overwritten deliberately. The event bank retains scan acceptance, laser transitions, completion, fault and recovery markers; noncritical events can use 14 slots, reserving five for completion/fault/end. Event capacity and queue overflow can still lose events; dropped counts are reported and persisted with completed record writes. A power cut before a metadata update can lose the latest drop count.

Each record contains time, angle error, faster filtered body rate, slower pointing-rate estimate, wheel RPM, exact signed applied PWM, integral hold bias, state, event, firing-entry flags and fault code. Values are direct signed quantities, not packed maxima or overloaded gate-window fields. Invalid-reference angle error is stored as zero with reference_valid false. Rates are filtered gyro estimates, not external measurements of laser motion. Pulse events are selected, not an exhaustive trace.

Dump columns:

| Field | Meaning / scale |
| --- | --- |
| time_ms | Time since recording began; 10 ms resolution |
| event | 2 scan accepted; 3/4 selected pulse start/end; 5/6 laser on/off; 9 fault; 10 log end; 14 completed hold; 15 recovery; 16 wrong search; 18 scan rejection; 19/20 speed guard on/off; 22 direct sample |
| state | 9 AIM; 10 FINE; 11 FIRE; 13 STOP; other V35 state numbers unchanged |
| error_cdeg | (IR midpoint minus integrated gyro angle) x 100 |
| fast_cdps | Faster filtered gyro body rate x 100 deg/s |
| pointing_cdps | 150 ms filtered pointing-rate estimate x 100 deg/s |
| wheel_rpm_d10 | Signed filtered encoder wheel RPM x 10 |
| pwm | Exact signed applied motor PWM, -255 through +255 |
| bias_cPWM | Learned hold bias x 100 PWM |
| flags | Bits 1 recent IR, 2 valid reference, 4 laser commanded HIGH, 8 entry-angle pass, 16 entry-rate pass, 32 wheel pass, 64 pulse active, 128 calibrated |
| stop | 0 normal, 1 I2C, 2 wheel overspeed, 3 gyro/acceleration/timing safety stop |

Flag bits describe the entry criteria at the sample instant, not the separate wider FIRE maintenance criteria. FIRE state can remain after laser turns off; inspect laser_high. Every record is invalidated before overwrite, written with CRC-8, then committed. Interrupted writes are rejected. CRC also detects corruption; it is an integrity check, not a physical sensor calibration. Dump merges valid ring/event records by timestamp, reports bad_crc and dropped, and remains visible because idle telemetry is suppressed. A cut during a record can lose that record. A full ring's order is reconstructed on reconnect to cover a cut before pointer metadata is committed.

## Validation

Nano ATmega328P old bootloader compile: **29,042/30,720 flash bytes**, 809/2,048 static RAM bytes (final build should match the included source). Not uploaded or physically tested as part of development. Hardware EEPROM timing, supply failure and real targeting still require validation on the Nano.

The native harness extracts production functions rather than duplicating the logger. It checks signed measurements, all 511 PWM values, ring wrap, reboot persistence, chronological dumping, interrupted write stages, single-bit record corruption, event reservation, queue overflow, pulse-time write deferral and address bounds. Source comparisons verify restoration of V35 fine/hold/coarse/recovery functions. It does not simulate suspension dynamics or prove bullseye accuracy.

From repository root:

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_3/validate.py MQ_Sentinel_V35/MQ_Sentinel_V35.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_2_3
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_3/decode_log.py dump.txt > decoded.csv
```

Python 3 and clang++ are needed for the harness. The decoder accepts only explicitly marked HM2.3 dumps and writes clear physical units.

## Next physical test

Keep one target stationary and use the same suspension. Upload HM 2.3, open Serial Monitor at 115200, send A and wait for ARMED_NEXT_BOOT, then power off/unplug USB and start from battery. Observe whether it aims accurately and produces a full two-second laser command; small jitter is acceptable. It should keep holding that reference after the shot. If it spins without accepting a scan, note that separately from failed fine centering. Power off, reconnect, send D, and save the complete EVENT_LOG_BEGIN to EVENT_LOG_END output before another armed run. Check samples >0, bad_crc=0 and dropped=0; nonzero counts need interpretation, not concealment. The last few seconds before power-off are the detailed tuning segment.

Minor version changes advance 2.3 -> 2.4; major control changes start 3.0. Keep prior releases and teammates' files intact on main.
