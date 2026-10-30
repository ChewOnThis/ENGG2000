# MQ Sentinel V31 — current baseline

Designated by the operator on 30 October 2026 after the four-location battery trial. V24 remains preserved separately as an earlier repeated-hold baseline.

## Verified scope and limitations

V31 retains V30 motion/firing control and adds a recorder with independent banks for 60 five-second gate summaries and 24 sparse events. The Nano build was verified with the old bootloader configuration: 29,790 bytes flash and 968 bytes static RAM.

The operator reported four target locations. The saved summaries identify distinct target references and commanded two-second holds. During the fourth location, the satellite stayed near the target for a long period without firing; the operator physically nudged it, and the subsequent attempt completed. That assisted acquisition must not be counted as autonomous four-target success.

At approximately 120–190 seconds, recorded window-end angle error was generally within +/-0.3 degrees, while interval flags reported acceptable IR, angle and wheel entry conditions. Rate was the dominant blocker. Peak filtered rates were approximately 7.2–8.0 deg/s and the longest all-entry-gates-valid intervals were only 26–36 ms, below the 85 ms firing confirmation. A commanded completed hold was recorded at about 197.98 seconds after the reported intervention.

The log does not establish whether this rate oscillation is physical yaw, sensor vibration, or both. A separate slower rate estimate for firing qualification has been proposed but is NOT implemented in this baseline. No claim of fully autonomous four-location success, validated scoring, or a flawless five-minute run is made.

## Hardware

Classic Nano ATmega328P; DRV8874 PWM mode with PMODE and nSLEEP HIGH. D5/D6 motor, D4 laser output, D3/D2 encoder, D11 active-LOW IR, A4/A5 MPU6050. Fixed X gyro bias is 24.3267 raw counts / 131 counts per deg/s. Zero motor command electrically brakes.

## Run and retrieve

Open MQ_Sentinel_V31.ino in Arduino IDE. Battery OFF; upload; open Serial Monitor at 115200 baud. Send A, wait ARMED_NEXT_BOOT, disconnect USB completely, then power from battery. At the end battery OFF, reconnect USB, send D and retain the entire dump. Unarmed boots stay stationary. Retrieve existing data before changing recorder versions, which can initialize EEPROM.

Read [TESTING.md](TESTING.md) for the four-location procedure, storage limits and summary-field decoding. The sparse event bank can fill before the end, while the summary bank remains independent. Logging stops at five minutes; motion continues until stopped. The controller uses gyro/IR evidence, not an independent beam-hit sensor.

Future control experiments should be separate revisions, compared against this file without silently replacing it.
