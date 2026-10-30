# MQ Sentinel V24 — confirmed repeat-hold baseline

Preserved baseline from the 30 October 2026 physical test.

The supplied event log recorded three uninterrupted laser-command holds of approximately 2.01 s, 2.00 s and 2.00 s. The operator reported successful target hits. These are commanded holds under gyro/IR gating, not independent measurements of scoring or proof of a complete five-minute demonstration.

V24 is the best confirmed repeated-hold baseline in this revision series. V26 also produced an excellent first acquisition and a two-second hold, but subsequent search failed. Later revisions should be compared against V24 rather than overwrite this source without history.

## Hardware and use

Classic Arduino Nano ATmega328P, DRV8874 in PWM mode (PMODE and nSLEEP HIGH). D5/D6 motor, D4 laser, D3/D2 encoder, D11 IR, A4/A5 MPU6050. Fixed X gyro bias: 24.3267 raw counts / 131 counts per degree/second. No startup averaging or laser self-test.

Open MQ_Sentinel_V24.ino in Arduino IDE. Serial Monitor: 115200 baud. Battery OFF, upload, send A, wait ARMED_NEXT_BOOT, disconnect USB completely, then battery ON. After the trial battery OFF, reconnect USB and send D to retrieve EEPROM events. Unarmed boots stay stationary. Changing recorder versions can initialize stored data; retrieve old logs before uploading another version.

## Known limitations

- Abrupt electrical-braking pauses at sweep completion and after a completed hold can kick the body backward; post-hit rates near -92 and -113 deg/s were logged.
- Search checks movement magnitude, not whether the body is progressing in the commanded direction. String-driven backward motion can defeat its stall detector.
- Sustained external torque and motor-model error can increase wheel speed and exhaust control headroom.
- EEPROM stores only 84 events; later events can be omitted after capacity is reached.
- The same beacon may be reacquired after a completed hold if it remains active.

Do not describe this version as fully validated or free of defects. Preserve it as a known successful targeting baseline while improving recovery separately.
