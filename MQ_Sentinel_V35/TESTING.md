# V35: return effort follows remaining error

Physically tested with five operator-confirmed target hits. The four support strings connect directly to the fishing swivel; the central vertical string has been bypassed. This suspension arrangement is part of the tested baseline. Earlier V33 versions remain preserved.

## Return settings for the coder

AIM now uses:

| Remaining error | Requested body rate | Rate-error gain | Correction limit |
| --- | --- | --- | --- |
| Above 20 deg | 6 deg/s | 7 PWM per deg/s | 100 PWM |
| 8-20 deg | 0.25*error + 1 (3-6 deg/s) | requested rate + 1 (4-7) | 80 PWM |
| Within 8 deg | min(3, 0.8*error) | 4 | 65 PWM |

Direction follows the sign of angle error. The correction is gain*(requested signed rate - measured signed rate), added to the existing wheel back-EMF estimate. These are correction limits, not constant motor duties. At rest far away, initial correction is 42 PWM versus V33's 12. When approaching too fast the feedback reverses the correction to brake. Rate targets are continuous at 8 and 20 degrees, though correction limits change by band. The rate target is not a guaranteed physical speed cap.

These thresholds are trial values, not parameters identified from the five-second logs. This is intended to make large returns stronger while retaining velocity damping. It does not guarantee overcoming tether torque or stiction.

Fine entry, fine pulse sizes/timing, hold bias, and laser criteria are preserved from V33. Fine pulses remain 65 PWM at >=1 deg error, 45 below 1 deg, 5-20 ms with 180 ms minimum spacing. The unintended V33 post-hit direction-watch qualification is restored: correctly signed applied drive >=80 is required. The startup net-progress watcher remains.

## Logging

V35 uses a new EEPROM format version. Save any earlier-version dump BEFORE uploading; V35 initializes storage if the stored format differs. Re-uploading the same format does not itself clear an existing V35 log.

Every five seconds while state is AIM, event 22 replaces the ordinary gate window and stores direct values:

    time_ms,22,state,error_cdeg,signed_fast_rate_cdps,wheel_rpm_d10,applied_pwm,instant_flags

Flags: 1 fresh IR, 2 valid target reference, 4 laser commanded HIGH. PWM has two-count resolution. This event uses the existing 60-record summary bank, not extra EEPROM. Other states retain event 21 gate summaries and packed peak rates. Event 16 is now actually permitted into the sparse bank (first three occurrences), allowing startup/post-hit search recovery to be seen, subject to queue/capacity limits.

Verbose state/pulse/lock-candidate serial prints were trimmed to fit flash. CSV telemetry and EEPROM remain. Compile: Nano ATmega328P old bootloader, 30,684 / 30,720 flash bytes, 880 static RAM bytes. Software checks verified preserved fine/hold/laser logic, restored direction eligibility, and the bounded desired-rate schedule. This does not replace a physical test.

## Run

Use the tested four-string direct-to-swivel suspension and keep the target stationary during each acquisition. Upload V35, open Serial Monitor at 115200 baud, send A, wait ARMED_NEXT_BOOT, fully power off and unplug USB, then battery on. After the run, power off, reconnect USB and send D. Note any intervention and whether springback or overshoot increased. Stop by cutting power if motion becomes unsafe.
