# MB V56 Recovery: automatic retry demo candidate

Based on the repository-root `V56` file (commit 07cb408). That original file is preserved. This copy removes automatic latched motor STOP behavior; it is not physically tested. Compilation and software checks do not establish demo reliability.

## Automatic recovery

- Wheel speed: at 190 filtered RPM or 220 instantaneous RPM, discard the old target, reverse search direction and unload the wheel. Existing early SEEK wheel-management can also request the same unloading path. Do not continue the ordinary wheel-speed feed-forward during unloading: at high wheel speed that estimate can demand full drive even when the body is trapped.
- Unloading ramps from the last applied PWM toward 65 PWM opposite the wheel's current drive direction, at 600 PWM units/s. It resumes CLEAR/search when filtered wheel speed has remained <=80 RPM for 50 ms, or after the bounded 1500 ms attempt. The timeout resumes search rather than latching STOP. Continued saturation can trigger another recovery.
- Body stall: all `recoverStall()` calls now reverse on the first attempt instead of first retrying in the same direction.
- AIM/FINE no progress: re-enable the existing four-second nonprogress/headroom recovery, which reverses and retries. The nine-second acquisition deadline also reverses and retries instead of departing in the same direction. These end an individual target attempt; they do not stop the program.
- I2C/gyro/timing/acceleration faults: motor and laser temporarily switch off, old target is invalidated and the controller enters BOOT recovery. Every 500 ms it retries MPU configuration and checks a valid sample before restarting with the fixed bias. It automatically resumes when valid data returns; a permanently failed sensor necessarily prevents controlled motion. Integrated yaw resets and a new target scan is required.
- Unexpected controller states use recovery rather than a latched STOP.

## Retained controls

Explicit Serial `D` (dump), `S` (stop), and `C` (clear log) stop the rig and remain stopped until reset. The laser aim/quality gates and hardware maximum-on watchdog remain active; ending an unsafe laser attempt continues acquisition rather than stopping the whole program. The old logger outcome named `SENSOR_STOP` is retained for compatibility, but in this variant it records entry to automatic sensor recovery, not a permanent STOP.

V56's normal startup is retained: POWER/RESET STARTS THE RUN after the BOOT checks. This is not the old A/armed-next-boot workflow. Keep clear of the moving rig on reset. No board upload is performed by creating/opening this sketch.

## Validation

Classic Nano ATmega328P old bootloader compile passed: 30,452 / 30,720 flash bytes, 819 static RAM bytes. Source comparison confirmed unchanged coarse/fine control, quiet hold, laser arming/abort gates, watchdog and EEPROM layout. Static check confirmed only the explicit serial command function enters STOP. Software checks covered unloading polarity, the 3-PWM/5-ms slew bound, convergence for both wheel directions and several initial outputs, plus recovery re-entry exclusion in BOOT/UNWIND/STOP. No physical test or new saturation measurements were available, so thresholds and unloading behavior remain trial settings.
