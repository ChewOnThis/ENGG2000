# Speed opportunities after the confirmed V35 run

Preserve this firmware and direct-four-string suspension as the rollback baseline. Avoid adding a software pointing offset before mechanical laser alignment is checked.

## Timing evidence

Completion intervals, including the final required two-second hold, were 11.40 seconds from startup, then 32.33, 15.23, 26.28, and 23.92 seconds. These include search, scan, recovery, return, settling, and target handling; they are not pure return-controller times.

The first saved scan event was at 3.33 seconds and its completed hold at 11.40, an 8.07-second scan-to-completion interval. From the first completion to the next saved scan event (27.58 seconds) was 16.18 seconds, including search/scan/recovery. That scan-to-completion interval was another 16.15 seconds. This shows that search and return/settling both contribute meaningful time. The logger omits later scan events once the sparse-event quota fills, so an exact full-run phase breakdown is unavailable.

## Priority 1: reduce fast arrival and settling

At 30 seconds, error was +29.64 degrees and rate +10.54 deg/s, approaching faster than the 6 deg/s request. At 35 seconds error was +9.92 degrees and rate +8.95 deg/s, above the approximately 3.48 deg/s request. Early laser entries were also interrupted by the existing fast-rate gate. The snapshots do not identify whether spring energy, estimator error, limited actuator authority, or another disturbance caused the excess rate.

The next candidate should address early deceleration based on remaining angle AND closing rate. A stopping-distance/velocity envelope or a bounded predictive angle may reduce arrival overshoot; it requires a measured, conservative deceleration estimate. Five-second samples cannot establish that estimate reliably. Do not increase PWM globally or relax the laser-rate gate to mask physical motion.

## Priority 2: search and departure

A confirmed wrong-direction event occurred at 22.50 seconds, after the first successful hold: body rate -4.72 deg/s while negative motor drive intended positive body rotation. The existing controlled reversal reacted. This suggests search-direction/recovery contributes delay, but does not prove its 400 ms confirmation should be shortened. Capture direction/progress transitions before tuning departure or search timers.

## Priority 3: improve the tuning evidence

Event 22 now exposes actual signed rate, wheel RPM and motor output during AIM. Use targeted, more frequent return samples around acquisition and deceleration in the next diagnostic iteration to distinguish persistent low-speed load from a fast spring release. EEPROM and flash are limited, so budget those records rather than adding unrestricted logs. Keep fine-pulse sizes and the successful final hold unchanged for the first comparison.

No next-version firmware has been implemented by this review.
