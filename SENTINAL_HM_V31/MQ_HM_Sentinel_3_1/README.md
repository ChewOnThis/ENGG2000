# HM 3.1 — continuous control through IR entry

Publish the previously local HM 3.1 transition fix, now with a recorded-jolt regression check. This release does not add an unrelated forced search reversal.

## What the visible dump shows

The current IDE was displaying HM 3.0, and the relevant dump marker was EVENT_LOG_BEGIN,HM3.0,format=23. Earlier unrelated firmware output was also present in the buffer; `reference_excerpt/observed_rows.csv` includes only selected rows from this HM3.0-labelled dump. It is explicitly an excerpt, not a complete log. Its reported footer was samples=40,events=14,bad_crc=0,dropped=1. Format-23 records do not embed their producer version, so the dumper label alone cannot prove which firmware generated them; the recorded scan command does match HM 3.0's fixed scan duty.

There was a successful two-second command at 19.23 seconds. Later SEEK samples track approximately 30 deg/s, rather than showing persistent zero search speed. At 46.38 s, SEEK has body rate +32.79 deg/s, wheel +123.8 RPM, PWM -203 and travel bias +13.09 PWM. At 46.58 s, SCAN has PWM -100, bias zero and body rate -35.50 deg/s; at 46.73 s the scan is rejected and recovery begins. Similar fixed-duty scan entries and wheel/body reversals appear around 41.85 and 51.04 s.

This is consistent with the search-to-scan command discontinuity braking the wheel and disturbing the body, then triggering a repeated rejected-scan loop. It does not establish every external disturbance or all possible jolt causes. Adding a new abrupt direction flip would not address this recorded transition.

## Fix

- Preserve the travel controller's load trim and direction/mode key when entering SLOW_SCAN.
- Run the same 30 deg/s feedback controller in SLOW_SCAN as in SEEK/CLEAR, rather than changing to fixed PWM 100 at first IR.
- Retain the existing fixed-command controlled-departure handoff. If IR is encountered during that ramp, both sides of the transition still use that same ramp command.

These are the same functional changes already made in the local HM 3.1 sketch. This publication adds verification and evidence, not another speculative reversal strategy. Existing low-progress and wrong-direction recoveries remain controlled transitions. HM 3.0 travel gains, coarse approach, EEPROM layout, HM 2.5 fine alignment and shooting logic remain. A continuous command removes the artificial step; it does not guarantee constant physical speed or perfect acquisition under all loads.

## Validation and next test

Nano ATmega328P old-bootloader compile: **30,298/30,720 flash bytes**, 807/2,048 static RAM bytes. Physical behavior still needs testing with the confirmed HM 3.1 image.

The native harness extracts the actual `firstIrEncounter`, state-entry routine and travel controller. It replays the recorded wheel/body state and its mirrored direction, verifying no travel-trim reset and no commanded-voltage step at first IR. It also checks continuity during the departure ramp, preserves fine/firing functions and existing constants against HM 2.5, compares travel gains/coarse/EEPROM logic against HM 3.0, and retains braking-direction and hardware pin-mapping checks. This is not a physical plant simulation.

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_3_1/validate_transition.py SENTINAL_HM_V31/MQ_HM_Sentinel_2_5/MQ_HM_Sentinel_2_5.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_3_1
```

Before the next battery test, confirm the Nano startup banner says **MQ HM 3.1 | CONTINUOUS IR SCAN**. An open editor alone does not prove the uploaded firmware version. The Nano was disconnected during this publication; no firmware upload or arming was performed. Use the established A/next-boot/D workflow, and save the full dump after observing whether IR entry stays smooth and scans complete. The decoder accepts HM 3.1 and earlier compatible dump markers.

Format-23 EEPROM remains 1,019/1,024 bytes. Earlier saved logs are not deliberately erased on upload. Label old dumps before another armed run. Minor updates advance 3.1 -> 3.2; major changes start 4.0. Preserve HM 2.5 and teammates' files.
