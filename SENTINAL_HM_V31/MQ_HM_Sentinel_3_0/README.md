# HM 3.0 — travel speed feedback, HM 2.5 precision retained

Major release because CLEAR/SEEK motor control changes from fixed duty to measured body-speed feedback. HM 2.5 remains the rollback baseline: the operator reported every shot as a perfect 10 and requested faster travel without changes to close alignment.

## Evidence and limits

The complete visible dump is preserved in `reference_run_HM_2_5/serial_dump.txt`, with decoded units alongside it. It contains 40 ring records and 19 retained events, bad_crc=0 and dropped=49. Recorded two-second completions occurred at 18.34, 78.41, 98.47, 124.64, 163.73 and 183.59 seconds. Intervals were 60.07, 20.06, 26.17, 39.09 and 19.86 seconds. They include all intervening phases and are not pure search or return times.

The retained final AIM samples show error around -4.2 to -3.36 degrees with uneven speed. Earlier direct search samples were not recorded, and event capacity was exhausted, so the full cause and phase breakdown of long delays cannot be reconstructed. The operator's observations of search/return stalls motivate this change; no external torque or motor parameters have been identified from these records.

## Travel controller

- During ordinary CLEAR/SEEK, request signed **30 deg/s body motion**. Every nominal 5 ms motor service, compare the gyro rate against that request. Rate gain is 4 PWM/(deg/s), with wheel-speed feedforward, correction capped at +/-150 PWM and a load-compensating integral bounded to +/-60 PWM. Integral gain is 3 PWM/(deg/s second). At rest with zero wheel RPM the initial correction is 120 PWM in the requested direction; continued low speed can raise it toward the 150 PWM correction limit.
- When moving too fast, rate error produces a braking correction relative to the wheel feedforward. This is a requested speed, not a guaranteed physical speed cap. Existing 60/45 deg/s search-speed guard, wheel overspeed stop, direction recovery and startup-progress checks remain.
- Existing controlled departure and its ramp-to-fixed-duty handoff remain. Search PI engages after that handoff; using a moving PI command during the old handoff could otherwise prevent its convergence check from completing. No integral accumulation occurs during departure/guard overrides.
- AIM keeps HM 2.5's requested-speed schedule: 6 deg/s beyond 20 degrees; 0.25*distance+1 between 8 and 20; min(3,0.8*distance) within 8. For error above 3.2 degrees, add the bounded load trim to the rate feedback. Correction limits become 125 PWM beyond 20 degrees and 100 between 8 and 20; the existing 65 limit remains inside 8.
- AIM trim allowance fades as `60*clamp((distance-3.2)/4.8,0,1)` PWM, reaching zero at 3.2 degrees. This reduces stored travel effort before handing off to precision control. At or inside 3.2 degrees the coarse command is numerically identical to HM 2.5.
- The integral resets on direction/mode change and on entering non-travel states. It has correction/voltage anti-windup checks and a capped integration timestep. Search and AIM do not share stale direction-dependent trim with each other or with fine hold.

These trial settings aim to reduce low-speed stalls under load, not raise every phase's top speed. Feedback cannot guarantee motion against excessive string torque, wheel saturation, wiring faults or insufficient actuator authority. Added integral effort can change arrival dynamics; physical comparison is required.

See [braking physics](BRAKING_PHYSICS.md) for the checked DRV8874 input truth table and reaction-wheel momentum exchange. Electrical wheel braking, coasting and gyro-controlled body deceleration are different operations. The new controller's overspeed response is a signed correction around wheel-speed feedforward; it does not simply turn the motor off.

## Precision scope preserved

The actual HM 2.5 `finePWM`, `fineHoldPWM`, bias learning, laser entry/maintenance, firing confirmation, two-second shot, controlled departure, scan acceptance, motor dispatcher and recovery functions are checked against source. Their logic and all existing constants match. The only addition to state entry clears the new travel-controller state; it does not change hold bias or fine-control state handling.

FINE/FIRE centering still targets zero, using the +/-0.18-degree pulse threshold with active position/rate hold inside it. Laser entry remains +/-0.9 degree, hold hysteresis +/-1.3 degrees. All motion/IR/wheel criteria remain. SLOW_SCAN retains the existing signed fixed-duty drive; its thresholds and geometry are unchanged. Different approach motion can still affect physical acquisition, so preserved source does not guarantee identical real-world accuracy.

To fit the Nano, verbose floating-point Serial lines for fixed gyro bias, arc entry and IR edge angles were shortened. Integer pulse counts, state messages, CSV and EEPROM remain; no sensor calculations or thresholds were removed.

## EEPROM

Same format-23 binary layout and **1,019/1,024 bytes**. Direct 200 ms samples now include CLEAR, SEEK and SCAN as well as AIM/FINE/FIRE, allowing the last search stall to be measured. Selected pulse samples and CRC/commit checks remain. The ring retains roughly the last 5-8 seconds of sampled activity, not the full run.

The `bias_cPWM` slot records travel integral during CLEAR/SEEK and AIM above 3.2 degrees; otherwise it records fine hold bias. Both use signed body-torque PWM units x100. The decoder labels this `control_bias_pwm`; use state/error to identify its role. Search desired rate is 30 deg/s, and the AIM desired schedule is specified above. Existing earlier-format-23 records are readable and are not deliberately cleared on upload. The current dumper banner alone does not prove which software produced preserved records; save/label the prior dump before another armed run.

## Validation

Nano ATmega328P old-bootloader build: **30,290/30,720 flash bytes**, 807/2,048 static RAM bytes. 430 flash bytes remain. Not uploaded or physically tested during development.

`validate_travel.py` checks actual production controller torque signs, slow/fast rate response, correction/trim bounds, voltage and override anti-windup, direction reset, unchanged scan duty and fixed-command ramp handoff. Numeric comparisons cover the protected <=3.2-degree coarse region across angle/rate/wheel-speed grids, including stale travel trim. Source checks verify precision functions and every existing constant against HM 2.5.

`validate_logger.py` checks signed encoding, all PWM values, CRC and interrupted writes, chronological dumps, ring/event/queue capacities, reboot persistence, search/coarse direct sampling and active-bias units, plus inherited two-second departure and bounded-jitter checks. Tests do not simulate suspension dynamics or prove a faster successful run.

From repository root:

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_3_0/validate_travel.py SENTINAL_HM_V31/MQ_HM_Sentinel_2_5/MQ_HM_Sentinel_2_5.ino
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_3_0/validate_logger.py MQ_Sentinel_V35/MQ_Sentinel_V35.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_3_0
```

Use the same physical setup and A/next-battery-boot/D-dump workflow at 115200 baud. Compare actual hit accuracy first, then completion intervals and whether stalls persist. If it stalls, stop after observing a few seconds so that phase stays in the rolling log. Save the full dump before another armed run. Minor updates now advance 3.0 -> 3.1; the next major control revision starts 4.0. Preserve HM 2.5 and teammates' files.
