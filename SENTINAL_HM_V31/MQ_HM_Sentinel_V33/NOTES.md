# V33: quiet fine hold after V32 battery test

EEPROM dump transcribed from Arduino Serial Monitor: 25 summary windows and 8 events, ending at 126.89 s. The file V32-test-log.csv preserves all 33 records.

At 100–120 s, window-end errors were -0.03, -0.17, -0.26, +0.11, +0.19 degrees. Tracking-window peak filtered rates were 13.16, 13.34, 12.64, 13.79, 13.96 deg/s. Each window had a longest all-entry-gates-valid interval of only 12 ms, below the required 85 ms. Dominant blocker was rate (state_or_block 74 = FINE state 10 plus blocker 4). These are interval peaks and endpoint errors, not simultaneous samples or an oscillation waveform.

One two-second command completed at 53.66 s. A later command at 126.67 s lost lock after 220 ms. A large 126.76 deg/s tracking peak near 65 s indicates acquisition/transient trouble also occurred; near-centre changes do not claim to resolve that entire run. Logs omit individual pulse events, so the contribution of kicks versus vibration/feedback instability cannot be isolated.

V32 permitted hard fine kicks inside the aiming tolerance whenever the rate gate failed. V33 enters a pulse-free angle band at fireTolerance(), leaves only beyond tolerance+0.35 degrees, and cancels an active pulse on entry. FIRE always uses continuous hold. Small position errors have a 0.30 degree dead zone in the rate request, with continuous rate damping. Fine hold gain returns from 8 to 4; desired fine rate returns to 0.6 times dead-zone-adjusted error capped at 1.2 deg/s. Bias learning returns from KI 2 to 0.8, retaining the bounded 30 PWM authority. Outside the quiet band, pulse doublets remain; feedback braking gain drops from 10 to 4 with a 120 ms inter-pulse interval.

Coarse 8 deg/s cap, IR acquisition, fixed X bias, hardware pins, firing tolerances, 85 ms confirmation, two-second dwell, protections and V31 EEPROM layout remain as in V32. No filtering or gate relaxation hides the measured rate.

Validation: classic Nano old-bootloader build passes: 30,214/30,720 flash bytes; 969/2,048 static RAM bytes. Host execution of the actual fine-control functions passes 30 near-centre error/rate cases plus hysteresis, outside-band pulse, FIRE cancellation and position dead-zone checks. This verifies software decisions, not physical settling. No upload or battery validation performed.

Use the usual A / next battery boot / D workflow. Compare completed unassisted holds and gate-valid intervals with the saved V32 dump. V31 and V32 sources are preserved.
