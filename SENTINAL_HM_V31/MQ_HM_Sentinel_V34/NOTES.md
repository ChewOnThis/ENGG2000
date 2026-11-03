# HM V34: restore non-HM V31 near-target control

Operator reports HM V33 still stops well but jitters on target; original non-HM V31 did not show that behavior. The latest test was touched while on target at an unknown time. Per operator instruction, that EEPROM run is excluded from tuning and causal conclusions.

Comparison against the local non-HM MQ_Sentinel_V31/MQ_Sentinel_V31.ino shows HM V33 changed the fine requested-rate dead zone, hold-bias limit, pulse gating and inter-pulse damping, as well as the coarse approach slope. These are plausible regression contributors; code comparison alone cannot establish physical cause. Wheel-speed feedforward, RPM sampling and gyro filtering are shared with V31 and are not unique HM regressions.

V34 is generated from non-HM V31. Its only behavioral changes are the coarse speed cap (3 to 8 deg/s) and routine telemetry interval (10 to 50 ms). V31's 0.8 error-to-rate slope is preserved: requested rates are identical to V31 within 3.75 degrees, including the 3.2-degree fine-entry region. Farther away, the higher cap retains faster travel. Original fineHoldPWM, updateHoldBias and finePWM are restored byte-for-byte, along with the original 20 PWM bias limit and 180 ms pulse spacing. V31's original fine pulses can occur inside angle tolerance when other centring gates fail; V34 intentionally restores that behavior for comparison instead of claiming to fix it. No additional gains or altered firing gates are introduced.

Fixed X bias, current pins, scan logic, firing tolerances and two-second dwell, A/D workflow, speed guard, protections and V31 EEPROM layout are retained. This is a controlled comparison candidate, not a demonstrated jitter fix. Returning from far away can still arrive with different momentum despite the same near-target request.

Validation: exact source comparison verifies restored fine functions; close-approach request equivalence checked through 3.75 degrees. Classic Nano old-bootloader compilation succeeds. No hardware upload or physical validation performed. Compare an untouched run with V31 and V33; if jitter persists, the suspension change or unchanged feedback dynamics remain possible causes.
