# MB V39: Observable laser gate

Independent diagnostic revision based on V38.

## Changes from V38

- Adds compact `G,<pointingRate>` diagnostics. This is the low-pass pointing-rate value used by the laser-entry gate but previously hidden.
- Reports `G,<pointingRate>` periodically whenever laser entry is rejected.
- Uses the stricter of `yawRateFiltered` and `pointingRate` for post-fire lock validation, matching the entry gate rather than allowing a looser post-fire rate.
- Keeps the V38 search-reversal change (`WRONG_SEARCH_CONFIRM_MS=250`) and all established mechanical, IR, fine-control, and laser timing settings.

Gate diagnostic bitmask:

The CSV fields identify the remaining entry conditions: IR, angle, filtered rate, and wheel RPM.

The CSV format is unchanged from v38. This revision does not claim that a laser output driver or diode is electrically functional; verify `LASER_D4_HIGH` and measure pin D4 if the gate is satisfied.

## Startup

The safety interlock is retained. A fresh upload or EEPROM format change starts in `STOP` and prints `IDLE,A=arm_next_boot,D=dump`. Send uppercase `A` in the Serial Monitor, power-cycle the Nano, and confirm that it prints `RECORDING,60_windows_plus_24_events` before expecting the motor to run.

## Validation

Use the [comparison protocol](../SPEED_TRIALS.md). V35 remains the confirmed rollback baseline.
