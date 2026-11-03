# MQ Sentinel V33

V33 keeps the V32 aiming, firing, hold, and post-hit control. Before the first completed two-second hold, initial SEEK now checks signed yaw progress in five-second windows. When the motor is being driven and net progress is under 4 degrees, it uses the existing controlled transition to reverse search. This startup-only watcher is disabled after the first completed hold.

The thresholds are a first physical trial setting. They have not been independently validated against string torque or all starting positions. The code cannot guarantee recovery from a mechanical obstruction.

## Test record

The first V33 EEPROM dump recorded five `TARGET_HOLD_COMPLETE` events at 11.60, 47.23, 80.47, 134.22, and 171.64 seconds. These are firmware laser-command holds; EEPROM cannot verify the external beam position or score. No startup no-progress reversal event was recorded in this run, so the new recovery condition did not trigger during the logged run. Review the dump and operator observations together before drawing conclusions about the startup change.

The classic Nano build (ATmega328P, old bootloader) uses 30,702 of 30,720 flash bytes and 985 bytes of static RAM. Flash headroom is only 18 bytes.

See [test procedure](TESTING.md) for arming, startup observations, and EEPROM retrieval. V31 remains the operator-selected baseline; V33 is an experimental iteration.
