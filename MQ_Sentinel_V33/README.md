# MQ Sentinel V33

V33 keeps the V32 aiming, firing, hold, and post-hit control. Before the first completed two-second hold, initial SEEK now checks signed yaw progress in five-second windows. When the motor is being driven and net progress is under 4 degrees, it uses the existing controlled transition to reverse search. This startup-only watcher is disabled after the first completed hold.

The thresholds are a first physical trial setting. They have not been independently validated against string torque or all starting positions. The code cannot guarantee recovery from a mechanical obstruction.

## Test record

The first V33 EEPROM dump recorded five `TARGET_HOLD_COMPLETE` events at 11.60, 47.23, 80.47, 134.22, and 171.64 seconds. These are firmware laser-command holds; EEPROM cannot verify the external beam position or score. The EEPROM whitelist excluded event 16, so absence of a startup recovery marker cannot establish whether it triggered. Review the dump and operator observations together before drawing conclusions about the startup change.

The classic Nano build (ATmega328P, old bootloader) uses 30,702 of 30,720 flash bytes and 985 bytes of static RAM. Flash headroom is only 18 bytes.

See [test procedure](TESTING.md) for arming, startup observations, and EEPROM retrieval. V35 is the newer confirmed baseline; V33 remains preserved as an earlier experiment.

## Source-review correction

V33 unintentionally relaxed post-hit wrong-direction eligibility from correctly signed applied PWM >=80 to any nonzero PWM. Its scope was therefore broader than startup-only. V35 restores the intended eligibility. The tested V33 source remains unchanged.
