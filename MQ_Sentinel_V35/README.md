# MQ Sentinel V35: confirmed five-hit baseline

V35 adds a return controller whose requested angular speed and correction effort depend on remaining error. It preserves the established final fine-pulse and hold behavior and restores the correctly signed-drive check accidentally relaxed in V33. See [settings and workflow](TESTING.md).

## Confirmed setup and result

The fishing swivel connects directly to the four support strings from the Sentinel, bypassing the short central vertical string. In the supplied V35 EEPROM run, the firmware recorded five completed two-second laser commands at 11.40, 43.73, 58.96, 85.24, and 109.16 seconds. The operator confirmed all five landed correctly, with small offsets on a couple believed to be mechanical laser alignment. That alignment cause remains unverified.

Five holds were completed in 109.16 seconds (21.83 seconds per hold including startup; 24.44 seconds average between subsequent completions). The earlier V33 run completed five by 171.64 seconds, but firmware and suspension both changed, so this is not an isolated software comparison. Repeated completions do not establish five unique target identities: the firmware has no target-ID tracking.

V35 logs direct AIM samples as event 22 and permits the first three search-direction recovery markers (event 16), subject to capacity. One post-hit wrong-direction event occurred at 22.50 seconds; no fault event was recorded. At 100 seconds a positive 13.38-degree error coincided with -6.33 deg/s body motion, consistent with moving away from the target while the controller corrected toward it.

The tested source is preserved here without firmware edits. Nano old-bootloader build: 30,684 of 30,720 flash bytes, 880 static RAM bytes. Only 36 flash bytes remain. Further changes will require reducing code size or diagnostic output.

This MQ_Sentinel_V35 is a separate experiment from the MQ_HM_Sentinel_V35 and V36 folders under SENTINAL_HM_V31. Version numbers across those families do not identify equivalent firmware.

See [speed review](SPEED_REVIEW.md) for possible next work; no further tuning is included in this baseline.
