# V33 startup progress test

Keep the shortened Sentinel-to-bearing string and target setup unchanged for the comparison. Keep the target stationary during acquisition.

1. If an earlier EEPROM run needs preserving, retrieve it before arming V33. Connect Serial Monitor at 115200 baud, send `D`, and save the complete dump.
2. Upload `MQ_Sentinel_V33.ino`. Send `A`, wait for `ARMED_NEXT_BOOT`, disconnect USB and fully power off, then power from the battery.
3. Observe initial SEEK. The startup watcher checks every five seconds while SEEK is being driven. If net signed yaw progress is below 4 degrees, it records a search-recovery event and changes direction through the controlled transition. Let it attempt this without a nudge if safe; cut power if the motion becomes unsafe.
4. Keep the target stationary and note the time of any reversal, first acquisition, and any intervention. Allow the run to continue if the rig remains safe.
5. Power off, reconnect USB, send `D`, and save the full EEPROM dump with observations.

Event 16 is requested by the search-direction recovery code, but V33 filters it out of EEPROM; absence from a V33 dump is not evidence that recovery did not happen. The same marker is also used by the existing post-hit wrong-direction watch, so use its timestamp and state to distinguish startup recovery from later recovery. The 4-degree/5-second threshold is experimental. It addresses low net yaw progress, not measured string torque.
