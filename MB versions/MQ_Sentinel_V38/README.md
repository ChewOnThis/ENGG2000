# MB V38: Quicker search reversal

Independent trial based on preserved [MQ Sentinel V35](../../MQ_Sentinel_V35/README.md), not a continuation of another speed trial. No physical validation yet. Use the direct-four-support-strings-to-swivel suspension of the confirmed V35 run. Separate from the MQ_HM_Sentinel family.

## Single control change

Reduce WRONG_SEARCH_CONFIRM_MS from 400 to 250 ms. This affects the existing post-hit wrong-direction monitor in CLEAR/SEEK/SCAN only when its signed-drive and speed-guard conditions qualify. Initial startup no-progress watcher and AIM remain V35. This trial includes neither V36 nor V37.

May start recovery 150 ms sooner after a qualifying continuous backward interval, but can reverse unnecessarily on shorter disturbances. Potential saving per event is modest; it does not remove all search delays.

The established fine pulse strengths, pulse timing, fine hold and laser criteria are retained. To fit Nano flash, all three trial versions share the same removal/shortening of verbose arc/recovery serial text. CSV and EEPROM data remain. Each has its own firmware/EEPROM version number; first boot with a different format version initializes storage. Retrieve old data BEFORE uploading a different trial.

## Validation

Classic Nano ATmega328P old bootloader compilation passed: 30,542 / 30,720 flash bytes; 880 static RAM bytes. Source comparisons confirmed preserved fine/hold/laser/state/logging control logic except common debug text and version labels. Braking direction/limit properties were checked across 56 model input cases; profile and retained-gain/cap properties were also checked. These software checks do not prove the physical result.

Follow the [comparison protocol](../SPEED_TRIALS.md). V35 remains the confirmed rollback baseline.
