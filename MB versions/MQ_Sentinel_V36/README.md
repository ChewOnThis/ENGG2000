# MB V36: Approach braking

Independent trial based on preserved [MQ Sentinel V35](../../MQ_Sentinel_V35/README.md), not a continuation of another speed trial. No physical validation yet. Use the direct-four-support-strings-to-swivel suspension of the confirmed V35 run. Separate from the MQ_HM_Sentinel family.

## Single control change

During AIM only, when motion is toward the target and faster than the requested rate, multiply rate-error correction by 1.5 before the existing correction clamp. Requested rate, gain schedule and maximum correction remain V35 values. At other rates and when moving away, correction remains V35.

May slow a fast arrival and shorten settling, but stronger braking can excite the suspension or chatter near thresholds. No stopping-distance calibration is assumed.

The established fine pulse strengths, pulse timing, fine hold and laser criteria are retained. To fit Nano flash, all three trial versions share the same removal/shortening of verbose arc/recovery serial text. CSV and EEPROM data remain. Each has its own firmware/EEPROM version number; first boot with a different format version initializes storage. Retrieve old data BEFORE uploading a different trial.

## Validation

Classic Nano ATmega328P old bootloader compilation passed: 30,688 / 30,720 flash bytes; 880 static RAM bytes. Source comparisons confirmed preserved fine/hold/laser/state/logging control logic except common debug text and version labels. Braking direction/limit properties were checked across 56 model input cases; profile and retained-gain/cap properties were also checked. These software checks do not prove the physical result.

Follow the [comparison protocol](../SPEED_TRIALS.md). V35 remains the confirmed rollback baseline.
