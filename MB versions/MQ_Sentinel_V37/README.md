# MB V37: Faster far return

Independent trial based on preserved [MQ Sentinel V35](../../MQ_Sentinel_V35/README.md), not a continuation of another speed trial. No physical validation yet. Use the direct-four-support-strings-to-swivel suspension of the confirmed V35 run. Separate from the MQ_HM_Sentinel family.

## Single control change

During AIM, increase requested rate above 8 degrees error: 3 deg/s at 8 degrees, linearly rising to 8 deg/s at 20 degrees, then 8 deg/s farther away. V35 uses 6 deg/s at 20 degrees and beyond. Retain V35 rate-error gains (4-7), correction limits (65/80/100), and the within-8-degree requested rate. This trial does not include V36 stronger braking.

Can shorten long travel but may increase overshoot, wheel momentum and settling. Test only after V36 results are understood; a faster request is not a hard physical speed cap.

The established fine pulse strengths, pulse timing, fine hold and laser criteria are retained. To fit Nano flash, all three trial versions share the same removal/shortening of verbose arc/recovery serial text. CSV and EEPROM data remain. Each has its own firmware/EEPROM version number; first boot with a different format version initializes storage. Retrieve old data BEFORE uploading a different trial.

## Validation

Classic Nano ATmega328P old bootloader compilation passed: 30,604 / 30,720 flash bytes; 880 static RAM bytes. Source comparisons confirmed preserved fine/hold/laser/state/logging control logic except common debug text and version labels. Braking direction/limit properties were checked across 56 model input cases; profile and retained-gain/cap properties were also checked. These software checks do not prove the physical result.

Follow the [comparison protocol](../SPEED_TRIALS.md). V35 remains the confirmed rollback baseline.
