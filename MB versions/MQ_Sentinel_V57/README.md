# V57: progress-aware return

Based on MB V56 Recovery (b93c13f). Physical performance is untested; V56 and other baselines are preserved.

- FINE returns to AIM whenever absolute error exceeds 4.4 degrees, without requiring body rate below 2 degrees/second. Existing continuous PWM state and slew limits are preserved.
- Remove the fixed nine-second AIM abandonment. Existing firing gates still mature with elapsed time; they are not relaxed further for long attempts.
- Sample absolute target error every second. Improvement of at least 0.20 degrees resets the nonprogress timer, even after springback and before recovering the previous best position. Retry/reverse after four seconds without such progress when error exceeds six degrees, or six seconds otherwise. A centred hold (error <=0.60 degrees and predicted error <=0.80 degrees) resets the timer. Existing wheel/headroom recovery remains.
- Progress recovery logs result 5 (TRACK_RECOVERY), with the existing gate diagnostic cause. It discards that acquisition and reverses search; it does not latch STOP.
- Nine control functions were compared with V56 Recovery and remain identical: coarsePWM, fineHoldPWM, updateHoldBias, updateQuietHold, updateLaserArming, fireGateForAge, fireAbortCause, motorUpdate, reverseForSaturation.

## Logging

Banner and dump identify V57. Compact CSV names save Nano flash. Field order, units, 22-byte EEPROM layout and CRC remain unchanged:

`run,detect_ms,gap_ms,scan_ms,aim_ms,laser_ms,arc_cdeg,error_cdeg,score_ms,peak_rpm,result,cause`

`error_cdeg` remains signed error at laser activation for shots. For a V57 no-fire acquisition ending with a valid aimed target, it now records signed terminal error. Other no-fire records can still contain zero/unset. Older records retained in EEPROM keep their original semantics; a V57 dump heading does not reclassify historical records. No instantaneous rate trace was added. The reversal footer is now REVERSALS_TOTAL (persistent cumulative counter).

## Validation and use

Nano ATmega328P old-bootloader compile: 30,656 / 30,720 flash bytes; 823 / 2,048 static RAM bytes. Source invariance checks and five synthetic progress-timer scenarios passed (long convergence, far stall, small-error stall, centred hold, springback followed by convergence). These are software checks, not a physical plant test.

Upload MQ_Sentinel_V57.ino using the existing Nano settings. POWER/RESET starts automatically after BOOT checks. D dumps and stops; S stops; C clears and stops; reset restarts. Automatic sensor retry, wheel unloading, manual stop and laser watchdog are retained. Creating/opening this version does not upload it to the board.

For the next run, observe the first acquisition and the first string-limited reversal, then collect D. Compare physical hits separately from completed laser commands. A stall should reverse and reacquire; a useful longer return should continue past nine seconds. A one-second net-progress measure cannot distinguish all oscillations or identify string torque; this remains a trial setting.
