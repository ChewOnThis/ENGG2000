# HM 2.4 — reliable dwell and departure

Minor update from HM 2.3 using the saved Nano test. V35 motor control and fine centering are retained. No return-speed increase, position deadband or output slew is introduced.

## What the recorded run establishes

- Laser-on was recorded at 10.24 s. A successful continuous two-second command completed at 21.26 s. The actual final successful command therefore began around 19.26 s; the earlier onset did not mean the laser stayed on for eleven seconds. Intermediate transitions were not preserved.
- HM 2.3 then intentionally stayed in FIRE with its laser off after completion. That single-target test behavior explains why it did not resume searching. It was not evidence of a disabled motor or a missed completion. HM 2.4 restores the V35 controlled departure after a successful dwell.
- At approximately 41.6-45.6 s, sampled angle error stayed around -0.14 to +0.33 degree. Faster filtered rates reached about +/-6 deg/s while the slower pointing rate was generally within +/-1.5 deg/s. This supports treating bounded, centred ripple differently from sustained drift; it does not prove all fast motion is harmless sensor noise.
- IR became stale around 45.84 s, followed by FINE and a larger position/rate disturbance. The dump does not establish its physical cause or why each early laser attempt ended. It contained 40 samples, 15 events, one rejected record and seven dropped records. Twelve early speed-guard messages consumed most ordinary event capacity.

## Changes

1. Entry angle window increases from +/-0.60 to **+/-0.90 degree**. Hold angle window increases from +/-0.80 to **+/-1.30 degrees**. This trades some acceptance precision for reliable dwell; it does not change the desired centre or add a pointing offset.
2. During FIRE only, faster-rate ripple between 4.3 and 8 deg/s is tolerated while the angle remains inside +/-0.90 degree and the ready slower pointing-rate estimate remains <=1.5 deg/s. Leaving this narrower position/rate combination restores the existing 40 ms confirmation for faster rate >4.3 deg/s. Faster rate >=8 deg/s still cuts immediately. IR, valid reference, calibration and <=185 wheel RPM remain required. Persistent IR loss is not masked.
3. After a continuous two-second command, record completion, turn the laser off and call the existing controlled departure, then clear/seek the next target. The deliberate HM 2.3 single-target latch is removed. Interrupted holds still return to FINE and must qualify again before starting a new two-second command.
4. Store at most the first two speed-guard transitions per run. This intentional diagnostic filter prevents that repeating pair from occupying all ordinary event slots and leaves more room for laser on/off markers. Queue/storage drops are still counted. Event capacity is finite; the filter is not a guarantee of every transition being stored.

Entry motion limits remain V35's pointing rate <=3.2 deg/s and faster filtered rate <8 deg/s for 85 ms. Bounded jitter can therefore be accepted; this is not an exact-zero-velocity requirement. The two-second startup remains.

## Logging and validation

Same 1,019/1,024-byte EEPROM layout, record units, commit markers, CRC and rolling diagnostics as [HM 2.3](../MQ_HM_Sentinel_2_3/README.md). Format number remains 23 so the earlier saved log is not deliberately cleared on upload. The dump marker now reports HM2.4,format=23; that identifies the current dumper, not proof that preserved old records were produced by HM 2.4. Save/label old data before arming a new run. The decoder accepts both HM2.3 and HM2.4 dump markers.

Nano ATmega328P old bootloader build: **29,568/30,720 flash bytes**, 810/2,048 static RAM bytes. Not uploaded or physically tested during this update.

The native harness checks the production FIRE branch: no departure at 1,999 ms, departure and laser-off at 2,000 ms, and lock-loss interruption without a false completion. It checks the centred-ripple exception, original outer-window rate timer, immediate hard-rate cutoff, IR/angle/wheel rejection, EEPROM integrity/ordering/bounds, and speed-event quota. V35 motor-control function comparisons and the direct-unit decoder checks also pass. These tests do not simulate the real suspension or guarantee hit accuracy.

From repository root:

```
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_4/validate.py MQ_Sentinel_V35/MQ_Sentinel_V35.ino
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old SENTINAL_HM_V31/MQ_HM_Sentinel_2_4
python3 SENTINAL_HM_V31/MQ_HM_Sentinel_2_4/decode_log.py dump.txt > decoded.csv
```

Next test: check whether each laser activation lasts two continuous seconds, then the Sentinel leaves and seeks again. Note actual aim accuracy within the wider hold window. After power-off, reconnect and send D at 115200 baud; save the complete dump before another armed run. If the IDE does not respond, restart only the serial connection; an empty monitor is not evidence of empty EEPROM.

Minor updates advance 2.4 -> 2.5; major control changes start 3.0. Preserve previous releases and teammates' files.
