# V31: V30 control, redesigned recorder

V30 source is preserved. V31 changes logging only: no motion tuning, speed-guard thresholds, laser gates or completion/recovery rules change. Recorder workload is different, so execution overhead cannot be identical, but EEPROM programming remains nonblocking and deferred during fine drive pulses.

## Test

1. Retrieve any old EEPROM dump before uploading; V31 uses a new layout/version.
2. Battery OFF, upload MQ_Sentinel_V31.ino. Serial Monitor 115200. Unarmed USB boots stay stationary.
3. Send A, wait ARMED_NEXT_BOOT, unplug USB fully, then battery ON.
4. Repeat the four-location test, keeping each location stationary during acquisition. After a completed two-second emission, turn the beacon off, relocate it, then turn it on. If it sits centred without firing, allow 10–15 seconds of that behavior to be recorded before moving on. Do not reposition the satellite. Note approximate beacon-on times/locations if practical.
5. Finish within five minutes; logging ends at five minutes, but motion continues. Stop earlier if winding/motion requires it.
6. Battery OFF, reconnect USB, send D and paste the ENTIRE block. A live D also attempts to save a partial final summary before stopping. Powering off can lose the final partial window/queued records, so leave several seconds after the behavior of interest if safe to do so.

The target stays fixed during each scan. This is a relocation test, not continuous moving-target tracking. A reported two-second command is not an independently measured score.

## Storage

The 1 KB EEPROM is partitioned: header 16 bytes, 60 summary records (720 bytes), 24 sparse event records (288 bytes). Each bank has its own committed count; filling the event bank cannot consume summary slots. Windows are attempted every 5 seconds for the five-minute run. Summary records remain best effort if the eight-record queue is full. The dump merges both banks by timestamp.

Sparse event bank: up to four accepted-arc events and twelve combined laser-on/off events; completion, gyro-detail, fault and recording-end events can use the remaining reserved space. Earlier critical events can reduce the number of ordinary events that fit. Later brief laser bursts may lack exact timestamps if the sparse bank fills, but summaries still mark whether laser output was on during each interval. Pulse, routine-state and speed-limiter events are omitted in this diagnostic build.

## Event 21: five-second firing-gate summary

Header: time_ms,event,state_or_block,error_cdeg,rate_cdps,rpm_or_centre,pwm_or_confirm,flags.

- time_ms: window capture/end time since recording started. First window begins at zero; normal windows are about five seconds. This is not a raw 5-second sample: the rate/gate statistics aggregate tracking observations within the interval.
- event: 21.
- state_or_block: low four bits are the controller state at capture; upper four bits are the most frequently failing gate by loop-observation count. Decode dominant = value >> 4: 0 none/no tracking samples, 1 IR, 2 no valid arc/reference, 3 angle, 4 rate, 5 wheel speed. Ties favor the earlier-listed gate. Multiple failures can coexist; dominant is not proof that fixing only that gate suffices.
- error_cdeg: CURRENT target-minus-yaw error at capture, times 100; meaningful only with a valid target reference. It does not describe maximum error throughout the whole interval.
- rate_cdps: MAXIMUM ABSOLUTE FILTERED body rate seen while AIM/FINE/FIRE during that interval, times 100. Zero may mean no tracking observations.
- rpm_or_centre: CURRENT stored target centre times 10 (NOT wheel RPM on event 21). This helps distinguish references at later locations even after sparse arc events are exhausted. Gyro angle is unwrapped and drift-prone, not an absolute heading; values saturate at +/-3276.7 degrees.
- pwm_or_confirm: longest observed continuous interval where all firing ENTRY gates passed, in milliseconds, quantised to 2 ms and capped at 254 ms (NOT PWM). A continuous interval can span a window boundary. This is independently observed in the logger and may differ by a loop interval from the actual arming timer.
- flags: failures observed at least once while tracking: 1 missing fresh IR, 2 missing valid arc, 8 angle outside entry tolerance, 16 rate outside entry tolerance, 32 wheel beyond entry limit. Additional bits: 4 laser output was HIGH at least once in the window; 64 all entry gates passed at least once. These are interval aggregates, not simultaneous conditions or current gate bits.

For example: dominant 4 plus longest-valid interval below 85 ms and small current error is evidence that rate repeatedly prevented confirmation. Dominant 1 instead points toward optical continuity. A peak-rate excursion alone does not establish why every attempt failed.

Other event rows retain V30 meanings, including ARC's special centre/width columns and GYRO_DETAIL's special timing/rate/acceleration columns. All fixed-point fields saturate at their documented integer limits. No added motion commands beyond A/D.

Compile/control-invariance and recorder layout/model tests are local verification; physical persistence and four-location diagnostics need the battery run.
