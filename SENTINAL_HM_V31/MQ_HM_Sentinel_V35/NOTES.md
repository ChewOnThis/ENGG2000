# HM V35: V33 baseline with smooth quiet-hold output

Operator reports V34 passed the IR exit then jolted in the wrong direction; V33 acquired/stopped better but still jittered. No EEPROM was obtained for the latest test. No data from that test or the previously touched run is used for this change. Recorder remains available; missing data was not interpreted as a request to remove it.

V35 is based on HM V33, not V34. Coarse/fine requested commands, acquisition, doublet logic, quiet-band hysteresis, laser gates and sequence match V33 exactly. The only added motor behavior is a slew bound on the final command during quiet FINE/FIRE hold: 600 PWM units/second (3 units per normal 5 ms motor update). This smooths both gyro-driven correction steps and 50 ms encoder feedforward steps, possible contributors to on-target chatter that were not limited in V33. Their causal contribution is unconfirmed without an untouched run or high-rate trace.

The limiter starts from the last applied command and persists in fractional units to avoid losing small changes to integer rounding. State transitions seed it from actual output. Search/coarse control and fine drive pulses bypass it; STOP/BOOT and other non-hold outputs retain immediate handling. The rate/angle/IR/wheel firing gates are unchanged and read the same sensors, so smoothing cannot qualify an otherwise invalid lock. No forced zero-voltage wheel stop was added.

Tradeoff: smoothing adds actuator response delay during hold and may worsen disturbance rejection if too restrictive. This is provisional tuning, not a verified jitter fix. V33 is preserved for rollback. Compare untouched target holds; if it rocks more slowly or loses lock, stop and report that behavior rather than increasing gains without measurements.

Validation: V33 coarse/fine functions, gates and sequence compared byte-for-byte; actual quietSlew helper passes 75 bound/no-overshoot cases and convergence. Classic Nano old-bootloader compilation checked. No upload or physical validation performed. Usual A / next battery boot / D workflow retained; no startup identification/calibration added.
