# MB speed trials: test separately

The confirmed V35 setup completed five operator-confirmed target hits by 109.16 seconds. The later supplied V35 run included nudging and handling; it recorded nine firmware holds through 258.02 seconds, with no fault event recorded. Its hold timing cannot be treated as an autonomous speed benchmark. Without intervention timestamps, individual intervals cannot be labeled untouched. Its signed-rate samples are observations under disturbance, not identified plant parameters.

## Order

1. V35 reference with the current mechanical setup and a repeatable target sequence, without nudges.
2. V36 stronger overspeed braking. Recommended first trial because the earlier untouched V35 run already showed arrival rates exceeding requested speed.
3. V37 faster far return. Compare independently with V35; do not assume V36 braking is included.
4. V38 quicker backward-search response. Lowest expected time saving; compare search behavior and unnecessary reversals.

Only combine changes after separate trials show which improve the result. The same version numbers in SENTINAL_HM_V31 are a different family.

## Comparable run

Keep suspension geometry, laser alignment, battery condition, starting orientation, starting twist, target distances and target sequence as consistent as practical. Keep each target stationary while scanning/returning/holding. Use the same operator-controlled target-movement process for every run and record movement times because handling contributes to elapsed time.

Retrieve the previous dump before uploading each trial. Upload, set Serial Monitor 115200, send A, wait ARMED_NEXT_BOOT, disconnect USB and power fully off, then battery on. Note version and run duration. If safe, avoid nudging; if intervention is necessary, record when and treat subsequent results as assisted. Cut power for unsafe motion. After the run, power off, reconnect USB and send D. Save the entire dump.

Compare operator-confirmed correct two-second hits, time to first completed hit, intervals between completed hits, whether the laser spot stayed correctly placed, return overshoot/springback, and search reversals. Repeated holds are not proof of unique target identity. Required two-second dwell and firing gates remain unchanged.

The five-second AIM event 22 samples provide error, signed body rate, wheel RPM and applied PWM; event 16 retains at most three search-direction markers subject to storage. Exact settling times or deceleration cannot always be reconstructed from these sparse samples. A missing marker is not proof of no recovery if quotas or queues fill. Do not make an additional gain change mid-comparison.
