# V32 candidate

Based on the open MQ_HM_Sentinel_V31 sketch. Original V31 source is unchanged.

Changes: coarse body-rate request cap 3 to 8 deg/s with distance-based slowdown; stronger fine rate damping; faster bounded hold-bias compensation; 80 ms measured-rate braking phase following fine pulses; suppress new pulses when motion already predicts arrival within firing tolerance; telemetry reduced to 20 Hz. Fine drive pulse amplitude and maximum duration remain unchanged.

Fixed X bias, current wiring, scan acquisition, firing gates, two-second laser dwell, A/D arming and V31 EEPROM format retained. No startup calibration or identification added.

Operator reports string shortened approximately from 10 to 5 (units unspecified). A changed restoring torque/oscillation response is plausible, not confirmed. No measured new plant parameters or battery run available. The changes are provisional tuning, not a verified 20-second acquisition or bullseye guarantee. The IR midpoint and laser alignment must correspond physically for a score.

Compiled successfully for arduino:avr:nano:cpu=atmega328old using AVR core 1.8.6: 30,098/30,720 flash bytes; 968/2,048 static RAM bytes. Used an isolated user-library directory because the normal Arduino library folder stalled. Not uploaded.

Run: retrieve any existing data first. Battery off; open/upload this sketch; Serial Monitor 115200, send A and wait for ARMED_NEXT_BOOT; disconnect USB, then battery on. Compare time from beacon-on to completed two-second laser hold, without nudging the satellite. Send D after reconnecting to retrieve logs. Existing V31 log decoding applies.

Physical background: https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/
