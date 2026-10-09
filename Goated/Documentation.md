# Goated Advanced Controllers

This folder contains the two most developed MQ Sentinel controller sketches.
Both target an Arduino Nano/ATmega328P, use the same common wiring, and store
short battery experiments in the Nano's EEPROM. They are standalone Arduino
sketches: open and compile each sketch from its own matching folder.

## Common Hardware

| Function | Pin |
| --- | --- |
| Left, centre, right IR receivers | D2, D3, D4 |
| Motor driver IN1, IN2 | D6, D7 |
| Laser | D8 |
| Encoder A, B | D9, D10 |
| Status LED | D13 (V8.8.3) |

The receivers are active-LOW demodulating 38 kHz IR modules. The motor is a
reaction-wheel actuator: wheel speed and torque influence chassis motion, but
encoder RPM is not a direct measurement of chassis yaw. Neither controller has
a gyro. All starting power and timing values require physical calibration.

## Controller Comparison

| Controller | Main purpose | Sensor processing | Alignment approach | Serial |
| --- | --- | --- | --- | --- |
| `MQ_Sentinel_V8_8_3_IR_Precision_Microadjust_EEPROM` | Precision filtered controller and detailed recorder | Pin-change interrupt pulse capture, pulse-width validation, two-pulse evidence confirmation | SEEK, TRACK, SETTLE, FIRE, with bounded micro-pulses | 115200 baud |
| `MQ_Sentinel_V2_Improved_Comparable_EEPROM` | A/B comparison of tuned and baseline control | `IRremote` receivers with held sensor zones | SEARCHING, CAPTURE_APPROACH, CAPTURE_DAMPING, FINE_ALIGN, TARGET_LOCKED | 115200 baud |

V8.8.3 is the more conservative sensor path: a sustained LOW, glitch, or
floating input does not by itself count as a target. V2 is useful for comparing
control strategies because its logger and power ceiling are shared between the
`TUNED_CONTROL` variants.

## V8.8.3 Precision Microadjust

### Control flow

The controller begins safely in `WAIT_FLASH` with the motor off. After a valid
IR flash it enters `TRACK`. If the target is absent for `MIN_TARGET_LOST_MS`
(230 ms), it enters `SEEK`. Centre-only (`010`) is the default centred mask;
all-three (`111`) remains ambiguous unless `ACCEPT_ALL_THREE_AS_CENTRED` is
explicitly enabled after physical calibration.

Tracking uses the following corrections:

- Outer-only detection uses a reduced coarse torque.
- Centre-plus-side overlap uses a short fine pulse after 70 ms of stable
  overlap, followed by a 240 ms cooldown.
- Entering centre applies one counter-torque pulse to reduce overshoot.
- A valid centre candidate settles for 550 ms before firing.
- The laser must remain on for 2100 ms to qualify as a completed shot.
- Brief centre dropouts receive an 80 ms grace period; longer loss aborts the
  laser cycle.

Motor output is passed through a slew-limited envelope. Direction reversals
cross zero with an 18 ms guard, reducing abrupt mechanical changes. Search
power is reduced as wheel RPM approaches the soft and hard limits of 145 and
195 RPM.

### Sensor validation

Pin-change interrupts capture falling and rising edges on D2-D4. Low pulses
must be between 500 microseconds and 20 milliseconds. Two valid pulses within
160 ms are required before a receiver becomes fresh; evidence expires after
65 ms without another validated mark. The encoder is also captured by an
interrupt on rising encoder-A edges and channel B supplies direction.

### Serial commands

Commands are single characters and are accepted in upper or lower case:

| Command | Action |
| --- | --- |
| `D` | Stop motion, finish/preserve the current record, and dump EEPROM data |
| `C` | Clear and arm the next battery boot; the current USB session stays stopped |
| `R` | Enter or resume IR-triggered standby when a recording exists |
| `T` | Toggle 250 ms diagnostic telemetry |
| `X` | Safe stop: laser off, motor emergency-off, and halt |

At startup, an armed boot waits five seconds. Sending `D` dumps instead;
sending `C` re-arms; sending `X` preserves the armed data and stops. If the
window expires, the five-minute recorder starts immediately while tracking
still waits for the first IR flash.

### EEPROM record

The V8.8.3 logger uses format version 6 and remains within the Nano's 1024-byte
EEPROM. It stores a header, the latest 52 event records, and up to 300
one-second snapshots. Events include target transitions, zone changes,
direction changes, centre acquisition, laser abort/completion, and micro
adjustments. EEPROM writes are queued and serviced one changed byte per loop
where possible to reduce timing disruption.

## V2 Improved Comparable EEPROM

### Control flow

The state machine is:

1. `SEARCHING`: ramp and limit wheel speed while scanning. Tuned control adds
   short coast intervals instead of continuously driving the wheel.
2. `CAPTURE_APPROACH`: baseline control uses wheel RPM and side-to-centre
   crossing time to choose a pre-brake pulse.
3. `CAPTURE_DAMPING`: baseline control applies a counter-torque pulse and
   observes the centre region.
4. `FINE_ALIGN`: tuned control uses small torque pulses, stable-zone timing,
   and wheel-speed holding to reduce left-centre-right hunting.
5. `TARGET_LOCKED`: centre-only or the configured all-three condition remains
   stable before the laser is allowed to fire.

The current build has `TUNED_CONTROL = true` and identifies itself as
`V2_IMPROVED_COMPARISON_R4`. Both comparison paths use a 190 PWM ceiling and a
1.2 second startup ramp. The tuned path uses 28 ms fine-zone confirmation and
22 ms hold-zone confirmation; centre lock requires 280 ms and target loss
returns to fine alignment after 130 ms.

### Wheel-speed control

The encoder is sampled to estimate wheel RPM. The controller learns the encoder
sign produced by the left-drive command, captures the wheel speed after
corrections, and can actively hold that speed. This compensates for
flywheel/gearbox drag returning momentum to the chassis after an apparently
stationary correction. It is still not a closed-loop chassis-angle controller.

### EEPROM experiment procedure

The V2 logger is designed for a five-minute battery experiment:

1. Connect USB and open Serial Monitor at 115200 baud.
2. Send `C` to clear the old record and arm the next boot.
3. Disconnect USB, reinstall/power the Nano from the battery, and allow the
   controller to run.
4. Power down after the experiment, reconnect USB, and send `D` to dump data.

The logger preserves data across USB resets and does not erase a completed
record at boot. A reset during an active recording is counted as an unexpected
restart and the logger resumes using the saved elapsed time.

### Dump contents

The dump is delimited by `BEGIN SENTINEL PRECISION V2 EEPROM` and
`END SENTINEL PRECISION V2 EEPROM`. It reports controller identity, phase,
arming status, EEPROM usage, event/snapshot counts, target and zone metrics,
alignment and shot counts, laser durations, encoder pulses, wheel RPM peak,
and acquisition timing. Event records and one-second snapshots follow in
comma-separated sections.

## Safe Calibration Procedure

1. Disconnect or physically shield the laser.
2. Secure the reaction wheel and begin with reduced motor power.
3. Confirm active-LOW polarity and left/centre/right mask behavior.
4. Verify encoder direction and measured RPM against the known motor command.
5. Tune one timing or power constant at a time and record the firmware name.
6. Test target loss, laser abort, safe stop, EEPROM dump, and power reset.
7. Only enable the laser after physical alignment has been verified.

These sketches contain experimental starting values, not proof of safe aim,
stable chassis pointing, or successful target engagement.