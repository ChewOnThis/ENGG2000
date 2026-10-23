# MQ Sentinel V8.8.3-S — Streamlined Software Subsystem Guide

**Project:** MQ Sentinel, ENGG2000 / ENGG3000  
**Controller:** Arduino Nano (ATmega328P)  
**Source:** User-supplied V8.8.3 three-sensor/filter/precision-microadjustment program  
**Updated sketch:** `MQ_Sentinel_V8_8_3_S_Streamlined_EEPROM.ino`  
**Scope:** Code refactoring, functionality map, measurement interpretation and testing requirements  
**Compatibility:** EEPROM V6, same event numbering and 1,021-byte layout  

## 1. Purpose and what was preserved

The firmware implements an autonomous sensing-to-pointing loop. Three active-LOW demodulating 38 kHz IR receivers indicate whether the beacon is left, centred, right, or ambiguous. The Arduino filters receiver pulses, interprets the three-bit sensor mask, requests reaction-wheel motor torque, waits for approximate alignment and enables the laser for a **maximum nominal 2,100 ms shot** if centre alignment continues. It can then search again. The controller also saves an up-to-five-minute experiment to internal EEPROM, and supports live diagnostics through Serial Monitor.

**Version note:** This document describes the **uploaded V8.8.3 firmware**, not the later V8.8.7 continuous-laser controller. In this firmware the laser shuts off at 2.1 seconds and the seek motor begins only *after* the first valid IR detection or a completed shot. Those behaviours were intentionally preserved when streamlining.

The refactor maintains pin assignments, PWM numbers, thresholds, sensor-mask interpretations, motor polarity, zero-crossing protection, six controller states, pulse validation, fine correction pulses, centre-only acceptance, 2.1-second shot and abort behaviour, serial commands, telemetry, V6 EEPROM field layout and historical event IDs. It removes unused parameters (`AMBIGUOUS_PWM`, `WHEEL_QUIET_RPM`), unused tracking timestamps (`modeSince`, `lastSideAt`, `zoneChangedAt` and `lastSensorMask`), and unifies repeated laser-duration calculations, status printing and recorder initialisation. These are maintainability changes, not demonstrated improvements in physical targeting.

## 2. Input/output wiring

| Arduino pin | External component | Direction | Role |
|---|---|---|---|
| D2 | Left IR receiver | Input | Left-side 38 kHz beacon detection |
| D3 | Centre IR receiver | Input | Main aiming reference |
| D4 | Right IR receiver | Input | Right-side 38 kHz beacon detection |
| D6 | DRV8874 IN1 | PWM output | Motor effort and directional drive |
| D7 | DRV8874 IN2 | Digital output | Motor polarity reference |
| D8 | Laser control interface | Digital output | HIGH during firing; LOW otherwise |
| D9 | Encoder channel A | Input / pin-change interrupt | Wheel rotation pulses |
| D10 | Encoder channel B | Input | Wheel direction at each A rising edge |
| D13 | Built-in LED | Output | HIGH in SETTLE and FIRE states |

Motor direction depends on actual wiring and reaction-wheel mounting. The program's negative command is intended to turn the satellite body anticlockwise (left), and its positive command clockwise (right). These assumptions require a physical check. D8 must interface with a properly rated laser driver; the Arduino pin is not a suitable direct laser-diode power supply.

## 3. Processing sequence

At startup, `setup()` first configures all I/O and commands both motor and laser OFF. It enables pin-change interrupts for the three IR receivers and encoder channel A. Next, it reads the existing EEPROM header to determine whether a valid previously saved experiment exists.

An explicitly armed future experiment (`phase=0, armed=1`) enters a five-second USB cancellation window. When this window ends, recording begins with `EVT_BOOT`. The motor still waits in `WAIT_FLASH` until a valid sensor detection occurs. This ensures time-to-first-detection includes the initial period without IR. With an already saved experiment, this older firmware enters its data/stop state rather than automatically running a fresh unlogged mission.

Every main-loop cycle calls, in order:

1. `serialCommands(now)`: handles operator commands and data retrieval.
2. `readSensors(now)`: consumes validated IR pulses and updates the filtered sensor mask.
3. `readWheel(now)`: periodically estimates wheel RPM from encoder ticks.
4. `logSensorTransition(now)`: updates event counts if recording is active.
5. `serviceController(now)`: applies the autonomous state machine.
6. `serviceMotor(now)`: moves the applied motor PWM gradually toward the requested value.
7. `sampleFlight(now)` and `serviceEEPROM()`: capture experimental snapshots and background writes.
8. Status LED and optional telemetry update.

At the five-minute recording limit, the existing V8.8.3 logic sets the HALT condition, turns actuators OFF and preserves EEPROM.

## 4. Interrupt-driven IR acquisition

The receiver modules output a demodulated envelope, not a measurement of the original 38 kHz carrier itself. Because these signals may be short, `ISR(PCINT2_vect)` monitors changes on pins D2–D4 without waiting for normal loop polling. Each falling edge marks the start of an active-LOW signal; the following rising edge marks its end.

For each input, the interrupt routine records:

- Raw falling-edge count (`irRawFalls`).
- Start timestamp (`irFallMicros`).
- Accepted completed pulses (`irAccepted`, `irCompleted`).
- Rejected completed pulses (`irRejected`).

A completed LOW pulse is accepted only if its duration is **500–20,000 microseconds**. This filters very short electrical disturbances and unusually long/stuck-LOW signals. It does not distinguish a direct IR reflection from a genuine optical path.

`readSensors()` copies accumulated pulse counts from interrupt-owned variables inside an atomic block, then resets those per-loop counts. Each channel needs **two validated pulses within a 160 ms evidence window** before becoming a recognised detection. A recognised detection remains active for **65 ms** after the last confirmed mark. This short persistence is intentional: without it, receivers can flicker between active and inactive states faster than the mechanical chassis can respond.

`rawIRMask` reports the instantaneous electrical LOW levels for diagnostics; `sensorMask` contains the filtered control decisions. The two may differ because one is instantaneous and the other includes pulse qualification and memory.

The code cannot tell a reflected 38 kHz signal from a direct one using these digital outputs. Internal optical barriers, rear shielding, transmitter-modulation checks and a stationary angular field-of-view test are still needed.

## 5. Three-bit sensor interpretation

The mask is calculated as `mask = L + 2*C + 4*R`, where L, C and R are 0 or 1. For display purposes, the telemetry prints **LCR** (left-centre-right) bit patterns, while the EEPROM stores the decimal mask.

| LCR display | Decimal mask | Interpretation | Requested behaviour |
|---|---:|---|---|
| 000 | 0 | No IR detected | Coast temporarily; seek after target-loss timeout |
| 100 | 1 | Left only | Coarse correction left |
| 010 | 2 | Centre only | Brake/settle; alignment candidate |
| 110 | 3 | Left + centre | Short fine left correction |
| 001 | 4 | Right only | Coarse correction right |
| 101 | 5 | Left + right | Ambiguous; stop guessing |
| 011 | 6 | Centre + right | Short fine right correction |
| 111 | 7 | All three | Ambiguous by default |

`isCentred(mask)` defaults to `mask == 2`. The source retains an opt-in switch for `mask == 7`, but **all-three must not be accepted as precise alignment until it has been calibrated against the physical laser point**. An all-three reading can result from overlapping sensor fields or reflections.

## 6. Finite-state machine

| Mode | Number | Entry or purpose | Exit |
|---|---:|---|---|
| WAIT_FLASH | 0 | Power-on standby, motor/laser off | First accepted IR -> TRACK |
| SEEK | 1 | Rotate to find target | Valid IR -> TRACK |
| TRACK | 2 | Steer toward centre using left/right receiver patterns | Centre-only persists -> SETTLE; signal absent for 230 ms -> SEEK |
| SETTLE | 3 | Wait for stable centre evidence and motor quiet | Conditions satisfied -> FIRE; sustained non-centre -> TRACK |
| FIRE | 4 | Laser HIGH while centre lock remains credible | 2.1 s complete -> SEEK; centre absent too long -> TRACK |
| HALT | 5 | Motor and laser disabled | Depends on serial arming/control sequence |

When a new SEEK state starts, `seekDirection` changes sign. The first entry flips its initial positive value, then later entries alternate. In this **older** design, direction does not periodically reverse during one uninterrupted seek; reversal happens each time `enterSeek()` is called.

## 7. Reaction-wheel control and microadjustments

`requestMotor()` requests signed PWM but does not drive the physical output immediately. `serviceMotor()` runs at a nominal 5 ms interval, updating the output through a second-order smoothing rule. The code uses a response parameter `MOTOR_OMEGA = 0.0125 ms^-1` and a maximum command-change rate of `0.76 PWM/ms`. On physical reversal the output passes through zero and observes an 18 ms polarity guard.

**Coarse tracking:** When the left-only or right-only receiver is active, the target PWM magnitude is **148** (of 255). The sign depends on the side.

**Fine correction:** Centre plus one side must persist for **70 ms**. The code then commands **100 PWM for 145 ms**, followed by a **240 ms cooldown/coast** before a new fine pulse is permitted. Because the motor command is smoothed, the actual PWM waveform does not have square edges matching these durations exactly.

**Centre capture:** When centre-only detection is reached, the controller optionally commands a **70 PWM counter-torque for 145 ms**, opposing the last correction direction. The intention is to reduce overshoot before the laser-alignment timer completes.

**Seeking:** Nominal motor demand is **174 PWM** while wheel RPM is at most 145. Between 145 and 195 RPM it ramps down linearly, reaching a minimum of **108 PWM** above 195 RPM. This speed limiting is only as reliable as the encoder input. The measured project logs have repeatedly shown zero encoder pulses, meaning that if this fault persists the program will treat wheel speed as approximately zero and request the *full 174 PWM seek demand*. That is a concrete reason to test the encoder before relying on this speed limiter.

**Limits:** Motor PWM is a torque/drive command, not a direct measurement of body rotation speed. The encoder measures the reaction wheel relative to the frame, not satellite yaw in the room. The string suspension may introduce restoring torque or oscillation. Software tuning must be checked against physical motion.

## 8. Laser eligibility and shot sequence

The original V8.8.3 laser sequence is intentionally retained:

1. Detect a centre-only candidate (`010` by default).
2. Maintain enough evidence for the centre-candidate timer to reach **550 ms**; gaps of at most **80 ms** are tolerated, but an unrelated non-centre pattern cannot itself establish alignment.
3. Complete any counter-torque and bring the smoothed motor demand near zero.
4. Satisfy an additional **180 ms motor-quiet** condition.
5. Record `EVT_ALIGNED`, move to `FIRE`, command D8 HIGH and count a shot attempt.
6. If centre alignment disappears beyond 80 ms during FIRE, turn D8 LOW, record an abort and resume TRACK.
7. If 2,100 ms elapses without a disqualifying loss, turn D8 LOW, record completion and resume seeking after the target-clear condition.

A `shot_attempt` metric proves that the firmware reached the D8 HIGH command; it does **not** prove that the laser emitted light, struck the target, or scored points. Physical boresight calibration and driver testing are still required.

**Important:** Unlike later V8.8.7, this sketch does *not* keep the laser on indefinitely while aligned.

## 9. EEPROM layout and persistence

The sketch uses the Nano's internal 1,024-byte EEPROM with a fixed **1,021-byte V6 layout**:

| Region | Quantity × size | Bytes | Purpose |
|---|---|---:|---|
| Flight header | 1 × 57 | 57 | Log validity, state, totals and timestamps |
| Event ring | 52 × 7 | 364 | Most recent events, timestamps and packed sensor/controller state |
| Snapshots | 300 × 2 | 600 | One state/RPM sample per second |
| **Total** | | **1,021** | Leaves 3 unused bytes |

This binary layout and the event ID numbering are unchanged. Older valid V6 records can therefore be interpreted by the same decoder. `LOG_MAGIC = 0x86A6` and `LOG_VERSION = 6` identify the expected header.

**Recorder state:** `phase = 0` means waiting to record, `phase = 1` means recording and `phase = 2` means saved/finished. The `armed` field determines whether the next suitable boot starts a new experiment.

**Snapshots:** `sampleFlight()` stores one record each second, for a maximum of 300. Each snapshot packs the 3-bit sensor mask, 3-bit controller state, 2-bit motor direction and signed RPM in units of 2 RPM.

**Events:** `logEvent()` stores elapsed time in units of 10 ms, event ID, packed state, approximate RPM and an auxiliary value. Only the 52 newest events are retained when the ring fills; cumulative header metrics remain available.

**Background writer:** A 160-entry queue buffers EEPROM byte updates so normal control iterations generally do at most one changed-byte write each time through the loop. If overloaded, the writer flushes synchronously to protect data. Such a flush can delay the main loop; pin-change interrupts still capture pulses but the filtering algorithm relies on timely processing of queued evidence.

**Recorded metrics:** target seen/lost; zone transitions; centre arrivals; direction changes; confirmed alignments; shot attempts, completions and aborts; total and longest laser ON times; encoder pulse count; peak approximate wheel RPM; acquisition counts and duration; first detection; first alignment; last elapsed time.

The field historically named `allThreeArrivals` is printed as `centre_arrivals` in the dump and, with default settings, increments for entry into the centre-only acceptance condition. It is **not literally an all-three count** in this configuration.

A timestamp of `4294967295` (0xFFFFFFFF) means a milestone has not been recorded. It must not be interpreted as an elapsed time.

## 10. Serial Monitor controls and retrieval

Use **115200 baud**. The commands preserve original V8.8.3 behaviour:

| Command | Action | Important consequence |
|---|---|---|
| `C` | Clear and arm a **future boot** | Replaces the previous log header; retrieve data first |
| `D` | Turn outputs off, preserve and print the log | Moves controller to HALT after dumping |
| `T` | Toggle live telemetry every ~250 ms | Prints filtered mask, raw LOW, falling edges, accepted/rejected pulses, RPM and commanded PWM |
| `X` | Safe stop; mark current recording complete if active | Requires a new operating sequence to resume |
| `R` | Return to IR standby **only during an active recording** | This V8.8.3 version does not support unlogged autonomous restart with an old saved log |

**Correct experimental procedure:** Connect by USB -> send `C` -> disconnect and power the Nano from the satellite batteries -> allow the five-second initial window -> point the beacon at the sensors -> allow the autonomous run -> power off, reconnect USB and send `D` -> save the entire dump. Do not send `C` again before saving the old data.

The microcontroller does not inherently know whether power comes from USB or batteries. If serial is connected during the armed five-second window and no command is sent, this older implementation may treat the session as a flight boot. That needs operational discipline.

## 11. What the refactor changed — and did not change

**Simplified:** shared recorder-header reset; shared laser-shot closing code for completed/aborted events; shared EEPROM status-output formatter; direct state assignments in place of a wrapper that only stored an unused timestamp; unused PWM/RPM constants; unused state timestamps; repeated local variables; repetitive housekeeping lines and comments.

**Unchanged:** all physical and logical pin assignments, active-LOW IR acquisition, pulse thresholds, accepted masks, PWM demands, smoothing equation, direction guard, fine correction timing, centre counter-torque, settle criteria, 2.1-second firing, abort grace, state-transition rules, five-minute flight limit, 52-event ring, 300 snapshots, EEPROM V6 addresses, serial command letters and telemetry categories.

**Known limitations intentionally retained:** tracking does not automatically resume with an old completed log on boot; initial WAIT_FLASH does not start seeking before an IR detection; the laser switches off after 2.1 seconds rather than remaining continuously on; and speed limiting depends on an encoder that has not yet produced nonzero logged pulses. Those behaviours were fixed or revised in *later* firmware branches, but porting them into this upload would be a control-system change, not a code-only refactor.

## 12. Verification and remaining testing

The original and streamlined code both passed the same **host-side C++ syntax check** and a common simulated behavioural harness. Both produced the same outcomes for rejecting a single IR pulse, confirming a second pulse, expiring detection after its 65 ms window, left and right signed motor demands, seek-direction reversal on entering SEEK, fine correction timing, centre-settle firing, 2.1-second completion, IR-loss abort, EEPROM reload and the full 300-snapshot recording. The test reported the same 1,021-byte EEPROM layout, including a 57-byte header, 52 × 7-byte events and 300 × 2-byte snapshots.

These checks do **not** establish that the software has been compiled by the official Arduino AVR toolchain, fits Nano flash/SRAM with the installed libraries, accurately detects real 38 kHz beacons, controls physical satellite yaw or illuminates the connected laser. Those require the Arduino IDE and physical hardware testing.

Recommended follow-up sequence: compile in Arduino IDE for the correct Nano/bootloader, verify `T` sensor telemetry while stationary, test a controlled left/right/centre sweep with laser disconnected, verify encoder pulses independently, verify the laser driver using a controlled electrical test, and finally perform several comparable timed target-acquisition trials. Use opaque side/rear baffles if raw IR telemetry establishes significant rear-facing reception.

## 13. Useful engineering interpretation of prior logs

Earlier V2 Improved R4 recorded 68 seconds, 46 laser attempts and a maximum uninterrupted activation of 595 ms. V8.8.4 recorded 34 seconds, no centre arrivals and no laser attempts. V8.8.5 recorded 96 seconds, 114 centre arrivals, four shot attempts and a maximum hold of 1,275 ms. None of those provided logs recorded a completed 2.1-second software hold; physical scoring was not verified.

These later measurements illustrate the difference between acquisition frequency and pointing quality. Frequent target-seen events can arise from receiver flicker, and reaching a centre sensor does not automatically mean the laser is on the bullseye. Further improvement should be assessed by measured hold stability and actual target placement, not raw IR detection counts alone.

**Bottom line:** V8.8.3-S is a cleaner, easier-to-maintain implementation of the *uploaded V8.8.3 programme*. It is not a claim that the tracking algorithm is intrinsically more accurate than V8.8.3, and it is not a substitute for the later V8.8.7 continuous-lock software.
