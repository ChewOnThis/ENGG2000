# MQ Sentinel: Logic Flow and Operation

## Brief Summary

The Arduino Nano controls a two-direction motor that turns an infrared (IR)
tracking system toward a target. Three IR sensors are used: left, centre, and
right. The motor turns toward a target detected on one side, stops when the
target is centred, and activates the laser after the target has remained
centred for 150 ms. The laser stays on for 2 seconds.

An IR remote toggles the system between enabled and disabled. When disabled,
the motor is electrically braked, the laser is off, and target processing is
skipped. A two-channel encoder counts motor movement so that the current shaft
angle can be calculated when required.

## Hardware Connections

| Component | Arduino pin |
| --- | --- |
| Left IR sensor | D2 |
| Centre IR sensor | D3 |
| Right IR sensor | D4 |
| Motor driver IN1 | D6 |
| Motor driver IN2 | D7 |
| Laser diode | D8 |
| Encoder channel A | D9 |
| Encoder channel B | D10 |
| IR remote receiver | D11 |

The IR sensors and the remote receiver are active LOW. The motor uses digital
direction commands rather than speed control. `motorStop()` sets both motor
inputs HIGH, providing electrical braking.

## Logic Flow

```text
START
	|
	v
setup()
	- Configure sensor, motor, laser, encoder, and remote pins
	- Stop the motor and switch the laser off
	- Store the initial encoder-A level
	- Start Serial communication at 9600 baud
	- Start the IR remote receiver
	|
	v
loop() repeats continuously
	|
	+--> updateEncoder()
	|      On each rising edge of encoder A, read encoder B and increment or
	|      decrement encoderCount to determine direction.
	|
	+--> Read current time with millis()
	|
	+--> updateRemote(now)
	|      If a remote code is received, toggle systemEnabled after the
	|      400 ms debounce period. Disabling also resets the target state.
	|
	+--> Is systemEnabled false?
	|       |
	|       +-- YES: stop motor, force laser OFF, return to the next loop
	|       |
	|       +-- NO: continue to sensor processing
	|
	+--> updateSensorState(now)
	|      - Read the three sensors.
	|      - Hold each recent detection for up to 80 ms to filter noise.
	|      - Update the last sensor activity time.
	|      |
	|      +--> Centre detected?
	|      |       |
	|      |       +-- YES: stop motor and start/continue the centre timer
	|      |       |       |
	|      |       |       +-- Centred for 150 ms and target not complete?
	|      |       |       |      Turn laser ON and start firing timer
	|      |       |       |
	|      |       |       +-- Firing for 2 seconds?
	|      |       |              Turn laser OFF and mark target complete
	|      |       |
	|      |       +-- NO: clear centre timer
	|      |               If firing, cancel firing and turn laser OFF
	|      |               |
	|      |               +-- Only left detected: turn motor left
	|      |               +-- Only right detected: turn motor right
	|      |               +-- Neither side detected: turn motor left
	|      |               +-- Both side sensors detected: stop motor
	|      |
	+--> Has every sensor been inactive for 1.5 seconds?
					|
					+-- YES: reset targetComplete, firing, centre timer, laser, and
									motor state so the next target can be processed
					+-- NO: continue looping
```

## How the Main Parts Work

### Startup

`setup()` configures all pins, starts Serial output and the IR remote
receiver, stops the motor, and ensures the laser starts off. The initial level
of encoder channel A is saved so the first loop can detect a rising edge
correctly.

### Sensor filtering and tracking

`updateSensorState()` records the time whenever a sensor is active. A sensor
is treated as detected until 80 ms have passed since its most recent active
reading. This short hold time prevents brief gaps or electrical noise from
causing rapid changes in motor direction.

The centre sensor has priority over the side sensors. A centre detection stops
the motor. If it remains stable for `CENTRE_STABLE_MS` (150 ms), the laser is
turned on. If the centre signal is lost during firing, the laser is turned off
immediately. Once the laser has been on for `LASER_TIME_MS` (2 seconds), the
laser is switched off and `targetComplete` prevents another firing cycle
until the target has left and the system has reset.

When the centre sensor is inactive, a left-only detection calls `motorLeft()`
and a right-only detection calls `motorRight()`. With no side detection the
motor turns left by default. If both side sensors are active, the motor stops
because the direction is ambiguous.

### Remote enable/disable

`updateRemote()` accepts any decoded remote button press as a toggle. The
400 ms debounce interval prevents a held button from toggling the system
repeatedly. Disabling the system calls `resetTargetState()`, which clears the
centre and firing timers, marks the target incomplete, switches the laser off,
and stops the motor.

### Encoder

`updateEncoder()` checks encoder channel A every loop. A rising edge causes
channel B to be sampled to determine rotation direction, incrementing or
decrementing `encoderCount`. `getAngleDegrees()` converts the count to an
angle using the approximate value of 700 counts per revolution:

```text
angle = encoderCount * 360 / 700
```

The encoder count is maintained continuously, although the optional Serial
status output that displays the angle is currently commented out.

### Reset after inactivity

The main loop checks for complete sensor inactivity separately from the
filtered sensor logic. If no sensor has been active for 1.5 seconds,
`resetTargetState()` is called and `lastSensorActivity` is cleared. This
allows the next detected target to start a fresh alignment and firing cycle.
