
/*
  ============================================================
  MQ SENTINEL - AUTONOMOUS IR TRACKING CONTROLLER
  ============================================================

  Arduino Nano / ATmega328P

  D2  LEFT IR
  D3  CENTRE FINE IR
  D4  RIGHT IR

  D6  MOTOR IN1
  D7  MOTOR IN2

  D8  LASER

  D9  ENCODER A
  D10 ENCODER B

  D13 BUILT-IN STATUS LED

  FEATURES:

  - Automatic target search
  - Left/right coarse alignment
  - Centre sensor priority
  - Continuous full-drive search (bounded total run time)
  - Short motor correction pulses
  - Reaction-wheel encoder monitoring
  - Fine alignment and settling verification
  - Laser interlock
  - Automatic reacquisition after target loss

  No Serial Monitor required during operation.
*/

// ============================================================
// SELECT IR DETECTION METHOD
// ============================================================

// 0 = Raw HIGH/LOW detection
// 1 = IRremote decoded detection

#define USE_DECODED_IR 0

#if USE_DECODED_IR

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES

void UserIRReceiveTimerInterruptHandler();

#include <IRremote.hpp>

#endif

// ============================================================
// PIN DEFINITIONS
// ============================================================

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

const byte STATUS_LED = LED_BUILTIN;

// ============================================================
// IR RECEIVERS
// ============================================================

#if USE_DECODED_IR

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

#endif

// ============================================================
// CONFIGURATION
// ============================================================

// Set true only after completing laser alignment
// calibration and confirming the operating conditions.

const bool ENABLE_LASER = false;  // Keep disabled for speed testing.

// Reverse motor direction if physical chassis movement
// is opposite to the intended tracking direction.

const bool REVERSE_MOTOR = false;

// ------------------------------------------------------------
// MOTOR PULSE SETTINGS
// ------------------------------------------------------------

// Initial experimental values.
// Calibrate using Tests 3 and 4.

// Single uninterrupted full-drive search, bounded per power cycle.
// Increase ONLY after checking motor/flywheel ratings and the video.
const unsigned long MAX_TOTAL_SEARCH_DRIVE_MS = 1500;

// A 500 ms coarse correction can overshoot a narrow centre sensor.
// Drive is still 100% while a correction pulse is active.

const unsigned long COARSE_PULSE_MS = 120;

// FINE_PULSE_MS was unused in the original state machine; removed.

// Observation interval after each motor pulse.

const unsigned long COAST_WAIT_MS = 50;

// Minimum time between correction pulses.

const unsigned long CORRECTION_INTERVAL_MS = 50;

// ------------------------------------------------------------
// IR SIGNAL SETTINGS
// ------------------------------------------------------------

#if USE_DECODED_IR

// Maximum gap between successful decoded frames.

const unsigned long IR_HOLD_MS = 250;

// Require at least this many separate centre frames
// during the alignment verification period.

const byte CENTRE_CONFIRMATIONS = 3;

#else

// Maximum gap between raw LOW readings.

const unsigned long IR_HOLD_MS = 35;

// Minimum number of LOW samples before firing.

const byte CENTRE_CONFIRMATIONS = 6;

#endif

// ------------------------------------------------------------
// STABILITY SETTINGS
// ------------------------------------------------------------

// Minimum continuous alignment verification time.

const unsigned long ALIGNMENT_HOLD_MS = 500;

// Observe encoder movement over this period.

const unsigned long SPEED_SAMPLE_MS = 200;

// Maximum encoder movement during a speed sample
// for the wheel to be considered sufficiently slow.
//
// This is NOT a chassis-speed measurement.

const long MAX_SETTLED_COUNTS = 1;

// Initial preparation time after power-on.

const unsigned long START_DELAY_MS = 30000;

// ============================================================
// MOTOR DIRECTION
// ============================================================

enum MotorDirection {

  MOTOR_NONE,
  MOTOR_LEFT,
  MOTOR_RIGHT
};

// ============================================================
// CONTROLLER STATES
// ============================================================

enum TrackingState {

  STARTUP,
  SEARCH,
  SEARCH_LIMIT,
  COARSE_ALIGN,
  FINE_ALIGN,
  VERIFY_ALIGN,
  FIRE_TRACK
};

TrackingState state = STARTUP;

// ============================================================
// IR DETECTION
// ============================================================

struct SensorData {

  bool left;
  bool centre;
  bool right;
};

SensorData sensors = {false, false, false};

unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;

bool seenLeft = false;
bool seenCentre = false;
bool seenRight = false;

// ============================================================
// ENCODER
// ============================================================

volatile long encoderCount = 0;

volatile byte previousEncoderA = LOW;

long previousSpeedCount = 0;

long recentEncoderMovement = 0;

unsigned long lastSpeedSample = 0;

bool speedSampleReady = false;

// ============================================================
// MOTOR STATE
// ============================================================

MotorDirection activeMotor = MOTOR_NONE;

MotorDirection lastDirection = MOTOR_LEFT;

unsigned long motorStartTime = 0;

unsigned long motorDuration = 0;

unsigned long motorStopTime = 0;

bool motorRunning = false;

bool motorHasStopped = false;

// A single power-cycle search budget prevents an unattended motor
// from repeatedly accelerating a wheel that may already be saturated.
unsigned long searchDriveUsedMs = 0;
bool searchDriveActive = false;

// ============================================================
// ALIGNMENT STATE
// ============================================================

unsigned long alignmentStart = 0;

unsigned int centreConfirmations = 0;

unsigned long lastCorrectionTime = 0;

// ============================================================
// ENCODER INTERRUPT
// ============================================================

// D9 = PB1 / PCINT1
// D10 = PB2
//
// Count channel A rising edges.
// Read channel B to determine direction.

ISR(PCINT0_vect) {

  byte currentA =
      (PINB & _BV(PB1)) ? HIGH : LOW;

  if (currentA == HIGH &&
      previousEncoderA == LOW) {

    byte currentB =
        (PINB & _BV(PB2)) ? HIGH : LOW;

    if (currentB != currentA) {

      encoderCount++;

    } else {

      encoderCount--;
    }
  }

  previousEncoderA = currentA;
}

// ============================================================
// SAFE ENCODER READING
// ============================================================

long readEncoder() {

  long count;

  noInterrupts();

  count = encoderCount;

  interrupts();

  return count;
}

// ============================================================
// ENCODER SETUP
// ============================================================

void initialiseEncoder() {

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  previousEncoderA =
      digitalRead(ENCODER_A);

  PCICR |= _BV(PCIE0);

  PCMSK0 |= _BV(PCINT1);

  previousSpeedCount = readEncoder();

  lastSpeedSample = millis();
}

// ============================================================
// ENCODER SPEED MONITOR
// ============================================================

void updateWheelSpeed(unsigned long now) {

  if (now - lastSpeedSample < SPEED_SAMPLE_MS) {
    return;
  }

  long currentCount = readEncoder();

  recentEncoderMovement =
      labs(currentCount - previousSpeedCount);

  previousSpeedCount = currentCount;

  lastSpeedSample = now;

  speedSampleReady = true;
}

bool wheelSettled() {

  return speedSampleReady &&
         recentEncoderMovement <= MAX_SETTLED_COUNTS;
}

// ============================================================
// MOTOR OUTPUTS
// ============================================================

void motorCoast() {

  // Verify LOW/LOW requests coast mode
  // for your actual motor driver.

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorLeft() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
}

void motorRight() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  } else {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }
}

// ============================================================
// STOP MOTOR
// ============================================================

void stopMotor(unsigned long now) {

  if (!motorRunning) {
    return;
  }

  // Account for the entire continuous search-drive interval.
  if (searchDriveActive) {
    unsigned long elapsed = now - motorStartTime;
    unsigned long remaining = MAX_TOTAL_SEARCH_DRIVE_MS - searchDriveUsedMs;
    searchDriveUsedMs += (elapsed < remaining) ? elapsed : remaining;
    searchDriveActive = false;
  }

  motorCoast();

  motorRunning = false;

  motorHasStopped = true;

  activeMotor = MOTOR_NONE;

  motorStopTime = now;

  lastCorrectionTime = now;
}

// ============================================================
// START MOTOR PULSE
// ============================================================

void startMotorPulse(
  MotorDirection direction,
  unsigned long duration,
  unsigned long now
) {

  if (motorRunning) {
    return;
  }

  // Never drive the motor while the laser is on.

  digitalWrite(LASER_PIN, LOW);

  // Avoid reversing direction while motor drive is active.

  if (direction == MOTOR_LEFT) {

    motorLeft();

  } else if (direction == MOTOR_RIGHT) {

    motorRight();

  } else {

    return;
  }

  activeMotor = direction;

  lastDirection = direction;

  motorStartTime = now;

  motorDuration = duration;

  motorRunning = true;

  motorHasStopped = false;
}

// ============================================================
// UPDATE MOTOR
// ============================================================

void updateMotor(unsigned long now) {

  if (!motorRunning) {
    return;
  }

  if (now - motorStartTime >= motorDuration) {

    stopMotor(now);
  }
}

// ============================================================
// MOTOR READY FOR ANOTHER PULSE
// ============================================================

bool motorReady(unsigned long now) {

  if (motorRunning) {
    return false;
  }

  if (!motorHasStopped) {
    return true;
  }

  return now - motorStopTime >= COAST_WAIT_MS &&
         now - lastCorrectionTime >=
             CORRECTION_INTERVAL_MS;
}

// ============================================================
// IR DETECTION UPDATE
// ============================================================

void updateIRSensors(unsigned long now) {

#if USE_DECODED_IR

  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  if (IrReceiver.decode()) {

    lastLeftIR = now;

    seenLeft = true;

    IrReceiver.resume();
  }

  // ----------------------------------------------------------
  // CENTRE
  // ----------------------------------------------------------

  if (centreReceiver.decode()) {

    lastCentreIR = now;

    seenCentre = true;

    if (centreConfirmations < 65535) {
      centreConfirmations++;
    }

    centreReceiver.resume();
  }

  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  if (rightReceiver.decode()) {

    lastRightIR = now;

    seenRight = true;

    rightReceiver.resume();
  }

#else

  // ----------------------------------------------------------
  // RAW HIGH/LOW DETECTION
  // ----------------------------------------------------------

  if (digitalRead(LEFT_SENSOR) == LOW) {

    lastLeftIR = now;
    seenLeft = true;
  }

  if (digitalRead(CENTRE_SENSOR) == LOW) {

    lastCentreIR = now;
    seenCentre = true;

    if (centreConfirmations < 65535) {
      centreConfirmations++;
    }
  }

  if (digitalRead(RIGHT_SENSOR) == LOW) {

    lastRightIR = now;
    seenRight = true;
  }

#endif

  // ----------------------------------------------------------
  // APPLY DETECTION HOLD
  // ----------------------------------------------------------

  sensors.left =
      seenLeft &&
      now - lastLeftIR <= IR_HOLD_MS;

  sensors.centre =
      seenCentre &&
      now - lastCentreIR <= IR_HOLD_MS;

  sensors.right =
      seenRight &&
      now - lastRightIR <= IR_HOLD_MS;
}

// ============================================================
// ALIGNMENT VERIFICATION
// ============================================================

void beginVerification(unsigned long now) {

  state = VERIFY_ALIGN;

  alignmentStart = now;

  centreConfirmations = 0;

  // Ensure the motor is no longer being driven.

  stopMotor(now);
}

// ============================================================
// TARGET DIRECTION
// ============================================================

MotorDirection targetDirection() {

  // No correction when centre detects the beacon.

  if (sensors.centre) {

    return MOTOR_NONE;
  }

  // Target on left.

  if (sensors.left && !sensors.right) {

    return MOTOR_LEFT;
  }

  // Target on right.

  if (sensors.right && !sensors.left) {

    return MOTOR_RIGHT;
  }

  // Both side sensors detect:
  // Direction is ambiguous.

  if (sensors.left && sensors.right) {

    return MOTOR_NONE;
  }

  // No target detected.

  return MOTOR_NONE;
}

// ============================================================
// MAIN TRACKING CONTROLLER
// ============================================================

void updateController(unsigned long now) {

  // ==========================================================
  // STARTUP
  // ==========================================================

  if (state == STARTUP) {

    if (now >= START_DELAY_MS) {

      state = SEARCH;
    }

    return;
  }

  // ==========================================================
  // CENTRE SENSOR PRIORITY
  // ==========================================================

  // Stop powered rotation as soon as centre detects IR.

  if (sensors.centre && motorRunning) {

    stopMotor(now);

    state = FINE_ALIGN;
  }

  // ==========================================================
  // STATE 1: SEARCH - UNINTERRUPTED FULL DRIVE
  // ==========================================================

  if (state == SEARCH) {

    digitalWrite(LASER_PIN, LOW);

    // Sensor handling has priority over the search-speed limit.
    if (sensors.centre) {
      stopMotor(now);
      state = FINE_ALIGN;
      return;
    }

    if (sensors.left || sensors.right) {
      stopMotor(now);
      state = COARSE_ALIGN;
      return;
    }

    // A previous coarse correction may still be in progress when
    // target detection disappears; do not inherit that drive command.
    if (motorRunning && !searchDriveActive) {
      stopMotor(now);
    }

    if (searchDriveUsedMs >= MAX_TOTAL_SEARCH_DRIVE_MS) {
      stopMotor(now);
      state = SEARCH_LIMIT;
      return;
    }

    // No repeated 50 ms pulses: keep full drive on until a sensor
    // detects IR or the bounded cumulative search budget expires.
    if (!motorRunning && motorReady(now)) {
      unsigned long remaining = MAX_TOTAL_SEARCH_DRIVE_MS - searchDriveUsedMs;
      startMotorPulse(lastDirection, remaining, now);
      searchDriveActive = motorRunning;
    }
    return;
  }

  // Search time exhausted: stop accelerating the wheel. A side
  // detection can still initiate a correction, but an unattended
  // search does not keep spinning the motor indefinitely.
  if (state == SEARCH_LIMIT) {
    digitalWrite(LASER_PIN, LOW);
    stopMotor(now);
    if (sensors.centre) {
      state = FINE_ALIGN;
    } else if (sensors.left || sensors.right) {
      state = COARSE_ALIGN;
    }
    return;
  }

  // ==========================================================
  // STATE 2: COARSE ALIGNMENT
  // ==========================================================

  if (state == COARSE_ALIGN) {

    digitalWrite(LASER_PIN, LOW);

    // Centre sensor always has priority.

    if (sensors.centre) {

      stopMotor(now);

      state = FINE_ALIGN;

      return;
    }

    MotorDirection direction =
        targetDirection();

    // Lost target completely.

    if (!sensors.left && !sensors.right) {

      stopMotor(now);
      state = SEARCH;
      return;
    }

    // Ambiguous side detection:
    // Stop rather than choosing randomly.

    if (direction == MOTOR_NONE) {

      stopMotor(now);

      return;
    }

    // Start a short correction pulse.

    if (motorReady(now)) {

      startMotorPulse(
        direction,
        COARSE_PULSE_MS,
        now
      );
    }

    return;
  }

  // ==========================================================
  // STATE 3: FINE ALIGNMENT
  // ==========================================================

  if (state == FINE_ALIGN) {

    digitalWrite(LASER_PIN, LOW);

    // If centre alignment is lost,
    // determine the new correction direction.

    if (!sensors.centre) {

      state = COARSE_ALIGN;

      if (!sensors.left && !sensors.right) {

        state = SEARCH;
      }

      return;
    }

    // Wait for residual motor movement after cutoff.

    if (!motorReady(now)) {

      return;
    }

    // Centre sensor still sees target:
    // begin alignment verification.

    beginVerification(now);

    return;
  }

  // ==========================================================
  // STATE 4: VERIFY ALIGNMENT
  // ==========================================================

  if (state == VERIFY_ALIGN) {

    digitalWrite(LASER_PIN, LOW);

    // Target has moved outside fine sensor.

    if (!sensors.centre) {

      state = FINE_ALIGN;
      return;
    }

    // Allow time for stable alignment.

    if (
      now - alignmentStart <
      ALIGNMENT_HOLD_MS
    ) {

      return;
    }

    // Require enough detection observations.

    if (
      centreConfirmations <
      CENTRE_CONFIRMATIONS
    ) {

      return;
    }

    // Require low reaction-wheel movement.

    if (!wheelSettled()) {

      return;
    }

    // All verification conditions satisfied.

    state = FIRE_TRACK;

    if (ENABLE_LASER) {

      digitalWrite(LASER_PIN, HIGH);
    }

    return;
  }

  // ==========================================================
  // STATE 5: FIRE AND TRACK
  // ==========================================================

  if (state == FIRE_TRACK) {

    // Lost target: immediately disable laser.

    if (!sensors.centre) {

      digitalWrite(LASER_PIN, LOW);

      state = FINE_ALIGN;

      return;
    }

    // Unexpected motor movement.

    if (motorRunning || !wheelSettled()) {

      digitalWrite(LASER_PIN, LOW);

      state = FINE_ALIGN;

      return;
    }

    // Target remains centred according to fine sensor.

    if (ENABLE_LASER) {

      digitalWrite(LASER_PIN, HIGH);
    }

    return;
  }
}

// ============================================================
// STATUS LED
// ============================================================

void updateStatusLED(unsigned long now) {

  // LED permanently ON while centre sensor detects IR.

  if (sensors.centre) {

    digitalWrite(STATUS_LED, HIGH);

    return;
  }

  // Blink during search / coarse alignment.

  if (state == SEARCH ||
      state == SEARCH_LIMIT ||
      state == COARSE_ALIGN) {

    digitalWrite(
      STATUS_LED,
      (now / 250) % 2 ? HIGH : LOW
    );

    return;
  }

  digitalWrite(STATUS_LED, LOW);
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  // Laser disabled during startup.

  digitalWrite(LASER_PIN, LOW);

  digitalWrite(STATUS_LED, LOW);

  // Motor initially coasting.

  motorCoast();

#if USE_DECODED_IR

  // LEFT uses built-in receiver.

  IrReceiver.begin(
    LEFT_SENSOR,
    false
  );

  // CENTRE fine receiver.

  centreReceiver.begin(
    CENTRE_SENSOR,
    false
  );

  // RIGHT receiver.

  rightReceiver.begin(
    RIGHT_SENSOR,
    false
  );

#else

  pinMode(LEFT_SENSOR, INPUT);

  pinMode(CENTRE_SENSOR, INPUT);

  pinMode(RIGHT_SENSOR, INPUT);

#endif

  // Initialise motor encoder.

  initialiseEncoder();

  state = STARTUP;
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long now = millis();

  // Keep the laser off unless the controller
  // has explicitly authorised activation.

  if (state != FIRE_TRACK) {

    digitalWrite(LASER_PIN, LOW);
  }

  // Read all three IR sensors.

  updateIRSensors(now);

  // Monitor reaction-wheel movement.

  updateWheelSpeed(now);

  // End motor pulse if its time has expired.

  updateMotor(now);

  // Execute autonomous tracking state machine.

  updateController(now);

  // Update visible LED indicator.

  updateStatusLED(now);
}

// ============================================================
// IRREMOTE MULTIPLE RECEIVER INTERRUPT HANDLER
// ============================================================

#if USE_DECODED_IR

void UserIRReceiveTimerInterruptHandler() {

  centreReceiver.ReceiveInterruptHandler();

  rightReceiver.ReceiveInterruptHandler();
}

#endif