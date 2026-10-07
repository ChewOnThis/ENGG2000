
/*
  MQ SENTINEL - TEST 2A
  FINE SENSOR ANGULAR ACCURACY
  NON-DECODING VERSION

  Board: Arduino Nano

  CENTRE IR: D3
  MOTOR IN1: D6
  MOTOR IN2: D7
  LASER: D8 (OFF)
  STATUS LED: D13

  LED ON  = IR detection window entered
  LED OFF = Detection window exited

  Motor stops after exiting the detection window.

  No USB connection or Serial Monitor required.
*/

// ============================================================
// PIN DEFINITIONS
// ============================================================

const byte CENTRE_SENSOR = 3;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte STATUS_LED = LED_BUILTIN;

// ============================================================
// SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

// Time to prepare the overhead camera.
const unsigned long START_DELAY_MS = 5000;

// Maximum time without LOW before confirming signal loss.
// Tune according to the actual beacon signal.
const unsigned long SIGNAL_LOSS_MS = 50;

// ============================================================
// TEST STATES
// ============================================================

enum TestState {
  STARTUP,
  SEARCHING,
  DETECTING,
  COMPLETE
};

TestState state = STARTUP;

// Time of most recent LOW reading.
unsigned long lastDetectionTime = 0;

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorLeft() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  }
}

void motorBrake() {

  // Preserve original braking command.
  // Verify HIGH/HIGH is braking on your driver.

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(CENTRE_SENSOR, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  // Keep laser disabled.
  digitalWrite(LASER_PIN, LOW);

  // Detection LED initially OFF.
  digitalWrite(STATUS_LED, LOW);

  // Keep motor stationary during startup.
  motorBrake();

  // Allow time for camera preparation.
  delay(START_DELAY_MS);

  // Only start outside the detection window.
  // If already detecting, stay stopped until
  // the Sentinel is repositioned and reset.

  if (digitalRead(CENTRE_SENSOR) == LOW) {

    state = COMPLETE;
    return;
  }

  // Begin searching.
  state = SEARCHING;

  motorLeft();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Test finished. Keep motor stopped.
  if (state == COMPLETE) {
    return;
  }

  unsigned long now = millis();

  byte sensorState = digitalRead(CENTRE_SENSOR);

  // ==========================================================
  // SEARCHING FOR DETECTION WINDOW ENTRY
  // ==========================================================

  if (state == SEARCHING) {

    // HIGH -> LOW: First detection.

    if (sensorState == LOW) {

      lastDetectionTime = now;

      // Visual marker for entry angle.
      digitalWrite(STATUS_LED, HIGH);

      // Continue rotating through the window.
      state = DETECTING;
    }

    return;
  }

  // ==========================================================
  // INSIDE DETECTION WINDOW
  // ==========================================================

  if (state == DETECTING) {

    // Refresh detection time whenever LOW is observed.

    if (sensorState == LOW) {

      lastDetectionTime = now;

      return;
    }

    // HIGH: Check whether signal loss is sustained.

    if (now - lastDetectionTime >= SIGNAL_LOSS_MS) {

      // Confirmed exit: brake before changing LED.
      motorBrake();

      // Visual marker for exit angle.
      digitalWrite(STATUS_LED, LOW);

      // Prevent automatic restart.
      state = COMPLETE;
    }
  }
}